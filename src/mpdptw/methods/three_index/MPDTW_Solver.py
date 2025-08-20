from mpdptw.common.cli import parse_instance_argv
from mpdptw.common.three_index_solution_printer import print_solution_summary
from mpdptw.common.parsers import build_milp_data
from gurobipy import *


def Run_Model(inst, model: Model):
    EPS = 1e-6
    # Sets (extended with sink depot)
    V = inst["V_ext"]  # all nodes including origin (0) and sink
    A = inst["A_feasible_ext"]  # feasible arcs 
    N = inst["N"]

    R = inst["R"]  # request ids
    R_dict = inst["R_dict"]
    Pr = inst["Pr"]  # pickups per request
    Dr = inst["Dr"]  # deliveries per request
    Dr_single = inst["Dr_single"]  # single delivery per request
    S_min = inst["S_minimal_ext"]  # minimal S-sets from paper

    # Build adjacency from A once 
    in_arcs  = {j: [] for j in V}
    out_arcs = {i: [] for i in V}
    for (i, j) in A:
        out_arcs[i].append((i, j))
        in_arcs[j].append((i, j))
    
    K = inst["K"]  # vehicles
    Q = inst["Q"]  # capacity

    # Parameters (extended)
    e = inst["e"]  # earliest start
    l = inst["l"]  # latest start

    d = inst["d"]  # service time
    q = inst["q"]  # demand
    t = inst["t_ext"]  # travel time (extended)
    c = inst["c_ext"]  # travel cost (extended)
    V_ext = inst["V_ext"]

    # Special nodes
    depot = inst["depot"]  # start depot (0)
    sink = inst["sink"]  # sink depot

    e[sink] = e[0]
    l[sink] = l[0]
    M_ij = {(i,j): max(0.0, l[i] + d[i] + t[i,j] - e[j]) for (i,j) in A}


    start_nodes = [j for (_, j) in out_arcs[depot] if j != sink]
    
    print(A)
    
    ### NEWWWWW
    # Decision variables (to be declared in solver)
    X = {(i, j, k): model.addVar(vtype=GRB.BINARY) for (i, j) in A for k in K}  # binary arc use, 1 if vehicle k goes from i to j
    Y = {(r, k): model.addVar(vtype=GRB.BINARY) for r in R for k in K} #1 if request r completed by vehicle k 
    S = {i: model.addVar(vtype=GRB.CONTINUOUS) for i in V}  # continuous service start times
    
    #NEED TO CHECK WITH BRO HOW R WORKS
    
    #A is a list of tuples, [(i,j), ...], N = {1, ...., p, p+1, ..., p+n} = P U D
    
    #A+(i) = set of nodes j such that there is an arc from i to j, all arcs leaving i, out arcs
    def A_plus(node):
        nodes_leaving = []
        for (i, j) in A:
            if node == i:
                #should return a j value
                nodes_leaving.append(j)
        return nodes_leaving
    
    #A-(i) = set of all nodes j such that arc j to i, all arcs entering i, in arcs
    def A_minus(node):
        nodes_entering = []
        for (i, j) in A:
            if node == j:
                #should return a j value
                nodes_entering.append(i)
        return nodes_entering
    
    
    node_to_request = {}
    for rid, nodes in R_dict.items():
        for i in nodes["pickups"] + nodes["deliveries"]:
            node_to_request[i] = rid

    model.setObjective(quicksum(quicksum(X[i, j, k] * c[i, j] for (i, j) in A) for k in K), GRB.MINIMIZE)
    
    #Constraints
    OnlyExitsNodeIfHandlingRequest =  {
    (i, k): model.addConstr(quicksum(X[i, j, k] for j in A_plus(i)) == Y[node_to_request[i], k])
    for i in N for k in K}
    
    OnlyEntersNodeIfHandlingRequest =  {
    (i, k): model.addConstr(quicksum(X[i, j, k] for i in A_minus(j)) == Y[node_to_request[i], k])
    for i in N for k in K}
    
    EachVehicleOneRouteLeavingOrigin = {
    k:  model.addConstr(quicksum(X[0,j,k] for j in A_plus(0)) <= 1)
    for k in K}
    
    RequestsHandledByOneVehicle = {
    r: model.addConstr(quicksum(Y[r,k] for k in K) == 1)
    for r in R}
    
    ServiceOfNextNodeOnlyAfterServiceAtThisNodeAndTravel = {
    (i,j): model.addConstr((quicksum(X[i,j,k] - l[sink] for k in K)*(d[i] + t[i,j] + l[sink]) + S[i]) <= S[j]) 
    for (i,j) in A}
    
    StartingServiceTimeLowerBound= {
    i: model.addConstr(e[i] <= S[i]) 
    for i in N}
    
    StartingServiceTimeUpperBound= {
    i: model.addConstr(S[i] <= l[i]) 
    for i in N}
    
    ServiceDeliveryNoEarlierThanServiceServiceTimeAndTravel = {(i, r): 
    model.addConstr(d[i] + S[i] + t[i, Dr_single[r]] <= S[Dr_single[r]])
    for r in R for i in Pr[r]}
    

    def build_Sset(xvals):
        Sset = set()
        succ =  {}
        for (i, j) in A:
            if i in N:
                if xvals[i, j] > 0.5:
                    succ[i] = j
        unvisited = set(N)

        while unvisited:
            u = unvisited.pop()
            path = []
            pos = {}
            while True:
                if u not in succ:
                    break
                if u in pos:
                    k = pos[u]
                    cycle = path[k:]
                    if len(cycle) >= 2:
                        Sset.add(frozenset(cycle))
                    break
                pos[u] = len(path)
                path.append(u)
                unvisited.discard(u)
                u = succ[u]
        return Sset

    def subtour_callback(model, where):
        if where == GRB.Callback.MIPSOL:
            XV = model.cbGetSolution(X)
            Sset = build_Sset(XV)
            for s in Sset:
                stronger = False
                for r in R:
                    if all(p in s for p in Pr[r]) and Dr_single[r] not in s:
                        stronger = True
                        break

                rhs = len(s) - 2 if stronger else len(s) - 1
                model.cbLazy(quicksum(X[i, j] for i in s for j in s if (i, j) in X) <= rhs)
                
    model.optimize(subtour_callback)

    print_solution_summary(model, V_ext, R, K, Pr, Dr, X, S, e, l, q, t=t, sink=sink, d=d)

def main(argv=None):
    path, _ = parse_instance_argv(argv, default_filename="l_4_25_4.txt")
    inst = build_milp_data(str(path))
    model = Model("MPDTW")
    model.setParam(GRB.Param.LazyConstraints, 1)
    Run_Model(inst, model)


if __name__ == "__main__":
    main()
