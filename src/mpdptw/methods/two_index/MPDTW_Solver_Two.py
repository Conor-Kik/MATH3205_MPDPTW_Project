from mpdptw.common.cli import parse_instance_argv
from mpdptw.common.two_index_solution_printer import print_solution_summary
from mpdptw.common.parsers import build_milp_data
from gurobipy import *
import re 
from mpdptw.methods.two_index.big_M import tight_bigM

def Run_Model(path, model: Model):
    inst = build_milp_data(str(path))
    filename = path.split("\\")[-1]   # "l_4_25_1.txt"

    match = re.search(r'_(\d+)_', filename)
    if match:
        number_str = match.group(1)   
        Request_Length = int(number_str)
        if Request_Length not in (4, 8):
            raise ValueError("Matched Value should be 4")
    else:
        Request_Length = 8


    EPS = 1e-6
    # Sets (extended with sink depot)
    V = inst["V_ext"]  # all nodes including origin (0) and sink
    A = inst["A_feasible_ext"]  # feasible arcs 
    N = inst["N"]

    R = inst["R"]  # request ids
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
    #M_ij = {(i,j): max(0.0, l[i] + d[i] + t[i,j] - e[j]) for (i,j) in A}
    M_ij, Earliest, Latest = tight_bigM(out_arcs, t, d, V, A, sink, e, l)


    start_nodes = [j for (_, j) in out_arcs[depot] if j != sink]
    # Decision variables (to be declared in solver)
    X = {(i, j): model.addVar(vtype=GRB.BINARY) for (i, j) in A}  # binary arc use
    S = {i: model.addVar(vtype=GRB.CONTINUOUS) for i in V}  # continuous service start times
    Z = {i : model.addVar(vtype=GRB.CONTINUOUS) for i in N}
    

    if  Request_Length == 8:
        print("**************************************************")
        print("Adding Capacity Constrain for Longer Request Data")
        print("**************************************************")
        C = {i : model.addVar(vtype= GRB.CONTINUOUS, lb=0, ub= Q) for i in V_ext}
        CapacityFlow = {(i, j):
                    model.addConstr(C[j] >= C[i] + q[j] -  max(0.0, Q +  q[j])*(1 - X[i,j]))
                    for (i, j) in A}

    model.setObjective(quicksum(X[i, j] * c[i, j] for (i, j) in A), GRB.MINIMIZE)


    # Degree (incoming = 1 for each customer j)
    DegreeConstrainIncome = {
        j: model.addConstr(quicksum(X[i, j] for (i, _) in in_arcs[j]) == 1)
        for j in N
    }

    # Degree (outgoing = 1 for each customer i)
    DegreeConstrainOutgoing = {
        i: model.addConstr(quicksum(X[i, j] for (_, j) in out_arcs[i]) == 1)
        for i in N
    }

    DepotBalance =  model.addConstr(
        quicksum(X[i, sink] for (i, _) in in_arcs[sink])
        == quicksum(X[0, j] for (_, j) in out_arcs[0])
    )

    TimeWindowFeas = {
        (i,j): model.addConstr(S[j] >= S[i] + d[i] + t[i,j] - M_ij[i,j] * (1 - X[i,j]))
        for (i,j) in A
    }
    TimeFeasEarliest = {i: model.addConstr(S[i] >= Earliest[i]) for i in V}
    TimeFeasLatest   = {i: model.addConstr(S[i] <= Latest[i])   for i in V}

    VehicleUsageLimit = model.addConstr(quicksum(X[0, j] for (_,j) in out_arcs[0]) <= len(K))
    
    #TimeWindowFeas = {(i, j): model.addConstr(S[j]>= S[i] + d[i] + t[i, j] - M_ij[i,j]*(1-X[i, j])) for (i, j) in A}
    

    #TimeFeasLatest = {i : model.addConstr(S[i] <= l[i]) for i in V}
    
    #TimeFeasEarliest = {i : model.addConstr(S[i] >= e[i]) for i in V}

    RequestPrec = {(i, r):
                   model.addConstr(S[Dr_single[r]] >= S[i] + d[i] + t[i, Dr_single[r]])
                   for r in R for i in Pr[r]}
    
    PDSameRoute = {(i, r):
                   model.addConstr(Z[Dr_single[r]] == Z[i])
    for r in R for i in Pr[r]}

    RouteIdentifier1 = {j:
                       model.addConstr(Z[j] >= j*X[0, j])
    for j in start_nodes}


    RouteIdentifier2 = {j:
                       model.addConstr(Z[j] <= j*X[0, j] - len(N)*(X[0, j] -1))
    for j in start_nodes}

    IndexFoward1 = {(i, j):
                    model.addConstr(Z[j] >= Z[i] + len(N)*(X[i, j] -1))
    for i in N for j in N if (i, j) in X}

    IndexFoward2 = {(i, j):
                    model.addConstr(Z[j] <= Z[i] + len(N)*(1-X[i, j]))
    for i in N for j in N if (i, j) in X}

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
    model = Model("MPDTW")
    model.setParam(GRB.Param.LazyConstraints, 1)
    Run_Model(str(path), model)


if __name__ == "__main__":
    main()
