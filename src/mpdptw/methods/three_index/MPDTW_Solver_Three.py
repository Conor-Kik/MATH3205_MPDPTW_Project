import time
from mpdptw.common.big_M import tight_bigM
from mpdptw.common.cli import parse_instance_argv
from mpdptw.common.printers.three_index_printer import print_solution_summary
from mpdptw.common.parsers import build_milp_data
from gurobipy import *


def Run_Model(path, model: Model):
    EPS = 1e-6


    start_time = time.perf_counter()
    inst = build_milp_data(str(path))
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
    A_minus  = {j: [] for j in V}
    A_plus = {i: [] for i in V}
    for (i, j) in A:
        A_plus[i].append((i, j))
        A_minus[j].append((i, j))
    
    K = inst["K"]  # vehicles
    Q = inst["Q"]  # capacity

    in_arcs  = {j: [] for j in V}
    out_arcs = {i: [] for i in V}
    for (i, j) in A:
        out_arcs[i].append((i, j))
        in_arcs[j].append((i, j))
    

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

    #e[sink] = e[0]
    #l[sink] = l[0]
    M_ij, Earliest, Latest = tight_bigM(out_arcs, t, d, V, A, sink, e, l, Pr=Pr, Dr_single=Dr_single)
    

    start_nodes = [j for (_, j) in A_plus[depot] if j != sink]

    # Decision variables (to be declared in solver)
    X = {(i, j, k): model.addVar(vtype=GRB.BINARY) for (i, j) in A for k in K}  # binary arc use, 1 if vehicle k goes from i to j
    Y = {(r, k): model.addVar(vtype=GRB.BINARY) for r in R for k in K} #1 if request r completed by vehicle k 
    S = {i: model.addVar(vtype=GRB.CONTINUOUS, lb=Earliest[i], ub = Latest[i]) for i in V}  # continuous service start times
    
    #NEED TO CHECK WITH BRO HOW R WORKS
    
    #A is a list of tuples, [(i,j), ...], N = {1, ...., p, p+1, ..., p+n} = P U D
    
    #A+(i) = set of nodes j such that there is an arc from i to j, all arcs leaving i, out arcs
    #A-(i) = set of all nodes j such that arc j to i, all arcs entering i, in arcs
    A_minus  = {j: [] for j in V}
    A_plus = {i: [] for i in V}
    for (i, j) in A:
        A_plus[i].append((i, j))
        A_minus[j].append((i, j))
    
    node_to_request = {}
    for r in R:
        for i in Pr[r] + Dr[r]:
            node_to_request[i] = r

    model.setObjective(quicksum(X[i, j, k] * c[i, j] for (i, j) in A for k in K), GRB.MINIMIZE)
    
    #Constraints
    OnlyExitsNodeIfHandlingRequest =  {(i, k): 
                             model.addConstr(quicksum(X[i, j, k] for (_, j) in A_plus[i]) == Y[node_to_request[i], k])
        for i in N for k in K
    }
    
    OnlyEntersNodeIfHandlingRequest = {(i, k): 
                             model.addConstr(quicksum(X[j, i, k] for (j, _) in A_minus[i]) == Y[node_to_request[i], k])
        for i in N for k in K
    }
    
    EachVehicleOneRouteLeavingOrigin = { k:
                model.addConstr(quicksum(X[depot, j, k] for (_, j) in A_plus[depot]) <= 1)
                for k in K}
    
    RequestsHandledByOneVehicle = {
    r: model.addConstr(quicksum(Y[r,k] for k in K) == 1)
    for r in R}
    
    ServiceOfNextNodeOnlyAfterServiceAtThisNodeAndTravel = {
    (i,j): model.addConstr(
        S[i] + (d[i] + t[i,j] + M_ij[i,j])*(quicksum(X[i,j,k] for k in K)) - M_ij[i,j]
                           <= S[j])
    for (i,j) in A}

    ServiceDeliveryNoEarlierThanServiceServiceTimeAndTravel = {(i, r): 
    model.addConstr(d[i] + S[i] + t[i, Dr_single[r]] <= S[Dr_single[r]])
    for r in R for i in Pr[r]}
    
    model.setParam('TimeLimit', 3600)
    model.optimize()
    end_time = time.perf_counter()
    #print_solution_summary(model, V_ext, R, K, Pr, Dr, X, S, e, l, q, t=t, sink=sink, d=d)
    print()
    if model.SolCount > 0:
        UB = model.ObjVal
    else:
        UB = float('inf')  # no feasible solution, UB = +∞

    print("Best Upper Bound (UB):", "∞" if UB == float('inf') else round(UB, 2))
    print("Best Bound (LB):", round(model.ObjBound, 2))
    print("MIP Gap:", round(model.MIPGap, 2))
    print("TIME:", round(end_time - start_time, 2))
    print("Work units used:", round(model.Work, 2))
def main(argv=None):
    path, _ = parse_instance_argv(argv, default_filename="l_4_25_4.txt")
    model = Model("MPDTW")
    Run_Model(path, model)


if __name__ == "__main__":
    main()
