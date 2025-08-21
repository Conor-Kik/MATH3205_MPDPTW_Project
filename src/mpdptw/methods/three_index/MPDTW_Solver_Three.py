from mpdptw.common.cli import parse_instance_argv
from mpdptw.common.three_index_printer import print_solution_summary
from mpdptw.common.parsers import build_milp_data
from gurobipy import *




def Run_Model(path, model: Model):
    inst = build_milp_data(str(path))



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
    

    inverse_request_map = inst["Nodes_To_Reqs"]


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
    M_ij = {(i,j): max(0.0, l[i] + d[i] + t[i,j] - e[j]) for (i,j) in A}


    start_nodes = [j for (_, j) in out_arcs[depot] if j != sink]
    # Decision variables (to be declared in solver)
    X = {(i, j, k): model.addVar(vtype=GRB.BINARY) for (i, j) in A  for k in K}  # binary arc use
    S = {i: model.addVar(vtype=GRB.CONTINUOUS) for i in V}  # continuous service start times
    Y = {(r, k) : model.addVar(vtype=GRB.BINARY) for  r in R for k in K}


                
    
    
    model.setObjective(quicksum(X[i, j, k] * c[i, j] for (i, j) in A for k in K) , GRB.MINIMIZE)




    # Degree (incoming = 1 for each customer j)
    DegreeConstrainIncome = {(i, k): 
                             model.addConstr(quicksum(X[i, j, k] for (_, j) in out_arcs[i]) == Y[inverse_request_map[i], k])
        for i in N for k in K
    }

    # Degree (outgoing = 1 for each customer i)
    DegreeConstrainOutgoing = {(i, k): 
                             model.addConstr(quicksum(X[j, i, k] for (j, _) in in_arcs[i]) == Y[inverse_request_map[i], k])
        for i in N for k in K
    }

    VehicleUsageLimit = { k:
                model.addConstr(quicksum(X[depot, j, k] for (_, j) in out_arcs[depot]) <= 1)
                for k in K}
    
    RequestAssigned = {r :
                       model.addConstr(quicksum(Y[r, k] for k in K) == 1)
                       for r in R}
    


    
    TimeWindowFeas = {(i, j):
                      model.addConstr(S[j]>= S[i] 
                                      + (d[i] + t[i, j] + l[sink])*quicksum(X[i, j , k] for k in K)
                                      - l[sink])
    for (i, j) in A}
    

    TimeFeasLatest = {i :
                       model.addConstr(S[i] <= l[i])
    for i in V}
    
    TimeFeasEarliest = {i :
                       model.addConstr(S[i] >= e[i])
    for i in V}

    RequestPrec = {(i, r):
                   model.addConstr(S[Dr_single[r]] >= S[i] + d[i] + t[i, Dr_single[r]])
                   for r in R for i in Pr[r]}
    

    model.optimize()

    print_solution_summary(
        model,
        V_ext=V_ext,
        R=R,
        K=K,
        Pr=Pr,
        Dr=Dr,
        X=X, S=S,
        e=e, l=l, q=q,
        Y=Y,           # pass your Y[r,k]
        t=t,
        sink=sink,
        d=d
    )
def main(argv=None):
    path, _ = parse_instance_argv(argv, default_filename="l_4_25_4.txt")
    model = Model("MPDTW")
    Run_Model(str(path), model)


if __name__ == "__main__":
    main()
