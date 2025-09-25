from mpdptw.common.cli import parse_instance_argv
from mpdptw.common.printers.arf_solution_printer import print_solution_summary
from mpdptw.common.parsers import build_milp_data
from gurobipy import *
from mpdptw.methods.arf.cluster_assignment import Run_Cluster_Assignment_Model
import time 
from mpdptw.common.big_M import tight_bigM


OUTPUT_REQ_MODEL = 0 # 1 Shows for request pair infeasibility model output
OUTPUT_CLUSTER_MODEL = 1 # 1 Shows full cluster_model output

def Run_Model(path, model: Model):
    start = time.perf_counter()
    inst = build_milp_data(str(path))
    cluster_model = Model("Optimal Cluster Assignment")
    
    EPS = 1e-6
    # Sets (extended with sink depot)
    V = inst["V_ext"]               # all nodes including origin (0) and sink
    A = inst["A_feasible_ext"]      # feasible arcs 
    N = inst["N"]
    PREPROCESSING_CUTOFF = len(N)//5
    R = inst["R"]                   # request ids (0-based range)
    Pr = inst["Pr"]                 # pickups per request
    Dr = inst["Dr"]                 # deliveries per request
    Dr_single = inst["Dr_single"]   # single delivery per request
    S_min = inst["S_minimal_ext"]   # minimal S-sets from paper

    # Build adjacency from A once 
    in_arcs  = {j: [] for j in V}
    out_arcs = {i: [] for i in V}
    for (i, j) in A:
        out_arcs[i].append((i, j))
        in_arcs[j].append((i, j))
    
    inverse_request_map = {}
    for r in R:
        for i in Pr[r] + Dr[r]:
            inverse_request_map[i] = r

    K = inst["K"]                  # vehicles
    Q = inst["Q"]                  # capacity

    # Parameters (extended)
    e = inst["e"]                  # earliest start
    l = inst["l"]                  # latest start
    d = inst["d"]                  # service time
    q = inst["q"]                  # demand
    t = inst["t_ext"]              # travel time (extended)
    c = inst["c_ext"]              # travel cost (extended)
    V_ext = inst["V_ext"]

    # Special nodes
    depot = inst["depot"]          # start depot (0)
    sink = inst["sink"]            # sink depot

    # Big-M per arc (tight)
    #M_ij = {(i,j): max(0.0, l[i] + d[i] + t[i,j] - e[j]) for (i,j) in A}
    M_ij, Earliest, Latest = tight_bigM(out_arcs, t, d, V, A, sink, e, l, Pr=Pr, Dr_single=Dr_single)
    # Infeasible request pairs, then ordering MIP (returns rank and pos)
    #W = Infeasible_Req_Pairs(R, e, l, Pr, Dr_single, c, q, sink, d, Q, inst, output_flag=OUTPUT_REQ_MODEL)
    W = inst["W"]
    (rank, pos), cluster_work = Run_Cluster_Assignment_Model(inst, cluster_model, W, PREPROCESSING_CUTOFF, outputflag = OUTPUT_CLUSTER_MODEL)  # rank[i] = position, pos[k] = request

    # Helpers (depots have no request)
    def req_of(i): 
        return inverse_request_map.get(i, None)

    def allowed_in_cluster(i, k):
        ri = req_of(i)
        return True if ri is None else (rank[ri] >= k)

    start_nodes = [j for (_, j) in out_arcs[depot] if j != sink]
    nodes_per_req = {r: len(Pr[r]) + len(Dr[r]) for r in R}   # usually len(Dr[r]) = 1
    # =========================
    # Variables
    # =========================
    X = {(i, j, k): model.addVar(vtype=GRB.BINARY)
         for (i, j) in A for k in R
         if allowed_in_cluster(i, k) and allowed_in_cluster(j, k)}
        
    Y = {(r, k): model.addVar(vtype=GRB.BINARY)
         for r in R for k in R if rank[r] >= k}

    S = {i: model.addVar(vtype=GRB.CONTINUOUS, lb=Earliest[i], ub=Latest[i]) for i in V}
    
    
    # =========================
    # Objective
    # =========================
    # Sum only over existing X keys
    model.setObjective(quicksum(c[i, j] * X[i, j, k] for (i, j, k) in X), GRB.MINIMIZE)
    # =========================
    # ARF/cluster-model linkage 
    # =========================
    n = len(R)
    for k in R:
        rk = pos[k]  # the request that is first in cluster/position k
        # sum_{r: rank[r] > k} y_{r,k} <= (n-1-k) * y_{pos[k], k}
        model.addConstr(
            quicksum(Y[r, k] for r in R if rank[r] > k and (r, k) in Y)
            <= (n - 1 - k) * Y[rk, k]
        )
    # =========================
    # Constraints
    # =========================

    # Degree constraints per customer node i and position k
    DegreeConstrainIncome = {
        (i, k): model.addConstr(
            quicksum(X[(i, j, k)] for (_, j) in out_arcs[i] if (i, j, k) in X)
            ==
            (Y[inverse_request_map[i], k] if (inverse_request_map[i], k) in Y else 0)
        )
        for i in N for k in R
    }

    DegreeConstrainOutgoing = {
        (i, k): model.addConstr(
            quicksum(X[(j, i, k)] for (j, _) in in_arcs[i] if (j, i, k) in X)
            ==
            (Y[inverse_request_map[i], k] if (inverse_request_map[i], k) in Y else 0)
        )
        for i in N for k in R
    }
    StartEq = {
        k: model.addConstr(
            quicksum(X[(depot, j, k)] for (_, j) in out_arcs[depot] if (depot, j, k) in X)
            ==
            (Y[(pos[k], k)] if (pos[k], k) in Y else 0),
            name=f"start_eq[{k}]"
        )
        for k in R
    }

    EndEq = {
        k: model.addConstr(
            quicksum(X[(i, sink, k)] for (i, _) in in_arcs[sink] if (i, sink, k) in X)
            ==
            (Y[(pos[k], k)] if (pos[k], k) in Y else 0),
            name=f"end_eq[{k}]"
        )
        for k in R
    }
        # At most one departure from depot per cluster/position k
    DepotCluster = {
        k: model.addConstr(
            quicksum(X[(depot, j, k)] for (_, j) in out_arcs[depot] if (depot, j, k) in X) <= 1
        )
        for k in R
    }

    # Vehicle limit: sum_k y_{pos[k],k} <= |K|
    VehicleLimit = model.addConstr(
        quicksum(Y[pos[k], k] for k in R if (pos[k], k) in Y) <= len(K)
    )

    # Each request assigned exactly once: sum_k y_{r,k} = 1 (over existing Y only)
    RequestAssigned = {
        r: model.addConstr(quicksum(Y[r, k] for k in R if (r, k) in Y) == 1)
        for r in R
    }
    
    # Infeasible request pairs cannot be in the same cluster k
    FeasibleClusterPairs = {
        (k, rr, r): model.addConstr(Y[r, k] + Y[rr, k] <= 1)
        for k in R for r in R for rr in R
        if (r, rr) in W and (r, k) in Y and (rr, k) in Y
    }

    # =========================
    # Time feasibility
    # =========================
    # Big-M with active clusters for (i,j): only if there exists any k with X[i,j,k]
    TimeWindowFeas = {
        (i, j,k): model.addConstr(
            S[j] >= S[i] + d[i] + t[i, j] - M_ij[i, j] * (1 - X[i, j, k])
        )
        for (i, j) in A for k in R
        if (i, j, k) in X
    }
    # Time windows
    #TimeFeasLatest = {i: model.addConstr(S[i] <= l[i]) for i in V}
    #TimeFeasEarliest = {i: model.addConstr(S[i] >= e[i]) for i in V}
    TimeFeasEarliest = {i: model.addConstr(S[i] >= Earliest[i]) for i in V}
    TimeFeasLatest   = {i: model.addConstr(S[i] <= Latest[i])   for i in V}
    # Precedence within each request: pickups before its delivery
    RequestPrec = {
        (i, r): model.addConstr(S[Dr_single[r]] >= S[i] + d[i] + t[i, Dr_single[r]])
        for r in R for i in Pr[r]
    }

    
    end = time.perf_counter()
    # Optimize & print
    model.setParam('TimeLimit', 3600)
    model.optimize()
    
    if False:
        print_solution_summary(
            model,
            V_ext,          
            R,              
            K,   
            Pr, Dr,
            X, Y,           
            S,              
            e, l, q,
            t,
            sink,
            d
        )
    print(f"PREPROCESSING RUNTIME: {end - start:.2f}{' - (Cluster Model cutoff activated at ' + str(PREPROCESSING_CUTOFF) + " Seconds)" if (end - start) > PREPROCESSING_CUTOFF else ''}")
    print(f"MODEL RUNTIME: {model.Runtime:.2f}")
    print(f"TOTAL: {model.Runtime + end - start:.2f}")

    print()
    if model.SolCount > 0:
        UB = model.ObjVal
    else:
        UB = float('inf')  # no feasible solution, UB = +∞

    print("Best Upper Bound (UB):", "∞" if UB == float('inf') else round(UB, 2))
    print("Best Bound (LB):", round(model.ObjBound, 2))
    print("MIP Gap:", round(model.MIPGap, 2))
    print(f"TOTAL: {model.Runtime + end - start:.2f}")
    print("Work units used:", round(model.Work + cluster_work, 2))

def main(argv=None):
    path, _ = parse_instance_argv(argv, default_filename="l_4_25_4.txt")
    model = Model("MPDTW")
    Run_Model(str(path), model)


if __name__ == "__main__":
    main()
