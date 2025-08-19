from mpdptw.common.cli import parse_instance_argv
from mpdptw.common.two_index_solution_printer import print_solution_summary
from mpdptw.common.parsers import build_milp_data
from gurobipy import *
from mpdptw.methods.arf.req_model import pair_feasible
import itertools

def Infeasible_Req_Pairs(R, e, l, Pr, Dr, c, q, sink, d, Q, inst):
    infeas = set()

    for r1, r2 in itertools.combinations(R, 2):

        # Candidate set: all pickups of both requests
        remaining = set(Pr[r1]) | set(Pr[r2])
        d1_added = False
        d2_added = False

        # Start at depot 
        curr = 0
        t = max(0, e.get(curr, 0))
        load = 0
        path = []

        feasible_greedy = True
        while True:
            # If all pickups of r1 done and D1 not yet added, add it now
            if not d1_added and remaining.isdisjoint(Pr[r1]):
                remaining.add(Dr[r1])
                d1_added = True
            if not d2_added and remaining.isdisjoint(Pr[r2]):
                remaining.add(Dr[r2])
                d2_added = True

            # If nothing left
            if not remaining:
                # Return to sink and check its window
                arrival = t + c[curr, sink]
                start   = max(arrival, e[sink])
                if start <= l[sink]:
                    # Greedy found a feasible sequence for this pair -> NOT infeasible
                    feasible_greedy = True
                else:
                    feasible_greedy = False
                break

            # Choose next node by earliest feasible start time from (curr, t)
            # Apply precedence implicitly: deliveries are in 'remaining' only when all its pickups were removed.
            best = None
            best_start = None
            best_finish = None
            best_load = None

            for n in list(remaining):
                # Travel and (maybe) wait
                arrival = t + c[curr, n]
                start   = max(arrival, e[n])
                if start > l[n]:
                    continue  # cannot serve n from here at time t

                finish = start + d[n]

                # capacity check at service
                new_load = load + q.get(n, 0)
                if new_load > Q:
                    continue

                # Greedy key: smallest feasible start time (tie-break by l[n] or finish)
                if best is None or (start < best_start) or (start == best_start and finish < best_finish):
                    best = n
                    best_start = start
                    best_finish = finish
                    best_load = new_load

            if best is None:
                feasible_greedy = False
                break

            # Commit to best move
            path.append(best)
            t = best_finish
            load = best_load
            remaining.remove(best)
            curr = best

        # If greedy failed, run small Gurobi Model to confirm
        if not feasible_greedy:
            no_feasible_route = Build_infeas_model(r1, r2, inst)
            
            if no_feasible_route:
                infeas.add(frozenset((r1, r2)))

    return infeas


def Build_infeas_model(r1, r2, inst):
    model = Model("Pair Check")
    feasible, work = pair_feasible(inst, r1, r2, model, output_flag=0)
    return not feasible

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
    
    inverse_request_map = {}
    for r in R:
        for i in Pr[r] + Dr[r]:
            inverse_request_map[i] = r

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

    W = Infeasible_Req_Pairs(R, e, l, Pr, Dr_single, c, q, sink, d, Q, inst)
    print(W)
    start_nodes = [j for (_, j) in out_arcs[depot] if j != sink]
    # Decision variables (to be declared in solver)
    X = {(i, j, k): model.addVar(vtype=GRB.BINARY) for (i, j) in A for k in R}  # binary arc use
    Y = {(r, k): model.addVar(vtype=GRB.BINARY) for r in R for k in R}  # continuous service start times
    S = {i : model.addVar(vtype=GRB.CONTINUOUS) for i in V}
    

    model.setObjective(quicksum(X[i, j, k] * c[i, j] for (i, j) in A for k in R) , GRB.MINIMIZE)

    DegreeConstrainIncome = {i: 
                             model.addConstr(quicksum(X[i, j, k] for (_, j) in out_arcs[i]) == Y[inverse_request_map[i], k])
        for i in N for k in R
    }

    # Degree (outgoing = 1 for each customer i)
    DegreeConstrainOutgoing = {i: 
                             model.addConstr(quicksum(X[j, i, k] for (j, _) in in_arcs[i]) == Y[inverse_request_map[i], k])
        for i in N for k in R
    }

    DepotCluster = { k:
                model.addConstr(quicksum(X[depot, j, k] for (_, j) in out_arcs[depot]) <= 1)
                for k in R}
    VehicleLimit = model.addConstr(quicksum(Y[k, k] for k in R) <= len(K))

    RequestAssigned = {r :
                       model.addConstr(quicksum(Y[r, k] for k in R) == 1)
                       for r in R}
    
    ClusterOpenLink = {k:
                       model.addConstr(quicksum(Y[r, k] for k in R if r > k) <= Y[k, k]*(len(R) - k))
                       for k in R}

def main(argv=None):
    path, _ = parse_instance_argv(argv, default_filename="l_4_25_4.txt")
    model = Model("MPDTW")
    model.setParam(GRB.Param.LazyConstraints, 1)
    Run_Model(str(path), model)


if __name__ == "__main__":
    main()
