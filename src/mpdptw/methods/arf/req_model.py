from mpdptw.common.two_index_solution_printer import print_solution_summary
from gurobipy import *

import itertools

def Infeasible_Req_Pairs(R, e, l, Pr, Dr, c, q, sink, d, Q, inst, output_flag):
    FN = 0
    TP = 0

    
    """
    This is a greedy search algorimth that attempts to find a feasible route by
    going to the next legal node with the lowest earliest start time.
    Try each pair of requests (r1, r2). For each pair we:
      1) Attempt a simple greedy construction of a feasible route on the extended graph.
      2) If the greedy attempt can't find a route, we confirm infeasibility by calling
         Build_infeas_model(r1, r2, inst). If confirmed, we record (r1, r2).
    """
    infeas = []

    # Allowed arcs on the extended graph
    A = set(inst["A_feasible_ext"])
    EPS = 1e-9

    # Global/depot window (node 0 is the depot)
    global_early = e[0]
    global_late = l[0]

    def is_pickup(n, r1, r2):
        """Is node n a pickup of r1 or r2?"""
        return (n in Pr[r1]) or (n in Pr[r2])

    def is_delivery(n, r1, r2):
        """Is node n the delivery of r1 or r2?"""
        return n == Dr[r1] or n == Dr[r2]

    def is_global_pickup(n, r1, r2):
        """
        A "global-window" pickup: same time window as the depot.
        This is important since we don't want to prioritise pickups with no time window
        """
        if not is_pickup(n, r1, r2):
            return False
        return (abs(e[n] - global_early) <= EPS) and (abs(l[n] - global_late) <= EPS)

    def consider_node(next_node, t_now, i_now, load_now):
        
        """
        If moving (i_now -> next_node) is feasible, return timing/load info.
        Otherwise return None.

        Returns (start_service_time, finish_time, new_load, travel_time).
        """
        if (i_now, next_node) not in A:
            return None

        arrival = t_now + c[i_now, next_node]
        start = max(arrival, e[next_node])
        if start > l[next_node] + EPS:
            return None

        new_load = load_now + q.get(next_node, 0)
        if new_load < 0 or new_load > Q:
            return None

        finish = start + d[next_node]
        travel = c[i_now, next_node]
        return (start, finish, new_load, travel)

    # ----------------------------------------------------------------------

    for r1, r2 in itertools.combinations(R, 2):
        # Start with pickups of the two requests
        remaining = set()
        remaining.update(Pr[r1])
        remaining.update(Pr[r2])

        d1_added = False  # whether delivery of r1 is unlocked/added
        d2_added = False  # whether delivery of r2 is unlocked/added

        # Start at the depot (node 0)
        current_node = 0
        current_time = max(0, e.get(current_node, 0))
        current_load = 0
        path = [] 

        feasible_greedy = True

        while True:
            # Once all pickups of a request are done, its delivery becomes available
            if (not d1_added) and remaining.isdisjoint(Pr[r1]):
                remaining.add(Dr[r1])
                d1_added = True
            if (not d2_added) and remaining.isdisjoint(Pr[r2]):
                remaining.add(Dr[r2])
                d2_added = True

            # If nothing remains, ensure we can go to the sink within its window
            if not remaining:
                if (current_node, sink) not in A:
                    feasible_greedy = False
                    break
                arrival_to_sink = current_time + c[current_node, sink]
                start_at_sink = max(arrival_to_sink, e[sink])
                feasible_greedy = (start_at_sink <= l[sink] + EPS)
                break  

            # Choose the next node in three phases with simple tie-breakers

            best_node = None
            best_key = None     # key is a tuple to compare lexicographically
            best_finish = None
            best_new_load = None

            # -------- Phase 1: non-global pickups → earliest start, tie by finish
            for n in sorted(remaining):
                if is_pickup(n, r1, r2) and ( not is_global_pickup(n, r1, r2) ):
                    info = consider_node(n, current_time, current_node, current_load)
                    if info is None:
                        continue
                    start, finish, new_load, travel = info
                    key = (start, finish)
                    if (best_key is None) or (key < best_key):
                        best_node = n
                        best_key = key
                        best_finish = finish
                        best_new_load = new_load

            # -------- Phase 2: global-window pickups → nearest first (then start, finish)
            if best_node is None:
                for n in sorted(remaining):
                    if is_global_pickup(n, r1, r2):
                        info = consider_node(n, current_time, current_node, current_load)
                        if info is None:
                            continue
                        start, finish, new_load, travel = info
                        key = (travel, start, finish)
                        if (best_key is None) or (key < best_key):
                            best_node = n
                            best_key = key
                            best_finish = finish
                            best_new_load = new_load

            # -------- Phase 3: deliveries → earliest start, tie by finish
            if best_node is None:
                for n in sorted(remaining):
                    if is_delivery(n, r1, r2):
                        info = consider_node(n, current_time, current_node, current_load)
                        if info is None:
                            continue
                        start, finish, new_load, travel = info
                        key = (start, finish)
                        if (best_key is None) or (key < best_key):
                            best_node = n
                            best_key = key
                            best_finish = finish
                            best_new_load = new_load

            # If there is no feasible route, the greedy attempt failed
            if best_node is None:
                feasible_greedy = False
                break

            # Commit the move
            path.append(best_node)
            current_time = best_finish
            current_load = best_new_load
            remaining.remove(best_node)
            current_node = best_node

        # If greedy failed, confirm infeasibility with the small MIP
        if not feasible_greedy:
            no_feasible_route = Build_infeas_model(r1, r2, inst, output_flag)
            if no_feasible_route:
                infeas.append((r1, r2))
            else:
                FN += 1
        else:
            TP += 1
    if output_flag:
        print("***********************************")
        print("RECALL OF GREEDY ALGO:", round(TP / (TP + FN),2))
        print("***********************************")
    return infeas


def Build_infeas_model(r1, r2, inst, output_flag):
    """
    Here we build a small model similar to the Two-Index and Three-Index formulations
    This is essentially a brute force way of checking feasiblity if the greedy
    algo fails
    """
    model = Model("Pair Check")
    feasible, work = pair_feasible(inst, r1, r2, model, output_flag)
    return not feasible


def pair_feasible(inst, r1, r2, model: Model, output_flag):
    if output_flag:
        print("***************************************")
        print("INITIAL GREEDY FAILED")
        print(f"NOW CHECKing REQ {r1} {r2} FOR FEASIBILIY")
        print("***************************************")
    Pr_all  = inst["Pr"]
    Dsingle = inst["Dr_single"]
    Q       = inst["Q"]
    e, l    = inst["e"], inst["l"]
    d, q    = inst["d"], inst["q"]
    t, c    = inst["t_ext"], inst["c_ext"]
    depot   = inst["depot"]
    sink    = inst["sink"]
    V_ext = inst["V_ext"]
    A = inst["A_feasible_ext"] 

    # --- Build pair subgraph ---
    P1, P2 = Pr_all[r1], Pr_all[r2]
    D1, D2 = Dsingle[r1], Dsingle[r2]
    N_pair = set(P1) | set(P2) | {D1, D2}
    V_pair = {depot, sink} | N_pair
    A_pair = [(i, j) for (i, j) in A if i in V_pair and j in V_pair]

    # adjacency
    in_arcs  = {j: [] for j in V_pair}
    out_arcs = {i: [] for i in V_pair}
    for (i, j) in A_pair:
        out_arcs[i].append((i, j))
        in_arcs[j].append((i, j))

    # restricted requests
    R = [r1, r2]
    Pr = {r1: P1, r2: P2}
    Dr = {r1: [D1], r2: [D2]}
    inverse_request_map = {}
    for r in R:
        for i in Pr[r] + Dr[r]:
            inverse_request_map[i] = r

    # tight Big-M
    M = {(i, j): max(0.0, l[i] + d[i] + t[i, j] - e[j]) for (i, j) in A_pair}

    # =========================
    # Variables
    # =========================
    X = {(i, j): model.addVar(vtype=GRB.BINARY) for (i, j) in A_pair}
    S = {i: model.addVar(vtype=GRB.CONTINUOUS) for i in V_pair}


    C = {i : model.addVar(vtype= GRB.CONTINUOUS, lb=0, ub= Q) for i in V_ext}
    CapacityFlow = {(i, j):
                    model.addConstr(C[j] >= C[i] + q[j] -  max(0.0, Q +  q[j])*(1 - X[i,j]))
                    for (i, j) in A_pair}
    # Objective
    model.setObjective(0, GRB.MINIMIZE)

    # =========================
    # Constraints
    # =========================

    # Time windows
    TimeFeasEarliest = {i: model.addConstr(S[i] >= e[i]) for i in V_pair}
    TimeFeasLatest   = {i: model.addConstr(S[i] <= l[i]) for i in V_pair}

    # Time propagation
    TimeWindowFeas = {
        (i, j): model.addConstr(
            S[j] >= S[i] + d[i] + t[i, j] - M[i, j] * (1 - X[i, j])
        )
        for (i, j) in A_pair
    }

    # Degree / assignment link (per node)
    DegreeConstrainOutgoing = {
        i: model.addConstr(quicksum(X[i, j] for (_, j) in out_arcs[i]) == 1)
        for i in N_pair
    }
    DegreeConstrainIncome = {
        i: model.addConstr(quicksum(X[j, i] for (j, _) in in_arcs[i]) == 1)
        for i in N_pair
    }

    # Depot/sink usage (exactly one route)
    LeaveDepot = model.addConstr(quicksum(X[depot, j] for (_, j) in out_arcs[depot]) == 1)
    EnterSink  = model.addConstr(quicksum(X[i, sink] for (i, _) in in_arcs[sink]) == 1)

    # Precedence (expanded linear form)
    Prec_r1 = {p: model.addConstr(S[D1] >= S[p] + d[p] + t[p, D1]) for p in P1}
    Prec_r2 = {p: model.addConstr(S[D2] >= S[p] + d[p] + t[p, D2]) for p in P2}

    model.Params.OutputFlag = output_flag
    model.optimize()

    feasible = model.status == GRB.OPTIMAL
    if output_flag and feasible:
        print_solution_summary(model, V_ext, R, range(1), Pr, Dr, X, S, e, l, q, t=t, sink=sink, d=d)
    return feasible, model.Work
