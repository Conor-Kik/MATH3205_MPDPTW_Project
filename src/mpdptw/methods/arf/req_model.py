from mpdptw.common.two_index_solution_printer import print_solution_summary
from gurobipy import *

def pair_feasible_three_index(inst, r1, r2, model: Model, output_flag=0):
    # --- Unpack full instance ---
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

    feasible = (model.status == GRB.OPTIMAL) or (model.status == GRB.TIME_LIMIT and model.SolCount > 0)

    if output_flag and feasible:
        print_solution_summary(model, V_ext, R, range(1), Pr, Dr, X, S, e, l, q, t=t, sink=sink, d=d)

    return feasible, model.Work
