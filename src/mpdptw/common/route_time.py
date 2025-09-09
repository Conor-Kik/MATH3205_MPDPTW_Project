from gurobipy import *
from mpdptw.common.big_M import tight_bigM



def Run_Time_Model(subset, inst, Time_Lim=False, Output=0, Time_Window = False, Capacity_Constraint = False):
    model = Model("Route_Time")
    
    if Output == 0:
        model.setParam("OutputFlag", 0)
    else:
        print("********************")
        print(subset)
        print("********************")
        model.setParam("OutputFlag", 1)
    if Time_Lim:
        model.setParam("TimeLimit", len(subset) * 3)

    # Instance data
    A_all     = inst["A_feasible_ext"]
    Pr        = inst["Pr"]
    Dr_single = inst["Dr_single"]
    e, l      = inst["e"], inst["l"]
    d, q      = inst["d"], inst["q"]
    t, c      = inst["t_ext"], inst["c_ext"]
    depot     = inst["depot"]
    sink      = inst["sink"]

    # Requests in this subproblem
    R         = subset
    Pickups   = [i for r in R for i in Pr[r]]
    Dels      = [Dr_single[r] for r in R]
    N         = set(Pickups) | set(Dels)
    V         = {depot, sink} | N
    Q         = inst["Q"]
    # Induced subgraph
    A_sub = [(i, j) for (i, j) in A_all
             if i in V and j in V and (i, j) in c and (i, j) in t]

    # Adjacency
    in_arcs  = {v: [] for v in V}
    out_arcs = {v: [] for v in V}
    for (i, j) in A_sub:
        out_arcs[i].append((i, j))
        in_arcs[j].append((i, j))

    dead_in  = [j for j in N if len(in_arcs[j])  == 0]
    dead_out = [i for i in N if len(out_arcs[i]) == 0]
    if dead_in or dead_out:
        raise ValueError(f"Subset yields isolated nodes — no-in={dead_in}, no-out={dead_out}")

    # Map node -> request (for precedence cuts)
    node_req = {}
    for r in R:
        for p in Pr[r]:
            node_req[p] = r
        node_req[Dr_single[r]] = r

    # Variables
    X = {(i, j): model.addVar(vtype=GRB.BINARY) for (i, j) in A_sub}

# Objective: travel + service-at-origin
    model.setObjective(quicksum(X[i, j] * (c[i, j] + d.get(i, 0.0)) for (i, j) in A_sub), GRB.MINIMIZE)

    if Capacity_Constraint:
        # Load on arcs after departing i along (i,j)
        F = {(i, j): model.addVar(vtype=GRB.CONTINUOUS, lb=0.0, ub=Q) for (i, j) in A_sub}

        # If arc is not used, its flow must be 0; otherwise flow ≤ Q
        CapCouple = {(i, j): model.addConstr(F[i, j] <= Q * X[i, j]) for (i, j) in A_sub}

        # Flow conservation at each customer node v:
        # sum_out F - sum_in F == q[v]
        FlowBal = {
            v: model.addConstr(
                quicksum(F[v, j] for (_, j) in out_arcs[v]) -
                quicksum(F[i, v] for (i, _) in in_arcs[v]) == q[v]
            )
            for v in N
        }

        ArriveEmpty = model.addConstr(quicksum(F[i, sink] for (i, _) in in_arcs[sink]) == 0.0)




    if Time_Window:
        M_ij, Earliest, Latest = tight_bigM(out_arcs, t, d, V, A_sub, sink, e, l)
        S = {i: model.addVar(vtype=GRB.CONTINUOUS, lb=Earliest[i], ub=Latest[i]) for i in V} 
        TimeWindowFeas = {
            (i,j): model.addConstr(S[j] >= S[i] + d[i] + t[i,j] - M_ij[i,j] * (1 - X[i,j]))
            for (i,j) in A_sub
        }
        TimeFeasEarliest = {i: model.addConstr(S[i] >= Earliest[i]) for i in V}
        TimeFeasLatest   = {i: model.addConstr(S[i] <= Latest[i])   for i in V}        

        
    # Degree = 1 on customer nodes 
    DegIn  = {j: model.addConstr(quicksum(X[i, j] for (i, _) in in_arcs[j]) == 1) for j in N}
    DegOut = {i: model.addConstr(quicksum(X[i, j] for (_, j) in out_arcs[i]) == 1) for i in N}

    # Depot/sink flow 
    model.addConstr(quicksum(X[i, sink] for (i, _) in in_arcs[sink]) ==
                                           quicksum(X[depot, j] for (_, j) in out_arcs[depot]))
    model.addConstr(quicksum(X[depot, j] for (_, j) in out_arcs[depot]) == 1)
    model.addConstr(quicksum(X[j, sink]  for (j, _) in in_arcs[sink])  == 1)


    model.Params.LazyConstraints = 1


    model.Params.Cutoff = float(l[sink])

    EPS = 1e-9
    N_only = set(N)

    def build_Sset(xvals):
        """Return internal components (cycles) entirely within N, using hard 0/1 rounding."""
        Sset = set()
        succ = {}
        for i in N_only:
            outs = [(j, xvals.get((i, j), 0.0)) for (ii, j) in out_arcs.get(i, []) if j in N_only]
            if not outs:
                continue
            j_best, v_best = max(outs, key=lambda z: z[1])
            if v_best >= 1.0 - EPS:
                succ[i] = j_best

        unvisited = set(N_only)
        while unvisited:
            u = unvisited.pop()
            path, pos = [], {}
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

    model._sec_seen  = set()
    model._seen = set()
    def subtour_callback(model, where):
        if where != GRB.Callback.MIPSOL:
            return

        XV = model.cbGetSolution(X)

        def sum_x_within(S):
            return quicksum(X[i, j] for i in S for j in S if (i, j) in X)

        def sum_x(from_set, to_set):
            return quicksum(X[i, j] for i in from_set for j in to_set if (i, j) in X)

        def add_cut_once(key, expr, rhs):
            if key in model._seen:
                return False
            model._seen.add(key)
            model.cbLazy(expr <= rhs)
            return True

        # ---- SECs on internal components ----
        cut_added = False
        Sset = build_Sset(XV)
        for S15 in Sset:
            key_sec = ("SEC", frozenset(S15))
            if key_sec not in model._sec_seen:
                lhs_val = sum(XV.get((i, j), 0.0) for i in S15 for j in S15 if (i, j) in X)
                rhs = len(S15) - 1
                if lhs_val > rhs + EPS:
                    model._sec_seen.add(key_sec)
                    model.cbLazy(sum_x_within(S15) <= rhs)
                    cut_added = True

            # Set cut (all pickups of r in C but delivery not in C): on S=C∪{sink}
            triggers = False
            for r in R:
                if set(Pr[r]).issubset(S15) and (Dr_single[r] not in S15):
                    triggers = True
                    break
            if triggers:
                S16 = set(S15) | {sink}
                key_16 = ("C16", frozenset(S16))
                if key_16 not in model._seen:
                    lhs_val_16 = sum(XV.get((i, j), 0.0) for i in S16 for j in S16 if (i, j) in X)
                    rhs_16 = len(S16) - 2
                    if lhs_val_16 > rhs_16 + EPS:
                        model._seen.add(key_16)
                        model.cbLazy(sum_x_within(S16) <= rhs_16)
                        cut_added = True

        # ---- Route-based precedence cuts: ----
        if not cut_added:
            start_arc = next((arc for arc in out_arcs[depot] if XV.get(arc, 0.0) > 0.5), None)

            if start_arc is not None:
                # Follow successors until sink or repetition
                i0, j0 = start_arc
                route = [i0, j0]
                posr = {i0: 0, j0: 1}
                seen = {i0, j0}
                node = j0
                for _ in range(len(V) + 2):
                    if node == sink:
                        break
                    succs = [j for (_, j) in out_arcs.get(node, []) if XV.get((node, j), 0.0) > 0.5]
                    if not succs:
                        break
                    j = succs[0]
                    if j in seen:
                        route.append(j)
                        posr[j] = len(route) - 1
                        break
                    route.append(j)
                    posr[j] = len(route) - 1
                    seen.add(j)
                    node = j

                # Gather requests appearing on the route
                reqs_on_route = {node_req[v] for v in route if v in node_req}

                # (54): delivery appears before one of its pickups
                for r in reqs_on_route:
                    dr = Dr_single[r]
                    if dr not in posr:
                        continue
                    idx_d = posr[dr]
                    for p in Pr[r]:
                        if p in posr and posr[p] > idx_d:
                            S_nodes = route[idx_d + 1: posr[p]]
                            if not S_nodes:
                                continue
                            key = ("54", dr, p, tuple(S_nodes))
                            expr = (sum_x({dr}, S_nodes)
                                    + sum_x_within(S_nodes)
                                    + sum_x(S_nodes, {p}))
                            add_cut_once(key, expr, len(S_nodes))
                            cut_added = True

                # (52): depot -> ... -> dr but not all pickups precede dr
                for r in reqs_on_route:
                    dr = Dr_single[r]
                    if dr not in posr:
                        continue
                    idx_d = posr[dr]
                    late_or_missing = any((p not in posr) or (posr[p] > idx_d) for p in Pr[r])
                    if not late_or_missing:
                        continue
                    S_nodes = route[1:idx_d]  # strictly between depot and dr
                    if not S_nodes:
                        continue
                    key = ("52", dr, tuple(S_nodes))
                    expr = sum_x({depot}, S_nodes) + sum_x_within(S_nodes + [dr])
                    add_cut_once(key, expr, len(S_nodes))
                    cut_added = True

    model.optimize(subtour_callback)

    try:
        best_bound = float(model.ObjBound)
    except Exception:
        best_bound = float("inf")

    if best_bound > float(l[sink]):
        return model, float(l[sink]) + 1.0, [], None

    if model.Status == GRB.OPTIMAL:
        used_arcs = [(i, j) for (i, j) in X if X[i, j].X > 0.5]
        if not Time_Window:
            return model, model.ObjVal, used_arcs, None
        else:
            return model, model.ObjVal, used_arcs,{i : S[i].X for i in S}
    else:
        return model, None, [], None
