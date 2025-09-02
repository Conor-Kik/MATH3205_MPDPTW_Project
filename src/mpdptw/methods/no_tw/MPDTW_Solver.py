from mpdptw.common.cli import parse_instance_argv
from mpdptw.common.arf_solution_printer import print_solution_summary
from mpdptw.common.parsers import build_milp_data
from gurobipy import *
from mpdptw.methods.arf.cluster_assignment import Run_Cluster_Assignment_Model
from mpdptw.methods.no_tw.warm_start import warm_start_solution
from collections import defaultdict
OUTPUT_REQ_MODEL = 0 # 1 Shows for request pair infeasibility model output
OUTPUT_CLUSTER_MODEL = 0 # 1 Shows full cluster_model output
PREPROCESSING_CUTOFF = 10 #Amount of time that cluster assignment model termininates (seconds)
PLOT_CLUSTERS = 1#1 Show plot of warm_start prediction

def Run_Model(path, model: Model):
    inst = build_milp_data(str(path))
    cluster_model = Model("Optimal Cluster Assignment")

    # Instance data
    V        = inst["V_ext"]
    A        = inst["A_feasible_ext"]
    N        = inst["N"]
    R        = inst["R"]
    Pr       = inst["Pr"]
    Dr       = inst["Dr"]
    Dr_single= inst["Dr_single"]


    K        = inst["K"]
    Q        = inst["Q"]
    nodes_to_reqs = inst["Nodes_To_Reqs"]
    e        = inst["e"]
    l        = inst["l"]
    d        = inst["d"]
    q        = inst["q"]
    t        = inst["t_ext"]
    c        = inst["c_ext"]
    V_ext    = inst["V_ext"]

    depot    = inst["depot"]
    sink     = inst["sink"]
    D_nodes  = set(Dr_single[r] for r in R)
    # Adjacency
    in_arcs, out_arcs = {j: [] for j in V}, {i: [] for i in V}
    for (i, j) in A:
        out_arcs[i].append((i, j))
        in_arcs[j].append((i, j))

    # Node → request
    inverse_request_map = {}
    for r in R:
        for i in Pr[r] + Dr[r]:
            inverse_request_map[i] = r


    def req_of(i):
        return inverse_request_map.get(i, None)
    def allowed_in_cluster(i, k):
        ri = req_of(i)
        return True if ri is None else (rank[ri] >= k)



    # Heuristic warm start
    init_sol, total_cost, clusters_to_reqs = warm_start_solution(inst, PLOT_CLUSTERS)
    used_labels = {k_lbl for (_, _), k_lbl in init_sol.items()}
    # Rank/position model for cluster ordering
    rank, pos = Run_Cluster_Assignment_Model(
        inst, cluster_model, None, PREPROCESSING_CUTOFF, outputflag=OUTPUT_CLUSTER_MODEL
    )
    # Variables
    X = {(i, j, k): model.addVar(vtype=GRB.BINARY)
         for (i, j) in A for k in R
         if allowed_in_cluster(i, k) and allowed_in_cluster(j, k)}
    Y = {(r, k): model.addVar(vtype=GRB.BINARY)
         for r in R for k in R if rank[r] >= k}
    
    

    # Map warm-start arcs to request sets per route label
    route_req = {}
    for (i, j), k_lbl in init_sol.items():
        ri = nodes_to_reqs.get(i)
        rj = nodes_to_reqs.get(j)
        s = route_req.setdefault(k_lbl, set())
        if ri is not None:
            s.add(ri)
        if rj is not None:
            s.add(rj)

    # Assign each heuristic route to k̂ = min_r rank[r], resolving collisions
    label_to_khat, taken_k = {}, set()
    for k_lbl, Rset in route_req.items():
        if not Rset:
            continue
        k_hat = min(rank[r] for r in Rset)
        if k_hat in taken_k:
            ok = False
            for alt in range(k_hat + 1, len(R)):
                if all(rank[r] >= alt for r in Rset) and alt not in taken_k:
                    k_hat, ok = alt, True
                    break
            if not ok:
                continue
        label_to_khat[k_lbl] = k_hat
        taken_k.add(k_hat)

    # Warm-start seeds
    for v in X.values():
        v.Start = 0.0
    for v in Y.values():
        v.Start = 0.0

    n_set_x = 0
    for (i, j), k_lbl in init_sol.items():
        if k_lbl not in label_to_khat:
            continue
        k_hat = label_to_khat[k_lbl]
        key = (i, j, k_hat)
        if key in X:
            X[key].Start = 1.0
            n_set_x += 1
    print(f"[warm] seeded {n_set_x} X vars")

    n_set_y = 0
    for k_lbl, Rset in route_req.items():
        if k_lbl not in label_to_khat:
            continue
        k_hat = label_to_khat[k_lbl]
        for r in Rset:
            if (r, k_hat) in Y:
                Y[(r, k_hat)].Start = 1.0
                n_set_y += 1
        if (pos[k_hat], k_hat) in Y:
            Y[(pos[k_hat], k_hat)].Start = 1.0
    print(f"[warm] seeded {n_set_y} Y vars")

    # Objective
    model.setObjective(quicksum(c[i, j] * X[i, j, k] for (i, j, k) in X), GRB.MINIMIZE)


    # Degree/linking per customer node i and position k
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
            (Y[(pos[k], k)] if (pos[k], k) in Y else 0)
        )
        for k in R
    }

    EndEq = {
        k: model.addConstr(
            quicksum(X[(i, sink, k)] for (i, _) in in_arcs[sink] if (i, sink, k) in X)
            ==
            (Y[(pos[k], k)] if (pos[k], k) in Y else 0)
        )
        for k in R
    }

    # Each request assigned exactly once
    RequestAssigned = {
        r: model.addConstr(quicksum(Y[r, k] for k in R if (r, k) in Y) == 1)
        for r in R
    }

    # Limit number of active clusters
    VehicleLimit = model.addConstr(
        quicksum(Y[pos[k], k] for k in R if (pos[k], k) in Y) <= len(K)
    )

    model._pairprec_seen = set()
    model._prec_seen = set()   
    model._sec_seen = set()

    def _components_for_cluster(k, xvals, X, depot, sink, N):
        adj = defaultdict(set)
        nodes = set()
        for (i, j, kk) in X.keys():
            if kk != k:
                continue
            if xvals.get((i, j, k), 0.0) <= 0.5:
                continue
            if i in (depot, sink) or j in (depot, sink):
                continue  # internal components only
            if i not in N or j not in N:
                continue
            nodes.add(i); nodes.add(j)
            adj[i].add(j); adj[j].add(i)

        Sset, seen = [], set()
        for v in nodes:
            if v in seen: 
                continue
            stack, comp = [v], set()
            while stack:
                u = stack.pop()
                if u in seen: 
                    continue
                seen.add(u); comp.add(u)
                for w in adj[u]:
                    if w not in seen:
                        stack.append(w)
            if len(comp) >= 2:
                Sset.append(comp)
        return Sset
    
    def lazy_cb(model, where):
        if where != GRB.Callback.MIPSOL:
            return

        XV = model.cbGetSolution(X)
        YV = model.cbGetSolution(Y)

        # ---- helpers ------------------------------------------------------------
        def add_cut_once(key, expr, rhs):
            if key in model._prec_seen:
                return False
            model._prec_seen.add(key)
            model.cbLazy(expr <= rhs)
            return True

        def sum_x_within(S, k):
            # sum_{i in S} sum_{j in S} x_{i,j,k}
            return quicksum(X[i, j, k] for i in S for j in S if (i, j, k) in X)

        def sum_x(from_set, to_set, k):
            # sum_{i in A} sum_{j in B} x_{i,j,k}
            return quicksum(X[i, j, k] for i in from_set for j in to_set if (i, j, k) in X)

        def build_route_for_k(k):
            # returns (route list, pos map, ok flag, duration)
            node, seen, duration, ok = depot, {depot}, 0.0, False
            route = [depot]
            for _ in range(len(V) + 1):
                # choose the unique active outgoing arc in incumbent
                nxts = [j for (_, j) in out_arcs[node] if (node, j, k) in X and XV.get((node, j, k), 0.0) > 0.5]
                if not nxts:
                    ok = False
                    break
                j = nxts[0]
                duration += d.get(node, 0.0) + t[node, j]
                node = j
                route.append(node)
                if node == sink:
                    ok = True
                    break
                if node in seen:
                    ok = False
                    break
                seen.add(node)
            pos = {v: i for i, v in enumerate(route)}
            return route, pos, ok, duration



        # ---- 1) SEC separation ---------------------------------------------------
        cut_added = False

        for k in R:
            Ssets = _components_for_cluster(k, XV, X, depot, sink, N)

            # (a) Plain SECs (|S|-1)
            for s in Ssets:
                key = (k, frozenset(s))
                if key in model._sec_seen:
                    continue
                lhs_val = sum(XV.get((i, j, k), 0.0) for i in s for j in s if (i, j, k) in X)
                rhs = len(s) - 1
                if lhs_val > rhs + 1e-9:
                    model._sec_seen.add(key)
                    model.cbLazy(quicksum(X[i, j, k] for i in s for j in s if (i, j, k) in X) <= rhs)
                    cut_added = True

            # (b) Pairing/precedence cut (constraint 16): |S|-2
            # Trigger when a component C contains all pickups of some request r
            # but excludes its delivery; then S = C ∪ {sink}
            for s in Ssets:
                triggers = False
                for r in Pr:  # Pr[r] is pickups list/set; Dr_single[r] is delivery node 
                    if set(Pr[r]).issubset(s) and (Dr_single[r] not in s):
                        triggers = True
                        break
                if not triggers:
                    continue

                S = set(s)
                S.add(sink) 

                key16 = (k, frozenset(S))
                if key16 in model._pairprec_seen:
                    continue

                lhs_val = 0.0
                for i in S:
                    for j in S:
                        if (i, j, k) in X:
                            lhs_val += XV.get((i, j, k), 0.0)

                rhs = len(S) - 2
                if lhs_val > rhs + 1e-9:
                    model._pairprec_seen.add(key16)
                    model.cbLazy(quicksum(X[i, j, k] for i in S for j in S if (i, j, k) in X) <= rhs)
                    cut_added = True


        # ---- 2) Precedence cuts ONLY if NO SEC was added ------------------------
        if not cut_added:
            


            for k in R:
                # active + assigned requests on k
                active = YV.get((pos.get(k), k), 0.0) > 0.5 or any(YV.get((r, k), 0.0) > 0.5 for r in R if (r, k) in Y)
                if not active:
                    continue
                assigned = [r for r in R if (r, k) in Y and YV.get((r, k), 0.0) > 0.5]
                if not assigned:
                    continue

                route, posr, ok, duration = build_route_for_k(k)
                if len(route) < 2:
                    continue

                # -------- (52): depot -> ... -> dr but NOT all pickups before dr
                for r in assigned:

                    dr = Dr_single[r]
                    if dr not in posr:
                        continue
                    idx_d = posr[dr]
                    # pickup missing or appears after dr
                    if not any((p not in posr) or (posr[p] > idx_d) for p in Pr[r]):
                        continue
                    S_nodes = route[1: idx_d]                # strictly between depot and dr
                    if not S_nodes:
                        continue
                    key = ("52", k, dr, tuple(S_nodes))
                    # x(0,S) + x(S ∪ {dr}) <= |S|
                    expr = sum_x({depot}, S_nodes, k) + sum_x_within(S_nodes + [dr], k)
                    add_cut_once(key, expr, len(S_nodes))
                    cut_added = True


                                # -------- (54): delivery -> ... -> pickup  (pickup after delivery)
                for r in assigned:
                    dr = Dr_single[r]
                    if dr not in posr:
                        continue
                    idx_d = posr[dr]
                    # all pickups of r that occur AFTER the delivery
                    for p in (pp for pp in Pr[r] if pp in posr and posr[pp] > idx_d):
                        idx_p = posr[p]
                        S_nodes = route[idx_d + 1: idx_p]     # strictly between dr and p
                        if not S_nodes:
                            continue
                        key = ("54", k, dr, p, tuple(S_nodes))
                        # x(dr,S) + x(S) + x(S,p) <= |S|
                        expr = sum_x({dr}, S_nodes, k) + sum_x_within(S_nodes, k) + sum_x(S_nodes, {p}, k)
                        add_cut_once(key, expr, len(S_nodes))
                        cut_added = True

        # ---- 3) time/scheduling cut (skip if any invalid ordering) ------------
        if not cut_added:
            for k in R:
                assigned = [r for r in R if (r, k) in Y and YV.get((r, k), 0.0) > 0.5]
               
                if not assigned:
                    continue
                
                route, posr, ok, duration = build_route_for_k(k)
                if ok and duration > l[depot] + 1e-6:
                    model.cbLazy(quicksum(Y[(r, k)] for r in assigned) <= len(assigned) - 1)





    def report_objective(m: Model):
        st = m.Status
        print(f"Status: {st}")
        if st == GRB.OPTIMAL:
            print(f"Optimal objective : {m.ObjVal:.6f}")
            print(f"Best bound        : {m.ObjBound:.6f}")
            print(f"MIP gap           : {m.MIPGap:.4%}")
        elif st in (GRB.TIME_LIMIT, GRB.INTERRUPTED):
            if m.SolCount > 0:
                print(f"Best feasible obj : {m.ObjVal:.6f}")
                print(f"Best bound        : {m.ObjBound:.6f}")
                try:
                    print(f"MIP gap           : {m.MIPGap:.4%}")
                except Exception:
                    pass
            else:
                print("No feasible solution found.")
                print(f"Best bound        : {m.ObjBound:.6f}")
        elif st == GRB.CUTOFF:
            print("All solutions are worse than the Cutoff; no incumbent kept.")
            print(f"Best bound (≥ Cutoff): {m.ObjBound:.6f}")
        elif st in (GRB.INFEASIBLE, GRB.UNBOUNDED, GRB.INF_OR_UNBD):
            print("Model is infeasible/unbounded/inf_or_unbd.")
        print(f"Runtime           : {m.Runtime:.2f}s")
        print(f"SolCount          : {m.SolCount}, Nodes: {getattr(m, 'NodeCount', 'n/a')}")
    model.Params.LazyConstraints = 1
    model.optimize(lazy_cb)
    
    report_objective(model)
    print_solution_summary(
        model,
        V_ext,
        R,
        K,
        Pr, Dr,
        X, Y,
        None,
        e, l, q,
        t,
        sink,
        d
    )
    if model.Status == GRB.OPTIMAL:
        print("Optimal Objective Found")
        print(f"Warm Start Obj: {total_cost:.2f}, Actual Obj: {model.ObjVal:.2f}, Gap: {100*abs(total_cost-model.ObjVal)/model.ObjVal:.2f}%")
def main(argv=None):
    path, _ = parse_instance_argv(argv, default_filename="l_4_25_4.txt")
    model = Model("MPDTW")
    Run_Model(str(path), model)


if __name__ == "__main__":
    main()
