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
PLOT_CLUSTERS = 1
def Run_Model(path, model: Model):
    inst = build_milp_data(str(path))
    cluster_model = Model("Optimal Cluster Assignment")

    # Sets / data
    V      = inst["V_ext"]
    A      = inst["A_feasible_ext"]
    N      = inst["N"]
    R      = inst["R"]
    Pr     = inst["Pr"]
    Dr     = inst["Dr"]
    Dr_single = inst["Dr_single"]

    K      = inst["K"]
    nodes_to_reqs = inst["Nodes_To_Reqs"]
    e      = inst["e"]
    l      = inst["l"]
    d      = inst["d"]
    q      = inst["q"]
    t      = inst["t_ext"]
    c      = inst["c_ext"]
    V_ext  = inst["V_ext"]

    depot  = inst["depot"]
    sink   = inst["sink"]

    # Adjacency lists
    in_arcs  = {j: [] for j in V}
    out_arcs = {i: [] for i in V}
    for (i, j) in A:
        out_arcs[i].append((i, j))
        in_arcs[j].append((i, j))

    # Node→request map
    inverse_request_map = {}
    for r in R:
        for i in Pr[r] + Dr[r]:
            inverse_request_map[i] = r

    def req_of(i):
        return inverse_request_map.get(i, None)
    def allowed_in_cluster(i, k):
        ri = req_of(i)
        return True if ri is None else (rank[ri] >= k)


    # Warm-start with ALNS
    init_sol, total_cost = warm_start_solution(inst, plot_clusters=PLOT_CLUSTERS)
    model.setParam("Cutoff", total_cost)

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

    # Warm-start X/Y from heuristic routes, respecting rank-domain
    route_req = {}
    for (i, j), k_lbl in init_sol.items():
        ri = nodes_to_reqs.get(i)
        rj = nodes_to_reqs.get(j)
        s = route_req.setdefault(k_lbl, set())
        if ri is not None: s.add(ri)
        if rj is not None: s.add(rj)

    label_to_khat = {}
    taken_k = set()
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

    for v in X.values(): v.Start = 0.0
    for v in Y.values(): v.Start = 0.0

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

    # Delivery can only leave if all pickups of its request leave (per cluster k)
    Closure_DelivOutBound = {}
    for r in R:
        dl = Dr_single[r]
        for k in R:
            deliv_out = quicksum(X[dl, j, k] for (_, j) in out_arcs[dl] if (dl, j, k) in X)
            pickups_out = quicksum(
                X[p, j, k] for p in Pr[r] for (_, j) in out_arcs[p] if (p, j, k) in X
            )
            if deliv_out.size() > 0:
                Closure_DelivOutBound[(r, k)] = model.addConstr(
                    deliv_out <= pickups_out,
                    name=f"clos_del_out_bound[r={r},k={k}]"
                )

    # Degree balance on customer nodes i at each k
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

    # Exactly one start/end per active cluster position k
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

    DepotCluster = {
        k: model.addConstr(
            quicksum(X[(depot, j, k)] for (_, j) in out_arcs[depot] if (depot, j, k) in X) <= 1
        )
        for k in R
    }

    # Each request assigned exactly once
    RequestAssigned = {
        r: model.addConstr(quicksum(Y[r, k] for k in R if (r, k) in Y) == 1)
        for r in R
    }

    # Number of active clusters bounded by |K|
    VehicleLimit = model.addConstr(
        quicksum(Y[pos[k], k] for k in R if (pos[k], k) in Y) <= len(K)
    )

    # ---------- Lazy callback helpers ----------

    def _components_for_cluster(k, xvals):
        adj = defaultdict(set)
        nodes = set()
        for (i, j, kk), _ in X.items():
            if kk != k or xvals[i, j, k] < 0.5:
                continue
            if i in (depot, sink) or j in (depot, sink):
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
                seen.add(u)
                comp.add(u)
                for w in adj[u]:
                    if w not in seen:
                        stack.append(w)
            if len(comp) >= 2:
                Sset.append(comp)
        return Sset

    def _stronger_rhs(s):
        stronger = any(all(p in s for p in Pr[r]) and (Dr_single[r] not in s) for r in R)
        return (len(s) - 2) if stronger else (len(s) - 1)

    def _route_for_cluster(k, XV):
        nxt = {}
        for (i, j, kk) in X.keys():
            if kk == k and XV[i, j, k] > 0.5:
                nxt[i] = j
        route, seen = [depot], {depot}
        cur = depot
        max_steps = len(nxt) + 5
        while cur in nxt and len(route) < max_steps:
            cur = nxt[cur]
            if cur in seen:
                break
            route.append(cur); seen.add(cur)
        return route

    model._sec_seen  = set()
    model._prec_seen = set()
    

    def lazy_cb(model, where):
        if where != GRB.Callback.MIPSOL:
            return

        XV = model.cbGetSolution(X)
        YV = model.cbGetSolution(Y)

        # (1) Subtour elimination with stronger RHS
        for k in R:
            Sset = _components_for_cluster(k, XV)
            for s in Sset:
                rhs = _stronger_rhs(s)
                lhs_vars = [X[i, j, k] for i in s for j in s if (i, j, k) in X]
                if not lhs_vars:
                    continue
                lhs_val = sum(XV[i, j, k] for i in s for j in s if (i, j, k) in X)
                if lhs_val <= rhs + 1e-9:
                    continue
                key = (k, frozenset(s), rhs)
                if key in model._sec_seen:
                    continue
                model._sec_seen.add(key)
                model.cbLazy(quicksum(lhs_vars) <= rhs)

        # (2) Precedence: forbid incumbent's entering-arc into a delivery that appears before its pickups
        for k in R:
            route = _route_for_cluster(k, XV)
            pos_route = {v: i for i, v in enumerate(route)}  # avoid shadowing outer 'pos'
            for r in R:
                delivery = Dr_single[r]
                if delivery not in pos_route:
                    continue
                idx_d = pos_route[delivery]
                prefix = set(route[1:idx_d])  # nodes before delivery (excluding depot)
                if all(p in prefix for p in Pr[r]):
                    continue
                pred = route[idx_d - 1]
                if (pred, delivery, k) in X and (k, pred, delivery) not in model._prec_seen:
                    model._prec_seen.add((k, pred, delivery))
                    model.cbLazy(X[pred, delivery, k] <= 0)

        # (3) Optional route-duration pruning for active clusters
        for k in R:
            active = YV.get((pos.get(k), k), 0.0) > 0.5 or any(
                YV.get((r, k), 0.0) > 0.5 for r in R if (r, k) in Y
            )
            if not active:
                continue

            assigned = [r for r in R if (r, k) in Y and YV[(r, k)] > 0.5]
            if not assigned:
                continue

            node, seen, duration, ok = depot, {depot}, 0.0, False
            for _ in range(10000):
                candidates = [
                    j for (_, j) in out_arcs[node]
                    if (node, j, k) in X and XV.get((node, j, k), 0.0) > 0.5
                ]
                if not candidates:
                    ok = False
                    break
                j = candidates[0]
                duration += d.get(node, 0.0) + t[node, j]
                node = j
                if node == sink:
                    ok = True
                    break
                if node in seen:
                    ok = False
                    break
                seen.add(node)

            if ok and duration > l[depot] + 1e-6:
                model.cbLazy(quicksum(Y[(r, k)] for r in assigned) <= len(assigned) - 1)

    # Solve
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


    model.setParam("LazyConstraints", 1)
    model.setParam("Presolve", 0)
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
        d=d
    )




def main(argv=None):
    path, _ = parse_instance_argv(argv, default_filename="l_4_25_4.txt")
    model = Model("MPDTW")
    Run_Model(str(path), model)


if __name__ == "__main__":
    main()
