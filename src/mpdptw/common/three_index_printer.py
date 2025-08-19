

def _as_delivery_node(d):
    if isinstance(d, (list, tuple, set)):
        return next(iter(d))
    return d

def _val(v):
    if hasattr(v, "X"):       # Gurobi
        return float(v.X)
    try:
        return float(v)
    except Exception:
        return 0.0

def _x3(X, i, j, k):
    """Safe getter for 3-index arc var X[i,j,k]; returns 0.0 if var missing."""
    try:
        v = X[i, j, k]
    except Exception:
        try:
            v = X.get((i, j, k), 0.0) if hasattr(X, "get") else 0.0
        except Exception:
            v = 0.0
    return _val(v) if v is not None else 0.0

def _y(Y, r, k):
    """Safe getter for Y[r,k]; returns 0.0 if var missing."""
    try:
        v = Y[r, k]
    except Exception:
        try:
            v = Y.get((r, k), 0.0) if hasattr(Y, "get") else 0.0
        except Exception:
            v = 0.0
    return _val(v) if v is not None else 0.0

def _build_node_index(R, Pr, Dr, sink=None):
    node_type = {0: ("DepotStart", None)}
    if sink is not None:
        node_type[sink] = ("DepotEnd", None)
    for r in R:
        for p in Pr[r]:
            node_type[p] = ("Pickup", r)
        dnode = _as_delivery_node(Dr[r])
        node_type[dnode] = ("Delivery", r)
    return node_type

def _succ_from_solution_k(V_ext, X, k, sink=None):
    """Successor map & start nodes for a specific vehicle k."""
    succ, starts = {}, []
    for i in V_ext:
        for j in V_ext:
            if i == j:
                continue
            if _x3(X, i, j, k) > 0.5:
                if i == 0:
                    starts.append(j)
                else:
                    succ[i] = j
    if sink is not None:
        starts = [j for j in starts if j != sink]
    return succ, starts

def _extract_routes_k(V_ext, X, k, sink=None):
    succ, starts = _succ_from_solution_k(V_ext, X, k, sink=sink)
    routes, visited = [], set()
    for start in starts:
        route = [0, start]
        visited.add(start)
        cur = start
        while True:
            nxt = succ.get(cur)
            if nxt is None:
                break
            route.append(nxt)
            if (sink is not None and nxt == sink) or (nxt in visited):
                break
            visited.add(nxt)
            cur = nxt
        routes.append(route)
    return routes

def _k_available(K):
    try:
        return len(K)
    except TypeError:
        return int(K)

def print_solution_summary(
    model,
    V_ext,          # nodes including 0 and sink
    R,              # request IDs
    K,              # vehicles
    Pr, Dr,         # pickups/deliveries per request
    X, S,           # X[i,j,k], S[i]
    e, l, q,        # time windows, demand (if any)
    Y=None,         # Y[r,k] (optional but recommended)
    t=None,         # travel times dict for (i,j) (optional)
    sink=None,
    d=None,         # service times (optional)
    show_arcs=True
):
    print("\n" + "="*82)
    print("SOLUTION SUMMARY — 3-INDEX/ARF (start depot 0, end depot = sink)")
    print("="*82)

    try:
        obj_val = float(model.ObjVal)
    except Exception:
        obj_val = getattr(model, "objective_value", 0.0) or 0.0
    print(f"Objective    : {obj_val:.3f}")

    # Vehicles used: arcs 0->j for any k (ignoring 0->sink)
    vehicles_used = 0
    used_by_k = {}
    for k in K:
        starts_k = [j for j in V_ext if j != 0 and (sink is None or j != sink) and _x3(X, 0, j, k) > 0.5]
        if starts_k:
            vehicles_used += 1
            used_by_k[k] = starts_k
    print(f"Vehicles     : {vehicles_used} used / {_k_available(K)} available")

    if Y is not None:
        print("\nVEHICLE → REQUESTS (Y[r,k] ≈ 1)")
        print("-"*82)
        for k in K:
            assigned = [r for r in R if _y(Y, r, k) > 0.5]
            print(f"Vehicle {k}: {assigned}")

    node_info = _build_node_index(R, Pr, Dr, sink=sink)

    # Per-vehicle routes
    print("\nROUTES PER VEHICLE (node, type, req | S within [e,l] | demand q | t(prev->v))")
    for k in K:
        routes_k = _extract_routes_k(V_ext, X, k, sink=sink)
        if not routes_k:
            continue
        print("-"*82)
        print(f"Vehicle {k}:")
        for ridx, route in enumerate(routes_k, 1):
            print("-"*82)
            print(f"  Route : " + ' -> '.join(map(str, route)))
            hdr = f"{'node':>7}  {'type':>10}  {'req':>4}  {'S':>10}   {'[e,l]':>13}  {'q':>7}  {'t(prev->v)':>11}"
            print("  " + hdr)
            print("  " + "-"*len(hdr))
            total_time = 0.0
            for idx, v in enumerate(route):
                typ, req = node_info.get(v, ("Other", None))
                s_val = _val(S[v]) if v in S else float("nan")
                e_v = e.get(v, 0.0)
                l_v = l.get(v, 0.0)
                q_v = q.get(v, 0.0)

                # inferred S at depot/sink if helpful
                if t is not None and v == 0 and idx + 1 < len(route):
                    j = route[idx + 1]
                    try:
                        s_val = _val(S[j]) - float(t[(0, j)])
                    except Exception:
                        pass
                if t is not None and sink is not None and v == sink and idx > 0:
                    i = route[idx - 1]
                    try:
                        s_val = _val(S[i]) + (d[i] if (d is not None and i in d) else 0.0) + float(t[(i, sink)])
                    except Exception:
                        pass

                # travel time from previous node
                if t is not None and idx > 0:
                    i = route[idx - 1]
                    try:
                        tij = float(t[(i, v)])
                    except Exception:
                        tij = float("nan")
                    tprev_str = f"{tij:.2f}" if tij == tij else "-"
                    if tij == tij:
                        total_time += tij
                else:
                    tprev_str = "-"

                req_str = "-" if req is None else str(req)
                el_str = f"[{e_v:.0f},{l_v:.0f}]"
                print(f"  {v:>7}  {typ:>10}  {req_str:>4}  {s_val:10.2f}   {el_str:>13}  {q_v:7.2f}  {tprev_str:>11}")
            print(f"{'':>64}Total travel: {round(total_time, 2)}")

    # Request-level precedence/timing check
    print("\nREQUEST TIMING (max pickup start ≤ delivery start)")
    print("-"*82)
    print(f"{'req':>4}  {'max S[pick]':>12}  {'S[del]':>10}  {'ok?':>5}")
    for r in R:
        dnode = _as_delivery_node(Dr[r])
        max_pick = max((_val(S[p]) for p in Pr[r]), default=float('-inf'))
        s_del = _val(S[dnode]) if dnode in S else float("nan")
        ok = "Y" if (s_del == s_del and s_del + 1e-6 >= max_pick) else "NO"
        print(f"{r:>4}  {max_pick:12.2f}  {s_del:10.2f}  {ok:>5}")

    # Chosen arcs (with vehicle)
    if show_arcs:
        print("\nCHOSEN ARCS (i -> j, k) with X[i,j,k] ≈ 1")
        print("-"*82)
        arcs = [(i, j, k) for i in V_ext for j in V_ext for k in K
                if i != j and _x3(X, i, j, k) > 0.5]
        print(arcs)

        if t is not None:
            print("\nCHOSEN ARCS WITH TRAVEL TIMES t[i,j]")
            print("-"*82)
            arcs_with_t = []
            for (i, j, k) in arcs:
                try:
                    arcs_with_t.append((i, j, k, round(float(t[(i, j)]), 1)))
                except Exception:
                    arcs_with_t.append((i, j, k, float("nan")))
            print(arcs_with_t)

    print("="*82)
