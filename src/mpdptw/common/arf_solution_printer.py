

def _as_delivery_node(d):
    if isinstance(d, (list, tuple, set)):
        return next(iter(d))
    return d

def _val(v):
    if hasattr(v, "X"):  # Gurobi Var
        return float(v.X)
    try:
        return float(v)
    except Exception:
        return 0.0

def _x3(X, i, j, k):
    """Safe getter for arc var X[i,j,k]; returns 0.0 if var doesn't exist."""
    try:
        v = X[i, j, k]
    except Exception:
        v = X.get((i, j, k), 0.0) if hasattr(X, "get") else 0.0
    return _val(v) if v is not None else 0.0

def _y(Y, r, k):
    try:
        v = Y[r, k]
    except Exception:
        v = Y.get((r, k), 0.0) if hasattr(Y, "get") else 0.0
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

def _extract_routes_arf(V_ext, X, Kset, sink=None):
    """
    For each k in Kset, build succ_k and a single route starting at the unique j with X[0,j,k]≈1.
    Returns: dict k -> route list [0, j, ..., sink?]
    """
    routes = {}
    for k in Kset:
        succ = {}
        starts = []
        for i in V_ext:
            for j in V_ext:
                if i == j:
                    continue
                if _x3(X, i, j, k) > 0.5:
                    if i == 0:
                        starts.append(j)
                    else:
                        succ[i] = j
        # keep starts excluding sink
        starts = [j for j in starts if sink is None or j != sink]
        if not starts:
            continue
        start = starts[0]  # at most one per k by your model; if multiple, we take the first
        route = [0, start]
        seen = set([start])
        cur = start
        while True:
            nxt = succ.get(cur)
            if nxt is None:
                break
            route.append(nxt)
            if (sink is not None and nxt == sink) or (nxt in seen):
                break
            seen.add(nxt)
            cur = nxt
        routes[k] = route
    return routes

def print_solution_summary(
    model,
    V_ext,          # nodes incl. start depot (=0) and sink
    R,              # request ids (also cluster ids k)
    K_available,    # available vehicle count or iterable of vehicles (for info)
    Pr, Dr,
    X, Y,           # 3-index X[i,j,k], 2-index Y[r,k]
    S,              # S[i]
    e, l, q,
    t=None,
    sink=None,
    d=None
):
    print("\n" + "="*76)
    print("SOLUTION SUMMARY (ARF: 3-index arcs per request-cluster k)")
    print("="*76)

    try:
        obj_val = float(model.ObjVal)
    except Exception:
        obj_val = getattr(model, "objective_value", 0.0) or 0.0
    print(f"Objective  : {obj_val:.3f}")

    # Which k are active? those with 0->j used
    active_k = []
    for k in R:
        used = any(_x3(X, 0, j, k) > 0.5 for j in V_ext if j != 0 and (sink is None or j != sink))
        if used:
            active_k.append(k)

    # vehicles available
    try:
        k_avail = len(K_available)
    except TypeError:
        k_avail = int(K_available)
    print(f"Clusters/vehicles used: {len(active_k)} / {k_avail}")

    node_info = _build_node_index(R, Pr, Dr, sink=sink)
    routes_by_k = _extract_routes_arf(V_ext, X, R, sink=sink)

    # Request → cluster assignment via Y
    assign = {r: [k for k in R if _y(Y, r, k) > 0.5] for r in R}

    print("\nASSIGNMENTS (request -> cluster k)")
    print("-"*76)
    for r in R:
        ks = assign[r]
        print(f"req {r}: {ks}")

    print("\nROUTES per active k (node, type, req | S within [e,l] | q | t(prev->v))")
    for k in active_k:
        route = routes_by_k.get(k, None)
        print("-"*76)
        print(f"Cluster k={k}: " + (" -> ".join(map(str, route)) if route else "(no route)"))
        if not route:
            continue
        hdr = f"{'node':>5}  {'type':>10}  {'req':>4}  {'S':>10}   {'[e,l]':>13}  {'q':>6}  {'t(prev->v)':>11}"
        print(hdr)
        print("-"*len(hdr))
        total_time = 0.0
        for idx, v in enumerate(route):
            typ, req = node_info.get(v, ("Other", None))
            s_val = _val(S.get(v, float("nan")))
            e_v = e.get(v, 0.0)
            l_v = l.get(v, 0.0)
            q_v = q.get(v, 0.0)

            # infer S at depot/sink if desired
            if t is not None and v == 0 and idx + 1 < len(route):
                j = route[idx + 1]
                try:
                    s_val = _val(S[j]) - float(t[(0, j)])
                except Exception:
                    pass
            if t is not None and sink is not None and v == sink and idx > 0:
                i = route[idx - 1]
                try:
                    s_val = _val(S[i]) + (d[i] if d is not None else 0.0) + float(t[(i, sink)])
                except Exception:
                    pass

            # travel time from previous
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
            print(f"{v:>5}  {typ:>10}  {req_str:>4}  {s_val:10.2f}   {el_str:>13}  {q_v:6.2f}  {tprev_str:>11}")
        print(f"{'':>56}Total:      {round(total_time, 2)}")

    print("\nREQUEST TIMING (max pickup start ≤ delivery start)")
    print("-"*76)
    print(f"{'req':>4}  {'max S[pick]':>12}  {'S[del]':>10}  {'ok?':>5}")
    for r in R:
        dnode = _as_delivery_node(Dr[r])
        max_pick = max((_val(S.get(p, float("nan"))) for p in Pr[r]), default=float('-inf'))
        s_del = _val(S.get(dnode, float("nan")))
        ok = "Y" if (s_del == s_del and s_del + 1e-6 >= max_pick) else "NO"
        print(f"{r:>4}  {max_pick:12.2f}  {s_del:10.2f}  {ok:>5}")

    print("\nCHOSEN ARCS (i -> j, k) with X[i,j,k] ≈ 1")
    print("-"*76)
    arcs = []
    for k in active_k:
        for i in V_ext:
            for j in V_ext:
                if i != j and _x3(X, i, j, k) > 0.5:
                    arcs.append((i, j, k))
    print(arcs)

    if t is not None:
        print("\nCHOSEN ARCS WITH TRAVEL TIMES t[i,j] (per k)")
        print("-"*76)
        arcs_with_t = []
        for (i, j, k) in arcs:
            try:
                tij = round(float(t[(i, j)]), 1)
            except Exception:
                tij = float("nan")
            arcs_with_t.append((i, j, k, tij))
        print(arcs_with_t)

    print("="*76)
