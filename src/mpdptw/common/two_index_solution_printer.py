# printer_ver2_robust.py

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

def _x(X, i, j):
    """Safe getter for arc var X[i,j]; returns 0.0 if var doesn't exist."""
    try:
        v = X[i, j]                      # works for dicts/tupledicts if present
    except Exception:
        v = X.get((i, j), 0.0) if hasattr(X, "get") else 0.0
    return _val(v) if v is not None else 0.0

def _s(S, i, default=float("nan")):
    """Safe getter for start-time var S[i]; returns NaN if S is None or missing."""
    if S is None:
        return default
    try:
        v = S[i]
    except Exception:
        try:
            v = S.get(i, default)
        except Exception:
            return default
    return _val(v) if v is not None else default

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

def _succ_from_solution(V_ext, X, sink=None):
    succ, depot_starts = {}, []
    for i in V_ext:
        for j in V_ext:
            if i == j:
                continue
            if _x(X, i, j) > 0.5:
                if i == 0:
                    depot_starts.append(j)
                else:
                    succ[i] = j
    if sink is not None:
        depot_starts = [j for j in depot_starts if j != sink]
    return succ, depot_starts

def _extract_routes(V_ext, X, sink=None):
    succ, starts = _succ_from_solution(V_ext, X, sink=sink)
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
    V_ext,          # nodes incl. start depot (=0) and sink
    R,
    K,
    Pr, Dr,
    X, S=None,      # X[i,j], S[i] (optional)
    e=None, l=None, q=None,
    t=None,
    sink=None,
    Vs=None,
    Rq=None,
    d=None
):
    # default empties for dict-like params that might be None
    e = e or {}
    l = l or {}
    q = q or {}

    print("\n" + "="*76)
    print("SOLUTION SUMMARY (start depot 0, end depot = sink)")
    print("="*76)

    try:
        obj_val = float(model.ObjVal)
    except Exception:
        obj_val = getattr(model, "objective_value", 0.0) or 0.0
    print(f"Objective  : {obj_val:.3f}")

    # Vehicles used: arcs leaving start depot (ignore direct 0->sink if it exists)
    starts = [j for j in V_ext if j != 0 and (sink is None or j != sink) and _x(X, 0, j) > 0.5]
    vehicles_used = len(starts)
    print(f"Vehicles   : {vehicles_used} used / {_k_available(K)} available")

    node_info = _build_node_index(R, Pr, Dr, sink=sink)
    routes = _extract_routes(V_ext, X, sink=sink)

    print("\nROUTES (node, type, req | S within [e,l] | demand q | t(prev->v))")
    for ridx, route in enumerate(routes, 1):
        print("-"*76)
        print(f"Route #{ridx}: " + ' -> '.join(map(str, route)))
        hdr = f"{'node':>5}  {'type':>10}  {'req':>4}  {'S':>10}   {'[e,l]':>13}  {'q':>6}  {'t(prev->v)':>11}"
        print(hdr)
        print("-"*len(hdr))
        total_time = 0.0
        for idx, v in enumerate(route):
            typ, req = node_info.get(v, ("Other", None))
            s_val = _s(S, v)  # NaN if S is None/missing
            e_v = e.get(v, 0.0)
            l_v = l.get(v, 0.0)
            q_v = q.get(v, 0.0)

            # Implied S at depot/sink if desired (only if S present for the relevant nodes)
            if t is not None and v == 0 and idx + 1 < len(route):
                j = route[idx + 1]
                sj = _s(S, j)
                if sj == sj:  # not NaN
                    try:
                        s_val = sj - float(t[(0, j)])
                    except Exception:
                        pass
            if t is not None and sink is not None and v == sink and idx > 0:
                i = route[idx - 1]
                si = _s(S, i)
                if si == si:  # not NaN
                    try:
                        s_val = si + (d[i] if d is not None else 0.0) + float(t[(i, sink)])
                    except Exception:
                        pass

            # Travel time from previous node
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
        max_pick = max((_s(S, p, float('-inf')) for p in Pr[r]), default=float('-inf'))
        s_del = _s(S, dnode)
        ok = "Y" if (s_del == s_del and s_del + 1e-6 >= max_pick) else "NO"
        print(f"{r:>4}  {max_pick:12.2f}  {s_del:10.2f}  {ok:>5}")

    if Vs is not None:
        print("\nVEHICLE → REQUESTS (Vs[r,k] ≈ 1)")
        print("-"*76)
        try:
            for k in K:
                assigned = [r for r in R if _val(Vs[r, k]) > 0.5]
                print(f"Vehicle {k}: {assigned}")
        except TypeError:
            pass

    if Rq is not None:
        print("\nSAME-VEHICLE REQUEST PAIRS (Rq[m,n] ≈ 1, m < n)")
        print("-"*76)
        pairs = [(m, n) for m in R for n in R if m < n and _val(Rq[m, n]) > 0.5]
        print(pairs)

    print("\nCHOSEN ARCS (i -> j) with X[i,j] ≈ 1")
    print("-"*76)
    arcs = [(i, j) for i in V_ext for j in V_ext if i != j and _x(X, i, j) > 0.5]
    print(arcs)

    if t is not None:
        print("\nCHOSEN ARCS WITH TRAVEL TIMES t[i,j]")
        print("-"*76)
        arcs_with_t = []
        for (i, j) in arcs:
            try:
                arcs_with_t.append((i, j, round(float(t[(i, j)]), 1)))
            except Exception:
                arcs_with_t.append((i, j, float("nan")))
        print(arcs_with_t)

    print("="*76)
