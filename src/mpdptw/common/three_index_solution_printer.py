# printer_ver3_three_index.py

def _as_delivery_node(d):
    if isinstance(d, (list, tuple, set)):
        return next(iter(d))
    return d

def _val(v):
    if hasattr(v, "X"):       # Gurobi variable
        return float(v.X)
    try:
        return float(v)
    except Exception:
        return 0.0

def _x(X, i, j, k=None):
    """Safe getter for arc var X[i,j,k]; returns 0.0 if var doesn't exist."""
    key = (i, j, k) if k is not None else (i, j)
    try:
        v = X[key]
    except Exception:
        v = X.get(key, 0.0) if hasattr(X, "get") else 0.0
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

def _succ_from_solution_3index(V_ext, K, X, sink=None):
    succ_map, depot_starts = {}, {}
    for k in K:
        succ = {}
        starts = []
        for i in V_ext:
            for j in V_ext:
                if i == j:
                    continue
                if _x(X, i, j, k) > 0.5:
                    if i == 0:
                        starts.append(j)
                    else:
                        succ[i] = j
        if sink is not None:
            starts = [j for j in starts if j != sink]
        succ_map[k] = succ
        depot_starts[k] = starts
    return succ_map, depot_starts

def _extract_routes_3index(V_ext, K, X, sink=None):
    succ_map, depot_starts = _succ_from_solution_3index(V_ext, K, X, sink)
    all_routes = {}
    for k in K:
        succ = succ_map[k]
        starts = depot_starts[k]
        routes = []
        visited = set()
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
        all_routes[k] = routes
    return all_routes

def _k_available(K):
    try:
        return len(K)
    except TypeError:
        return int(K)

def print_solution_summary(
    model,
    V_ext, R, K,
    Pr, Dr,
    X, S,           # X[i,j,k], S[i]
    e, l, q,
    t=None,
    sink=None,
    Vs=None,
    Rq=None,
    d=None
):
    print("\n" + "="*76)
    print("SOLUTION SUMMARY (start depot 0, end depot = sink)")
    print("="*76)

    try:
        obj_val = float(model.ObjVal)
    except Exception:
        obj_val = getattr(model, "objective_value", 0.0) or 0.0
    print(f"Objective  : {obj_val:.3f}")

    vehicles_used = sum(
        any(_x(X, 0, j, k) > 0.5 for j in V_ext if j != 0 and (sink is None or j != sink))
        for k in K
    )
    print(f"Vehicles   : {vehicles_used} used / {_k_available(K)} available")

    node_info = _build_node_index(R, Pr, Dr, sink=sink)
    routes_by_vehicle = _extract_routes_3index(V_ext, K, X, sink=sink)

    print("\nROUTES (node, type, req | S within [e,l] | demand q | t(prev->v))")
    for k in K:
        vehicle_routes = routes_by_vehicle.get(k, [])
        for ridx, route in enumerate(vehicle_routes, 1):
            print("-"*76)
            print(f"Vehicle {k} — Route #{ridx}: " + ' -> '.join(map(str, route)))
            hdr = f"{'node':>5}  {'type':>10}  {'req':>4}  {'S':>10}   {'[e,l]':>13}  {'q':>6}  {'t(prev->v)':>11}"
            print(hdr)
            print("-"*len(hdr))
            total_time = 0.0
            for idx, v in enumerate(route):
                typ, req = node_info.get(v, ("Other", None))
                s_val = _val(S[v]) if v in S else float("nan")
                e_v = e.get(v, 0.0)
                l_v = l.get(v, 0.0)
                q_v = q.get(v, 0.0)

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
        max_pick = max((_val(S[p]) for p in Pr[r]), default=float('-inf'))
        s_del = _val(S[dnode]) if dnode in S else float("nan")
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

    print("\nCHOSEN ARCS (i -> j, vehicle k) with X[i,j,k] ≈ 1")
    print("-"*76)
    arcs = [(i, j, k) for k in K for i in V_ext for j in V_ext if i != j and _x(X, i, j, k) > 0.5]
    print(arcs)

    if t is not None:
        print("\nCHOSEN ARCS WITH VEHICLE k WITH TRAVEL TIMES t[i,j]")
        print("-"*76)
        arcs_with_t = []
        for (i, j, k) in arcs:
            try:
                arcs_with_t.append((i, j, k, round(float(t[(i, j)]), 1)))
            except Exception:
                arcs_with_t.append((i, j, k, float("nan")))
        print(arcs_with_t)

    print("="*76)
