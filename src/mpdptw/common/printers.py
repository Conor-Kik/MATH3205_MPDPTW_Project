# printers.py

def _as_delivery_node(d):
    if isinstance(d, (list, tuple, set)):
        return next(iter(d))
    return d

def _build_node_index(R, Pr, Dr):
    node_type = {0: ("Depot", None)}
    for r in R:
        for p in Pr[r]:
            node_type[p] = ("Pickup", r)
        d = _as_delivery_node(Dr[r])
        node_type[d] = ("Delivery", r)
    return node_type

def _succ_from_solution(V, X):
    succ, depot_starts = {}, []
    for i in V:
        for j in V:
            if i != j and X[i, j].X > 0.5:
                if i == 0:
                    depot_starts.append(j)
                else:
                    succ[i] = j
    return succ, depot_starts

def _extract_routes(V, X):
    succ, starts = _succ_from_solution(V, X)
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
            if nxt == 0 or nxt in visited:
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
    model, V, N, R, K, Pr, Dr, X, S, C, e, l, q, t=None, Vs=None, Rq=None, d=None
):
    print("\n" + "="*72)
    print("SOLUTION SUMMARY")
    print("="*72)

    print(f"Objective  : {model.ObjVal:.3f}")

    vehicles_used = sum(1 for j in N if X[0, j].X > 0.5)
    print(f"Vehicles   : {vehicles_used} used / {_k_available(K)} available")

    node_info = _build_node_index(R, Pr, Dr)
    routes = _extract_routes(V, X)

    print("\nROUTES (node, type, req | S within [e,l] | load C | demand q | t(prev->v))")
    for ridx, route in enumerate(routes, 1):
        print("-"*72)
        print(f"Route #{ridx}: " + ' -> '.join(map(str, route)))
        hdr = f"{'node':>5}  {'type':>8}  {'req':>4}  {'S':>10}   {'[e,l]':>13}  {'C':>8}  {'q':>6}  {'t(prev->v)':>11}"
        print(hdr)
        print("-"*len(hdr))
        total_time = 0
        for idx, v in enumerate(route):
            typ, req = node_info.get(v, ("Other", None))
            s_val = S[v].X
            c_val = C[v].X
            e_v = e[v]
            l_v = l[v]
            q_v = q.get(v, 0)

            # implied depot times using adjacent arcs
            if v == 0 and t is not None:
                if idx + 1 < len(route):
                    j = route[idx + 1]
                    s_val = round(S[j].X - t[0, j])
                if idx > 0:
                    i = route[idx - 1]
                    s_val = S[i].X + t[i, 0] + d[i] if d is not None else S[i].X + t[i, 0]

            # travel time from previous node to current node (if exists)
            if t is not None and idx > 0:
                i = route[idx - 1]
                try:
                    tij = t[i, v]
                    
                except KeyError:
                    tij = float("nan")
                tprev_str = f"{tij:.2f}"
                total_time += tij
            else:
                tprev_str = "-"

            req_str = "-" if req is None else str(req)
            el_str = f"[{e_v:.0f},{l_v:.0f}]"
            print(f"{v:>5}  {typ:>8}  {req_str:>4}  {s_val:10.2f}   {el_str:>13}  {c_val:8.2f}  {q_v:6.2f}  {tprev_str:>11}")
        print(f"                                                              Total:      {round(total_time,2)}")
    print("\nREQUEST TIMING (max pickup start ≤ delivery start)")
    print("-"*72)
    print(f"{'req':>4}  {'max S[pick]':>12}  {'S[del]':>10}  {'ok?':>5}")
    for r in R:
        dnode = _as_delivery_node(Dr[r])
        max_pick = max(S[p].X for p in Pr[r]) if Pr[r] else float('-inf')
        s_del = S[dnode].X
        ok = "Y" if s_del + 1e-6 >= max_pick else "NO"
        print(f"{r:>4}  {max_pick:12.2f}  {s_del:10.2f}  {ok:>5}")

    if Vs is not None:
        print("\nVEHICLE → REQUESTS (Vs[r,k] ≈ 1)")
        print("-"*72)
        try:
            for k in K:
                assigned = [r for r in R if Vs[r, k].X > 0.5]
                print(f"Vehicle {k}: {assigned}")
        except TypeError:
            pass

    if Rq is not None:
        print("\nSAME-VEHICLE REQUEST PAIRS (Rq[m,n] ≈ 1, m < n)")
        print("-"*72)
        pairs = [(m, n) for m in R for n in R if m < n and Rq[m, n].X > 0.5]
        print(pairs)

    print("\nCHOSEN ARCS (i -> j) with X[i,j] ≈ 1")
    print("-"*72)
    arcs = [(i, j) for i in V for j in V if i != j and X[i, j].X > 0.5]
    print(arcs)

    if t is not None:
        print("\nCHOSEN ARCS WITH TRAVEL TIMES t[i,j]")
        print("-"*72)
        arcs_with_t = []
        for (i, j) in arcs:
            try:
                arcs_with_t.append((i, j, round(float(t[i, j]), 1)))
            except KeyError:
                arcs_with_t.append((i, j, float("nan")))
        print(arcs_with_t)

    print("="*72)
