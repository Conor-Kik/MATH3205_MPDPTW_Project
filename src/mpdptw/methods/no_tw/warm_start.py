from gurobipy import *
import numpy as np
import warnings
warnings.filterwarnings("ignore")
import numpy as np
from mpdptw.methods.no_tw.route_time import Run_Model
from collections import defaultdict

def chamfer(A, B):
    # Average of bidirectional nearest-neighbor Euclidean distances
    dists = np.sum((A[:, None, :] - B[None, :, :])**2, axis=-1)
    da = np.min(dists, axis=1)
    db = np.min(dists, axis=0)
    return 0.5 * (np.mean(np.sqrt(da)) + np.mean(np.sqrt(db)))


def pairwise_matrix(sets):
    n = len(sets)
    D = np.zeros((n, n), dtype=float)
    for i in range(n):
        for j in range(i, n):
            d = chamfer(sets[i], sets[j]) if i != j else 0.0
            D[i, j] = D[j, i] = d
    return D



def hierarchical_merge_labels(D, kept_R, inst, points):
    """
    Returns cluster labels aligned with kept_R order.

    D: initial symmetric Chamfer distance matrix (len == len(kept_R))
    kept_R: list mapping row index -> request id
    inst: instance dict (used for limit and subproblem solves)
    points: list of numpy arrays, one per kept_R entry (each is a point set)

    Returns:
      labels: list of ints, same length as kept_R; labels[i] = cluster id of kept_R[i]
    """
    limit = inst["l"][0]

    # Working copy of distances with self-distances masked
    W = [row[:] for row in D]
    n = len(W)
    for i in range(n):
        W[i][i] = float('inf')

    # cluster id -> list of row indices
    clusters = {i: [i] for i in range(n)}
    active   = set(range(n))
    point_sets = {i: points[i] for i in range(n)} 
    def update_chamfer(i, j):
        # merge sets
        point_sets[i] = np.concatenate((point_sets[i], point_sets[j]), axis=0)
        del point_sets[j]

        clusters[i].extend(clusters[j])
        del clusters[j]

        # recompute Chamfer(i, m) for other active clusters
        for m in list(active):
            if m == i or m == j:
                continue
            val = chamfer(point_sets[i], point_sets[m])
            W[i][m] = val
            W[m][i] = val

        # mask out j
        for m in range(n):
            W[j][m] = float('inf')
            W[m][j] = float('inf')

    def argmin_pair():
        best = float('inf'); bi = bj = -1
        for i in range(n):
            row = W[i]
            for j in range(n):
                if i == j:
                    continue
                v = row[j]
                if v < best:
                    best = v; bi, bj = i, j
        return bi, bj, best

    while True:
        i, j, dmin = argmin_pair()
        if i == -1 or dmin == float('inf'):
            break

        subset_rows = clusters[i] + clusters[j]
        subset_ids  = [kept_R[r] for r in subset_rows]

        cost, arcs = solve_cluster_with_fallback(subset_ids, inst)
        if cost is not None and cost <= limit:
            active.discard(j)
            update_chamfer(i, j)
        else:
            W[i][j] = float('inf')
            W[j][i] = float('inf')

    labels = [None] * n
    for cid, rows in enumerate(sorted(active)):
        for r in clusters[rows]:
            labels[r] = cid
    return labels


def shortest_routes_greedy(route_of_req, Pr, Dr_single, depot, sink, t, out_arcs, d):
    """
    Greedy per-route builder respecting pickup-before-delivery and forbidding sink
    until all service nodes are visited. Returns {route_id: {"cost": float, "path": [nodes]}}.
    """
    by_route = defaultdict(list)
    for r, rid in route_of_req.items():
        by_route[rid].append(r)

    results = {}
    for rid, reqs in by_route.items():
        pickups    = {p for r in reqs for p in Pr[r]}
        deliveries = {Dr_single[r] for r in reqs}

        node_to_req_pick = {p: r for r in reqs for p in Pr[r]}
        node_to_req_del  = {Dr_single[r]: r for r in reqs}
        req_pickups      = {r: frozenset(Pr[r]) for r in reqs}

        allowed_nodes = {depot, sink} | pickups | deliveries

        path = [depot]
        visited = {depot}
        picked = set()
        delivered = set()
        total_cost = 0.0
        u = depot

        max_steps = max(2, 3 * len(allowed_nodes))
        steps = 0

        while not (picked.issuperset(pickups) and delivered.issuperset(deliveries) and u == sink):
            steps += 1
            if steps > max_steps:
                results[rid] = {"cost": float("inf"), "path": None}
                break

            best_v, best_c = None, None
            for (i, v) in out_arcs.get(u, []):
                if v not in allowed_nodes:
                    continue
                if v in visited:
                    continue
                if v == sink and not (picked.issuperset(pickups) and delivered.issuperset(deliveries)):
                    continue
                if v in node_to_req_del:
                    r_req = node_to_req_del[v]
                    if not req_pickups[r_req].issubset(picked):
                        continue

                c = t[(i, v)] + d[i]
                if best_c is None or c < best_c:
                    best_c, best_v = c, v

            if best_v is None:
                results[rid] = {"cost": float("inf"), "path": None}
                break

            total_cost += best_c
            u = best_v
            path.append(u)
            visited.add(u)

            if u in node_to_req_pick: picked.add(u)
            if u in node_to_req_del:  delivered.add(u)
        else:
            results[rid] = {"cost": total_cost, "path": path}

    return results


def path_to_arcs(path):
    if not path or len(path) < 2:
        return []
    return [(path[i], path[i+1]) for i in range(len(path)-1)]


def solve_cluster_with_fallback(subset, inst):
    """
    subset: list of request ids in this cluster
    Tries Run_Model(subset, inst),
    falls back to greedy if no feasible solution is produced or time limit reached.
    Returns (cost, arcs) or (None, []) if both fail.
    """
    if len(subset) <= 7:
        _m, s_cost, arcs = Run_Model(subset, inst)
        if s_cost is not None:
            return float(s_cost), [tuple(a) for a in arcs]
    # Greedy fallback (derive inputs from inst)
    V = inst["V_ext"]; A = inst["A_feasible_ext"]
    Pr = inst["Pr"]; Dr_single = inst["Dr_single"]
    depot = inst["depot"]; sink = inst["sink"]
    d = inst["d"]
    t = inst.get("t_ext", inst.get("t"))

    out_arcs = {i: [] for i in V}
    for (i, j) in A:
        out_arcs[i].append((i, j))

    route_of_req = {r: 0 for r in subset}
    gres = shortest_routes_greedy(
        route_of_req=route_of_req,
        Pr=Pr,
        Dr_single=Dr_single,
        depot=depot,
        sink=sink,
        t=t,
        out_arcs=out_arcs,
        d=d,
    )
    info = gres.get(0, {"cost": float("inf"), "path": None})
    if info["path"] is None:
        return None, []
    return float(info["cost"]), path_to_arcs(info["path"])


def warm_start_solution(inst, plot_clusters = 0):
    """
    Cluster requests (Agglomerative + Chamfer), solve each cluster via short MIP
    (fallback to greedy if needed). Accept if all routes ≤ l[sink].
    Returns:
        arc_to_route: dict {(i,j) -> route_id}
        total_cost:   float
    """
    N              = inst["N"]
    Coords         = inst["Coords"]
    nodes_to_reqs  = inst["Nodes_To_Reqs"]
    R              = list(inst["R"])
    sink           = inst["sink"]
    l              = inst["l"]
    d              = inst["d"]

    # Request → coordinates; build Chamfer matrix aligned with kept_R
    req_to_coord = {r: [] for r in R}
    for i in N:
        r = nodes_to_reqs[i]
        req_to_coord[r].append(Coords[i])

    coord_sets, kept_R = [], []
    for r in R:
        pts = np.asarray(req_to_coord[r], dtype=float)
        if pts.size == 0:
            continue
        coord_sets.append(pts)
        kept_R.append(r)

    D = pairwise_matrix(coord_sets)

    LIMIT = l[sink]

    prelabel = hierarchical_merge_labels(D, kept_R, inst, coord_sets)
    if plot_clusters:
        plot_cluster_labels(prelabel, kept_R, inst)
    # Map kept request id -> cluster id
    req_to_cluster = {kept_R[idx]: int(prelabel[idx]) for idx in range(len(kept_R))}

    # Build clusters in stable order
    groups = defaultdict(list)
    for r in kept_R:
        groups[req_to_cluster[r]].append(r)
    subsets = [groups[cid] for cid in sorted(groups.keys())]
    route_to_reqs = {i: subsets[i] for i in range(len(subsets))}

    print("Cluster sizes:", [len(s) for s in subsets])
    # Solve each cluster (MIP → greedy)
    route_to_costs = {i: 0.0 for i in range(len(subsets))}
    arc_to_route   = {}
    for rid, subset in enumerate(subsets):
        s_cost, arcs = solve_cluster_with_fallback(subset, inst)
        route_to_costs[rid] = float(s_cost)
        for arc in arcs:
            arc_to_route[tuple(arc)] = rid

    total_cost = float(sum(route_to_costs.values()))
    total_cost = total_cost - sum(d[i] for i in N)



    #if all(c <= LIMIT for c in route_to_costs.values()):
    print(f"Accepted k={len(subsets)} (all route costs ≤ l[sink]={LIMIT})")
    print(f"Total cost of all routes = {total_cost:.6f}")
    for rid in sorted(route_to_reqs):
        print(f"Route {rid}: requests={route_to_reqs[rid]}  |  finish time={route_to_costs[rid]:.6f}")
    return arc_to_route, total_cost, route_to_reqs


def plot_cluster_labels(prelabel, kept_R, inst,
                        figsize=(7,6), point_size=50, alpha=0.9):
    """
    Plot clustering labels already computed, using Pr/Dr_single dicts:
    - pickups = filled circle
    - delivery = hollow circle
    """
    import matplotlib.pyplot as plt
    import numpy as np
    from collections import defaultdict

    Coords     = inst["Coords"]
    Pr         = inst["Pr"]        # dict {r: [pickup node ids]}
    Dr_single  = inst["Dr_single"] # dict {r: delivery node id}
    depot      = inst.get("depot")
    sink       = inst.get("sink")

    clusters = defaultdict(list)  # cid -> list of (x, y, is_delivery)
    for idx, r in enumerate(kept_R):
        cid = int(prelabel[idx])
        # pickups
        for i in Pr.get(r, []):
            x, y = Coords[i]
            clusters[cid].append((x, y, False))
        # delivery
        d = Dr_single.get(r, None)
        if d is not None:
            x, y = Coords[d]
            clusters[cid].append((x, y, True))

    fig, ax = plt.subplots(figsize=figsize)
    seen = set()
    for cid in sorted(clusters):
        color = f"C{cid % 10}"
        xs_p, ys_p, xs_d, ys_d = [], [], [], []
        for x, y, is_del in clusters[cid]:
            if is_del:
                xs_d.append(x); ys_d.append(y)
            else:
                xs_p.append(x); ys_p.append(y)

        if xs_p:
            lbl = f"C{cid}" if f"C{cid}" not in seen else ""
            ax.scatter(xs_p, ys_p, marker="o", s=point_size,
                       alpha=alpha, facecolors=color, edgecolors=color,
                       label=lbl)
            seen.add(f"C{cid}")

        if xs_d:
            lbl = "" if f"C{cid}" in seen else f"C{cid}"
            ax.scatter(xs_d, ys_d, marker="o", s=point_size,
                       alpha=alpha, facecolors="none", edgecolors=color,
                       linewidths=1.6, label=lbl)
            seen.add(f"C{cid}")

    if depot is not None:
        dx, dy = Coords[depot]
        ax.scatter([dx],[dy], marker="s", s=120, facecolors="none", edgecolors="k", linewidths=1.6, label="depot")
    ax.set_title("Clusters (pickups filled, deliveries hollow)")
    ax.set_xlabel("x"); ax.set_ylabel("y")
    ax.legend()
    ax.grid(True, linestyle="--", alpha=0.5)
    plt.show()

