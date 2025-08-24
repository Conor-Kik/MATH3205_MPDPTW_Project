
from heapq import heappush, heappop



def tight_bigM(out_arcs, t, d, V, A, sink, e, l ):
    def dijkstra(source):
        dist = {v: float('inf') for v in V}
        dist[source] = 0.0
        pq = [(0.0, source)]
        while pq:
            du,u = heappop(pq)
            if du != dist[u]: continue
            for (_,y) in out_arcs[u]:
                v = y
                w = t[u,v] + d[u]  # finish u then travel
                nd = du + w
                if nd < dist[v]:
                    dist[v] = nd
                    heappush(pq, (nd, v))
        return dist
    fw = dijkstra(0)
    rev_out = {j: [] for j in V}
    for (i,j) in A:
        rev_out[j].append((j,i))
    bw = {}


    dist = {v: float('inf') for v in V}
    dist[sink] = 0.0
    pq = [(0.0, sink)]
    while pq:
        du,u = heappop(pq)
        if du != dist[u]: continue
        for (_,i) in rev_out[u]:
            w = t[i,u] + d[i]
            nd = du + w
            if nd < dist[i]:
                dist[i] = nd
                heappush(pq, (nd, i))
    bw = dist



    Earliest = {i: max(e[i], e[0] + max(0.0, fw[i] - d[i])) for i in V}
    Latest   = {i: min(l[i],   l[sink] - max(0.0, bw[i]))       for i in V}

    # rebuild M using tightened bounds
    M_ij = {(i,j): max(0.0, Earliest[i] + d[i] + t[i,j] - Earliest[j],
                            Latest[i]    + d[i] + t[i,j] - Earliest[j]) for (i,j) in A}
    return M_ij, Earliest, Latest