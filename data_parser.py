# ==========================
# MPDTW parser
# ==========================
from pathlib import Path

# ── Sets and parameters (conceptual) ───────────────────────────────────────────
# V = P ∪ D ∪ {0} : all vertices; node 0 is the depot (source/sink).
# G = (V, A)      : directed graph on V with arc set A (here: complete without self-loops).
# P = {1,…,p}     : pickup nodes (customer vertices of type 0).
# D = {p+1,…,p+n} : delivery nodes (customer vertices of type 1); typically |D| = n and |P| ≥ n.
# N = P ∪ D       : set of customer nodes (all non-depot vertices).
# R = {r1,…,rn}   : requests to route. Each request r has:
#                   Pr ⊆ P (its pickup nodes) and Dr ⊆ D (its delivery nodes; often |Dr|=1).
#
# Data (parameters):
# K    : number of vehicles (homogeneous fleet).
# Q    : capacity of each vehicle.
# e_j  : opening time of the time window at node j ∈ V.
# l_j  : closing time of the time window at node j ∈ V.
# d_j  : service time at node j ∈ V.
# q_j  : demand at node j ∈ V (positive for pickup, negative for delivery).
# t_ij : travel time from i to j for (i, j) ∈ A.
# c_ij : travel cost from i to j for (i, j) ∈ A (here c_ij = t_ij).


# ── Line parsers ───────────────────────────────────────────────────────────────
def _parse_header(line):
    parts = line.strip().split()
    if len(parts) < 2:
        raise ValueError("Header must have two integers: '<vehicles> <capacity>'")
    return int(parts[0]), int(parts[1])

def _parse_node(line):
    # node_id, x, y, demand, tw_start, tw_end, service, node_type(0/1), request_id
    parts = line.strip().split()
    if len(parts) < 9:
        raise ValueError("Node line must have 9 fields")
    return {
        "raw_id": int(parts[0]),
        "x": int(parts[1]),
        "y": int(parts[2]),
        "demand": int(parts[3]),
        "tw_start": int(parts[4]),
        "tw_end": int(parts[5]),
        "service": int(parts[6]),
        "node_type": int(parts[7]),   # 0=pickup ∈ P, 1=delivery ∈ D
        "request_id": int(parts[8]),  # -1 only for depot line
    }


# ── Core parser: text → internal instance ─────────────────────────────────────
def parse_instance_text(text, name="instance"):
    """
    Returns a dict organized with internal indices:
      depot = 0, customers = 1..m (so V = {0} ∪ N and N = P ∪ D).
      Fields provided match the sets/parameters used in build_milp_data.
    """
    # Remove empty/comment lines
    lines = []
    for ln in text.splitlines():
        s = ln.strip()
        if s and not s.startswith("#"):
            lines.append(s)
    if not lines:
        raise ValueError("Empty instance text")

    # First line → K, Q
    vehicles, capacity = _parse_header(lines[0])
    if len(lines) < 2:
        raise ValueError("Missing depot line (second line)")

    # Second line must be the depot (request_id = -1)
    depot_node = _parse_node(lines[1])
    if depot_node["request_id"] != -1:
        raise ValueError("Second line must be the depot (request_id = -1)")

    # Remaining lines → customer nodes (compose N = P ∪ D, with P from node_type==0, D from node_type==1)
    raw_nodes = [_parse_node(x) for x in lines[2:]]

    # Internal indices: 0 = depot, 1..m = customers
    V = [0]                         # V = {0} ∪ N
    coords = {0: (depot_node["x"], depot_node["y"])}
    demand = {0: depot_node["demand"]}         # q_0
    tw_a   = {0: depot_node["tw_start"]}       # e_0
    tw_b   = {0: depot_node["tw_end"]}         # l_0
    service= {0: depot_node["service"]}        # d_0
    node_type = {0: -1}                        # depot marker
    raw_id = {0: depot_node["raw_id"]}
    depot = 0

    C, P, D = [], [], []             # N, P, D (with N = P ∪ D)
    R = {}                           # rid → {"pickups": Pr, "deliveries": Dr}

    for i, n in enumerate(raw_nodes, start=1):
        V.append(i); C.append(i)     # add to V and N
        coords[i]   = (n["x"], n["y"])
        demand[i]   = n["demand"]    # q_i (positive for pickup, negative for delivery, as given)
        tw_a[i]     = n["tw_start"]  # e_i
        tw_b[i]     = n["tw_end"]    # l_i
        service[i]  = n["service"]   # d_i
        node_type[i]= n["node_type"]
        raw_id[i]   = n["raw_id"]

        rid = n["request_id"]
        if rid < 0:
            raise ValueError("Non-depot node with negative request_id")

        # Build Pr and Dr for each request r ∈ R
        if rid not in R:
            R[rid] = {"pickups": [], "deliveries": []}

        if n["node_type"] == 0:
            P.append(i); R[rid]["pickups"].append(i)     # i ∈ P, i ∈ Pr
        elif n["node_type"] == 1:
            D.append(i); R[rid]["deliveries"].append(i)  # i ∈ D, i ∈ Dr (often singleton)
        else:
            raise ValueError("node_type must be 0 (pickup) or 1 (delivery)")

    return {
        "name": Path(name).stem,
        "vehicles": vehicles,     # K
        "capacity": capacity,     # Q
        "depot": depot,           # 0
        "V": V,                   # {0} ∪ N
        "C": C,                   # N = P ∪ D
        "P": P,
        "D": D,
        "R": R,                   # r ↦ (Pr, Dr)
        "coords": coords,         # i ↦ (x_i, y_i)
        "demand": demand,         # q_i
        "tw_a": tw_a,             # e_i
        "tw_b": tw_b,             # l_i
        "service": service,       # d_i
        "node_type": node_type,   # 0/1 or -1 for depot
        "raw_id": raw_id,         # original id from file
        "meta": None,
    }

def parse_instance_file(filename):
    """
    If filename has no path separators, reads from 'mpdtw_instances_2019/<filename>'.
    """
    if ("/" in filename) or ("\\" in filename):
        path = filename
    else:
        path = f"mpdtw_instances_2019/{filename}"
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()
    return parse_instance_text(text, name=filename)


# ── MILP builder: create (V,P,D,N,A,R,Pr,Dr) and (K,Q,e,l,d,q,t,c) ───────────
def _euclid(p, q):
    dx = p[0] - q[0]
    dy = p[1] - q[1]
    return (dx*dx + dy*dy) ** 0.5

def build_milp_data(filename, cost_equals_time=True, speed=1.0):
    """
    Returns a dictionary with:
      Sets : V, P, D, N, A, R, Pr, Dr
      Data : K, Q, e, l, d, q, t, c  (with c == t by default)
    Travel time uses Euclidean distance / speed; cost equals time unless overridden.
    """
    inst = parse_instance_file(filename)

    # Sets
    V = inst["V"]                 # V = {0} ∪ N
    P = inst["P"]                 # P ⊆ V\{0}
    D = inst["D"]                 # D ⊆ V\{0}
    N = inst["C"]                 # N = P ∪ D
    R = list(inst["R"].keys())    # R = {r1,…,rn}
    Pr = {r: list(inst["R"][r]["pickups"])    for r in R}   # Pr ⊆ P for each r
    Dr = {r: list(inst["R"][r]["deliveries"]) for r in R}   # Dr ⊆ D for each r (often |Dr|=1)
    # A: complete digraph on V without self-loops
    A = [(i, j) for i in V for j in V]

    # Data (parameters)
    K = range(int(inst["vehicles"]))  # number of vehicles
    Q = inst["capacity"]          # capacity per vehicle
    e = inst["tw_a"]              # e_j
    l = inst["tw_b"]              # l_j
    d = inst["service"]           # d_j
    q = inst["demand"]            # q_j
    coords = inst["coords"]

    # Travel time t_ij and cost c_ij (here c_ij = t_ij)
    t, c = {}, {}
    for (i, j) in A:
        dist = _euclid(coords[i], coords[j])
        tij = dist / speed
        t[(i, j)] = tij
        c[(i, j)] = tij if cost_equals_time else tij  # customize here if cost ≠ time

    return {
        # sets
        "V": V, "P": P, "D": D, "N": N, "A": A,
        "R": R, "Pr": Pr, "Dr": Dr,

        # parameters
        "K": K, "Q": Q,
        "e": e, "l": l, "d": d, "q": q,
        "t": t, "c": c,

        # convenience
        "depot": inst["depot"],
        "name": inst["name"],
        "raw_id": inst["raw_id"],
        "meta": inst["meta"],
    }

# --------- Example  ---------
if __name__ == "__main__":
    milp = build_milp_data("w_8_100_4.txt")

    print(f"Instance: {milp['name']}")
    print(f"Total requests: {len(milp['R'])}")

    e = milp["e"]
    l = milp["l"]
    d = milp["d"]
    q = milp["q"]
    Pr = milp["Pr"]
    Dr = milp["Dr"]

    for rid in milp["R"]:
        print(f"\nRequest {rid + 1}:")
        print("  Pickups:")
        for j in Pr[rid]:
            print(f"    node {j:>3} : e={e[j]}, l={l[j]}, d={d[j]}, q={q[j]}")
        print("  Deliveries:")
        for j in Dr[rid]:
            print(f"    node {j:>3} : e={e[j]}, l={l[j]}, d={d[j]}, q={q[j]}")


