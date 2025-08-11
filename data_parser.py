# ==========================
# MPDTW parser
# ==========================

def _basename(path):
    p = path.replace("\\", "/")
    return p.split("/")[-1]

def _strip_ext(name):
    dot = name.rfind(".")
    return name[:dot] if dot != -1 else name

def _parse_meta_from_name(path_or_name):
    """
    Parse names like: l_4_25_1.txt  -> {'tw_type': 'Large', 'max_request_len': 4, 'num_customers': 25, 'index': 1}
    Returns None if it doesn't match the convention.
    """
    name = _strip_ext(_basename(path_or_name)).lower()
    parts = name.split("_")
    if len(parts) < 3:
        return None
    tw_key = parts[0]
    if tw_key not in ("l", "n", "w"):
        return None
    tw_map = {"l": "Large", "n": "Normal", "w": "Without"}
    # parse ints safely
    try:
        max_req_len = int(parts[1])
        num_customers = int(parts[2])
    except:
        return None
    idx = None
    if len(parts) >= 4:
        try:
            idx = int(parts[3])
        except:
            idx = None
    return {
        "tw_type": tw_map[tw_key],
        "max_request_len": max_req_len,
        "num_customers": num_customers,
        "index": idx,
    }

# --------- line parsers ---------
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
        "node_type": int(parts[7]),   # 0=pickup, 1=delivery
        "request_id": int(parts[8]),
    }

# --------- core parser ---------
def parse_instance_text(text, name="instance"):
    """
    Parse an MPDTW instance from a text string (no imports).
    Returns a pure-Python dict with internal node indices:
      - depot = 0, customers = 1..n
    Keys:
      vehicles, capacity, depot
      V, C, P, D, R (rid -> {pickups:[...], deliveries:[...]})
      coords, demand, tw_a, tw_b, service, node_type, raw_id
      meta (None here; set in parse_instance_file)
    """
    # strip comments/empties
    lines = []
    for ln in text.splitlines():
        s = ln.strip()
        if s and not s.startswith("#"):
            lines.append(s)

    if not lines:
        raise ValueError("Empty instance text")

    vehicles, capacity = _parse_header(lines[0])
    if len(lines) < 2:
        raise ValueError("Missing depot line (second line)")

    # depot must be the second line with request_id = -1
    depot_node = _parse_node(lines[1])
    if depot_node["request_id"] != -1:
        raise ValueError("Second line must be the depot (request_id = -1)")

    # parse ONLY customer lines (skip header + depot)
    raw_nodes = []
    for k in range(2, len(lines)):
        raw_nodes.append(_parse_node(lines[k]))

    # internal indices: 0 = depot, 1..n = customers
    V = [0]
    coords = {0: (depot_node["x"], depot_node["y"])}
    demand = {0: depot_node["demand"]}
    tw_a = {0: depot_node["tw_start"]}
    tw_b = {0: depot_node["tw_end"]}
    service = {0: depot_node["service"]}
    node_type = {0: -1}  # depot marker
    raw_id = {0: depot_node["raw_id"]}
    depot = 0

    C, P, D = [], [], []
    R = {}  # rid -> {"pickups":[...], "deliveries":[...]}

    idx = 1
    for n in raw_nodes:
        i = idx
        idx += 1
        V.append(i)
        C.append(i)
        coords[i] = (n["x"], n["y"])
        demand[i] = n["demand"]
        tw_a[i] = n["tw_start"]
        tw_b[i] = n["tw_end"]
        service[i] = n["service"]
        node_type[i] = n["node_type"]
        raw_id[i] = n["raw_id"]

        rid = n["request_id"]
        if rid < 0:
            raise ValueError("Non-depot node with negative request_id")

        if rid not in R:
            R[rid] = {"pickups": [], "deliveries": []}

        if n["node_type"] == 0:
            P.append(i)
            R[rid]["pickups"].append(i)
        elif n["node_type"] == 1:
            D.append(i)
            R[rid]["deliveries"].append(i)
        else:
            raise ValueError("node_type must be 0 (pickup) or 1 (delivery)")

    return {
        "name": _strip_ext(_basename(name)),
        "vehicles": vehicles,     # K
        "capacity": capacity,     # Q
        "depot": depot,           # 0
        "V": V,                   # {0} ∪ P ∪ D
        "C": C,                   # N = P ∪ D
        "P": P,
        "D": D,
        "R": R,                   # rid -> {"pickups":[...], "deliveries":[...]}
        "coords": coords,         # i -> (x,y)
        "demand": demand,         # q_i (signed)
        "tw_a": tw_a,             # e_i
        "tw_b": tw_b,             # l_i
        "service": service,       # d_i
        "node_type": node_type,   # 0/1 or -1 for depot
        "raw_id": raw_id,         # original id from file
        "meta": None,
    }

def parse_instance_file(filename):
    """
    Read from the fixed subdirectory: mpdtw_instances_2019/<filename>
    Example: inst = parse_instance_file("l_4_25_1.txt")
    """
    # if caller already passed a path with a slash, respect it; else prepend subdir
    if ("/" in filename) or ("\\" in filename):
        path = filename
    else:
        path = "mpdtw_instances_2019/" + filename

    with open(path, "r", encoding="utf-8") as f:
        text = f.read()
    inst = parse_instance_text(text, name=filename)
    inst["meta"] = _parse_meta_from_name(filename)
    return inst

# --------- MILP builder: sets A and matrices t,c ---------
def _euclid(p, q):
    # Euclidean distance (float) without imports
    dx = p[0] - q[0]
    dy = p[1] - q[1]
    return (dx*dx + dy*dy) ** 0.5

def build_milp_data(filename, include_self_loops=False, cost_equals_time=True, speed=1.0):
    """
    From a parsed 'inst', produce an MPDPTW data dict aligned to your math:
      - Sets: V, P, D, N, A, R, Pr, Dr
      - Params: K, Q, e, l, d, q, t, c (with c == t by default)
    Distances are Euclidean; travel time = distance / speed.
    """
    inst = parse_instance_file(filename)
    V = inst["V"]
    P = inst["P"]
    D = inst["D"]
    N = inst["C"]
    K = inst["vehicles"]
    Q = inst["capacity"]

    coords = inst["coords"]
    e = inst["tw_a"]
    l = inst["tw_b"]
    d = inst["service"]
    q = inst["demand"]

    # Requests (keep general: possibly multiple pickups/deliveries)
    R = []
    Pr = {}
    Dr = {}
    for rid in inst["R"]:
        R.append(rid)
        Pr[rid] = list(inst["R"][rid]["pickups"])
        Dr[rid] = list(inst["R"][rid]["deliveries"])

    # Arc set A: complete digraph (toggle self-loops as needed)
    A = []
    for i in V:
        for j in V:
            if include_self_loops or (i != j):
                A.append((i, j))

    # Travel time and cost
    t = {}
    c = {}
    for (i, j) in A:
        if i == j:
            tij = 0.0
        else:
            dist = _euclid(coords[i], coords[j])
            tij = dist / speed
        t[(i, j)] = tij
        c[(i, j)] = tij if cost_equals_time else tij  # easy hook to change cost later

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
    milp = build_milp_data("l_4_25_1.txt")

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


