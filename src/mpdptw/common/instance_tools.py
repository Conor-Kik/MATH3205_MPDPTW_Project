from pathlib import Path

def parse_instance(path: str):
    p = Path(path)
    lines = [ln.strip() for ln in p.read_text().splitlines() if ln.strip()]
    if len(lines) < 3:
        raise ValueError(f"Bad instance: {path}")

    # header
    parts = lines[0].split()
    n_veh, cap = int(parts[0]), int(parts[1])

    # rows: id x y demand a b s t q
    rows = []
    for ln in lines[1:]:
        vals = ln.split()
        if len(vals) != 9:
            raise ValueError(f"Expect 9 columns; got {len(vals)} in '{ln}'")
        rid = int(vals[0])
        rows.append({
            "id": rid,
            "x": int(vals[1]),
            "y": int(vals[2]),
            "d": int(vals[3]),
            "a": int(vals[4]),
            "b": int(vals[5]),
            "s": int(vals[6]),
            "t": int(vals[7]),   # 0 pickup, 1 delivery
            "q": int(vals[8]),   # request id; depot has -1
        })
    return {"vehicles": n_veh, "capacity": cap, "rows": rows}

def write_instance(path: str, vehicles: int, capacity: int, rows):
    out = [f"{vehicles} {capacity}"]
    for r in rows:
        out.append(f"{r['id']} {r['x']} {r['y']} {r['d']} {r['a']} {r['b']} {r['s']} {r['t']} {r['q']}")
    Path(path).write_text("\n".join(out) + "\n")

def split_depot(rows):
    dep = [r for r in rows if r["q"] == -1]
    if len(dep) != 1:
        raise ValueError("Expect exactly one depot (q = -1).")
    depot = dep[0]
    rest = [r for r in rows if r is not depot]
    return depot, rest

def reindex(rows, start_id=1, req_offset=0):
    """Contiguous node ids; offset request ids except depot (q=-1)."""
    new = []
    nid = start_id
    for r in rows:
        rr = dict(r)
        rr["id"] = nid
        if rr["q"] >= 0:
            rr["q"] = rr["q"] + req_offset
        new.append(rr)
        nid += 1
    return new

def merge_instances(path_a: str, path_b: str, out_path: str,
                    vehicles: int | None = None, capacity: int | None = None):
    A = parse_instance(path_a)
    B = parse_instance(path_b)

    veh = vehicles if vehicles is not None else A["vehicles"]
    cap = capacity if capacity is not None else A["capacity"]

    dep_a, rest_a = split_depot(A["rows"])
    _dep_b, rest_b = split_depot(B["rows"])

    # Offset request ids of B to avoid collisions
    max_req_a = max((r["q"] for r in rest_a if r["q"] >= 0), default=-1)
    req_offset_b = max_req_a + 1

    ra = reindex(rest_a, start_id=1, req_offset=0)
    rb = reindex(rest_b, start_id=1 + len(ra), req_offset=req_offset_b)

    depot = dict(dep_a)
    depot["id"] = 0

    combined = [depot] + ra + rb
    write_instance(out_path, veh, cap, combined)
    return out_path
