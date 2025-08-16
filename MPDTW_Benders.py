from gurobipy import *
from data_parser import build_milp_data
import math

Filename = "l_4_25_1.txt"
inst = build_milp_data(Filename)


V = inst["V"]
P = inst["P"]
N = inst["N"]
K = inst["K"]
R = inst["R"] 
Q = inst["Q"]

Dr = inst["Dr"]
Pr = inst["Pr"]
c = inst["c"]
e = inst["e"]
l = inst["l"]
t = inst["t"]
d = inst["d"]
q = inst["q"]


# normalize Dr to an int (paper uses one delivery per request)
Dr = {r: (Dr[r][0] if isinstance(Dr[r], (list, tuple)) else Dr[r]) for r in R}

# handy: nodes per request
ReqNodes = {r: Pr[r] + [Dr[r]] for r in R}
M = 100000#{(i, j) : max(0, l[i] + d[i] + t[i,j] - e[j]) for i in N + [0] for j in N + [0] if i != j }

# model
model = Model("Base_MPD")

# x[i,j,k] only for i != j
X = {(i,j,k): model.addVar(vtype=GRB.BINARY, name=f"x_{i}_{j}_{k}")
     for i in V for j in V if i != j for k in K}

# y[j,k] = 1 if vehicle k visits node j
Y = {(j,k): model.addVar(vtype=GRB.BINARY, name=f"y_{j}_{k}") for j in V for k in K}

# v[r,k] = 1 if vehicle k serves request r (all its nodes)
Vreq = {(r,k): model.addVar(vtype=GRB.BINARY, name=f"v_{r}_{k}") for r in R for k in K}

# z[k] = 1 if vehicle k is used (starts/leaves depot)
Z = {k: model.addVar(vtype=GRB.BINARY, name=f"z_{k}") for k in K}

# load after visiting j on vehicle k (capacity flow)
C = {(j,k): model.addVar(lb=0.0, ub=Q, name=f"C_{j}_{k}") for j in V for k in K}


S = {j : model.addVar() for j in V}




# -------------------
# Degree constraints (across vehicles)
# each customer node visited exactly once
for j in N:
    model.addConstr(quicksum(X[i,j,k] for k in K for i in V if i != j) == 1, name=f"in_{j}")
    model.addConstr(quicksum(X[j,h,k] for k in K for h in V if h != j) == 1, name=f"out_{j}")

# per-vehicle flow equals visit
for k in K:
    for j in V:
        model.addConstr(quicksum(X[i,j,k] for i in V if i != j) == Y[j,k], name=f"yin_{j}_{k}")
        model.addConstr(quicksum(X[j,h,k] for h in V if h != j) == Y[j,k], name=f"yout_{j}_{k}")

# depot balance and vehicle-use
for k in K:
    model.addConstr(quicksum(X[0,j,k] for j in V if j != 0) == Z[k], name=f"start_{k}")
    model.addConstr(quicksum(X[i,0,k] for i in V if i != 0) == Z[k], name=f"end_{k}")

# forbid self loops (we didn’t create them, but in case your generator does)
# for k in K:
#     for i in V:
#         if (i,i,k) in X: m.addConstr(X[i,i,k] == 0)

# -------------------
# Request→vehicle coupling (keep all nodes of request r on exactly one vehicle)
for r in R:
    # exactly one vehicle serves r
    model.addConstr(quicksum(Vreq[r,k] for k in K) == 1, name=f"assign_req_{r}")
    # if k doesn’t serve r, it can’t visit its nodes; if it does, it must visit all r's nodes
    nodes_r = ReqNodes[r]
    for k in K:
        # visit implies assignment
        for j in nodes_r:
            model.addConstr(Y[j,k] <= Vreq[r,k], name=f"visit_implies_assign_{r}_{j}_{k}")
        # all-or-nothing: sum of visits to r's nodes on k equals |nodes_r| * Vreq[r,k]
        model.addConstr(quicksum(Y[j,k] for j in nodes_r) == len(nodes_r) * Vreq[r,k],
                    name=f"all_or_nothing_{r}_{k}")


# If vehicle k visits any node, it must be marked used (and thus leave/return depot)
for k in K:
    for j in N:
        model.addConstr(Y[j,k] <= Z[k], name=f"use_link_{j}_{k}")

# Capacity variables only active when the node is visited by k
for k in K:
    for j in V:
        model.addConstr(C[j,k] <= Q * Y[j,k], name=f"cap_active_ub_{j}_{k}")

# -------------------
# Capacity flow (per vehicle). Standard load propagation along arcs.
Mbig = Q  # sufficient for capacity
for k in K:
    model.addConstr(C[0,k] == 0, name=f"load_depot_{k}")  # start empty (adjust if needed)
    for i in V:
        for j in V:
            if i == j: continue
            if (i,j,k) not in X: continue
            # if arc (i->j) on k is used, carry load forward with q[j]
            model.addConstr(C[j,k] >= C[i,k] + q[j] - Mbig*(1 - X[i,j,k]),
                        name=f"cap_flow_{i}_{j}_{k}")
    # stay within capacity on visited nodes
    for j in V:
        model.addConstr(C[j,k] <= Q, name=f"cap_ub_{j}_{k}")

# (Optional) Disallow incoming arcs to depot except as route end, and disallow leaving depot directly to depot
# Already handled by degree/visit, but you can prune infeasible arcs at var-creation time if you wish.

# Objective: minimize travel cost
model.setObjective(quicksum(c[i,j] * X[i,j,k] for (i,j,k) in X), GRB.MINIMIZE)

# --- Totals at depot: departures == arrivals (across all vehicles) ---
model.addConstr(
    quicksum(X[0, j, k] for k in K for j in N if (0, j, k) in X)
    ==
    quicksum(X[i, 0, k] for k in K for i in N if (i, 0, k) in X)
)

# --- No node to itself ---
for i in V:
    for k in K:
        if (i, i, k) in X:
            model.addConstr(X[i, i, k] == 0)

# --- No pickup -> depot (only if mid-route depot visits are possible) ---
for r in R:
    for p in Pr[r]:
        for k in K:
            if (p, 0, k) in X:
                model.addConstr(X[p, 0, k] == 0)

# --- No depot -> delivery ---
for r in R:
        for k in K:
            if (0, Dr[r], k) in X:
                model.addConstr(X[0, Dr[r], k] == 0)

# --- Per-vehicle 2-cycle elimination ---
for i in V:
    for j in V:
        if i < j:
            for k in K:
                lhs = 0
                if (i, j, k) in X:
                    lhs += X[i, j, k]
                if (j, i, k) in X:
                    lhs += X[j, i, k]
                model.addConstr(lhs <= 1)








model.optimize()
def print_solution_with_node_types(model, X, Y, Vreq, Z, C, V, N, K, R, Q, c, Pr, Dr):
    """
    Pretty printer:
      - Labels nodes: depot, pickup (with request id and pickup index), delivery (with request id)
      - Prints per-vehicle route, leg-by-leg costs, and load progression
    """

    if model.SolCount == 0:
        print("No solution.")
        return

    # ---- Build reverse map: node -> (type, request, pickup_index, total_pickups_for_request)
    node_info = {0: ("depot", None, None, None)}
    for r, pickups in Pr.items():
        for idx, j in enumerate(pickups, start=1):
            node_info[j] = ("pickup", r, idx, len(pickups))
    for r, dj in Dr.items():
        # Dr may be list or int; normalize
        dj = dj[0] if isinstance(dj, (list, tuple)) else dj
        node_info[dj] = ("delivery", r, None, None)

    def label(j):
        t, r, idx, tot = node_info.get(j, ("unknown", None, None, None))
        if t == "depot":    return f"{j}[depot]"
        if t == "pickup":   return f"{j}[pickup r={r} p{idx}/{tot}]"
        if t == "delivery": return f"{j}[delivery r={r}]"
        return f"{j}[unknown]"

    # ---- Objective and used vehicles
    total_cost = sum(c[i, j] * X[i, j, k].X for (i, j, k) in X if X[i, j, k].X > 0.5)
    used = [k for k in K if Z[k].X > 0.5]

    print("\n=== Solution Summary ===")
    print(f"Objective (total distance/cost): {total_cost:.3f}")
    print(f"Vehicles used: {len(used)} / {len(K)}  -> {used}")

    if not used:
        return

    # ---- For each used vehicle, reconstruct its route and print details
    for k in used:
        # successor map for k
        succ = {}
        for (i, j, kk), var in X.items():
            if kk == k and var.X > 0.5:
                succ[i] = j

        # follow route from depot
        route = [0]
        cur = succ.get(0, None)
        seen = set(route)
        while cur is not None and cur not in seen:
            route.append(cur)
            seen.add(cur)
            cur = succ.get(cur, None)

        print(f"\nVehicle {k}:")
        print("  Route: " + " -> ".join(label(n) for n in route))

        # leg-by-leg details
        leg_costs = 0.0
        for u, v in zip(route, route[1:]):
            arc_cost = c.get((u, v), 0.0)
            leg_costs += arc_cost
            load_at_v = C[v, k].X if (v, k) in C else float("nan")
            print(f"    {label(u)}  -->  {label(v)}   cost={arc_cost:.2f}   load@{v}={load_at_v:.1f}")

        # requests served by k
        served_r = [r for r in R if Vreq[r, k].X > 0.5]
        print(f"  Requests served: {served_r}")

        # load summary
        loads = [C[n, k].X for n in route if (n, k) in C]
        if loads:
            print(f"  Max load: {max(loads):.1f} (Q={Q})  |  End load at depot: {C[0,k].X:.1f}")


# ---- Call it once after model.optimize()
print_solution_with_node_types(
    model, X, Y, Vreq, Z, C, V, N, K, R, Q, c, Pr, Dr
)
