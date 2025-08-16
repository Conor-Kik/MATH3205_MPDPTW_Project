from gurobipy import *
from data_parser import build_milp_data
from solution_printer import print_solution_summary

# ---------------------- load data ----------------------
Filename = "w_8_25_5.txt"
inst = build_milp_data(Filename)

V  = inst["V"]           # all nodes incl. depot 0
N  = inst["N"]           # customer nodes (no depot)
K  = inst["K"]           # vehicle indices
R  = inst["R"]           # request ids
Q  = inst["Q"]

Pr = inst["Pr"]          # dict r -> list of pickup node ids
Dr = inst["Dr"]          # dict r -> list of delivery node ids
c  = inst["c"]           # (i,j) -> cost
e  = inst["e"]           # earliest
l  = inst["l"]           # latest
t  = inst["t"]           # travel time
d  = inst["d"]           # service time
q  = inst["q"]           # demand (pickup +, delivery -)

ReqNodes = {r: Dr[r] + Pr[r] for r in R}
depot = 0

# ---------------------- big-M / W ----------------------
M = {(i, j): max(0.0, l[i] + d[i] + t[i, j] - e[j]) for i in N + [0] for j in N + [0] if i != j}
W = {(i, j): min(Q, Q + q[i]) for i in N + [0] for j in N + [0] if i != j}

# ---------------------- model + vars ----------------------
model = Model("MPDPTW")

# Arc vars (include self for compatibility with your printer; we’ll avoid using them elsewhere)
X = {(i, j): model.addVar(vtype=GRB.BINARY) for i in V for j in V}

# Tighter bounds via variable domains (instead of separate constraints)
S = {j: model.addVar(lb=e[j], ub=l[j]) for j in V}
C = {j: model.addVar(lb=0.0, ub=Q) for j in V}

Rq = {(m, n): model.addVar(vtype=GRB.BINARY) for m in R for n in R}
Vs = {(r, k): model.addVar(vtype=GRB.BINARY) for r in R for k in K}

# node -> request map
req_of = {i: r for r in R for i in (Pr[r] + Dr[r])}

# Node→vehicle membership
A = {(i, k): model.addVar(vtype=GRB.BINARY) for i in N for k in K}

# Arc→vehicle indicator (only for i != j)
B = {(i, j, k): model.addVar(vtype=GRB.BINARY) for i in V for j in V for k in K if i != j}

# ---------------------- objective ----------------------
model.setObjective(quicksum(X[i, j] * c[i, j] for i in V for j in V), GRB.MINIMIZE)

# ---------------------- core constraints (1)–(20) ----------------------
IncomingFlow = {j: model.addConstr(quicksum(X[i, j] for i in V if j != i) == 1) for j in N}
OutgoingFlow = {i: model.addConstr(quicksum(X[i, j] for j in V if j != i) == 1) for i in N}

VehicleCapMax = model.addConstr(len(K) >= quicksum(X[0, j] for j in N))
VehicleCapMin = model.addConstr(quicksum(X[0, j] for j in N) >= 1)

# time windows propagation (lifted “star” where applicable)
ServiceWindowAStar = {(i, j): model.addConstr(
    S[j] >= S[i] + d[i] + t[i, j] - M[i, j] * (1 - X[i, j])
          + (M[i, j] - d[i] - t[i, j] - max(d[i] + t[j, i], e[i] - e[j])) * X[i, j]
) for i in N for j in N if i != j}

ServiceWindowBStar = {j: model.addConstr(S[j] >= d[0] + t[0, j] - M[0, j] * (1 - X[0, j])) for j in N}
ServiceWindowCStar = {i: model.addConstr(S[0] >= S[i] + d[i] + t[i, 0] - M[i, 0] * (1 - X[i, 0])) for i in N}

# precedence (pickup before delivery) for each request
Precendence = {(r, dd, pp): model.addConstr(S[dd] >= S[pp]) for r in R for dd in Dr[r] for pp in Pr[r]}

# request connectivity / same-vehicle gating (paper’s Rq logic)
ConnectedRequests = {(k, m, n): model.addConstr(Rq[m, n] <= 1 + Vs[m, k] - Vs[n, k])
                     for k in K for m in R for n in R}
TwoConnectedRequests = {(m, n, i, j): model.addConstr(X[i, j] <= Rq[m, n])
                        for m in R for n in R if m != n
                        for i in ReqNodes[m] for j in ReqNodes[n] if (i, j) in X}

# each request served by exactly one vehicle
VehicleServes = {r: model.addConstr(quicksum(Vs[r, k] for k in K) == 1) for r in R}

# capacity recursion + bounds
LoadCap = {j: model.addConstr(C[j] <= Q) for j in N}
VehicleCapAtNodeA = {(i, j): model.addConstr(
    C[j] >= C[i] + q[j] - W[i, j] * (1 - X[i, j]) + (W[i, j] - q[i] - q[j]) * X[j, i]
) for i in N for j in N if i != j}
VehicleCapAtNodeB = {j: model.addConstr(
    C[j] >= q[j] - W[0, j] * (1 - X[0, j]) + (W[0, j] - q[0] - q[j]) * X[j, 0]
) for j in N}

# depot balance + structural forbids
StartFinishDepot = model.addConstr(quicksum(X[0, j] for j in N) == quicksum(X[i, 0] for i in N))
NoNodeToItself = model.addConstr(quicksum(X[i, i] for i in V) == 0)
NoPickupToDepot = model.addConstr(quicksum(X[i, 0] for r in R for i in Pr[r]) == 0)
NoDeliveryToPick = model.addConstr(quicksum(X[dd, i] for r in R for i in Pr[r] for dd in Dr[r]) == 0)
NoDepotToDelivery = model.addConstr(quicksum(X[0, dd] for r in R for dd in Dr[r]) == 0)
OneWayConnection = {(i, j): model.addConstr(X[i, j] + X[j, i] <= 1) for i in V for j in V if i < j}
OneRequestNodeToDepot = {r: model.addConstr(quicksum(X[0, j] for j in Pr[r]) <= 1) for r in R}
ServiceWindowInfeasible = {(i, j): model.addConstr(X[i, j] == 0)
                           for i in V for j in V if l[j] < e[i] + d[i] + t[i, j]}

# ---------------------- node/arc ↔ vehicle linking (Z-free) ----------------------
NodeAssign = {i: model.addConstr(quicksum(A[i, k] for k in K) == 1) for i in N}
BindReqNodes = {(i, k): model.addConstr(A[i, k] >= Vs[req_of[i], k]) for i in N for k in K}

ArcVehNN1 = {(i, j, k): model.addConstr(B[i, j, k] <= A[i, k]) for i in N for j in N for k in K if i != j}
ArcVehNN2 = {(i, j, k): model.addConstr(B[i, j, k] <= A[j, k]) for i in N for j in N for k in K if i != j}
StartVeh  = {(j, k):   model.addConstr(B[0, j, k] <= A[j, k]) for j in N for k in K if (0, j, k) in B}
EndVeh    = {(i, k):   model.addConstr(B[i, 0, k] <= A[i, k]) for i in N for k in K if (i, 0, k) in B}

StartLimit = {k: model.addConstr(quicksum(B[0, j, k] for j in N if (0, j, k) in B) <= 1) for k in K}
EndLimit   = {k: model.addConstr(quicksum(B[i, 0, k] for i in N if (i, 0, k) in B) <= 1) for k in K}
StartEndEq = {k: model.addConstr(
    quicksum(B[0, j, k] for j in N if (0, j, k) in B) ==
    quicksum(B[i, 0, k] for i in N if (i, 0, k) in B)
) for k in K}

NodesImpliesStart = {k: model.addConstr(
    quicksum(A[i, k] for i in N) <= len(N) * quicksum(B[0, j, k] for j in N if (0, j, k) in B)
) for k in K}
StartImpliesNodes = {k: model.addConstr(
    quicksum(B[0, j, k] for j in N if (0, j, k) in B) <= quicksum(A[i, k] for i in N)
) for k in K}

ArcEnable = {(i, j): model.addConstr(quicksum(B[i, j, k] for k in K if (i, j, k) in B) == X[i, j])
             for i in V for j in V if i != j}

# ---------------------- helpers for cuts (21)-(22) ----------------------
def find_fractional_cycles(x, Nset, depot=0, thr=0.5):
    succ = {}
    Vloc = list(Nset) + [depot]
    for ii in Nset:
        best_j, best_val = None, -1.0
        for jj in Vloc:
            if jj == ii: 
                continue
            val = x.get((ii, jj), 0.0)
            if val > best_val:
                best_j, best_val = jj, val
        if best_val > thr:
            succ[ii] = best_j
    visited, cycles = set(), []
    for start in list(succ.keys()):
        if start in visited:
            continue
        path = []
        cur = start
        while cur in succ and cur not in path:
            path.append(cur)
            visited.add(cur)
            cur = succ[cur]
        if cur in path:
            k0 = path.index(cur)
            cyc = path[k0:]
            if depot not in cyc and len(cyc) > 1:
                cycles.append(set(cyc))
    return cycles

def single_vehicle_feasible(w, inst, time_limit=5.0):
    R, Pr, Dr = inst["R"], inst["Pr"], inst["Dr"]
    Q, cc, ee, ll, tt, dd, qq = inst["Q"], inst["c"], inst["e"], inst["l"], inst["t"], inst["d"], inst["q"]
    depot = 0
    P_nodes = [p for r in w for p in Pr[r]]
    D_nodes = [u for r in w for u in Dr[r]]
    Nw = sorted(set(P_nodes + D_nodes))
    Vw = Nw + [depot]
    if not Nw:
        return True
    A = [(i, j) for i in Vw for j in Vw if i != j and not (ll[j] < ee[i] + dd[i] + tt[i, j])]
    if not A:
        return False
    Mm = {(i, j): max(0.0, ll[i] + dd[i] + tt[i, j] - ee[j]) for (i, j) in A}
    Wm = {(i, j): min(Q, Q + qq[i]) for (i, j) in A}
    m = Model("SVF"); m.Params.OutputFlag = 0; m.Params.TimeLimit = time_limit
    Xs = {(i, j): m.addVar(vtype=GRB.BINARY) for (i, j) in A}
    Ss = {v: m.addVar(lb=ee[v], ub=ll[v]) for v in Vw}
    Cs = {v: m.addVar(lb=0.0, ub=Q) for v in Vw}
    m.setObjective(quicksum(cc[i, j] * Xs[i, j] for (i, j) in A))
    for jj in Nw:
        m.addConstr(quicksum(Xs[ii, jj] for (ii, jj2) in A if jj2 == jj) == 1)
    for ii in Nw:
        m.addConstr(quicksum(Xs[ii2, jj] for (ii2, jj) in A if ii2 == ii) == 1)
    m.addConstr(quicksum(Xs[depot, j] for j in Nw if (depot, j) in A) == 1)
    m.addConstr(quicksum(Xs[i, depot] for i in Nw if (i, depot) in A) == 1)
    for (i, j) in A:
        m.addConstr(Ss[j] >= Ss[i] + dd[i] + tt[i, j] - Mm[i, j] * (1 - Xs[i, j]))
    for j in Nw:
        if (depot, j) in A:
            m.addConstr(Ss[j] >= dd[0] + tt[0, j] - Mm[depot, j] * (1 - Xs[depot, j]))
    for i in Nw:
        if (i, depot) in A:
            m.addConstr(Ss[depot] >= Ss[i] + dd[i] + tt[i, depot] - Mm[i, depot] * (1 - Xs[i, depot]))
    for (i, j) in A:
        m.addConstr(Cs[j] >= Cs[i] + qq[j] - Wm[i, j] * (1 - Xs[i, j]))
    for j in Nw:
        if (depot, j) in A:
            m.addConstr(Cs[j] >= qq[j] - Q * (1 - Xs[depot, j]))
            m.addConstr(Cs[j] <= qq[j] + Q * (1 - Xs[depot, j]))
    for i in Nw:
        if (i, depot) in A:
            m.addConstr(Cs[i] <= Q * (1 - Xs[i, depot]))
    m.optimize()
    return m.status == GRB.OPTIMAL

# ---------------------- (22) precompute infeasible pairs and add cuts ----------------------
Wsets = set()
R_list = list(R)
for idx, r1 in enumerate(R_list):
    for r2 in R_list[idx + 1:]:
        if not single_vehicle_feasible({r1, r2}, inst):
            Wsets.add(frozenset({r1, r2}))
print(f"(22) infeasible pairs found: {len(Wsets)}")

IISOR = {(tuple(sorted(w)), k): model.addConstr(quicksum(Vs[r, k] for r in w) <= len(w) - 1)
         for w in Wsets for k in K}

# ---------------------- (21) + per-vehicle subtours via lazy callback ----------------------
def bc_cb(m, where):
    if where != GRB.Callback.MIPSOL:
        return
    # aggregated X-level CSTECs (customer-only cycles)
    x = {(i, j): m.cbGetSolution(m._X[i, j]) for (i, j) in m._X}
    for s in find_fractional_cycles(x, m._N, depot=m._depot, thr=0.5):
        m.cbLazy(quicksum(m._X[i, j] for i in s for j in s) <= len(s) - 1)
    # per-vehicle cycles on B
    b = {(i, j, k): m.cbGetSolution(m._B[i, j, k]) for (i, j, k) in m._B}
    a = {(i, k): m.cbGetSolution(m._A[i, k]) for (i, k) in m._A}
    for k in m._K:
        Nk = [i for i in m._N if a.get((i, k), 0.0) > 0.5]
        if len(Nk) <= 1:
            continue
        Vk = Nk + [m._depot]
        xk = {(i, j): b.get((i, j, k), 0.0) for i in Vk for j in Vk if i != j}
        for S in find_fractional_cycles(xk, Nk, depot=m._depot, thr=0.5):
            m.cbLazy(quicksum(m._B[i, j, k] for i in S for j in S if i != j) <= len(S) - 1)

model.Params.LazyConstraints = 1
model._X = X
model._B = B
model._A = A
model._N = list(N)
model._K = list(K)
model._depot = depot

# ---------------------- solve + print ----------------------
model.optimize(bc_cb)

if model.SolCount > 0:
    print_solution_summary(model, V, N, R, K, Pr, Dr, X, S, C, e, l, q, t=t, Vs=Vs, Rq=Rq, d=d)
else:
    print("No feasible incumbent found.")
