from gurobipy import *
from data_parser import build_milp_data
from solution_printer import print_solution_summary

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


ReqNodes = {r: Dr[r] + Pr[r] for r in R}


M = {
    (i, j): max(0, l[i] + d[i] + t[i, j] - e[j])
    for i in N + [0]
    for j in N + [0]
    if i != j
}
W = {(i, j): min(Q, Q + q[i]) for i in N + [0] for j in N + [0] if i != j}

model = Model("MPDTW")

X = {(i, j): model.addVar(vtype=GRB.BINARY) for i in V for j in V}
S = {j: model.addVar() for j in V}
C = {j: model.addVar() for j in V}
Rq = {(m, n): model.addVar(vtype=GRB.BINARY) for m in R for n in R}
Vs = {(r, k): model.addVar(vtype=GRB.BINARY) for k in K for r in R}



IncomingFlow = {
    j: model.addConstr(quicksum(X[i, j] for i in V if j != i) == 1) for j in N
}
OutgoingFlow = {
    i: model.addConstr(quicksum(X[i, j] for j in V if j != i) == 1) for i in N
}

VehicleCapMax = model.addConstr(len(K) >= quicksum(X[0, j] for j in N))
VehicleCapMin = model.addConstr(quicksum(X[0, j] for j in N) >= 1)

EarliestService = {j: model.addConstr(S[j] >= e[j]) for j in V}
LatestService = {j: model.addConstr(S[j] <= l[j]) for j in V}


ServiceWindowAStar = {
    (i, j): model.addConstr(
        S[j]
        >= S[i]
        + d[i]
        + t[i, j]
        - M[i, j] * (1 - X[i, j])
        + (M[i, j] - d[i] - t[i, j] - max(d[i] + t[j, i], e[i] - e[j])) * X[i, j]
    )
    for i in N
    for j in N
    if i != j
}
ServiceWindowBStar = {
    j: model.addConstr(S[j] >= d[0] + t[0, j] - M[0, j] * (1 - X[0, j])) for j in N
}

ServiceWindowCStar = {
    i: model.addConstr(S[0] >= S[i] + d[i] + t[i, 0] - M[i, 0] * (1 - X[i, 0]))
    for i in N
}


Precendence = {
    (r, d, p): model.addConstr(S[d] >= S[p]) for r in R for d in Dr[r] for p in Pr[r]
}

ConnectedRequests = {
    (k, m, n): model.addConstr(Rq[m, n] <= 1 + Vs[m, k] - Vs[n, k])
    for k in K
    for m in R
    for n in R
}

TwoConnectedRequests = {
    (m, n, i, j): model.addConstr(X[i, j] <= Rq[m, n])
    for m in R
    for n in R
    if m != n
    for i in ReqNodes[m]
    for j in ReqNodes[n]
    if (i, j) in X
}


VehicleServes = {r: model.addConstr(quicksum(Vs[r, k] for k in K) == 1) for r in R}


LoadCap = {j: model.addConstr(C[j] <= Q) for j in N}


VehicleCapAtNodeA = {
    (i, j): model.addConstr(
        C[j]
        >= C[i] + q[j] - W[i, j] * (1 - X[i, j]) + (W[i, j] - q[i] - q[j]) * X[j, i]
    )
    for i in N
    for j in N
    if i != j
}

VehicleCapAtNodeB = {
    j: model.addConstr(
        C[j] >= q[j] - W[0, j] * (1 - X[0, j]) + (W[0, j] - q[0] - q[j]) * X[j, 0]
    )
    for j in N
}


StartFinishDepot = model.addConstr(
    quicksum(X[0, j] for j in N) == quicksum(X[i, 0] for i in N)
)

NoNodeToItself = model.addConstr(quicksum(X[i, i] for i in V) == 0)

NoPickupToDepot = model.addConstr(quicksum(X[i, 0] for r in R for i in Pr[r]) == 0)

NoDeliveryToPick = model.addConstr(
    quicksum(X[d, i] for r in R for i in Pr[r] for d in Dr[r]) == 0
)

NoDepotToDelivery = model.addConstr(quicksum(X[0, d] for r in R for d in Dr[r]) == 0)

OneWayConnection = {
    (i, j): model.addConstr(X[i, j] + X[j, i] <= 1) for i in V for j in V if i < j
}


OneRequestNodeToDepot = {
    r: model.addConstr(quicksum(X[0, j] for j in Pr[r]) <= 1) for r in R
}

ServiceWindowInfeasible = {
    (i, j): model.addConstr(X[i, j] == 0)
    for i in V
    for j in V
    if l[j] < e[i] + d[i] + t[i, j]
}





model.setObjective(quicksum(X[i, j] * c[i, j] for i in V for j in V))








model.optimize
if model.status == GRB.OPTIMAL:

    print_solution_summary(
model, V, N, R, K, Pr, Dr, X, S, C, e, l, q, t=t, Vs=Vs, Rq=Rq, d=d
)

else:
    print("INFEASIBLE")


