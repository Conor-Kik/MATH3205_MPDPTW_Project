from gurobipy import *
from data_parser import build_milp_data
import math


Filename = "l_4_25_3.txt"
inst = build_milp_data(Filename)

M = 10000
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

model = Model("MPDTW")

X = {(i, j): model.addVar(vtype=GRB.BINARY) for i in V for j in V}
S = {j : model.addVar() for j in V}
C = {j : model.addVar() for j in V}
Rq ={(m,n) : model.addVar(vtype=GRB.BINARY) for m in R for n in R}
Vs = {(r, k) : model.addVar(vtype=GRB.BINARY) for k in range(K) for r in R}

model.setObjective(quicksum(X[i, j]*c[i, j] for i in V for j in V))

IncomingFlow = {j :
                model.addConstr(quicksum(X[i,j] for i in V if j != i) == 1)
                for j in N}
OutgoingFlow = {i :
                model.addConstr(quicksum(X[i,j] for j in V if j != i) == 1)
                for i in N}

VehicleCapMax = model.addConstr(K >= quicksum(X[0, j] for j in N))
VehicleCapMin = model.addConstr(quicksum(X[0, j] for j in N)>= 1)

EarliestService = {j:
                   model.addConstr(S[j] >= e[j]) 
                   for j in V}
LatestService = {j:
                   model.addConstr(S[j] <= l[j]) 
                   for j in V}

ServiceWindowA  = {(i, j) :
                   model.addConstr(S[j] + M*(1-X[i,j])>= S[i] + t[i, j]+ d[i])
                   for i in N for j in V if i != j}
ServiceWindowB = {j :
                  model.addConstr(S[j] + M*(1-X[0,j])>= t[0,j]+ d[0])
                  for j in N}

Precendence = {(r, d, p):
            model.addConstr(S[d] >= S[p] )
            for r in R for d in Dr[r] for p in Pr[r]}

ConnectedRequests = {(k, m, n):
                     model.addConstr(Rq[m,n] <= 1 + Vs[m,k] - Vs[n,k])
                     for k in range(K) for m in R for n in R}

TwoConnectedRequests = {(m,n, i, j):
                        model.addConstr(X[i, j] <= Rq[m, n])
                        for m in R for n in R if m != n 
                        for i in ReqNodes[m] 
                        for j in ReqNodes[n] 
                        if (i, j) in X}


VehicleServes = {r :
                 model.addConstr(quicksum(Vs[r,k] for k in range(K)) == 1)
                 for r in R}


LoadCap = {j :
           model.addConstr(C[j] <= Q)
           for j in N }


VehicleCapAtNodeA = {(i, j):
                     model.addConstr(C[j] + M*(1-X[i, j]) >= C[i] + q[j])
                     for i in N for j in V if i != j}

VehicleCapAtNodeB = {j:
                     model.addConstr(C[j] + M*(1-X[0, j]) >= q[j])
                     for j in N}


StartFinishDepot = model.addConstr(quicksum(X[0, j] for j in N) == quicksum(X[i, 0] for i in N))

model.optimize()

