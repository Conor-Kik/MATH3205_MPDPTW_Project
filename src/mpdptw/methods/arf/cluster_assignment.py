import itertools
from gurobipy import *
from mpdptw.common.parsers import build_milp_data




def pair_req_feasible_arc(R, Pr,Dr,  A):
    e_ij = {}
    for i in R:
        from_nodes = set(Pr[i]) | {Dr[i]}
        for j in R:
            if i == j:
                continue
            to_nodes = set(Pr[j]) | {Dr[j]}
            cnt = 0
            for u in from_nodes:
                for v in to_nodes:
                    if (u, v) in A:        
                        cnt += 1
            e_ij[(i, j)] = cnt
    return e_ij




def single_req_feasible_arcs(R, Pr, Dr, A, sink):
    e_i = {}
    for r in R:
        nodes = set(Pr[r]) | {Dr[r]}
        cnt = 0
        for p in Pr[r]:
            if (0, p) in A:
                cnt += 1

        if (Dr[r], sink) in A:
            cnt += 1

        for u in nodes:
            for v in nodes:
                if (u, v) in A:
                    cnt += 1

        e_i[r] = cnt
    return e_i


def extract_order(U, R):
    # pos: position k -> request i   
    pos = {}
    for k in R:
        firsts = [i for i in R if U[i, k].x > 0.5]
        pos[k] = firsts[0]
    # rank: request i -> position k  
    rank = {i: k for k, i in pos.items()}
    return rank, pos


def Run_Cluster_Assignment_Model(inst, model: Model, W, outputflag):
    model.setParam("MIPFocus", 1)
    model.setParam("TimeLimit", 10) 
    if outputflag:
        print(W)
        print("********************************")

    EPS = 1e-6
    # Sets (extended with sink depot)
    V = inst["V_ext"]  # all nodes including origin (0) and sink
    A = inst["A_feasible_ext"]  # feasible arcs 
    N = inst["N"]

    R = inst["R"]  # request ids
    K = R
    Pr = inst["Pr"]  # pickups per request
    Dr_single = inst["Dr_single"]  # single delivery per request
    depot = inst["depot"]  # start depot (0)
    sink = inst["sink"]  # sink depot

    e_i = single_req_feasible_arcs(R, Pr, Dr_single, A, sink)
    e_ij = pair_req_feasible_arc(R, Pr, Dr_single, A)

    U = {(r, k) : model.addVar(vtype=GRB.BINARY) for r in R for k in K}    
    V = {(r,k): model.addVar(lb=0, ub=1) for r in R for k in K}
    Z = {(i,j,k): model.addVar(lb=0, ub=1) for i in R for j in R for k in K if i<j}

    W_i = {i : [j for (ii, j) in W if i == ii]
           for i in R}

    model.setObjective(
        quicksum(e_ij[i, j]*Z[i,j,k] for k in R for j in R for i in R if i < j)
        + quicksum(e_i[j]*V[j, k] for k in K for j in R), GRB.MINIMIZE
    )

    OneFirstPositionA = { i:
                        model.addConstr(quicksum(U[i, k] for k in K) == 1)
                        for i in R}
    OneFirstPositionB = { k:
                        model.addConstr(quicksum(U[i, k] for i in R) == 1)
                        for k in K}
    
    UtoV = {(i, k):
            model.addConstr(V[i, k] >= U[i, k])
    for i in R for k in K}

    ZtoV = {(i, j, k):
            model.addConstr(Z[i, j, k] >= V[i, k] + V[j, k] - 1)
            for i in R for j in R for k in K if i < j}
    
    InfeasClusterAssignment = {(i, l, k):
                               model.addConstr(V[i,l] >= U[i, k] - quicksum(U[j, l] for j in W_i[i]))
    for i in R for l in R for k in K if l < k}


    model.params.OutputFlag = outputflag
    model.optimize()
    if outputflag:
        for k in K:
            if sum(V[i, k].x for i in R) > 0.5:
                reqs = [i for i in R if V[i, k].x > 0.5]
                print(f"Cluster {k}: {reqs}")

                # also print the U info for this cluster
                firsts = [i for i in R if U[i, k].x > 0.5]
                if firsts:
                    print(f"    First request(s) in cluster {k}: {firsts}")

        print("Infeasible pairs W:", W)
        print("Request incompatibility sets W_i:")
        for r, wr in W_i.items():
            print(f"   Request {r}: incompatible with {wr}")

        print("e_i", e_i)
        #print("e_ij", e_ij)

    return extract_order(U, R)
