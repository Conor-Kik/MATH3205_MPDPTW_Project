from gurobipy import *

def Run_Model(subset, inst):
    model = Model("BSP")
    model.setParam("OutputFlag", 0)
    model.setParam("MIPFocus", 1)
    model.setParam("TimeLimit", len(subset) * 2)

    # Instance data
    A_all   = inst["A_feasible_ext"]
    Pr      = inst["Pr"]
    Dr_single = inst["Dr_single"]
    e, l    = inst["e"], inst["l"]
    d, q    = inst["d"], inst["q"]
    t, c    = inst["t_ext"], inst["c_ext"]
    depot   = inst["depot"]
    sink    = inst["sink"]

    # Requests in this subproblem
    R       = subset
    Pickups = [i for r in R for i in Pr[r]]
    Dels    = [Dr_single[r] for r in R]
    N       = set(Pickups) | set(Dels)
    V       = {depot, sink} | N

    # Induced subgraph
    A_sub = [(i, j) for (i, j) in A_all if i in V and j in V and (i, j) in c and (i, j) in t]

    # Adjacency
    in_arcs  = {v: [] for v in V}
    out_arcs = {v: [] for v in V}
    for (i, j) in A_sub:
        out_arcs[i].append((i, j))
        in_arcs[j].append((i, j))

    dead_in  = [j for j in N if len(in_arcs[j])  == 0]
    dead_out = [i for i in N if len(out_arcs[i]) == 0]
    if dead_in or dead_out:
        raise ValueError(f"Subset yields isolated nodes — no-in={dead_in}, no-out={dead_out}")

    # Variables
    X = {(i, j): model.addVar(vtype=GRB.BINARY) for (i, j) in A_sub}

    # Objective: travel + service-at-origin
    model.setObjective(quicksum(X[i, j] * (c[i, j] + d.get(i, 0.0)) for (i, j) in A_sub), GRB.MINIMIZE)

    # Anti 2-cycle (added once per unordered pair)
    Anti2Cycle = {
        (i, j): model.addConstr(X[i, j] + X[j, i] <= 1)
        for (i, j) in A_sub if (j, i) in A_sub and i < j
    }

    # Degree = 1 on customer nodes
    DegIn =  {
        j: model.addConstr(quicksum(X[i, j] for (i, _) in in_arcs[j]) == 1)
        for j in N
    }
    DegOut = {
        i: model.addConstr(quicksum(X[i, j] for (_, j) in out_arcs[i]) == 1)
        for i in N
    }

    # Depot/sink flow
    model.addConstr(quicksum(X[i, sink] for (i, _) in in_arcs[sink]) ==
                                quicksum(X[depot, j] for (_, j) in out_arcs[depot]))
    
    model.addConstr(quicksum(X[depot, j] for (_, j) in out_arcs[depot]) == 1)
    model.addConstr(quicksum(X[j, sink]  for (j, _) in in_arcs[sink])  == 1)

    # MTZ order variables and propagation
    U_MAX = len(N) + 1
    u = {v: model.addVar(vtype=GRB.CONTINUOUS, lb=0.0, ub=U_MAX) for v in V}
    FixOrderEnds = {
        depot: model.addConstr(u[depot] == 0.0),
        sink:  model.addConstr(u[sink]  == U_MAX),
    }
    OrderProp = {
        (i, j): model.addConstr(u[j] >= u[i] + 1 - U_MAX * (1 - X[i, j]))
        for (i, j) in A_sub if i != sink and j != depot and i != j
    }

    # Pickup-before-delivery precedence
    Precedence = {
        (r, p): model.addConstr(u[Dr_single[r]] >= u[p] + 1)
        for r in R for p in Pr[r]
    }

    # Defensive: forbid incoming to depot / outgoing from sink (only if arcs exist)
    DepotGuards = {}
    if in_arcs.get(depot, []):
        DepotGuards["no_in_depot"] = model.addConstr(quicksum(X[i, depot] for (i, _) in in_arcs[depot]) == 0)
    if out_arcs.get(sink, []):
        DepotGuards["no_out_sink"] = model.addConstr(quicksum(X[sink, j] for (_, j) in out_arcs[sink]) == 0)

    # Early-stop on global lower bound vs. route limit
    model.setParam("BestBdStop", float(l[sink]) + 1e-9)

    model.optimize()

    # Best bound guard for caller logic
    try:
        best_bound = float(model.ObjBound)
    except Exception:
        best_bound = float("inf")

    if best_bound > float(l[sink]):
        return model, float(l[sink]) + 1.0, []

    if model.Status == GRB.OPTIMAL:
        used_arcs = [(i, j) for (i, j) in X if X[i, j].X > 0.5]
        return model, model.ObjVal, used_arcs
    else:
        return model, None, []
