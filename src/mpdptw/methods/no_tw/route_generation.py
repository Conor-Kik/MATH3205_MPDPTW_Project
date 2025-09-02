from gurobipy import *
from mpdptw.common.cli import parse_instance_argv
from mpdptw.common.arf_solution_printer import print_solution_summary
from mpdptw.common.parsers import build_milp_data
from mpdptw.methods.arf.cluster_assignment import Run_Cluster_Assignment_Model
from mpdptw.methods.no_tw.warm_start import warm_start_solution
from mpdptw.methods.no_tw.route_time import Run_Model
from collections import defaultdict
OUTPUT_REQ_MODEL = 0 # 1 Shows for request pair infeasibility model output
OUTPUT_CLUSTER_MODEL = 0 # 1 Shows full cluster_model output
PREPROCESSING_CUTOFF = 10 #Amount of time that cluster assignment model termininates (seconds)
PLOT_CLUSTERS = 1#1 Show plot of warm_start prediction
import time
from itertools import combinations
OUTPUT = 0
def subsets_up_to_k(R, K):
    for r in range(1, min(R, K) + 1):
        for comb in combinations(range(R), r): 
            yield frozenset(comb)


def Generate_Routes(path, model : Model):
    inst = build_milp_data(str(path))
    V        = inst["V_ext"]
    A        = inst["A_feasible_ext"]
    N        = inst["N"]
    R        = inst["R"]
    Pr       = inst["Pr"]
    Dr       = inst["Dr"]
    Dr_single= inst["Dr_single"]


    K        = inst["K"]
    Q        = inst["Q"]
    nodes_to_reqs = inst["Nodes_To_Reqs"]
    e        = inst["e"]
    l        = inst["l"]
    d        = inst["d"]
    q        = inst["q"]
    t        = inst["t_ext"]
    c        = inst["c_ext"]
    V_ext    = inst["V_ext"]

    depot    = inst["depot"]
    sink     = inst["sink"]
    start_time = time.perf_counter()
    routes = list(subsets_up_to_k(len(R), len(R)))
    costs = {}
    print("Number of Route to Check", len(routes))
    for subset in routes:
        _m, s_cost, arcs = Run_Model(subset, inst, False, OUTPUT)
        if _m.Status in (GRB.USER_OBJ_LIMIT, GRB.INFEASIBLE):
            continue

        nodes = set()
        for r in subset:
            nodes.update(Pr[r])            
            nodes.add(Dr_single[r])       

        service_time = sum(d.get(node_id, 0.0) for node_id in nodes)
        costs[subset] = s_cost - service_time
    
    result = {i: [] for i in range(len(R))}
    for s in costs.keys():
        for elem in s:
            result[elem].append(s)


    Z = {p : model.addVar(vtype=GRB.BINARY) for p in costs}

    model.setObjective(quicksum(Z[p] *costs[p] for p in costs.keys()), GRB.MINIMIZE)

    for r in R:
        model.addConstr(quicksum(Z[ss] for ss in result[r]) == 1)

    model.optimize()
    end_time = time.perf_counter()
    print(round(model.ObjVal,2))
    print(f"Total time {end_time-start_time:.2f}")
    for p in costs.keys():
        if Z[p].x > 0.5:
            print(p, round(costs[p], 2))

def main(argv=None):
    path, _ = parse_instance_argv(argv, default_filename="l_4_25_4.txt")
    model = Model("MPDTW")
    Generate_Routes(str(path), model)


if __name__ == "__main__":
    main()
