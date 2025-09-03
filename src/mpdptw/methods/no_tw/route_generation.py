from gurobipy import *
from mpdptw.common.cli import parse_instance_argv
from mpdptw.common.parsers import build_milp_data
from mpdptw.methods.no_tw.route_time import Run_Model
from mpdptw.common.no_tw_solution_printer import print_subset_solution
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
    routes = list(subsets_up_to_k(len(R), len(R) ))
    costs = {}
    total = len(routes)
    print("Number of routes to check:", total)
    checkpoints = {int(total * p / 100) for p in range(5, 101, 5)}  # 5%, 10%, … 100%
    for idx, subset in enumerate(routes, start=1):
        _m, s_cost, arcs = Run_Model(subset, inst, False, OUTPUT)

        if _m.Status not in (GRB.INFEASIBLE, GRB.CUTOFF):
            nodes = set()
            for r in subset:
                nodes.update(Pr[r])
                nodes.add(Dr_single[r])       

            service_time = sum(d.get(node_id, 0.0) for node_id in nodes)
            costs[subset] = s_cost - service_time

        if idx in checkpoints:
            pct = (idx / total) * 100
            print(f"Progress: {pct:.0f}% ({idx}/{total})")

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
    

    for p in costs.keys():
        if Z[p].x > 0.5:
            print("\nRequests:", list(p),"Cost:", round(costs[p], 2))
            print_subset_solution(inst, p)  
    print("**********************************************8")
    print(f"Total runtime {end_time-start_time:.2f}")
    print("Obj Value",round(model.ObjVal,2))
def main(argv=None):
    path, _ = parse_instance_argv(argv, default_filename="l_4_25_4.txt")
    model = Model("MPDTW")
    Generate_Routes(str(path), model)


if __name__ == "__main__":
    main()
