from gurobipy import *
from mpdptw.common.cli import parse_instance_argv
from mpdptw.common.parsers import build_milp_data
from mpdptw.common.route_time import Run_Time_Model
from mpdptw.common.col_gen_solution_printer import print_subset_solution
import time
from pathlib import Path

VEHICLE_CAPACITY = 0 #1 add vehicle capacity constraints
COL_GEN_OUTPUT = 0
PRINT_ROUTES = 1


def Generate_Routes(instance: str, model: Model):
    if VEHICLE_CAPACITY:
        print("Running with Vehicle Capacity Constraints")
    
    inst = build_milp_data(str(instance))
    start_time = time.perf_counter()
    path = Path(instance)
    filename = path.name
    Time_Window = not filename.startswith("w")

    R        = inst["R"]
    Pr       = inst["Pr"]
    Dr_single= inst["Dr_single"]
    EPS      = 1e-9
    Q        = inst["Q"]              
    d        = inst["d"]
    q        = inst["q"]
    depot    = inst["depot"]
    sink     = inst["sink"]

    def is_capacity_ok(arcs):
        succ = {i: j for (i, j) in arcs}
        route = [depot]
        cur = depot
        for _ in range(len(arcs) + 2):
            if cur == sink:
                break
            cur = succ[cur]
            route.append(cur)
        load = 0.0
        for v in route:
            load += q.get(v, 0.0)
            if load > Q + EPS:
                return False
        return True


    n = len(R)

    infeasible_masks = set()

    def contains_infeasible_subset(mask):
        return any((mask & bad) == bad for bad in infeasible_masks)

    def add_infeasible_mask(mask):
        # keep only minimal infeasible masks
        for bad in infeasible_masks:
            if (mask & bad) == bad:   # existing ⊆ new
                return
        to_remove = [bad for bad in infeasible_masks if (bad & mask) == mask]
        for bad in to_remove:
            infeasible_masks.remove(bad)
        infeasible_masks.add(mask)

    # ---------- Stream subsets of fixed size k with early pruning ----------
    def gen_size_k(k):
        curr = []  # positions [0..n-1]
        def bt(start_idx, mask):
            if len(curr) == k:
                # final candidate; drop it if it contains a known-bad subset
                if contains_infeasible_subset(mask):
                    return
                yield tuple(curr), mask
                return

            for p in range(start_idx, n):
                new_mask = mask | (1 << p)
                # early prune by infeasible-subset masks only
                if contains_infeasible_subset(new_mask):
                    continue
                curr.append(p)
                yield from bt(p + 1, new_mask)
                curr.pop()

        yield from bt(0, 0)
    costs = {}
    pruned = processed = 0
    total_pruned = 0
    W_max = n  
    service_time_r = {
                r: sum(d.get(v, 0.0) for v in Pr[r]) + d.get(Dr_single[r], 0.0)
                for r in R
}
    for k in range(1, W_max + 1):
        
        routes_to_check = sum(1 for _ in gen_size_k(k))
        pruned = 0
        total_pruned += pruned
        if routes_to_check:
            print(f"[size {k}] Starting. Routes to check {routes_to_check}")
            for subset_ids, mask in gen_size_k(k):

                _m, s_cost, arcs, _ = Run_Time_Model(subset_ids, inst, False, COL_GEN_OUTPUT, Time_Window)

                if _m.Status in (GRB.INFEASIBLE, GRB.CUTOFF):
                    add_infeasible_mask(mask)
                    pruned += 1
                    continue

                if VEHICLE_CAPACITY and not is_capacity_ok(arcs):
                    _m, s_cost, arcs, _ = Run_Time_Model(subset_ids, inst, False, COL_GEN_OUTPUT, Time_Window, VEHICLE_CAPACITY)
                    if _m.Status in (GRB.INFEASIBLE, GRB.CUTOFF):
                        add_infeasible_mask(mask)
                        pruned += 1
                        continue

                # feasible: compute service time (A already handles arc infeas)
                nodes_set = set()
                for r in subset_ids:
                    nodes_set.update(Pr[r])
                    nodes_set.add(Dr_single[r])
                service_time = sum(service_time_r[r] for r in subset_ids)
                costs[subset_ids] = s_cost - service_time
                processed += 1

            print(f"[size {k}]"
                f" Total Pruned: {pruned}, Total Valid Routes Found: {processed}\n")
        else: 
            print(f"Skipping - All subsets pruned")
            break

    print(f"\nAll columns generated. Valid Routes: {processed}. "
          f"\nUnique Pruned: {pruned}, Total Pruned: {(pow(2, n) - 1) - processed}")
    print("**********************************************")

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
            if PRINT_ROUTES:
                print_subset_solution(inst, p, VEHICLE_CAPACITY)  
            else:
                print("\nRequests:", list(p),"Cost:", round(costs[p], 2))
    print("**********************************************")
    print(f"Total runtime {end_time-start_time:.2f}")
    print(f"Obj Value: {model.ObjVal:.2f}")
    print("**********************************************")

def main(argv=None):
    path, _ = parse_instance_argv(argv, default_filename="l_4_25_4.txt")
    model = Model("MPDTW")
    Generate_Routes(str(path), model)


if __name__ == "__main__":
    main()
