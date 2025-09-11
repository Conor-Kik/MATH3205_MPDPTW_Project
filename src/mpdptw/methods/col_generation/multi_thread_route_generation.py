from fileinput import filename
from math import comb
from gurobipy import *
from mpdptw.common.cli import parse_instance_argv
from mpdptw.common.parsers import build_milp_data
from mpdptw.common.route_time import Run_Time_Model
from mpdptw.common.col_gen_solution_printer import print_subset_solution
import time
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed
import psutil


COL_GEN_OUTPUT = 0
PRINT_ROUTES = 1


# per-process init: create a private Env only
def _pricing_worker_init():
    global _ENV
    _ENV = Env(empty=True)
    _ENV.setParam("OutputFlag", 0)
    _ENV.start()

def _solve_subset(ids_tuple, mask, inst,Time_Window, threads):
    try:
        _m, s_cost, arcs, _ = Run_Time_Model(
            ids_tuple, inst,
            False,
            Time_Window=Time_Window,
            Threads=threads, ENV=_ENV
        )
        return (ids_tuple, mask, _m.Status, s_cost, arcs)
    except Exception as e:
        print("ERROR")
        return (ids_tuple, mask, 'EXC', str(e), None)

def run_k_in_parallel(subsets_k, inst, Time_Window, workers, pricing_threads):
    results = []
    with ProcessPoolExecutor(max_workers=workers,
                             initializer=_pricing_worker_init) as pool:
        futures = [pool.submit(_solve_subset, tuple(ids), mask, inst, Time_Window
                               ,pricing_threads)
                   for ids, mask in subsets_k]
        for fut in as_completed(futures):
            results.append(fut.result())
    return results

def generate_routes(instance: str, model: Model):

    
    
    path = Path(instance)
    filename = path.name
    Time_Window = not filename.startswith("w")

    print("USING MULTI-THREADING FOR ROUTE GENERATION")
    print("Workers to use:", psutil.cpu_count(logical=False))
    inst = build_milp_data(str(instance))
    start_time = time.perf_counter()
    R        = inst["R"]
    Pr       = inst["Pr"]
    Dr_single= inst["Dr_single"]
          
    d        = inst["d"]



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

    costs = {}
    pruned = processed = 0
    W_max = n  
    service_time_r = {
                r: sum(d.get(v, 0.0) for v in Pr[r]) + d.get(Dr_single[r], 0.0)
                for r in R
}
    frontier = []
    for p in range(n):
        m = 1 << p
        if not contains_infeasible_subset(m):
            frontier.append(((p,), m))   # (ids_tuple, mask)
    max_n = 0 
    processed_total = 0  # if you want cumulative stats
    total_pruned = 0
    for k in range(1, W_max + 1):
        subsets_k = frontier                # already filtered by masks
        valid_subsets = []
        if not subsets_k:
            max_n = k
            print("Skipping - All subsets pruned")
            break

        print(f"[size {k}] Starting. Routes to check {len(subsets_k)}")
        start_process = processed
        total_pruned += pruned
        pruned = 0
        
        if  len(subsets_k) > 100:
            # --- parallel path ---
            batch = run_k_in_parallel(
                subsets_k, inst, Time_Window,
                workers=psutil.cpu_count(logical=False), pricing_threads=1,
            )
            for subset_ids, mask, status, s_cost, _ in batch:
                if status in (GRB.INFEASIBLE, GRB.CUTOFF, "EXC"):
                    add_infeasible_mask(mask)
                    pruned += 1
                    continue
                valid_subsets.append((subset_ids, mask))
                costs[tuple(subset_ids)] = s_cost - sum(service_time_r[r] for r in subset_ids)
                processed += 1
        else:
            # --- sequential path ---
            for subset_ids, mask in subsets_k:
                _m, s_cost, _, _ = Run_Time_Model(subset_ids, inst, False, 0, Time_Window)
                if _m.Status in (GRB.INFEASIBLE, GRB.CUTOFF):
                    add_infeasible_mask(mask)
                    pruned += 1
                    continue
                valid_subsets.append((subset_ids, mask))
                costs[tuple(subset_ids)] = s_cost - sum(service_time_r[r] for r in subset_ids)
                processed += 1
        print(f"[size {k}] Infeasible Found: {pruned}, Valid Routes Found: {processed- start_process}\n")
        processed_total += processed

        # ------- build frontier for k+1 by extending each k-subset --------
        # Extend lexicographically to avoid duplicates: only add indices > last
        next_frontier = []
        for ids, mask in valid_subsets:
            start = ids[-1] + 1
            for p in range(start, n):
                new_mask = mask | (1 << p)
                if contains_infeasible_subset(new_mask):
                    continue
                next_frontier.append((ids + (p,), new_mask))

        frontier = next_frontier

    print(f"\nAll columns generated. Valid Routes: {processed}. "
          f"\nInfeasible Found: {total_pruned}, Subsets (≤ length {max_n - 1}) pruned from infeasible: {sum(comb(n, k) for k in range(1, max_n)) - processed- total_pruned}")
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
                print_subset_solution(inst, p)  
            else:
                print("\nRequests:", list(p),"Cost:", round(costs[p], 2))
    print("**********************************************")
    print(f"Total runtime {end_time-start_time:.2f}")
    print(f"Obj Value: {model.ObjVal:.2f}")
    print("**********************************************")

def main(argv=None):
    path, _ = parse_instance_argv(argv, default_filename="l_4_25_4.txt")
    model = Model("MPDTW")
    generate_routes(str(path), model)


if __name__ == "__main__":
    main()
