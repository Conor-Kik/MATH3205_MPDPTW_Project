from gurobipy import *
from mpdptw.common.cli import parse_instance_argv
from mpdptw.common.parsers import build_milp_data
from mpdptw.common.route_time_lifted import Run_Time_Model
from mpdptw.common.printers.col_gen_solution_printer import print_subset_solution
import time
from pathlib import Path
from math import comb

# ---------------------------- Configuration Flags ---------------------------- #
VEHICLE_CAPACITY = 0  # set to 1 to add vehicle capacity constraints (re-run check)
COL_GEN_OUTPUT = 0    # Whether to show solver output of subproblem
PRINT_ROUTES = 1      # set to 1 to print full routes instead of compact output


# --------------------------------- Main Logic -------------------------------- #
def generate_routes(instance: str, model: Model):
    """
    Enumerate request subsets with early pruning, price each subset with
    Run_Time_Model, optionally re-check capacity feasibility, then solve the
    master selection problem over generated columns.

    Notes
    -----
    - The subset enumeration uses bitmask-based infeasibility pruning:
      once a subset is infeasible, all supersets are skipped.
    - Capacity feasibility (if VEHICLE_CAPACITY == 1) is verified by a quick
      route traversal; if violated, the subset is re-solved with explicit
      capacity constraints via Run_Time_Model(..., VEHICLE_CAPACITY=1).
    - Function prints progress per subset size and final solution stats.
    """
    if VEHICLE_CAPACITY:
        print("Running with Vehicle Capacity Constraints")

    # Parse instance and basic settings
    inst = build_milp_data(str(instance), generate_W_set=False)
    start_time = time.perf_counter()
    path = Path(instance)
    filename = path.name
    Time_Window = not filename.startswith("w")  # "w..." instances disable TW


    R = inst["R"]
    Pr = inst["Pr"]
    Dr_single = inst["Dr_single"]
    EPS = 1e-9
    Q = inst["Q"]
    d = inst["d"]
    q = inst["q"]
    depot = inst["depot"]
    sink = inst["sink"]
    l    = inst["l"]

    # ------------------------ Lightweight capacity check --------------------- #
    def is_capacity_ok(arcs):
        """
        Follow successor pointers from depot to sink, accumulating node loads.
        Returns False upon exceeding capacity Q + EPS at any point.
        """
        succ = {i: j for (i, j) in arcs}

        # Reconstruct route (depot -> ... -> sink) using successor map
        route = [depot]
        cur = depot
        for _ in range(len(arcs) + 2):  # +2 safety margin
            if cur == sink:
                break
            cur = succ[cur]
            route.append(cur)

        # Accumulate load and compare to capacity
        load = 0.0
        for v in route:
            load += q.get(v, 0.0)
            if load > Q + EPS:
                return False
        return True

    n = len(R)

    # ----------------- Infeasible-subset masks for early pruning -------------- #
    infeasible_masks = set()

    def contains_infeasible_subset(mask):
        """Return True if 'mask' contains any known infeasible submask."""
        return any((mask & bad) == bad for bad in infeasible_masks)

    def add_infeasible_mask(mask):
        """
        Maintain a minimal family of infeasible masks:
        - If an existing mask is subset of 'mask', we keep the existing one
          (and skip adding 'mask').
        - Remove any existing masks that are supersets of the new one, then add.
        """
        for bad in infeasible_masks:
            if (mask & bad) == bad:  # existing ⊆ new -> no need to add
                return
        to_remove = [bad for bad in infeasible_masks if (bad & mask) == mask]
        for bad in to_remove:
            infeasible_masks.remove(bad)
        infeasible_masks.add(mask)


    # ------------------- Subset Streaming with Early Pruning (sequential) ------------------ #

    costs = {}       # subset -> cost (objective contribution)
    cap_fails = 0
    curr_time = time.perf_counter()
    pruned = processed = optimal_pruning = 0
    W_max = n  # upper bound on subset size

    # Precompute service time of each request's required nodes
    service_time_r = {
        r: sum(d.get(v, 0.0) for v in Pr[r]) + d.get(Dr_single[r], 0.0) for r in R
    }

    # Initialize frontier with singletons (as tuples) that aren't immediately pruned
    frontier = []
    for p in range(n):
        m = 1 << p
        if not contains_infeasible_subset(m):
            frontier.append(((p,), m))  # (ids_tuple, mask)

    max_n = 0
    total_pruned = 0

    for k in range(1, W_max + 1):
        costs_time = {}        # raw s_cost per subset (for pruning bound)
        subsets_k = frontier   # already filtered by masks
        valid_subsets = []

        if not subsets_k:
            max_n = k
            print("Skipping - All subsets pruned")
            break

        print(
            f"[size {k}] Starting.\n"
            f"Routes to check {len(subsets_k)}"
            f"{f' - Infeasible Routes pre-pruned: {optimal_pruning}' if optimal_pruning else ''}"
        )

        start_process = processed
        total_pruned += pruned
        pruned = 0

        # ------------------------- Sequential processing path ------------------------- #
        for subset_ids, mask in subsets_k:
            # First pass (no explicit capacity constraint)
            _m, s_cost, arcs, _, runtime = Run_Time_Model(
                subset_ids, inst, Time_Window=Time_Window
            )
            if _m.Status in (GRB.INFEASIBLE, GRB.CUTOFF):
                add_infeasible_mask(mask)
                pruned += 1
                continue

            # Optional capacity re-check; if violated, re-run with capacity constraint
            if VEHICLE_CAPACITY and not is_capacity_ok(arcs):
                _m, s_cost, arcs, _, runtime2 = Run_Time_Model(
                    subset_ids,
                    inst,
                    Output=COL_GEN_OUTPUT,
                    Time_Window=Time_Window,
                    Capacity_Constraint=True,
                )
                if _m.Status in (GRB.INFEASIBLE, GRB.CUTOFF):
                    cap_fails += 1
                    add_infeasible_mask(mask)
                    pruned += 1
                    continue
                runtime += runtime2  # accumulate runtime for reporting

            # Keep for frontier extension and cost computation
            valid_subsets.append((subset_ids, mask, runtime))
            costs_time[tuple(subset_ids)] = s_cost
            costs[tuple(subset_ids)] = s_cost - sum(service_time_r[r] for r in subset_ids)
            processed += 1

        print(
            f"Time: {time.perf_counter()-curr_time:.2f}, "
            f"Infeasible Found: {pruned}, "
            f"Valid Routes Found: {processed - start_process}\n"
        )
        curr_time = time.perf_counter()

        # ----------------- Build frontier for k+1 by extending k ---------------- #
        # Lexicographic extension to avoid duplicates: only add indices > last
        optimal_pruning = 0
        next_frontier = []


        for ids, mask, _ in valid_subsets:
            start = ids[-1] + 1
            for p in range(start, n):
                new_mask = mask | (1 << p)
                #Superset pruning - Lemma 1
                if contains_infeasible_subset(new_mask):
                    continue
                #Lemma 2 - Quick feasibility bound: s_cost + service_time_r[p] <= l[depot] 
                if costs_time[tuple(ids)] + service_time_r[p] > l[depot]:
                    optimal_pruning += 1
                    total_pruned += 1
                    continue
                next_frontier.append((ids + (p,), new_mask))

        frontier = next_frontier

    if max_n == 0:
        max_n = W_max + 1

    print(
        f"\nAll columns generated. Valid Routes: {processed}. "
        f"\nInfeasible Found: {total_pruned}, "
        f"Subsets (≤ length {max_n - 1}) pruned from infeasible: "
        f"{sum(comb(n, k) for k in range(1, max_n)) - processed - total_pruned}"
    )
    if VEHICLE_CAPACITY:
        print(f"Capacity re-checks that failed: {cap_fails}")
    print("**********************************************")


    # ------------------------- Master Problem Assembly ------------------------ #
    # Map each request to all generated subsets that include it
    result = {i: [] for i in range(len(R))}
    for s in costs.keys():
        for elem in s:
            result[elem].append(s)

    # Binary selection variables per generated subset
    Z = {p: model.addVar(vtype=GRB.BINARY) for p in costs}

    # Objective: minimize total cost
    model.setObjective(
        quicksum(Z[p] * costs[p] for p in costs.keys()), GRB.MINIMIZE
    )

    # Cover each request exactly once
    for r in R:
        model.addConstr(quicksum(Z[ss] for ss in result[r]) == 1)

    # Optimize master problem
    model.optimize()
    end_time = time.perf_counter()

    # Report chosen subsets
    for p in costs.keys():
        if Z[p].x > 0.5:
            if PRINT_ROUTES:
                print_subset_solution(inst, p, VEHICLE_CAPACITY)
            else:
                print("\nRequests:", list(p), "Cost:", round(costs[p], 2))

    print("**********************************************")
    print(f"Total runtime {end_time - start_time:.2f}")
    print(f"Obj Value: {model.ObjVal:.2f}")
    print("**********************************************")


def main(argv=None):
    path, _ = parse_instance_argv(argv, default_filename="l_4_25_4.txt")
    model = Model("MPDTW")
    generate_routes(str(path), model)


if __name__ == "__main__":
    main()
