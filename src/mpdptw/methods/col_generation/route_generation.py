from gurobipy import *
from mpdptw.common.cli import parse_instance_argv
from mpdptw.common.parsers import build_milp_data
from mpdptw.common.route_time_lifted import Run_Time_Model
from mpdptw.common.col_gen_solution_printer import print_subset_solution
import time
from pathlib import Path


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

    # -------------------- Subset generator of fixed size k -------------------- #
    def gen_size_k(k):
        """
        Yield (subset_ids_tuple, bitmask) for all k-sized subsets, skipping
        those that contain a known infeasible submask.
        """
        curr = []  # positions [0..n-1]

        def bt(start_idx, mask):
            if len(curr) == k:
                # final candidate; drop if it contains a known-bad subset
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

    # -------------------- Pricing / Column Generation Loop -------------------- #
    costs = {}           # subset_ids(tuple) -> reduced cost
    pruned = processed = 0
    total_pruned = 0
    W_max = n

    # Precompute service time per request (sum of node service times)
    service_time_r = {
        r: sum(d.get(v, 0.0) for v in Pr[r]) + d.get(Dr_single[r], 0.0) for r in R
    }

    for k in range(1, W_max + 1):
        routes_to_check = sum(1 for _ in gen_size_k(k))  # count upfront
        pruned = 0
        total_pruned += pruned

        if routes_to_check:
            print(f"[size {k}] Starting. Routes to check {routes_to_check}")

            for subset_ids, mask in gen_size_k(k):
                # Solve pricing subproblem for this subset (keep call signature)
                _m, s_cost, arcs, _, _ = Run_Time_Model(
                    subset_ids, inst, False, COL_GEN_OUTPUT, Time_Window
                )

                # If infeasible, record mask for pruning and skip
                if _m.Status in (GRB.INFEASIBLE, GRB.CUTOFF):
                    add_infeasible_mask(mask)
                    pruned += 1
                    continue

                # Optional capacity re-check; if violated, re-run with capacity
                if VEHICLE_CAPACITY and not is_capacity_ok(arcs):
                    _m, s_cost, arcs, _, _ = Run_Time_Model(
                        subset_ids,
                        inst,
                        False,
                        COL_GEN_OUTPUT,
                        Time_Window,
                        VEHICLE_CAPACITY,
                    )
                    if _m.Status in (GRB.INFEASIBLE, GRB.CUTOFF):
                        add_infeasible_mask(mask)
                        pruned += 1
                        continue

                # Compute reduced cost: subtract service-time contribution
                # (A already handles arc infeasibilities)
                nodes_set = set()
                for r in subset_ids:
                    nodes_set.update(Pr[r])
                    nodes_set.add(Dr_single[r])
                service_time = sum(service_time_r[r] for r in subset_ids)
                costs[subset_ids] = s_cost - service_time
                processed += 1

            print(
                f"[size {k}]"
                f" Total Pruned: {pruned}, Total Valid Routes Found: {processed}\n"
            )
        else:
            print("Skipping - All subsets pruned")
            break

    print(
        f"\nAll columns generated. Valid Routes: {processed}. "
        f"\nUnique Pruned: {pruned}, Total Pruned: {(pow(2, n) - 1) - processed}"
    )
    print("**********************************************")

    # ------------------------- Master Problem Assembly ------------------------ #
    # Map each request to all generated subsets that include it
    result = {i: [] for i in range(len(R))}
    for s in costs.keys():
        for elem in s:
            result[elem].append(s)

    # Binary selection variables per generated subset
    Z = {p: model.addVar(vtype=GRB.BINARY) for p in costs}

    # Objective: minimize total reduced cost
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
