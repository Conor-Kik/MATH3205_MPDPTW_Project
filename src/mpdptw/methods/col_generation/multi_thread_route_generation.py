
from math import comb
from gurobipy import *
from mpdptw.common.cli import parse_instance_argv
from mpdptw.common.parsers import build_milp_data
from mpdptw.common.route_time_lifted import Run_Time_Model
from mpdptw.common.col_gen_solution_printer import print_subset_solution
import time
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed
import psutil


# ---------------------------- Configuration Flags ---------------------------- #
VEHICLE_CAPACITY = 0  # whether to enforce capacity feasibility via re-run
COL_GEN_OUTPUT = 0    # Whether to show solver output of subproblem
PRINT_ROUTES = 1      # whether to print selected routes after optimization


# -------------------------- Capacity Check Utilities ------------------------- #
def _is_capacity_ok(arcs, q, Q, depot, sink, EPS=1e-9):
    """
    Lightweight, worker-side capacity check along a constructed route.

    Parameters
    ----------
    arcs : iterable[tuple[int, int]]
        Set of arcs (i, j) describing a single path from depot to sink.
    q : dict[int, float]
        Node demands (positive pickups / negative deliveries, depending on model).
    Q : float
        Vehicle capacity.
    depot : int
        Depot node id.
    sink : int
        Sink node id.
    EPS : float
        Numerical tolerance for capacity violation.

    Returns
    -------
    bool
        True if capacity never exceeds Q + EPS along the path, False otherwise.
    """
    succ = {i: j for (i, j) in arcs}

    # Reconstruct route (depot -> ... -> sink) by following successors.
    route = [depot]
    cur = depot
    for _ in range(len(arcs) + 2):  # +2 safety to avoid infinite loops on malformed arcs
        if cur == sink:
            break
        cur = succ.get(cur)
        if cur is None:
            break
        route.append(cur)

    # Accumulate load along the route and check against capacity.
    load = 0.0
    for v in route:
        load += q.get(v, 0.0)
        if load > Q + EPS:
            return False
    return True


# ----------------------- Worker Initialization for Gurobi -------------------- #
def _pricing_worker_init():
    """
    Initialize a private Gurobi environment per process to avoid shared-state
    issues across processes. Suppresses solver output in workers.
    """
    global _ENV
    _ENV = Env(empty=True)
    _ENV.setParam("OutputFlag", 0)
    _ENV.start()


# --------------------------- Worker Task Definition -------------------------- #
def _solve_subset(
    ids_tuple,
    mask,
    inst,
    Time_Window,
    threads,
    do_capacity_check,
    cap_params,
):
    """
    Solve the time-window pricing subproblem for a given subset in a worker.

    Returns
    -------
    tuple
        (ids_tuple, mask, status, s_cost, arcs, runtime)

        - status can be GRB statuses, "EXC" on exception, or "CAP_FAIL" if
          the re-run with capacity constraint is infeasible.
    """
    try:
        _m, s_cost, arcs, _, runtime = Run_Time_Model(
            ids_tuple,
            inst,
            False,
            Time_Window=Time_Window,
            Threads=threads,
            ENV=_ENV,
        )

        if _m.Status in (GRB.INFEASIBLE, GRB.CUTOFF):
            return (ids_tuple, mask, _m.Status, s_cost, None, runtime)

        if do_capacity_check:
            q, Q, depot, sink, EPS = cap_params
            if not _is_capacity_ok(arcs, q, Q, depot, sink, EPS):
                # Re-run with capacity constraints activated.
                _m2, s_cost2, arcs2, _, runtime2 = Run_Time_Model(
                    ids_tuple,
                    inst,
                    Output=COL_GEN_OUTPUT,
                    Time_Window=Time_Window,
                    Threads=threads,
                    ENV=_ENV,
                    Capacity_Constraint=True,
                )
                if _m2.Status in (GRB.INFEASIBLE, GRB.CUTOFF):
                    return (ids_tuple, mask, "CAP_FAIL", s_cost2, None, None)
                return (
                    ids_tuple,
                    mask,
                    _m2.Status,
                    s_cost2,
                    arcs2,
                    runtime + runtime2,
                )

        return (ids_tuple, mask, _m.Status, s_cost, arcs, runtime)

    except Exception as e:
        # Keep worker robust and return error info upstream.
        print("ERROR", e)
        return (ids_tuple, mask, "EXC", str(e), None, 0.0)


# ------------------------------- Parallel Runner ----------------------------- #
def run_k_in_parallel(
    subsets_k,
    inst,
    Time_Window,
    workers,
    pricing_threads,
    do_capacity_check,
    cap_params,
):
    """
    Submit all k-sized subsets to the process pool.

    Returns
    -------
    list
        List of worker results, each matching _solve_subset()'s return format.
    """
    results = []
    with ProcessPoolExecutor(
        max_workers=workers, initializer=_pricing_worker_init
    ) as pool:
        futures = [
            pool.submit(
                _solve_subset,
                tuple(ids),
                mask,
                inst,
                Time_Window,
                pricing_threads,
                do_capacity_check,
                cap_params,
            )
            for ids, mask in subsets_k
        ]
        for fut in as_completed(futures):
            results.append(fut.result())
    return results


# ------------------------------- Main Generation ----------------------------- #
def generate_routes(instance: str, model: Model):
    """
    Generate route columns by enumerating request subsets with pruning,
    optionally checking capacity feasibility, then solve the master problem.

    Notes
    -----
    - Uses a frontier that grows by subset size k with early pruning:
      * Infeasible masks (by time-window or capacity) are stored to avoid
        exploring supersets.
      * A quick service-time bound prunes obvious time-infeasible extensions.
    - Parallelization kicks in for large candidate sets.
    - Function prints summary stats and (optionally) chosen routes.
    """
    path = Path(instance)
    filename = path.name
    Time_Window = not filename.startswith("w")  # "w..." instances disable TW

    print("USING MULTI-THREADING FOR ROUTE GENERATION")
    print("Workers to use:", psutil.cpu_count(logical=False))

    inst = build_milp_data(str(instance), generate_W_set=False)

    start_time = time.perf_counter()


    R = inst["R"]
    Pr = inst["Pr"]
    Dr_single = inst["Dr_single"]
    d = inst["d"]
    l = inst["l"]
    V = inst["V"]
    depot = inst["depot"]
    Q = inst["Q"]
    q = inst["q"]
    sink = inst["sink"]

    EPS = 1e-9
    cap_params = (q, Q, depot, sink, EPS)

    # Local capacity checker for the sequential path (same as worker-side)
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

    # Track masks that are known infeasible to prune supersets quickly.
    infeasible_masks = set()

    def contains_infeasible_subset(mask):
        return any((mask & bad) == bad for bad in infeasible_masks)

    def add_infeasible_mask(mask):
        """
        Maintain a minimal family: keep only masks that are not supersets
        of existing ones (and remove those that are supersets of the new one).
        """
        for bad in infeasible_masks:
            if (mask & bad) == bad:  # existing ⊆ new
                return
        to_remove = [bad for bad in infeasible_masks if (bad & mask) == mask]
        for bad in to_remove:
            infeasible_masks.remove(bad)
        infeasible_masks.add(mask)

    # ------------------- Subset Streaming with Early Pruning ------------------ #
    costs = {}       # subset -> reduced cost (objective contribution)
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
    processed_total = 0
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

        if len(subsets_k) > 100:
            # Parallel path (capacity recheck happens inside the worker)
            batch = run_k_in_parallel(
                subsets_k,
                inst,
                Time_Window,
                workers=psutil.cpu_count(logical=False),
                pricing_threads=1,
                do_capacity_check=bool(VEHICLE_CAPACITY),
                cap_params=cap_params,  # (q, Q, depot, sink, EPS)
            )
            for subset_ids, mask, status, s_cost, arcs, runtime in batch:
                if status in (GRB.INFEASIBLE, GRB.CUTOFF, "EXC", "CAP_FAIL"):
                    if status == "CAP_FAIL":
                        cap_fails += 1
                    add_infeasible_mask(mask)
                    pruned += 1
                    continue

                valid_subsets.append((subset_ids, mask, runtime))
                costs_time[tuple(subset_ids)] = s_cost
                # Reduced cost: subtract service time contribution
                costs[tuple(subset_ids)] = s_cost - sum(
                    service_time_r[r] for r in subset_ids
                )
                processed += 1
        else:
            # Sequential path (mirror the worker behavior, re-run if needed)
            for subset_ids, mask in subsets_k:
                _m, s_cost, arcs, _, runtime = Run_Time_Model(
                    subset_ids, inst, Time_Window=Time_Window
                )
                if _m.Status in (GRB.INFEASIBLE, GRB.CUTOFF):
                    add_infeasible_mask(mask)
                    pruned += 1
                    continue

                if VEHICLE_CAPACITY and not is_capacity_ok(arcs):
                    # Re-run with explicit capacity constraint enforcement
                    _m, s_cost, arcs, _, runtime2 = Run_Time_Model(
                        subset_ids, inst,Output=COL_GEN_OUTPUT, Time_Window=Time_Window, Capacity_Constraint=True
                    )
                    if _m.Status in (GRB.INFEASIBLE, GRB.CUTOFF):
                        cap_fails += 1
                        add_infeasible_mask(mask)
                        pruned += 1
                        continue
                    runtime += runtime2  # accumulate runtime for reporting

                valid_subsets.append((subset_ids, mask, runtime))
                costs_time[tuple(subset_ids)] = s_cost
                costs[tuple(subset_ids)] = s_cost - sum(
                    service_time_r[r] for r in subset_ids
                )
                processed += 1

        print(
            f"Time: {time.perf_counter()-curr_time:.2f}, "
            f"Infeasible Found: {pruned}, "
            f"Valid Routes Found: {processed - start_process}\n"
        )
        curr_time = time.perf_counter()
        processed_total += processed

        # ----------------- Build frontier for k+1 by extending k ---------------- #
        # Lexicographic extension to avoid duplicates: only add indices > last
        optimal_pruning = 0
        next_frontier = []

        # heuristic: process longer-running subsets first
        valid_subsets.sort(key=lambda x: x[2], reverse=True)

        for ids, mask, _ in valid_subsets:
            start = ids[-1] + 1
            for p in range(start, n):
                new_mask = mask | (1 << p)
                if contains_infeasible_subset(new_mask):
                    continue

                # Quick feasibility bound: s_cost + service_time_r[p] <= l[depot]
                if costs_time[tuple(ids)] + service_time_r[p] > l[depot]:
                    optimal_pruning += 1
                    total_pruned += 1
                    continue

                next_frontier.append((ids + (p,), new_mask))

        frontier = next_frontier

    # ------------------------------- Summary Log ------------------------------ #
    print(
        f"\nAll columns generated. Valid Routes: {processed}. "
        f"\nInfeasible Found: {total_pruned}, "
        f"Subsets (≤ length {max_n - 1}) pruned from infeasible: "
        f"{sum(comb(n, k) for k in range(1, max_n)) - processed - total_pruned}"
    )
    if VEHICLE_CAPACITY:
        print(f"Capacity re-checks that failed: {cap_fails}")
    print("**********************************************")

    # Map each request to all subsets that contain it
    result = {i: [] for i in range(len(R))}
    for s in costs.keys():
        for elem in s:
            result[elem].append(s)

    # Master selection variables over generated subsets
    Z = {p: model.addVar(vtype=GRB.BINARY) for p in costs}

    # Minimize total reduced cost
    model.setObjective(quicksum(Z[p] * costs[p] for p in costs.keys()), GRB.MINIMIZE)

    # Each request must be covered exactly once
    OneRequest = {
        r: model.addConstr(quicksum(Z[ss] for ss in result[r]) == 1) for r in R
    }

    if not PRINT_ROUTES:
        model.setParam("OutputFlag", 0)

    model.optimize()

    end_time = time.perf_counter()

    # Print chosen routes or compact info per selected subset
    for p in costs.keys():
        if Z[p].x > 0.5:
            if PRINT_ROUTES:
                print_subset_solution(inst, p, Capacity_Constraint=VEHICLE_CAPACITY)
            else:
                print("Requests:", list(p), "Cost:", round(costs[p], 2))

    print("**********************************************")
    print(f"Total runtime {end_time - start_time:.2f}")
    print(f"Obj Value: {model.ObjVal:.2f}")
    print("**********************************************")


# ------------------------------------ CLI ----------------------------------- #
def main(argv=None):

    path, _ = parse_instance_argv(argv, default_filename="l_4_25_4.txt")
    model = Model("MPDTW")
    generate_routes(str(path), model)


if __name__ == "__main__":
    main()
