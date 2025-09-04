from gurobipy import *
from mpdptw.common.cli import parse_instance_argv
from mpdptw.common.parsers import build_milp_data
from mpdptw.methods.col_generation.route_time import Run_Model
from mpdptw.common.col_gen_solution_printer import print_subset_solution
import time
from itertools import combinations
from pathlib import Path


COL_GEN_OUTPUT = 0


def subsets_up_to_k(R, K, W=()):
    # Build conflict adjacency: adj[x] = {y that conflicts with x}
    adj = [set() for _ in range(R)]
    for a, b in W:
        adj[a].add(b)
        adj[b].add(a)

    # Heuristic: consider high-conflict nodes earlier to prune sooner
    nodes = sorted(range(R), key=lambda x: -len(adj[x]))

    curr = []

    def backtrack(start, banned):
        # Emit current subset (sizes 1..K)
        if 1 <= len(curr) <= K:
            yield frozenset(curr)
        if len(curr) == K:
            return

        for i in range(start, R):
            v = nodes[i]
            if v in banned:
                continue
            # Add v; update banned with its conflicts
            curr.append(v)
            new_banned = banned | adj[v]
            # Note: we don't need to add 'v' itself to banned because
            # we move forward with 'start=i+1', preventing reuse.
            yield from backtrack(i + 1, new_banned)
            curr.pop()

    yield from backtrack(0, set())

def Generate_Routes(instance : str, model : Model):
    start_time = time.perf_counter()
    inst = build_milp_data(str(instance))
    
    path = Path(instance)
    filename = path.name
    Time_Window = not filename.startswith("w")
    R        = inst["R"]
    Pr       = inst["Pr"]
    Dr_single= inst["Dr_single"]

    EPS      = 1e-9
    Q        = inst["Q"]
    W        = inst["W"]
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


    
    routes = list(subsets_up_to_k(len(R), len(R), W))
    costs = {}
    total = len(routes)
    print("Number of columns to generate:", total)
    print("Beginning Column Generation:")
    checkpoints = {int(total * p / 100) for p in range(5, 101, 5)}  # 5%, 10%, … 100%
    for idx, subset in enumerate(routes, start=1):
        _m, s_cost, arcs = Run_Model(subset, inst, False, COL_GEN_OUTPUT,Time_Window)

        if _m.Status not in (GRB.INFEASIBLE, GRB.CUTOFF):
            if not is_capacity_ok(arcs):
                continue 

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
    print("**********************************************")
    print(f"Total runtime {end_time-start_time:.2f}")
    print("Obj Value",round(model.ObjVal,3))
    print("**********************************************")
def main(argv=None):
    path, _ = parse_instance_argv(argv, default_filename="l_4_25_4.txt")
    model = Model("MPDTW")
    Generate_Routes(str(path), model)


if __name__ == "__main__":
    main()
