# cli.py (project root)
import os, sys, importlib
from typing import Dict

# Ensure src/ is importable
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

# Registry: map method -> "module.path:callable"
REGISTRY: Dict[str, str] = {
    "two_index": "mpdptw.methods.two_index.MPDTW_Solver_Two:main",
    "three_index": "mpdptw.methods.three_index.MPDTW_Solver:main",
    "arf": "mpdptw.methods.arf.MPDTW_Solver_Arf:main",
    "col_gen": "mpdptw.methods.col_generation.route_generation:main",
    "col_gen_mt": "mpdptw.methods.col_generation.multi_thread_route_generation:main",
}

def load_entry(entry: str):
    """Load 'package.module:func' and return the func."""
    module_path, func_name = entry.split(":")
    mod = importlib.import_module(module_path)
    try:
        func = getattr(mod, func_name)
    except AttributeError as e:
        raise SystemExit(f"Callable '{func_name}' not found in {module_path}") from e
    return func

def main():
    # Expect at least <method> <instance_file>
    if len(sys.argv) < 3 or sys.argv[1] in {"-h", "--help"}:
        print("Usage: python cli.py <method> <instance_filename> [solver-args...]")
        print("\nAvailable methods:")
        for m in ["two_index", "three_index", "arf", "col_gen"]:
            print(f"  {m}")
        print("\nUse --mt with col_gen for multi-threaded version.")
        sys.exit(0)

    method = sys.argv[1]
    if method not in REGISTRY:
        print(f"Unknown method '{method}'. Known: two_index, three_index, arf, col_gen")
        sys.exit(2)

    # Everything after <method> is passed to the solver
    solver_argv = sys.argv[2:]

    # Special handling: col_gen with --mt flag
    if method == "col_gen" and "--mt" in solver_argv:
        solver_argv = [arg for arg in solver_argv if arg != "--mt"]  # remove flag
        entry = REGISTRY["col_gen_mt"]
    else:
        entry = REGISTRY[method]

    func = load_entry(entry)
    func(solver_argv)

if __name__ == "__main__":
    main()
