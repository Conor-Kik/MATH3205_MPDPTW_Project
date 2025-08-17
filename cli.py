# cli.py (project root)
import os, sys, importlib
from typing import Dict

# Ensure src/ is importable
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

# Registry: map (method, approach) -> "module.path:callable"
REGISTRY: Dict[str, Dict[str, str]] = {
    "two_index": {
        "baseline": "mpdptw.methods.two_index.MPDTW_Solver:main",

    },
    "three_index": {
    },
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
    # Parse only the method/approach here; defer the rest to the solver.
    if len(sys.argv) < 2 or sys.argv[1] in {"-h", "--help"}:
        print("Usage: python cli.py <method> <approach> [solver-args...]")
        print("\nAvailable:")
        for m, approaches in REGISTRY.items():
            print(f"  {m}: {', '.join(sorted(approaches.keys())) or '(none)'}")
        sys.exit(0)

    method = sys.argv[1]
    if method not in REGISTRY:
        print(f"Unknown method '{method}'. Known: {', '.join(REGISTRY.keys())}")
        sys.exit(2)

    if len(sys.argv) < 3:
        print(f"Missing approach for method '{method}'. Options: {', '.join(REGISTRY[method].keys()) or '(none)'}")
        sys.exit(2)

    approach = sys.argv[2]
    if approach not in REGISTRY[method]:
        print(f"Unknown approach '{approach}' for method '{method}'. "
              f"Options: {', '.join(REGISTRY[method].keys())}")
        sys.exit(2)

    # Everything after method + approach belongs to the solver
    solver_argv = sys.argv[3:]
    entry = REGISTRY[method][approach]
    func = load_entry(entry)

    # Call the solver's main, forwarding its argv
    func(solver_argv)

if __name__ == "__main__":
    main()
