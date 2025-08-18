# MATH3205 MPDPTW Project

This repository contains our implementation and experiments for solving the Multi-Pickup and Delivery Problem with Time Windows (MPDPTW).

---

## Project Structure

```
MATH3205_MPDPTW_Project/
├── src/
│   └── mpdptw/
│       ├── common/                 # Shared utilities (parsers, printers, etc.)
│       └── methods/                # Different solution methods
│           ├── two_index_formulation/
│           │   └── MPDTW_Solver.py
│           └── <other_method>/
│               └── <Approach>.py
│
├── mpdtw_instances_2019/           # Benchmark instance files (.txt)
├── docs/                           # Project report, LaTeX documentation, papers used
├── cli.py                          # CLI entrypoint
└── README.txt                      # This file

```

---

## Running the Project (CLI)

The entry point is `cli.py` at the project root.

### General usage

```bash
python cli.py <method> <approach> <instance_filename>
```

- `<method>` → the method name (e.g., `two_index`)
- `<approach>` → the specific solver/approach file
- `<instance_filename>` → the instance file located inside `mpdtw_instances_2019/`

### Example

```bash
python cli.py two_index baseline l_4_25_1.txt
```

This will:
1. Load `mpdtw/methods/two_index/MPDTW_Solver.py`
2. Run it on the instance `mpdtw_instances_2019/l_4_25_1.txt`

---

##  Documentation

- All project reports, LaTeX sources, and reference papers are stored in the `docs/` folder.  

---

## Notes

- Always run from the **project root** so paths resolve correctly.
- The project uses **Gurobi** as the solver, so ensure it is properly installed and licensed.
