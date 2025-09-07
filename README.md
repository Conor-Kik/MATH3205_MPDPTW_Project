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
python cli.py <method> <instance_filename>
```

- `<method>` → the method name (e.g., `two_index`)
- `<instance_filename>` → the instance file located inside `mpdtw_instances_2019/`

---

### Available Methods

Currently implemented methods include:

- `two_index` → Two-index formulation solver  
- `three_index` → Three-index formulation solver  
- `arf` → Asymmetric representatives formulation 
- `col_gen` → Column Generation approach  

Each method has its own subdirectory under `src/mpdptw/methods/`.

---

### Example Commands

```bash
python cli.py two_index MPDTW_Solver l_4_25_1.txt
python cli.py three_index ThreeIndexSolver l_6_50_3.txt
python cli.py arf ARF_Solver l_8_75_2.txt
python cli.py col_gen ColGenSolver l_10_100_1.txt
```

---

## Documentation

- All project reports, LaTeX sources, and reference papers are stored in the `docs/` folder.  

---

## Notes

- Always run from the **project root** so paths resolve correctly.  
- The project uses **Gurobi** as the solver, so ensure it is properly installed and licensed.  
