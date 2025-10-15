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
python cli.py <method> <instance_filename> [solver-args...]
```

- `<method>` → the method name (e.g., `two_index`)  
- `<instance_filename>` → the instance file located inside `mpdtw_instances_2019/`  
- `[solver-args...]` → optional additional arguments for the solver  

---

### Available Methods

Currently implemented methods include:

- `two_index` → Two-index formulation solver  
- `three_index` → Three-index formulation solver  
- `arf` → Asymmetric representatives formulation  
- `col_gen` → Column Generation approach  
  - Add `--mt` after the instance filename to use the **multi-threaded column generation** implementation.  
  - Add `--cap` after the instance filename to use the **capacitated constraints**

Each method has its own subdirectory under `src/mpdptw/methods/`.

---

### Example Commands

```bash
python cli.py two_index l_4_25_1.txt
python cli.py three_index l_4_35_2.txt
python cli.py arf n_4_50_3.txt
python cli.py col_gen w_8_100_4.txt
python cli.py col_gen w_8_100_4.txt --mt   # Multi-threaded column generation
```

---

## Documentation

- All project reports, LaTeX sources, and reference papers are stored in the `docs/` folder.  

---

## Notes

- Always run from the **project root** so paths resolve correctly.  
- The project uses **Gurobi** as the solver, so ensure it is properly installed and licensed.  

## References

- This project builds on the work of [Aziez, Côté, and Coelho (2020)](https://www.sciencedirect.com/science/article/pii/S0377221720300771), *Exact algorithms for the multi-pickup and delivery problem with time windows*, *European Journal of Operational Research, 284(3)*, pp. 906–919. The Two- and Three-index models, as well as the ARF model, follow the formulations presented in this paper.  
- The column generation approach has been developed independently as part of this project.
