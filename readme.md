# Tower Placement Optimization — CSP + Heuristic Comparison

**Constraint Crusaders** — Interdisciplinary Applications in AI, Phase 1

Paramata Swaroop Nanda · Saankhya Samanta · Karan Punja Kavatra · Atharva Sunil Pote

---

## What This Does

Places cell towers on a city grid to maximize demand-weighted average Quality of Service (QoS) under a budget constraint. Two fundamentally different approaches are compared:

- **Part 1 (Small Grid):** Exact CSP solver vs. heuristic SA algorithms on the same problem — validates that the heuristics reach the same optimal QoS.
- **Part 2 (300×250 Grid):** Only heuristic algorithms run — CSP can't scale to this size. Three SA variants (A, B, C) are compared.

---

## Repository Structure

```
├── tower.cpp              # Full source — physics engine, CSP solver, SA algorithms, main()
├── plot.py                # Visualization script — reads all CSVs, generates all plots
├── README.md              # This file
└── report.tex             # LaTeX project report
```

After running `./tower`, these CSVs are generated:

```
├── small_grid_info.csv          # Grid dimensions (rows, cols) for Part 1
├── small_buildings.csv          # Building cell positions
├── small_demand.csv             # Demand values per cell
├── small_results.csv            # Score/towers/cost/nodes/time for all algorithms
├── small_D_MRV_towers.csv       # Tower placements per algorithm (one file each)
├── small_D_MRV_LCV_towers.csv
├── small_A_WindowSA_towers.csv
├── small_B_GlobalGreedy_towers.csv
├── small_C_ClusterSA_towers.csv
├── big_grid_info.csv            # Grid dimensions for Part 2
├── big_buildings.csv
├── big_demand.csv
├── big_results.csv
├── big_A_WindowSA_towers.csv
├── big_B_GlobalGreedy_towers.csv
└── big_C_ClusterSA_towers.csv
```

After running `python plot.py`, these plots are generated:

```
├── plot_small_grid.png    # All algorithm placements on the small grid
├── plot_big_grid.png      # All SA placements on the 300×250 grid
└── plot_comparison.png    # Bar charts: node counts, QoS scores, tower counts
```

Note: `small_D_PlainBT_towers.csv` is not generated because Plain BT typically times out without finding a solution.

---

## Build & Run

```bash
# Compile
g++ -O2 -std=c++17 -o tower tower.cpp

# Run with default 10×10 small grid
./tower

# Run with custom small grid size (e.g. 15×15)
./tower 15 15

# Run with 20×20 small grid
./tower 20 20

# Generate plots (requires matplotlib)
pip install matplotlib
python plot.py
```

The small grid size is capped at 50×50 so the CSP doesn't run forever. The large grid (Part 2) is always 300×250.

---

## How It Works

### Physics Model

Every tower's signal is computed with three propagation effects:

1. **Path Loss** — Power decays as `P_tx / d^n` (n = 3.5, typical urban).
2. **Building Penetration** — A DDA ray is cast from tower to cell. Distance through building cells is accumulated and applied as `exp(-attenuation * penetration_distance)`.
3. **Diffraction** — If the direct path hits a building but the receiver is outdoors, signal bending around building corners is computed using a simplified UTD model.

After all tower signals are summed, **Shannon capacity** (`log2(1 + SINR)`) determines each cell's spectral efficiency, and a proportional-fair capacity allocator distributes total tower capacity across demand cells. The final score is the demand-weighted average QoS (0–100%).

Note: building cells are excluded from QoS scoring. Even though buildings have demand assigned (5× normal, simulating indoor users), only non-building demand cells count toward the score.

### CSP Solver (Part 1 only)

Each demand cell is a variable; its domain is the set of towers that can serve it. The solver finds a minimum-cost assignment where every cell is covered and no tower is overloaded.

| Technique | What It Does |
|---|---|
| **MRV** | Pick the unassigned cell with the fewest remaining feasible towers. Causes failures early. |
| **LCV** | Try towers in order of how few other cells they'd restrict. Least constraining first. |
| **Forward Checking** | After assignment, remove infeasible towers from neighbors' domains. Wipeout → backtrack. |
| **AC-3** | Pairwise arc consistency propagation across cells sharing a tower. |
| **Branch-and-Bound** | Prune any branch whose cost already exceeds the best known solution. |
| **Dominance Pruning** | Pre-processing: remove tower A if tower B covers a superset of A's cells at ≤ cost. |

Three configurations are run: Plain BT (often times out), MRV, MRV+LCV.

### Heuristic Algorithms (Parts 1 & 2)

All three share a **greedy fill** (pick highest marginal-Shannon-gain per cost) and a **simulated annealing** refinement (4 move types: nudge 60%, swap type 15%, add 15%, remove 10%; Metropolis acceptance on geometric cooling). They differ in initialization:

| Algorithm | Initialization | SA Iters | Key Idea |
|---|---|---|---|
| **A: Window+SA** | Divide grid into overlapping windows, greedy fill each with equal budget share | 500 | Divide-and-conquer; parallelizable |
| **B: Greedy+SA** | Single global greedy on an 8-cell coarse grid | 600 | Global budget allocation; no local fragmentation |
| **C: Cluster+SA** | Demand-weighted k-means → seed cheapest towers at centroids → greedy fill gaps | 600 | Demand-aware; targets hotspots directly |

---

## Key Code Sections

**`calculateBuildingPenetrationDDA()`** — DDA ray marching through the grid. Walks cell boundaries along the ray, accumulates distance through building cells.

**`applyTowerSignal()`** — Full signal computation for one tower: path loss + DDA penetration + corner diffraction. Pre-computes max-range cutoff from noise floor.

**`resolveNetworkCapacity()`** — Converts raw signal to QoS. Shannon efficiency per cell, proportional-fair scaling if total demand exceeds total capacity.

**`buildCSP()` + `CSPSolver::solve()`** — CSP construction (coverage map, domains, dominance pruning) and recursive backtracking with configurable MRV/LCV/FC/AC-3.

**`simulatedAnnealing()`** — SA backbone. Four move types, geometric cooling, Metropolis acceptance.

**`snapToValid()`** — Converts floating-point coordinates (k-means centroids) to nearest valid grid cell via spiral search.

---

## Configuration

Key parameters in `main()`:

```
Part 1: NxN grid (default 10x10, configurable via CLI), budget $25,000
        Tower types: Macro (power 0.15, cost $5000, cap 30)
                     Micro (power 0.02, cost $2500, cap 15)
                     Pico  (power 0.004, cost $1000, cap 8)

Part 2: 300x250 grid (fixed), budget $400,000
        Tower types: Macro (power 600, cost $60000, cap 4000)
                     Micro (power 150, cost $30000, cap 1200)
                     Pico  (power 25,  cost $12000, cap 300)
```

Hyperparameters:
- `path_loss_exp = 3.5`
- `atten_per_unit = 0.20`
- `diff_strength = 0.60`
- `diff_exp = 4.0`
- `noise_floor = 1e-4`

Random seed is fixed at 42 for reproducibility.