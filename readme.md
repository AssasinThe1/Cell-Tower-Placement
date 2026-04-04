# Tower Placement Optimization — CSP + Heuristic Comparison

**Constraint Crusaders** — Interdisciplinary Applications in AI, Phase 1

Paramata Swaroop Nanda · Saankhya Samanta · Karan Punja Kavatra · Atharva Sunil Pote

---

## What This Does

Places cell towers on a city grid to maximize demand-weighted average Quality of Service (QoS) under a budget constraint. The codebase implements two fundamentally different approaches and compares them:

- **Part 1 (20×20 grid):** Exact CSP solver vs. heuristic SA algorithms on the same small problem — validates that the heuristics reach the same optimal QoS.
- **Part 2 (300×250 grid):** Only heuristic algorithms run — CSP can't scale to this size. Three SA variants (A, B, C) are compared.

---

## Build & Run

```bash
g++ -O2 -std=c++17 -o tower tower.cpp
./tower
```

No external dependencies. Standard C++17.

---

## How It Works

### Physics Model

Every tower's signal is computed with three propagation effects:

1. **Path Loss** — Power decays as `P_tx / d^n` (n = 3.5, typical urban).
2. **Building Penetration** — A DDA ray is cast from tower to cell. Distance through building cells is accumulated and applied as `exp(-attenuation * penetration_distance)`.
3. **Diffraction** — If the direct path hits a building but the receiver is outdoors, signal bending around building corners is computed using a simplified UTD model.

After all tower signals are summed, **Shannon capacity** (`log2(1 + SINR)`) determines each cell's spectral efficiency, and a proportional-fair capacity allocator distributes total tower capacity across demand cells. The final score is the demand-weighted average QoS (0–100%).

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

Three configurations are run: Plain BT, MRV, MRV+LCV.

### Heuristic Algorithms (Parts 1 & 2)

All three share a **greedy fill** (pick highest marginal-Shannon-gain per cost) and a **simulated annealing** refinement (4 move types: nudge, swap type, add, remove; Metropolis acceptance on geometric cooling). They differ in initialization:

| Algorithm | Initialization | SA Iters | Key Idea |
|---|---|---|---|
| **A: Window+SA** | Divide grid into overlapping windows, greedy fill each with equal budget share | 500 | Divide-and-conquer; parallelizable |
| **B: Greedy+SA** | Single global greedy on an 8-cell coarse grid | 600 | Global budget allocation; no local fragmentation |
| **C: Cluster+SA** | Demand-weighted k-means → seed cheapest towers at centroids → greedy fill gaps | 600 | Demand-aware; targets hotspots directly |

---

## Output Files

The program exports CSVs for visualization:

| File | Contents |
|---|---|
| `small_buildings.csv` | Building cell positions (20×20 grid) |
| `small_demand.csv` | Demand values per cell (20×20 grid) |
| `small_csp_towers.csv` | Best CSP placement |
| `small_sa_towers.csv` | Best SA placement |
| `big_buildings.csv` / `big_demand.csv` | Same for the 300×250 grid |
| `A_WindowSA_towers.csv` etc. | Per-algorithm placements (large grid) |

---

## Key Code Sections

**`calculateBuildingPenetrationDDA()`** — DDA ray marching through the grid. Walks cell boundaries along the ray, accumulates distance through building cells. Core of the penetration loss model.

**`applyTowerSignal()`** — Full signal computation for one tower: path loss + DDA penetration + corner diffraction. Pre-computes a max-range cutoff from the noise floor to avoid scanning the whole grid.

**`resolveNetworkCapacity()`** — Converts raw signal to QoS. Computes Shannon efficiency per cell, calculates required capacity, applies proportional-fair scaling if total demand exceeds total capacity.

**`buildCSP()` + `CSPSolver::solve()`** — CSP construction (covers map, domains, dominance pruning) and recursive backtracking with configurable MRV/LCV/FC/AC-3.

**`simulatedAnnealing()`** — SA backbone shared by all heuristics. Four move types, geometric cooling, Metropolis acceptance.

**`snapToValid()`** — Converts floating-point coordinates (e.g., k-means centroids) to the nearest valid grid cell via spiral search.

---

## Configuration

Key parameters in `main()`:

```
Part 1: 20×20 grid, budget $25,000, tower types Macro/Micro/Pico (power 0.004–0.15)
Part 2: 300×250 grid, budget $400,000, tower types Macro/Micro/Pico (power 25–600)
```

Hyperparameters (set at top of `main()`):
- `path_loss_exp = 3.5`
- `atten_per_unit = 0.20`
- `diff_strength = 0.60`
- `diff_exp = 4.0`
- `noise_floor = 1e-4`

Random seed is fixed at 42 for reproducibility.
