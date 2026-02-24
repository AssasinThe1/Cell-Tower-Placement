# Cell Tower Placement as a Constraint Satisfaction Problem

## 1. Problem Statement

We are given a rectangular grid of size **M × N**. Each cell `(r, c)` in this grid has an associated **demand value** `d[r][c]`, representing the amount of network resource (bandwidth, signal strength, user load, etc.) that must be satisfied at that location.

We have a set of **tower configurations** available to us. Each configuration `k` is described by three parameters:

- **Radius `R_k`**: the maximum distance (under a chosen metric) from which this tower type can serve a cell.
- **Capacity `C_k`**: the total budget of demand units this tower can serve across all cells it covers. This is **not** a per-cell threshold — it is a shared pool. If a tower with capacity 10 serves cells with demands 3, 4, and 5, it has used up 12 units and has exceeded its budget. It would need to drop one.
- **Cost `W_k`**: the price of deploying a tower with this configuration.

A tower may be placed at **any** grid cell. Different towers can use different configurations — there is no requirement that all towers be identical. When placed at position `(tr, tc)` with configuration `k`, a tower **can potentially reach** any cell `(cr, cc)` satisfying `dist((tr,tc), (cr,cc)) ≤ R_k`. However, actually **serving** a cell deducts that cell's demand from the tower's capacity pool. The tower must choose which reachable cells to serve, subject to the constraint that total served demand does not exceed `C_k`.

**The goal:** find a placement of towers (positions + configurations) and an assignment of every grid cell to exactly one tower, such that:

1. Every cell in the grid is served by exactly one tower.
2. Each tower's total served demand does not exceed its capacity.
3. The total deployment cost is minimized.

This is a **combinatorial optimization problem** that naturally decomposes into constraint satisfaction (feasibility) wrapped in a cost minimization loop.

---

## 2. Why This Is a CSP

A Constraint Satisfaction Problem is defined by three components:

- **Variables**: the unknowns we must assign values to.
- **Domains**: the set of possible values each variable can take.
- **Constraints**: the rules that valid assignments must satisfy.

Our problem maps cleanly onto this framework.

### 2.1 Variables

We model this as a **tower-centric** CSP rather than a cell-centric one. The decision at each step is: given a candidate tower `(pos, config_idx)`, which subset of reachable cells should it serve?

The implicit variable for each grid cell remains: "which tower is responsible for me?" But the solver branches on towers, not cells — deciding for each tower whether to activate it and which cells to pack into its capacity budget.

### 2.2 Domains

For each cell `(r, c)`, the set of towers that could potentially serve it is:

```
domain(r, c) = { (pos, ci) : dist(pos, (r,c)) ≤ R[ci]  AND  C[ci] ≥ d[r][c] }
```

Conversely, for each candidate tower `(pos, ci)`, its **coverage set** is:

```
covers(pos, ci) = { (r, c) : dist(pos, (r,c)) ≤ R[ci]  AND  C[ci] ≥ d[r][c] }
```

These are computed upfront. The capacity filter `C[ci] ≥ d[r][c]` prunes towers that cannot even serve a single cell's demand.

### 2.3 Constraints

**Coverage constraint (hard):** Every cell must be assigned to exactly one tower.

**Capacity constraint (hard, global):** For any active tower `(pos, ci)`, the sum of demands of all cells assigned to it must not exceed its capacity:

```
Σ d[r][c] for all (r,c) assigned to (pos, ci)  ≤  C[ci]
```

This is a **global constraint** that couples together all cells sharing a tower. It is what makes this problem fundamentally harder than simple set cover.

**Geometric constraint (hard):** A cell can only be assigned to a tower within range. Enforced by domain construction — infeasible pairs never enter the domain.

---

## 3. The Capacity Budget Model

This deserves special attention because it changes the nature of the problem fundamentally compared to simpler "threshold" models.

In a **threshold model**, a tower with capacity `C` can serve any cell with demand `≤ C`, independently. There is no interaction between cells served by the same tower. The problem reduces to standard set cover.

In our **budget model**, serving cell A affects whether the tower can also serve cell B. Each tower is essentially a **knapsack**: it has a weight limit (capacity), and each cell it serves has a weight (demand). The tower must choose a subset of reachable cells whose total demand fits within its capacity.

The solver must jointly solve all these knapsacks while ensuring global coverage. This creates rich interdependencies: if Tower 1 drops a cell to stay within budget, some other tower must pick it up, which may cause *that* tower to exceed its capacity and drop a different cell, and so on. These cascading effects are what make the problem NP-hard.

---

## 4. Why Tower-Centric Branching

An earlier iteration of our solver used **cell-centric** branching: pick an unassigned cell, try assigning it to each candidate tower. This has a critical flaw with the capacity budget model.

When we assign cell `(r, c)` to tower `T`, we deduct demand from `T`'s remaining capacity. But we have no picture of what `T`'s full load will look like — we're building it one cell at a time across many levels of the search tree. The capacity constraint only gets checked incrementally, leading to:

- Deep search trees before discovering a tower is overloaded.
- Massive backtracking when a tower that was committed to early turns out to be infeasible.
- Awkward bookkeeping: every cell assignment modifies shared tower state.

**Tower-centric branching** flips this: at each step, we pick a candidate tower and decide its **entire service set** at once. The capacity constraint is satisfied **by construction** — we pack cells into the tower's budget using a knapsack subroutine before committing. The branch point becomes "use this tower with this subset, or skip it entirely."

Benefits:

- **Capacity never violated during search.** The knapsack packing guarantees feasibility.
- **Lower branching factor.** We branch on towers (tens to hundreds) rather than cell-tower pairs (potentially thousands).
- **Natural pruning.** If skipping a tower leaves some cell unreachable by any remaining candidate, we detect infeasibility immediately.

---

## 5. Solver Architecture

### 5.1 Preprocessing: Domain and Coverage Construction

Before search begins, we compute:

- `covers[(pos, ci)]`: the set of cells reachable by tower `(pos, ci)` with demand ≤ capacity.
- `domains[cell]`: the set of `(pos, ci)` pairs that can serve this cell.
- `remaining_cap[(pos, ci)]`: initialized to `C[ci]` for every candidate.

This is pure geometric + capacity filtering. For each cell, we enumerate all grid positions and all configurations, checking distance and minimum capacity. The result is a bipartite structure: towers on one side, cells on the other, edges representing "can serve."

### 5.2 Tower Selection Heuristic

At each node in the search tree, we must pick which tower to branch on. We use a **desperation-weighted** heuristic:

For each uncovered cell, count how many remaining candidate towers can cover it. Cells with fewer options are more "desperate." A tower's score is the sum of `1 / options(c)` for each uncovered cell `c` in its coverage set.

```
score(tower) = Σ (1 / options(c))  for c in covers[tower] ∩ uncovered
```

This prioritizes towers that serve cells which have no other good options — a form of the **Most Constrained Variable** (MRV) heuristic applied at the tower level. If a cell has only one tower that can reach it, that tower gets an infinite score and is selected immediately.

### 5.3 Subset Generation via Greedy Knapsack

Once a tower is selected, we need to decide which cells it should serve. This is a knapsack problem: maximize coverage subject to the capacity budget. Since the full knapsack is NP-hard itself, we generate a small set of **candidate subsets** using greedy heuristics:

1. **Highest demand first:** Sort reachable uncovered cells by demand descending, greedily pack. This prioritizes hard-to-place cells — high-demand cells have fewer towers that can accommodate them.

2. **Lowest demand first:** Sort ascending, greedily pack. This maximizes the number of cells served, which is useful when many small-demand cells remain.

3. **Most desperate first:** Sort by `options(c)` ascending (fewest alternative towers), greedily pack. This ensures cells with no other options get served first.

Each heuristic produces a different subset. Duplicates are removed. The solver tries each subset as a separate branch.

### 5.4 Backtracking with Branch-and-Bound

The core search is depth-first backtracking. At each node:

1. **Base case:** If all cells are covered, record the solution if its cost improves the current best.

2. **Bound check:** If the cost so far already exceeds the best known solution, prune this branch.

3. **Feasibility check:** For every uncovered cell, verify that at least one remaining candidate tower can reach it. If any cell is unreachable, prune.

4. **Select tower** using the desperation heuristic.

5. **For each candidate subset** of that tower:
   - Assign cells to the tower, deduct capacity.
   - Recurse on the remaining uncovered cells with this tower removed from candidates.
   - Undo: restore capacity, unassign cells.

6. **Skip branch:** Recurse without using this tower at all (it might not be needed).

The "skip" branch is critical — it allows the solver to discover that a cheaper combination of other towers can cover the same cells.

### 5.5 Feasibility Pruning

The feasibility check at each node is the primary pruning mechanism, replacing the forward checking and AC-3 from the earlier cell-centric formulation:

```
for each uncovered cell c:
    if no candidate tower in remaining set can reach c:
        prune (backtrack immediately)
```

This is fast to compute and catches dead ends early. When we skip a tower, any cell that was *only* coverable by that tower becomes a dead end, triggering immediate backtrack.

---

## 6. Distance Metrics

The coverage radius can be interpreted under different distance metrics, each producing a different "shape" of coverage:

- **Manhattan distance**: `|r1-r2| + |c1-c2| ≤ R` — diamond-shaped coverage. Most common in grid-based models.
- **Euclidean distance**: `√((r1-r2)² + (c1-c2)²) ≤ R` — circular coverage. More realistic for radio propagation.
- **Chebyshev distance**: `max(|r1-r2|, |c1-c2|) ≤ R` — square coverage. Covers more cells per radius unit.

The choice of metric affects coverage set sizes, constraint density, and solver performance. Manhattan produces the tightest coverage (smallest sets), Chebyshev the loosest.

---

## 7. Complexity Analysis

The problem generalizes both **set cover** (NP-hard) and **bin packing** (NP-hard). Our solver handles this through:

- **Domain pruning** (geometric + capacity filtering) eliminates infeasible tower-cell pairs before search begins.
- **Tower selection heuristic** (desperation-weighted MRV) ensures we branch on the most constrained decisions first, minimizing wasted exploration.
- **Multiple greedy subsets** provide diverse branching options without the exponential cost of enumerating all feasible subsets.
- **Branch-and-bound** prunes any partial solution that already exceeds the best known cost.
- **Feasibility pruning** catches dead ends immediately when a cell becomes unreachable.

For moderate grid sizes (up to ~8×8 with 3–4 configurations), this approach finds optimal or near-optimal solutions within seconds. For larger instances, the solver can be wrapped with a time limit, returning the best solution found so far.

---

## 8. Data Structures

| Structure | Type | Purpose |
|-----------|------|---------|
| `covers[(pos, ci)]` | `set of cells` | Cells reachable by each candidate tower |
| `domains[cell]` | `set of (pos, ci)` | Candidate towers for each cell |
| `remaining_cap[(pos, ci)]` | `int` | Remaining capacity of each tower during search |
| `assignment[cell]` | `(pos, ci) or None` | Which tower serves each cell in current partial solution |
| `candidates` | `set of (pos, ci)` | Towers still available for selection |
| `uncovered` | `set of cells` | Cells not yet assigned to any tower |
| `best` | `dict` | Tracks best complete solution and its cost |

---

## 9. Algorithm Pseudocode

```
SOLVE(grid, demand, tower_configs):
    Precompute covers, domains, remaining_cap
    candidates ← all (pos, config) pairs with non-empty coverage
    best ← {cost: ∞, solution: None}
    BACKTRACK(all cells, cost=0, candidates)
    return best

BACKTRACK(uncovered, cost_so_far, candidates):
    if uncovered is empty:
        if cost_so_far < best.cost:
            best ← current assignment
        return

    if cost_so_far ≥ best.cost:
        return  // bound

    for each cell in uncovered:
        if no candidate can cover cell:
            return  // infeasible

    tower ← SELECT_TOWER(candidates, uncovered)  // MRV-style

    for each subset in GENERATE_SUBSETS(tower, uncovered):
        Assign subset to tower, deduct capacity
        BACKTRACK(uncovered - subset, cost + tower.cost, candidates - {tower})
        Undo assignment, restore capacity

    BACKTRACK(uncovered, cost_so_far, candidates - {tower})  // skip tower

SELECT_TOWER(candidates, uncovered):
    return tower maximizing Σ 1/options(c) for c in its uncovered reachable cells

GENERATE_SUBSETS(tower, uncovered):
    reachable ← tower's coverage ∩ uncovered
    return greedy knapsack packings: by demand↓, demand↑, desperation↓
```

---

## 10. Summary

| Component | Role |
|-----------|------|
| **Grid + Demand** | Defines what needs to be covered and how much |
| **Tower Configs** | Defines available tools: radius, capacity, cost |
| **Coverage Map** | Precomputed bipartite structure: which towers reach which cells |
| **Tower Selection** | Desperation-weighted MRV: branch on most critical tower first |
| **Greedy Knapsack** | Generates feasible cell subsets per tower without full enumeration |
| **Backtracking** | Depth-first search over tower activation/subset decisions |
| **Branch-and-Bound** | Prunes branches exceeding current best cost |
| **Feasibility Check** | Immediate backtrack if any cell becomes unreachable |

The solver proceeds by iteratively selecting towers, packing cells into their capacity budgets, backtracking on dead ends, and pruning suboptimal branches — all without external solver libraries, using only fundamental CSP techniques: backtracking search, constraint propagation via feasibility checks, variable ordering heuristics, and branch-and-bound optimization.