#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <set>
#include <map>
#include <deque>
#include <chrono>
#include <string>
#include <numeric>

// ============================================================
//  Problem Setup — 10x10 grid, same idea as main.cpp
// ============================================================

struct TowerType
{
  std::string name;
  double cost;
  double capacity;  // max total demand it can serve
  double radius;    // coverage radius (Euclidean cells)
};

struct CellInfo
{
  int x, y;
  double demand;
};

// A candidate tower: position + type
struct TowerKey
{
  int px, py;
  int type_idx;

  bool operator==(const TowerKey &o) const
  { return px == o.px && py == o.py && type_idx == o.type_idx; }
  bool operator<(const TowerKey &o) const
  {
    if (px != o.px) return px < o.px;
    if (py != o.py) return py < o.py;
    return type_idx < o.type_idx;
  }
};

// ============================================================
//  CSP Problem
// ============================================================
struct CSPProblem
{
  std::vector<CellInfo> demand_cells;               // variables
  std::vector<TowerKey> all_towers;                  // all candidates
  std::map<TowerKey, std::set<int>> covers;          // tower -> cell indices
  std::vector<std::set<TowerKey>> domains;           // domains[cell] -> towers
  std::vector<TowerType> tower_types;
};

CSPProblem buildProblem(
    int rows, int cols,
    const std::vector<std::vector<double>> &demand,
    const std::vector<std::vector<bool>> &buildings,
    const std::vector<TowerType> &tower_types,
    const std::vector<std::pair<int,int>> &tower_positions)  // fixed spots
{
  CSPProblem prob;
  prob.tower_types = tower_types;

  // Collect demand cells
  for (int j = 0; j < rows; ++j)
    for (int i = 0; i < cols; ++i)
      if (demand[j][i] > 0 && !buildings[j][i])
        prob.demand_cells.push_back({i, j, demand[j][i]});

  int n = (int)prob.demand_cells.size();
  prob.domains.resize(n);

  // Build candidates from fixed positions only
  for (const auto &[px, py] : tower_positions)
  {
    if (buildings[py][px]) continue;
    for (int t = 0; t < (int)tower_types.size(); ++t)
    {
      TowerKey tk{px, py, t};
      std::set<int> covered;
      for (int c = 0; c < n; ++c)
      {
        double d = std::hypot(prob.demand_cells[c].x - px,
                              prob.demand_cells[c].y - py);
        if (d <= tower_types[t].radius)
          covered.insert(c);
      }
      if (!covered.empty())
      {
        prob.covers[tk] = covered;
        prob.all_towers.push_back(tk);
        for (int c : covered)
          prob.domains[c].insert(tk);
      }
    }
  }

  // Dominance pruning
  std::set<TowerKey> dominated;
  for (int i = 0; i < (int)prob.all_towers.size(); ++i)
  {
    const auto &a = prob.all_towers[i];
    if (dominated.count(a)) continue;
    for (int j = 0; j < (int)prob.all_towers.size(); ++j)
    {
      if (i == j) continue;
      const auto &b = prob.all_towers[j];
      if (dominated.count(b)) continue;
      if (std::includes(prob.covers[b].begin(), prob.covers[b].end(),
                        prob.covers[a].begin(), prob.covers[a].end()) &&
          tower_types[a.type_idx].cost >= tower_types[b.type_idx].cost &&
          !(a == b))
      { dominated.insert(a); break; }
    }
  }
  for (const auto &d : dominated)
  {
    prob.covers.erase(d);
    for (auto &dom : prob.domains) dom.erase(d);
  }
  prob.all_towers.erase(
      std::remove_if(prob.all_towers.begin(), prob.all_towers.end(),
                     [&](const TowerKey &t) { return dominated.count(t); }),
      prob.all_towers.end());

  return prob;
}

// ============================================================
//  Solver with toggleable MRV / LCV / FC / AC3
// ============================================================

struct SolverConfig
{
  std::string name;
  bool use_mrv, use_lcv, use_forward_check, use_ac3;
};

struct SolverResult
{
  bool solved;
  double cost;
  int nodes_explored;
  int backtracks;
  double time_seconds;
  std::map<int, TowerKey> assignment;
};

class CSPSolver
{
  const CSPProblem &prob;
  SolverConfig config;

  std::vector<std::set<TowerKey>> domains;
  std::map<TowerKey, double> remaining_cap;
  std::map<int, TowerKey> assignment;
  int nodes_explored, backtracks;
  double best_cost;
  std::map<int, TowerKey> best_assignment;
  bool found_any;

  // ---- MRV ----
  int selectVariable(const std::vector<int> &unassigned)
  {
    if (!config.use_mrv) return unassigned[0];

    int best = unassigned[0];
    int best_sz = (int)domains[best].size();
    for (int cid : unassigned)
    {
      int sz = (int)domains[cid].size();
      if (sz < best_sz) { best_sz = sz; best = cid; }
    }
    return best;
  }

  // ---- LCV ----
  std::vector<TowerKey> orderValues(int cell_id, const std::vector<int> &unassigned)
  {
    std::vector<TowerKey> ordered(domains[cell_id].begin(), domains[cell_id].end());
    if (!config.use_lcv || ordered.size() <= 1) return ordered;

    double my_d = prob.demand_cells[cell_id].demand;
    std::vector<int> impact(ordered.size(), 0);
    for (int v = 0; v < (int)ordered.size(); ++v)
    {
      double new_cap = remaining_cap[ordered[v]] - my_d;
      for (int o : unassigned)
      {
        if (o == cell_id) continue;
        if (!domains[o].count(ordered[v])) continue;
        if (new_cap < prob.demand_cells[o].demand) impact[v]++;
      }
    }
    std::vector<int> idx(ordered.size());
    std::iota(idx.begin(), idx.end(), 0);
    std::sort(idx.begin(), idx.end(), [&](int a, int b){ return impact[a]<impact[b]; });

    std::vector<TowerKey> res(ordered.size());
    for (int i = 0; i < (int)idx.size(); ++i) res[i] = ordered[idx[i]];
    return res;
  }

  // ---- Forward Checking ----
  struct Prune { int cid; TowerKey tk; };

  std::vector<Prune> forwardCheck(int cell, const TowerKey &tk,
                                   const std::vector<int> &unassigned, bool &wipeout)
  {
    std::vector<Prune> pruned;
    wipeout = false;
    for (int o : unassigned)
    {
      if (o == cell) continue;
      if (!domains[o].count(tk)) continue;
      if (remaining_cap[tk] < prob.demand_cells[o].demand)
      {
        domains[o].erase(tk);
        pruned.push_back({o, tk});
        if (domains[o].empty()) { wipeout = true; return pruned; }
      }
    }
    return pruned;
  }

  // ---- AC-3 ----
  std::vector<Prune> ac3(const std::vector<int> &unassigned, bool &wipeout)
  {
    std::vector<Prune> pruned;
    wipeout = false;
    std::set<int> uset(unassigned.begin(), unassigned.end());

    std::map<TowerKey, std::vector<int>> t2c;
    for (int cid : unassigned)
      for (const auto &tk : domains[cid])
        t2c[tk].push_back(cid);

    struct Arc { int ci, cj; TowerKey tk; };
    std::deque<Arc> queue;
    for (const auto &[tk, cl] : t2c)
      for (int i = 0; i < (int)cl.size(); ++i)
        for (int j = i+1; j < (int)cl.size(); ++j)
        {
          queue.push_back({cl[i], cl[j], tk});
          queue.push_back({cl[j], cl[i], tk});
        }

    while (!queue.empty())
    {
      auto [ci, cj, tk] = queue.front(); queue.pop_front();
      if (!uset.count(ci) || !uset.count(cj)) continue;
      if (!domains[ci].count(tk)) continue;

      double cap_both = remaining_cap[tk] - prob.demand_cells[ci].demand
                        - prob.demand_cells[cj].demand;
      if (cap_both < 0)
      {
        auto alt = domains[cj]; alt.erase(tk);
        if (alt.empty())
        {
          domains[ci].erase(tk);
          pruned.push_back({ci, tk});
          if (domains[ci].empty()) { wipeout = true; return pruned; }
          for (const auto &tk2 : domains[ci])
            for (int nb : t2c[tk2])
              if (nb != ci && uset.count(nb))
                queue.push_back({nb, ci, tk2});
        }
      }
    }
    return pruned;
  }

  // ---- Backtracking ----
  void backtrack(std::vector<int> &unassigned, double cost)
  {
    if (unassigned.empty())
    {
      if (cost < best_cost)
      { best_cost = cost; best_assignment = assignment; found_any = true; }
      return;
    }
    if (cost >= best_cost) return;
    for (int cid : unassigned)
      if (domains[cid].empty()) { backtracks++; return; }

    int cell = selectVariable(unassigned);

    auto it = std::find(unassigned.begin(), unassigned.end(), cell);
    std::swap(*it, unassigned.back());
    unassigned.pop_back();

    auto ordered = orderValues(cell, unassigned);

    for (const TowerKey &tk : ordered)
    {
      bool reused = false;
      for (const auto &[a, atk] : assignment)
        if (atk == tk) { reused = true; break; }

      double added = reused ? 0.0 : prob.tower_types[tk.type_idx].cost;
      if (cost + added >= best_cost) continue;

      assignment[cell] = tk;
      remaining_cap[tk] -= prob.demand_cells[cell].demand;
      nodes_explored++;

      auto saved = domains;
      bool wipeout = false;

      if (config.use_forward_check && !wipeout)
      {
        forwardCheck(cell, tk, unassigned, wipeout);
        if (wipeout) backtracks++;
      }
      if (config.use_ac3 && !wipeout)
      {
        ac3(unassigned, wipeout);
        if (wipeout) backtracks++;
      }
      if (!wipeout)
        backtrack(unassigned, cost + added);

      domains = saved;
      remaining_cap[tk] += prob.demand_cells[cell].demand;
      assignment.erase(cell);
    }

    unassigned.push_back(cell);
    backtracks++;
  }

public:
  CSPSolver(const CSPProblem &p, SolverConfig c) : prob(p), config(c) {}

  SolverResult solve(double budget)
  {
    domains = prob.domains;
    remaining_cap.clear();
    for (const auto &[tk, _] : prob.covers)
      remaining_cap[tk] = prob.tower_types[tk.type_idx].capacity;
    assignment.clear();
    nodes_explored = backtracks = 0;
    best_cost = budget + 1.0;
    found_any = false;

    std::vector<int> unassigned;
    for (int i = 0; i < (int)prob.demand_cells.size(); ++i)
      if (!domains[i].empty()) unassigned.push_back(i);

    auto t0 = std::chrono::steady_clock::now();
    backtrack(unassigned, 0.0);
    auto t1 = std::chrono::steady_clock::now();

    return {found_any, found_any ? best_cost : 0,
            nodes_explored, backtracks,
            std::chrono::duration<double>(t1 - t0).count(),
            best_assignment};
  }
};

// ============================================================
//  Print tower assignments on the grid
// ============================================================
void printSolution(const SolverResult &res, const CSPProblem &prob,
                   int rows, int cols,
                   const std::vector<std::vector<double>> &demand,
                   const std::vector<std::vector<bool>> &buildings)
{
  if (!res.solved) { std::cout << "    No solution.\n"; return; }

  std::map<TowerKey, std::vector<int>> tower_cells;
  for (const auto &[cid, tk] : res.assignment)
    tower_cells[tk].push_back(cid);

  std::map<TowerKey, char> tlabel;
  char lab = 'A';
  for (const auto &[tk, _] : tower_cells) tlabel[tk] = lab++;

  std::map<std::pair<int,int>, char> clabel;
  for (const auto &[cid, tk] : res.assignment)
    clabel[{prob.demand_cells[cid].x, prob.demand_cells[cid].y}] = tlabel[tk];

  std::set<std::pair<int,int>> tpos;
  for (const auto &[tk, _] : tower_cells)
    tpos.insert({tk.px, tk.py});

  std::cout << "\n    Grid (letter = tower assignment, * = tower location):\n";
  for (int j = 0; j < rows; ++j)
  {
    std::cout << "    ";
    for (int i = 0; i < cols; ++i)
    {
      if (buildings[j][i])
        std::cout << " ## ";
      else if (tpos.count({i, j}) && clabel.count({i, j}))
        std::cout << " " << clabel[{i, j}] << "* ";
      else if (tpos.count({i, j}))
        std::cout << " ** ";
      else if (clabel.count({i, j}))
        std::cout << "  " << clabel[{i, j}] << " ";
      else if (demand[j][i] > 0)
        std::cout << "  ? ";
      else
        std::cout << "  . ";
    }
    std::cout << "\n";
  }

  std::cout << "\n    Towers placed:\n";
  for (const auto &[tk, cells] : tower_cells)
  {
    const auto &t = prob.tower_types[tk.type_idx];
    double load = 0;
    for (int c : cells) load += prob.demand_cells[c].demand;
    std::cout << "      " << tlabel[tk] << " = " << t.name
              << " @ (" << tk.px << "," << tk.py << ")"
              << "  $" << t.cost
              << "  load=" << load << "/" << t.capacity << "\n";
  }
}

// ============================================================
//  Main
// ============================================================
int main()
{
  const int R = 10, C = 10;

  // Demand map — ~20 cells: enough to show differences, BT still finishes
  std::vector<std::vector<double>> demand = {
  //  0    1    2    3    4    5    6    7    8    9
    { 3,   0,   0,   0,   5,   0,   0,   0,   0,   4},   // 0
    { 0,   6,   0,   0,   0,   0,   8,   0,   0,   0},   // 1
    { 0,   0,   0,   0,   0,   4,   0,   6,   0,   0},   // 2
    { 0,   4,   0,   0,   0,   0,   0,   0,   0,   0},   // 3
    { 6,   0,   0,   0,   0,   0,   0,   5,   0,   7},   // 4
    { 0,   0,   0,   5,   0,   0,   0,   0,   6,   0},   // 5
    { 4,   0,   0,   0,   9,   0,   0,   0,   0,   5},   // 6
    { 0,   7,   0,   0,   0,   0,   0,   0,   0,   0},   // 7
    { 0,   0,   5,   0,   0,   7,   0,   8,   0,   0},   // 8
    { 0,   4,   0,   0,   0,   0,   6,   0,   0,   3},   // 9
  };

  // Buildings — two clusters blocking the middle
  std::vector<std::vector<bool>> buildings(R, std::vector<bool>(C, false));
  buildings[3][4] = buildings[3][5] = true;
  buildings[4][4] = buildings[4][5] = true;
  buildings[7][7] = buildings[7][8] = true;

  // Zero demand on buildings
  for (int j = 0; j < R; ++j)
    for (int i = 0; i < C; ++i)
      if (buildings[j][i]) demand[j][i] = 0;

  // Tower types — tight capacities force interesting trade-offs
  std::vector<TowerType> tower_types = {
    {"Macro",  5000.0,  30.0,  3.0},  // covers many cells but capacity fills fast
    {"Micro",  2500.0,  15.0,  2.0},  // mid range, mid cap
    {"Pico",   1000.0,   8.0,  1.2},  // short, cheap, very limited
  };

  // Fixed candidate tower positions (21 spots)
  std::vector<std::pair<int,int>> tower_positions = {
    {0,0}, {4,0}, {9,0},
    {1,1}, {6,1},
    {2,2}, {5,2}, {7,2},
    {1,3},
    {0,4}, {7,4}, {9,4},
    {3,5}, {8,5},
    {0,6}, {4,6}, {9,6},
    {1,7},
    {2,8}, {5,8}, {7,8},
    {1,9}, {6,9}, {9,9},
  };

  double budget = 40000.0;

  // ---- Print setup ----
  std::cout << "============================================================\n";
  std::cout << "  Tower Placement CSP — 10x10 Grid\n";
  std::cout << "  Comparing: Plain BT, +MRV, +FC, +AC3, +LCV, All Combined\n";
  std::cout << "============================================================\n\n";

  std::cout << "Grid (numbers = demand, # = building, T = tower site):\n";
  std::set<std::pair<int,int>> tpset(tower_positions.begin(), tower_positions.end());
  for (int j = 0; j < R; ++j)
  {
    std::cout << "  ";
    for (int i = 0; i < C; ++i)
    {
      if (buildings[j][i])
        std::cout << std::setw(4) << "#";
      else if (demand[j][i] > 0 && tpset.count({i,j}))
        std::cout << " " << (int)demand[j][i] << "T";
      else if (demand[j][i] > 0)
        std::cout << std::setw(4) << (int)demand[j][i];
      else if (tpset.count({i,j}))
        std::cout << "  T ";
      else
        std::cout << "  . ";
    }
    std::cout << "\n";
  }

  std::cout << "\nTower types:\n";
  for (const auto &t : tower_types)
    std::cout << "  " << std::setw(6) << t.name
              << ": radius=" << t.radius
              << "  capacity=" << t.capacity
              << "  cost=$" << t.cost << "\n";
  std::cout << "Budget: $" << budget << "\n\n";

  // ---- Build problem ----
  std::cout << "Building CSP...\n";
  CSPProblem prob = buildProblem(R, C, demand, buildings, tower_types, tower_positions);

  std::cout << "  Variables (demand cells): " << prob.demand_cells.size() << "\n";
  std::cout << "  Towers after pruning:     " << prob.all_towers.size() << "\n";

  // Print domain sizes
  std::cout << "  Domain sizes: ";
  for (int i = 0; i < (int)prob.demand_cells.size(); ++i)
    std::cout << "|D" << i << "|=" << prob.domains[i].size() << " ";
  std::cout << "\n\n";

  // ---- Solver configs ----
  std::vector<SolverConfig> configs = {
    {"Plain Backtracking",        false, false, false, false},
    {"MRV only",                  true,  false, false, false},
    {"FC only",                   false, false, true,  false},
    {"AC3 only",                  false, false, false, true },
    {"MRV + FC",                  true,  false, true,  false},
    {"MRV + LCV",                 true,  true,  false, false},
    {"MRV + FC + AC3",            true,  false, true,  true },
    {"All (MRV+LCV+FC+AC3)",     true,  true,  true,  true },
  };

  std::vector<SolverResult> results;

  for (const auto &cfg : configs)
  {
    std::cout << "Running: " << cfg.name << " ...";
    std::cout.flush();

    CSPSolver solver(prob, cfg);
    SolverResult res = solver.solve(budget);
    results.push_back(res);

    if (res.solved)
      std::cout << " SOLVED ($" << res.cost << ")"
                << "  nodes=" << res.nodes_explored
                << "  bt=" << res.backtracks
                << "  " << std::fixed << std::setprecision(4) << res.time_seconds << "s\n";
    else
      std::cout << " FAILED"
                << "  nodes=" << res.nodes_explored
                << "  bt=" << res.backtracks
                << "  " << std::fixed << std::setprecision(4) << res.time_seconds << "s\n";
  }

  // ---- Comparison table ----
  std::cout << "\n============================================================\n";
  std::cout << "  COMPARISON TABLE\n";
  std::cout << "============================================================\n\n";

  std::cout << std::left
            << std::setw(28) << "Configuration"
            << std::right
            << std::setw(8)  << "Solved"
            << std::setw(10) << "Cost($)"
            << std::setw(10) << "Nodes"
            << std::setw(12) << "Backtracks"
            << std::setw(12) << "Time(s)"
            << "\n" << std::string(80, '-') << "\n";

  for (int i = 0; i < (int)configs.size(); ++i)
  {
    std::cout << std::left << std::setw(28) << configs[i].name << std::right;
    std::cout << std::setw(8) << (results[i].solved ? "YES" : "NO");
    if (results[i].solved)
      std::cout << std::setw(10) << std::fixed << std::setprecision(0) << results[i].cost;
    else
      std::cout << std::setw(10) << "-";
    std::cout << std::setw(10) << results[i].nodes_explored
              << std::setw(12) << results[i].backtracks
              << std::setw(12) << std::fixed << std::setprecision(4) << results[i].time_seconds
              << "\n";
  }

  // ---- Speedup vs plain ----
  if (results[0].nodes_explored > 0)
  {
    std::cout << "\n  Speedup vs Plain Backtracking:\n";
    double bn = results[0].nodes_explored;
    double bt = results[0].time_seconds;
    for (int i = 1; i < (int)configs.size(); ++i)
    {
      double ns = (results[i].nodes_explored > 0) ? bn / results[i].nodes_explored : 0;
      double ts = (results[i].time_seconds > 1e-8) ? bt / results[i].time_seconds : 0;
      std::cout << "    " << std::left << std::setw(28) << configs[i].name
                << std::right
                << std::fixed << std::setprecision(1)
                << std::setw(8) << ns << "x fewer nodes"
                << std::setw(10) << ts << "x faster\n";
    }
  }

  // ---- Show best solution on grid ----
  int best_idx = -1;
  for (int i = 0; i < (int)results.size(); ++i)
    if (results[i].solved && (best_idx < 0 || results[i].cost < results[best_idx].cost))
      best_idx = i;

  if (best_idx >= 0)
  {
    std::cout << "\n============================================================\n";
    std::cout << "  Best solution from: " << configs[best_idx].name
              << "  (cost=$" << results[best_idx].cost << ")\n";
    std::cout << "============================================================\n";
    printSolution(results[best_idx], prob, R, C, demand, buildings);
  }

  return 0;
}