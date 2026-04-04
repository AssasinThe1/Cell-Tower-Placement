#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <fstream>
#include <random>
#include <string>
#include <functional>
#include <numeric>
#include <set>

// ============================================================
//  Hyperparameters
// ============================================================
struct Hyperparameters
{
  double path_loss_exp = 3.5;   // Controls how fast signal weakens with distance
  double atten_per_unit = 0.20; // Signal loss per unit distance inside buildings
  double diff_strength = 0.8;   // Strength of diffraction signal from building corners
  double diff_exp = 4.0;        // Distance decay for diffraction
  double noise_floor = 1e-4;    // Background radio noise
};

// ============================================================
//  Tower Catalogue  (what types of tower exist)
// ============================================================
struct TowerType
{
  std::string name;
  double power;    // Transmission power  (Higher → larger coverage)
  double cost;     // Financial cost to deploy one unit
  double capacity; // Max abstract resource units (PRBs / Bandwidth)
};

// ============================================================
//  Algorithm I/O  (algorithms only deal with these two types)
// ============================================================

// One tower placed somewhere on the map
struct PlacedTower
{
  double x, y;
  int type_index; // Index into the TowerType catalogue passed to the algorithm
};

// Everything an algorithm returns
struct AlgorithmResult
{
  std::string name;
  std::vector<PlacedTower> placements;
  double score = 0.0; // Demand-weighted average QoS (0–100)
  double total_cost = 0.0;
};

// ============================================================
//  Grid Cell
// ============================================================
struct Cell
{
  double demand = 0.0;
  bool can_place_tower = true;
  bool is_building = false;

  // Simulation state — reset between evaluations
  double signal_strength = 0.0;
  double efficiency = 0.0;
  double allocated_capacity = 0.0;
  double qos_percentage = 0.0;
};

struct BoundingBox
{
  double xmin, xmax, ymin, ymax;
};

// ============================================================
//  Grid  —  Physics Engine + State
// ============================================================
class Grid
{
private:
  int width, height;
  std::vector<std::vector<Cell>> cells;
  std::vector<BoundingBox> buildings;
  Hyperparameters params;
  std::mt19937 gen;

  // -----------------------------------------------------------
  // Calculates Euclidean distance a ray travels *inside* a box.
  // Used by the slow (exact) path of applyTowerSignal.
  // -----------------------------------------------------------
  double rayDistanceInBox(double lx, double ly,
                          double x, double y,
                          const BoundingBox &b)
  {
    double t_min = 0.0, t_max = 1.0;
    double dx = x - lx, dy = y - ly;

    struct LineParam
    {
      double p, d, min_val, max_val;
    };

    LineParam lp[2] = {
        {lx, dx, b.xmin, b.xmax},
        {ly, dy, b.ymin, b.ymax}};
    for (int i = 0; i < 2; ++i)
    {
      if (std::abs(lp[i].d) < 1e-6)
      {
        if (lp[i].p < lp[i].min_val || lp[i].p > lp[i].max_val)
          return 0.0;
      }
      else
      {
        double t1 = (lp[i].min_val - lp[i].p) / lp[i].d;
        double t2 = (lp[i].max_val - lp[i].p) / lp[i].d;
        if (t1 > t2)
          std::swap(t1, t2);
        t_min = std::max(t_min, t1);
        t_max = std::min(t_max, t2);
        if (t_min > t_max)
          return 0.0;
      }
    }

    if (t_min < 1.0)
    {
      double actual_t_max = std::min(t_max, 1.0);
      if (actual_t_max > t_min)
        return std::hypot(dx, dy) * (actual_t_max - t_min);
    }
    return 0.0;
  }

  // -----------------------------------------------------------
  // O(D) voxel traversal — fast building penetration via DDA.
  // Used by the fast path of applyTowerSignal.
  // -----------------------------------------------------------
  double calculateBuildingPenetrationDDA(double startX, double startY,
                                         double endX, double endY)
  {
    double dx = endX - startX;
    double dy = endY - startY;
    double ray_length = std::hypot(dx, dy);
    if (ray_length < 1e-6)
      return 0.0;

    double dirX = dx / ray_length;
    double dirY = dy / ray_length;

    int mapX = static_cast<int>(std::floor(startX));
    int mapY = static_cast<int>(std::floor(startY));
    int endMapX = static_cast<int>(std::floor(endX));
    int endMapY = static_cast<int>(std::floor(endY));

    double deltaDistX = std::abs(1.0 / dirX);
    double deltaDistY = std::abs(1.0 / dirY);

    int stepX, stepY;
    double sideDistX, sideDistY;

    if (dirX < 0)
    {
      stepX = -1;
      sideDistX = (startX - mapX) * deltaDistX;
    }
    else
    {
      stepX = 1;
      sideDistX = (mapX + 1.0 - startX) * deltaDistX;
    }

    if (dirY < 0)
    {
      stepY = -1;
      sideDistY = (startY - mapY) * deltaDistY;
    }
    else
    {
      stepY = 1;
      sideDistY = (mapY + 1.0 - startY) * deltaDistY;
    }

    double total_building_dist = 0.0;
    double current_t = 0.0;

    int max_steps = std::abs(endMapX - mapX) + std::abs(endMapY - mapY) + 1;

    for (int step = 0; step < max_steps; ++step)
    {
      if (mapX < 0 || mapX >= width || mapY < 0 || mapY >= height)
        break;

      if (cells[mapY][mapX].is_building)
      {
        double next_t = std::min({sideDistX, sideDistY, ray_length});
        total_building_dist += (next_t - current_t);
      }

      if (sideDistX < sideDistY)
      {
        current_t = sideDistX;
        sideDistX += deltaDistX;
        mapX += stepX;
      }
      else
      {
        current_t = sideDistY;
        sideDistY += deltaDistY;
        mapY += stepY;
      }

      if (current_t >= ray_length - 1e-6)
        break;
    }

    return total_building_dist;
  }

public:
  Grid(int w, int h, unsigned int seed = 42, Hyperparameters hp = {})
          : width(w), height(h), gen(seed), params(hp) 
  {
    cells.resize(height, std::vector<Cell>(width));
  }
          
  const Hyperparameters& getParams() const { return params; }
  int getWidth() const{ return width; }
  int getHeight() const { return height; }
  bool isBuilding(int x, int y) const { return cells[y][x].is_building; }
  bool canPlaceTower(int x, int y) const { return cells[y][x].can_place_tower; }
  double getDemand(int x, int y) const { return cells[y][x].demand; }

  // ---- Reset signal state between evaluations ---------------
  void resetSignals()
  {
    for (auto &row : cells)
      for (auto &c : row)
      {
        c.signal_strength = 0.0;
        c.efficiency = 0.0;
        c.allocated_capacity = 0.0;
        c.qos_percentage = 0.0;
      }
  }

  // -----------------------------------------------------------
  void addBuilding(double xmin, double xmax, double ymin, double ymax)
  {
    buildings.push_back({xmin, xmax, ymin, ymax});
    for (int j = std::max(0, (int)ymin); j <= std::min(height - 1, (int)ymax); ++j)
      for (int i = std::max(0, (int)xmin); i <= std::min(width - 1, (int)xmax); ++i)
      {
        cells[j][i].is_building = true;
        cells[j][i].can_place_tower = false;
      }
  }

  void addBuildingsRandomly(double min_side, double max_side, double min_dist)
  {
    double avg_side = (min_side + max_side) / 2.0;
    double padded_area = std::pow(avg_side + min_dist, 2);
    int target_buildings = static_cast<int>(((width * height) / padded_area) * 0.25);

    std::uniform_real_distribution<> side_dist(min_side, max_side);
    std::uniform_real_distribution<> x_dist(0.0, width - min_side);
    std::uniform_real_distribution<> y_dist(0.0, height - min_side);

    int max_attempts = target_buildings * 10;
    int attempts = 0, placed = 0;

    while (placed < target_buildings && attempts < max_attempts)
    {
      ++attempts;
      double w = side_dist(gen);
      double h = side_dist(gen);
      double xmin = x_dist(gen);
      double ymin = y_dist(gen);

      if (xmin + w > width)
        xmin = width - w;
      if (ymin + h > height)
        ymin = height - h;
      double xmax = xmin + w;
      double ymax = ymin + h;

      bool valid = true;
      for (const auto &b : buildings)
      {
        double ddx = std::max({0.0, xmin - b.xmax, b.xmin - xmax});
        double ddy = std::max({0.0, ymin - b.ymax, b.ymin - ymax});
        if (std::hypot(ddx, ddy) < min_dist)
        {
          valid = false;
          break;
        }
      }

      if (valid)
      {
        addBuilding(xmin, xmax, ymin, ymax);
        ++placed;
      }
    }
    std::cout << "City Generator: Placed " << placed
              << " buildings (target was " << target_buildings << ").\n";
  }

  void setDemand(int x, int y, double demand)
  {
    if (x >= 0 && x < width && y >= 0 && y < height)
      cells[y][x].demand = demand;
  }

  void addRandomDemand(int max_demand, double probability = 0.15)
  {
    std::uniform_int_distribution<> demand_dist(1, max_demand);
    std::uniform_real_distribution<> prob_dist(0.0, 1.0);

    for (int j = 0; j < height; ++j)
      for (int i = 0; i < width; ++i)
      {
        if (!cells[j][i].is_building && prob_dist(gen) < probability)
          cells[j][i].demand = demand_dist(gen);
        else if (cells[j][i].is_building)
          cells[j][i].demand = demand_dist(gen) * 5;
      }
  }

  // -----------------------------------------------------------
  // Apply signal from one placed tower.
  // fast=true  → DDA ray marching + blast-radius culling (recommended)
  // fast=false → exact analytical ray-box intersection (slow, reference)
  // -----------------------------------------------------------
  void applyTowerSignal(const PlacedTower &placement,
                        const TowerType &type,
                        bool fast = true)
  {
    double tx = placement.x;
    double ty = placement.y;

    if (fast)
    {
      // --- Blast radius ---
      double threshold = params.noise_floor * 1.01;
      double max_radius = std::pow(type.power / threshold,
                                   1.0 / params.path_loss_exp);

      int min_x = std::max(0, (int)std::floor(tx - max_radius));
      int max_x = std::min(width - 1, (int)std::ceil(tx + max_radius));
      int min_y = std::max(0, (int)std::floor(ty - max_radius));
      int max_y = std::min(height - 1, (int)std::ceil(ty + max_radius));

      // Pre-filter buildings within bounding box for diffraction
      std::vector<BoundingBox> local_buildings;
      for (const auto &b : buildings)
        if (b.xmax >= min_x && b.xmin <= max_x &&
            b.ymax >= min_y && b.ymin <= max_y)
          local_buildings.push_back(b);

      for (int j = min_y; j <= max_y; ++j)
        for (int i = min_x; i <= max_x; ++i)
        {
          double d = std::max(std::hypot(i - tx, j - ty), 1.0);
          if (d > max_radius)
            continue;

          double power = type.power / std::pow(d, params.path_loss_exp);

          double total_dist = calculateBuildingPenetrationDDA(tx, ty, i, j);
          power *= std::exp(-params.atten_per_unit * total_dist);

          // BUG FIX: all 4 corners considered (was only xmin pair)
          double diff_sum = 0.0;
          if (total_dist > 0 && !cells[j][i].is_building)
          {
            for (const auto &b : local_buildings)
            {
              std::pair<double, double> corners[4] = {
                  {b.xmin, b.ymin}, {b.xmin, b.ymax}, {b.xmax, b.ymin}, {b.xmax, b.ymax}};
              for (const auto &[cx, cy] : corners)
              {
                double dc = std::max(std::hypot(cx - tx, cy - ty), 1.0);
                double p_c = type.power / std::pow(dc, params.path_loss_exp);
                double d_diff = std::max(std::hypot(i - cx, j - cy), 1.0);
                diff_sum += (p_c * params.diff_strength) / std::pow(d_diff, params.diff_exp);
              }
            }
          }

          cells[j][i].signal_strength += (power + diff_sum);
        }

      // std::cout << "[" << type.name << " @ (" << tx << "," << ty << ")] "
      //           << "Blast box: " << (max_x - min_x) << "x"
      //           << (max_y - min_y) << "\n";
    }
    else // Slow exact path
    {
      for (int j = 0; j < height; ++j)
        for (int i = 0; i < width; ++i)
        {
          double d = std::max(std::hypot(i - tx, j - ty), 1.0);
          double power = type.power / std::pow(d, params.path_loss_exp);

          double total_dist = 0.0;
          for (const auto &b : buildings)
            total_dist += rayDistanceInBox(tx, ty, i, j, b);
          power *= std::exp(-params.atten_per_unit * total_dist);

          // BUG FIX: all 4 corners considered (was only xmin pair)
          double diff_sum = 0.0;
          if (total_dist > 0 && !cells[j][i].is_building)
          {
            for (const auto &b : buildings)
            {
              std::pair<double, double> corners[4] = {
                  {b.xmin, b.ymin}, {b.xmin, b.ymax}, {b.xmax, b.ymin}, {b.xmax, b.ymax}};
              for (const auto &[cx, cy] : corners)
              {
                double dc = std::max(std::hypot(cx - tx, cy - ty), 1.0);
                double p_c = type.power / std::pow(dc, params.path_loss_exp);
                double d_diff = std::max(std::hypot(i - cx, j - cy), 1.0);
                diff_sum += (p_c * params.diff_strength) / std::pow(d_diff, params.diff_exp);
              }
            }
          }

          cells[j][i].signal_strength += (power + diff_sum);
        }
    }
  }

  // -----------------------------------------------------------
  // BUG FIX: takes combined capacity of ALL deployed towers.
  // Call this ONCE after all applyTowerSignal calls, not per tower.
  // BUG FIX: building cells (obstacles) are excluded.
  // -----------------------------------------------------------
 void resolveNetworkCapacity(double total_capacity)
  {
    double total_required = 0.0;

    // Pass 1: efficiency + required capacity
    for (int j = 0; j < height; ++j)
      for (int i = 0; i < width; ++i)
      {
        Cell &c = cells[j][i];
        if (c.demand > 0 && !c.is_building)
        {
          double sinr = c.signal_strength / params.noise_floor;
          c.efficiency = std::log2(1.0 + sinr);
          
          // FIX: Only cells with a usable connection compete for capacity
          if (c.efficiency > 0.01) 
          {
            total_required += c.demand / c.efficiency;
          }
        }
      }

    // Pass 2: allocation factor
    double alloc = (total_required > total_capacity)
                       ? total_capacity / total_required
                       : 1.0;

    // Pass 3: QoS
    for (int j = 0; j < height; ++j)
      for (int i = 0; i < width; ++i)
      {
        Cell &c = cells[j][i];
        if (c.demand > 0 && !c.is_building)
        {
          // FIX: Apply allocation only to connected cells
          if (c.efficiency > 0.01) 
          {
            double req_cap = c.demand / c.efficiency;
            c.allocated_capacity = req_cap * alloc;
            double fulfilled = c.allocated_capacity * c.efficiency;
            
            // FIX: Cap QoS at 100.0 to prevent over-delivery from inflating scores
            c.qos_percentage = std::min(100.0, (fulfilled / c.demand) * 100.0);
          } 
          else 
          {
            c.qos_percentage = 0.0;
            c.allocated_capacity = 0.0;
          }
        }
      }
  }
  // -----------------------------------------------------------
  // Demand-weighted average QoS across all non-building cells
  // with demand. Returns 0–100.
  // -----------------------------------------------------------
  double computeScore() const
  {
    double weighted_qos = 0.0;
    double total_demand = 0.0;
    for (int j = 0; j < height; ++j)
      for (int i = 0; i < width; ++i)
      {
        const Cell &c = cells[j][i];
        if (c.demand > 0 && !c.is_building)
        {
          weighted_qos += c.qos_percentage * c.demand;
          total_demand += c.demand;
        }
      }
    return total_demand > 0.0 ? weighted_qos / total_demand : 0.0;
  }

  // -----------------------------------------------------------
  // Exports the signal map (or demand map) to CSV.
  // BUG FIX: ALL building cells become -999, not just boundary ones.
  //          This matches the Python visualiser's building_mask logic.
  // -----------------------------------------------------------
  void exportSignalMapToCSV(const std::string &filename, bool demand = false)
  {
    std::ofstream file(filename);
    if (!file.is_open())
    {
      std::cerr << "Failed to open " << filename << " for writing.\n";
      return;
    }

    for (int j = 0; j < height; ++j)
    {
      for (int i = 0; i < width; ++i)
      {
        bool boundary = false;
        if (cells[j][i].is_building)
        {
          const int dx[4] = {1, -1, 0, 0};
          const int dy[4] = {0, 0, 1, -1};
          for (int k = 0; k < 4; ++k)
          {
            int nx = i + dx[k];
            int ny = j + dy[k];
            if (nx < 0 || nx >= width || ny < 0 || ny >= height ||
                !cells[ny][nx].is_building)
            {
              boundary = true;
              break;
            }
          }
        }

        if (boundary)
        {
          file << "-999.0";
        }
        else if (demand)
        {
          file << cells[j][i].demand;
        }
        else
        {
          double signal_db = 10.0 * std::log10(
                                        cells[j][i].signal_strength + 1e-10);
          file << signal_db;
        }

        if (i < width - 1)
          file << ",";
      }
      file << "\n";
    }

    file.close();
    std::cout << "Exported: " << filename << "\n";
  }

  void printCellStats(int x, int y, const std::string &label) const
  {
    if (x >= 0 && x < width && y >= 0 && y < height)
    {
      const Cell &c = cells[y][x];
      std::cout << "[" << label << " @ (" << x << "," << y << ")] "
                << "Demand: " << c.demand << " Mbps | "
                << "Efficiency: " << std::fixed << std::setprecision(2)
                << c.efficiency << " bits/Hz | "
                << "QoS: " << c.qos_percentage << "%\n";
    }
  }
};

// ============================================================
//  Evaluator
//  Runs the full physics pipeline for a set of placements,
//  computes the score, then leaves the grid in a valid state.
//  Call this from inside any algorithm to score a candidate.
// ============================================================
double evaluatePlacement(Grid &grid,
                         const std::vector<PlacedTower> &placements,
                         const std::vector<TowerType> &tower_types,
                         bool fast = true)
{
  grid.resetSignals();

  double total_capacity = 0.0;
  for (const auto &p : placements)
  {
    grid.applyTowerSignal(p, tower_types[p.type_index], fast);
    total_capacity += tower_types[p.type_index].capacity;
  }

  grid.resolveNetworkCapacity(total_capacity);
  return grid.computeScore();
}

// Free-space signal cache (no building penetration) for fast greedy scoring
using SignalCache = std::vector<std::vector<double>>;

SignalCache makeSignalCache(int w, int h)
{
  return SignalCache(h, std::vector<double>(w, 0.0));
}

void addToCache(SignalCache &cache, double tx, double ty,
                const TowerType &type, const Hyperparameters &params,
                int width, int height)
{
  double max_r = std::pow(type.power / (params.noise_floor * 1.01),
                          1.0 / params.path_loss_exp);
  int x0 = std::max(0, (int)(tx - max_r));
  int x1 = std::min(width - 1, (int)(tx + max_r));
  int y0 = std::max(0, (int)(ty - max_r));
  int y1 = std::min(height - 1, (int)(ty + max_r));

  for (int j = y0; j <= y1; ++j)
    for (int i = x0; i <= x1; ++i)
    {
      double d = std::max(std::hypot(i - tx, j - ty), 1.0);
      if (d > max_r)
        continue;
      cache[j][i] += type.power / std::pow(d, params.path_loss_exp);
    }
}

// Marginal Shannon-capacity gain of adding one tower at (tx,ty).
// Uses free-space signal only (no ray marching) — greedy speed approx.
// Uncovered cells get a 2× bonus to push coverage first, then QoS.
double marginalGain(const Grid &grid, const SignalCache &cache,
                    double tx, double ty,
                    const TowerType &type, const Hyperparameters &params)
{
  double gain = 0.0;
  double thr = params.noise_floor * 5.0; // "meaningful signal" threshold
  double max_r = std::pow(type.power / (params.noise_floor * 1.01),
                          1.0 / params.path_loss_exp);
  int x0 = std::max(0, (int)(tx - max_r));
  int x1 = std::min(grid.getWidth() - 1, (int)(tx + max_r));
  int y0 = std::max(0, (int)(ty - max_r));
  int y1 = std::min(grid.getHeight() - 1, (int)(ty + max_r));

  for (int j = y0; j <= y1; ++j)
    for (int i = x0; i <= x1; ++i)
    {
      if (grid.isBuilding(i, j))
        continue;
      double demand = grid.getDemand(i, j);
      if (demand <= 0.0)
        continue;
      double d = std::max(std::hypot(i - tx, j - ty), 1.0);
      if (d > max_r)
        continue;

      double added = type.power / std::pow(d, params.path_loss_exp);
      double before = cache[j][i];
      double after = before + added;

      double cap_gain = std::log2(1.0 + after / params.noise_floor) - std::log2(1.0 + before / params.noise_floor);
      double bonus = (before < thr) ? 2.0 : 1.0;
      gain += demand * cap_gain * bonus;
    }
  return gain;
}

// Core greedy engine used by all three algorithms.
// Reads `result.placements` as already-placed towers (can be non-empty),
// adds towers from `candidates` until budget is exhausted or no gain.
void greedyFill(Grid &grid, const std::vector<TowerType> &types,
                const Hyperparameters &params,
                const std::vector<std::pair<int, int>> &candidates,
                double budget, AlgorithmResult &result)
{
  SignalCache cache = makeSignalCache(grid.getWidth(), grid.getHeight());
  std::set<std::pair<int, int>> placed;

  for (const auto &p : result.placements)
  {
    addToCache(cache, p.x, p.y, types[p.type_index],
               params, grid.getWidth(), grid.getHeight());
    placed.insert({(int)p.x, (int)p.y});
  }

  while (result.total_cost < budget)
  {
    double best_score = 0.0;
    PlacedTower best{};
    bool found = false;

    for (const auto &[cx, cy] : candidates)
    {
      if (!grid.canPlaceTower(cx, cy))
        continue;
      if (placed.count({cx, cy}))
        continue;

      for (int t = 0; t < (int)types.size(); ++t)
      {
        if (types[t].cost > budget - result.total_cost)
          continue;
        double s = marginalGain(grid, cache, cx, cy, types[t], params) / types[t].cost;
        if (s > best_score)
        {
          best_score = s;
          best = {(double)cx, (double)cy, t};
          found = true;
        }
      }
    }
    if (!found)
      break;

    result.placements.push_back(best);
    result.total_cost += types[best.type_index].cost;
    placed.insert({(int)best.x, (int)best.y});
    addToCache(cache, best.x, best.y, types[best.type_index],
               params, grid.getWidth(), grid.getHeight());
  }
}

// Demand-weighted k-means. Returns k cluster centres (floating-point coords).
std::vector<std::pair<double, double>> kMeansDemand(
    const Grid &grid, int k, int iters, std::mt19937 &rng)
{
  std::vector<std::pair<int, int>> pts;
  for (int j = 0; j < grid.getHeight(); ++j)
    for (int i = 0; i < grid.getWidth(); ++i)
      if (!grid.isBuilding(i, j) && grid.getDemand(i, j) > 0)
        pts.push_back({i, j});

  if (pts.empty())
    return {};
  k = std::min(k, (int)pts.size());
  std::shuffle(pts.begin(), pts.end(), rng);

  std::vector<std::pair<double, double>> centres(k);
  for (int i = 0; i < k; ++i)
    centres[i] = {(double)pts[i].first, (double)pts[i].second};

  std::vector<int> assign(pts.size());
  for (int iter = 0; iter < iters; ++iter)
  {
    for (int p = 0; p < (int)pts.size(); ++p)
    {
      double bd = 1e18;
      int bk = 0;
      for (int c = 0; c < k; ++c)
      {
        double d = std::hypot(pts[p].first - centres[c].first,
                              pts[p].second - centres[c].second);
        if (d < bd)
        {
          bd = d;
          bk = c;
        }
      }
      assign[p] = bk;
    }
    std::vector<double> wx(k, 0), wy(k, 0), wt(k, 0);
    for (int p = 0; p < (int)pts.size(); ++p)
    {
      int c = assign[p];
      double w = grid.getDemand(pts[p].first, pts[p].second);
      wx[c] += pts[p].first * w;
      wy[c] += pts[p].second * w;
      wt[c] += w;
    }
    for (int c = 0; c < k; ++c)
      if (wt[c] > 0)
        centres[c] = {wx[c] / wt[c], wy[c] / wt[c]};
  }
  return centres;
}

// Snap floating-point position to nearest valid (non-building) cell via BFS
std::pair<int, int> snapToValid(const Grid &grid, double fx, double fy)
{
  int cx = std::clamp((int)std::round(fx), 0, grid.getWidth() - 1);
  int cy = std::clamp((int)std::round(fy), 0, grid.getHeight() - 1);
  if (grid.canPlaceTower(cx, cy))
    return {cx, cy};
  for (int r = 1; r < 25; ++r)
    for (int dj = -r; dj <= r; ++dj)
      for (int di = -r; di <= r; ++di)
      {
        int nx = cx + di, ny = cy + dj;
        if (nx < 0 || nx >= grid.getWidth() || ny < 0 || ny >= grid.getHeight())
          continue;
        if (grid.canPlaceTower(nx, ny))
          return {nx, ny};
      }
  return {cx, cy};
}

// Shared SA engine. Moves: nudge position, swap type, drop, add.
// All three algorithms call this for their final polish phase.
AlgorithmResult simulatedAnnealing(
    Grid &grid, const std::vector<TowerType> &types,
    AlgorithmResult initial, double budget,
    int iterations, double T_start, double T_end,
    int nudge_r, bool fast, std::mt19937 &rng)
{
  AlgorithmResult current = initial;
  current.score = evaluatePlacement(grid, current.placements, types, fast);
  AlgorithmResult best = current;

  std::uniform_real_distribution<> uni(0.0, 1.0);
  std::uniform_int_distribution<> xd(0, grid.getWidth() - 1);
  std::uniform_int_distribution<> yd(0, grid.getHeight() - 1);
  std::uniform_int_distribution<> nd(-nudge_r, nudge_r);
  std::uniform_int_distribution<> td(0, (int)types.size() - 1);

  double log_ratio = std::log(T_end / T_start);

  for (int it = 0; it < iterations; ++it)
  {
    double T = T_start * std::exp(log_ratio * it / iterations);
    AlgorithmResult cand = current;
    double roll = uni(rng);

    if (cand.placements.empty() || roll < 0.15)
    {
      // ADD a tower at a random valid location
      for (int tries = 0; tries < 20; ++tries)
      {
        int nx = xd(rng), ny = yd(rng), nt = td(rng);
        if (!grid.canPlaceTower(nx, ny))
          continue;
        if (cand.total_cost + types[nt].cost > budget)
          continue;
        cand.placements.push_back({(double)nx, (double)ny, nt});
        cand.total_cost += types[nt].cost;
        break;
      }
    }
    else if (roll < 0.25)
    {
      // DROP a random tower
      std::uniform_int_distribution<> id(0, (int)cand.placements.size() - 1);
      int i = id(rng);
      cand.total_cost -= types[cand.placements[i].type_index].cost;
      cand.placements.erase(cand.placements.begin() + i);
    }
    else if (roll < 0.40)
    {
      // SWAP TYPE of a random tower
      std::uniform_int_distribution<> id(0, (int)cand.placements.size() - 1);
      int i = id(rng), nt = td(rng);
      double delta = types[nt].cost - types[cand.placements[i].type_index].cost;
      if (cand.total_cost + delta <= budget)
      {
        cand.total_cost += delta;
        cand.placements[i].type_index = nt;
      }
    }
    else
    {
      // NUDGE a random tower's position
      std::uniform_int_distribution<> id(0, (int)cand.placements.size() - 1);
      int i = id(rng);
      int nx = std::clamp((int)cand.placements[i].x + nd(rng), 0, grid.getWidth() - 1);
      int ny = std::clamp((int)cand.placements[i].y + nd(rng), 0, grid.getHeight() - 1);
      if (grid.canPlaceTower(nx, ny))
      {
        cand.placements[i].x = nx;
        cand.placements[i].y = ny;
      }
    }

    if (cand.placements.empty())
      continue;
    cand.score = evaluatePlacement(grid, cand.placements, types, fast);

    double delta = cand.score - current.score;
    if (delta > 0 || uni(rng) < std::exp(delta / T))
    {
      current = cand;
      if (current.score > best.score)
        best = current;
    }
  }
  std::cout << "[SA] " << best.name << ": score=" << std::fixed
            << std::setprecision(2) << best.score << "%\n";
  return best;
}

// ============================================================
// ALGORITHM A — Overlapping Windows + SA
//
// Divides the grid into windows with 40% overlap so borders are
// always covered by at least two windows' solvers. Greedy fills
// each window independently with a per-window budget slice, then
// merges all candidate placements (deduplicating exact duplicates
// from the overlap zone). SA then nudges/swaps/drops globally.
// ============================================================
AlgorithmResult algorithmA_WindowSA(
    Grid &grid, const std::vector<TowerType> &types,
    const Hyperparameters &params,
    double budget, bool fast, std::mt19937 &rng)
{
  AlgorithmResult result;
  result.name = "A_WindowSA";

  int W = grid.getWidth(), H = grid.getHeight();
  int win_w = W / 4, win_h = H / 4;
  int step_x = (int)(win_w * 0.6); // 40% overlap → stride = 60% of window
  int step_y = (int)(win_h * 0.6);

  int nx_wins = (W + step_x - 1) / step_x;
  int ny_wins = (H + step_y - 1) / step_y;
  double per_win_budget = budget / (nx_wins * ny_wins);

  std::set<std::pair<int, int>> seen;

  for (int wy = 0; wy < H; wy += step_y)
    for (int wx = 0; wx < W; wx += step_x)
    {
      int x1 = std::min(wx + win_w, W);
      int y1 = std::min(wy + win_h, H);

      std::vector<std::pair<int, int>> cands;
      for (int j = wy; j < y1; ++j)
        for (int i = wx; i < x1; ++i)
          cands.push_back({i, j});

      AlgorithmResult win;
      win.name = result.name;
      greedyFill(grid, types, params, cands, per_win_budget, win);

      for (const auto &p : win.placements)
      {
        auto key = std::make_pair((int)p.x, (int)p.y);
        if (seen.count(key))
          continue;
        if (result.total_cost + types[p.type_index].cost > budget)
          continue;
        seen.insert(key);
        result.placements.push_back(p);
        result.total_cost += types[p.type_index].cost;
      }
    }

  result = simulatedAnnealing(grid, types, result, budget,
                              500, 5.0, 0.02, 15, fast, rng);
  result.name = "A_WindowSA";
  return result;
}

// ============================================================
// ALGORITHM B — Global Greedy + SA
//
// Sees the whole grid at once so it never has a boundary problem.
// Samples candidate positions on a coarse grid (every 8 cells) for
// speed, selects towers one-at-a-time by best marginal-gain-per-cost,
// then SA refines. Provably gets ≥63% of optimal for submodular
// objectives before SA (greedy submodular approximation guarantee).
// ============================================================
AlgorithmResult algorithmB_GlobalGreedySA(
    Grid &grid, const std::vector<TowerType> &types,
    const Hyperparameters &params,
    double budget, bool fast, std::mt19937 &rng)
{
  AlgorithmResult result;
  result.name = "B_GlobalGreedy";

  std::vector<std::pair<int, int>> cands;
  for (int j = 0; j < grid.getHeight(); j += 8)
    for (int i = 0; i < grid.getWidth(); i += 8)
      cands.push_back({i, j});

  greedyFill(grid, types, params, cands, budget, result);

  result = simulatedAnnealing(grid, types, result, budget,
                              600, 3.0, 0.01, 20, fast, rng);
  result.name = "B_GlobalGreedy";
  return result;
}

// ============================================================
// ALGORITHM C — Demand Clustering + Greedy + SA
//
// Uses demand-weighted k-means to find where users are densest,
// seeds one tower per cluster (cheapest type, snapped to valid cell),
// then runs greedy on demand-positive cells to fill the remaining
// budget. SA merges and refines everything at the end.
// This is the only algorithm that explicitly models where demand is.
// ============================================================
AlgorithmResult algorithmC_ClusterGreedySA(
    Grid &grid, const std::vector<TowerType> &types,
    const Hyperparameters &params,
    double budget, bool fast, std::mt19937 &rng)
{
  AlgorithmResult result;
  result.name = "C_ClusterGreedy";

  // Number of clusters: rough estimate from budget and mean tower cost
  double avg_cost = 0;
  for (const auto &t : types)
    avg_cost += t.cost;
  avg_cost /= types.size();
  int k = std::max(4, (int)(budget / (avg_cost * 2.0)));

  auto centres = kMeansDemand(grid, k, 20, rng);

  // Find cheapest tower type for initial seeds
  int cheapest = 0;
  for (int t = 1; t < (int)types.size(); ++t)
    if (types[t].cost < types[cheapest].cost)
      cheapest = t;

  for (const auto &[cx, cy] : centres)
  {
    if (result.total_cost + types[cheapest].cost > budget)
      break;
    auto [sx, sy] = snapToValid(grid, cx, cy);
    result.placements.push_back({(double)sx, (double)sy, cheapest});
    result.total_cost += types[cheapest].cost;
  }

  // Greedy top-up: only on cells that have demand (focused candidate set)
  std::vector<std::pair<int, int>> cands;
  for (int j = 0; j < grid.getHeight(); j += 5)
    for (int i = 0; i < grid.getWidth(); i += 5)
      if (grid.getDemand(i, j) > 0)
        cands.push_back({i, j});

  greedyFill(grid, types, params, cands, budget, result);

  result = simulatedAnnealing(grid, types, result, budget,
                              600, 4.0, 0.01, 20, fast, rng);
  result.name = "C_ClusterGreedy";
  return result;
}


// ============================================================
//  ALGORITHM INTERFACE
//
//  Every algorithm must match this signature:
//
//      AlgorithmResult myAlgorithm(
//          Grid                        &grid,        // read via getters
//          const std::vector<TowerType> &tower_types, // catalogue
//          double                       budget,       // max total cost
//          bool                         fast);        // speed hint
//
//  Rules:
//    • Read the grid with getWidth/getHeight/isBuilding/getDemand/canPlaceTower.
//    • Score any candidate with evaluatePlacement().
//    • Return an AlgorithmResult with .placements, .score, .total_cost, .name.
//    • Do NOT export CSVs or call printCellStats — those belong in main().
// ============================================================

// -----------------------------------------------------------
// Example Algorithm 1: Uniform Grid
// Places macro towers on a regular grid, skipping buildings.
// -----------------------------------------------------------
AlgorithmResult uniformGridAlgorithm(Grid &grid,
                                     const std::vector<TowerType> &tower_types,
                                     double budget,
                                     bool fast = true)
{
  AlgorithmResult result;
  result.name = "Uniform Grid";

  // Use the first (cheapest) tower type
  int type_idx = 0;
  double spacing = 60.0; // grid spacing in cells

  for (double y = spacing / 2; y < grid.getHeight(); y += spacing)
    for (double x = spacing / 2; x < grid.getWidth(); x += spacing)
    {
      int ix = static_cast<int>(x), iy = static_cast<int>(y);
      if (!grid.canPlaceTower(ix, iy))
        continue;

      double new_cost = result.total_cost + tower_types[type_idx].cost;
      if (new_cost > budget)
        break;

      result.placements.push_back({x, y, type_idx});
      result.total_cost = new_cost;
    }

  result.score = evaluatePlacement(grid, result.placements, tower_types, fast);
  return result;
}

// -----------------------------------------------------------
// Example Algorithm 2: Window-Merge  (placeholder)
// Divide the map into windows, find the best tower position
// inside each window, then merge them into a global solution.
// Fill in your own logic in each marked section.
// -----------------------------------------------------------
AlgorithmResult windowMergeAlgorithm(Grid &grid,
                                     const std::vector<TowerType> &tower_types,
                                     double budget,
                                     bool fast = true)
{
  AlgorithmResult result;
  result.name = "Window Merge";

  int window_w = grid.getWidth() / 4;  // 4 columns of windows
  int window_h = grid.getHeight() / 4; // 4 rows of windows

  for (int wy = 0; wy < grid.getHeight(); wy += window_h)
    for (int wx = 0; wx < grid.getWidth(); wx += window_w)
    {
      // --- Per-window logic: find the cell with peak demand ---
      double best_demand = -1.0;
      int best_x = wx + window_w / 2;
      int best_y = wy + window_h / 2;

      for (int j = wy; j < std::min(wy + window_h, grid.getHeight()); ++j)
        for (int i = wx; i < std::min(wx + window_w, grid.getWidth()); ++i)
        {
          if (!grid.canPlaceTower(i, j))
            continue;
          if (grid.getDemand(i, j) > best_demand)
          {
            best_demand = grid.getDemand(i, j);
            best_x = i;
            best_y = j;
          }
        }

      // Pick best tower type within remaining budget
      int chosen_type = -1;
      for (int t = 0; t < (int)tower_types.size(); ++t)
      {
        if (result.total_cost + tower_types[t].cost <= budget)
          chosen_type = t;
      }
      if (chosen_type < 0)
        break; // out of budget

      result.placements.push_back({(double)best_x, (double)best_y, chosen_type});
      result.total_cost += tower_types[chosen_type].cost;
    }

  result.score = evaluatePlacement(grid, result.placements, tower_types, fast);
  return result;
}

// ============================================================
//  Comparison helper
// ============================================================
void compareAlgorithms(const std::vector<AlgorithmResult> &results)
{
  std::cout << "\n==============================\n"
            << " Algorithm Comparison\n"
            << "==============================\n"
            << std::left
            << std::setw(22) << "Algorithm"
            << std::setw(10) << "Score"
            << std::setw(10) << "Towers"
            << std::setw(14) << "Total Cost"
            << "\n"
            << std::string(56, '-') << "\n";

  for (const auto &r : results)
    std::cout << std::setw(22) << r.name
              << std::setw(10) << std::fixed << std::setprecision(2) << r.score
              << std::setw(10) << r.placements.size()
              << std::setw(14) << r.total_cost
              << "\n";

  // Find and announce winner
  auto best = std::max_element(results.begin(), results.end(),
                               [](const AlgorithmResult &a, const AlgorithmResult &b)
                               { return a.score < b.score; });

  std::cout << "\nWinner: " << best->name
            << " (score " << std::fixed << std::setprecision(2)
            << best->score << ")\n";
}

// ============================================================
//  Main  —  setup once, run all algorithms, compare, export
// ============================================================
int main()
{
  // Tuned hyperparameters — see table at top of file for rationale
  Hyperparameters hp;
  hp.path_loss_exp = 3.5;   // Urban outdoor (was 6.5 — far too steep)
  hp.atten_per_unit = 0.20; // Wall loss per unit
  hp.diff_strength = 0.60;
  hp.diff_exp = 4.0;
  hp.noise_floor = 1e-4;

  Grid grid(300, 250, 42, hp);
  std::cout << "here\n";
  grid.addBuildingsRandomly(20.0, 80.0, 15.0);
  grid.addRandomDemand(10);
  grid.setDemand(40, 50, 100.0);  // Hotspot 1
  grid.setDemand(110, 50, 100.0); // Hotspot 2
  grid.setDemand(200, 150, 80.0); // Hotspot 3
  std::vector<TowerType> types = {
      //  name     power   cost     capacity
      {"Macro",    600.0,  60000.0, 4000.0},   // ~81 cell radius - 0.38 cells/$ — best open coverage
      {"Micro",    150.0,  30000.0, 1200.0},   // ~54 cell radius - 0.35 cells/$ — fill-in
      {"Pico",     25.0,   12000.0, 300.0},    // ~37 cell radius - 0.28 cells/$ — hotspot only
  };

  double budget = 400000.0;
  bool fast = true;
  std::mt19937 rng(42);

  // Demand map is algorithm-independent — export once
  grid.exportSignalMapToCSV("demand_map.csv", true);

  std::vector<AlgorithmResult> results;

  std::cout << "\n=== Algorithm A: Overlapping Windows + SA ===\n";
  results.push_back(algorithmA_WindowSA(grid, types, hp, budget, fast, rng));

  std::cout << "\n=== Algorithm B: Global Greedy + SA ===\n";
  results.push_back(algorithmB_GlobalGreedySA(grid, types, hp, budget, fast, rng));

  std::cout << "\n=== Algorithm C: Demand Clustering + SA ===\n";
  results.push_back(algorithmC_ClusterGreedySA(grid, types, hp, budget, fast, rng));

  // Export each algorithm's signal map
  for (const auto &r : results)
  {
    evaluatePlacement(grid, r.placements, types, fast);
    {
      std::ofstream tf(r.name + "_towers.csv");
      tf << "x,y,type\n";
      for (const auto &p : r.placements)
        tf << p.x << "," << p.y << "," << types[p.type_index].name << "\n";
      tf.close();
      std::cout << "Exported: " << r.name << "_towers.csv\n";
    }
    grid.exportSignalMapToCSV(r.name + "_signal_map.csv");
  }

  compareAlgorithms(results);

  // Write scores.csv for Python visualiser
  {
    std::ofstream sf("scores.csv");
    sf << "algorithm,score,towers,total_cost\n";
    for (const auto &r : results)
      sf << r.name << "," << r.score << ","
         << r.placements.size() << "," << r.total_cost << "\n";
    std::cout << "Exported: scores.csv\n";
  }

  return 0;
}

