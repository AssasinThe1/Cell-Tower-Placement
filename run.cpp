// ============================================================
//  Tower Placement — CSP + Heuristic Comparison
//
//  This program solves the problem of optimally placing cellular
//  network towers on a 2D grid to maximize coverage quality (QoS)
//  subject to a monetary budget constraint.
//
//  Two approaches are compared:
//
//  Part 1: Small grid (default 10x10) — CSP exact solvers vs SA heuristics.
//          Demonstrates that the exact CSP finds the same or better solution,
//          but only scales to small grids due to exponential search complexity.
//
//  Part 2: Large grid (300x250) — SA heuristics only.
//          CSP cannot handle this size; SA algorithms scale gracefully.
//
//  Physics model used for signal computation:
//    - Path loss: power drops as distance^(-path_loss_exp)
//    - Building attenuation: exponential decay per unit of building thickness
//      traversed along the line-of-sight (computed via DDA ray-marching)
//    - Diffraction: signal bends around building corners (simplified model)
//    - Shannon capacity: spectral efficiency = log2(1 + SINR)
//    - QoS: demand-weighted average of how much capacity each cell received
// ============================================================

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
#include <map>
#include <deque>
#include <chrono>
#include <sstream>

// ============================================================
//  Hyperparameters
//  These control the physical propagation model.
// ============================================================
struct Hyperparameters
{
  // Exponent in the path loss formula: received_power = tx_power / distance^path_loss_exp
  // Typical urban values range from 3.0 (open) to 4.5 (dense urban).
  double path_loss_exp   = 3.5;

  // How much signal is attenuated per unit of building material traversed.
  // Applied as: power *= exp(-atten_per_unit * building_thickness)
  double atten_per_unit  = 0.20;

  // Strength multiplier for the diffraction component (signal bending around corners).
  double diff_strength   = 0.8;

  // How quickly diffracted signal fades with distance from the corner.
  double diff_exp        = 4.0;

  // Minimum detectable signal (noise floor). Signals below this are treated as zero.
  double noise_floor     = 1e-4;
};

// ============================================================
//  TowerType
//  Defines the hardware specification of a tower variant.
// ============================================================
struct TowerType
{
  std::string name;    // Human-readable label (e.g. "Macro", "Micro", "Pico")
  double power;        // Transmit power (arbitrary units matching the path loss model)
  double cost;         // Monetary cost to place one tower of this type
  double capacity;     // Maximum total data capacity this tower can serve
};

// ============================================================
//  PlacedTower
//  Records the location and type of a single tower in a solution.
// ============================================================
struct PlacedTower
{
  double x, y;      // Grid coordinates (column, row) of the tower
  int type_index;   // Index into the TowerType vector
};

// ============================================================
//  AlgorithmResult
//  Stores the complete output of one algorithm run.
// ============================================================
struct AlgorithmResult
{
  std::string name;                      // Algorithm identifier
  std::vector<PlacedTower> placements;   // The set of towers placed
  double score      = 0.0;              // Final QoS score (demand-weighted average %)
  double total_cost = 0.0;              // Total money spent on towers
  int    nodes      = 0;                // CSP: number of search nodes explored (0 for SA)
  double time_sec   = 0.0;             // Wall-clock time the algorithm took
};

// ============================================================
//  Cell
//  Represents one grid square. Each cell can hold demand (users
//  who need connectivity), be a building (blocks towers), and
//  accumulate signal from nearby towers during evaluation.
// ============================================================
struct Cell
{
  double demand = 0.0;           // How much bandwidth this cell demands (0 = unoccupied)
  bool can_place_tower = true;   // False if this cell is a building or otherwise blocked
  bool is_building     = false;  // True if this cell is a building (blocks signal)
  double signal_strength    = 0.0;  // Accumulated received signal from all towers
  double efficiency         = 0.0;  // Shannon spectral efficiency = log2(1 + SINR)
  double allocated_capacity = 0.0;  // How much capacity was allocated to this cell
  double qos_percentage     = 0.0;  // Percentage of demand actually served (0–100)
};

// ============================================================
//  BoundingBox
//  Axis-aligned rectangle, used to represent buildings.
// ============================================================
struct BoundingBox { double xmin, xmax, ymin, ymax; };


// ============================================================
//  Grid — Physics Engine
//
//  The central data structure. Manages a 2D array of Cells plus
//  a list of buildings, and provides all physics computations:
//  ray-marching for building penetration, signal propagation,
//  capacity allocation, and QoS scoring.
// ============================================================
class Grid
{
  int width, height;                         // Grid dimensions in cells
  std::vector<std::vector<Cell>> cells;      // 2D cell array (indexed [row][col])
  std::vector<BoundingBox> buildings;        // List of placed buildings (for diffraction)
  Hyperparameters params;                    // Physics constants
  std::mt19937 gen;                          // RNG for random city/demand generation

  // ----------------------------------------------------------
  //  calculateBuildingPenetrationDDA
  //
  //  Uses the Digital Differential Analyzer (DDA) algorithm —
  //  the same technique used in raycasting renderers — to trace
  //  a straight line from (sx,sy) to (ex,ey) and compute the
  //  total length of that line that passes through building cells.
  //
  //  The result is used to compute exponential signal attenuation:
  //    attenuated_power = power * exp(-atten_per_unit * result)
  //
  //  DDA works by stepping one grid edge at a time, always choosing
  //  the axis whose next crossing comes sooner. This guarantees
  //  we visit every cell the ray passes through, in order.
  // ----------------------------------------------------------
  double calculateBuildingPenetrationDDA(double sx, double sy, double ex, double ey)
  {
    double dx=ex-sx, dy=ey-sy, len=std::hypot(dx,dy);
    if (len<1e-6) return 0;  // Tower and cell are at the same point — no penetration

    // Unit direction vector
    double dirX=dx/len, dirY=dy/len;

    // Current and end cell indices (integer grid squares)
    int mx=(int)std::floor(sx), my=(int)std::floor(sy);
    int emx=(int)std::floor(ex), emy=(int)std::floor(ey);

    // How far (in ray units) between consecutive vertical/horizontal grid crossings
    double ddx=std::abs(1.0/dirX), ddy=std::abs(1.0/dirY);

    // Step direction and initial distance to first crossing
    int stepX,stepY; double sdx,sdy;
    if(dirX<0){stepX=-1;sdx=(sx-mx)*ddx;}else{stepX=1;sdx=(mx+1.0-sx)*ddx;}
    if(dirY<0){stepY=-1;sdy=(sy-my)*ddy;}else{stepY=1;sdy=(my+1.0-sy)*ddy;}

    double total=0, cur=0;
    // Max steps = Manhattan distance between start and end cells
    int maxs=std::abs(emx-mx)+std::abs(emy-my)+1;

    for(int s=0;s<maxs;++s){
      if(mx<0||mx>=width||my<0||my>=height)break;
      // If the current cell is a building, accumulate the segment length inside it
      if(cells[my][mx].is_building){double nxt=std::min({sdx,sdy,len});total+=nxt-cur;}
      // Advance to the next grid crossing (choose the closer axis)
      if(sdx<sdy){cur=sdx;sdx+=ddx;mx+=stepX;}else{cur=sdy;sdy+=ddy;my+=stepY;}
      if(cur>=len-1e-6)break;  // Reached the destination cell
    }
    return total;
  }

public:
  // Constructor: allocates the cell grid and seeds the RNG.
  Grid(int w, int h, unsigned seed=42, Hyperparameters hp={})
    : width(w), height(h), gen(seed), params(hp)
  { cells.resize(h, std::vector<Cell>(w)); }

  // Simple accessors
  const Hyperparameters& getParams() const { return params; }
  int getWidth()  const { return width; }
  int getHeight() const { return height; }
  bool isBuilding(int x, int y)    const { return cells[y][x].is_building; }
  bool canPlaceTower(int x, int y) const { return cells[y][x].can_place_tower; }
  double getDemand(int x, int y)   const { return cells[y][x].demand; }

  // ----------------------------------------------------------
  //  resetSignals
  //  Clears all computed signal/QoS values so the grid is ready
  //  for a fresh evaluation with a new set of tower placements.
  // ----------------------------------------------------------
  void resetSignals() {
    for(auto &row:cells)for(auto &c:row)
    {c.signal_strength=0;c.efficiency=0;c.allocated_capacity=0;c.qos_percentage=0;}}

  // ----------------------------------------------------------
  //  addBuilding
  //  Marks a rectangular region of cells as building material.
  //  Building cells cannot have towers placed on them and block
  //  line-of-sight signal (attenuated by atten_per_unit).
  // ----------------------------------------------------------
  void addBuilding(double xmin, double xmax, double ymin, double ymax) {
    buildings.push_back({xmin,xmax,ymin,ymax});
    for(int j=std::max(0,(int)ymin);j<=std::min(height-1,(int)ymax);++j)
      for(int i=std::max(0,(int)xmin);i<=std::min(width-1,(int)xmax);++i)
      {cells[j][i].is_building=true;cells[j][i].can_place_tower=false;}}

  // ----------------------------------------------------------
  //  addBuildingsRandomly
  //  Procedurally generates a city by randomly placing rectangular
  //  buildings of random size (within [min_side, max_side]) while
  //  ensuring at least min_dist spacing between buildings.
  //
  //  Target count is 25% of the maximum packing density, so the
  //  city feels reasonably dense but not completely packed.
  // ----------------------------------------------------------
  void addBuildingsRandomly(double min_side, double max_side, double min_dist) {
    double avg=(min_side+max_side)/2.0;
    // Estimate how many buildings fit at ~25% density
    int target=(int)(((width*height)/std::pow(avg+min_dist,2))*0.25);
    std::uniform_real_distribution<> sd(min_side,max_side),xd(0,width-min_side),yd(0,height-min_side);
    int att=0,placed=0;
    while(placed<target&&att<target*10){++att;
      double w=sd(gen),h=sd(gen),xm=xd(gen),ym=yd(gen);
      // Clamp to grid bounds
      if(xm+w>width)xm=width-w;if(ym+h>height)ym=height-h;
      double xx=xm+w,yy=ym+h; bool ok=true;
      // Reject if too close to an existing building (minimum separation check)
      for(auto &b:buildings){double dx=std::max({0.0,xm-b.xmax,b.xmin-xx});
        double dy=std::max({0.0,ym-b.ymax,b.ymin-yy});
        if(std::hypot(dx,dy)<min_dist){ok=false;break;}}
      if(ok){addBuilding(xm,xx,ym,yy);++placed;}}
    std::cout<<"City Generator: Placed "<<placed<<" buildings (target "<<target<<").\n";}

  // ----------------------------------------------------------
  //  setDemand
  //  Assigns a specific demand value to a single cell.
  //  Used to place hotspots (e.g. train stations, stadiums).
  // ----------------------------------------------------------
  void setDemand(int x, int y, double d) { if(x>=0&&x<width&&y>=0&&y<height) cells[y][x].demand=d; }

  // ----------------------------------------------------------
  //  addRandomDemand
  //  Randomly assigns demand to cells.
  //  Open (non-building) cells get demand with probability `prob`.
  //  Building cells always get demand (people inside buildings
  //  need connectivity too), weighted 5x higher.
  // ----------------------------------------------------------
  void addRandomDemand(int maxd, double prob=0.15) {
    std::uniform_int_distribution<> dd(1,maxd); std::uniform_real_distribution<> pd(0,1);
    for(int j=0;j<height;++j)for(int i=0;i<width;++i){
      if(!cells[j][i].is_building&&pd(gen)<prob)cells[j][i].demand=dd(gen);
      else if(cells[j][i].is_building)cells[j][i].demand=dd(gen)*5;}}

  // ----------------------------------------------------------
  //  applyTowerSignal
  //  Computes the signal contribution of one tower to every cell
  //  within its effective range and accumulates it into cell.signal_strength.
  //
  //  Signal model (for each cell at distance d from tower):
  //    1. Free-space path loss: pw = power / d^path_loss_exp
  //    2. Building attenuation (via DDA ray-march):
  //         pw *= exp(-atten_per_unit * building_depth)
  //    3. Diffraction (only if LOS is blocked and cell is not inside a building):
  //       For each building corner visible from the tower, compute
  //       the signal that would arrive at that corner, then model
  //       it diffracting outward to the target cell.
  //         diff += (corner_power * diff_strength) / dist_to_corner^diff_exp
  //
  //  The bounding-box culling (lb) avoids checking buildings that
  //  are entirely outside the tower's effective radius.
  // ----------------------------------------------------------
  void applyTowerSignal(const PlacedTower &p, const TowerType &t, bool fast=true) {
    double tx=p.x,ty=p.y;
    // Compute maximum range where signal still exceeds noise floor
    double thr=params.noise_floor*1.01;
    double maxr=std::pow(t.power/thr,1.0/params.path_loss_exp);

    // Bounding box of cells within range
    int x0=std::max(0,(int)(tx-maxr)),x1=std::min(width-1,(int)(tx+maxr));
    int y0=std::max(0,(int)(ty-maxr)),y1=std::min(height-1,(int)(ty+maxr));

    // Collect only buildings that overlap the tower's influence area
    std::vector<BoundingBox> lb;
    for(auto &b:buildings)if(b.xmax>=x0&&b.xmin<=x1&&b.ymax>=y0&&b.ymin<=y1)lb.push_back(b);

    for(int j=y0;j<=y1;++j)for(int i=x0;i<=x1;++i){
      double d=std::max(std::hypot(i-tx,j-ty),1.0);if(d>maxr)continue;

      // Step 1: free-space path loss
      double pw=t.power/std::pow(d,params.path_loss_exp);

      // Step 2: building penetration attenuation
      double bd=calculateBuildingPenetrationDDA(tx,ty,i,j);
      pw*=std::exp(-params.atten_per_unit*bd);

      // Step 3: diffraction around nearby building corners
      double diff=0;
      if(bd>0&&!cells[j][i].is_building)  // Only diffract if LOS was blocked and cell is open
        for(auto &b:lb){
          // Check all 4 corners of each building
          std::pair<double,double> cs[4]={{b.xmin,b.ymin},{b.xmin,b.ymax},{b.xmax,b.ymin},{b.xmax,b.ymax}};
          for(auto &[cx,cy]:cs){
            double dc=std::max(std::hypot(cx-tx,cy-ty),1.0);
            double pc=t.power/std::pow(dc,params.path_loss_exp);  // Power at the corner
            double dd=std::max(std::hypot(i-cx,j-cy),1.0);        // Corner-to-cell distance
            diff+=(pc*params.diff_strength)/std::pow(dd,params.diff_exp);
          }}
      cells[j][i].signal_strength+=pw+diff;}}

  // ----------------------------------------------------------
  //  resolveNetworkCapacity
  //  After all tower signals have been applied, this function
  //  distributes the finite network capacity among demand cells.
  //
  //  Algorithm:
  //    1. For each demand cell, compute Shannon spectral efficiency
  //       using SINR = signal_strength / noise_floor.
  //       efficiency = log2(1 + SINR)  [bits/s/Hz]
  //
  //    2. Each cell's required capacity = demand / efficiency
  //       (how many Hz of bandwidth it needs to serve its demand).
  //
  //    3. If total required capacity exceeds the tower network's
  //       total capacity, scale proportionally (fair sharing).
  //
  //    4. QoS% for each cell = min(100, served_data / demand * 100)
  // ----------------------------------------------------------
  void resolveNetworkCapacity(double total_cap) {
    double total_req=0;
    for(int j=0;j<height;++j)for(int i=0;i<width;++i){Cell &c=cells[j][i];
      if(c.demand>0&&!c.is_building){
        double sinr=c.signal_strength/params.noise_floor;
        c.efficiency=std::log2(1.0+sinr);           // Shannon formula
        if(c.efficiency>0.01)total_req+=c.demand/c.efficiency;  // Required spectral resource
      }}

    // Allocation ratio: 1.0 if capacity is sufficient, <1.0 if overloaded
    double alloc=(total_req>total_cap)?total_cap/total_req:1.0;

    for(int j=0;j<height;++j)for(int i=0;i<width;++i){Cell &c=cells[j][i];
      if(c.demand>0&&!c.is_building){
        if(c.efficiency>0.01){
          double req=c.demand/c.efficiency;
          c.allocated_capacity=req*alloc;
          // QoS = what fraction of demand was actually served
          c.qos_percentage=std::min(100.0,(c.allocated_capacity*c.efficiency/c.demand)*100.0);
        }
        else{c.qos_percentage=0;c.allocated_capacity=0;}}}}

  // ----------------------------------------------------------
  //  computeScore
  //  Computes the overall QoS metric: demand-weighted average of
  //  each cell's QoS percentage. A score of 100 means every cell
  //  got exactly the bandwidth it demanded; 0 means no service.
  //
  //  This is the objective function that all algorithms optimize.
  // ----------------------------------------------------------
  double computeScore() const {
    double wq=0,td=0;
    for(int j=0;j<height;++j)for(int i=0;i<width;++i){const Cell &c=cells[j][i];
      if(c.demand>0&&!c.is_building){wq+=c.qos_percentage*c.demand;td+=c.demand;}}
    return td>0?wq/td:0;}
};


// ============================================================
//  evaluatePlacement
//  Top-level evaluation function used by all algorithms.
//  Resets the grid, applies every tower's signal, resolves
//  capacity allocation, and returns the QoS score.
// ============================================================
double evaluatePlacement(Grid &grid, const std::vector<PlacedTower> &pl,
                         const std::vector<TowerType> &types, bool fast=true) {
  grid.resetSignals();
  double cap=0;
  for(auto &p:pl){
    grid.applyTowerSignal(p,types[p.type_index],fast);
    cap+=types[p.type_index].capacity;  // Accumulate total network capacity
  }
  grid.resolveNetworkCapacity(cap);
  return grid.computeScore();
}


// ############################################################
//  GREEDY + SA ALGORITHMS (A, B, C)
//
//  These three heuristic algorithms all use greedy initialization
//  followed by Simulated Annealing (SA) local search.
//  They differ in how they choose candidate tower positions.
// ############################################################

// SignalCache: a 2D grid of accumulated signal strengths, used by
// the greedy algorithm to cheaply estimate marginal gain without
// re-evaluating the full physics model for every candidate.
using SignalCache = std::vector<std::vector<double>>;

// ----------------------------------------------------------
//  addToCache
//  Adds the approximate signal footprint of one tower to the
//  cache. Uses simplified path loss (no buildings/diffraction)
//  for speed — this is a "fast approximation" used only during
//  greedy selection, not for final scoring.
// ----------------------------------------------------------
void addToCache(SignalCache &c,double tx,double ty,const TowerType &t,const Hyperparameters &p,int W,int H){
  // Effective range: distance where signal just crosses noise floor
  double mr=std::pow(t.power/(p.noise_floor*1.01),1.0/p.path_loss_exp);
  int x0=std::max(0,(int)(tx-mr)),x1=std::min(W-1,(int)(tx+mr));
  int y0=std::max(0,(int)(ty-mr)),y1=std::min(H-1,(int)(ty+mr));
  for(int j=y0;j<=y1;++j)for(int i=x0;i<=x1;++i){
    double d=std::max(std::hypot(i-tx,j-ty),1.0);if(d>mr)continue;
    c[j][i]+=t.power/std::pow(d,p.path_loss_exp);}}

// ----------------------------------------------------------
//  marginalGain
//  Estimates how much QoS improvement a new tower at (tx,ty)
//  would contribute, given the current accumulated signal cache.
//
//  For each cell in range:
//    gain += demand * delta_efficiency * coverage_bonus
//  where:
//    delta_efficiency = log2(1 + new_signal/noise) - log2(1 + old_signal/noise)
//    coverage_bonus = 2x for cells with very weak existing signal
//                     (prioritises coverage over capacity)
//
//  Dividing by tower cost (in greedyFill) gives gain-per-dollar,
//  which is the greedy selection criterion.
// ----------------------------------------------------------
double marginalGain(const Grid &g,const SignalCache &c,double tx,double ty,
                    const TowerType &t,const Hyperparameters &p){
  double gain=0,thr=p.noise_floor*5;  // 5x noise floor = "weak signal" threshold
  double mr=std::pow(t.power/(p.noise_floor*1.01),1.0/p.path_loss_exp);
  int x0=std::max(0,(int)(tx-mr)),x1=std::min(g.getWidth()-1,(int)(tx+mr));
  int y0=std::max(0,(int)(ty-mr)),y1=std::min(g.getHeight()-1,(int)(ty+mr));
  for(int j=y0;j<=y1;++j)for(int i=x0;i<=x1;++i){
    if(g.isBuilding(i,j))continue;
    double dem=g.getDemand(i,j);if(dem<=0)continue;
    double d=std::max(std::hypot(i-tx,j-ty),1.0);if(d>mr)continue;
    double add=t.power/std::pow(d,p.path_loss_exp);
    double bef=c[j][i],aft=bef+add;
    // Shannon capacity improvement from adding this tower's signal
    double cg=std::log2(1+aft/p.noise_floor)-std::log2(1+bef/p.noise_floor);
    // Double the gain weight for cells that are currently underserved
    gain+=dem*cg*((bef<thr)?2.0:1.0);}
  return gain;}

// ----------------------------------------------------------
//  greedyFill
//  Iteratively places the tower with the best marginal-gain/cost
//  ratio until the budget runs out or no improvement is possible.
//
//  This is a submodular greedy algorithm: it approximates the
//  optimal set cover by always picking the locally best option.
//  The signal cache avoids re-evaluating all towers from scratch
//  every iteration — only the newly added tower updates the cache.
// ----------------------------------------------------------
void greedyFill(Grid &g,const std::vector<TowerType> &types,const Hyperparameters &p,
                const std::vector<std::pair<int,int>> &cands,double budget,AlgorithmResult &r){
  // Start from whatever placements are already in r (allows warm starts)
  SignalCache cache(g.getHeight(),std::vector<double>(g.getWidth(),0));
  std::set<std::pair<int,int>> placed;
  for(auto &pt:r.placements){
    addToCache(cache,pt.x,pt.y,types[pt.type_index],p,g.getWidth(),g.getHeight());
    placed.insert({(int)pt.x,(int)pt.y});}

  while(r.total_cost<budget){
    double bs=0;PlacedTower bp{};bool found=false;
    // Evaluate every candidate position × every tower type
    for(auto &[cx,cy]:cands){
      if(!g.canPlaceTower(cx,cy)||placed.count({cx,cy}))continue;
      for(int t=0;t<(int)types.size();++t){
        if(types[t].cost>budget-r.total_cost)continue;  // Skip if over budget
        double s=marginalGain(g,cache,cx,cy,types[t],p)/types[t].cost;  // Gain per dollar
        if(s>bs){bs=s;bp={(double)cx,(double)cy,t};found=true;}}}
    if(!found)break;  // No beneficial placement remains within budget
    r.placements.push_back(bp);
    r.total_cost+=types[bp.type_index].cost;
    placed.insert({(int)bp.x,(int)bp.y});
    addToCache(cache,bp.x,bp.y,types[bp.type_index],p,g.getWidth(),g.getHeight());}}

// ----------------------------------------------------------
//  snapToValid
//  Rounds a floating-point grid coordinate to the nearest
//  cell that can legally hold a tower. Used after k-means
//  clustering, where centroids may land on buildings.
//  Searches outward in expanding rings until a free cell is found.
// ----------------------------------------------------------
std::pair<int,int> snapToValid(const Grid &g,double fx,double fy){
  int cx=std::clamp((int)std::round(fx),0,g.getWidth()-1);
  int cy=std::clamp((int)std::round(fy),0,g.getHeight()-1);
  if(g.canPlaceTower(cx,cy))return{cx,cy};
  // Expanding ring search up to radius 24
  for(int r=1;r<25;++r)for(int dj=-r;dj<=r;++dj)for(int di=-r;di<=r;++di){
    int nx=cx+di,ny=cy+dj;
    if(nx>=0&&nx<g.getWidth()&&ny>=0&&ny<g.getHeight()&&g.canPlaceTower(nx,ny))return{nx,ny};}
  return{cx,cy};}  // Fallback: return original (may be invalid)

// ----------------------------------------------------------
//  simulatedAnnealing
//  Core SA optimizer shared by all three heuristic algorithms.
//
//  Starting from an initial solution (e.g. from greedy), SA
//  iteratively proposes random perturbations and accepts them
//  according to the Metropolis criterion:
//    - Always accept improvements (delta > 0)
//    - Accept worsening moves with probability exp(delta/T)
//      where T is the current "temperature"
//
//  Temperature decays exponentially from T0 down to T1 over
//  all iterations (geometric cooling schedule).
//
//  Four types of moves are attempted at each step (roll determines which):
//    roll < 0.15: ADD a random new tower (if budget allows)
//    roll < 0.25: REMOVE a random existing tower
//    roll < 0.40: SWAP a tower's type for a random different type
//    else:        MOVE a tower by a random displacement (±nr cells)
//
//  The best solution ever seen is returned (not just the final state).
// ----------------------------------------------------------
AlgorithmResult simulatedAnnealing(Grid &grid,const std::vector<TowerType> &types,
    AlgorithmResult init,double budget,int iters,double T0,double T1,int nr,bool fast,std::mt19937 &rng){
  auto cur=init;
  cur.score=evaluatePlacement(grid,cur.placements,types,fast);
  auto best=cur;  // Track all-time best, not just current state

  std::uniform_real_distribution<> uni(0,1);
  std::uniform_int_distribution<> xd(0,grid.getWidth()-1),yd(0,grid.getHeight()-1),nd(-nr,nr),td(0,(int)types.size()-1);

  // Precompute the log ratio for the exponential cooling schedule
  double lr=std::log(T1/T0);

  for(int it=0;it<iters;++it){
    // Temperature at this iteration (geometric decay from T0 to T1)
    double T=T0*std::exp(lr*it/iters);
    auto cand=cur;
    double roll=uni(rng);

    // --- Propose a move ---
    if(cand.placements.empty()||roll<0.15){
      // ADD: try up to 20 random positions for a new tower
      for(int t=0;t<20;++t){int nx=xd(rng),ny=yd(rng),nt=td(rng);
        if(!grid.canPlaceTower(nx,ny))continue;
        if(cand.total_cost+types[nt].cost>budget)continue;
        cand.placements.push_back({(double)nx,(double)ny,nt});
        cand.total_cost+=types[nt].cost;break;}}
    else if(roll<0.25){
      // REMOVE: delete a random tower from the solution
      std::uniform_int_distribution<> id(0,(int)cand.placements.size()-1);int i=id(rng);
      cand.total_cost-=types[cand.placements[i].type_index].cost;
      cand.placements.erase(cand.placements.begin()+i);}
    else if(roll<0.40){
      // TYPE SWAP: change a random tower to a different type
      std::uniform_int_distribution<> id(0,(int)cand.placements.size()-1);int i=id(rng),nt=td(rng);
      double d=types[nt].cost-types[cand.placements[i].type_index].cost;
      if(cand.total_cost+d<=budget){cand.total_cost+=d;cand.placements[i].type_index=nt;}}
    else{
      // MOVE: shift a random tower by a random displacement (clamped to grid)
      std::uniform_int_distribution<> id(0,(int)cand.placements.size()-1);int i=id(rng);
      int nx=std::clamp((int)cand.placements[i].x+nd(rng),0,grid.getWidth()-1);
      int ny=std::clamp((int)cand.placements[i].y+nd(rng),0,grid.getHeight()-1);
      if(grid.canPlaceTower(nx,ny)){cand.placements[i].x=nx;cand.placements[i].y=ny;}}

    if(cand.placements.empty())continue;

    // Evaluate the candidate solution
    cand.score=evaluatePlacement(grid,cand.placements,types,fast);
    double delta=cand.score-cur.score;

    // Metropolis acceptance criterion
    if(delta>0||uni(rng)<std::exp(delta/T)){
      cur=cand;
      if(cur.score>best.score)best=cur;  // Track global best
    }}
  return best;}

// ----------------------------------------------------------
//  algorithmA_WindowSA  (Algorithm A)
//
//  Strategy: Divide-and-conquer greedy + global SA refinement.
//
//  The grid is divided into overlapping windows (each W/4 × H/4
//  in size, sliding by 60% of the window size so windows overlap).
//  Each window gets a proportional share of the budget, and greedy
//  placement is run independently within each window.
//
//  The per-window results are merged into a global solution,
//  deduplicating any positions placed by multiple overlapping windows.
//  Finally, global SA refines the merged solution.
//
//  Rationale: small windows let greedy converge quickly on local
//  structure, and the overlap prevents gaps at window boundaries.
// ----------------------------------------------------------
AlgorithmResult algorithmA_WindowSA(Grid &grid,const std::vector<TowerType> &types,
    const Hyperparameters &hp,double budget,bool fast,std::mt19937 &rng){
  AlgorithmResult r;r.name="A_WindowSA";
  int W=grid.getWidth(),H=grid.getHeight(),ww=W/4,wh=H/4;
  int sx=(int)(ww*0.6),sy=(int)(wh*0.6);  // Stride = 60% of window = 40% overlap
  int nw=(W+sx-1)/sx,nh=(H+sy-1)/sy;
  double pwb=budget/(nw*nh);  // Budget per window
  std::set<std::pair<int,int>> seen;  // Prevents duplicate tower positions across windows

  for(int wy=0;wy<H;wy+=sy)for(int wx=0;wx<W;wx+=sx){
    int x1=std::min(wx+ww,W),y1=std::min(wy+wh,H);
    // Enumerate all candidate positions within this window
    std::vector<std::pair<int,int>> c;
    for(int j=wy;j<y1;++j)for(int i=wx;i<x1;++i)c.push_back({i,j});
    AlgorithmResult w;w.name=r.name;
    greedyFill(grid,types,hp,c,pwb,w);
    // Merge towers from this window, skipping duplicates and budget overruns
    for(auto &p:w.placements){auto k=std::make_pair((int)p.x,(int)p.y);
      if(seen.count(k))continue;
      if(r.total_cost+types[p.type_index].cost>budget)continue;
      seen.insert(k);r.placements.push_back(p);r.total_cost+=types[p.type_index].cost;}}

  // SA refinement: 500 iterations, T: 5.0 → 0.02, max move ±15 cells
  r=simulatedAnnealing(grid,types,r,budget,500,5.0,0.02,15,fast,rng);
  r.name="A_WindowSA";return r;}

// ----------------------------------------------------------
//  algorithmB_GlobalGreedySA  (Algorithm B)
//
//  Strategy: Sparse global greedy + global SA refinement.
//
//  Candidate positions are sampled on a coarse 8×8 grid across
//  the entire map (i.e. every 8th cell in both dimensions).
//  Greedy placement runs once globally across all candidates.
//
//  This is the simplest of the three algorithms. The coarse
//  sampling reduces computation but may miss optimal positions
//  between grid points — SA compensates by moving towers afterward.
// ----------------------------------------------------------
AlgorithmResult algorithmB_GlobalGreedySA(Grid &grid,const std::vector<TowerType> &types,
    const Hyperparameters &hp,double budget,bool fast,std::mt19937 &rng){
  AlgorithmResult r;r.name="B_GlobalGreedy";
  // Sample candidate positions every 8 cells
  std::vector<std::pair<int,int>> c;
  for(int j=0;j<grid.getHeight();j+=8)for(int i=0;i<grid.getWidth();i+=8)c.push_back({i,j});
  greedyFill(grid,types,hp,c,budget,r);
  // SA refinement: 600 iterations, T: 3.0 → 0.01, max move ±20 cells
  r=simulatedAnnealing(grid,types,r,budget,600,3.0,0.01,20,fast,rng);
  r.name="B_GlobalGreedy";return r;}

// ----------------------------------------------------------
//  algorithmC_ClusterGreedySA  (Algorithm C)
//
//  Strategy: Demand-aware k-means seeding + dense greedy + SA.
//
//  Step 1: Estimate k = budget / (2 * avg_tower_cost) as the
//          number of towers affordable. Run weighted k-means with
//          demand as the weight to find k cluster centroids —
//          these are naturally located where users are.
//
//  Step 2: Place the cheapest tower type at each centroid as a
//          warm start (snap to nearest valid cell if centroid
//          lands on a building).
//
//  Step 3: Run dense greedy over every 5th cell that has demand,
//          filling in remaining budget.
//
//  Step 4: SA refinement.
//
//  Rationale: demand-weighted centroids are better starting points
//  than random or uniform grid positions, especially when demand
//  is spatially clustered (e.g. hotspots, buildings).
// ----------------------------------------------------------
AlgorithmResult algorithmC_ClusterGreedySA(Grid &grid,const std::vector<TowerType> &types,
    const Hyperparameters &hp,double budget,bool fast,std::mt19937 &rng){
  AlgorithmResult r;r.name="C_ClusterGreedy";

  // Estimate number of clusters from budget and average tower cost
  double ac=0;for(auto &t:types)ac+=t.cost;ac/=types.size();
  int k=std::max(4,(int)(budget/(ac*2)));

  // Collect all non-building cells with demand as k-means input points
  std::vector<std::pair<int,int>> pts;
  for(int j=0;j<grid.getHeight();++j)for(int i=0;i<grid.getWidth();++i)
    if(!grid.isBuilding(i,j)&&grid.getDemand(i,j)>0)pts.push_back({i,j});
  if(pts.empty())return r;

  // Cap k at the number of demand points
  k=std::min(k,(int)pts.size());
  std::shuffle(pts.begin(),pts.end(),rng);

  // Initialize centroids by selecting the first k shuffled demand points
  std::vector<std::pair<double,double>> cen(k);
  for(int i=0;i<k;++i)cen[i]={(double)pts[i].first,(double)pts[i].second};

  // Run 20 iterations of weighted k-means (demand is the weight)
  for(int it=0;it<20;++it){
    std::vector<double> wx(k,0),wy(k,0),wt(k,0);
    for(auto &[px,py]:pts){
      // Assign each point to its nearest centroid
      double bd=1e18;int bk=0;
      for(int c=0;c<k;++c){double d=std::hypot(px-cen[c].first,py-cen[c].second);if(d<bd){bd=d;bk=c;}}
      // Accumulate weighted sum for centroid update
      double w=grid.getDemand(px,py);wx[bk]+=px*w;wy[bk]+=py*w;wt[bk]+=w;}
    // Move each centroid to the demand-weighted mean of its cluster
    for(int c=0;c<k;++c)if(wt[c]>0)cen[c]={wx[c]/wt[c],wy[c]/wt[c]};}

  // Find the cheapest tower type for the initial centroid seeding
  int ch=0;for(int t=1;t<(int)types.size();++t)if(types[t].cost<types[ch].cost)ch=t;

  // Place cheap towers at each cluster centroid as the warm start
  for(auto &[cx,cy]:cen){
    if(r.total_cost+types[ch].cost>budget)break;
    auto [sx,sy]=snapToValid(grid,cx,cy);
    r.placements.push_back({(double)sx,(double)sy,ch});
    r.total_cost+=types[ch].cost;}

  // Dense greedy over demand cells sampled every 5 cells
  std::vector<std::pair<int,int>> c;
  for(int j=0;j<grid.getHeight();j+=5)for(int i=0;i<grid.getWidth();i+=5)
    if(grid.getDemand(i,j)>0)c.push_back({i,j});
  greedyFill(grid,types,hp,c,budget,r);

  // SA refinement: 600 iterations, T: 4.0 → 0.01, max move ±20 cells
  r=simulatedAnnealing(grid,types,r,budget,600,4.0,0.01,20,fast,rng);
  r.name="C_ClusterGreedy";return r;}


// ############################################################
//  CSP SOLVER (Backtracking + MRV + LCV + FC + AC3)
//
//  This section implements an exact Constraint Satisfaction Problem
//  solver for the tower placement problem on small grids.
//
//  Problem formulation:
//    Variables:   each demand cell that needs coverage
//    Domain:      set of (tower_position, tower_type) pairs that
//                 can deliver sufficient signal to that cell
//    Constraint:  each tower used has finite capacity — it cannot
//                 serve more total demand than its capacity value
//    Objective:   minimize total tower cost (branch-and-bound)
//
//  The solver supports four optional enhancements:
//    MRV   (Minimum Remaining Values): pick the variable with the
//          fewest valid domain values first — prunes dead ends early
//    LCV   (Least Constraining Value): try domain values that
//          eliminate the fewest options for other variables first
//    FC    (Forward Checking): after assigning a value, immediately
//          remove it from other variables' domains if the tower
//          would be over-capacity
//    AC3   (Arc Consistency 3): propagate constraints further after
//          each assignment, pruning pairs of cells sharing a tower
// ############################################################

// ----------------------------------------------------------
//  TowerKey
//  Uniquely identifies a (position, type) combination.
//  Used as a map/set key — must be totally ordered (operator<).
// ----------------------------------------------------------
struct TowerKey {
  int px,py,ti;  // Column, row, type index
  bool operator==(const TowerKey &o)const{return px==o.px&&py==o.py&&ti==o.ti;}
  bool operator<(const TowerKey &o)const{if(px!=o.px)return px<o.px;if(py!=o.py)return py<o.py;return ti<o.ti;}
};

// DemandCell: a cell that requires coverage — a CSP variable.
struct DemandCell{int x,y;double demand;};

// CSPProblem: the fully assembled CSP instance.
struct CSPProblem{
  std::vector<DemandCell> cells;          // CSP variables (demand cells)
  std::vector<TowerKey> towers;           // All candidate towers (after pruning)
  std::map<TowerKey,std::set<int>> covers; // covers[tower] = set of cell indices it can cover
  std::vector<std::set<TowerKey>> domains; // domains[cell] = towers that can cover it
};

// CSPResult: output of one solver run.
struct CSPResult{
  bool solved;                      // Was a feasible solution found within budget+time?
  double cost;                      // Cost of the best solution found
  int nodes,bt;                     // Nodes explored and backtracks performed
  std::map<int,TowerKey> assignment; // cell_index → tower assigned to cover it
};

// ----------------------------------------------------------
//  towerSignal  (CSP version)
//  Computes signal strength from tower (tx,ty) of type t to
//  cell (cx,cy), used during CSP problem construction.
//  Uses a simplified ray-march for building penetration
//  (linear sampling, not DDA) and a building-edge scan for
//  diffraction — slightly different from the Grid version
//  but sufficient for determining whether a tower can cover a cell.
// ----------------------------------------------------------
double towerSignal(int tx,int ty,int cx,int cy,const TowerType &t,const Hyperparameters &hp,
                   const std::vector<std::vector<bool>>&bl,int R,int C){
  double d=std::max(std::hypot(cx-tx,cy-ty),1.0);
  double pw=t.power/std::pow(d,hp.path_loss_exp);  // Free-space path loss

  // Building penetration via linear ray sampling (sample every 1/3 cell)
  double dx=cx-tx,dy=cy-ty,len=std::hypot(dx,dy),bd=0;
  if(len>0.1){
    int st=(int)(len*3)+1;
    for(int s=0;s<=st;++s){double f=(double)s/st;
      int si=(int)std::floor(tx+dx*f),sj=(int)std::floor(ty+dy*f);
      if(si>=0&&si<C&&sj>=0&&sj<R&&bl[sj][si])bd+=len/st;}}
  pw*=std::exp(-hp.atten_per_unit*bd);

  // Diffraction around building edges (only if path was blocked and target is open)
  double diff=0;
  if(bd>0&&!(cy>=0&&cy<R&&cx>=0&&cx<C&&bl[cy][cx])){
    int d4x[4]={1,-1,0,0},d4y[4]={0,0,1,-1};
    // Find building cells with at least one non-building neighbour (= edge cells)
    for(int bj=0;bj<R;++bj)for(int bi=0;bi<C;++bi){if(!bl[bj][bi])continue;
      bool edge=false;for(int k=0;k<4;++k){int nx=bi+d4x[k],ny=bj+d4y[k];
        if(nx<0||nx>=C||ny<0||ny>=R||!bl[ny][nx]){edge=true;break;}}
      if(!edge)continue;
      double dc=std::max(std::hypot(bi-tx,bj-ty),1.0);
      double pc=t.power/std::pow(dc,hp.path_loss_exp);  // Signal at edge cell
      double dd=std::max(std::hypot(cx-bi,cy-bj),1.0);
      diff+=(pc*hp.diff_strength)/std::pow(dd,hp.diff_exp);}}
  return pw+diff;}

// ----------------------------------------------------------
//  buildCSP
//  Constructs the CSPProblem from the grid state.
//
//  1. Enumerate all demand cells (CSP variables).
//  2. For each candidate tower position × type, compute which
//     demand cells receive signal above 5× noise floor.
//     Only towers that cover at least one cell are kept.
//  3. Dominance pruning: if tower A's coverage ⊆ tower B's coverage
//     and A costs ≥ B, then A is dominated — remove it from all
//     domains. This reduces the search space without losing optimality.
// ----------------------------------------------------------
CSPProblem buildCSP(int R,int C,const std::vector<std::vector<double>>&dem,
    const std::vector<std::vector<bool>>&bl,const std::vector<TowerType>&types,
    const std::vector<std::pair<int,int>>&tpos,const Hyperparameters &hp){
  CSPProblem p;
  // Step 1: collect demand cells
  for(int j=0;j<R;++j)for(int i=0;i<C;++i)if(dem[j][i]>0&&!bl[j][i])p.cells.push_back({i,j,dem[j][i]});
  int n=(int)p.cells.size();
  p.domains.resize(n);
  double ms=hp.noise_floor*5;  // Minimum "usable" signal threshold

  // Step 2: for every candidate tower position × type, compute coverage
  for(auto &[px,py]:tpos){if(bl[py][px])continue;
    for(int t=0;t<(int)types.size();++t){
      TowerKey tk{px,py,t};std::set<int> cov;
      for(int c=0;c<n;++c){
        double sig=towerSignal(px,py,p.cells[c].x,p.cells[c].y,types[t],hp,bl,R,C);
        if(sig>=ms)cov.insert(c);}
      if(!cov.empty()){
        p.covers[tk]=cov;p.towers.push_back(tk);
        for(int c:cov)p.domains[c].insert(tk);}}}  // Add tower to each covered cell's domain

  // Step 3: dominance pruning
  // A tower is dominated if another tower covers a superset of its cells at no greater cost
  std::set<TowerKey> dom;
  for(int i=0;i<(int)p.towers.size();++i){auto &a=p.towers[i];if(dom.count(a))continue;
    for(int j=0;j<(int)p.towers.size();++j){if(i==j)continue;auto &b=p.towers[j];if(dom.count(b))continue;
      if(std::includes(p.covers[b].begin(),p.covers[b].end(),p.covers[a].begin(),p.covers[a].end())&&
         types[a.ti].cost>=types[b.ti].cost&&!(a==b)){dom.insert(a);break;}}}
  // Remove dominated towers from the problem
  for(auto &d:dom){p.covers.erase(d);for(auto &dm:p.domains)dm.erase(d);}
  p.towers.erase(std::remove_if(p.towers.begin(),p.towers.end(),[&](const TowerKey&t){return dom.count(t);}),p.towers.end());
  return p;}

// ----------------------------------------------------------
//  CSPSolver
//  Backtracking search with optional MRV, LCV, FC, AC3.
// ----------------------------------------------------------
class CSPSolver{
  const CSPProblem &prob;          // The problem instance (read-only)
  const std::vector<TowerType>&types; // Tower specs (for costs)
  bool useMRV,useLCV,useFC,useAC3; // Which enhancements are enabled

  // Mutable solver state (modified and restored during backtracking)
  std::vector<std::set<TowerKey>> domains; // Current domain for each cell variable
  std::map<TowerKey,double> remCap;        // Remaining capacity of each tower in current branch
  std::map<int,TowerKey> assign;           // Current partial assignment: cell → tower
  int nodes,bt;                            // Statistics counters
  double bestCost;                         // Cost of best complete solution found so far
  std::map<int,TowerKey> bestAssign;       // Assignment for the best solution
  bool found,timedOut;                     // Flags
  std::chrono::steady_clock::time_point deadline; // Hard time limit

  // ----------------------------------------------------------
  //  pickVar  (MRV heuristic)
  //  Returns the index of the unassigned cell with the smallest
  //  domain — the one most likely to cause a contradiction if
  //  chosen poorly. This focuses search on constrained variables
  //  first, pruning large subtrees early.
  // ----------------------------------------------------------
  int pickVar(const std::vector<int>&u){
    if(!useMRV)return u[0];  // No heuristic: pick first in list
    int b=u[0],bs=(int)domains[b].size();
    for(int c:u){int s=(int)domains[c].size();if(s<bs){bs=s;b=c;}}
    return b;}

  // ----------------------------------------------------------
  //  orderVals  (LCV heuristic)
  //  Returns the domain values for cell `cid` sorted by how many
  //  other cells they would constrain if chosen.
  //  "Least constraining" = fewest future domain reductions → try first.
  //
  //  For each tower value, count how many other unassigned cells
  //  would lose this tower from their domain (because the tower
  //  would have insufficient remaining capacity).
  // ----------------------------------------------------------
  std::vector<TowerKey> orderVals(int cid,const std::vector<int>&u){
    std::vector<TowerKey> ord(domains[cid].begin(),domains[cid].end());
    if(!useLCV||ord.size()<=1)return ord;  // No LCV or only one choice
    double md=prob.cells[cid].demand;
    std::vector<int> imp(ord.size(),0);  // "impact" = number of other cells constrained
    for(int v=0;v<(int)ord.size();++v){
      double nc=remCap[ord[v]]-md;  // Remaining capacity if this tower is chosen
      for(int o:u){if(o==cid)continue;
        // If assigning tower ord[v] to cid would leave it unable to serve cell o
        if(domains[o].count(ord[v])&&nc<prob.cells[o].demand)imp[v]++;}}
    std::vector<int> idx(ord.size());std::iota(idx.begin(),idx.end(),0);
    std::sort(idx.begin(),idx.end(),[&](int a,int b){return imp[a]<imp[b];});  // Least constraining first
    std::vector<TowerKey> r(ord.size());for(int i=0;i<(int)idx.size();++i)r[i]=ord[idx[i]];return r;}

  // ----------------------------------------------------------
  //  fc  (Forward Checking)
  //  After assigning tower `tk` to `cell`, check all other
  //  unassigned cells that share this tower in their domain.
  //  If the tower now lacks capacity to serve them, remove it
  //  from their domain. If any domain becomes empty (wipe-out),
  //  set wipe=true to signal immediate backtrack.
  // ----------------------------------------------------------
  void fc(int cell,const TowerKey &tk,const std::vector<int>&u,bool &wipe){wipe=false;
    for(int o:u){if(o==cell||!domains[o].count(tk))continue;
      if(remCap[tk]<prob.cells[o].demand){
        domains[o].erase(tk);
        if(domains[o].empty()){wipe=true;return;}}}}

  // ----------------------------------------------------------
  //  arc3  (AC-3 Arc Consistency)
  //  Enforces pairwise arc consistency among unassigned cells.
  //  For each pair of cells (ci, cj) that share a tower tk:
  //    if tk doesn't have enough capacity for BOTH ci AND cj,
  //    remove tk from ci's domain (since if ci uses it, cj can't).
  //  Propagates removals through the constraint graph using a queue.
  //
  //  This is more aggressive than FC — it looks two assignments
  //  ahead — but also more expensive. The wipe-out detection
  //  allows early termination.
  // ----------------------------------------------------------
  void arc3(const std::vector<int>&u,bool &wipe){wipe=false;
    std::set<int> us(u.begin(),u.end());
    // Build tower-to-cells index for efficient arc enumeration
    std::map<TowerKey,std::vector<int>> t2c;
    for(int c:u)for(auto &tk:domains[c])t2c[tk].push_back(c);
    struct A{int i,j;TowerKey t;};
    std::deque<A> q;
    // Initialize queue with all arcs (pairs of cells sharing a tower, both directions)
    for(auto &[tk,cl]:t2c)for(int i=0;i<(int)cl.size();++i)for(int j=i+1;j<(int)cl.size();++j)
      {q.push_back({cl[i],cl[j],tk});q.push_back({cl[j],cl[i],tk});}
    while(!q.empty()){auto [ci,cj,tk]=q.front();q.pop_front();
      if(!us.count(ci)||!us.count(cj)||!domains[ci].count(tk))continue;
      // Check if tower can serve both ci and cj simultaneously
      double cb=remCap[tk]-prob.cells[ci].demand-prob.cells[cj].demand;
      if(cb<0){
        // Can't serve both — remove tk from ci's domain
        auto alt=domains[cj];alt.erase(tk);
        if(alt.empty()){domains[ci].erase(tk);
          if(domains[ci].empty()){wipe=true;return;}
          // Propagate: re-queue arcs involving ci
          for(auto &tk2:domains[ci])for(int nb:t2c[tk2])if(nb!=ci&&us.count(nb))q.push_back({nb,ci,tk2});}}}}

  // ----------------------------------------------------------
  //  solve  (recursive backtracking)
  //  Core search procedure. Assigns towers to cells one at a time.
  //
  //  Pruning strategies:
  //    1. Time limit: abort if deadline exceeded
  //    2. Cost bound: prune if current partial cost ≥ best known
  //    3. Empty domain: backtrack immediately if any variable
  //       has no valid assignment
  //    4. FC / AC3: reduce future domains after each assignment
  //
  //  When all variables are assigned (u.empty()), record the solution
  //  if it's cheaper than the current best.
  //
  //  Branch-and-bound: "add" is 0 if the chosen tower is already
  //  in use by another cell (shared tower cost), otherwise it's
  //  the tower's full cost.
  // ----------------------------------------------------------
  void solve(std::vector<int>&u,double cost){
    if(std::chrono::steady_clock::now()>deadline){timedOut=true;return;}
    if(timedOut)return;
    if(u.empty()){
      // All cells assigned — record if this solution is the best so far
      if(cost<bestCost){bestCost=cost;bestAssign=assign;found=true;}
      return;}
    if(cost>=bestCost)return;  // Prune: can't beat current best
    for(int c:u)if(domains[c].empty()){bt++;return;}  // Prune: dead end

    // Select variable (MRV or first)
    int cell=pickVar(u);
    auto it=std::find(u.begin(),u.end(),cell);std::swap(*it,u.back());u.pop_back();

    // Order values (LCV or default)
    auto ord=orderVals(cell,u);

    for(auto &tk:ord){
      if(timedOut)break;
      // Check if this tower is already used (shared cost)
      bool reused=false;
      for(auto &[a,at]:assign)if(at==tk){reused=true;break;}
      double add=reused?0:types[tk.ti].cost;
      if(cost+add>=bestCost)continue;  // Bound: can't improve

      // Make assignment
      assign[cell]=tk;
      remCap[tk]-=prob.cells[cell].demand;  // Consume capacity
      nodes++;

      // Save domains so we can restore on backtrack
      auto saved=domains;bool wipe=false;
      if(useFC&&!wipe){fc(cell,tk,u,wipe);if(wipe)bt++;}
      if(useAC3&&!wipe){arc3(u,wipe);if(wipe)bt++;}
      if(!wipe)solve(u,cost+add);  // Recurse

      // Undo assignment (backtrack)
      domains=saved;
      remCap[tk]+=prob.cells[cell].demand;
      assign.erase(cell);}

    u.push_back(cell);bt++;}  // Restore variable to unassigned list

public:
  // Constructor: takes problem instance and flags for each heuristic
  CSPSolver(const CSPProblem &p,const std::vector<TowerType>&t,bool m,bool l,bool f,bool a)
    :prob(p),types(t),useMRV(m),useLCV(l),useFC(f),useAC3(a){}

  // ----------------------------------------------------------
  //  run
  //  Public entry point. Initialises solver state and starts search.
  //  `budget`: maximum allowed tower cost
  //  `tl`:     time limit in seconds (default 60)
  // ----------------------------------------------------------
  CSPResult run(double budget,int tl=60){
    // Initialise mutable domains from problem
    domains=prob.domains;
    remCap.clear();
    // Set each tower's remaining capacity to its full capacity
    for(auto &[tk,_]:prob.covers)remCap[tk]=types[tk.ti].capacity;
    assign.clear();nodes=bt=0;
    bestCost=budget+1;  // Start with "infinity" so any solution improves it
    found=false;timedOut=false;
    deadline=std::chrono::steady_clock::now()+std::chrono::seconds(tl);
    // Collect all cells with non-empty domains as the initial unassigned set
    std::vector<int> u;
    for(int i=0;i<(int)prob.cells.size();++i)if(!domains[i].empty())u.push_back(i);
    solve(u,0);
    return{found,found?bestCost:0,nodes,bt,bestAssign};}
};

// ----------------------------------------------------------
//  cspToResult
//  Converts a raw CSPResult into the unified AlgorithmResult
//  format used by the rest of the program.
//  The assignment maps cells to towers; we deduplicate towers
//  (a single tower can cover many cells) and compute cost + score.
// ----------------------------------------------------------
AlgorithmResult cspToResult(Grid &g,const CSPResult &cr,const CSPProblem &prob,
    const std::vector<TowerType>&types,const std::string &name){
  AlgorithmResult r;r.name=name;
  if(!cr.solved)return r;  // Return empty result if no solution was found
  // Deduplicate: multiple cells may map to the same tower
  std::set<TowerKey> used;
  for(auto &[c,tk]:cr.assignment)used.insert(tk);
  for(auto &tk:used){
    r.placements.push_back({(double)tk.px,(double)tk.py,tk.ti});
    r.total_cost+=types[tk.ti].cost;}
  r.score=evaluatePlacement(g,r.placements,types);
  r.nodes=cr.nodes;
  return r;}


// ############################################################
//  HELPERS
// ############################################################

// ----------------------------------------------------------
//  exportPlacements
//  Writes a tower placement solution to a CSV file with columns:
//    x, y, type (index), name (string label)
//  Used by the Python plotting script (plot.py).
// ----------------------------------------------------------
void exportPlacements(const std::string &fn,const AlgorithmResult &r,const std::vector<TowerType>&types){
  std::ofstream f(fn);f<<"x,y,type,name\n";
  for(auto &p:r.placements)
    f<<p.x<<","<<p.y<<","<<p.type_index<<","<<types[p.type_index].name<<"\n";}

// ----------------------------------------------------------
//  exportGrid
//  Writes the grid layout to two CSV files:
//    bf (buildings file): x,y of every building cell
//    df (demand file):    x,y,demand of every non-building demand cell
//  Also used by plot.py to render the background map.
// ----------------------------------------------------------
void exportGrid(const std::string &bf,const std::string &df,int R,int C,
    const std::vector<std::vector<double>>&dem,const std::vector<std::vector<bool>>&bl){
  {std::ofstream f(bf);f<<"x,y\n";
   for(int j=0;j<R;++j)for(int i=0;i<C;++i)if(bl[j][i])f<<i<<","<<j<<"\n";}
  {std::ofstream f(df);f<<"x,y,demand\n";
   for(int j=0;j<R;++j)for(int i=0;i<C;++i)
     if(dem[j][i]>0&&!bl[j][i])f<<i<<","<<j<<","<<dem[j][i]<<"\n";}}

// ----------------------------------------------------------
//  printTable
//  Prints a formatted comparison table of all algorithm results
//  to stdout, showing score, tower count, cost, nodes, and time.
// ----------------------------------------------------------
void printTable(const std::vector<AlgorithmResult>&all){
  std::cout<<std::left<<std::setw(22)<<"Algorithm"<<std::right<<std::setw(10)<<"Score"
    <<std::setw(10)<<"Towers"<<std::setw(14)<<"Total Cost"<<std::setw(10)<<"Nodes"
    <<std::setw(12)<<"Time(s)"<<"\n"<<std::string(78,'-')<<"\n";
  for(auto &r:all)std::cout<<std::left<<std::setw(22)<<r.name<<std::right
    <<std::setw(10)<<std::fixed<<std::setprecision(2)<<r.score
    <<std::setw(10)<<r.placements.size()
    <<std::setw(14)<<std::setprecision(0)<<r.total_cost
    <<std::setw(10)<<r.nodes
    <<std::setw(12)<<std::setprecision(4)<<r.time_sec<<"\n";}

// ----------------------------------------------------------
//  loadTowerTypes
//  Reads tower type specifications from a CSV file with columns:
//    name, power, cost, capacity
//  Used for Part 2 to allow flexible tower definitions.
// ----------------------------------------------------------
std::vector<TowerType> loadTowerTypes(const std::string &path) {
  std::vector<TowerType> types;
  std::ifstream f(path);
  if(!f.is_open()) return types;
  std::string line;
  std::getline(f, line); // skip header: name,power,cost,capacity
  while(std::getline(f, line)) {
    if(line.empty()) continue;
    std::istringstream ss(line);
    std::string name;
    double power, cost, capacity;
    char comma;
    std::getline(ss, name, ',');
    ss >> power >> comma >> cost >> comma >> capacity;
    types.push_back({name, power, cost, capacity});
  }
  return types;
}
// ############################################################
//  MAIN
//  Entry point — orchestrates both experiment parts.
// ############################################################

int main(int argc, char* argv[])
{
  // Global physics parameters (shared between Part 1 and Part 2)
  Hyperparameters hp;
  hp.path_loss_exp=3.5; hp.atten_per_unit=0.20;
  hp.diff_strength=0.60; hp.diff_exp=4.0; hp.noise_floor=1e-4;

  // ============================================================
  //  PART 1: Small Grid — CSP vs SA
  //
  //  Three modes depending on command-line arguments:
  //    ./tower                    → 10×10 random grid (default)
  //    ./tower 15 15              → custom R×C random grid
  //    ./tower --load grid.csv    → load grid from GUI-generated CSV
  //    ./tower --load grid.csv 30000  → same but with custom budget
  //
  //  Runs three CSP variants and three SA variants, then exports
  //  results to small_*.csv for plotting.
  // ============================================================

  {
    // Default grid size and budget for Part 1
    int R = 10, C = 10;
    double budget = 25000;
    bool load_mode = false;
    std::string load_path;

    // --- Parse command-line arguments ---
    if(argc >= 3 && std::string(argv[1]) == "--load") {
      // Load mode: read grid from a CSV file produced by gui.py
      load_mode = true;
      load_path = argv[2];
      if(argc >= 4) budget = std::atof(argv[3]);  // Optional budget override
    } else if(argc >= 3) {
      // Size mode: user specifies custom grid dimensions
      R = std::atoi(argv[1]);
      C = std::atoi(argv[2]);
      if(R < 3) R = 3; if(C < 3) C = 3;   // Minimum grid size
      if(R > 50) R = 50; if(C > 50) C = 50; // Maximum grid size (CSP scalability limit)
      if(argc >= 4) budget = std::atof(argv[3]);
    }

    // Load tower types from CSV if it exists, otherwise use defaults
    std::vector<TowerType> types;
    if(std::ifstream("tower_types.csv").good()) {
      types = loadTowerTypes("tower_types.csv");
      std::cout << "Loaded " << types.size() << " tower types from tower_types.csv\n";
    }
    if(types.empty()) {
      types = {{"Macro", 0.15, 5000, 30}, {"Micro", 0.02, 2500, 15}, {"Pico", 0.004, 1000, 8}};
      std::cout << "Using default tower types\n";
    }
    for(auto &t : types)
      std::cout << "  " << t.name << ": power=" << t.power << " cost=" << t.cost << " cap=" << t.capacity << "\n";

    if(load_mode) {
      // ── LOAD MODE: read grid layout from CSV ──
      // Expected CSV format (with header): x,y,is_building,demand
      std::ifstream f(load_path);
      if(!f.is_open()) { std::cerr << "Cannot open " << load_path << "\n"; return 1; }

      // Read all rows to determine grid dimensions and cell data
      std::string line;
      std::getline(f, line); // Skip CSV header row
      int maxX = 0, maxY = 0;
      struct CellData { int x, y; bool bld; double dem; };
      std::vector<CellData> cells_data;
      while(std::getline(f, line)) {
        if(line.empty()) continue;
        int x, y, b; double d; char comma;
        std::istringstream ss(line);
        ss >> x >> comma >> y >> comma >> b >> comma >> d;
        cells_data.push_back({x, y, b != 0, d});
        maxX = std::max(maxX, x);  // Track max coordinates to infer grid size
        maxY = std::max(maxY, y);
      }
      // Grid dimensions = max coordinate + 1 (0-indexed)
      C = maxX + 1;
      R = maxY + 1;

      std::cout << "============================================================\n"
                << "  Part 1: " << R << "x" << C << " Grid — Loaded from " << load_path << "\n"
                << "  Budget: $" << budget << "\n"
                << "============================================================\n\n";

      // Construct the Grid object with loaded dimensions
      Grid grid(C, R, 42, hp);

      // Populate the grid with buildings and demand from the loaded data
      for(auto &cd : cells_data) {
        if(cd.bld) grid.addBuilding(cd.x, cd.x, cd.y, cd.y);  // 1×1 building per flagged cell
        if(cd.dem > 0) grid.setDemand(cd.x, cd.y, cd.dem);
      }

      // Extract building/demand arrays and valid tower positions for the CSP builder
      std::vector<std::vector<double>> random_dem(R, std::vector<double>(C, 0));
      std::vector<std::vector<bool>> random_bl(R, std::vector<bool>(C, false));
      std::vector<std::pair<int, int>> tpos;  // All non-building cells = candidate tower sites
      for(int j = 0; j < R; ++j)
        for(int i = 0; i < C; ++i) {
          random_bl[j][i] = grid.isBuilding(i, j);
          random_dem[j][i] = grid.getDemand(i, j);
          if(!random_bl[j][i]) tpos.push_back({i, j});
        }

      // Build the CSP problem (enumerate coverage, prune dominated towers)
      CSPProblem csp = buildCSP(R, C, random_dem, random_bl, types, tpos, hp);
      std::cout << "CSP: " << csp.cells.size() << " variables, " << csp.towers.size() << " candidate towers\n\n";

      std::vector<AlgorithmResult> all;

      // --- Run CSP variants (D = exact backtracking) ---
      // Each variant uses a different combination of heuristics
      struct Cfg { std::string n; bool m, l, f, a; };
      for(auto &cfg : std::vector<Cfg>{
          {"D_PlainBT",  false, false, false, false},  // Plain backtracking (no heuristics)
          {"D_MRV",      true,  false, false, false},  // + MRV variable ordering
          {"D_MRV_LCV",  true,  true,  false, false}   // + MRV + LCV value ordering
      }) {
        std::cout << "Running " << cfg.n << " ..."; std::cout.flush();
        CSPSolver s(csp, types, cfg.m, cfg.l, cfg.f, cfg.a);
        auto t0 = std::chrono::steady_clock::now();
        CSPResult cr = s.run(budget, 60);  // 60-second time limit
        double sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        AlgorithmResult ar = cspToResult(grid, cr, csp, types, cfg.n);
        ar.nodes = cr.nodes; ar.time_sec = sec;
        std::cout << (cr.solved ? " SOLVED" : " FAIL")
          << "  score=" << std::fixed << std::setprecision(2) << ar.score
          << "%  $" << ar.total_cost << "  nodes=" << cr.nodes
          << "  " << std::setprecision(4) << sec << "s\n";
        all.push_back(ar);
      }

      // --- Run SA variants ---
      // Lambda wrapper to time and record each SA algorithm
      auto runSA = [&](auto fn, const std::string &label) {
        std::mt19937 rng(42);  // Fixed seed for reproducibility
        std::cout << "Running " << label << " ..."; std::cout.flush();
        auto t0 = std::chrono::steady_clock::now();
        AlgorithmResult r = fn(grid, types, hp, budget, true, rng);
        r.time_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        r.name = label;
        std::cout << "  score=" << std::fixed << std::setprecision(2) << r.score
          << "%  $" << r.total_cost
          << "  " << std::setprecision(4) << r.time_sec << "s\n";
        all.push_back(r);
      };
      runSA(algorithmA_WindowSA,        "A_WindowSA");
      runSA(algorithmB_GlobalGreedySA,  "B_GlobalGreedy");
      runSA(algorithmC_ClusterGreedySA, "C_ClusterSA");

      std::cout << "\n"; printTable(all);

      // --- Export all results to CSV for plot.py ---
      exportGrid("small_buildings.csv", "small_demand.csv", R, C, random_dem, random_bl);
      {std::ofstream f("small_grid_info.csv"); f << "rows,cols\n" << R << "," << C << "\n";}
      for(auto &r : all) {
        if(r.placements.empty()) continue;
        exportPlacements("small_" + r.name + "_towers.csv", r, types);
      }
      // Summary CSV with one row per algorithm
      {std::ofstream f("small_results.csv");
       f << "algorithm,score,towers,cost,nodes,time\n";
       for(auto &r : all)
         f << r.name << "," << std::fixed << std::setprecision(2) << r.score
           << "," << r.placements.size()
           << "," << std::setprecision(0) << r.total_cost
           << "," << r.nodes
           << "," << std::setprecision(4) << r.time_sec << "\n";
      }

    } else {
      // ── RANDOM MODE: procedurally generate a city ──
      std::cout << "============================================================\n"
                << "  Part 1: " << R << "x" << C << " Grid — CSP vs Heuristic Algorithms\n"
                << "============================================================\n\n";

      Grid grid(C, R, 42, hp);
      // Small buildings (1–2 cells), min 1 cell spacing
      grid.addBuildingsRandomly(1.0, 2.0, 1.0);
      // 40% of open cells get demand; buildings also get demand (users inside)
      grid.addRandomDemand(10, 0.4);

      // Extract arrays for CSP builder
      std::vector<std::vector<double>> random_dem(R, std::vector<double>(C, 0));
      std::vector<std::vector<bool>> random_bl(R, std::vector<bool>(C, false));
      std::vector<std::pair<int, int>> tpos;
      for(int j = 0; j < R; ++j)
        for(int i = 0; i < C; ++i) {
          random_bl[j][i] = grid.isBuilding(i, j);
          random_dem[j][i] = grid.getDemand(i, j);
          if(!random_bl[j][i]) tpos.push_back({i, j});
        }

      CSPProblem csp = buildCSP(R, C, random_dem, random_bl, types, tpos, hp);
      std::cout << "CSP: " << csp.cells.size() << " variables, " << csp.towers.size() << " candidate towers\n\n";

      std::vector<AlgorithmResult> all;

      // Run CSP variants
      struct Cfg { std::string n; bool m, l, f, a; };
      for(auto &cfg : std::vector<Cfg>{
          {"D_PlainBT",  false, false, false, false},
          {"D_MRV",      true,  false, false, false},
          {"D_MRV_LCV",  true,  true,  false, false}
      }) {
        std::cout << "Running " << cfg.n << " ..."; std::cout.flush();
        CSPSolver s(csp, types, cfg.m, cfg.l, cfg.f, cfg.a);
        auto t0 = std::chrono::steady_clock::now();
        CSPResult cr = s.run(budget, 60);
        double sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        AlgorithmResult ar = cspToResult(grid, cr, csp, types, cfg.n);
        ar.nodes = cr.nodes; ar.time_sec = sec;
        std::cout << (cr.solved ? " SOLVED" : " FAIL")
          << "  score=" << std::fixed << std::setprecision(2) << ar.score
          << "%  $" << ar.total_cost << "  nodes=" << cr.nodes
          << "  " << std::setprecision(4) << sec << "s\n";
        all.push_back(ar);
      }

      // Run SA variants
      auto runSA = [&](auto fn, const std::string &label) {
        std::mt19937 rng(42);
        std::cout << "Running " << label << " ..."; std::cout.flush();
        auto t0 = std::chrono::steady_clock::now();
        AlgorithmResult r = fn(grid, types, hp, budget, true, rng);
        r.time_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        r.name = label;
        std::cout << "  score=" << std::fixed << std::setprecision(2) << r.score
          << "%  $" << r.total_cost
          << "  " << std::setprecision(4) << r.time_sec << "s\n";
        all.push_back(r);
      };
      runSA(algorithmA_WindowSA,        "A_WindowSA");
      runSA(algorithmB_GlobalGreedySA,  "B_GlobalGreedy");
      runSA(algorithmC_ClusterGreedySA, "C_ClusterSA");

      std::cout << "\n"; printTable(all);

      // Export results for plotting
      exportGrid("small_buildings.csv", "small_demand.csv", R, C, random_dem, random_bl);
      {std::ofstream f("small_grid_info.csv"); f << "rows,cols\n" << R << "," << C << "\n";}
      for(auto &r : all) {
        if(r.placements.empty()) continue;
        exportPlacements("small_" + r.name + "_towers.csv", r, types);
      }
      {std::ofstream f("small_results.csv");
       f << "algorithm,score,towers,cost,nodes,time\n";
       for(auto &r : all)
         f << r.name << "," << std::fixed << std::setprecision(2) << r.score
           << "," << r.placements.size()
           << "," << std::setprecision(0) << r.total_cost
           << "," << r.nodes
           << "," << std::setprecision(4) << r.time_sec << "\n";
      }
    }
  }

  // ============================================================
  //  PART 2: 300×250 Grid — Heuristic Algorithms at Scale
  //
  //  This part always uses random city generation and runs only
  //  the SA algorithms (A, B, C). The CSP is skipped because
  //  a grid this large would have thousands of variables and
  //  millions of candidate towers — computationally infeasible.
  //
  //  Demonstrates that SA algorithms handle large instances and
  //  still achieve high QoS scores.
  // ============================================================
  std::cout << "\n\n============================================================\n"
            << "  Part 2: 300x250 Grid — Heuristic Algorithms at Scale\n"
            << "============================================================\n\n";
  {
    // Create a 300-wide × 250-tall grid
    Grid grid(300,250,42,hp);
    // Large buildings (20–80 cells), min 15 cell spacing between buildings
    grid.addBuildingsRandomly(20.0,80.0,15.0);
    // Default 15% cell coverage with demand up to 10 units
    grid.addRandomDemand(10);
    // Manually add three high-demand hotspots (e.g. stadiums, train stations)
    grid.setDemand(40,50,100.0);    // Hotspot at (40,50)
    grid.setDemand(110,50,100.0);   // Hotspot at (110,50)
    grid.setDemand(200,150,80.0);   // Hotspot at (200,150)

    // Tower types for Part 2 (large grid — much higher power levels)
    std::vector<TowerType> types={
        {"Macro", 600,   60000, 4000},  // High-power macro cell, expensive
        {"Micro", 150,   30000, 1200},  // Medium power
        {"Pico",   25,   12000,  300}   // Low power, cheapest
    };
    double budget=400000;  // Total deployment budget
    bool fast=true;        // Use fast (no-building) signal evaluation for speed

    // Count and sum demand to show scale context
    int nd=0;double td=0;
    for(int j=0;j<250;++j)for(int i=0;i<300;++i)
      if(!grid.isBuilding(i,j)&&grid.getDemand(i,j)>0){nd++;td+=grid.getDemand(i,j);}
    std::cout<<"Grid: 300x250  Demand cells: "<<nd<<"  Total demand: "<<td<<"\n";
    std::cout<<"Budget: $"<<budget<<"\n";
    std::cout<<"CSP would need "<<nd<<" variables — computationally infeasible.\n\n";

    // Write grid dimensions for plot.py
    {std::ofstream f("big_grid_info.csv");f<<"rows,cols\n250,300\n";}

    std::vector<AlgorithmResult> results;

    // Lambda to run, time, print, and record one SA algorithm
    auto runAlg=[&](auto fn,const std::string &label){
      std::mt19937 rng(42);
      std::cout<<"Running "<<label<<" ...";std::cout.flush();
      auto t0=std::chrono::steady_clock::now();
      AlgorithmResult r=fn(grid,types,hp,budget,fast,rng);
      r.time_sec=std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count();
      r.name=label;
      std::cout<<"  score="<<std::fixed<<std::setprecision(2)<<r.score
        <<"%  $"<<std::setprecision(0)<<r.total_cost
        <<"  "<<r.placements.size()<<" towers  "
        <<std::setprecision(2)<<r.time_sec<<"s\n";
      results.push_back(r);};

    runAlg(algorithmA_WindowSA,        "A_WindowSA");
    runAlg(algorithmB_GlobalGreedySA,  "B_GlobalGreedy");
    runAlg(algorithmC_ClusterGreedySA, "C_ClusterSA");

    std::cout<<"\n"; printTable(results);

    // Identify and announce the winning algorithm
    auto best=std::max_element(results.begin(),results.end(),
      [](const AlgorithmResult &a,const AlgorithmResult &b){return a.score<b.score;});
    std::cout<<"\nWinner: "<<best->name<<" (score "<<std::fixed<<std::setprecision(2)<<best->score<<"%)\n";

    // Extract grid state for export
    std::vector<std::vector<bool>> bl2(250,std::vector<bool>(300,false));
    std::vector<std::vector<double>> dem2(250,std::vector<double>(300,0));
    for(int j=0;j<250;++j)for(int i=0;i<300;++i){
      bl2[j][i]=grid.isBuilding(i,j);
      dem2[j][i]=grid.getDemand(i,j);}

    // Export grid layout and all algorithm placements to CSV
    exportGrid("big_buildings.csv","big_demand.csv",250,300,dem2,bl2);
    for(auto &r:results)
      exportPlacements("big_"+r.name+"_towers.csv",r,types);

    // Summary results table for plotting
    {std::ofstream f("big_results.csv");
     f<<"algorithm,score,towers,cost,nodes,time\n";
     for(auto &r:results)
       f<<r.name<<","<<std::fixed<<std::setprecision(2)<<r.score
        <<","<<r.placements.size()
        <<","<<std::setprecision(0)<<r.total_cost
        <<","<<r.nodes
        <<","<<std::setprecision(4)<<r.time_sec<<"\n";}
  }

  return 0;
}
