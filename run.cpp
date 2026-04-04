// ============================================================
//  Tower Placement — CSP + Heuristic Comparison
//
//  Part 1: 9x9 grid — CSP (exact) vs SA (heuristic)
//          Shows both reach same optimal QoS.
//
//  Part 2: 300x250 grid — SA only (A/B/C algorithms)
//          CSP can't scale; SA handles it with high QoS.
//
//  Physics: path loss, building attenuation, diffraction,
//           Shannon capacity, demand-weighted QoS scoring.
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

// ============================================================
//  Hyperparameters
// ============================================================
struct Hyperparameters
{
  double path_loss_exp   = 3.5;
  double atten_per_unit  = 0.20;
  double diff_strength   = 0.8;
  double diff_exp        = 4.0;
  double noise_floor     = 1e-4;
};

struct TowerType
{
  std::string name;
  double power, cost, capacity;
};

struct PlacedTower
{
  double x, y;
  int type_index;
};

struct AlgorithmResult
{
  std::string name;
  std::vector<PlacedTower> placements;
  double score      = 0.0;
  double total_cost = 0.0;
  int    nodes      = 0;
  double time_sec   = 0.0;
};

struct Cell
{
  double demand = 0.0;
  bool can_place_tower = true;
  bool is_building     = false;
  double signal_strength    = 0.0;
  double efficiency         = 0.0;
  double allocated_capacity = 0.0;
  double qos_percentage     = 0.0;
};

struct BoundingBox { double xmin, xmax, ymin, ymax; };

// ============================================================
//  Grid  — Physics Engine
// ============================================================
class Grid
{
  int width, height;
  std::vector<std::vector<Cell>> cells;
  std::vector<BoundingBox> buildings;
  Hyperparameters params;
  std::mt19937 gen;

  double calculateBuildingPenetrationDDA(double sx, double sy, double ex, double ey)
  {
    double dx=ex-sx, dy=ey-sy, len=std::hypot(dx,dy);
    if (len<1e-6) return 0;
    double dirX=dx/len, dirY=dy/len;
    int mx=(int)std::floor(sx), my=(int)std::floor(sy);
    int emx=(int)std::floor(ex), emy=(int)std::floor(ey);
    double ddx=std::abs(1.0/dirX), ddy=std::abs(1.0/dirY);
    int stepX,stepY; double sdx,sdy;
    if(dirX<0){stepX=-1;sdx=(sx-mx)*ddx;}else{stepX=1;sdx=(mx+1.0-sx)*ddx;}
    if(dirY<0){stepY=-1;sdy=(sy-my)*ddy;}else{stepY=1;sdy=(my+1.0-sy)*ddy;}
    double total=0, cur=0;
    int maxs=std::abs(emx-mx)+std::abs(emy-my)+1;
    for(int s=0;s<maxs;++s){
      if(mx<0||mx>=width||my<0||my>=height)break;
      if(cells[my][mx].is_building){double nxt=std::min({sdx,sdy,len});total+=nxt-cur;}
      if(sdx<sdy){cur=sdx;sdx+=ddx;mx+=stepX;}else{cur=sdy;sdy+=ddy;my+=stepY;}
      if(cur>=len-1e-6)break;}
    return total;
  }

public:
  Grid(int w, int h, unsigned seed=42, Hyperparameters hp={})
    : width(w), height(h), gen(seed), params(hp)
  { cells.resize(h, std::vector<Cell>(w)); }

  const Hyperparameters& getParams() const { return params; }
  int getWidth()  const { return width; }
  int getHeight() const { return height; }
  bool isBuilding(int x, int y)    const { return cells[y][x].is_building; }
  bool canPlaceTower(int x, int y) const { return cells[y][x].can_place_tower; }
  double getDemand(int x, int y)   const { return cells[y][x].demand; }

  void resetSignals() {
    for(auto &row:cells)for(auto &c:row)
    {c.signal_strength=0;c.efficiency=0;c.allocated_capacity=0;c.qos_percentage=0;}}

  void addBuilding(double xmin, double xmax, double ymin, double ymax) {
    buildings.push_back({xmin,xmax,ymin,ymax});
    for(int j=std::max(0,(int)ymin);j<=std::min(height-1,(int)ymax);++j)
      for(int i=std::max(0,(int)xmin);i<=std::min(width-1,(int)xmax);++i)
      {cells[j][i].is_building=true;cells[j][i].can_place_tower=false;}}

  void addBuildingsRandomly(double min_side, double max_side, double min_dist) {
    double avg=(min_side+max_side)/2.0;
    int target=(int)(((width*height)/std::pow(avg+min_dist,2))*0.25);
    std::uniform_real_distribution<> sd(min_side,max_side),xd(0,width-min_side),yd(0,height-min_side);
    int att=0,placed=0;
    while(placed<target&&att<target*10){++att;
      double w=sd(gen),h=sd(gen),xm=xd(gen),ym=yd(gen);
      if(xm+w>width)xm=width-w;if(ym+h>height)ym=height-h;
      double xx=xm+w,yy=ym+h; bool ok=true;
      for(auto &b:buildings){double dx=std::max({0.0,xm-b.xmax,b.xmin-xx});
        double dy=std::max({0.0,ym-b.ymax,b.ymin-yy});
        if(std::hypot(dx,dy)<min_dist){ok=false;break;}}
      if(ok){addBuilding(xm,xx,ym,yy);++placed;}}
    std::cout<<"City Generator: Placed "<<placed<<" buildings (target "<<target<<").\n";}

  void setDemand(int x, int y, double d) { if(x>=0&&x<width&&y>=0&&y<height) cells[y][x].demand=d; }

  void addRandomDemand(int maxd, double prob=0.15) {
    std::uniform_int_distribution<> dd(1,maxd); std::uniform_real_distribution<> pd(0,1);
    for(int j=0;j<height;++j)for(int i=0;i<width;++i){
      if(!cells[j][i].is_building&&pd(gen)<prob)cells[j][i].demand=dd(gen);
      else if(cells[j][i].is_building)cells[j][i].demand=dd(gen)*5;}}

  void applyTowerSignal(const PlacedTower &p, const TowerType &t, bool fast=true) {
    double tx=p.x,ty=p.y;
    double thr=params.noise_floor*1.01;
    double maxr=std::pow(t.power/thr,1.0/params.path_loss_exp);
    int x0=std::max(0,(int)(tx-maxr)),x1=std::min(width-1,(int)(tx+maxr));
    int y0=std::max(0,(int)(ty-maxr)),y1=std::min(height-1,(int)(ty+maxr));
    std::vector<BoundingBox> lb;
    for(auto &b:buildings)if(b.xmax>=x0&&b.xmin<=x1&&b.ymax>=y0&&b.ymin<=y1)lb.push_back(b);
    for(int j=y0;j<=y1;++j)for(int i=x0;i<=x1;++i){
      double d=std::max(std::hypot(i-tx,j-ty),1.0);if(d>maxr)continue;
      double pw=t.power/std::pow(d,params.path_loss_exp);
      double bd=calculateBuildingPenetrationDDA(tx,ty,i,j);
      pw*=std::exp(-params.atten_per_unit*bd);
      double diff=0;
      if(bd>0&&!cells[j][i].is_building)
        for(auto &b:lb){
          std::pair<double,double> cs[4]={{b.xmin,b.ymin},{b.xmin,b.ymax},{b.xmax,b.ymin},{b.xmax,b.ymax}};
          for(auto &[cx,cy]:cs){double dc=std::max(std::hypot(cx-tx,cy-ty),1.0);
            double pc=t.power/std::pow(dc,params.path_loss_exp);
            double dd=std::max(std::hypot(i-cx,j-cy),1.0);
            diff+=(pc*params.diff_strength)/std::pow(dd,params.diff_exp);}}
      cells[j][i].signal_strength+=pw+diff;}}

  void resolveNetworkCapacity(double total_cap) {
    double total_req=0;
    for(int j=0;j<height;++j)for(int i=0;i<width;++i){Cell &c=cells[j][i];
      if(c.demand>0&&!c.is_building){double sinr=c.signal_strength/params.noise_floor;
        c.efficiency=std::log2(1.0+sinr);
        if(c.efficiency>0.01)total_req+=c.demand/c.efficiency;}}
    double alloc=(total_req>total_cap)?total_cap/total_req:1.0;
    for(int j=0;j<height;++j)for(int i=0;i<width;++i){Cell &c=cells[j][i];
      if(c.demand>0&&!c.is_building){
        if(c.efficiency>0.01){double req=c.demand/c.efficiency;c.allocated_capacity=req*alloc;
          c.qos_percentage=std::min(100.0,(c.allocated_capacity*c.efficiency/c.demand)*100.0);}
        else{c.qos_percentage=0;c.allocated_capacity=0;}}}}

  double computeScore() const {
    double wq=0,td=0;
    for(int j=0;j<height;++j)for(int i=0;i<width;++i){const Cell &c=cells[j][i];
      if(c.demand>0&&!c.is_building){wq+=c.qos_percentage*c.demand;td+=c.demand;}}
    return td>0?wq/td:0;}
};

// ============================================================
//  Evaluator
// ============================================================
double evaluatePlacement(Grid &grid, const std::vector<PlacedTower> &pl,
                         const std::vector<TowerType> &types, bool fast=true) {
  grid.resetSignals(); double cap=0;
  for(auto &p:pl){grid.applyTowerSignal(p,types[p.type_index],fast);cap+=types[p.type_index].capacity;}
  grid.resolveNetworkCapacity(cap); return grid.computeScore();}


// ############################################################
//  GREEDY + SA ALGORITHMS (A, B, C)
// ############################################################

using SignalCache = std::vector<std::vector<double>>;

void addToCache(SignalCache &c,double tx,double ty,const TowerType &t,const Hyperparameters &p,int W,int H){
  double mr=std::pow(t.power/(p.noise_floor*1.01),1.0/p.path_loss_exp);
  int x0=std::max(0,(int)(tx-mr)),x1=std::min(W-1,(int)(tx+mr));
  int y0=std::max(0,(int)(ty-mr)),y1=std::min(H-1,(int)(ty+mr));
  for(int j=y0;j<=y1;++j)for(int i=x0;i<=x1;++i){
    double d=std::max(std::hypot(i-tx,j-ty),1.0);if(d>mr)continue;
    c[j][i]+=t.power/std::pow(d,p.path_loss_exp);}}

double marginalGain(const Grid &g,const SignalCache &c,double tx,double ty,
                    const TowerType &t,const Hyperparameters &p){
  double gain=0,thr=p.noise_floor*5;
  double mr=std::pow(t.power/(p.noise_floor*1.01),1.0/p.path_loss_exp);
  int x0=std::max(0,(int)(tx-mr)),x1=std::min(g.getWidth()-1,(int)(tx+mr));
  int y0=std::max(0,(int)(ty-mr)),y1=std::min(g.getHeight()-1,(int)(ty+mr));
  for(int j=y0;j<=y1;++j)for(int i=x0;i<=x1;++i){
    if(g.isBuilding(i,j))continue;double dem=g.getDemand(i,j);if(dem<=0)continue;
    double d=std::max(std::hypot(i-tx,j-ty),1.0);if(d>mr)continue;
    double add=t.power/std::pow(d,p.path_loss_exp);double bef=c[j][i],aft=bef+add;
    double cg=std::log2(1+aft/p.noise_floor)-std::log2(1+bef/p.noise_floor);
    gain+=dem*cg*((bef<thr)?2.0:1.0);}
  return gain;}

void greedyFill(Grid &g,const std::vector<TowerType> &types,const Hyperparameters &p,
                const std::vector<std::pair<int,int>> &cands,double budget,AlgorithmResult &r){
  SignalCache cache(g.getHeight(),std::vector<double>(g.getWidth(),0));
  std::set<std::pair<int,int>> placed;
  for(auto &pt:r.placements){addToCache(cache,pt.x,pt.y,types[pt.type_index],p,g.getWidth(),g.getHeight());
    placed.insert({(int)pt.x,(int)pt.y});}
  while(r.total_cost<budget){
    double bs=0;PlacedTower bp{};bool found=false;
    for(auto &[cx,cy]:cands){if(!g.canPlaceTower(cx,cy)||placed.count({cx,cy}))continue;
      for(int t=0;t<(int)types.size();++t){if(types[t].cost>budget-r.total_cost)continue;
        double s=marginalGain(g,cache,cx,cy,types[t],p)/types[t].cost;
        if(s>bs){bs=s;bp={(double)cx,(double)cy,t};found=true;}}}
    if(!found)break;
    r.placements.push_back(bp);r.total_cost+=types[bp.type_index].cost;
    placed.insert({(int)bp.x,(int)bp.y});
    addToCache(cache,bp.x,bp.y,types[bp.type_index],p,g.getWidth(),g.getHeight());}}

std::pair<int,int> snapToValid(const Grid &g,double fx,double fy){
  int cx=std::clamp((int)std::round(fx),0,g.getWidth()-1);
  int cy=std::clamp((int)std::round(fy),0,g.getHeight()-1);
  if(g.canPlaceTower(cx,cy))return{cx,cy};
  for(int r=1;r<25;++r)for(int dj=-r;dj<=r;++dj)for(int di=-r;di<=r;++di){
    int nx=cx+di,ny=cy+dj;
    if(nx>=0&&nx<g.getWidth()&&ny>=0&&ny<g.getHeight()&&g.canPlaceTower(nx,ny))return{nx,ny};}
  return{cx,cy};}

AlgorithmResult simulatedAnnealing(Grid &grid,const std::vector<TowerType> &types,
    AlgorithmResult init,double budget,int iters,double T0,double T1,int nr,bool fast,std::mt19937 &rng){
  auto cur=init;cur.score=evaluatePlacement(grid,cur.placements,types,fast);auto best=cur;
  std::uniform_real_distribution<> uni(0,1);
  std::uniform_int_distribution<> xd(0,grid.getWidth()-1),yd(0,grid.getHeight()-1),nd(-nr,nr),td(0,(int)types.size()-1);
  double lr=std::log(T1/T0);
  for(int it=0;it<iters;++it){double T=T0*std::exp(lr*it/iters);auto cand=cur;double roll=uni(rng);
    if(cand.placements.empty()||roll<0.15){for(int t=0;t<20;++t){int nx=xd(rng),ny=yd(rng),nt=td(rng);
        if(!grid.canPlaceTower(nx,ny))continue;if(cand.total_cost+types[nt].cost>budget)continue;
        cand.placements.push_back({(double)nx,(double)ny,nt});cand.total_cost+=types[nt].cost;break;}}
    else if(roll<0.25){std::uniform_int_distribution<> id(0,(int)cand.placements.size()-1);int i=id(rng);
      cand.total_cost-=types[cand.placements[i].type_index].cost;cand.placements.erase(cand.placements.begin()+i);}
    else if(roll<0.40){std::uniform_int_distribution<> id(0,(int)cand.placements.size()-1);int i=id(rng),nt=td(rng);
      double d=types[nt].cost-types[cand.placements[i].type_index].cost;
      if(cand.total_cost+d<=budget){cand.total_cost+=d;cand.placements[i].type_index=nt;}}
    else{std::uniform_int_distribution<> id(0,(int)cand.placements.size()-1);int i=id(rng);
      int nx=std::clamp((int)cand.placements[i].x+nd(rng),0,grid.getWidth()-1);
      int ny=std::clamp((int)cand.placements[i].y+nd(rng),0,grid.getHeight()-1);
      if(grid.canPlaceTower(nx,ny)){cand.placements[i].x=nx;cand.placements[i].y=ny;}}
    if(cand.placements.empty())continue;
    cand.score=evaluatePlacement(grid,cand.placements,types,fast);
    double delta=cand.score-cur.score;
    if(delta>0||uni(rng)<std::exp(delta/T)){cur=cand;if(cur.score>best.score)best=cur;}}
  return best;}

AlgorithmResult algorithmA_WindowSA(Grid &grid,const std::vector<TowerType> &types,
    const Hyperparameters &hp,double budget,bool fast,std::mt19937 &rng){
  AlgorithmResult r;r.name="A_WindowSA";
  int W=grid.getWidth(),H=grid.getHeight(),ww=W/4,wh=H/4;
  int sx=(int)(ww*0.6),sy=(int)(wh*0.6);
  int nw=(W+sx-1)/sx,nh=(H+sy-1)/sy;double pwb=budget/(nw*nh);
  std::set<std::pair<int,int>> seen;
  for(int wy=0;wy<H;wy+=sy)for(int wx=0;wx<W;wx+=sx){
    int x1=std::min(wx+ww,W),y1=std::min(wy+wh,H);
    std::vector<std::pair<int,int>> c;for(int j=wy;j<y1;++j)for(int i=wx;i<x1;++i)c.push_back({i,j});
    AlgorithmResult w;w.name=r.name;greedyFill(grid,types,hp,c,pwb,w);
    for(auto &p:w.placements){auto k=std::make_pair((int)p.x,(int)p.y);
      if(seen.count(k))continue;if(r.total_cost+types[p.type_index].cost>budget)continue;
      seen.insert(k);r.placements.push_back(p);r.total_cost+=types[p.type_index].cost;}}
  r=simulatedAnnealing(grid,types,r,budget,500,5.0,0.02,15,fast,rng);r.name="A_WindowSA";return r;}

AlgorithmResult algorithmB_GlobalGreedySA(Grid &grid,const std::vector<TowerType> &types,
    const Hyperparameters &hp,double budget,bool fast,std::mt19937 &rng){
  AlgorithmResult r;r.name="B_GlobalGreedy";
  std::vector<std::pair<int,int>> c;for(int j=0;j<grid.getHeight();j+=8)for(int i=0;i<grid.getWidth();i+=8)c.push_back({i,j});
  greedyFill(grid,types,hp,c,budget,r);
  r=simulatedAnnealing(grid,types,r,budget,600,3.0,0.01,20,fast,rng);r.name="B_GlobalGreedy";return r;}

AlgorithmResult algorithmC_ClusterGreedySA(Grid &grid,const std::vector<TowerType> &types,
    const Hyperparameters &hp,double budget,bool fast,std::mt19937 &rng){
  AlgorithmResult r;r.name="C_ClusterGreedy";
  double ac=0;for(auto &t:types)ac+=t.cost;ac/=types.size();
  int k=std::max(4,(int)(budget/(ac*2)));
  // k-means
  std::vector<std::pair<int,int>> pts;
  for(int j=0;j<grid.getHeight();++j)for(int i=0;i<grid.getWidth();++i)
    if(!grid.isBuilding(i,j)&&grid.getDemand(i,j)>0)pts.push_back({i,j});
  if(pts.empty())return r;
  k=std::min(k,(int)pts.size());std::shuffle(pts.begin(),pts.end(),rng);
  std::vector<std::pair<double,double>> cen(k);
  for(int i=0;i<k;++i)cen[i]={(double)pts[i].first,(double)pts[i].second};
  for(int it=0;it<20;++it){
    std::vector<double> wx(k,0),wy(k,0),wt(k,0);
    for(auto &[px,py]:pts){double bd=1e18;int bk=0;
      for(int c=0;c<k;++c){double d=std::hypot(px-cen[c].first,py-cen[c].second);if(d<bd){bd=d;bk=c;}}
      double w=grid.getDemand(px,py);wx[bk]+=px*w;wy[bk]+=py*w;wt[bk]+=w;}
    for(int c=0;c<k;++c)if(wt[c]>0)cen[c]={wx[c]/wt[c],wy[c]/wt[c]};}
  int ch=0;for(int t=1;t<(int)types.size();++t)if(types[t].cost<types[ch].cost)ch=t;
  for(auto &[cx,cy]:cen){if(r.total_cost+types[ch].cost>budget)break;
    auto [sx,sy]=snapToValid(grid,cx,cy);r.placements.push_back({(double)sx,(double)sy,ch});r.total_cost+=types[ch].cost;}
  std::vector<std::pair<int,int>> c;for(int j=0;j<grid.getHeight();j+=5)for(int i=0;i<grid.getWidth();i+=5)
    if(grid.getDemand(i,j)>0)c.push_back({i,j});
  greedyFill(grid,types,hp,c,budget,r);
  r=simulatedAnnealing(grid,types,r,budget,600,4.0,0.01,20,fast,rng);r.name="C_ClusterGreedy";return r;}


// ############################################################
//  CSP SOLVER (FC + AC3 + MRV + LCV + Backtracking)
// ############################################################

struct TowerKey {
  int px,py,ti;
  bool operator==(const TowerKey &o)const{return px==o.px&&py==o.py&&ti==o.ti;}
  bool operator<(const TowerKey &o)const{if(px!=o.px)return px<o.px;if(py!=o.py)return py<o.py;return ti<o.ti;}
};
struct DemandCell{int x,y;double demand;};
struct CSPProblem{
  std::vector<DemandCell> cells;std::vector<TowerKey> towers;
  std::map<TowerKey,std::set<int>> covers;std::vector<std::set<TowerKey>> domains;};
struct CSPResult{bool solved;double cost;int nodes,bt;std::map<int,TowerKey> assignment;};

double towerSignal(int tx,int ty,int cx,int cy,const TowerType &t,const Hyperparameters &hp,
                   const std::vector<std::vector<bool>>&bl,int R,int C){
  double d=std::max(std::hypot(cx-tx,cy-ty),1.0),pw=t.power/std::pow(d,hp.path_loss_exp);
  double dx=cx-tx,dy=cy-ty,len=std::hypot(dx,dy),bd=0;
  if(len>0.1){int st=(int)(len*3)+1;for(int s=0;s<=st;++s){double f=(double)s/st;
    int si=(int)std::floor(tx+dx*f),sj=(int)std::floor(ty+dy*f);
    if(si>=0&&si<C&&sj>=0&&sj<R&&bl[sj][si])bd+=len/st;}}
  pw*=std::exp(-hp.atten_per_unit*bd);
  double diff=0;
  if(bd>0&&!(cy>=0&&cy<R&&cx>=0&&cx<C&&bl[cy][cx])){
    int d4x[4]={1,-1,0,0},d4y[4]={0,0,1,-1};
    for(int bj=0;bj<R;++bj)for(int bi=0;bi<C;++bi){if(!bl[bj][bi])continue;
      bool edge=false;for(int k=0;k<4;++k){int nx=bi+d4x[k],ny=bj+d4y[k];
        if(nx<0||nx>=C||ny<0||ny>=R||!bl[ny][nx]){edge=true;break;}}
      if(!edge)continue;
      double dc=std::max(std::hypot(bi-tx,bj-ty),1.0),pc=t.power/std::pow(dc,hp.path_loss_exp);
      double dd=std::max(std::hypot(cx-bi,cy-bj),1.0);
      diff+=(pc*hp.diff_strength)/std::pow(dd,hp.diff_exp);}}
  return pw+diff;}

CSPProblem buildCSP(int R,int C,const std::vector<std::vector<double>>&dem,
    const std::vector<std::vector<bool>>&bl,const std::vector<TowerType>&types,
    const std::vector<std::pair<int,int>>&tpos,const Hyperparameters &hp){
  CSPProblem p;
  for(int j=0;j<R;++j)for(int i=0;i<C;++i)if(dem[j][i]>0&&!bl[j][i])p.cells.push_back({i,j,dem[j][i]});
  int n=(int)p.cells.size();p.domains.resize(n);double ms=hp.noise_floor*5;
  for(auto &[px,py]:tpos){if(bl[py][px])continue;
    for(int t=0;t<(int)types.size();++t){TowerKey tk{px,py,t};std::set<int> cov;
      for(int c=0;c<n;++c){double sig=towerSignal(px,py,p.cells[c].x,p.cells[c].y,types[t],hp,bl,R,C);
        if(sig>=ms)cov.insert(c);}
      if(!cov.empty()){p.covers[tk]=cov;p.towers.push_back(tk);for(int c:cov)p.domains[c].insert(tk);}}}
  // Dominance pruning
  std::set<TowerKey> dom;
  for(int i=0;i<(int)p.towers.size();++i){auto &a=p.towers[i];if(dom.count(a))continue;
    for(int j=0;j<(int)p.towers.size();++j){if(i==j)continue;auto &b=p.towers[j];if(dom.count(b))continue;
      if(std::includes(p.covers[b].begin(),p.covers[b].end(),p.covers[a].begin(),p.covers[a].end())&&
         types[a.ti].cost>=types[b.ti].cost&&!(a==b)){dom.insert(a);break;}}}
  for(auto &d:dom){p.covers.erase(d);for(auto &dm:p.domains)dm.erase(d);}
  p.towers.erase(std::remove_if(p.towers.begin(),p.towers.end(),[&](const TowerKey&t){return dom.count(t);}),p.towers.end());
  return p;}

class CSPSolver{
  const CSPProblem &prob;const std::vector<TowerType>&types;
  bool useMRV,useLCV,useFC,useAC3;
  std::vector<std::set<TowerKey>> domains;std::map<TowerKey,double> remCap;
  std::map<int,TowerKey> assign;int nodes,bt;double bestCost;
  std::map<int,TowerKey> bestAssign;bool found,timedOut;
  std::chrono::steady_clock::time_point deadline;

  int pickVar(const std::vector<int>&u){if(!useMRV)return u[0];
    int b=u[0],bs=(int)domains[b].size();for(int c:u){int s=(int)domains[c].size();if(s<bs){bs=s;b=c;}}return b;}

  std::vector<TowerKey> orderVals(int cid,const std::vector<int>&u){
    std::vector<TowerKey> ord(domains[cid].begin(),domains[cid].end());
    if(!useLCV||ord.size()<=1)return ord;double md=prob.cells[cid].demand;
    std::vector<int> imp(ord.size(),0);
    for(int v=0;v<(int)ord.size();++v){double nc=remCap[ord[v]]-md;
      for(int o:u){if(o==cid)continue;if(domains[o].count(ord[v])&&nc<prob.cells[o].demand)imp[v]++;}}
    std::vector<int> idx(ord.size());std::iota(idx.begin(),idx.end(),0);
    std::sort(idx.begin(),idx.end(),[&](int a,int b){return imp[a]<imp[b];});
    std::vector<TowerKey> r(ord.size());for(int i=0;i<(int)idx.size();++i)r[i]=ord[idx[i]];return r;}

  void fc(int cell,const TowerKey &tk,const std::vector<int>&u,bool &wipe){wipe=false;
    for(int o:u){if(o==cell||!domains[o].count(tk))continue;
      if(remCap[tk]<prob.cells[o].demand){domains[o].erase(tk);if(domains[o].empty()){wipe=true;return;}}}}

  void arc3(const std::vector<int>&u,bool &wipe){wipe=false;
    std::set<int> us(u.begin(),u.end());std::map<TowerKey,std::vector<int>> t2c;
    for(int c:u)for(auto &tk:domains[c])t2c[tk].push_back(c);
    struct A{int i,j;TowerKey t;};std::deque<A> q;
    for(auto &[tk,cl]:t2c)for(int i=0;i<(int)cl.size();++i)for(int j=i+1;j<(int)cl.size();++j)
      {q.push_back({cl[i],cl[j],tk});q.push_back({cl[j],cl[i],tk});}
    while(!q.empty()){auto [ci,cj,tk]=q.front();q.pop_front();
      if(!us.count(ci)||!us.count(cj)||!domains[ci].count(tk))continue;
      double cb=remCap[tk]-prob.cells[ci].demand-prob.cells[cj].demand;
      if(cb<0){auto alt=domains[cj];alt.erase(tk);if(alt.empty()){domains[ci].erase(tk);
        if(domains[ci].empty()){wipe=true;return;}
        for(auto &tk2:domains[ci])for(int nb:t2c[tk2])if(nb!=ci&&us.count(nb))q.push_back({nb,ci,tk2});}}}}

  void solve(std::vector<int>&u,double cost){
    if(std::chrono::steady_clock::now()>deadline){timedOut=true;return;}if(timedOut)return;
    if(u.empty()){if(cost<bestCost){bestCost=cost;bestAssign=assign;found=true;}return;}
    if(cost>=bestCost)return;for(int c:u)if(domains[c].empty()){bt++;return;}
    int cell=pickVar(u);auto it=std::find(u.begin(),u.end(),cell);std::swap(*it,u.back());u.pop_back();
    auto ord=orderVals(cell,u);
    for(auto &tk:ord){if(timedOut)break;bool reused=false;
      for(auto &[a,at]:assign)if(at==tk){reused=true;break;}
      double add=reused?0:types[tk.ti].cost;if(cost+add>=bestCost)continue;
      assign[cell]=tk;remCap[tk]-=prob.cells[cell].demand;nodes++;
      auto saved=domains;bool wipe=false;
      if(useFC&&!wipe){fc(cell,tk,u,wipe);if(wipe)bt++;}
      if(useAC3&&!wipe){arc3(u,wipe);if(wipe)bt++;}
      if(!wipe)solve(u,cost+add);
      domains=saved;remCap[tk]+=prob.cells[cell].demand;assign.erase(cell);}
    u.push_back(cell);bt++;}
public:
  CSPSolver(const CSPProblem &p,const std::vector<TowerType>&t,bool m,bool l,bool f,bool a)
    :prob(p),types(t),useMRV(m),useLCV(l),useFC(f),useAC3(a){}
  CSPResult run(double budget,int tl=60){
    domains=prob.domains;remCap.clear();
    for(auto &[tk,_]:prob.covers)remCap[tk]=types[tk.ti].capacity;
    assign.clear();nodes=bt=0;bestCost=budget+1;found=false;timedOut=false;
    deadline=std::chrono::steady_clock::now()+std::chrono::seconds(tl);
    std::vector<int> u;for(int i=0;i<(int)prob.cells.size();++i)if(!domains[i].empty())u.push_back(i);
    solve(u,0);return{found,found?bestCost:0,nodes,bt,bestAssign};}
};

AlgorithmResult cspToResult(Grid &g,const CSPResult &cr,const CSPProblem &prob,
    const std::vector<TowerType>&types,const std::string &name){
  AlgorithmResult r;r.name=name;if(!cr.solved)return r;
  std::set<TowerKey> used;for(auto &[c,tk]:cr.assignment)used.insert(tk);
  for(auto &tk:used){r.placements.push_back({(double)tk.px,(double)tk.py,tk.ti});r.total_cost+=types[tk.ti].cost;}
  r.score=evaluatePlacement(g,r.placements,types);r.nodes=cr.nodes;return r;}


// ############################################################
//  HELPERS
// ############################################################
void exportPlacements(const std::string &fn,const AlgorithmResult &r,const std::vector<TowerType>&types){
  std::ofstream f(fn);f<<"x,y,type,name\n";
  for(auto &p:r.placements)f<<p.x<<","<<p.y<<","<<p.type_index<<","<<types[p.type_index].name<<"\n";}

void exportGrid(const std::string &bf,const std::string &df,int R,int C,
    const std::vector<std::vector<double>>&dem,const std::vector<std::vector<bool>>&bl){
  {std::ofstream f(bf);f<<"x,y\n";for(int j=0;j<R;++j)for(int i=0;i<C;++i)if(bl[j][i])f<<i<<","<<j<<"\n";}
  {std::ofstream f(df);f<<"x,y,demand\n";for(int j=0;j<R;++j)for(int i=0;i<C;++i)
    if(dem[j][i]>0&&!bl[j][i])f<<i<<","<<j<<","<<dem[j][i]<<"\n";}}

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


// ############################################################
//  MAIN
// ############################################################
int main()
{
  Hyperparameters hp;
  hp.path_loss_exp=3.5; hp.atten_per_unit=0.20;
  hp.diff_strength=0.60; hp.diff_exp=4.0; hp.noise_floor=1e-4;

  // ============================================================
  //  PART 1: 9x9 Grid — CSP vs SA
  // ============================================================
  
  {// ============================================================
//  PART 1: Randomized 9x9 Grid — CSP vs SA
// ============================================================

    const int R = 20, C = 20; 
    double budget = 25000; // Adjusted budget for small scale
    std::vector<TowerType> types = {
        {"Macro", 0.15, 5000, 30}, 
        {"Micro", 0.02, 2500, 15}, 
        {"Pico", 0.004, 1000, 8}
    };
    std::cout << "============================================================\n"
            << "  Part 1: 10x10 Grid — CSP vs Heuristic Algorithms\n"
            << "============================================================\n\n";
    // Use a fixed seed for Part 1 so you can compare CSP vs SA fairly
    Grid grid(C, R, 42, hp); 

    // --- RANDOM GENERATION FOR SMALL SCALE ---
    // Small buildings (1-2 cells) so we don't block the whole map
    grid.addBuildingsRandomly(1.0, 2.0, 1.0); 
    
    // Higher probability (40%) so the 9x9 doesn't end up empty
    grid.addRandomDemand(10, 0.4); 

    // --- EXTRACT DATA FOR CSP ---
    // The CSP solver needs the state of the grid after randomization
    std::vector<std::vector<double>> random_dem(R, std::vector<double>(C, 0));
    std::vector<std::vector<bool>> random_bl(R, std::vector<bool>(C, false));
    std::vector<std::pair<int, int>> tpos;

    for(int j = 0; j < R; ++j) {
        for(int i = 0; i < C; ++i) {
            random_bl[j][i] = grid.isBuilding(i, j);
            random_dem[j][i] = grid.getDemand(i, j);
            if(!random_bl[j][i]) tpos.push_back({i, j});
        }
    }

    // Build the CSP problem from the randomized state
    CSPProblem csp = buildCSP(R, C, random_dem, random_bl, types, tpos, hp);
    
    std::cout<<"CSP: "<<csp.cells.size()<<" variables, "<<csp.towers.size()<<" candidate towers\n\n";

    std::vector<AlgorithmResult> all;

    // CSP variants
    struct Cfg{std::string n;bool m,l,f,a;};
    for(auto &cfg:std::vector<Cfg>{{"D: Plain BT",false,false,false,false},
        {"D: MRV",true,false,false,false},{"D: MRV+LCV",true,true,false,false}}){
      std::cout<<"Running "<<cfg.n<<" ..."; std::cout.flush();
      CSPSolver s(csp,types,cfg.m,cfg.l,cfg.f,cfg.a);
      auto t0=std::chrono::steady_clock::now();CSPResult cr=s.run(budget,60);
      double sec=std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count();
      AlgorithmResult ar=cspToResult(grid,cr,csp,types,cfg.n);ar.nodes=cr.nodes;ar.time_sec=sec;
      std::cout<<(cr.solved?" SOLVED":" FAIL")<<"  score="<<std::fixed<<std::setprecision(2)<<ar.score
        <<"%  $"<<ar.total_cost<<"  "<<std::setprecision(4)<<sec<<"s\n";
      all.push_back(ar);}

    // SA variants on same small grid
    auto runSA=[&](auto fn,const std::string &label){
      std::mt19937 rng(42);std::cout<<"Running "<<label<<" ...";std::cout.flush();
      auto t0=std::chrono::steady_clock::now();
      AlgorithmResult r=fn(grid,types,hp,budget,true,rng);
      r.time_sec=std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count();r.name=label;
      std::cout<<"  score="<<std::fixed<<std::setprecision(2)<<r.score<<"%  $"<<r.total_cost
        <<"  "<<std::setprecision(4)<<r.time_sec<<"s\n";
      all.push_back(r);};
    runSA(algorithmA_WindowSA,"A: Window+SA");
    runSA(algorithmB_GlobalGreedySA,"B: Greedy+SA");
    runSA(algorithmC_ClusterGreedySA,"C: Cluster+SA");

    std::cout<<"\n"; printTable(all);

    // Export small grid data
    exportGrid("small_buildings.csv","small_demand.csv",R,C,random_dem,random_bl);
    AlgorithmResult *bC=nullptr,*bS=nullptr;
    for(auto &r:all){if(r.name[0]=='D'&&(!bC||r.total_cost<bC->total_cost||(r.total_cost==bC->total_cost&&r.score>bC->score)))bC=&r;
      if(r.name[0]!='D'&&(!bS||r.score>bS->score))bS=&r;}
    if(bC)exportPlacements("small_csp_towers.csv",*bC,types);
    if(bS)exportPlacements("small_sa_towers.csv",*bS,types);

    std::cout<<"\nBest CSP: "<<(bC?bC->name:"-")<<"  score="<<(bC?bC->score:0)<<"%  $"<<(bC?bC->total_cost:0)<<"\n";
    std::cout<<"Best SA:  "<<(bS?bS->name:"-")<<"  score="<<(bS?bS->score:0)<<"%  $"<<(bS?bS->total_cost:0)<<"\n";
    if(bC&&bS&&std::abs(bC->score-bS->score)<0.5)
      std::cout<<"\n>>> MATCH: Both approaches reach the same QoS! <<<\n";
  }

  // ============================================================
  //  PART 2: 300x250 Grid — SA Algorithms
  // ============================================================
  std::cout << "\n\n============================================================\n"
            << "  Part 2: 300x250 Grid — Heuristic Algorithms at Scale\n"
            << "============================================================\n\n";
  {
    Grid grid(300,250,42,hp);
    grid.addBuildingsRandomly(20.0,80.0,15.0);
    grid.addRandomDemand(10);
    grid.setDemand(40,50,100.0);
    grid.setDemand(110,50,100.0);
    grid.setDemand(200,150,80.0);

    std::vector<TowerType> types={{"Macro",600,60000,4000},{"Micro",150,30000,1200},{"Pico",25,12000,300}};
    double budget=400000; bool fast=true;

    int nd=0;double td=0;
    for(int j=0;j<250;++j)for(int i=0;i<300;++i)
      if(!grid.isBuilding(i,j)&&grid.getDemand(i,j)>0){nd++;td+=grid.getDemand(i,j);}
    std::cout<<"Grid: 300x250  Demand cells: "<<nd<<"  Total demand: "<<td<<"\n";
    std::cout<<"Budget: $"<<budget<<"\n";
    std::cout<<"CSP would need "<<nd<<" variables — computationally infeasible.\n\n";

    std::vector<AlgorithmResult> results;
    auto runAlg=[&](auto fn,const std::string &label){
      std::mt19937 rng(42);
      std::cout<<"Running "<<label<<" ...";std::cout.flush();
      auto t0=std::chrono::steady_clock::now();
      AlgorithmResult r=fn(grid,types,hp,budget,fast,rng);
      r.time_sec=std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count();r.name=label;
      std::cout<<"  score="<<std::fixed<<std::setprecision(2)<<r.score
        <<"%  $"<<std::setprecision(0)<<r.total_cost
        <<"  "<<r.placements.size()<<" towers  "
        <<std::setprecision(2)<<r.time_sec<<"s\n";
      results.push_back(r);};

    runAlg(algorithmA_WindowSA,"A_WindowSA");
    runAlg(algorithmB_GlobalGreedySA,"B_GlobalGreedy");
    runAlg(algorithmC_ClusterGreedySA,"C_ClusterGreedy");

    std::cout<<"\n"; printTable(results);

    auto best=std::max_element(results.begin(),results.end(),
      [](const AlgorithmResult &a,const AlgorithmResult &b){return a.score<b.score;});
    std::cout<<"\nWinner: "<<best->name<<" (score "<<std::fixed<<std::setprecision(2)<<best->score<<"%)\n";

    // Export big grid data
    // Build buildings/demand arrays for export
    std::vector<std::vector<bool>> bl2(250,std::vector<bool>(300,false));
    std::vector<std::vector<double>> dem2(250,std::vector<double>(300,0));
    for(int j=0;j<250;++j)for(int i=0;i<300;++i){
      bl2[j][i]=grid.isBuilding(i,j);
      dem2[j][i]=grid.getDemand(i,j);}
    exportGrid("big_buildings.csv","big_demand.csv",250,300,dem2,bl2);
    exportPlacements("big_sa_towers.csv",*best,types);

    // Export each algorithm
    for(auto &r:results)exportPlacements(r.name+"_towers.csv",r,types);
  }

  return 0;
}
