#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace ego_nav2_controller {
struct TurnSafety {
  enum State {Clear, Blocked, Invalid} state{Invalid};
  int64_t map_stamp{};
  std::string reason;
};
// Count distinct, advancing map stamps, not control ticks or duplicate DDS samples.
class TurnObstacleWait {
  bool waiting_{};
  double since_{};
  int64_t last_map_{};
  int clear_maps_{};
public:
  enum Result {Proceed, Hold, Timeout};
  void reset() {waiting_=false;since_=0;last_map_=0;clear_maps_=0;}
  bool waiting() const {return waiting_;}
  Result update(double now,bool clear,int64_t map_stamp,bool stopped) {
    if(!waiting_ && clear)return Proceed;
    if(!waiting_) {waiting_=true;since_=now;last_map_=map_stamp;clear_maps_=0;}
    if(now<since_ || now-since_>=3.0)return Timeout;
    if(!clear || !stopped) {clear_maps_=0;last_map_=std::max(last_map_,map_stamp);return Hold;}
    if(map_stamp<last_map_) {clear_maps_=0;return Hold;}
    if(map_stamp>last_map_) {last_map_=map_stamp;++clear_maps_;}
    if(clear_maps_<2)return Hold;
    waiting_=false;clear_maps_=0;return Proceed;
  }
};
using TurnPoint=std::array<double,2>;
struct TurnGrid {
  double resolution{},origin_x{},origin_y{};
  unsigned width{},height{};
  const std::vector<unsigned char> *cells{};
};
// Convex published footprint against an occupied cell square. Expanding the
// square by sweep_error conservatively covers motion between angular samples.
inline bool turnPolygonCell(const std::vector<TurnPoint> &poly,TurnPoint cell,double half) {
  std::vector<TurnPoint> axes{{1,0},{0,1}};
  for(size_t i=0;i<poly.size();++i) {
    const auto &a=poly[i],&b=poly[(i+1)%poly.size()];
    axes.push_back({-(b[1]-a[1]),b[0]-a[0]});
  }
  for(const auto &axis:axes) {
    double lo=1e100,hi=-1e100;
    for(const auto &p:poly) {double v=p[0]*axis[0]+p[1]*axis[1];lo=std::min(lo,v);hi=std::max(hi,v);}
    const double mid=cell[0]*axis[0]+cell[1]*axis[1],extent=half*(std::abs(axis[0])+std::abs(axis[1]));
    if(hi<mid-extent || lo>mid+extent)return false;
  }
  return true;
}
inline bool validTurnFootprint(const std::vector<TurnPoint> &footprint) {
  if(footprint.size()<3)return false;
  double sign=0;
  for(size_t i=0;i<footprint.size();++i) {
    auto a=footprint[i],b=footprint[(i+1)%footprint.size()],c=footprint[(i+2)%footprint.size()];
    if(!std::isfinite(a[0])||!std::isfinite(a[1]))return false;
    double cross=(b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]);
    if(std::abs(cross)<1e-10)return false;
    if(sign*cross<0)return false;
    sign=cross;
  }
  return true;
}
// Check the requested angular arc and its continuum, with the actual padded
// footprint and a 0.60 m disk at base_link (0.25 m ahead of the drive axle).
inline TurnSafety::State checkTurnSweep(const TurnGrid &grid,const std::vector<TurnPoint> &footprint,
                                       double x,double y,double yaw,double angle) {
  if(!validTurnFootprint(footprint) || !grid.cells || grid.resolution<=0 ||
     !std::isfinite(grid.resolution) || !std::isfinite(grid.origin_x)||!std::isfinite(grid.origin_y)||
     grid.width==0 || grid.height==0 || grid.cells->size()!=size_t(grid.width)*grid.height ||
     !std::isfinite(x)||!std::isfinite(y)||!std::isfinite(yaw)||!std::isfinite(angle))return TurnSafety::Invalid;
  double radius=.85;
  for(auto p:footprint)radius=std::max(radius,std::hypot(p[0],p[1]));
  // Angles beyond a complete revolution have the same spatial sweep.
  angle=std::clamp(angle,-2*M_PI,2*M_PI);
  const int count=std::max(1,int(std::ceil(std::abs(angle)/std::min(.02,grid.resolution/(2*radius)))));
  const double step=std::abs(angle)/count,error=radius*step*.5+1e-9;
  std::vector<TurnPoint> obstacles;
  int xmin=std::max(0,int(std::floor((x-radius-error-grid.origin_x)/grid.resolution)));
  int xmax=std::min(int(grid.width)-1,int(std::floor((x+radius+error-grid.origin_x)/grid.resolution)));
  int ymin=std::max(0,int(std::floor((y-radius-error-grid.origin_y)/grid.resolution)));
  int ymax=std::min(int(grid.height)-1,int(std::floor((y+radius+error-grid.origin_y)/grid.resolution)));
  for(int j=ymin;j<=ymax;++j)for(int i=xmin;i<=xmax;++i)
    if((*grid.cells)[j*grid.width+i]>=254)
      obstacles.push_back({grid.origin_x+(i+.5)*grid.resolution,grid.origin_y+(j+.5)*grid.resolution});
  for(int sample=0;sample<=count;++sample) {
    const double a=yaw+angle*sample/count,c=std::cos(a),s=std::sin(a);
    std::vector<TurnPoint> poly;
    for(auto p:footprint)poly.push_back({x+c*p[0]-s*p[1],y+s*p[0]+c*p[1]});
    TurnPoint center{x+.25*c,y+.25*s};const double center_radius=.60+.25*step*.5+1e-9;
    for(auto p:poly)if(p[0]-error<grid.origin_x || p[1]-error<grid.origin_y ||
      p[0]+error>=grid.origin_x+grid.width*grid.resolution ||
      p[1]+error>=grid.origin_y+grid.height*grid.resolution)return TurnSafety::Blocked;
    if(center[0]-center_radius<grid.origin_x || center[1]-center_radius<grid.origin_y ||
       center[0]+center_radius>=grid.origin_x+grid.width*grid.resolution ||
       center[1]+center_radius>=grid.origin_y+grid.height*grid.resolution)return TurnSafety::Blocked;
    for(auto cell:obstacles) {
      const double dx=std::max(0.,std::abs(cell[0]-center[0])-grid.resolution*.5);
      const double dy=std::max(0.,std::abs(cell[1]-center[1])-grid.resolution*.5);
      if(std::hypot(dx,dy)<=center_radius || turnPolygonCell(poly,cell,grid.resolution*.5+error))
        return TurnSafety::Blocked;
    }
  }
  return TurnSafety::Clear;
}
}
