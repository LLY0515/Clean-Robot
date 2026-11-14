#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

namespace ego_nav2_controller {
inline double brakingDistance(double from,double to,double decel,double reaction,double margin) {
  if(from<=to)return 0;
  return (from*from-to*to)/(2*decel)+from*reaction+margin;
}
// The highest current speed which can reach the local limit before the zone.
inline double approachSpeed(double limit,double distance,double decel,double reaction,double margin) {
  const double d=std::max(0.0,distance-margin),ar=decel*reaction;
  return std::max(limit,std::sqrt(ar*ar+limit*limit+2*decel*d)-ar);
}
struct SpeedEnvelope {double cap;bool slow_now{false};};
template<class Point,class SlowZone>
SpeedEnvelope previewEnvelope(const std::vector<Point> &path,size_t nearest,double preview,
  double lookahead,double maximum,double slow_speed,double max_w,double lateral_accel,
  double curvature_threshold,double decel,double reaction,double margin,SlowZone slow_zone) {
  SpeedEnvelope result{maximum,false};double travel=0;
  for(size_t i=nearest;i<path.size();++i) {
    if(i>nearest)travel+=std::hypot(path[i].x-path[i-1].x,path[i].y-path[i-1].y);
    if(travel>preview)break;
    double k=0;
    if(i>0 && i+1<path.size()) {
      const double ax=path[i].x-path[i-1].x,ay=path[i].y-path[i-1].y;
      const double bx=path[i+1].x-path[i].x,by=path[i+1].y-path[i].y;
      const double denom=std::hypot(ax,ay)*std::hypot(bx,by)*std::hypot(ax+bx,ay+by);
      if(denom>1e-9)k=2*std::abs(ax*by-ay*bx)/denom;
    }
    double local=maximum;
    if(k>=curvature_threshold || slow_zone(path[i]))local=std::min(local,slow_speed);
    if(k>1e-6)local=std::min({local,max_w/k,std::sqrt(lateral_accel/k)});
    // Pursuit begins steering toward the bend before physically reaching it.
    const double available=std::max(0.0,travel-lookahead);
    if(local<maximum) {
      result.cap=std::min(result.cap,approachSpeed(local,available,decel,reaction,margin));
      if(available<=1e-6 && local<=slow_speed)result.slow_now=true;
    }
  }
  return result;
}
}  // namespace ego_nav2_controller
