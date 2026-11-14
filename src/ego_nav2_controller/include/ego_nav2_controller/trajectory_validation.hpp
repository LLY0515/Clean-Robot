#pragma once
#include <algorithm>
#include <cmath>

namespace ego_nav2_controller {
// Piecewise linear position / shortest-angle interpolation. The extra margin
// bounds every body point's displacement to its nearest sampled pose, so a
// collision-free sample set certifies the entire interpolated segment.
// collision(x,y,yaw,margin) must test occupied cell AREAS, unknown and map edges.
template<class Collision, class Deadline>
bool safeMotionSegment(double ax,double ay,double aa,double bx,double by,double ba,
                       double radius,double resolution,Collision collision,Deadline deadline) {
  if(!std::isfinite(ax)||!std::isfinite(ay)||!std::isfinite(aa)||
     !std::isfinite(bx)||!std::isfinite(by)||!std::isfinite(ba)||
     !std::isfinite(radius)||radius<=0||!std::isfinite(resolution)||resolution<=0)return false;
  const double angle=std::atan2(std::sin(ba-aa),std::cos(ba-aa));
  const double travel=std::hypot(bx-ax,by-ay)+radius*std::abs(angle);
  // Bound malformed/huge input before converting to an integer.
  const double count=std::ceil(travel/(resolution*.5));
  if(!std::isfinite(count)||count>100000)return false;
  const int steps=std::max(1,int(count));
  const double margin=travel/(2*steps)+1e-9;
  for(int i=0;i<=steps;++i) {
    deadline();
    const double f=double(i)/steps;
    if(collision(ax+(bx-ax)*f,ay+(by-ay)*f,aa+angle*f,margin))return false;
  }
  return true;
}
}
