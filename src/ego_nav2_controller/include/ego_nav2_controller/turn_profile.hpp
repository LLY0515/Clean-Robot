#pragma once
#include <algorithm>
#include <cmath>

namespace ego_nav2_controller {
// A spatial heading servo, with a latency-aware braking envelope. The command
// ramp is independent of odometry noise; completion requires fresh measurements.
struct TurnSettings {
  double max_speed{0.30}, min_speed{0.10}, acceleration{0.40}, reaction_time{0.20};
  double yaw_tolerance{0.03490658503988659}, stopped_speed{0.03}, settle_time{0.20};
};
class TurnProfile {
  TurnSettings settings_;
  double command_{}, quiet_since_{-1}, last_sample_{-1};
  bool holding_{};
public:
  explicit TurnProfile(TurnSettings settings = {}) : settings_(settings) {}
  void reset(double measured_speed = 0) {
    command_=std::clamp(measured_speed,-settings_.max_speed,settings_.max_speed);
    quiet_since_=last_sample_=-1;holding_=false;
  }
  struct Output {double speed;bool settled;};
  Output update(double error,double measured_speed,double dt,double sample_time) {
    const double magnitude=std::abs(error);
    // Enter a smaller deadband than the completion tolerance. A drift out of
    // tolerance resumes correction rather than getting stuck in a latched stop.
    if(magnitude<=settings_.yaw_tolerance*.5 ||
       (magnitude<=settings_.yaw_tolerance && std::abs(measured_speed)<=settings_.stopped_speed))holding_=true;
    if(magnitude>settings_.yaw_tolerance)holding_=false;
    const double available=magnitude;
    const double ar=settings_.acceleration*settings_.reaction_time;
    const double braking_speed=std::sqrt(ar*ar+2*settings_.acceleration*available)-ar;
    const double floor=magnitude>settings_.yaw_tolerance?settings_.min_speed:0.;
    double desired=holding_?0:std::copysign(std::min({settings_.max_speed,
      std::max(floor,1.5*magnitude),braking_speed}),error);
    // Do not drive farther into the target while measured motion is already
    // sufficient to reach it through delay and braking.
    const double stopping=std::abs(measured_speed)*settings_.reaction_time+
      measured_speed*measured_speed/(2*settings_.acceleration);
    if(error*measured_speed>0 && stopping>=available)desired=0;
    command_=std::clamp(desired,command_-settings_.acceleration*dt,command_+settings_.acceleration*dt);
    const bool quiet=magnitude<=settings_.yaw_tolerance &&
      std::abs(measured_speed)<=settings_.stopped_speed && std::abs(command_)<1e-6;
    if(!quiet || sample_time<last_sample_ || sample_time-last_sample_>.300001)quiet_since_=-1;
    if(quiet && sample_time>last_sample_ && quiet_since_<0)quiet_since_=sample_time;
    last_sample_=sample_time;
    return {command_,quiet && quiet_since_>=0 && sample_time-quiet_since_>=settings_.settle_time};
  }
};
}
