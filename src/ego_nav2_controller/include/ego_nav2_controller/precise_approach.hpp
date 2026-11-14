#pragma once
#include <algorithm>
#include <cmath>
#include <deque>
#include <stdexcept>
#include <string>

namespace ego_nav2_controller {
// Forward-only terminal maneuver. Positional turns latch their heading; once
// XY is captured, complete final yaw before considering another translation.
class PreciseApproach {
  enum Phase {Idle, Settle, Face, Drive, Align, Done};
  Phase phase_{Idle};
  double since_{}, quiet_{-1}, last_yaw_{}, turned_{}, target_turn_{}, heading_{}, best_{}, drive_since_{};
  double diverging_since_{-1};
  int attempts_{};
  bool captured_{};
  struct Sample {double t,dx,dy;};
  std::deque<Sample> stationary_, moving_;
  std::string failure_;
  static double wrap(double a) {return std::atan2(std::sin(a),std::cos(a));}
public:
  struct Command {double v{},w{};};
  void reset() {phase_=Idle;quiet_=-1;attempts_=0;captured_=false;stationary_.clear();moving_.clear();failure_.clear();}
  bool active() const {return phase_!=Idle;}
  const char *phaseName() const {
    switch(phase_) {case Idle:return "idle";case Settle:return "settle";case Face:return "face_position";
      case Drive:return "drive";case Align:return "align_final_yaw";case Done:return "done";}
    return "unknown";
  }
  int corrections() const {return attempts_;}
  Command step(double now,double x,double y,double yaw,double gx,double gy,double ga,
               double measured_v,double measured_w,double xy_tol,double yaw_tol) {
    const double distance=std::hypot(gx-x,gy-y);
    if(!failure_.empty())throw std::runtime_error(failure_);
    auto fail=[&](const char *reason) {
      failure_=std::string(reason)+"; phase="+phaseName()+"; xy="+std::to_string(distance)+
        " m; yaw="+std::to_string(wrap(ga-yaw))+" rad; translations="+std::to_string(attempts_);
      throw std::runtime_error(failure_);
    };
    if(phase_==Idle){phase_=Settle;since_=now;}
    if(now<since_ || now-since_>90 || distance>.60)
      fail("Precise start alignment exceeded time/distance bound");
    const bool stopped=std::abs(measured_v)<.04 && std::abs(measured_w)<.03;
    moving_.push_back({now,gx-x,gy-y});
    while(moving_.size()>1 && now-moving_.front().t>.3)moving_.pop_front();
    double moving_dx=0,moving_dy=0;
    for(const auto &s:moving_){moving_dx+=s.dx;moving_dy+=s.dy;}
    const double moving_distance=std::hypot(moving_dx,moving_dy)/moving_.size();
    // Only use a short stationary window to choose a new positional bearing.
    // Command collision checks and goal acceptance still use the actual pose.
    if(!stopped || phase_==Drive)stationary_.clear();
    else {
      stationary_.push_back({now,gx-x,gy-y});
      while(stationary_.size()>1 && now-stationary_.front().t>.6)stationary_.pop_front();
    }
    double dx=gx-x,dy=gy-y;
    if(!stationary_.empty()) {
      dx=dy=0;
      for(const auto &s:stationary_){dx+=s.dx;dy+=s.dy;}
      dx/=stationary_.size();dy/=stationary_.size();
    }
    const double settled_distance=std::hypot(dx,dy);
    auto quiet=[&]() {
      if(!stopped){quiet_=-1;return false;}
      if(quiet_<0)quiet_=now;
      return now-quiet_>=.4;
    };
    auto settle=[&]() {phase_=Settle;quiet_=-1;};
    if(moving_distance<=xy_tol && phase_==Drive) {
      captured_=true;
      settle();return {};
    }
    if(phase_==Done) {
      if(distance<=xy_tol && std::abs(wrap(ga-yaw))<=yaw_tol)return {};
      phase_=Align;quiet_=-1;
    }
    if(phase_==Settle) {
      if(!quiet())return {};
      last_yaw_=yaw;turned_=0;quiet_=-1;
      if(captured_ || settled_distance<=xy_tol) {phase_=Align;captured_=true;}
      else {
        heading_=std::atan2(dy,dx);target_turn_=wrap(heading_-yaw);phase_=Face;
      }
    }
    if(phase_==Face || phase_==Align) {
      turned_+=wrap(yaw-last_yaw_);last_yaw_=yaw;
      // Final yaw follows the actual goal orientation, including map updates.
      // Its deadband must be smaller than the goal checker's yaw tolerance.
      const double error=phase_==Align?wrap(ga-yaw):target_turn_-turned_;
      const double deadband=phase_==Align?std::min(.02,yaw_tol*.5):.045;
      if(std::abs(error)>deadband) {
        quiet_=-1;
        return {0,std::clamp(.65*error-.35*measured_w,-.20,.20)};
      }
      if(!quiet())return {};
      quiet_=-1;
      if(phase_==Align) {
        if(settled_distance<=xy_tol) {
          if(distance<=xy_tol && std::abs(wrap(ga-yaw))<=yaw_tol)phase_=Done;
          return {};
        }
        captured_=false;settle();return {};
      }
      if(settled_distance<=xy_tol){captured_=true;settle();return {};}
      const double direction=wrap(std::atan2(dy,dx)-yaw);
      if(std::abs(direction)>.20){settle();return {};}
      // Count actual translations, not a noisy bearing recheck while stopped.
      if(attempts_>=3)fail("Precise start alignment exhausted three position corrections");
      ++attempts_;
      phase_=Drive;heading_=yaw;best_=distance;drive_since_=now;
      diverging_since_=-1;
    }
    if(phase_==Drive) {
      best_=std::min(best_,distance);
      const double along=(gx-x)*std::cos(heading_)+(gy-y)*std::sin(heading_);
      if(distance>best_+.035) {if(diverging_since_<0)diverging_since_=now;}
      else diverging_since_=-1;
      if(along<=.01 || (diverging_since_>=0 && now-diverging_since_>.5) || now-drive_since_>12.) {
        settle();return {};
      }
      const double speed=std::clamp(.7*(distance-xy_tol*.5),.025,.06);
      return {speed,std::clamp(.9*wrap(heading_-yaw)-.25*measured_w,-.08,.08)};
    }
    return {};
  }
};
}
