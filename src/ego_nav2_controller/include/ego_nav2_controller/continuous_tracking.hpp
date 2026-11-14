#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
#include <deque>
#include "ego_nav2_controller/speed_envelope.hpp"

namespace ego_nav2_controller {
inline double trackingAngle(double a) {return std::atan2(std::sin(a),std::cos(a));}
// A continuous transition avoids the old 2.5 cm / 0.03 rad speed cliff.
// These are speed thresholds only; raw-pose stop limits remain 8 cm / 0.10 rad.
inline double continuousTrackingSpeedCap(double maximum,double lateral,double heading,
                                         double decel,double reaction) {
  auto blend=[](double value,double free,double slow) {
    double t=std::clamp((std::abs(value)-free)/(slow-free),0.0,1.0);
    return t*t*(3-2*t);
  };
  double fraction=std::max(blend(lateral,.04,.075),blend(heading,.04,.09));
  double cap=maximum-(maximum-std::min(maximum,.08))*fraction;
  // heading is reference minus vehicle yaw. Opposite signs mean that forward
  // motion increases lateral error. Reserve 1 cm before the hard bound and
  // include reaction plus braking distance instead of accelerating outwards.
  if(lateral*heading<0 && std::abs(heading)>1e-4) {
    const double available=std::max(0.0,.07-std::abs(lateral))/std::abs(std::sin(heading));
    cap=std::min(cap,std::max(std::min(maximum,.08),approachSpeed(0,available,decel,reaction,0)));
  }
  return cap;
}
// Reserve 0.11 rad/s for feedback; bound curvature change feedforward as well.
inline double continuousCurveLimit(double curvature,double derivative) {
  return std::min({.2,.19/std::max(1e-9,std::abs(curvature)),
    std::sqrt(.2/std::max(1e-9,std::abs(derivative)))});
}
// Bend speed is capped by geometry and then by measured steering response.
class ContinuousTurnResponse {
  struct Sample {double time, command;};
  std::deque<Sample> history_;
  double last_{-1},ratio_{1},bad_since_{-1},cap_{.2};
public:
  void reset() {history_.clear();last_=-1;ratio_=1;bad_since_=-1;cap_=.2;}
  double step(double now,double curvature,double lateral,double heading,
              double measured_w,double previous_w,double nominal=.06) {
    if(last_<0 || now<=last_ || now-last_>.5)reset();
    double dt=last_<0?.1:now-last_;last_=now;
    history_.push_back({now,previous_w});
    while(history_.size()>1 && history_[1].time<=now-.3)history_.pop_front();
    if(std::abs(curvature)<.05) {ratio_=1;bad_since_=-1;cap_=.2;return 1e6;}
    const double delayed=history_.front().command;
    const bool comparable=history_.front().time<=now-.3 && std::abs(delayed)>.06 &&
      delayed*curvature>0 && previous_w*delayed>0;
    if(comparable) {
      const double observed=std::clamp(measured_w/delayed,0.0,1.0);
      ratio_+=(observed-ratio_)*(1-std::exp(-dt/.5));
      if(ratio_<.85) {if(bad_since_<0)bad_since_=now;}
      else bad_since_=-1;
    } else {ratio_=1;bad_since_=-1;}
    auto severity=[](double error,double lo,double hi) {
      double t=std::clamp((std::abs(error)-lo)/(hi-lo),0.,1.);return t*t*(3-2*t);
    };
    nominal=std::min(nominal,.2);
    double target=nominal-(nominal-std::min(nominal,.03))*std::max(severity(lateral,.035,.075),severity(heading,.05,.09));
    // The yaw servo compensates persistent actuator gain error. Slow further
    // only when tracking also shows that compensation has not caught up.
    if(bad_since_>=0 && now-bad_since_>=.4 &&
       (std::abs(heading)>.04 || std::abs(lateral)>.035))
      target=std::min(target,nominal*std::clamp(ratio_,.5,1.));
    // Lower caps immediately; recover slowly. Controller still applies its
    // normal acceleration/braking limits and validates the actual command arc.
    cap_=std::min(target,cap_+.01*dt);
    return cap_;
  }
};
// Only straight portions
// cruise faster; preview includes actuator reaction and a spatial brake margin.
template<class Point, class SlowZone>
double continuousSpeedCap(const std::vector<Point> &path,double maximum,double decel,
                          double reaction,double margin,SlowZone slow_zone) {
  double cap=maximum,arc=0,last_obstacle_check=-1;
  for(size_t i=0;i<path.size();++i) {
    if(i+1<path.size()) {
      const double ds=std::hypot(path[i+1].x-path[i].x,path[i+1].y-path[i].y);
      if(ds>1e-6) {
        double k=trackingAngle(path[i+1].z-path[i].z)/ds,kp=0;
        if(i+2<path.size()) {
          const double next=std::hypot(path[i+2].x-path[i+1].x,path[i+2].y-path[i+1].y);
          if(next>1e-6)kp=(trackingAngle(path[i+2].z-path[i+1].z)/next-k)/((ds+next)*.5);
        }
        if(std::abs(k)>.01 || std::abs(kp)>.01)
          cap=std::min(cap,approachSpeed(continuousCurveLimit(k,kp),arc,decel,reaction,margin));
      }
    }
    if(arc-last_obstacle_check>=.10) {
      if(slow_zone(path[i]))cap=std::min(cap,approachSpeed(.25,arc,decel,reaction,margin));
      last_obstacle_check=arc;
    }
    if(i+1<path.size())arc+=std::hypot(path[i+1].x-path[i].x,path[i+1].y-path[i].y);
  }
  return cap;
}
// Average route curvature slightly ahead of the axle (~0.375 s at 0.08 m/s).
// This is feedforward, not a displaced goal: ordered route geometry is unchanged.
template<class Point>
double continuousCurvature(const std::vector<Point> &path) {
  if(path.size()<2)return 0;
  auto heading=[&](double target) {
    double arc=0;
    for(size_t i=1;i<path.size();++i) {
      double ds=std::hypot(path[i].x-path[i-1].x,path[i].y-path[i-1].y);
      if(ds>1e-6 && arc+ds>=target)
        return double(path[i-1].z)+trackingAngle(path[i].z-path[i-1].z)*(target-arc)/ds;
      arc+=ds;
    }
    return double(path.back().z);
  };
  double length=0;for(size_t i=1;i<path.size();++i)
    length+=std::hypot(path[i].x-path[i-1].x,path[i].y-path[i-1].y);
  const double a=std::min(.01,length),b=std::min(.05,length);
  return b-a>1e-4 ? std::clamp(trackingAngle(heading(b)-heading(a))/(b-a),-2.001,2.001) : 0.;
}
inline double continuousYawRate(double speed,double curvature,double lateral_error,
                                double heading_error,double measured_yaw_rate) {
  const double feedforward=speed*curvature;
  // Small bounded corrections avoid the 1/lookahead^2 noise amplification
  // of pursuit at 12 cm. Damping uses measured motion, never commanded motion.
  const double correction=.9*heading_error-.6*lateral_error+.2*(feedforward-measured_yaw_rate);
  return feedforward+std::clamp(correction,-.10,.10);
}
class ContinuousYawServo {
  double integral_{},last_{-1},last_curvature_{};
public:
  void reset() {integral_=0;last_=-1;last_curvature_=0;}
  double step(double now,double speed,double curvature,double lateral,double heading,double measured) {
    if(last_<0 || now<=last_ || now-last_>.5 || curvature*last_curvature_<0)integral_=0;
    const double dt=last_<0?.1:std::clamp(now-last_,0.,.2);
    last_=now;last_curvature_=curvature;
    // Feedback target, with a bounded extra lateral correction in curves.
    double target=continuousYawRate(speed,curvature,lateral,heading,measured);
    if(std::abs(curvature)>.05) {
      target-=std::clamp(.4*lateral,-.03,.03);
      const double desired=speed*curvature+std::clamp(.9*heading-1.0*lateral,-.10,.10);
      const double error=desired-measured;
      if(std::abs(target+integral_)<.29 || error*(target+integral_)<0)
        integral_=std::clamp(integral_+.35*error*dt,-.10,.10);
    } else integral_*=std::exp(-dt/.4);
    return target+integral_;
  }
};
}
