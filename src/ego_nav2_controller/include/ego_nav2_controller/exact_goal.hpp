#pragma once
#include <nav_msgs/msg/path.hpp>
#include <nav2_core/exceptions.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "ego_nav2_controller/trajectory_validation.hpp"
namespace ego_nav2_controller {
template<class Collision>
void appendExactGoal(nav_msgs::msg::Path &path,const geometry_msgs::msg::PoseStamped &goal,
                     double resolution,double radius,Collision collision) {
  if(path.poses.empty() || goal.header.frame_id!=path.header.frame_id)
    throw nav2_core::PlannerException("Exact goal requires a nonempty path in the goal frame");
  const auto a=path.poses.back().pose;const auto b=goal.pose;
  const double da=std::atan2(std::sin(tf2::getYaw(b.orientation)-tf2::getYaw(a.orientation)),
                            std::cos(tf2::getYaw(b.orientation)-tf2::getYaw(a.orientation)));
  if(!std::isfinite(da) || std::abs(da)>.40 ||
     std::hypot(a.position.x-b.position.x,a.position.y-b.position.y)>resolution*std::sqrt(2.)*.5+1e-5)
    throw nav2_core::PlannerException("Planner endpoint exceeds one-cell quantization bound; exact goal rejected");
  if(!safeMotionSegment(a.position.x,a.position.y,tf2::getYaw(a.orientation),
      b.position.x,b.position.y,tf2::getYaw(b.orientation),radius,resolution,collision,[](){}))
    throw nav2_core::PlannerException("Exact goal connector violates swept footprint/center clearance");
  auto exact=goal;
  // The map goal is fixed in map, not at the map->odom transform at planning time.
  exact.header.stamp=builtin_interfaces::msg::Time();
  if(std::hypot(a.position.x-b.position.x,a.position.y-b.position.y)<1e-8 && std::abs(da)<1e-8)
    path.poses.back()=exact;
  else path.poses.push_back(exact);
}
}
