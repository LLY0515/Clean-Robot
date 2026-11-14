#pragma once
#include <mutex>
#include <nav2_behaviors/plugins/spin.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "ego_nav2_controller/turn_profile.hpp"
#include "ego_nav2_controller/spin_safety.hpp"

namespace ego_nav2_controller {
class SmoothSpin : public nav2_behaviors::Spin {
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  std::mutex odom_mutex_;
  TurnSettings settings_;
  TurnProfile profile_;
  TurnObstacleWait obstacle_wait_;
  std::string heading_frame_;
  double max_age_{}, correction_yaw_limit_{}, correction_xy_limit_{};
  double last_cycle_{}, last_odom_stamp_{}, previous_map_yaw_{}, relative_map_yaw_{};
  geometry_msgs::msg::TransformStamped last_correction_;
  double parameter(const std::string &key,double value) {
    auto node=node_.lock();const auto name=behavior_name_+"."+key;
    nav2_util::declare_parameter_if_not_declared(node,name,rclcpp::ParameterValue(value));
    value=node->get_parameter(name).as_double();
    if(!std::isfinite(value)||value<=0)throw std::runtime_error("Invalid smooth Spin parameter: "+name);
    return value;
  }
  static double wrap(double value) {return std::atan2(std::sin(value),std::cos(value));}
  nav2_behaviors::Status fail(const std::string &reason) {
    stopRobot();RCLCPP_WARN(logger_,"EGO Spin stopped: %s",reason.c_str());
    return nav2_behaviors::Status::FAILED;
  }
  nav_msgs::msg::Odometry::ConstSharedPtr odometry() {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if(!odom_)throw std::runtime_error("missing /odom_nav");
    const double age=(clock_->now()-rclcpp::Time(odom_->header.stamp)).seconds();
    const auto &v=odom_->twist.twist;
    if(age<-.1 || age>max_age_ || odom_->header.frame_id!=global_frame_ ||
       odom_->child_frame_id!=robot_base_frame_ || !std::isfinite(v.angular.z) ||
       !std::isfinite(v.linear.x) || !std::isfinite(v.linear.y))
      throw std::runtime_error("stale/invalid drive-axle odometry");
    return odom_;
  }
  geometry_msgs::msg::TransformStamped correction() {
    // static mode also supplies map->odom. An absent map is never silently
    // replaced by odometry: that would change the meaning of a coverage turn.
    auto value=tf_->lookupTransform(heading_frame_,global_frame_,tf2::TimePointZero);
    if(rclcpp::Time(value.header.stamp).nanoseconds()!=0) {
      const double age=(clock_->now()-rclcpp::Time(value.header.stamp)).seconds();
      if(age<-.1 || age>max_age_)throw std::runtime_error("stale heading-frame correction");
    }
    return value;
  }
protected:
  virtual TurnSafety inspectTurn(const geometry_msgs::msg::PoseStamped &,double,double) {
    return {TurnSafety::Invalid,0,"missing swept-footprint guard"};
  }
public:
  void onConfigure() override {
    nav2_behaviors::Spin::onConfigure();auto node=node_.lock();
    if(global_frame_!="odom" || robot_base_frame_!="drive_base_link")
      throw std::runtime_error("Smooth Spin requires odom/drive_base_link");
    settings_.max_speed=max_rotational_vel_;settings_.min_speed=min_rotational_vel_;
    settings_.acceleration=rotational_acc_lim_;
    settings_.reaction_time=parameter("reaction_time",.20);
    settings_.yaw_tolerance=parameter("yaw_tolerance",0.03490658503988659);
    settings_.stopped_speed=parameter("stopped_angular_vel",.03);
    settings_.settle_time=parameter("settle_time",.20);
    max_age_=parameter("max_odom_age",.50);
    correction_yaw_limit_=parameter("max_correction_yaw",.10);
    correction_xy_limit_=parameter("max_correction_translation",.20);
    if(!std::isfinite(max_rotational_vel_) || max_rotational_vel_<=0 || max_rotational_vel_>.43 ||
       !std::isfinite(rotational_acc_lim_) || rotational_acc_lim_<=0 ||
       !std::isfinite(min_rotational_vel_) || min_rotational_vel_<0 || min_rotational_vel_>max_rotational_vel_)
      throw std::runtime_error("Invalid smooth Spin speed/acceleration");
    nav2_util::declare_parameter_if_not_declared(node,behavior_name_+".heading_frame",rclcpp::ParameterValue("map"));
    heading_frame_=node->get_parameter(behavior_name_+".heading_frame").as_string();
    if(heading_frame_.empty())throw std::runtime_error("Empty Spin heading frame");
    profile_=TurnProfile(settings_);
    odom_sub_=node->create_subscription<nav_msgs::msg::Odometry>("/odom_nav",rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg){std::lock_guard<std::mutex> lock(odom_mutex_);odom_=msg;});
  }
  void onCleanup() override {
    odom_sub_.reset();{std::lock_guard<std::mutex> lock(odom_mutex_);odom_.reset();}
    nav2_behaviors::Spin::onCleanup();
  }
  nav2_behaviors::Status onRun(const std::shared_ptr<const nav2_msgs::action::Spin::Goal> goal) override {
    try {
      if(!std::isfinite(goal->target_yaw))return fail("nonfinite turn");
      auto odom=odometry();last_correction_=correction();
      geometry_msgs::msg::PoseStamped pose;
      if(!nav2_util::getCurrentPose(pose,*tf_,global_frame_,robot_base_frame_,transform_tolerance_))
        return fail("missing turn pose");
      tf2::Quaternion map_odom,odom_base;
      tf2::fromMsg(last_correction_.transform.rotation,map_odom);tf2::fromMsg(pose.pose.orientation,odom_base);
      previous_map_yaw_=tf2::getYaw(map_odom*odom_base);relative_map_yaw_=0;
      prev_yaw_=tf2::getYaw(pose.pose.orientation);relative_yaw_=0;cmd_yaw_=goal->target_yaw;
      command_time_allowance_=goal->time_allowance;end_time_=clock_->now()+command_time_allowance_;
      RCLCPP_INFO(logger_,"EGO smooth Spin %.3f rad, anchored in %s",cmd_yaw_,heading_frame_.c_str());
      last_cycle_=clock_->now().seconds();last_odom_stamp_=rclcpp::Time(odom->header.stamp).seconds();
      profile_.reset(odom->twist.twist.angular.z);obstacle_wait_.reset();
      return nav2_behaviors::Status::SUCCEEDED;
    } catch(const std::exception &e) {return fail(e.what());}
  }
  nav2_behaviors::Status onCycleUpdate() override {
    try {
      const double now=clock_->now().seconds(),dt=now-last_cycle_;
      if(dt<0 || dt>.5)return fail("control clock discontinuity");
      last_cycle_=now;
      if(clock_->now()>end_time_ && command_time_allowance_.seconds()>0)return fail("turn timeout");
      auto odom=odometry();const double stamp=rclcpp::Time(odom->header.stamp).seconds();
      if(stamp<last_odom_stamp_)return fail("odometry time rewind");
      last_odom_stamp_=stamp;
      geometry_msgs::msg::PoseStamped pose;
      if(!nav2_util::getCurrentPose(pose,*tf_,global_frame_,robot_base_frame_,transform_tolerance_))
        return fail("missing turn pose");
      const double pose_age=(clock_->now()-rclcpp::Time(pose.header.stamp)).seconds();
      if(pose_age<-.1 || pose_age>max_age_)return fail("stale turn pose");
      const double yaw=tf2::getYaw(pose.pose.orientation);
      relative_yaw_+=wrap(yaw-prev_yaw_);prev_yaw_=yaw;
      auto current=correction();
      tf2::Transform before,after;tf2::fromMsg(last_correction_.transform,before);tf2::fromMsg(current.transform,after);
      tf2::Quaternion body;tf2::fromMsg(pose.pose.orientation,body);
      const double map_yaw=tf2::getYaw(after.getRotation()*body);
      const double delta=wrap(map_yaw-tf2::getYaw(before.getRotation()*body));
      const tf2::Vector3 point(pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);
      if(std::abs(delta)>correction_yaw_limit_ || ((after*point)-(before*point)).length()>correction_xy_limit_)
        return fail("large localization correction; reinitialize the turn after localization is stable");
      // Compose full rotations before extracting yaw; map/odom roll and pitch
      // need not be zero. Unwrap only increments, preserving half-turn direction.
      relative_map_yaw_+=wrap(map_yaw-previous_map_yaw_);previous_map_yaw_=map_yaw;last_correction_=current;
      feedback_->angular_distance_traveled=relative_yaw_;
      if(action_server_->is_running())action_server_->publish_feedback(feedback_);
      const auto &measured=odom->twist.twist;
      const bool translating=std::hypot(measured.linear.x,measured.linear.y)>.04;
      const double remaining=cmd_yaw_-relative_map_yaw_;
      const double braking_angle=std::copysign(std::abs(measured.angular.z)*settings_.reaction_time+
        measured.angular.z*measured.angular.z/(2*settings_.acceleration),measured.angular.z);
      if(translating)profile_.reset(0);
      const auto output=translating?TurnProfile::Output{0,false}:
        profile_.update(remaining,measured.angular.z,std::clamp(dt,.001,.2),stamp);
      geometry_msgs::msg::Twist command;command.angular.z=output.speed;
      // Check the imminent commanded arc and measured stopping arc, not the
      // all-heading disk. Future commands are rechecked on every cycle.
      const double duration=std::max(simulate_ahead_time_,settings_.reaction_time+
        std::max(std::abs(output.speed),std::abs(measured.angular.z))/settings_.acceleration);
      // Heading progress remains live while held; never restart onRun() or
      // reset relative_map_yaw_ when an obstacle clears.
      auto safety=inspectTurn(pose,output.speed*duration,braking_angle);
      if(safety.state==TurnSafety::Invalid)return fail(safety.reason);
      // Retain Nav2's independent command/braking-arc footprint check.
      if(safety.state==TurnSafety::Clear)for(double t=0;t<=duration;t+=.05) {
        geometry_msgs::msg::Pose2D sample;sample.x=pose.pose.position.x;sample.y=pose.pose.position.y;
        sample.theta=yaw+output.speed*t;
        if(!collision_checker_->isCollisionFree(sample,t==0)) {
          safety.state=TurnSafety::Blocked;safety.reason="command/braking footprint blocked";break;
        }
      }
      const bool was_waiting=obstacle_wait_.waiting();
      const auto wait=obstacle_wait_.update(now,safety.state==TurnSafety::Clear,safety.map_stamp,
        !translating && std::abs(measured.angular.z)<=settings_.stopped_speed);
      if(wait==TurnObstacleWait::Timeout)return fail("obstacle wait exceeded 3 seconds; turn remains blocked/unconfirmed");
      if(wait==TurnObstacleWait::Hold) {
        profile_.reset(0);stopRobot();
        if(!was_waiting)RCLCPP_WARN(logger_,"EGO Spin waiting at zero velocity: %s; need 2 fresh clear maps within 3 seconds",safety.reason.c_str());
        return nav2_behaviors::Status::RUNNING;
      }
      if(was_waiting)RCLCPP_INFO(logger_,"EGO Spin clearance confirmed; continuing remaining %.3f rad",remaining);
      // FollowPath success is not evidence that translation has stopped.
      if(translating) {stopRobot();return nav2_behaviors::Status::RUNNING;}
      if(output.settled) {stopRobot();return nav2_behaviors::Status::SUCCEEDED;}
      vel_pub_->publish(command);return nav2_behaviors::Status::RUNNING;
    } catch(const std::exception &e) {return fail(e.what());}
  }
};
}
