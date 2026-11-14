#include <algorithm>
#include <cmath>
#include <mutex>
#include <type_traits>
#include <nav2_behaviors/plugins/spin.hpp>
#include <nav2_behaviors/plugins/back_up.hpp>
#include <nav2_behaviors/plugins/drive_on_heading.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav2_costmap_2d/footprint_subscriber.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "ego_nav2_controller/smooth_spin.hpp"
namespace ego_nav2_controller {
// Keep recovery actions from bypassing the center-clearance rule.
// The drive-axle disk encloses both the offset center disk and the real padded body.
template<class Base,class Action> class GuardedBehavior : public Base {
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr sub_;
  std::unique_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
  nav2_msgs::msg::Costmap::SharedPtr map_;
  std::mutex mutex_;
  double extra_{};
public:
  void onConfigure() override {
    Base::onConfigure();auto node=this->node_.lock();std::string topic;
    node->get_parameter("costmap_topic",topic);
    sub_=node->template create_subscription<nav2_msgs::msg::Costmap>(topic,
      rclcpp::QoS(1).transient_local().reliable(),[this](nav2_msgs::msg::Costmap::SharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);map_=m;
      });
    std::string footprint_topic;node->get_parameter("footprint_topic",footprint_topic);
    auto lifecycle_node=std::dynamic_pointer_cast<nav2_util::LifecycleNode>(node);
    if(!lifecycle_node || footprint_topic.empty())
      throw std::runtime_error("EGO recovery requires a lifecycle node and footprint_topic");
    footprint_sub_=std::make_unique<nav2_costmap_2d::FootprintSubscriber>(
      lifecycle_node,footprint_topic,*this->tf_,this->robot_base_frame_,this->transform_tolerance_);
  }
  void onCleanup() override {sub_.reset();footprint_sub_.reset();map_.reset();Base::onCleanup();}
  nav2_behaviors::Status onRun(const std::shared_ptr<const typename Action::Goal> goal) override {
    extra_=0;
    if constexpr(!std::is_same<Action,nav2_msgs::action::Spin>::value) {
      if(!std::isfinite(goal->speed)||std::abs(goal->speed)>.15) return nav2_behaviors::Status::FAILED;
      extra_=2.0*std::abs(goal->speed);
    }
    return Base::onRun(goal);
  }
  nav2_behaviors::Status onCycleUpdate() override {
    nav2_msgs::msg::Costmap::SharedPtr m;
    {std::lock_guard<std::mutex> lock(mutex_);m=map_;}
    auto fail=[this]() {
      this->vel_pub_->publish(geometry_msgs::msg::Twist());
      RCLCPP_WARN(this->logger_,"EGO recovery blocked: missing/invalid footprint, stale costmap/TF or obstacle within clearance/body envelope");
      return nav2_behaviors::Status::FAILED;
    };
    if(!m || m->metadata.resolution<=0 || m->data.size()!=size_t(m->metadata.size_x)*m->metadata.size_y) return fail();
    double age=(this->clock_->now()-rclcpp::Time(m->header.stamp)).seconds();
    if(age<-.1||age>1.0)return fail();
    geometry_msgs::msg::TransformStamped tf;
    try {tf=this->tf_->lookupTransform(m->header.frame_id,this->robot_base_frame_,tf2::TimePointZero);}
    catch(const tf2::TransformException &) {return fail();}
    double tf_age=(this->clock_->now()-rclcpp::Time(tf.header.stamp)).seconds();
    if(tf_age<-.1||tf_age>.5)return fail();
    std::vector<geometry_msgs::msg::Point> footprint;std_msgs::msg::Header footprint_header;
    if(!footprint_sub_ || !footprint_sub_->getFootprintInRobotFrame(footprint,footprint_header) ||
       footprint.size()<3)return fail();
    double radius=.60+.25;
    for(const auto &p:footprint) {
      if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))return fail();
      const double corner=std::hypot(p.x,p.y);
      if(!std::isfinite(corner))return fail();
      radius=std::max(radius,corner);
    }
    radius+=extra_;
    const auto &meta=m->metadata;
    double x=tf.transform.translation.x-meta.origin.position.x;
    double y=tf.transform.translation.y-meta.origin.position.y;
    if(std::abs(meta.origin.orientation.z)>1e-6 || std::abs(meta.origin.orientation.w-1)>1e-6)return fail();
    if(x-radius<0||y-radius<0||x+radius>=meta.size_x*meta.resolution||y+radius>=meta.size_y*meta.resolution)return fail();
    int lo_x=int((x-radius)/meta.resolution),hi_x=int((x+radius)/meta.resolution);
    int lo_y=int((y-radius)/meta.resolution),hi_y=int((y+radius)/meta.resolution);
    for(int my=lo_y;my<=hi_y;++my)for(int mx=lo_x;mx<=hi_x;++mx) {
      if(m->data[my*meta.size_x+mx]<254)continue;
      if(std::hypot((mx+.5)*meta.resolution-x,(my+.5)*meta.resolution-y)<=radius+meta.resolution*.7072)return fail();
    }
    return Base::onCycleUpdate();
  }
};
// Spin uses the imminent command and measured braking sweeps. Drive/backup retain their full-disk guard.
class ClearanceSpin : public SmoothSpin {
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr map_sub_;
  std::unique_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
  nav2_msgs::msg::Costmap::ConstSharedPtr map_;
  std::mutex mutex_;
protected:
  TurnSafety inspectTurn(const geometry_msgs::msg::PoseStamped &pose,double command_arc,double braking) override {
    nav2_msgs::msg::Costmap::ConstSharedPtr map;
    {std::lock_guard<std::mutex> lock(mutex_);map=map_;}
    auto invalid=[](const std::string &why){return TurnSafety{TurnSafety::Invalid,0,why};};
    if(!map)return invalid("missing spin costmap");
    const auto stamp=rclcpp::Time(map->header.stamp).nanoseconds();
    const double age=(clock_->now()-rclcpp::Time(map->header.stamp)).seconds();
    if(age<-.1||age>1.)return invalid("stale spin costmap");
    if(map->header.frame_id!=pose.header.frame_id)return invalid("spin costmap frame mismatch");
    const auto &meta=map->metadata;
    const auto &q=meta.origin.orientation;
    if(!std::isfinite(q.x)||!std::isfinite(q.y)||!std::isfinite(q.z)||!std::isfinite(q.w)||
       std::abs(q.x)>1e-6||std::abs(q.y)>1e-6||std::abs(q.z)>1e-6||std::abs(q.w-1)>1e-6)
      return invalid("invalid spin costmap orientation");
    std::vector<geometry_msgs::msg::Point> footprint;std_msgs::msg::Header header;
    if(!footprint_sub_ || !footprint_sub_->getFootprintInRobotFrame(footprint,header))
      return invalid("missing spin footprint/TF");
    std::vector<TurnPoint> polygon;
    for(const auto &p:footprint) {
      if(!std::isfinite(p.z))return invalid("nonfinite spin footprint");
      polygon.push_back({p.x,p.y});
    }
    TurnGrid grid{meta.resolution,meta.origin.position.x,meta.origin.position.y,
                  meta.size_x,meta.size_y,&map->data};
    const double yaw=tf2::getYaw(pose.pose.orientation);
    auto state=checkTurnSweep(grid,polygon,pose.pose.position.x,pose.pose.position.y,yaw,command_arc);
    if(state==TurnSafety::Clear)state=checkTurnSweep(grid,polygon,pose.pose.position.x,pose.pose.position.y,yaw,braking);
    return {state,stamp,state==TurnSafety::Invalid?"invalid spin grid/footprint":
      state==TurnSafety::Blocked?"command/braking sweep violates padded footprint or 0.60 m center clearance":"clear"};
  }
public:
  void onConfigure() override {
    SmoothSpin::onConfigure();auto node=node_.lock();std::string topic,footprint;
    node->get_parameter("costmap_topic",topic);node->get_parameter("footprint_topic",footprint);
    auto lifecycle=std::dynamic_pointer_cast<nav2_util::LifecycleNode>(node);
    if(!lifecycle || footprint.empty())throw std::runtime_error("Spin requires footprint_topic");
    map_sub_=node->create_subscription<nav2_msgs::msg::Costmap>(topic,rclcpp::QoS(1).transient_local().reliable(),
      [this](nav2_msgs::msg::Costmap::ConstSharedPtr msg){std::lock_guard<std::mutex> lock(mutex_);map_=msg;});
    footprint_sub_=std::make_unique<nav2_costmap_2d::FootprintSubscriber>(lifecycle,footprint,*tf_,robot_base_frame_,transform_tolerance_);
  }
  void onCleanup() override {
    map_sub_.reset();footprint_sub_.reset();{std::lock_guard<std::mutex> lock(mutex_);map_.reset();}
    SmoothSpin::onCleanup();
  }
};
class ClearanceBackUp : public GuardedBehavior<nav2_behaviors::BackUp,nav2_msgs::action::BackUp> {};
class ClearanceDrive : public GuardedBehavior<nav2_behaviors::DriveOnHeading<>,nav2_msgs::action::DriveOnHeading> {};
}
PLUGINLIB_EXPORT_CLASS(ego_nav2_controller::ClearanceSpin,nav2_core::Behavior)
PLUGINLIB_EXPORT_CLASS(ego_nav2_controller::ClearanceBackUp,nav2_core::Behavior)
PLUGINLIB_EXPORT_CLASS(ego_nav2_controller::ClearanceDrive,nav2_core::Behavior)
