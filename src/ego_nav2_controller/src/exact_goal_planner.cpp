#include <nav2_smac_planner/smac_planner_lattice.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "ego_nav2_controller/exact_goal.hpp"
#include "ego_nav2_controller/footprint_safety.hpp"
namespace ego_nav2_controller {
class ExactGoalLattice : public nav2_smac_planner::SmacPlannerLattice {
  std::shared_ptr<tf2_ros::Buffer> mount_tf_;
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override {
    mount_tf_=tf;
    nav2_smac_planner::SmacPlannerLattice::configure(parent,name,tf,map);
  }
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &start,
                               const geometry_msgs::msg::PoseStamped &goal) override {
    auto path=nav2_smac_planner::SmacPlannerLattice::createPlan(start,goal);
    if(path.poses.empty())return path;
    if(_costmap_ros->getBaseFrameID()!="drive_base_link")
      throw nav2_core::PlannerException("Exact goal requires drive_base_link geometry");
    geometry_msgs::msg::TransformStamped mount;
    try {mount=mount_tf_->lookupTransform("drive_base_link","base_link",tf2::TimePointZero);}
    catch(const tf2::TransformException &e) {throw nav2_core::PlannerException(e.what());}
    const double cx=mount.transform.translation.x,cy=mount.transform.translation.y;
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*_costmap->getMutex());
    const auto footprint=_costmap_ros->getRobotFootprint();
    double radius=.60+std::hypot(cx,cy);
    for(const auto &p:footprint)radius=std::max(radius,std::hypot(p.x,p.y));
    appendExactGoal(path,goal,_costmap->getResolution(),radius,
      [&](double x,double y,double a,double margin) {
        return bodyCollision(_costmap,footprint,cx,cy,.60,x,y,a,margin);
      });
    return path;
  }
};
}
PLUGINLIB_EXPORT_CLASS(ego_nav2_controller::ExactGoalLattice,nav2_core::GlobalPlanner)
