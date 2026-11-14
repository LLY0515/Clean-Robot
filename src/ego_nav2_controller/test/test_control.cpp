#include <gtest/gtest.h>
#include <fstream>
#include <deque>
#include <nav2_core/controller.hpp>
#include <nav2_core/exceptions.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
class Control : public ::testing::Test {
protected:
  pluginlib::ClassLoader<nav2_core::Controller> loader{"nav2_core","nav2_core::Controller"};
  std::shared_ptr<nav2_core::Controller> controller;
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map;
  std::shared_ptr<tf2_ros::Buffer> tf;
  virtual double maxLinearSpeed() const {return .15;}
  virtual double bodyClearance() const {return .60;}
  virtual double maxAngularSpeed() const {return .2;}
  virtual bool continuousCurve() const {return false;}
  virtual bool preciseApproach() const {return false;}
  virtual double planningBudget() const {return .25;}
  virtual double footprintPadding() const {return .01;}
  virtual std::string footprintSpec() const {return "[[0.75,0.4],[0.75,-0.4],[-0.25,-0.4],[-0.25,0.4]]";}
  virtual bool smoothDetourEnabled() const {return false;}
  void SetUp() override {
    if(!rclcpp::ok())rclcpp::init(0,nullptr);
    parent=std::make_shared<rclcpp_lifecycle::LifecycleNode>("ego_control_test");
    parent->declare_parameter("EgoTest.max_linear_vel",maxLinearSpeed());
    parent->declare_parameter("EgoTest.planning_time_budget",planningBudget());
    parent->declare_parameter("EgoTest.obstacle_clearance",bodyClearance());
    parent->declare_parameter("EgoTest.max_angular_vel",maxAngularSpeed());
    parent->declare_parameter("EgoTest.precise_goal_alignment",preciseApproach());
    if(smoothDetourEnabled()) {
      parent->declare_parameter("EgoTest.smooth_detour_enabled",true);
      parent->declare_parameter("EgoTest.planning_horizon",4.5);
      parent->declare_parameter("EgoTest.lookahead_dist",.7);
      parent->declare_parameter("EgoTest.return_min_forward_distance",1.2);
      if(maxLinearSpeed()>.4)parent->declare_parameter("EgoTest.curve_preview_dist",3.0);
    }
    if(continuousCurve()) {
      parent->declare_parameter("EgoTest.preserve_reference_curve",true);
      parent->declare_parameter("EgoTest.max_linear_accel",maxLinearSpeed()>.4?.2:.05);
      parent->declare_parameter("EgoTest.planning_horizon",maxLinearSpeed()>.4?3.5:1.2);
      if(maxLinearSpeed()>.4)parent->declare_parameter("EgoTest.curve_preview_dist",3.0);
      parent->declare_parameter("EgoTest.lookahead_dist",.12);
    }
    map=std::make_shared<nav2_costmap_2d::Costmap2DROS>("ego_test_costmap");
    map->set_parameters({rclcpp::Parameter("plugins",std::vector<std::string>{}),
      rclcpp::Parameter("global_frame","odom"),rclcpp::Parameter("robot_base_frame","drive_base_link"),
      rclcpp::Parameter("rolling_window",true),
      rclcpp::Parameter("width",10),rclcpp::Parameter("height",10),rclcpp::Parameter("origin_x",-5.0),
      rclcpp::Parameter("origin_y",-5.0),rclcpp::Parameter("resolution",0.05),
      rclcpp::Parameter("footprint",footprintSpec()),rclcpp::Parameter("footprint_padding",footprintPadding())});
    ASSERT_EQ(map->on_configure(rclcpp_lifecycle::State()),nav2_util::CallbackReturn::SUCCESS);
    if(smoothDetourEnabled())map->getCostmap()->resizeMap(240,240,.05,-6,-6);
    tf=std::make_shared<tf2_ros::Buffer>(parent->get_clock());
    geometry_msgs::msg::TransformStamped mount;mount.header.frame_id="drive_base_link";
    mount.child_frame_id="base_link";mount.transform.translation.x=.25;mount.transform.rotation.w=1;
    tf->setTransform(mount,"test",true);
    controller=loader.createSharedInstance("ego_nav2_controller::EgoController");
    controller->configure(parent,"EgoTest",tf,map);controller->activate();
  }
  void TearDown() override {
    if(controller){controller->deactivate();controller->cleanup();controller.reset();}
    map->on_cleanup(rclcpp_lifecycle::State());map.reset();tf.reset();parent.reset();
  }
  geometry_msgs::msg::PoseStamped pose(double x=0,double y=0,double yaw=0) {
    geometry_msgs::msg::PoseStamped p;p.header.frame_id="odom";p.header.stamp=parent->now();
    p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.z=sin(yaw/2);p.pose.orientation.w=cos(yaw/2);return p;
  }
  void path() {
    nav_msgs::msg::Path path;path.header.frame_id="odom";
    for(int i=0;i<=15;++i)path.poses.push_back(pose(i*.2));controller->setPlan(path);
  }
};
class SpeedProfileControl : public Control, public ::testing::WithParamInterface<double> {
protected:
  double maxLinearSpeed() const override {return GetParam();}
  double maxAngularSpeed() const override {return .30;}
};
class SmoothCoverageControl : public Control, public ::testing::WithParamInterface<double> {
protected:
  double maxLinearSpeed() const override {return GetParam();}
  double maxAngularSpeed() const override {return .3;}
  bool smoothDetourEnabled() const override {return true;}
};
INSTANTIATE_TEST_SUITE_P(ApproachAndCoverage, SmoothCoverageControl, ::testing::Values(.25,.40));
class HighSpeedCoverageControl : public Control {
protected:
  double maxLinearSpeed() const override {return .8;}
  double maxAngularSpeed() const override {return .3;}
  bool smoothDetourEnabled() const override {return true;}
};
TEST_F(HighSpeedCoverageControl, CruiseThenBrakeForNewObstacleAndResume) {
  parent->set_parameter(rclcpp::Parameter("use_sim_time",true));
  auto *clock=parent->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL),RCL_RET_OK);
  nav_msgs::msg::Path route;route.header.frame_id="odom";
  for(int i=0;i<=160;++i)route.poses.push_back(pose(i*.1));controller->setPlan(route);
  double x=0,y=0,yaw=0,cruise=0,after=0,min_distance=10,peak_maneuver=0;bool reached=false;
  geometry_msgs::msg::Twist measured;std::ofstream trace("/tmp/ego-speed-080.csv");
  trace<<"step,x,y,yaw,v,w,measured_v\n";
  for(int i=0;i<2000;++i) {
    ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL+int64_t(i)*100000000LL),RCL_RET_OK);
    auto *grid=map->getCostmap();grid->updateOrigin(x-6,y-6);
    if(7-x<=4) {unsigned int ox,oy;if(grid->worldToMap(7,0,ox,oy))grid->setCost(ox,oy,nav2_costmap_2d::LETHAL_OBSTACLE);}
    geometry_msgs::msg::TwistStamped cmd;
    try{cmd=controller->computeVelocityCommands(pose(x,y,yaw),measured,nullptr);}
    catch(const std::exception &e){FAIL()<<"step="<<i<<" pose="<<x<<","<<y<<","<<yaw<<" speed="<<measured.linear.x<<": "<<e.what();}
    EXPECT_LE(cmd.twist.linear.x,.800001);EXPECT_LE(std::abs(cmd.twist.angular.z),.300001);
    if(x<3)cruise=std::max(cruise,cmd.twist.linear.x);
    if(x>11)after=std::max(after,cmd.twist.linear.x);
    if(std::abs(y)>.12 && x<10){peak_maneuver=std::max(peak_maneuver,cmd.twist.linear.x);EXPECT_LE(cmd.twist.linear.x,.250001);}
    measured.linear.x+=.4*(cmd.twist.linear.x-measured.linear.x);
    measured.angular.z+=.5*(cmd.twist.angular.z-measured.angular.z);
    x+=measured.linear.x*std::cos(yaw)*.1;y+=measured.linear.x*std::sin(yaw)*.1;yaw+=measured.angular.z*.1;
    const double d=std::hypot(x+.25*std::cos(yaw)-7.025,y+.25*std::sin(yaw)-.025);
    min_distance=std::min(min_distance,d);EXPECT_GE(d,.60);
    trace<<i<<","<<x<<","<<y<<","<<yaw<<","<<cmd.twist.linear.x<<","<<cmd.twist.angular.z<<","<<measured.linear.x<<"\n";
    if(std::hypot(x-16,y)<.08 && std::abs(std::atan2(std::sin(yaw),std::cos(yaw)))<.08){reached=true;break;}
  }
  EXPECT_TRUE(reached)<<x<<","<<y;EXPECT_NEAR(cruise,.8,1e-6);EXPECT_NEAR(after,.8,1e-6);
  std::cout<<"HIGH_SPEED cruise="<<cruise<<" maneuver="<<peak_maneuver<<" resumed="<<after<<" min_clearance="<<min_distance<<std::endl;
}
TEST_F(HighSpeedCoverageControl, SuddenObstacleInsideMeasuredBrakingDistanceRejectsMotion) {
  path();unsigned int ox,oy;map->getCostmap()->worldToMap(1.5,0,ox,oy);
  map->getCostmap()->setCost(ox,oy,nav2_costmap_2d::LETHAL_OBSTACLE);
  geometry_msgs::msg::Twist measured;measured.linear.x=.8;
  EXPECT_THROW(controller->computeVelocityCommands(pose(),measured,nullptr),nav2_core::PlannerException);
}
TEST_F(HighSpeedCoverageControl, SuddenHeadingChangeBrakesBeforeRotation) {
  nav_msgs::msg::Path route;route.header.frame_id="odom";
  route.poses={pose(),pose(0,.5,1.57079632679)};controller->setPlan(route);
  geometry_msgs::msg::Twist measured;measured.linear.x=.8;
  auto cmd=controller->computeVelocityCommands(pose(),measured,nullptr);
  EXPECT_LT(cmd.twist.linear.x,.8);EXPECT_NEAR(cmd.twist.linear.x,.78,1e-6);
  EXPECT_DOUBLE_EQ(cmd.twist.angular.z,0);
}
TEST_F(HighSpeedCoverageControl, RejectsInsufficientPreviewAtHighSpeed) {
  controller->deactivate();controller->cleanup();controller.reset();
  parent->set_parameter(rclcpp::Parameter("EgoTest.curve_preview_dist",1.5));
  auto invalid=loader.createSharedInstance("ego_nav2_controller::EgoController");
  EXPECT_THROW(invalid->configure(parent,"EgoTest",tf,map),nav2_core::PlannerException);
}
TEST_F(HighSpeedCoverageControl, SlowsBeforeUnobstructedNinetyDegreeBend) {
  parent->set_parameter(rclcpp::Parameter("use_sim_time",true));
  auto *clock=parent->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL),RCL_RET_OK);
  nav_msgs::msg::Path route;route.header.frame_id="odom";
  for(int i=0;i<=60;++i)route.poses.push_back(pose(i*.1));
  for(int i=1;i<=60;++i)route.poses.push_back(pose(6,i*.1,1.57079632679));controller->setPlan(route);
  double x=0,y=0,yaw=0,peak=0,bend_peak=0;bool reached=false;geometry_msgs::msg::Twist measured;
  for(int i=0;i<1800;++i) {
    ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL+int64_t(i)*100000000LL),RCL_RET_OK);
    map->getCostmap()->updateOrigin(x-6,y-6);
    auto cmd=controller->computeVelocityCommands(pose(x,y,yaw),measured,nullptr);
    peak=std::max(peak,cmd.twist.linear.x);
    if(std::hypot(x-6,y)<1.) {bend_peak=std::max(bend_peak,measured.linear.x);EXPECT_LE(measured.linear.x,.27);}
    measured.linear.x+=.4*(cmd.twist.linear.x-measured.linear.x);
    measured.angular.z+=.5*(cmd.twist.angular.z-measured.angular.z);
    x+=measured.linear.x*std::cos(yaw)*.1;y+=measured.linear.x*std::sin(yaw)*.1;yaw+=measured.angular.z*.1;
    if(std::hypot(x-6,y-6)<.08 && std::abs(yaw-1.57079632679)<.08){reached=true;break;}
  }
  EXPECT_TRUE(reached)<<x<<","<<y<<","<<yaw;EXPECT_NEAR(peak,.8,1e-6);
  std::cout<<"HIGH_SPEED_BEND cruise="<<peak<<" measured_bend_peak="<<bend_peak<<std::endl;
}
TEST_P(SmoothCoverageControl, NewObstacleTriggersReplanBeforeCommand) {
  nav_msgs::msg::Path route;route.header.frame_id="odom";
  for(int i=0;i<=80;++i)route.poses.push_back(pose(i*.1));controller->setPlan(route);
  controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr);
  unsigned int ox,oy;map->getCostmap()->worldToMap(2,0,ox,oy);
  map->getCostmap()->setCost(ox,oy,nav2_costmap_2d::LETHAL_OBSTACLE);
  auto cmd=controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr);
  EXPECT_GE(cmd.twist.linear.x,0);EXPECT_LE(cmd.twist.linear.x,.4);
  EXPECT_LE(std::abs(cmd.twist.angular.z),.3);
  // An impossible wall must still reject motion, including the fallback.
  for(double y=-6;y<6;y+=.05)if(map->getCostmap()->worldToMap(.5,y,ox,oy))
    map->getCostmap()->setCost(ox,oy,nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr),nav2_core::PlannerException);
}
TEST_P(SmoothCoverageControl, EarlyDetourRejoinsWithSafeContinuousCommands) {
  parent->set_parameter(rclcpp::Parameter("use_sim_time",true));
  auto *clock=parent->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL),RCL_RET_OK);
  nav_msgs::msg::Path route;route.header.frame_id="odom";
  for(int i=0;i<=80;++i)route.poses.push_back(pose(i*.1));controller->setPlan(route);
  unsigned int ox,oy;map->getCostmap()->worldToMap(2,0,ox,oy);
  map->getCostmap()->setCost(ox,oy,nav2_costmap_2d::LETHAL_OBSTACLE);
  double x=0,y=0,yaw=0,depart=10,rejoin=-1,min_clearance=10,max_offset=0;
  double previous_w=0;int stopped_during_detour=0;bool passed=false,reached=false;
  geometry_msgs::msg::Twist measured;
  std::ofstream trace("/tmp/ego-smooth-detour.csv");
  trace<<"step,x,y,yaw,v,w\n";
  for(int i=0;i<1600;++i) {
    ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL+int64_t(i)*100000000LL),RCL_RET_OK);
    map->getCostmap()->updateOrigin(x-6,y-6);
    geometry_msgs::msg::TwistStamped cmd;
    try {cmd=controller->computeVelocityCommands(pose(x,y,yaw),measured,nullptr);}
    catch(const std::exception &e){FAIL()<<"step="<<i<<" pose="<<x<<","<<y<<","<<yaw<<": "<<e.what();}
    EXPECT_LE(cmd.twist.linear.x,.400001);EXPECT_LE(std::abs(cmd.twist.angular.z),.300001);
    EXPECT_LE(std::abs(cmd.twist.angular.z-previous_w),.040001);previous_w=cmd.twist.angular.z;
    if(i>10 && x<3.5 && cmd.twist.linear.x<.005)++stopped_during_detour;
    measured=cmd.twist;x+=measured.linear.x*std::cos(yaw)*.1;y+=measured.linear.x*std::sin(yaw)*.1;yaw+=measured.angular.z*.1;
    const double clearance=std::hypot(x+.25*std::cos(yaw)-2.025,y+.25*std::sin(yaw)-.025);
    min_clearance=std::min(min_clearance,clearance);EXPECT_GE(clearance,.60);
    if(std::abs(y)>.05)depart=std::min(depart,x);
    if(x>2.5 && std::abs(y)>.2)passed=true;
    if(passed && rejoin<0 && std::abs(y)<.08)rejoin=x;
    max_offset=std::max(max_offset,std::abs(y));
    trace<<i<<","<<x<<","<<y<<","<<yaw<<","<<measured.linear.x<<","<<measured.angular.z<<"\n";
    if(std::hypot(x-8,y)<.08 && std::abs(std::atan2(std::sin(yaw),std::cos(yaw)))<.08){reached=true;break;}
  }
  EXPECT_TRUE(reached);EXPECT_LT(depart,1.0);EXPECT_GT(rejoin,3.2);EXPECT_LT(rejoin,6.0);
  EXPECT_LT(max_offset,1.8);EXPECT_EQ(stopped_during_detour,0);
  std::cout<<"SMOOTH depart="<<depart<<" rejoin="<<rejoin<<" clearance="<<min_clearance<<" max_offset="<<max_offset<<std::endl;
}
INSTANTIATE_TEST_SUITE_P(ApproachAndCoverage, SpeedProfileControl, ::testing::Values(.25,.40));
TEST_F(Control, ProducesBoundedForwardCommand) {
  path();geometry_msgs::msg::Twist measured;
  auto cmd=controller->computeVelocityCommands(pose(),measured,nullptr);
  EXPECT_GT(cmd.twist.linear.x,0);EXPECT_LE(cmd.twist.linear.x,.15);
  EXPECT_LE(std::abs(cmd.twist.angular.z),maxAngularSpeed());EXPECT_EQ(cmd.header.frame_id,"drive_base_link");
}
TEST_F(Control, RejectsStalePose) {
  path();auto old=pose();old.header.stamp=parent->now()-rclcpp::Duration::from_seconds(2);
  EXPECT_THROW(controller->computeVelocityCommands(old,geometry_msgs::msg::Twist(),nullptr),nav2_core::PlannerException);
}
TEST_F(Control, RejectsObstacleInsideFootprint) {
  path();unsigned int x,y;map->getCostmap()->worldToMap(.2,0,x,y);
  map->getCostmap()->setCost(x,y,nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr),nav2_core::PlannerException);
}
TEST_F(Control, StopsAtGoalAndRespectsSpeedLimit) {
  path();controller->setSpeedLimit(10,true);
  auto cmd=controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr);EXPECT_LE(cmd.twist.linear.x,.015);
  cmd=controller->computeVelocityCommands(pose(3),geometry_msgs::msg::Twist(),nullptr);
  EXPECT_DOUBLE_EQ(cmd.twist.linear.x,0);EXPECT_DOUBLE_EQ(cmd.twist.angular.z,0);
}

TEST_F(Control, RejectsWithin60cmOfBodyCenter) {
  path();unsigned int x,y;map->getCostmap()->worldToMap(.275,.575,x,y);
  map->getCostmap()->setCost(x,y,nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr),nav2_core::PlannerException);
}
TEST_F(Control, AllowsObstacleBetween60And70cmOutsideFootprint) {
  // Use a cell center: a boundary coordinate can round into the nearer cell.
  path();unsigned int x,y;ASSERT_TRUE(map->getCostmap()->worldToMap(.275,.675,x,y));
  double wx,wy;map->getCostmap()->mapToWorld(x,y,wx,wy);
  ASSERT_GT(std::hypot(wx-.25,wy),.60+.05*std::sqrt(2.)/2);
  ASSERT_LT(std::hypot(wx-.25,wy),.70);
  map->getCostmap()->setCost(x,y,nav2_costmap_2d::LETHAL_OBSTACLE);
  auto cmd=controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr);
  EXPECT_GT(std::abs(cmd.twist.linear.x)+std::abs(cmd.twist.angular.z),0);
}
class FiftyClearanceControl : public Control {
protected:
  double bodyClearance() const override {return .50;}
};
TEST_F(FiftyClearanceControl, RejectsFootprintInteriorOutside50cmCircle) {
  path();unsigned int x,y;ASSERT_TRUE(map->getCostmap()->worldToMap(.70,.35,x,y));
  double wx,wy;map->getCostmap()->mapToWorld(x,y,wx,wy);
  ASSERT_GT(std::hypot(wx-.25,wy),.50+.05*std::sqrt(2.)/2);
  map->getCostmap()->setCost(x,y,nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr),nav2_core::PlannerException);
}
TEST_F(Control, MountOffsetIsRequired) {
  path();geometry_msgs::msg::TransformStamped mount;mount.header.frame_id="drive_base_link";
  mount.child_frame_id="base_link";mount.transform.translation.x=.5;mount.transform.rotation.w=1;
  tf->setTransform(mount,"test",true);
  EXPECT_THROW(controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr),nav2_core::PlannerException);
}

TEST_F(Control, ReplansAroundObstacle) {
  nav_msgs::msg::Path route;route.header.frame_id="odom";
  for(int i=0;i<=20;++i)route.poses.push_back(pose(i*.2));controller->setPlan(route);
  unsigned int x,y;map->getCostmap()->worldToMap(2,0,x,y);
  map->getCostmap()->setCost(x,y,nav2_costmap_2d::LETHAL_OBSTACLE);
  auto cmd=controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr);
  EXPECT_GT(std::abs(cmd.twist.linear.x)+std::abs(cmd.twist.angular.z),0);
  EXPECT_LE(cmd.twist.linear.x,.15);EXPECT_LE(std::abs(cmd.twist.angular.z),maxAngularSpeed());
}
#include <fstream>
#include <cstdlib>
TEST_P(SpeedProfileControl, RecordedSiteRouteSnapshots) {
  const char *file=std::getenv("EGO_SITE_CASES");
  if(!file)GTEST_SKIP()<<"Set EGO_SITE_CASES to exported real-map snapshots";
  std::ifstream in(file,std::ios::binary);ASSERT_TRUE(in.good());
  uint32_t count;in.read(reinterpret_cast<char*>(&count),4);
  for(uint32_t k=0;k<count;++k) {
    uint32_t stage,index,n;double start[3];
    in.read(reinterpret_cast<char*>(&stage),4);in.read(reinterpret_cast<char*>(&index),4);in.read(reinterpret_cast<char*>(&n),4);
    in.read(reinterpret_cast<char*>(start),sizeof(start));
    nav_msgs::msg::Path route;route.header.frame_id="odom";
    for(uint32_t i=0;i<n;++i) {double p[3];in.read(reinterpret_cast<char*>(p),sizeof(p));route.poses.push_back(pose(p[0],p[1],p[2]));}
    auto *grid=map->getCostmap();grid->resizeMap(200,200,.05,start[0]-5,start[1]-5);
    in.read(reinterpret_cast<char*>(grid->getCharMap()),40000);ASSERT_TRUE(in.good());
    controller->setPlan(route);
    SCOPED_TRACE("stage="+std::to_string(stage)+" index="+std::to_string(index));
    try {auto cmd=controller->computeVelocityCommands(pose(start[0],start[1],start[2]),geometry_msgs::msg::Twist(),nullptr);
      EXPECT_LE(cmd.twist.linear.x,maxLinearSpeed());EXPECT_LE(std::abs(cmd.twist.angular.z),maxAngularSpeed());}
    catch(const std::exception &e) {ADD_FAILURE()<<e.what();}
  }
}
#include <rcl/time.h>
TEST_F(Control, CommandsRampWithStationaryOdometryAndResetAfterCollision) {
  parent->set_parameter(rclcpp::Parameter("use_sim_time",true));
  auto *clock=parent->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL),RCL_RET_OK);
  path();double last=0;
  for(int i=0;i<10;++i) {
    ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL+int64_t(i)*100000000LL),RCL_RET_OK);
    auto cmd=controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr);
    EXPECT_LE(cmd.twist.linear.x-last,.020001);last=cmd.twist.linear.x;
  }
  EXPECT_NEAR(last,.15,1e-6);
  nav_msgs::msg::Path rotate;rotate.header.frame_id="odom";rotate.poses.push_back(pose(0,0,1.57));
  controller->setPlan(rotate);last=0;
  for(int i=0;i<6;++i) {
    ASSERT_EQ(rcl_set_ros_time_override(clock,102000000000LL+int64_t(i)*100000000LL),RCL_RET_OK);
    auto cmd=controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr);
    EXPECT_LE(cmd.twist.angular.z-last,.040001);last=cmd.twist.angular.z;
    EXPECT_DOUBLE_EQ(cmd.twist.linear.x,0);
  }
  EXPECT_NEAR(last,.2,1e-6);
  unsigned int x,y;map->getCostmap()->worldToMap(.2,0,x,y);
  map->getCostmap()->setCost(x,y,nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr),nav2_core::PlannerException);
  map->getCostmap()->setCost(x,y,nav2_costmap_2d::FREE_SPACE);
  auto cmd=controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr);
  EXPECT_LE(cmd.twist.angular.z,.040001);
}
TEST_P(SpeedProfileControl, ClosedLoopDifferentialDriveDetour) {
  parent->set_parameter(rclcpp::Parameter("use_sim_time",true));
  auto *clock=parent->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL),RCL_RET_OK);
  nav_msgs::msg::Path route;route.header.frame_id="odom";
  for(int i=0;i<=20;++i)route.poses.push_back(pose(i*.2));controller->setPlan(route);
  unsigned int ox,oy;map->getCostmap()->worldToMap(2,0,ox,oy);
  map->getCostmap()->setCost(ox,oy,nav2_costmap_2d::LETHAL_OBSTACLE);
  std::ofstream trace;
  if(std::getenv("EGO_TRACE"))trace.open("/tmp/ego-detour-"+std::to_string(int(maxLinearSpeed()*100))+".csv");
  double x=0,y=0,yaw=0,max_deviation=0,travel=0;geometry_msgs::msg::Twist measured;
  bool reached=false;
  for(int i=0;i<1500;++i) {
    ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL+int64_t(i)*100000000LL),RCL_RET_OK);
    // Runtime uses a rolling local map. Keep the braking arc inside that map
    // as the robot advances, while retaining overlapping obstacle cells.
    map->getCostmap()->updateOrigin(x-5,y-5);
    geometry_msgs::msg::TwistStamped cmd;
    try {cmd=controller->computeVelocityCommands(pose(x,y,yaw),measured,nullptr);}
    catch(const std::exception &e) {FAIL()<<"step="<<i<<" pose="<<x<<","<<y<<","<<yaw<<": "<<e.what();}
    measured=cmd.twist;
    EXPECT_LE(cmd.twist.linear.x,maxLinearSpeed());
    EXPECT_LE(std::abs(cmd.twist.angular.z),maxAngularSpeed());
    x+=measured.linear.x*std::cos(yaw)*.1;y+=measured.linear.x*std::sin(yaw)*.1;yaw+=measured.angular.z*.1;
    max_deviation=std::max(max_deviation,std::abs(y));
    travel+=measured.linear.x*.1;
    if(trace.is_open())trace<<i<<","<<x<<","<<y<<","<<yaw<<","<<measured.linear.x<<","<<measured.angular.z<<"\n";
    EXPECT_GE(std::hypot(x+.25*std::cos(yaw)-2,y+.25*std::sin(yaw)),.60);
    if(std::hypot(x-4,y)<.08 && std::abs(std::atan2(std::sin(yaw),std::cos(yaw)))<.08){reached=true;break;}
  }
  EXPECT_TRUE(reached)<<x<<","<<y<<","<<yaw;EXPECT_GT(max_deviation,.6);
  std::cout<<"DETOUR speed="<<maxLinearSpeed()<<" max_offset="<<max_deviation<<" length="<<travel<<std::endl;
  EXPECT_LT(max_deviation,1.5);EXPECT_LT(travel,7.0);
}
TEST_P(SpeedProfileControl, NearbyGoalConvergesWithHeadingErrorAndPositionNoise) {
  parent->set_parameter(rclcpp::Parameter("use_sim_time",true));
  auto *clock=parent->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL),RCL_RET_OK);
  nav_msgs::msg::Path route;route.header.frame_id="odom";
  // A lattice path may contain a loop even when the goal is only 20 cm away.
  route.poses={pose(0,0,1.9),pose(-.2,.2),pose(.3,.2),pose(.2,0,0)};
  controller->setPlan(route);
  double x=0,y=0,yaw=1.9;geometry_msgs::msg::Twist measured;bool reached=false;
  for(int i=0;i<1000;++i) {
    ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL+int64_t(i)*100000000LL),RCL_RET_OK);
    auto cmd=controller->computeVelocityCommands(pose(x+.015*std::sin(i*.3),y+.015*std::cos(i*.3),yaw),measured,nullptr);
    measured=cmd.twist;measured.linear.x*=.8;measured.angular.z*=.8;
    x+=measured.linear.x*std::cos(yaw)*.1;y+=measured.linear.x*std::sin(yaw)*.1;yaw+=measured.angular.z*.1;
    if(std::hypot(x-.2,y)<.08 && std::abs(yaw)<.08){reached=true;break;}
  }
  EXPECT_TRUE(reached)<<x<<","<<y<<","<<yaw;
}
TEST_F(Control, NewObstacleInvalidatesCachedPath) {
  path();controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr);
  unsigned int x,y;map->getCostmap()->worldToMap(1.5,0,x,y);
  map->getCostmap()->setCost(x,y,nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr),nav2_core::PlannerException);
}

TEST_P(SpeedProfileControl, LongStraightCoverageWithNoiseAndSlowChassis) {
  parent->set_parameter(rclcpp::Parameter("use_sim_time",true));
  auto *clock=parent->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL),RCL_RET_OK);
  nav_msgs::msg::Path route;route.header.frame_id="odom";
  for(int i=0;i<=250;++i)route.poses.push_back(pose(i*.08));
  controller->setPlan(route);
  std::ofstream states,plans;
  if(std::getenv("EGO_TRACE")){states.open("/tmp/ego-longline-state.csv");plans.open("/tmp/ego-longline-plans.csv");}
  int tick=0;
  auto sub=parent->create_subscription<nav_msgs::msg::Path>("EgoTest/ego_local_plan",10,
    [&](nav_msgs::msg::Path::ConstSharedPtr p){if(plans.is_open())for(const auto &q:p->poses)
      plans<<tick<<","<<q.pose.position.x<<","<<q.pose.position.y<<"\n";});
  double x=0,y=-.12,yaw=.07,max_deviation=0,peak_speed=0;geometry_msgs::msg::Twist measured;
  bool reached=false;
  for(int i=0;i<3500;++i) {
    tick=i;
    ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL+int64_t(i)*100000000LL),RCL_RET_OK);
    map->getCostmap()->updateOrigin(x-5,y-5);
    auto cmd=controller->computeVelocityCommands(pose(x+.01*std::sin(i*.3),y+.01*std::cos(i*.3),yaw),measured,nullptr);
    peak_speed=std::max(peak_speed,cmd.twist.linear.x);
    EXPECT_LE(cmd.twist.linear.x,maxLinearSpeed());
    EXPECT_LE(std::abs(cmd.twist.angular.z),maxAngularSpeed());
    rclcpp::spin_some(parent->get_node_base_interface());
    if(states.is_open())states<<i<<","<<x<<","<<y<<","<<yaw<<","<<cmd.twist.linear.x<<","<<cmd.twist.angular.z<<"\n";
    measured.linear.x += .5*(.7*cmd.twist.linear.x-measured.linear.x);
    measured.angular.z += .5*(cmd.twist.angular.z-measured.angular.z);
    x+=measured.linear.x*std::cos(yaw)*.1;y+=measured.linear.x*std::sin(yaw)*.1;yaw+=measured.angular.z*.1;
    max_deviation=std::max(max_deviation,std::abs(y));
    if(std::hypot(x-20,y)<.08 && std::abs(yaw)<.08){reached=true;break;}
  }
  EXPECT_TRUE(reached)<<x<<","<<y<<","<<yaw;
  EXPECT_LT(max_deviation,.3);
  EXPECT_NEAR(peak_speed,maxLinearSpeed(),1e-6);
}

TEST_P(SpeedProfileControl, ClearedObstacleReturnsBeforeOldPathEnds) {
  parent->set_parameter(rclcpp::Parameter("use_sim_time",true));
  auto *clock=parent->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL),RCL_RET_OK);
  nav_msgs::msg::Path route;route.header.frame_id="odom";
  for(int i=0;i<=60;++i)route.poses.push_back(pose(i*.1));controller->setPlan(route);
  nav_msgs::msg::Path published;int publications=0;
  auto sub=parent->create_subscription<nav_msgs::msg::Path>("EgoTest/ego_local_plan",10,
    [&](nav_msgs::msg::Path::ConstSharedPtr p){published=*p;++publications;});
  for(int i=0;i<100 && !sub->get_publisher_count();++i)rclcpp::sleep_for(std::chrono::milliseconds(5));
  unsigned int ox,oy;map->getCostmap()->worldToMap(2,0,ox,oy);
  map->getCostmap()->setCost(ox,oy,nav2_costmap_2d::LETHAL_OBSTACLE);
  geometry_msgs::msg::Twist measured;double x=0,y=0,yaw=0;
  int tick=0;
  for(;tick<500;++tick) {
    ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL+int64_t(tick)*100000000LL),RCL_RET_OK);
    auto cmd=controller->computeVelocityCommands(pose(x,y,yaw),measured,nullptr);
    rclcpp::spin_some(parent->get_node_base_interface());
    measured=cmd.twist;x+=measured.linear.x*std::cos(yaw)*.1;y+=measured.linear.x*std::sin(yaw)*.1;yaw+=measured.angular.z*.1;
    if(std::abs(y)>.40 && x>.7)break;
  }
  ASSERT_LT(tick,500);ASSERT_FALSE(published.poses.empty());
  const auto old=published;
  ASSERT_GT(std::hypot(old.poses.back().pose.position.x-x,old.poses.back().pose.position.y-y),1.2);
  map->getCostmap()->setCost(ox,oy,nav2_costmap_2d::FREE_SPACE);
  // Match the real rolling costmap: the new 3.5 m horizon must not hit the
  // edge of a test map still centered at the initial robot position.
  map->getCostmap()->updateOrigin(x-5,y-5);
  const int before=publications;
  // Freeze observed pose: only the new return trigger can replace this plan;
  // the normal "within 1 m of cached endpoint" trigger remains false.
  bool replaced=false;
  for(int j=1;j<=30;++j) {
    ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL+int64_t(tick+j)*100000000LL),RCL_RET_OK);
    controller->computeVelocityCommands(pose(x,y,yaw),measured,nullptr);
    rclcpp::spin_some(parent->get_node_base_interface());
    if(publications>before){EXPECT_GE(j,5);replaced=true;break;}
  }
  EXPECT_TRUE(replaced);
  auto excess=[](const nav_msgs::msg::Path &path) {
    double sum=0;for(const auto &p:path.poses)sum+=std::abs(p.pose.position.y);
    return sum/path.poses.size();
  };
  EXPECT_LT(excess(published),excess(old));
}

TEST_F(Control, NearbyLaterLaneCannotStealCurrentSegment) {
  nav_msgs::msg::Path route;route.header.frame_id="odom";
  for(int i=0;i<=20;++i)route.poses.push_back(pose(i*.1,0));
  for(int i=1;i<=3;++i)route.poses.push_back(pose(2,i*.1));
  for(int i=19;i>=-10;--i)route.poses.push_back(pose(i*.1,.3,3.14159265359));
  controller->setPlan(route);
  // The later reverse lane is closer, but belongs after the uncompleted turn.
  auto cmd=controller->computeVelocityCommands(pose(0,.25,0),geometry_msgs::msg::Twist(),nullptr);
  EXPECT_GT(cmd.twist.linear.x,0);
}

TEST_F(Control, HeadingHysteresisPreventsRepeatedRestartNearThreshold) {
  parent->set_parameter(rclcpp::Parameter("use_sim_time",true));
  auto *clock=parent->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL),RCL_RET_OK);
  nav_msgs::msg::Path route;route.header.frame_id="odom";
  route.poses={pose(),pose(.4)};controller->setPlan(route);
  geometry_msgs::msg::Twist measured;
  const std::vector<double> errors{.51,.49,.51,.48,.30,.24};
  for(size_t i=0;i<errors.size();++i) {
    ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL+int64_t(i)*100000000LL),RCL_RET_OK);
    auto cmd=controller->computeVelocityCommands(pose(0,0,-errors[i]),measured,nullptr);
    if(i+1<errors.size())EXPECT_DOUBLE_EQ(cmd.twist.linear.x,0);
    else EXPECT_GT(cmd.twist.linear.x,0);
    EXPECT_GT(cmd.twist.angular.z,0);measured=cmd.twist;
  }
}

TEST_F(Control, AccumulatedMapRotationInvalidatesLongCachedPathImmediately) {
  parent->set_parameter(rclcpp::Parameter("use_sim_time",true));
  auto *clock=parent->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL),RCL_RET_OK);
  geometry_msgs::msg::TransformStamped correction;correction.header.frame_id="map";
  correction.child_frame_id="odom";correction.transform.rotation.w=1;
  tf->setTransform(correction,"test",true);
  nav_msgs::msg::Path route;route.header.frame_id="map";
  for(int i=0;i<=20;++i){auto p=pose(i*.2);p.header.frame_id="map";route.poses.push_back(p);}
  controller->setPlan(route);geometry_msgs::msg::Twist measured;
  auto cmd=controller->computeVelocityCommands(pose(),measured,nullptr);
  EXPECT_NEAR(cmd.twist.angular.z,0,1e-4);
  for(int i=1;i<=5;++i) {
    const double yaw=i*.004;
    correction.transform.rotation.z=std::sin(yaw/2);correction.transform.rotation.w=std::cos(yaw/2);
    tf->setTransform(correction,"test",true);
    ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL+int64_t(i)*100000000LL),RCL_RET_OK);
    cmd=controller->computeVelocityCommands(pose(),measured,nullptr);
  }
  // Robot has not moved, old endpoint is still > 3 m away, old path age < 3 s.
  // Only frame invalidation can turn toward the newly rotated map line now.
  EXPECT_LT(cmd.twist.angular.z,-1e-4);
}

TEST_F(Control, FailedReplanAfterMapCorrectionCannotReuseOldSafePath) {
  geometry_msgs::msg::TransformStamped correction;correction.header.frame_id="map";
  correction.child_frame_id="odom";correction.transform.rotation.w=1;
  tf->setTransform(correction,"test",true);
  nav_msgs::msg::Path route;route.header.frame_id="map";
  for(int i=0;i<=20;++i){auto p=pose(i*.2);p.header.frame_id="map";route.poses.push_back(p);}
  controller->setPlan(route);
  controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr);
  correction.transform.translation.y=5.;tf->setTransform(correction,"test",true);
  EXPECT_THROW(controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr),nav2_core::PlannerException);
  correction.transform.translation.y=0.;tf->setTransform(correction,"test",true);
  EXPECT_NO_THROW(controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr));
}

TEST_P(SpeedProfileControl, RecordedOctober27BlockedHorizonTarget) {
  // 27日因避开障碍物而停止, 1793093801.4: raw local costmap in odom.
  // These are all eight lethal cells. Other recorded costs are <254 and do
  // not enter EGO's obstacle mask or footprint rejection. No synthetic ROS replay.
  auto *grid=map->getCostmap();grid->resizeMap(200,200,.05,-20.95,-4.0);
  for(const auto &cell:std::vector<std::pair<unsigned int,unsigned int>>{
      {44,93},{44,94},{45,94},{44,98},{43,99},{44,99},{43,100},{44,100}})
    grid->setCost(cell.first,cell.second,nav2_costmap_2d::LETHAL_OBSTACLE);
  nav_msgs::msg::Path route;route.header.frame_id="odom";
  // The original straight third cleaning row, transformed using recorded TF.
  // Prefix already traversed at capture time is omitted; goal is unchanged.
  for(int i=0;i<889;++i) {
    const double f=double(i)/888;
    route.poses.push_back(pose(-15.990677016617+f*(-8.876636040967),
                               1.013916418856+f*(0.009418864239),-3.140532));
  }
  for(double speed:{0.,maxLinearSpeed()}) {
    controller->setPlan(route);geometry_msgs::msg::Twist measured;measured.linear.x=speed;
    auto cmd=controller->computeVelocityCommands(pose(-15.988496,1.008495,-3.089028),measured,nullptr);
    EXPECT_GT(std::abs(cmd.twist.linear.x)+std::abs(cmd.twist.angular.z),0);
    EXPECT_LE(cmd.twist.linear.x,maxLinearSpeed());
    EXPECT_LE(std::abs(cmd.twist.angular.z),maxAngularSpeed());
  }
}

TEST_P(SpeedProfileControl, NewlyBlockedHorizonDetoursAndReturnsToActualGoal) {
  parent->set_parameter(rclcpp::Parameter("use_sim_time",true));
  auto *clock=parent->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL),RCL_RET_OK);
  nav_msgs::msg::Path route;route.header.frame_id="odom";
  for(int i=0;i<=400;++i)route.poses.push_back(pose(i*.02));controller->setPlan(route);
  controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr);
  double x=0,y=0,yaw=0,max_offset=0;int failures=0,max_failures=0;bool reached=false;
  geometry_msgs::msg::Twist measured;
  for(int i=1;i<=2400;++i) {
    ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL+int64_t(i)*100000000LL),RCL_RET_OK);
    auto *grid=map->getCostmap();grid->updateOrigin(x-5,y-5);
    unsigned int mx,my;if(grid->worldToMap(2.7,0,mx,my))grid->setCost(mx,my,nav2_costmap_2d::LETHAL_OBSTACLE);
    geometry_msgs::msg::Twist cmd;
    try {cmd=controller->computeVelocityCommands(pose(x,y,yaw),measured,nullptr).twist;failures=0;}
    catch(const nav2_core::PlannerException &e) {
      max_failures=std::max(max_failures,++failures);
      ASSERT_LT(failures,100)<<"10 s patience exceeded at "<<x<<","<<y<<": "<<e.what();
    }
    measured=cmd;
    EXPECT_LE(cmd.linear.x,maxLinearSpeed());EXPECT_LE(std::abs(cmd.angular.z),maxAngularSpeed());
    x+=cmd.linear.x*std::cos(yaw)*.1;y+=cmd.linear.x*std::sin(yaw)*.1;yaw+=cmd.angular.z*.1;
    max_offset=std::max(max_offset,std::abs(y));
    EXPECT_GE(std::hypot(x+.25*std::cos(yaw)-2.7,y+.25*std::sin(yaw)),.60);
    if(std::hypot(x-8,y)<.08 && std::abs(std::atan2(std::sin(yaw),std::cos(yaw)))<.08){reached=true;break;}
  }
  EXPECT_TRUE(reached)<<x<<","<<y<<","<<yaw;
  EXPECT_GT(max_offset,.6);EXPECT_LT(max_offset,1.8);
  std::cout<<"BLOCKED_HORIZON speed="<<maxLinearSpeed()<<" max_offset="<<max_offset
           <<" longest_failed_cycles="<<max_failures<<std::endl;
}

TEST_F(Control, BlockedActualGoalIsNotReplacedWithLateralGoal) {
  path();unsigned int x,y;map->getCostmap()->worldToMap(3,0,x,y);
  map->getCostmap()->setCost(x,y,nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr),nav2_core::PlannerException);
}

TEST_F(Control, FullyBlockedHorizonStillRejectsMotion) {
  nav_msgs::msg::Path route;route.header.frame_id="odom";
  for(int i=0;i<=400;++i)route.poses.push_back(pose(i*.02));controller->setPlan(route);
  auto *grid=map->getCostmap();
  for(double y=-4.9;y<5;y+=.025){unsigned int mx,my;ASSERT_TRUE(grid->worldToMap(2.7,y,mx,my));
    grid->setCost(mx,my,nav2_costmap_2d::LETHAL_OBSTACLE);}
  EXPECT_THROW(controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr),nav2_core::PlannerException);
}

TEST_F(Control, BlockedHorizonAfterMapCorrectionCanReplan) {
  geometry_msgs::msg::TransformStamped correction;correction.header.frame_id="map";
  correction.child_frame_id="odom";correction.transform.rotation.w=1;
  tf->setTransform(correction,"test",true);
  nav_msgs::msg::Path route;route.header.frame_id="map";
  for(int i=0;i<=400;++i){auto p=pose(i*.02);p.header.frame_id="map";route.poses.push_back(p);}
  controller->setPlan(route);
  controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr);
  correction.transform.translation.y=.04;tf->setTransform(correction,"test",true);
  unsigned int x,y;map->getCostmap()->worldToMap(2.7,-.04,x,y);
  map->getCostmap()->setCost(x,y,nav2_costmap_2d::LETHAL_OBSTACLE);
  auto cmd=controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr);
  EXPECT_GT(std::abs(cmd.twist.linear.x)+std::abs(cmd.twist.angular.z),0);
  EXPECT_LE(cmd.twist.linear.x,maxLinearSpeed());EXPECT_LE(std::abs(cmd.twist.angular.z),maxAngularSpeed());
}

TEST_F(Control, UnknownSideIsRejectedAndFreeSideUsed) {
  nav_msgs::msg::Path::ConstSharedPtr accepted;
  auto sub=parent->create_subscription<nav_msgs::msg::Path>("EgoTest/ego_local_plan",10,
    [&](nav_msgs::msg::Path::ConstSharedPtr p){accepted=p;});
  nav_msgs::msg::Path route;route.header.frame_id="odom";
  for(int i=0;i<=400;++i)route.poses.push_back(pose(i*.02));controller->setPlan(route);
  auto *grid=map->getCostmap();
  // Keep the start outside the optimizer's 0.90 m inflated unknown boundary.
  for(unsigned int x=0;x<200;++x)for(unsigned int y=124;y<200;++y)
    grid->setCost(x,y,nav2_costmap_2d::NO_INFORMATION);
  unsigned int x,y;grid->worldToMap(2.7,0,x,y);grid->setCost(x,y,nav2_costmap_2d::LETHAL_OBSTACLE);
  auto cmd=controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr);
  // The spline may initially align before turning away. Verify its actual
  // accepted endpoint and clearance, not the sign of one angular command.
  for(int i=0;i<100 && !accepted;++i) {
    rclcpp::spin_some(parent->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_TRUE(accepted);ASSERT_FALSE(accepted->poses.empty());
  EXPECT_LT(accepted->poses.back().pose.position.y,0);
  for(const auto &p:accepted->poses)EXPECT_LT(p.pose.position.y,.35);
}

TEST_F(Control, UnsafeBodyStartIsNotMovedToEscapeObstacle) {
  nav_msgs::msg::Path route;route.header.frame_id="odom";
  for(int i=0;i<=400;++i)route.poses.push_back(pose(i*.02));controller->setPlan(route);
  unsigned int x,y;auto *grid=map->getCostmap();
  // Inside the actual body-center clearance, not only the search envelope.
  grid->worldToMap(-.30,0,x,y);grid->setCost(x,y,nav2_costmap_2d::LETHAL_OBSTACLE);
  grid->worldToMap(2.7,0,x,y);grid->setCost(x,y,nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr),nav2_core::PlannerException);
}

#include "ego_nav2_controller/obstacle_memory.hpp"
class MemoryControl : public SpeedProfileControl {
protected:
  std::shared_ptr<nav2_costmap_2d::Layer> memory;
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> memory_loader{"nav2_costmap_2d","nav2_costmap_2d::Layer"};
  void SetUp() override {
    SpeedProfileControl::SetUp();
    memory=memory_loader.createSharedInstance("ego_nav2_controller::ObstacleMemory");
    memory->initialize(map->getLayeredCostmap(),"obstacle_memory",tf.get(),parent,
      parent->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
  }
  void TearDown() override {memory.reset();SpeedProfileControl::TearDown();}
  void merge(double x,double y) {
    double lx=1e9,ly=1e9,hx=-1e9,hy=-1e9;
    memory->updateBounds(x,y,0,&lx,&ly,&hx,&hy);
    auto *g=map->getCostmap();memory->updateCosts(*g,0,0,g->getSizeInCellsX(),g->getSizeInCellsY());
  }
};
INSTANTIATE_TEST_SUITE_P(ApproachAndCoverage, MemoryControl, ::testing::Values(.25,.40));

TEST_P(MemoryControl, MissingNearObstacleRemainsBlockedAcrossResetAndNewGoal) {
  auto *g=map->getCostmap();unsigned int ox,oy;g->worldToMap(2,0,ox,oy);
  g->setCost(ox,oy,nav2_costmap_2d::LETHAL_OBSTACLE);merge(0,0);
  g->resetMap(0,0,200,200);memory->reset();merge(1.4,0);
  EXPECT_FALSE(memory->isClearable());EXPECT_EQ(g->getCost(ox,oy),254);
  path();EXPECT_THROW(controller->computeVelocityCommands(pose(1.4),geometry_msgs::msg::Twist(),nullptr),nav2_core::PlannerException);
  // Time passing, a recovery reset, and a new action are not clearance evidence.
  g->resetMap(0,0,200,200);merge(1.4,0);path();
  EXPECT_THROW(controller->computeVelocityCommands(pose(1.4),geometry_msgs::msg::Twist(),nullptr),nav2_core::PlannerException);
  // Once the robot is well away, memory is released; old observations don't
  // survive a later visit. Incoming live lethal cells would still be retained.
  g->updateOrigin(1,-5);g->resetMap(0,0,200,200);merge(6,0);
  ASSERT_TRUE(g->worldToMap(2,0,ox,oy));EXPECT_EQ(g->getCost(ox,oy),0);
  g->resetMap(0,0,200,200);merge(1.4,0);EXPECT_EQ(g->getCost(ox,oy),0);
}

TEST_P(MemoryControl, BlindSpotAndRepeatedMapCorrectionsKeepDetourSafe) {
  parent->set_parameter(rclcpp::Parameter("use_sim_time",true));
  auto *clock=parent->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL),RCL_RET_OK);
  geometry_msgs::msg::TransformStamped correction;correction.header.frame_id="map";
  correction.child_frame_id="odom";correction.transform.rotation.w=1;
  tf->setTransform(correction,"test",true);
  nav_msgs::msg::Path route;route.header.frame_id="map";
  for(int i=0;i<=400;++i){auto p=pose(i*.02);p.header.frame_id="map";route.poses.push_back(p);}
  controller->setPlan(route);
  double x=0,y=0,yaw=0,min_clear=99,max_offset=0;bool reached=false,hidden=false;
  int failures=0;geometry_msgs::msg::Twist measured;
  for(int i=0;i<2400;++i) {
    ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL+int64_t(i)*100000000LL),RCL_RET_OK);
    correction.transform.translation.y=.04*std::sin(i*.07);tf->setTransform(correction,"test",true);
    auto *g=map->getCostmap();g->updateOrigin(x-5,y-5);g->resetMap(0,0,200,200);
    // Initially visible, then gone from raw STVL for the entire encounter.
    if(x<.7){unsigned int mx,my;g->worldToMap(2.7,0,mx,my);g->setCost(mx,my,254);}
    else hidden=true;
    merge(x,y);geometry_msgs::msg::Twist cmd;
    try {cmd=controller->computeVelocityCommands(pose(x,y,yaw),measured,nullptr).twist;failures=0;}
    catch(const nav2_core::PlannerException &e){ASSERT_LT(++failures,100)<<x<<","<<y<<": "<<e.what();}
    measured=cmd;x+=cmd.linear.x*std::cos(yaw)*.1;y+=cmd.linear.x*std::sin(yaw)*.1;yaw+=cmd.angular.z*.1;
    min_clear=std::min(min_clear,std::hypot(x+.25*std::cos(yaw)-2.7,y+.25*std::sin(yaw)));
    max_offset=std::max(max_offset,std::abs(y));
    EXPECT_GE(min_clear,.60);
    if(std::hypot(x-8,y)<.08 && std::abs(std::atan2(std::sin(yaw),std::cos(yaw)))<.08){reached=true;break;}
  }
  EXPECT_TRUE(hidden);EXPECT_TRUE(reached)<<x<<","<<y;
  EXPECT_GT(max_offset,.6);EXPECT_LT(max_offset,1.8);
  std::cout<<"BLIND_SPOT speed="<<maxLinearSpeed()<<" minimum_center_distance="<<min_clear<<" max_offset="<<max_offset<<std::endl;
}

TEST_P(MemoryControl, RecordedBlindSpotSequence) {
  const char *file=std::getenv("EGO_BLINDSPOT_CASES");
  if(!file)GTEST_SKIP()<<"Set EGO_BLINDSPOT_CASES to export_blindspot_bag.py output";
  std::ifstream in(file,std::ios::binary);ASSERT_TRUE(in.good());
  uint32_t count;in.read(reinterpret_cast<char*>(&count),4);
  int dangerous_raw_frames=0,blocked_frames=0,retained_frames=0;
  for(uint32_t k=0;k<count;++k) {
    double p[3],meta[3];uint32_t nx,ny;
    in.read(reinterpret_cast<char*>(p),sizeof(p));
    in.read(reinterpret_cast<char*>(&nx),4);in.read(reinterpret_cast<char*>(&ny),4);
    in.read(reinterpret_cast<char*>(meta),sizeof(meta));
    auto *g=map->getCostmap();g->resizeMap(nx,ny,meta[0],meta[1],meta[2]);
    in.read(reinterpret_cast<char*>(g->getCharMap()),nx*ny);ASSERT_TRUE(in.good());
    const auto nearest=[&]() {
      double result=99;const double cx=p[0]+.25*std::cos(p[2]),cy=p[1]+.25*std::sin(p[2]);
      for(unsigned int x=0;x<nx;++x)for(unsigned int y=0;y<ny;++y)if(g->getCost(x,y)==254) {
        double wx,wy;g->mapToWorld(x,y,wx,wy);result=std::min(result,std::hypot(wx-cx,wy-cy));
      }
      return result;
    };
    double before=nearest();merge(p[0],p[1]);double after=nearest();
    if(after<before-.01)++retained_frames;
    // Classify retained obstacles against the current center hard limit.
    if(before>.7 && after<.60) {
      ++dangerous_raw_frames;
      nav_msgs::msg::Path route;route.header.frame_id="odom";
      route.poses={pose(p[0],p[1],p[2]),pose(p[0]+2*std::cos(p[2]),p[1]+2*std::sin(p[2]),p[2])};
      controller->setPlan(route);
      try {controller->computeVelocityCommands(pose(p[0],p[1],p[2]),geometry_msgs::msg::Twist(),nullptr);}
      catch(const nav2_core::PlannerException &e) {
        EXPECT_NE(std::string(e.what()).find("Current pose violates"),std::string::npos);++blocked_frames;
      }
    }
  }
  EXPECT_GT(retained_frames,0);EXPECT_GT(dangerous_raw_frames,0);
  EXPECT_EQ(blocked_frames,dangerous_raw_frames);
  std::cout<<"BAG_BLIND_SPOT frames="<<count<<" retained_frames="<<retained_frames
           <<" unsafe_recorded_poses="<<dangerous_raw_frames<<" rejected="<<blocked_frames<<std::endl;
}

class OneObservation : public nav2_costmap_2d::Layer {
public:
  bool visible{true};
  void onInitialize() override {enabled_=true;current_=true;}
  void reset() override {}
  bool isClearable() override {return true;}
  void updateBounds(double,double,double,double*,double*,double*,double*) override {}
  void updateCosts(nav2_costmap_2d::Costmap2D &g,int,int,int,int) override {
    unsigned int x,y;if(visible && g.worldToMap(2,0,x,y))g.setCost(x,y,254);
  }
};
TEST_P(MemoryControl, LayeredUpdatesPreserveThenReleaseWithoutSelfRelatching) {
  auto *layers=map->getLayeredCostmap();auto source=std::make_shared<OneObservation>();
  source->initialize(layers,"test_source",tf.get(),parent,
    parent->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
  layers->addPlugin(source);layers->addPlugin(memory);
  auto *g=layers->getCostmap();unsigned int x,y;
  layers->updateMap(0,0,0);source->visible=false;
  // Exercise the actual Nav2 master reset/order/bounds and rolling origin.
  for(int i=0;i<20;++i) {
    layers->updateMap(.5+i*.025,0,0);ASSERT_TRUE(g->worldToMap(2,0,x,y));EXPECT_EQ(g->getCost(x,y),254);
  }
  layers->updateMap(6,0,0);ASSERT_TRUE(g->worldToMap(2,0,x,y));EXPECT_EQ(g->getCost(x,y),0);
  layers->updateMap(1,0,0);ASSERT_TRUE(g->worldToMap(2,0,x,y));EXPECT_EQ(g->getCost(x,y),0);
}

TEST_F(Control, FeasibleStartOutsideBodyClearanceButInsideOldAxleEnvelope) {
  // Recorded failures: axle 0.755/0.824 m, body center 0.714/0.726 m.
  // A point beside and slightly behind the axle must not immobilize a robot
  // whose full forward route and body-center circle are clear.
  path();unsigned int x,y;
  map->getCostmap()->worldToMap(-.10,.75,x,y);
  map->getCostmap()->setCost(x,y,nav2_costmap_2d::LETHAL_OBSTACLE);
  auto cmd=controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr);
  EXPECT_GT(cmd.twist.linear.x,0);
  EXPECT_LE(std::abs(cmd.twist.angular.z),maxAngularSpeed());
}

TEST_F(Control, TerminalSearchRetryKeepsGoalAndStopsForNewObstacle) {
  path();unsigned int x,y;auto *grid=map->getCostmap();
  ASSERT_TRUE(grid->worldToMap(3.025,.825,x,y));grid->setCost(x,y,254);
  auto cmd=controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr);
  EXPECT_GT(cmd.twist.linear.x,0);
  // A new obstacle at the real goal must invalidate the cached retry trajectory.
  ASSERT_TRUE(grid->worldToMap(3.025,0,x,y));grid->setCost(x,y,254);
  EXPECT_THROW(controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr),nav2_core::PlannerException);
}
TEST_F(Control, TerminalRetryRejectsActualGoalCollisionAndUnknown) {
  auto *grid=map->getCostmap();unsigned int x,y;
  for(unsigned char cost:{static_cast<unsigned char>(254),static_cast<unsigned char>(255)}) {
    grid->resetMap(0,0,200,200);path();
    ASSERT_TRUE(grid->worldToMap(3.025,.525,x,y));grid->setCost(x,y,cost);
    EXPECT_THROW(controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr),nav2_core::PlannerException);
  }
}
TEST_F(Control, TerminalRetryRejectsBlockedCorridor) {
  path();auto *grid=map->getCostmap();unsigned int x,y;
  ASSERT_TRUE(grid->worldToMap(3.025,.825,x,y));grid->setCost(x,y,254);
  ASSERT_TRUE(grid->worldToMap(1.5,0,x,y));
  for(unsigned j=0;j<grid->getSizeInCellsY();++j)grid->setCost(x,j,254);
  EXPECT_THROW(controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr),nav2_core::PlannerException);
}
class TerminalSnapshotControl : public Control {
protected:
  double footprintPadding() const override {return .05;}
  std::string footprintSpec() const override {return "[[0.75,0.325],[0.75,-0.325],[-0.25,-0.325],[-0.25,0.325]]";}
  double maxLinearSpeed() const override {return .60;}
  double maxAngularSpeed() const override {return .30;}
  bool smoothDetourEnabled() const override {return true;}
};
TEST_F(TerminalSnapshotControl, RecordedObstacleStopExactGoal) {
  const char *file=std::getenv("EGO_TERMINAL_SNAPSHOT");
  if(!file)GTEST_SKIP()<<"Set EGO_TERMINAL_SNAPSHOT to ego_snapshot.txt";
  parent->get_logger().set_level(rclcpp::Logger::Level::Debug);
  std::ifstream in(file);ASSERT_TRUE(in.good());
  double res,ox,oy,x,y,yaw,gx,gy;unsigned w,h;
  in>>res>>ox>>oy>>w>>h>>x>>y>>yaw>>gx>>gy;
  ASSERT_TRUE(in.good());ASSERT_GT(w,0u);ASSERT_GT(h,0u);
  auto *grid=map->getCostmap();grid->resizeMap(w,h,res,ox,oy);
  for(unsigned j=0;j<h;++j)for(unsigned i=0;i<w;++i){int cost;in>>cost;grid->setCost(i,j,cost);}
  ASSERT_FALSE(in.fail());
  const double heading=std::atan2(gy-y,gx-x);
  nav_msgs::msg::Path route;route.header.frame_id="odom";
  for(int i=0;i<=100;++i)route.poses.push_back(pose(x+(gx-x)*i/100,y+(gy-y)*i/100,heading));
  controller->setPlan(route);
  const auto start=std::chrono::steady_clock::now();
  auto cmd=controller->computeVelocityCommands(pose(x,y,yaw),geometry_msgs::msg::Twist(),nullptr);
  EXPECT_GT(cmd.twist.linear.x,0);EXPECT_LE(cmd.twist.linear.x,.6);
  EXPECT_LE(std::abs(cmd.twist.angular.z),.3);
  std::cout<<"TERMINAL_SNAPSHOT cycle_seconds="<<std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<<std::endl;
}

#include "ego_nav2_controller/trajectory_validation.hpp"
TEST(TrajectorySweep, ChecksBetweenSamplesAndRotationAndDeadline) {
  auto noop=[](){};
  auto obstacle=[](double x,double,double,double margin){return std::abs(x-.5)<=.01+margin;};
  EXPECT_FALSE(ego_nav2_controller::safeMotionSegment(0,0,0,1,0,0,1,.05,obstacle,noop));
  auto rotation=[](double,double,double yaw,double margin){return std::abs(yaw-.5)<=.01+margin;};
  EXPECT_FALSE(ego_nav2_controller::safeMotionSegment(0,0,0,0,0,1,1,.05,rotation,noop));
  auto clear=[](double,double,double,double){return false;};
  EXPECT_TRUE(ego_nav2_controller::safeMotionSegment(0,0,0,1,0,0,1,.05,clear,noop));
  EXPECT_THROW(ego_nav2_controller::safeMotionSegment(0,0,0,1,0,0,1,.05,clear,
    [](){throw std::runtime_error("deadline");}),std::runtime_error);
}

TEST_F(Control, TerminalRetryConvergesToOriginalGoal) {
  parent->set_parameter(rclcpp::Parameter("use_sim_time",true));
  auto *clock=parent->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL),RCL_RET_OK);
  path();unsigned int mx,my;auto *grid=map->getCostmap();
  ASSERT_TRUE(grid->worldToMap(3.025,.825,mx,my));grid->setCost(mx,my,254);
  double x=0,y=0,yaw=0;bool reached=false;geometry_msgs::msg::Twist measured;
  for(int i=0;i<500;++i) {
    ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL+int64_t(i)*100000000LL),RCL_RET_OK);
    const auto cmd=controller->computeVelocityCommands(pose(x,y,yaw),measured,nullptr).twist;
    EXPECT_LE(cmd.linear.x,.15);EXPECT_LE(std::abs(cmd.angular.z),.2);
    x+=cmd.linear.x*std::cos(yaw)*.1;y+=cmd.linear.x*std::sin(yaw)*.1;yaw+=cmd.angular.z*.1;measured=cmd;
    EXPECT_GE(std::hypot(x+.25*std::cos(yaw)-3.025,y+.25*std::sin(yaw)-.825),.60);
    if(std::hypot(x-3,y)<.08 && std::abs(yaw)<.08 && cmd.linear.x==0){reached=true;break;}
  }
  EXPECT_TRUE(reached)<<x<<","<<y<<","<<yaw;
}
class TerminalDeadlineControl : public Control {
protected:
  double planningBudget() const override {return 1e-9;}
};
TEST_F(TerminalDeadlineControl, RejectsExpiredBudget) {
  path();unsigned int x,y;auto *grid=map->getCostmap();
  ASSERT_TRUE(grid->worldToMap(3.025,.825,x,y));grid->setCost(x,y,254);
  try {
    controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr);
    FAIL()<<"An expired budget must not produce a command";
  } catch(const nav2_core::PlannerException &e) {
    EXPECT_NE(std::string(e.what()).find("time budget"),std::string::npos);
  }
}

class ContinuousGoalChecker : public nav2_core::GoalChecker {
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &,const std::string &,
                  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void reset() override {}
  bool isGoalReached(const geometry_msgs::msg::Pose &,const geometry_msgs::msg::Pose &,
                     const geometry_msgs::msg::Twist &) override {return false;}
  bool getTolerances(geometry_msgs::msg::Pose &p,geometry_msgs::msg::Twist &) override {
    p.position.x=p.position.y=.04;p.orientation.z=sin(.06/2);p.orientation.w=cos(.06/2);return true;
  }
};
class ContinuousControl : public Control {
protected:
  bool continuousCurve() const override {return true;}
  double maxLinearSpeed() const override {return .08;}
  double maxAngularSpeed() const override {return .30;}
  std::vector<geometry_msgs::msg::PoseStamped> semicircle() {
    std::vector<geometry_msgs::msg::PoseStamped> points;
    for(int i=0;i<=200;++i)points.push_back(pose(i*.01));
    for(int i=1;i<=400;++i) {double a=M_PI*i/400;points.push_back(pose(2+.5*sin(a),.5-.5*cos(a),a));}
    for(int i=1;i<=200;++i)points.push_back(pose(2-i*.01,1,M_PI));
    return points;
  }
  void drive(const std::vector<geometry_msgs::msg::PoseStamped> &points,int steps,bool lagged=false,bool expect_cruise=false,double lateral_bias=0,double yaw_gain=1.,bool expect_fast_bend=false,double correction_step=0) {
    parent->set_parameter(rclcpp::Parameter("use_sim_time",true));
    auto *clock=parent->get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL),RCL_RET_OK);
    nav_msgs::msg::Path route;route.header.frame_id="odom";route.poses=points;controller->setPlan(route);
    double x=points.front().pose.position.x,y=points.front().pose.position.y;
    double yaw=2*atan2(points.front().pose.orientation.z,points.front().pose.orientation.w);
    const auto &goal=points.back().pose.position;bool reached=false;geometry_msgs::msg::Twist measured;
    ContinuousGoalChecker checker;
    std::deque<geometry_msgs::msg::Twist> delayed(2);
    double actual_v=0,actual_w=0,previous_command_w=0,peak_v=0,bend_peak=0;
    for(int i=0;i<steps;++i) {
      ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL+int64_t(i)*100000000LL),RCL_RET_OK);
      map->getCostmap()->updateOrigin(x-5,y-5);
      geometry_msgs::msg::Twist cmd;
      const double lateral_noise=lateral_bias+correction_step*((i%100>=50)?.5:-.5)+(lagged?(.014*std::sin(i*1.7)+.008*std::sin(i*.31)):0);
      const double heading_noise=lagged?.008*std::sin(i*.73):0;
      try {cmd=controller->computeVelocityCommands(pose(x-lateral_noise*sin(yaw),y+lateral_noise*cos(yaw),yaw+heading_noise),measured,&checker).twist;}
      catch(const std::exception &e){FAIL()<<"step="<<i<<" pose="<<x<<","<<y<<","<<yaw<<": "<<e.what();}
      peak_v=std::max(peak_v,cmd.linear.x);
      if(std::abs(yaw)>.5 && std::abs(yaw)<2.5)bend_peak=std::max(bend_peak,cmd.linear.x);
      EXPECT_LE(cmd.linear.x,maxLinearSpeed()+1e-6);EXPECT_LE(abs(cmd.angular.z),.300001);
      EXPECT_LE(abs(cmd.angular.z-previous_command_w),.040001);previous_command_w=cmd.angular.z;
      if(lagged) {
        delayed.push_back(cmd);const auto applied=delayed.front();delayed.pop_front();
        actual_v+=(applied.linear.x-actual_v)*(1-std::exp(-.1/.2));
        actual_w+=(yaw_gain*applied.angular.z-actual_w)*(1-std::exp(-.1/.35));
        measured.linear.x=actual_v;measured.angular.z=actual_w+.005*std::sin(i*.51);
      } else {actual_v=cmd.linear.x;actual_w=cmd.angular.z;measured=cmd;}
      x+=actual_v*cos(yaw)*.1;y+=actual_v*sin(yaw)*.1;yaw+=actual_w*.1;
      if(std::hypot(x-goal.x,y-goal.y)<.04 && cmd.linear.x==0 && cmd.angular.z==0){reached=true;break;}
      if(i%5000==0)std::cout<<"CONTINUOUS step="<<i<<" x="<<x<<" y="<<y<<std::endl;
    }
    EXPECT_TRUE(reached)<<x<<","<<y;
    if(expect_cruise)EXPECT_GT(peak_v,.79);
    if(expect_fast_bend)EXPECT_GT(bend_peak,.19);
    std::cout<<"CONTINUOUS peak="<<peak_v<<std::endl;
  }
};
TEST_F(ContinuousControl, FollowsHalfMeterTurnWithoutSpin) {drive(semicircle(),1100);}
TEST_F(ContinuousControl, RejectsBlockedRouteAndTrackingJump) {
  nav_msgs::msg::Path route;route.header.frame_id="odom";route.poses=semicircle();controller->setPlan(route);
  unsigned int x,y;ASSERT_TRUE(map->getCostmap()->worldToMap(1,0,x,y));map->getCostmap()->setCost(x,y,254);
  EXPECT_THROW(controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr),nav2_core::PlannerException);
  map->getCostmap()->setCost(x,y,0);
  EXPECT_THROW(controller->computeVelocityCommands(pose(0,.2),geometry_msgs::msg::Twist(),nullptr),nav2_core::PlannerException);
}
TEST_F(ContinuousControl, FullNewRouteInDifferentialDriveSimulation) {
  const char *file=std::getenv("EGO_CONTINUOUS_ROUTE");if(!file)GTEST_SKIP()<<"Optional new route pose export";
  std::ifstream in(file);ASSERT_TRUE(in.good());double x,y,a;std::vector<geometry_msgs::msg::PoseStamped> points;
  while(in>>x>>y>>a)points.push_back(pose(x,y,a));ASSERT_GT(points.size(),30000u);
  drive(points,41000,std::getenv("EGO_CONTINUOUS_LAGGED")!=nullptr);
}

TEST_F(ContinuousControl, DelayedNoisyHalfMeterTurn) {drive(semicircle(),1400,true);}
TEST_F(ContinuousControl, MovingHandoverRejectedUntilMeasuredStop) {
  nav_msgs::msg::Path route;route.header.frame_id="odom";route.poses=semicircle();controller->setPlan(route);
  geometry_msgs::msg::Twist moving;moving.linear.x=.1017;moving.angular.z=.2347;
  try {controller->computeVelocityCommands(pose(),moving,nullptr);FAIL()<<"moving handover accepted";}
  catch(const nav2_core::PlannerException &e){EXPECT_NE(std::string(e.what()).find("handover requires measured stop"),std::string::npos);}
  EXPECT_GT(controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr).twist.linear.x,0);
  controller->setPlan(route);
  EXPECT_THROW(controller->computeVelocityCommands(pose(),moving,nullptr),nav2_core::PlannerException);
}
TEST_F(ContinuousControl, RelaxedStartToleranceCanEnterContinuousTracking) {
  parent->set_parameter(rclcpp::Parameter("use_sim_time",true));
  auto *clock=parent->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
  for(auto initial:std::vector<std::array<double,3>>{
      {-.055,0,3.3*M_PI/180},{-.0769,-.00557,.06654},{0,.079,0},{0,-.079,0}}) {
    ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL),RCL_RET_OK);
    nav_msgs::msg::Path route;route.header.frame_id="odom";
    for(int i=0;i<=200;++i)route.poses.push_back(pose(i*.01));controller->setPlan(route);
    double x=initial[0],y=initial[1],a=initial[2],v=0,w=0;
    std::deque<geometry_msgs::msg::Twist> delayed(2);ContinuousGoalChecker checker;
    for(int i=0;i<150;++i) {
      ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL+int64_t(i)*100000000LL),RCL_RET_OK);
      geometry_msgs::msg::Twist measured;measured.linear.x=v;measured.angular.z=w;
      geometry_msgs::msg::Twist cmd;
      ASSERT_NO_THROW(cmd=controller->computeVelocityCommands(pose(x,y,a),measured,&checker).twist);
      delayed.push_back(cmd);const auto applied=delayed.front();delayed.pop_front();
      v+=(applied.linear.x-v)*(1-std::exp(-.1/.2));w+=(applied.angular.z-w)*(1-std::exp(-.1/.35));
      x+=v*cos(a)*.1;y+=v*sin(a)*.1;a+=w*.1;
    }
    EXPECT_GT(x,.5);EXPECT_LT(std::abs(y),.08);
  }
}
#include "ego_nav2_controller/continuous_tracking.hpp"
TEST(ContinuousTracking, CentimeterNoiseDoesNotProduceLargeSteering) {
  // 2 cm lateral error previously requested ~0.22 rad/s at 0.08 m/s with 12 cm pursuit.
  EXPECT_LT(std::abs(ego_nav2_controller::continuousYawRate(.08,0,.02,0,0)),.02);
  EXPECT_NEAR(ego_nav2_controller::continuousYawRate(.08,2,0,0,.16),.16,1e-9);
  EXPECT_LT(ego_nav2_controller::continuousYawRate(.08,0,0,0,.15),0);
}

#include "ego_nav2_controller/exact_goal.hpp"
#include "ego_nav2_controller/footprint_safety.hpp"
class ExactStartChecker : public ContinuousGoalChecker {
public:
  bool getTolerances(geometry_msgs::msg::Pose &p,geometry_msgs::msg::Twist &) override {
    p.position.x=p.position.y=.03;p.orientation.z=sin(.04/2);p.orientation.w=cos(.04/2);return true;
  }
};
class PreciseControl : public Control {
protected:
  bool preciseApproach() const override {return true;}
  double maxLinearSpeed() const override {return .30;}
  double maxAngularSpeed() const override {return .3;}
};
TEST_F(PreciseControl, RelaxedStoppedGoalAcceptsRecordedPoseAndKeepsMotionGates) {
  pluginlib::ClassLoader<nav2_core::GoalChecker> checker_loader("nav2_core","nav2_core::GoalChecker");
  auto checker=checker_loader.createSharedInstance("nav2_controller::StoppedGoalChecker");
  parent->declare_parameter("Start8cm.xy_goal_tolerance",.08);
  parent->declare_parameter("Start8cm.yaw_goal_tolerance",5.*M_PI/180);
  parent->declare_parameter("Start8cm.stateful",false);
  parent->declare_parameter("Start8cm.trans_stopped_velocity",.04);
  parent->declare_parameter("Start8cm.rot_stopped_velocity",.03);
  checker->initialize(parent,"Start8cm",map);
  geometry_msgs::msg::Twist stopped;stopped.linear.x=.011;stopped.angular.z=.006;
  EXPECT_TRUE(checker->isGoalReached(pose(-.055,0,3.3*M_PI/180).pose,pose().pose,stopped));
  EXPECT_TRUE(checker->isGoalReached(pose(-.079,0,4.9*M_PI/180).pose,pose().pose,stopped));
  EXPECT_FALSE(checker->isGoalReached(pose(-.081).pose,pose().pose,stopped));
  EXPECT_FALSE(checker->isGoalReached(pose(0,0,5.1*M_PI/180).pose,pose().pose,stopped));
  geometry_msgs::msg::Twist moving;moving.linear.x=.041;
  EXPECT_FALSE(checker->isGoalReached(pose().pose,pose().pose,moving));
  moving.linear.x=0;moving.angular.z=.031;
  EXPECT_FALSE(checker->isGoalReached(pose().pose,pose().pose,moving));
  // At 5.5 cm with heading still wrong, retain terminal alignment instead of
  // falling back to positional pursuit because xy tolerance is now 8 cm.
  parent->set_parameter(rclcpp::Parameter("use_sim_time",true));
  auto *clock=parent->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL),RCL_RET_OK);
  nav_msgs::msg::Path path;path.header.frame_id="odom";path.poses={pose(-.055),pose()};controller->setPlan(path);
  EXPECT_EQ(controller->computeVelocityCommands(pose(-.055,0,.3),stopped,checker.get()).twist.linear.x,0);
  ASSERT_EQ(rcl_set_ros_time_override(clock,100500000000LL),RCL_RET_OK);
  const auto cmd=controller->computeVelocityCommands(pose(-.055,0,.3),stopped,checker.get()).twist;
  EXPECT_EQ(cmd.linear.x,0);EXPECT_LT(cmd.angular.z,0);
}
TEST_F(PreciseControl, BrakesBeforeFineAlignmentAtApproachSpeed) {
  parent->set_parameter(rclcpp::Parameter("use_sim_time",true));
  auto *clock=parent->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL),RCL_RET_OK);
  nav_msgs::msg::Path path;path.header.frame_id="odom";path.poses={pose(-.4),pose()};controller->setPlan(path);
  ExactStartChecker checker;double x=-.4,v=.30;bool stopped=false;
  geometry_msgs::msg::Twist initial;initial.linear.x=.30;
  std::deque<geometry_msgs::msg::Twist> delayed(2,initial);
  for(int i=0;i<30;++i) {
    ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL+int64_t(i)*100000000LL),RCL_RET_OK);
    geometry_msgs::msg::Twist measured;measured.linear.x=v;
    const auto command=controller->computeVelocityCommands(pose(x),measured,&checker).twist;
    if(i==0)EXPECT_NEAR(command.linear.x,.28,1e-6);
    delayed.push_back(command);auto applied=delayed.front();delayed.pop_front();
    v+=(applied.linear.x-v)*(1-std::exp(-.1/.2));x+=v*.1;
    EXPECT_LT(x,-.03);  // Stop before the target instead of crossing it.
    if(v<.03){stopped=true;break;}
  }
  EXPECT_TRUE(stopped)<<"remaining="<<-x<<" speed="<<v;
}
TEST_F(PreciseControl, NearbyGoalsConvergeWithDelayAndNoise) {
  parent->set_parameter(rclcpp::Parameter("use_sim_time",true));
  auto *clock=parent->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
  for(double noise:{.004,.025})for(auto initial:std::vector<std::array<double,3>>{
      {.08,.03,0},{.05,-.05,0},{.02,.02,1.5},{-.30,.15,0},{-.045,-.083,-.042}}) {
    ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL),RCL_RET_OK);
    nav_msgs::msg::Path path;path.header.frame_id="odom";path.poses={pose(initial[0],initial[1],initial[2]),pose()};controller->setPlan(path);
    double x=initial[0],y=initial[1],a=initial[2],v=0,w=0,quiet=0;
    std::deque<geometry_msgs::msg::Twist> delayed(2);ExactStartChecker checker;bool done=false;
    for(int i=0;i<900;++i) {
      ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL+int64_t(i)*100000000LL),RCL_RET_OK);
      geometry_msgs::msg::Twist measured;measured.linear.x=v;measured.angular.z=w;
      geometry_msgs::msg::Twist cmd;
      try {cmd=controller->computeVelocityCommands(pose(x+noise*sin(i*.7),y+noise*sin(i*1.1),a+.003*sin(i*.8)),measured,&checker).twist;}
      catch(const std::exception &e){FAIL()<<"initial="<<initial[0]<<","<<initial[1]<<" step="<<i<<" pose="<<x<<","<<y<<","<<a<<" "<<e.what();}
      EXPECT_LE(cmd.linear.x,.06001);EXPECT_LE(std::abs(cmd.angular.z),.20001);
      delayed.push_back(cmd);auto applied=delayed.front();delayed.pop_front();
      v+=(applied.linear.x-v)*(1-std::exp(-.1/.2));w+=(applied.angular.z-w)*(1-std::exp(-.1/.35));
      x+=v*cos(a)*.1;y+=v*sin(a)*.1;a+=w*.1;
      quiet=(std::hypot(x,y)<.04 && std::abs(atan2(sin(a),cos(a)))<.06 && std::abs(v)<.04 && std::abs(w)<.03)?quiet+.1:0;
      if(quiet>1.){done=true;break;}
    }
    EXPECT_TRUE(done)<<x<<","<<y<<","<<a;
  }
}
TEST_F(PreciseControl, ExactGoalRejectsUnsafeOrApproximateEndpoints) {
  nav_msgs::msg::Path path;path.header.frame_id="odom";path.poses={pose(.011109,.021320)};
  auto goal=pose();
  auto clear=[&](double x,double y,double a,double margin){return ego_nav2_controller::bodyCollision(map->getCostmap(),map->getRobotFootprint(),.25,0,.6,x,y,a,margin);};
  ego_nav2_controller::appendExactGoal(path,goal,.05,.90,clear);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x,0);EXPECT_DOUBLE_EQ(path.poses.back().pose.position.y,0);
  EXPECT_EQ(path.poses.back().header.stamp.sec,0);
  path.poses={pose(.12,0)};
  EXPECT_THROW(ego_nav2_controller::appendExactGoal(path,goal,.05,.90,clear),nav2_core::PlannerException);
  path.poses={pose(.011109,.021320)};unsigned int mx,my;
  ASSERT_TRUE(map->getCostmap()->worldToMap(.25,0,mx,my));map->getCostmap()->setCost(mx,my,255);
  EXPECT_THROW(ego_nav2_controller::appendExactGoal(path,goal,.05,.90,clear),nav2_core::PlannerException);
  EXPECT_THROW(controller->computeVelocityCommands(pose(.08),geometry_msgs::msg::Twist(),nullptr),nav2_core::PlannerException);
}

#include <nav2_core/global_planner.hpp>
TEST_F(PreciseControl, ActualLatticePluginKeepsRequestedSubcellGoal) {
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> planner_loader("nav2_core","nav2_core::GlobalPlanner");
  auto planner=planner_loader.createSharedInstance("ego_nav2_controller::ExactGoalLattice");
  parent->declare_parameter("ExactTest.lattice_filepath",std::string(EGO_LATTICE_FILE));
  parent->declare_parameter("ExactTest.allow_unknown",false);
  planner->configure(parent,"ExactTest",tf,map);planner->activate();
  auto goal=pose(.011109,.021320,0);
  auto planned=planner->createPlan(pose(-.4,.02,0),goal);
  ASSERT_FALSE(planned.poses.empty());
  EXPECT_DOUBLE_EQ(planned.poses.back().pose.position.x,goal.pose.position.x);
  EXPECT_DOUBLE_EQ(planned.poses.back().pose.position.y,goal.pose.position.y);
  EXPECT_EQ(planned.poses.back().header.stamp.sec,0);
  planner->deactivate();planner->cleanup();
}

class FastContinuousControl : public ContinuousControl {
protected:
  double maxLinearSpeed() const override {return .8;}
};
TEST_F(FastContinuousControl, BrakesBeforeHalfMeterTurnWithDelayAndNoise) {drive(semicircle(),1400,true);}
TEST_F(FastContinuousControl, FullNewRouteWithDelayAndNoise) {
  const char *file=std::getenv("EGO_CONTINUOUS_ROUTE");if(!file)GTEST_SKIP();
  std::ifstream in(file);ASSERT_TRUE(in.good());double x,y,a;std::vector<geometry_msgs::msg::PoseStamped> points;
  while(in>>x>>y>>a)points.push_back(pose(x,y,a));ASSERT_GT(points.size(),30000u);
  drive(points,41000,true);
}
TEST_F(FastContinuousControl, ReachesCruiseAndBrakesAtEndpoint) {
  std::vector<geometry_msgs::msg::PoseStamped> points;
  for(int i=0;i<=1200;++i)points.push_back(pose(i*.01));
  drive(points,1500,true,true);
}
TEST_F(FastContinuousControl, BrakingCorridorRejectsSuddenObstacle) {
  nav_msgs::msg::Path route;route.header.frame_id="odom";
  for(int i=0;i<=400;++i)route.poses.push_back(pose(i*.01));controller->setPlan(route);
  controller->computeVelocityCommands(pose(),geometry_msgs::msg::Twist(),nullptr);
  unsigned int x,y;ASSERT_TRUE(map->getCostmap()->worldToMap(2,0,x,y));map->getCostmap()->setCost(x,y,254);
  geometry_msgs::msg::Twist measured;measured.linear.x=.8;
  EXPECT_THROW(controller->computeVelocityCommands(pose(),measured,nullptr),nav2_core::PlannerException);
}

class FastPreciseControl : public HighSpeedCoverageControl {
protected:
  bool preciseApproach() const override {return true;}
};
TEST_F(FastPreciseControl, CruiseBrakesBeforePointFourMeterAlignmentZone) {
  parent->set_parameter(rclcpp::Parameter("use_sim_time",true));
  auto *clock=parent->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL),RCL_RET_OK);
  nav_msgs::msg::Path route;route.header.frame_id="odom";
  for(int i=0;i<=80;++i)route.poses.push_back(pose(-8.+i*.1));controller->setPlan(route);
  ExactStartChecker checker;double x=-8,v=0,peak=0;bool done=false;
  std::deque<geometry_msgs::msg::Twist> delayed(2);
  for(int i=0;i<1600;++i) {
    ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL+int64_t(i)*100000000LL),RCL_RET_OK);
    map->getCostmap()->updateOrigin(x-6,-6);
    geometry_msgs::msg::Twist measured;measured.linear.x=v;
    auto cmd=controller->computeVelocityCommands(pose(x),measured,&checker).twist;
    peak=std::max(peak,cmd.linear.x);
    delayed.push_back(cmd);auto applied=delayed.front();delayed.pop_front();
    v+=(applied.linear.x-v)*(1-std::exp(-.1/.2));x+=v*.1;
    if(x>-.4)EXPECT_LT(v,.12);
    EXPECT_LE(x,.04);
    if(std::abs(x)<.04 && v<.01 && cmd.linear.x==0){done=true;break;}
  }
  EXPECT_GT(peak,.79);EXPECT_TRUE(done)<<x<<","<<v;
}

TEST_F(FastContinuousControl, RelaxedStartConvergesBeforeCruise) {
  parent->set_parameter(rclcpp::Parameter("use_sim_time",true));
  auto *clock=parent->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
  for(auto initial:std::vector<std::array<double,3>>{
      {-.055,0,3.3*M_PI/180},{-.0769,-.00557,.06654},{0,.079,0},{0,-.079,0}}) {
    ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL),RCL_RET_OK);
    nav_msgs::msg::Path route;route.header.frame_id="odom";
    for(int i=0;i<=200;++i)route.poses.push_back(pose(i*.01));controller->setPlan(route);
    double x=initial[0],y=initial[1],a=initial[2],v=0,w=0;
    std::deque<geometry_msgs::msg::Twist> delayed(2);ContinuousGoalChecker checker;
    for(int i=0;i<150;++i) {
      ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL+int64_t(i)*100000000LL),RCL_RET_OK);
      geometry_msgs::msg::Twist measured;measured.linear.x=v;measured.angular.z=w;
      geometry_msgs::msg::Twist cmd;
      ASSERT_NO_THROW(cmd=controller->computeVelocityCommands(pose(x,y,a),measured,&checker).twist);
      delayed.push_back(cmd);const auto applied=delayed.front();delayed.pop_front();
      v+=(applied.linear.x-v)*(1-std::exp(-.1/.2));w+=(applied.angular.z-w)*(1-std::exp(-.1/.35));
      x+=v*cos(a)*.1;y+=v*sin(a)*.1;a+=w*.1;
    }
    EXPECT_GT(x,.5);EXPECT_LT(std::abs(y),.08);
  }
}

TEST(ContinuousTracking, ProgressiveSpeedKeepsCruiseForCentimeterNoise) {
  using ego_nav2_controller::continuousTrackingSpeedCap;
  for(double e:{-.039,-.026,0.,.026,.039})
    EXPECT_DOUBLE_EQ(continuousTrackingSpeedCap(.8,e,0,.2,.6),.8);
  EXPECT_DOUBLE_EQ(continuousTrackingSpeedCap(.8,.035,.03,.2,.6),.8);
  double previous=.8;
  for(double e=.04;e<.08;e+=.001) {
    double cap=continuousTrackingSpeedCap(.8,e,0,.2,.6);
    EXPECT_LE(cap,previous+1e-9);EXPECT_GE(cap,.08-1e-9);
    EXPECT_LT(previous-cap,.04);previous=cap;
  }
  EXPECT_NEAR(continuousTrackingSpeedCap(.8,.075,0,.2,.6),.08,1e-9);
  EXPECT_LT(continuousTrackingSpeedCap(.8,.05,-.04,.2,.6),
            continuousTrackingSpeedCap(.8,.05,.04,.2,.6));
}

TEST_F(FastContinuousControl, ThreeCentimeterLocalizationBiasStillAllowsCruise) {
  std::vector<geometry_msgs::msg::PoseStamped> points;
  for(int i=0;i<=1200;++i)points.push_back(pose(i*.01));
  drive(points,1500,true,true,.03);
}

TEST_F(FastContinuousControl, WeakSteeringFollowsFullRoute) {
  const char *file=std::getenv("EGO_CONTINUOUS_ROUTE");if(!file)GTEST_SKIP();
  std::ifstream in(file);ASSERT_TRUE(in.good());double x,y,a;std::vector<geometry_msgs::msg::PoseStamped> points;
  while(in>>x>>y>>a)points.push_back(pose(x,y,a));ASSERT_GT(points.size(),30000u);
  drive(points,41000,true,true,0.,.65);
}
TEST(ContinuousTracking, TurnResponseFiltersLagAndReducesSpeed) {
  ego_nav2_controller::ContinuousTurnResponse response;
  EXPECT_DOUBLE_EQ(response.step(0,2,0,0,0,.2),.06);
  // A single bad observation does not assert persistent response loss.
  EXPECT_DOUBLE_EQ(response.step(.1,2,0,0,0,.2),.06);
  double cap=.06;
  for(int i=2;i<40;++i)cap=response.step(i*.1,2,0,.045,.13,.2);
  EXPECT_LT(cap,.041);EXPECT_GE(cap,.03);
  EXPECT_LE(response.step(4.,2,.075,0,.13,.2),.03001);
  EXPECT_LE(response.step(4.1,2,0,0,.2,.2),.03101);
  response.reset();EXPECT_DOUBLE_EQ(response.step(10,2,0,0,.2,.2),.06);
  EXPECT_GT(response.step(10.1,0,0,0,0,0),.8);
}

TEST(ContinuousTracking, BendCeilingReservesSteeringAndAcceleration) {
  EXPECT_DOUBLE_EQ(ego_nav2_controller::continuousCurveLimit(.5,0),.2);
  EXPECT_NEAR(ego_nav2_controller::continuousCurveLimit(2,0),.095,1e-9);
  EXPECT_LT(ego_nav2_controller::continuousCurveLimit(2,28),.09);
  EXPECT_DOUBLE_EQ(ego_nav2_controller::continuousCurveLimit(-.5,0),.2);
}

TEST_F(FastContinuousControl, WideBendCanReachPointTwo) {
  std::vector<geometry_msgs::msg::PoseStamped> points;
  for(int i=0;i<=500;++i)points.push_back(pose(i*.01));
  for(int i=1;i<=800;++i) {
    double a=M_PI*i/800;points.push_back(pose(5+2*sin(a),2-2*cos(a),a));
  }
  for(int i=1;i<=300;++i)points.push_back(pose(5-i*.01,4,M_PI));
  drive(points,2200,true,true,0.,1.,true);
}

TEST(ContinuousTracking, YawServoCompensatesWeakSteeringWithoutUnboundedWindup) {
  ego_nav2_controller::ContinuousYawServo servo;
  double measured=0,cmd=0;
  for(int i=0;i<250;++i) {
    cmd=servo.step(i*.1,.09,2,0,0,measured);
    measured+=(.65*cmd-measured)*(1-std::exp(-.1/.35));
    EXPECT_LE(std::abs(cmd),.30);
  }
  EXPECT_NEAR(measured,.18,.012);
  servo.reset();EXPECT_LT(servo.step(30,.09,-2,0,0,0),0);
  for(int i=0;i<100;++i)cmd=servo.step(31+i*.1,.09,2,-.07,.08,0);
  EXPECT_LT(std::abs(cmd),.4);
  servo.reset();EXPECT_NEAR(servo.step(50,.8,0,0,0,0),0,1e-9);
}

TEST_F(FastContinuousControl, WeakSteeringAndMapCorrectionsFollowFullRoute) {
  const char *file=std::getenv("EGO_CONTINUOUS_ROUTE");if(!file)GTEST_SKIP();
  std::ifstream in(file);ASSERT_TRUE(in.good());double x,y,a;std::vector<geometry_msgs::msg::PoseStamped> points;
  while(in>>x>>y>>a)points.push_back(pose(x,y,a));ASSERT_GT(points.size(),30000u);
  drive(points,41000,true,true,0.,.65,false,.025);
}
