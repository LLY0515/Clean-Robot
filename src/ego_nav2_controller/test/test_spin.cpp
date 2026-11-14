#include <gtest/gtest.h>
#include <nav2_behaviors/plugins/spin.hpp>
#include <nav2_costmap_2d/costmap_topic_collision_checker.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pluginlib/class_loader.hpp>
#include <rcl/time.h>
#include <limits>

class Spin : public ::testing::Test {
protected:
  pluginlib::ClassLoader<nav2_core::Behavior> loader{"nav2_core","nav2_core::Behavior"};
  std::shared_ptr<nav2_behaviors::Spin> spin;
  std::shared_ptr<nav2_util::LifecycleNode> node;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> maps;
  std::unique_ptr<nav2_costmap_2d::FootprintSubscriber> footprints;
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> checker;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::Costmap>::SharedPtr map_pub;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub;
  geometry_msgs::msg::Twist command;
  std::pair<double,double> obstacle_position{.775,.025};
  std::vector<std::pair<float,float>> footprint_points{{.80,.375},{.80,-.375},{-.30,-.375},{-.30,.375}};
  double time{100};
  double map_stamp_override{-1};
  bool send_map{true};
  bool send_tf{true};
  virtual bool publishFootprint() const {return true;}
  void pump() {
    for(int i=0;i<3;++i){rclcpp::spin_some(node->get_node_base_interface());rclcpp::sleep_for(std::chrono::milliseconds(1));}
  }
  void SetUp() override {
    if(!rclcpp::ok())rclcpp::init(0,nullptr);
    node=std::make_shared<nav2_util::LifecycleNode>("ego_spin_test");
    node->set_parameter(rclcpp::Parameter("use_sim_time",true));
    pump();
    ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()),RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),100000000000LL),RCL_RET_OK);
    node->declare_parameter("cycle_frequency",10.);node->declare_parameter("global_frame","odom");
    node->declare_parameter("robot_base_frame","drive_base_link");node->declare_parameter("transform_tolerance",.1);
    node->declare_parameter("costmap_topic","spin_test_map");
    node->declare_parameter("footprint_topic","spin_test_footprint");
    node->declare_parameter("max_rotational_vel",.3);node->declare_parameter("min_rotational_vel",.1);
    node->declare_parameter("rotational_acc_lim",.4);
    tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf->setUsingDedicatedThread(true);  // Every required transform is injected before each cycle.
    maps=std::make_unique<nav2_costmap_2d::CostmapSubscriber>(node,"spin_test_map");
    footprints=std::make_unique<nav2_costmap_2d::FootprintSubscriber>(node,"spin_test_footprint",*tf,"drive_base_link");
    checker=std::make_shared<nav2_costmap_2d::CostmapTopicCollisionChecker>(*maps,*footprints);
    spin=std::dynamic_pointer_cast<nav2_behaviors::Spin>(loader.createSharedInstance("ego_nav2_controller::ClearanceSpin"));
    ASSERT_NE(spin,nullptr);spin->configure(node,"spin",tf,checker);spin->activate();
    odom_pub=node->create_publisher<nav_msgs::msg::Odometry>("/odom_nav",10);odom_pub->on_activate();
    map_pub=node->create_publisher<nav2_msgs::msg::Costmap>("spin_test_map",rclcpp::QoS(1).transient_local().reliable());map_pub->on_activate();
    footprint_pub=node->create_publisher<geometry_msgs::msg::PolygonStamped>("spin_test_footprint",10);footprint_pub->on_activate();
    command_sub=node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel",10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg){command=*msg;});
    for(int i=0;i<100 && (odom_pub->get_subscription_count()<1 || map_pub->get_subscription_count()<2 ||
        footprint_pub->get_subscription_count()<2);++i)pump();
    publish(0,0);
  }
  void TearDown() override {
    spin->deactivate();spin->cleanup();spin.reset();
    command_sub.reset();odom_pub.reset();map_pub.reset();footprint_pub.reset();
    checker.reset();footprints.reset();maps.reset();tf.reset();node.reset();
  }
  void publish(double yaw,double angular,double correction=0,double linear=0,bool obstacle=false,bool new_odom=true) {
    ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),int64_t(std::llround(time*1e9))),RCL_RET_OK);
    geometry_msgs::msg::TransformStamped transform;transform.header.frame_id="odom";
    transform.child_frame_id="drive_base_link";transform.header.stamp=node->now();
    transform.transform.rotation.z=std::sin(yaw/2);transform.transform.rotation.w=std::cos(yaw/2);
    if(send_tf)tf->setTransform(transform,"test");
    transform.header.frame_id="map";transform.child_frame_id="odom";
    transform.transform.rotation.z=std::sin(correction/2);transform.transform.rotation.w=std::cos(correction/2);
    tf->setTransform(transform,"test",true);
    if(new_odom) {
      nav_msgs::msg::Odometry odom;odom.header.frame_id="odom";odom.child_frame_id="drive_base_link";
      odom.header.stamp=node->now();odom.twist.twist.angular.z=angular;odom.twist.twist.linear.x=linear;
      odom_pub->publish(odom);
    }
    nav2_msgs::msg::Costmap grid;grid.header.frame_id="odom";grid.header.stamp=node->now();
    if(map_stamp_override>=0)grid.header.stamp=rclcpp::Time(int64_t(std::llround(map_stamp_override*1e9)));
    grid.metadata.resolution=.05;grid.metadata.size_x=grid.metadata.size_y=100;
    grid.metadata.origin.position.x=grid.metadata.origin.position.y=-2.5;grid.metadata.origin.orientation.w=1;
    grid.data.resize(10000,0);
    if(obstacle) {
      const int mx=int((obstacle_position.first-grid.metadata.origin.position.x)/grid.metadata.resolution);
      const int my=int((obstacle_position.second-grid.metadata.origin.position.y)/grid.metadata.resolution);
      grid.data[my*100+mx]=254;
    }
    if(send_map)map_pub->publish(grid);
    geometry_msgs::msg::PolygonStamped footprint;footprint.header.frame_id="drive_base_link";footprint.header.stamp=node->now();
    for(const auto &xy:footprint_points) {
      geometry_msgs::msg::Point32 p;p.x=xy.first;p.y=xy.second;footprint.polygon.points.push_back(p);
    }
    if(publishFootprint())footprint_pub->publish(footprint);
    pump();
  }
  nav2_behaviors::Status start(double angle) {
    auto goal=std::make_shared<nav2_msgs::action::Spin::Goal>();goal->target_yaw=angle;goal->time_allowance.sec=90;
    return spin->onRun(goal);
  }
  nav2_behaviors::Status step() {auto status=spin->onCycleUpdate();pump();return status;}
};

TEST_F(Spin, CrossesPiAndTracksSmallMapCorrectionUntilMeasuredStop) {
  double yaw=3.05,w=0,last=0;publish(yaw,w);
  ASSERT_EQ(start(.6),nav2_behaviors::Status::SUCCEEDED);
  bool settled=false;
  for(int i=1;i<300;++i) {
    time+=.1;publish(yaw,w,i>=10?.04:0);
    auto status=step();ASSERT_NE(status,nav2_behaviors::Status::FAILED);
    EXPECT_LE(std::abs(command.angular.z-last),.040001);last=command.angular.z;
    if(status==nav2_behaviors::Status::SUCCEEDED){settled=true;break;}
    w+=.5*(command.angular.z-w);yaw+=w*.1;
  }
  EXPECT_TRUE(settled);EXPECT_NEAR(yaw+.04,3.65,.035);EXPECT_LE(std::abs(w),.03);
  EXPECT_DOUBLE_EQ(command.angular.z,0);
}
TEST_F(Spin, LargeMapCorrectionStopsInsteadOfChasingIt) {
  ASSERT_EQ(start(1.57),nav2_behaviors::Status::SUCCEEDED);
  time+=.1;publish(0,0,.15);
  EXPECT_EQ(step(),nav2_behaviors::Status::FAILED);EXPECT_DOUBLE_EQ(command.angular.z,0);
}
TEST_F(Spin, StaleOdometryStopsEvenWithFreshTfAndMap) {
  ASSERT_EQ(start(1.57),nav2_behaviors::Status::SUCCEEDED);
  for(int i=0;i<4;++i){time+=.1;publish(0,0,0,0,false,false);
    ASSERT_EQ(step(),nav2_behaviors::Status::RUNNING);}
  time+=.11;publish(0,0,0,0,false,false);
  EXPECT_EQ(step(),nav2_behaviors::Status::FAILED);EXPECT_DOUBLE_EQ(command.angular.z,0);
}
TEST_F(Spin, WaitsForTranslationToStopBeforePivot) {
  publish(0,0,0,.12);ASSERT_EQ(start(1.57),nav2_behaviors::Status::SUCCEEDED);
  time+=.1;publish(0,0,0,.12);
  EXPECT_EQ(step(),nav2_behaviors::Status::RUNNING);EXPECT_DOUBLE_EQ(command.angular.z,0);
  time+=.1;publish(0,0);
  EXPECT_EQ(step(),nav2_behaviors::Status::RUNNING);EXPECT_GT(command.angular.z,0);
}
TEST_F(Spin, RetainsBodyCenterObstacleGuard) {
  ASSERT_EQ(start(1.57),nav2_behaviors::Status::SUCCEEDED);
  time+=.1;publish(0,0,0,0,true);
  EXPECT_EQ(step(),nav2_behaviors::Status::RUNNING);EXPECT_DOUBLE_EQ(command.angular.z,0);
  for(int i=0;i<31;++i) {time+=.1;publish(0,0,0,0,true);auto status=step();
    EXPECT_DOUBLE_EQ(command.angular.z,0);if(i==30)EXPECT_EQ(status,nav2_behaviors::Status::FAILED);}
}
TEST_F(Spin, RejectsObstacleInsideFrontCornerBeyondCenterClearanceDisk) {
  // A larger published footprint must expand the 0.85 m center-clearance disk.
  footprint_points={{.85,.45},{.85,-.45},{-.25,-.45},{-.25,.45}};
  obstacle_position={.825,.425};
  ASSERT_GT(std::hypot(obstacle_position.first,obstacle_position.second),.85+.05*std::sqrt(2.)/2);
  ASSERT_EQ(start(1.57),nav2_behaviors::Status::SUCCEEDED);
  time+=.1;publish(0,0,0,0,true);
  EXPECT_EQ(step(),nav2_behaviors::Status::RUNNING);EXPECT_DOUBLE_EQ(command.angular.z,0);
  for(int i=0;i<31;++i) {time+=.1;publish(0,0,0,0,true);auto status=step();
    EXPECT_DOUBLE_EQ(command.angular.z,0);if(i==30)EXPECT_EQ(status,nav2_behaviors::Status::FAILED);}
}
TEST_F(Spin, RejectsNonfiniteFootprint) {
  ASSERT_EQ(start(1.57),nav2_behaviors::Status::SUCCEEDED);
  footprint_points.front().first=std::numeric_limits<float>::quiet_NaN();
  time+=.1;publish(0,0);
  EXPECT_EQ(step(),nav2_behaviors::Status::FAILED);EXPECT_DOUBLE_EQ(command.angular.z,0);
}
class SpinWithoutFootprint : public Spin {
protected:
  bool publishFootprint() const override {return false;}
};
TEST_F(SpinWithoutFootprint, RejectsMissingFootprint) {
  ASSERT_EQ(start(1.57),nav2_behaviors::Status::SUCCEEDED);
  time+=.1;publish(0,0);
  EXPECT_EQ(step(),nav2_behaviors::Status::FAILED);EXPECT_DOUBLE_EQ(command.angular.z,0);
}
TEST_F(Spin, RepeatedOdometryCannotSatisfySettlingWindow) {
  ASSERT_EQ(start(0),nav2_behaviors::Status::SUCCEEDED);
  for(int i=0;i<4;++i){time+=.1;publish(0,0,0,0,false,false);EXPECT_EQ(step(),nav2_behaviors::Status::RUNNING);}
  for(int i=0;i<4;++i){time+=.1;publish(0,0);auto status=step();
    if(i==3)EXPECT_EQ(status,nav2_behaviors::Status::SUCCEEDED);}
}

TEST_F(Spin, TransientObstacleResumesRemainingAngleAfterTwoNewMaps) {
  ASSERT_EQ(start(1.),nav2_behaviors::Status::SUCCEEDED);
  time+=.1;publish(.3,.2,0,0,true);
  EXPECT_EQ(step(),nav2_behaviors::Status::RUNNING);EXPECT_DOUBLE_EQ(command.angular.z,0);
  time+=.1;publish(.32,.02);map_stamp_override=time;
  EXPECT_EQ(step(),nav2_behaviors::Status::RUNNING);EXPECT_DOUBLE_EQ(command.angular.z,0);
  time+=.1;publish(.32,0);
  EXPECT_EQ(step(),nav2_behaviors::Status::RUNNING);EXPECT_DOUBLE_EQ(command.angular.z,0);
  map_stamp_override=-1;time+=.1;publish(.32,0);
  EXPECT_EQ(step(),nav2_behaviors::Status::RUNNING);EXPECT_GT(command.angular.z,0);
  double yaw=.32,w=0;bool settled=false;
  for(int i=0;i<250;++i) {
    w+=.5*(command.angular.z-w);yaw+=w*.1;time+=.1;publish(yaw,w);
    auto status=step();ASSERT_NE(status,nav2_behaviors::Status::FAILED);
    if(status==nav2_behaviors::Status::SUCCEEDED){settled=true;break;}
  }
  EXPECT_TRUE(settled);EXPECT_NEAR(yaw,1.,.035); // Not .32 + the original 1 rad.
}
TEST_F(Spin, LocalizationCorrectionStillFailsDuringObstacleWait) {
  ASSERT_EQ(start(1.),nav2_behaviors::Status::SUCCEEDED);
  time+=.1;publish(0,0,0,0,true);EXPECT_EQ(step(),nav2_behaviors::Status::RUNNING);
  time+=.1;publish(0,0,.15,0,true);
  EXPECT_EQ(step(),nav2_behaviors::Status::FAILED);EXPECT_DOUBLE_EQ(command.angular.z,0);
}
TEST_F(Spin, StaleMapStillFailsDuringObstacleWait) {
  ASSERT_EQ(start(1.),nav2_behaviors::Status::SUCCEEDED);
  time+=.1;publish(0,0,0,0,true);EXPECT_EQ(step(),nav2_behaviors::Status::RUNNING);
  send_map=false;
  for(int i=0;i<11;++i) {time+=.1;publish(0,0);auto status=step();
    EXPECT_DOUBLE_EQ(command.angular.z,0);if(i==10)EXPECT_EQ(status,nav2_behaviors::Status::FAILED);}
}
TEST_F(Spin, StaleTfStillFailsDuringObstacleWait) {
  ASSERT_EQ(start(1.),nav2_behaviors::Status::SUCCEEDED);
  time+=.1;publish(0,0,0,0,true);EXPECT_EQ(step(),nav2_behaviors::Status::RUNNING);
  send_tf=false;
  for(int i=0;i<6;++i) {time+=.1;publish(0,0,0,0,true);auto status=step();
    EXPECT_DOUBLE_EQ(command.angular.z,0);if(i==5)EXPECT_EQ(status,nav2_behaviors::Status::FAILED);}
}
