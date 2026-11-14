#include <gtest/gtest.h>
#include <nav2_core/controller.hpp>
#include <nav2_core/global_planner.hpp>
#include <pluginlib/class_loader.hpp>
TEST(Plugin, LoadsAsNav2Controller) {
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
  auto plugin=loader.createSharedInstance("ego_nav2_controller::EgoController");
  ASSERT_NE(plugin,nullptr);
}
TEST(Plugin, LoadsExactGoalLattice) {
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader("nav2_core", "nav2_core::GlobalPlanner");
  EXPECT_NE(loader.createSharedInstance("ego_nav2_controller::ExactGoalLattice"),nullptr);
}

#include <nav2_core/behavior.hpp>
TEST(Plugin, LoadsClearanceBehaviors) {
  pluginlib::ClassLoader<nav2_core::Behavior> loader("nav2_core", "nav2_core::Behavior");
  for(const auto *name:{"ego_nav2_controller::ClearanceSpin", "ego_nav2_controller::ClearanceBackUp", "ego_nav2_controller::ClearanceDrive"})
    EXPECT_NE(loader.createSharedInstance(name),nullptr);
}
