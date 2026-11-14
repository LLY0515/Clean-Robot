#include <gtest/gtest.h>
#include <plan_env/grid_map.h>
#include <plan_manage/planner_interface.h>
TEST(Grid, RollingClearsOldObstacles) {
  GridMap2D g(.1,Eigen::Vector2i(8,8));g.setCurPose(0,0);
  g.setObstacle(g.worldToGrid(Eigen::Vector2d(1,1)));g.inflateObstacles(.2);
  EXPECT_TRUE(g.getInflateOccupancy(Eigen::Vector2d(1,1)));
  g.setCurPose(.1,0);
  EXPECT_FALSE(g.getInflateOccupancy(Eigen::Vector2d(1,1)));
  EXPECT_TRUE(g.getInflateOccupancy(Eigen::Vector2d(99,0)));
}
TEST(Planner, StraightAndFailureClearsTrajectory) {
  ego_planner::PlannerInterface p;p.initParam(.15,.2,1);
  p.initEsdfMap(8,8,0,.1,Eigen::Vector3d::Zero(),.1);
  ego_planner::PathPoint cur{0,0,0,0};p.setGridMap(cur);p.setCurrentVehiclePos(cur);
  std::vector<ego_planner::ObstacleInfo> obstacles;p.setObstacles(obstacles);
  std::vector<ego_planner::PathPoint> path;
  for(int i=0;i<=10;++i)path.push_back({float(i*.2),0,0,0});
  p.setPathPoint(path);p.makePlan();std::vector<ego_planner::PathPoint> out;p.getLocalPlanTrajResults(out);
  ASSERT_GT(out.size(),2u);
  for(const auto &q:out) {EXPECT_TRUE(std::isfinite(q.x));EXPECT_NEAR(q.y,0,.05);}
  path.clear();p.setPathPoint(path);p.makePlan();p.getLocalPlanTrajResults(out);EXPECT_TRUE(out.empty());
}

TEST(Grid, InflationMatchesEuclideanDisk) {
  GridMap2D g(.1,Eigen::Vector2i(6,6));g.setCurPose(0,0);
  const std::vector<Eigen::Vector2d> points{{0,0},{1.4,1.2},{-1,-1.8}};
  for(const auto &p:points)g.setObstacle(g.worldToGrid(p));
  g.inflateObstacles(.70);
  for(int x=0;x<60;++x)for(int y=0;y<60;++y) {
    auto p=g.gridToWorld(Eigen::Vector2i(x,y));bool expected=false;
    for(const auto &q:points)if((p-q).norm()<=.70000001)expected=true;
    EXPECT_EQ(g.getInflateOccupancy(p),expected)<<x<<","<<y;
  }
}

TEST(Planner, DetoursAroundObstacle) {
  ego_planner::PlannerInterface p;p.initParam(.15,.2,1);
  p.initEsdfMap(10,10,0,.05,Eigen::Vector3d::Zero(),1.0);
  ego_planner::PathPoint cur{0,0,0,0};p.setGridMap(cur);p.setCurrentVehiclePos(cur);
  std::vector<ego_planner::ObstacleInfo> obstacles{{2,0,0}};p.setObstacles(obstacles);
  std::vector<ego_planner::PathPoint> path;
  for(int i=0;i<=20;++i)path.push_back({float(i*.2),0,0,0});
  p.setPathPoint(path);p.makePlan();std::vector<ego_planner::PathPoint> out;p.getLocalPlanTrajResults(out);
  ASSERT_GT(out.size(),2u);
  for(const auto &q:out) EXPECT_GE(std::hypot(q.x-2,q.y),.95);
}
#include <path_searching/dyn_a_star.h>
TEST(Search, OccupiedCoincidentEndpointsReturnWithoutOscillating) {
  auto g=std::make_shared<GridMap2D>(.1,Eigen::Vector2i(6,6));g->setCurPose(0,0);
  g->setObstacle(g->worldToGrid(Eigen::Vector2d(0,0)));g->inflateObstacles(.6);
  AStar search;search.initGridMap(g,Eigen::Vector2i(100,100));
  EXPECT_FALSE(search.AstarSearch(.1,Eigen::Vector2d(0,0),Eigen::Vector2d(.01,0)));
}

TEST(ReturnCost, AnalyticGradientAndNoAttractionAcrossBlockedRoute) {
  auto grid=std::make_shared<GridMap2D>(.05,Eigen::Vector2i(10,10));grid->setCurPose(0,0);
  ego_planner::BsplineOptimizer opt;opt.setParam();opt.setEnvironment(grid);
  opt.setReturnPolicy(.1,.4,.05);
  opt.setRouteReference({Eigen::Vector2d(-3,0),Eigen::Vector2d(3,0)});
  Eigen::MatrixXd q(2,10);for(int i=0;i<10;++i){q(0,i)=-2+i*.4;q(1,i)=.35+.01*i;}
  Eigen::MatrixXd gradient=Eigen::MatrixXd::Zero(2,10);double cost;
  opt.prepareRouteTargets(q);
  opt.calcRouteCost(q,cost,gradient);
  for(int i=0;i<q.cols();++i)for(int axis=0;axis<2;++axis) {
    auto plus=q,minus=q;plus(axis,i)+=1e-6;minus(axis,i)-=1e-6;
    Eigen::MatrixXd tmp=gradient;double a,b;
    opt.calcRouteCost(plus,a,tmp);opt.calcRouteCost(minus,b,tmp);
    EXPECT_NEAR(gradient(axis,i),(a-b)/2e-6,1e-6);
  }
  EXPECT_GT(gradient.row(1).sum(),0); // descent returns towards y=0.
  grid->setObstacle(grid->worldToGrid(Eigen::Vector2d(0,0)));grid->inflateObstacles(.6);
  opt.setReturnPolicy(.1,.4,0);
  opt.setRouteReference({Eigen::Vector2d(-.5,0),Eigen::Vector2d(.5,0)});
  opt.prepareRouteTargets(q);
  opt.calcRouteCost(q,cost,gradient);
  EXPECT_DOUBLE_EQ(cost,0);EXPECT_DOUBLE_EQ(gradient.norm(),0);
  EXPECT_THROW(opt.setReturnPolicy(0,.4,.05),std::invalid_argument);
}

TEST(Planner, StoppedSplinePreservesInitialHeading) {
  ego_planner::PlannerInterface p;p.initParam(.6,.2,1);
  p.initEsdfMap(10,10,0,.05,Eigen::Vector3d::Zero(),.9);
  ego_planner::PathPoint cur{0,0,float(-M_PI/2),0};
  p.setGridMap(cur);p.setCurrentVehiclePos(cur);
  std::vector<ego_planner::ObstacleInfo> obstacles;p.setObstacles(obstacles);
  std::vector<ego_planner::PathPoint> path;
  for(int i=0;i<=20;++i)path.push_back({0,float(-i*.2),0,0});
  p.setPathPoint(path);p.makePlan();std::vector<ego_planner::PathPoint> out;p.getLocalPlanTrajResults(out);
  ASSERT_GT(out.size(),2u);EXPECT_NEAR(out.front().z,cur.z,1e-5);
}
