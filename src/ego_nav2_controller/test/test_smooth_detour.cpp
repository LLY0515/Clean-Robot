#include <gtest/gtest.h>
#include "ego_nav2_controller/smooth_detour.hpp"
using namespace ego_nav2_controller;
struct P {double x{},y{},z{},v{};};
static std::vector<P> line() {return {{0,0},{1,0},{2,0},{3,0},{4.5,0}};}
TEST(SmoothDetour, EarlyDepartureAndTangentReturnWithClearance) {
  auto blocked=[](double x,double y){return std::hypot(x-2,y)<.9;};
  auto p=smoothDetour(line(),blocked,1.5,1.8,.025);
  ASSERT_GT(p.size(),100u);
  double first_departure=10,last_deviation=0;
  for(auto q:p) {
    EXPECT_FALSE(blocked(q.x,q.y));
    if(std::abs(q.y)>.05){first_departure=std::min(first_departure,q.x);last_deviation=q.x;}
  }
  EXPECT_LT(first_departure,1.0);EXPECT_GT(last_deviation,3.2);
  EXPECT_NEAR(p.front().y,0,1e-9);EXPECT_NEAR(p.back().y,0,1e-9);
  EXPECT_LT(std::abs((p[1].y-p[0].y)/.025),.002);
  EXPECT_LT(std::abs((p.back().y-p[p.size()-2].y)/.025),.002);
}
TEST(SmoothDetour, UsesOpenSideAndRejectsBlockedCorridor) {
  auto p=smoothDetour(line(),[](double x,double y){return std::hypot(x-2,y)<.9 || y>.25;},1.5,1.8,.025);
  ASSERT_FALSE(p.empty());EXPECT_LT(p[p.size()/2].y,-.8);
  EXPECT_TRUE(smoothDetour(line(),[](double x,double y){return std::hypot(x-2,y)<.9 || std::abs(y)>.5;},1.5,1.8,.025).empty());
}
TEST(SmoothDetour, NoGuideForFreeLineTurnsLateDetectionOrBlockedEndpoint) {
  EXPECT_TRUE(smoothDetour(line(),[](double,double){return false;},1.5,1.8,.025).empty());
  auto turn=line();turn[2].y=.5;
  EXPECT_TRUE(smoothDetour(turn,[](double x,double y){return std::hypot(x-2,y)<.9;},1.5,1.8,.025).empty());
  EXPECT_TRUE(smoothDetour(line(),[](double x,double y){return std::hypot(x-.7,y)<.6;},1.5,1.8,.025).empty());
  EXPECT_TRUE(smoothDetour(line(),[](double x,double){return x>4;},1.5,1.8,.025).empty());
}
TEST(SmoothDetour, RotationDoesNotChangeGeometry) {
  auto r=line();for(auto &q:r){double x=q.x;q.x=3;q.y=x-1;}
  auto p=smoothDetour(r,[](double x,double y){return std::hypot(x-3,y-1)<.9;},1.5,1.8,.025);
  ASSERT_FALSE(p.empty());EXPECT_NEAR(p.back().x,3,1e-9);EXPECT_NEAR(p.back().y,3.5,1e-9);
}
TEST(CurvePreview, SlowsBeforeTightTurnButKeepsStraightSpeed) {
  std::vector<P> p;
  for(int i=0;i<6;++i)p.push_back({i*.02,0});
  for(int i=1;i<=20;++i){double a=i*.05;p.push_back({.1+.2*std::sin(a),.2*(1-std::cos(a))});}
  const double cap=curvePreviewSpeed(p,0,1.5,.4,.3,.12,.2);
  EXPECT_LT(cap,.30);EXPECT_GT(cap,.06);
  EXPECT_DOUBLE_EQ(curvePreviewSpeed(line(),0,1.5,.4,.3,.12,.2),.4);
}
