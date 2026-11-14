#include <gtest/gtest.h>
#include "ego_nav2_controller/speed_envelope.hpp"
using namespace ego_nav2_controller;
struct S {double x{},y{};};
TEST(SpeedEnvelope, BrakeBeforeZoneIncludingReactionAndMargin) {
  const double d=brakingDistance(.8,.25,.2,.6,.15);
  EXPECT_NEAR(d,2.07375,1e-8);
  EXPECT_NEAR(approachSpeed(.25,d,.2,.6,.15),.8,1e-8);
  EXPECT_DOUBLE_EQ(approachSpeed(.25,0,.2,.6,.15),.25);
  EXPECT_LT(approachSpeed(.25,1,.2,.6,.15),.8);
}
TEST(SpeedEnvelope, PursuitOffsetStartsBrakingBeforeSteeringPoint) {
  std::vector<S> path;for(int i=0;i<=90;++i)path.push_back({i*.05,0});
  auto zone=[](S q){return q.x>=2.;};
  auto e=previewEnvelope(path,0,3.,.7,.8,.25,.3,.12,.15,.2,.6,.15,zone);
  EXPECT_LT(e.cap,.65);EXPECT_FALSE(e.slow_now);
  e=previewEnvelope(path,30,3.,.7,.8,.25,.3,.12,.15,.2,.6,.15,zone);
  EXPECT_LE(e.cap,.25);EXPECT_TRUE(e.slow_now);
  e=previewEnvelope(path,0,3.,.7,.8,.25,.3,.12,.15,.2,.6,.15,[](S){return false;});
  EXPECT_DOUBLE_EQ(e.cap,.8);
}
TEST(SpeedEnvelope, CurvatureRestrictsEvenWithoutObstacleFlag) {
  std::vector<S> path;for(int i=0;i<=60;++i){double a=i*.025;path.push_back({2*std::sin(a),2*(1-std::cos(a))});}
  auto e=previewEnvelope(path,0,3.,.7,.8,.25,.3,.12,.15,.2,.6,.15,[](S){return false;});
  EXPECT_LE(e.cap,.25);EXPECT_TRUE(e.slow_now);
}
TEST(SpeedEnvelope, DelayedSpeedResponseStillReachesLimitBeforeZone) {
  double x=0,command=.8,actual=.8;
  for(int i=0;i<500 && x<4.;++i) {
    double desired=approachSpeed(.25,4-x,.2,.6,.15);
    command=std::clamp(desired,command-.2*.05,command+.2*.05);
    actual+=.2*(command-actual);x+=actual*.05;
    if(x>=3.9)EXPECT_LE(actual,.255);
  }
  EXPECT_GE(x,4.);
}
