#include <gtest/gtest.h>
#include <deque>
#include "ego_nav2_controller/turn_profile.hpp"
using ego_nav2_controller::TurnProfile;
TEST(TurnProfile, DoesNotCompleteWhileMovingOrOnRepeatedOdometry) {
  TurnProfile profile;profile.reset(.15);
  for(int i=0;i<10;++i)EXPECT_FALSE(profile.update(0,.15,.1,10+i*.1).settled);
  for(int i=0;i<20;++i)EXPECT_FALSE(profile.update(0,0,.1,12).settled);
  EXPECT_TRUE(profile.update(0,0,.1,12.3).settled);
}
TEST(TurnProfile, CorrectsOvershootAndDoesNotLatchOutsideTolerance) {
  TurnProfile profile;
  profile.update(.01,0,.1,1.);
  auto correction=profile.update(-.06,0,.1,1.1);
  EXPECT_LT(correction.speed,0);EXPECT_FALSE(correction.settled);
}
TEST(TurnProfile, MissingMeasurementsRestartSettlingWindow) {
  TurnProfile profile;
  EXPECT_FALSE(profile.update(0,0,.1,10.).settled);
  EXPECT_FALSE(profile.update(0,0,.1,10.4).settled);
  EXPECT_TRUE(profile.update(0,0,.1,10.7).settled);
}
TEST(TurnProfile, DelayedChassisCompletesBothDirectionsAndHalfTurns) {
  for(double target:{1.57079632679,-1.57079632679,3.21,-3.21}) {
    SCOPED_TRACE(target);
    TurnProfile profile;
    std::deque<double> delay(2,0.);
    double angle=0,speed=0,last=0,max_overshoot=0;bool settled=false;
    int reversals=0;double direction=std::copysign(1.,target);
    for(int i=0;i<1500;++i) {
      auto out=profile.update(target-angle,speed,.1,10+i*.1);
      EXPECT_LE(std::abs(out.speed-last),.040001);EXPECT_LE(std::abs(out.speed),.300001);
      if(out.speed*direction<-.01){++reversals;direction=-direction;}
      last=out.speed;
      delay.push_back(out.speed);const double desired=delay.front();delay.pop_front();
      speed+=std::clamp(desired-speed,-.035,.035);
      angle+=speed*.1;
      max_overshoot=std::max(max_overshoot,(angle-target)*std::copysign(1.,target));
      if(out.settled){settled=true;break;}
    }
    EXPECT_TRUE(settled);EXPECT_LT(std::abs(target-angle),.035);
    EXPECT_LE(std::abs(speed),.03);EXPECT_LT(max_overshoot,.035);EXPECT_LE(reversals,1);
  }
}
TEST(TurnProfile, ChangedMapHeadingIsCorrectedWithoutCommandJump) {
  TurnProfile profile;double yaw=0,w=0,last=0;bool settled=false;
  for(int i=0;i<500;++i) {
    const double map_correction=i>=30?.05:0.;
    auto output=profile.update(1.57-yaw-map_correction,w,.1,10+i*.1);
    EXPECT_LE(std::abs(output.speed-last),.040001);last=output.speed;
    w+=.5*(output.speed-w);yaw+=w*.1;
    if(output.settled){settled=true;break;}
  }
  EXPECT_TRUE(settled);EXPECT_NEAR(yaw+.05,1.57,.035);
}
TEST(TurnProfile, BreakawayDeadbandDoesNotPreventCompletion) {
  TurnProfile profile;double yaw=0,w=0;bool settled=false;
  for(int i=0;i<800;++i) {
    auto output=profile.update(1.57-yaw,w,.1,10+i*.1);
    const double desired=std::abs(output.speed)<.095?0:output.speed;
    w+=std::clamp(desired-w,-.04,.04);yaw+=w*.1;
    if(output.settled){settled=true;break;}
  }
  EXPECT_TRUE(settled);EXPECT_NEAR(yaw,1.57,.035);
}
