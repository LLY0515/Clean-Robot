#include <gtest/gtest.h>
#include <ego_nav2_controller/precise_approach.hpp>
#include <array>
#include <vector>

using ego_nav2_controller::PreciseApproach;

TEST(PreciseApproach, FinalYawDeadbandIsInsideGoalTolerance) {
  PreciseApproach control;
  control.step(0,0,0,.043,0,0,0,0,0,.03,.04);
  const auto command=control.step(.5,0,0,.043,0,0,0,0,0,.03,.04);
  EXPECT_LT(command.w,0);  // 0.043 rad is outside the 0.04 rad goal tolerance.
  EXPECT_EQ(command.v,0);
}

TEST(PreciseApproach, CapturedPositionFinishesFinalYawDespiteSettlingDrift) {
  PreciseApproach control;
  const double yaw=1.0;
  auto at=[&](double time,double distance,double v=0.) {
    return control.step(time,-distance*std::cos(yaw),-distance*std::sin(yaw),yaw,
                        0,0,0,v,0,.03,.04);
  };
  at(0,.10);at(.5,.10);
  ASSERT_GT(at(1.,.10).v,0);
  double captured_at=0;
  for(int i=1;i<=36;++i) {
    const double time=1.+i*.1,remaining=.1-i*.0025;
    if(at(time,remaining,.025).v==0) {
      EXPECT_LT(remaining,.03);captured_at=time;break;
    }
  }
  ASSERT_GT(captured_at,0);
  // The recorded task entered 1.94 cm, then centimetre-scale drift sent the
  // old state machine back toward the positional bearing instead of goal yaw.
  control.step(captured_at+.1,.04,-.01,yaw,0,0,0,.01,0,.03,.04);
  const auto command=control.step(captured_at+.8,.04,-.01,yaw,0,0,0,.01,0,.03,.04);
  EXPECT_LT(command.w,0);  // Final yaw is zero, never turn away toward +pi.
  EXPECT_EQ(command.v,0);
  control.step(captured_at+2.,.08,0,0,0,0,0,0,0,.03,.04);
  control.step(captured_at+2.5,.08,0,0,0,0,0,0,0,.03,.04);
  EXPECT_STRNE(control.phaseName(),"done");  // Latching yaw never relaxes XY acceptance.
}

TEST(PreciseApproach, ConvergesWithCentimetreNoiseAndChassisDelay) {
  for(double noise:{.004,.025})for(auto initial:std::vector<std::array<double,3>>{
      {.08,.03,0},{.05,-.05,0},{.02,.02,1.5},{-.30,.15,0},{-.045,-.083,-.042}}) {
    PreciseApproach control;
    double x=initial[0],y=initial[1],a=initial[2],v=0,w=0,last_v=0,last_w=0,quiet=0;
    std::deque<PreciseApproach::Command> delay(2);
    bool reached=false;
    for(int i=0;i<900;++i) {
      PreciseApproach::Command cmd;
      try {cmd=control.step(i*.1,x+noise*std::sin(i*.7),y+noise*std::sin(i*1.1),
                            a+.003*std::sin(i*.8),0,0,0,v,w,.03,.04);}
      catch(const std::exception &e){FAIL()<<"noise="<<noise<<" initial="<<initial[0]<<","<<initial[1]
        <<" step="<<i<<" actual="<<x<<","<<y<<","<<a<<" "<<e.what();}
      cmd.v=std::clamp(cmd.v,std::max(0.,last_v-.2*.1),last_v+.05*.1);
      cmd.w=std::clamp(cmd.w,last_w-.4*.1,last_w+.4*.1);
      last_v=cmd.v;last_w=cmd.w;
      delay.push_back(cmd);const auto applied=delay.front();delay.pop_front();
      v+=(applied.v-v)*(1-std::exp(-.1/.2));w+=(applied.w-w)*(1-std::exp(-.1/.35));
      x+=v*std::cos(a)*.1;y+=v*std::sin(a)*.1;a+=w*.1;
      quiet=(std::hypot(x,y)<.04 && std::abs(std::atan2(std::sin(a),std::cos(a)))<.04 &&
             std::abs(v)<.04 && std::abs(w)<.03)?quiet+.1:0;
      if(quiet>1.){reached=true;break;}
    }
    EXPECT_TRUE(reached)<<"noise="<<noise<<" initial="<<initial[0]<<","<<initial[1]
      <<" actual="<<x<<","<<y<<","<<a;
  }
}

TEST(PreciseApproach, PersistentFailureNeverProducesAnotherCommand) {
  PreciseApproach control;
  control.step(0,-.2,0,0,0,0,0,0,0,.03,.04);
  EXPECT_THROW(control.step(91,-.2,0,0,0,0,0,0,0,.03,.04),std::runtime_error);
  // Even if the clock or position later returns inside its bound, the failed
  // maneuver cannot alternate successful zero commands and rejected cycles.
  EXPECT_THROW(control.step(1,-.2,0,0,0,0,0,0,0,.03,.04),std::runtime_error);
}
