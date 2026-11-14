#include <gtest/gtest.h>
#include "ego_nav2_controller/spin_safety.hpp"
using namespace ego_nav2_controller;
TEST(TurnWait, TwoDistinctClearMapsAndMeasuredStop) {
  TurnObstacleWait wait;
  EXPECT_EQ(wait.update(1,false,10,true),TurnObstacleWait::Hold);
  EXPECT_EQ(wait.update(1.1,true,11,false),TurnObstacleWait::Hold);
  EXPECT_EQ(wait.update(1.2,true,11,true),TurnObstacleWait::Hold);
  EXPECT_EQ(wait.update(1.3,true,12,true),TurnObstacleWait::Hold);
  EXPECT_EQ(wait.update(1.4,true,12,true),TurnObstacleWait::Hold);
  EXPECT_EQ(wait.update(1.5,true,13,true),TurnObstacleWait::Proceed);
}
TEST(TurnWait, FlickerDoesNotResetDeadlineOrAccumulateClearFrames) {
  TurnObstacleWait wait;
  EXPECT_EQ(wait.update(1,false,10,true),TurnObstacleWait::Hold);
  EXPECT_EQ(wait.update(2,true,11,true),TurnObstacleWait::Hold);
  EXPECT_EQ(wait.update(2.1,false,12,true),TurnObstacleWait::Hold);
  EXPECT_EQ(wait.update(3,true,13,true),TurnObstacleWait::Hold);
  EXPECT_EQ(wait.update(4,true,14,true),TurnObstacleWait::Timeout);
}
TEST(TurnWait, RewindDoesNotCountAsNewMap) {
  TurnObstacleWait wait;wait.update(1,false,10,true);wait.update(1.1,true,11,true);
  EXPECT_EQ(wait.update(1.2,true,9,true),TurnObstacleWait::Hold);
  EXPECT_EQ(wait.update(1.3,true,11,true),TurnObstacleWait::Hold);
  EXPECT_EQ(wait.update(1.4,true,12,true),TurnObstacleWait::Hold);
  EXPECT_EQ(wait.update(1.5,true,13,true),TurnObstacleWait::Proceed);
}
class TurnGeometry : public ::testing::Test {
protected:
  std::vector<unsigned char> cells=std::vector<unsigned char>(10000,0);
  TurnGrid grid{.05,-2.5,-2.5,100,100,&cells};
  std::vector<TurnPoint> footprint{{.8,.375},{.8,-.375},{-.3,-.375},{-.3,.375}};
  void obstacle(double x,double y,unsigned char cost=254) {
    cells[int((y+2.5)/.05)*100+int((x+2.5)/.05)]=cost;
  }
};
TEST_F(TurnGeometry, DoesNotRejectUnsweptRearAreaOfBoundingDisk) {
  obstacle(-.825,.025);
  EXPECT_EQ(checkTurnSweep(grid,footprint,0,0,0,.1),TurnSafety::Clear);
  EXPECT_EQ(checkTurnSweep(grid,footprint,0,0,0,M_PI),TurnSafety::Blocked);
}
TEST_F(TurnGeometry, RetainsCenterClearanceOutsidePhysicalFootprint) {
  obstacle(-.325,.025);
  std::vector<TurnPoint> small{{.1,.1},{.1,-.1},{-.1,-.1},{-.1,.1}};
  EXPECT_EQ(checkTurnSweep(grid,small,0,0,0,0),TurnSafety::Blocked);
}
TEST_F(TurnGeometry, UnknownAndInteriorCellsBlock) {
  obstacle(.425,.025,255);
  EXPECT_EQ(checkTurnSweep(grid,footprint,0,0,0,-M_PI),TurnSafety::Blocked);
}
TEST_F(TurnGeometry, PaddedCornerCollisionBetweenStartAndEnd) {
  obstacle(.875,-.175);
  EXPECT_EQ(checkTurnSweep(grid,footprint,0,0,.10879,-3.284),TurnSafety::Blocked);
  EXPECT_EQ(checkTurnSweep(grid,footprint,0,0,.10879,0),TurnSafety::Clear);
}
TEST_F(TurnGeometry, PreservesDynamicLargerFootprint) {
  obstacle(1.025,.025);
  EXPECT_EQ(checkTurnSweep(grid,footprint,0,0,0,.1),TurnSafety::Clear);
  footprint={{1.1,.4},{1.1,-.4},{-.3,-.4},{-.3,.4}};
  EXPECT_EQ(checkTurnSweep(grid,footprint,0,0,0,.1),TurnSafety::Blocked);
}
TEST_F(TurnGeometry, RejectsInvalidGeometryAndOutOfMapSweep) {
  EXPECT_EQ(checkTurnSweep(grid,footprint,2.2,0,0,.2),TurnSafety::Blocked);
  footprint[0][0]=NAN;
  EXPECT_EQ(checkTurnSweep(grid,footprint,0,0,0,.2),TurnSafety::Invalid);
}
