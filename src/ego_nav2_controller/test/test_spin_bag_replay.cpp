// Optional offline audit executable: no ROS initialization or velocity output.
// Input: time stamp resolution origin_x origin_y width height axle_x axle_y yaw arc,
// followed by width*height cost values. Geometry matches the measured robot profile.
#include <fstream>
#include <iostream>
#include <iomanip>
#include "ego_nav2_controller/spin_safety.hpp"
using namespace ego_nav2_controller;
int main(int argc,char **argv) {
  if(argc!=2)return 2;
  std::ifstream input(argv[1]);if(!input)return 2;
  TurnObstacleWait wait;double time,x,y,yaw,angle;int64_t stamp;TurnGrid grid;
  std::vector<TurnPoint> footprint{{.8,.375},{.8,-.375},{-.3,-.375},{-.3,.375}};
  while(input>>time>>stamp>>grid.resolution>>grid.origin_x>>grid.origin_y>>grid.width>>grid.height>>x>>y>>yaw>>angle) {
    if(grid.width>10000 || grid.height>10000)return 2;
    std::vector<unsigned char> cells(size_t(grid.width)*grid.height);unsigned cost;
    for(auto &cell:cells){if(!(input>>cost)||cost>255)return 2;cell=cost;}grid.cells=&cells;
    const auto state=checkTurnSweep(grid,footprint,x,y,yaw,angle);
    if(state==TurnSafety::Invalid)return 1;
    const auto action=wait.update(time,state==TurnSafety::Clear,stamp,true);
    std::cout<<std::fixed<<std::setprecision(6)<<time<<","<<state<<","<<action<<"\n";
  }
  return 0;
}
