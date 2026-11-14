#pragma once
#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

namespace ego_nav2_controller {
// An odom-space, distance-bounded latch. A free STVL cell is NOT positive
// evidence of clearance: it may have aged out in the lidar's near blind zone.
class ObstacleMemory : public nav2_costmap_2d::Layer {
  std::map<std::pair<long long,long long>,std::pair<double,double>> cells_;
  double radius_{3.5}, x_{}, y_{}, resolution_{};
public:
  void onInitialize() override {
    if(layered_costmap_->getGlobalFrameID()!="odom" || !layered_costmap_->isRolling())
      throw std::runtime_error("ObstacleMemory requires a rolling odom costmap");
    declareParameter("retain_distance",rclcpp::ParameterValue(3.5));
    auto node=node_.lock();
    node->get_parameter(name_+".retain_distance",radius_);
    if(!std::isfinite(radius_) || radius_<2.5)
      throw std::runtime_error("ObstacleMemory retain_distance must be >= 2.5 m");
    enabled_=true;current_=true;
  }
  // Recovery clearing and controller/action switches must not erase a nearby
  // obstacle. Cleanup/recreation of the costmap starts a new odom session.
  void reset() override {current_=true;}
  bool isClearable() override {return false;}
  void updateBounds(double x,double y,double,double *lx,double *ly,double *hx,double *hy) override {
    x_=x;y_=y;
    auto *g=layered_costmap_->getCostmap();
    *lx=std::min(*lx,g->getOriginX());*ly=std::min(*ly,g->getOriginY());
    *hx=std::max(*hx,g->getOriginX()+g->getSizeInMetersX());
    *hy=std::max(*hy,g->getOriginY()+g->getSizeInMetersY());
  }
  void updateCosts(nav2_costmap_2d::Costmap2D &g,int lx,int ly,int hx,int hy) override {
    const double r=g.getResolution();
    if(resolution_ && std::abs(resolution_-r)>1e-9)
      throw std::runtime_error("ObstacleMemory resolution changed; recreate costmap");
    resolution_=r;
    for(auto it=cells_.begin();it!=cells_.end();) {
      if(std::hypot(it->second.first-x_,it->second.second-y_)>radius_)it=cells_.erase(it);
      else ++it;
    }
    // Runs AFTER both STVL layers and BEFORE inflation. The master is reset
    // for the full update bounds each cycle, so our own output is not re-read.
    for(int mx=lx;mx<hx;++mx)for(int my=ly;my<hy;++my) {
      if(g.getCost(mx,my)!=nav2_costmap_2d::LETHAL_OBSTACLE)continue;
      double wx,wy;g.mapToWorld(mx,my,wx,wy);
      if(std::hypot(wx-x_,wy-y_)<=radius_)
        cells_[{static_cast<long long>(std::floor(wx/r+1e-6)),static_cast<long long>(std::floor(wy/r+1e-6))}]={wx,wy};
    }
    for(const auto &item:cells_) {
      unsigned int mx,my;
      if(g.worldToMap(item.second.first,item.second.second,mx,my) &&
         int(mx)>=lx && int(mx)<hx && int(my)>=ly && int(my)<hy)
        g.setCost(mx,my,nav2_costmap_2d::LETHAL_OBSTACLE);
    }
    current_=true;
  }
};
}
