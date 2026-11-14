#pragma once
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include "ego_nav2_controller/spin_safety.hpp"
namespace ego_nav2_controller {
inline bool bodyCollision(nav2_costmap_2d::Costmap2D *map,const std::vector<geometry_msgs::msg::Point> &footprint,
 double center_x,double center_y,double clearance,double x,double y,double yaw,double sweep_margin=-1) {
    const double margin=std::max(0.0,sweep_margin);
    const double hard_radius=clearance+margin;
    // Clearance is measured from base_link, not the drive axle or footprint edge.
    const double cx=x+std::cos(yaw)*center_x-std::sin(yaw)*center_y;
    const double cy=y+std::sin(yaw)*center_x+std::cos(yaw)*center_y;
    const double cell_guard=map->getResolution()*std::sqrt(2.0)/2;
    unsigned int ax,ay,bx,by;
    if(!map->worldToMap(cx-hard_radius,cy-hard_radius,ax,ay) ||
       !map->worldToMap(cx+hard_radius,cy+hard_radius,bx,by)) return true;
    for(unsigned int mx=ax;mx<=bx;++mx) for(unsigned int my=ay;my<=by;++my) {
      if(map->getCost(mx,my)<nav2_costmap_2d::LETHAL_OBSTACLE) continue;
      double wx,wy;map->mapToWorld(mx,my,wx,wy);
      if(sweep_margin>=0) {
        const double dx=std::max(0.0,std::abs(wx-cx)-map->getResolution()*.5);
        const double dy=std::max(0.0,std::abs(wy-cy)-map->getResolution()*.5);
        if(std::hypot(dx,dy)<=hard_radius)return true;
      } else if(std::hypot(wx-cx,wy-cy)<=hard_radius+cell_guard) return true;
    }
    if (footprint.size()<3) return true;
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(map);
    if (checker.footprintCostAtPose(x,y,yaw,footprint)>=nav2_costmap_2d::LETHAL_OBSTACLE) return true;
    std::vector<geometry_msgs::msg::Point> polygon;
    double xmin=1e9,xmax=-1e9,ymin=1e9,ymax=-1e9;
    for (auto p:footprint) {
      double px=x+std::cos(yaw)*p.x-std::sin(yaw)*p.y;
      double py=y+std::sin(yaw)*p.x+std::cos(yaw)*p.y;
      p.x=px;p.y=py;polygon.push_back(p);
      xmin=std::min(xmin,px);xmax=std::max(xmax,px);ymin=std::min(ymin,py);ymax=std::max(ymax,py);
    }
    unsigned int lo_x,lo_y,hi_x,hi_y;
    if (!map->worldToMap(xmin-margin,ymin-margin,lo_x,lo_y)||!map->worldToMap(xmax+margin,ymax+margin,hi_x,hi_y)) return true;
    for(unsigned int mx=lo_x;mx<=hi_x;++mx) for(unsigned int my=lo_y;my<=hi_y;++my) {
      if(map->getCost(mx,my)<nav2_costmap_2d::LETHAL_OBSTACLE) continue;
      double wx,wy;map->mapToWorld(mx,my,wx,wy); bool inside=false;
      if(sweep_margin>=0) {
        std::vector<TurnPoint> poly;for(const auto &p:polygon)poly.push_back({p.x,p.y});
        if(turnPolygonCell(poly,{wx,wy},map->getResolution()*.5+margin))return true;
        continue;
      }
      for(size_t i=0,j=polygon.size()-1;i<polygon.size();j=i++) {
        const auto &a=polygon[i];const auto &b=polygon[j];
        if((a.y>wy)!=(b.y>wy) && wx<(b.x-a.x)*(wy-a.y)/(b.y-a.y)+a.x) inside=!inside;
      }
      if(inside) return true;
    }
    return false;
}
}
