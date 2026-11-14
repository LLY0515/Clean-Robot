#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "plan_manage/planner_interface.h"
#include "ego_nav2_controller/smooth_detour.hpp"
#include "ego_nav2_controller/speed_envelope.hpp"
#include "ego_nav2_controller/spin_safety.hpp"
#include "ego_nav2_controller/trajectory_validation.hpp"
#include "ego_nav2_controller/continuous_tracking.hpp"
#include "ego_nav2_controller/precise_approach.hpp"
#include "ego_nav2_controller/footprint_safety.hpp"

namespace ego_nav2_controller {
using Pose = geometry_msgs::msg::PoseStamped;
using Point = ego_planner::PathPoint;
static double distance(const Pose &a, const Pose &b) {
  return std::hypot(a.pose.position.x-b.pose.position.x, a.pose.position.y-b.pose.position.y);
}
static double wrap(double a) { return std::atan2(std::sin(a),std::cos(a)); }
class EgoController : public nav2_core::Controller {
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D *map_{};
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr pub_;
  std::unique_ptr<ego_planner::PlannerInterface> planner_;
  nav_msgs::msg::Path plan_;
  std::vector<Point> trajectory_;
  std::vector<Point> correction_seed_;  // Solver input only; never executable.
  std::vector<Point> route_reference_;  // Original ordered route, never the warm-start detour.
  double return_check_period_{0.5}, return_deviation_{0.2}, return_improvement_{0.05}, max_path_age_{3.0};
  double last_return_check_{-1};
  int return_confirmations_{}, required_return_confirmations_{2};
  Point last_return_point_{};
  bool return_available_{};
  Point return_point_{};
  bool smooth_detour_enabled_{}, detour_active_{};
  // 已选定的绕行方向（+1 / -1，相对航线前进方向的左/右），绕行期间锁存。
  // 障碍表面被逐帧扫描补全时，两侧所需横移量会来回变化；如果每周期都
  // 重新比较，参考会左右换边，车先往一侧压再甩向另一侧（实测 0.5 m
  // 箱子先向右压 0.2 m 再向左横移 1.1 m，画出一个 S 形）。绕行一旦开始
  // 就保持同一侧，直到车沿参考回轨。
  int detour_side_{1};
  Point detour_end_{};
  // 已验证的平滑绕行参考（绝对坐标）。车心越过障碍后 forward 扫描就看不到
  // 障碍了，生成器会返回空；此时车尾还在障碍旁边，必须沿用这一份参考直到
  // 它走完，"通过障碍后车屁股摆尾撞障碍" 就是这样产生的。
  std::vector<Point> detour_guide_;
  // 连续被优化器拒绝的绕行参考次数：只失败一帧不清除参考（见 replan 里的
  // "detour reference rejected" 注释），连续 3 次才放弃。
  int guide_fail_count_{};
  double detour_transition_, detour_return_, detour_offset_, detour_weight_, return_forward_;
  double soft_margin_, route_weight_, length_weight_, curve_preview_, lateral_accel_;
  double maneuver_speed_, slow_curvature_, slow_obstacle_distance_, speed_reaction_, speed_margin_;
  std::string name_;
  double max_v_, max_w_, accel_, angular_accel_, lookahead_, horizon_, replan_period_, budget_, max_pose_age_;
  double speed_limit_, last_plan_time_{-1}, last_attempt_time_{-1}, last_command_time_{-1};
  double last_linear_command_{}, last_angular_command_{};
  double inflation_{}, clearance_{0.60}, center_x_{}, center_y_{};
  double axle_offset_x_{}, axle_offset_y_{};
  size_t cursor_{};
  bool active_{};
  bool preserve_reference_curve_{};
  bool continuous_started_{};
  ContinuousTurnResponse turn_response_;
  ContinuousYawServo yaw_servo_;
  bool precise_goal_alignment_{};
  PreciseApproach precise_approach_;
  bool trajectory_requires_sweep_{};
  bool aligning_goal_{};
  bool rotating_heading_{};
  int rotate_sign_{0};  // 原地旋转时锁存的转向（+1 逆时针 / -1 顺时针）
  double rotate_target_yaw_{0};  // 原地旋转时锁存的**绝对**目标朝向（map 系）
  double replan_hold_until_{-1.0};  // 转完原地旋转后短暂冻结重规划，避免目标翻转
  double heading_enter_{},heading_exit_{},frame_translation_threshold_{},frame_rotation_threshold_{};
  geometry_msgs::msg::TransformStamped cycle_transform_;
  tf2::Transform trajectory_transform_;
  bool trajectory_transform_valid_{};
  std::mutex mutex_;
  double param(const std::string &key, double value) {
    nav2_util::declare_parameter_if_not_declared(node_,name_+"."+key,rclcpp::ParameterValue(value));
    double v=node_->get_parameter(name_+"."+key).as_double();
    if (!std::isfinite(v) || v<=0) throw nav2_core::PlannerException("Invalid EGO parameter: "+key);
    return v;
  }
  Pose transform(const Pose &p,const std::string &frame) {
    if (p.header.frame_id==frame) return p;
    Pose out;
    // One transform snapshot per control cycle: every reference point and the
    // final goal must use the same map correction during optimization.
    if(p.header.frame_id==plan_.header.frame_id && frame==mapFrame()) {
      tf2::doTransform(p,out,cycle_transform_);return out;
    }
    if(p.header.frame_id==mapFrame() && frame==plan_.header.frame_id) {
      tf2::Transform forward;tf2::fromMsg(cycle_transform_.transform,forward);
      auto reverse=cycle_transform_;reverse.header.frame_id=plan_.header.frame_id;
      reverse.child_frame_id=mapFrame();reverse.transform=tf2::toMsg(forward.inverse());
      tf2::doTransform(p,out,reverse);return out;
    }
    try { tf_->transform(p,out,frame,tf2::durationFromSec(0.05)); }
    catch(const tf2::TransformException &e) { throw nav2_core::PlannerException(e.what()); }
    return out;
  }
  bool collision(double x,double y,double yaw,double sweep_margin=-1) {
    return bodyCollision(map_,costmap_ros_->getRobotFootprint(),center_x_,center_y_,clearance_,x,y,yaw,sweep_margin);
  }
  template<class Deadline>
  bool safeTrajectory(const std::vector<Point> &path,size_t first,const Pose &pose,Deadline deadline) {
    double radius=clearance_+std::hypot(center_x_,center_y_);
    const auto footprint=costmap_ros_->getRobotFootprint();
    std::vector<TurnPoint> poly;
    for(const auto &p:footprint) {radius=std::max(radius,std::hypot(p.x,p.y));poly.push_back({p.x,p.y});}
    if(!validTurnFootprint(poly))return false;
    double x=pose.pose.position.x,y=pose.pose.position.y,yaw=tf2::getYaw(pose.pose.orientation);
    for(size_t i=first;i<path.size();++i) {
      const auto &p=path[i];
      if(!safeMotionSegment(x,y,yaw,p.x,p.y,p.z,radius,map_->getResolution(),
          [&](double px,double py,double a,double margin){return collision(px,py,a,margin);},deadline)) {
        RCLCPP_DEBUG(node_->get_logger(),"EGO swept check rejected segment %zu: (%.3f, %.3f, %.3f) -> (%.3f, %.3f, %.3f)",
          i,x,y,yaw,p.x,p.y,p.z);
        return false;
      }
      x=p.x;y=p.y;yaw=p.z;
    }
    return first<path.size();
  }
  void checkCommand(const Pose &p,double v,double w) {
    if(max_v_>.4 && std::abs(v)>maneuver_speed_) {
      checkBrakingArc(p,v,w);return;
    }
    double x=p.pose.position.x,y=p.pose.position.y,yaw=tf2::getYaw(p.pose.orientation);
    // Check the commanded arc through reaction delay and full braking time.
    double duration=std::max(1.0,std::max(std::abs(v)/accel_,std::abs(w)/angular_accel_)+0.35);
    double dt=std::min(0.05,map_->getResolution()/(2*(std::abs(v)+std::abs(w)*1.0+0.01)));
    for(double t=0;t<=duration;t+=dt) {
      if(collision(x,y,yaw)) throw nav2_core::PlannerException("EGO command footprint collision/unknown map");
      x+=v*std::cos(yaw)*dt;y+=v*std::sin(yaw)*dt;yaw+=w*dt;
    }
  }
  bool closeObstacle(double x,double y,double yaw) {
    const double cx=x+std::cos(yaw)*center_x_,cy=y+std::sin(yaw)*center_x_;
    unsigned int ax,ay,bx,by;
    if(!map_->worldToMap(cx-slow_obstacle_distance_,cy-slow_obstacle_distance_,ax,ay) ||
       !map_->worldToMap(cx+slow_obstacle_distance_,cy+slow_obstacle_distance_,bx,by))return true;
    for(unsigned int mx=ax;mx<=bx;++mx)for(unsigned int my=ay;my<=by;++my) {
      if(map_->getCost(mx,my)<nav2_costmap_2d::LETHAL_OBSTACLE)continue;
      double wx,wy;map_->mapToWorld(mx,my,wx,wy);
      if(std::hypot(wx-cx,wy-cy)<=slow_obstacle_distance_)return true;
    }
    return false;
  }
  void checkBrakingArc(const Pose &p,double v,double w) {
    if(std::abs(v)<=maneuver_speed_)return;
    double x=p.pose.position.x,y=p.pose.position.y,yaw=tf2::getYaw(p.pose.orientation);
    const double reaction=speed_reaction_+speed_margin_/std::max(.1,std::abs(v));
    const double duration=reaction+std::max(std::abs(v)/accel_,std::abs(w)/angular_accel_);
    const double dt=std::min(.025,map_->getResolution()/(2*(std::abs(v)+std::abs(w)+.01)));
    auto reduce=[](double value,double amount){return std::copysign(std::max(0.0,std::abs(value)-amount),value);};
    for(double t=0;t<=duration+dt;t+=dt) {
      if(collision(x,y,yaw))throw nav2_core::PlannerException("EGO speed braking corridor blocked");
      x+=v*std::cos(yaw)*dt;y+=v*std::sin(yaw)*dt;yaw+=w*dt;
      if(t>=reaction){v=reduce(v,accel_*dt);w=reduce(w,angular_accel_*dt);}
    }
  }
  static double segmentDistance(double x,double y,const Point &a,const Point &b) {
    double dx=b.x-a.x,dy=b.y-a.y;
    double f=std::clamp(((x-a.x)*dx+(y-a.y)*dy)/std::max(1e-12,dx*dx+dy*dy),0.0,1.0);
    return std::hypot(x-a.x-f*dx,y-a.y-f*dy);
  }
  double routeDistance(double x,double y) const {
    double result=1e9;
    for(size_t i=1;i<route_reference_.size();++i)
      result=std::min(result,segmentDistance(x,y,route_reference_[i-1],route_reference_[i]));
    return result;
  }
  // Compare the same near-term travel distance, not paths with different horizons.
  double returnScore(const std::vector<Point> &path,const Pose &pose) const {
    size_t near=0;double nearest=1e9;
    for(size_t i=0;i<path.size();++i) {
      double d=std::hypot(path[i].x-pose.pose.position.x,path[i].y-pose.pose.position.y);
      if(d<nearest){nearest=d;near=i;}
    }
    double length=0,area=0;
    for(size_t i=near+1;i<path.size() && length<1.5;++i) {
      double ds=std::min(1.5-length,double(std::hypot(path[i].x-path[i-1].x,path[i].y-path[i-1].y)));
      area+=ds*(routeDistance(path[i-1].x,path[i-1].y)+routeDistance(path[i].x,path[i].y))*.5;
      length+=ds;
    }
    return length>.1?area/length:1e9;
  }
  bool safeSegment(const Pose &a,const Pose &b,double yaw) {
    const int steps=std::max(1,int(std::ceil(distance(a,b)/(map_->getResolution()*.5))));
    for(int i=0;i<=steps;++i)
      if(collision(a.pose.position.x+(b.pose.position.x-a.pose.position.x)*i/steps,
                   a.pose.position.y+(b.pose.position.y-a.pose.position.y)*i/steps,yaw)) return false;
    return true;
  }
  // Earliest forward connection in the current ordered segment, with a safe run-out.
  // This is only a candidate: the actual spline and command arc are still checked.
  size_t returnCandidate(const Pose &pose,const std::vector<Pose> &local) {
    for(size_t i=1;i+1<local.size();++i) {
      const double dx=local[i].pose.position.x-pose.pose.position.x,dy=local[i].pose.position.y-pose.pose.position.y;
      if(std::hypot(dx,dy)<std::max(lookahead_,return_forward_)) continue;
      const double heading=std::atan2(dy,dx);
      if(std::abs(wrap(heading-tf2::getYaw(pose.pose.orientation)))>1.57079632679) continue;
      double runout=0;bool safe=true;
      for(size_t j=i+1;j<local.size() && runout<lookahead_;++j) {
        const double route_yaw=std::atan2(local[j].pose.position.y-local[j-1].pose.position.y,
                                        local[j].pose.position.x-local[j-1].pose.position.x);
        if(std::abs(wrap(route_yaw-heading))>1.57079632679 || !safeSegment(local[j-1],local[j],route_yaw)){safe=false;break;}
        runout+=distance(local[j-1],local[j]);
      }
      if(!safe || runout<lookahead_ || !safeSegment(pose,local[i],heading))continue;
      return i;
    }
    return local.size();
  }
  std::vector<Point> reference(const Pose &pose,bool prefer_return=false) {
    Pose query=pose;query.header.stamp=rclcpp::Time(0);query=transform(query,plan_.header.frame_id);
    double best=1e9,arc=0,best_arc=0;size_t closest=cursor_;
    for(size_t i=cursor_;i<plan_.poses.size();++i) {
      if(i>cursor_) arc+=distance(plan_.poses[i-1],plan_.poses[i]);
      if(arc>std::max(3.0,horizon_*2)) break;
      // Stop after the first local distance minimum. A nearby later coverage
      // lane must not steal progress from the current segment.
      // 【修复"原地左右旋转"的根因（run27 第 20/22 条 swath，各转 185 s）】
      // 原逻辑只看"第一个局部最小 + 0.75 m"就停，前提是车贴着航线。掉头时
      // 车会抄近道从半圆内侧切过去、离航线 1.5 m：此时沿线 0.75 m 内最近
      // 的仍是弧线起点（车刚离开那里），真正最近的航线点在 2.4 m 之外，
      // cursor_ 由此永久卡在 1。之后每次 reference() 都从弧线起点往回铺
      // 参考 → 样条绕圈 → 前视点塌缩到几厘米（look=(0.05,0.03)）→ 转向
      // 取噪声方向 → 车原地左右旋转，且自我维持到 FollowPath 180 s 超时。
      // 只有车确实在航线上（最近距离 <=0.35 m）才信任第一个局部最小；
      // 离航线较远时继续向前找全局最近点（仍受上面的 9 m 弧长上限约束，
      // 不会跳到远处的平行车道）。
      if(arc-best_arc>0.75 && best<=0.35) break;
      double d=distance(query,plan_.poses[i]);if(d<best) {best=d;closest=i;best_arc=arc;}
    }
    if(best>horizon_) throw nav2_core::PlannerException("Robot too far from reference path");
    cursor_=closest;
    std::vector<Pose> local{pose};double length=0;
    for(size_t i=std::min(closest+1,plan_.poses.size()-1);i<plan_.poses.size();++i) {
      Pose q=plan_.poses[i];q.header=plan_.header;q.header.stamp=rclcpp::Time(0);q=transform(q,mapFrame());
      unsigned int mx,my;
      if(!map_->worldToMap(q.pose.position.x,q.pose.position.y,mx,my)) break;
      if(distance(pose,q)>horizon_) break;
      double d=distance(local.back(),q);if(d<0.01)continue;
      length+=d;local.push_back(q);if(length>=horizon_) break;
    }
    if(local.size()<2 || length<0.02) throw nav2_core::PlannerException("No local reference inside costmap");
    route_reference_.clear();
    Pose anchor=plan_.poses[closest];anchor.header=plan_.header;anchor.header.stamp=rclcpp::Time(0);anchor=transform(anchor,mapFrame());
    route_reference_.push_back({float(anchor.pose.position.x),float(anchor.pose.position.y),0,0});
    for(size_t i=1;i<local.size();++i)
      route_reference_.push_back({float(local[i].pose.position.x),float(local[i].pose.position.y),0,0});
    return_available_=false;
    if(prefer_return) {
      const size_t join=returnCandidate(pose,local);
      if(join<local.size()) {
        return_available_=true;
        return_point_={float(local[join].pose.position.x),float(local[join].pose.position.y),0,0};
        local.erase(local.begin()+1,local.begin()+join);
        length=0;for(size_t i=1;i<local.size();++i)length+=distance(local[i-1],local[i]);
      }
    }
    // Seed replanning with the previous optimized detour instead of repeatedly
    // pulling the initial guess back through the obstacle on the global line.
    const auto &seed=trajectory_.empty()?correction_seed_:trajectory_;
    if(!prefer_return && !seed.empty()) {
      size_t near=0;double best_distance=1e9;
      for(size_t i=0;i<seed.size();++i) {
        double d=std::hypot(seed[i].x-pose.pose.position.x,seed[i].y-pose.pose.position.y);
        if(d<best_distance){best_distance=d;near=i;}
      }
      if(best_distance<.5 && near+2<seed.size()) {
        std::vector<Pose> warm{pose};double warm_length=0;
        for(size_t i=near+1;i<seed.size();++i) {
          Pose q=pose;q.pose.position.x=seed[i].x;q.pose.position.y=seed[i].y;
          double d=distance(warm.back(),q);if(d<.01)continue;
          warm_length+=d;warm.push_back(q);
        }
        size_t join=0;double join_dist=1e9;
        for(size_t i=0;i<local.size();++i){double d=distance(warm.back(),local[i]);if(d<join_dist){join_dist=d;join=i;}}
        for(size_t i=join+1;i<local.size()&&warm_length<horizon_;++i){warm_length+=distance(warm.back(),local[i]);warm.push_back(local[i]);}
        // 【修复"原地左右旋转"再现（run24 第 20 条 swath，连转 185 s）】
        // 缓存轨迹快走完时（车距其终点 <0.5 m、只剩两三个点），warm 只剩
        // 0.1~0.3 m 的尾巴，却把整条 4.5 m 航线路由**替换掉**：参考只剩
        // 0.1 m → 样条目标点贴着车 → lookahead 塌缩到 0.1 m；尾巴方向若
        // 朝后就成了 180° 翻转，车原地转半圈；而这条小样条又成为下次的
        // seed，状态自我维持，直到 FollowPath 180 s 超时。
        // 只有 warm 仍能撑起有效前向长度时才采用，否则保留航线路由。
        if(warm.size()>2 && warm_length>=1.0){local=std::move(warm);length=warm_length;}
      }
    }
    // Arc-length sampling keeps spline size bounded on 16k-point coverage paths.
    const int n=std::max(8,static_cast<int>(std::ceil(length/0.2))+1);
    std::vector<Point> result;size_t seg=1;double passed=0;
    for(int i=0;i<n;++i) {
      double target=length*i/(n-1);
      while(seg+1<local.size() && passed+distance(local[seg-1],local[seg])<target) {
        passed+=distance(local[seg-1],local[seg]);++seg;
      }
      double d=distance(local[seg-1],local[seg]);double f=std::clamp((target-passed)/d,0.0,1.0);
      auto a=local[seg-1].pose.position,b=local[seg].pose.position;
      result.push_back({float(a.x+f*(b.x-a.x)),float(a.y+f*(b.y-a.y)),0,0});
    }
    return result;
  }
  std::vector<Point> curveReference(const Pose &pose) {
    Pose query=pose;query.header.stamp=rclcpp::Time(0);query=transform(query,plan_.header.frame_id);
    double best=distance(query,plan_.poses[cursor_]),arc=0;size_t closest=cursor_;
    // Only inspect the immediate ordered neighborhood; self-intersecting turns
    // must never jump to a later pass through the same position.
    for(size_t i=cursor_+1;i<plan_.poses.size();++i) {
      arc+=distance(plan_.poses[i-1],plan_.poses[i]);if(arc>.25)break;
      const double d=distance(query,plan_.poses[i]);
      if(d<best){best=d;closest=i;}
      else if(i>closest+2)break;
    }
    cursor_=closest;
    const double heading_error=wrap(tf2::getYaw(query.pose.orientation)-tf2::getYaw(plan_.poses[closest].pose.orientation));
    if(best>.08 || std::abs(heading_error)>.10)
      throw nav2_core::PlannerException("Continuous coverage tracking error: position="+std::to_string(best)+
        " m, yaw="+std::to_string(heading_error)+" rad; limits 0.08 m / 0.10 rad; stop and realign");
    std::vector<Point> ref;arc=0;
    for(size_t i=closest;i<plan_.poses.size();++i) {
      if(i>closest)arc+=distance(plan_.poses[i-1],plan_.poses[i]);
      Pose q=plan_.poses[i];q.header=plan_.header;q.header.stamp=rclcpp::Time(0);q=transform(q,mapFrame());
      ref.push_back({float(q.pose.position.x),float(q.pose.position.y),float(tf2::getYaw(q.pose.orientation)),0});
      if(arc>=horizon_)break;
    }
    return ref;
  }
  std::string mapFrame() const {return costmap_ros_->getGlobalFrameID();}
  bool replan(const Pose &pose,const geometry_msgs::msg::Twist &velocity,double now,bool opportunistic=false) {
    auto start=std::chrono::steady_clock::now();
    auto ref=reference(pose,opportunistic);
    correction_seed_.clear();  // One proposal only; failed replans cannot track it.
    if(opportunistic && !return_available_) return false;
    Point cur{float(pose.pose.position.x),float(pose.pose.position.y),float(tf2::getYaw(pose.pose.orientation)),float(velocity.linear.x)};
    // 【修复 U 型掉头处"左右来回转"的根因】PlannerInterface::makePlan()
    // 用 start_vel = cur.v * (cos yaw, sin yaw) 作为 B 样条起点的切向约束。
    // 原地旋转/停车的周期里实测 v≈0，这个约束就消失了，样条起点方向变成
    // 无约束：在 U 型掉头顶点（航线同时在车头前方和后方）优化器有两个
    // 局部最优，每 0.5 s 重规划就翻一次，lookahead 在车前后跳变，车被
    // 要求左转、再右转，原地来回转（run20 第 4 条 swath 转了 190 s）。
    // 这里给起点切向一个**沿航线前进方向**的小虚拟速度，把它钉死。
    // 注意不能用"沿车头方向"做种子（run22 实测）：掉头顶点车头朝后，
    // 样条要从车头甩一圈绕回航线，扫掠进旁边的障碍区，footprint 校验
    // 连续失败直接 patience。沿航线方向种子既稳定又贴合可执行弧线。
    Point seed_state=cur;
    seed_state.v=float(std::max(std::abs(velocity.linear.x),0.25));
    if(std::abs(velocity.linear.x)<0.15 && ref.size()>1) {
      double acc=0;size_t i=1;
      for(;i<ref.size();++i) {
        acc+=std::hypot(ref[i].x-ref[i-1].x,ref[i].y-ref[i-1].y);
        if(acc>=0.6) break;
      }
      // 剩余航线不足 0.6 m（条带末端）时退回"指向剩余航线端点"，否则
      // 种子又变成无约束，末端会出现"朝终点/背终点"翻转。
      if(i>=ref.size() && ref.size()>1) i=ref.size()-1;
      if(i<ref.size()) {
        const double dx=ref[i].x-ref[0].x,dy=ref[i].y-ref[0].y;
        if(std::hypot(dx,dy)>0.05) seed_state.z=float(std::atan2(dy,dx));
      }
    }
    // Core rolls/reinitializes its grid; explicitly cleared by the vendor patch.
    planner_->setSearchInflation(inflation_);
    planner_->setGridMap(cur);planner_->setCurrentVehiclePos(seed_state);
    std::vector<ego_planner::ObstacleInfo> obstacles;
    const double width=std::ceil(map_->getSizeInMetersX()+2.0),height=std::ceil(map_->getSizeInMetersY()+2.0);
    const double res=map_->getResolution();
    for(double x=cur.x-width/2+res/2;x<cur.x+width/2;x+=res)
      for(double y=cur.y-height/2+res/2;y<cur.y+height/2;y+=res) {
        unsigned int mx,my;
        if(!map_->worldToMap(x,y,mx,my) || map_->getCost(mx,my)>=nav2_costmap_2d::LETHAL_OBSTACLE)
          obstacles.push_back({float(x),float(y),0});
      }
    planner_->setObstacles(obstacles);
    Pose terminal=plan_.poses.back();terminal.header=plan_.header;terminal.header.stamp=rclcpp::Time(0);
    terminal=transform(terminal,mapFrame());
    const bool terminal_blocked=std::hypot(ref.back().x-terminal.pose.position.x,
      ref.back().y-terminal.pose.position.y)<.02 && planner_->inflatedOccupied(ref.back().x,ref.back().y);
    if(planner_->inflatedOccupied(cur.x,cur.y) && !terminal_blocked) {
      // The enclosing axle circle can reject a safe body pose (recorded on
      // Oct 28). Only in that case use a less conservative SEARCH heuristic.
      // It cannot authorize motion: solve() and checkCommand() still enforce
      // the configured offset body-center circle and complete physical footprint.
      planner_->setSearchInflation(clearance_);
      planner_->setGridMap(cur);
      planner_->setObstacles(obstacles);
    }
    // A verified, elongated guide becomes the optimization reference as well
    // as the seed, so attraction to the original line cannot collapse it into
    // a short boundary-hugging arc. Original route_reference_ stays unchanged.
    const auto original_ref=ref;
    bool guided=false;
    std::vector<Point> guide;
    // 【关键】参考的可行性判据必须和 checkCommand() 一致，不能直接用
    // EGO 的 0.50 m 膨胀场：checkCommand 要求车心离 lethal 栅格 >=
    // clearance 0.45 + margin 0.10 + 栅格余量 ≈ 0.59 m。参考若只按
    // 0.50 m 排布，就会把车带到"参考合法、命令非法"的位置——实测绕行
    // 到障碍侧旁后连续报 "EGO command footprint collision/unknown map"，
    // 直到 Controller patience exceeded 停车。这里按 0.60 m 判。
    // guide_free 返回 true 表示该点离 lethal 栅格太近（名字沿用历史，
    // 与 smoothDetour 的 occupied 参数同义）。
    // 【run52】再留 0.10 m 感知余量：箱子是逐帧补全的，刚发现时只看到一部分，
    // 按"刚好 0.60 m"生成的 0.8 m 横移参考，等箱体补全后就不够了，绕行半途
    // 必须把横移量加到 1.0 m——此时离障碍只剩 1.5 m，坡度段被压得跟不住。
    // 0.65 m 让 0.5 m 箱子从第一帧起就选 1.0 m 横移，避免中途升级。
    const double guide_clear=clearance_+soft_margin_+0.10;
    auto guide_free=[&](double x,double y) {
      unsigned int ax,ay,bx,by;
      if(!map_->worldToMap(x-guide_clear,y-guide_clear,ax,ay) ||
         !map_->worldToMap(x+guide_clear,y+guide_clear,bx,by))return true;
      for(unsigned int mx=ax;mx<=bx;++mx) for(unsigned int my=ay;my<=by;++my) {
        if(map_->getCost(mx,my)<nav2_costmap_2d::LETHAL_OBSTACLE)continue;
        double wx,wy;map_->mapToWorld(mx,my,wx,wy);
        const double dx=std::max(0.0,std::abs(wx-x)-res*.5);
        const double dy=std::max(0.0,std::abs(wy-y)-res*.5);
        if(std::hypot(dx,dy)<=guide_clear)return true;
      }
      return false;
    };
    // 参考生成要用**原航线**（route_reference_，条带直线），不能用 ref：
    // 绕行进行中 reference() 会用上一次的绕行轨迹做 warm seed，ref 本身就
    // 带着 0.5~0.9 m 的横向偏移，smoothDetour 的直线性检查会整条拒绝，
    // 于是每个周期都生成不出新参考、只能死抱旧参考，旧参考一旦被新栅格
    // 判为过近就整段丢掉绕行（实测在箱体旁边出现"retained guide blocked"
    // 后立刻退回原航线，车尾扫向障碍）。这里把车位姿接在航线前面。
    std::vector<Point> guide_route;
    if(route_reference_.size()>1) {
      guide_route.push_back({float(cur.x),float(cur.y),0,0});
      for(const auto &p:route_reference_)
        if(std::hypot(p.x-cur.x,p.y-cur.y)>0.3)guide_route.push_back(p);
    }
    // 【run50 修复】绕行一旦开始，**优先沿用上一周期已验证的参考**，而不是
    // 每周期按当前车位姿重新生成。参考是在离障碍更远、横移起点更早时生成的，
    // 坡度段完整；车靠近后重新生成会把同样的横移量压进剩余的很短距离里
    // （实测 1.0 m 横移的坡度段被压到 0.75 m，曲率 ~10 1/m），优化器必然
    // 拒绝，随后退回 ±0.4 m 侧向种子、左右换边，车贴着箱子蹭过去。
    bool was_retained=false;
    if(smooth_detour_enabled_ && !opportunistic && detour_active_ && !detour_guide_.empty()) {
      size_t nearest=0;double best=1e9;
      for(size_t i=0;i<detour_guide_.size();++i) {
        const double d=std::hypot(detour_guide_[i].x-cur.x,detour_guide_[i].y-cur.y);
        if(d<best){best=d;nearest=i;}
      }
      double remaining=0;
      for(size_t i=nearest+1;i<detour_guide_.size();++i)
        remaining+=std::hypot(detour_guide_[i].x-detour_guide_[i-1].x,
                              detour_guide_[i].y-detour_guide_[i-1].y);
      // 车必须在参考附近（否则参考已失效），裁掉的部分不能是整条参考。
      // 1.0 m 的容差用来吸收跟踪滞后：车若暂时没跟上参考的横移，参考本身
      // 仍然有效，不应因此重新生成（重新生成正是坡度段被压缩的根源）。
      if(best<1.0 && remaining>.3) {
        // 保留的参考必须仍满足 guide_clear：前方扫描看不到障碍后这里不再
        // 重新生成参考，若期间地图里出现了新栅格（或障碍表面被扫描补全），
        // 直接沿用旧参考会把车带回障碍旁边。
        bool still_free=true;
        for(size_t i=nearest;i<detour_guide_.size() && still_free;i+=4)
          still_free=!guide_free(detour_guide_[i].x,detour_guide_[i].y);
        if(still_free) {
          guide.assign(detour_guide_.begin()+nearest,detour_guide_.end());
          if(best>0.15)guide.insert(guide.begin(),{float(cur.x),float(cur.y),0,0});
          was_retained=true;
        }
      }
    }
    std::string guide_why;
    if(guide.empty() && smooth_detour_enabled_ && !opportunistic && guide_route.size()>1) {
      // 朝向门限：条带起点刚完成转场时车头可能还偏 0.2~0.3 rad，0.25 会让
      // 绕行参考在最需要的时刻（障碍刚出现、车还在对齐）整段失效；参考曲线
      // 本身从车位姿出发、横向偏移逐段爬升，0.35 rad 以内都能跟住。
      const double heading=std::atan2(guide_route.back().y-guide_route.front().y,
                                      guide_route.back().x-guide_route.front().x);
      // 绕行进行中车头本来就斜着走（参考的坡度段要求航向偏 15°~25°，实测
      // 0.36 rad 就把 0.35 的门限顶破）：这时若因障碍表面补全而需要把横移量
      // 从 0.8 加到 1.0，生成器会被门限拦下，整条绕行被清掉，退化成 ±0.4 m
      // 侧向种子并翻到障碍另一侧（run52）。绕行期间把门限放宽到 0.9 rad；
      // 还没开始绕行时保持 0.35，避免转场掉头瞬间生成无意义参考。
      const double heading_limit=detour_active_?0.9:0.35;
      if(std::abs(wrap(heading-cur.z))<heading_limit) {
        // 车头/车尾相对 base_link 的长度：横向偏移必须在车头够到障碍前完成，
        // 并且一直保持到车尾（含后角点）完全越过障碍再回轨——否则回轨的转弯
        // 会把车尾扫进障碍（用户实测"车屁股摆尾容易撞到障碍物"）。
        double front_extent=0,rear_extent=0;
        for(const auto &p:costmap_ros_->getRobotFootprint()) {
          front_extent=std::max(front_extent,p.x);rear_extent=std::max(rear_extent,-p.x);
        }
        // 绕行开始后锁存上一次选定的方向（见 detour_side_ 的说明）；
        // 还没开始绕行时传 0，由 smoothDetour 自动选横移量小的一侧。
        const int preferred_side=detour_active_?detour_side_:0;
        // 坡度段的长度按曲率上限反推（smoothDetour 内部），上限取车在最高速
        // 下能跟住的最大曲率 max_w/max_v 再留 15% 余量。
        const double kappa=max_v_>1e-3?0.85*max_w_/max_v_:0.9;
        guide=smoothDetour(guide_route,guide_free,
                           [&](double x,double y){unsigned int mx,my;
                             return map_->worldToMap(x,y,mx,my) &&
                               map_->getCost(mx,my)>=nav2_costmap_2d::LETHAL_OBSTACLE;},
                           detour_transition_,front_extent+.15,rear_extent+.25,
                           detour_return_,detour_offset_,map_->getResolution()*.5,
                           preferred_side,kappa,&guide_why);
      } else {
        guide_why="heading error > 0.35 rad";
      }
    }
    if(!guide.empty()) {
      // 从已接受的参考反推绕行方向并锁存：取横向偏离航线最大的点，符号即
      // 绕行侧（与 smoothDetour 的 lateral 约定一致）。横移量太小时不动，
      // 保留上一次的取值。
      if(guide_route.size()>1) {
        const double ax=guide_route.front().x, ay=guide_route.front().y;
        double tx=guide_route.back().x-ax, ty=guide_route.back().y-ay;
        const double norm=std::hypot(tx,ty);
        if(norm>1e-6) {
          tx/=norm;ty/=norm;
          double peak=0;
          for(const auto &p:guide) {
            const double lateral=-(p.x-ax)*ty+(p.y-ay)*tx;
            if(std::abs(lateral)>std::abs(peak))peak=lateral;
          }
          if(std::abs(peak)>0.15)detour_side_=peak>0?1:-1;
        }
      }
      detour_guide_=guide;
      ref.clear();
      const size_t stride=std::max(size_t(1),size_t(std::round(.2/(map_->getResolution()*.5))));
      for(size_t i=0;i<guide.size();i+=stride)ref.push_back(guide[i]);
      if(std::hypot(ref.back().x-guide.back().x,ref.back().y-guide.back().y)>1e-5)ref.push_back(guide.back());
      // 【run53】参考必须覆盖完整前视距离。绕行参考的末端按构造已经回到航线
      // （横向偏移归零），如果车已经走到参考尾段，ref 只剩零点几米，样条也会
      // 终止在那里，控制器按"快到终点"把速度压到 0.02 m/s——实测绕过箱子后
      // 原地爬行 12 s，最后由执行器的"进度停滞"兜底才继续。把航线在参考终点
      // 之后的点接上，保证优化器总有 ≥ 4.5 m 的可行参考。
      if(guide_route.size()>1) {
        const double gx=guide_route.back().x-guide_route.front().x;
        const double gy=guide_route.back().y-guide_route.front().y;
        const double gn=std::hypot(gx,gy);
        if(gn>1e-6) {
          const double tx=gx/gn,ty=gy/gn;
          const double end_s=(ref.back().x-guide_route.front().x)*tx+
                             (ref.back().y-guide_route.front().y)*ty;
          const double end_lat=-(ref.back().x-guide_route.front().x)*ty+
                                (ref.back().y-guide_route.front().y)*tx;
          // 【run60】参考末端还没回到航线时绝不能拼航线点：横向会突跳
          // 0.5~1.1 m，优化器只能把它拟合成曲率 ~10 1/m 的折角，曲率限速
          // 随后把速度压到 0.05 m/s（run58/59 回轨汇合点爬行 20 s 的根因）。
          // 让参考停在坡度段即可，下一周期按新位姿继续回轨。
          if(std::abs(end_lat)<=0.08)
            for(const auto &p:route_reference_) {
              const double ps=(p.x-guide_route.front().x)*tx+(p.y-guide_route.front().y)*ty;
              if(ps>end_s+0.05)ref.push_back(p);
            }
        }
      }
      guided=true;
      if(!was_retained && !guide_why.empty())
        RCLCPP_INFO_THROTTLE(node_->get_logger(),*node_->get_clock(),2000,
          "EGO detour reference: %s",guide_why.c_str());
    } else if(detour_active_) {
      // 参考已走完（车已沿参考回轨），恢复正常参考与回轨逻辑
      detour_active_=false;detour_guide_.clear();guide_fail_count_=0;
    } else if(!guide_why.empty() &&
              guide_why.rfind("no blocked",0)!=0 && guide_why.rfind("route ",0)!=0 &&
              guide_why.rfind("remaining route",0)!=0 && guide_why.rfind("heading error",0)!=0) {
      // "前方没有障碍/航线不直/航向还没对齐"是正常驾驶的常态，不刷日志；
      // 只有"确实看到障碍却生成不出参考"才值得告警。
      RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),2000,
        "EGO detour reference not generated: %s",guide_why.c_str());
    }
    planner_->setReturnPolicy(soft_margin_,guided?detour_weight_:route_weight_,length_weight_);
    planner_->setRouteReference(guided?ref:route_reference_);
    auto check_budget=[&]() {
      if(std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()>budget_)
        throw nav2_core::PlannerException("EGO planning/validation exceeded time budget; result discarded");
    };
    std::vector<Point> candidate;
    std::string failure;
    bool terminal_retry=false;
    auto solve=[&](std::vector<Point> seed) {
      check_budget();
      planner_->setPathPoint(seed);planner_->makePlan();
      check_budget();
      planner_->getLocalPlanTrajResults(candidate);
      if(candidate.size()<2){failure="EGO optimization failed: "+planner_->lastError();return false;}
      for(const auto &p:candidate) {
        if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)) {
          failure="Nonfinite EGO trajectory";candidate.clear();return false;
        }
        if(collision(p.x,p.y,p.z)) {
          failure="New EGO trajectory fails clearance/footprint verification";candidate.clear();return false;
        }
      }
      if(terminal_retry) {
        // Do not accept spline endpoint drift as an alternate action goal.
        if(std::hypot(candidate.back().x-terminal.pose.position.x,candidate.back().y-terminal.pose.position.y)>.05 ||
           std::hypot(candidate.front().x-cur.x,candidate.front().y-cur.y)>.05) {
          failure="EGO terminal retry endpoint drift";candidate.clear();return false;
        }
        candidate.push_back({float(terminal.pose.position.x),float(terminal.pose.position.y),
                             float(tf2::getYaw(terminal.pose.orientation)),0});
        if(!safeTrajectory(candidate,0,pose,check_budget)) {
          failure="EGO terminal retry fails swept clearance/footprint verification";candidate.clear();return false;
        }
      }
      check_budget();return true;
    };
    bool solved=solve(ref);
    // A guide is optional. If optimization rejects it, restore the normal
    // reference before trying the existing bounded lateral fallback.
    // 【run50】不要因为一帧优化失败就丢弃整条绕行：代价地图同一帧里既有
    // 新补全的栅格又有旧的膨胀，参考被判一次"不可行"很常见；立刻清掉
    // detour_active_/detour_guide_ 会让方向锁存失效，下一帧退化成 ±0.4 m
    // 侧向种子并左右换边（实测先 +0.4 再 −0.4）。保留参考和方向，下一周期
    // 继续用；连续 3 次都被拒才真正放弃。
    if(!solved && guided) {
      const std::string guide_failure=failure;
      guided=false;++guide_fail_count_;
      RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
        "EGO detour reference rejected (%d/3): %s",guide_fail_count_,guide_failure.c_str());
      if(guide_fail_count_>=3) {
        detour_active_=false;detour_guide_.clear();
        RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
          "EGO detour reference dropped after 3 rejections");
      }
      planner_->setReturnPolicy(soft_margin_,route_weight_,length_weight_);
      planner_->setRouteReference(route_reference_);
      ref=original_ref;solved=solve(ref);
    }
    if(solved && guided)guide_fail_count_=0;
    if(!solved && terminal_blocked && !opportunistic) {
      // One exact-goal attempt only. Reduced inflation generates a candidate;
      // it never changes the body-center hard clearance or authorizes motion.
      check_budget();
      if(collision(terminal.pose.position.x,terminal.pose.position.y,
                   tf2::getYaw(terminal.pose.orientation),0))
        throw nav2_core::PlannerException("EGO terminal retry rejected: actual goal clearance/footprint blocked");
      terminal_retry=true;guided=false;detour_active_=false;
      planner_->setSearchInflation(clearance_);planner_->setGridMap(cur);planner_->setObstacles(obstacles);
      planner_->setReturnPolicy(soft_margin_,route_weight_,length_weight_);
      planner_->setRouteReference(route_reference_);
      ref=original_ref;
      ref.back().x=terminal.pose.position.x;ref.back().y=terminal.pose.position.y;
      solved=solve(ref);
      if(!solved)throw nav2_core::PlannerException(failure+"; exact-goal retry rejected");
      RCLCPP_INFO_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
        "EGO terminal search retry accepted after swept validation; action goal and hard clearance unchanged");
    }
    if(!solved) {
      // A fixed horizon can terminate inside an obstacle, or the straight
      // seed can trap rebound optimization even with a free endpoint.
      // Try bounded lateral seeds instead of repeating the identical failure.
      // Only substitute an intermediate target, never the actual action goal,
      // a blocked start, or an opportunistic return-to-route proposal.
      Pose goal=plan_.poses.back();goal.header=plan_.header;goal.header.stamp=rclcpp::Time(0);
      goal=transform(goal,mapFrame());
      const bool blocked_start=planner_->inflatedOccupied(cur.x,cur.y);
      const bool intermediate=std::hypot(ref.back().x-goal.pose.position.x,
                                         ref.back().y-goal.pose.position.y)>.5;
      if(opportunistic || blocked_start || !intermediate)
        throw nav2_core::PlannerException(failure);
      // Keep the original route for return scoring; only the local solver seed
      // moves sideways. The signed offsets are bounded and tried nearest first.
      const auto &a=route_reference_[route_reference_.size()-2],&b=route_reference_.back();
      const double dx=b.x-a.x,dy=b.y-a.y,norm=std::hypot(dx,dy);
      if(norm<1e-6)throw nav2_core::PlannerException(failure);
      const std::string original_failure=failure;
      // 【run50/run52】侧向种子按"已经偏向的那一侧"依次加大，而不是固定从
      // +0.4 起正负交替：0.5 m 箱子在航线正中时，车心至少要横移
      // 0.25+0.45+0.10 ≈ 0.80 m 才够得着；而且车若已经偏在一侧（绕行中），
      // 再从对面试种子会让它甩到障碍另一侧（实测 +0.34 → −0.36，画出一个
      // S 形，最后贴着箱子停住）。优先用锁存的绕行侧，其次用车相对航线的
      // 实际横向偏差符号；同一侧 0.4/0.8/1.2/1.6 试完再试另一侧。
      double side_pref=detour_active_?detour_side_:0;
      if(side_pref==0 && norm>1e-6) {
        const double lat=((cur.x-a.x)*-dy+(cur.y-a.y)*dx)/norm;
        if(std::abs(lat)>0.15)side_pref=lat>0?1:-1;
      }
      std::vector<double> offsets;
      if(side_pref!=0) {
        for(double mag:{.4,.8,1.2,1.6})offsets.push_back(side_pref*mag);
        for(double mag:{.4,.8,1.2,1.6})offsets.push_back(-side_pref*mag);
      } else {
        offsets={.4,-.4,.8,-.8,1.2,-1.2,1.6,-1.6};
      }
      // 【run55】种子不能把车往障碍方向拉：车已经绕到 +0.5 m 时再接受
      // 0.40 m 的种子，等于让车心向箱子靠近；实测这一帧被"验证通过"，
      // 车随后在箱子旁边贴到 0.28 m，下一帧所有样条都过不了净空校验，
      // patience 超时。已经偏离航线时，同侧种子不得小于当前横向偏移。
      double cur_lat=0;
      if(norm>1e-6) cur_lat=((cur.x-a.x)*-dy+(cur.y-a.y)*dx)/norm;
      for(double offset:offsets) {
        check_budget();
        if(std::abs(cur_lat)>.35 && offset*cur_lat>0 &&
           std::abs(offset)<std::abs(cur_lat)-.05)continue;
        const double x=ref.back().x-offset*dy/norm,y=ref.back().y+offset*dx/norm;
        if(planner_->inflatedOccupied(x,y))continue;
        auto seed=ref;
        for(size_t i=1;i<seed.size();++i) {
          const double f=double(i)/(seed.size()-1);
          seed[i].x-=f*offset*dy/norm;seed[i].y+=f*offset*dx/norm;
        }
        if(solve(std::move(seed))) {
          RCLCPP_INFO_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
            "EGO local optimization fallback: accepted verified lateral detour %.2f m; action goal unchanged",offset);
          break;
        }
      }
      // 【新增】终点落在膨胀区、且横向偏移全部失败时，缩短前视距离再解。
      // 横向偏移与航线垂直：障碍若横跨航线（一段墙/一排障碍正好压在
      // 前视终点上），±0.4~1.2 m 的六个横向种子仍全部落在障碍里，于是
      // 每个控制周期都报 "start/target inside inflated obstacle"，
      // Controller patience 一路超时（run17 转场第 7 条 swath 卡死即此）。
      // 把前视终点沿航线退到障碍之前，先规划一段合法轨迹把车带过去；
      // 真正的 action goal 仍由 terminal 检查把关，这里绝不放宽终点。
      if(candidate.size()<2 &&
         original_failure.find("inside inflated obstacle")!=std::string::npos) {
        size_t last_n=ref.size();
        for(double keep:{.75,.5,.3}) {
          check_budget();
          const size_t n=std::max<size_t>(size_t(4),
            static_cast<size_t>(std::lround(ref.size()*keep)));
          if(n>=last_n)continue;
          last_n=n;
          auto seed=ref;seed.resize(n);
          if(planner_->inflatedOccupied(seed.back().x,seed.back().y))continue;
          if(solve(std::move(seed))) {
            RCLCPP_INFO_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
              "EGO horizon shortened to %zu/%zu points: endpoint was inside inflated obstacle",
              n,ref.size());
            break;
          }
        }
      }
      if(candidate.size()<2)
        throw nav2_core::PlannerException(original_failure+"; no verified lateral target within 1.2 m");
    }
    if(opportunistic && !trajectory_.empty() &&
       returnScore(candidate,pose)+return_improvement_>=returnScore(trajectory_,pose)) return false;
    if(std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()>budget_)
      throw nav2_core::PlannerException("EGO planning/validation exceeded time budget; result discarded");
    trajectory_=std::move(candidate);
    trajectory_requires_sweep_=terminal_retry;
    if(guided) {
      if(!detour_active_)detour_end_=ref.back();
      detour_active_=true;
      RCLCPP_INFO_THROTTLE(node_->get_logger(),*node_->get_clock(),3000,
        "EGO smooth detour: verified guide, gradual return, %.2f m transition",detour_transition_);
    }
    correction_seed_.clear();
    tf2::fromMsg(cycle_transform_.transform,trajectory_transform_);trajectory_transform_valid_=true;
    last_plan_time_=now;
    nav_msgs::msg::Path path;path.header.frame_id=mapFrame();path.header.stamp=node_->now();
    for(auto &p:trajectory_) {
      Pose q;q.header=path.header;q.pose.position.x=p.x;q.pose.position.y=p.y;
      q.pose.orientation.z=std::sin(p.z/2);q.pose.orientation.w=std::cos(p.z/2);path.poses.push_back(q);
    }
    pub_->publish(path);
    return true;
  }
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    node_=parent.lock();if(!node_) throw nav2_core::PlannerException("Missing parent node");
    name_=name;tf_=tf;costmap_ros_=costmap;map_=costmap->getCostmap();
    max_v_=param("max_linear_vel",0.15);max_w_=param("max_angular_vel",0.2);
    accel_=param("max_linear_accel",0.2);angular_accel_=param("max_angular_accel",0.4);
    lookahead_=param("lookahead_dist",0.5);horizon_=param("planning_horizon",3.5);
    replan_period_=param("replan_period",0.5);budget_=param("planning_time_budget",0.25);
    max_pose_age_=param("max_pose_age",0.5);speed_limit_=max_v_;
    nav2_util::declare_parameter_if_not_declared(node_,name_+".preserve_reference_curve",rclcpp::ParameterValue(false));
    preserve_reference_curve_=node_->get_parameter(name_+".preserve_reference_curve").as_bool();
    nav2_util::declare_parameter_if_not_declared(node_,name_+".precise_goal_alignment",rclcpp::ParameterValue(false));
    precise_goal_alignment_=node_->get_parameter(name_+".precise_goal_alignment").as_bool();
    nav2_util::declare_parameter_if_not_declared(node_,name_+".start_handover_revision",rclcpp::ParameterValue(6));
    if(preserve_reference_curve_) {
      nav2_util::declare_parameter_if_not_declared(node_,name_+".continuous_tracking_revision",rclcpp::ParameterValue(7));
      if(node_->get_parameter(name_+".continuous_tracking_revision").as_int()!=7)
        throw nav2_core::PlannerException("Unsupported continuous tracking revision");
    }
    if(preserve_reference_curve_ && (max_v_>.8 || accel_>.2 || max_w_>.30 || angular_accel_>.4 || lookahead_>.12 || horizon_<1.2))
      throw nav2_core::PlannerException("Continuous coverage exceeds cruise/turn limits");
    heading_enter_=param("rotate_enter_angle",.5);heading_exit_=param("rotate_exit_angle",.25);
    frame_translation_threshold_=param("replan_frame_translation",.03);
    frame_rotation_threshold_=param("replan_frame_rotation",.017453292519943295);
    if(heading_exit_>=heading_enter_ || heading_enter_>=1.57079632679)
      throw nav2_core::PlannerException("Invalid EGO heading hysteresis");
    if(max_v_>0.8 || max_w_>0.43) throw nav2_core::PlannerException("EGO exceeds configured chassis speed limits");
    clearance_=param("obstacle_clearance",0.60);
    // 【改动】车体中心到驱动轴的偏移量改为可配（默认 0）：工控机实车里
    // base_link 在驱动轴前方 0.25 m，而 Gazebo 仿真里 base_link 就是驱动轴
    // 中心，偏移为 0。原来硬编码 +0.25 会让仿真每周期报
    // "base_link/drive axle installation mismatch" 并拒绝所有指令。
    nav2_util::declare_parameter_if_not_declared(node_,name_+".drive_axle_offset_x",rclcpp::ParameterValue(0.0));
    nav2_util::declare_parameter_if_not_declared(node_,name_+".drive_axle_offset_y",rclcpp::ParameterValue(0.0));
    axle_offset_x_=node_->get_parameter(name_+".drive_axle_offset_x").as_double();
    axle_offset_y_=node_->get_parameter(name_+".drive_axle_offset_y").as_double();
    // Prefer the enclosing axle circle; exact body geometry validates output.
    inflation_=clearance_+std::hypot(axle_offset_x_,axle_offset_y_)+map_->getResolution();
    planner_=std::make_unique<ego_planner::PlannerInterface>();planner_->initParam(max_v_,accel_,1.0);
    planner_->initEsdfMap(std::ceil(map_->getSizeInMetersX()+2),std::ceil(map_->getSizeInMetersY()+2),0,
      map_->getResolution(),Eigen::Vector3d::Zero(),inflation_);
    soft_margin_=param("optimization_clearance_margin",0.10);
    route_weight_=param("route_return_weight",0.4);length_weight_=param("route_length_weight",0.05);
    planner_->setReturnPolicy(soft_margin_,route_weight_,length_weight_);
    nav2_util::declare_parameter_if_not_declared(node_,name_+".smooth_detour_enabled",rclcpp::ParameterValue(false));
    smooth_detour_enabled_=node_->get_parameter(name_+".smooth_detour_enabled").as_bool();
    detour_transition_=param("detour_transition_length",1.1);
    detour_return_=param("detour_return_length",1.2);
    detour_offset_=param("detour_max_offset",1.8);
    detour_weight_=param("detour_reference_weight",1.5);
    return_forward_=param("return_min_forward_distance",.5);
    curve_preview_=param("curve_preview_dist",1.5);
    lateral_accel_=param("max_lateral_accel",.12);
    maneuver_speed_=param("maneuver_max_linear_vel",.25);
    slow_curvature_=param("maneuver_curvature_threshold",.15);
    slow_obstacle_distance_=param("obstacle_slow_distance",1.2);
    speed_reaction_=param("speed_reaction_time",.6);
    speed_margin_=param("speed_transition_margin",.15);
    if(max_v_>.4 && ((!preserve_reference_curve_ && !smooth_detour_enabled_) || curve_preview_<
       brakingDistance(max_v_,maneuver_speed_,accel_,speed_reaction_,speed_margin_)+lookahead_ ||
       horizon_<curve_preview_+.5 || slow_obstacle_distance_<=clearance_))
      throw nav2_core::PlannerException("EGO high-speed configuration lacks braking preview/obstacle margin");
    return_check_period_=param("return_check_period",0.5);
    return_deviation_=param("return_min_deviation",0.20);
    return_improvement_=param("return_min_improvement",0.05);
    max_path_age_=param("return_max_path_age",3.0);
    nav2_util::declare_parameter_if_not_declared(node_,name_+".return_confirmations",rclcpp::ParameterValue(2));
    required_return_confirmations_=node_->get_parameter(name_+".return_confirmations").as_int();
    if(required_return_confirmations_<2)throw nav2_core::PlannerException("EGO return needs at least two confirmations");
    pub_=node_->create_publisher<nav_msgs::msg::Path>(name_+"/ego_local_plan",1);
  }
  void activate() override {active_=true;pub_->on_activate();}
  void deactivate() override {active_=false;trajectory_.clear();correction_seed_.clear();pub_->on_deactivate();}
  void cleanup() override {trajectory_.clear();correction_seed_.clear();planner_.reset();pub_.reset();}
  void setPlan(const nav_msgs::msg::Path &path) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if(path.poses.empty()||path.header.frame_id.empty()) throw nav2_core::PlannerException("Empty/unframed EGO path");
    for(const auto &p:path.poses) if(!std::isfinite(p.pose.position.x)||!std::isfinite(p.pose.position.y))
      throw nav2_core::PlannerException("Nonfinite reference path");
    if(preserve_reference_curve_)for(size_t i=1;i<path.poses.size();++i) {
      const double ds=distance(path.poses[i-1],path.poses[i]);
      const double da=std::abs(wrap(tf2::getYaw(path.poses[i].pose.orientation)-tf2::getYaw(path.poses[i-1].pose.orientation)));
      if(ds<1e-5 || ds>.025 || !std::isfinite(da) || da>2.001*ds)
        throw nav2_core::PlannerException("Invalid continuous coverage sampling/curvature");
    }
    continuous_started_=false;
    turn_response_.reset();
    yaw_servo_.reset();
    precise_approach_.reset();
    plan_=path;cursor_=0;trajectory_.clear();trajectory_requires_sweep_=false;aligning_goal_=false;rotating_heading_=false;rotate_sign_=0;rotate_target_yaw_=0;replan_hold_until_=-1.0;
    detour_active_=false;detour_guide_.clear();
    correction_seed_.clear();
    trajectory_transform_valid_=false;
    last_plan_time_=-1;last_attempt_time_=-1;last_command_time_=-1;
    last_return_check_=-1;return_confirmations_=0;route_reference_.clear();return_available_=false;
  }
  void setSpeedLimit(const double &limit,const bool &percentage) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!std::isfinite(limit)||limit<0) throw nav2_core::PlannerException("Invalid speed limit");
    speed_limit_=(limit==0)?max_v_:std::min(max_v_,percentage?max_v_*limit/100.0:limit);
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const Pose &input,const geometry_msgs::msg::Twist &velocity,
    nav2_core::GoalChecker *goal_checker) override {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto cycle_start=std::chrono::steady_clock::now();
    const double previous_time=last_command_time_;
    // A failed cycle invalidates the command ramp; resume from measured motion.
    last_command_time_=-1;
    if(!active_||plan_.poses.empty()||!map_->getSizeInCellsX()||!costmap_ros_->isCurrent())
      throw nav2_core::PlannerException("EGO inactive, no path, or stale costmap");
    double now=node_->now().seconds(),age=now-rclcpp::Time(input.header.stamp).seconds();
    if(age<-.1||age>max_pose_age_||!std::isfinite(velocity.linear.x)||!std::isfinite(velocity.angular.z))
      throw nav2_core::PlannerException("Stale pose or invalid measured velocity");
    try {
      const auto center=tf_->lookupTransform(costmap_ros_->getBaseFrameID(),"base_link",tf2::TimePointZero);
      center_x_=center.transform.translation.x;center_y_=center.transform.translation.y;
      if(std::abs(center_x_-axle_offset_x_)>1e-4 || std::abs(center_y_-axle_offset_y_)>1e-4)
        throw nav2_core::PlannerException("EGO base_link/drive axle installation mismatch");
    } catch(const tf2::TransformException &e) {throw nav2_core::PlannerException(e.what());}
    try {
      cycle_transform_=tf_->lookupTransform(mapFrame(),plan_.header.frame_id,tf2::TimePointZero);
    } catch(const tf2::TransformException &e) {throw nav2_core::PlannerException(e.what());}
    if(rclcpp::Time(cycle_transform_.header.stamp).nanoseconds()!=0) {
      const double frame_age=now-rclcpp::Time(cycle_transform_.header.stamp).seconds();
      // 【改动】未来方向容差从 -0.1 放宽到 -max_pose_age（±0.5 s 对称）：
      // 本项目的 3D 重定位像 AMCL 一样把 map->odom 时间戳打到 now + 0.2 s
      // （relocalization_params.yaml 的 tf_future_stamp），原 -0.1 s 会把这条
      // 有意超前的 TF 判成 stale，EGO 每个控制周期都拒绝指令、车完全不动。
      // 放宽后仍能拒绝真正冻结/过期的 map->odom（超过 0.5 s）。
      if(frame_age<-max_pose_age_ || frame_age>max_pose_age_)
        throw nav2_core::PlannerException("Stale EGO reference-frame transform");
    }
    Pose pose=transform(input,mapFrame());
    if(trajectory_transform_valid_ && !trajectory_.empty()) {
      tf2::Transform current;tf2::fromMsg(cycle_transform_.transform,current);
      const auto delta=current*trajectory_transform_.inverse();
      // 【修复】原实现用 (delta*point - point).length() 衡量"机器人位置的
      // 位移"，但该量会把绕 odom 原点的微小旋转按 |point| 放大：本项目地图
      // 约 20x19 m，机器人在 (-8.7,-7.7) 时离原点 11.6 m，FAST-LIO 重定位
      // 正常的 0.7° 角抖动就会变成 11.6*0.012 ≈ 0.14 m 位移，叠加几厘米
      // 平移抖动后越过 0.15 m 阈值，EGO 每 8 s 就清一次轨迹（实测 run12
      // 87 s 内 11 次），表现为一顿一顿的乱晃。改为分别检验 delta 的
      // 平移量和旋转量：只吸收真正的定位跳变（平移 > 0.15 m 或旋转 > 5°），
      // 不再被距离放大。continuous 的漂移由 replan_period 0.5 s 的周期重规划处理。
      // 只取 x/y：本机器人是 2D 导航，FAST-LIO 3D 重定位会让 map->odom 的
      // z 分量在 ±0.1 m 内漂移（实测 0.064~0.188），把 z 计入平移量会
      // 让 0.15 m 阈值虚警。z 不影响平面轨迹，忽略。
      const double delta_translation=std::hypot(delta.getOrigin().x(),delta.getOrigin().y());
      const double delta_rotation=delta.getRotation().getAngleShortestPath();
      if(delta_translation>frame_translation_threshold_ ||
         delta_rotation>frame_rotation_threshold_) {
        // Local obstacles and the old curve are both in odom: a small global
        // correction must not erase the chosen detour side. Keep only a solver
        // seed, never a command fallback; a new validated plan is mandatory.
        double deviation=0;
        if(route_reference_.size()>1)for(const auto &p:trajectory_)
          deviation=std::max(deviation,routeDistance(p.x,p.y));
        correction_seed_.clear();
        if(deviation>=return_deviation_ && delta_translation<=.20 &&
           delta_rotation<=.10)
          correction_seed_=trajectory_;
        trajectory_.clear();route_reference_.clear();trajectory_transform_valid_=false;
        if(correction_seed_.empty()){detour_active_=false;detour_guide_.clear();}
        return_available_=false;return_confirmations_=0;last_attempt_time_=-1;last_return_check_=-1;
        aligning_goal_=false;
        RCLCPP_INFO_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
          "EGO map correction changed: replan before motion (detour seed points: %zu)",correction_seed_.size());
      }
    }
    Pose goal=plan_.poses.back();goal.header=plan_.header;goal.header.stamp=rclcpp::Time(0);goal=transform(goal,mapFrame());
    double xy_tol=0.08,yaw_tol=0.08;
    if(goal_checker) {geometry_msgs::msg::Pose p;geometry_msgs::msg::Twist v;
      if(goal_checker->getTolerances(p,v)) {xy_tol=std::max(0.01,p.position.x);yaw_tol=std::abs(tf2::getYaw(p.orientation));}}
    double v=0,w=0,yaw=tf2::getYaw(pose.pose.orientation);bool reached=false,slow_now=false,continuous_turning=false;
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*map_->getMutex());
    if(collision(pose.pose.position.x,pose.pose.position.y,yaw))
      throw nav2_core::PlannerException("Current pose violates body-center clearance/footprint");
    if(max_v_>.4)checkBrakingArc(pose,velocity.linear.x,velocity.angular.z);
    if(preserve_reference_curve_) {
      if(!continuous_started_ && (std::abs(velocity.linear.x)>=.04 || std::abs(velocity.angular.z)>=.03))
        throw nav2_core::PlannerException("Continuous coverage handover requires measured stop below 0.04 m/s / 0.03 rad/s");
      trajectory_=curveReference(pose);
    }
    const bool final_section=!preserve_reference_curve_ ||
      (cursor_+1>=plan_.poses.size() || (distance(plan_.poses[cursor_],plan_.poses.back())<.20 && cursor_+50>=plan_.poses.size()));
    const double goal_distance=final_section?distance(pose,goal):1e6;
    const bool precise=precise_goal_alignment_ && !preserve_reference_curve_ && xy_tol<=.08 &&
      (precise_approach_.active() || goal_distance<=.40);
    // Enter final rotation inside the position tolerance, with room for pose
    // noise. Keep that phase stable while rotating; goal_checker still decides
    // success using its original position and heading tolerances.
    if(goal_distance<=xy_tol*.5) aligning_goal_=true;
    if(goal_distance>xy_tol+.10) aligning_goal_=false;
    if(aligning_goal_ && !precise) {
      double error=wrap(tf2::getYaw(goal.pose.orientation)-yaw);
      if(std::abs(error)>yaw_tol) w=std::clamp(1.5*error,-max_w_,max_w_);
      else if(goal_distance<=xy_tol) reached=true;
      else aligning_goal_=false;
    }
    auto pursue=[&](double dx,double dy,double chord_heading) {
      double error=wrap(std::atan2(dy,dx)-yaw);
      // 【修复 U 形换轨处"原地左右旋转"】朝 lookahead 点的方位角接近
      // ±π 时，wrap() 在 ±π 处翻转符号：误差在 -3.14 与 +3.14 之间跳，
      // w 每周期在 -max_w/+max_w 之间来回，车原地左右晃 185 s 才转出
      // 掉头弧（run15 第 3→4 条 swath）。改用"最近点到 lookahead 点的
      // 弦方向"作转向参考：弦方向沿弧连续、没有 ±π 歧义，转动方向稳定。
      if(std::abs(error)>2.6) {
        const double chord=wrap(chord_heading-yaw);
        if(std::abs(chord)>1e-3) error=chord;
      }
      if(std::abs(error)>=heading_enter_) {
        if(!rotating_heading_) {
          rotate_sign_=error>0?+1:-1;
          // 【修复"原地转圈停不下来"】只锁转向还不够（run19 第 11 条
          // swath）：EGO 每 0.5 s 用当前位姿重新规划，起点切线跟着车头
          // 走，lookahead 点绕车滑动，航向误差始终贴着 ±π、永远掉不到
          // rotate_exit_angle 以下，车以 0.43 rad/s 连转了 180 s。
          // 进入旋转时再锁存一个**绝对**目标朝向，旋转期间只用
          // wrap(锁存朝向 - yaw) 计算误差；配合下面"旋转期间不重规划"，
          // 目标在 map 系里固定不动，误差必然单调收敛。
          rotate_target_yaw_=wrap(yaw+error);
        }
        rotating_heading_=true;
      }
      if(rotating_heading_) {
        // 【修复"原地左右旋转"】掉头/换轨时目标常在正后方，航向误差贴着
        // ±π：每周期重新取误差符号，误差在 +3.14/-3.14 之间跳一次，w 就
        // 反一次，车原地左右晃，永远转不过去（run18 第 3→4 条 swath：
        // hdg_err +2.8 → -3.1 → -2.8 → +2.7 循环，v=0 持续数十秒）。
        // 转向与目标朝向都锁存后，越过 ±π 时 wrap 出的误差会单调回升到 0，
        // 车能连续转过去；退出条件仍是 |误差| <= rotate_exit_angle。
        const double locked=wrap(rotate_target_yaw_-yaw);
        if(std::abs(locked)<=heading_exit_) {
          rotating_heading_=false;rotate_sign_=0;
          // 【修复 U 型掉头处"左右来回转"（run20 第 4 条 swath）】转完后
          // 如果下一周期立刻重规划，EGO 用旋转后的新车头重解 B 样条，前
          // 0.7 m 的切线可能又指回反方向，车被要求再转回去，于是
          // w=+0.43 / -0.43 交替、v=0，连转 190 s 直到 FollowPath 超时。
          // 转完先沿缓存轨迹稳稳前进 1 s 再允许重规划：此时起点已经离开
          // 掉头顶点、有了前向速度，重解出来的样条方向是连续的。
          replan_hold_until_=now+1.0;
        } else {
          w=std::clamp(rotate_sign_*1.5*std::abs(locked),-max_w_,max_w_);
        }
      }
      if(!rotating_heading_) {
        double curvature=2*(-std::sin(yaw)*dx+std::cos(yaw)*dy)/std::max(0.01,dx*dx+dy*dy);
        double blend=std::clamp((heading_enter_-std::abs(error))/(heading_enter_-heading_exit_),0.0,1.0);
        blend=blend*blend*(3-2*blend);
        v=std::min({speed_limit_,max_w_/std::max(1e-6,std::abs(curvature)),
          std::sqrt(2*accel_*std::max(0.0,goal_distance-xy_tol*.25))})*blend;
        w=blend*(curvature*v)+(1-blend)*std::clamp(1.5*error,-max_w_,max_w_);
      }
    };
    // 【修复条带末端"绕着终点转圈"（run23 第 13 条 swath）】最后 0.7~1.7 m
    // 是 B 样条的退化区：剩余航线比样条地平线还短，起点切向约束和参考点
    // 都几乎消失，优化结果在"朝终点/背终点"之间翻，车在终点附近来回转、
    // goal_d 在 0.74 与 1.7 之间振荡 187 s。把"直线接近终点"的半径放宽到
    // 1.2 m：这一段本来就该直线走完，且下面逐点做 footprint 碰撞检查，
    // 直线被挡时自动回退到样条。
    bool direct_goal=goal_distance<=std::max(lookahead_,1.2);
    if(direct_goal) {
      const double dx=goal.pose.position.x-pose.pose.position.x;
      const double dy=goal.pose.position.y-pose.pose.position.y;
      const double heading=std::atan2(dy,dx);
      const int steps=std::max(1,int(std::ceil(goal_distance/(map_->getResolution()*.5))));
      for(int i=0;i<=steps;++i)if(collision(pose.pose.position.x+dx*i/steps,
          pose.pose.position.y+dy*i/steps,heading)){direct_goal=false;break;}
    }
    if(precise) {
      aligning_goal_=false;rotating_heading_=false;rotate_sign_=0;trajectory_.clear();
      try {
        const auto previous_phase=precise_approach_.phaseName();
        const auto cmd=precise_approach_.step(now,pose.pose.position.x,pose.pose.position.y,yaw,
          goal.pose.position.x,goal.pose.position.y,tf2::getYaw(goal.pose.orientation),
          velocity.linear.x,velocity.angular.z,xy_tol,yaw_tol);
        v=cmd.v;w=cmd.w;
        if(std::string(previous_phase)!=precise_approach_.phaseName())
          RCLCPP_INFO(node_->get_logger(),
            "Precise start: %s -> %s; xy=%.3f m; yaw=%.3f rad; translations=%d",
            previous_phase,precise_approach_.phaseName(),goal_distance,
            wrap(tf2::getYaw(goal.pose.orientation)-yaw),precise_approach_.corrections());
      } catch(const std::exception &e) {throw nav2_core::PlannerException(e.what());}
    } else if(!aligning_goal_ && !reached && direct_goal && !preserve_reference_curve_) {
      // A nearby goal uses a fully collision-checked straight approach followed
      // by in-place alignment, avoiding tiny lattice/spline loops at the endpoint.
      trajectory_.clear();
      pursue(goal.pose.position.x-pose.pose.position.x,goal.pose.position.y-pose.pose.position.y,
             tf2::getYaw(goal.pose.orientation));
      if(max_v_>.4) {
        slow_now=true;
        if(v>maneuver_speed_){w*=maneuver_speed_/v;v=maneuver_speed_;}
      }
    } else if(!aligning_goal_ && !reached && preserve_reference_curve_) {
      auto deadline=[&]() {
        if(std::chrono::duration<double>(std::chrono::steady_clock::now()-cycle_start).count()>budget_)
          throw nav2_core::PlannerException("Continuous coverage validation exceeded time budget");
      };
      if(!safeTrajectory(trajectory_,0,pose,deadline))
        throw nav2_core::PlannerException("Continuous coverage swept corridor blocked; no route skipping");
      const auto &anchor=trajectory_.front();
      const double heading_error=wrap(anchor.z-yaw);
      const double lateral_error=-(pose.pose.position.x-anchor.x)*std::sin(anchor.z)+
                                  (pose.pose.position.y-anchor.y)*std::cos(anchor.z);
      v=std::min(speed_limit_,std::sqrt(2*accel_*std::max(0.0,goal_distance-xy_tol*.5)));
      const double route_cap=continuousSpeedCap(trajectory_,speed_limit_,accel_,speed_reaction_,speed_margin_,
        [&](const Point &p){return closeObstacle(p.x,p.y,p.z);});
      const double tracking_cap=continuousTrackingSpeedCap(speed_limit_,lateral_error,heading_error,accel_,speed_reaction_);
      const double curvature=continuousCurvature(trajectory_);
      continuous_turning=std::abs(curvature)>.01;
      const double turn_cap=turn_response_.step(now,curvature,lateral_error,heading_error,
        velocity.angular.z,previous_time>=0?last_angular_command_:velocity.angular.z,
        std::min(route_cap,continuousCurveLimit(curvature,0)));
      v=std::min({v,route_cap,tracking_cap,turn_cap});
      RCLCPP_INFO_THROTTLE(node_->get_logger(),*node_->get_clock(),2000,
        "Continuous tracking: lateral=%.3f m heading=%.3f rad route_cap=%.3f tracking_cap=%.3f turn_cap=%.3f m/s measured_w=%.3f rad/s",
        lateral_error,heading_error,route_cap,tracking_cap,std::min(speed_limit_,turn_cap),velocity.angular.z);
      // The endpoint must enter the brake preview well before the last 20 cm.
      if(cursor_+trajectory_.size()>=plan_.poses.size()) {
        double remaining=0;
        for(size_t i=1;i<trajectory_.size();++i)
          remaining+=std::hypot(trajectory_[i].x-trajectory_[i-1].x,trajectory_[i].y-trajectory_[i-1].y);
        if(final_section)remaining=goal_distance;
        v=std::min(v,approachSpeed(0,std::max(0.0,remaining-xy_tol*.25),accel_,speed_reaction_,0));
      }
      // Use this cycle's reachable speed for curvature feedforward during ramps.
      const bool ramp=previous_time>=0 && now>=previous_time && now-previous_time<=.5;
      const double step=ramp?std::clamp(now-previous_time,.001,.2):.1;
      const double previous=ramp?last_linear_command_:velocity.linear.x;
      const double ramp_accel=(continuous_turning || v<=.080001)?std::min(accel_,.05):accel_;
      const double reachable=std::clamp(v,std::max(0.0,previous-accel_*step),std::max(0.0,previous+ramp_accel*step));
      w=yaw_servo_.step(now,reachable,curvature,lateral_error,heading_error,velocity.angular.z);
      rotating_heading_=false;rotate_sign_=0;
      nav_msgs::msg::Path display;display.header.frame_id=mapFrame();display.header.stamp=node_->now();
      for(const auto &p:trajectory_) {Pose q;q.header=display.header;q.pose.position.x=p.x;q.pose.position.y=p.y;
        q.pose.orientation.z=std::sin(p.z/2);q.pose.orientation.w=std::cos(p.z/2);display.poses.push_back(q);}
      pub_->publish(display);
    } else if(!aligning_goal_ && !reached) {
      if(detour_active_ && std::hypot(detour_end_.x-pose.pose.position.x,
                                     detour_end_.y-pose.pose.position.y)<.5)
        {detour_active_=false;detour_guide_.clear();}
      // New obstacles invalidate the old curve before pursuit. With the new
      // policy, attempt a fresh detour in this cycle instead of first failing
      // the command and waiting for the following controller cycle.
      if(smooth_detour_enabled_ && !trajectory_.empty()) {
        size_t near=0;double best=1e9;
        for(size_t i=0;i<trajectory_.size();++i) {
          double d=std::hypot(trajectory_[i].x-pose.pose.position.x,trajectory_[i].y-pose.pose.position.y);
          if(d<best){best=d;near=i;}
        }
        for(size_t i=near;i<trajectory_.size();++i)if(collision(trajectory_[i].x,trajectory_[i].y,trajectory_[i].z)) {
          // 【车尾摆尾修复】只丢掉这条缓存轨迹，保留 detour_active_/
          // detour_guide_：下一步 replan 会按最新地图重新生成参考，生成
          // 不出来（车已太靠近障碍、坡度段放不下）时还能沿用已验证的参考
          // 保持横向偏移直到车尾越过障碍。原先在这里一并清掉 detour 状态，
          // 参考在障碍旁一整段消失，机器人直接切回原航线，车尾扫进障碍。
          trajectory_.clear();break;
        }
      }
      // 【修复"原地转圈停不下来"】原地旋转期间冻结重规划：EGO 每 0.5 s 以
      // 当前位姿为起点重新生成轨迹，起点切线跟着车头转，lookahead 点绕车
      // 滑动，航向误差永远贴着 ±π（run19 第 11 条 swath 连转 180 s）。
      // 旋转时 v=0、只有 w，缓存轨迹不会被走偏；转完立刻恢复重规划
      // （last_attempt_time_ 未更新，下一周期必然触发）。
      if((!rotating_heading_||trajectory_.empty()) && now>=replan_hold_until_ &&
         (trajectory_.empty()||now<last_attempt_time_ ||
         (now-last_attempt_time_>=replan_period_ &&
          (detour_active_ ||
           std::hypot(trajectory_.back().x-pose.pose.position.x,
                      trajectory_.back().y-pose.pose.position.y)<(smooth_detour_enabled_?horizon_*.55:1.0))))) {
        last_attempt_time_=now;
        try {replan(pose,velocity,now);}
        catch(const nav2_core::PlannerException &e) {
          // This is a spatial path, not a scheduled trajectory. Its validity
          // comes from the fresh map/pose and full collision checks every cycle,
          // not its creation time. Never continue beyond the cached endpoint.
          if(trajectory_.empty()||now<last_plan_time_ ||
             std::hypot(trajectory_.back().x-pose.pose.position.x,
                        trajectory_.back().y-pose.pose.position.y)<.15 &&
              std::hypot(trajectory_.back().x-goal.pose.position.x,
                         trajectory_.back().y-goal.pose.position.y)>xy_tol) {trajectory_.clear();throw;}
          RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
            "EGO replan rejected (%s); revalidating cached spatial path",e.what());
        }
      }
      if(now<last_return_check_){last_return_check_=-1;return_confirmations_=0;}
      if(!rotating_heading_ && now>=replan_hold_until_ && !detour_active_ && !trajectory_.empty() && now-last_return_check_>=return_check_period_ &&
         now-last_attempt_time_>=replan_period_) {
        last_return_check_=now;
        try {
          reference(pose,true);
          const bool worth_checking=routeDistance(pose.pose.position.x,pose.pose.position.y)>=return_deviation_ ||
                                   now-last_plan_time_>=max_path_age_;
          if(worth_checking && return_available_) {
            const bool same=std::hypot(return_point_.x-last_return_point_.x,return_point_.y-last_return_point_.y)<=0.5;
            return_confirmations_=same?return_confirmations_+1:1;
            last_return_point_=return_point_;
          } else return_confirmations_=0;
          if(return_confirmations_>=required_return_confirmations_) {
            last_attempt_time_=now;return_confirmations_=0;
            if(replan(pose,velocity,now,true))
              RCLCPP_DEBUG(node_->get_logger(),"EGO accepted shorter return to original route");
          }
        } catch(const nav2_core::PlannerException &e) {
          return_confirmations_=0;
          RCLCPP_DEBUG(node_->get_logger(),"EGO return proposal rejected: %s",e.what());
        }
      }
      size_t nearest=0;double best=1e9;
      for(size_t i=0;i<trajectory_.size();++i) {
        double d=std::hypot(trajectory_[i].x-pose.pose.position.x,trajectory_[i].y-pose.pose.position.y);
        if(d<best) {best=d;nearest=i;}
      }
      // Reject any newly obstructed remaining trajectory before tracking it.
      for(size_t i=nearest;i<trajectory_.size();++i)
        if(collision(trajectory_[i].x,trajectory_[i].y,trajectory_[i].z)) {
          trajectory_.clear();throw nav2_core::PlannerException("EGO trajectory footprint blocked; replan required");
        }
      if(trajectory_requires_sweep_) {
        auto deadline=[&]() {
          if(std::chrono::duration<double>(std::chrono::steady_clock::now()-cycle_start).count()>budget_)
            throw nav2_core::PlannerException("EGO terminal trajectory validation exceeded time budget");
        };
        if(!safeTrajectory(trajectory_,nearest,pose,deadline)) {
          trajectory_.clear();throw nav2_core::PlannerException("EGO terminal trajectory swept corridor blocked");
        }
      }
      size_t target=nearest;double length=0;
      while(target+1<trajectory_.size() && length<lookahead_) {
        length+=std::hypot(trajectory_[target+1].x-trajectory_[target].x,trajectory_[target+1].y-trajectory_[target].y);++target;
      }
      double dx=trajectory_[target].x-pose.pose.position.x,dy=trajectory_[target].y-pose.pose.position.y;
      // 弦方向 = 最近点 -> lookahead 点；仅在朝 lookahead 的方位角接近
      // ±π 时作为稳定的转向参考（见 pursue 内注释）。
      const auto &p_near=trajectory_[nearest];
      double chord=target>nearest?
        std::atan2(trajectory_[target].y-p_near.y,trajectory_[target].x-p_near.x):
        double(p_near.z);
      // 【兜底：样条前视点塌缩】缓存样条快走完（或重解出极短样条）时，
      // lookahead 只剩几厘米，pursue 会把噪声方向当成目标：车先停下、
      // 再原地转半圈，然后重规划又塌缩，来回转直到 FollowPath 超时
      // （run24 第 20 条 swath 185 s、run26 第 11 条 swath 同样特征）。
      // 覆盖任务的目标本来就是沿航线直行：此时改用车头在**覆盖航线**
      // （route_reference_，与样条无关、始终是 4.5 m 前向链）上的投影
      // 前视点。绕行（detour_active_）时不启用，避免把车拉回障碍物上。
      if(std::hypot(dx,dy)<0.30*lookahead_ && goal_distance>2.0 && !detour_active_ &&
         route_reference_.size()>1) {
        size_t rn=0;double rb=1e9;
        for(size_t i=0;i<route_reference_.size();++i) {
          const double d=std::hypot(route_reference_[i].x-pose.pose.position.x,
                                    route_reference_[i].y-pose.pose.position.y);
          if(d<rb){rb=d;rn=i;}
        }
        double arc=0;size_t rt=rn;
        while(rt+1<route_reference_.size() && arc<lookahead_) {
          arc+=std::hypot(route_reference_[rt+1].x-route_reference_[rt].x,
                          route_reference_[rt+1].y-route_reference_[rt].y);++rt;
        }
        const double rdx=route_reference_[rt].x-pose.pose.position.x;
        const double rdy=route_reference_[rt].y-pose.pose.position.y;
        if(rt>rn && std::hypot(rdx,rdy)>std::max(std::hypot(dx,dy),0.3)) {
          chord=std::atan2(route_reference_[rt].y-route_reference_[rn].y,
                           route_reference_[rt].x-route_reference_[rn].x);
          dx=rdx;dy=rdy;
          RCLCPP_INFO_THROTTLE(node_->get_logger(),*node_->get_clock(),500,
            "EGO route lookahead fallback: spline look=%.2f m -> route look=%.2f m",
            std::hypot(trajectory_[target].x-pose.pose.position.x,
                       trajectory_[target].y-pose.pose.position.y),std::hypot(rdx,rdy));
        }
      }
      pursue(dx,dy,chord);
      RCLCPP_INFO_THROTTLE(node_->get_logger(),*node_->get_clock(),250,
        "EGO track: goal_d=%.2f look=(%.2f,%.2f) hdg_err=%.2f chord_err=%.2f lat=%.2f v=%.2f w=%.2f rot=%d",
        goal_distance,dx,dy,wrap(std::atan2(dy,dx)-yaw),wrap(chord-yaw),
        -(pose.pose.position.x-p_near.x)*std::sin(p_near.z)+
         (pose.pose.position.y-p_near.y)*std::cos(p_near.z),
        v,w,(int)rotating_heading_);
      if(smooth_detour_enabled_ && v>0) {
        double curve_cap=curvePreviewSpeed(trajectory_,nearest,curve_preview_,speed_limit_,max_w_,lateral_accel_,accel_);
        double cap=curve_cap,env_cap=curve_cap,rem_cap=curve_cap,remaining=0;
        if(max_v_>.4) {
          const auto envelope=previewEnvelope(trajectory_,nearest,curve_preview_,lookahead_,speed_limit_,maneuver_speed_,
            max_w_,lateral_accel_,slow_curvature_,accel_,speed_reaction_,speed_margin_,[&](const Point &p){
              return closeObstacle(p.x,p.y,p.z) || routeDistance(p.x,p.y)>.08;
            });
          env_cap=envelope.cap;
          cap=std::min(cap,envelope.cap);slow_now=envelope.slow_now;
          for(size_t i=nearest+1;i<trajectory_.size();++i)
            remaining+=std::hypot(trajectory_[i].x-trajectory_[i-1].x,trajectory_[i].y-trajectory_[i-1].y);
          rem_cap=approachSpeed(0,remaining,accel_,speed_reaction_,speed_margin_);
          cap=std::min(cap,rem_cap);
        }
        // 【run59 诊断】run58 回轨段出现 0.02~0.05 m/s 的蠕行，而 EGO track
        // 打印的是限速前的 v：必须把限速来源区分开。curve / envelope / rem
        // 三者哪一个给出最小值，以及在曲线项里是哪一段的曲率把速度压死。
        if(cap<0.15) {
          size_t ki=nearest;double kk=0,kab=0,kbc=0,ktravel=0,travel=0;
          for(size_t i=std::max(size_t(1),nearest);i+1<trajectory_.size();++i) {
            const double ab=std::hypot(trajectory_[i].x-trajectory_[i-1].x,trajectory_[i].y-trajectory_[i-1].y);
            travel+=ab;if(travel>curve_preview_)break;
            const double bc=std::hypot(trajectory_[i+1].x-trajectory_[i].x,trajectory_[i+1].y-trajectory_[i].y);
            const double ac=std::hypot(trajectory_[i+1].x-trajectory_[i-1].x,trajectory_[i+1].y-trajectory_[i-1].y);
            if(ab*bc*ac<1e-9)continue;
            const double k=2*std::abs((trajectory_[i].x-trajectory_[i-1].x)*(trajectory_[i+1].y-trajectory_[i].y)
                                     -(trajectory_[i].y-trajectory_[i-1].y)*(trajectory_[i+1].x-trajectory_[i].x))/(ab*bc*ac);
            if(k>kk){kk=k;ki=i;kab=ab;kbc=bc;ktravel=travel;}
          }
          RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),500,
            "EGO speed cap %.3f: curve=%.3f envelope=%.3f rem=%.3f slow=%d "
            "(n=%zu nearest=%zu remain=%.2f) worst k=%.1f at i=%zu ab=%.3f bc=%.3f travel=%.2f "
            "close=%d route=%.2f",
            cap,curve_cap,env_cap,rem_cap,(int)slow_now,
            trajectory_.size(),nearest,remaining,kk,ki,kab,kbc,ktravel,
            (int)closeObstacle(trajectory_[ki].x,trajectory_[ki].y,trajectory_[ki].z),
            routeDistance(trajectory_[ki].x,trajectory_[ki].y));
        }
        if(v>cap){w*=cap/v;v=cap;}
      }
    }
    const bool continuous=previous_time>=0 && now>=previous_time && now-previous_time<=0.5;
    double dt=continuous?std::clamp(now-previous_time,0.001,0.2):0.1;
    // Slew successive commands. Re-seeding every cycle from zero measured speed
    // traps commands below the chassis breakaway speed and prevents startup.
    double previous_v=continuous?last_linear_command_:velocity.linear.x;
    double previous_w=continuous?last_angular_command_:velocity.angular.z;
    if(max_v_>.4) {
      // New obstacles or new curves may appear inside the planned braking
      // distance. Do not introduce a turn at cruising speed: brake first;
      // measured-speed corridor and final command arc still must be safe.
      slow_now=slow_now || closeObstacle(pose.pose.position.x,pose.pose.position.y,yaw) ||
               (v>1e-4 && std::abs(w)/v>=slow_curvature_) || rotating_heading_ || aligning_goal_;
      if(slow_now) {
        if(v>maneuver_speed_){w*=maneuver_speed_/v;v=maneuver_speed_;}
        if(std::max(std::abs(velocity.linear.x),std::abs(previous_v))>maneuver_speed_+.02) {
          v=std::min(v,maneuver_speed_);w=0;
        }
      }
      // Final heading changes require a measured stop, not only a zero target.
      if((rotating_heading_ || aligning_goal_) && std::abs(velocity.linear.x)>.04){v=0;w=0;}
    }
    if(precise_goal_alignment_ && !precise && v>0) {
      const double cap=approachSpeed(.06,std::max(0.0,goal_distance-.40),accel_,speed_reaction_,speed_margin_);
      if(v>cap){w*=cap/v;v=cap;}
    }
    const double command_accel=(precise || (preserve_reference_curve_ && (continuous_turning || v<=.080001)))?std::min(accel_,.05):accel_;
    // Precision creep limits acceleration only. Retain the configured braking
    // rate when entering from 0.30 m/s: 0.05 m/s^2 would need 0.90 m to stop.
    v=std::clamp(v,std::max(0.0,previous_v-accel_*dt),std::max(0.0,previous_v+command_accel*dt));
    v=std::clamp(v,0.0,speed_limit_);
    w=std::clamp(w,previous_w-angular_accel_*dt,previous_w+angular_accel_*dt);w=std::clamp(w,-max_w_,max_w_);
    if(reached){v=0;w=0;}
    try {checkCommand(pose,v,w);}
    catch(const nav2_core::PlannerException &) {
      // A collision-free spline can still demand an unsafe pursuit arc.
      // Do not retry the same cached curve indefinitely while Nav2 sends zero.
      trajectory_.clear();trajectory_transform_valid_=false;
      throw;
    }
    if((trajectory_requires_sweep_ || preserve_reference_curve_) && std::chrono::duration<double>(std::chrono::steady_clock::now()-cycle_start).count()>budget_)
      throw nav2_core::PlannerException("EGO terminal retry control cycle exceeded time budget; command discarded");
    geometry_msgs::msg::TwistStamped result;result.header.frame_id=costmap_ros_->getBaseFrameID();result.header.stamp=node_->now();
    result.twist.linear.x=v;result.twist.angular.z=w;
    if(preserve_reference_curve_)continuous_started_=true;
    last_linear_command_=v;last_angular_command_=w;last_command_time_=now;return result;
  }
};
}
PLUGINLIB_EXPORT_CLASS(ego_nav2_controller::EgoController,nav2_core::Controller)
