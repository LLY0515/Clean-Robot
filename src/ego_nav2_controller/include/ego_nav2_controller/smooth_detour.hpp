#pragma once
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace ego_nav2_controller {
// C2 lateral transition: zero slope and curvature at both joins. This is only
// a solver reference; the optimized footprint and commanded arc remain guarded.
inline double smoothStep5(double u) {
  u=std::clamp(u,0.0,1.0);
  return u*u*u*(10+u*(-15+6*u));
}

// 定点绕行参考（只作优化参考，真正的碰撞由完整 footprint 校验把关）。
//
// 【重写理由】原实现在"临时加入的障碍物"上表现不好，用户实测两个问题：
//   1) 大老远就向旁边拐。原实现把横向偏移的峰值放在障碍中心
//      （peak=(first+last)/2），到障碍中心才达到最大偏移，等于车**贴着
//      障碍物在转弯**；而且只要 route.front()（车当前位姿）与航线有
//      0.10 m 以上横向跟踪误差就整条返回空——实测车侧偏 0.1~0.27 m，
//      这个绕行参考从来没生效过，避障全靠 EGO 样条在 4.5 m 视域里自己
//      弯，于是离障碍 3~4 m 就开始偏。
//   2) 从障碍旁边走过后车尾摆尾撞障碍。车长 1.09 m、车尾角点距
//      base_link 0.66 m，比 0.5 m 的搜索安全包络还大；原实现一见障碍
//      后退就开始回轨（end=last+transition），此时车尾还在障碍旁边，
//      回轨的转弯把车尾扫进障碍。
//
// 新曲线分三段（s 从车位姿在航线上的投影算起）：
//   [begin, first-front_clear]   平滑横移到最大偏移  —— 在车头到达障碍
//                                前就完成，不再贴着障碍转弯；
//   [first-front_clear, last+rear_clear]  保持偏移 —— 一直保持到车尾
//                                （rear_clear = 车尾长度 + 余量）越过障碍；
//   [last+rear_clear, end]       平滑回轨 —— 此时车身已完全越过障碍。
//
// 轴线用航线自身走向（后半段）拟合，不再把车当前位姿的横向误差当作
// "航线不直"，这样轻微侧偏时绕行参考也能正常生成。
template<class Point, class Occupied, class Blocked>
std::vector<Point> smoothDetour(const std::vector<Point> &route, Occupied occupied, Blocked blocked,
                               double entry, double front_clear, double rear_clear,
                               double exit_length, double max_offset, double step,
                               int preferred_side=1, double max_curvature=0.9,
                               std::string *why=nullptr) {
  auto fail=[&](const char *reason){if(why)*why=reason;return std::vector<Point>{};};
  if(route.size()<3 || entry<=0 || max_offset<=0 || step<=0)return fail("route/entry/offset invalid");
  const auto &b=route.back();
  const auto &a=route[std::max<size_t>(1,route.size()/2)];
  const double axis_length=std::hypot(b.x-a.x,b.y-a.y);
  if(axis_length<1.0)return fail("route axis too short");
  const double tx=(b.x-a.x)/axis_length,ty=(b.y-a.y)/axis_length;
  // 只用车之后的航线路由判定直线性/单调性：首点是车当前位姿，它的横向
  // 跟踪误差不属于"航线不直"。
  double previous=-1e9;
  for(size_t i=1;i<route.size();++i) {
    const auto &p=route[i];
    const double s=(p.x-a.x)*tx+(p.y-a.y)*ty;
    if(std::abs(-(p.x-a.x)*ty+(p.y-a.y)*tx)>.30 || s<previous-.01)return fail("route not straight ahead");
    previous=s;
  }
  // 以车位姿在轴线上的投影为原点，s 就是"沿航线还要走多远"。
  const auto &front=route.front();
  const double s0=(front.x-a.x)*tx+(front.y-a.y)*ty;
  const double ox=a.x+s0*tx,oy=a.y+s0*ty;
  const double length=(b.x-ox)*tx+(b.y-oy)*ty;
  if(length<1.0)return fail("remaining route too short");
  // 车当前相对轴线的横向偏移（与下面 lateral 同符号）。参考必须从它出发：
  // 实测车带着 0.28 m 跟踪误差进入绕行，若参考立刻回到轴线，下一步的横向
  // 跳变会被优化器判为不可行，整条参考被拒（run50 绕过半程后参考作废，
  // 侧向种子在 ±0.4 m 之间来回换边，车最后贴着箱子蹭过去）。
  const double lateral0=-(front.x-a.x)*ty+(front.y-a.y)*tx;
  // first/last 取**物理障碍**（lethal 栅格）的范围：它决定横移起止位置。
  // 带安全余量的 occupied 只用于保证绕行参考自身离障碍足够远。
  const int n=std::max(2,int(std::ceil(length/step)));
  double first=-1,last=-1;
  for(int i=0;i<=n;++i) {
    const double s=length*i/n;
    if(blocked(ox+s*tx,oy+s*ty)) {if(first<0)first=s;last=s;}
  }
  if(first<0)return fail("no blocked cell ahead");
  const double hold_begin=first-front_clear;
  // 回轨必须等车尾完全越过障碍：last 是障碍尾缘，rear_clear 是车尾长度+余量。
  const double hold_end=std::min(length,last+rear_clear);
  if(hold_end<hold_begin)return fail("hold segment empty");
  // 1) 两侧分别求"保持段"所需的最小横移量。障碍不一定压在航线正中央：
  //    只按 preferred_side 搜索时，若障碍恰好在那一侧，就会被逼出一个很大的
  //    横移量，甚至绕到障碍那一侧去（实测 0.5 m 箱子偏在航线一侧，参考选了
  //    箱子所在的一侧、横移 0.80 m，7 s 后地图把箱体补全才发现参考穿进箱体，
  //    绕行整段作废）。两侧都算，取需要横移更小的一侧。
  double minimum_side[2]={0.0,0.0};
  for(int k=0;k<2;++k) {
    const int side=(k==0)?1:-1;
    for(double amplitude=.2;amplitude<=max_offset+1e-6;amplitude+=.2) {
      bool free=true;
      for(int i=0;i<=n && free;++i) {
        const double s=length*i/n;
        if(s<hold_begin||s>hold_end)continue;
        const double lateral=side*amplitude;
        Point p=a;p.x=ox+s*tx-lateral*ty;p.y=oy+s*ty+lateral*tx;
        if(occupied(p.x,p.y))free=false;
      }
      if(free){minimum_side[k]=amplitude;break;}
    }
  }
  if(minimum_side[0]<=0 && minimum_side[1]<=0)return fail("both sides blocked within max offset");
  // 2) 坡度段长度由**曲率**决定，而不是"1.5 倍横移量"：smoothStep5 的最大
  //    二阶导数是 5.77/L²·A，要压到车在最高速下也能跟住的曲率
  //    max_curvature(=max_w/max_v)，L 必须 >= sqrt(5.77·A/max_curvature)。
  //    实测 1.0 m 横移只给 1.5 m 坡度段（曲率 2.6 1/m，半径 0.39 m），车
  //    0.5 m/s 时根本转不过来，参考被优化器丢成"贴着航线走"，到箱子边上
  //    才被迫急转；run50 就是这样卡死的。
  // preferred_side==0 表示"自动选边"：走横移量小的那一侧（两侧都能通过时
  // 走 +1 一侧）。绕行已经开始的周期传上一次的绕行方向，保持同一侧——否则
  // 障碍表面被扫描逐步补全时，两侧所需横移量来回变化，参考会左右换边，
  // 车先往一侧压再甩回另一侧，画出一个 S 形（实测 0.5 m 箱子先向右压
  // 0.2 m 再向左横移 1.1 m）。
  int sides[2];
  if(preferred_side==0) {
    const bool flip=minimum_side[1]>0 && (minimum_side[0]<=0 || minimum_side[1]<minimum_side[0]);
    sides[0]=flip?-1:1;sides[1]=-sides[0];
  } else {
    sides[0]=preferred_side;sides[1]=-preferred_side;
  }
  auto fmt=[&](const char *f,double v1,double v2=0.0,double v3=0.0,double v4=0.0) {
    char buf[160];std::snprintf(buf,sizeof(buf),f,v1,v2,v3,v4);return std::string(buf);
  };
  std::string last_reason="no side free";
  for(int side:sides) {
    double start=(side>0)?minimum_side[0]:minimum_side[1];
    if(start<=0)continue;
    // 【run55】最小横移量只保证"参考点刚好 0.65 m"，但实际跟踪会少走 0.05~
    // 0.10 m（B 样条优化器在障碍附近的曲率/回轨代价会让实际横移小于参考），
    // 而且障碍是逐帧补全的。按最小值走等于把余量全吃掉：实测车心只到
    // 0.91 m，车体侧面离箱面只剩 0.28 m，下一帧局部代价地图补全后所有样条
    // 都过不了 0.45 m 车体净空校验（"no verified lateral target within 1.2 m"），
    // 车在箱子旁边被 patience 判死。多留一个 0.2 m 档，实际通过横移 ~1.1 m。
    start=std::min(max_offset,start+.2);
    for(double amplitude=start;amplitude<=max_offset+1e-6;amplitude+=.2) {
      const double rise=std::max(entry,std::sqrt(5.7735*amplitude/std::max(1e-6,max_curvature)));
      const double begin_unclamped=first-front_clear-rise;
      // 障碍已经近到"来不及横移完"：把坡度段压到车当前位置开始（参考仍然
      // 从车位姿出发），只要保持段本身还有空间就还能用。真到了连保持段起点
      // 都落在车旁/身后，才放弃，交给优化器就地绕行。
      if(begin_unclamped<=0 && hold_begin<=0.25)return fail("obstacle too close for any lateral offset");
      const double begin=std::max(0.0,begin_unclamped);
      // 回轨段同样受曲率限制：原来固定 exit_length=1.2 m 把 1.0 m 横移压进
      // 1.2 m（坡度峰值航向 51°，曲率 4.0 1/m），车根本跟不住——参考回轨结束
      // 时车头还斜着 0.4~0.6 rad，参考一断就变成"原地急转"，曲率限速把车压到
      // 0.03 m/s 爬行（run53/run54 绕过箱子后连续两次"进度停滞"）。
      const double exit_rise=std::max(exit_length,
          std::sqrt(5.7735*amplitude/std::max(1e-6,max_curvature)));
      // 【run59】回轨段绝不能被航线参考的剩余长度截断。原来 end=min(length,…)：
      // 当前视参考只剩 0.7 m 时，等于把 1.2 m 横移压进 0.7 m，坡度峰值斜率
      // 3.0、曲率 ~10 1/m；优化器只能把这个折角拟合进样条，随后
      // previewEnvelope 按 max_w/k 把速度压到 0.05 m/s，车在回轨汇合点爬行
      // ~20 s（run58/run59 实测 k=8.6, ab=bc=0.007）。参考越过 route 末端
      // 后继续沿条带直线方向外推（occupied 逐点校验），回轨才是切向的。
      const double end=hold_end+exit_rise;
      const double reach=std::max(length,end);
      const int m=std::max(2,int(std::ceil(reach/step)));
      std::vector<Point> candidate;candidate.reserve(m+1);bool free=true;int bad=0;
      double clipped_at=-1;
      for(int i=0;i<=m;++i) {
        const double s=reach*i/m;
        double lateral;
        if(s<=hold_begin) {
          // 从车当前横向偏移平滑爬到目标偏移；begin 之前保持车当前偏移。
          const double u=(begin_unclamped<=0)?smoothStep5(s/std::max(1e-6,hold_begin)):
            (s<=begin?0.0:smoothStep5((s-begin)/std::max(1e-6,hold_begin-begin)));
          lateral=lateral0+(side*amplitude-lateral0)*u;
        } else if(s>hold_end) {
          lateral=side*amplitude*(1-smoothStep5((s-hold_end)/std::max(1e-6,end-hold_end)));
        } else {
          lateral=side*amplitude;
        }
        Point p=a;p.x=ox+s*tx-lateral*ty;p.y=oy+s*ty+lateral*tx;
        if(i==0){p.x=front.x;p.y=front.y;}  // 参考从车当前位置出发
        if(occupied(p.x,p.y)) {
          // 【run60】回轨外推段被静态停放物挡住时**截断**，不要整条作废：
          // 本场景条带前方 x≈-6 之后横向 ±0.5 m 就只剩 0.4~0.65 m 净空，
          // 3 m 的回轨空间根本不存在；若按"不满足就整条失败"，回轨期间会
          // 完全没有参考（实测 run60 参考一个都生成不出来，靠优化器兜底，
          // 连续"进度停滞"）。截断后参考停在坡度段中间，由下一周期重新
          // 生成，既保住绕行参考，又不会把坡度压成折角。
          if(s>length && !candidate.empty()) {clipped_at=s;break;}
          free=false;bad=i;break;
        }
        candidate.push_back(p);
      }
      if(free) {
        if(why) *why=fmt("guide side=%+.0f amp=%.2f lat0=%+.2f begin=%.2f",
                         side,amplitude,lateral0,begin)+
          fmt(" hold=[%.2f,%.2f] ramp=%.2f exit=%.2f",hold_begin,hold_end,
              hold_begin-begin,exit_rise)+
          (clipped_at>0?fmt(" clipped=%.2f",clipped_at):std::string());
        return candidate;
      }
      last_reason=fmt("guide side=%+.0f amp=%.2f too close at s=%.2f",side,amplitude,reach*bad/m);
    }
  }
  return fail(last_reason.c_str());
}


// Bound speed now so braking can reach a curvature-safe speed before the bend.
// This supplements, rather than replaces, the instantaneous pursuit limit.
template<class Point>
double curvePreviewSpeed(const std::vector<Point> &path,size_t nearest,double preview,
                         double maximum,double max_w,double lateral_accel,double brake_accel) {
  double cap=maximum,travel=0;
  for(size_t i=std::max(size_t(1),nearest+1);i+1<path.size();++i) {
    const double ax=path[i].x-path[i-1].x,ay=path[i].y-path[i-1].y;
    const double bx=path[i+1].x-path[i].x,by=path[i+1].y-path[i].y;
    const double ab=std::hypot(ax,ay),bc=std::hypot(bx,by),ac=std::hypot(ax+bx,ay+by);
    travel+=ab;if(travel>preview)break;
    if(ab*bc*ac<1e-9)continue;
    const double k=2*std::abs(ax*by-ay*bx)/(ab*bc*ac);
    if(k<1e-6)continue;
    const double bend_speed=std::min(max_w/k,std::sqrt(lateral_accel/k));
    cap=std::min(cap,std::sqrt(bend_speed*bend_speed+2*brake_accel*travel));
  }
  return cap;
}
}  // namespace ego_nav2_controller
