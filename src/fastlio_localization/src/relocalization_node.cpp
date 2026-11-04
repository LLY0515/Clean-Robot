// ============================================================================
//  fastlio_localization —— 基于 3D 点云先验地图的全局重定位节点
//
//  作用：
//    用 FAST-LIO 的 /cloud_registered（odom 系）与事先保存的 PCD 先验地图做
//    点云配准，估计 map->odom，发布 TF，替代 Nav2 的 AMCL。
//
//  流程：
//    1. 启动时加载先验 PCD 并体素下采样，建立 KD-Tree；
//    2. 先发布单位阵 map->odom（让 Nav2 的 TF 链先通），再做 (x, y, yaw)
//       栅格全局搜索：每个候选位姿把当前扫描投到地图上，统计最近邻距离
//       小于 inlier_distance 的点占比（内点率）；
//    3. 取内点率最高的若干候选，用 ICP 精配准，选内点率最高的结果作为
//       初始 map->odom；
//    4. 之后每来一帧点云，以上一帧 map->odom 为初值做 ICP 跟踪；
//       连续多帧内点率过低或位姿跳变过大时，自动重新全局重定位。
//
//  为什么只用 /cloud_registered：
//    该话题的点已经在 odom 系下，ICP 求出的变换直接就是 map->odom，
//    不需要再查 odom->base_link 做坐标复合，链路上少一层时间同步误差。
//
//  为什么把结果投影回平面：
//    地面机器人只需要 map->odom 的 (x, y, yaw)；6DoF ICP 在稀疏点云上
//    偶尔会在 z/roll/pitch 上漂一点，投影后 Nav2 的代价地图更稳定。
// ============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace {

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

constexpr double kPi = 3.14159265358979323846;

// 把 4x4 变换投影为地面机器人可用的 (x, y, yaw)：z、roll、pitch 归零。
Eigen::Matrix4f planarize(const Eigen::Matrix4f& T) {
  const Eigen::Matrix3f R = T.block<3, 3>(0, 0);
  const float yaw = std::atan2(R(1, 0), R(0, 0));
  Eigen::Matrix4f out = Eigen::Matrix4f::Identity();
  out.block<3, 3>(0, 0) =
      Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  out(0, 3) = T(0, 3);
  out(1, 3) = T(1, 3);
  out(2, 3) = 0.0f;
  return out;
}

Eigen::Matrix4f poseToMatrix(const geometry_msgs::msg::Pose& p) {
  Eigen::Quaternionf q(static_cast<float>(p.orientation.w),
                       static_cast<float>(p.orientation.x),
                       static_cast<float>(p.orientation.y),
                       static_cast<float>(p.orientation.z));
  q.normalize();
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T.block<3, 3>(0, 0) = q.toRotationMatrix();
  T(0, 3) = static_cast<float>(p.position.x);
  T(1, 3) = static_cast<float>(p.position.y);
  T(2, 3) = static_cast<float>(p.position.z);
  return T;
}

// 对两个位姿做插值：平移线性插值，旋转球面插值。
Eigen::Matrix4f blend(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b,
                      float alpha) {
  alpha = std::clamp(alpha, 0.0f, 1.0f);
  Eigen::Quaternionf qa(a.block<3, 3>(0, 0));
  Eigen::Quaternionf qb(b.block<3, 3>(0, 0));
  qa.normalize();
  qb.normalize();
  const Eigen::Quaternionf q = qa.slerp(alpha, qb);
  Eigen::Matrix4f out = Eigen::Matrix4f::Identity();
  out.block<3, 3>(0, 0) = q.toRotationMatrix();
  out.block<3, 1>(0, 3) =
      (1.0f - alpha) * a.block<3, 1>(0, 3) + alpha * b.block<3, 1>(0, 3);
  return out;
}

// 判断两个平面位姿是否“同一个解”（用于剔重和歧义检测）。
bool samePose(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b) {
  const double dx = a(0, 3) - b(0, 3);
  const double dy = a(1, 3) - b(1, 3);
  if (std::hypot(dx, dy) > 0.75) {
    return false;
  }
  const double ya = std::atan2(a(1, 0), a(0, 0));
  const double yb = std::atan2(b(1, 0), b(0, 0));
  const double dyaw = std::remainder(ya - yb, 2.0 * kPi);
  return std::abs(dyaw) <= 0.5;  // 约 29°
}

}  // namespace

class RelocalizationNode : public rclcpp::Node {
 public:
  RelocalizationNode() : Node("fastlio_relocalization") {
    declareParameters();

    // 先发布单位阵，保证 Nav2 启动阶段 map->odom 这条 TF 一直存在。
    T_map_odom_.setIdentity();
    has_map_to_odom_ = true;
    localized_ = !relocalize_on_start_;
    global_search_pending_ = relocalize_on_start_;

    loadPriorMap();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    pub_pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "localization_pose", 10);
    pub_prior_map_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "prior_map", rclcpp::QoS(1).transient_local().reliable());
    pub_aligned_scan_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "aligned_scan", 10);

    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        scan_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          cloudCallback(msg);
        });

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_odom_ = *msg;
          has_odom_ = true;
        });

    srv_relocalize_ = create_service<std_srvs::srv::Trigger>(
        "relocalize",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          std::lock_guard<std::mutex> lock(mutex_);
          localized_ = false;
          global_search_pending_ = true;
          fail_count_ = 0;
          res->success = true;
          res->message = "Global relocalization requested.";
          RCLCPP_INFO(get_logger(), "收到 /relocalize 请求，准备重新全局重定位。");
        });

    const double rate = std::max(1.0, publish_rate_);
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate),
        std::bind(&RelocalizationNode::timerCallback, this));

    publishPriorMap();

    RCLCPP_INFO(get_logger(),
                "3D 重定位节点启动：先验地图 %s，点云话题 %s，map->odom 发布频率 %.1f Hz",
                prior_map_path_.c_str(), scan_topic_.c_str(), rate);
  }

 private:
  void declareParameters() {
    prior_map_path_ = declare_parameter<std::string>("prior_map_path", "");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/cloud_registered");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    map_voxel_size_ = declare_parameter<double>("map_voxel_size", 0.15);
    scan_voxel_size_ = declare_parameter<double>("scan_voxel_size", 0.20);
    global_scan_voxel_size_ =
        declare_parameter<double>("global_scan_voxel_size", 0.45);
    min_scan_range_ = declare_parameter<double>("min_scan_range", 0.5);
    max_scan_range_ = declare_parameter<double>("max_scan_range", 15.0);

    icp_max_iterations_ = declare_parameter<int>("icp_max_iterations", 30);
    icp_corr_dist_tracking_ =
        declare_parameter<double>("icp_corr_dist_tracking", 0.7);
    icp_corr_dist_relocalize_ =
        declare_parameter<double>("icp_corr_dist_relocalize", 1.5);
    inlier_distance_ = declare_parameter<double>("inlier_distance", 0.35);
    min_inlier_ratio_ = declare_parameter<double>("min_inlier_ratio", 0.55);
    relocalize_fail_count_ =
        declare_parameter<int>("relocalize_fail_count", 5);
    max_tracking_jump_ = declare_parameter<double>("max_tracking_jump", 1.0);
    max_tracking_yaw_jump_ =
        declare_parameter<double>("max_tracking_yaw_jump", 0.6);
    correction_alpha_ = declare_parameter<double>("correction_alpha", 0.6);

    global_search_xy_step_ =
        declare_parameter<double>("global_search_xy_step", 0.5);
    global_search_yaw_step_deg_ =
        declare_parameter<double>("global_search_yaw_step_deg", 15.0);
    global_search_margin_ =
        declare_parameter<double>("global_search_margin", 0.6);
    global_search_top_k_ = declare_parameter<int>("global_search_top_k", 8);
    global_search_max_points_ =
        declare_parameter<int>("global_search_max_points", 200);
    // 只用先验地图“顶部”这一段高度内的结构做全局搜索评分。
    //
    // 原因：本项目的雷达装得比 1 m 高的围墙还高、且前倾 30°，建图点云里
    // 绝大部分是地面和墙脚；地面在 (x, y, yaw) 上是平移不变的，几乎不提供
    // 任何定位信息，反而会把“90° 旋转后仍然贴着墙”的对称解分数抬高。
    // 真正能区分方位的是墙顶、箱顶、球顶这些高处结构，因此默认只取地图
    // 最高点以下 0.35 m 内的点参与全局搜索。跟踪阶段仍用完整点云。
    // <= 0 表示关闭该过滤，退回全图搜索。
    global_search_height_ =
        declare_parameter<double>("global_search_height", 0.35);

    constrain_planar_ = declare_parameter<bool>("constrain_planar", true);
    relocalize_on_start_ =
        declare_parameter<bool>("relocalize_on_start", true);
    publish_aligned_scan_ =
        declare_parameter<bool>("publish_aligned_scan", true);
    publish_rate_ = declare_parameter<double>("publish_rate", 10.0);
    // AMCL 发布 map->odom 时会把时间戳打到 now + transform_tolerance，
    // 避免 Nav2 在“当前时刻”查询时出现 “extrapolation into the future”。
    // 这里沿用同样做法：TF 时间戳超前一点，10 Hz 发布也不会让查询卡在最近一帧之后。
    tf_future_stamp_ = declare_parameter<double>("tf_future_stamp", 0.2);
  }

  void loadPriorMap() {
    if (prior_map_path_.empty()) {
      RCLCPP_ERROR(get_logger(),
                   "prior_map_path 为空！请先用 build_3d_map.sh 生成先验地图。");
      return;
    }

    CloudT::Ptr raw(new CloudT);
    if (pcl::io::loadPCDFile<PointT>(prior_map_path_, *raw) < 0) {
      RCLCPP_ERROR(get_logger(), "加载先验地图失败：%s",
                   prior_map_path_.c_str());
      return;
    }
    if (raw->empty()) {
      RCLCPP_ERROR(get_logger(), "先验地图 %s 为空。", prior_map_path_.c_str());
      return;
    }

    std::vector<int> nan_indices;
    pcl::removeNaNFromPointCloud(*raw, *raw, nan_indices);

    CloudT::Ptr down(new CloudT);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(raw);
    vg.setLeafSize(static_cast<float>(map_voxel_size_),
                   static_cast<float>(map_voxel_size_),
                   static_cast<float>(map_voxel_size_));
    vg.filter(*down);

    map_ = down;
    map_tree_.setInputCloud(map_);
    map_tree_ready_ = true;
    pcl::getMinMax3D(*map_, map_min_, map_max_);

    // 全局搜索用的“上部结构”子图：只保留最高点以下 global_search_height_ 内的点。
    if (global_search_height_ > 0.0 && !map_->empty()) {
      const float z_min =
          map_max_.z - static_cast<float>(global_search_height_);
      CloudT::Ptr top(new CloudT);
      top->reserve(map_->size());
      for (const auto& p : map_->points) {
        if (p.z >= z_min) {
          top->push_back(p);
        }
      }
      if (top->size() >= 100) {
        map_top_ = top;
        map_top_tree_.setInputCloud(map_top_);
        map_top_ready_ = true;
        RCLCPP_INFO(get_logger(),
                    "全局搜索使用上部结构：z >= %.2f m，%zu 点（全图 %zu 点）",
                    z_min, map_top_->size(), map_->size());
      } else {
        RCLCPP_WARN(get_logger(),
                    "上部结构只有 %zu 点，改用全图做全局搜索。", top->size());
      }
    }

    icp_.setInputTarget(map_);
    icp_.setMaximumIterations(icp_max_iterations_);
    icp_.setTransformationEpsilon(1e-8);
    icp_.setEuclideanFitnessEpsilon(1e-8);
    icp_.setUseReciprocalCorrespondences(false);

    RCLCPP_INFO(get_logger(),
                "先验地图加载完成：%zu 点（原始 %zu 点），包围盒 x[%.2f, %.2f] "
                "y[%.2f, %.2f] z[%.2f, %.2f]",
                map_->size(), raw->size(), map_min_.x, map_max_.x, map_min_.y,
                map_max_.y, map_min_.z, map_max_.z);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!map_loaded()) {
      return;
    }

    CloudT::Ptr raw(new CloudT);
    pcl::fromROSMsg(*msg, *raw);

    CloudT::Ptr filtered(new CloudT);
    filtered->reserve(raw->size());
    const float min_r2 =
        static_cast<float>(min_scan_range_ * min_scan_range_);
    const float max_r2 =
        static_cast<float>(max_scan_range_ * max_scan_range_);
    for (const auto& p : raw->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      const float r2 = p.x * p.x + p.y * p.y + p.z * p.z;
      if (r2 < min_r2 || r2 > max_r2) {
        continue;
      }
      filtered->push_back(p);
    }

    if (filtered->size() < 30) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    latest_scan_ = filtered;
    ++scan_seq_;
  }

  void timerCallback() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!map_loaded() || !latest_scan_) {
      // 地图还没加载或还没有点云：至少把 map->odom 维持住。
      if (has_map_to_odom_) {
        publishTfAndPose();
      }
      return;
    }
    if (scan_seq_ == last_processed_seq_) {
      if (has_map_to_odom_) {
        publishTfAndPose();
      }
      return;
    }
    last_processed_seq_ = scan_seq_;

    CloudT::Ptr scan(new CloudT);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(latest_scan_);
    vg.setLeafSize(static_cast<float>(scan_voxel_size_),
                   static_cast<float>(scan_voxel_size_),
                   static_cast<float>(scan_voxel_size_));
    vg.filter(*scan);
    if (scan->size() < 30) {
      if (has_map_to_odom_) {
        publishTfAndPose();
      }
      return;
    }

    if (global_search_pending_) {
      globalSearch(scan);
    } else if (localized_) {
      track(scan);
    }

    if (has_map_to_odom_) {
      publishTfAndPose();
    }
  }

  // -------------------------------------------------------------------------
  //  跟踪：以上一帧 map->odom 为初值做 ICP
  // -------------------------------------------------------------------------
  void track(const CloudT::Ptr& scan) {
    Eigen::Matrix4f estimate;
    if (!runIcp(scan, T_map_odom_, icp_corr_dist_tracking_, estimate)) {
      onTrackingFailure("ICP 未收敛");
      return;
    }
    if (constrain_planar_) {
      estimate = planarize(estimate);
    }

    double mean_dist = 0.0;
    const double score =
        scorePose(*scan, estimate, inlier_distance_, mean_dist);
    if (score < min_inlier_ratio_) {
      onTrackingFailure("内点率过低");
      return;
    }

    // 防止单帧 ICP 把 map->odom 拉飞：与上一帧相比平移/偏航跳变过大就丢弃。
    const Eigen::Matrix4f delta = T_map_odom_.inverse() * estimate;
    const double jump = delta.block<3, 1>(0, 3).norm();
    const double yaw_jump =
        std::abs(std::atan2(delta(1, 0), delta(0, 0)));
    if (jump > max_tracking_jump_ || yaw_jump > max_tracking_yaw_jump_) {
      onTrackingFailure("位姿跳变过大");
      return;
    }

    T_map_odom_ = blend(T_map_odom_, estimate,
                        static_cast<float>(correction_alpha_));
    if (constrain_planar_) {
      T_map_odom_ = planarize(T_map_odom_);
    }
    fail_count_ = 0;
    last_score_ = score;
    last_mean_dist_ = mean_dist;

    if (publish_aligned_scan_) {
      publishAlignedScan(scan, T_map_odom_);
    }
  }

  void onTrackingFailure(const char* reason) {
    ++fail_count_;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "3D 跟踪失败（%s），连续失败 %d/%d 次", reason,
                         fail_count_, relocalize_fail_count_);
    if (fail_count_ >= relocalize_fail_count_) {
      RCLCPP_WARN(get_logger(), "连续跟踪失败，触发全局重定位。");
      fail_count_ = 0;
      localized_ = false;
      global_search_pending_ = true;
    }
  }

  // -------------------------------------------------------------------------
  //  全局重定位：(x, y, yaw) 栅格搜索 + ICP 精配准
  // -------------------------------------------------------------------------
  void globalSearch(const CloudT::Ptr& scan) {
    const auto begin = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "开始全局重定位：在 %.2f m 栅格上搜索 (x, y, yaw) ...",
                global_search_xy_step_);

    // 优先只用地面上方的结构做搜索（见 global_search_height 的说明）。
    CloudT::Ptr search_scan = scan;
    pcl::KdTreeFLANN<PointT>* search_tree = &map_tree_;
    if (map_top_ready_) {
      CloudT::Ptr scan_top(new CloudT);
      const float z_min =
          map_max_.z - static_cast<float>(global_search_height_);
      scan_top->reserve(scan->size());
      for (const auto& p : scan->points) {
        if (p.z >= z_min) {
          scan_top->push_back(p);
        }
      }
      if (scan_top->size() >= 40) {
        search_scan = scan_top;
        search_tree = &map_top_tree_;
      } else {
        RCLCPP_WARN(get_logger(),
                    "当前帧上部结构只有 %zu 点，改用整帧点云做全局搜索。",
                    scan_top->size());
      }
    }

    CloudT::Ptr coarse(new CloudT);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(search_scan);
    vg.setLeafSize(static_cast<float>(global_scan_voxel_size_),
                   static_cast<float>(global_scan_voxel_size_),
                   static_cast<float>(global_scan_voxel_size_));
    vg.filter(*coarse);
    if (static_cast<int>(coarse->size()) < 30) {
      coarse = search_scan;
    }
    if (static_cast<int>(coarse->size()) > global_search_max_points_) {
      CloudT::Ptr sampled(new CloudT);
      const int step = std::max(
          1, static_cast<int>(std::ceil(
                 coarse->size() /
                 static_cast<double>(global_search_max_points_))));
      for (std::size_t i = 0; i < coarse->size(); i += step) {
        sampled->push_back(coarse->points[i]);
      }
      coarse = sampled;
    }

    const float margin = static_cast<float>(global_search_margin_);
    const float step = static_cast<float>(global_search_xy_step_);
    const float min_x = map_min_.x + margin;
    const float max_x = map_max_.x - margin;
    const float min_y = map_min_.y + margin;
    const float max_y = map_max_.y - margin;

    struct Candidate {
      double score;
      Eigen::Matrix4f T;
    };
    std::vector<Candidate> candidates;

    const float yaw_step = static_cast<float>(
        global_search_yaw_step_deg_ * kPi / 180.0);
    const int yaw_bins =
        std::max(1, static_cast<int>(std::round(2.0 * kPi / yaw_step)));
    const float yaw0 = static_cast<float>(-kPi);

    if (max_x > min_x && max_y > min_y) {
      for (float x = min_x; x <= max_x + 1e-4f; x += step) {
        for (float y = min_y; y <= max_y + 1e-4f; y += step) {
          for (int i = 0; i < yaw_bins; ++i) {
            const float yaw = yaw0 + i * yaw_step;
            Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
            T.block<3, 3>(0, 0) =
                Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ())
                    .toRotationMatrix();
            T(0, 3) = x;
            T(1, 3) = y;
            T(2, 3) = 0.0f;
            double mean_dist = 0.0;
            const double score = scorePose(*coarse, T, inlier_distance_,
                                           mean_dist, search_tree);
            candidates.push_back({score, T});
          }
        }
      }
    } else {
      // 地图包围盒异常小，退化为原地搜索。
      RCLCPP_WARN(get_logger(),
                  "先验地图包围盒过小，全局搜索退化为原点附近。");
      Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
      for (int i = 0; i < yaw_bins; ++i) {
        T.block<3, 3>(0, 0) =
            Eigen::AngleAxisf(yaw0 + i * yaw_step, Eigen::Vector3f::UnitZ())
                .toRotationMatrix();
        double mean_dist = 0.0;
        candidates.push_back(
            {scorePose(*coarse, T, inlier_distance_, mean_dist, search_tree),
             T});
      }
    }

    const std::size_t keep =
        std::min(candidates.size(),
                 static_cast<std::size_t>(std::max(1, global_search_top_k_)));
    std::partial_sort(
        candidates.begin(), candidates.begin() + keep, candidates.end(),
        [](const Candidate& a, const Candidate& b) {
          return a.score > b.score;
        });

    double best_score = -1.0;
    Eigen::Matrix4f best_T = Eigen::Matrix4f::Identity();
    double second_score = -1.0;
    Eigen::Matrix4f second_T = Eigen::Matrix4f::Identity();
    for (std::size_t i = 0; i < keep; ++i) {
      Eigen::Matrix4f estimate;
      if (!runIcp(scan, candidates[i].T, icp_corr_dist_relocalize_,
                  estimate)) {
        continue;
      }
      if (constrain_planar_) {
        estimate = planarize(estimate);
      }
      double mean_dist = 0.0;
      // 用与粗搜索相同的“上部结构”来评分，才能真正区分对称解；
      // ICP 本身仍用整帧点云，保证收敛稳定。
      const double score = scorePose(*search_scan, estimate, inlier_distance_,
                                     mean_dist, search_tree);
      if (score > best_score) {
        // 原第一名降到第二名，但要排除只是同一位姿的重复候选。
        if (best_score >= 0.0 && !samePose(estimate, best_T)) {
          second_score = best_score;
          second_T = best_T;
        }
        best_score = score;
        best_T = estimate;
        last_mean_dist_ = mean_dist;
      } else if (score > second_score && !samePose(estimate, best_T)) {
        second_score = score;
        second_T = estimate;
      }
    }

    // 对称场地（正方形围墙）容易出现 90°/180° 的“鬼影解”。如果第二名与
    // 第一名的得分很接近、位置却相差很远，明确告警，方便排查是否定位错。
    if (best_score > 0.0 && second_score >= 0.0 &&
        second_score >= 0.9 * best_score && !samePose(best_T, second_T)) {
      RCLCPP_WARN(get_logger(),
                  "全局重定位存在歧义：最优 (%.2f, %.2f, %.1f°) 得分 %.2f，"
                  "次优 (%.2f, %.2f, %.1f°) 得分 %.2f。若位置明显不对，"
                  "请检查先验地图或手动调用 /relocalize。",
                  best_T(0, 3), best_T(1, 3),
                  std::atan2(best_T(1, 0), best_T(0, 0)) * 180.0 / kPi,
                  best_score, second_T(0, 3), second_T(1, 3),
                  std::atan2(second_T(1, 0), second_T(0, 0)) * 180.0 / kPi,
                  second_score);
    }

    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - begin)
            .count();

    if (best_score >= min_inlier_ratio_) {
      T_map_odom_ = best_T;
      localized_ = true;
      global_search_pending_ = false;
      fail_count_ = 0;
      last_score_ = best_score;
      RCLCPP_INFO(get_logger(),
                  "全局重定位成功：内点率 %.2f，平均最近邻距离 %.3f m，"
                  "map->odom = (x=%.2f, y=%.2f, yaw=%.1f°)，耗时 %.2f s",
                  best_score, last_mean_dist_, best_T(0, 3), best_T(1, 3),
                  std::atan2(best_T(1, 0), best_T(0, 0)) * 180.0 / kPi,
                  elapsed);
    } else {
      localized_ = false;
      global_search_pending_ = true;
      RCLCPP_WARN(get_logger(),
                  "全局重定位失败：最高内点率 %.2f < %.2f（耗时 %.2f s），"
                  "保持上一次 map->odom 并稍后重试。",
                  best_score, min_inlier_ratio_, elapsed);
    }
  }

  bool runIcp(const CloudT::Ptr& source, const Eigen::Matrix4f& initial,
              double max_corr_dist, Eigen::Matrix4f& result) {
    icp_.setInputSource(source);
    icp_.setMaxCorrespondenceDistance(
        static_cast<float>(max_corr_dist));
    CloudT aligned;
    icp_.align(aligned, initial);
    if (!icp_.hasConverged()) {
      return false;
    }
    result = icp_.getFinalTransformation();
    return true;
  }

  // 内点率：变换后每个点的最近邻距离小于 inlier_dist 的比例。
  // tree 为空时用整张先验地图；全局搜索会传入“上部结构”子图的 KD-Tree。
  double scorePose(const CloudT& src, const Eigen::Matrix4f& T,
                   double inlier_dist, double& mean_dist,
                   pcl::KdTreeFLANN<PointT>* tree = nullptr) {
    pcl::KdTreeFLANN<PointT>& kd = tree ? *tree : map_tree_;
    const float d2_threshold =
        static_cast<float>(inlier_dist * inlier_dist);
    int inliers = 0;
    double sum_dist = 0.0;
    const int n = static_cast<int>(src.size());
    for (int i = 0; i < n; ++i) {
      const PointT& p = src.points[i];
      PointT q;
      q.x = T(0, 0) * p.x + T(0, 1) * p.y + T(0, 2) * p.z + T(0, 3);
      q.y = T(1, 0) * p.x + T(1, 1) * p.y + T(1, 2) * p.z + T(1, 3);
      q.z = T(2, 0) * p.x + T(2, 1) * p.y + T(2, 2) * p.z + T(2, 3);
      q.intensity = 0.0f;
      if (kd.nearestKSearch(q, 1, nn_indices_, nn_distances_) > 0) {
        sum_dist += std::sqrt(nn_distances_[0]);
        if (nn_distances_[0] <= d2_threshold) {
          ++inliers;
        }
      }
    }
    mean_dist = n > 0 ? sum_dist / n : 1e9;
    return n > 0 ? static_cast<double>(inliers) / n : 0.0;
  }

  // -------------------------------------------------------------------------
  //  输出
  // -------------------------------------------------------------------------
  void publishTfAndPose() {
    const rclcpp::Time stamp = now();
    const rclcpp::Time tf_stamp =
        stamp + rclcpp::Duration::from_seconds(tf_future_stamp_);

    const Eigen::Quaternionf q(T_map_odom_.block<3, 3>(0, 0));
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = tf_stamp;
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = odom_frame_;
    tf.transform.translation.x = T_map_odom_(0, 3);
    tf.transform.translation.y = T_map_odom_(1, 3);
    tf.transform.translation.z = T_map_odom_(2, 3);
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);

    if (!has_odom_) {
      return;
    }
    const Eigen::Matrix4f T_odom_base = poseToMatrix(latest_odom_.pose.pose);
    const Eigen::Matrix4f T_map_base = T_map_odom_ * T_odom_base;
    const Eigen::Quaternionf q_map_base(T_map_base.block<3, 3>(0, 0));

    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = map_frame_;
    pose.pose.pose.position.x = T_map_base(0, 3);
    pose.pose.pose.position.y = T_map_base(1, 3);
    pose.pose.pose.position.z = T_map_base(2, 3);
    pose.pose.pose.orientation.x = q_map_base.x();
    pose.pose.pose.orientation.y = q_map_base.y();
    pose.pose.pose.orientation.z = q_map_base.z();
    pose.pose.pose.orientation.w = q_map_base.w();
    pub_pose_->publish(pose);
  }

  void publishPriorMap() {
    if (!map_loaded()) {
      return;
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*map_, msg);
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    pub_prior_map_->publish(msg);
  }

  void publishAlignedScan(const CloudT::Ptr& scan,
                          const Eigen::Matrix4f& T_map_odom) {
    CloudT::Ptr transformed(new CloudT);
    pcl::transformPointCloud(*scan, *transformed, T_map_odom);
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*transformed, msg);
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    pub_aligned_scan_->publish(msg);
  }

  bool map_loaded() const { return map_tree_ready_ && map_ && !map_->empty(); }

  // ---------------- 参数 ----------------
  std::string prior_map_path_;
  std::string scan_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;

  double map_voxel_size_ = 0.15;
  double scan_voxel_size_ = 0.20;
  double global_scan_voxel_size_ = 0.45;
  double min_scan_range_ = 0.5;
  double max_scan_range_ = 15.0;

  int icp_max_iterations_ = 30;
  double icp_corr_dist_tracking_ = 0.7;
  double icp_corr_dist_relocalize_ = 1.5;
  double inlier_distance_ = 0.35;
  double min_inlier_ratio_ = 0.55;
  int relocalize_fail_count_ = 5;
  double max_tracking_jump_ = 1.0;
  double max_tracking_yaw_jump_ = 0.6;
  double correction_alpha_ = 0.6;

  double global_search_xy_step_ = 0.5;
  double global_search_yaw_step_deg_ = 15.0;
  double global_search_margin_ = 0.6;
  int global_search_top_k_ = 8;
  int global_search_max_points_ = 200;
  double global_search_height_ = 0.35;

  bool constrain_planar_ = true;
  bool relocalize_on_start_ = true;
  bool publish_aligned_scan_ = true;
  double publish_rate_ = 10.0;
  double tf_future_stamp_ = 0.2;

  // ---------------- 状态 ----------------
  std::mutex mutex_;
  CloudT::Ptr map_;
  pcl::KdTreeFLANN<PointT> map_tree_;
  bool map_tree_ready_ = false;
  PointT map_min_;
  PointT map_max_;

  // 地图最高点以下 global_search_height_ 内的“上部结构”子图，
  // 只用于全局搜索评分（不再包含地面/墙脚这些对 (x,y,yaw) 无信息量的点）。
  CloudT::Ptr map_top_;
  pcl::KdTreeFLANN<PointT> map_top_tree_;
  bool map_top_ready_ = false;

  pcl::IterativeClosestPoint<PointT, PointT> icp_;

  CloudT::Ptr latest_scan_;
  std::size_t scan_seq_ = 0;
  std::size_t last_processed_seq_ = 0;

  nav_msgs::msg::Odometry latest_odom_;
  bool has_odom_ = false;

  Eigen::Matrix4f T_map_odom_ = Eigen::Matrix4f::Identity();
  bool has_map_to_odom_ = false;
  bool localized_ = false;
  bool global_search_pending_ = false;
  int fail_count_ = 0;
  double last_score_ = 0.0;
  double last_mean_dist_ = 0.0;

  std::vector<int> nn_indices_{0};
  std::vector<float> nn_distances_{0.0f};

  // ---------------- ROS 接口 ----------------
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pub_pose_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_prior_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_aligned_scan_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_relocalize_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RelocalizationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
