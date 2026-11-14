# sam_bot_description 仿真架构说明

本文档详细说明仿真系统如何从**机器人 URDF 模型**出发，经过 **Gazebo 插件**、**FAST-LIO 激光惯性里程计**、**3D 点云重定位**，再到**建图（2D SLAM / 3D PCD）**与**导航规划（Nav2）**的完整链路，并解释各文件之间的调用关系、TF 关系与数据流通关系。

---

## 目录

1. [项目总览](#1-项目总览)
2. [机器人模型构建（URDF）](#2-机器人模型构建urdf)
3. [Gazebo 插件：传感器与执行器](#3-gazebo-插件传感器与执行器)
4. [FAST-LIO 里程计与 3D 重定位](#4-fast-lio-里程计与-3d-重定位)
5. [TF 坐标关系](#5-tf-坐标关系)
6. [建图（2D 栅格图 / 3D 先验点云）](#6-建图2d-栅格图--3d-先验点云)
7. [导航规划（Nav2）](#7-导航规划nav2)
8. [文件调用关系](#8-文件调用关系)
9. [数据流通关系总图](#9-数据流通关系总图)
10. [运行环境与性能（WSL2 / GPU）](#10-运行环境与性能wsl2--gpu)

---

## 1. 项目总览

**技术栈**：ROS2 Humble + Gazebo Classic（gzserver/gzclient）+ FAST-LIO（LiDAR+IMU 紧耦合里程计）+ fastlio_localization（3D 点云重定位）+ Nav2（DWB/DWA 局部规划器 + Navfn 全局规划器）。AMCL 与 robot_localization EKF 均已移除。

**目录结构**（工作区 `src/`）：

```
src/
├── sam_bot_description/                # 机器人描述 + 导航配置包
│   ├── package.xml / CMakeLists.txt    # 包依赖声明 / colcon 安装规则
│   ├── description/
│   │   └── sam_bot_description.urdf    # 机器人模型 + Gazebo 插件（xacro 格式）
│   ├── launch/
│   │   ├── navigation.launch.py        # 完整导航仿真（Gazebo + FAST-LIO + 重定位 + Nav2 + RViz）
│   │   ├── display.launch.py           # 模型显示 / 建图启动（Gazebo + RSP + FAST-LIO + RViz）
│   │   └── rtabmap_mapping.launch.py   # build_map.sh 默认建图算法（/scan + FAST-LIO 里程计）
│   ├── config/
│   │   ├── nav2_params.yaml            # Nav2 全套导航参数（无 amcl 段）
│   │   └── mapper_params_online_async.yaml  # slam_toolbox 在线建图参数（备选算法）
│   ├── rviz/
│   │   └── urdf_config.rviz            # display.launch.py 用的 RViz 配置
│   ├── world/
│   │   ├── my_carpark                  # 默认场景：约 20×20 m 停车场（围墙 + 隔墙/隔离墩 + 2 辆车）
│   │   ├── my_world.sdf                # 备选：10×10 m 方围墙 + 箱子/球
│   │   ├── my_newworld.sdf             # 备选：建模导出的楼宇（配套 model.sdf/model.config）
│   │   └── my_carrom/                  # 备选：车辆模型场景
│   ├── tools/
│   │   ├── save_map.py                 # 订阅 /map 存成 pgm + yaml（build_map.sh 与自动保存用）
│   │   ├── topic_ready.py              # 等某个话题出数据（带超时，脚本启动检查用）
│   │   └── generate_world.py           # 生成 world 场景的辅助脚本
│   └── map/
│       ├── my_map.yaml                 # 2D 地图元数据（分辨率/原点）
│       ├── my_map.pgm                  # 2D 地图栅格图像（占用网格）
│       └── prior_3d_map.pcd            # 3D 先验点云（重定位用）
├── fast_lio/                           # FAST-LIO 里程计包
│   └── config/
│       ├── mid360_sim.yaml             # 导航用参数（lidar_type=4）
│       ├── mid360_sim_2dmap.yaml       # 2D 建图用（点云全量参与配准，精度优先）
│       └── mid360_sim_mapping.yaml     # 3D 先验建图用（pcd_save_en / map_en）
├── fastlio_localization/               # 3D 点云重定位包（替代 AMCL）
│   ├── src/relocalization_node.cpp
│   ├── config/relocalization_params.yaml
│   └── tools/auto_drive_map.py, shift_pcd.py
└── mid360_simulation/                  # Livox Mid-360 仿真插件包
```

**依赖（`sam_bot_description/package.xml` 摘录）**：

| 依赖 | 用途 |
|------|------|
| `xacro` | 把 .urdf.xacro 展开成纯 URDF |
| `robot_state_publisher` | 从 URDF + joint_states 发布机器人 TF |
| `joint_state_publisher` | 发布非仿真关节状态（display 用） |
| `gazebo_ros` / `gazebo_plugins` | Gazebo 仿真与传感器/驱动插件 |
| `fast_lio` | LiDAR+IMU 紧耦合里程计，发布 odom→base_link |
| `fastlio_localization` | 3D 点云重定位，发布 map→odom |
| `nav2_bringup` / `nav2_rviz_plugins` | 导航栈与 RViz 导航插件 |
| `rtabmap_ros` | 2D 建图默认算法（`build_map.sh`，运行期不启动） |
| `slam_toolbox` | 2D 建图备选算法（`SLAM_ALGO=slam_toolbox ./build_map.sh`） |
| `tf2_ros` | TF 坐标变换 |

---

## 2. 机器人模型构建（URDF）

文件：`src/sam_bot_description/description/sam_bot_description.urdf`（xacro 格式，根元素带 `xmlns:xacro`）。

### 2.1 模型来源与坐标系约定

模型来自 OSURDF 导出的 CAD 网格（`meshes/visual/*.STL`），原生坐标是 **X 横向、Y 前进、Z 朝上**，质量/惯量由 CAD 直接给出（整车约 33 kg）。

ROS / Nav2 / 差速插件要求 `base_link` 的 **X 轴朝前、Y 轴朝左**，所以模型里做了两层坐标：

- `base_link`：标准 ROS 根坐标系（空 link，X 朝前）。
- `base_chassis`：CAD 原生坐标系，用固定关节 `chassis_joint`（`rpy="0 0 -π/2"`）挂在 `base_link` 下。所有 CAD 网格、车轮、关节都随它一起旋转 90°，外部看到的仍是标准 ROS 朝向。

整车包络约 **1.05 m(长) × 0.74 m(宽) × 1.08 m(高)**，驱动轮最低点在 `base_link` 系 z=−0.24，因此 `base_footprint` 放在 z=−0.24。

> **碰撞体**：`base_link.STL` 有 78 万个三角面，直接做碰撞会让 ODE 极慢，所以 visual 用 STL、collision 用简单几何体（两个长方体 + 车轮圆柱）。

### 2.2 link 与 joint 结构

机器人由以下 **link** 组成（`base_link` 为树根）：

| link | 质量 | 说明 |
|------|------|------|
| `base_link` | — | 空 link，ROS 标准根坐标系（X 朝前） |
| `base_footprint` | — | 空 link，地面投影点（z=−0.24，2D 导航基准） |
| `base_chassis` | 28.11 kg | CAD 车体本体（旋转 −90° 后的坐标系） |
| `new_link` / `new_link_1` | 2.147 kg | 左右驱动轮（后轴） |
| `new_link_2` / `new_link_4` | 0.0626 kg | 前部万向轮转向支架 |
| `new_link_3` / `new_link_5` | 0.162 kg | 前部万向轮小轮 |
| `livox` | — | Mid-360 激光雷达（STL 网格） |
| `livox_imu` | — | Mid-360 内置 IMU 坐标系 |
| `camera_link` | 0.035 kg | 深度相机安装点 |
| `camera_depth_frame` | — | 相机光学坐标系 |

**joint 连接关系**（所有 joint 由 `robot_state_publisher` 解析为 TF）：

| joint | 类型 | parent → child | 说明 |
|-------|------|----------------|------|
| `base_footprint_joint` | fixed | base_link → base_footprint | z 偏移 −0.24 |
| `chassis_joint` | fixed | base_link → base_chassis | 绕 Z 转 −90°，把 CAD 的 +Y 对到 ROS 的 +X |
| `new_joint` | **continuous** | base_chassis → new_link | 左驱动轮，转轴 (−1,0,0)（取反后正速度=前进） |
| `new_link_1_joint` | **continuous** | base_chassis → new_link_1 | 右驱动轮，转轴 (−1,0,0) |
| `new_link_2_joint` / `new_link_4_joint` | **continuous** | base_chassis → 支架 | 万向轮转向（绕 Z，被动） |
| `new_link_3_joint` / `new_link_5_joint` | **continuous** | 支架 → 小轮 | 万向轮滚动（被动） |
| `livox_joint` | fixed | base_link → livox | (0.346, 0, 0.93)，**整条 link 前倾 30°**（真实斜装，点云也倾斜；已抬高以避开前脸横杠） |
| `livox_imu_joint` | fixed | livox → livox_imu | CAD 外参 + `imu_rpy=(0,−π/6,0)` 反向旋回水平，使 IMU 与 base_link 对齐 |
| `camera_joint` | fixed | base_link → camera_link | (0.50, 0, 0.30)，车头前脸 |
| `camera_depth_joint` | fixed | camera_link → camera_depth_frame | 相机光轴旋转 |

> 6 个车轮/万向轮关节都是 `continuous`，其余为 `fixed`。Gazebo 物理仿真更新转角后，通过 `joint_state_publisher` 插件发布，供 `robot_state_publisher` 计算轮子 TF。
>
> **雷达安装**：CAD 网格原本画了一个 Mid-360（z=0.84~0.915），已从 `base_link.STL` 中删除，只保留 z=0.80~0.84 的安装底座板。实测 CAD 里的雷达装在一个 30° 斜面上、朝前进方向前倾 30°，故 `livox` link **整条前倾 30°**（`livox_joint` rpy=(0, π/6, 0)、原点 (0.346, 0, 0.93)），雷达点云也真实斜装发射。但 30° 前倾后向下波束会扫到车头前脸/横杠（碰撞盒 `collision_upper` 前上缘 x≈0.475、z≈0.80），把自身识别为障碍物、遮住前方点云，故把雷达整体抬高 0.051 m（底面 z=0.84→0.891）、倾斜角不变。为不破坏 FAST-LIO：IMU 坐标系用 `imu_rpy=(0,−π/6,0)` 反向旋回水平（与 `base_link` 对齐），雷达相对 IMU 的 30° 旋转通过 FAST-LIO 的 `extrinsic_R=Ry(30°)`、`extrinsic_T=(0.012534,−0.02329,0.04371)` 精确补偿（只描述雷达↔IMU 相对位姿，抬高后无需改动）。若 IMU 跟着 link 一起倾斜且外参不修正，`odom→base_link` 姿态会发散（实测 pitch 漂到 60°）。后方/侧方陡峭的向下波束仍会扫到 z≈0.80 的车顶，`pointcloud_to_laserscan` 的 `max_height` 设为 0.80，既滤掉自身回波又保留全部墙面点。

---

## 3. Gazebo 插件：传感器与执行器

URDF 里的 `<gazebo>` 标签在机器人被 spawn 进 Gazebo 时生效，每个 `<plugin>` 加载一个 Gazebo-ROS 桥接插件。共 **5 个插件**：

| 插件 | 加载库 | 发布/订阅 | 作用 |
|------|--------|----------|------|
| diff_drive | `libgazebo_ros_diff_drive.so` | 订 `/cmd_vel`，发 `/demo/odom` | 差速驱动 + 轮式里程计 |
| imu_sensor | `libgazebo_ros_imu_sensor.so` | 发 `/livox/imu`（200 Hz） | Mid-360 内置 IMU（角速度+线加速度） |
| mid360 点云 | `libmid360_plugin.so` | 发 `/livox/lidar`（PointCloud2，10 Hz） | Mid-360 非重复扫描 3D 点云 |
| camera（深度） | `libgazebo_ros_camera.so` | 发 `/depth_camera/*` | 深度相机 |
| joint_state_publisher | `libgazebo_ros_joint_state_publisher.so` | 发 `/joint_states`（30 Hz） | 轮子真实转角 |

### 3.1 diff_drive 差速驱动（关键）

```xml
<plugin name='diff_drive' filename='libgazebo_ros_diff_drive.so'>
  <ros>
    <namespace>/demo</namespace>
    <remapping>cmd_vel:=/cmd_vel</remapping>
  </ros>
  <left_joint>new_joint</left_joint>
  <right_joint>new_link_1_joint</right_joint>
  <wheel_separation>0.481</wheel_separation>
  <wheel_diameter>0.2</wheel_diameter>
  <publish_odom>true</publish_odom>
  <publish_odom_tf>false</publish_odom_tf>   <!-- 重要：不发布 odom->base_link TF -->
  <odometry_frame>odom</odometry_frame>
  <robot_base_frame>base_link</robot_base_frame>
</plugin>
```

要点：

- **订阅** `/cmd_vel`（`geometry_msgs/Twist`），把线速度 `vx` 和角速度 `wz` 换算成左右轮转速驱动轮子。
- **发布** `/demo/odom`（`nav_msgs/Odometry`），由轮子转角积分得到里程计（x、y 位置 + 航向 yaw）。
- `publish_odom_tf=false`：**故意不让 diff_drive 发 odom→base_link 的 TF**，这段 TF 改由 FAST-LIO 发布（见第 4 节），用 LiDAR+IMU 代替轮式里程计。
- **已知残余漂移**：`cmd_vel=0` 时 diff_drive 只把目标轮速设为 0、不抱死，车体会以约 **13 cm/min** 缓慢滑行（无指令）。FAST-LIO 实时跟随、不影响定位，但车在 Gazebo 里「停不死」。如需精确驻停，给驱动轮关节加大 `damping` 或加抱死摩擦。详见 [运行文档 §6.14](运行文档.md)。

### 3.2 IMU 传感器

- 内置于 Mid-360，命名空间 `/`，发布 `/livox/imu`（`sensor_msgs/Imu`，200 Hz）。
- 配置了高斯噪声（角速度 stddev 1e-5、线加速度 stddev 1e-4），模拟真实 BMI088 IMU 误差。

### 3.3 激光雷达

- 发布 `/livox/lidar`（`sensor_msgs/PointCloud2`，10 Hz），`frame_id: livox`，由 `libmid360_plugin.so` 按 `mid360.csv` 定义的非重复扫描轨迹生成 3D 点云，量程 0.1~15.0 m。
- Nav2 只认 2D 激光，故 launch 里另起 `pointcloud_to_laserscan` 节点把 `/livox/lidar` 投影成 `/scan`（`LaserScan`，frame `base_link`，10 Hz，量程 0.1~15.0 m）。投影薄片 `min_height −0.05` / `max_height 2.50`：
  - 下限必须高于地面（地面在 `base_link` 系约 −0.24 m），否则地面环被投影成假墙；
  - 雷达 30° 前倾时约 62% 的射线落在 15 m 内的地面上，落在薄片内的只有约 38%，360 束里有效回波 153~166 束（水平安装时可达 258~276 束）；
  - `use_inf: false`：无回波方向填 `range_max + inf_epsilon = 16.0`，而不是 `+inf`。slam_toolbox 会自己丢掉 inf，但 RTAB-Map 把 LaserScan 转点云时会把 inf 带进去，建到几百个节点后内存爆掉（实测 ~350 节点时 OpenCV 分配 132 GB 后崩溃）。

---

## 4. FAST-LIO 里程计与 3D 重定位

定位链路分两层（取代原来的 EKF + AMCL）：

```
/livox/lidar + /livox/imu ──► FAST-LIO ──► TF: odom→base_link（10 Hz）
                                    └────► /cloud_registered（odom 系去畸变点云）
                                                    │
prior_3d_map.pcd ──► fastlio_relocalization ──► TF: map→odom（10 Hz）
```

- **里程计层**（`fast_lio`）：LiDAR+IMU 紧耦合 iEKF，输出高频、低漂移的 `odom→base_link`；
- **全局定位层**（`fastlio_localization`）：把实时点云与先验 PCD 做 ICP，求 `map→odom`。

> 除 `/cloud_registered` 外 FAST-LIO 还发 `/cloud_registered_body`，那是
> **IMU 体系**（`livox_imu`）的点云，两张代价地图的障碍层都用它（传感器原点
> 就是车上的雷达，见 7.2）。`header.frame_id` 必须是 `livox_imu`——早先误标成
> `base_link`，nav2 会按 `body_to_base_T` 的 z（0.8863 m）把整帧点云再压低一次，
> 障碍全部掉出高度带，详见《运行文档》6.13 第 3 条。

### 4.1 为什么用 FAST-LIO 替代轮式里程计 + EKF

`diff_drive` 只输出轮式里程计，长时间会漂移且没有绝对航向参考。原方案用 EKF 融合 IMU 的 yaw 只能部分缓解；FAST-LIO 直接对 LiDAR 点云做**帧到地图的迭代误差状态卡尔曼滤波**，在纯旋转、打滑时也比轮式里程计稳，同时给出 3D 点云地图。AMCL 依赖稀疏的 2D `/scan` 与栅格地图，在正方围墙 + 少量障碍物的世界里角度歧义明显；改用 3D 点云重定位后不再需要人工给初始位姿。

### 4.2 FAST-LIO 配置要点（`fast_lio/config/mid360_sim.yaml`）

| 参数 | 值 | 说明 |
|------|-----|------|
| `common.lid_topic` / `imu_topic` | `/livox/lidar` / `/livox/imu` | 输入 |
| `preprocess.lidar_type` | 4 | 标准 `PointCloud2` 输入（Livox 自定义格式用 1） |
| `preprocess.scan_rate` | **1000**（仿真）/ 10（真机） | 仿真雷达每帧是"瞬时快照"，没有 100 ms 扫描过程；`mid360_handler` 却按 `Δ方位角 / (0.361 × scan_rate)` 反推每点时间，`scan_rate: 10` 会把整帧判成跨越 99.7 ms，`lidar_end_time` 比真实拍摄时刻晚 0.1 s，FAST-LIO 给 T 时刻的姿态打上 T+0.1 的戳，转向时地图被抹歪（实测偏航误差 7.3° → 0.67°） |
| `mapping.gyr_cov` | **0.01** | 仿真陀螺几乎无噪声；默认 0.1 会让纯旋转 yaw 明显低估 |
| `mapping.extrinsic_R` | Ry(+30°) | 雷达物理前倾 30°，IMU 水平安装，补偿这 30° |
| `mapping.extrinsic_T` | `[0.0125, −0.0233, 0.0437]` | IMU 在雷达系中的位置（反向旋转后） |
| `mapping.body_to_base_T` | `[0.3335, 0.0233, 0.8863]` | IMU 在 `base_link` 中的位置。FAST-LIO 状态是 IMU 位姿，发布 `odom→base_link` 时右乘 `T_base_imu⁻¹` 换算成真正的车体原点，否则 RViz/Nav2 足迹整体前移 0.33 m |
| `mapping.body_to_base_R` | 单位阵（9 元素行主序） | IMU 与 `base_link` 姿态一致（`livox` 的 +30° 被 `livox_imu` 的 −30° 抵消）。缺省为 `I`/`0`，向后兼容 |
| `publish.dense_publish_en` | **true** | `/cloud_registered` 发去畸变全帧（~590 点）而非降采样特征（~100 点），重定位 ICP 更稳 |
| `pcd_save.pcd_save_en` | `false`（导航）/ **`true`**（建图配置） | 建图时把累积点云存成 PCD |

### 4.3 3D 重定位算法（`fastlio_localization/relocalization_node.cpp`）

节点是纯 C++（PCL），核心流程：

1. **加载先验地图**：`pcl::io::loadPCDFile` 读入 `prior_3d_map.pcd`（`PointXYZI`），体素下采样到 0.15 m；同时预计算地图的最高点 `map_max_.z`，并提取 `z >= map_max_.z - global_search_height` 的**上部结构子图**（本项目 0.35 m：墙顶、箱顶、球顶），供全局搜索评分用。
2. **z 初值估计**：FAST-LIO 的 odom 高度会漂移（实测可达 2 m 级），过期的 `map→odom` z 会让 ICP 在 z 方向直接跟丢。节点用**点云 z 分布中位数之差**估计 `map→odom` 的 z；该估计与场地/视角有关，有 0.3~0.6 m 的系统偏差，因此用成功跟踪帧上的真实对齐结果做 EMA 校准（`z_median_bias_`）。
3. **全局搜索**（启动时或 `/relocalize`）：在 (x, y, yaw) 三维栅格上以 0.5 m / 15° 步长枚举候选，用**上部结构**的点云做快速内点率评分，取 `global_search_top_k`（8）个候选；z 用上一步的估计值做初值。
4. **ICP 精配准**：对每个候选用完整实时帧做 `pcl::IterativeClosestPoint`（对应距离 1.5 m），再用上部结构重新评分排序。若第二候选分数 ≥ 最优的 90% 且位姿不同，打印**歧义告警**。
5. **结果验收**（三道闸，任一不满足都保持旧 `map→odom` 并重试）：
   - 内点率 ≥ `min_inlier_ratio`（0.55）；
   - 平均最近邻距离 ≤ `max_relocalize_mean_dist`（0.30 m）——只靠内点率挡不住“大片点近似贴墙、平均距离 0.49 m”的误匹配，实测这种解会把 `map→odom` 拉偏 3 m；
   - 相对搜索开始时 `map→odom` 的**平面**位移 ≤ `max_relocalize_jump`（2.0 m），除非内点率 ≥ `strict_relocalize_score`（0.85）——兼顾“机器人被搬动后重新定位”。
6. **逐帧跟踪**：以上一帧 `map→odom`（z 初值按第 2 步修正）做 ICP（对应距离 0.7 m），内点率达标且相对上一帧的**平面**平移 < 1.0 m、yaw < 0.6 rad 才接受；结果按 `correction_alpha`（0.6）平滑。z 初值被修正过时，还会用**原始初值**再跑一次 ICP，取评分更高的结果（双初值取优）——中位数估计有抖动，双初值保证修正只会帮忙、不会帮倒忙。
7. **失败恢复**：连续 5 帧失败自动重新触发全局搜索。
8. **平面约束**：地面机器人只保留 x/y/z 平移和 yaw（`constrain_planar`，roll/pitch 归零）。z 必须保留：FAST-LIO 的高度漂移靠它吸收；Nav2 的 2D 代价地图与覆盖规划只看 x/y/yaw，z 平移没有副作用。
9. **输出**：10 Hz 发布 `map→odom`（时间戳 `now + 0.2 s`）、`/localization_pose`、`/aligned_scan`（配准后的点云）；`/prior_map` 以 transient_local QoS 发布先验地图；`/relocalize`（`std_srvs/Trigger`）可手动触发。

> 启动时先发布单位阵 `map→odom`，保证 Nav2 生命周期管理阶段 TF 链完整。

---

## 5. TF 坐标关系

### 5.1 完整 TF 树

```
map ──► odom ──► base_link ──► base_footprint
                        │
                        ├──► drivewhl_l_link
                        ├──► drivewhl_r_link
                        ├──► front_caster
                        ├──► livox_imu
                        ├──► livox
                        └──► camera_link ──► camera_depth_frame
```

### 5.2 每段 TF 的发布者

| TF | 发布者 | 来源 |
|----|--------|------|
| `map → odom` | **fastlio_relocalization** | 实时点云与先验 PCD 做 ICP，得到全局定位 |
| `odom → base_link` | **FAST-LIO**（`fastlio_mapping`） | LiDAR + IMU 紧耦合 iEKF |
| `base_link → 所有子 link` | **robot_state_publisher** | 解析 URDF 的 joint |

> FAST-LIO 的 `base_link` 实际就是 IMU 位姿（雷达位置约在物理 base_link 前方 0.346 m），重定位节点求出的 `map→odom` 也以这个 `base_link` 为参考，两者一致，Nav2 无需关心偏移。

### 5.3 base_link vs base_footprint

- `base_link`：车体本体坐标系（URDF 树根）。
- `base_footprint`：base_link 正下方 −0.15 m 处的地面投影点，是 2D 导航的"机器人中心"。
- 重定位节点以 `base_link` 为参考帧，代价地图/控制器用 `robot_base_frame: base_link`，两者一致。

### 5.4 坐标意义

| 坐标系 | 含义 |
|--------|------|
| `map` | 全局固定世界坐标系（与 2D 地图 / 3D 先验点云对齐） |
| `odom` | FAST-LIO 里程计坐标系（启动时与 map 重合，之后缓慢漂移） |
| `base_link` / `base_footprint` | 机器人本体坐标系 |

重定位节点发布的 `map → odom`，就是"把会漂移的 odom 坐标系对齐到全局 map 坐标系"的那层校正；导航时的定位误差主要来自这一层。

---

## 6. 建图（2D 栅格图 / 3D 先验点云）

> 说明：导航运行期不启动任何 SLAM 节点，以下两条离线链路分别生成本项目用到的两份地图。

### 6.1 2D 建图流程（`build_map.sh`）

默认用 **RTAB-Map** 建图，`SLAM_ALGO=slam_toolbox ./build_map.sh` 可切回 slam_toolbox：

1. 用 `display.launch.py` 启动 Gazebo + `robot_state_publisher` + FAST-LIO（**`mid360_sim_2dmap.yaml`**）+ `/scan` 投影，Gazebo 窗口关闭（`gui:=False`，RViz 保留）；
2. 启动建图节点：
   - 默认 `rtabmap_mapping.launch.py`：订阅 `/scan` 和 FAST-LIO 里程计，发布 `/map` 与 `map → odom`；
   - 切到 slam_toolbox 时改用 online async 模式（参数 `mapper_params_online_async.yaml`）；
3. 手动控制机器人（如 `teleop_twist_keyboard` 发 `/cmd_vel`）遍历环境；
4. 用 `tools/save_map.py` 订阅 `/map` 存成 pgm + yaml；脚本另有每 20 s 一次的后台自动保存，Ctrl+C 中断最多丢 20 s 的建图成果。

> 2D 建图专用配置 `mid360_sim_2dmap.yaml` 与导航默认的 `mid360_sim.yaml` 只差点云密度四项（`point_filter_num 3→1`、`filter_size_surf 0.5→0.2`、`filter_size_map 0.5→0.2`、`max_iteration 3→4`）：仿真插件每帧只有 2500 条射线，默认配置再"每 3 点取 1 + 0.5 m 体素"后每帧只剩约 800 点，ICP 约束弱、里程计漂移大。实测方形路线 16.7 m，漂移从 0.225 m 降到 0.06~0.12 m，slam_toolbox 的修正跳变和"地图轮廓叠轮廓"重影随之减轻。

### 6.2 3D 先验点云流程（`build_3d_map.sh`）

1. 用 `mid360_sim_mapping.yaml` 启动 FAST-LIO（打开 `pcd_save_en` 与 `publish.map_en`）；
2. 用 `fastlio_localization/tools/auto_drive_map.py` 或键盘遥控遍历环境，FAST-LIO 在 `odom` 系累积点云并发布 `/Laser_map`；
3. 调用 `/map_save` 服务，把累积点云写成 `src/sam_bot_description/map/prior_3d_map.pcd`；
4. **不做平移**：FAST-LIO 保存的点云在 `odom` 系，原点是建图起点的 IMU 位姿；而 slam_toolbox / RTAB-Map 建 2D 地图时 `map → odom` 初始为单位阵，2D 地图的 `map` 原点**同样是**建图起点的 IMU。两者天然对齐，再按 IMU 安装偏移平移反而会让重定位输出的 `map → odom` 多出 0.33 m（实测 `map → base_link` 从正确的 (−0.305, −0.022) 变成 (0.012, −0.016)，表现为 RViz 里没撞、Gazebo 里撞）。`tools/shift_pcd.py` 的默认平移已停用，`build_3d_map.sh` 也不再调用它，只保留手工调试用途；
5. PCD 与 2D 栅格图共用"建图起点的 IMU"作为原点，也就是导航时的 `map` 系。

> 2D 地图与 3D 先验点云必须来自**同一次起点**的建图（都在世界原点附近出发），否则 `map` 不一致。详见 [运行文档 §3.4](运行文档.md)。
>
> 自检：机器人静止、刚启动时 `tf2_echo map base_link` 应约为 **x ≈ −0.31~−0.33、y ≈ −0.02**（base_link 在建图起点 IMU 后方 0.33 m）。注意 `map` 原点是建图起点的 IMU、不是 Gazebo 世界原点，所以不能直接和 `gz model -m sam_bot -p` 的绝对值比。若 x ≈ +0.01，说明 PCD 被多余地平移过一次，需重建或手工反向平移。

### 6.3 2D 地图文件（`src/sam_bot_description/map/my_map.yaml`）

```yaml
image: my_map.pgm
mode: trinary
resolution: 0.05          # 每像素 5 cm
origin: [-13.0481, -13.2706, 0.0]  # 地图左下角在 map 坐标系中的位置
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

- 地图是 **420×426** 像素、0.05 m/pix，覆盖约 **21.0 m × 21.3 m**；
- `origin` 使地图覆盖 map 坐标系约 `[-13.05, 7.95] × [-13.27, 8.03]`；
- 建图场景是 `world/my_carpark`（约 20×20 m 停车场），机器人出生在世界坐标 **(2.0, 2.7)**，与 `display.launch.py` / `navigation.launch.py` 的 spawn 参数一致；
- `map` 原点 = 建图起点的 IMU 位姿（见 §6.2），因此不是 Gazebo 世界原点，数值上不要拿世界坐标直接对照。

---

## 7. 导航规划（Nav2）

导航由 `navigation.launch.py` 调用 `nav2_bringup` 的 `navigation_launch.py` 启动（**不含 AMCL**），参数全部在 `src/sam_bot_description/config/nav2_params.yaml`。`map_server` 单独启动，并由一个独立的 `lifecycle_manager_localization` 管理（延迟 5 s 启动，避开启动阶段的 DDS 发现竞态）。

### 7.1 Nav2 各节点职责

| 节点 | 插件/算法 | 职责 |
|------|-----------|------|
| `map_server` | — | 读取 `my_map.yaml`，发布 `/map`（静态地图，只给 global_costmap） |
| `fastlio_relocalization` | 3D ICP | 用 `/cloud_registered` + 先验 PCD 定位，发布 `map→odom` TF |
| `planner_server` | NavfnPlanner | 全局路径规划（在全局代价地图上 A*/Dijkstra） |
| `controller_server` | DWB（DWA） | 局部路径跟踪，输出速度指令 |
| `smoother_server` | SimpleSmoother | 平滑全局路径 |
| `behavior_server` | spin/backup/wait 等 | 恢复行为（卡住时后退/旋转） |
| `bt_navigator` | 行为树 | 编排整个导航流程 |
| `velocity_smoother` | — | 平滑/限幅速度指令 |
| `waypoint_follower` | — | 多航点任务（可选） |
| `lifecycle_manager_*` | — | 管理上述节点生命周期 |

> `collision_monitor`、`docking_server`、`loopback_simulator` 在参数文件里有配置段，但**默认的 bringup_launch.py 并未加载它们**（运行 `ros2 node list` 里没有这几个节点）。

### 7.2 两个代价地图

| 代价地图 | 全局/局部 | 坐标系 | 更新频率 | 图层 |
|----------|-----------|--------|----------|------|
| `global_costmap` | 全局 | `map` | 1 Hz | static_layer + obstacle_layer + inflation_layer |
| `local_costmap` | 局部（滚动） | `odom` | 5 Hz | obstacle_layer + inflation_layer |

- 两者都用**两个观测源**：`/scan`（`LaserScan`）+ 一路 FAST-LIO 配准点云
  （`PointCloud2`，约 10 Hz），`marking`/`clearing` 都打开：
  - `/scan` 由 `/livox/lidar` 投影而来，已经把地面按高度滤掉，标记可靠；
  - 点云源每帧上千点、覆盖整个前方，作为**稠密清除射线**。
    只用 `/scan` 时，30° 前倾使 360 束里只剩 120 多束有效回波，障碍物拖走后
    没有任何射线穿过的栅格永远清不掉 —— 这正是"障碍物移开了、代价地图上的
    障碍包络却一直留着"的原因。加上点云源后实测 4~6 s 内包络连同膨胀全部消失。
  - **两张图都用 `/cloud_registered_body`（`livox_imu` 系点云，FAST-LIO 的
    IMU 体系）**。nav2 做标记和射线清除时把**点云所在坐标系的原点**当作
    传感器原点：`/cloud_registered` 的 frame 是 `odom`，原点是 FAST-LIO 的
    起始位姿，车一开出去就落在 4×4 m 滚动窗口外，整条观测被 `continue`
    跳过（日志刷 `Sensor origin at (0.00, 0.00) is out of map bounds`），
    局部图彻底失明；全局图虽然原点在 map 内，但清除射线会从一个错误的位置
    发散，把沿途的真实墙一起清掉。`livox_imu` 的原点就是车上的雷达，
    标记/清除都正确。
  - **`/cloud_registered_body` 的 frame 必须是 `livox_imu`，不能是
    `base_link`**：FAST-LIO 的 `publish_frame_body` 用
    `RGBpointBodyLidarToIMU` 把点转到了 IMU 系，早先后缀被错标成
    `base_link`。URDF 里 IMU 比 `base_link` 高 0.89 m，nav2 于是用
    `odom←base_link`（z ≈ -0.96）而不是 `odom←livox_imu`（z ≈ -0.07）去
    变换，整帧点云被压低 0.89 m，地面从 -1.15 变成 -2.04、1 m 高的箱子
    从 `[-1.15,-0.15]` 变成 `[-2.04,-1.04]`，全部落进
    `min_obstacle_height: -0.85` 以下被静默丢掉 —— 表现就是"拖进来的
    障碍物检测不到、直接撞上去"。已在 `laserMapping.cpp` 里改回
    `livox_imu`，变换后的点云与 `/cloud_registered` 完全重合。
  - 点云源用 `min_obstacle_height: -0.85` 把地面（odom/map 系 z ≈ -1.15）
    滤掉，否则整片地面会被标成障碍；上限 2.0 m 覆盖箱子和墙。
    注意 nav2 的 `ObservationBuffer` 是**先把点云变换到代价地图的
    `global_frame`、再按 z 过滤**，所以这个高度带永远是"odom/map 系"的
    数值，不是点云自己坐标系里的数值。
  - 清除射程局部图取 6 m（窗口只有 4×4 m）、全局图也取 6 m：全局图 21×21 m，
    若整幅扫描，单次更新会超过 `update_frequency` 周期，缓冲区越积越多。
- 两张图的 `transform_tolerance` 都设成 **1.0 s**（默认 0.3）：
  FAST-LIO 的 `odom→base_link` 用雷达帧时间打时间戳，但要等该帧处理完才
  发出去，实测比仿真时钟晚 0.3~0.6 s。nav2 的 `ObstacleLayer` 把这个值当作
  `tf2_ros::MessageFilter` 的缓存时长，默认 0.3 s 时时间戳≈当前时刻的
  `/scan` 等不到迟到的 TF，被判
  `the timestamp on the message is earlier than all the data in the transform cache`
  后整条丢掉（实测 10 分钟丢 200+ 条），局部图收不到任何激光点。改 1.0 s
  后丢弃条数降到 0。
- 全局代价地图叠加了 `/map` 静态地图，用于全局规划；
- 局部代价地图只保留机器人周围 4×4 m 的滚动窗口，用于局部避障；
- 局部图原来用的是 `VoxelLayer`，但它把 z 索引塞进 uint32 的低/高 16 位、
  `z_voxels` 上限 16，`origin_z`/`z_voxels` 一配错就会**静默**丢掉所有点
  （表现为局部图恒为 0/6400、怎么拖障碍物都没反应）。`/scan` 本来就是
  一平面 2D 扫描，没有立体信息，因此改成和全局图一样的 `ObstacleLayer`，
  只用 `min/max_obstacle_height` 做高度过滤，不再有隐式 z 窗口。
- 两个**图层级**的 `min_obstacle_height` 都是 `-2.0`：FAST-LIO 的 odom 原点在
  IMU 起始位姿上（离地约 1.15 m），`base_link` 在 odom 系 z ≈ -0.96，
  `/scan` 的点也在这一高度，用默认的 0.0 会把所有点当"地面以下"丢掉。
  注意高度过滤执行两次：`ObservationBuffer` 先按**数据源级**参数过滤一遍，
  `updateBounds` 再用**图层级**参数过滤一遍，两级都必须放行。

### 7.3 速度控制链（已实测核实）

```
controller_server ──/cmd_vel_nav──► velocity_smoother ──/cmd_vel──► diff_drive(Gazebo)
behavior_server(恢复行为) ──────────────────/cmd_vel──► diff_drive
```

- `controller_server` 发布 **`/cmd_vel_nav`**（DWB 原始输出）；
- `velocity_smoother` 订阅 `/cmd_vel_nav`，做加减速限幅后发布 **`/cmd_vel`**；
- `behavior_server` 的恢复行为（spin/backup 等）**直接发布 `/cmd_vel`**，绕过 smoother；
- Gazebo 里的 `diff_drive` 订阅 **`/cmd_vel`**。

### 7.4 导航一次完整的数据流

1. RViz 点击"2D Nav Goal" → 发 `NavigateToPose` action 给 `bt_navigator`；
2. `bt_navigator` 调 `planner_server` 的 `ComputePathToPose`；
3. `planner_server` 在 `global_costmap` 上规划出全局路径 → 返回 `bt_navigator`；
4. `bt_navigator` 把路径通过 `FollowPath` action 发给 `controller_server`；
5. `controller_server`（DWB）结合 `local_costmap` 和路径，计算出速度指令 → `/cmd_vel_nav`；
6. `velocity_smoother` 平滑限幅 → `/cmd_vel`；
7. Gazebo `diff_drive` 执行速度，机器人移动；
8. 轮子转动 → `/joint_states`；IMU → `/livox/imu`；雷达 → `/livox/lidar`；
9. FAST-LIO → `odom→base_link` TF + `/cloud_registered`；重定位节点做 ICP → `map→odom` TF；
10. 代价地图更新、机器人到达目标（误差 < `xy_goal_tolerance` + `yaw_goal_tolerance` 即判定成功）。

### 7.5 全覆盖清扫任务（`coverage_mission`）

面向"地下车库洗地机器人"的整区覆盖任务。`navigation.launch.py` 加
`coverage:=true` 时会额外拉起 `coverage_mission/launch/coverage.launch.py`，
其中包含两个节点：

| 节点 | 来源 | 职责 |
|------|------|------|
| `coverage_server` | `opennav_coverage`（Fields2Cover 2.1.1） | 提供 `compute_coverage_path` action，把区域多边形规划成有序 swath |
| `coverage_executor` | `src/coverage_mission` | 任务状态机：去起点 → 规划 → 逐条 swath 转场/跟踪 → 避障回轨 |

参数在 `src/coverage_mission/config/coverage_params.yaml`，覆盖区域在
`config/coverage_area.yaml`（由 `area_extractor.py` 从 `my_map.pgm` 提取，
外环 68 点、5 个孔洞对应车库里的柱子）。

**规划阶段**

- 区域多边形已按车体尺寸内缩，**不生成 headland / 分解**，直接在多边形上
  生成 swath（`generate_headland=false`、`generate_decomp=false`），
  `operation_width=0.8` 即洗盘宽度 = 相邻 swath 间距；
- **规划时忽略孔洞**（`plan_ignore_holes=true`）：Fields2Cover 对"外环 + 孔洞"
  的多边形会按子区域分别生成条带，各子区域锚点不同，实测出现 0.1~0.5 m 的
  重复行与 0.2~1.0 m 的漏扫行——34 条/273 m，按 0.8 m 宽度栅格化的覆盖率只有
  81%（核心区 85%）。去掉孔洞后条带是整场统一的 0.8 m 网格：23 条直条带，
  覆盖率 94%、核心区 98%；
- **执行前按孔洞裁剪**（`clip_swaths_at_holes=true`，`hole_clearance=0.15`）：
  规划忽略孔洞是为了网格统一，但条带不能真的开进柱子/结构物。执行器沿每条
  swath 每 0.05 m 采样，把落在孔洞内或距孔洞边界不足 0.15 m 的部分裁掉，
  剩下的连续段逐段执行（段内 `FollowPath`，段间 `NavigateToPose` 绕过去）。
  区域多边形已让孔洞比真实障碍外扩约 0.55 m，所以断点处车头（前伸 0.545 m）
  离障碍仍有约 0.15 m。本场地 23 条 → 37 段、总长 326.8 → 282.3 m；
  临时出现的障碍物（如测试箱子）仍由执行期前视检查绕行回轨处理——
  这正是需求里"避让后回到原轨道"要处理的情况。实测一次完整任务
  （run11）：37/37 段全部执行完成，0 跳过、0 终止；静态柱子全部由
  预裁剪绕开，只发生 3 次段末停滞兜底（均 1 次 tick 内立即回轨）；
- 只请求 `generate_route=true`，不请求 `generate_path`：DUBIN 生成的连接段
  会被标成 swath 混进列表，变成横穿车库的斜线；
- 本车库实测：23 条 swath，`compute_coverage_path` 往返耗时约 0.1 s。

**执行状态机**（`coverage_executor.py`）

```
WAIT_READY → START_NAV_SEND/WAIT
           → PLAN_SEND/WAIT
           → 每条 swath: TRANSIT_SEND → TRANSIT_PLAN_WAIT → EXEC_SEND/WAIT
                         ↳ 被挡/停滞: DETOUR_CANCEL_WAIT → DETOUR_PLAN_WAIT
                           → 回轨后继续 EXEC
           → DONE
```

- **去清扫起点**：机器人不在起点时先 `NavigateToPose` 到
  `coverage_area.yaml` 里的起点（本图 `[-2.173, -4.996]`），到达后才请求规划；
- **弧形换轨**（`arc_connector=true`，优先）：换轨不再走点到点规划，而是
  以当前车头方向和下一条带的行进方向为起终切向生成曲线（弓字形的 180°
  掉头用半圆，其余用三次贝塞尔半椭圆），再接上条带，
  **一条 `FollowPath` 直接跟踪**。起点切向就是车头方向，因此不需要
  "先停、原地转向、再起步"；终点切向就是条带方向，并入时是相切的。
  曲线只由条带间距/朝向生成，与具体地图无关；下发前用代价地图的
  致命栅格（`arc_lethal_threshold=100`）+ 车体半宽
  （`arc_clearance=0.35 m`）校验，贴到障碍物/地图未就绪才回退。
  合并路径发布在 `coverage/transit_path`，RViz 里可直接看到。
  - **提前转弯**（`turn_inset_ratio=0.5`、`turn_inset_max=0.6 m`）：如果下一条
    swath 的起点就在本段末端附近（典型 U 形换轨），当前条带在末端前
    `0.5×横向间距` 处结束跟踪，机器人从一个更靠里的点开始画弧，弧顶不会
    越过条带端头顶进障碍物膨胀层；纵向已经错开的换轨（跨孔洞/斜向）自动
    不提前，避免额外漏扫。
  - **对称 U 形掉头**：掉头时弧线终点不硬拉到贴近墙面的下一条带真正起点，
    而是取"与起点同一纵向坐标、横向一个车道间距"的点，用真正的半圆
    （半径 = 半个车道间距，0.8 m 车道时 0.4 m ≥ `min_turning_radius`）
    连接；下一条 swath 从半圆终点对应的 `s` 开始跟踪，端头那一小段由
    半圆扫过的 0.8 m 清洗带覆盖。这样弧顶只到"提前停轨点 − 0.4 m"，
    距条带端点还有余量，既不贴墙也不进膨胀层；曲率恒定，比贝塞尔更容易
    让控制器平稳跟随。横向间距 < 0.7 m（半径低于最小转弯半径）时自动
    退回贝塞尔换轨。
  - **掉头速度**：局部规划器已从 MPPI 换成 DWB（DWA）。MPPI 时期掉头曾慢到
    16 s 才走完 1.26 m 半圆（均速 0.078 m/s），根因是采样标准差过小——半圆
    要求持续 `vx≈0.5、wz≈1.25`，对应 2.5σ/3.1σ，白噪声几乎采不出能跟住
    半圆的轨迹，softmax 加权平均退化成"原地极慢转"；当时靠把 `vx_std/wz_std`
    放大到 0.4/0.8 才降到约 4 s。DWB 的速度采样是均匀网格（0.5/19≈0.026 m/s、
    3.8/19≈0.20 rad/s），不存在采样不到的问题，`max_vel_theta=1.9` 足以覆盖
    半径 0.4 m 半圆所需的 1.25 rad/s。
  - 距离 > `arc_connector_max_length`（3.0 m，TSP 跨区跳跃）或弧线不可行时，
    回退到下面的规划器转场；
- **规划器平滑转场**（`smooth_transit=true`，回退）：`TRANSIT_SEND` 调用
  planner_server 的 `compute_path_to_pose`（`use_start=false`，NavFn，
  实测 0.05~0.1 s）规划一条到条带起点的路径，再把从起点到终点的条带按
  0.1 m 间隔拼到路径末尾，**只发一条 `FollowPath`**（`coverage_goal_checker`）。
  DWB 沿路径拐点自然并入条带，全程没有停车转向。
  - 规划失败/超时（`plan_timeout=10 s`）自动回退 `NavigateToPose`
    （`general_goal_checker` 0.6 m / 0.6 rad）；
  - 距离小于 `transit_tolerance`（0.6 m）时仍跳过转场，直接交给
    `FollowPath` 顺带走过去——实测（MPPI 时期）在"小距离 + 大角度"终点会反复
    触发进度检查失败；
  - 合并路径前半段是转场，机器人可能离当前条带很远：只有横向距离
    ≤ `on_line_tolerance`（1.0 m）才做前视障碍/停滞检查，避免把转场
    途中路过的柱子误判成"当前条带被挡"；转场段停滞 5 s 回退
    `NavigateToPose`，连续两次则跳过该条带;
- **逐条跟踪**：`FollowPath` + `coverage_goal_checker`（0.12 m / 0.10 rad），
  BT 里必须显式写 `goal_checker_id`（有两个 goal checker 时空字符串会报错）。
  终点容差 0.12 m：条带终点离墙只有约 0.3 m，控制器在膨胀层附近压不进 5 cm，
  实测每条 swath 都在终点 0.07~0.10 m 处停住，5 s 后被"进度停滞"兜底误判成
  绕行（多花 5 s 且发出一条假的 `blocked_interval`）；
- **避障回轨**：见下节。

**避障与"回到同一条轨道"**

`SwathSupervisor` 订阅 `global_costmap/costmap`，沿当前 swath 前视
`lookahead=3.0 m`，在以 swath 为中心、半宽 `corridor_half_width=0.25 m`
的走廊里采样代价，只有 `>= lethal_cost_threshold(100)` 才算被挡
（避开膨胀层的 253 内接值误报）：

1. `find_rejoin` 从障碍物之后继续扫描，找一段连续 `clear_run=1.4 m`
   的干净走廊；
2. 回轨点取这段干净走廊起点再往后 `rejoin_margin=1.3 m`，**严格落在原
   swath 直线上**（不是回到原点，也不是回到障碍物前的那个点）；
3. 执行器取消 `FollowPath`，用同一套平滑机制回轨：先
   `compute_path_to_pose` 规划到回轨点的路径，再把回轨点之后的剩余条带
   拼上去，发一条 `FollowPath`（规划失败才回退 `NavigateToPose`）；
   成功后把 `_swath_start_s` 设为回轨点的弧长 s，**继续跟踪同一条
   swath 的剩余段**；
4. 找不到回轨点 / 绕行导航失败 / 单条 swath 绕行超过
   `max_detours_per_swath=3` 次 → 跳过该 swath 剩余段，继续下一条，
   任务不会因为一个障碍物整体中断。

**障碍密集区跳扫**（`dense_obstacle_skip=true`）

静态孔洞由预裁剪处理，但孔洞之外还可能有噪声、堆放物等密集障碍。
`SwathSupervisor.scan()` 以 0.1 m 步长扫描走廊，返回被挡比例和连续
被挡区间，执行器据此做三档决策：

- **转场前预检**：整条 swath 被挡比例 ≥ `dense_swath_ratio`（0.5）
  → 整条跳过，根本不开过去（换轨道）；
- **执行中**：剩余段被挡比例 ≥ 0.5 → 跳过剩余段，换下一条 swath；
- **前方障碍群**：`dense_scan_ahead`（6 m）内存在连续长度
  ≥ `dense_cluster_length`（2 m）的障碍群 → 直接跳到障碍群之后
  `rejoin_margin` 处，从那里继续清扫（换到轨道的某一点开始）；
- 不满足以上条件才走原来的"单障碍绕行回轨"逻辑，避免一个障碍物一个
  障碍物地反复绕行、把时间耗在密集区里。

> `rejoin_margin` 不能太小：2D 代价地图只能看到障碍物**朝向机器人一侧的
> 表面**，回轨点是从表面之后量起的；车身后半段长 0.54 m，若障碍物本身
> 有 0.5 m 厚，余量取 1.0 m 时回轨瞬间车尾会离箱体远侧只剩十几厘米。
> 权衡后取 1.3 m，多跳过的 0.3 m 远小于撞上再重试的代价。

**进度停滞兜底**：代价地图看不到的卡死（例如贴着障碍物原地磨）由
`0.1 m / 0.15 rad / 5 s` 的停滞检测兜住，此时用 `find_free_ahead`
在前方 2.5 m 内找一个可通行点作为绕行目标，然后同样回轨。若停滞/失败点
距条带终点 ≤ `end_stall_tolerance`（1 m），不再绕行，直接结束该段：
条带终点离墙约 0.3 m，控制器在膨胀层附近压不进 0.12 m 的终点容差，
否则会在段末反复"停滞 → 回轨到同一点"（实测每次白耗几十秒）。

**定位跳变保护**：执行器每个 tick 检查 `map→base_link`，相邻 tick（≤1 s）
跳变超过 `pose_jump_limit=5 m`、或跑出所有分区包围盒外
`pose_sanity_margin=5 m`，立即取消当前 goal 并终止任务。实测 FAST-LIO 在
地图噪声/障碍密集区会突然发散（位姿几秒内飘出几十米，odom 最终到公里级），
没有这道保护时执行器会继续用发散位姿发指令，车在场地里盲开；现在会立刻
停车并打印 `任务终止：定位跳变 …`，重新定位后再 `/coverage/start` 即可。

**话题 / 服务**

| 名称 | 类型 | 说明 |
|------|------|------|
| `coverage/status` | `std_msgs/String` | 状态文本（含 swath 序号、绕行次数、跳过数） |
| `coverage/current_swath` | `nav_msgs/Path` | 当前执行的 swath（TRANSIENT_LOCAL，晚加入的 RViz 也能拿到） |
| `coverage/planned_path` | `nav_msgs/Path` | 全部 swath：每条依次放起点、终点，连起来即完整往复覆盖路线（TRANSIENT_LOCAL） |
| `coverage_view.rviz` | RViz 配置 | `coverage:=true` 时自动加载，预置上述话题的显示项并俯视整场 |
| `coverage/transit_path` | `nav_msgs/Path` | 平滑转场实际下发的"接近路径 + 条带"合并路径（TRANSIENT_LOCAL） |
| `coverage/blocked_interval` | `visualization_msgs/Marker` | 本次绕行被跳过的区间 |
| `coverage/rejoin_pose` | `geometry_msgs/PoseStamped` | 绕行回轨点 |
| `coverage/start` / `coverage/cancel` | `std_srvs/Trigger` | 手动开始/取消（`auto_start=false` 时用） |

**依赖注意**：`coverage_server` 需要 Fields2Cover 2.1.1（`ros-humble-fields2cover`）。
本机未用 apt 安装，验证时用 `/tmp/f2c_prefix` 前缀提供；正式部署请
`sudo apt install ros-humble-fields2cover` 后重新 `colcon build`
`opennav_coverage_msgs opennav_coverage coverage_mission`。

---

## 8. 文件调用关系

### 8.1 启动链

```
ros2 launch sam_bot_description navigation.launch.py
│
├─ IncludeLaunch(gzserver.launch.py)   ← 加载 src/sam_bot_description/world/my_carpark（Gazebo 世界）
├─ IncludeLaunch(gzclient.launch.py)   ← Gazebo 图形界面（gui:=false 时跳过）
├─ Node(robot_state_publisher)         ← 参数：xacro 展开后的 robot_description
├─ Node(spawn_entity.py)               ← 把机器人 spawn 进 Gazebo（-x 2.0 -y 2.7 -z 0.24）
├─ Node(fastlio_mapping)               ← 参数：config/mid360_sim.yaml + use_sim_time
├─ Node(pointcloud_to_laserscan)       ← /livox/lidar → /scan
├─ IncludeLaunch(nav2 navigation_launch.py) ← 参数：params_file + use_sim_time（无 amcl）
│    ├─ planner_server / controller_server / bt_navigator / ...
│    └─ velocity_smoother
├─ Node(map_server)                    ← 加载 map/my_map.yaml
├─ TimerAction(5s) → Node(lifecycle_manager_localization) ← 只管理 map_server
├─ Node(relocalization_node)           ← 参数：relocalization_params.yaml + prior_map_path
└─ Node(rviz2)                          ← 加载 nav2_default_view.rviz
```

### 8.2 关键参数传递

| 启动文件 | 读取的文件 | 作用 |
|----------|-----------|------|
| `navigation.launch.py` | `description/sam_bot_description.urdf` | 经 `Command(['xacro ', ...])` 展开成纯 URDF |
| `navigation.launch.py` | `world/my_carpark` | Gazebo 世界（文件名无 `.sdf` 后缀，按内容解析） |
| `navigation.launch.py` | `src/sam_bot_description/map/my_map.yaml` | 传给 map_server（经 Nav2 RewrittenYaml 改写 `yaml_filename`） |
| `navigation.launch.py` | `src/sam_bot_description/map/prior_3d_map.pcd` | 传给 relocalization_node 的 `prior_map_path` |
| `navigation.launch.py` | `config/nav2_params.yaml` | Nav2 全套参数 |
| `navigation.launch.py` | `fastlio_localization/config/relocalization_params.yaml` | 3D 重定位参数 |
| `display.launch.py` | `rviz/urdf_config.rviz` | RViz 显示配置 |

### 8.3 CMakeLists.txt 安装规则

```
install(DIRECTORY launch rviz description config world behavior_trees map
        DESTINATION share/${PROJECT_NAME})
```

构建（`colcon build`）后，这些文件被装到 `install/.../share/sam_bot_description/` 下，启动文件通过 `get_package_share_directory('sam_bot_description')` 找到它们。

---

## 9. 数据流通关系总图

### 9.1 传感器数据流

```
                    ┌───────────────────────── Gazebo 仿真 ─────────────────────────┐
                    │                                                                │
   /cmd_vel ────► diff_drive 插件 ──► 轮子转动 ──► /demo/odom（轮式里程计，仅调试）  │
                    │                     │                                          │
                    │                     └──► /joint_states（轮子转角）              │
                    │                                                                │
                    imu_sensor 插件 ──────────► /livox/imu（角速度+加速度）            │
                    mid360 插件 ──────────► /livox/lidar（3D 点云）                    │
                    pointcloud_to_laserscan ──► /scan（2D 激光投影）                    │
                    camera 插件 ──────────────► /depth_camera/*（深度图+点云）        │
                    └────────────────────────────────────────────────────────────────┘
```

### 9.2 融合与 TF 数据流

```
/livox/lidar ──┐
               ├──► fastlio_mapping ──► TF: odom→base_link ──┐
/livox/imu ────┘                     └──► /cloud_registered   │
                                                              ├──► 完整 TF 树
/joint_states ──► robot_state_publisher ──► TF: base_link→* ──┘
                 （+ /robot_description）

/cloud_registered ──┐
                    ├──► fastlio_relocalization ──► TF: map→odom ──► 完整 TF 树
prior_3d_map.pcd ───┘            ├──► /localization_pose（定位结果）
                                 └──► /aligned_scan（配准后的点云）
```

### 9.3 导航数据流

```
src/sam_bot_description/map/my_map.yaml ──► map_server ──► /map ──► global_costmap
                                            │
/scan ──────────┐                           │
/cloud_registered_body ─┴──► 两张图的障碍层（标记 + 稠密清除，frame=livox_imu）
                                            ▼
RViz 目标 ──► bt_navigator ──► planner_server ──► 全局路径 ──► controller_server
                                                                  │
local_costmap ────────────────────────────────────────────────► controller_server
                                                                  │
                                        controller_server ──► /cmd_vel_nav
                                                                  │
                                        velocity_smoother ──► /cmd_vel ──► diff_drive
                                        behavior_server（恢复）──► /cmd_vel
```

### 9.4 话题速查表

| 话题 | 类型 | 发布者 | 主要订阅者 |
|------|------|--------|-----------|
| `/cmd_vel` | Twist | velocity_smoother、behavior_server | diff_drive |
| `/cmd_vel_nav` | Twist | controller_server | velocity_smoother |
| `/demo/odom` | Odometry | diff_drive | （仅调试用，导航不消费） |
| `/livox/imu` | Imu | imu_sensor 插件 | fastlio_mapping |
| `/livox/lidar` | PointCloud2 | mid360 插件 | fastlio_mapping、pointcloud_to_laserscan |
| `/cloud_registered` | PointCloud2 | fastlio_mapping（odom 系、去畸变） | fastlio_relocalization（ICP） |
| `/cloud_registered_body` | PointCloud2 | fastlio_mapping（**livox_imu** 系、去畸变） | 两张代价地图的障碍层（标记 + 稠密清除） |
| `/Odometry` | Odometry | fastlio_mapping | 重定位节点（取时间戳/调试） |
| `/scan` | LaserScan | pointcloud_to_laserscan（由 `/livox/lidar` 投影） | global_costmap、local_costmap |
| `/joint_states` | JointState | joint_state_publisher 插件 | robot_state_publisher |
| `/map` | OccupancyGrid | map_server | global_costmap |
| `/prior_map` | PointCloud2 | fastlio_relocalization（transient_local） | RViz |
| `/aligned_scan` | PointCloud2 | fastlio_relocalization | RViz |
| `/localization_pose` | PoseWithCovarianceStamped | fastlio_relocalization | RViz 等 |
| `/relocalize` | std_srvs/Trigger | — | fastlio_relocalization（手动触发全局重定位） |

---

## 10. 运行环境与性能（WSL2 / GPU）

本机是 WSL2（Ubuntu 22.04）+ WSLg，桌面 5120×1600。实测**仿真/数据链路不是瓶颈，卡顿来自 GUI 渲染**，排查结论如下。

### 10.1 渲染路径

- Gazebo 和 RViz 都走 Mesa 的 `d3d12` 后端（硬件加速），**不是** llvmpipe 软件渲染。`swrast_dri.so` 只是 Mesa 启动时加载的备用模块，出现在 `/proc/<pid>/maps` 里不代表在用软件渲染——以 `glGetString(GL_RENDERER)` 为准。
- Mesa 默认选中的是 **Intel UHD 核显**（`D3D12 (Intel(R) UHD Graphics)`），RTX 5060 完全闲置。`run.sh` / `build_map.sh` / `build_3d_map.sh` 现在统一设置：
  ```bash
  export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
  ```
  之后 `GL_RENDERER` 变为 `D3D12 (NVIDIA GeForce RTX 5060 Laptop GPU)`；RViz 日志里的 `OpenGl version` 从 `2.1 (GLSL 1.2)` 变成 `4.2 (GLSL 4.2)`（Intel 只支持到 4.1，可用来确认选卡生效）。
- `navigation.launch.py` 以前给 rviz2 传 `MESA_GL_VERSION_OVERRIDE=2.1`，现已移除：Mesa 默认给的是 GL 4.1 兼容模式（不是 core profile），实测 rviz2 正常；而 Map 显示器的 `indexed_8bit_image` GLSL link 错误（`active samplers with a different type refer to the same texture image unit`）在 2.1 和 4.1 下都会出现，是 RViz 的 glsl120 shader 在 Mesa 上的老问题，与该变量无关，也不影响显示。

### 10.2 实测数据（GUI 全开）

| 项目 | 数值 |
|------|------|
| 实时因子 RTF | 0.94~0.97，仿真准实时 |
| `/clock`、`/livox/lidar`、`/scan`、`/cloud_registered` | 约 9.6~9.7 Hz，无积压 |
| 消息时间戳滞后 | scan −0.03 s、cloud 0.07 s、两张 costmap 0.00 s |
| CPU（24 核） | 约 3 核：gzserver ~1.3 核、gzclient ~0.7、rviz2 ~0.3、FAST-LIO ~0.33、重定位 ~0.35 |
| 同尺寸 GL 吞吐（2258×1217，整套栈运行时） | Intel 核显 29 FPS → RTX 5060 57 FPS |
| 代价地图发布频率（实测） | local ~1.6 Hz（配置 `publish_frequency: 2.0`）、global ~0.5 Hz（配置 `update_frequency: 1.0`，单次更新耗时超过周期） |

### 10.3 还能做什么

- Gazebo 的 `<gui>` 段没有 `update_rate`，gzclient 会不设上限地重绘。两个窗口各接近半屏（约 2.7 + 3.0 M 像素）时，核显很容易被占满。
- 仍然觉得卡时：`HEADLESS=1 ./run.sh`（只开 RViz、不开 Gazebo 窗口），或把 Gazebo 窗口拉小一档。
- WSLg 的合成/推流路径（XWayland → Weston → RDP）本身有固定开销：即使渲染已交给独显，大窗口仍要合成后送到 Windows，这是换显卡也消不掉的延迟。

---

## 附：一个完整导航周期的时间线

1. **启动**：Gazebo 加载世界 → spawn 机器人 → 各插件开始发布传感器数据。
2. **TF 建立**：robot_state_publisher 发 `base_link→*`；FAST-LIO 收到前几帧 LiDAR/IMU 后发 `odom→base_link`。
3. **地图加载**：map_server（延迟 5 s 由 lifecycle manager 激活）发布 `/map`；重定位节点加载 `prior_3d_map.pcd` 并先发单位阵 `map→odom`。
4. **初始定位**：`relocalize_on_start: true`，重定位节点自动做一次全局搜索 + ICP，成功后开始发真实 `map→odom`——**无需在 RViz 手动给初始位姿**。
5. **下发目标**：RViz "2D Nav Goal" → bt_navigator 开始规划与跟踪。
6. **运动闭环**：规划 → DWB 跟踪 → 速度指令 → 机器人移动 → LiDAR/IMU 回传 → FAST-LIO 更新 `odom→base_link` → 重定位节点更新 `map→odom` → 代价地图更新 → 直到到达目标。
7. **跟踪丢失**：若连续 5 帧 ICP 内点率不足，节点自动重新全局搜索；也可手动调用 `/relocalize`。
