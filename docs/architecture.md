# sam_bot_description 仿真架构说明

本文档详细说明 `sam_bot_description` 包如何从**机器人 URDF 模型**出发，经过 **Gazebo 插件**、**里程计/IMU 数据融合（EKF）**，再到**建图（SLAM）**与**导航规划（Nav2）**的完整链路，并解释各文件之间的调用关系、TF 关系与数据流通关系。

---

## 目录

1. [项目总览](#1-项目总览)
2. [机器人模型构建（URDF）](#2-机器人模型构建urdf)
3. [Gazebo 插件：传感器与执行器](#3-gazebo-插件传感器与执行器)
4. [里程计与 IMU 数据融合（EKF）](#4-里程计与-imu-数据融合ekf)
5. [TF 坐标关系](#5-tf-坐标关系)
6. [建图（SLAM → 地图保存）](#6-建图slam--地图保存)
7. [导航规划（Nav2）](#7-导航规划nav2)
8. [文件调用关系](#8-文件调用关系)
9. [数据流通关系总图](#9-数据流通关系总图)

---

## 1. 项目总览

**技术栈**：ROS2 Humble + Gazebo Classic（gzserver/gzclient）+ robot_localization（EKF）+ Nav2（MPPI / Navfn / AMCL）。

**目录结构**：

```
sam_bot_description/
├── package.xml                         # 包依赖声明
├── CMakeLists.txt                      # colcon 构建与安装规则
├── src/
│   ├── description/
│   │   └── sam_bot_description.urdf    # 机器人模型 + Gazebo 插件（xacro 格式）
│   ├── launch/
│   │   ├── navigation.launch.py        # 完整导航仿真启动（Gazebo + EKF + Nav2 + RViz）
│   │   └── display.launch.py           # 模型显示 / 建图启动（Gazebo + RSP + EKF + RViz）
│   ├── config/
│   │   ├── ekf.yaml                    # robot_localization EKF 融合配置
│   │   └── nav2_params.yaml            # Nav2 全套导航参数
│   ├── rviz/
│   │   └── urdf_config.rviz            # display.launch.py 用的 RViz 配置
│   └── world/
│       └── my_world.sdf                # Gazebo 世界（围墙 + 障碍物）
├── map/
│   ├── my_map.yaml                     # 地图元数据（分辨率/原点）
│   └── my_map.pgm                      # 地图栅格图像（占用网格）
└── install/                            # colcon 安装产物（构建后生成）
```

**依赖（package.xml 摘录）**：

| 依赖 | 用途 |
|------|------|
| `xacro` | 把 .urdf.xacro 展开成纯 URDF |
| `robot_state_publisher` | 从 URDF + joint_states 发布机器人 TF |
| `joint_state_publisher` | 发布非仿真关节状态（display 用） |
| `gazebo_ros` / `gazebo_plugins` | Gazebo 仿真与传感器/驱动插件 |
| `robot_localization` | EKF 里程计+IMU 融合 |
| `nav2_bringup` / `nav2_rviz_plugins` | 导航栈与 RViz 导航插件 |
| `tf2_ros` | TF 坐标变换 |

---

## 2. 机器人模型构建（URDF）

文件：`src/description/sam_bot_description.urdf`（xacro 格式，根元素带 `xmlns:xacro`）。

### 2.1 xacro 常量与宏

模型顶部用 `<xacro:property>` 定义了尺寸常量，避免重复写死：

| 常量 | 值 | 含义 |
|------|-----|------|
| `base_length` / `base_width` / `base_height` | 0.42 / 0.31 / 0.18 | 车体（长方体）长宽高 |
| `wheel_radius` / `wheel_width` | 0.10 / 0.04 | 驱动轮半径 / 宽度 |
| `wheel_xoff` / `wheel_zoff` / `wheel_ygap` | 0.12 / 0.05 / 0.025 | 轮子 x/z 偏移、横向间隙 |

三个**惯性宏**自动计算转动惯量（`box_inertia`、`cylinder_inertia`、`sphere_inertia`），供 Gazebo 物理引擎使用。

### 2.2 link 与 joint 结构

机器人由以下 **link** 组成（`base_link` 为树根）：

| link | 形状 / 质量 | 说明 |
|------|------------|------|
| `base_link` | 0.42×0.31×0.18 箱体，15 kg | 车体本体（URDF 树根） |
| `base_footprint` | 空 link | 地面投影点（2D 导航基准） |
| `drivewhl_l_link` / `drivewhl_r_link` | 圆柱 r=0.10，0.5 kg | 左右驱动轮 |
| `front_caster` | 球 r≈0.06，0.5 kg | 前万向轮（摩擦设 0） |
| `imu_link` | 0.1 箱体，0.1 kg | IMU 安装点 |
| `lidar_link` | 圆柱 r=0.0508，0.125 kg | 激光雷达安装点 |
| `camera_link` | 0.015×0.130×0.022 箱体 | 深度相机安装点 |
| `camera_depth_frame` | 空 link | 相机光学坐标系 |

**joint 连接关系**（所有 joint 由 `robot_state_publisher` 解析为 TF）：

| joint | 类型 | parent → child | 说明 |
|-------|------|----------------|------|
| `base_joint` | fixed | base_link → base_footprint | z 偏移 −0.15，把 footprint 放到地面 |
| `drivewhl_l_joint` | **continuous** | base_link → drivewhl_l_link | 左轮，转轴 (0,1,0) |
| `drivewhl_r_joint` | **continuous** | base_link → drivewhl_r_link | 右轮，转轴 (0,1,0) |
| `caster_joint` | fixed | base_link → front_caster | 前万向轮 |
| `imu_joint` | fixed | base_link → imu_link | z +0.01 |
| `lidar_joint` | fixed | base_link → lidar_link | z +0.12 |
| `camera_joint` | fixed | base_link → camera_link | (0.215, 0, 0.05) |
| `camera_depth_joint` | fixed | camera_link → camera_depth_frame | 相机光轴旋转 |

> 只有左右驱动轮是 `continuous`（可无限旋转），其余都是 `fixed`。`continuous` 关节的转角由 Gazebo 物理仿真更新，再通过 `joint_state_publisher` 插件发布，供 `robot_state_publisher` 计算轮子 TF。

---

## 3. Gazebo 插件：传感器与执行器

URDF 里的 `<gazebo>` 标签在机器人被 spawn 进 Gazebo 时生效，每个 `<plugin>` 加载一个 Gazebo-ROS 桥接插件。共 **5 个插件**：

| 插件 | 加载库 | 发布/订阅 | 作用 |
|------|--------|----------|------|
| diff_drive | `libgazebo_ros_diff_drive.so` | 订 `/cmd_vel`，发 `/demo/odom` | 差速驱动 + 轮式里程计 |
| imu_sensor | `libgazebo_ros_imu_sensor.so` | 发 `/demo/imu`（100 Hz） | IMU（角速度+线加速度） |
| ray_sensor（激光） | `libgazebo_ros_ray_sensor.so` | 发 `/scan`（15 Hz） | 360 线激光雷达 |
| camera（深度） | `libgazebo_ros_camera.so` | 发 `/depth_camera/*` | 深度相机 |
| joint_state_publisher | `libgazebo_ros_joint_state_publisher.so` | 发 `/joint_states`（30 Hz） | 轮子真实转角 |

### 3.1 diff_drive 差速驱动（关键）

```xml
<plugin name='diff_drive' filename='libgazebo_ros_diff_drive.so'>
  <ros>
    <namespace>/demo</namespace>
    <remapping>cmd_vel:=/cmd_vel</remapping>
  </ros>
  <left_joint>drivewhl_l_joint</left_joint>
  <right_joint>drivewhl_r_joint</right_joint>
  <wheel_separation>0.36</wheel_separation>
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
- `publish_odom_tf=false`：**故意不让 diff_drive 发 odom→base_link 的 TF**，这段 TF 改由 EKF 发布（见第 4 节），这样才能用 IMU 校正里程计漂移。

### 3.2 IMU 传感器

- 命名空间 `/demo`，发布 `/demo/imu`（`sensor_msgs/Imu`）。
- 配置了高斯噪声（角速度 stddev 2e-4、线加速度 stddev 1.7e-2）和 bias，模拟真实 IMU 误差。

### 3.3 激光雷达

- 发布 `/scan`（`sensor_msgs/LaserScan`），`frame_name: lidar_link`。
- 360 个采样点、量程 0.12~3.5 m、分辨率 0.015 m，匹配 Nav2 里的 `laser_max_range: 3.5`。

---

## 4. 里程计与 IMU 数据融合（EKF）

文件：`src/config/ekf.yaml`，由 `navigation.launch.py` 启动 `ekf_filter_node`（robot_localization）。

### 4.1 为什么需要 EKF

`diff_drive` 只输出**轮式里程计**，它有两个问题：

1. 长时间会漂移（轮子打滑、累计误差）；
2. 没有绝对航向参考。

EKF 把**轮式里程计**和 **IMU** 融合，得到更稳的位姿估计，并发布 `odom → base_link` 的 TF。

### 4.2 ekf.yaml 核心配置

```yaml
frequency: 30.0
two_d_mode: true              # 平面差分车：只估计 x / y / yaw

odom_frame: odom
base_link_frame: base_link
world_frame: odom

odom0: demo/odom              # 轮式里程计
odom0_config: [true, true, false,   # x, y 位置融合
               false, false, false, # 姿态不融合
               false, false, false, # 线速度不融合
               false, false, true,  # yaw 角速度融合
               false, false, false] # 加速度不融合

imu0: demo/imu                # IMU
imu0_config: [false, false, false, # 位置不融合
              false, false, true,  # yaw 姿态融合（关键）
              false, false, false, # 线速度不融合
              false, false, false, # 角速度不融合
              false, false, false] # 加速度不融合
```

### 4.3 融合矩阵解读

`*_config` 是 15 个布尔值，依次对应：`x/y/z 位置、roll/pitch/yaw 姿态、vx/vy/vz 线速度、roll/pitch/yaw 角速度、ax/ay/az 线加速度`。

| 传感器 | 融合的状态量 | 作用 |
|--------|-------------|------|
| `odom0`（里程计） | x 位置、y 位置、yaw 角速度 | 提供平面位移 + 转向速率 |
| `imu0`（IMU） | yaw 姿态 | 提供绝对航向，抑制里程计航向漂移 |

> `two_d_mode: true` 是**必须的**：URDF 里 diff_drive 的 odom 只有 x/y/yaw 有效，若用默认 3D 模式融合 IMU 姿态和 odom 的 z，会导致高度状态发散（历史上一度 z 漂到 −70m，引起导航中止）。

EKF 输出：

- **TF**：`odom → base_link`（`publish_tf: true`）；
- **话题**：`/odometry/filtered`（滤波后的里程计）。

---

## 5. TF 坐标关系

### 5.1 完整 TF 树

```
map ──► odom ──► base_link ──► base_footprint
                        │
                        ├──► drivewhl_l_link
                        ├──► drivewhl_r_link
                        ├──► front_caster
                        ├──► imu_link
                        ├──► lidar_link
                        └──► camera_link ──► camera_depth_frame
```

### 5.2 每段 TF 的发布者

| TF | 发布者 | 来源 |
|----|--------|------|
| `map → odom` | **AMCL** | 激光扫描与静态地图匹配，得到全局定位 |
| `odom → base_link` | **EKF**（`ekf_filter_node`） | 里程计 + IMU 融合 |
| `base_link → 所有子 link` | **robot_state_publisher** | 解析 URDF 的 joint |

### 5.3 base_link vs base_footprint

- `base_link`：车体本体坐标系（URDF 树根）。
- `base_footprint`：base_link 正下方 −0.15 m 处的地面投影点，是 2D 导航的"机器人中心"。
- AMCL 用 `base_frame_id: base_footprint`，代价地图/控制器用 `robot_base_frame: base_link`，两者靠 `base_link → base_footprint` 固定 TF 连接，等价可用。

### 5.4 坐标意义

| 坐标系 | 含义 |
|--------|------|
| `map` | 全局固定世界坐标系（与静态地图对齐） |
| `odom` | 里程计坐标系（机器人出发点为原点，会漂移） |
| `base_link` / `base_footprint` | 机器人本体坐标系 |

AMCL 发布的 `map → odom` 变换，就是"把会漂移的 odom 坐标系对齐到全局 map 坐标系"的那层校正。

---

## 6. 建图（SLAM → 地图保存）

> 说明：原 `src/config/slam.yaml` 已删除（本项目当前用预建地图导航，不再走 SLAM）。以下是历史建图流程，用于说明地图从哪来。

### 6.1 建图流程

1. 用 `display.launch.py` 启动 Gazebo + `robot_state_publisher` + EKF + RViz；
2. 运行 `slam_toolbox`（online async 模式）订阅 `/scan` 和 TF，发布 `map → odom` 变换与 `/map` 话题；
3. 手动控制机器人（如 `teleop_twist_keyboard` 发 `/cmd_vel`）遍历环境；
4. 用 `nav2_map_server` 的 `map_saver` 把 `/map` 保存成文件。

### 6.2 地图文件（map/my_map.yaml）

```yaml
image: my_map.pgm
mode: trinary
resolution: 0.05          # 每像素 5 cm
origin: [-4.91, -4.91, 0] # 地图左下角在 map 坐标系中的位置
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

- 地图是 **198×199** 像素、0.05 m/pix，覆盖约 9.9 m × 9.95 m；
- `origin [-4.91, -4.91]` 使地图覆盖 map 坐标系约 `[-4.91, 4.99] × [-4.91, 5.04]`，与 `my_world.sdf` 里四面墙围成的约 `[-5, 5] × [-5, 5]` 区域对齐；
- 机器人 spawn 在 Gazebo 世界原点 (0,0)，对应 map 坐标约为 (0,0)（因为建图时也是从原点出发）。

---

## 7. 导航规划（Nav2）

导航由 `navigation.launch.py` 调用 `nav2_bringup` 的 `bringup_launch.py` 启动，参数全部在 `src/config/nav2_params.yaml`。

### 7.1 Nav2 各节点职责

| 节点 | 插件/算法 | 职责 |
|------|-----------|------|
| `map_server` | — | 读取 `my_map.yaml`，发布 `/map`（静态地图） |
| `amcl` | 粒子滤波 | 用 `/scan` + `/map` 定位，发布 `map→odom` TF |
| `planner_server` | NavfnPlanner | 全局路径规划（在全局代价地图上 A*/Dijkstra） |
| `controller_server` | MPPI | 局部路径跟踪，输出速度指令 |
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
| `local_costmap` | 局部（滚动） | `odom` | 10 Hz | voxel_layer + inflation_layer |

- 两者都用 `/scan`（激光）标记障碍物；
- 全局代价地图叠加了 `/map` 静态地图，用于全局规划；
- 局部代价地图只保留机器人周围 3×3 m 的滚动窗口，用于局部避障。

### 7.3 速度控制链（已实测核实）

```
controller_server ──/cmd_vel_nav──► velocity_smoother ──/cmd_vel──► diff_drive(Gazebo)
behavior_server(恢复行为) ──────────────────/cmd_vel──► diff_drive
```

- `controller_server` 发布 **`/cmd_vel_nav`**（MPPI 原始输出）；
- `velocity_smoother` 订阅 `/cmd_vel_nav`，做加减速限幅后发布 **`/cmd_vel`**；
- `behavior_server` 的恢复行为（spin/backup 等）**直接发布 `/cmd_vel`**，绕过 smoother；
- Gazebo 里的 `diff_drive` 订阅 **`/cmd_vel`**。

### 7.4 导航一次完整的数据流

1. RViz 点击"2D Nav Goal" → 发 `NavigateToPose` action 给 `bt_navigator`；
2. `bt_navigator` 调 `planner_server` 的 `ComputePathToPose`；
3. `planner_server` 在 `global_costmap` 上规划出全局路径 → 返回 `bt_navigator`；
4. `bt_navigator` 把路径通过 `FollowPath` action 发给 `controller_server`；
5. `controller_server`（MPPI）结合 `local_costmap` 和路径，计算出速度指令 → `/cmd_vel_nav`；
6. `velocity_smoother` 平滑限幅 → `/cmd_vel`；
7. Gazebo `diff_drive` 执行速度，机器人移动；
8. 轮子转动 → `/demo/odom` + `/joint_states`；IMU → `/demo/imu`；
9. EKF 融合 → `odom→base_link` TF；AMCL 用 `/scan` 校正 → `map→odom` TF；
10. 代价地图更新、机器人到达目标（误差 < `xy_goal_tolerance` + `yaw_goal_tolerance` 即判定成功）。

---

## 8. 文件调用关系

### 8.1 启动链

```
ros2 launch sam_bot_description navigation.launch.py
│
├─ IncludeLaunch(gzserver.launch.py)   ← 加载 src/world/my_world.sdf（Gazebo 世界）
├─ IncludeLaunch(gzclient.launch.py)   ← Gazebo 图形界面
├─ Node(robot_state_publisher)         ← 参数：xacro 展开后的 robot_description
├─ Node(spawn_entity.py)               ← 把机器人 spawn 进 Gazebo（-topic robot_description）
├─ Node(ekf_filter_node)               ← 参数：config/ekf.yaml + use_sim_time
├─ IncludeLaunch(nav2 bringup_launch.py) ← 参数：map + params_file + use_sim_time
│    ├─ map_server      ← 加载 map/my_map.yaml
│    ├─ amcl / planner_server / controller_server / bt_navigator / ...
│    └─ velocity_smoother
└─ Node(rviz2)                          ← 加载 nav2_default_view.rviz
```

### 8.2 关键参数传递

| 启动文件 | 读取的文件 | 作用 |
|----------|-----------|------|
| `navigation.launch.py` | `description/sam_bot_description.urdf` | 经 `Command(['xacro ', ...])` 展开成纯 URDF |
| `navigation.launch.py` | `world/my_world.sdf` | Gazebo 世界 |
| `navigation.launch.py` | `map/my_map.yaml` | 传给 map_server（经 Nav2 RewrittenYaml 改写 `yaml_filename`） |
| `navigation.launch.py` | `config/nav2_params.yaml` | Nav2 全套参数 |
| `navigation.launch.py` | `config/ekf.yaml` | EKF 融合参数 |
| `display.launch.py` | `rviz/urdf_config.rviz` | RViz 显示配置 |

### 8.3 CMakeLists.txt 安装规则

```
install(DIRECTORY src/launch src/rviz src/description src/config src/world map
        DESTINATION share/${PROJECT_NAME})
```

构建（`colcon build`）后，这些文件被装到 `install/.../share/sam_bot_description/` 下，启动文件通过 `get_package_share_directory('sam_bot_description')` 找到它们。

---

## 9. 数据流通关系总图

### 9.1 传感器数据流

```
                    ┌───────────────────────── Gazebo 仿真 ─────────────────────────┐
                    │                                                                │
   /cmd_vel ────► diff_drive 插件 ──► 轮子转动 ──► /demo/odom（轮式里程计）          │
                    │                     │                                          │
                    │                     └──► /joint_states（轮子转角）              │
                    │                                                                │
                    imu_sensor 插件 ──────────► /demo/imu（角速度+加速度）            │
                    ray_sensor 插件 ──────────► /scan（激光雷达）                      │
                    camera 插件 ──────────────► /depth_camera/*（深度图+点云）        │
                    └────────────────────────────────────────────────────────────────┘
```

### 9.2 融合与 TF 数据流

```
/demo/odom ──┐
             ├──► ekf_filter_node ──► TF: odom→base_link ──┐
/demo/imu ───┘                                              │
                                                            ├──► 完整 TF 树
/joint_states ──► robot_state_publisher ──► TF: base_link→* ─┘
                 （+ /robot_description）

/scan ──┐
        ├──► AMCL ──► TF: map→odom ──► 完整 TF 树
/map ───┘        └──► /amcl_pose（定位结果）
```

### 9.3 导航数据流

```
map/my_map.yaml ──► map_server ──► /map ──► global_costmap
                                            │
/scan ──► global_costmap（障碍层）          │
/scan ──► local_costmap（voxel 层）         │
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
| `/demo/odom` | Odometry | diff_drive | ekf_filter_node、velocity_smoother、bt_navigator |
| `/demo/imu` | Imu | imu_sensor 插件 | ekf_filter_node |
| `/scan` | LaserScan | ray_sensor 插件 | AMCL、global_costmap、local_costmap |
| `/joint_states` | JointState | joint_state_publisher 插件 | robot_state_publisher |
| `/map` | OccupancyGrid | map_server | AMCL、global_costmap |
| `/amcl_pose` | PoseWithCovarianceStamped | AMCL | RViz 等 |
| `/odometry/filtered` | Odometry | ekf_filter_node | （可选） |

---

## 附：一个完整导航周期的时间线

1. **启动**：Gazebo 加载世界 → spawn 机器人 → 各插件开始发布传感器数据。
2. **TF 建立**：robot_state_publisher 发 `base_link→*`；EKF 收到第一条 odom/imu 后发 `odom→base_link`。
3. **地图加载**：map_server 发布 `/map`。
4. **初始定位**：用户在 RViz 用 "2D Pose Estimate" 给 AMCL 初始位姿，AMCL 开始发 `map→odom`。
5. **下发目标**：RViz "2D Nav Goal" → bt_navigator 开始规划与跟踪。
6. **运动闭环**：规划 → MPPI 跟踪 → 速度指令 → 机器人移动 → 里程计/IMU/激光回传 → TF 更新 → 代价地图更新 → 直到到达目标。
