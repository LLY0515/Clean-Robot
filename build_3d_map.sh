#!/usr/bin/env bash
# ============================================================================
#  生成 3D 先验地图（供 fastlio_localization 做全局重定位）
#
#  流程：
#    1. Gazebo 仿真 + FAST-LIO（mapping 配置：pcd_save_en=true）；
#    2. 等 FAST-LIO 输出 /Odometry；
#    3. auto_drive_map.py 自动跑一个小方环采集点云；
#    4. 调用 FAST-LIO 的 /map_save 服务，把内部累积的 odom 系点云写成 PCD；
#    5. 关闭仿真。
#
#  输出：src/sam_bot_description/map/prior_3d_map.pcd
#
#  用法：./build_3d_map.sh                  # 默认键盘遥控（当前场景 my_carpark 约 20×20 m）
#        ./build_3d_map.sh --auto            # 小场景：固定小方环自动行驶
#        ./build_3d_map.sh [--manual|--auto] /path/to/other.pcd
# ============================================================================

set -e

cd "$(dirname "$0")"

WS="$(pwd)"
# 【改动】默认改为手动模式：当前场景 my_carpark 约 20×20 m，有内部隔墙和
# 两辆车，auto_drive_map.py 的 1.1 m 小方环采不到几米外的墙面，
# 所以默认用键盘遥控（teleop_twist_keyboard），Ctrl+C 结束遥控后自动存图。
#   ./build_3d_map.sh            # 键盘遥控（默认）
#   ./build_3d_map.sh --auto     # 小场景自动小方环
MANUAL=1
if [ "${1:-}" = "--manual" ]; then
  MANUAL=1
  shift
elif [ "${1:-}" = "--auto" ]; then
  MANUAL=0
  shift
fi
MAP_OUT="${1:-$WS/src/sam_bot_description/map/prior_3d_map.pcd}"
MAPPING_CONFIG="$WS/src/fast_lio/config/mid360_sim_mapping.yaml"
AUTO_DRIVE="$WS/src/fastlio_localization/tools/auto_drive_map.py"
SIM_LOG="/tmp/build_3d_map_sim.log"

echo "=============================================="
echo "  3D 先验地图生成"
echo "  输出：$MAP_OUT"
echo "=============================================="

# ---------- 清理残留进程 ----------
echo "[清理] 关闭残留的仿真/定位进程 ..."
pkill -9 -f "gzserver|gzclient|gazebo|robot_state_publisher|component_container|rviz2|joint_state_publisher|spawn_entity|slam_toolbox|fastlio_mapping|pointcloud_to_laserscan|teleop_twist_keyboard|map_saver|ros2 launch sam_bot_description|fastlio_relocalization" 2>/dev/null || true
sleep 2

# ---------- 环境 ----------
source /opt/ros/humble/setup.bash
if [ ! -f install/setup.bash ]; then
  echo "✗ 找不到 install/setup.bash，请先构建工作区：./build.sh"
  exit 1
fi
source install/setup.bash

if [ -d "$HOME/.local/share/glib-2.0/schemas" ]; then
  export GSETTINGS_SCHEMA_DIR="$HOME/.local/share/glib-2.0/schemas"
fi
export QT_QPA_PLATFORM=xcb
# 【改动】与 run.sh 一致：让 Gazebo 用独显渲染，否则默认落到 Intel 核显。
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA

if [ ! -f "$AUTO_DRIVE" ]; then
  echo "✗ 找不到自动行驶脚本：$AUTO_DRIVE"
  exit 1
fi

mkdir -p "$(dirname "$MAP_OUT")"
rm -f "$MAP_OUT"

# ---------- 启动仿真（mapping 配置） ----------
echo "[1/4] 启动 Gazebo + FAST-LIO（mapping 配置，无 RViz）..."
setsid ros2 launch sam_bot_description display.launch.py \
  gui:=True rviz:=False \
  fastlio_config:="$MAPPING_CONFIG" \
  fastlio_map_file:="$MAP_OUT" \
  > "$SIM_LOG" 2>&1 < /dev/null &

# ---------- 等待 FAST-LIO 里程计 ----------
# 说明：本机的 `ros2 topic echo` 会因 ros2cli 守护进程异常直接报
# "!rclpy.ok()"，不能用来探测数据；改用 `ros2 topic hz` 看是否真有速率。
echo "[2/4] 等待 FAST-LIO 输出 /Odometry ..."
READY=0
HZ_LOG="/tmp/build_3d_map_odom_hz.log"
for _ in $(seq 1 20); do
  timeout 6 ros2 topic hz /Odometry > "$HZ_LOG" 2>&1 || true
  if grep -q "average rate" "$HZ_LOG"; then
    READY=1
    break
  fi
  sleep 1
done
if [ "$READY" != "1" ]; then
  echo "✗ 120 秒内没有 /Odometry，请查看日志：$SIM_LOG"
  pkill -9 -f "gzserver|fastlio_mapping|ros2 launch sam_bot_description" 2>/dev/null || true
  exit 1
fi

# FAST-LIO 刚初始化时机器人可能还在微落/回弹，等它稳定。
sleep 4

# ---------- 采集点云 ----------
if [ "$MANUAL" = "1" ]; then
  # 【改动】手动遥控：Ctrl+C 结束遥控（SIGINT 也会送到本脚本，用 trap 吃掉，
  # 继续执行后面的 /map_save）。建议把整个场景走一遍，越全重定位越稳。
  echo "[3/4] 手动遥控采集点云：用键盘把车开遍场景，完成后按 Ctrl+C 结束遥控并保存。"
  trap 'true' INT
  ros2 run teleop_twist_keyboard teleop_twist_keyboard || true
  trap - INT
  # 遥控退出后差速插件会保持最后一条速度指令，这里明确发一次 0 让车停住。
  timeout 5 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.0}}" > /dev/null 2>&1 || true
  sleep 1
else
  echo "[3/4] 自动行驶采集点云（约 45 秒；小方环只适合小场景）..."
  python3 "$AUTO_DRIVE"
fi

# ---------- 保存 PCD ----------
echo "[4/4] 调用 /map_save 保存 PCD ..."
if ! timeout 60 ros2 service call /map_save std_srvs/srv/Trigger "{}"; then
  echo "✗ /map_save 调用失败"
fi

sleep 1

# 【重要·不要再平移 PCD】
# FAST-LIO 保存的点云在 odom 系，原点 = 建图起点的 IMU 位姿；
# slam_toolbox 建的 2D 栅格地图同样在 map 系，而 map->odom 初始为恒等变换，
# 所以 map 系原点**也是**建图起点的 IMU（实测 map->base_link 在出生点就是
# (-0.339, -0.028)，正好是 IMU 安装偏移的负值），并不是 base_link。
# 因此先验 PCD 保持 odom 系即可，任何 xy 平移都会让 map->odom 多出同样的
# 偏移，导航时 RViz 里的机器人会比 Gazebo 中"更靠近"前方障碍物（实测多 0.33 m）。
# 旧的 shift_pcd.py（+0.3335, +0.0233）是在"2D 地图原点是 base_link"的错误
# 前提下加的，已停用；脚本每次都会先删旧 PCD，重新生成即可。
if [ -s "$MAP_OUT" ]; then
  echo ">>> PCD 保持 FAST-LIO odom 系（与 slam_toolbox 的 map 系一致，无需平移）"
fi

if [ -s "$MAP_OUT" ]; then
  echo ">>> ✓ 已生成先验地图：$MAP_OUT"
  ls -lh "$MAP_OUT" | sed 's/^/    /'

  # --symlink-install 只会在 build 时给“已存在”的文件建软链；先验地图通常是
  # build 之后才生成的，这里补一条软链，navigation.launch.py 才能从 install
  # 目录（share/sam_bot_description/map/）找到它。
  INSTALL_MAP="$WS/install/sam_bot_description/share/sam_bot_description/map"
  if [ -d "$INSTALL_MAP" ]; then
    ln -sf "$MAP_OUT" "$INSTALL_MAP/prior_3d_map.pcd"
    echo ">>> 已更新 install 目录软链：$INSTALL_MAP/prior_3d_map.pcd"
  fi
else
  echo "✗ 没有生成 $MAP_OUT，请检查："
  echo "    - $SIM_LOG 里 pcd_save_en 是否为 true；"
  echo "    - FAST-LIO 的 map_en 是否开启（累积点云 pcl_wait_pub 是否为空）。"
fi

# ---------- 关闭仿真 ----------
echo ">>> 关闭仿真进程 ..."
pkill -INT -f "ros2 launch sam_bot_description" 2>/dev/null || true
sleep 2
pkill -9 -f "gzserver|gzclient|gazebo|robot_state_publisher|joint_state_publisher|spawn_entity|fastlio_mapping|pointcloud_to_laserscan|rviz2" 2>/dev/null || true
echo ">>> 完成。"
