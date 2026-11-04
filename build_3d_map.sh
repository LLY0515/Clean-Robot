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
#  用法：./build_3d_map.sh
#        ./build_3d_map.sh /path/to/other.pcd
# ============================================================================

set -e

cd "$(dirname "$0")"

WS="$(pwd)"
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

if [ ! -f "$AUTO_DRIVE" ]; then
  echo "✗ 找不到自动行驶脚本：$AUTO_DRIVE"
  exit 1
fi

mkdir -p "$(dirname "$MAP_OUT")"
rm -f "$MAP_OUT"

# ---------- 启动仿真（mapping 配置） ----------
echo "[1/4] 启动 Gazebo + FAST-LIO（mapping 配置，无 RViz）..."
setsid ros2 launch sam_bot_description display.launch.py \
  gui:=False rviz:=False \
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

# ---------- 自动行驶 ----------
echo "[3/4] 自动行驶采集点云（约 45 秒）..."
python3 "$AUTO_DRIVE"

# ---------- 保存 PCD ----------
echo "[4/4] 调用 /map_save 保存 PCD ..."
if ! timeout 60 ros2 service call /map_save std_srvs/srv/Trigger "{}"; then
  echo "✗ /map_save 调用失败"
fi

sleep 1

# FAST-LIO 保存的是 odom(IMU 起点)系点云，原点在 IMU；2D 栅格地图的原点在
# base_link（世界原点），两者相差 IMU 安装偏移 (0.3335, 0.0233) m。这里把 PCD 的
# x/y 平移过去，保证 3D 重定位的 map 系与 map_server 的 2D 地图一致。
# 注意：只平移 x/y，z 保持雷达平面为 0（重定位的 constrain_planar 会把
# map->odom 的 z 约束为 0）。脚本每次都会先删除旧 PCD，不会重复平移。
if [ -s "$MAP_OUT" ]; then
  echo ">>> 平移 PCD 到 base_link 起点坐标系（与 2D 地图对齐）..."
  python3 "$WS/src/fastlio_localization/tools/shift_pcd.py" "$MAP_OUT" || \
    echo "⚠️  PCD 平移失败，3D 重定位与 2D 地图可能相差 0.33 m"
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
