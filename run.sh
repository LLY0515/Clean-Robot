#!/usr/bin/env bash
set -e

# 【改动】用脚本自身所在目录作为工作区，避免硬编码旧路径 /home/rostwo/nav2xuexi/...
cd "$(dirname "$0")"

# 清理之前残留的进程
# 【改动】补上可能残留的 slam_toolbox / fastlio_mapping / pointcloud_to_laserscan：
#   之前 build_map.sh（或手动起的 slam_toolbox）退出不干净时，残留的 slam_toolbox
#   会和 Nav2 的 map_server 同时往 /map 发数据，表现为"导航时地图还在边开边长"，
#   看起来像还在建图。
echo "清理残留进程..."
pkill -9 -f "gzserver|gzclient|gazebo|robot_state_publisher|component_container|rviz2|joint_state_publisher|spawn_entity|slam_toolbox|fastlio_mapping|relocalization_node|pointcloud_to_laserscan|teleop_twist_keyboard|map_saver|ros2 launch sam_bot_description|nav2_map_server|nav2_lifecycle_manager|nav2_planner|nav2_controller|nav2_bt_navigator|nav2_behaviors|nav2_smoother|nav2_velocity_smoother|nav2_waypoint_follower" 2>/dev/null || true
sleep 2

# source 环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 本机（虚拟显卡 + Qt GTK 主题）需要的环境变量，否则 rviz2 崩溃
# 【改动】原路径硬编码为 /home/rostwo，本机不存在该目录；指向不存在的
# GSETTINGS_SCHEMA_DIR 会让 GTK 找不到系统 schema，反而更容易崩。
if [ -d "$HOME/.local/share/glib-2.0/schemas" ]; then
  export GSETTINGS_SCHEMA_DIR="$HOME/.local/share/glib-2.0/schemas"
fi
export QT_QPA_PLATFORM=xcb

# 【改动】WSL2 下 Mesa 默认选到 Intel 核显（d3d12 驱动），本机的 RTX 5060 完全闲置。
# 实测同一窗口尺寸下独显的 GL 吞吐是核显的 2~5 倍，两个 GUI 同时开时差距更大。
# 没有独显的机器删掉这一行即可（不设则自动选默认适配器）。
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA

# 启动完整导航
# 【改动】GL 走的是硬件 D3D12（核显或上面指定的独显），不是 llvmpipe 软件渲染；
# 但 Gazebo + RViz 两个大窗口同时渲染仍然吃紧。
# 设 HEADLESS=1 ./run.sh 可以只跑 gzserver + RViz，不打开 Gazebo 窗口。
if [ "${HEADLESS:-0}" = "1" ]; then
  echo "HEADLESS=1：不启动 Gazebo GUI（rviz 仍会打开）"
  exec ros2 launch sam_bot_description navigation.launch.py gui:=false
fi
exec ros2 launch sam_bot_description navigation.launch.py
