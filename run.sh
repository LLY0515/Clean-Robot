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
pkill -9 -f "gzserver|gzclient|gazebo|robot_state_publisher|component_container|rviz2|joint_state_publisher|spawn_entity|slam_toolbox|fastlio_mapping|pointcloud_to_laserscan|teleop_twist_keyboard|map_saver|ros2 launch sam_bot_description" 2>/dev/null || true
sleep 2

# source 环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 本机（VMware 虚拟显卡 + Qt GTK 主题）需要的环境变量，否则 rviz2 崩溃
export GSETTINGS_SCHEMA_DIR=/home/rostwo/.local/share/glib-2.0/schemas
export QT_QPA_PLATFORM=xcb

# 启动完整导航
exec ros2 launch sam_bot_description navigation.launch.py
