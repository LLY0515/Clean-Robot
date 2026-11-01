#!/usr/bin/env bash
set -e

# 【改动】用脚本自身所在目录作为工作区，避免硬编码旧路径 /home/rostwo/nav2xuexi/...
cd "$(dirname "$0")"

# 清理之前残留的进程
echo "清理残留进程..."
pkill -9 -f "gzserver|gzclient|gazebo|robot_state_publisher|ekf_node|component_container|rviz2|joint_state_publisher|spawn_entity|ros2 launch sam_bot_description" 2>/dev/null || true
sleep 2

# source 环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 本机（VMware 虚拟显卡 + Qt GTK 主题）需要的环境变量，否则 rviz2 崩溃
export GSETTINGS_SCHEMA_DIR=/home/rostwo/.local/share/glib-2.0/schemas
export QT_QPA_PLATFORM=xcb

# 启动完整导航
exec ros2 launch sam_bot_description navigation.launch.py
