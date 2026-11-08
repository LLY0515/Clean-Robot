#!/usr/bin/env bash
# ============================================================
#  全覆盖清扫一键启动（仿真 + Nav2 + opennav_coverage + 执行器）
#
#  与 run.sh 的区别：给 navigation.launch.py 传 coverage:=true，
#  额外启动：
#    - coverage_server      (opennav_coverage 的 ComputeCoveragePath 服务端)
#    - coverage_executor    (转场、逐条 swath、避障后回原轨道)
#
#  用法：
#    ./run_coverage.sh
#    HEADLESS=1 ./run_coverage.sh      # 不打开 Gazebo 窗口
#
#  手动触发/取消（coverage_params.yaml 里 auto_start: false 时）：
#    ros2 service call /coverage/start std_srvs/srv/Trigger
#    ros2 service call /coverage/cancel std_srvs/srv/Trigger
# ============================================================
set -e

cd "$(dirname "$0")"

echo "清理残留进程..."
pkill -9 -f "gzserver|gzclient|gazebo|robot_state_publisher|component_container|rviz2|joint_state_publisher|spawn_entity|slam_toolbox|rtabmap|fastlio_mapping|relocalization_node|pointcloud_to_laserscan|teleop_twist_keyboard|map_saver|ros2 launch sam_bot_description|opennav_coverage|coverage_executor|nav2_map_server|nav2_lifecycle_manager|nav2_planner|nav2_controller|nav2_bt_navigator|nav2_behaviors|nav2_smoother|nav2_velocity_smoother|nav2_waypoint_follower" 2>/dev/null || true
sleep 2

source /opt/ros/humble/setup.bash
source install/setup.bash

# 运行库检查：opennav_coverage 动态链接 Fields2Cover。
# 这个包只能由你本人用 sudo 安装，脚本不代为输入密码。
if ! find /opt/ros/humble -name 'libFields2Cover.so*' -print -quit 2>/dev/null | grep -q .; then
  echo "缺少 Fields2Cover 运行库，coverage_server 无法启动。"
  echo "请先执行："
  echo "  sudo apt install -y ros-humble-fields2cover"
  echo "然后重新编译："
  echo "  colcon build --packages-select opennav_coverage_msgs opennav_coverage coverage_mission --symlink-install"
  exit 1
fi

# 与 run.sh 保持一致：WSL2 下让 Gazebo/RViz 用独显渲染
export QT_QPA_PLATFORM=xcb
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA

if [ "${HEADLESS:-0}" = "1" ]; then
  echo "HEADLESS=1：不启动 Gazebo GUI"
  exec ros2 launch sam_bot_description navigation.launch.py \
    gui:=false coverage:=true
fi

exec ros2 launch sam_bot_description navigation.launch.py coverage:=true
