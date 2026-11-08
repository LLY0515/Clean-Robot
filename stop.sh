#!/usr/bin/env bash
# ============================================================
# 一键关闭导航/仿真相关进程
#
# 用法：
#   ./stop.sh            # 关闭全部（ros2 launch + gazebo + rviz + nav2 + fastlio ...）
#   ./stop.sh rviz       # 只关闭 rviz2
#   ./stop.sh gazebo     # 只关闭 gazebo（gzserver / gzclient / gazebo）
#
# 说明：
#   - 这里用进程名精确匹配（-x）来杀 rviz2/gazebo，避免匹配到本脚本自身
#     或命令行里恰好带 "gazebo" 字样的其他进程。
#   - 全关时先给 ros2 launch 发 SIGINT 让它自己收尾，2 秒后再兜底强杀残留。
# ============================================================
set -u

kill_remnants() {
  # 兜底强杀。匹配串里不要出现本脚本名，否则会把自己也杀掉。
  pkill -9 -f "gzserver|gzclient|gazebo_ros|robot_state_publisher|component_container|rviz2|spawn_entity|slam_toolbox|rtabmap|fastlio_mapping|pointcloud_to_laserscan|teleop_twist_keyboard|map_saver|joint_state_publisher|opennav_coverage|coverage_executor" 2>/dev/null || true
  pkill -9 -x gazebo 2>/dev/null || true
}

case "${1:-all}" in
  rviz)
    pkill -9 -x rviz2 2>/dev/null || true
    echo "已关闭 rviz2"
    ;;
  gazebo)
    pkill -9 -x gzserver 2>/dev/null || true
    pkill -9 -x gzclient 2>/dev/null || true
    pkill -9 -x gazebo 2>/dev/null || true
    echo "已关闭 gazebo (gzserver/gzclient)"
    ;;
  all|"")
    echo "正在关闭导航栈..."
    # 1) 温和通知 launch 退出，它会负责清理自己启动的子进程
    pkill -INT -f "ros2 launch sam_bot_description" 2>/dev/null || true
    sleep 2
    # 2) 兜底强杀所有残留
    kill_remnants
    echo "已关闭全部相关进程"
    ;;
  -h|--help)
    echo "用法: $0 [all|rviz|gazebo]"
    exit 0
    ;;
  *)
    echo "未知参数: $1"
    echo "用法: $0 [all|rviz|gazebo]"
    exit 1
    ;;
esac
