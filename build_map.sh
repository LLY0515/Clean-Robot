#!/usr/bin/env bash
# ============================================================
#  一键键盘建图脚本（重新生成地图）
#
#  流程：Gazebo 仿真 + FAST-LIO 里程计 + slam_toolbox 在线建图
#        + 键盘遥控，边开边建；按 Ctrl+C 结束驾驶后自动保存地图。
#
#  用法：./build_map.sh [地图名]      （地图名默认 my_map，会覆盖原图）
#  输出：src/sam_bot_description/map/<地图名>.yaml / .pgm
# ============================================================

# 用脚本自身所在目录作为工作区根目录
cd "$(dirname "$0")"

# ---------- 可配置项 ----------
MAP_NAME="${1:-my_map}"                              # 保存的地图名
MAP_DIR="$(pwd)/src/sam_bot_description/map"         # 保存目录
SLAM_PARAMS="$(pwd)/src/sam_bot_description/config/mapper_params_online_async.yaml"
AUTOSAVE_SEC=20                                      # 后台自动保存间隔（秒）

# ---------- 清理残留进程 ----------
echo "[清理] 关闭残留的仿真/建图进程 ..."
pkill -9 -f "gzserver|gzclient|robot_state_publisher|fastlio_mapping|pointcloud_to_laserscan|slam_toolbox|teleop_twist_keyboard|rviz2|joint_state_publisher|spawn_entity" 2>/dev/null || true
sleep 2

# ---------- 环境 ----------
source /opt/ros/humble/setup.bash
if [ ! -f install/setup.bash ]; then
  echo "✗ 找不到 install/setup.bash，请先构建工作区：./build.sh"
  exit 1
fi
source install/setup.bash

# 本机（VMware 虚拟显卡 + Qt GTK 主题）需要的环境变量，否则 rviz2 崩溃
export GSETTINGS_SCHEMA_DIR=/home/rostwo/.local/share/glib-2.0/schemas
export QT_QPA_PLATFORM=xcb

# ---------- 后台进程用 setsid 隔离到独立会话，避免 Ctrl+C 把它们一起杀掉 ----------
echo "[1/4] 启动 Gazebo 仿真 + FAST-LIO + 点云转扫描 + RViz（后台）..."
setsid ros2 launch sam_bot_description display.launch.py gui:=False \
  > /tmp/build_map_sim.log 2>&1 < /dev/null &

echo "[2/4] 等待激光 /scan 有数据 ..."
READY=0
for _ in $(seq 1 60); do
  if timeout 3 ros2 topic echo /scan --once --qos-reliability best_effort >/dev/null 2>&1; then
    READY=1; break
  fi
  sleep 1
done
if [ "$READY" != "1" ]; then
  echo "✗ 60 秒内 /scan 仍无数据，请查看日志：/tmp/build_map_sim.log"
  exit 1
fi

echo "[3/4] 启动 slam_toolbox 在线建图（后台）..."
setsid ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:="$SLAM_PARAMS" \
  > /tmp/build_map_slam.log 2>&1 < /dev/null &

# 等 SLAM 完成初始化并发出第一张 /map
echo "      等待 /map 生成 ..."
MAP_READY=0
for _ in $(seq 1 30); do
  if timeout 3 ros2 topic echo /map --once >/dev/null 2>&1; then
    MAP_READY=1; break
  fi
  sleep 1
done
if [ "$MAP_READY" = "1" ]; then
  echo "      ✓ /map 已开始发布"
else
  echo "      ! 暂未收到 /map（继续运行，保存时再检查）"
fi

# ---------- 保存 ----------
SAVED=0

save_map() {
  [ "$SAVED" = "1" ] && return 0
  SAVED=1
  # 先停掉后台自动保存，避免两个 map_saver 同时写同一个文件
  kill "$AUTOSAVE_PID" 2>/dev/null || true

  mkdir -p "$MAP_DIR"
  echo ""
  echo ">>> 保存地图到 $MAP_DIR/$MAP_NAME ..."
  # 注意：map_saver_cli 的 -t 是"地图话题名"不是超时；收不到 /map 会一直等，
  # 所以外面套 timeout 兜底。
  timeout 60 ros2 run nav2_map_server map_saver_cli \
      -f "$MAP_DIR/$MAP_NAME" -t /map --ros-args -p use_sim_time:=True
  local rc=$?

  if [ -f "$MAP_DIR/$MAP_NAME.yaml" ] && [ -f "$MAP_DIR/$MAP_NAME.pgm" ]; then
    echo ">>> ✓ 已保存：$MAP_DIR/$MAP_NAME.yaml"
    echo ">>> ✓        $MAP_DIR/$MAP_NAME.pgm"
    ls -lh "$MAP_DIR/$MAP_NAME".{yaml,pgm} 2>/dev/null | sed 's/^/    /'
  else
    echo ">>> ✗ 保存失败（退出码 $rc）：没有收到 /map 或写入失败"
    echo "    请检查：/tmp/build_map_slam.log 里是否有 'Message Filter dropping message'"
    SAVED=0
    return 1
  fi
}

# ---------- 后台自动保存：即使 Ctrl+C 处理异常，也最多丢 20 秒的建图成果 ----------
autosave_loop() {
  while true; do
    sleep "$AUTOSAVE_SEC"
    [ "$SAVED" = "1" ] && break
    timeout 30 ros2 run nav2_map_server map_saver_cli \
      -f "$MAP_DIR/$MAP_NAME" -t /map --ros-args -p use_sim_time:=True >/dev/null 2>&1
  done
}

# ---------- 清理 ----------
CLEANED=0
cleanup() {
  [ "$CLEANED" = "1" ] && return 0
  CLEANED=1
  kill "$AUTOSAVE_PID" 2>/dev/null || true
  echo ">>> 关闭后台仿真/建图进程 ..."
  pkill -9 -f "gzserver|gzclient|robot_state_publisher|fastlio_mapping|pointcloud_to_laserscan|slam_toolbox|rviz2|joint_state_publisher|spawn_entity" 2>/dev/null || true
  echo ">>> 完成。"
}

# Ctrl+C / kill：先存图再清理。即使这里没被触发，teleop 退出后的主流程也会存图。
trap 'echo; echo ">>> 收到 Ctrl+C"; save_map; cleanup; exit 0' INT TERM
trap cleanup EXIT

# 后台兜底自动保存
autosave_loop &
AUTOSAVE_PID=$!

# ---------- 键盘驾驶 ----------
echo ""
echo "=============================================="
echo "  键盘驾驶中，边开边建图"
echo "  u i o / j k l / m , .   控制移动"
echo "  q/z 加减线速度   w/x 加减角速度"
echo "  按 Ctrl+C 结束驾驶，脚本会自动保存地图"
echo "  （每 ${AUTOSAVE_SEC}s 还会自动存一次，防丢）"
echo "=============================================="

ros2 run teleop_twist_keyboard teleop_twist_keyboard

# teleop 退出（通常是 Ctrl+C）后，主流程继续 —— 不依赖 trap 是否触发
save_map
cleanup
exit 0
