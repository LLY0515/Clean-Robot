#!/usr/bin/env bash
# ============================================================
#  一键键盘建图脚本（重新生成地图）
#
#  流程：Gazebo 仿真 + FAST-LIO 里程计 + 在线建图（RTAB-Map 或 slam_toolbox）
#        + 键盘遥控，边开边建；按 Ctrl+C 结束驾驶后自动保存地图。
#
#  用法：./build_map.sh [地图名]      （地图名默认 my_map，会覆盖原图）
#        SLAM_ALGO=slam_toolbox ./build_map.sh   切回 slam_toolbox
#  输出：src/sam_bot_description/map/<地图名>.yaml / .pgm
# ============================================================

# 用脚本自身所在目录作为工作区根目录
cd "$(dirname "$0")"

# ---------- 可配置项 ----------
MAP_NAME="${1:-my_map}"                              # 保存的地图名
MAP_DIR="$(pwd)/src/sam_bot_description/map"         # 保存目录
SLAM_PARAMS="$(pwd)/src/sam_bot_description/config/mapper_params_online_async.yaml"
# 【改动】建图算法可切换：rtabmap（默认）| slam_toolbox
#   用环境变量覆盖：SLAM_ALGO=slam_toolbox ./build_map.sh
SLAM_ALGO="${SLAM_ALGO:-rtabmap}"
# 保存地图时读取的话题（两种算法都发 /map）
MAP_TOPIC="${MAP_TOPIC:-/map}"
# RTAB-Map 数据库：每趟建图都从零开始，否则会接着上一趟的图继续
RTABMAP_DB="/tmp/rtabmap_${MAP_NAME}.db"
# 【改动】建图专用 FAST-LIO 配置：点云密度比导航默认高一档
# （每帧全部点、0.2 m 体素）。仿真雷达每帧只有 2500 条射线，导航默认配置
# 又每 3 点取 1 + 0.5 m 体素，里程计实测漂 1.6%（16.7 m 走 0.22 m）；
# 用这份配置后同样路线误差降到 0.06~0.12 m，重影明显减轻。
FASTLIO_CONFIG="$(pwd)/src/fast_lio/config/mid360_sim_2dmap.yaml"
# 【改动】话题就绪探针：用 rclpy 订阅，不用 `ros2 topic echo`
# （本机 ros2cli 守护进程异常，echo 有数据也会报 "!rclpy.ok()" 返回非 0，
#   导致脚本误判"60 秒内 /scan 仍无数据"）
TOPIC_PROBE="$(pwd)/src/sam_bot_description/tools/topic_ready.py"
# 【改动】存图改用自带的 save_map.py，不用 nav2 的 map_saver_cli。
# map_saver_cli 只等 2 秒（save_map_timeout），而 slam_toolbox 每 5 秒才发一次
# /map、rtabmap 只在图变化时才发，机器人一停就容易错过 → 报
# "Failed to spin map subscription" 并存不出图（实测 5 次里坏 3 次）。
SAVE_TOOL="$(pwd)/src/sam_bot_description/tools/save_map.py"
AUTOSAVE_SEC=20                                      # 后台自动保存间隔（秒）

# ---------- 清理残留进程 ----------
echo "[清理] 关闭残留的仿真/建图进程 ..."
pkill -9 -f "gzserver|gzclient|robot_state_publisher|fastlio_mapping|pointcloud_to_laserscan|slam_toolbox|rtabmap|teleop_twist_keyboard|rviz2|joint_state_publisher|spawn_entity" 2>/dev/null || true
sleep 2

# ---------- 环境 ----------
source /opt/ros/humble/setup.bash
if [ ! -f install/setup.bash ]; then
  echo "✗ 找不到 install/setup.bash，请先构建工作区：./build.sh"
  exit 1
fi
source install/setup.bash

# 建图算法可用性检查
if [ "$SLAM_ALGO" = "rtabmap" ] && ! ros2 pkg prefix rtabmap_ros >/dev/null 2>&1; then
  echo "✗ 选择了 RTAB-Map，但系统里没有 rtabmap_ros。"
  echo "  安装：sudo apt install -y ros-humble-rtabmap-ros"
  echo "  或切回旧算法：SLAM_ALGO=slam_toolbox ./build_map.sh"
  exit 1
fi

# 本机（虚拟显卡 + Qt GTK 主题）需要的环境变量，否则 rviz2 崩溃
# 【改动】原路径硬编码为 /home/rostwo，本机不存在该目录；指向不存在的
# GSETTINGS_SCHEMA_DIR 会让 GTK 找不到系统 schema，反而更容易崩。
if [ -d "$HOME/.local/share/glib-2.0/schemas" ]; then
  export GSETTINGS_SCHEMA_DIR="$HOME/.local/share/glib-2.0/schemas"
fi
export QT_QPA_PLATFORM=xcb
# 【改动】与 run.sh 一致：让 Gazebo/RViz 用独显渲染，否则默认落到 Intel 核显。
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA

# ---------- 后台进程用 setsid 隔离到独立会话，避免 Ctrl+C 把它们一起杀掉 ----------
echo "[1/4] 启动 Gazebo 仿真 + FAST-LIO + 点云转扫描 + RViz（后台）..."
setsid ros2 launch sam_bot_description display.launch.py gui:=False \
  fastlio_config:="$FASTLIO_CONFIG" \
  > /tmp/build_map_sim.log 2>&1 < /dev/null &

echo "[2/4] 等待激光 /scan 有数据 ..."
if ! python3 "$TOPIC_PROBE" /scan --timeout 90 --best-effort; then
  echo "✗ 90 秒内 /scan 仍无数据，请查看日志：/tmp/build_map_sim.log"
  exit 1
fi
echo "      ✓ /scan 已开始发布"

# 记下本次拉起的 gzserver PID：后台自动保存只在它活着时才写文件，
# 避免仿真被外部杀掉后，把别的进程发布的 /map 覆盖到正式地图上。
SIM_PID="$(pgrep -n -f 'gzserver .*my_carpark' 2>/dev/null || true)"

if [ "$SLAM_ALGO" = "rtabmap" ]; then
  echo "[3/4] 启动 RTAB-Map 在线建图（后台）..."
  rm -f "$RTABMAP_DB"
  setsid ros2 launch sam_bot_description rtabmap_mapping.launch.py \
    database_path:="$RTABMAP_DB" \
    > /tmp/build_map_slam.log 2>&1 < /dev/null &
else
  echo "[3/4] 启动 slam_toolbox 在线建图（后台）..."
  setsid ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:="$SLAM_PARAMS" \
    > /tmp/build_map_slam.log 2>&1 < /dev/null &
fi

# 等 SLAM 完成初始化并发出第一张 /map（slam_toolbox 的 map_update_interval=5 s，
# RTAB-Map 拿到第一帧扫描后立即发；给 30 s 足够）
echo "      等待 /map 生成 ..."
if python3 "$TOPIC_PROBE" "$MAP_TOPIC" --timeout 30; then
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
  timeout 90 python3 "$SAVE_TOOL" "$MAP_DIR/$MAP_NAME" --topic "$MAP_TOPIC"
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
    # 【防护】本次脚本的 gzserver 已经退出 → 停止自动保存。
    # 否则会把这个进程之外、其它仿真/测试节点发布的 /map 写进正式地图文件。
    if [ -n "$SIM_PID" ] && ! kill -0 "$SIM_PID" 2>/dev/null; then
      echo ""
      echo ">>> ! 仿真进程已退出，停止后台自动保存（地图不再更新）"
      break
    fi
    timeout 60 python3 "$SAVE_TOOL" "$MAP_DIR/$MAP_NAME" --topic "$MAP_TOPIC" \
      --timeout 10 --settle 1 >/dev/null 2>&1
  done
}

# ---------- 清理 ----------
CLEANED=0
cleanup() {
  [ "$CLEANED" = "1" ] && return 0
  CLEANED=1
  kill "$AUTOSAVE_PID" 2>/dev/null || true
  echo ">>> 关闭后台仿真/建图进程 ..."
  pkill -9 -f "gzserver|gzclient|robot_state_publisher|fastlio_mapping|pointcloud_to_laserscan|slam_toolbox|rtabmap|rviz2|joint_state_publisher|spawn_entity" 2>/dev/null || true
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
