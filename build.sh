#!/usr/bin/env bash
set -e

# 【说明】标准 colcon 工作区布局：根目录（本脚本所在处）不再是包，
# 所有包平铺在 src/ 下（src/sam_bot_description、src/mid360_simulation），
# 普通 colcon build 即可自动发现全部包，无需 --base-paths。
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash

colcon build --symlink-install
