#!/usr/bin/env bash
set -eo pipefail
# 手动关闭旧导航后运行；从本轮快照恢复场景，不发送目标或速度指令。
# 第一个可选参数为 RViz true/false，已有独立 RViz 时传 false。
SMARTCAR_RESUME_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SMARTCAR_RESUME_RVIZ="${1:-true}"
if (( $# )); then shift; fi
source "$SMARTCAR_RESUME_DIR/../../../../ros2_ws/tools/env.bash"
exec ros2 launch smartcar_nav2 navigation.launch.py \
  world:="$SMARTCAR_RESUME_DIR/resume_world.sdf" \
  initial_x:=-0.473147 initial_y:=2.94194 initial_yaw:=-0.119911 \
  headless:=false rviz:="$SMARTCAR_RESUME_RVIZ" "$@"
