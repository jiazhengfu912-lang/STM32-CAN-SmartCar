#!/usr/bin/env bash
set -eo pipefail
# 在独立shell中清除旧工作空间覆盖，避免构建产物引用开发者机器上的另一份包。
SMARTCAR_BUILD_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
cd "$SMARTCAR_BUILD_WS"
colcon build --symlink-install --packages-select smartcar_sim smartcar_slam smartcar_nav2 "$@"
