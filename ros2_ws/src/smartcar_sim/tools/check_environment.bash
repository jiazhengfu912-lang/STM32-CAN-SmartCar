#!/usr/bin/env bash
set -eu
# 只读检查环境并输出版本，供首次运行失败时定位问题。
cat /etc/os-release
echo "ROS_DISTRO=${ROS_DISTRO:-未加载}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}, ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}"
command -v ros2
command -v colcon
command -v ign
ign gazebo --versions
ros2 pkg prefix ros_gz_bridge
ros2 pkg prefix smartcar_sim
if command -v glxinfo >/dev/null 2>&1; then glxinfo -B; fi
