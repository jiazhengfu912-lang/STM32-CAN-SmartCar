#!/usr/bin/env bash
set -euo pipefail
# 密码仅由本机sudo交互读取；不修改ROS发行版或自动添加第三方软件源。
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 22.04 ]]; then
  echo '本入口要求 Ubuntu 22.04 + ROS 2 Humble + Gazebo Fortress。' >&2
  exit 1
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo '请先按 ROS 官方文档安装 Humble，并配置 Gazebo Fortress 官方软件源。' >&2
  exit 1
fi
sudo apt-get update
sudo apt-get install -y build-essential cmake ignition-fortress \
  python3-colcon-common-extensions python3-pytest python3-numpy python3-matplotlib \
  python3-yaml python3-pil liburdfdom-tools \
  ros-humble-ament-cmake ros-humble-ament-cmake-pytest ros-humble-ros-gz \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 ros-humble-rqt-image-view ros-humble-tf2-ros \
  ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-rviz-plugins
