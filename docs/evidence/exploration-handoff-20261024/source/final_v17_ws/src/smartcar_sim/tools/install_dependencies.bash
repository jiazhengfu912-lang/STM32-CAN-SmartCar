#!/usr/bin/env bash
set -euo pipefail
# 只用于用户确认的 Ubuntu22.04/Humble，不安装或迁移其它 ROS 发行版。
source /etc/os-release
if [ "$ID" != ubuntu ] || [ "$VERSION_ID" != 22.04 ]; then
  echo '此安装入口仅针对 Ubuntu22.04，请先确认虚拟机版本。'; exit 1
fi
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo '未找到 ROS2 Humble。请先完成 Humble 安装及官方软件源设置。'; exit 1
fi
sudo apt update
sudo apt install -y build-essential cmake ros-humble-ament-cmake \
  ignition-fortress ros-humble-ros-gz ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui ros-humble-rviz2 ros-humble-rqt-image-view \
  python3-colcon-common-extensions python3-matplotlib python3-yaml python3-pil ros-humble-tf2-ros liburdfdom-tools
echo '依赖安装完成；下一步在工作空间执行 colcon build。'
