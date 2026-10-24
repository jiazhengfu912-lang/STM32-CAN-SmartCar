#!/usr/bin/env bash
# 从脚本位置找到本仓库工作空间；移动/克隆目录后无需修改绝对路径。
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo '未找到 ROS 2 Humble：/opt/ros/humble/setup.bash' >&2
  return 1 2>/dev/null || exit 1
fi
export SMARTCAR_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "$SMARTCAR_WS/install/local_setup.bash" ]]; then
  source "$SMARTCAR_WS/install/local_setup.bash"
fi
# 默认保持原仿真域；并行仿真须同时隔离 ROS_DOMAIN_ID 和 IGN_PARTITION。
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-86}"
export ROS_LOCALHOST_ONLY=1
