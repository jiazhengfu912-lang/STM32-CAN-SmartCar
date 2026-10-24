#!/usr/bin/env bash
set -eo pipefail
# 仅运行软件测试；Gazebo运行验收由独立入口明确执行并保留日志。
source "$(dirname -- "${BASH_SOURCE[0]}")/env.bash"
cd "$SMARTCAR_WS"
colcon test --packages-select smartcar_sim smartcar_slam smartcar_nav2 --event-handlers console_direct+
colcon test-result --verbose
