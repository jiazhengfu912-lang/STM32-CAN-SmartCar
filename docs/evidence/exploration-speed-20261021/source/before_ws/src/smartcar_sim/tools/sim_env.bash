#!/usr/bin/env bash
# 兼容源码、符号链接和普通安装入口，向上寻找本仓库的工作空间。
_smartcar_env_search="$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")"
while [[ "$_smartcar_env_search" != / ]]; do
  if [[ -f "$_smartcar_env_search/tools/env.bash" && -f "$_smartcar_env_search/src/smartcar_sim/package.xml" ]]; then
    if source "$_smartcar_env_search/tools/env.bash"; then
      unset _smartcar_env_search
      return 0 2>/dev/null || exit 0
    fi
    unset _smartcar_env_search
    return 1 2>/dev/null || exit 1
  fi
  _smartcar_env_search="$(dirname -- "$_smartcar_env_search")"
done
unset _smartcar_env_search
echo '未找到源码工作空间；请从仓库根目录 source ros2_ws/tools/env.bash。' >&2
return 1 2>/dev/null || exit 1
