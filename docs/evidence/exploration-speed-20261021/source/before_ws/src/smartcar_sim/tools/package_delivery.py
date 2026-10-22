#!/usr/bin/env python3
"""停用原单包打包入口；当前仿真依赖整个 ros2_ws。"""
import sys


def main():
    """拒绝输出缺少两个上层包与环境入口的不完整 ZIP。"""
    print('原单包交付已停用。请克隆完整 STM32-CAN-SmartCar 仓库，或在仓库根目录运行：\n'
          'git archive --format=zip --output=/绝对路径/smartcar-source.zip HEAD\n'
          '归档只包含已提交文件；构建与运行入口见 ros2_ws/README.md。', file=sys.stderr)
    return 2


if __name__ == '__main__':
    raise SystemExit(main())
