#!/usr/bin/env python3
"""W/S前后，A/D转弯，空格停止；松键或终端退出后不保持运动。"""
import os
import json
from pathlib import Path
import select
import sys
import termios
import time
import tty
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist


def main():
    if not sys.stdin.isatty():raise SystemExit('请在 Ubuntu 交互终端运行键盘控制。')
    if os.environ.get('ROS_DOMAIN_ID')!='86' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise SystemExit('请先 source 本工程 tools/sim_env.bash，使用仿真 domain86 / localhost。')
    rclpy.init();node=rclpy.create_node('sim_keyboard');publisher=node.create_publisher(Twist,'/sim/cmd_vel',10)
    settings=termios.tcgetattr(sys.stdin);last_key=-1;command=(0.0,0.0)
    config=json.loads((Path(get_package_share_directory('smartcar_sim'))/'config/robot_parameters.json').read_text())['control']
    speed=config['keyboard_linear_mps'];turn=config['keyboard_angular_radps']
    forward=config.get('keyboard_forward_mps',speed)
    keys={'w':(forward,0.0),'s':(-speed,0.0),'a':(0.0,turn),'d':(0.0,-turn),' ':(0.0,0.0)}
    print(f'仿真专用：W前进 {forward:g} m/s，S后退 {speed:g} m/s，A/D转弯 {turn:g} rad/s，空格停止，Ctrl+C退出。\n短按点动；按住通过键盘自动重复持续运动。')
    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin],[],[],.05)[0]:
                char=sys.stdin.read(1).lower()
                if char=='\x03':break
                command=keys.get(char,(0.0,0.0));last_key=time.monotonic()
            current=command if time.monotonic()-last_key<.20 else (0.0,0.0)
            msg=Twist();msg.linear.x,msg.angular.z=current;publisher.publish(msg)
            rclpy.spin_once(node,timeout_sec=0)
    finally:
        termios.tcsetattr(sys.stdin,termios.TCSADRAIN,settings)
        if rclpy.ok():publisher.publish(Twist());node.destroy_node();rclpy.shutdown()
        print('\n已发送停止指令。')


if __name__=='__main__':main()
