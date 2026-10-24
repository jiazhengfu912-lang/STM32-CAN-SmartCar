#!/usr/bin/env python3
"""仅服务 /sim/ 话题；指令断流300ms后发布零速度。"""
import time
import json
from pathlib import Path
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from geometry_msgs.msg import Twist
from command_guard_core import CommandGuard


class GuardNode(Node):
    def __init__(self):
        super().__init__('sim_cmd_guard')
        config=json.loads((Path(get_package_share_directory('smartcar_sim'))/'config/robot_parameters.json').read_text())['control']
        self.guard=CommandGuard(config['timeout_s'],config['max_linear_mps'],config['max_angular_radps'])
        self.publisher=self.create_publisher(Twist,'/sim/drive_cmd_vel',10)
        self.create_subscription(Twist,'/sim/cmd_vel',self.receive,10)
        self.create_timer(0.02,self.publish)

    def receive(self,msg):
        self.guard.receive(msg.linear.x,msg.angular.z,time.monotonic())

    def publish(self):
        linear,angular=self.guard.output(time.monotonic())
        msg=Twist();msg.linear.x=linear;msg.angular.z=angular;self.publisher.publish(msg)


def main():
    rclpy.init();node=GuardNode()
    try:rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):pass
    finally:
        try:
            if rclpy.ok():
                node.publisher.publish(Twist())
        except _rclpy.RCLError:
            # Humble may invalidate the context between ok() and publish() on SIGINT.
            if rclpy.ok():raise
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__=='__main__':main()
