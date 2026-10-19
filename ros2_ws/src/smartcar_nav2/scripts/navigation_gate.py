#!/usr/bin/env python3
"""Gate only navigation commands. Keyboard /sim/cmd_vel is disconnected in Nav2 mode."""
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from std_srvs.srv import SetBool
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from gate_core import NavigationGate


class GateNode(Node):
    def __init__(self):
        super().__init__('navigation_gate')
        self.core = NavigationGate()
        config = json.loads((Path(get_package_share_directory('smartcar_sim'))/'config/robot_parameters.json').read_text())
        self.laser_xy = config['lidar']['mount_xyz_relative_to_axle_m'][:2]
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.output_pub = self.create_publisher(Twist, '/nav/safe_cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/navigation/safety_status', 10)
        self.create_subscription(Twist, '/nav/cmd_vel', self.command, 10)
        self.create_subscription(LaserScan, '/sim/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/sim/odom', self.odom, 10)
        self.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose, 10)
        self.cancel_clients = [self.create_client(CancelGoal, name+'/_action/cancel_goal')
                               for name in ['/navigate_to_pose', '/navigate_through_poses', '/follow_waypoints']]
        self.create_service(SetBool, '/navigation/stop', self.stop)
        self.last_status = 0.
        self.last_interface_check = 0.
        self.interfaces_ok = False
        # Wall clock is intentional: paused simulation must not keep nonzero commands alive.
        self.create_timer(.05, self.publish)

    def command(self, msg):
        self.core.receive_command(msg.linear.x, msg.angular.z, time.monotonic())

    def scan(self, msg):
        if msg.header.frame_id != 'laser_frame' or not msg.ranges:
            self.core.scan_time = None
            return
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if self.core.clock_value is not None and not -.2 <= self.core.clock_value-stamp <= .8:
            self.core.scan_time = None
            return
        points = []
        for i, distance in enumerate(msg.ranges):
            if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:
                angle = msg.angle_min+i*msg.angle_increment
                points.append((self.laser_xy[0]+distance*math.cos(angle),
                               self.laser_xy[1]+distance*math.sin(angle)))
        if not points:
            self.core.scan_time = None
            return
        self.core.points = points
        self.core.scan_time = time.monotonic()

    def odom(self, msg):
        if msg.header.frame_id == 'odom' and msg.child_frame_id == 'base_footprint':
            self.core.odom_time = time.monotonic()

    def clock(self, msg):
        self.core.receive_clock(msg.clock.sec+msg.clock.nanosec/1e9, time.monotonic())

    def pose(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.core.localized = msg.header.frame_id == 'map' and all(math.isfinite(v) for v in [p.x,p.y,q.z,q.w])

    def stop(self, request, response):
        self.core.estop = request.data
        if request.data:
            for client in self.cancel_clients:
                if client.service_is_ready():
                    client.call_async(CancelGoal.Request())
        self.publish()
        response.success = True
        response.message = 'Stop latched; active NavigateToPose cancellation requested.' if request.data else 'Stop released; send a new goal.'
        return response

    def publish(self):
        now = time.monotonic()
        if now-self.last_interface_check > 1.:
            self.interfaces_ok = (self.count_publishers('/sim/scan') == 1 and
                                  self.count_publishers('/sim/drive_cmd_vel') == 1 and
                                  self.count_publishers('/nav/safe_cmd_vel') == 1 and
                                  [p.node_name for p in self.get_publishers_info_by_topic('/map')] == ['map_server'])
            self.last_interface_check = now
        vx, wz, reason = self.core.output(now)
        if reason == 'READY':
            if not self.interfaces_ok:
                vx, wz, reason = 0., 0., 'INTERFACE_CONFLICT'
            try:
                transform = self.buffer.lookup_transform('map', 'odom', Time())
                stamp = transform.header.stamp.sec+transform.header.stamp.nanosec/1e9
                if self.core.clock_value is None or not -1.0 <= self.core.clock_value-stamp <= 1.5:
                    raise RuntimeError('map->odom stale')
                self.buffer.lookup_transform('odom', 'base_footprint', Time())
            except Exception:
                vx, wz, reason = 0., 0., 'TF_UNAVAILABLE'
        msg = Twist();msg.linear.x = vx;msg.angular.z = wz
        self.output_pub.publish(msg)
        if now-self.last_status > .25:
            status = String();status.data = json.dumps({'state': reason, 'vx': vx, 'wz': wz, 'stop_latched': self.core.estop})
            self.status_pub.publish(status);self.last_status = now


def main():
    rclpy.init();node = GateNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node();rclpy.try_shutdown()


if __name__ == '__main__':
    main()
