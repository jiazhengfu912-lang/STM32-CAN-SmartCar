#!/usr/bin/env python3
"""Read-only Nav2 readiness snapshot; never sends a navigation or velocity command."""
import argparse
import json
import os
from pathlib import Path
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not os.environ.get('ROS_DOMAIN_ID') or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('First source smartcar_sim/tools/sim_env.bash.')
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init();node = Node('smartcar_nav2_check')
    buffer = Buffer();listener = TransformListener(buffer, node)
    received = {};checks = {};states = {};error = None
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def callback(name):
        return lambda msg: received.update({name: msg})
    subs = [node.create_subscription(OccupancyGrid, '/map', callback('map'), qos),
            node.create_subscription(LaserScan, '/sim/scan', callback('scan'), qos_profile_sensor_data),
            node.create_subscription(Odometry, '/sim/odom', callback('odom'), 10),
            node.create_subscription(String, '/navigation/safety_status', callback('gate'), 10)]
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    try:
        until = time.monotonic()+5
        while time.monotonic()<until:
            rclpy.spin_once(node, timeout_sec=.1)
        checks.update({name+'_received': name in received for name in ['map','scan','odom','gate']})
        checks['action_server_ready'] = client.server_is_ready()
        checks['one_map_authority'] = [p.node_name for p in node.get_publishers_info_by_topic('/map')] == ['map_server']
        checks['single_scan_publisher'] = node.count_publishers('/sim/scan') == 1
        checks['single_drive_publisher'] = node.count_publishers('/sim/drive_cmd_vel') == 1
        for name in ['map_server','amcl','controller_server','planner_server','behavior_server','bt_navigator','velocity_smoother']:
            service = node.create_client(GetState, '/'+name+'/get_state')
            if service.wait_for_service(timeout_sec=2):
                future = service.call_async(GetState.Request());rclpy.spin_until_future_complete(node, future, timeout_sec=3)
                states[name] = future.result().current_state.label if future.done() and future.result() else 'no_response'
            else:states[name] = 'unavailable'
            checks[name+'_active'] = states[name] == 'active'
        for parent, child in [('map','odom'),('odom','base_footprint'),('base_footprint','laser_frame')]:
            try:buffer.lookup_transform(parent, child, Time());checks['tf_'+parent+'_'+child] = True
            except Exception:checks['tf_'+parent+'_'+child] = False
        if 'gate' in received:
            gate = json.loads(received['gate'].data)
            checks['stop_not_latched'] = not gate['stop_latched']
            checks['sensors_and_localization_ready'] = gate['state'] in ['READY','COMMAND_STALE']
        else:gate = None
    except Exception as exc:
        error = str(exc);gate = None
    report = {'result': 'PASS' if checks and all(checks.values()) and not error else 'FAIL',
              'scope':'READ_ONLY_NAV2_READINESS_NOT_GOAL_EXECUTION', 'checks':checks, 'lifecycle_states':states, 'gate':gate, 'error':error}
    (args.output/'result.json').write_text(json.dumps(report, indent=2));print(json.dumps(report, indent=2))
    node.destroy_node();rclpy.try_shutdown()
    raise SystemExit(0 if report['result']=='PASS' else 1)


if __name__ == '__main__':
    main()
