#!/usr/bin/env python3
"""Verify real SLAM messages; optional closed-loop motion is explicit."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener
from map_validation import inspect_map


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


class Recorder(Node):
    def __init__(self, folder):
        super().__init__('smartcar_slam_verifier', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.folder = folder
        self.start_wall = time.monotonic()
        self.phase = 'wait'
        self.map = self.scan = self.odom = None
        self.last_odom_wall = None
        self.maps = []
        self.odom_rows = []
        self.command_rows = []
        self.scan_count = self.odom_count = 0
        self.first_clock = self.last_clock = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(Twist, '/sim/cmd_vel', 10)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.receivers = [
            self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos),
            self.create_subscription(LaserScan, '/sim/scan', self.on_scan, qos_profile_sensor_data),
            self.create_subscription(Odometry, '/sim/odom', self.on_odom, 10),
            self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)]
        self.csv_files = {}
        for name, columns in [
                ('map_updates', ['wall_s', 'sim_stamp_s', 'phase', 'width', 'height', 'free', 'occupied', 'unknown']),
                ('odom', ['wall_s', 'phase', 'sim_stamp_s', 'x', 'y', 'yaw', 'vx', 'wz']),
                ('commands', ['wall_s', 'phase', 'vx', 'wz'])]:
            stream = (folder / (name + '.csv')).open('w', newline='')
            writer = csv.writer(stream)
            writer.writerow(columns)
            self.csv_files[name] = (stream, writer)

    def record(self, name, row):
        stream, writer = self.csv_files[name]
        writer.writerow(row)
        stream.flush()

    def on_clock(self, msg):
        self.last_clock = msg.clock.sec + msg.clock.nanosec / 1e9
        if self.first_clock is None:
            self.first_clock = self.last_clock

    def on_map(self, msg):
        self.map = msg
        info, pose = msg.info, msg.info.origin
        details = inspect_map(msg.header.frame_id, info.width, info.height, info.resolution,
                              [pose.position.x, pose.position.y, pose.position.z,
                               pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
                              msg.data)
        self.maps.append(details)
        self.record('map_updates', [time.monotonic()-self.start_wall,
                                   msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                                   self.phase, info.width, info.height,
                                   details['counts']['free'], details['counts']['occupied'],
                                   details['counts']['unknown']])
        if len(self.maps) == 1 and details['checks']['dimensions_match_data']:
            self.save_map_sample('map_initial')

    def on_scan(self, msg):
        self.scan = msg
        self.scan_count += 1

    def on_odom(self, msg):
        self.odom = msg
        self.last_odom_wall = time.monotonic()
        self.odom_count += 1
        p, v = msg.pose.pose, msg.twist.twist
        row = [time.monotonic()-self.start_wall, self.phase,
               msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
               p.position.x, p.position.y, yaw(p.orientation), v.linear.x, v.angular.z]
        self.odom_rows.append(row)
        self.record('odom', row)

    def save_map_sample(self, name):
        msg = self.map
        values = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        np.savez_compressed(self.folder/(name+'.npz'), occupancy=values,
                            resolution=msg.info.resolution,
                            origin=[msg.info.origin.position.x, msg.info.origin.position.y])

    def command(self, vx, wz):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(vx), float(wz)
        self.publisher.publish(msg)
        row = [time.monotonic()-self.start_wall, self.phase, vx, wz]
        self.command_rows.append(row)
        self.record('commands', row)

    def spin_for(self, seconds, stopped=False):
        end = time.monotonic()+seconds
        next_command = 0
        while rclpy.ok() and time.monotonic() < end:
            if stopped and time.monotonic() >= next_command:
                self.command(0, 0)
                next_command = time.monotonic()+.1
            rclpy.spin_once(self, timeout_sec=.02)

    def pose(self):
        p = self.odom.pose.pose
        return p.position.x, p.position.y, yaw(p.orientation)

    def move_until(self, condition, vx, wz, phase, timeout=40):
        self.phase = phase
        end = time.monotonic()+timeout
        next_command = 0
        while not condition():
            if not rclpy.ok() or time.monotonic() >= end:
                raise TimeoutError(f'Motion stage timed out: {phase}; thresholds were not relaxed.')
            if self.last_odom_wall is None or time.monotonic()-self.last_odom_wall > .75:
                raise RuntimeError(f'Odometry stale during {phase}; stopped motion.')
            if time.monotonic() >= next_command:
                self.command(vx, wz)
                next_command = time.monotonic()+.1
            rclpy.spin_once(self, timeout_sec=.01)
        self.phase = phase+'_stop'
        self.spin_for(1.2, stopped=True)

    def drive_square(self, side):
        if self.count_publishers('/sim/cmd_vel') != 1:
            raise RuntimeError('Close keyboard/other command publishers before --drive-square.')
        origin = self.pose()
        for index in range(4):
            a = self.pose()
            self.move_until(lambda: math.hypot(self.pose()[0]-a[0], self.pose()[1]-a[1]) >= side,
                            .10, 0, f'straight_{index}')
            heading = self.pose()[2]
            self.move_until(lambda: angle(self.pose()[2]-heading) <= -math.pi/2+.02,
                            0, -.35, f'turn_{index}')
        last = self.pose()
        return {'side_m': side, 'start_odom': origin, 'end_odom': last,
                'return_distance_m': math.hypot(last[0]-origin[0], last[1]-origin[1]),
                'return_heading_error_rad': abs(angle(last[2]-origin[2]))}


def plot(node, folder):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    if node.map:
        msg = node.map
        values = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)
        image = np.where(values < 0, .5, 1-values/100)
        x, y = msg.info.origin.position.x, msg.info.origin.position.y
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.imshow(image, origin='lower', cmap='gray', vmin=0, vmax=1,
                  extent=[x, x+msg.info.width*msg.info.resolution,
                          y, y+msg.info.height*msg.info.resolution], interpolation='nearest')
        ax.set(xlabel='map x (m)', ylabel='map y (m)',
               title='Actual SLAM Toolbox OccupancyGrid / simulation, NOT hardware')
        for ext in ['png', 'svg']:
            fig.savefig(folder/('map.'+ext), dpi=150)
        plt.close(fig)
    if node.odom_rows:
        a = np.asarray([[r[0], *r[3:]] for r in node.odom_rows], dtype=float)
        fig, axes = plt.subplots(1, 2, figsize=(11, 4))
        axes[0].plot(a[:, 1], a[:, 2]); axes[0].set_aspect('equal', adjustable='datalim')
        axes[0].set(xlabel='odom x (m)', ylabel='odom y (m)', title='Actual wheel odometry')
        axes[1].plot(a[:, 0], a[:, 4], label='vx (m/s)')
        axes[1].plot(a[:, 0], a[:, 5], label='wz (rad/s)'); axes[1].legend()
        axes[1].set(xlabel='wall time (s)', title='Actual received odometry velocities')
        for ax in axes:
            ax.grid()
        fig.tight_layout()
        for ext in ['png', 'svg']:
            fig.savefig(folder/('motion.'+ext), dpi=150)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=20)
    parser.add_argument('--drive-square', action='store_true',
                        help='Send motion in a clear 0.6m clockwise square; close keyboard first.')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '86' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('First export SMARTCAR_WS and source smartcar_sim/tools/sim_env.bash.')
    if not math.isfinite(args.duration) or args.duration < 5:
        raise SystemExit('--duration must be at least 5 wall seconds.')
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    try:
        node = Recorder(args.output)
    except Exception as exc:
        report = {'result':'FAIL', 'scope':'ACTUAL_RUNTIME_INITIALIZATION', 'error':str(exc)}
        (args.output/'result.json').write_text(json.dumps(report, indent=2))
        rclpy.shutdown()
        raise SystemExit(str(exc))
    report = {'result': 'FAIL', 'scope': 'ACTUAL_GAZEBO_SLAM_ROS_MESSAGES_NOT_HARDWARE',
              'motion_requested': args.drive_square, 'checks': {}}
    error = None
    try:
        deadline = time.monotonic()+45
        while (node.map is None or node.scan is None or node.odom is None) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if node.map is None or node.scan is None or node.odom is None:
            raise RuntimeError('Missing /map, /sim/scan or /sim/odom after 45 wall seconds; no motion performed.')
        if args.drive_square:
            report['motion'] = node.drive_square(.6)
        node.phase = 'observe'
        node.spin_for(args.duration, stopped=args.drive_square)
        checks = report['checks']
        checks.update(node.maps[-1]['checks'])
        checks['single_map_publisher'] = node.count_publishers('/map') == 1
        checks['map_updates_received'] = len(node.maps) >= 2
        checks['clock_advancing'] = node.first_clock is not None and node.last_clock > node.first_clock
        checks['scan_received'] = node.scan_count >= 5 and node.scan.header.frame_id == 'laser_frame'
        checks['odom_received'] = node.odom_count >= 5
        checks['odom_frames'] = (node.odom.header.frame_id, node.odom.child_frame_id) == ('odom', 'base_footprint')
        transforms = {}
        for parent, child in [('map', 'odom'), ('odom', 'base_footprint'),
                              ('base_footprint', 'base_link'), ('base_link', 'laser_frame')]:
            transform = node.buffer.lookup_transform(parent, child, Time())
            t, q = transform.transform.translation, transform.transform.rotation
            values = [t.x, t.y, t.z, q.x, q.y, q.z, q.w]
            transforms[parent+'->'+child] = values
            checks['tf_'+parent+'_'+child] = all(math.isfinite(v) for v in values)
        node.buffer.lookup_transform('map', 'laser_frame', Time.from_msg(node.scan.header.stamp))
        checks['map_to_laser_at_scan_stamp'] = True
        map_stamp = node.map.header.stamp.sec+node.map.header.stamp.nanosec/1e9
        report['map_age_sim_s'] = node.last_clock-map_stamp
        checks['map_fresh'] = -.5 <= report['map_age_sim_s'] <= 6
        report.update(map=node.maps[-1], initial_map=node.maps[0], transforms=transforms,
                      map_updates=len(node.maps), scan_messages=node.scan_count, odom_messages=node.odom_count)
        if args.drive_square:
            checks['returned_near_start'] = report['motion']['return_distance_m'] < .15
            checks['returned_near_heading'] = report['motion']['return_heading_error_rad'] < .2
            checks['stopped_after_motion'] = abs(node.odom.twist.twist.linear.x) < .02 and abs(node.odom.twist.twist.angular.z) < .05
    except (Exception, KeyboardInterrupt) as exc:
        error = str(exc) or type(exc).__name__
    finally:
        if args.drive_square and rclpy.ok():
            node.phase = 'final_stop'
            try:
                node.spin_for(.5, stopped=True)
            except Exception:
                pass
        if node.map is not None:
            try:
                node.save_map_sample('map_final')
            except Exception as exc:
                error = (error+'; ' if error else '')+'Map snapshot failed: '+str(exc)
        if node.scan is not None:
            with (args.output/'scan.csv').open('w', newline='') as stream:
                writer = csv.writer(stream); writer.writerow(['angle_rad', 'range_m'])
                writer.writerows((node.scan.angle_min+i*node.scan.angle_increment, v)
                                  for i, v in enumerate(node.scan.ranges))
        for stream, writer in node.csv_files.values():
            stream.close()
        try:
            plot(node, args.output)
        except Exception as exc:
            error = (error+'; ' if error else '')+'Plot failed: '+str(exc)
        report['error'] = error
        report['result'] = 'PASS' if report['checks'] and all(report['checks'].values()) and error is None else 'FAIL'
        (args.output/'result.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps(report, indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if report['result'] == 'PASS' else 1)


if __name__ == '__main__':
    main()
