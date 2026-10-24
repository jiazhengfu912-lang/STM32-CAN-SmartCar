#!/usr/bin/env python3
"""显式启动隔离的 Gazebo，验证一个短距离 Nav2 目标并保留全部原始证据。

使用：python3 ros2_ws/tools/verify_navigation.py --output /绝对路径/全新目录
默认使用 ROS 域 88；拒绝域 86。自动加载本仓库环境并创建随机 Gazebo 分区。
仅验证仿真导航集成，不验证 STM32 PI、实物、绕障、多目标或恢复行为。
"""

import argparse
import csv
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback
import uuid


def arguments():
    """读取显式运行参数，禁止进入用户原有仿真域。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--domain-id', type=int, default=88)
    parser.add_argument('--startup-timeout', type=float, default=60.)
    parser.add_argument('--goal-timeout', type=float, default=120.)
    parser.add_argument('--map-file',type=Path,help='显式验证新建地图；默认保持原示例地图')
    parser.add_argument('--evidence-label',default='ROS integration 20261018')
    parser.add_argument('--_environment-ready', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.domain_id == 86 or not 0 <= args.domain_id <= 232:
        parser.error('必须使用 0..232 内且不是 86 的隔离 ROS 域；默认 88。')
    if not 0 < args.startup_timeout <= 60 or not math.isfinite(args.goal_timeout) or args.goal_timeout <= 0:
        parser.error('启动等待须为 0..60 秒，目标等待须为有限正数。')
    args.output = args.output.expanduser().resolve()
    if args.map_file:
        args.map_file=args.map_file.expanduser().resolve()
        if not args.map_file.is_file():parser.error('地图文件不存在')
    return args


def bootstrap(args, workspace):
    """在导入 ROS 前加载本仓库环境，清除其他工作空间覆盖路径。"""
    if args._environment_ready:
        return
    if args.output.exists():
        raise FileExistsError('拒绝覆盖已有证据目录：' + str(args.output))
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(args.domain_id)
    env['ROS_LOCALHOST_ONLY'] = '1'
    command = ('set -e; unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH '
               'ROS_PACKAGE_PATH PYTHONPATH; source "$1"; '
               'exec /usr/bin/python3 "$2" "${@:3}" --_environment-ready')
    os.execvpe('bash', ['bash', '-c', command, 'smartcar-navigation-verifier',
                      str(workspace/'tools/env.bash'), str(Path(__file__).resolve()),
                      *sys.argv[1:]], env)


def live_group_members(group):
    """仅列出本脚本创建的进程组中的非僵尸进程，不按程序名称匹配。"""
    found = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry/'stat').read_text().rsplit(')', 1)[1].split()
            if int(fields[2]) == group and fields[0] != 'Z':
                found.append(int(entry.name))
        except (OSError, ValueError, IndexError):
            continue
    return found


def stop_group(process, log_event, pump=None):
    """先 SIGINT，再有界升级信号；始终只清理自己创建的会话进程组。"""
    group = process.pid
    result = {'process_group': group, 'signals': [], 'remaining_pids': []}
    for sig, limit in [(signal.SIGINT, 15.), (signal.SIGTERM, 5.), (signal.SIGKILL, 3.)]:
        if not live_group_members(group):
            break
        result['signals'].append(sig.name)
        log_event('shutdown_signal', signal=sig.name, process_group=group)
        try:
            os.killpg(group, sig)
        except ProcessLookupError:
            break
        until = time.monotonic()+limit
        while time.monotonic() < until and live_group_members(group):
            process.poll()
            if pump is not None:
                try:
                    pump()
                except Exception as exc:
                    log_event('shutdown_recording_error', error=str(exc))
                    pump = None
            else:
                time.sleep(.1)
    try:
        process.wait(timeout=1.)
    except subprocess.TimeoutExpired:
        pass
    result['returncode'] = process.poll()
    result['remaining_pids'] = live_group_members(group)
    return result


def create_plots(output, label='ROS integration 20261018'):
    """离线读取原始 CSV 逐点作图；无平滑、无插值，长缺口断线。"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    def read(name):
        """读取完整 CSV，保留启动、运行、停止和故障期间所有样本。"""
        with (output/name).open(newline='') as stream:
            return list(csv.DictReader(stream))

    def line(rows, xkey, ykey, gap=1.):
        """只插入 NaN 断开采样缺口，不补造数据点。"""
        xs, ys, previous = [], [], None
        for row in rows:
            wall = float(row['wall_s'])
            if previous is not None and wall-previous > gap:
                xs.append(float('nan')); ys.append(float('nan'))
            xs.append(float(row[xkey])); ys.append(float(row[ykey]))
            previous = wall
        return xs, ys

    odom, commands = read('odom.csv'), read('safe_cmd_vel.csv')
    title = (label+' / firmware source v131 / SIMULATION_ONLY\n'
             'STM32 P: right reverse .45 -> .45, others .30 -> .30; '
             'I: steady .010 -> .010, start .030 -> .030 / 10 ms\n'
             'PI not exercised; firmware algorithm unchanged; Gazebo DiffDrive + Nav2 single goal')
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    for axis, actual, desired, label in [(axes[0], 'vx_mps', 'vx_mps', 'Linear x [m/s]'),
                                       (axes[1], 'wz_rps', 'wz_rps', 'Angular z [rad/s]')]:
        if odom:
            axis.plot(*line(odom, 'wall_s', actual), '.-', markersize=2, linewidth=.6, label='/sim/odom')
        if commands:
            axis.plot(*line(commands, 'wall_s', desired), '.-', markersize=1.5, linewidth=.5,
                      label='/nav/safe_cmd_vel (command)')
        if not odom and not commands:
            axis.text(.5, .5, 'NO DATA RECEIVED', transform=axis.transAxes, ha='center')
        axis.set_ylabel(label); axis.grid(True)
        if odom or commands:
            axis.legend()
    axes[-1].set_xlabel('Monotonic wall time since verifier start [s]')
    fig.suptitle(title); fig.tight_layout()
    for suffix in ['png', 'svg']:
        fig.savefig(output/('velocity.'+suffix), dpi=160)
    plt.close(fig)
    fig, axis = plt.subplots(figsize=(9, 7))
    if odom:
        axis.plot(*line(odom, 'x_m', 'y_m'), '.-', markersize=2, linewidth=.6,
                  label='Raw /sim/odom, odom frame')
        axis.scatter([float(odom[0]['x_m'])], [float(odom[0]['y_m'])], marker='o', label='First sample')
        axis.scatter([float(odom[-1]['x_m'])], [float(odom[-1]['y_m'])], marker='x', label='Last sample')
        axis.legend()
    else:
        axis.text(.5, .5, 'NO ODOMETRY RECEIVED', transform=axis.transAxes, ha='center')
    axis.set_xlabel('Odometry x [m]'); axis.set_ylabel('Odometry y [m]')
    axis.set_aspect('equal', adjustable='datalim'); axis.grid(True)
    fig.suptitle(title); fig.tight_layout()
    for suffix in ['png', 'svg']:
        fig.savefig(output/('trajectory.'+suffix), dpi=160)
    plt.close(fig)


def verify(args, workspace):
    """记录真实消息，等待就绪，执行短目标并验证停止，失败也完整收尾。"""
    args.output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    partition = 'smartcar-verify-'+uuid.uuid4().hex
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY='1',
                      IGN_PARTITION=partition, GZ_PARTITION=partition,
                      ROS_LOG_DIR=str(args.output/'ros_logs'), RCUTILS_COLORIZED_OUTPUT='0')
    os.environ.pop('ROS_DISCOVERY_SERVER', None)
    report = {'result': 'FAIL', 'scope': 'SIMULATION_ONLY',
              'started_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'domain_id': args.domain_id, 'ros_localhost_only': 1, 'ign_partition': partition,
              'goal': {'frame': 'map', 'x_m': 1.5, 'y_m': 0., 'yaw_rad': 0.},
              'checks': {}, 'lifecycle_states': {}, 'error': None,
              'firmware': {'pi_exercised': False, 'pi_changed': False, 'algorithm_changed': False,
                           'version': 131,
                           'p_i_before_after': 'P right reverse .45 -> .45, others .30 -> .30; I steady .010 -> .010, start .030 -> .030 per 10 ms motor-rpm error; not exercised in Gazebo'},
              'not_verified': ['STM32 hardware/PI', 'physical vehicle', 'obstacle avoidance',
                               'recovery behaviors', 'multi-goal navigation', 'arbitrary localization'],
              'measurement': 'Gazebo-generated ROS odometry, not external physical ground truth'}
    report['verifier_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    if args.map_file:
        import yaml
        image=args.map_file.parent/yaml.safe_load(args.map_file.read_text())['image']
        report['map_override']={'yaml':str(args.map_file),'yaml_sha256':hashlib.sha256(args.map_file.read_bytes()).hexdigest(),
                                'image_sha256':hashlib.sha256(image.read_bytes()).hexdigest()}
        (args.output/'map_used.yaml').write_bytes(args.map_file.read_bytes())
        (args.output/'map_used.pgm').write_bytes(image.read_bytes())
    streams, writers, counts = {}, {}, {}
    events = (args.output/'events.jsonl').open('w', buffering=1)

    def event(name, **values):
        """以追加方式保存操作与异常时间线。"""
        row = {'wall_s': time.monotonic()-started, 'event': name, **values}
        events.write(json.dumps(row, ensure_ascii=False)+'\n')

    def open_csv(name, fields):
        """立即写入表头；缺测时也保留可审计的空 CSV。"""
        streams[name] = (args.output/name).open('w', newline='', buffering=1)
        writers[name] = csv.writer(streams[name]); writers[name].writerow(fields)
        counts[name] = 0

    def row(name, values):
        """逐条保存收到的数据，不做均值化或过滤。"""
        writers[name].writerow([time.monotonic()-started, *values]); counts[name] += 1

    open_csv('odom.csv', ['wall_s', 'stamp_s', 'frame_id', 'child_frame_id', 'x_m', 'y_m', 'z_m',
                         'qx', 'qy', 'qz', 'qw', 'vx_mps', 'vy_mps', 'vz_mps', 'wx_rps', 'wy_rps', 'wz_rps'])
    open_csv('safe_cmd_vel.csv', ['wall_s', 'latest_clock_s', 'vx_mps', 'vy_mps', 'vz_mps', 'wx_rps', 'wy_rps', 'wz_rps'])
    open_csv('safety_status.csv', ['wall_s', 'latest_clock_s', 'raw_json'])
    open_csv('clock.csv', ['wall_s', 'clock_s', 'sec', 'nanosec'])
    open_csv('feedback.csv', ['wall_s', 'distance_remaining_m', 'number_of_recoveries'])
    launch_log = (args.output/'launch.log').open('w', buffering=1)
    process = node = handle = action_client = None
    rclpy = None
    received, stamps, latest = {}, {}, {'clock': None}
    lifecycle_clients, futures = {}, {}
    stop_window = []
    try:
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
        from rclpy.time import Time
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import OccupancyGrid, Odometry
        from nav2_msgs.action import NavigateToPose
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        from rosgraph_msgs.msg import Clock
        from lifecycle_msgs.srv import GetState
        from tf2_ros import Buffer, TransformListener
        from ament_index_python.packages import get_package_prefix

        for package in ['smartcar_sim', 'smartcar_nav2']:
            prefix = Path(get_package_prefix(package)).resolve()
            if not prefix.is_relative_to((workspace/'install').resolve()):
                raise RuntimeError(f'{package} 不是本仓库构建产物：{prefix}')
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = Node('smartcar_isolated_navigation_verifier',
                    parameter_overrides=[Parameter('use_sim_time', value=True)])
        buffer = Buffer(); listener = TransformListener(buffer, node)

        def seen(name, message):
            """更新数据新鲜度，保留真实 ROS 消息供就绪检查。"""
            received[name] = message; stamps[name] = time.monotonic()

        def odom(message):
            """记录里程计原始姿态和六轴速度。"""
            seen('odom', message)
            p, q, v, w = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist.linear, message.twist.twist.angular
            row('odom.csv', [message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
                            message.header.frame_id, message.child_frame_id, p.x, p.y, p.z,
                            q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z])
            stop_window.append((time.monotonic(), math.hypot(v.x, v.y), abs(w.z), p.x, p.y))
            stop_window[:] = [item for item in stop_window if item[0] >= time.monotonic()-3.]

        def command(message):
            """记录门控后的完整速度指令。"""
            seen('command', message)
            v, w = message.linear, message.angular
            row('safe_cmd_vel.csv', [latest['clock'], v.x, v.y, v.z, w.x, w.y, w.z])

        def safety(message):
            """保留门控原始 JSON，包含故障状态。"""
            seen('gate', message)
            row('safety_status.csv', [latest['clock'], message.data])

        def clock(message):
            """记录原始仿真时间并检测时间是否推进。"""
            value = message.clock.sec+message.clock.nanosec*1e-9
            if latest['clock'] is None or value > latest['clock']:
                stamps['clock_progress'] = time.monotonic()
            latest['clock'] = value; seen('clock', message)
            row('clock.csv', [value, message.clock.sec, message.clock.nanosec])

        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscriptions = [node.create_subscription(Odometry, '/sim/odom', odom, 100),
                         node.create_subscription(Twist, '/nav/safe_cmd_vel', command, 100),
                         node.create_subscription(String, '/navigation/safety_status', safety, 100),
                         node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data),
                         node.create_subscription(OccupancyGrid, '/map', lambda msg: seen('map', msg), map_qos),
                         node.create_subscription(LaserScan, '/sim/scan', lambda msg: seen('scan', msg), qos_profile_sensor_data)]
        # 在启动前确认测试域没有其他仿真，防止干扰同域未知会话。
        until = time.monotonic()+1.
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        conflicts = [topic for topic in ['/clock', '/map', '/sim/scan', '/sim/odom', '/nav/safe_cmd_vel']
                     if node.count_publishers(topic)]
        if conflicts:
            raise RuntimeError('隔离域已有发布者，拒绝启动：'+', '.join(conflicts))
        action_client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        for name in ['map_server', 'amcl', 'controller_server', 'smoother_server', 'planner_server',
                     'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother']:
            lifecycle_clients[name] = node.create_client(GetState, '/'+name+'/get_state')
        launch_command = ['ros2', 'launch', 'smartcar_nav2', 'navigation.launch.py',
                          'headless:=true', 'rviz:=false']
        if args.map_file:launch_command.append('map:='+str(args.map_file))
        report['launch_command'] = launch_command
        process = subprocess.Popen(launch_command, cwd=workspace, env=os.environ.copy(),
                                   stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
        event('launch_started', pid=process.pid, command=launch_command)

        def spin():
            """处理真实消息，同时检查自建 launch 是否提前退出。"""
            rclpy.spin_once(node, timeout_sec=.05)
            if process.poll() is not None:
                raise RuntimeError(f'launch 提前退出，返回码 {process.returncode}，见 launch.log')

        until = time.monotonic()+args.startup_timeout
        next_poll = 0.
        while time.monotonic() < until:
            spin()
            for name, future in list(futures.items()):
                if future.done():
                    reply = future.result()
                    report['lifecycle_states'][name] = reply.current_state.label if reply else 'no_response'
                    del futures[name]
            now = time.monotonic()
            if now < next_poll:
                continue
            next_poll = now+.25
            for name, service in lifecycle_clients.items():
                if name not in futures and report['lifecycle_states'].get(name) != 'active' and service.service_is_ready():
                    futures[name] = service.call_async(GetState.Request())
            checks = {name+'_active': report['lifecycle_states'].get(name) == 'active' for name in lifecycle_clients}
            checks.update({name+'_fresh': now-stamps.get(name, 0.) < 1. for name in ['scan', 'odom', 'gate', 'clock_progress']})
            checks['map_received'] = 'map' in received and bool(received['map'].data)
            checks['clock_advancing'] = counts['clock.csv'] > 1 and (latest['clock'] or 0.) > 0.
            checks['action_server_ready'] = action_client.server_is_ready()
            checks['single_publishers'] = all(node.count_publishers(t) == 1 for t in ['/clock', '/map', '/sim/scan', '/sim/odom', '/nav/safe_cmd_vel'])
            for parent, child in [('map', 'odom'), ('odom', 'base_footprint'), ('base_footprint', 'laser_frame')]:
                checks['tf_'+parent+'_'+child] = buffer.can_transform(parent, child, Time())
            gate = json.loads(received['gate'].data) if 'gate' in received else {}
            checks['gate_ready'] = gate.get('state') in ['READY', 'COMMAND_STALE'] and gate.get('stop_latched') is False
            report['readiness'] = checks
            if all(checks.values()):
                report['checks']['startup_ready'] = True
                event('startup_ready', checks=checks, gate=gate)
                break
        else:
            raise TimeoutError('60 秒启动窗口内未全部就绪；见 result.json/readiness 和 launch.log。')

        def wait_future(future, seconds):
            """有界等待 ROS 请求，不暂停记录其他订阅。"""
            deadline = time.monotonic()+seconds
            while not future.done() and time.monotonic() < deadline:
                spin()
            if not future.done():
                raise TimeoutError('等待导航响应超时')
            return future.result()

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'; goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = 1.5; goal.pose.pose.orientation.w = 1.
        initial_position = received['odom'].pose.pose.position
        report['initial_odom_position_m'] = {'x': initial_position.x, 'y': initial_position.y, 'z': initial_position.z}

        def feedback(message):
            """保存每条导航距离与恢复次数反馈，不将恢复次数当成恢复验收。"""
            row('feedback.csv', [message.feedback.distance_remaining, message.feedback.number_of_recoveries])

        event('goal_sent', goal=report['goal'])
        handle = wait_future(action_client.send_goal_async(goal, feedback_callback=feedback), 10.)
        report['checks']['goal_accepted'] = handle.accepted
        if not handle.accepted:
            raise RuntimeError('Nav2 拒绝目标')
        response = wait_future(handle.get_result_async(), args.goal_timeout)
        report['action_status'] = response.status
        report['checks']['goal_succeeded'] = response.status == 4
        event('goal_result', action_status=response.status)
        if response.status != 4:
            raise RuntimeError('导航未返回 SUCCEEDED，状态码：'+str(response.status))
        stop_start = time.monotonic()
        while time.monotonic()-stop_start < 8.:
            spin()
            sample = [item for item in stop_window if item[0] >= stop_start and item[0] >= time.monotonic()-1.5]
            if len(sample) >= 10 and sample[-1][0]-sample[0][0] >= 1. and time.monotonic()-sample[-1][0] < .5:
                displacement = math.hypot(sample[-1][3]-sample[0][3], sample[-1][4]-sample[0][4])
                cmd = received.get('command')
                stopped = (all(speed < .02 and turn < .03 for _, speed, turn, _, _ in sample)
                           and displacement < .02 and cmd is not None
                           and time.monotonic()-stamps.get('command', 0.) < .5
                           and abs(cmd.linear.x) < .001 and abs(cmd.angular.z) < .001)
                if stopped:
                    report['checks']['odom_and_command_stopped'] = True
                    report['stop_evidence'] = {'samples': len(sample), 'duration_wall_s': sample[-1][0]-sample[0][0],
                                               'max_linear_mps': max(item[1] for item in sample),
                                               'max_angular_rps': max(item[2] for item in sample),
                                               'displacement_m': displacement}
                    event('stop_verified', **report['stop_evidence'])
                    break
        else:
            raise RuntimeError('目标成功后未确认持续静止；保留全部停止阶段样本。')
        p = received['odom'].pose.pose.position
        report['final_odom_position_m'] = {'x': p.x, 'y': p.y, 'z': p.z}
        report['odom_displacement_m'] = math.hypot(p.x-initial_position.x, p.y-initial_position.y)
        report['checks']['actual_odom_moved'] = report['odom_displacement_m'] > .8
        if not report['checks']['actual_odom_moved']:
            raise RuntimeError('虽收到目标成功结果，但实际里程计位移不足 0.8 m。')
        report['result'] = 'PASS'
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = str(exc) or type(exc).__name__
        (args.output/'exception.log').write_text(traceback.format_exc())
        event('failure', error=report['error'])
    finally:
        if handle is not None and handle.accepted and node is not None and rclpy is not None:
            try:
                cancel = handle.cancel_goal_async()
                deadline = time.monotonic()+2.
                while not cancel.done() and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.05)
                event('cancel_on_exit', response_received=cancel.done())
            except Exception as exc:
                event('cancel_on_exit_error', error=str(exc))
        if process is not None:
            pump = (lambda: rclpy.spin_once(node, timeout_sec=.1)) if node is not None else None
            report['shutdown'] = stop_group(process, event, pump)
            if report['shutdown']['remaining_pids']:
                report['result'] = 'FAIL'
                report['error'] = (report['error'] or '')+'; 自建进程组未完全退出'
        launch_log.close()
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.try_shutdown()
        for stream in streams.values():
            stream.close()
        report['sample_counts'] = counts
        report['duration_wall_s'] = time.monotonic()-started
        try:
            create_plots(args.output,args.evidence_label)
            report['plots'] = ['velocity.png', 'velocity.svg', 'trajectory.png', 'trajectory.svg']
        except Exception as exc:
            report['plot_error'] = str(exc)
            report['result'] = 'FAIL'
            (args.output/'plot_exception.log').write_text(traceback.format_exc())
        report['raw_sha256'] = {name: hashlib.sha256((args.output/name).read_bytes()).hexdigest() for name in streams}
        (args.output/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        event('finished', result=report['result'])
        events.close()
    print(json.dumps({'result': report['result'], 'scope': report['scope'], 'output': str(args.output),
                      'error': report['error']}, ensure_ascii=False, indent=2))
    return 0 if report['result'] == 'PASS' else 1


def main():
    """只有用户显式执行脚本时才创建仿真与发送目标。"""
    args = arguments()
    workspace = Path(__file__).resolve().parents[1]
    bootstrap(args, workspace)
    return verify(args, workspace)


if __name__ == '__main__':
    raise SystemExit(main())
