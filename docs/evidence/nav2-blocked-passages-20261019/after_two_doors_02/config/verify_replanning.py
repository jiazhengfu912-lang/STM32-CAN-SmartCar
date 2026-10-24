#!/usr/bin/env python3
"""隔离 Gazebo 中途插入障碍；保存真实路径、扫描、代价图、速度与停车证据。"""

import argparse
import csv
import copy
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback
import uuid
import xml.etree.ElementTree as ET

from verify_navigation import stop_group


def arguments():
    """指定全新输出、障碍工况与隔离域，不接入实车或默认用户域。"""
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--case', choices=['detour', 'blocked', 'two_doors'], default='detour')
    p.add_argument('--reopen', action='store_true', help='双门封堵失败后移走右门箱体，并显式发送一个新目标')
    p.add_argument('--through-poses', action='store_true', help='使用连续多目标动作入口和对应行为树')
    p.add_argument('--bt-xml', type=Path, help='显式指定单目标行为树，便于核对修复前版本')
    p.add_argument('--domain-id', type=int, default=89)
    p.add_argument('--goal-timeout', type=float, default=180.)
    p.add_argument('--obstacle-x', type=float, default=2.)
    p.add_argument('--obstacle-width', type=float, default=.5)
    p.add_argument('--insert-x', type=float, default=.45)
    p.add_argument('--_ready', action='store_true', help=argparse.SUPPRESS)
    a = p.parse_args()
    if a.domain_id == 86 or not 0 <= a.domain_id <= 232:
        p.error('须使用 0..232 内且不是 86 的隔离 ROS 域。')
    if not all(math.isfinite(v) and v > 0 for v in
               [a.goal_timeout, a.obstacle_x, a.obstacle_width, a.insert_x]):
        p.error('时间、尺寸和插入位置必须为有限正数。')
    if a.obstacle_x-.2-a.insert_x < .6:
        p.error('障碍插入时距车辆参考点须至少 0.6 m。')
    a.output = a.output.expanduser().resolve()
    if a.bt_xml:
        a.bt_xml = a.bt_xml.expanduser().resolve()
        if not a.bt_xml.is_file():
            p.error('行为树文件不存在。')
    if a.reopen and a.case != 'two_doors':
        p.error('--reopen 只适用于 two_doors。')
    return a


def bootstrap(a, ws):
    """加载仓库自身环境，隔离其他 overlay，不在默认域启动任何节点。"""
    if a._ready:
        return
    if a.output.exists():
        raise FileExistsError('拒绝覆盖已有证据：'+str(a.output))
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(a.domain_id), ROS_LOCALHOST_ONLY='1')
    command = ('set -e; unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH '
               'ROS_PACKAGE_PATH PYTHONPATH; source "$1"; exec /usr/bin/python3 "$2" "${@:3}" --_ready')
    os.execvpe('bash', ['bash', '-c', command, 'smartcar-replanning-verifier',
                      str(ws/'tools/env.bash'), str(Path(__file__).resolve()), *sys.argv[1:]], env)


def verify(a, ws):
    """保持一次目标不重发，实测障碍出现前后规划和车辆运动。"""
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from rosidl_runtime_py.convert import message_to_ordereddict
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
    from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
    from nav2_msgs.msg import BehaviorTreeLog
    from sensor_msgs.msg import LaserScan
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, TransformListener
    from lifecycle_msgs.srv import GetState

    a.output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    partition = 'smartcar-replan-'+uuid.uuid4().hex
    os.environ.update(ROS_DOMAIN_ID=str(a.domain_id), ROS_LOCALHOST_ONLY='1',
                      IGN_PARTITION=partition, GZ_PARTITION=partition,
                      ROS_LOG_DIR=str(a.output/'ros_logs'), RCUTILS_COLORIZED_OUTPUT='0')
    os.environ.pop('ROS_DISCOVERY_SERVER', None)
    two_doors = a.case == 'two_doors'
    report = {'result': 'FAIL', 'scope': 'SIMULATION_ONLY', 'case': a.case,
              'started_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'domain_id': a.domain_id, 'ign_partition': partition,
              'goal': [1.03, -2.92, 0.] if two_doors else [4.5, 0., 0.], 'goal_send_count': 0, 'checks': {},
              'firmware_source_version': 131, 'pi_exercised': False, 'pi_changed': False,
              'not_verified': ['physical vehicle', 'moving obstacles', 'arbitrary routes', 'multi-goal GUI']}
    # 完全封堵工况把目标区域占满；不把全局另有通路的走廊封条称为无路可走。
    box = {'x': a.obstacle_x if a.case == 'detour' else 4.5, 'y': 0.,
           'dx': .4 if a.case == 'detour' else 1.2,
           'dy': a.obstacle_width if a.case == 'detour' else 1.85, 'dz': .6}
    report['obstacle'] = box
    blockers = [
        {'name': 'blocked_left', 'shape': 'sphere', 'x': -2.9082, 'y': .2833, 'z': .5, 'radius': .5},
        {'name': 'blocked_right', 'shape': 'box', 'x': 3.0925, 'y': .8227, 'z': .5, 'size': [1., 1., 1.]},
    ] if two_doors else []
    report['blockers'] = blockers
    report['reopen_requested'] = a.reopen
    report['action_type'] = 'NavigateThroughPoses' if a.through_poses else 'NavigateToPose'
    source_paths = [Path(__file__), ws/'tools/verify_navigation.py',
                    ws/'src/smartcar_nav2/config/nav2.yaml',
                    ws/'src/smartcar_nav2/config/navigate_recovery.xml',
                    ws/'src/smartcar_nav2/config/navigate_through_recovery.xml',
                    ws/'src/smartcar_nav2/launch/navigation.launch.py']
    report['source_sha256'] = {str(p.relative_to(ws.parent)): hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in source_paths}
    (a.output/'config').mkdir()
    for p in source_paths:
        (a.output/'config'/p.name).write_bytes(p.read_bytes())
    if a.bt_xml:
        report['bt_override_sha256'] = hashlib.sha256(a.bt_xml.read_bytes()).hexdigest()
        (a.output/'config/executed_bt.xml').write_bytes(a.bt_xml.read_bytes())
    streams, writers = {}, {}
    latest, received_at, poses, paths = {}, {}, [], []
    inserted_at = None
    process = truth_bridge = node = handle = None
    result_future = None
    launch_log = (a.output/'launch.log').open('w', buffering=1)

    def stream(name):
        """逐条写入原始记录，异常和关闭过程也保留。"""
        if name not in streams:
            streams[name] = (a.output/name).open('w', buffering=1)
        return streams[name]

    def event(name, **values):
        """记录墙钟时间与真实操作结果。"""
        item = {'wall_s': time.monotonic()-started, 'event': name, **values}
        stream('events.jsonl').write(json.dumps(item, ensure_ascii=False)+'\n')
        print(json.dumps(item, ensure_ascii=False), flush=True)

    def raw(name, msg):
        """以 ROS 消息字段保存原样数值；非有限雷达量保留 Python JSON 标记。"""
        record = {'wall_s': time.monotonic()-started, 'msg': message_to_ordereddict(msg)}
        stream(name).write(json.dumps(record, separators=(',', ':'))+'\n')

    def csv_row(name, fields, values):
        """写入全部采样点，保持独立完整 CSV。"""
        if name not in writers:
            writers[name] = csv.writer(stream(name))
            writers[name].writerow(['wall_s', *fields])
        writers[name].writerow([time.monotonic()-started, *values])

    def seen(key, msg):
        """跟踪消息新鲜度。"""
        latest[key] = msg
        received_at[key] = time.monotonic()

    def yaw(q):
        """四元数转换为偏航角，不改变采样。"""
        return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def odom(msg):
        """记录真实仿真里程计姿态和速度。"""
        seen('odom', msg)
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        csv_row('odom.csv', ['stamp_s', 'x', 'y', 'yaw', 'vx', 'wz'],
                [msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9, p.x, p.y, yaw(q), v.linear.x, v.angular.z])

    def truth(msg):
        """保存 Gazebo 世界位姿，独立于 AMCL；不向控制器提供世界真值。"""
        for t in msg.transforms:
            if t.child_frame_id == 'smartcar':
                # 仅保存车辆原始变换，不重复记录所有静态场景模型的位姿。
                raw('world_pose.jsonl', t)
                p, q = t.transform.translation, t.transform.rotation
                value = [time.monotonic()-started, p.x, p.y, yaw(q)]
                poses.append(value)
                seen('truth', value)
                csv_row('world_pose.csv', ['x', 'y', 'yaw'], value[1:])

    def plan(msg):
        """记录每次全局路径，用于证明原路线确实被替换。"""
        seen('plan', msg)
        raw('plans.jsonl', msg)
        paths.append({'wall_s': time.monotonic()-started, 'frame': msg.header.frame_id,
                      'points': [[p.pose.position.x, p.pose.position.y] for p in msg.poses]})

    def callback(key, filename):
        """将指定 ROS 话题完整记录并保留最后样本。"""
        def receive(msg):
            """接收一个真实消息。"""
            seen(key, msg)
            raw(filename, msg)
            if key == '/global_costmap/costmap' and blockers and '/map' in latest:
                # 只计静态底图中原本空闲、但实际全局图现为致命障碍的格子。
                static = latest['/map']
                values = []
                for obstacle in blockers:
                    count = 0
                    for iy in range(max(0, int((obstacle['y']-.7-msg.info.origin.position.y)/msg.info.resolution)),
                                    min(msg.info.height, int((obstacle['y']+.7-msg.info.origin.position.y)/msg.info.resolution)+1)):
                        for ix in range(max(0, int((obstacle['x']-.7-msg.info.origin.position.x)/msg.info.resolution)),
                                        min(msg.info.width, int((obstacle['x']+.7-msg.info.origin.position.x)/msg.info.resolution)+1)):
                            x = msg.info.origin.position.x+(ix+.5)*msg.info.resolution
                            y = msg.info.origin.position.y+(iy+.5)*msg.info.resolution
                            sx = int((x-static.info.origin.position.x)/static.info.resolution)
                            sy = int((y-static.info.origin.position.y)/static.info.resolution)
                            if (0 <= sx < static.info.width and 0 <= sy < static.info.height
                                    and static.data[sy*static.info.width+sx] == 0
                                    and msg.data[iy*msg.info.width+ix] == 100):
                                count += 1
                    values.append(count)
                csv_row('obstacle_memory.csv', [b['name'] for b in blockers], values)
        return receive

    def command(topic):
        """记录控制器、平滑器、保护及仿真驱动四级速度。"""
        def receive(msg):
            """保存完整 Twist。"""
            seen(topic, msg)
            csv_row('velocity.csv', ['topic', 'vx', 'vy', 'vz', 'wx', 'wy', 'wz'],
                    [topic, msg.linear.x, msg.linear.y, msg.linear.z,
                     msg.angular.x, msg.angular.y, msg.angular.z])
        return receive

    def spin():
        """持续采样并检查 launch 未提前退出。"""
        rclpy.spin_once(node, timeout_sec=.025)
        if process is not None and process.poll() is not None:
            raise RuntimeError('launch 提前退出：'+str(process.returncode))

    def wait(future, timeout):
        """有界等待请求，始终处理采样订阅。"""
        deadline = time.monotonic()+timeout
        while not future.done() and time.monotonic() < deadline:
            spin()
        if not future.done():
            raise TimeoutError('等待 ROS 响应超时')
        return future.result()

    def gazebo_create():
        """仅向本次随机 Gazebo 分区插入有碰撞和雷达可见几何的静态箱体。"""
        sdf = ('<sdf version="1.8"><model name="replanning_block"><static>true</static>'
               f'<pose>{box["x"]} 0 .3 0 0 0</pose><link name="link">'
               f'<collision name="c"><geometry><box><size>{box["dx"]} {box["dy"]} .6</size>'
               '</box></geometry></collision><visual name="v"><geometry><box>'
               f'<size>{box["dx"]} {box["dy"]} .6</size></box></geometry></visual></link></model></sdf>')
        request = 'sdf: '+json.dumps(sdf)+' allow_renaming: false'
        command_line = ['ign', 'service', '-s', '/world/smartcar_world/create',
                        '--reqtype', 'ignition.msgs.EntityFactory', '--reptype', 'ignition.msgs.Boolean',
                        '--timeout', '5000', '--req', request]
        log = stream('gazebo_service.log')
        log.write(json.dumps(command_line)+'\n')
        child = subprocess.Popen(command_line, stdout=log, stderr=subprocess.STDOUT)
        deadline = time.monotonic()+8.
        while child.poll() is None and time.monotonic() < deadline:
            spin()
        if child.poll() is None:
            child.kill(); child.wait()
            raise TimeoutError('插入障碍请求超时')
        log.flush()
        if child.returncode or 'data: true' not in (a.output/'gazebo_service.log').read_text():
            raise RuntimeError('插入障碍请求失败，见 gazebo_service.log')

    def gazebo_remove(name):
        """仅移除隔离验收中明确命名的障碍，不接触用户的场景。"""
        command_line = ['ign', 'service', '-s', '/world/smartcar_world/remove',
                        '--reqtype', 'ignition.msgs.Entity', '--reptype', 'ignition.msgs.Boolean',
                        '--timeout', '5000', '--req', f'name: "{name}" type: MODEL']
        log = stream('gazebo_remove.log')
        child = subprocess.Popen(command_line, stdout=log, stderr=subprocess.STDOUT)
        deadline = time.monotonic()+8.
        while child.poll() is None and time.monotonic() < deadline:
            spin()
        if child.poll() is None:
            child.kill(); child.wait()
            raise TimeoutError('移除隔离障碍超时')
        log.flush()
        if child.returncode or 'data: true' not in (a.output/'gazebo_remove.log').read_text():
            raise RuntimeError('移除隔离障碍失败')

    try:
        for name in ['smartcar_sim', 'smartcar_nav2']:
            if not Path(get_package_prefix(name)).resolve().is_relative_to((ws/'install').resolve()):
                raise RuntimeError('包不来自本仓库：'+name)
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = Node('smartcar_replanning_verifier', parameter_overrides=[Parameter('use_sim_time', value=True)])
        buffer = Buffer(); listener = TransformListener(buffer, node)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscriptions = [
            node.create_subscription(Odometry, '/sim/odom', odom, 100),
            node.create_subscription(TFMessage, '/sim/world_pose', truth, 100),
            node.create_subscription(NavPath, '/plan', plan, 100),
            node.create_subscription(LaserScan, '/sim/scan', callback('scan', 'scan.jsonl'), qos_profile_sensor_data),
            node.create_subscription(Clock, '/clock', callback('clock', 'clock.jsonl'), qos_profile_sensor_data),
            node.create_subscription(String, '/navigation/safety_status', callback('gate', 'safety.jsonl'), 100),
            node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', callback('amcl', 'amcl.jsonl'), qos),
            node.create_subscription(BehaviorTreeLog, '/behavior_tree_log', callback('bt', 'behavior_tree.jsonl'), 100)]
        for topic in ['/map', '/local_costmap/costmap', '/global_costmap/costmap']:
            subscriptions.append(node.create_subscription(OccupancyGrid, topic,
                                 callback(topic, topic.strip('/').replace('/', '_')+'.jsonl'), qos))
        for topic in ['/nav/cmd_vel_raw', '/nav/cmd_vel', '/nav/safe_cmd_vel', '/sim/drive_cmd_vel']:
            subscriptions.append(node.create_subscription(Twist, topic, command(topic), 100))
        until = time.monotonic()+1.
        while time.monotonic() < until:
            spin()
        conflicts = [t for t in ['/clock', '/map', '/sim/scan', '/sim/odom', '/nav/safe_cmd_vel'] if node.count_publishers(t)]
        if conflicts:
            raise RuntimeError('隔离域已有发布者：'+str(conflicts))
        action_type = NavigateThroughPoses if a.through_poses else NavigateToPose
        nav = ActionClient(node, action_type, '/navigate_through_poses' if a.through_poses else '/navigate_to_pose')
        names = ['map_server', 'amcl', 'controller_server', 'smoother_server', 'planner_server',
                 'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother']
        clients = {n: node.create_client(GetState, '/'+n+'/get_state') for n in names}
        states, futures = {}, {}
        launch_args = ['ros2', 'launch', 'smartcar_nav2', 'navigation.launch.py', 'headless:=true', 'rviz:=false']
        if a.bt_xml:
            launch_args.append(('through_poses_bt_xml' if a.through_poses else 'bt_xml')+':='+str(a.bt_xml))
        if two_doors:
            # 按用户世界位姿构造同源场景，障碍从启动前就存在，目标仍由真实 Nav2 执行。
            tree = ET.parse(ws/'src/smartcar_sim/worlds/office_loop.sdf')
            world = tree.find('world')
            include = next(i for i in world.findall('include') if i.findtext('name') == 'smartcar')
            include.find('pose').text = '-2.99 1.91 .003 0 0 0'
            for b in blockers:
                model = ET.SubElement(world, 'model', name=b['name'])
                ET.SubElement(model, 'static').text = 'true'
                ET.SubElement(model, 'pose').text = f'{b["x"]} {b["y"]} {b["z"]} 0 0 0'
                link = ET.SubElement(model, 'link', name='link')
                for kind in ['collision', 'visual']:
                    geometry = ET.SubElement(ET.SubElement(link, kind, name=kind), 'geometry')
                    shape = ET.SubElement(geometry, b['shape'])
                    ET.SubElement(shape, 'radius' if b['shape'] == 'sphere' else 'size').text = (
                        str(b['radius']) if b['shape'] == 'sphere' else ' '.join(map(str, b['size'])))
            world_path = a.output/'test_world.sdf'
            tree.write(world_path, encoding='utf-8', xml_declaration=True)
            launch_args += ['world:='+str(world_path), 'initial_x:=-2.99', 'initial_y:=1.91']
            report['checks']['blockers_present_before_goal'] = True
        report['launch_command'] = launch_args
        process = subprocess.Popen(launch_args, cwd=ws, env=os.environ.copy(),
                                   stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
        event('launch_started', pid=process.pid, partition=partition)
        # 世界真值桥只用于验收记录，不进入控制、定位或规划链路。
        truth_bridge = subprocess.Popen(
            ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             '/world/smartcar_world/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
             '--ros-args', '-r', '/world/smartcar_world/pose/info:=/sim/world_pose',
             '-r', '__node:=replanning_truth_bridge'],
            env=os.environ.copy(), stdout=stream('truth_bridge.log'), stderr=subprocess.STDOUT,
            start_new_session=True)
        until, poll = time.monotonic()+60., 0.
        while time.monotonic() < until:
            spin()
            if time.monotonic() < poll:
                continue
            poll = time.monotonic()+.25
            for n, f in list(futures.items()):
                if f.done():
                    states[n] = f.result().current_state.label
                    del futures[n]
            for n, c in clients.items():
                if states.get(n) != 'active' and n not in futures and c.service_is_ready():
                    futures[n] = c.call_async(GetState.Request())
            fresh = all(time.monotonic()-received_at.get(k, 0.) < 1. for k in ['odom', 'scan', 'clock', 'gate', 'truth'])
            unique = all(node.count_publishers(t) == 1 for t in ['/clock', '/map', '/sim/scan', '/sim/odom', '/nav/safe_cmd_vel'])
            tf_ok = all(buffer.can_transform(p, c, Time()) for p, c in
                        [('map', 'odom'), ('odom', 'base_footprint'), ('base_footprint', 'laser_frame')])
            gate = json.loads(latest['gate'].data) if 'gate' in latest else {}
            if (all(states.get(n) == 'active' for n in names) and fresh and unique and tf_ok
                    and '/map' in latest and nav.server_is_ready()
                    and gate.get('state') in ['READY', 'COMMAND_STALE'] and gate.get('stop_latched') is False):
                break
        else:
            raise TimeoutError('60 秒内未就绪；生命周期：'+str(states))
        report['checks']['startup_ready'] = True
        report['lifecycle_states'] = states
        event('ready', states=states)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'; goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = report['goal'][0]
        goal.pose.pose.position.y = report['goal'][1]
        goal.pose.pose.orientation.w = 1.
        if a.through_poses:
            final_pose = goal.pose
            goal = NavigateThroughPoses.Goal()
            initial_pose = copy.deepcopy(final_pose)
            initial_pose.pose.position.x = -2.99 if two_doors else 0.
            initial_pose.pose.position.y = 1.91 if two_doors else 0.
            goal.poses = [initial_pose, final_pose]
        report['goal_send_count'] += 1
        handle = wait(nav.send_goal_async(goal, feedback_callback=lambda msg: raw('feedback.jsonl', msg.feedback)), 10.)
        if not handle.accepted:
            raise RuntimeError('导航目标被拒绝')
        result_future = handle.get_result_async()
        event('goal_accepted')
        until = time.monotonic()+a.goal_timeout
        while not result_future.done() and time.monotonic() < until:
            spin()
            if not two_doors and inserted_at is None and latest['truth'][1] >= a.insert_x and paths:
                event('obstacle_request', robot=latest['truth'], box=box)
                gazebo_create()
                inserted_at = time.monotonic()-started
                report['inserted_wall_s'] = inserted_at
                event('obstacle_inserted', box=box, robot=latest['truth'])
        if not result_future.done():
            raise TimeoutError('目标执行超时')
        result = result_future.result()
        report['action_status'] = result.status
        event('goal_result', status=result.status)
        # 停车须有连续新鲜采样，不能只凭 action 状态或一次零指令。
        stop_after = time.monotonic()-started
        until = time.monotonic()+3.
        while time.monotonic() < until:
            spin()
        with (a.output/'odom.csv').open() as f:
            rows = [r for r in csv.DictReader(f) if float(r['wall_s']) >= stop_after+1.]
        cmd = latest.get('/nav/safe_cmd_vel')
        report['checks']['stopped'] = (len(rows) >= 10 and float(rows[-1]['wall_s'])-float(rows[0]['wall_s']) >= 1.
            and all(abs(float(r['vx'])) < .02 and abs(float(r['wz'])) < .03 for r in rows)
            and cmd is not None and abs(cmd.linear.x) < .001 and abs(cmd.angular.z) < .001
            and time.monotonic()-received_at.get('/nav/safe_cmd_vel', 0.) < .5)
        report['stop_samples'] = len(rows)
        if not two_doors:
            report['checks']['obstacle_inserted_during_action'] = inserted_at is not None
        report['checks']['single_goal_only'] = report['goal_send_count'] == 1
        report['checks']['expected_action_result'] = result.status == (4 if a.case == 'detour' else 6)
        report['final_world_pose'] = latest['truth'][1:]
        if a.case == 'detour' and inserted_at is not None:
            before = [p for p in paths if p['wall_s'] < inserted_at]
            after = [p for p in paths if p['wall_s'] >= inserted_at]
            # 比较原直行路线与新路径在箱体附近的横向偏移，排除仅缩短剩余路径。
            def offset(path):
                """取箱体横截面附近的路径偏移。"""
                return max([abs(y) for x, y in path['points'] if abs(x-box['x']) < .3] or [0.])
            detours = [p for p in after if offset(p) > box['dy']/2+.2]
            report['checks']['original_path_straight'] = bool(before) and offset(before[0]) < .2
            report['checks']['new_path_detours'] = bool(detours)
            report['first_detour_wall_s'] = detours[0]['wall_s'] if detours else None
            report['replan_latency_wall_s'] = detours[0]['wall_s']-inserted_at if detours else None
            traversed = [p for p in poses if p[0] >= inserted_at and abs(p[1]-box['x']) < .3]
            report['checks']['actual_vehicle_detours'] = bool(traversed) and min(abs(p[2]) for p in traversed) > box['dy']/2+.16
            report['checks']['reached_goal_area'] = math.hypot(latest['truth'][1]-4.5, latest['truth'][2]) < .25
        if two_doors:
            with (a.output/'obstacle_memory.csv').open() as f:
                memory = list(csv.DictReader(f))
            report['checks']['both_blockers_observed'] = all(any(int(r[b['name']]) > 0 for r in memory) for b in blockers)
            both = next((i for i, r in enumerate(memory) if all(int(r[b['name']]) > 0 for b in blockers)), None)
            report['checks']['blockers_retained_until_abort'] = both is not None and all(
                all(int(r[b['name']]) > 0 for b in blockers) for r in memory[both:])
            if a.reopen and result.status == 6:
                event('remove_right_blocker_after_abort', robot=latest['truth'])
                gazebo_remove('blocked_right')
                removed_wall_s = time.monotonic()-started
                report['removed_right_wall_s'] = removed_wall_s
                until = time.monotonic()+4.
                while time.monotonic() < until:
                    spin()
                # 这是显式新目标，不自动续跑已经失败的旧动作。
                report['goal_send_count'] += 1
                if a.through_poses:
                    goal.poses = [goal.poses[-1]]
                handle = wait(nav.send_goal_async(goal, feedback_callback=lambda msg: raw('feedback.jsonl', msg.feedback)), 10.)
                if not handle.accepted:
                    raise RuntimeError('移除箱体后的新目标被拒绝')
                event('new_goal_after_removal')
                reopened = wait(handle.get_result_async(), a.goal_timeout)
                report['reopened_action_status'] = reopened.status
                report['checks']['new_goal_after_removal_succeeded'] = reopened.status == 4
                stop_time = time.monotonic()-started
                until = time.monotonic()+3.
                while time.monotonic() < until:
                    spin()
                with (a.output/'odom.csv').open() as f:
                    stopped_rows = [r for r in csv.DictReader(f) if float(r['wall_s']) > stop_time+1.]
                report['checks']['reopened_stopped'] = len(stopped_rows) >= 10 and all(
                    abs(float(r['vx'])) < .02 and abs(float(r['wz'])) < .03 for r in stopped_rows)
                with (a.output/'obstacle_memory.csv').open() as f:
                    memory = list(csv.DictReader(f))
                report['checks']['removed_blocker_cleared_by_scan'] = any(
                    float(r['wall_s']) > removed_wall_s and int(r['blocked_right']) == 0 for r in memory)
        report['result'] = 'PASS' if all(report['checks'].values()) else 'FAIL'
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = str(exc) or type(exc).__name__
        (a.output/'exception.log').write_text(traceback.format_exc())
        event('failure', error=report['error'])
    finally:
        if handle is not None and handle.accepted and node is not None:
            try:
                wait(handle.cancel_goal_async(), 2.)
                event('cancel_on_exit')
            except Exception as exc:
                event('cancel_error', error=str(exc))
        if process is not None:
            report['shutdown'] = stop_group(process, event, lambda: rclpy.spin_once(node, timeout_sec=.05))
            if report['shutdown']['remaining_pids']:
                report['result'] = 'FAIL'
        if truth_bridge is not None:
            report['truth_bridge_shutdown'] = stop_group(truth_bridge, event)
            if report['truth_bridge_shutdown']['remaining_pids']:
                report['result'] = 'FAIL'
        launch_log.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()
        report['paths_received'] = len(paths)
        for f in streams.values():
            f.close()
        report['duration_wall_s'] = time.monotonic()-started
        report['raw_sha256'] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in a.output.iterdir() if p.is_file()}
        (a.output/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0 if report['result'] == 'PASS' else 1


if __name__ == '__main__':
    args = arguments()
    workspace = Path(__file__).resolve().parents[1]
    bootstrap(args, workspace)
    raise SystemExit(verify(args, workspace))
