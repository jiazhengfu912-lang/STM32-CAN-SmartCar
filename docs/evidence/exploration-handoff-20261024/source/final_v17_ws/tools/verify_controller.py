#!/usr/bin/env python3
"""隔离运行真实 FollowPath 服务，验证 TF 故障与重复生命周期，不连接 Gazebo/实车。"""
import argparse
import csv
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid


def main():
    """仅在独立 DDS 域发布合成传感器，保留每个真实动作及生命周期响应。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--domain-id', type=int, default=91)
    parser.add_argument('--executable', required=True, type=Path)
    parser.add_argument('--cycles', type=int, default=3)
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 232 or args.domain_id == 86:
        parser.error('拒绝用户默认域 86，须使用独立 DDS 域。')
    args.executable = args.executable.resolve()
    if not os.access(args.executable, os.X_OK) or not 1 <= args.cycles <= 20:
        parser.error('控制器须已构建且可执行，cycles 须为 1..20。')
    out = args.output.resolve(); out.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY='1',
                      IGN_PARTITION='controller-test-'+uuid.uuid4().hex,
                      ROS_LOG_DIR=str(out/'ros_logs'), RCUTILS_COLORIZED_OUTPUT='0')
    os.environ['GZ_PARTITION'] = os.environ['IGN_PARTITION']
    os.environ.pop('ROS_DISCOVERY_SERVER', None)
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from nav2_msgs.action import FollowPath
    from sensor_msgs.msg import LaserScan
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    from lifecycle_msgs.srv import ChangeState, GetState
    from rclpy.qos import qos_profile_sensor_data
    import yaml

    ws = Path(__file__).resolve().parents[1]
    params = yaml.safe_load((ws/'src/smartcar_nav2/config/nav2.yaml').read_text())
    # 传感器无障碍、车体固定：只验证服务的错误处理，不模拟车辆到达。
    params['controller_server']['ros__parameters']['bond_heartbeat_period'] = 0.
    params_path = out/'params.yaml'; params_path.write_text(yaml.safe_dump(params))
    rclpy.init(); node = Node('controller_regression_observer')
    until = time.monotonic()+1.
    while time.monotonic()<until: rclpy.spin_once(node, timeout_sec=.1)
    if any(node.count_publishers(t) for t in ['/clock', '/sim/scan', '/sim/odom']):
        node.destroy_node(); rclpy.shutdown()
        raise RuntimeError('隔离域已有传感器发布者，拒绝启动测试')
    publishers = {
        'clock': node.create_publisher(Clock, '/clock', 10),
        'tf': node.create_publisher(TFMessage, '/tf', 100),
        'scan': node.create_publisher(LaserScan, '/sim/scan', qos_profile_sensor_data),
        'odom': node.create_publisher(Odometry, '/sim/odom', 10)}
    start = time.monotonic(); samples = []; events = []; proc = None
    stream = (out/'velocity.csv').open('w'); writer = csv.writer(stream)
    writer.writerow(['wall_s', 'vx', 'wz'])
    def command(msg):
        """保存控制服务真实输出；该话题没有任何驱动订阅者。"""
        sample = [time.monotonic()-start, msg.linear.x, msg.angular.z]
        samples.append(sample); writer.writerow(sample); stream.flush()
    sub = node.create_subscription(Twist, '/controller_test/cmd_vel', command, 100)
    def event(name, **values):
        """记录原样服务响应与故障，不把超时改写为成功。"""
        item = {'wall_s': time.monotonic()-start, 'event': name, **values}
        events.append(item); print(json.dumps(item), flush=True)
        with (out/'events.jsonl').open('a') as f: f.write(json.dumps(item)+'\n')
    def tick():
        """以墙钟推进合成仿真时钟，发布相同时间基准的 TF、扫描与里程计。"""
        t = 20.+time.monotonic()-start
        clock = Clock(); clock.clock.sec = int(t); clock.clock.nanosec = int(t%1*1e9)
        publishers['clock'].publish(clock)
        transforms = []
        for parent, child in [('map', 'odom'), ('odom', 'base_footprint'),
                              ('base_footprint', 'laser_frame')]:
            tf = TransformStamped(); tf.header.stamp = clock.clock
            tf.header.frame_id = parent; tf.child_frame_id = child; tf.transform.rotation.w = 1.
            transforms.append(tf)
        publishers['tf'].publish(TFMessage(transforms=transforms))
        scan = LaserScan(); scan.header.stamp = clock.clock; scan.header.frame_id = 'laser_frame'
        scan.angle_min = -math.pi; scan.angle_max = math.pi; scan.angle_increment = math.pi/180
        scan.range_min = .05; scan.range_max = 12.; scan.ranges = [10.]*361
        publishers['scan'].publish(scan)
        odom = Odometry(); odom.header.stamp = clock.clock; odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'; odom.pose.pose.orientation.w = 1.
        publishers['odom'].publish(odom)
        rclpy.spin_once(node, timeout_sec=.02)
    def wait(future, timeout):
        """等待期间不断更新传感器，区分数据失效与生命周期卡住。"""
        until = time.monotonic()+timeout
        while not future.done() and time.monotonic() < until: tick()
        return future.result() if future.done() else None
    change = node.create_client(ChangeState, '/controller_server/change_state')
    state = node.create_client(GetState, '/controller_server/get_state')
    def transition(label, number):
        """发送本隔离服务的生命周期请求，完整保存耗时。"""
        request = ChangeState.Request(); request.transition.id = number; begin = time.monotonic()
        reply = wait(change.call_async(request), 8.)
        good = reply is not None and reply.success
        event(label, success=good, response='timeout' if reply is None else str(reply.success),
              duration_wall_s=time.monotonic()-begin)
        return good
    def allow_debugger():
        """只允许诊断器读取本测试子进程堆栈，不附加到用户节点。"""
        ctypes.CDLL(None).prctl(0x59616d61, ctypes.c_ulong(-1).value, 0, 0, 0)
    report = {'result': 'FAIL', 'scope': 'SYNTHETIC_ROS_NO_ACTUATION', 'domain': args.domain_id,
              'controller_executable': str(args.executable), 'checks': {},
              'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'binary_sha256': hashlib.sha256(args.executable.read_bytes()).hexdigest()}
    (out/'verify_controller.py').write_bytes(Path(__file__).read_bytes())
    log = (out/'controller.log').open('w')
    try:
        proc = subprocess.Popen([str(args.executable), '--ros-args', '-r', '__node:=controller_server',
                                 '--params-file', str(params_path), '-r',
                                 'cmd_vel:=/controller_test/cmd_vel'], stdout=log,
                                stderr=subprocess.STDOUT, start_new_session=True, preexec_fn=allow_debugger)
        until = time.monotonic()+10
        while not change.service_is_ready() and time.monotonic()<until: tick()
        if not change.service_is_ready(): raise RuntimeError('controller service unavailable')
        if not transition('configure_initial', 1) or not transition('activate_initial', 3):
            raise RuntimeError('initial bringup failed')
        until = time.monotonic()+1
        while time.monotonic()<until: tick()
        action = ActionClient(node, FollowPath, '/follow_path')
        until = time.monotonic()+5
        while not action.server_is_ready() and time.monotonic()<until: tick()
        goal = FollowPath.Goal(); goal.controller_id = 'FollowPath'; goal.goal_checker_id = 'general_goal_checker'
        from geometry_msgs.msg import PoseStamped
        goal.path.header.frame_id = 'map'
        # 故意让路径时间比最新 TF 超前，复现日志中目标变换超龄；车辆仍在原点。
        t = 20.+time.monotonic()-start+3.
        goal.path.header.stamp.sec = int(t); goal.path.header.stamp.nanosec = int(t%1*1e9)
        for i in range(101):
            pose = PoseStamped(); pose.header = goal.path.header; pose.pose.position.x = i*.05
            pose.pose.orientation.w = 1.; goal.path.poses.append(pose)
        event('future_tf_goal_sent', distance_m=5., future_offset_s=3.)
        handle = wait(action.send_goal_async(goal), 5.)
        if handle is None or not handle.accepted: raise RuntimeError('FollowPath rejected')
        reply = wait(handle.get_result_async(), 8.)
        report['tf_fault_status'] = reply.status if reply else None
        event('future_tf_goal_result', status=report['tf_fault_status'])
        report['checks']['tf_fault_aborts'] = report['tf_fault_status'] == 6
        report['checks']['tf_fault_zero_output'] = not any(abs(x[1])>.001 or abs(x[2])>.001 for x in samples)
        good = True
        for i in range(args.cycles):
            if not transition(f'deactivate_{i}', 4) or not transition(f'cleanup_{i}', 2):
                good = False
                subprocess.run(['gdb', '-batch', '-ex', 'set pagination off', '-ex',
                                'thread apply all bt', '-p', str(proc.pid)],
                               stdout=(out/'cleanup_backtrace.txt').open('w'), stderr=subprocess.STDOUT,
                               timeout=12)
                break
            if not transition(f'configure_{i}', 1) or not transition(f'activate_{i}', 3):
                good = False; break
        report['checks']['repeat_reset_startup'] = good
        if good:
            # Reset/Startup 后不自动续跑旧动作；仅接收本验证器显式发出的新目标。
            before = len(samples); until = time.monotonic()+1.
            while time.monotonic()<until: tick()
            report['checks']['reset_does_not_resume'] = not any(
                abs(x[1])>.001 or abs(x[2])>.001 for x in samples[before:])
            fresh = FollowPath.Goal(); fresh.controller_id = 'FollowPath'
            fresh.goal_checker_id = 'general_goal_checker'; fresh.path.header.frame_id = 'map'
            pose = PoseStamped(); pose.header.frame_id = 'map'; pose.pose.orientation.w = 1.
            fresh.path.poses = [pose]
            handle = wait(action.send_goal_async(fresh), 5.)
            reply = wait(handle.get_result_async(), 5.) if handle and handle.accepted else None
            report['checks']['new_valid_goal_succeeds'] = reply is not None and reply.status == 4
            event('valid_goal_after_reset', status=reply.status if reply else None)
            # 有效远目标必须保持执行，显式取消后终止，不能因为修复而所有目标立即失败。
            goal.path.header.stamp.sec = 0; goal.path.header.stamp.nanosec = 0
            for pose in goal.path.poses: pose.header = goal.path.header
            handle = wait(action.send_goal_async(goal), 5.)
            result = handle.get_result_async() if handle and handle.accepted else None
            until = time.monotonic()+.6
            while time.monotonic()<until: tick()
            report['checks']['valid_far_goal_runs'] = result is not None and not result.done()
            if result is not None:
                wait(handle.cancel_goal_async(), 3.); reply = wait(result, 3.)
                report['checks']['cancel_terminates'] = reply is not None and reply.status == 5
                event('valid_far_goal_cancelled', status=reply.status if reply else None)
            handle = wait(action.send_goal_async(goal), 5.)
            result = handle.get_result_async() if handle and handle.accepted else None
            until = time.monotonic()+.3
            while time.monotonic()<until: tick()
            good = transition('deactivate_active_goal', 4) and transition('cleanup_active_goal', 2)
            reply = wait(result, 3.) if result else None
            report['checks']['active_goal_reset_terminates'] = good and reply is not None and reply.status in [5, 6]
            before = len(samples)
            good = good and transition('configure_after_active_goal', 1) and transition('activate_after_active_goal', 3)
            until = time.monotonic()+1.
            while time.monotonic()<until: tick()
            report['checks']['active_reset_no_resume'] = good and not any(
                abs(x[1])>.001 or abs(x[2])>.001 for x in samples[before:])
            event('active_goal_reset_complete', status=reply.status if reply else None,
                  zero_after_restart=report['checks']['active_reset_no_resume'])
        report['result'] = 'PASS' if all(report['checks'].values()) else 'FAIL'
    except Exception as exc:
        report['error'] = str(exc); event('exception', error=str(exc))
    finally:
        if proc is not None:
            os.killpg(proc.pid, signal.SIGINT)
            try: proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL); proc.wait()
        report['duration_wall_s'] = time.monotonic()-start
        report['command_samples'] = len(samples)
        (out/'result.json').write_text(json.dumps(report, indent=2))
        stream.close(); log.close(); node.destroy_node(); rclpy.shutdown()
        (out/'SHA256.json').write_text(json.dumps({str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest()
                                                 for p in out.rglob('*') if p.is_file()},indent=2))
        print(json.dumps(report, indent=2), flush=True)
    return 0 if report['result']=='PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
