#!/usr/bin/env python3
"""Send one map-frame Nav2 goal, report completion; Ctrl+C requests cancellation."""
import argparse
import json
import math
import os
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float)
    parser.add_argument('--y', type=float)
    parser.add_argument('--yaw', type=float, default=0., help='Radians in the map frame.')
    parser.add_argument('--timeout', type=float, default=180.)
    parser.add_argument('--cancel', action='store_true')
    args = parser.parse_args()
    if not os.environ.get('ROS_DOMAIN_ID') or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('First export SMARTCAR_WS and source smartcar_sim/tools/sim_env.bash.')
    if not args.cancel and (args.x is None or args.y is None or not all(math.isfinite(v) for v in [args.x,args.y,args.yaw,args.timeout]) or args.timeout <= 0):
        parser.error('Provide finite --x, --y, --yaw and positive --timeout.')
    # Keep the ROS context alive during Ctrl+C so cancellation can reach Nav2.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('smartcar_navigation_goal', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    handle = None
    code = 1
    def wait(future, seconds):
        end = time.monotonic()+seconds
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        if not future.done():
            raise TimeoutError('Nav2 response timed out.')
        return future.result()
    try:
        if args.cancel:
            cancel = node.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
            if not cancel.wait_for_service(timeout_sec=10.):
                raise RuntimeError('NavigateToPose cancellation service unavailable.')
            response = wait(cancel.call_async(CancelGoal.Request()), 10)
            print(json.dumps({'cancel_return_code': response.return_code, 'goals_canceling': len(response.goals_canceling)}))
            code = 0
        else:
            if not client.wait_for_server(timeout_sec=30.):
                raise RuntimeError('NavigateToPose server unavailable; start navigation.launch.py.')
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = args.x, args.y
            goal.pose.pose.orientation.z = math.sin(args.yaw/2)
            goal.pose.pose.orientation.w = math.cos(args.yaw/2)
            last_print = [0.]
            def feedback(msg):
                if time.monotonic()-last_print[0] > 1.:
                    print(json.dumps({'distance_remaining_m':msg.feedback.distance_remaining,
                                      'recoveries':msg.feedback.number_of_recoveries}), flush=True)
                    last_print[0] = time.monotonic()
            handle = wait(client.send_goal_async(goal, feedback_callback=feedback), 15)
            if not handle.accepted:
                raise RuntimeError('Goal rejected by Nav2.')
            result = wait(handle.get_result_async(), args.timeout)
            status = {GoalStatus.STATUS_SUCCEEDED:'SUCCEEDED', GoalStatus.STATUS_CANCELED:'CANCELED',
                      GoalStatus.STATUS_ABORTED:'ABORTED'}.get(result.status, str(result.status))
            print(json.dumps({'navigation_result': status}))
            code = 0 if result.status == GoalStatus.STATUS_SUCCEEDED else 1
    except (Exception, KeyboardInterrupt) as exc:
        if handle is not None and handle.accepted:
            try:
                wait(handle.cancel_goal_async(), 5)
                wait(handle.get_result_async(), 5)
            except Exception:
                pass
        print(json.dumps({'error': str(exc) or type(exc).__name__, 'cancellation_requested': handle is not None}))
    finally:
        node.destroy_node();rclpy.try_shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
