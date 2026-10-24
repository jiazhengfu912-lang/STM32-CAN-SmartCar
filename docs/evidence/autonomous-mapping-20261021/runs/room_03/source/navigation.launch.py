"""启动一套仿真和Nav2；默认已知地图AMCL，显式slam模式使用在线建图。"""
import math
import os
from pathlib import Path
import time
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    share = Path(get_package_share_directory('smartcar_nav2'))
    sim = Path(get_package_share_directory('smartcar_sim'))
    value = lambda name: LaunchConfiguration(name).perform(context)
    mapping = value('slam').lower() == 'true'
    map_path = Path(value('map')).expanduser().resolve()
    params = Path(value('params_file')).expanduser().resolve()
    bt_xml = Path(value('bt_xml')).expanduser().resolve()
    through_bt_xml = Path(value('through_poses_bt_xml')).expanduser().resolve()
    if not params.is_file() or (not mapping and not map_path.is_file()):
        raise RuntimeError(f'Missing map YAML or Nav2 parameters: {map_path}, {params}')
    if not bt_xml.is_file() or not through_bt_xml.is_file():
        raise RuntimeError(f'Missing navigation behavior tree: {bt_xml}, {through_bt_xml}')
    if not mapping:
        metadata = yaml.safe_load(map_path.read_text())
        image = map_path.parent / metadata['image']
        if not image.is_file() or not math.isfinite(float(metadata['resolution'])) or metadata['resolution'] <= 0:
            raise RuntimeError('Map YAML requires an existing image and positive resolution.')
    initial = {k: float(value('initial_'+k)) for k in ['x', 'y', 'yaw']}
    if not all(math.isfinite(v) for v in initial.values()):
        raise RuntimeError('Initial pose must be finite.')
    servers = [('smartcar_nav2', 'controller_server'), ('nav2_smoother', 'smoother_server'),
               ('smartcar_nav2', 'planner_server'), ('nav2_behaviors', 'behavior_server'),
               ('nav2_bt_navigator', 'bt_navigator'), ('nav2_waypoint_follower', 'waypoint_follower'),
               ('nav2_velocity_smoother', 'velocity_smoother')]
    for package, executable in servers + [('nav2_amcl', 'amcl'), ('nav2_map_server', 'map_server'),
                                           ('smartcar_nav2', 'navigation_gate.py')]:
        binary = 'safe_'+executable if package == 'smartcar_nav2' and executable in ['controller_server', 'planner_server'] else executable
        path = Path(get_package_prefix(package))/'lib'/package/binary
        if not os.access(path, os.X_OK):
            raise RuntimeError(f'Missing executable, install dependencies/rebuild before starting: {path}')
    # A second simulator or SLAM would mix clocks, scans and map->odom authorities.
    import rclpy
    rclpy.init(args=[])
    probe = rclpy.create_node('navigation_startup_check')
    try:
        until = time.monotonic() + 1.0
        while time.monotonic() < until:
            rclpy.spin_once(probe, timeout_sec=.1)
        conflicts = [topic for topic in ['/clock', '/map', '/sim/scan', '/sim/odom', '/nav/safe_cmd_vel']
                     if probe.count_publishers(topic)]
        if conflicts:
            raise RuntimeError('Stop the previous simulation/SLAM/Nav2 first. Existing publishers: '+', '.join(conflicts))
    finally:
        probe.destroy_node()
        rclpy.shutdown()
    common = [str(params), {'use_sim_time': True}]
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sim/'launch/sim.launch.py')),
        launch_arguments={'world': value('world'), 'headless': value('headless'),
                          'rviz': 'false', 'lidar': 'true', 'camera': 'false',
                          'cmd_vel_topic': '/nav/safe_cmd_vel'}.items())]
    if mapping:
        slam = Path(get_package_share_directory('smartcar_slam'))
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(slam/'launch/slam.launch.py')),
                        launch_arguments={'use_sim_time':'true','map_file':'',
                                          'params_file':str(slam/'config/slam.yaml')}.items()))
    else:
        actions += [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=common+[{'yaml_filename': str(map_path)}], output='screen'),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             parameters=common+[{'initial_pose.x': initial['x'], 'initial_pose.y': initial['y'],
                                 'initial_pose.z': 0.0, 'initial_pose.yaw': initial['yaw']}], output='screen')]
    for package, name in servers:
        remaps = []
        extra = []
        if name in ['controller_server', 'behavior_server']:
            remaps = [('cmd_vel', '/nav/cmd_vel_raw')]
        elif name == 'velocity_smoother':
            remaps = [('cmd_vel', '/nav/cmd_vel_raw'), ('cmd_vel_smoothed', '/nav/cmd_vel')]
        elif name == 'bt_navigator':
            # 单目标与连续多目标均保留已观测的堵路信息，避免落回自动清图的默认树。
            extra = [{'default_nav_to_pose_bt_xml': str(bt_xml),
                      'default_nav_through_poses_bt_xml': str(through_bt_xml)}]
        binary = 'safe_'+name if package == 'smartcar_nav2' else name
        server = Node(package=package, executable=binary, name=name, parameters=common+extra,
                      remappings=remaps, output='screen')
        actions.extend([server, RegisterEventHandler(OnProcessExit(target_action=server,
                        on_exit=[EmitEvent(event=Shutdown(reason=name+' exited'))]))])
    managers = [('navigation', [name for _, name in servers])]
    if not mapping:managers.insert(0, ('localization', ['map_server', 'amcl']))
    for suffix, names in managers:
        manager = Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                            name='lifecycle_manager_'+suffix,
                            parameters=[{'use_sim_time': True, 'autostart': True, 'bond_timeout': 8.0,
                                         'node_names': names}], output='screen')
        # Give local DDS discovery and software-rendered sensors time before lifecycle RPCs.
        actions.append(TimerAction(period=2.0 if suffix == 'localization' else 5.0, actions=[manager]))
    gate = Node(package='smartcar_nav2', executable='navigation_gate.py', name='navigation_gate',
                parameters=[{'localization_source':'slam' if mapping else 'amcl',
                             'exploration_watchdog':value('exploration_watchdog').lower()=='true'}], output='screen')
    actions.extend([gate, RegisterEventHandler(OnProcessExit(target_action=gate,
                    on_exit=[EmitEvent(event=Shutdown(reason='Navigation command gate exited'))]))])
    if value('rviz').lower() == 'true':
        actions.append(Node(package='rviz2', executable='rviz2',
                            arguments=['-d', str(share/'rviz/navigation.rviz')],
                            parameters=[{'use_sim_time': True}], output='screen'))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('smartcar_nav2'))
    # 示例地图随包安装，无需依赖开发者机器上的工作空间绝对路径。
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='office_loop'),
        DeclareLaunchArgument('slam', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('exploration_watchdog', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('map', default_value=str(share/'maps/office_01/map.yaml')),
        DeclareLaunchArgument('params_file', default_value=str(share/'config/nav2.yaml')),
        DeclareLaunchArgument('bt_xml', default_value=str(share/'config/navigate_recovery.xml')),
        DeclareLaunchArgument('through_poses_bt_xml', default_value=str(share/'config/navigate_through_recovery.xml')),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        OpaqueFunction(function=start)])
