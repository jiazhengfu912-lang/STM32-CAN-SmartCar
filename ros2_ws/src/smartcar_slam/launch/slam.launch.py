"""Attach SLAM to an existing simulator. Does not open RViz or move the car."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start(context):
    """可选叠加配置用于自主探索；空配置沿用原始手动建图参数。"""
    params=Path(LaunchConfiguration('params_file').perform(context)).expanduser()
    if not params.is_file():
        raise RuntimeError(f'SLAM parameter file does not exist: {params}')
    files=[str(params)]
    overlay=LaunchConfiguration('override_params_file').perform(context)
    if overlay:
        overlay=Path(overlay).expanduser()
        if not overlay.is_file():raise RuntimeError(f'SLAM override file does not exist: {overlay}')
        files.append(str(overlay))
    prefix=LaunchConfiguration('map_file').perform(context)
    if prefix:
        prefix=str(Path(prefix).expanduser().resolve())
        missing=[prefix+suffix for suffix in ['.posegraph','.data'] if not Path(prefix+suffix).is_file()]
        if missing:
            raise RuntimeError('Resume requires a serialized SLAM prefix, not a PGM/YAML map. Missing: '+', '.join(missing))
    return [Node(package='slam_toolbox',executable='async_slam_toolbox_node',name='slam_toolbox',
                 parameters=files+[{'use_sim_time':LaunchConfiguration('use_sim_time').perform(context)=='true',
                                         'map_file_name':ParameterValue(prefix, value_type=str),'map_start_at_dock':True}],output='screen')]


def generate_launch_description():
    share=Path(get_package_share_directory('smartcar_slam'))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('params_file',default_value=str(share/'config/slam.yaml')),
        DeclareLaunchArgument('override_params_file',default_value='',description='Optional autonomous-mapping parameter overlay.'),
        DeclareLaunchArgument('map_file',default_value='',description='Optional serialized graph prefix; robot must be near its original mapping start.'),
        OpaqueFunction(function=start)])
