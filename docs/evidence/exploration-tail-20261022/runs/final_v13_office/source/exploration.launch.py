"""从空白地图启动自主探索；地图与map→odom仅由SLAM发布。"""
import datetime
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """导航和建图共用原有保护，探索器只发送导航目标，不直接发布速度。"""
    nav=Path(get_package_share_directory('smartcar_nav2'))
    default=Path.home()/'smartcar_maps'/('explore_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    explorer=Node(package='smartcar_slam',executable='explore.py',name='frontier_explorer',output='screen',
             parameters=[{'use_sim_time':True,'output_dir':LaunchConfiguration('output_dir'),
                          'max_sim_seconds':ParameterValue(LaunchConfiguration('max_sim_seconds'),value_type=float)}])
    return LaunchDescription([
        DeclareLaunchArgument('world',default_value='office_loop'),
        DeclareLaunchArgument('rviz',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('headless',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('output_dir',default_value=str(default)),
        DeclareLaunchArgument('max_sim_seconds',default_value='900.0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav/'launch/navigation.launch.py')),
            launch_arguments={'world':LaunchConfiguration('world'),'slam':'true','exploration_watchdog':'true','rviz':LaunchConfiguration('rviz'),
                              'headless':LaunchConfiguration('headless')}.items()),
        explorer,RegisterEventHandler(OnProcessExit(target_action=explorer,
            on_exit=[EmitEvent(event=Shutdown(reason='Explorer exited; stopping autonomous session'))]))])
