"""Start one lidar simulator and SLAM together; RViz is opt-in."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim=Path(get_package_share_directory('smartcar_sim'))
    slam=Path(get_package_share_directory('smartcar_slam'))
    return LaunchDescription([
        DeclareLaunchArgument('world',default_value='smartcar',description='smartcar_sim scene name or absolute SDF template path.'),
        DeclareLaunchArgument('rviz',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('headless',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('map_file',default_value=''),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(sim/'launch/sim.launch.py')),
                                 launch_arguments={'world':LaunchConfiguration('world'),'lidar':'true','camera':'false','rviz':LaunchConfiguration('rviz'),
                                                   'headless':LaunchConfiguration('headless')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(slam/'launch/slam.launch.py')),
                                 launch_arguments={'use_sim_time':'true','map_file':LaunchConfiguration('map_file')}.items())])
