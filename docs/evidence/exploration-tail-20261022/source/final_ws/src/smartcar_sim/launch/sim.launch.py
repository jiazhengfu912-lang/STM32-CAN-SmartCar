"""启动 Fortress、桥接、机器人 TF 和独立超时保护。"""
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_world(worlds, selection):
    """Load a packaged scene or an absolute SDF template before starting processes."""
    path = Path(selection).expanduser()
    if not path.is_absolute():
        path = worlds / path
    if not path.suffix:
        path = path.with_suffix('.sdf')
    if not path.is_file():
        available = ', '.join(sorted(p.stem for p in worlds.glob('*.sdf')))
        raise ValueError(f'World not found: {path}. Available scenes: {available}')
    tree = ET.parse(path)
    if len(tree.findall('./world')) != 1:
        raise ValueError(f'World template must contain exactly one <world>: {path}')
    includes = [item for item in tree.findall('./world/include')
                if item.findtext('name') == 'smartcar' and item.findtext('uri') == '__MODEL_SDF__']
    if len(includes) != 1:
        raise ValueError(f'World template requires one include named smartcar with URI __MODEL_SDF__: {path}')
    return path.resolve(), tree, includes[0].find('uri')


def start(context):
    """按启用的传感器选择同源模型，避免在低配虚拟机上强制渲染。"""
    share=Path(get_package_share_directory('smartcar_sim'))
    world_source,world,model_uri=load_world(share/'worlds',LaunchConfiguration('world').perform(context))
    flag=lambda name: LaunchConfiguration(name).perform(context).lower()=='true'
    lidar,camera=flag('lidar'),flag('camera')
    profile=LaunchConfiguration('camera_profile').perform(context)
    variant=('sensors_1080p' if profile=='1080p' else 'sensors_preview') if camera else ('lidar' if lidar else 'physics')
    model=share/f'models/smartcar/model_{variant}.sdf'
    # 摄像头模式的预生成文件包含雷达；用户关闭雷达时在临时副本中去掉该传感器。
    source=ET.parse(model)
    if not lidar:
        link=source.find("./model/link[@name='lidar_link']")
        for sensor in list(link.findall('sensor')):link.remove(sensor)
    with tempfile.NamedTemporaryFile(mode='wb',prefix='smartcar_model_',suffix='.sdf',delete=False) as f:
        model_path=f.name;source.write(f,encoding='utf-8',xml_declaration=True)
    model_uri.text=model_path
    if (lidar or camera) and not any(p.get('name')=='ignition::gazebo::systems::Sensors' for p in world.findall('./world/plugin')):
        plugin=ET.SubElement(world.find('world'),'plugin',filename='ignition-gazebo-sensors-system',name='ignition::gazebo::systems::Sensors')
        ET.SubElement(plugin,'render_engine').text='ogre2'
    with tempfile.NamedTemporaryFile(mode='wb',prefix='smartcar_world_',suffix='.sdf',delete=False) as f:
        world_path=f.name;world.write(f,encoding='utf-8',xml_declaration=True)
    def cleanup(_context,*args,**kwargs):
        for filename in [model_path,world_path]:
            try:os.unlink(filename)
            except FileNotFoundError:pass
        return []
    resource=os.pathsep.join([str(share/'models'),str(world_source.parent)])
    if os.environ.get('IGN_GAZEBO_RESOURCE_PATH'):resource+=os.pathsep+os.environ['IGN_GAZEBO_RESOURCE_PATH']
    description=(share/'urdf/smartcar.urdf').read_text()
    # Use discrete wheel events: Qt's XI2 scrolling is sluggish through VMware/XWayland.
    gazebo=ExecuteProcess(cmd=['ign','gazebo','-r','-v','3']+(['-s'] if flag('headless') else ['--gui-config',str(share/'config/gazebo_gui.config')])+[world_path],
                          output='screen',additional_env={'QT_XCB_NO_XI2':'1'})
    guard=Node(package='smartcar_sim',executable='cmd_guard.py',name='sim_cmd_guard',
               remappings=[('/sim/cmd_vel',LaunchConfiguration('cmd_vel_topic'))],output='screen')
    actions=[SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',resource),
             RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),gazebo,
             RegisterEventHandler(OnProcessExit(target_action=gazebo,on_exit=[EmitEvent(event=Shutdown(reason='Gazebo exited'))])),
             Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':description,'use_sim_time':True}],output='screen'),
             Node(package='ros_gz_bridge',executable='parameter_bridge',name='state_bridge',parameters=[{'config_file':str(share/'config/bridge.yaml')}],output='screen'),
             guard,
             RegisterEventHandler(OnProcessExit(target_action=guard,on_exit=[EmitEvent(event=Shutdown(reason='Command timeout guard exited'))]))]
    if lidar:
        actions.append(Node(package='ros_gz_bridge',executable='parameter_bridge',name='laser_bridge',arguments=['/sim/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],parameters=[{'override_frame_id':'laser_frame'}],output='screen'))
    if camera:
        actions.append(Node(package='ros_gz_bridge',executable='parameter_bridge',name='camera_bridge',arguments=['/sim/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image','/sim/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'],parameters=[{'override_frame_id':'camera_optical_frame'}],output='screen'))
    if flag('rviz'):
        actions.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'rviz/smartcar.rviz')],parameters=[{'use_sim_time':True}],output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world',default_value='smartcar',description='Scene name under worlds/ (e.g. office_loop), or absolute SDF template path.'),
        DeclareLaunchArgument('cmd_vel_topic',default_value='/sim/cmd_vel',description='Input topic to the existing timeout guard; navigation uses a separate gated topic.'),
        DeclareLaunchArgument('headless',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('rviz',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('lidar',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('camera',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('camera_profile',default_value='preview',choices=['preview','1080p']),
        OpaqueFunction(function=start)])
