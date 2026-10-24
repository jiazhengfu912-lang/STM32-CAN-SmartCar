"""只启动隔离的地图服务器，核对PGM/YAML加载后的未知栅格，不启动机器人或发速度。"""
import argparse,os,subprocess,time,json,signal
from pathlib import Path
import rclpy
from nav_msgs.msg import OccupancyGrid
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
p=argparse.ArgumentParser();p.add_argument('map',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=False);os.environ.update(ROS_DOMAIN_ID='184',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(a.output/'ros_logs'))
log=(a.output/'server.log').open('w');process=None;received=[];report={'scope':'READONLY_MAP_SERVER_NO_ROBOT','map':str(a.map.resolve())}
rclpy.init();node=rclpy.create_node('saved_map_roundtrip_probe')
node.create_subscription(OccupancyGrid,'/map',lambda msg:received.append(msg),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))

def until(predicate,seconds=15.):
    """有界等待本测试节点，未收到数据时明确失败。"""
    end=time.monotonic()+seconds
    while time.monotonic()<end and not predicate():rclpy.spin_once(node,timeout_sec=.05)
    if not predicate():raise TimeoutError('map server probe timeout')

try:
    process=subprocess.Popen(['ros2','run','nav2_map_server','map_server','--ros-args','-r','__node:=roundtrip_map_server','-p','yaml_filename:='+str(a.map.resolve())],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    client=node.create_client(ChangeState,'/roundtrip_map_server/change_state');until(client.service_is_ready)
    for transition in [Transition.TRANSITION_CONFIGURE,Transition.TRANSITION_ACTIVATE]:
        request=ChangeState.Request();request.transition.id=transition;future=client.call_async(request);until(future.done)
        assert future.result().success
    until(lambda:bool(received));values=received[-1].data
    report.update(result='PASS',cells=len(values),unknown=sum(v<0 for v in values),free=sum(v==0 for v in values),occupied=sum(v>=65 for v in values))
except Exception as exc:report.update(result='FAIL',error=str(exc))
finally:
    if process:
        os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=10.)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
    log.close();node.destroy_node();rclpy.shutdown();(a.output/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
