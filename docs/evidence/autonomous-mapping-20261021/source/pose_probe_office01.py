import rclpy,time,os,json,subprocess,signal,math
from pathlib import Path
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
folder=Path('ros2_ws/verification_runs/exploration-20261021-office-01')
pid=json.loads((folder/'verifier_events.jsonl').read_text().splitlines()[0])['pid']
env=dict(x.split(b'=',1) for x in Path(f'/proc/{pid}/environ').read_bytes().split(b'\0') if b'=' in x)
os.environ['IGN_PARTITION']=env[b'IGN_PARTITION'].decode();os.environ['GZ_PARTITION']=os.environ['IGN_PARTITION']
log=(folder/'pose_probe_bridge.log').open('w');proc=subprocess.Popen(['ros2','run','ros_gz_bridge','parameter_bridge','/world/smartcar_world/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V','--ros-args','-r','/world/smartcar_world/pose/info:=/verification/world_pose','-r','__node:=exploration_pose_probe_bridge'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();node=rclpy.create_node('exploration_pose_probe');latest={}
def pose(key,p,q):latest[key]={'x':p.x,'y':p.y,'yaw':math.atan2(2*q.w*q.z,1-2*q.z*q.z),'monotonic':time.monotonic()}
def truth(msg):
 for t in msg.transforms:
  if t.child_frame_id=='smartcar':pose('world',t.transform.translation,t.transform.rotation)
node.create_subscription(TFMessage,'/verification/world_pose',truth,10)
node.create_subscription(Odometry,'/sim/odom',lambda m:pose('odom',m.pose.pose.position,m.pose.pose.orientation),10)
node.create_subscription(PoseWithCovarianceStamped,'/pose',lambda m:pose('slam',m.pose.pose.position,m.pose.pose.orientation),10)
try:
 end=time.monotonic()+8
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
 (folder/'pose_probe.json').write_text(json.dumps(latest,indent=2));print(latest)
finally:
 os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=10);log.close();node.destroy_node();rclpy.shutdown()
