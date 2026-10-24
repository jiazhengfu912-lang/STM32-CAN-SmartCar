"""在首个位姿之后才创建建图门控，验证无需运动/人工目标即可就绪，输出始终为零。"""
from pathlib import Path
import sys,os,time,uuid,json,csv,subprocess,hashlib
root=Path.cwd();ws=root/'ros2_ws'
out=ws/'verification_runs/discovery-lab-20261021-late-gate';out.mkdir(exist_ok=False)
partition='discovery-late-'+uuid.uuid4().hex
os.environ.update(ROS_DOMAIN_ID='162',ROS_LOCALHOST_ONLY='1',IGN_PARTITION=partition,GZ_PARTITION=partition,ROS_LOG_DIR=str(out/'ros_logs'))
sys.path[:0]=[str(ws/'src/smartcar_nav2/scripts'),str(ws/'tools')]
from verify_navigation import stop_group
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped,Twist
from nav_msgs.msg import OccupancyGrid,Odometry
from std_msgs.msg import String
from navigation_gate import GateNode
rclpy.init(args=['--ros-args','-p','localization_source:=slam'])
probe=rclpy.create_node('late_gate_probe');executor=SingleThreadedExecutor();executor.add_node(probe)
started=time.monotonic();maps=[];poses=[];states=[];commands=[];odometry=[]
probe.create_subscription(OccupancyGrid,'/map',lambda m:maps.append(time.monotonic()),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
probe.create_subscription(PoseWithCovarianceStamped,'/pose',lambda m:poses.append(time.monotonic()),10)
probe.create_subscription(String,'/navigation/safety_status',lambda m:states.append((time.monotonic(),json.loads(m.data))),10)
probe.create_subscription(Twist,'/nav/safe_cmd_vel',lambda m:commands.append((time.monotonic()-started,m.linear.x,m.angular.z)),10)
probe.create_subscription(Odometry,'/sim/odom',lambda m:odometry.append((time.monotonic()-started,m.pose.pose.position.x,m.pose.pose.position.y,m.twist.twist.linear.x,m.twist.twist.angular.z)),10)
log=(out/'launch.log').open('w');process=None;gate=None
report={'result':'FAIL','scope':'SIMULATION_ONLY','domain_id':162,'partition':partition,'no_commands_or_goals_sent':True}

def until(predicate,seconds):
    """只处理订阅与门控定时器，不发布导航目标或驾驶速度。"""
    end=time.monotonic()+seconds
    while time.monotonic()<end and not predicate():executor.spin_once(timeout_sec=.05)
    if not predicate():raise TimeoutError('late gate verification timeout')

try:
    process=subprocess.Popen(['ros2','launch','smartcar_slam','mapping.launch.py','world:=discovery_lab','headless:=true','rviz:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    until(lambda:bool(maps),100.);until(lambda:time.monotonic()-maps[0]>3.,10.)
    before=len(poses);gate_started=time.monotonic();gate=GateNode();executor.add_node(gate)
    until(lambda:bool(states) and states[-1][1]['state']=='COMMAND_STALE',15.)
    until(lambda:time.monotonic()-gate_started>4.,10.)
    report.update({'gate_delay_after_first_map_s':gate_started-maps[0],'pose_samples_before_gate':before,
                   'pose_samples_after_gate':len(poses)-before,'localized':gate.core.localized,'last_state':states[-1][1],
                   'checks':{'gate_late_join_ready':gate.core.localized and states[-1][1]['state']=='COMMAND_STALE',
                             'no_pose_received_after_gate':len(poses)==before,'all_commands_zero':bool(commands) and all(abs(v)<1e-9 and abs(w)<1e-9 for _,v,w in commands)}})
    report['result']='PASS' if all(report['checks'].values()) else 'FAIL'
except Exception as e:report['error']=str(e)
finally:
    if gate:executor.remove_node(gate);gate.destroy_node()
    if process:report['shutdown']=stop_group(process,lambda name,**kw:None,pump=lambda:executor.spin_once(timeout_sec=.01))
    log.close();probe.destroy_node();rclpy.shutdown()
    for name,header,data in [('commands',['wall_s','vx','wz'],commands),('odom',['wall_s','x','y','vx','wz'],odometry)]:
        with (out/(name+'.csv')).open('w',newline='') as f:w=csv.writer(f);w.writerow(header);w.writerows(data)
    (out/'gate.json').write_text(json.dumps([(t-started,s) for t,s in states],indent=2))
    report['raw_sha256']={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in out.rglob('*') if p.is_file()}
    (out/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
raise SystemExit(0 if report['result']=='PASS' else 1)
