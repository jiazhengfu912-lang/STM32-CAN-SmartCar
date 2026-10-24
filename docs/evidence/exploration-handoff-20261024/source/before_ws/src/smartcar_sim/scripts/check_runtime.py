#!/usr/bin/env python3
"""记录真实 Gazebo/ROS 消息；默认只读，--motion 才发送仿真动作。"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class Recorder(Node):
    def __init__(self,folder):
        super().__init__('sim_acceptance_recorder')
        self.folder=folder;self.start=time.monotonic();self.phase='wait';self.odom=[];self.joints=set();self.scan=None;self.image=None;self.info=None;self.clock_count=0;self.clock_first=None;self.clock_last=None;self.odom_frames=None
        self.joint_rows=[];self.world_rows=[];self.command_rows=[];self.last_publish_wall=None;self.signal_loss_last_publish=None;self.timeout_delay=None
        self.raw_files={}
        for name,header in [('joints',['wall_s','phase','ros_stamp_s','name','position','velocity']),('commands',['wall_s','phase','source','vx','wz']),('world_pose',['wall_s','phase','x','y','z','yaw'])]:
            f=(folder/(name+'.csv')).open('w',newline='');w=csv.writer(f);w.writerow(header);self.raw_files[name]=(f,w)
        self.guarded=(0.0,0.0);self.last_input=(0.0,0.0);self.phase_samples={};self.counts={'odom':0,'joint_states':0,'scan':0,'image':0,'camera_info':0}
        self.csv_file=(folder/'motion.csv').open('w',newline='')
        self.writer=csv.writer(self.csv_file);self.writer.writerow(['wall_s','ros_stamp_s','phase','input_vx','input_wz','guard_vx','guard_wz','odom_vx','odom_wz','odom_x','odom_y','odom_yaw'])
        self.publisher=self.create_publisher(Twist,'/sim/cmd_vel',10)
        self.create_subscription(Odometry,'/sim/odom',self.on_odom,10)
        self.create_subscription(TFMessage,'/sim/world_pose',self.on_world,10)
        self.create_subscription(JointState,'/joint_states',self.on_joints,10)
        self.create_subscription(Twist,'/sim/drive_cmd_vel',self.on_guard,10)
        self.create_subscription(Clock,'/clock',self.on_clock,10)
        self.create_subscription(LaserScan,'/sim/scan',self.on_scan,qos_profile_sensor_data)
        self.create_subscription(Image,'/sim/camera/image_raw',self.on_image,qos_profile_sensor_data)
        self.create_subscription(CameraInfo,'/sim/camera/camera_info',self.on_info,qos_profile_sensor_data)

    def on_odom(self,msg):
        self.counts['odom']+=1;p=msg.pose.pose;v=msg.twist.twist
        self.odom_frames=(msg.header.frame_id,msg.child_frame_id)
        row=[time.monotonic()-self.start,msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,self.phase,*self.last_input,*self.guarded,v.linear.x,v.angular.z,p.position.x,p.position.y,yaw_of(p.orientation)]
        self.writer.writerow(row);self.csv_file.flush();self.odom.append(row);self.phase_samples.setdefault(self.phase,[]).append(row)

    def raw(self,name,row):
        f,w=self.raw_files[name];w.writerow(row);f.flush()

    def on_joints(self,msg):
        self.joints.update(msg.name);self.counts['joint_states']+=1
        for i,name in enumerate(msg.name):
            row=[time.monotonic()-self.start,self.phase,msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,name,msg.position[i] if i<len(msg.position) else float('nan'),msg.velocity[i] if i<len(msg.velocity) else float('nan')]
            self.raw('joints',row);self.joint_rows.append(row)

    def on_guard(self,msg):
        self.guarded=(msg.linear.x,msg.angular.z)
        row=[time.monotonic()-self.start,self.phase,'guard',*self.guarded]
        self.command_rows.append(row);self.raw('commands',row)
        if self.phase=='signal_loss' and self.timeout_delay is None and self.signal_loss_last_publish is not None and all(abs(v)<.001 for v in self.guarded):
            self.timeout_delay=time.monotonic()-self.signal_loss_last_publish

    def on_world(self,msg):
        for transform in msg.transforms:
            if transform.child_frame_id=='smartcar':
                t=transform.transform;p=t.translation
                row=[time.monotonic()-self.start,self.phase,p.x,p.y,p.z,yaw_of(t.rotation)]
                self.world_rows.append(row);self.raw('world_pose',row)

    def on_clock(self,msg):
        self.clock_count+=1;self.clock_last=msg.clock.sec+msg.clock.nanosec/1e9
        if self.clock_first is None:self.clock_first=self.clock_last

    def on_scan(self,msg):
        self.counts['scan']+=1;self.scan={'frame_id':msg.header.frame_id,'samples':len(msg.ranges),'range_min':msg.range_min,'range_max':msg.range_max,'angle_increment':msg.angle_increment,'finite_ranges':sum(math.isfinite(v) for v in msg.ranges)}

    def on_image(self,msg):
        self.counts['image']+=1;self.image={'frame_id':msg.header.frame_id,'width':msg.width,'height':msg.height,'encoding':msg.encoding}

    def on_info(self,msg):
        self.counts['camera_info']+=1;self.info={'frame_id':msg.header.frame_id,'width':msg.width,'height':msg.height,'k':list(msg.k),'d':list(msg.d)}

    def spin_for(self,seconds,phase,command=None):
        """动作和停发阶段都保留原始数据；停发时不伪造零输入消息。"""
        self.phase=phase;end=time.monotonic()+seconds;next_publish=0
        if phase=='signal_loss':self.signal_loss_last_publish=self.last_publish_wall
        while time.monotonic()<end and rclpy.ok():
            if command is not None and time.monotonic()>=next_publish:
                msg=Twist();msg.linear.x,msg.angular.z=command;self.publisher.publish(msg);self.last_input=command;self.last_publish_wall=time.monotonic();self.raw('commands',[self.last_publish_wall-self.start,self.phase,'input',*command]);next_publish=time.monotonic()+.1
            rclpy.spin_once(self,timeout_sec=.02)


def plot_rows(rows,folder):
    """绘制全部收到的原始消息；超长缺口断线，不平滑或补点。"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,1,figsize=(11,6),sharex=True)
    times=[];values=[[],[],[],[]];previous=None
    for row in rows:
        if previous is not None and row[0]-previous>.5:
            times.append(float('nan'))
            for series in values:series.append(float('nan'))
        times.append(row[0]);previous=row[0]
        for series,column in zip(values,[5,7,6,8]):series.append(row[column])
    for ax,offset,label in [(axes[0],0,'linear velocity (m/s)'),(axes[1],2,'angular velocity (rad/s)')]:
        ax.plot(times,values[offset],label='guard output (received ROS messages)');ax.plot(times,values[offset+1],label='Gazebo wheel odometry');ax.set_ylabel(label);ax.grid();ax.legend()
    axes[1].set_xlabel('wall time (s)')
    fig.suptitle('SmartCar sim v1 / Fortress / raw samples, NOT hardware\nMCU P/I unchanged; firmware algorithm unchanged; ideal Gazebo DiffDrive')
    fig.tight_layout();fig.savefig(folder/'motion.png',dpi=160);fig.savefig(folder/'motion.svg');plt.close(fig)


def evaluate(node,args):
    """依据收到的消息判断，不用离线运动学计算替代 Gazebo 运行。"""
    expected={'left_wheel_joint','right_wheel_joint','left_caster_swivel_joint','left_caster_roll_joint','right_caster_swivel_joint','right_caster_roll_joint'}
    checks={'odom_received':node.counts['odom']>=5,'clock_advancing':node.clock_count>=2 and node.clock_last>node.clock_first,'all_moving_joints_received':expected.issubset(node.joints),
            'odom_frames':node.odom_frames==('odom','base_footprint'),'odom_finite':bool(node.odom) and all(math.isfinite(value) for row in node.odom for value in row[7:]),
            'odom_fresh':bool(node.odom) and time.monotonic()-node.start-node.odom[-1][0]<1.0}
    if args.lidar:checks['c1_scan']=bool(node.scan and node.scan['frame_id']=='laser_frame' and node.scan['samples']==500 and node.scan['finite_ranges']>0)
    if args.camera:
        target=(1920,1080) if args.camera=='1080p' else (640,360)
        checks['camera_image']=bool(node.image and (node.image['width'],node.image['height'])==target and node.image['frame_id']=='camera_optical_frame')
        checks['camera_info']=bool(node.info and (node.info['width'],node.info['height'])==target and node.info['frame_id']=='camera_optical_frame' and node.info['k'][0]>0)
    if args.motion:
        for phase in ['forward','backward','turn_left','turn_right']:
            rows=node.phase_samples.get(phase,[])
            if len(rows)<5:checks[phase]=False;continue
            delta=(rows[-1][9]-rows[0][9]) if phase in ['forward','backward'] else math.atan2(math.sin(rows[-1][11]-rows[0][11]),math.cos(rows[-1][11]-rows[0][11]))
            checks[phase]=delta>.04 if phase=='forward' else delta<-.04 if phase=='backward' else delta>.2 if phase=='turn_left' else delta<-.2
        for phase in ['stop_forward','stop_backward','stop_left','stop_right','signal_loss']:
            rows=node.phase_samples.get(phase,[]);tail=rows[-5:]
            checks[phase]=len(rows)>=5 and all(abs(r[5])<.001 and abs(r[6])<.001 and abs(r[7])<.02 and abs(r[8])<.05 for r in tail)
        checks['guard_timeout_wall_time']=node.timeout_delay is not None and .25<=node.timeout_delay<=.45
        for phase in ['forward','backward','turn_left','turn_right']:
            delta=[]
            for name in ['left_wheel_joint','right_wheel_joint']:
                rows=[r for r in node.joint_rows if r[1]==phase and r[3]==name]
                delta.append(rows[-1][4]-rows[0][4] if len(rows)>=5 else float('nan'))
            left,right=delta
            checks['wheel_rotation_'+phase]=(left>.5 and right>.5) if phase=='forward' else (left<-.5 and right<-.5) if phase=='backward' else (left<-.2 and right>.2) if phase=='turn_left' else (left>.2 and right<-.2)
    if args.world_truth:
        checks['world_pose_received']=len(node.world_rows)>=5
        if args.motion:
            for phase in ['forward','backward','turn_left','turn_right']:
                rows=[r for r in node.world_rows if r[1]==phase]
                if len(rows)<5:checks['world_'+phase]=False;continue
                a,b=rows[0],rows[-1]
                displacement=(b[2]-a[2])*math.cos(a[5])+(b[3]-a[3])*math.sin(a[5])
                turn=math.atan2(math.sin(b[5]-a[5]),math.cos(b[5]-a[5]))
                checks['world_'+phase]=displacement>.04 if phase=='forward' else displacement<-.04 if phase=='backward' else turn>.2 if phase=='turn_left' else turn<-.2
            for phase in ['stop_forward','stop_backward','stop_left','stop_right','signal_loss']:
                rows=[r for r in node.world_rows if r[1]==phase]
                tail=[r for r in rows if rows and r[0]>=rows[-1][0]-.25]
                checks['world_'+phase]=len(tail)>=3 and math.hypot(tail[-1][2]-tail[0][2],tail[-1][3]-tail[0][3])<.005 and abs(math.atan2(math.sin(tail[-1][5]-tail[0][5]),math.cos(tail[-1][5]-tail[0][5])))<.02
    return checks


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--motion',action='store_true');parser.add_argument('--world-truth',action='store_true',help='Require Gazebo scene poses bridged to /sim/world_pose');parser.add_argument('--lidar',action='store_true');parser.add_argument('--camera',choices=['preview','1080p']);args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='86' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise SystemExit('请先 source tools/sim_env.bash；验收仅针对 localhost 的 domain86 仿真。')
    args.output.mkdir(parents=True,exist_ok=False)
    rclpy.init();node=Recorder(args.output);error=None;checks={};motion_started=False
    try:
        deadline=time.monotonic()+30
        while len(node.odom)<5 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
        if len(node.odom)<5:raise RuntimeError('30秒内未收到 /sim/odom；未执行动作。请确认 Gazebo 与桥接已启动。')
        if args.motion:
            motion_started=True
            for seconds,phase,command in [(2,'settle',(0.0,0.0)),(3,'forward',(.10,0.0)),(1,'stop_forward',(0.0,0.0)),(3,'backward',(-.10,0.0)),(1,'stop_backward',(0.0,0.0)),(2,'turn_left',(0.0,.5)),(1,'stop_left',(0.0,0.0)),(2,'turn_right',(0.0,-.5)),(1,'stop_right',(0.0,0.0)),(1,'before_loss',(.10,0.0)),(2,'signal_loss',None)]:node.spin_for(seconds,phase,command)
        else:node.spin_for(8,'observe')
        checks=evaluate(node,args)
    except (Exception,KeyboardInterrupt) as exc:error=str(exc) or type(exc).__name__
    finally:
        if motion_started and rclpy.ok():
            try:node.spin_for(.3,'final_stop',(0.0,0.0))
            except Exception:pass
        node.csv_file.close()
        for f,w in node.raw_files.values():f.close()
        report={'result':'PASS' if checks and all(checks.values()) and not error else 'FAIL','scope':'ACTUAL_GAZEBO_ROS_MESSAGES; NOT_PHYSICAL_HARDWARE','motion_requested':args.motion,'world_truth_requested':args.world_truth,'guard_timeout_observed_wall_s':node.timeout_delay,'guard_timeout_configured_s':.3,'guard_timeout_measurement_limit_s':.45,'motion_started':motion_started,'checks':checks,'counts':node.counts,'joints':sorted(node.joints),'scan':node.scan,'image':node.image,'camera_info':node.info,'error':error,'mcu_firmware_PI_algorithm':'UNCHANGED'}
        try:plot_rows(node.odom,args.output)
        except Exception as exc:report['plot_error']=str(exc);report['result']='FAIL'
        (args.output/'result.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(json.dumps(report,indent=2));node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    raise SystemExit(0 if report['result']=='PASS' else 1)


if __name__=='__main__':main()
