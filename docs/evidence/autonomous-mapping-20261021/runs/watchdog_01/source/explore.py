#!/usr/bin/env python3
"""基于实时SLAM前沿自动发导航目标；暂停/故障停车，结束后保存地图和图结构。"""
from collections import deque
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import time
import numpy as np
import yaml
from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String, Empty
from std_srvs.srv import SetBool, Trigger
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from rclpy.clock import Clock as RclpyClock
from rclpy.clock import ClockType
from frontier_core import select_frontiers


class Explorer(Node):
    def __init__(self):
        """创建独立会话文件；目录存在或服务路径非法时在发目标前拒绝启动。"""
        super().__init__('frontier_explorer')
        self.folder=Path(self.declare_parameter('output_dir','').value).expanduser().resolve()
        if not re.fullmatch(r'[A-Za-z0-9_./-]+',str(self.folder)) or not self.get_parameter('output_dir').value:
            raise ValueError('output_dir须为非空ASCII路径，不含空格')
        self.folder.mkdir(parents=True,exist_ok=False)
        self.max_sim=float(self.declare_parameter('max_sim_seconds',900.).value)
        if not math.isfinite(self.max_sim) or self.max_sim<=0:raise ValueError('max_sim_seconds须为有限正数')
        self.wall_start=time.monotonic();self.sim_start=None;self.map_msg=None;self.map_wall=None
        self.status='WAITING';self.reason=None;self.gate={};self.gate_wall=None
        self.goal=None;self.sending=None;self.result=None;self.cancel_future=None
        self.goal_wall=None;self.goal_sim=None;self.goals=[];self.excluded=[];self.failures=0
        self.next_sim=0.;self.empty_since=None;self.empty_maps=0;self.map_version=0;self.empty_version=-1;self.empty_reason=None
        self.odom_window=deque();self.safe=(0.,0.);self.safe_wall=None
        self.stop_sent=False;self.stop_deadline=None;self.save_future=None;self.save_phase=None;self.save_deadline=None
        self.stop_verified=False;self.last_status=0.;self.streams={};self.last_plan=0.
        self.resume_deadline=None
        self.lifecycle={name:self.create_client(GetState,'/'+name+'/get_state') for name in
                        ['bt_navigator','controller_server','planner_server','velocity_smoother']}
        self.lifecycle_states={};self.lifecycle_futures={};self.lifecycle_poll=0.
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.navigator=ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.stop_client=self.create_client(SetBool,'/navigation/stop')
        self.save_client=self.create_client(SaveMap,'/slam_toolbox/save_map')
        self.graph_client=self.create_client(SerializePoseGraph,'/slam_toolbox/serialize_map')
        self.publisher=self.create_publisher(String,'/exploration/status',10)
        self.heartbeat_pub=self.create_publisher(Empty,'/exploration/heartbeat',10)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,QoSProfile(depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Odometry,'/sim/odom',self.on_odom,10)
        self.create_subscription(Twist,'/nav/safe_cmd_vel',self.on_safe,10)
        self.create_subscription(String,'/navigation/safety_status',self.on_gate,10)
        self.create_service(SetBool,'/exploration/pause',self.pause)
        self.create_service(Trigger,'/exploration/finish',self.finish_service)
        # 任务存活和墙钟预算不能随Gazebo暂停或低实时系数一起停摆。
        self.create_timer(.2,self.tick,clock=RclpyClock(clock_type=ClockType.STEADY_TIME))
        self.event('session_started',scope='SIMULATION_ONLY',prior_map_loaded=False,algorithm='frontier + NavFn A* + RPP + SLAM Toolbox')
        self.write_summary()

    def sim(self):
        """导航预算使用仿真时间，另设墙钟总上限应对暂停或性能不足。"""
        return self.get_clock().now().nanoseconds/1.e9

    def event(self,name,**values):
        """完整保存事件、动作响应与异常，不覆盖失败记录。"""
        row={'wall_s':time.monotonic()-self.wall_start,'sim_s':self.sim(),'event':name,**values}
        if 'events' not in self.streams:self.streams['events']=(self.folder/'events.jsonl').open('w',buffering=1)
        self.streams['events'].write(json.dumps(row,ensure_ascii=False)+'\n')
        self.get_logger().info(json.dumps(row,ensure_ascii=False))

    def row(self,name,columns,values):
        """原样保存实际采样CSV，未收到的数据不补点。"""
        if name not in self.streams:
            stream=(self.folder/(name+'.csv')).open('w',newline='');writer=csv.writer(stream);writer.writerow(['wall_s',*columns]);self.streams[name]=(stream,writer)
        stream,writer=self.streams[name];writer.writerow([time.monotonic()-self.wall_start,*values]);stream.flush()

    def map_snapshot(self,name):
        """保存地图原始占据值、分辨率和原点，供离线检查和逐点比较。"""
        msg=self.map_msg
        if msg is not None:
            np.savez_compressed(self.folder/(name+'.npz'),occupancy=np.asarray(msg.data,dtype=np.int8).reshape(msg.info.height,msg.info.width),
                resolution=msg.info.resolution,origin=[msg.info.origin.position.x,msg.info.origin.position.y])

    def on_map(self,msg):
        """只接受轴对齐的有效在线地图；地图扩大和原点变化按新消息重算。"""
        q=msg.info.origin.orientation
        if (msg.header.frame_id!='map' or msg.info.width*msg.info.height!=len(msg.data) or not msg.data or
            not math.isfinite(msg.info.resolution) or msg.info.resolution<=0 or
            not all(math.isfinite(v) for v in [msg.info.origin.position.x,msg.info.origin.position.y,q.x,q.y,q.z,q.w]) or
            abs(q.x)+abs(q.y)+abs(q.z)>1.e-6 or abs(abs(q.w)-1)>1.e-6):
            self.map_wall=None;return
        initial=self.map_msg is None;self.map_msg=msg;self.map_wall=time.monotonic();self.map_version+=1
        a=np.asarray(msg.data);self.row('map_updates',['sim_s','width','height','resolution','free','occupied','unknown'],
            [self.sim(),msg.info.width,msg.info.height,msg.info.resolution,int(((a>=0)&(a<=20)).sum()),int((a>=65).sum()),int((a<0).sum())])
        if initial:self.map_snapshot('map_initial');self.event('first_online_map',known_cells=int((a>=0).sum()))

    def on_odom(self,msg):
        """连续低速、新鲜里程计窗口用于保存前的实际停车证明。"""
        now=time.monotonic();v=msg.twist.twist;p=msg.pose.pose
        self.odom_window.append((now,v.linear.x,v.angular.z))
        while self.odom_window and now-self.odom_window[0][0]>3.:self.odom_window.popleft()
        self.row('odom',['sim_s','x','y','z','qw','qz','vx_mps','wz_rps'],
            [self.sim(),p.position.x,p.position.y,p.position.z,p.orientation.w,p.orientation.z,v.linear.x,v.angular.z])

    def on_safe(self,msg):
        """记录真实门控输出，不由探索器绕过门控直接驱动底盘。"""
        self.safe=(msg.linear.x,msg.angular.z);self.safe_wall=time.monotonic()
        self.row('safe_cmd_vel',['vx_mps','wz_rps'],list(self.safe))

    def on_gate(self,msg):
        """外部停车锁存会暂停任务，探索器不自行解除或续发目标。"""
        self.gate=json.loads(msg.data);self.gate_wall=time.monotonic()
        self.row('safety',['state','vx','wz','stop_latched'],[self.gate.get(k) for k in ['state','vx','wz','stop_latched']])
        if self.status=='RESUMING' and not self.gate.get('stop_latched'):
            self.status='WAITING';self.next_sim=self.sim()+3.;self.event('resume_confirmed')
        if self.gate.get('stop_latched') and self.reason is None and self.status not in ['PAUSED','RESUMING']:
            self.status='PAUSED';self.cancel();self.event('external_stop_paused')

    def latch(self,enabled):
        """只在用户显式恢复时解除锁存；自动结束和故障只请求锁存停车。"""
        if not self.stop_client.service_is_ready():return False
        req=SetBool.Request();req.data=enabled;self.stop_client.call_async(req);return True

    def cancel(self):
        """取消自己持有的目标；取消响应到达前不发送另一个目标。"""
        if self.goal is not None and self.cancel_future is None:self.cancel_future=self.goal.cancel_goal_async()

    def pause(self,request,response):
        """显式暂停保持停车，显式恢复才允许选择一个新目标。"""
        if self.reason is not None:
            response.success=False;response.message='任务正在结束或已保存；重新启动须使用新目录';return response
        if request.data:
            self.status='PAUSED';self.cancel();self.latch(True);self.event('user_paused')
        else:
            if not self.stop_client.service_is_ready():
                response.success=False;response.message='停车服务不可用，保持暂停';return response
            self.latch(False);self.status='RESUMING';self.resume_deadline=time.monotonic()+3.
            self.event('user_resumed_new_goal_required')
        response.success=True;response.message='已暂停并请求停车' if request.data else '已显式恢复，等待地图后选择新目标';return response

    def finish_service(self,request,response):
        """显式提前结束也先停车，再保存部分地图与终止原因。"""
        self.finish('USER_FINISHED_PARTIAL');response.success=True;response.message='先停车，随后保存地图；请等待/exploration/status为DONE';return response

    def finish(self,reason):
        """完成、受阻、预算耗尽和故障均有终止原因，不把部分地图说成完全建图。"""
        if self.reason is not None:return
        self.reason=reason;self.status='STOPPING';self.stop_deadline=time.monotonic()+30.;self.cancel()
        self.event('mission_stopping',reason=reason)

    def write_summary(self,error=None):
        """持续更新会话结论；完整探索仅指未发现满足阈值的可探索前沿。"""
        report={'state':self.status,'reason':self.reason,'complete':self.reason=='NO_FRONTIERS' and self.status=='DONE',
                'stop_verified':self.stop_verified,'scope':'SIMULATION_ONLY','prior_map_loaded':False,
                'goals':self.goals,'navigation_failures':self.failures,'duration_wall_s':time.monotonic()-self.wall_start,
                'firmware_source_version':131,'pi_changed':False,'pi_exercised':False,
                'not_verified':['physical vehicle','all unseen spaces','unreachable rooms','arbitrary environments']}
        if error:report['error']=error
        (self.folder/'session.json').write_text(json.dumps(report,indent=2,ensure_ascii=False))

    def stopped(self,now):
        """至少1秒、10个新鲜低速样本及新鲜零指令；缺测不能证明停车。"""
        if not self.gate.get('stop_latched') or not self.odom_window or now-self.odom_window[-1][0]>.5 or self.safe_wall is None or now-self.safe_wall>.3:return False
        if max(map(abs,self.safe))>1.e-3:return False
        low=[]
        for item in reversed(self.odom_window):
            if abs(item[1])>.02 or abs(item[2])>.03:break
            low.append(item)
        return len(low)>=10 and low[0][0]-low[-1][0]>=1.

    def navigation_ready(self,now):
        """动作服务已出现不代表生命周期已激活；全部关键节点active才发目标。"""
        for name,future in list(self.lifecycle_futures.items()):
            if future.done():
                response=future.result();self.lifecycle_states[name]=response.current_state.id if response else 0
                del self.lifecycle_futures[name]
        if now-self.lifecycle_poll>1.:
            self.lifecycle_poll=now
            for name,client in self.lifecycle.items():
                if not client.service_is_ready():self.lifecycle_states[name]=0
                elif name not in self.lifecycle_futures:self.lifecycle_futures[name]=client.call_async(GetState.Request())
        return all(self.lifecycle_states.get(name)==3 for name in self.lifecycle)

    def saving(self,now):
        """顺序调用真实SLAM保存和序列化服务，校验文件，超时保留部分结果。"""
        if not self.stop_sent:self.stop_sent=self.latch(True)
        if not self.stop_verified:
            if self.stopped(now):self.stop_verified=True;self.event('stop_verified')
            elif now>self.stop_deadline:raise TimeoutError('停车证据不足，不宣称任务成功')
            else:return
        if self.sending is not None or self.goal is not None:return
        prefix=str(self.folder/'map')
        if self.save_future is None:
            if not self.save_client.service_is_ready():
                if now>self.stop_deadline:raise TimeoutError('SLAM保存服务不可用')
                return
            req=SaveMap.Request();req.name.data=prefix;self.save_future=self.save_client.call_async(req)
            self.save_phase='image';self.save_deadline=now+90.;self.status='SAVING';return
        if not self.save_future.done():
            if now>self.save_deadline:raise TimeoutError('地图保存服务超时：'+self.save_phase)
            return
        response=self.save_future.result()
        if response is None or response.result!=0:raise RuntimeError('SLAM服务保存失败：'+str(response))
        if self.save_phase=='image':
            if not self.graph_client.service_is_ready():raise RuntimeError('SLAM图结构序列化服务不可用')
            req=SerializePoseGraph.Request();req.filename=prefix;self.save_future=self.graph_client.call_async(req)
            self.save_phase='graph';self.save_deadline=now+45.;return
        files={}
        for suffix in ['.yaml','.pgm','.posegraph','.data']:
            path=Path(prefix+suffix)
            if not path.is_file() or not path.stat().st_size:raise RuntimeError('地图文件缺失或为空：'+str(path))
            files[path.name]=hashlib.sha256(path.read_bytes()).hexdigest()
        metadata=yaml.safe_load(Path(prefix+'.yaml').read_text())
        with Image.open(self.folder/metadata['image']) as image:
            lo,hi=image.getextrema()
            if lo==hi or metadata['resolution']<=0:raise RuntimeError('保存地图无有效占据变化')
        self.map_snapshot('map_final');self.status='DONE';self.event('map_saved',sha256=files)
        self.write_summary();self.save_phase='finished'

    def tick(self):
        """一条任务状态机；已知自由区选目标，前沿稳定消失后停车保存。"""
        now=time.monotonic();sim=self.sim()
        try:
            if self.sending is not None and self.sending.done():
                handle=self.sending.result();self.sending=None
                if handle is None or not handle.accepted:
                    self.failures+=1;self.event('goal_rejected');self.next_sim=sim+3.
                else:
                    self.goal=handle;self.result=handle.get_result_async()
                    if self.status=='PAUSED' or self.reason is not None:self.cancel()
            if self.result is not None and self.result.done():
                status=self.result.result().status;self.goals[-1]['action_status']=status
                if status!=4 and self.status!='PAUSED' and self.reason is None:self.failures+=1
                self.event('goal_result',status=status)
                if status==5 and self.cancel_future is None and self.reason is None:
                    self.status='PAUSED';self.latch(True);self.event('external_cancel_paused')
                self.goal=None;self.result=None;self.cancel_future=None;self.next_sim=sim+3.;self.write_summary()
            if self.status in ['DONE','ERROR']:return
            if self.reason is not None:self.saving(now);return
            if self.status=='PAUSED':return
            if self.status=='RESUMING':
                if now>self.resume_deadline:self.status='PAUSED';self.latch(True);self.event('resume_not_confirmed')
                return
            if sim>0 and self.sim_start is None:self.sim_start=sim
            if now-self.wall_start>3600 or (self.sim_start is not None and sim-self.sim_start>self.max_sim):
                self.finish('TIME_LIMIT_PARTIAL');return
            if self.failures>=8:self.finish('NAVIGATION_FAILURE_LIMIT_PARTIAL');return
            if self.map_msg is None or self.gate_wall is None:
                if now-self.wall_start>120:self.finish('STARTUP_FAILED_PARTIAL')
                return
            if now-self.map_wall>10 or now-self.gate_wall>1.5:
                self.finish('INPUT_STALE_PARTIAL');return
            if self.goal is not None:
                if (sim-self.goal_sim>120 or now-self.goal_wall>600) and self.cancel_future is None:
                    self.cancel();self.failures+=1;self.event('goal_timeout_cancel')
                return
            if self.sending is not None or sim<self.next_sim or now-self.last_plan<1.:return
            if not self.navigation_ready(now):
                if not self.goals and now-self.wall_start>120:self.finish('STARTUP_FAILED_PARTIAL')
                return
            if self.gate.get('state') not in ['READY','COMMAND_STALE']:return
            if not self.navigator.server_is_ready():return
            if not self.buffer.can_transform('map','base_footprint',Time()):return
            transform=self.buffer.lookup_transform('map','base_footprint',Time())
            stamp=transform.header.stamp.sec+transform.header.stamp.nanosec/1.e9
            if not -1.<=sim-stamp<=1.5:raise RuntimeError('探索位姿TF过期')
            robot=(transform.transform.translation.x,transform.transform.translation.y)
            msg=self.map_msg;self.last_plan=now
            candidates,details=select_frontiers(np.asarray(msg.data).reshape(msg.info.height,msg.info.width),msg.info.resolution,
                (msg.info.origin.position.x,msg.info.origin.position.y),robot,self.excluded)
            self.event('frontier_search',robot=robot,**details)
            if not candidates:
                if self.empty_since is None or self.empty_reason!=details['reason']:
                    self.empty_since=sim;self.empty_maps=0;self.empty_version=-1;self.empty_reason=details['reason']
                if self.map_version!=self.empty_version:self.empty_maps+=1;self.empty_version=self.map_version
                if sim-self.empty_since>=6 and self.empty_maps>=3:
                    self.finish('NO_FRONTIERS' if details['reason']=='NO_FRONTIERS' else 'NO_REACHABLE_FRONTIERS_PARTIAL')
                return
            self.empty_since=None;self.empty_reason=None;target=candidates[0]
            request=NavigateToPose.Goal();request.pose.header.frame_id='map';request.pose.header.stamp=self.get_clock().now().to_msg()
            request.pose.pose.position.x=target['x'];request.pose.pose.position.y=target['y']
            request.pose.pose.orientation.z=math.sin(target['yaw']/2);request.pose.pose.orientation.w=math.cos(target['yaw']/2)
            self.goals.append({**target,'sent_sim_s':sim,'action_status':None})
            self.excluded.append((target['x'],target['y'],.65))
            self.goal_wall=now;self.goal_sim=sim;self.status='EXPLORING'
            self.sending=self.navigator.send_goal_async(request);self.event('goal_sent',target=target);self.write_summary()
        except Exception as exc:
            if self.reason is None:self.finish('ERROR_PARTIAL')
            else:self.status='ERROR'
            self.latch(True);self.event('error',error=str(exc));self.write_summary(str(exc))
        finally:
            self.heartbeat_pub.publish(Empty())
            if now-self.last_status>1.:
                msg=String();msg.data=json.dumps({'state':self.status,'reason':self.reason,'goals':len(self.goals),
                    'failures':self.failures,'stop_verified':self.stop_verified,'output_dir':str(self.folder)})
                self.publisher.publish(msg);self.last_status=now


def main():
    """退出停止本节点，不把中断记录伪装为完成；底层原超时门控保持。"""
    rclpy.init();node=Explorer()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        if node.status not in ['DONE','ERROR']:
            node.reason='INTERRUPTED_PARTIAL';node.status='INTERRUPTED';node.write_summary()
        for stream in node.streams.values():
            (stream[0] if isinstance(stream,tuple) else stream).close()
        node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
