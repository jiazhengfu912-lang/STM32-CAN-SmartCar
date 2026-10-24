#!/usr/bin/env python3
"""隔离运行自主建图或静止基线；只监测，不发速度或探索导航目标。"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid
from verify_navigation import stop_group


def main():
    """加载仓库环境并隔离ROS与Gazebo，拒绝覆盖已有证据。"""
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--case',choices=['baseline','autonomous','watchdog','complete'],default='autonomous')
    p.add_argument('--world',default='office_loop');p.add_argument('--domain-id',type=int,default=145)
    p.add_argument('--timeout',type=float,default=1200.);p.add_argument('--_ready',action='store_true')
    a=p.parse_args();a.output=a.output.resolve();ws=Path(__file__).resolve().parents[1]
    if a.domain_id==86 or not 0<=a.domain_id<=232:p.error('使用非86的隔离ROS域')
    if not a._ready:
        if a.output.exists():raise FileExistsError(a.output)
        env=os.environ.copy();env.update(ROS_DOMAIN_ID=str(a.domain_id),ROS_LOCALHOST_ONLY='1')
        os.execvpe('bash',['bash','-c','set -e; unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH; source "$1"; exec /usr/bin/python3 "$2" "${@:3}" --_ready',
            'exploration-verifier',str(ws/'tools/env.bash'),str(Path(__file__).resolve()),*sys.argv[1:]],env)
    verify(a,ws)


def verify(a,ws):
    """记录地图增长、真实位移、任务结果、故障停止与全部退出日志。"""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    from nav_msgs.msg import OccupancyGrid,Odometry
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from std_srvs.srv import SetBool,Trigger
    from rcl_interfaces.srv import GetParameters
    from rclpy.signals import SignalHandlerOptions
    a.output.mkdir(parents=True,exist_ok=False)
    partition='smartcar-explore-'+uuid.uuid4().hex
    os.environ.update(ROS_DOMAIN_ID=str(a.domain_id),ROS_LOCALHOST_ONLY='1',IGN_PARTITION=partition,
        GZ_PARTITION=partition,ROS_LOG_DIR=str(a.output/'ros_logs'),RCUTILS_COLORIZED_OUTPUT='0')
    started=time.monotonic();events=(a.output/'verifier_events.jsonl').open('w',buffering=1)
    process=None;log=(a.output/'launch.log').open('w',buffering=1);streams={};latest={};maps=[];poses=[];commands=[];states=[]
    report={'result':'FAIL','case':a.case,'scope':'SIMULATION_ONLY','domain_id':a.domain_id,'partition':partition,'checks':{}}
    frozen=a.output/'source';frozen.mkdir()
    sources=[ws/'src/smartcar_slam/scripts'/n for n in ['explore.py','frontier_core.py']]
    sources +=[ws/'src/smartcar_nav2/launch'/n for n in ['navigation.launch.py','exploration.launch.py']]
    sources +=[ws/'src/smartcar_nav2/scripts'/n for n in ['navigation_gate.py','gate_core.py']]
    sources +=[ws/'src/smartcar_nav2/config/nav2.yaml',ws/'src/smartcar_slam/config/slam.yaml',Path(__file__)]
    report['source_sha256']={str(f.relative_to(ws.parent)):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources}
    for source in sources:(frozen/source.name).write_bytes(source.read_bytes())
    def event(name,**values):
        """保存故障注入、控制服务与清理步骤的实际时间。"""
        item={'wall_s':time.monotonic()-started,'event':name,**values};events.write(json.dumps(item)+'\n');print(json.dumps(item),flush=True)
    def row(name,columns,values):
        """观察器只读回调写CSV，不向车辆发送命令。"""
        if name not in streams:
            stream=(a.output/(name+'.csv')).open('w',newline='');writer=csv.writer(stream);writer.writerow(['wall_s',*columns]);streams[name]=(stream,writer)
        stream,writer=streams[name];writer.writerow([time.monotonic()-started,*values]);stream.flush()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Node('exploration_verifier')
    def on_map(msg):
        """保存占据数量，增长由实时地图而非预载地图证明。"""
        import numpy as np
        data=np.asarray(msg.data);sample={'wall_s':time.monotonic()-started,'known':int((data>=0).sum()),'free':int(((data>=0)&(data<=20)).sum()),
            'occupied':int((data>=65).sum()),'resolution':msg.info.resolution,'width':msg.info.width,'height':msg.info.height}
        maps.append(sample);row('maps',list(sample)[1:],list(sample.values())[1:]);latest['map']=msg
    def on_odom(msg):
        """保存完整移动和停止样本，不用动作成功代替真实运动。"""
        v=msg.twist.twist;p=msg.pose.pose.position;sample=(time.monotonic(),p.x,p.y,v.linear.x,v.angular.z)
        poses.append(sample);row('odom',['x','y','vx','wz'],list(sample[1:]));latest['odom']=sample
    def on_safe(msg):
        """记录保护链最终输出。"""
        sample=(time.monotonic(),msg.linear.x,msg.angular.z);commands.append(sample);row('commands',['vx','wz'],list(sample[1:]))
    def on_state(msg):
        """保存探索状态而不参与前沿选择。"""
        latest['explorer']=json.loads(msg.data)
    def on_gate(msg):
        """保存真实停车锁存和门控故障。"""
        value=json.loads(msg.data);states.append((time.monotonic(),value));latest['gate']=value
        row('gate',['state','vx','wz','stop_latched'],[value[k] for k in ['state','vx','wz','stop_latched']])
    node.create_subscription(OccupancyGrid,'/map',on_map,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
    node.create_subscription(Odometry,'/sim/odom',on_odom,10);node.create_subscription(Twist,'/nav/safe_cmd_vel',on_safe,10)
    node.create_subscription(String,'/exploration/status',on_state,10);node.create_subscription(String,'/navigation/safety_status',on_gate,10)
    def spin():
        """有界处理订阅，并检测被测启动组异常退出。"""
        rclpy.spin_once(node,timeout_sec=.05)
        if process and process.poll() is not None:raise RuntimeError('启动进程提前退出')
    def until(predicate,timeout):
        """保持记录的有界等待，超时不删失败。"""
        deadline=time.monotonic()+timeout
        while not predicate() and time.monotonic()<deadline:spin()
        if not predicate():raise TimeoutError('验证条件超时')
    def call(service,typ,data=None):
        """故障用例只调用显式任务暂停、恢复和结束服务，不发送导航目标。"""
        client=node.create_client(typ,service)
        until(client.service_is_ready,10.)
        request=typ.Request()
        if data is not None:request.data=data
        future=client.call_async(request);until(future.done,10.);response=future.result()
        if not response.success:raise RuntimeError(response.message)
        event('service_response',service=service,data=data,message=response.message)
    def stopped():
        """连续1秒且至少10个低速新鲜样本，指令也必须新鲜为零。"""
        now=time.monotonic();samples=[x for x in poses if now-x[0]<=1.5]
        return bool(len(samples)>=10 and now-samples[-1][0]<.5 and samples[-1][0]-samples[0][0]>=1. and
            all(abs(x[3])<=.02 and abs(x[4])<=.03 for x in samples) and commands and now-commands[-1][0]<.3 and
            max(abs(commands[-1][1]),abs(commands[-1][2]))<1.e-3)
    try:
        if a.case=='baseline':
            command=['ros2','launch','smartcar_slam','mapping.launch.py','world:='+a.world,'headless:=true','rviz:=false']
        else:
            command=['ros2','launch','smartcar_nav2','exploration.launch.py','world:='+a.world,'headless:=true','rviz:=false',
                'output_dir:='+str(a.output/'mission'),'max_sim_seconds:=600.0']
        report['launch_command']=command;process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        event('launch_started',pid=process.pid);until(lambda:bool(maps),120.)
        if a.case=='baseline':
            first=time.monotonic();until(lambda:time.monotonic()-first>=20.,25.)
            travel=sum(math.hypot(q[1]-p[1],q[2]-p[2]) for p,q in zip(poses,poses[1:]))
            report['checks']={'map_received':bool(maps),'no_manual_movement':travel<.02,
                'no_exploration_node':'frontier_explorer' not in node.get_node_names()}
        else:
            until(lambda:bool(latest.get('explorer')),120.)
            if a.case=='watchdog':
                until(lambda:len(poses)>5 and abs(poses[-1][3])>.06,180.)
                explorer_pid=None
                for entry in Path('/proc').iterdir():
                    if not entry.name.isdigit():continue
                    try:
                        args=(entry/'cmdline').read_bytes().split(b'\0')
                        if os.getpgid(int(entry.name))==process.pid and any(arg.endswith(b'/explore.py') for arg in args):explorer_pid=int(entry.name);break
                    except (OSError,ValueError):pass
                if explorer_pid is None:raise RuntimeError('找不到本测试探索器进程')
                count=len(json.loads((a.output/'mission/session.json').read_text())['goals']);os.kill(explorer_pid,signal.SIGSTOP);event('explorer_frozen',pid=explorer_pid)
                try:
                    until(lambda:latest.get('gate',{}).get('state')=='EXPLORER_UNAVAILABLE' and stopped(),12.)
                    report['checks']['watchdog_stops_and_latches']=bool(latest['gate']['stop_latched'])
                finally:os.kill(explorer_pid,signal.SIGCONT);event('explorer_resumed',pid=explorer_pid)
                until(lambda:latest['explorer']['state']=='PAUSED',10.)
                hold=time.monotonic();until(lambda:time.monotonic()-hold>=4.,6.)
                report['checks']['heartbeat_does_not_auto_resume']=latest['explorer']['goals']==count and stopped()
                call('/exploration/pause',SetBool,False)
                until(lambda:latest['explorer']['goals']>count,40.)
                report['checks']['explicit_resume_selects_new_goal']=True
                call('/exploration/pause',SetBool,True);until(stopped,15.)
                report['checks']['explicit_pause_stops']=True
                call('/exploration/finish',Trigger)
            until(lambda:latest.get('explorer',{}).get('state') in ['DONE','ERROR'],a.timeout)
            session=json.loads((a.output/'mission/session.json').read_text());report['session']=session
            effective={}
            for name,names in [('controller_server',['FollowPath.desired_linear_vel','FollowPath.rotate_to_heading_angular_vel']),
                               ('velocity_smoother',['max_velocity','max_accel'])]:
                client=node.create_client(GetParameters,'/'+name+'/get_parameters');until(client.service_is_ready,5.)
                request=GetParameters.Request();request.names=names;future=client.call_async(request);until(future.done,5.)
                effective[name]={key:(list(value.double_array_value) if value.type==8 else value.double_value)
                                 for key,value in zip(names,future.result().values)}
            report['effective_parameters']=effective
            report['checks'].update({'finished_and_saved':session['state']=='DONE','stop_verified':session['stop_verified'],
                'no_prior_map':not session['prior_map_loaded'],'one_slam_map_source':[x.node_name for x in node.get_publishers_info_by_topic('/map')]==['slam_toolbox'],
                'no_amcl_or_map_server':not {'amcl','map_server'}.intersection(node.get_node_names()),
                'mission_state_recorded':bool(session['state'])})
            if a.case!='complete':report['checks']['autonomous_goals']=len(session['goals'])>=1
            else:report['checks']['no_remaining_frontiers']=session['complete'] and session['reason']=='NO_FRONTIERS'
            if a.case=='autonomous':
                report['checks']['map_grew']=maps[-1]['known']>maps[0]['known']+100
                report['checks']['multiple_observations']=len(maps)>=5
        travel=sum(math.hypot(q[1]-p[1],q[2]-p[2]) for p,q in zip(poses,poses[1:]));report['travel_m']=travel
        report['map_initial']=maps[0];report['map_final']=maps[-1]
        if a.case in ['autonomous','watchdog']:report['checks']['actual_motion']=travel>(.01 if a.case=='watchdog' else 1.)
        report['result']='PASS' if all(report['checks'].values()) else 'FAIL'
    except (Exception,KeyboardInterrupt) as exc:
        report['error']=str(exc) or type(exc).__name__;event('failure',error=report['error'])
    finally:
        if process is not None:
            try:report['shutdown']=stop_group(process,event,pump=lambda:rclpy.spin_once(node,timeout_sec=.01))
            except Exception as exc:report['shutdown_error']=str(exc)
        report['duration_wall_s']=time.monotonic()-started
        for stream,_ in streams.values():stream.close()
        log.close();events.close();node.destroy_node();rclpy.try_shutdown()
        report['raw_sha256']={str(p.relative_to(a.output)):hashlib.sha256(p.read_bytes()).hexdigest() for p in a.output.rglob('*') if p.is_file() and 'source' not in p.parts}
        (a.output/'result.json').write_text(json.dumps(report,indent=2,ensure_ascii=False));print(json.dumps(report,ensure_ascii=False),flush=True)
    raise SystemExit(0 if report['result']=='PASS' else 1)


if __name__=='__main__':main()
