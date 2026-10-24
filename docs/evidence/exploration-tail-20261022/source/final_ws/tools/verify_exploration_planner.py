#!/usr/bin/env python3
"""隔离启动真实地图服务器和Nav2规划器；只查询历史地图路径，不启动仿真或运动。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid
import numpy as np
import yaml
from verify_navigation import stop_group


def main():
    """验证异步规划、真实代价图及候选间路径记录，所有进程只清理自己的进程组。"""
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--run',type=Path,required=True);parser.add_argument('--domain-id',type=int,default=153)
    args=parser.parse_args();args.output=args.output.resolve();run=args.run.resolve();ws=Path(__file__).resolve().parents[1]
    if args.domain_id==86 or not 0<=args.domain_id<=232:parser.error('必须使用非用户域的独立ROS域')
    args.output.mkdir(parents=True,exist_ok=False);partition='smartcar-path-probe-'+uuid.uuid4().hex
    frozen=args.output/'source';frozen.mkdir()
    for name in ['frontier_core.py','exploration_memory.py','route_core.py','route_planner.py']:
        source=ws/'src/smartcar_slam/scripts'/name;(frozen/name).write_bytes(source.read_bytes())
    (frozen/Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id),ROS_LOCALHOST_ONLY='1',IGN_PARTITION=partition,GZ_PARTITION=partition,
                      ROS_LOG_DIR=str(args.output/'ros_logs'))
    sys.path.insert(0,str(ws/'src/smartcar_slam/scripts'))
    import rclpy
    from rclpy.node import Node
    from route_planner import RoutePlanner
    from frontier_core import select_frontiers
    class Probe(Node):
        def __init__(self):
            """只提供规划记录接口，不创建速度或导航目标发布者。"""
            super().__init__('exploration_path_probe');self.folder=args.output;self.wall_start=time.monotonic()
        def sim(self):
            """该离线规划诊断使用系统时钟，不伪造Gazebo时钟。"""
            return self.get_clock().now().nanoseconds/1.e9
        def event(self,name,**values):
            """记录全部规划事件和错误，完整路径另存JSONL。"""
            with (self.folder/'events.jsonl').open('a') as stream:
                stream.write(json.dumps({'wall_s':time.monotonic()-self.wall_start,'event':name,**values})+'\n')
    params=yaml.safe_load((ws/'src/smartcar_nav2/config/nav2.yaml').read_text())
    def wall_clock(value):
        """只在本次诊断参数副本中使用墙钟，生产参数文件保持不变。"""
        if isinstance(value,dict):
            for key,item in value.items():
                if key=='use_sim_time':value[key]=False
                else:wall_clock(item)
    wall_clock(params);param_file=args.output/'params.yaml';param_file.write_text(yaml.safe_dump(params))
    d=np.load(run/'mission/map_final.npz');grid=d['occupancy'];res=float(d['resolution']);origin=d['origin']
    events=[json.loads(x) for x in (run/'mission/events.jsonl').read_text().splitlines()]
    robot=next(e['robot'] for e in events[::-1] if e['event']=='frontier_search')
    candidates,details=select_frontiers(grid,res,origin,robot)
    commands=[['ros2','run','nav2_map_server','map_server','--ros-args','--params-file',str(param_file),
               '-p','yaml_filename:='+str(run/'mission/map.yaml')],
              ['ros2','run','smartcar_nav2','safe_planner_server','--ros-args','--params-file',str(param_file)],
              ['ros2','run','tf2_ros','static_transform_publisher','--x',str(robot[0]),'--y',str(robot[1]),
               '--frame-id','map','--child-frame-id','base_footprint'],
              ['ros2','run','nav2_lifecycle_manager','lifecycle_manager','--ros-args','-p','autostart:=true',
               '-p','use_sim_time:=false','-p','node_names:=[map_server,planner_server]']]
    report={'scope':'STATIC_ARCHIVED_MAP_NAV2_PLANNING_ONLY','result':'FAIL','source_run':str(run),
            'domain_id':args.domain_id,'partition':partition,'commands':commands,'search':details,
            'source_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in frozen.iterdir()},
            'input_map_sha256':hashlib.sha256((run/'mission/map_final.npz').read_bytes()).hexdigest()}
    processes=[];streams=[];rclpy.init();node=Probe();router=RoutePlanner(node)
    try:
        for index,command in enumerate(commands):
            stream=(args.output/f'process_{index}.log').open('w');streams.append(stream)
            processes.append(subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True))
        started=time.monotonic();begun=False
        while time.monotonic()-started<70.:
            rclpy.spin_once(node,timeout_sec=.05);router.tick(time.monotonic())
            if time.monotonic()-started>8. and not begun:begun=router.begin(candidates,robot,0.,grid,res,origin,1)
            if begun and router.done:break
        report.update(begun=begun,done=router.done,reason=router.reason,target=router.target)
        report['result']='PASS' if router.done and router.target is not None and (args.output/'nav2_paths.jsonl').is_file() else 'FAIL'
    except Exception as exc:report['error']=str(exc)
    finally:
        router.cancel();report['shutdown']=[]
        for process in reversed(processes):
            report['shutdown'].append(stop_group(process,lambda name,**kw:node.event(name,**kw),
                                                 pump=lambda:rclpy.spin_once(node,timeout_sec=.01)))
        for stream in streams:stream.close()
        node.destroy_node();rclpy.shutdown();(args.output/'result.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:report.get(k) for k in ['result','reason','error']}))
    return 0 if report['result']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
