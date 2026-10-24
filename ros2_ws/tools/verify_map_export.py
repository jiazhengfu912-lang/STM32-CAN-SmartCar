#!/usr/bin/env python3
"""只读验证三值PGM/YAML经真实Nav2地图服务器重载后保持自由/障碍/未知，不启动机器人。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    """隔离地图话题/生命周期服务，拒绝覆盖记录或进入用户默认域86。"""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-file',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--domain-id',type=int,default=184)
    parser.add_argument('--_ready',action='store_true',help=argparse.SUPPRESS)
    args=parser.parse_args();args.map_file=args.map_file.expanduser().resolve();args.output=args.output.expanduser().resolve()
    if args.domain_id==86 or not 0<=args.domain_id<=232:parser.error('使用非86的隔离ROS域')
    if not args.map_file.is_file():parser.error('地图YAML不存在')
    if args.output.exists():parser.error('记录目录已存在，不覆盖旧记录')
    if not args._ready:
        env=os.environ.copy();env.update(ROS_DOMAIN_ID=str(args.domain_id),ROS_LOCALHOST_ONLY='1')
        setup=Path(__file__).resolve().parent/'env.bash'
        os.execvpe('bash',['bash','-c','set -e; unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH; source "$1"; exec /usr/bin/python3 "$2" "${@:3}" --_ready',
            'map-export-verifier',str(setup),str(Path(__file__).resolve()),*sys.argv[1:]],env)
    verify(args)


def verify(args):
    """比较全部重载栅格而非只看服务成功；完整保留阈值不匹配和缺测。"""
    import numpy as np
    import yaml
    from PIL import Image
    import rclpy
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    args.output.mkdir(parents=True,exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id),ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(args.output/'ros_logs'))
    report={'result':'FAIL','scope':'READONLY_TRINARY_MAP_RELOAD_NO_ROBOT','map_file':str(args.map_file),'domain_id':args.domain_id}
    process=None;log=(args.output/'server.log').open('w');received=[]
    rclpy.init();node=rclpy.create_node('map_export_verifier_'+str(os.getpid()))
    name='map_export_server_'+str(os.getpid());topic='/verification/export_map_'+str(os.getpid())
    node.create_subscription(OccupancyGrid,topic,received.append,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))

    def until(predicate,seconds=15.):
        """有界处理本测试节点的消息，超时明确失败，不补造地图。"""
        end=time.monotonic()+seconds
        while not predicate() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        if not predicate():raise TimeoutError('地图服务器响应超时')

    try:
        metadata=yaml.safe_load(args.map_file.read_text())
        if metadata.get('mode','trinary')!='trinary' or metadata.get('negate',0)!=0:raise ValueError('此检查器用于本项目negate=0三值导出')
        image_path=args.map_file.parent/metadata['image'];image=np.asarray(Image.open(image_path).convert('L'))
        if not set(np.unique(image)).issubset({0,205,254}):raise ValueError('不是本项目0/205/254三值图像')
        expected=np.flipud(np.where(image==0,100,np.where(image==254,0,-1))).astype(np.int8)
        report['input_sha256']={'yaml':hashlib.sha256(args.map_file.read_bytes()).hexdigest(),'image':hashlib.sha256(image_path.read_bytes()).hexdigest()}
        process=subprocess.Popen(['ros2','run','nav2_map_server','map_server','--ros-args','-r','__node:='+name,
            '-r','map:='+topic,'-p','yaml_filename:='+str(args.map_file)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        client=node.create_client(ChangeState,'/'+name+'/change_state');until(client.service_is_ready)
        for transition in [Transition.TRANSITION_CONFIGURE,Transition.TRANSITION_ACTIVATE]:
            request=ChangeState.Request();request.transition.id=transition;future=client.call_async(request);until(future.done)
            if not future.result().success:raise RuntimeError('地图生命周期激活失败')
        until(lambda:bool(received));msg=received[-1];actual=np.asarray(msg.data,dtype=np.int8).reshape(msg.info.height,msg.info.width)
        checks={'all_cells_equal':actual.shape==expected.shape and np.array_equal(actual,expected),
                'resolution':abs(msg.info.resolution-float(metadata['resolution']))<1.e-6,
                'origin_xy':abs(msg.info.origin.position.x-metadata['origin'][0])<1.e-6 and abs(msg.info.origin.position.y-metadata['origin'][1])<1.e-6}
        report.update(checks=checks,unknown_expected=int((expected<0).sum()),unknown_reloaded=int((actual<0).sum()),
                      expected_grid_sha256=hashlib.sha256(expected.tobytes()).hexdigest(),received_grid_sha256=hashlib.sha256(actual.tobytes()).hexdigest())
        report['result']='PASS' if all(checks.values()) else 'FAIL'
    except (Exception,KeyboardInterrupt) as exc:report['error']=str(exc) or type(exc).__name__
    finally:
        if process:
            try:os.killpg(process.pid,signal.SIGINT)
            except ProcessLookupError:pass
            try:process.wait(timeout=10.)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
            report['server_returncode']=process.returncode
        log.close();node.destroy_node();rclpy.try_shutdown()
        (args.output/'result.json').write_text(json.dumps(report,indent=2,ensure_ascii=False));print(json.dumps(report,indent=2,ensure_ascii=False))
    raise SystemExit(0 if report['result']=='PASS' else 1)


if __name__=='__main__':main()
