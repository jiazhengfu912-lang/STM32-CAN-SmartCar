#!/usr/bin/env python3
"""Exercise real keyboard node through a PTY and record received ROS commands."""
import argparse
import csv
import json
import os
from pathlib import Path
import pty
import signal
import subprocess
import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='86' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise SystemExit('Source sim_env.bash first')
    args.output.mkdir(parents=True,exist_ok=False);rclpy.init();node=Node('keyboard_runtime_verifier')
    start=time.monotonic();phase='startup';rows=[]
    def receive(source,msg):rows.append([time.monotonic()-start,phase,source,msg.linear.x,msg.angular.z])
    subscriptions=[node.create_subscription(Twist,'/sim/cmd_vel',lambda m:receive('input',m),10),node.create_subscription(Twist,'/sim/drive_cmd_vel',lambda m:receive('guard',m),10)]
    master,slave=pty.openpty();log=(args.output/'keyboard_process.log').open('w')
    keyboard=Path(__file__).resolve().parents[1]/'scripts/keyboard_drive.py'
    process=subprocess.Popen([sys.executable,str(keyboard)],stdin=slave,stdout=log,stderr=log,start_new_session=True);os.close(slave)
    checks={};error=None
    def spin(seconds,key=None):
        end=time.monotonic()+seconds;next_key=0
        while time.monotonic()<end:
            if key is not None and time.monotonic()>=next_key:os.write(master,key);next_key=time.monotonic()+.08
            rclpy.spin_once(node,timeout_sec=.01)
    def tail(source):return [r for r in rows if r[1]==phase and r[2]==source][-5:]
    try:
        spin(2)
        for name,key,v,w in [('forward',b'w',.15,0),('backward',b's',-.15,0),('left',b'a',0,.8),('right',b'd',0,-.8),('space',b' ',0,0)]:
            phase=name;spin(.8,key)
            checks[name]=all(len(tail(source))>=3 and all(abs(r[3]-v)<1e-6 and abs(r[4]-w)<1e-6 for r in tail(source)) for source in ['input','guard'])
        phase='release';spin(.5,b'w');release=time.monotonic();spin(.7)
        checks['release_stops']=all(len(tail(source))>=3 and all(abs(r[3])+abs(r[4])<1e-6 for r in tail(source)) for source in ['input','guard'])
        phase='before_kill';spin(.6,b'w');killed=time.monotonic();os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)
        phase='keyboard_killed';spin(.8)
        guard=tail('guard');checks['killed_keyboard_stops']=len(guard)>=3 and all(abs(r[3])+abs(r[4])<1e-6 for r in guard)
        zeros=[r for r in rows if r[1]=='keyboard_killed' and r[2]=='guard' and abs(r[3])+abs(r[4])<1e-6]
        delay=zeros[0][0]-(killed-start) if zeros else None
        checks['kill_timeout_wall_time']=delay is not None and 0<=delay<=.45
    except Exception as exc:error=str(exc);delay=None
    finally:
        if process.poll() is None:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
        os.close(master);log.close()
        with (args.output/'keyboard_commands.csv').open('w',newline='') as f:
            writer=csv.writer(f);writer.writerow(['wall_s','phase','source','vx','wz']);writer.writerows(rows)
        report={'result':'PASS' if checks and all(checks.values()) and error is None else 'FAIL','scope':'ACTUAL_PTY_KEYBOARD_NODE_AND_ROS_MESSAGES_NOT_HARDWARE','checks':checks,'kill_to_guard_zero_wall_s':delay,'error':error}
        (args.output/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));node.destroy_node();rclpy.shutdown()
    raise SystemExit(0 if report['result']=='PASS' else 1)


if __name__=='__main__':main()
