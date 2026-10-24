#!/usr/bin/env python3
"""Save actual ROS sensor samples, timing and TF; no synthetic samples."""
import argparse
import csv
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan
from tf2_ros import Buffer, TransformListener
from PIL import Image as PillowImage


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    rclpy.init();node=Node('sensor_evidence',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
    buffer=Buffer();listener=TransformListener(buffer,node)
    samples={'scan':[],'image':[]};latest={}
    def receive(key,msg):
        samples[key].append((time.monotonic(),msg.header.stamp.sec+msg.header.stamp.nanosec/1e9))
        latest[key]=msg
    subscriptions=[node.create_subscription(LaserScan,'/sim/scan',lambda m:receive('scan',m),qos_profile_sensor_data),node.create_subscription(Image,'/sim/camera/image_raw',lambda m:receive('image',m),qos_profile_sensor_data)]
    deadline=time.monotonic()+12
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
    report={'scope':'ACTUAL_GAZEBO_ROS_SAMPLES_NOT_HARDWARE','timing':{},'transforms':{}}
    for key,rows in samples.items():
        report['timing'][key]={'count':len(rows)}
        if len(rows)>1:
            wall=rows[-1][0]-rows[0][0];sim=rows[-1][1]-rows[0][1]
            report['timing'][key].update(wall_hz=(len(rows)-1)/wall,simulation_hz=(len(rows)-1)/sim if sim>0 else None,simulation_to_wall_ratio=sim/wall)
    if 'scan' in latest:
        scan=latest['scan']
        with (args.output/'scan.csv').open('w',newline='') as f:
            writer=csv.writer(f);writer.writerow(['angle_rad','range_m'])
            writer.writerows((scan.angle_min+i*scan.angle_increment,r) for i,r in enumerate(scan.ranges))
    if 'image' in latest:
        msg=latest['image'];modes={'rgb8':('RGB','RGB'),'bgr8':('RGB','BGR'),'rgba8':('RGBA','RGBA'),'bgra8':('RGBA','BGRA'),'mono8':('L','L')}
        if msg.encoding in modes:
            mode,rawmode=modes[msg.encoding]
            im=PillowImage.frombytes(mode,(msg.width,msg.height),bytes(msg.data),'raw',rawmode,msg.step)
            im.save(args.output/'camera.png');report['image_extrema']=im.getextrema()
        else:report['image_error']='Unsupported encoding: '+msg.encoding
    for frame in ['laser_frame','camera_optical_frame']:
        try:
            t=buffer.lookup_transform('base_link',frame,Time())
            v=t.transform.translation;q=t.transform.rotation
            report['transforms'][frame]={'translation':[v.x,v.y,v.z],'quaternion':[q.x,q.y,q.z,q.w]}
        except Exception as exc:report['transforms'][frame]={'error':str(exc)}
    report['result']='PASS' if all(len(samples[k])>=5 for k in samples) and 'image_extrema' in report and all('error' not in v for v in report['transforms'].values()) else 'FAIL'
    (args.output/'sensor_evidence.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
    node.destroy_node();rclpy.shutdown()
    raise SystemExit(0 if report['result']=='PASS' else 1)


if __name__=='__main__':main()
