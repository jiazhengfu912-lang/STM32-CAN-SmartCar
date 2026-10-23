#!/usr/bin/env python3
"""Plot recorded CSV rows without smoothing, resampling or invented data."""
import argparse
import csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser=argparse.ArgumentParser();parser.add_argument('folder',type=Path);args=parser.parse_args();p=args.folder
    def read(name):return list(csv.DictReader((p/(name+'.csv')).open()))
    joints,world,commands=read('joints'),read('world_pose'),read('commands')
    fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True)
    for name in ['left_wheel_joint','right_wheel_joint']:
        rows=[r for r in joints if r['name']==name]
        axes[0].plot([float(r['wall_s']) for r in rows],[float(r['position']) for r in rows],label=name)
    for source in ['input','guard']:
        rows=[r for r in commands if r['source']==source]
        axes[1].plot([float(r['wall_s']) for r in rows],[float(r['vx']) for r in rows],'.',ms=2,label=source+' vx')
        axes[2].plot([float(r['wall_s']) for r in rows],[float(r['wz']) for r in rows],'.',ms=2,label=source+' wz')
    for ax,label in zip(axes,['wheel angle (rad)','linear command (m/s)','angular command (rad/s)']):ax.set_ylabel(label);ax.grid();ax.legend()
    axes[-1].set_xlabel('wall time (s)');fig.suptitle('Actual ROS joint and command samples / Gazebo simulation, NOT hardware');fig.tight_layout()
    for ext in ['png','svg']:fig.savefig(p/('joints_commands.'+ext),dpi=160)
    plt.close(fig)
    if world:
        fig,axes=plt.subplots(2,1,figsize=(11,6),sharex=True)
        for column in ['x','y']:axes[0].plot([float(r['wall_s']) for r in world],[float(r[column]) for r in world],label='world '+column)
        axes[1].plot([float(r['wall_s']) for r in world],[float(r['yaw']) for r in world],label='world yaw')
        axes[0].set_ylabel('position (m)');axes[1].set_ylabel('yaw (rad)');axes[1].set_xlabel('wall time (s)')
        for ax in axes:ax.grid();ax.legend()
        fig.suptitle('Actual Gazebo scene poses / independent of wheel odometry, NOT hardware');fig.tight_layout()
        for ext in ['png','svg']:fig.savefig(p/('world_motion.'+ext),dpi=160)
        plt.close(fig)


if __name__=='__main__':main()
