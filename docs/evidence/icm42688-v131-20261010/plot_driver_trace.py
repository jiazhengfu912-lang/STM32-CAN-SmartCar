"""逐点展示真实驱动的HAL替身运行记录；不连接设备，不生成实测数据。"""
from pathlib import Path
import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parent
rows=list(csv.DictReader((ROOT/'driver-trace.csv').open(encoding='utf-8')))
title=('v131 ICM-42688-P | HOST INJECTION ONLY - no hardware\n'
       'P right reverse 0.45 -> 0.45, others 0.30 -> 0.30; '
       'I steady 0.010 -> 0.010, startup 0.030 -> 0.030 per 10ms\n'
       'Sensor driver/decoding changed; motor PI, startup, attitude fusion and navigation unchanged')


def line(ax, data, key, label):
    """直接画记录点，缺测超过20ms断线，不平滑或插值。"""
    x=[];y=[];previous=None
    for r in data:
        stamp=int(r['time_ms'])
        if previous is not None and stamp-previous>20:
            x.append(np.nan);y.append(np.nan)
        x.append(stamp);y.append(float(r[key]));previous=stamp
    ax.plot(x,y,label=label,linewidth=.9,marker='.',markersize=2)


def save(fig,name):
    """保存同一数据的PNG与SVG，全部轴保留原样点。"""
    fig.savefig(ROOT/f'{name}.png',dpi=160)
    fig.savefig(ROOT/f'{name}.svg')
    plt.close(fig)


data=[r for r in rows if r['scenario']=='sample-and-calibration']
fig,axes=plt.subplots(4,1,figsize=(13,10),sharex=True)
fig.suptitle(title,fontsize=10)
for ax,fields in zip(axes,[(('sequence','sample sequence'),),
                         (('ready','configuration ready'),('fresh','sample fresh'),('calibration','calibration 0/1/2/3')),
                         (('gz_rad_s','decoded gyro Z (rad/s)'),),
                         (('errors','I2C errors'),('restarts','driver restarts'),('gaps','sample gaps'))]):
    for key,label in fields:line(ax,data,key,label)
    ax.legend(loc='upper left',fontsize=8);ax.grid(alpha=.25)
axes[-1].set_xlabel('Virtual MCU time (ms), not physical measurement')
fig.tight_layout(rect=(0,0,1,.90));save(fig,'driver-trace')

cases=['wrong-identity','config-mismatch','reset-not-done','busy-and-timeout','invalid-sample','calibration-data-gap']
fig,axes=plt.subplots(3,2,figsize=(13,10))
fig.suptitle(title,fontsize=10)
for ax,case in zip(axes.flat,cases):
    data=[r for r in rows if r['scenario']==case]
    for key,label in [('ready','ready'),('errors','errors'),('timeouts','timeouts'),('restarts','restarts'),('fresh','fresh')]:
        line(ax,data,key,label)
    ax.set_title(case);ax.set_xlabel('Virtual time (ms)');ax.grid(alpha=.25);ax.legend(fontsize=7)
fig.tight_layout(rect=(0,0,1,.90));save(fig,'driver-failures')
