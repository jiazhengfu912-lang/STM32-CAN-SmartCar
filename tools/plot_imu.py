"""只读绘制一次真实IMU采集；不平滑、不插值，缺口断线，保留PNG/SVG。"""
import argparse
import json
from pathlib import Path
import math


def main():
    """从capture_imu输出绘图，元数据与实测版本不符时拒绝生成。"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    parser=argparse.ArgumentParser()
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    meta=json.loads((args.input/'metadata.json').read_text(encoding='utf-8'))
    result=json.loads((args.input/'imu.json').read_text(encoding='utf-8'))
    rows=result['samples']
    if not rows: raise ValueError('没有真实采样，不生成占位波形')
    if any(r.get('version')!=meta['version'] for r in rows): raise ValueError('版本混杂或与元数据不符，请保留原始输入并分段验收')
    args.output.mkdir(parents=True,exist_ok=False)
    time=[]; expanded=[]; previous=None; elapsed=0
    for row in rows:
        if previous is not None:
            delta=(row['uptime_ms']-previous)&0xffffffff
            if delta>0x80000000: raise ValueError('本轮发生设备复位，请按复位点分段，不拼接轨迹')
            if delta>50: time.append(float('nan'));expanded.append({})
            elapsed+=delta
        time.append(elapsed/1000);expanded.append(row);previous=row['uptime_ms']
    config=meta['nav_config']
    title=f"v{meta['version']} | wheel P=0.30/0.45 I=0.010 (startup I=0.030) | complementary tau=0.5s\nnav={config} | raw points; no plot smoothing"
    fig,axes=plt.subplots(5,1,figsize=(13,15),sharex=True)
    groups=[(('ax','ay','az'),'raw accel counts'),(('gx','gy','gz'),'raw gyro counts'),
            (('roll_rad','pitch_rad','yaw_rad','heading_rad'),'angle rad'),
            (('target_left','target_right','rpm_left','rpm_right'),'motor rpm'),
            (('state','fault','calibration','i2c_errors','tx_dropped'),'state / counters')]
    for ax,(fields,label) in zip(axes,groups):
        for field in fields:
            values=[r.get(field,math.nan) if r.get('valid',False) or label=='state / counters' else math.nan for r in expanded]
            ax.plot(time,values,'.-',markersize=2,linewidth=.7,label=field)
        ax.set_ylabel(label);ax.grid(True);ax.legend(loc='upper right',ncol=3)
    axes[-1].set_xlabel('MCU elapsed s');fig.suptitle(title,fontsize=9);fig.tight_layout(rect=(0,0,1,.94))
    for ext in ('png','svg'):fig.savefig(args.output/f'imu.{ext}',dpi=150)
    plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,8));last_origin=None;x=[];y=[]
    for r in expanded:
        if not r.get('position_valid') or r.get('origin_id')!=last_origin:
            x.append(math.nan);y.append(math.nan)
        if r.get('position_valid'):x.append(r['x_m']);y.append(r['y_m'])
        last_origin=r.get('origin_id')
    ax.plot(x,y,'.-',markersize=2,linewidth=.7);ax.set_aspect('equal',adjustable='datalim')
    ax.set_xlabel('X m');ax.set_ylabel('Y m');ax.grid(True);fig.suptitle(title,fontsize=8)
    for ext in ('png','svg'):fig.savefig(args.output/f'trajectory.{ext}',dpi=150)
    plt.close(fig)


if __name__=='__main__':main()
