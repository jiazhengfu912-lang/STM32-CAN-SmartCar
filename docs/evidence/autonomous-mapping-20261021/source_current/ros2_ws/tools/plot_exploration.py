#!/usr/bin/env python3
"""只读自主建图实收数据，输出逐点波形、地图及独立仿真几何对照。"""
import argparse
import csv
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Circle
matplotlib.rcParams['path.simplify']=False
TITLE=('Autonomous mapping 20261021, SIMULATION ONLY, firmware source v131\n'
       'P right reverse .45 -> .45, others .30 -> .30; I steady .010 -> .010, start .030 -> .030 / 10 ms\n'
       'STM32 PI not exercised; new frontier exploration + existing SLAM / NavFn / RPP')


def read(path):
    """读取原始CSV全部点，缺少话题时不伪造零值。"""
    if not path.exists():return []
    with path.open() as stream:return list(csv.DictReader(stream))


def line(rows,key):
    """墙钟间隔超过1秒断线，不插值、不平滑、不删异常。"""
    xs=[];ys=[];previous=None
    for row in rows:
        t=float(row['wall_s'])
        if previous is not None and t-previous>1.:xs.append(np.nan);ys.append(np.nan)
        xs.append(t);ys.append(float(row[key]));previous=t
    return xs,ys


def save(fig,folder,name):
    """同时保留PNG和SVG，图中注明版本、PI及算法变化。"""
    fig.suptitle(TITLE,fontsize=11,y=.995);fig.tight_layout(rect=(0,0,1,.90))
    for suffix in ['png','svg']:fig.savefig(folder/(name+'.'+suffix),dpi=135,bbox_inches='tight')
    plt.close(fig)


def obstacles(path):
    """场景几何只供离线验收对照，不进入探索或定位输入。"""
    result=[]
    for model in ET.parse(path).find('world').findall('model'):
        pose=[float(x) for x in (model.findtext('pose') or '0 0 0 0 0 0').split()]
        for geometry in model.findall('.//collision/geometry'):
            if geometry.find('box') is not None:
                size=[float(x) for x in geometry.findtext('box/size').split()]
                result.append(('box',pose,size))
            elif geometry.find('cylinder') is not None:
                result.append(('circle',pose,float(geometry.findtext('cylinder/radius'))))
    return result


def quality(grid,resolution,origin,geometry):
    """逐个占据格中心距场景障碍表面的距离；固定初始世界基准，不拟合配准。"""
    iy,ix=np.nonzero(grid>=65);x=origin[0]+(ix+.5)*resolution;y=origin[1]+(iy+.5)*resolution
    distance=np.full(len(x),np.inf)
    for shape,pose,size in geometry:
        dx=x-pose[0];dy=y-pose[1]
        if shape=='circle':d=np.abs(np.hypot(dx,dy)-size)
        else:
            c=np.cos(pose[5]);s=np.sin(pose[5]);u=np.abs(c*dx+s*dy)-size[0]/2;v=np.abs(-s*dx+c*dy)-size[1]/2
            d=np.abs(np.hypot(np.maximum(u,0),np.maximum(v,0))+np.minimum(np.maximum(u,v),0))
        distance=np.minimum(distance,d)
    return {'occupied_cells':len(x),'reference':'SDF surfaces, initial map/world axes, no fitted alignment',
            'surface_distance_median_m':float(np.median(distance)) if len(x) else None,
            'surface_distance_p95_m':float(np.percentile(distance,95)) if len(x) else None,
            'occupied_support_fraction_within_015m':float(np.mean(distance<=.15)) if len(x) else None,
            'scope':'OFFLINE_SIMULATION_GEOMETRY_PROXY_NOT_HARDWARE_OR_FULL_MAP_ACCURACY_ACCEPTANCE'}


def main():
    """为已归档的新轮次绘图，不改动CSV、地图或原始结果JSON。"""
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('root',type=Path);args=parser.parse_args()
    root=args.root.resolve();plots=root/'plots';plots.mkdir(exist_ok=True);runs=root/'runs';analysis={}
    fig,axes=plt.subplots(2,1,figsize=(12,8),sharex=True)
    for name,label in [('baseline','Before: SLAM only, stationary'),('office_02','After: autonomous exploration')]:
        rows=read(runs/name/'maps.csv')
        for key,ax in [('known',axes[0]),('free',axes[1])]:
            derived=[{**r,'area':float(r[key])*float(r['resolution'])**2} for r in rows]
            ax.plot(*line(derived,'area'),'.-',markersize=1,linewidth=.6,label=label)
    axes[0].set_ylabel('Observed SLAM known area [m²]');axes[1].set_ylabel('Observed SLAM free area [m²]')
    axes[1].set_xlabel('Monotonic wall time [s]; observed area is NOT geometric accuracy or coverage')
    for ax in axes:ax.grid();ax.legend()
    save(fig,plots,'map_growth')
    watchdog=next((n for n in ['watchdog_04','watchdog_03','watchdog_02'] if (runs/n/'result.json').exists()),None)
    for name in ['office_02']+([watchdog] if watchdog else []):
        folder=runs/name
        if not (folder/'result.json').exists():continue
        fig,axes=plt.subplots(2,1,figsize=(12,8),sharex=True)
        odom=read(folder/'odom.csv');commands=read(folder/'commands.csv')
        for ax,key in zip(axes,['vx','wz']):
            ax.plot(*line(odom,key),'.-',markersize=1,linewidth=.6,label='Measured /sim/odom')
            ax.plot(*line(commands,key),'.-',markersize=1,linewidth=.5,label='Actual /nav/safe_cmd_vel')
            ax.grid();ax.legend()
        events=[json.loads(x) for x in (folder/'verifier_events.jsonl').read_text().splitlines()]
        for e in events:
            if e['event'] in ['explorer_frozen','explorer_resumed','service_response']:
                for ax in axes:ax.axvline(e['wall_s'],linestyle='--',linewidth=.5)
                label={'explorer_frozen':'SIGSTOP','explorer_resumed':'SIGCONT'}.get(e['event'])
                if label is None:label='FINISH' if e.get('service','').endswith('finish') else ('PAUSE' if e.get('data') else 'RESUME')
                axes[0].text(e['wall_s'],.97,label,transform=axes[0].get_xaxis_transform(),rotation=90,
                             ha='right',va='top',fontsize=7)
        axes[0].set_ylabel('Linear velocity [m/s]');axes[1].set_ylabel('Angular velocity [rad/s]')
        axes[0].set_ylim(-.17,.34);axes[1].set_ylim(-.85,.85);axes[1].set_xlabel('Monotonic wall time [s] / '+name)
        save(fig,plots,name+'_velocity')
    for name,world in [('room_03','smartcar'),('office_01','office_loop'),('office_02','office_loop'),('complete_room','observed_room')]:
        folder=runs/name/'mission'
        if not (folder/'map_final.npz').exists():continue
        geometry=obstacles(root/'source'/('world_'+world+'.sdf'))
        fig,axes=plt.subplots(1,2,figsize=(14,7))
        for index,phase in enumerate(['initial','final']):
            axes[index].set_facecolor('#cdcdcd')
            data=np.load(folder/('map_'+phase+'.npz'));grid=data['occupancy'];r=float(data['resolution']);origin=data['origin']
            image=np.full(grid.shape,205,dtype=np.uint8);image[(grid>=0)&(grid<=20)]=255;image[grid>=65]=0
            image[(grid>20)&(grid<65)]=125
            axes[index].imshow(image,origin='lower',extent=[origin[0],origin[0]+grid.shape[1]*r,origin[1],origin[1]+grid.shape[0]*r],
                               cmap='gray',vmin=0,vmax=255,interpolation='nearest')
            axes[index].set_title(name+' / real SLAM '+phase+' map');axes[index].set_aspect('equal')
            axes[index].set(xlabel='Map X [m]',ylabel='Map Y [m]');axes[index].grid(alpha=.2)
            if phase=='final':analysis[name]=quality(grid,r,origin,geometry)
        limits=[min(a.get_xlim()[0] for a in axes),max(a.get_xlim()[1] for a in axes),
                min(a.get_ylim()[0] for a in axes),max(a.get_ylim()[1] for a in axes)]
        for ax in axes:ax.set_xlim(limits[:2]);ax.set_ylim(limits[2:])
        save(fig,plots,name+'_maps')
        fig,ax=plt.subplots(figsize=(10,9));ax.set_facecolor('#cdcdcd');ax.imshow(image,origin='lower',extent=[origin[0],origin[0]+grid.shape[1]*r,
            origin[1],origin[1]+grid.shape[0]*r],cmap='gray',vmin=0,vmax=255,interpolation='nearest')
        for shape,pose,size in geometry:
            if shape=='box':
                # 当前两个场景均使用轴对齐墙体；有旋转的箱体也按原始模型角度绘制。
                dx,dy=size[:2];c=np.cos(pose[5]);s=np.sin(pose[5]);bottom=(pose[0]-c*dx/2+s*dy/2,pose[1]-s*dx/2-c*dy/2)
                ax.add_patch(Rectangle(bottom,dx,dy,angle=pose[5]*180/np.pi,fill=False,edgecolor='tab:red',linewidth=.6))
            else:ax.add_patch(Circle(pose[:2],size,fill=False,edgecolor='tab:red',linewidth=.6))
        ax.set(aspect='equal',xlabel='Map / initial world X [m]',ylabel='Map / initial world Y [m]',
               title=name+' / SDF outlines are offline reference ONLY; no alignment fit')
        save(fig,plots,name+'_reference')
    (root/'map_quality_proxy.json').write_text(json.dumps(analysis,indent=2))
    # 同一办公室、同一坐标尺度保留早期畸变与调整后地图，不裁掉失败区域。
    fig,axes=plt.subplots(1,2,figsize=(14,8));limits=[]
    for ax,name in zip(axes,['office_01','office_02']):
        d=np.load(runs/name/'mission/map_final.npz');g=d['occupancy'];r=float(d['resolution']);o=d['origin']
        im=np.full(g.shape,205,dtype=np.uint8);im[(g>=0)&(g<=20)]=255;im[g>=65]=0;im[(g>20)&(g<65)]=125
        extent=[o[0],o[0]+g.shape[1]*r,o[1],o[1]+g.shape[0]*r];limits.append(extent)
        ax.set_facecolor('#cdcdcd');ax.imshow(im,origin='lower',extent=extent,cmap='gray',vmin=0,vmax=255,interpolation='nearest')
        ax.set(aspect='equal',xlabel='Initial map X [m]',ylabel='Initial map Y [m]',title=name+' / '+('earlier fast mapping' if name=='office_01' else 'revised slow mapping'))
    for ax in axes:
        ax.set_xlim(min(v[0] for v in limits),max(v[1] for v in limits));ax.set_ylim(min(v[2] for v in limits),max(v[3] for v in limits));ax.grid(alpha=.2)
    save(fig,plots,'quality_before_after')
    fig,axes=plt.subplots(2,2,figsize=(14,9),sharex=True,sharey='row')
    for col,name in enumerate(['watchdog_03','watchdog_04']):
        folder=runs/name
        for ax,key in zip(axes[:,col],['vx','wz']):
            ax.plot(*line(read(folder/'odom.csv'),key),'.-',markersize=1,linewidth=.6,label='Measured odom')
            ax.plot(*line(read(folder/'commands.csv'),key),'.-',markersize=1,linewidth=.5,label='Safe command');ax.grid();ax.legend()
        axes[0,col].set_title(name+' / '+('image save failed' if col==0 else 'image + graph saved'))
        axes[1,col].set_xlabel('Monotonic wall time [s]');axes[0,col].set_ylim(-.17,.34);axes[1,col].set_ylim(-.85,.85)
    axes[0,0].set_ylabel('vx [m/s]');axes[1,0].set_ylabel('wz [rad/s]');save(fig,plots,'export_before_after')
    print(json.dumps(analysis,indent=2))


if __name__=='__main__':main()
