#!/usr/bin/env python3
"""离线复核自主探索前后覆盖速度、地图几何和起停波形；场景真值从不反馈车辆。"""
import argparse
import csv
import json
import math
from pathlib import Path
from collections import deque
import numpy as np
import yaml
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from plot_exploration import obstacles,quality,read,line

TITLE=('Exploration efficiency 20261021 | final v6, baseline / pilots as labeled | SIMULATION ONLY | firmware source v131\n'
       'P right reverse .45 -> .45, others .30 -> .30; I steady .010 -> .010, start .030 -> .030 / 10 ms\n'
       'PI NOT exercised; frontier / SLAM profile variants per panel; NavFn / RPP and motion limits unchanged')
plt.rcParams['path.simplify']=False


def reference_points(world):
    """在固定世界坐标栅格中取0.37m安全连通区，作为离线观测覆盖的共同分母。"""
    geometry=obstacles(world);bounds=[]
    for kind,p,size in geometry:
        if kind=='circle':hx=hy=size
        else:
            c,s=abs(math.cos(p[5])),abs(math.sin(p[5]));hx=(c*size[0]+s*size[1])/2;hy=(s*size[0]+c*size[1])/2
        bounds.append((p[0]-hx,p[0]+hx,p[1]-hy,p[1]+hy))
    r=.05;xs=np.arange(min(p[0] for p in bounds)-.5,max(p[1] for p in bounds)+.5,r)
    ys=np.arange(min(p[2] for p in bounds)-.5,max(p[3] for p in bounds)+.5,r)
    x,y=np.meshgrid(xs,ys);distance=np.full(x.shape,np.inf)
    for kind,p,size in geometry:
        dx=x-p[0];dy=y-p[1]
        if kind=='circle':d=np.hypot(dx,dy)-size
        else:
            c,s=math.cos(p[5]),math.sin(p[5]);u=np.abs(c*dx+s*dy)-size[0]/2;v=np.abs(-s*dx+c*dy)-size[1]/2
            d=np.hypot(np.maximum(u,0),np.maximum(v,0))+np.minimum(np.maximum(u,v),0)
        distance=np.minimum(distance,d)
    safe=distance>=.37;start=(int(np.argmin(abs(ys))),int(np.argmin(abs(xs))))
    if not safe[start]:raise ValueError('参考出生点不在安全区域')
    # 验收连通性单独计算，不调用被测探索器的搜索实现，避免共享同一个错误。
    mask=np.zeros_like(safe);mask[start]=True;queue=deque([start])
    while queue:
        row,col=queue.popleft()
        for nr,nc in [(row-1,col),(row+1,col),(row,col-1),(row,col+1)]:
            if 0<=nr<safe.shape[0] and 0<=nc<safe.shape[1] and safe[nr,nc] and not mask[nr,nc]:
                mask[nr,nc]=True;queue.append((nr,nc))
    return x[mask],y[mask],geometry


def sustained_milestones(timeline):
    """连续三次真实更新确认覆盖阈值，单次跳高不能算达到目标。"""
    result={}
    for threshold in [.8,.9,.95]:
        index=next((i for i in range(max(0,len(timeline)-2)) if all(p['observed_fraction']>=threshold for p in timeline[i:i+3])),None)
        result[str(threshold)]={k:timeline[index][k] for k in ['sim_s','wall_s']} if index is not None else None
    return result


def evaluate(run,world):
    """全部地图逐帧比较，保留未完成/失败；不拟合坐标、不插值跨越阈值时间。"""
    result=json.loads((run/'result.json').read_text()) if (run/'result.json').exists() else {}
    session=json.loads((run/'mission/session.json').read_text()) if (run/'mission/session.json').exists() else {}
    x,y,geometry=reference_points(world);timeline=[]
    snapshots=sorted((run/'map_snapshots').glob('*.npz'))
    for path in snapshots:
        d=np.load(path);g=d['occupancy'];r=float(d['resolution']);o=d['origin']
        ix=np.floor((x-o[0])/r).astype(int);iy=np.floor((y-o[1])/r).astype(int)
        inside=(ix>=0)&(ix<g.shape[1])&(iy>=0)&(iy<g.shape[0]);observed=np.zeros(len(x),dtype=bool)
        values=g[iy[inside],ix[inside]];observed[inside]=(values>=0)&(values<=20)
        timeline.append({'sim_s':float(d['sim_s']),'wall_s':float(d['wall_s']),'observed_fraction':float(observed.mean())})
    milestones=sustained_milestones(timeline)
    milestone_quality={}
    for threshold,point in milestones.items():
        if point is None:continue
        index=next(i for i,row in enumerate(timeline) if row['sim_s']==point['sim_s'])
        d=np.load(snapshots[index]);milestone_quality[threshold]=quality(d['occupancy'],float(d['resolution']),d['origin'],geometry)
    odom=read(run/'mission/odom.csv');angles=[2*math.atan2(float(p['qz']),float(p['qw'])) for p in odom]
    angular=sum(abs(math.atan2(math.sin(b-a),math.cos(b-a))) for a,b in zip(angles,angles[1:]))
    idle=turn_only=0.
    for a,b in zip(odom,odom[1:]):
        dt=max(0.,float(b['sim_s'])-float(a['sim_s']));v,w=abs(float(a['vx_mps'])),abs(float(a['wz_rps']))
        if v<.01 and w<.02:idle+=dt
        if v<.02 and w>.05:turn_only+=dt
    final=run/'mission/map_final.npz';geometry_quality=None
    if final.exists():
        d=np.load(final);geometry_quality=quality(d['occupancy'],float(d['resolution']),d['origin'],geometry)
    exported_quality=None;metadata_path=run/'mission/map.yaml'
    if metadata_path.is_file():
        metadata=yaml.safe_load(metadata_path.read_text());image_path=metadata_path.parent/metadata['image']
        if image_path.is_file():
            # 实际交付的PGM也检查，不能用保存之后的在线末帧代替导出质量。
            pixels=np.asarray(Image.open(image_path).convert('L'))
            if not set(np.unique(pixels)).issubset({0,205,254}):raise ValueError('导出图像不是本项目三值PGM')
            exported=np.flipud(np.where(pixels==0,100,np.where(pixels==254,0,-1)))
            exported_quality=quality(exported,float(metadata['resolution']),metadata['origin'][:2],geometry)
    goals=session.get('goals',[])
    probe=read(run/'pose_probe.csv');pose_diagnostics={}
    for label in ['odom','slam']:
        errors=np.asarray([math.hypot(float(p[label+'_x'])-float(p['world_x']),float(p[label+'_y'])-float(p['world_y'])) for p in probe])
        valid=errors[np.isfinite(errors)]
        pose_diagnostics[label]={'received_samples':len(errors),'missing_samples':int((~np.isfinite(errors)).sum()),
                                 'median_position_error_m':float(np.median(valid)) if len(valid) else None,
                                 'p95_position_error_m':float(np.percentile(valid,95)) if len(valid) else None,
                                 'max_position_error_m':float(valid.max()) if len(valid) else None,
                                 'scope':'Near-time passive pose samples; no interpolation or fitted alignment'}
    summary={'result':result.get('result'),'reason':session.get('reason'),'complete':session.get('complete',False),
             'stop_verified':session.get('stop_verified',False),'goals':len(goals),'succeeded':sum(g.get('action_status')==4 for g in goals),
             'navigation_failures':session.get('navigation_failures'),'travel_m':result.get('travel_m'),'angular_travel_rad':angular,
             'idle_sim_s':idle,'turn_only_sim_s':turn_only,'duration_wall_s':result.get('duration_wall_s'),
             'end_sim_s':timeline[-1]['sim_s'] if timeline else None,'observed_fraction_final':timeline[-1]['observed_fraction'] if timeline else None,
             'reference_area_m2':len(x)*.05**2,'milestones':milestones,'map_geometry_proxy':geometry_quality,
             'exported_map_geometry_proxy':exported_quality,
             'milestone_map_geometry_proxy':milestone_quality,
             'passive_pose_diagnostics':pose_diagnostics,
             'metric_scope':'Observed free fraction of fixed SDF reachable center-space; offline only, not physical accuracy or all-space coverage'}
    return summary,timeline


def save(fig,folder,name):
    """图上标明版本与PI，保存逐点PNG/SVG，不改写原始数据。"""
    fig.suptitle(TITLE,fontsize=10,y=.995);fig.tight_layout(rect=(0,0,1,.89))
    folder.mkdir(exist_ok=True)
    for suffix in ['png','svg']:fig.savefig(folder/(name+'.'+suffix),dpi=135,bbox_inches='tight')
    plt.close(fig)


def plot_velocity(root,names,stem):
    """按同一对比工况画完整起停速度，所有样本保留且纵轴一致。"""
    fig,axes=plt.subplots(2,len(names),figsize=(max(10,6*len(names)),8),squeeze=False,sharey='row',sharex=True)
    for col,name in enumerate(names):
        folder=root/'runs'/name
        for ax,key in zip(axes[:,col],['vx','wz']):
            ax.plot(*line(read(folder/'odom.csv'),key),'.-',markersize=1,linewidth=.5,label='Actual odom')
            ax.plot(*line(read(folder/'commands.csv'),key),'.-',markersize=1,linewidth=.5,label='Actual safe cmd')
            ax.grid(alpha=.3);ax.legend(fontsize=7)
        event_file=folder/'verifier_events.jsonl'
        if event_file.exists():
            for raw in event_file.read_text().splitlines():
                event=json.loads(raw);label=None
                if event['event']=='explorer_frozen':label='SIGSTOP explorer'
                elif event['event']=='explorer_resumed':label='SIGCONT explorer'
                elif event['event']=='service_response':
                    if event.get('service')=='/exploration/pause':label='Explicit pause' if event.get('data') else 'Explicit resume'
                    elif event.get('service')=='/exploration/finish':label='Finish / save'
                if label:
                    # 验证器事件和波形共用同一单调墙钟，不换算或拟合对齐时间。
                    for row,ax in enumerate(axes[:,col]):
                        ax.axvline(event['wall_s'],color='gray',linestyle=':',linewidth=.7,label=label if row==0 else None)
            axes[0,col].legend(fontsize=6)
        axes[0,col].set_title(name);axes[1,col].set_xlabel('Monotonic wall time [s]')
        axes[0,col].set_ylim(-.17,.34);axes[1,col].set_ylim(-.85,.85)
    axes[0,0].set_ylabel('Linear speed [m/s]');axes[1,0].set_ylabel('Angular speed [rad/s]')
    save(fig,root/'plots',stem)


def plot_maps(root,names,stem):
    """原始末帧地图同尺度对照，不做配准或裁掉误差区域。"""
    fig,axes=plt.subplots(1,len(names),figsize=(max(10,6*len(names)),7),squeeze=False);extents=[]
    for ax,name in zip(axes[0],names):
        path=root/'runs'/name/'mission/map_final.npz'
        if not path.exists():ax.set_title(name+' / NO FINAL MAP');continue
        d=np.load(path);g=d['occupancy'];r=float(d['resolution']);o=d['origin']
        image=np.full(g.shape,205,dtype=np.uint8);image[(g>=0)&(g<=20)]=255;image[g>=65]=0;image[(g>20)&(g<65)]=125
        extent=[o[0],o[0]+g.shape[1]*r,o[1],o[1]+g.shape[0]*r];extents.append(extent)
        ax.set_facecolor('#cdcdcd');ax.imshow(image,origin='lower',extent=extent,cmap='gray',vmin=0,vmax=255,interpolation='nearest')
        ax.set(aspect='equal',title=name+' / actual final online grid',xlabel='Initial map X [m]',ylabel='Initial map Y [m]')
    for ax in axes[0]:
        if extents:ax.set_xlim(min(v[0] for v in extents),max(v[1] for v in extents));ax.set_ylim(min(v[2] for v in extents),max(v[3] for v in extents))
    save(fig,root/'plots',stem)


def plot_pose(root,name):
    """真值仅作离线诊断，逐点保留时序、缺测和坐标差，不拟合对齐。"""
    rows=read(root/'runs'/name/'pose_probe.csv');fig,axes=plt.subplots(3,1,figsize=(12,9),sharex=True)
    for index,axis in enumerate(['x','y']):
        for source in ['world','odom','slam']:axes[index].plot(*line(rows,source+'_'+axis),'.-',markersize=1,linewidth=.5,label=source)
        axes[index].set_ylabel(axis+' [m]');axes[index].grid();axes[index].legend()
    for source in ['odom','slam']:
        derived=[{**p,'error':math.hypot(float(p[source+'_x'])-float(p['world_x']),float(p[source+'_y'])-float(p['world_y']))} for p in rows]
        axes[2].plot(*line(derived,'error'),'.-',markersize=1,linewidth=.5,label=source+' vs world')
    axes[2].set(ylabel='Near-time position difference [m]',xlabel='Monotonic wall time [s] / '+name);axes[2].grid();axes[2].legend()
    save(fig,root/'plots','pose_'+name)


def plot_quality_timeline(root,names,world,stem):
    """逐帧计算固定坐标几何代理，显示后期退化；不借配准、平滑或缺测补值隐藏偏差。"""
    geometry=obstacles(world);fig,axes=plt.subplots(2,1,figsize=(12,8),sharex=True)
    for name in names:
        rows=[]
        for path in sorted((root/'runs'/name/'map_snapshots').glob('*.npz')):
            d=np.load(path);q=quality(d['occupancy'],float(d['resolution']),d['origin'],geometry)
            rows.append({'sim_s':float(d['sim_s']),'wall_s':float(d['wall_s']),
                         'surface_p95_m':q['surface_distance_p95_m'],
                         'support_within_015m':q['occupied_support_fraction_within_015m']})
        with (root/(name+'_quality.csv')).open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=['sim_s','wall_s','surface_p95_m','support_within_015m'])
            writer.writeheader();writer.writerows(rows)
        for key,ax in zip(['surface_p95_m','support_within_015m'],axes):
            xs=[];ys=[];previous=None
            for row in rows:
                if previous and (row['sim_s']-previous['sim_s']>6 or row['wall_s']-previous['wall_s']>6):
                    xs.append(np.nan);ys.append(np.nan)
                xs.append(row['sim_s']);ys.append(np.nan if row[key] is None else row[key]);previous=row
            ax.plot(xs,ys,'.-',markersize=1,linewidth=.6,label=name)
    axes[0].axhline(.15,color='gray',linestyle='--',linewidth=.7,label='Final acceptance: 0.15 m')
    axes[1].axhline(.95,color='gray',linestyle='--',linewidth=.7,label='Final acceptance: 0.95')
    axes[0].set_ylabel('Occupied to SDF surface P95 [m]')
    axes[1].set(ylabel='Occupied support within 0.15 m',xlabel='Actual simulation time [s]',ylim=(0,1.02))
    for ax in axes:ax.grid();ax.legend(fontsize=7)
    save(fig,root/'plots',stem)


def main():
    """单轮诊断输出JSON，或按归档manifest批量复核并生成真实前后对比图。"""
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('root',type=Path)
    parser.add_argument('--world',type=Path);args=parser.parse_args();root=args.root.resolve()
    if args.world:
        summary,timeline=evaluate(root,args.world.resolve());print(json.dumps(summary,indent=2));return
    manifest=json.loads((root/'runs.json').read_text());summary={};timelines={}
    for name,item in manifest.items():
        summary[name],timelines[name]=evaluate(root/'runs'/name,root/item['world'])
    (root/'summary.json').write_text(json.dumps(summary,indent=2))
    plots=root/'plots';worlds=list(dict.fromkeys(item['world'] for item in manifest.values()))
    fig,axes=plt.subplots(len(worlds),1,figsize=(12,5*len(worlds)),squeeze=False,sharex=True)
    for name,rows in timelines.items():
        item=manifest[name];ax=axes[worlds.index(item['world']),0];xs=[];ys=[];previous=None
        for row in rows:
            # 地图名义每2秒更新，任一时钟超过6秒的缺口断线，不补造中间观测。
            if previous and (row['sim_s']-previous['sim_s']>6. or row['wall_s']-previous['wall_s']>6.):
                xs.append(np.nan);ys.append(np.nan)
            xs.append(row['sim_s']);ys.append(row['observed_fraction']);previous=row
        ax.plot(xs,ys,'.-',markersize=1,linewidth=.6,label=name+' / '+item['version'])
        ax.set_title(Path(item['world']).stem+' / fixed reachable center-space reference')
        with (root/(name+'_coverage.csv')).open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=['sim_s','wall_s','observed_fraction']);writer.writeheader();writer.writerows(rows)
    for ax in axes[:,0]:
        ax.set(xlabel='Actual simulation time [s]',ylabel='Observed reference free fraction',ylim=(0,1.02))
        ax.grid();ax.legend(fontsize=8)
    save(fig,plots,'coverage')
    groups={}
    for name,item in manifest.items():
        groups.setdefault(item.get('pair',name.replace('before_','').replace('after_','')),[]).append(name)
    for pair,names in groups.items():
        plot_velocity(root,names,'velocity_'+pair)
        plot_maps(root,names,'maps_'+pair)
    for world in worlds:
        names=[name for name,item in manifest.items() if item['world']==world]
        plot_quality_timeline(root,names,root/world,'quality_'+Path(world).stem)
    for name in manifest:
        if (root/'runs'/name/'pose_probe.csv').exists():plot_pose(root,name)
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
