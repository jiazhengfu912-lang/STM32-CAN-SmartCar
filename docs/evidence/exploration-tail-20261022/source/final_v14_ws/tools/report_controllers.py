#!/usr/bin/env python3
"""只读汇总真实Nav2对比试验，生成CSV、逐点图及中文选型报表。"""
import argparse
import base64
import csv
import gzip
import hashlib
import html
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
matplotlib.rcParams['path.simplify']=False


NAMES=['DWB','RPP','MPPI']
COLORS={'DWB':'tab:blue','RPP':'tab:orange','MPPI':'tab:green'}
TITLE=('DWB / RPP / MPPI, 20261019-20, SIMULATION ONLY, firmware source v131\n'
       'P right reverse .45 -> .45, others .30 -> .30; I steady .010 -> .010, start .030 -> .030 per 10 ms\n'
       'STM32 PI not exercised; local controller changed; common NavFn, BT, costmaps, smoother and safety gates')


def read_csv(path):
    """读取全部原始CSV，不平滑、不改写输入。"""
    if not path.exists():return []
    with path.open() as f:return list(csv.DictReader(f))


def json_lines(path):
    """支持保留原始字节哈希的无损gzip归档。"""
    if path.exists():stream=path.open()
    elif path.with_suffix(path.suffix+'.gz').exists():stream=gzip.open(path.with_suffix(path.suffix+'.gz'),'rt')
    else:return []
    with stream:return [json.loads(line) for line in stream]


def point_segments(points, first, second):
    """向量化计算点到线段距离，不插值生成车辆采样点。"""
    delta=second-first
    t=np.sum((points-first)*delta,axis=-1)/np.maximum(np.sum(delta*delta,axis=-1),1.e-20)
    return np.linalg.norm(points-first-np.clip(t,0.,1.)[...,None]*delta,axis=-1)


def polygon_clearance(body, obstacle):
    """分离轴判相交，顶点到边的最小值给出二维多边形间隙。"""
    count=len(body); separated=np.zeros(count,dtype=bool)
    for shape in [body,np.broadcast_to(obstacle,(count,*obstacle.shape))]:
        edge=np.roll(shape,-1,axis=1)-shape
        for i in range(4):
            normal=np.stack([-edge[:,i,1],edge[:,i,0]],axis=-1)
            first=np.sum(body*normal[:,None,:],axis=-1)
            second=np.sum(obstacle[None,:,:]*normal[:,None,:],axis=-1)
            separated|=(first.max(axis=1)<second.min(axis=1))|(second.max(axis=1)<first.min(axis=1))
    first=point_segments(body[:,:,None,:],obstacle[None,None,:,:],np.roll(obstacle,-1,axis=0)[None,None,:,:]).min(axis=(1,2))
    second=point_segments(obstacle[None,:,None,:],body[:,None,:,:],np.roll(body,-1,axis=1)[:,None,:,:]).min(axis=(1,2))
    return np.where(separated,np.minimum(first,second),0.)


def world_obstacles(ws):
    """提取办公室静态碰撞形状；地板平面不作为二维障碍。"""
    obstacles=[]
    for model in ET.parse(ws/'src/smartcar_sim/worlds/office_loop.sdf').find('world').findall('model'):
        pose=[float(v) for v in (model.findtext('pose') or '0 0 0 0 0 0').split()]
        for geom in model.findall('.//collision/geometry'):
            if geom.find('box') is not None:
                size=[float(v) for v in geom.findtext('box/size').split()]
                obstacles.append({'shape':'box','x':pose[0],'y':pose[1],'yaw':pose[5],'size':size[:2]})
            elif geom.find('cylinder') is not None:
                obstacles.append({'shape':'circle','x':pose[0],'y':pose[1],
                                  'radius':float(geom.findtext('cylinder/radius'))})
    return obstacles


def clearance_trace(rows, obstacles, inserted=None):
    """用实际世界位姿和带padding车体计算模型间隙；不是接触传感器。"""
    if not rows:return [],float('nan'),0
    t=np.array([float(r['wall_s']) for r in rows]);x=np.array([float(r['x']) for r in rows])
    y=np.array([float(r['y']) for r in rows]);a=np.array([float(r['yaw']) for r in rows])
    c=np.cos(a);s=np.sin(a);local=np.array([[-.16,-.18],[.27,-.18],[.27,.18],[-.16,.18]])
    body=np.stack([x[:,None]+c[:,None]*local[None,:,0]-s[:,None]*local[None,:,1],
                   y[:,None]+s[:,None]*local[None,:,0]+c[:,None]*local[None,:,1]],axis=-1)
    closest=np.full(len(rows),np.inf)
    for obstacle in obstacles:
        if obstacle['shape']=='box':
            hx,hy=np.array(obstacle['size'][:2])/2;theta=obstacle.get('yaw',0.)
            vertices=np.array([[-hx,-hy],[hx,-hy],[hx,hy],[-hx,hy]])
            rotation=np.array([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])
            poly=vertices@rotation.T+np.array([obstacle['x'],obstacle['y']]);distance=polygon_clearance(body,poly)
        else:
            dx=obstacle['x']-x;dy=obstacle['y']-y
            u=dx*c+dy*s;v=-dx*s+dy*c
            distance=np.maximum(0.,np.hypot(u-np.clip(u,-.16,.27),v-np.clip(v,-.18,.18))-obstacle['radius'])
        if obstacle.get('inserted') and inserted is not None:distance=np.where(t>=inserted,distance,np.inf)
        closest=np.minimum(closest,distance)
    trace=[{'wall_s':float(ti),'clearance_m':float(di)} for ti,di in zip(t,closest)]
    return trace,float(closest.min()),int((closest<=1.e-9).sum())


def write_csv(path, rows):
    """派生表另写新文件，保留原数据文件。"""
    if not rows:return
    with path.open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)


def run_metrics(root,item,ws):
    """严格区分导航到达成功、预期不可达失败、超时及停车证据。"""
    folder=root/item['run'];report=json.loads((folder/'result.json').read_text())
    events=json_lines(folder/'events.jsonl');start=next((e for e in events if e['event']=='goal_accepted'),None)
    end=next((e for e in events if e['event'] in ['goal_result','goal_timeout','failure'] and start and e['wall_s']>=start['wall_s']),None)
    begin=start['wall_s'] if start else 0.;finish=end['wall_s'] if end else report.get('duration_wall_s',0.)
    active=lambda r:begin<=float(r['wall_s'])<=finish
    all_truth=read_csv(folder/'world_pose.csv')
    truth=[r for r in all_truth if active(r)]
    safety_truth=[r for r in all_truth if float(r['wall_s'])>=begin]
    odom=[r for r in read_csv(folder/'odom.csv') if active(r)]
    resources=[r for r in read_csv(folder/'resources.csv') if active(r)]
    elapsed_sim=(end['sim_s']-start['sim_s']) if start and end and end.get('sim_s') is not None and start.get('sim_s') is not None else None
    elapsed_wall=finish-begin if start and end else None
    distance=sum(math.hypot(float(q['x'])-float(p['x']),float(q['y'])-float(p['y'])) for p,q in zip(truth,truth[1:]))
    tv=0.;acc=[]
    for first,second in zip(odom,odom[1:]):
        dt=float(second['stamp_s'])-float(first['stamp_s'])
        if 1.e-4<dt<=.2:
            tv+=abs(float(second['wz'])-float(first['wz']))
            acc.append(((float(second['vx'])-float(first['vx']))/dt)**2)
    obstacles=world_obstacles(ws)
    for b in report.get('blockers',[]):obstacles.append({**b,'shape':'circle' if b['shape']=='sphere' else 'box'})
    if report.get('inserted_wall_s') is not None:
        b=report['obstacle'];obstacles.append({'shape':'box','x':b['x'],'y':b['y'],'size':[b['dx'],b['dy']],'inserted':True})
    trace,minimum,overlap=clearance_trace(safety_truth,obstacles,report.get('inserted_wall_s'))
    write_csv(folder/'clearance_derived.csv',trace)
    goal=report['goal'];final=report.get('final_world_pose')
    position_error=math.hypot(final[0]-goal[0],final[1]-goal[1]) if final else None
    heading_error=abs(math.atan2(math.sin(final[2]-goal[2]),math.cos(final[2]-goal[2]))) if final else None
    blocked=item['case']=='two_doors'
    task_ok=(report['result']=='PASS' and bool(report.get('stop_evidence',{}).get('verified'))
             and report.get('action_status')==(6 if blocked else 4))
    if not blocked:task_ok=task_ok and position_error is not None and position_error<.25 and heading_error is not None and heading_error<.3
    cpu=None;cpu_seconds=None
    if len(resources)>1:
        cpu_seconds=float(resources[-1]['cpu_total_s'])-float(resources[0]['cpu_total_s'])
        cpu=100*cpu_seconds/(float(resources[-1]['wall_s'])-float(resources[0]['wall_s']))
    gate_counts={}
    for sample in json_lines(folder/'safety.jsonl'):
        if begin<=sample['wall_s']<=finish:
            state=json.loads(sample['msg']['data'])['state']
            gate_counts[state]=gate_counts.get(state,0)+1
    log=(folder/'launch.log').read_text()
    safe=[r for r in read_csv(folder/'velocity.csv') if r['topic']=='/nav/safe_cmd_vel' and float(r['wall_s'])>=begin]
    limit_violations=sum(not(-.150001<=float(r['vx'])<=.300001 and abs(float(r['wz']))<=.600001) for r in safe)
    missed=len(re.findall(r'\[controller_server\]: Control loop missed its desired rate',log))
    recoveries=sum('Running '+kind in line for line in log.splitlines() for kind in ['wait','spin','backup'])
    failure_checks=[name for name,good in report.get('checks',{}).items() if not good]
    failure_category=('STARTUP_NO_GOAL' if start is None else
                      'GOAL_WALL_TIMEOUT' if end and end['event']=='goal_timeout' else
                      'TASK_CHECK_FAILED' if not task_ok else 'NONE')
    result={'controller':item['controller'],'case':item['case'],'repeat':item['repeat'],'run':item['run'],
            'report_result':report['result'],'action_status':report.get('action_status'),'task_correct_and_stopped':bool(task_ok),
            'goal_sent':start is not None,'failure_category':failure_category,
            'stop_verified':report.get('stop_evidence',{}).get('verified'),
            'model_overlap_samples':overlap,'min_model_clearance_m':minimum,
            'safe_command_limit_violations':limit_violations,
            'elapsed_sim_s':elapsed_sim,'elapsed_wall_s':elapsed_wall,
            'terminal_event':end['event'] if end else None,
            'real_time_factor':elapsed_sim/elapsed_wall if elapsed_sim is not None and elapsed_wall else None,
            'travel_m':distance,'position_error_m':position_error,'heading_error_rad':heading_error,
            'angular_tv_per_sim_s':tv/elapsed_sim if elapsed_sim else None,
            'linear_accel_rms_mps2':math.sqrt(sum(acc)/len(acc)) if acc else None,
            'controller_process_cpu_percent':cpu,'controller_process_peak_rss_mib':max([float(r['rss_bytes'])/1024**2 for r in resources] or [0.]),
            'controller_process_cpu_s':cpu_seconds,
            'controller_process_cpu_s_per_sim_s':cpu_seconds/elapsed_sim if cpu_seconds is not None and elapsed_sim else None,
            'host_load1_mean':float(np.mean([float(r['host_load1']) for r in resources])) if resources else None,
            'gate_states_task_phase':json.dumps(gate_counts,sort_keys=True),
            'control_loop_overruns':missed,'recovery_actions':recoveries,'truth_samples':len(truth),
            'progress_failures':log.count('Failed to make progress'),
            'shutdown_escalations':log.count('failed to terminate'),
            'error':report.get('error') or '; '.join(failure_checks) or None}
    result['task_correct_and_stopped']=bool(result['task_correct_and_stopped'] and limit_violations==0)
    (folder/'metrics.json').write_text(json.dumps(result,indent=2));return result


def summary_rows(metrics, repeats):
    """先按可靠性筛选；只有全计划轮次通过且无模型重叠者才比较效率代价。"""
    rows=[]
    for name in NAMES:
        trials=[r for r in metrics if r['controller']==name];reachable=[r for r in trials if r['case']!='two_doors']
        blocked=[r for r in trials if r['case']=='two_doors']
        row={'controller':name,'trials':len(trials),'correct_tasks':sum(r['task_correct_and_stopped'] for r in trials),
             'startup_no_goal':sum(r.get('failure_category')=='STARTUP_NO_GOAL' for r in trials),
             'goal_wall_timeouts':sum(r.get('failure_category')=='GOAL_WALL_TIMEOUT' for r in trials),
             'goals_attempted':sum(r.get('goal_sent',True) for r in trials),
             'reachable_success':sum(r['task_correct_and_stopped'] for r in reachable),'reachable_trials':len(reachable),
             'blocked_correct':sum(r['task_correct_and_stopped'] for r in blocked),'blocked_trials':len(blocked),
             'model_overlap_trials':sum(r['model_overlap_samples']>0 for r in trials),
             'min_model_clearance_m':min([r['min_model_clearance_m'] for r in trials] or [float('nan')]),
             'eligible':len(trials)==repeats*4 and all(r['task_correct_and_stopped'] and r['model_overlap_samples']==0 for r in trials),
             'normalized_cost':None}
        for key in ['elapsed_sim_s','elapsed_wall_s','travel_m','angular_tv_per_sim_s','controller_process_cpu_percent',
                    'controller_process_peak_rss_mib','position_error_m','control_loop_overruns']:
            values=[float(r[key]) for r in reachable if r['task_correct_and_stopped'] and r[key] is not None]
            row[key+'_mean']=float(np.mean(values)) if values else None
            row[key+'_std']=float(np.std(values,ddof=1)) if len(values)>1 else None
        rows.append(row)
    eligible=[r['controller'] for r in rows if r['eligible']]
    costs={name:[] for name in eligible}
    for case in ['straight','turn','detour']:
        total={name:0. for name in eligible}
        for key,weight in [('elapsed_sim_s',.4),('travel_m',.25),('angular_tv_per_sim_s',.2),('controller_process_cpu_percent',.15)]:
            means={name:float(np.mean([r[key] for r in metrics if r['controller']==name and r['case']==case])) for name in eligible}
            if means:
                low=min(means.values());high=max(means.values())
                for name,value in means.items():total[name]+=weight*((value-low)/(high-low) if high-low>1.e-9 else 0.)
        for name in eligible:costs[name].append(total[name])
    for row in rows:
        if row['controller'] in costs:row['normalized_cost']=float(np.mean(costs[row['controller']]))
    ranking=sorted(rows,key=lambda r:(not r['eligible'],-r['correct_tasks'],r['model_overlap_trials'],
                                    r['normalized_cost'] if r['normalized_cost'] is not None else float('inf')))
    return rows,ranking


def raw_line(rows,key,begin):
    """长于一墙钟秒的缺测断线，既不平滑也不补造测量样本。"""
    xs=[];ys=[];previous=None
    for row in rows:
        t=float(row['wall_s'])
        if previous is not None and t-previous>1.:xs.append(float('nan'));ys.append(float('nan'))
        xs.append(t-begin);ys.append(float(row[key]));previous=t
    return xs,ys


def exploratory_candidate(metrics, rows):
    """事后给出后续验证候选；不放宽或替代冻结协议的全部通过门槛。"""
    maximum=max((r['correct_tasks'] for r in rows),default=0)
    names=[r['controller'] for r in rows if r['correct_tasks']==maximum and not r['model_overlap_trials']
           and all(any(m['controller']==r['controller'] and m['case']==case and m['task_correct_and_stopped']
                       for m in metrics) for case in ['straight','turn','detour'])]
    costs={name:0. for name in names}
    for case in ['straight','turn','detour']:
        for key,weight in [('elapsed_sim_s',.4),('travel_m',.25),('angular_tv_per_sim_s',.2),('controller_process_cpu_percent',.15)]:
            means={name:float(np.mean([m[key] for m in metrics if m['controller']==name and m['case']==case
                                      and m['task_correct_and_stopped']])) for name in names}
            if means:
                low=min(means.values());high=max(means.values())
                for name,value in means.items():costs[name]+=weight*((value-low)/(high-low) if high-low>1.e-9 else 0.)/3
    ordered=sorted(costs,key=costs.get)
    return {'candidate':ordered[0] if ordered else None,'costs':costs,'post_hoc':True,
            'qualification':'EXPLORATORY_ONLY_NOT_ACCEPTANCE',
            'method':'Among controllers tied for most correct planned tasks and no model overlap, require at least one completion in each reachable case; compare successful-case means using original efficiency weights. Failed planned trials remain failures. This is a post-hoc validation priority, not the frozen protocol winner.'}


def plot_cases(root,metrics,ws):
    """同场景同尺度展示每轮真实速度、间隙与轨迹，失败同样画出。"""
    from matplotlib.patches import Rectangle,Circle
    plots=root/'plots';plots.mkdir(exist_ok=True)
    for case in ['straight','turn','detour','two_doors']:
        trials=[r for r in metrics if r['case']==case]
        if not trials:continue
        fig,axes=plt.subplots(3,1,figsize=(13,10),sharex=True)
        trajectory,ax=plt.subplots(figsize=(10,9))
        for obstacle in world_obstacles(ws):
            if obstacle['shape']=='box':
                dx,dy=obstacle['size'];ax.add_patch(Rectangle((obstacle['x']-dx/2,obstacle['y']-dy/2),dx,dy,color='gray',alpha=.55))
            else:ax.add_patch(Circle((obstacle['x'],obstacle['y']),obstacle['radius'],color='gray',alpha=.55))
        sample=json.loads((root/trials[0]['run']/'result.json').read_text())
        for b in sample.get('blockers',[]):
            if b['shape']=='sphere':ax.add_patch(Circle((b['x'],b['y']),b['radius'],color='red',alpha=.65))
            else:
                dx,dy=b['size'][:2];ax.add_patch(Rectangle((b['x']-dx/2,b['y']-dy/2),dx,dy,color='red',alpha=.65))
        if sample.get('inserted_wall_s') is not None:
            b=sample['obstacle'];ax.add_patch(Rectangle((b['x']-b['dx']/2,b['y']-b['dy']/2),b['dx'],b['dy'],color='red',alpha=.65))
        ax.scatter([sample['goal'][0]],[sample['goal'][1]],marker='*',s=80,color='black',label='Goal')
        for index,trial in enumerate(trials):
            folder=root/trial['run'];events=json_lines(folder/'events.jsonl')
            start=next((e['wall_s'] for e in events if e['event']=='goal_accepted'),0.)
            odom=[r for r in read_csv(folder/'odom.csv') if float(r['wall_s'])>=start]
            truth=[r for r in read_csv(folder/'world_pose.csv') if float(r['wall_s'])>=start]
            clearance=read_csv(folder/'clearance_derived.csv');label=f'{trial["controller"]} r{trial["repeat"]} {trial["report_result"]}'
            style=['-','--',':'][(trial['repeat']-1)%3];color=COLORS[trial['controller']]
            for a,key in zip(axes[:2],['vx','wz']):a.plot(*raw_line(odom,key,start),style,color=color,linewidth=.6,marker='.',markersize=1,alpha=.7,label=label)
            axes[2].plot(*raw_line(clearance,'clearance_m',start),style,color=color,linewidth=.6,alpha=.7,label=label)
            ax.plot(raw_line(truth,'x',start)[1],raw_line(truth,'y',start)[1],style,color=color,linewidth=.8,marker='.',markersize=1,label=label)
        for a,title in zip(axes,['Measured odom vx [m/s]','Measured odom wz [rad/s]','2D padded footprint clearance [m]']):a.set_ylabel(title);a.grid()
        axes[0].set_ylim(-.17,.34);axes[1].set_ylim(-.85,.85);axes[2].set_ylim(bottom=0.)
        axes[0].legend(ncol=3,fontsize=7);axes[2].set_xlabel('Wall time since goal acceptance [s]; differing RTF is reported separately')
        fig.suptitle(TITLE+'\n'+case+' / all recorded trials, including failures');fig.tight_layout()
        ax.set(xlim=(-6.3,6.3),ylim=(-5.3,5.3),aspect='equal',xlabel='Gazebo world X [m]',ylabel='Gazebo world Y [m]');ax.grid();ax.legend(fontsize=7)
        trajectory.suptitle(TITLE+'\n'+case+' / complete actual trajectories');trajectory.tight_layout()
        for ext in ['png','svg']:
            fig.savefig(plots/(case+'_traces.'+ext),dpi=130,bbox_inches='tight')
            trajectory.savefig(plots/(case+'_trajectory.'+ext),dpi=130,bbox_inches='tight')
        plt.close(fig);plt.close(trajectory)


def case_statistics(metrics):
    """分别列各场景的样本数、完成率与成功轮次均值/样本标准差。"""
    rows=[]
    keys=['elapsed_sim_s','elapsed_wall_s','travel_m','angular_tv_per_sim_s','controller_process_cpu_percent',
          'controller_process_peak_rss_mib','position_error_m','min_model_clearance_m','real_time_factor']
    for case in ['straight','turn','detour','two_doors']:
        for name in NAMES:
            selected=[r for r in metrics if r['case']==case and r['controller']==name]
            row={'case':case,'controller':name,'n':len(selected),'correct':sum(r['task_correct_and_stopped'] for r in selected)}
            for key in keys:
                values=[r[key] for r in selected if r['task_correct_and_stopped'] and r[key] is not None]
                row[key+'_mean']=float(np.mean(values)) if values else None
                row[key+'_std']=float(np.std(values,ddof=1)) if len(values)>1 else None
            rows.append(row)
    return rows


def plot_overview(root,rows,stats):
    """总览均值与标准差，计数图包含失败，效率图明确只含完成轮次。"""
    fig,axes=plt.subplots(2,2,figsize=(14,10));x=np.arange(3)
    axes[0,0].bar(x,[r['correct_tasks'] for r in rows],color=[COLORS[n] for n in NAMES])
    axes[0,0].set(xticks=x,xticklabels=NAMES,ylabel='Correct task outcome + verified stop [count]',ylim=(0,max([r['trials'] for r in rows] or [12])+1))
    for i,row in enumerate(rows):axes[0,0].text(i,row['correct_tasks']+.15,f'{row["correct_tasks"]}/{row["trials"]}',ha='center')
    for i,row in enumerate(rows):
        if row['normalized_cost'] is not None:axes[0,1].bar(i,row['normalized_cost'],color=COLORS[row['controller']])
        else:axes[0,1].text(i,.5,'NOT\nELIGIBLE',ha='center')
    axes[0,1].set(xticks=x,xticklabels=NAMES,xlim=(-.6,2.6),ylim=(0,1.05),ylabel='Normalized efficiency cost, eligible only [lower better]')
    for ax,key,title in [(axes[1,0],'elapsed_sim_s','Completed reachable tasks: simulation duration [s]'),
                         (axes[1,1],'controller_process_cpu_percent','Completed reachable tasks: process CPU [% of one core]')]:
        for index,name in enumerate(NAMES):
            selected=[r for r in stats if r['controller']==name and r['case']!='two_doors']
            vals=[r[key+'_mean'] if r[key+'_mean'] is not None else float('nan') for r in selected]
            errors=[r[key+'_std'] or 0. for r in selected]
            ax.bar(np.arange(3)+(index-1)*.24,vals,width=.24,yerr=errors,capsize=3,label=name,color=COLORS[name])
        ax.set(xticks=np.arange(3),xticklabels=['straight','turn','detour'],ylabel=title);ax.legend();ax.grid(axis='y')
    fig.suptitle(TITLE+'\nMeans ± sample SD where n > 1; failed trials remain in task counts');fig.tight_layout()
    (root/'plots').mkdir(exist_ok=True)
    for ext in ['png','svg']:fig.savefig(root/'plots'/('overview.'+ext),dpi=140,bbox_inches='tight')
    plt.close(fig)


def main():
    """完成全部试验后才给出有条件选型结论，报表不修改默认运行配置。"""
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('root',type=Path)
    args=parser.parse_args();root=args.root.resolve();ws=Path(__file__).resolve().parents[1]
    protocol=json.loads((root/'protocol.json').read_text());progress=json.loads((root/'progress.json').read_text())
    metrics=[run_metrics(root,item,ws) for item in progress if (root/item['run']/'result.json').exists()]
    write_csv(root/'trials.csv',metrics);rows,ranking=summary_rows(metrics,protocol['repeats']);write_csv(root/'summary.csv',rows)
    stats=case_statistics(metrics);write_csv(root/'case_summary.csv',stats)
    complete=len(progress)==len(protocol['order']) and len(metrics)==len(progress)
    winner=ranking[0]['controller'] if complete and ranking and ranking[0]['eligible'] else None
    exploratory=exploratory_candidate(metrics,rows) if complete else None
    result={'complete':complete,'planned_trials':len(protocol['order']),'recorded_trials':len(metrics),
            'recommended_controller':winner,'ranking':ranking,'scope':'CURRENT_PLATFORM_AND_FROZEN_PROFILES_ONLY',
            'exploratory_validation_priority':exploratory,
            'selection_policy':protocol['selection_policy'],'limitations':protocol['limitations']}
    (root/'comparison.json').write_text(json.dumps(result,indent=2));plot_cases(root,metrics,ws);plot_overview(root,rows,stats)
    def fmt(value):
        """缺测明确留空，不把缺失计算成零。"""
        return '缺测' if value is None or not math.isfinite(value) else f'{value:.3f}'
    lines=['# DWB、RPP、MPPI 对比测试报表','',
           f'记录 {len(metrics)}/{len(protocol["order"])} 轮；推荐：**{winner or "尚无满足全部试验要求的控制器"}**。',
           '结论仅适用于当前两轮差速模型、办公室地图、冻结参数和本机负载，不代表算法在所有机器人上的普遍排名。','',
           '| 控制器 | 可达目标完成 | 封堵正确失败停车 | 模型重叠轮次 | 可达任务平均仿真秒 | 平均路程m | CPU%（一个核=100） | 合格后归一化代价↓ |',
           '|---|---:|---:|---:|---:|---:|---:|---:|']
    for row in rows:
        lines.append(f'| {row["controller"]} | {row["reachable_success"]}/{row["reachable_trials"]} | {row["blocked_correct"]}/{row["blocked_trials"]} | {row["model_overlap_trials"]} | {fmt(row["elapsed_sim_s_mean"])} | {fmt(row["travel_m_mean"])} | {fmt(row["controller_process_cpu_percent_mean"])} | {fmt(row["normalized_cost"])} |')
    lines+=['','汇总耗时/路程/CPU均值为成功轮次混合均值，各控制器成功场景数量不同，不能直接据此推断性能差值；比较以分场景表为准。']
    if exploratory and exploratory['candidate']:
        lines+=['',f'**工程候选：{exploratory["candidate"]}，优先作下一步验证；不自动替换默认RPP。** 严格合格者仍为上面的结论。',
          '这是事后探索：先取任务正确次数并列最多且无模型重叠者，再按原权重比较其成功场景均值，失败仍保留；不是冻结协议的正式胜者。探索代价（低者优先）：'+', '.join(f'{k}={v:.3f}' for k,v in exploratory['costs'].items())+'。',
          '该探索代价对权重及min-max归一化敏感，毫米级路程差异也可能占据一个指标权重；需结合分场景原始差值判断，不能只读分值。',
          '本轮没有控制器全部通过，不能宣称已选出经完整验收的最优算法。还需排除启动异常及主机负载影响，扩大路线覆盖和重复数。']
    lines+=['','| 控制器 | 已发目标轮次 | 启动未就绪、未发目标 | 目标墙钟超时 |',
            '|---|---:|---:|---:|']
    for row in rows:lines.append(f'| {row["controller"]} | {row["goals_attempted"]} | {row["startup_no_goal"]} | {row["goal_wall_timeouts"]} |')
    lines+=['','![任务正确率及完成任务效率总览，误差条为样本标准差](plots/overview.png)']
    lines+=['',f'阶段 {protocol["phase"]}，每种控制器正式计划4场景×{protocol["repeats"]}重复，顺序交错、每轮新建独立ROS域及Gazebo分区。直行目标(4.5,0,0)，转弯过门目标(3,2.7,π/2)，中途障碍为(2,0)处0.4×0.5×0.6m箱体，双出口封堵期望ABORTED并持续停车。',
      '三种配置共享10Hz配置控制频率、0.30m/s前进上限、最终0.60rad/s角速度限幅、0.15m/0.20rad到达容差。统一最终角速度由共同平滑器保证：本测试将其上限由默认±0.8降至±0.6，仅改测试副本，不修改生产默认参数；RPP的rotate_to_heading参数本身只管原地转向。NavFn、1Hz重规划、代价地图、同一速度平滑配置、最多4轮恢复和所有保护完全相同。DWB使用ObstacleFootprint，MPPI使用DiffDrive和完整footprint成本；RPP插件参数保留现有配置。','',
      '选型先要求所有计划任务结果正确且停车证据通过，并要求没有二维带padding车体与障碍模型重叠；可达任务按场景等权。合格者再比较场景内min-max归一化代价：仿真时间40%、路径长度25%、角速度总变差/仿真秒20%、控制器进程CPU15%。代价不是成功概率，也不是统计显著性。只有一个合格者时，零代价仅表示无需进一步排序。',
      '耗时与效率表只统计可达且完成的任务；失败不从任务正确率分母删除。封堵失败是正确结果，不能将其混同导航成功。目标完成还检查Gazebo世界终点误差<0.25m、朝向误差<0.30rad与连续停车，防止只看动作SUCCEEDED。','',
      '## 实测明细','',
      '| 控制器 | 场景 | 重复 | 原验证结果 | 任务及停车正确 | 仿真秒 | 墙钟秒 | 路程m | 最小模型间隙m | 终点误差m | 控制超期条数 |',
      '|---|---|---:|---|---|---:|---:|---:|---:|---:|---:|']
    for r in metrics:lines.append(f'| {r["controller"]} | {r["case"]} | {r["repeat"]} | {r["report_result"]} | {r["task_correct_and_stopped"]} | {fmt(r["elapsed_sim_s"])} | {fmt(r["elapsed_wall_s"])} | {fmt(r["travel_m"])} | {fmt(r["min_model_clearance_m"])} | {fmt(r["position_error_m"])} | {r["control_loop_overruns"]} |')
    lines+=['','## 失败及关闭异常','']
    for r in metrics:
        if not r['task_correct_and_stopped'] or r['model_overlap_samples']:
            lines.append(f'- {r["run"]}：分类={r["failure_category"]}，action={r["action_status"]}，失败检查={r["error"]}，无进展日志{r["progress_failures"]}条，模型重叠{r["model_overlap_samples"]}点。详细原因须结合原始日志及轨迹判断。')
    lines.append('STARTUP_NO_GOAL表示未通过启动就绪检查、没有发目标，不能归因为局部控制算法失败；仍保留在计划执行率分母。GOAL_WALL_TIMEOUT表示到达墙钟预算后主动取消，并不等于导航器自行判定无路。')
    escalated=[r['run'] for r in metrics if r['shutdown_escalations']]
    lines.append('关闭阶段存在内部进程退出升级（SIGTERM/SIGKILL）日志的轮次：'+(', '.join(escalated) or '无')+'。这些记录保留并单独报告，不等同任务运行期间失败；每轮关闭后的进程组检查见result.json。')
    lines+=['','## 逐点波形与完整轨迹','',
      'STM32源码v131，P右反0.45→0.45、其它0.30→0.30；I稳态0.010→0.010、启动0.030→0.030，每10ms按电机轴rpm误差累计，D0。PI未改、未运行。改变的是ROS局部控制器；全局算法与保护不变。以下均为本轮ROS/Gazebo实收点，无平滑、补点、删异常；长缺口断线，不是实车左右轮速或电机电流。']
    for case in ['straight','turn','detour','two_doors']:
        lines+=['',f'### {case}','',f'![{case}速度与模型间隙，含失败轮次](plots/{case}_traces.png)','',f'![{case}真实轨迹，含失败轮次](plots/{case}_trajectory.png)']
    lines+=['','## 适用边界','',
      '- 模型间隙由Gazebo世界位姿、配置的带padding矩形与场景二维碰撞几何派生；不是接触传感器，不能据此独立证明无碰撞或实车间隙。',
      '- CPU和RSS采样来自实际safe_controller_server进程，包括本地代价图和DDS，不是纯优化器耗时；超期为真实控制循环日志计数。',
      '- 墙钟时长受用户现有GUI与虚拟机负载影响，同时提供仿真时间、RTF与主机load；不将仿真运行速度当算法运动速度。',
      '- MPPI/AMCL/传感器随机性未固定种子，三次重复只能给出描述性结果，不证明普遍最优或显著差异；没有进行全参数搜索。RPP沿用本项目已有调参，DWB/MPPI按同版官方示例适配，调参投入并不等同。',
      '- 目录日期为开始日期，试验跨2026-10-19—20；观察到系统日历与单调计时相差约3653秒，原因未确认。时间指标使用单调时钟和ROS仿真时间，另存calendar_clock_observation.json；不由UTC相减计算耗时。',
      '- 用户Gazebo/RViz与默认控制器未自动修改，未连接硬件、未实现实车桥，未提交/推送/合并本轮修改。',
      '- 预检与正式试验分开归档。初始配置核查矩阵未继续执行，原因是发现RPP旋转参数不等价于全局限幅；此前两次完整结果及一次关闭阶段主动中断记录均保留，不混入统一最终限幅后的正式36轮。',
      '', '参数参考：[Nav2 1.1.20 DWB官方配置](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_bringup/params/nav2_params.yaml)、[同版MPPI官方说明](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_mppi_controller/README.md)。实际测试以configs内完整YAML及哈希为准。', '', '[冻结协议与参数](protocol.json) · [全部试验CSV](trials.csv) · [分场景均值/标准差](case_summary.csv) · [汇总CSV](summary.csv) · [机器可读结果](comparison.json)']
    (root/'REPORT.md').write_text('\n'.join(lines)+'\n')
    # 单文件HTML内嵌本轮PNG，离开仓库后仍可查看图表；原始CSV/JSON另行提供。
    def embedded(name):
        """嵌入原样绘图文件，不修改图片内容。"""
        path=root/'plots'/name
        return '<img alt="'+html.escape(name)+'" src="data:image/png;base64,'+base64.b64encode(path.read_bytes()).decode()+'">' if path.exists() else ''
    def table(headers,contents):
        """统一转义表格数据，避免把日志文字解释为HTML。"""
        return '<div class="scroll"><table><thead><tr>'+''.join('<th>'+html.escape(str(x))+'</th>' for x in headers)+'</tr></thead><tbody>'+''.join('<tr>'+''.join('<td>'+html.escape(str(x))+'</td>' for x in row)+'</tr>' for row in contents)+'</tbody></table></div>'
    summary_table=table(['控制器','可达目标完成','封堵正确失败停车','模型重叠轮次','平均仿真秒','平均路程m','CPU%','峰值RSS均值MiB','归一化代价↓'],[
        [r['controller'],f'{r["reachable_success"]}/{r["reachable_trials"]}',f'{r["blocked_correct"]}/{r["blocked_trials"]}',r['model_overlap_trials'],
         fmt(r['elapsed_sim_s_mean']),fmt(r['travel_m_mean']),fmt(r['controller_process_cpu_percent_mean']),fmt(r['controller_process_peak_rss_mib_mean']),fmt(r['normalized_cost'])] for r in rows])
    cases_table=table(['场景','控制器','符合预期','仿真秒均值±SD','墙钟秒均值','路程m均值','CPU%均值','RTF均值'],[
        [r['case'],r['controller'],f'{r["correct"]}/{r["n"]}',fmt(r['elapsed_sim_s_mean'])+' ± '+fmt(r['elapsed_sim_s_std']),
         fmt(r['elapsed_wall_s_mean']),fmt(r['travel_m_mean']),fmt(r['controller_process_cpu_percent_mean']),fmt(r['real_time_factor_mean'])] for r in stats])
    trials_table=table(['控制器','场景','重复','验证结果','动作状态','任务及停车','仿真秒','路程m','模型间隙m','错误'],[
        [r['controller'],r['case'],r['repeat'],r['report_result'],r['action_status'],r['task_correct_and_stopped'],
         fmt(r['elapsed_sim_s']),fmt(r['travel_m']),fmt(r['min_model_clearance_m']),r['error'] or ''] for r in metrics])
    page='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>SmartCar 局部控制器对比报表</title><style>
*{box-sizing:border-box}body{font:16px/1.7 system-ui,sans-serif;color:#192c3b;background:#f3f6fa;margin:0}main{max-width:1400px;margin:auto;padding:30px}
header,section{background:white;border:1px solid #dce5ed;border-radius:12px;padding:26px;margin-bottom:20px}h1{font-size:32px;margin:0 0 8px}h2{margin-top:0}
.eyebrow{color:#52667b;font-size:14px}.result{font-size:24px;font-weight:700;color:#125b75}.note{border-left:4px solid #608ba1;background:#eef5f8;padding:12px 18px}
.scroll{overflow:auto}table{border-collapse:collapse;min-width:100%;font-size:14px}td,th{border-bottom:1px solid #dce5ed;padding:11px;text-align:left;white-space:nowrap}th{background:#edf3f7}tbody tr:hover{background:#f2f8fb}
img{display:block;max-width:100%;height:auto;margin:18px auto}details{border-top:1px solid #dce5ed;padding:15px 0}summary{cursor:pointer;font-weight:600}a{color:#125b75}.small{font-size:13px;color:#52667b}
</style><main><header><div class="eyebrow">SmartCar · 两轮差速 · ROS 2 Humble / Nav2 1.1.20 · 2026-10-19—20</div><h1>DWB、RPP、MPPI 对比报表</h1>'''
    page+='<div class="result">当前配置推荐：'+html.escape(winner or '尚无满足全部要求者')+'</div><p>完成 '+str(len(metrics))+'/'+str(len(protocol['order']))+' 轮；三种控制器、四类场景、每组 '+str(protocol['repeats'])+' 次重复。所有正式试验使用相同最终速度边界和保护。</p><div class="note">结论只适用于当前模型、场景、参数和电脑负载。三次重复不证明普遍最优或统计显著性。实车桥尚未实现，本报告没有实车验收结论。</div></header>'
    page+='<section><h2>选型结果</h2>'+summary_table+embedded('overview.png')+'<p>先比较正确任务结果与持续停车，再检查模型重叠；仅全部通过者比较效率。效率代价按场景等权，指标权重：仿真时间40%、路程25%、角速度总变差/仿真秒20%、控制器进程CPU15%。场景内min-max归一化，越低越好；不是成功概率。汇总均值来自成功场景的混合样本，性能差值应查看分场景表。</p></section>'
    if exploratory and exploratory['candidate']:
        page+='<section><h2>工程候选：'+html.escape(exploratory['candidate'])+'</h2><p>建议优先验证'+html.escape(exploratory['candidate'])+'，当前默认RPP保持。严格门槛仍无全部通过者，不能宣称已获得完整验收的最优算法。</p><p>事后探索：在正确任务次数最多且无模型重叠者中，使用原权重比较成功场景均值；失败不删除。探索代价：'+html.escape(json.dumps(exploratory['costs']))+'。这不是冻结协议的正式排名。</p><p>探索代价对权重和归一化敏感，毫米级路程差异也可能影响一个指标的全部权重。需查看分场景原始差值，并排除启动和负载异常后再做上线选型。</p></section>'
    page+='<section><h2>分场景数据</h2>'+cases_table+'<p class="small">效率均值只含完成轮次，失败保留在完成率分母。封堵场景期望失败停车。CPU一个核=100%，包含代价图与DDS；RTF=仿真秒/墙钟秒。</p></section>'
    page+='<section><h2>全部轮次</h2>'+trials_table+'</section><section><h2>逐点波形与轨迹</h2><p>本轮ROS/Gazebo原始采样点，无平滑、补点或删异常；长缺口断线。P右反0.45→0.45，其余0.30→0.30；I稳态0.010→0.010、启动0.030→0.030/10ms，D0，STM32 v131 PI未改、未运行。更换的是局部控制器，NavFn、BT、地图与保护相同。</p>'
    for case in ['straight','turn','detour','two_doors']:page+='<details><summary>'+html.escape(case)+'</summary>'+embedded(case+'_traces.png')+embedded(case+'_trajectory.png')+'</details>'
    page+='</section><section><h2>方法与限制</h2><ul><li>共享最终角速度限幅±0.6rad/s、前进上限0.3m/s；仅测试副本将平滑器默认角速度上限±0.8降到±0.6，生产配置未修改。</li><li>模型间隙是Gazebo世界位姿与带padding二维车体/障碍的几何派生量，不是接触传感器或实车测量。</li><li>AMCL、MPPI与传感器随机性没有固定种子；本机用户GUI继续运行，记录主机负载及RTF。比较的是同一导航栈中的冻结配置，RPP沿用项目已有调参，DWB/MPPI按官方示例适配，未做同等参数搜索。</li><li>测试跨10月19—20日。系统日历与单调计时存在约3653秒差异，原因未确认；本报告耗时使用单调时钟和仿真时间，另存环境异常原始记录。</li><li>初始参数核查、主动中断和正式数据分开保留，正式失败均不删除。没有自动修改运行窗口或默认控制器。</li></ul><p>配套文件：REPORT.md、protocol.json、trials.csv、case_summary.csv、summary.csv、comparison.json。每轮原始数据与哈希在报告归档目录中。</p></section></main></html>'
    (root/'REPORT.html').write_text(page)
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
