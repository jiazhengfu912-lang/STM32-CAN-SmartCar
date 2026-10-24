#!/usr/bin/env python3
"""只读真实探索记录，比较尾段路程、Nav2路径和独立几何可观测区域。"""
import argparse
from collections import deque
import csv
import hashlib
import inspect
import itertools
import json
import math
import re
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from report_exploration_speed import evaluate,reference_points
from plot_exploration import read,line

TITLE=('Exploration algorithm versions as labeled | SIMULATION ONLY | firmware source v131\n'
       'P right reverse .45 -> .45, others .30 -> .30; I steady .010 -> .010, start .030 -> .030 / 10 ms\n'
       'PI NOT exercised; target visibility / regional Nav2 open order; NavFn / RPP and motion limits unchanged')
plt.rcParams['path.simplify']=False
ALGORITHMS={
    'v6':'original gain / estimated travel time + BFS local insertion',
    'v7':'small frontiers + visible gain / actual Nav2 insertion',
    'v8':'async frontier search + viewpoint memory / actual Nav2 insertion',
    'v9':'wider endpoint margin / Nav2 two-hop insertion',
    'v10':'directed Nav2 open visit order / wider endpoint margin',
    'v11':'geodesic regional shortlist / Nav2 open order / wider endpoint margin',
    'v12':'assigned-target visibility / original legal margin + actual stopping approach',
    'v13':'assigned-target visibility / task-aware memory / area gain / regional Nav2 open order',
    'v14':'v13 observation fixes + information completion order within near-minimum Nav2 travel budget',
    'v15':'fresh lifecycle readiness + explicit resume reset; unchanged v14 observation and route policy',
    'v16':'bounded approach frontier preparation / event-driven handoff / per-batch geometry cache'}


def geometry_distance(x,y,geometry):
    """按真实SDF几何计算有符号距离，只用于离线验收，不进入探索节点。"""
    distance=np.full(x.shape,np.inf)
    for kind,p,size in geometry:
        dx=x-p[0];dy=y-p[1]
        if kind=='circle':d=np.hypot(dx,dy)-size
        else:
            c,s=math.cos(p[5]),math.sin(p[5]);u=np.abs(c*dx+s*dy)-size[0]/2;v=np.abs(-s*dx+c*dy)-size[1]/2
            d=np.hypot(np.maximum(u,0),np.maximum(v,0))+np.minimum(np.maximum(u,v),0)
        distance=np.minimum(distance,d)
    return distance


def segment_visible(point,starts,geometry):
    """解析线段与旋转矩形/圆相交，避免用被测探索器的射线实现来验收自身。"""
    visible=np.ones(len(starts),dtype=bool)
    for kind,p,size in geometry:
        a=starts-np.asarray(p[:2]);b=np.asarray(point)-np.asarray(p[:2]);delta=b-a
        if kind=='circle':
            t=np.clip(-(a*delta).sum(axis=1)/np.maximum((delta*delta).sum(axis=1),1.e-15),0.,1.)
            blocked=np.linalg.norm(a+t[:,None]*delta,axis=1)<=size
        else:
            c,s=math.cos(p[5]),math.sin(p[5]);rotation=np.asarray([[c,-s],[s,c]])
            a=a@rotation;b=b@rotation;delta=b-a;half=np.asarray(size[:2])/2
            with np.errstate(divide='ignore',invalid='ignore'):
                t0=(-half-a)/delta;t1=(half-a)/delta
            parallel=np.abs(delta)<1.e-12
            lo=np.where(parallel,-np.inf,np.minimum(t0,t1));hi=np.where(parallel,np.inf,np.maximum(t0,t1))
            outside=(parallel&(np.abs(a)>half)).any(axis=1)
            blocked=(np.maximum(lo.max(axis=1),0.)<=np.minimum(hi.min(axis=1),1.))&~outside
        visible&=~blocked
        if not visible.any():break
    return visible


def observable_reference(world):
    """固定5厘米几何代理：从0.37米连通安全中心，以12米无遮挡线段可见的自由格。"""
    cx,cy,geometry=reference_points(world);centers=np.column_stack((cx,cy));bounds=[]
    for kind,p,size in geometry:
        if kind=='circle':hx=hy=size
        else:
            c,s=abs(math.cos(p[5])),abs(math.sin(p[5]));hx=(c*size[0]+s*size[1])/2;hy=(s*size[0]+c*size[1])/2
        bounds.append((p[0]-hx,p[0]+hx,p[1]-hy,p[1]+hy))
    xs=np.arange(min(p[0] for p in bounds)-.5,max(p[1] for p in bounds)+.5,.05)
    ys=np.arange(min(p[2] for p in bounds)-.5,max(p[3] for p in bounds)+.5,.05)
    x,y=np.meshgrid(xs,ys);distance=geometry_distance(x,y,geometry);free=distance>0.
    start=(int(np.argmin(abs(ys))),int(np.argmin(abs(xs))));connected=np.zeros_like(free);connected[start]=True;queue=deque([start])
    while queue:
        row,col=queue.popleft()
        for nr,nc in [(row-1,col),(row+1,col),(row,col-1),(row,col+1)]:
            if 0<=nr<len(ys) and 0<=nc<len(xs) and free[nr,nc] and not connected[nr,nc]:
                connected[nr,nc]=True;queue.append((nr,nc))
    fx,fy=x[connected],y[connected];observable=np.zeros(len(fx),dtype=bool)
    for index,point in enumerate(zip(fx,fy)):
        ds=((centers-np.asarray(point))**2).sum(axis=1);near=np.flatnonzero(ds<=144.)
        if not len(near):continue
        if ds[near].min()<1.e-10:observable[index]=True;continue
        count=min(32,len(near));closest=near[np.argpartition(ds[near],count-1)[:count]]
        if segment_visible(point,centers[closest],geometry).any():observable[index]=True
        elif len(near)>count and segment_visible(point,centers[near],geometry).any():observable[index]=True
    return fx[observable],fy[observable]


def map_values(data,x,y):
    """逐格读取真实地图，地图以外按缺测记录，不补造自由格。"""
    grid=data['occupancy'];r=float(data['resolution']);o=data['origin']
    ix=np.floor((x-o[0])/r).astype(int);iy=np.floor((y-o[1])/r).astype(int)
    inside=(ix>=0)&(ix<grid.shape[1])&(iy>=0)&(iy<grid.shape[0]);values=np.full(len(x),-1,dtype=np.int16)
    values[inside]=grid[iy[inside],ix[inside]];return values


def save(fig,folder,name):
    """保存全部采样点的PNG和SVG，图上明确PI与算法及仿真边界。"""
    version='v6' if name.startswith('before_') else ('v8' if name.startswith('after_') else None)
    matched=re.search(r'(?:before_|final_|pilot_|watchdog_|complete_)(v\d+)',name)
    if matched:version=matched.group(1)
    title=TITLE
    if version:
        title=(f'Exploration tail {version} | SIMULATION ONLY | firmware source v131\n'+
               TITLE.split('\n')[1]+'\nPI NOT exercised; '+ALGORITHMS[version]+'; NavFn / RPP unchanged')
    fig.suptitle(title,fontsize=9);fig.tight_layout(rect=(0,0,1,.86))
    for suffix in ['png','svg']:fig.savefig(folder/(name+'.'+suffix),dpi=135,bbox_inches='tight')
    plt.close(fig)


def validate_orders(run, queries, folder):
    """用独立穷举复核真实Nav2矩阵的开放顺序，并与同矩阵信息/耗时贪心比较。"""
    mission=run/'mission' if (run/'mission').is_dir() else run
    events=[json.loads(x) for x in (mission/'events.jsonl').read_text().splitlines()]
    results=[]
    for event in events:
        target=event.get('selected') or {}
        if event['event']!='planning_batch_finished' or 'nav2_visit_order' not in target:continue
        rows=[q for q in queries if q['batch']==event['batch']];origin={q['to_index']:q['cost'] for q in rows if q['from_index'] is None and q['cost']}
        edges={(q['from_index'],q['to_index']):q['cost'] for q in rows if q['from_index'] is not None and q['cost']}
        indices=list(origin)
        if len(indices)<2:continue
        def cost(order,key):
            """按原始查询表相加，独立于生产动态规划实现。"""
            return origin[order[0]][key]+sum(edges[(a,b)][key] for a,b in zip(order,order[1:]))
        best=None
        for size in range(len(indices),0,-1):
            feasible=(order for order in itertools.permutations(indices,size)
                      if all((i,j) in edges for i,j in zip(order,order[1:])))
            best=min(feasible,key=lambda order:cost(order,'estimated_cost_s'),default=None)
            if best is not None:break
        selected=target['nav2_visit_order'];candidates=rows[0]['candidates'];greedy=[];remaining=set(indices)
        while remaining:
            last=greedy[-1] if greedy else None
            available=remaining if last is None else {i for i in remaining if (last,i) in edges}
            if not available:break
            chosen=max(available,key=lambda i:max(candidates[i]['predicted_gain_m2'],candidates[i].get('target_visible_gain_m2',0.))/
                       (origin[i] if last is None else edges[(last,i)])['estimated_cost_s'])
            greedy.append(chosen);remaining.remove(chosen)
        selected_cost=cost(selected,'estimated_cost_s');optimum=cost(best,'estimated_cost_s');greedy_cost=cost(greedy,'estimated_cost_s')
        matches=len(selected)==len(best) and abs(selected_cost-optimum)<1.e-6
        objective='minimum_open_travel_cost'
        if target.get('nav2_visit_objective')=='information_completion_with_travel_budget':
            objective=target['nav2_visit_objective']
            def latency(order):
                """独立按各点累计到达耗时乘预计面积，复核生产动态规划目标。"""
                elapsed=origin[order[0]]['estimated_cost_s'];value=0.
                for k,i in enumerate(order):
                    if k:elapsed+=edges[(order[k-1],i)]['estimated_cost_s']
                    gain=max(candidates[i]['predicted_gain_m2'],candidates[i].get('target_visible_gain_m2',candidates[i].get('visible_gain_m2',0.)))
                    value+=gain*elapsed
                return value
            weighted=min((p for p in itertools.permutations(best) if all((i,j) in edges for i,j in zip(p,p[1:]))),key=latency)
            expected=weighted if cost(weighted,'estimated_cost_s')<=optimum+max(2.,.05*optimum) and latency(weighted)<latency(best)*.99 else best
            matches=set(selected)==set(expected) and abs(latency(selected)-latency(expected))<1.e-6 and abs(selected_cost-cost(expected,'estimated_cost_s'))<1.e-6
        results.append({'batch':event['batch'],'views':len(indices),'selected_order':selected,'greedy_order':greedy,
            'selected_cost_s':selected_cost,'enumerated_optimum_s':optimum,'greedy_cost_s':greedy_cost,
            'selected_length_m':cost(selected,'length_m'),'greedy_length_m':cost(greedy,'length_m'),
            'greedy_visits_same_subset':set(selected)==set(greedy),
            'estimated_cost_reduction_percent':100*(1-selected_cost/greedy_cost) if set(selected)==set(greedy) else None,
            'validation_objective':objective,'matches_independent_optimum':matches,
            'scope':'Same fixed candidate set and real Nav2 directed matrix; offline order comparison, not executed time savings'})
    (folder/(run.name+'_order_validation.json')).write_text(json.dumps(results,indent=2))
    comparable=[r['estimated_cost_reduction_percent'] for r in results if r['estimated_cost_reduction_percent'] is not None]
    return {'verified_batches':len(results),'all_match_independent_optimum':all(r['matches_independent_optimum'] for r in results),
        'same_subset_greedy_comparisons':len(comparable),
        'mean_static_estimated_cost_reduction_percent':float(np.mean(comparable)) if comparable else None}


def comparison_plots(root, names, folder, world):
    """正式前后地图和速度使用相同坐标范围，异常点和缺测保持原样。"""
    if len(names)<2:return
    maps=[np.load(root/'runs'/n/'mission/map_final.npz') for n in names]
    bounds=[(d['origin'][0],d['origin'][0]+d['occupancy'].shape[1]*float(d['resolution']),
             d['origin'][1],d['origin'][1]+d['occupancy'].shape[0]*float(d['resolution'])) for d in maps]
    limits=(min(b[0] for b in bounds),max(b[1] for b in bounds),min(b[2] for b in bounds),max(b[3] for b in bounds))
    fig,axes=plt.subplots(1,len(names),figsize=(6*len(names),7),squeeze=False,sharex=True,sharey=True)
    for ax,name,d,b in zip(axes[0],names,maps,bounds):
        g=d['occupancy'];ax.imshow(np.where(g<0,127,np.where(g>20,0,255)),origin='lower',cmap='gray',vmin=0,vmax=255,extent=b)
        probe=read(root/'runs'/name/'pose_probe.csv')
        if probe:ax.plot([float(p['world_x']) for p in probe],[float(p['world_y']) for p in probe],'.-',markersize=.4,linewidth=.35,c='tab:blue')
        ax.set_xlim(limits[:2]);ax.set_ylim(limits[2:]);ax.set_title(name);ax.set_xlabel('Initial x [m], no fitted alignment');ax.set_aspect('equal')
    axes[0,0].set_ylabel('Initial y [m]');save(fig,folder,'map_comparison_'+Path(world).stem)
    fig,axes=plt.subplots(2,len(names),figsize=(6*len(names),7),squeeze=False,sharex=True,sharey='row')
    for col,name in enumerate(names):
        for ax,key in zip(axes[:,col],['vx','wz']):
            for filename,label in [('odom.csv','Independent measured odom'),('commands.csv','Independent safe command')]:
                samples=read(root/'runs'/name/filename);ax.plot(*line(samples,key),'.-',markersize=.5,linewidth=.4,label=label)
            ax.grid();ax.legend();ax.set_title(name)
        axes[1,col].set_xlabel('Monotonic wall seconds; all original samples')
    axes[0,0].set_ylabel('Linear m/s');axes[1,0].set_ylabel('Angular rad/s');save(fig,folder,'velocity_comparison_'+Path(world).stem)


def analyze(run,reference,folder):
    """完整保留成功、失败与部分结束，实际路径和执行里程分开统计。"""
    world=run/'source/world.sdf';summary,timeline=evaluate(run,world);x,y=reference
    snapshots=sorted((run/'map_snapshots').glob('*.npz'));observable=[]
    for p in snapshots:
        d=np.load(p);v=map_values(d,x,y)
        observable.append({'sim_s':float(d['sim_s']),'known_fraction':float((v>=0).mean()),
                           'free_fraction':float(((v>=0)&(v<=20)).mean())})
    final=np.load(run/'mission/map_final.npz');v=map_values(final,x,y)
    summary['observable_proxy']={'reference_cells':len(x),'known_fraction':float((v>=0).mean()),
        'free_fraction':float(((v>=0)&(v<=20)).mean()),'missing_unknown_cells':int((v<0).sum()),
        'missing_unknown_area_m2':float((v<0).sum()*.05**2),
        'scope':'Offline planar SDF line of sight, 5cm free samples, 0.37m connected centers, 12m range; not hardware accuracy or scan-beam completeness'}
    summary['observable_milestones']={}
    for threshold in [.9,.95,.98,.99,.995]:
        index=next((i for i in range(len(observable)-2) if
                    all(p['known_fraction']>=threshold for p in observable[i:i+3])),None)
        summary['observable_milestones'][str(threshold)]=observable[index]['sim_s'] if index is not None else None
    np.savez_compressed(folder/(run.name+'_observable_missing.npz'),x=x[v<0],y=y[v<0])
    odom=read(run/'mission/odom.csv');ts=np.asarray([float(p['sim_s']) for p in odom]);pos=np.asarray([[float(p['x']),float(p['y'])] for p in odom])
    ds=np.linalg.norm(np.diff(pos,axis=0),axis=1);session=json.loads((run/'mission/session.json').read_text())
    dt=np.maximum(0.,np.diff(ts));linear=np.abs([float(p['vx_mps']) for p in odom[:-1]])
    angular=np.abs([float(p['wz_rps']) for p in odom[:-1]])
    summary['tail']={}
    for key,m in summary['milestones'].items():
        if m is None:continue
        summary['tail'][key]={'start_sim_s':m['sim_s'],'remaining_sim_s':timeline[-1]['sim_s']-m['sim_s'],
             'travel_m':float(ds[ts[1:]>=m['sim_s']].sum()),
             'goals_sent':sum(g['sent_sim_s']>=m['sim_s'] for g in session['goals'])}
    summary['observable_tail']={}
    start=summary['observable_milestones']['0.95']
    for threshold in ['0.98','0.99','0.995']:
        end=summary['observable_milestones'][threshold]
        summary['observable_tail'][threshold]=None if start is None or end is None else {
            'from_fraction':.95,'to_fraction':float(threshold),'simulation_seconds':end-start,
            'travel_m':float(ds[(ts[1:]>=start)&(ts[1:]<=end)].sum())}
    summary['observable_intervals']={}
    for first,last in [('.9','.95'),('.9','.98'),('.95','.98'),('.95','.99'),('.98','.99')]:
        start=summary['observable_milestones'][str(float(first))];end=summary['observable_milestones'][str(float(last))]
        summary['observable_intervals'][first+'->'+last]=None if start is None or end is None else {
            'simulation_seconds':end-start,'travel_m':float(ds[(ts[1:]>=start)&(ts[1:]<=end)].sum())}
        if start is not None and end is not None:
            selected=(ts[1:]>=start)&(ts[1:]<=end);valid=selected&(dt<=1.)
            summary['observable_intervals'][first+'->'+last].update({
                'near_stationary_sim_s':float(dt[valid&(linear<.02)&(angular<.05)].sum()),
                'turning_in_place_sim_s':float(dt[valid&(linear<.02)&(angular>=.05)].sum()),
                'translation_sim_s':float(dt[valid&(linear>=.02)].sum()),
                'unclassified_gap_sim_s':float(dt[selected&(dt>1.)].sum())})
    summary['interval_timing_definition']='Raw odom message intervals: stationary |v|<0.02m/s and |w|<0.05rad/s; turn |v|<0.02 and |w|>=0.05; translation |v|>=0.02. Gaps >1 simulation second unclassified; no interpolation.'
    observation_events=[json.loads(s) for s in (run/'mission/events.jsonl').read_text().splitlines()]
    summary['input_stale_pause_recorded']=any(e['event']=='input_stale_paused' for e in observation_events)
    summary['intervention_files']=[p.name for p in run.glob('intervention*.json')]
    summary['run_classification']=('PROTECTED_INTERRUPTED_FINISHED' if summary['input_stale_pause_recorded'] and summary['reason']=='USER_FINISHED_PARTIAL' else
        ('EXPLICITLY_FINISHED_PARTIAL' if summary['reason']=='USER_FINISHED_PARTIAL' else 'NATURAL_TERMINATION'))
    observations=[e for e in observation_events if e['event']=='observation_result' and 'expected_target_cells' in e]
    summary['target_observation']={'recorded_results':len(observations),
        'expected_target_cells':sum(e['expected_target_cells'] for e in observations),
        'observed_target_cells':sum(e['observed_target_cells'] for e in observations),
        'observed_target_free_cells':sum(e['observed_target_free_cells'] for e in observations),
        'zero_target_yield_actions':sum(e['observed_target_cells']==0 for e in observations),
        'scope':'Per-action recorded target-cell counts; repeated cells counted per action, not unique covered area'}
    execution=[];sent=[i for i,e in enumerate(observation_events) if e['event']=='goal_sent']
    for index,(begin,goal) in enumerate(zip(sent,session['goals'])):
        until=sent[index+1] if index+1<len(sent) else len(observation_events)
        end=next((e for e in observation_events[begin:until] if e['event'] in ['goal_result','goal_rejected']),None)
        finish=end['sim_s'] if end is not None else None;planned=goal.get('nav2_path',{})
        execution.append({'goal_index':index,'stage':goal.get('stage',''),'action_status':goal.get('action_status'),
            'sent_sim_s':goal['sent_sim_s'],'finished_sim_s':finish,
            'nav2_planned_length_m':planned.get('length_m'),'nav2_estimated_cost_s':planned.get('estimated_cost_s'),
            'executed_sim_s':finish-goal['sent_sim_s'] if finish is not None else None,
            'executed_odom_m':float(ds[(ts[1:]>=goal['sent_sim_s'])&(ts[1:]<=finish)].sum()) if finish is not None else None,
            'expected_target_cells':len(goal['target_visible_cells']) if 'target_visible_cells' in goal else None,
            'observed_target_cells':goal.get('observed_target_cells'),'observed_target_free_cells':goal.get('observed_target_free_cells')})
    if execution:
        with (folder/(run.name+'_execution_costs.csv')).open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(execution[0]));writer.writeheader();writer.writerows(execution)
    summary['sampling_gaps_wall_s']={}
    for filename in ['odom.csv','commands.csv','mission/odom.csv']:
        times=np.asarray([float(p['wall_s']) for p in read(run/filename)])
        gaps=np.diff(times)
        summary['sampling_gaps_wall_s'][filename]={'samples':len(times),'gaps_over_1s':int((gaps>1.).sum()),
            'maximum_gap_s':float(gaps.max()) if len(gaps) else None}
    summary['zero_predicted_gain_goals']=sum(g['predicted_gain_m2']<=0 for g in session['goals'])
    paths=run/'mission/nav2_paths.jsonl';queries=[json.loads(s) for s in paths.read_text().splitlines()] if paths.exists() else []
    summary['nav2_queries']={'total':len(queries),'valid':sum(p['cost'] is not None for p in queries),
        'pair_queries':sum(p['from_index'] is not None for p in queries)}
    summary['nav2_local_insertions']=sum(g.get('route_reason')=='nav2_local_insertion' for g in session['goals'])
    summary['nav2_order_validation']=validate_orders(run,queries,folder)
    if queries:
        with (folder/(run.name+'_path_costs.csv')).open('w',newline='') as stream:
            fields=['sim_s','batch','from_index','to_index','action_status','nav2_reported_planning_s','bfs_distance_m',
                    'nav2_length_m','nav2_estimated_cost_s','inflation_integral_m']
            writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
            for row in queries:
                cost=row['cost'] or {};target=row['candidates'][row['to_index']]
                writer.writerow({**{k:row[k] for k in fields[:5]},'nav2_reported_planning_s':row['planning_cpu_s'],
                    'bfs_distance_m':target['path_distance_m'] if row['from_index'] is None else '',
                    'nav2_length_m':cost.get('length_m',''),'nav2_estimated_cost_s':cost.get('estimated_cost_s',''),
                    'inflation_integral_m':cost.get('inflation_integral_m','')})
    with (folder/(run.name+'_coverage.csv')).open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=['sim_s','wall_s','observed_fraction','observable_known_fraction','observable_free_fraction']);writer.writeheader()
        for a,b in zip(timeline,observable):writer.writerow({**a,'observable_known_fraction':b['known_fraction'],'observable_free_fraction':b['free_fraction']})
    velocity_plot(run,folder)
    fig,ax=plt.subplots(figsize=(9,7));g=final['occupancy'];r=float(final['resolution']);o=final['origin']
    ax.imshow(np.where(g<0,127,np.where(g>20,0,255)),origin='lower',cmap='gray',vmin=0,vmax=255,
              extent=(o[0],o[0]+g.shape[1]*r,o[1],o[1]+g.shape[0]*r))
    probe=read(run/'pose_probe.csv')
    if probe:ax.plot([float(p['world_x']) for p in probe],[float(p['world_y']) for p in probe],'.-',markersize=.6,linewidth=.4,label='Passive world trajectory')
    if (v<0).any():ax.scatter(x[v<0],y[v<0],s=4,c='red',label='Unknown but geometrically observable')
    ax.set_title(run.name+' | '+str(summary['reason']));ax.set_xlabel('Initial world/map x [m], no fitted alignment');ax.set_ylabel('y [m]');ax.legend();save(fig,folder,run.name+'_map')
    return summary,timeline,observable


def velocity_plot(run, folder):
    """完整绘制独立验收器采样，失败无最终地图时也保留冻结和停止波形。"""
    fig,axes=plt.subplots(2,1,figsize=(11,7),sharex=True)
    for ax,key,label in zip(axes,['vx','wz'],['linear m/s','angular rad/s']):
        independent=read(run/'odom.csv');ax.plot(*line(independent,key),'.-',markersize=.7,linewidth=.5,label='Independent measured odom')
        commands=read(run/'commands.csv');ax.plot(*line(commands,key),'.-',markersize=.6,linewidth=.4,label='Independent safe command')
        ax.set_ylabel(label);ax.grid();ax.legend();ax.set_title(run.name)
    for event in [json.loads(x) for x in (run/'verifier_events.jsonl').read_text().splitlines()]:
        if event['event'] not in ['explorer_frozen','explorer_resumed','service_response']:continue
        label={'explorer_frozen':'SIGSTOP','explorer_resumed':'SIGCONT'}.get(event['event'])
        if label is None:label='FINISH' if event.get('service','').endswith('finish') else ('PAUSE' if event.get('data') else 'EXPLICIT RESUME')
        for ax in axes:ax.axvline(event['wall_s'],linewidth=.6,linestyle='--')
        axes[0].text(event['wall_s'],.98,label,rotation=90,ha='right',va='top',fontsize=7,transform=axes[0].get_xaxis_transform())
    axes[-1].set_xlabel('Monotonic wall seconds, all original samples');save(fig,folder,run.name+'_velocity')


def analyze_failed(run, result, folder):
    """最终地图缺失是失败事实，不用最后快照冒充完整保存结果。"""
    path=run/'mission/session.json';session=json.loads(path.read_text()) if path.exists() else {}
    velocity_plot(run,folder)
    return {'result':result.get('result'),'reason':session.get('reason',result.get('error')),
        'error':result.get('error'),'complete':False,'stop_verified':session.get('stop_verified',False),
        'goals':len(session.get('goals',[])),'travel_m':result.get('travel_m'),'end_sim_s':None,
        'observed_fraction_final':None,'observable_proxy':None,'observable_intervals':{},
        'run_classification':'FAILED_NO_FINAL_MAP','map_artifacts_missing':[str(p) for p in
            ['map_final.npz','map.yaml','map.pgm','map.data','map.posegraph'] if not (run/'mission'/p).is_file()],
        'metric_scope':'Missing final artifacts retained as missing; original CSV and snapshots preserved, no substitute final coverage'}


def main():
    """输出新目录，历史数据只读；不同场景单独比较，不把部分结束当完整完成时间。"""
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);metrics={};groups={};references={}
    cache=a.root/'derived_reference';cache.mkdir(exist_ok=True)
    for run in sorted((a.root/'runs').iterdir()):
        if not (run/'result.json').is_file():continue
        if not (run/'mission/map_final.npz').is_file():
            result=json.loads((run/'result.json').read_text());metrics[run.name]=analyze_failed(run,result,a.output)
            print(run.name,'FAILED_NO_FINAL_MAP',flush=True);continue
        result=json.loads((run/'result.json').read_text());world=result.get('world',{}).get('selection','unknown')
        if world not in references:
            world_file=run/'source/world.sdf'
            key=hashlib.sha256(world_file.read_bytes()+''.join(inspect.getsource(f) for f in
                [observable_reference,segment_visible,geometry_distance,reference_points]).encode()).hexdigest()
            cached=cache/(key+'.npz')
            if cached.exists():
                data=np.load(cached);references[world]=(data['x'],data['y'])
            else:
                references[world]=observable_reference(world_file)
                np.savez_compressed(cached,x=references[world][0],y=references[world][1],
                    world_sha256=hashlib.sha256(world_file.read_bytes()).hexdigest(),algorithm_sha256=key)
        summary,timeline,observable=analyze(run,references[world],a.output);metrics[run.name]=summary
        groups.setdefault(world,[]).append((run.name,timeline,observable))
        print(run.name,{k:summary[k] for k in ['reason','observed_fraction_final','travel_m']},summary['observable_proxy'],flush=True)
    for world,runs in groups.items():
        fig,axes=plt.subplots(2,1,figsize=(11,8),sharex=True)
        for name,timeline,observable in runs:
            axes[0].plot([p['sim_s'] for p in timeline],[p['observed_fraction']*100 for p in timeline],'.-',markersize=.8,linewidth=.5,label=name)
            axes[1].plot([p['sim_s'] for p in observable],[p['known_fraction']*100 for p in observable],'.-',markersize=.8,linewidth=.5,label=name)
        for ax in axes:ax.set_ylim(0,100.5);ax.grid();ax.legend();ax.set_title(world)
        axes[0].set_ylabel('Fixed reachable-center free %');axes[1].set_ylabel('Geometric observable known %');axes[1].set_xlabel('Simulation seconds; task end may be PARTIAL')
        save(fig,a.output,'coverage_'+Path(world).stem)
        formal_file=a.root/'source/v14_serial_protocol.json'
        formal=json.loads(formal_file.read_text()).get('formal_runs',[]) if formal_file.exists() else []
        names=[name for name,_,_ in runs if name.startswith('before_') or name in formal or
               name in ['final_v13_discovery_r1','final_v13_office']]
        if not names:continue
        comparison_plots(a.root,names,a.output,world)
        fig,axes=plt.subplots(2,1,figsize=(11,8),sharex=True)
        for name,_,observable in runs:
            if name not in names:continue
            for ax in axes:ax.plot([p['sim_s'] for p in observable],[p['known_fraction']*100 for p in observable],
                                   '.-',markersize=.8,linewidth=.6,label=name)
        for ax in axes:
            ax.grid();ax.legend();ax.set_title(world);ax.set_ylabel('Geometric observable known %')
            for level in [90,95,98,99]:ax.axhline(level,color='gray',linewidth=.4,linestyle=':')
        axes[0].set_ylim(0,100.5);axes[1].set_ylim(90,100.2)
        axes[1].set_xlabel('Simulation seconds; zoom on final 5-10%, all original map samples')
        save(fig,a.output,'formal_coverage_'+Path(world).stem)
    (a.output/'metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
