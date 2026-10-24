#!/usr/bin/env python3
"""只读前后探索记录，分离近静止、目标交接和相同覆盖区间的任务吞吐。"""
import argparse
import csv
import hashlib
import inspect
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from plot_exploration import read
from report_exploration_tail import (analyze,analyze_failed,observable_reference,
                                     segment_visible,geometry_distance,reference_points,save)


def dump_rows(path, rows):
    """保存逐段派生记录；原始CSV和JSONL始终只读。"""
    if not rows:return
    with path.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)


def event_phases(events):
    """对新旧版使用相同事件归因；后台预备搜索不把正在导航标成停车规划。"""
    phase='WAITING';active=False;times=[];phases=[]
    for event in events:
        name=event['event']
        if name=='goal_sent':phase='NAVIGATING';active=True
        elif name=='goal_result':phase='OBSERVATION_MAP_WAIT';active=False
        elif name=='goal_rejected':phase='FAILURE_WAIT';active=False
        elif name=='observation_result':phase='FRONTIER_SEARCH'
        elif name=='frontier_search_started' and not active:phase='FRONTIER_SEARCH'
        elif name=='frontier_search' and not active:phase='NAV2_ROUTE_QUERY'
        elif name=='planning_batch_finished' and not active:phase='DISPATCH_WAIT'
        elif name=='navigation_wait' and not active:phase='READINESS_WAIT'
        elif name in ['input_stale_paused','external_stop_paused','user_paused','external_cancel_paused']:phase='PAUSED'
        elif name=='user_resumed_new_goal_required':phase='RESUMING'
        elif name=='resume_confirmed':phase='WAITING'
        elif name=='mission_stopping':phase='STOPPING'
        elif name=='stop_verified':phase='SAVING'
        elif name=='map_saved':phase='DONE'
        times.append(event['sim_s']);phases.append(phase)
    return np.asarray(times),np.asarray(phases)


def timing(run, folder, coverage):
    """按原始里程计区间积分；超过1仿真秒或时间倒退的区间不补点、不推断静止。"""
    mission=run/'mission';events=[json.loads(row) for row in (mission/'events.jsonl').read_text().splitlines()]
    rows=read(mission/'odom.csv')
    if len(rows)<2:return {'missing_odom':True}
    times=np.asarray([float(row['sim_s']) for row in rows]);dt=np.diff(times)
    velocity=np.asarray([float(row['vx_mps']) for row in rows[:-1]])
    angular=np.asarray([float(row['wz_rps']) for row in rows[:-1]])
    position=np.asarray([[float(row['x']),float(row['y'])] for row in rows]);distance=np.linalg.norm(np.diff(position,axis=0),axis=1)
    valid=(dt>=0)&(dt<=1.);low=(abs(velocity)<.02)&(abs(angular)<.05)
    event_times,event_phase=event_phases(events);indices=np.searchsorted(event_times,times[:-1],side='right')-1
    phases=event_phase[np.maximum(indices,0)]
    sent=[event for event in events if event['event']=='goal_sent'];handoffs=[]
    for first,last in zip(sent,sent[1:]):
        result=next((event for event in events if event['event'] in ['goal_result','goal_rejected'] and
                     first['sim_s']<=event['sim_s']<last['sim_s']),None)
        if result is None:continue
        start,end=result['sim_s'],last['sim_s'];mask=(times[:-1]>=start)&(times[1:]<=end)
        after=[event for event in events if start<=event['sim_s']<=end]
        def when(name):
            """保留原始事件的仿真时刻，缺少阶段事件时记录空值。"""
            return next((event['sim_s'] for event in after if event['event']==name),None)
        observation,search,planning=[when(name) for name in ['observation_result','frontier_search','planning_batch_finished']]
        handoffs.append({'handoff_index':len(handoffs),'from_goal_result_sim_s':start,'next_goal_sent_sim_s':end,
            'result_status':result.get('status',6),'gap_sim_s':end-start,
            'near_stationary_sim_s':float(dt[mask&valid&low].sum()),
            'observation_wait_sim_s':observation-start if observation is not None else None,
            'post_observation_to_frontier_event_sim_s':search-observation if search is not None and observation is not None else None,
            'frontier_event_to_planning_finished_sim_s':planning-search if planning is not None and search is not None else None,
            'planning_finished_to_send_sim_s':end-planning if planning is not None else None})
    dump_rows(folder/(run.name+'_handoffs.csv'),handoffs)
    first=sent[0]['sim_s'] if sent else None
    stop=next((event['sim_s'] for event in events if event['event']=='mission_stopping'),None)
    intervals={'task':(first,stop)}
    milestones=coverage.get('observable_milestones',{})
    for lower,upper in [(.9,.95),(.9,.98),(.95,.98),(.98,.99)]:
        intervals[f'{lower}->{upper}']=(milestones.get(str(lower)),milestones.get(str(upper)))
    results={}
    for label,(start,end) in intervals.items():
        if start is None or end is None:results[label]=None;continue
        mask=(times[:-1]>=start)&(times[1:]<=end);good=mask&valid
        phase_seconds={phase:float(dt[good&low&(phases==phase)].sum()) for phase in sorted(set(phases[good&low]))}
        bouts=[];begin=None
        for i,selected in enumerate(mask&valid&low):
            if selected and begin is None:begin=i
            if begin is not None and (not selected or i==len(dt)-1):
                finish=i if not selected else i+1;duration=float(dt[begin:finish].sum())
                if duration>=.5:bouts.append(duration)
                begin=None
        result={'start_sim_s':start,'end_sim_s':end,'simulation_seconds':end-start,
                'near_stationary_sim_s':float(dt[good&low].sum()),
                'turning_in_place_sim_s':float(dt[good&(abs(velocity)<.02)&~low].sum()),
                'translation_sim_s':float(dt[good&(abs(velocity)>=.02)].sum()),
                'unclassified_gap_sim_s':float(np.maximum(dt[mask&~valid],0.).sum()),
                'travel_m':float(distance[good].sum()),'near_stationary_bouts_ge_0_5s':len(bouts),
                'near_stationary_by_event_phase_sim_s':phase_seconds,
                'goals_sent':sum(start<=event['sim_s']<=end for event in sent)}
        if label!='task':
            lower,upper=map(float,label.split('->'));cells=coverage['observable_proxy']['reference_cells']
            result['normalized_reference_area_m2_per_sim_s']=(upper-lower)*cells*.05**2/(end-start) if end>start else None
        results[label]=result
    dump_rows(folder/(run.name+'_odom_intervals.csv'),[
        {'start_sim_s':float(times[i]),'end_sim_s':float(times[i+1]),'dt_sim_s':float(dt[i]),
         'vx_mps':float(velocity[i]),'wz_rps':float(angular[i]),'valid_interval':bool(valid[i]),
         'near_stationary':bool(low[i]),'event_phase':str(phases[i])} for i in range(len(dt))])
    prepared={name:sum(event['event']==name for event in events) for name in
              ['frontier_preparation_used','frontier_preparation_discarded','frontier_preparation_empty_recheck',
               'frontier_preparation_start_unsafe_recheck','frontier_preparation_observed_recheck',
               'frontier_preparation_filtered_recheck','observation_already_current']}
    pipeline=read(mission/'pipeline.csv');jitter=[float(row['tick_interval_wall_s']) for row in pipeline if row.get('tick_interval_wall_s')]
    return {'intervals':results,'handoffs':{'count':len(handoffs),'total_gap_sim_s':sum(row['gap_sim_s'] for row in handoffs),
            'median_gap_sim_s':float(np.median([row['gap_sim_s'] for row in handoffs])) if handoffs else None,
            'total_near_stationary_sim_s':sum(row['near_stationary_sim_s'] for row in handoffs)},
            'preparation':prepared,'pipeline_samples':len(pipeline),'maximum_tick_interval_wall_s':max(jitter) if jitter else None,
            'definition':'Raw odom intervals <=1 sim s. Near-stationary |v|<0.02 m/s and |w|<0.05 rad/s. Event phases use identical inference for both versions; phase association is not CPU attribution. Task excludes startup and final save. Throughput is fixed reference area between thresholds / elapsed simulation time; not exact newly scanned beam area. Bouts are kinematic intervals, not proof of commanded stops.'}


def comparison(root, pair, summaries, output):
    """用相同坐标尺度绘制全部新旧原始速度与逐次交接，不删停顿或异常点。"""
    names=pair;fig,axes=plt.subplots(2,2,figsize=(14,7),sharex=True,sharey='row')
    for column,name in enumerate(names):
        mission=root/'runs'/name/'mission';odom=read(mission/'odom.csv')
        measured=np.asarray([float(row['sim_s']) for row in odom])
        # 同图命令保持独立墙钟波形在各轮velocity图；这里只用有明确仿真时刻的真实里程计。
        for row,(key,label) in enumerate([('vx_mps','linear m/s'),('wz_rps','angular rad/s')]):
            values=np.asarray([float(item[key]) for item in odom]);delta=np.diff(measured)
            cut=np.flatnonzero((delta>1.)|(delta<0.))+1
            for segment in np.split(np.arange(len(measured)),cut):
                axes[row,column].plot(measured[segment],values[segment],'.-',markersize=.6,linewidth=.4)
            axes[row,column].set_title(name);axes[row,column].set_ylabel(label);axes[row,column].grid()
            axes[row,column].set_xlabel('Simulation seconds; all raw odom samples, gaps broken')
        gaps=read(output/(name+'_handoffs.csv'))
        for gap in gaps:
            for ax in axes[:,column]:ax.axvspan(float(gap['from_goal_result_sim_s']),float(gap['next_goal_sent_sim_s']),color='orange',alpha=.16)
    version=names[-1].split('_')[1]
    save(fig,output,'final_'+version+'_handoff_wave_'+names[-1].split('_')[-1])
    fig,axes=plt.subplots(2,1,figsize=(11,7))
    for name in names:
        gaps=read(output/(name+'_handoffs.csv'))
        axes[0].plot([int(row['handoff_index']) for row in gaps],[float(row['gap_sim_s']) for row in gaps],'.-',linewidth=.6,label=name)
        timeline=read(output/(name+'_coverage.csv'))
        axes[1].plot([float(row['sim_s']) for row in timeline],[float(row['observable_known_fraction'])*100 for row in timeline],'.-',markersize=.8,linewidth=.5,label=name)
    axes[0].set_ylabel('Result -> next goal seconds');axes[0].set_xlabel('Handoff index; targets differ across runs')
    axes[1].set_ylabel('Geometric observable known %');axes[1].set_xlabel('Simulation seconds');axes[1].set_ylim(90,100.2)
    for ax in axes:ax.grid();ax.legend()
    save(fig,output,'final_'+version+'_handoff_coverage_'+names[-1].split('_')[-1])


def main():
    """只在新的输出目录生成派生报表，缓存参考必须绑定同一几何与算法哈希。"""
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--reference-cache',type=Path);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    summaries={};references={};cache=a.root/'derived_reference';cache.mkdir(exist_ok=True)
    for run in sorted((a.root/'runs').iterdir()):
        if not (run/'result.json').is_file():continue
        raw=json.loads((run/'result.json').read_text())
        if not (run/'mission/map_final.npz').is_file():summaries[run.name]=analyze_failed(run,raw,a.output);continue
        world=run/'source/world.sdf';key=hashlib.sha256(world.read_bytes()+''.join(inspect.getsource(f) for f in
            [observable_reference,segment_visible,geometry_distance,reference_points]).encode()).hexdigest()
        if key not in references:
            cached=cache/(key+'.npz');prior=a.reference_cache/(key+'.npz') if a.reference_cache else None
            if not cached.exists() and prior and prior.exists():cached.write_bytes(prior.read_bytes())
            if cached.exists():
                data=np.load(cached)
                if str(data['world_sha256'])!=hashlib.sha256(world.read_bytes()).hexdigest():raise ValueError('Cached geometry mismatch')
                references[key]=(data['x'],data['y'])
            else:
                references[key]=observable_reference(world)
                np.savez_compressed(cached,x=references[key][0],y=references[key][1],world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),algorithm_sha256=key)
        summary,_,_=analyze(run,references[key],a.output)
        summary['handoff_timing']=timing(run,a.output,summary);summaries[run.name]=summary
        print(run.name,summary['reason'],summary['handoff_timing']['handoffs'],flush=True)
    protocol_path=a.root/'source/protocol_v17.json'
    if not protocol_path.exists():protocol_path=a.root/'source/protocol_v16.json'
    protocol=json.loads(protocol_path.read_text())
    for pair in protocol['formal_pairs']+protocol.get('diagnostic_pairs',[]):
        if all(name in summaries and summaries[name].get('observable_proxy') for name in pair):comparison(a.root,pair,summaries,a.output)
    (a.output/'metrics.json').write_text(json.dumps(summaries,indent=2,ensure_ascii=False))


if __name__=='__main__':main()
