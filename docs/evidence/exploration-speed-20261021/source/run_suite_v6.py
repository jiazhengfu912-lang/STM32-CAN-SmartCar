"""顺序运行本轮正式比较和保护回归；保留每轮结果，质量不合格时停止后续发布验证。"""
from pathlib import Path
import subprocess,json,os,sys,hashlib,datetime
root=Path.cwd();out=root/'out/exploration-speed-20261021';ws=root/'ros2_ws'
sys.path.insert(0,str(ws/'tools'))
from report_exploration_speed import evaluate
frozen=json.loads((out/'final_v6_source.json').read_text())
state={'status':'RUNNING','pid':os.getpid(),'started_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'runs':[]}
plan=[
 ('after_discovery_r1','v6-discovery-r1','autonomous','discovery_lab',202,False,'before-discovery-r1'),
 ('after_discovery_r2','v6-discovery-r2','autonomous','discovery_lab',203,False,'before-discovery-r2'),
 ('before_office','before-office','autonomous','office_loop',204,True,None),
 ('after_office','v6-office','autonomous','office_loop',205,False,'before-office'),
 ('watchdog','watchdog','watchdog','discovery_lab',206,False,None),
 ('complete_room','complete-room','complete',str(root/'docs/evidence/autonomous-mapping-20261021/source/world_observed_room.sdf'),207,False,None)]

def record():
    """原子更新试验索引，实时状态不能替代实际进程和各轮result.json。"""
    p=out/'suite_v6_progress.tmp';p.write_text(json.dumps(state,indent=2));p.replace(out/'suite_v6_progress.json')

for name,suffix,case,world,domain,before,paired in plan:
    if not all(hashlib.sha256((root/k).read_bytes()).hexdigest()==v for k,v in frozen.items()):
        state.update(status='SOURCE_CHANGED',active=None);record();raise SystemExit(3)
    folder=ws/f'verification_runs/exploration-speed-20261021-{suffix}'
    if folder.exists():raise FileExistsError(folder)
    script=(out/'before_ws/tools/verify_exploration.py') if before else ws/'tools/verify_exploration.py'
    command=['python3',str(script),'--case',case,'--world',world,'--domain-id',str(domain),'--max-sim-seconds','900','--record-pose','--timeout','3000','--output',str(folder)]
    state['active']={'name':name,'folder':str(folder),'command':command};record()
    with (out/(name+'_v6_suite.log')).open('w') as stream:
        child=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT)
        state['active']['pid']=child.pid;record();code=child.wait()
    result=json.loads((folder/'result.json').read_text()) if (folder/'result.json').exists() else {}
    entry={'name':name,'suffix':suffix,'returncode':code,'runtime_result':result.get('result'),'folder':str(folder)}
    if case=='autonomous':
        world_path=ws/'src/smartcar_sim/worlds'/(world+'.sdf')
        metrics,_=evaluate(folder,world_path);(out/(name+'_v6_metrics.json')).write_text(json.dumps(metrics,indent=2))
        entry['metrics']=metrics
        if paired:
            old,_=evaluate(ws/f'verification_runs/exploration-speed-20261021-{paired}',world_path)
            q=metrics['map_geometry_proxy'] or {}
            checks={'coverage95_reached':metrics['milestones']['0.95'] is not None,
                    'final_coverage':metrics['observed_fraction_final'] is not None and metrics['observed_fraction_final']>=max(.95,(old['observed_fraction_final'] or 0)-.005),
                    'map_p95':q.get('surface_distance_p95_m',999)<=.15,
                    'map_support':q.get('occupied_support_fraction_within_015m',0)>=.95,
                    'stopped_and_saved':metrics['stop_verified'] and result.get('checks',{}).get('finished_and_saved',False)}
            entry['quality_gate']=checks
    state['runs'].append(entry);state['active']=None;record()
    if code!=0 or result.get('result')!='PASS':
        state['status']='STOPPED_RUNTIME_FAILURE';record();raise SystemExit(1)
    if entry.get('quality_gate') and not all(entry['quality_gate'].values()):
        state['status']='STOPPED_QUALITY_GATE';record();raise SystemExit(2)
state.update(status='COMPLETE',ended_utc=datetime.datetime.now(datetime.timezone.utc).isoformat());record()
print(json.dumps(state,indent=2))
