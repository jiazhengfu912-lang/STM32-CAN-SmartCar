"""只读核对全部原始证据及冻结源码，然后生成全目录SHA清单。"""
from pathlib import Path
import json,hashlib,subprocess,datetime
root=Path.cwd();e=root/'docs/evidence/exploration-tail-20261022'
def digest(path):
    """逐块散列大文件，避免完整读入重复候选路径日志。"""
    h=hashlib.sha256()
    with path.open('rb') as stream:
        while data:=stream.read(1024*1024):h.update(data)
    return h.hexdigest()
checks={};mismatches=[]
for run in sorted((e/'runs').iterdir()):
    p=run/'result.json'
    if not p.exists():raise RuntimeError('Unfinished run: '+str(run))
    r=json.loads(p.read_text());bad=[]
    for name,expected in r.get('raw_sha256',{}).items():
        target=run/name
        if not target.exists() or digest(target)!=expected:bad.append(name)
    source_bad=[]
    for name,expected in r.get('source_sha256',{}).items():
        target=run/'source'/Path(name).name
        if not target.exists() or digest(target)!=expected:source_bad.append(name)
    four={name:(run/'mission'/('map.'+name)).is_file() and (run/'mission'/('map.'+name)).stat().st_size>0 for name in ['yaml','pgm','data','posegraph']}
    shutdown_ok=all(not r.get(key,{}).get('remaining_pids',[]) for key in ['shutdown','pose_probe_shutdown'])
    checks[run.name]={'functional_result':r.get('result'),'raw_files_verified':len(r.get('raw_sha256',{})),
        'raw_hash_mismatches':bad,'source_files_verified':len(r.get('source_sha256',{})),
        'source_hash_mismatches':source_bad,'four_map_artifacts_present':four,'four_required_for_pass':r.get('result')=='PASS',
        'missing_final_map_is_recorded_failure':not all(four.values()) and r.get('result')=='FAIL','owned_groups_cleaned':shutdown_ok}
    if bad or source_bad or (r.get('result')=='PASS' and not all(four.values())) or not shutdown_ok:mismatches.append(run.name)
protocol=json.loads((e/'source/v14_serial_protocol.json').read_text());recovery=json.loads((e/'source/v15_recovery_protocol.json').read_text())
runtime={p:digest(root/p)==h for p,h in recovery['source_sha256'].items()}
formal_source={}
for name in protocol['formal_runs']+['watchdog_v14','complete_v14']:
    r=json.loads((e/'runs'/name/'result.json').read_text())
    formal_source[name]=all(r['source_sha256'].get(p)==h for p,h in protocol['source_sha256'].items())
recovery_source={}
for name in recovery['recovery_runs']:
    r=json.loads((e/'runs'/name/'result.json').read_text());recovery_source[name]=all(r['source_sha256'].get(p)==h for p,h in recovery['source_sha256'].items())
unchanged_route={p:digest(root/p)==h for p,h in protocol['source_sha256'].items() if Path(p).name in ['frontier_core.py','exploration_memory.py','route_core.py','route_planner.py']}
baseline=json.loads((e/'runs/before_discovery/result.json').read_text())
def source_hash_by_relative_path(record,key):
    """按包内相对路径匹配冻结源码，避免工作区目录前缀造成误判。"""
    relative=key.split('/src/',1)[1]
    matches=[value for path,value in record['source_sha256'].items() if path.endswith('/src/'+relative)]
    if len(matches)!=1:raise RuntimeError('Source path is missing or ambiguous: '+key)
    return matches[0]
unchanged_params={}
for name in protocol['formal_runs']:
    r=json.loads((e/'runs'/name/'result.json').read_text())
    unchanged_params[name]={key:source_hash_by_relative_path(r,key)==source_hash_by_relative_path(baseline,key)==digest(root/key) for key in
       ['ros2_ws/src/smartcar_nav2/config/nav2.yaml','ros2_ws/src/smartcar_slam/config/slam.yaml','ros2_ws/src/smartcar_slam/config/exploration_slam.yaml','ros2_ws/src/smartcar_nav2/scripts/gate_core.py','ros2_ws/src/smartcar_nav2/scripts/navigation_gate.py']}
report={'recorded_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'scope':'Raw evidence integrity, not functional or coverage success',
        'runs':checks,'source_runtime_matches_v15_freeze':runtime,'performance_runs_match_v14_freeze':formal_source,
        'recovery_runs_match_v15_freeze':recovery_source,'v15_route_observation_modules_identical_to_v14':unchanged_route,
        'unchanged_protection_navigation_slam_sources':unchanged_params,'raw_mismatch_runs':mismatches,
        'unchanged_firmware_protocol_desktop':{p:subprocess.run(['git','diff','--exit-code','HEAD','--',p],stdout=subprocess.DEVNULL).returncode==0 for p in ['firmware','protocol','desktop']}}
archives={}
for version in ['v13','v14','v15']:
    manifest=json.loads((e/'source'/('final_'+version+'_source_manifest.json')).read_text())
    folder=e/'source'/('final_'+version+'_ws');bad=[]
    for name,expected in manifest['files'].items():
        path=folder/name
        if not path.is_file() or digest(path)!=expected:bad.append(name)
    archives[version]={'files_verified':len(manifest['files']),'mismatches':bad}
report['full_source_archives']=archives
report['all_integrity_checks_pass']=not mismatches and all(runtime.values()) and all(formal_source.values()) and all(recovery_source.values()) and all(unchanged_route.values()) and all(all(v.values()) for v in unchanged_params.values()) and all(report['unchanged_firmware_protocol_desktop'].values()) and not any(v['mismatches'] for v in archives.values())
(e/'integrity.json').write_text(json.dumps(report,indent=2))
files={str(p.relative_to(e)):digest(p) for p in sorted(e.rglob('*')) if p.is_file() and p!=e/'SHA256.json'}
(e/'SHA256.json').write_text(json.dumps({'algorithm':'SHA256','scope':'All files except SHA256.json itself; includes original failures and derived figures','files':files},indent=2))
verified=all(digest(e/p)==h for p,h in files.items())
print(json.dumps({'integrity':report['all_integrity_checks_pass'],'raw_mismatch_runs':mismatches,'files':len(files),'full_manifest_recheck':verified}))
if not report['all_integrity_checks_pass'] or not verified:raise SystemExit(1)
