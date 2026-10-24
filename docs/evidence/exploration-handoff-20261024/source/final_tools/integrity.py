"""逐文件核对原始证据、运行源码与前后保护配置，最后生成完整SHA256清单。"""
from pathlib import Path
import hashlib,json,subprocess
root=Path.cwd();e=root/'docs/evidence/exploration-handoff-20261024'

def digest(path):
    """分块读取大日志，避免散列时完整复制进内存。"""
    result=hashlib.sha256()
    with path.open('rb') as stream:
        while block:=stream.read(1024*1024):result.update(block)
    return result.hexdigest()

def verify_manifest(folder,files):
    """缺失和不一致都作为失败保留，不跳过异常文件。"""
    return [name for name,expected in files.items() if not (folder/name).is_file() or digest(folder/name)!=expected]

checks={};bad=[]
protocol=json.loads((e/'source/protocol_v17.json').read_text())
v16=json.loads((e/'source/protocol_v16.json').read_text())
baseline_head=json.loads((e/'source/before_manifest.json').read_text())['head']
for run in sorted((e/'runs').iterdir()):
    result=json.loads((run/'result.json').read_text())
    raw_bad=verify_manifest(run,result['raw_sha256'])
    source_bad=[name for name,expected in result['source_sha256'].items()
                if not (run/'source'/Path(name).name).is_file() or digest(run/'source'/Path(name).name)!=expected]
    artifacts={suffix:(run/'mission'/('map.'+suffix)).is_file() and (run/'mission'/('map.'+suffix)).stat().st_size>0
               for suffix in ['yaml','pgm','data','posegraph']}
    own_clean=all(field in result and not result[field].get('remaining_pids',[]) for field in ['shutdown','pose_probe_shutdown'])
    runtime={}
    if not run.name.startswith('before_'):
        frozen=v16 if '_v16_' in run.name else protocol
        for name,expected in result['source_sha256'].items():
            if name in frozen['runtime_source_sha256']:runtime[name]=expected==frozen['runtime_source_sha256'][name]
    checks[run.name]={'functional_result':result['result'],'raw_files':len(result['raw_sha256']),
        'raw_mismatches':raw_bad,'source_mismatches':source_bad,'four_artifacts_present':artifacts,
        'missing_final_artifacts_recorded_failure':not all(artifacts.values()) and result['result']=='FAIL',
        'owned_groups_cleaned':own_clean,'recorded_runtime_matches_own_version':runtime}
    if raw_bad or source_bad or not own_clean or not all(runtime.values()) or (result['result']=='PASS' and not all(artifacts.values())):bad.append(run.name)
archives={}
for version in ['before','final_v16','final_v17']:
    manifest=json.loads((e/'source'/(version+'_manifest.json')).read_text())
    archives[version]={'files':len(manifest['files']),'mismatches':verify_manifest(e/'source'/(version+'_ws'),manifest['files'])}
unchanged={}
before=e/'source/before_ws'
for name in ['src/smartcar_nav2/config/nav2.yaml','src/smartcar_nav2/config/navigate_recovery.xml',
             'src/smartcar_nav2/config/navigate_through_recovery.xml','src/smartcar_nav2/scripts/gate_core.py',
             'src/smartcar_nav2/scripts/navigation_gate.py','src/smartcar_slam/config/slam.yaml',
             'src/smartcar_slam/config/exploration_slam.yaml']:
    unchanged[name]=digest(before/name)==digest(root/'ros2_ws'/name)
runtime_bad=verify_manifest(root,protocol['runtime_source_sha256'])
untouched={name:subprocess.run(['git','diff','--exit-code',baseline_head,'--',name],stdout=subprocess.DEVNULL).returncode==0
           for name in ['firmware','protocol','desktop']}
conditions={}
for before_name,after_name in protocol['formal_pairs']:
    before_run=e/'runs'/before_name;after_run=e/'runs'/after_name
    b=json.loads((before_run/'result.json').read_text());a=json.loads((after_run/'result.json').read_text())
    conditions[after_name]={'same_world_sdf':digest(before_run/'source/world.sdf')==digest(after_run/'source/world.sdf'),
                            'same_recorded_effective_motion_slam_parameters':bool(b.get('effective_parameters')) and b.get('effective_parameters')==a.get('effective_parameters')}
report={'scope':'File integrity only; functional success and coverage judged separately','runs':checks,
        'baseline_head':baseline_head,'current_head':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        'source_archives':archives,'runtime_mismatches':runtime_bad,'unchanged_navigation_slam_protection':unchanged,
        'unchanged_firmware_protocol_desktop':untouched,'matched_formal_conditions':conditions,'mismatch_runs':bad}
report['matched_build_settings']=json.loads((e/'source/build_conditions_matched.json').read_text())
report['all_checks_pass']=not bad and not runtime_bad and not any(v['mismatches'] for v in archives.values()) and all(unchanged.values()) and all(untouched.values()) and all(all(v.values()) for v in conditions.values()) and report['matched_build_settings']['identical_recorded_build_settings']
(e/'integrity.json').write_text(json.dumps(report,indent=2))
files={str(path.relative_to(e)):digest(path) for path in sorted(e.rglob('*')) if path.is_file() and path!=e/'SHA256.json'}
(e/'SHA256.json').write_text(json.dumps({'algorithm':'SHA256','scope':'All evidence files except SHA256.json itself; includes failed commands and all original runs','files':files},indent=2))
recheck=not verify_manifest(e,files)
print(json.dumps({'all_checks_pass':report['all_checks_pass'],'manifest_recheck':recheck,'files':len(files),'mismatch_runs':bad}))
if not report['all_checks_pass'] or not recheck:raise SystemExit(1)
