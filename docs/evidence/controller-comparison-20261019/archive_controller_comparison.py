"""原样归档全部试验，JSONL仅无损压缩；不改写历史证据。"""
from pathlib import Path
import datetime,gzip,hashlib,json,shutil,subprocess
repo=Path.cwd();runs=repo/'ros2_ws/verification_runs';out=repo/'docs/evidence/controller-comparison-20261019'
out.mkdir(exist_ok=False)
compression=[]
for name,source in [('formal','controller-comparison-20261019-formal-v2'),('pilot','controller-comparison-20261019-pilot'),('method_precheck','controller-comparison-20261019-formal')]:
    shutil.copytree(runs/source,out/name,ignore=shutil.ignore_patterns('__pycache__'))
    for path in (out/name).rglob('*.jsonl'):
        if path.stat().st_size<131072:continue
        raw=path.read_bytes();target=path.with_suffix('.jsonl.gz')
        with target.open('wb') as stream:
            with gzip.GzipFile(filename='',mode='wb',fileobj=stream,compresslevel=1,mtime=0) as gz:gz.write(raw)
        assert gzip.decompress(target.read_bytes())==raw
        compression.append({'original_path':str(path.relative_to(out)),'original_bytes':len(raw),
          'original_sha256':hashlib.sha256(raw).hexdigest(),'gzip_path':str(target.relative_to(out)),
          'gzip_bytes':target.stat().st_size,'gzip_sha256':hashlib.sha256(target.read_bytes()).hexdigest()})
        path.unlink()
logs=out/'logs';logs.mkdir()
for p in runs.glob('controller-comparison-20261019*.log'):shutil.copy2(p,logs/p.name)
source=out/'source_current';source.mkdir()
for name in ['ros2_ws/tools/benchmark_controllers.py','ros2_ws/tools/verify_replanning.py','ros2_ws/tools/report_controllers.py',
             'ros2_ws/src/smartcar_nav2/tests/test_controller_benchmark.py','ros2_ws/src/smartcar_nav2/CMakeLists.txt',
             'ros2_ws/src/smartcar_nav2/package.xml','docs/CONTROLLER_BENCHMARK.md']:
    target=source/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(repo/name,target)
shutil.copy2(Path(__file__),out/'archive_controller_comparison.py')
(out/'compression.json').write_text(json.dumps({'method':'lossless gzip, level1, mtime0; original ignored runtime files retained','files':compression},indent=2))
(out/'provenance.json').write_text(json.dumps({'archived_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
 'head':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
 'branch':subprocess.check_output(['git','branch','--show-current'],text=True).strip(),
 'default_config_sha256':hashlib.sha256((repo/'ros2_ws/src/smartcar_nav2/config/nav2.yaml').read_bytes()).hexdigest(),
 'firmware_protocol_desktop_unchanged_vs_head':subprocess.run(['git','diff','--quiet','HEAD','--','firmware','protocol','desktop']).returncode==0,
 'scope':'SIMULATION_ONLY; production default RPP not switched; no commit/push/merge; no hardware',
 'formal_trials':36,'pilot_excluded_from_ranking':True,'method_precheck_excluded_from_ranking':True},indent=2))
issues=[];checked=0;reports=0
for p in out.rglob('result.json'):
    r=json.loads(p.read_text());reports+=1
    for name,want in r.get('raw_sha256',{}).items():
        path=p.parent/name
        data=path.read_bytes() if path.exists() else gzip.decompress(path.with_suffix(path.suffix+'.gz').read_bytes())
        checked+=1
        if hashlib.sha256(data).hexdigest()!=want:issues.append(str(path.relative_to(out)))
(out/'integrity.json').write_text(json.dumps({'result':'PASS' if not issues else 'FAIL','raw_files_checked':checked,
 'completed_result_reports':reports,'issues':issues,'compression_roundtrip_verified':True},indent=2))
print(json.dumps({'archive':str(out),'compressed_files':len(compression),'raw_checked':checked,'issues':issues}))
