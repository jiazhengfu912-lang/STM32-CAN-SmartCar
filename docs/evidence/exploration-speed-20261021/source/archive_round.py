"""只归档已结束的本轮全部尝试，逐文件核对原始哈希，不覆盖旧证据。"""
from pathlib import Path
import hashlib,json,shutil
root=Path.cwd();out=root/'out/exploration-speed-20261021';archive=root/'docs/evidence/exploration-speed-20261021'
runs={
 'before_discovery_r1':'before-discovery-r1','pilot_v1':'after-discovery-r1','pilot_v2':'final-discovery-r1',
 'pilot_v3':'v3-discovery-r1','interrupted_v4':'v4-discovery-r1',
 'after_discovery_r1':'v4-discovery-r1-clean','before_discovery_r2':'before-discovery-r2','after_discovery_r2':'v4-discovery-r2',
 'before_office':'before-office','after_office':'v4-office','watchdog':'watchdog','complete_room':'complete-room'}
for suffix in runs.values():
 if not (root/f'ros2_ws/verification_runs/exploration-speed-20261021-{suffix}/result.json').exists():raise RuntimeError('尚未结束的运行: '+suffix)
archive.mkdir(exist_ok=False)
for name in ['runs','source','plots','logs']:(archive/name).mkdir()
checks=[];attempts={};formal={}
for name,suffix in runs.items():
 src=root/f'ros2_ws/verification_runs/exploration-speed-20261021-{suffix}';dst=archive/'runs'/name
 shutil.copytree(src,dst);result=json.loads((dst/'result.json').read_text());session=result.get('session',{})
 attempts[name]={'result':result.get('result'),'reason':session.get('reason'),'complete':session.get('complete',False),'stop_verified':session.get('stop_verified',False),
                 'classification':'DEVELOPMENT_PILOT' if name.startswith('pilot') else ('ENVIRONMENT_INTERRUPTION' if name.startswith('interrupted') else ('SAFETY_OR_COMPLETION_REGRESSION' if name in ['watchdog','complete_room'] else 'FORMAL_COMPARE')),
                 'shutdown':result.get('shutdown')}
 for rel,digest in result['raw_sha256'].items():
  p=dst/rel;checks.append({'run':name,'file':rel,'match':p.is_file() and hashlib.sha256(p.read_bytes()).hexdigest()==digest})
 if name.startswith(('before_','after_')):
  formal[name]={'world':'source/world_office_loop.sdf' if 'office' in name else 'source/world_discovery_lab.sdf','version':'before' if name.startswith('before') else 'v4','pair':name.replace('before_','').replace('after_','')}
for rel in ['ros2_ws/src/smartcar_slam/scripts/frontier_core.py','ros2_ws/src/smartcar_slam/scripts/explore.py',
            'ros2_ws/src/smartcar_slam/tests/test_frontier_core.py','ros2_ws/tools/verify_exploration.py',
            'ros2_ws/tools/report_exploration_speed.py','ros2_ws/tools/plot_exploration.py',
            'ros2_ws/src/smartcar_nav2/launch/exploration.launch.py','ros2_ws/src/smartcar_nav2/launch/navigation.launch.py']:
 dst=archive/'source'/rel;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(root/rel,dst)
for name in ['discovery_lab','office_loop']:shutil.copy2(root/f'ros2_ws/src/smartcar_sim/worlds/{name}.sdf',archive/'source'/f'world_{name}.sdf')
shutil.copy2(root/'docs/evidence/autonomous-mapping-20261021/source/world_observed_room.sdf',archive/'source/world_complete_room.sdf')
for p in out.glob('*'):
 if not p.is_file():continue
 if p.suffix=='.log':shutil.copy2(p,archive/'logs'/p.name)
 elif p.suffix in ['.json','.py']:shutil.copy2(p,archive/'source'/p.name)
for p in out.iterdir():
 if p.is_dir() and p.name.startswith('map_reload_'):shutil.copytree(p,archive/'diagnostics'/p.name)
(archive/'runs.json').write_text(json.dumps(formal,indent=2))
(archive/'attempts.json').write_text(json.dumps(attempts,indent=2))
(archive/'integrity.json').write_text(json.dumps({'result':'PASS' if all(x['match'] for x in checks) else 'FAIL','raw_files_checked':len(checks),'issues':[x for x in checks if not x['match']]},indent=2))
assert all(x['match'] for x in checks)
print(archive,'raw files checked',len(checks))
