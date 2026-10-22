"""归档本轮全部真实尝试和最终源码；只复制，不重写旧结果或历史证据。"""
from pathlib import Path
import datetime,hashlib,json,shutil,subprocess
repo=Path.cwd();runtime=repo/'ros2_ws/verification_runs';root=repo/'docs/evidence/autonomous-mapping-20261021'
root.mkdir(exist_ok=False);(root/'runs').mkdir();(root/'logs').mkdir();(root/'source').mkdir()
names=['baseline','room-01','room-02','room-03','office-01','office-02','watchdog-01','watchdog-02','watchdog-03','watchdog-04','complete-room','map-reload']
for name in names:
 source=runtime/('exploration-20261021-'+name)
 if source.exists():shutil.copytree(source,root/'runs'/name.replace('-','_'),ignore=shutil.ignore_patterns('__pycache__'))
for source in runtime.glob('exploration-20261021*.log'):shutil.copy2(source,root/'logs'/source.name)
for name in ['smartcar','office_loop']:
 shutil.copy2(repo/'ros2_ws/src/smartcar_sim/worlds'/(name+'.sdf'),root/'source'/('world_'+name+'.sdf'))
shutil.copy2(runtime/'exploration-20261021-observed-room.sdf',root/'source/world_observed_room.sdf')
if Path('/tmp/exploration_pose_probe.py').exists():shutil.copy2('/tmp/exploration_pose_probe.py',root/'source/pose_probe_office01.py')
paths=['ros2_ws/tools/verify_exploration.py','ros2_ws/tools/plot_exploration.py','ros2_ws/tools/verify_navigation.py',
 'ros2_ws/src/smartcar_slam/scripts/explore.py','ros2_ws/src/smartcar_slam/scripts/frontier_core.py',
 'ros2_ws/src/smartcar_slam/tests/test_frontier_core.py','ros2_ws/src/smartcar_slam/CMakeLists.txt','ros2_ws/src/smartcar_slam/package.xml',
 'ros2_ws/src/smartcar_slam/config/slam.yaml','ros2_ws/src/smartcar_slam/launch/slam.launch.py',
 'ros2_ws/src/smartcar_nav2/launch/navigation.launch.py','ros2_ws/src/smartcar_nav2/launch/exploration.launch.py',
 'ros2_ws/src/smartcar_nav2/scripts/navigation_gate.py','ros2_ws/src/smartcar_nav2/scripts/gate_core.py',
 'ros2_ws/src/smartcar_nav2/tests/test_gate_core.py','ros2_ws/src/smartcar_nav2/config/nav2.yaml',
 'ros2_ws/src/smartcar_sim/config/robot_parameters.json','ros2_ws/src/smartcar_sim/config/bridge.yaml',
 'ros2_ws/src/smartcar_sim/urdf/smartcar.urdf','ros2_ws/src/smartcar_sim/models/smartcar/model_lidar.sdf','docs/AUTONOMOUS_MAPPING.md']
for name in paths:
 target=root/'source_current'/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(repo/name,target)
shutil.copy2(Path(__file__),root/'archive_exploration_20261021.py')
checks=0;issues=[];rows=[]
for folder in (root/'runs').iterdir():
 if not (folder/'result.json').exists():continue
 result=json.loads((folder/'result.json').read_text())
 for name,want in result.get('raw_sha256',{}).items():
  checks+=1
  if hashlib.sha256((folder/name).read_bytes()).hexdigest()!=want:issues.append(str((folder/name).relative_to(root)))
 session=result.get('session',{})
 rows.append({'run':folder.name,'result':result['result'],'reason':session.get('reason') or result.get('error'),
   'goals':len(session.get('goals',[])),'succeeded':sum(g['action_status']==4 for g in session.get('goals',[])),
   'stop_verified':session.get('stop_verified') or result.get('checks',{}).get('odom_and_command_stopped'),
   'travel_m':result.get('travel_m',result.get('odom_displacement_m')),
   'complete':session.get('complete',False),'remaining_pids':result.get('shutdown',{}).get('remaining_pids')})
(root/'trials.json').write_text(json.dumps(rows,indent=2,ensure_ascii=False))
(root/'integrity.json').write_text(json.dumps({'result':'PASS' if not issues else 'FAIL','raw_files_checked':checks,'issues':issues},indent=2))
(root/'provenance.json').write_text(json.dumps({'archived_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
 'head':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
 'branch':subprocess.check_output(['git','branch','--show-current'],text=True).strip(),
 'firmware_protocol_desktop_unchanged':subprocess.run(['git','diff','--quiet','HEAD','--','firmware','protocol','desktop']).returncode==0,
 'navigation_default_yaml_unchanged':subprocess.run(['git','diff','--quiet','HEAD','--','ros2_ws/src/smartcar_nav2/config/nav2.yaml']).returncode==0,
 'slam_default_yaml_unchanged':subprocess.run(['git','diff','--quiet','HEAD','--','ros2_ws/src/smartcar_slam/config/slam.yaml']).returncode==0,
 'world_models_unchanged':subprocess.run(['git','diff','--quiet','HEAD','--','ros2_ws/src/smartcar_sim/worlds','ros2_ws/src/smartcar_sim/models']).returncode==0,
 'scope':'SIMULATION_ONLY; no hardware; no manual velocity or exploration goals from verifier; no commit/push/merge',
 'source_policy':'Per-trial source snapshots identify each running variant. source_current contains final export-server version; office_02 validates final exploration motion and map geometry, watchdog_04 and complete_room validate the subsequent export-service correction.'},indent=2))
print(json.dumps({'runs':len(rows),'raw_checked':checks,'issues':issues,'root':str(root)}))
