"""串行验证恢复修复，路线/观测模块保持已测v14字节不变。"""
from pathlib import Path
import json,hashlib,fcntl,subprocess
root=Path.cwd();e=root/'docs/evidence/exploration-tail-20261022';out=root/'out/exploration-tail-20261022'
files=['explore.py','frontier_core.py','exploration_memory.py','route_core.py','route_planner.py','readiness_core.py']
hashes={str(Path('ros2_ws/src/smartcar_slam/scripts')/name):hashlib.sha256((root/'ros2_ws/src/smartcar_slam/scripts'/name).read_bytes()).hexdigest() for name in files}
protocol={'algorithm':'v15 lifecycle query expiry/explicit resume refresh; v14 observation and route policy unchanged','source_sha256':hashes,'performance_protocol':'source/v14_serial_protocol.json','performance_runs':['final_v14_office','final_v14_discovery_r1','final_v14_discovery_r2'],'recovery_runs':['watchdog_v15_r1','watchdog_v15_r2','complete_v15'],'scope':'v15 recovery/start/save regressions only; full exploration performance measured on frozen v14, not relabeled as v15'}
(e/'source/v15_recovery_protocol.json').write_text(json.dumps(protocol,indent=2));records=[]
with (out/'simulation.lock').open('w') as lock:
 fcntl.flock(lock,fcntl.LOCK_EX)
 for name,case,domain in [('watchdog_v15_r1','watchdog',70),('watchdog_v15_r2','watchdog',71),('complete_v15','complete',72)]:
  if not all(hashlib.sha256((root/p).read_bytes()).hexdigest()==h for p,h in hashes.items()):raise RuntimeError('Frozen recovery source changed')
  world='discovery_lab' if case=='watchdog' else str(root/'docs/evidence/exploration-speed-20261021/runs/complete_room/source/world.sdf')
  command=['/usr/bin/python3','ros2_ws/tools/verify_exploration.py','--output',str(e/'runs'/name),'--case',case,'--world',world,'--domain-id',str(domain),'--max-sim-seconds','180','--timeout','500','--record-pose']
  with (out/(name+'.console.log')).open('w') as stream:result=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT)
  records.append({'name':name,'command':command,'returncode':result.returncode});(out/'recovery_v15_results.json').write_text(json.dumps(records,indent=2));print(name,result.returncode,flush=True)
