from pathlib import Path
import json,subprocess,time
root=Path.cwd()
record=root/'docs/evidence/exploration-tail-20261022/runs/after_discovery_r1/result.json'
while not record.exists():time.sleep(5)
command=['/usr/bin/python3','ros2_ws/tools/verify_exploration.py','--output','docs/evidence/exploration-tail-20261022/runs/after_office','--world','office_loop','--max-sim-seconds','900','--timeout','2600','--domain-id','156','--record-pose']
with (root/'out/exploration-tail-20261022/after_office.console.log').open('w') as stream:
    result=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT)
(root/'out/exploration-tail-20261022/office_runner_result.json').write_text(json.dumps({'command':command,'returncode':result.returncode},indent=2))
