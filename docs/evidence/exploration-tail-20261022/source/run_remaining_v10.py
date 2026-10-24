from pathlib import Path
import hashlib,json,subprocess,time,signal
root=Path.cwd();e=root/'docs/evidence/exploration-tail-20261022';out=root/'out/exploration-tail-20261022'
manifest=json.loads((e/'source/v10_protocol.json').read_text())['source_sha256'];records=[];active=None

def run(name,command):
    global active
    for file,digest in manifest.items():
        if hashlib.sha256((root/file).read_bytes()).hexdigest()!=digest:raise RuntimeError('Code changed, stopping queued v10 runs: '+file)
    with (out/(name+'.console.log')).open('w') as stream:
        active=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT)
        code=active.wait();active=None
    records.append({'name':name,'command':command,'returncode':code})
    (out/'remaining_v10_results.json').write_text(json.dumps(records,indent=2))
    print(name,'exit',code,flush=True)

try:
    while not (e/'runs/final_v10_discovery_r1/result.json').exists():time.sleep(5)
    for name,world,domain in [('final_v10_office','office_loop',158),('final_v10_discovery_r2','discovery_lab',159)]:
        run(name,['/usr/bin/python3','ros2_ws/tools/verify_exploration.py','--output',str(e/'runs'/name),'--world',world,'--max-sim-seconds','900','--timeout','2600','--domain-id',str(domain),'--record-pose'])
    run('watchdog_v10',['/usr/bin/python3','ros2_ws/tools/verify_exploration.py','--output',str(e/'runs/watchdog_v10'),'--world','discovery_lab','--case','watchdog','--max-sim-seconds','180','--timeout','600','--domain-id','160','--record-pose'])
    observed=root/'docs/evidence/exploration-speed-20261021/runs/complete_room/source/world.sdf'
    run('complete_v10',['/usr/bin/python3','ros2_ws/tools/verify_exploration.py','--output',str(e/'runs/complete_v10'),'--world',str(observed),'--case','complete','--max-sim-seconds','120','--timeout','400','--domain-id','162','--record-pose'])
    for index,name in enumerate(['final_v10_discovery_r1','final_v10_discovery_r2','final_v10_office']):
        mapfile=e/'runs'/name/'mission/map.yaml'
        if mapfile.is_file():run('reload_'+name,['/usr/bin/python3','ros2_ws/tools/verify_map_export.py','--map-file',str(mapfile),'--output',str(e/'diagnostics'/('reload_'+name)),'--domain-id',str(170+index)])
    run('new_map_navigation',['/usr/bin/python3','ros2_ws/tools/verify_navigation.py','--output',str(e/'diagnostics/new_map_navigation'),'--world','discovery_lab','--map-file',str(e/'runs/final_v10_discovery_r1/mission/map.yaml'),'--domain-id','173','--evidence-label','Exploration tail 20261022 v10'])
except BaseException:
    if active is not None:
        active.send_signal(signal.SIGINT)
        try:active.wait(timeout=30)
        except subprocess.TimeoutExpired:active.terminate()
    raise
