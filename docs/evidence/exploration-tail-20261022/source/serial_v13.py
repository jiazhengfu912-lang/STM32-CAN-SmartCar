from pathlib import Path
import fcntl,hashlib,json,subprocess,time,signal
root=Path.cwd();e=root/'docs/evidence/exploration-tail-20261022';out=root/'out/exploration-tail-20261022'
manifest=json.loads((e/'source/v13_serial_protocol.json').read_text())['source_sha256'];records=[];active=None

def interrupted(signum,frame):
    """终止队列时先让当前验收器清理自己的仿真进程组。"""
    raise KeyboardInterrupt()

signal.signal(signal.SIGTERM,interrupted)

def run(name,command):
    """源码一致且上一轮已完成清理，才允许启动下一轮。"""
    global active
    for file,digest in manifest.items():
        if hashlib.sha256((root/file).read_bytes()).hexdigest()!=digest:raise RuntimeError('Source changed: '+file)
    unfinished=[str(p.parent.parent) for p in (e/'runs').glob('*/mission/session.json') if not (p.parent.parent/'result.json').exists()]
    if unfinished:raise RuntimeError('Previous run unfinished: '+str(unfinished))
    with (out/(name+'.console.log')).open('w') as stream:
        active=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT)
        code=active.wait();active=None
    records.append({'name':name,'command':command,'returncode':code})
    (out/'serial_v13_results.json').write_text(json.dumps(records,indent=2))
    print(name,'exit',code,flush=True)
    return code

with (out/'simulation.lock').open('w') as lock:
    fcntl.flock(lock,fcntl.LOCK_EX)
    try:
        time.sleep(5)
        if run('planner_v13',['/usr/bin/python3','ros2_ws/tools/verify_exploration_planner.py','--run','docs/evidence/exploration-speed-20261021/runs/after_discovery_r2','--output',str(e/'diagnostics/planner_v13'),'--domain-id','219'])!=0:raise RuntimeError('Static actual Nav2 planner check failed')
        for name,world,domain in [('final_v13_discovery_r1','discovery_lab',220),('final_v13_office','office_loop',221),('final_v13_discovery_r2','discovery_lab',222)]:
            run(name,['/usr/bin/python3','ros2_ws/tools/verify_exploration.py','--output',str(e/'runs'/name),'--world',world,'--max-sim-seconds','900','--timeout','2600','--domain-id',str(domain),'--record-pose'])
        run('watchdog_v13',['/usr/bin/python3','ros2_ws/tools/verify_exploration.py','--output',str(e/'runs/watchdog_v13'),'--world','discovery_lab','--case','watchdog','--max-sim-seconds','180','--timeout','600','--domain-id','223','--record-pose'])
        observed=root/'docs/evidence/exploration-speed-20261021/runs/complete_room/source/world.sdf'
        run('complete_v13',['/usr/bin/python3','ros2_ws/tools/verify_exploration.py','--output',str(e/'runs/complete_v13'),'--world',str(observed),'--case','complete','--max-sim-seconds','120','--timeout','400','--domain-id','224','--record-pose'])
        for index,name in enumerate(['final_v13_discovery_r1','final_v13_discovery_r2','final_v13_office']):
            mapfile=e/'runs'/name/'mission/map.yaml'
            if mapfile.is_file():run('reload_'+name,['/usr/bin/python3','ros2_ws/tools/verify_map_export.py','--map-file',str(mapfile),'--output',str(e/'diagnostics'/('reload_'+name)),'--domain-id',str(225+index)])
        run('new_map_navigation_v13',['/usr/bin/python3','ros2_ws/tools/verify_navigation.py','--output',str(e/'diagnostics/new_map_navigation_v13'),'--world','discovery_lab','--map-file',str(e/'runs/final_v13_discovery_r1/mission/map.yaml'),'--domain-id','228','--evidence-label','Exploration tail 20261022 v13'])
    except BaseException:
        if active is not None:
            active.send_signal(signal.SIGINT)
            try:active.wait(timeout=30)
            except subprocess.TimeoutExpired:active.terminate()
        raise
