"""串行前后对比；运行中校验冻结源码，任何失败都保留并停止队列。"""
from pathlib import Path
import json,hashlib,subprocess,time,signal
root=Path.cwd();e=root/'docs/evidence/exploration-handoff-20261024';out=root/'out/exploration-handoff-20261024'
protocol=json.loads((e/'source/protocol_v17.json').read_text());records=[];active=None

def interrupted(signum,frame):
    """外部终止先让验收器清理本轮自己的进程组。"""
    raise KeyboardInterrupt()

def run(name,world,domain,case='autonomous',before=False):
    """检查运行源码和上一轮清理，再以独立输出、ROS域和Gazebo分区执行。"""
    global active
    if not before:
        for path,expected in protocol['runtime_source_sha256'].items():
            if hashlib.sha256((root/path).read_bytes()).hexdigest()!=expected:raise RuntimeError('Frozen source changed: '+path)
    ws=out/'before_ws' if before else root/'ros2_ws'
    command=['/usr/bin/python3',str(ws/'tools/verify_exploration.py'),'--output',str(e/'runs'/name),
             '--world',str(world),'--domain-id',str(domain),'--case',case,'--max-sim-seconds',
             '900' if case=='autonomous' else '180','--timeout','2600' if case=='autonomous' else '600','--record-pose']
    with (out/(name+'.console.log')).open('w') as stream:
        active=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT);code=active.wait();active=None
    records.append({'name':name,'command':command,'returncode':code});(out/'serial_v17_results.json').write_text(json.dumps(records,indent=2))
    print(name,code,flush=True)
    result=json.loads((e/'runs'/name/'result.json').read_text())
    if code or result['result']!='PASS':raise RuntimeError('Recorded failed run: '+name)
    if result.get('shutdown',{}).get('remaining_pids'):raise RuntimeError('Owned processes not cleaned')

signal.signal(signal.SIGTERM,interrupted)
try:
    deadline=time.monotonic()+3600
    first=e/'runs/before_v15_office/result.json'
    while not first.is_file():
        if time.monotonic()>deadline:raise TimeoutError('Initial baseline did not finish')
        time.sleep(5)
    initial=json.loads(first.read_text())
    if initial['result']!='PASS' or initial.get('shutdown',{}).get('remaining_pids'):raise RuntimeError('Initial baseline failed or unclean')
    baseline=e/'runs/before_v15_discovery/result.json'
    if baseline.exists():
        result=json.loads(baseline.read_text())
        if result['result']!='PASS' or result.get('shutdown',{}).get('remaining_pids'):raise RuntimeError('Recorded lab baseline failed or unclean')
    else:run('before_v15_discovery','discovery_lab',82,before=True)
    run('final_v17_discovery','discovery_lab',88)
    run('final_v17_office','office_loop',89)
    run('watchdog_v17','discovery_lab',90,case='watchdog')
    room=root/'docs/evidence/exploration-speed-20261021/runs/complete_room/source/world.sdf'
    run('complete_v17',room,91,case='complete')
except BaseException:
    if active is not None:
        active.send_signal(signal.SIGINT)
        try:active.wait(timeout=30)
        except subprocess.TimeoutExpired:active.terminate()
    raise
