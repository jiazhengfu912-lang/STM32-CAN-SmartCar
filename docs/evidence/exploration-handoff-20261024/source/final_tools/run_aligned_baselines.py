"""补齐相同编译优化等级的冻结v15基线，原始不匹配轮次不覆盖、不删除。"""
from pathlib import Path
import hashlib,json,signal,subprocess,time
root=Path.cwd();e=root/'docs/evidence/exploration-handoff-20261024';out=root/'out/exploration-handoff-20261024';active=None;records=[]

def interrupted(signum,frame):
    """外部终止先交给验收器清理自己拥有的仿真进程。"""
    raise KeyboardInterrupt()

signal.signal(signal.SIGTERM,interrupted)
try:
    deadline=time.monotonic()+4000
    while not (e/'runs/complete_v17/result.json').is_file():
        if time.monotonic()>deadline:raise TimeoutError('v17 motion queue incomplete')
        time.sleep(5)
    for name in ['final_v17_discovery','final_v17_office','watchdog_v17','complete_v17']:
        result=json.loads((e/'runs'/name/'result.json').read_text())
        if result['result']!='PASS' or result['shutdown']['remaining_pids'] or result['pose_probe_shutdown']['remaining_pids']:raise RuntimeError('v17 queue failed or unclean: '+name)
    before=out/'before_ws'
    for name,expected in json.loads((e/'source/before_manifest.json').read_text())['files'].items():
        if hashlib.sha256((before/name).read_bytes()).hexdigest()!=expected:raise RuntimeError('Frozen baseline source changed: '+name)
    with (e/'software/build_before_matched.log').open('w') as stream:
        command=['bash','-c','source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo']
        result=subprocess.run(command,cwd=before,stdout=stream,stderr=subprocess.STDOUT)
    if result.returncode:raise RuntimeError('Aligned baseline build failed')
    fields=['CMAKE_BUILD_TYPE','CMAKE_CXX_COMPILER','CMAKE_CXX_FLAGS','CMAKE_CXX_FLAGS_RELWITHDEBINFO','BUILD_TESTING'];conditions={}
    for label,folder in [('matched_v15',before),('v17',root/'ros2_ws')]:
        lines=(folder/'build/smartcar_nav2/CMakeCache.txt').read_text().splitlines();conditions[label]={line.split(':',1)[0]:line.split('=',1)[1] for line in lines if '=' in line and line.split(':',1)[0] in fields}
    conditions['identical_recorded_build_settings']=conditions['matched_v15']==conditions['v17']
    (e/'source/build_conditions_matched.json').write_text(json.dumps(conditions,indent=2))
    if not conditions['identical_recorded_build_settings']:raise RuntimeError('Build settings still differ')
    for name,world,domain in [('before_v15_matched_discovery','discovery_lab',94),('before_v15_matched_office','office_loop',95)]:
        command=['/usr/bin/python3',str(before/'tools/verify_exploration.py'),'--output',str(e/'runs'/name),
                 '--world',world,'--domain-id',str(domain),'--max-sim-seconds','900','--timeout','2600','--record-pose']
        with (out/(name+'.console.log')).open('w') as stream:
            active=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT);code=active.wait();active=None
        records.append({'name':name,'command':command,'returncode':code});(out/'aligned_baseline_results.json').write_text(json.dumps(records,indent=2));print(name,code,flush=True)
        result=json.loads((e/'runs'/name/'result.json').read_text())
        if code or result['result']!='PASS' or result['shutdown']['remaining_pids']:raise RuntimeError('Recorded baseline failure: '+name)
except BaseException:
    if active is not None:
        active.send_signal(signal.SIGINT)
        try:active.wait(timeout=30)
        except subprocess.TimeoutExpired:active.terminate()
    raise
