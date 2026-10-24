"""归档本轮全部终止尝试及对照，逐文件核验，不覆盖历史证据。"""
from pathlib import Path
import hashlib
import json
import shutil

ROOT=Path.cwd()
OUT=ROOT/'out/exploration-speed-20261021'
ARCHIVE=ROOT/'docs/evidence/exploration-speed-20261021'
RUNS={
    'before_discovery_r1':'before-discovery-r1',
    'before_discovery_r2':'before-discovery-r2',
    'pilot_v1':'after-discovery-r1',
    'pilot_v2':'final-discovery-r1',
    'pilot_v3':'v3-discovery-r1',
    'interrupted_v4':'v4-discovery-r1',
    'pilot_v4_r1':'v4-discovery-r1-clean',
    'pilot_v4_r2':'v4-discovery-r2',
    'pilot_v5':'v5-discovery-r1',
    'after_discovery_r1':'v6-discovery-r1',
    'after_discovery_r2':'v6-discovery-r2',
    'before_office':'before-office',
    'after_office':'v6-office',
    'watchdog':'watchdog',
    'complete_room':'complete-room',
}


def digest(path):
    """计算原始字节哈希，不转换换行或修改输入。"""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    """只有所有预定运行均终止后建立新证据目录，失败仍必须保留。"""
    folders={name:ROOT/f'ros2_ws/verification_runs/exploration-speed-20261021-{suffix}' for name,suffix in RUNS.items()}
    for name,folder in folders.items():
        if not (folder/'result.json').is_file():raise RuntimeError('运行尚未结束: '+name)
    ARCHIVE.mkdir(exist_ok=False)
    for name in ['runs','source','plots','logs','diagnostics']:(ARCHIVE/name).mkdir()
    checks=[];attempts={};formal={}
    for name,src in folders.items():
        dst=ARCHIVE/'runs'/name;shutil.copytree(src,dst)
        result=json.loads((dst/'result.json').read_text());session=result.get('session',{})
        classification=('DEVELOPMENT_PILOT' if name.startswith('pilot') else
                        'ENVIRONMENT_INTERRUPTION' if name.startswith('interrupted') else
                        'SAFETY_OR_COMPLETION_REGRESSION' if name in ['watchdog','complete_room'] else 'FORMAL_COMPARE')
        attempts[name]={'result':result.get('result'),'reason':session.get('reason'),
                        'complete':session.get('complete',False),'stop_verified':session.get('stop_verified',False),
                        'classification':classification,'shutdown':result.get('shutdown')}
        for rel,expected in result['raw_sha256'].items():
            path=dst/rel;checks.append({'run':name,'file':rel,'match':path.is_file() and digest(path)==expected})
        for rel,expected in result.get('source_sha256',{}).items():
            path=dst/'source'/Path(rel).name
            checks.append({'run':name,'file':'source/'+Path(rel).name,'match':path.is_file() and digest(path)==expected})
        if name.startswith(('before_','after_')):
            formal[name]={'world':'source/world_office_loop.sdf' if 'office' in name else 'source/world_discovery_lab.sdf',
                          'version':'before' if name.startswith('before') else 'v6',
                          'pair':name.replace('before_','').replace('after_','')}
    # 保留完整基线源码及最终源文件，构建产物和临时旧工作空间不作为运行依赖。
    shutil.copytree(OUT/'before_ws/src',ARCHIVE/'source/before_ws/src',ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    shutil.copytree(OUT/'before_ws/tools',ARCHIVE/'source/before_ws/tools',ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    shutil.copytree(OUT/'before_ws/maps',ARCHIVE/'source/before_ws/maps')
    for base in ['ros2_ws/src/smartcar_slam','ros2_ws/src/smartcar_nav2']:
        for path in (ROOT/base).rglob('*'):
            if not path.is_file() or '__pycache__' in path.parts:continue
            target=ARCHIVE/'source'/path.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,target)
    for path in (ROOT/'ros2_ws/tools').glob('*'):
        if path.is_file():shutil.copy2(path,ARCHIVE/'source'/path.name)
    for name in ['discovery_lab','office_loop']:
        shutil.copy2(ROOT/f'ros2_ws/src/smartcar_sim/worlds/{name}.sdf',ARCHIVE/'source'/f'world_{name}.sdf')
    shutil.copy2(ROOT/'docs/evidence/autonomous-mapping-20261021/source/world_observed_room.sdf',ARCHIVE/'source/world_complete_room.sdf')
    for path in OUT.iterdir():
        if path.is_file() and path.suffix in ['.json','.py','.yaml','.log','.patch','.txt']:
            shutil.copy2(path,ARCHIVE/('logs' if path.suffix=='.log' else 'source')/path.name)
        elif path.is_dir() and (path.name.startswith('map_reload_') or path.name.startswith('replay_v')):
            shutil.copytree(path,ARCHIVE/'diagnostics'/path.name)
    shutil.copytree(OUT/'replay_probe',ARCHIVE/'diagnostics/replay_probe')
    navigation=ROOT/'ros2_ws/verification_runs/exploration-speed-20261021-new-map-navigation'
    if not (navigation/'result.json').is_file():raise RuntimeError('缺少新地图导航重载记录')
    shutil.copytree(navigation,ARCHIVE/'diagnostics/new-map-navigation')
    for name,value in [('runs.json',formal),('attempts.json',attempts),('integrity.json',{
        'result':'PASS' if all(row['match'] for row in checks) else 'FAIL',
        'raw_and_source_files_checked':len(checks),'issues':[row for row in checks if not row['match']]} )]:
        (ARCHIVE/name).write_text(json.dumps(value,indent=2,ensure_ascii=False))
    assert all(row['match'] for row in checks)
    print(ARCHIVE,'verified files',len(checks))


if __name__=='__main__':main()
