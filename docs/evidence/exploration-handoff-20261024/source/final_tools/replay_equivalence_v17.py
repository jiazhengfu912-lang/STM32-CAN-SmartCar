"""用归档源码和真实快照复现候选等价检查；结果必须写入新目录。"""
from pathlib import Path
import argparse,csv,hashlib,importlib.util,json,time
import numpy as np


def module(name,path):
    """独立加载对应冻结实现，不读取当前可能已变更的工作区模块。"""
    spec=importlib.util.spec_from_file_location(name,path);value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value);return value


def main():
    """对相同真实地图、原点和起点逐项比较全部候选与诊断，计时仅限离线回放。"""
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    root=next(p for p in Path(__file__).resolve().parents if (p/'AGENTS.md').is_file())
    evidence=root/'docs/evidence/exploration-handoff-20261024';args.output.mkdir(parents=True,exist_ok=False)
    implementations={version:module('replay_'+version,evidence/'source'/folder/'src/smartcar_slam/scripts/frontier_core.py')
                     for version,folder in [('before','before_ws'),('v17','final_v17_ws')]};rows=[]
    for run in ['before_v15_discovery','before_v15_office','final_v16_discovery']:
        mission=evidence/'runs'/run/'mission';events=[json.loads(s) for s in (mission/'events.jsonl').read_text().splitlines()]
        snapshots=sorted((mission/'planning_maps').glob('*.npz'))
        for index in np.linspace(0,len(snapshots)-1,4,dtype=int):
            path=snapshots[index];data=np.load(path);sim=float(data['sim_s'])
            event=next(v for v in reversed(events) if v['event']=='frontier_search' and v['sim_s']<=sim);values={}
            for version,implementation in implementations.items():
                wall=time.perf_counter();cpu=time.process_time()
                result=implementation.select_frontiers(data['occupancy'],float(data['resolution']),data['origin'],event['robot'])
                values[version+'_cpu_s']=time.process_time()-cpu;values[version+'_wall_s']=time.perf_counter()-wall
                values[version+'_result_sha256']=hashlib.sha256(json.dumps(result,sort_keys=True).encode()).hexdigest()
            same=values['before_result_sha256']==values['v17_result_sha256']
            rows.append({'run':run,'snapshot':path.name,'input_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
                         'identical_candidates_and_details':same,**values})
    (args.output/'equivalence.json').write_text(json.dumps(rows,indent=2))
    with (args.output/'equivalence.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    if not all(row['identical_candidates_and_details'] for row in rows):raise SystemExit(1)


if __name__=='__main__':main()
