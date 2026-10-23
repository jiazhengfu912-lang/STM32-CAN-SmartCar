"""只读当前已收地图与里程计，输出三地图确认的阶段性同阈值成本。"""
from pathlib import Path
import sys,json,hashlib,csv
import numpy as np
root=Path('docs/evidence/exploration-tail-20261022');name=sys.argv[1];run=root/'runs'/name
world_hash=hashlib.sha256((run/'source/world.sdf').read_bytes()).hexdigest()
references=[np.load(p) for p in (root/'derived_reference').glob('*.npz')]
ref=next(d for d in references if str(d['world_sha256'])==world_hash);x,y=ref['x'],ref['y'];timeline=[]
for p in sorted((run/'map_snapshots').glob('*.npz')):
 d=np.load(p);g=d['occupancy'];r=float(d['resolution']);ix=np.floor((x-d['origin'][0])/r).astype(int);iy=np.floor((y-d['origin'][1])/r).astype(int);inside=(ix>=0)&(iy>=0)&(ix<g.shape[1])&(iy<g.shape[0]);v=np.full(len(x),-1);v[inside]=g[iy[inside],ix[inside]];timeline.append((float(d['sim_s']),float((v>=0).mean())))
marks={str(t):next((timeline[i][0] for i in range(len(timeline)-2) if all(row[1]>=t for row in timeline[i:i+3])),None) for t in [.9,.95,.98,.99,.995]}
rows=list(csv.DictReader((run/'mission/odom.csv').open()));ts=np.asarray([float(p['sim_s']) for p in rows]);pos=np.asarray([[float(p['x']),float(p['y'])] for p in rows]);ds=np.linalg.norm(np.diff(pos,axis=0),axis=1);first,last=marks['0.95'],marks['0.98']
result={'scope':'INTERIM only; final report recomputes full completed data','run':name,'milestones':marks,'tail95_to98':None if first is None or last is None else {'sim_s':last-first,'odom_m':float(ds[(ts[1:]>=first)&(ts[1:]<=last)].sum())}}
(root/'diagnostics'/(name+'_interim_milestones.json')).write_text(json.dumps(result,indent=2));print(json.dumps(result))
