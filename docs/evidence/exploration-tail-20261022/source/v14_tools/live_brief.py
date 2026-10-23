from pathlib import Path
import json,csv
root=Path('docs/evidence/exploration-tail-20261022')
for run in sorted((root/'runs').iterdir()):
 if 'v13' not in run.name:continue
 result=run/'result.json';session=run/'mission/session.json'
 if result.exists():
  r=json.loads(result.read_text());s=json.loads(session.read_text()) if session.exists() else {}
  print(run.name,r.get('result'),s.get('reason'),s.get('complete'),'goals',len(s.get('goals',[])),'m',r.get('travel_m'))
 elif session.exists():
  s=json.loads(session.read_text());rows=list(csv.DictReader((run/'maps.csv').open()));events=(run/'mission/events.jsonl').read_text().splitlines()
  last=json.loads(events[-1]) if events else {};print(run.name,s.get('state'),s.get('reason'),'goals',len(s.get('goals',[])),'last',last.get('event'),'sim',last.get('sim_s'),'maps',rows[-1] if rows else {})
import numpy as np,hashlib
for run in sorted((root/'runs').glob('*v13*')):
 if (run/'result.json').exists() or not (run/'source/world.sdf').exists():continue
 files=sorted((run/'map_snapshots').glob('*.npz'))
 if not files:continue
 world_hash=hashlib.sha256((run/'source/world.sdf').read_bytes()).hexdigest()
 for cache in (root/'derived_reference').glob('*.npz'):
  ref=np.load(cache)
  if str(ref['world_sha256'])!=world_hash:continue
  d=np.load(files[-1]);x,y=ref['x'],ref['y'];ix=np.floor((x-d['origin'][0])/float(d['resolution'])).astype(int);iy=np.floor((y-d['origin'][1])/float(d['resolution'])).astype(int);g=d['occupancy'];inside=(ix>=0)&(iy>=0)&(ix<g.shape[1])&(iy<g.shape[0]);v=np.full(len(ix),-1);v[inside]=g[iy[inside],ix[inside]]
  print(run.name,'live proxy known/free %',round(100*(v>=0).mean(),3),round(100*((v>=0)&(v<=20)).mean(),3),'unknown m2',round((v<0).sum()*.0025,4))
