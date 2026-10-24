"""独立重建前沿分组，核对真实选中批次是否包含指定前沿的可见格。"""
from collections import deque
from pathlib import Path
import json,hashlib
import numpy as np
root=Path.cwd();e=root/'docs/evidence/exploration-tail-20261022';rows=[]
for run in sorted((e/'runs').glob('*v13*')):
    path=run/'mission/events.jsonl'
    if not path.exists():continue
    for line in path.read_text().splitlines():
        event=json.loads(line);selected=event.get('selected')
        if event['event']!='planning_batch_finished' or not selected:continue
        file=run/'mission/planning_maps'/f"batch_{event['batch']:04d}.npz";d=np.load(file);g=d['occupancy'];h,w=g.shape
        free=(g>=0)&(g<=20);unknown=g<0;adj=np.zeros_like(free)
        adj[1:]|=unknown[:-1];adj[:-1]|=unknown[1:];adj[:,1:]|=unknown[:,:-1];adj[:,:-1]|=unknown[:,1:]
        frontier=free&adj;seen=set();groups=[]
        for y,x in zip(*np.nonzero(frontier)):
            if (y,x) in seen:continue
            todo=deque([(y,x)]);seen.add((y,x));group=[]
            while todo:
                yy,xx=todo.popleft();group.append((yy,xx))
                for dy in [-1,0,1]:
                    for dx in [-1,0,1]:
                        q=(yy+dy,xx+dx)
                        if 0<=q[0]<h and 0<=q[1]<w and frontier[q] and q not in seen:seen.add(q);todo.append(q)
            groups.append(group)
        wanted=set()
        for y,x in groups[selected['frontier_group']]:
            for dy,dx in [(1,0),(-1,0),(0,1),(0,-1)]:
                yy,xx=y+dy,x+dx
                if 0<=yy<h and 0<=xx<w and unknown[yy,xx]:wanted.add(yy*w+xx)
        visible=set(selected['visible_cells']);target=set(selected.get('target_visible_cells',[]))
        rows.append({'run':run.name,'batch':event['batch'],'assigned_frontier_group':selected['frontier_group'],
            'visible_assigned_cells':len(wanted&visible),'target_cells':len(target),
            'target_metadata_matches_assigned_group':bool(target) and target<=wanted and target<=visible,
            'planning_map_sha256':hashlib.sha256(file.read_bytes()).hexdigest()})
result={'scope':'Independent connected-component/adjacency audit of recorded selected visible-cell metadata; actual observation yield is separate',
        'selected_batches':len(rows),'zero_assigned_visible_batches':sum(r['visible_assigned_cells']==0 for r in rows),
        'all_target_metadata_matches_assigned_group':all(r['target_metadata_matches_assigned_group'] for r in rows),'rows':rows}
(e/'diagnostics/v13_selected_target_audit.json').write_text(json.dumps(result,indent=2));print({k:v for k,v in result.items() if k!='rows'})
