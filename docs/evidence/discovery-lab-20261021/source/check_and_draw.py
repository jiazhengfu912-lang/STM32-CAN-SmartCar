"""离线检查新场景碰撞及通道连通性，并绘制SDF示意；不向ROS发布地图。"""
from pathlib import Path
import sys
import json
import hashlib
import importlib.util
from collections import deque
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle

root=Path.cwd()
sys.path.insert(0,str(root/'ros2_ws/tools'))
from plot_exploration import obstacles
path=root/'ros2_ws/src/smartcar_sim/worlds/discovery_lab.sdf'
world=ET.parse(path).find('world')
geometry=obstacles(path)
models=world.findall('model')
checks={'single_robot_placeholder':len(world.findall("include[uri='__MODEL_SDF__']"))==1,
        'unique_names':len({m.get('name') for m in models})==len(models),
        'static_models':all(m.findtext('static')=='true' for m in models),
        'collision_matches_visual':all(ET.tostring(m.find('./link/collision/geometry')).strip()==ET.tostring(m.find('./link/visual/geometry')).strip() for m in models)}
xx=np.arange(-4.95,5,.05);yy=np.arange(-3.95,4,.05);x,y=np.meshgrid(xx,yy);distance=np.full(x.shape,np.inf)
for shape,pose,size in geometry:
    dx=x-pose[0];dy=y-pose[1]
    if shape=='circle':d=np.hypot(dx,dy)-size
    else:
        c=np.cos(pose[5]);s=np.sin(pose[5]);u=np.abs(c*dx+s*dy)-size[0]/2;v=np.abs(-s*dx+c*dy)-size[1]/2
        d=np.hypot(np.maximum(u,0),np.maximum(v,0))+np.minimum(np.maximum(u,v),0)
    distance=np.minimum(distance,d)
safe=distance>=.4

def cell(point):
    """检查点只用于离线场景连通性检验，绝不交给探索器做目标。"""
    return int(np.argmin(abs(yy-point[1]))),int(np.argmin(abs(xx-point[0])))

start=cell((0,0));seen={start};queue=deque([start])
while queue:
    r,c=queue.popleft()
    for nr,nc in [(r-1,c),(r+1,c),(r,c-1),(r,c+1)]:
        if 0<=nr<safe.shape[0] and 0<=nc<safe.shape[1] and safe[nr,nc] and (nr,nc) not in seen:
            seen.add((nr,nc));queue.append((nr,nc))
points={'spawn':(0,0),'west_entry':(-2,0),'west_lab':(-2,2),'west_store':(-4,-1),
        'north_gallery':(0,3.3),'south_gallery':(0,-3.3),'east_entry':(3.5,-1.2),
        'east_store':(4.2,.4),'east_lab':(3.3,2.2)}
checks['spawn_clear_040m']=bool(safe[start])
checks['rooms_connected_at_040m']=all(cell(p) in seen for p in points.values())
report={'result':'PASS' if all(checks.values()) else 'FAIL','checks':checks,'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
        'world':'discovery_lab','size_m':[10,8],'navigation_map_generated':False,'scope':'OFFLINE_GEOMETRY_NOT_RUNTIME',
        'radius_m':.4,'resolution_m':.05,'room_probes':{n:{'xy':p,'reachable':cell(p) in seen} for n,p in points.items()}}
(root/'out/discovery_lab_20261021/geometry_checks.json').write_text(json.dumps(report,indent=2))
fig,ax=plt.subplots(figsize=(10,8))
for shape,pose,size in geometry:
    if shape=='box':
        from matplotlib.transforms import Affine2D
        patch=Rectangle((-size[0]/2,-size[1]/2),size[0],size[1],facecolor='#46566b',edgecolor='#162536')
        patch.set_transform(Affine2D().rotate(pose[5]).translate(*pose[:2])+ax.transData);ax.add_patch(patch)
    else:ax.add_patch(Circle(pose[:2],size,facecolor='#447fa2',edgecolor='#162536'))
ax.plot(0,0,'o',color='#eb5b24',markersize=9)
ax.annotate('Start (0, 0)',(0,0),(.2,-.6),arrowprops={'arrowstyle':'->'})
ax.arrow(0,0,.6,0,width=.025,color='#eb5b24',length_includes_head=True)
for x0,y0,label in [(-3,2,'West lab'),(-3,-1,'Storage'),(.5,3.5,'North gallery'),(.5,-1,'Central hall'),(3.6,3.5,'East lab'),(3.6,-2,'East bay')]:
    ax.text(x0,y0,label,ha='center',fontsize=10,color='#195a47')
ax.set(xlim=(-5.6,5.6),ylim=(-4.6,4.6),aspect='equal',xlabel='Gazebo world X [m]',ylabel='Gazebo world Y [m]')
ax.set_xticks(np.arange(-5,6));ax.set_yticks(np.arange(-4,5));ax.grid(alpha=.2)
ax.set_title('discovery_lab | 10 x 8 m | New scene 2026-10-21\nSDF geometry reference only; NOT a SLAM map or runtime screenshot')
fig.tight_layout()
for suffix in ['png','svg']:fig.savefig(root/f'out/discovery_lab_20261021/layout.{suffix}',dpi=140)
print(json.dumps(report,indent=2))
assert all(checks.values())
