"""归档新场景两次真实运行并逐点绘图；仅读取已结束试验的原始文件。"""
from pathlib import Path
import json
import hashlib
import shutil
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path.cwd();out=root/'out/discovery_lab_20261021';archive=root/'docs/evidence/discovery-lab-20261021'
archive.mkdir(exist_ok=False)
for name in ['source','runs','plots','logs']:(archive/name).mkdir()
sys.path.insert(0,str(root/'ros2_ws/tools'))
from plot_exploration import read,line
plt.rcParams['path.simplify']=False
runs={'baseline':'baseline','startup_01':'smoke','diagnostic':'diagnostic','late_gate':'late-gate','smoke':'smoke-final'}
for name,folder in runs.items():
    src=root/f'ros2_ws/verification_runs/discovery-lab-20261021-{folder}'
    shutil.copytree(src,archive/'runs'/name)
for name in ['create_scene.py','check_and_draw.py','archive_and_plot.py','geometry_checks.json','geometry_probe_01.json','late_join.py','pose_diagnostic.json']:
    shutil.copy2(out/name,archive/'source'/name)
for name in ['layout.png','layout.svg']:shutil.copy2(out/name,archive/name)
for path in out.glob('*.log'):shutil.copy2(path,archive/'logs'/path.name)
for rel in ['ros2_ws/src/smartcar_sim/worlds/discovery_lab.sdf','ros2_ws/tools/verify_exploration.py',
            'ros2_ws/src/smartcar_nav2/launch/exploration.launch.py','ros2_ws/src/smartcar_nav2/launch/navigation.launch.py',
            'ros2_ws/src/smartcar_slam/scripts/explore.py','ros2_ws/src/smartcar_slam/scripts/frontier_core.py',
            'ros2_ws/src/smartcar_nav2/scripts/navigation_gate.py','ros2_ws/src/smartcar_nav2/scripts/gate_core.py']:
    dst=archive/'source'/rel;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(root/rel,dst)

TITLE=('New discovery_lab scene 20261021 | SIMULATION ONLY | firmware source v131\n'
       'P right reverse .45 -> .45, others .30 -> .30; I steady .010 -> .010, start .030 -> .030 / 10 ms\n'
       'PI NOT exercised; frontier / SLAM / NavFn / RPP unchanged; SLAM gate startup fixed; new physical layout')

def save(fig,name):
    """图中保留版本/PI/算法与起停数据，PNG和SVG同步输出。"""
    fig.suptitle(TITLE,fontsize=10,y=.995);fig.tight_layout(rect=(0,0,1,.88))
    for suffix in ['png','svg']:fig.savefig(archive/'plots'/(name+'.'+suffix),dpi=135,bbox_inches='tight')
    plt.close(fig)

fig,axes=plt.subplots(2,3,figsize=(17,8),sharex='col',sharey='row')
for col,name in enumerate(['baseline','startup_01','smoke']):
    folder=archive/'runs'/name
    for ax,key in zip(axes[:,col],['vx','wz']):
        ax.plot(*line(read(folder/'odom.csv'),key),'.-',markersize=1,linewidth=.5,label='Measured /sim/odom')
        if (folder/'commands.csv').exists():ax.plot(*line(read(folder/'commands.csv'),key),'.-',markersize=1,linewidth=.5,label='Actual /nav/safe_cmd_vel')
        ax.grid(alpha=.3);ax.legend(fontsize=8)
    axes[0,col].set_title({'baseline':'SLAM only, stationary','startup_01':'Before fix: waiting for missed pose','smoke':'After fix: 90 sim-second smoke; PARTIAL'}[name])
    axes[1,col].set_xlabel('Monotonic wall time [s]')
    axes[0,col].set_ylim(-.17,.34);axes[1,col].set_ylim(-.85,.85)
axes[0,0].set_ylabel('Linear velocity [m/s]');axes[1,0].set_ylabel('Angular velocity [rad/s]');save(fig,'velocity')
fig,axes=plt.subplots(2,1,figsize=(12,7),sharex=True)
for name in ['baseline','smoke']:
    rows=read(archive/'runs'/name/'maps.csv')
    for key,ax in zip(['known','free'],axes):
        values=[{**r,'area':float(r[key])*float(r['resolution'])**2} for r in rows]
        ax.plot(*line(values,'area'),'.-',markersize=2,linewidth=.7,label=name)
for ax in axes:ax.grid(alpha=.3);ax.legend()
axes[0].set_ylabel('Observed known area [m²]');axes[1].set_ylabel('Observed free area [m²]')
axes[1].set_xlabel('Monotonic wall time [s]; observed area is not geometric accuracy or full coverage');save(fig,'map_growth')
fig,axes=plt.subplots(1,2,figsize=(13,7));extents=[]
for ax,phase in zip(axes,['initial','final']):
    data=np.load(archive/'runs/smoke/mission'/('map_'+phase+'.npz'));g=data['occupancy'];r=float(data['resolution']);o=data['origin']
    im=np.full(g.shape,205,dtype=np.uint8);im[(g>=0)&(g<=20)]=255;im[g>=65]=0;im[(g>20)&(g<65)]=125
    extent=[o[0],o[0]+g.shape[1]*r,o[1],o[1]+g.shape[0]*r];extents.append(extent)
    ax.set_facecolor('#cdcdcd');ax.imshow(im,origin='lower',extent=extent,cmap='gray',vmin=0,vmax=255,interpolation='nearest')
    ax.set(aspect='equal',xlabel='SLAM map X [m]',ylabel='SLAM map Y [m]',title=phase+' actual online map')
for ax in axes:
    ax.set_xlim(min(e[0] for e in extents),max(e[1] for e in extents));ax.set_ylim(min(e[2] for e in extents),max(e[3] for e in extents))
save(fig,'maps')
checks=[]
for name in runs:
    folder=archive/'runs'/name;result=json.loads((folder/'result.json').read_text())
    for relative,digest in result['raw_sha256'].items():
        p=folder/relative;checks.append({'run':name,'file':relative,'match':p.exists() and hashlib.sha256(p.read_bytes()).hexdigest()==digest})
(archive/'integrity.json').write_text(json.dumps({'result':'PASS' if all(x['match'] for x in checks) else 'FAIL','raw_files_checked':len(checks),'issues':[x for x in checks if not x['match']]},indent=2))
print('Archive:',archive)
for name in runs:
    s=json.loads((archive/'runs'/name/'result.json').read_text());session=s.get('session',{})
    print(name,json.dumps({'result':s['result'],'travel_m':s.get('travel_m'),'reason':session.get('reason'),'complete':session.get('complete'),'goals':len(session.get('goals',[])),'known_initial':s.get('map_initial',{}).get('known'),'known_final':s.get('map_final',{}).get('known')},ensure_ascii=False))
