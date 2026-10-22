"""独立检查三帧阈值判据与参考空间分母，避免单帧面积跳高构成提速结论。"""
from pathlib import Path
import sys,json
root=Path.cwd();sys.path.insert(0,str(root/'ros2_ws/tools'))
from report_exploration_speed import sustained_milestones,reference_points
rows=[{'sim_s':i*2.,'wall_s':i*3.,'observed_fraction':v} for i,v in enumerate([.5,.96,.7,.91,.92,.93])]
result=sustained_milestones(rows)
assert result['0.9']=={'sim_s':6.,'wall_s':9.}
assert result['0.95'] is None
x,y,_=reference_points(root/'ros2_ws/src/smartcar_sim/worlds/discovery_lab.sdf')
assert (abs(x)<5).all() and (abs(y)<4).all() and len(x)>10000
report={'result':'PASS','three_frame_milestone':result,'reference_area_m2':len(x)*.05**2,'scope':'OFFLINE_METRIC_CHECK_NOT_ROBOT_RUN'}
(root/'out/exploration-speed-20261021/metric_check.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report))
