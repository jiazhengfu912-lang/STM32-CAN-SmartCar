"""验证漏覆盖、观察记忆和真实路径代价的边界，不连接硬件。"""
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from frontier_core import select_frontiers,visible_unknown_cells,assign_observation_regions
from exploration_memory import ViewpointMemory
from route_core import path_cost,rank_routes,shortlist,open_visit_order


def test_subthreshold_frontier_has_visible_safe_observer():
    """小于原0.8米门槛的未知缝隙仍能生成安全观察点。"""
    g=np.zeros((100,100),dtype=np.int16);g[0,:]=g[-1,:]=g[:,0]=g[:,-1]=100
    g[45:50,70:75]=-1
    goals,details=select_frontiers(g,.025,(0.,0.),(.75,1.2))
    assert goals and details['small_clusters']>0
    assert all(x['visible_cells'] and x['visible_gain_m2']>0 for x in goals)


def test_direct_rays_find_tiny_opening_and_stop_at_first_unknown():
    """单格开口不会因5度角采样漏掉，未知内部也不重复计作已可见边界。"""
    g=np.zeros((120,120),dtype=np.int16);g[:,70:]=-1;g[:,69]=100;g[51,69]=0
    cells=visible_unknown_cells(g,.05,(0.,0.),(1.525,2.575))
    assert 51*120+70 in cells
    assert all(cell%120==70 for cell in cells)
    g[51,69]=100
    assert not visible_unknown_cells(g,.05,(0.,0.),(1.525,2.575))


def test_visit_memory_is_local_and_reopens_after_relevant_map_change():
    """成功视点只抑制原位置，附近替代位置保留，局部新观测会释放原位置。"""
    g=np.zeros((100,100),dtype=np.int16);g[40:60,65:80]=-1
    target={'x':2.5,'y':2.5};alternate={'x':3.,'y':2.5};m=ViewpointMemory()
    m.record(target,4,g,.05,(0.,0.),10.)
    available,details=m.filter([target,alternate],g,.05,(0.,0.),100.)
    assert available==[alternate] and details['exhausted_views']==1
    expanded=np.pad(g,((10,0),(20,0)),constant_values=-1)
    available,_=m.filter([target,alternate],expanded,.05,(-1.,-.5),100.)
    assert available==[alternate]
    g[40:60,65:80]=0
    available,_=m.filter([target,alternate],g,.05,(0.,0.),100.)
    assert available==[target,alternate]


def test_failed_viewpoint_has_cooldown_and_finite_retry():
    """单次失败冷却后允许一次重试，两次失败且地图不变时停止重复尝试。"""
    g=np.zeros((80,80),dtype=np.int16);target={'x':2.,'y':2.};m=ViewpointMemory()
    m.record(target,6,g,.05,(0.,0.),10.)
    available,d=m.filter([target],g,.05,(0.,0.),20.)
    assert not available and d['cooling_views']==1 and d['retry_sim']==40.
    assert m.filter([target],g,.05,(0.,0.),41.)[0]==[target]
    m.record(target,6,g,.05,(0.,0.),42.)
    assert not m.filter([target],g,.05,(0.,0.),100.)[0]


def test_path_cost_uses_real_detour_and_costmap_inflation():
    """相同端点的绕墙路径按实际长度收费，膨胀代价另外计入。"""
    short=path_cost([(1.,1.),(2.,1.),(3.,1.)],0.)
    detour=path_cost([(1.,1.),(1.,3.),(3.,3.),(3.,1.)],0.)
    assert short['length_m']==2 and detour['length_m']==6
    assert detour['estimated_cost_s']>short['estimated_cost_s']
    grid=np.full((60,60),126,dtype=np.uint8)
    risky=path_cost([(1.,1.),(2.,1.),(3.,1.)],0.,(grid,.1,(0.,0.)))
    assert risky['inflation_integral_m']==pytest.approx(1.)
    assert risky['estimated_cost_s']>short['estimated_cost_s']
    grid[:]=255
    assert path_cost([(1.,1.),(2.,1.)],0.,(grid,.1,(0.,0.))) is None


def test_nav2_insertion_finishes_small_local_region_before_far_trip():
    """实际候选间路径显示插入便宜时，低于旧0.8平方米门槛的小区也先补。"""
    far={'x':8.,'y':1.,'frontier_group':0,'stage':'fine','visible_cells':[1,2,3,4],
         'visible_gain_m2':.1,'predicted_gain_m2':.2,'path_distance_m':8.}
    local={'x':1.,'y':2.,'frontier_group':1,'stage':'fine','visible_cells':[5],
           'visible_gain_m2':.0025,'predicted_gain_m2':.01,'path_distance_m':1.}
    a={'estimated_cost_s':42.,'length_m':8.,'arrival_yaw':0.}
    b={'estimated_cost_s':8.,'length_m':1.,'arrival_yaw':0.}
    edge={'estimated_cost_s':40.,'length_m':7.,'arrival_yaw':0.}
    selected=rank_routes([far,local],[a,b],{(1,0):edge})
    assert selected['x']==local['x'] and selected['route_reason']=='nav2_open_visit_order'
    assert selected['nav2_visit_cost_s']==48.
    local['visible_cells']=[1]
    assert len(shortlist([{**far,'estimated_cost_s':42.},{**local,'estimated_cost_s':8.}],6))==1
    assert rank_routes([far,local],[None,b])['x']==local['x']


def test_shortlist_keeps_near_independent_observation_without_duplicates():
    """有限规划预算仍保留附近独立区域，同一观测内容不占满预算。"""
    candidates=[{'x':float(i),'y':1.,'frontier_group':i//3,'visible_cells':[i//3],
                 'visible_gain_m2':.1,'estimated_cost_s':float(12-i),'path_distance_m':10-i} for i in range(10)]
    selected=shortlist(candidates,6)
    assert len(selected)<=6 and len({tuple(x['visible_cells']) for x in selected})==len(selected)
    assert any(x['path_distance_m']<=3 for x in selected)


def test_tiny_local_insertion_does_not_delay_opening_large_unknown_region():
    """主探索阶段不因尾段微小补盲连续延迟打开大区，保留附近显著收益的插入。"""
    far={'x':8.,'y':1.,'stage':'coarse','visible_cells':[1,2,3],
         'visible_gain_m2':.1,'predicted_gain_m2':7.}
    tiny={'x':1.,'y':2.,'stage':'fine','visible_cells':[4],
          'visible_gain_m2':.0025,'predicted_gain_m2':.02}
    costs=[{'estimated_cost_s':42.,'length_m':8.,'arrival_yaw':0.},
           {'estimated_cost_s':8.,'length_m':1.,'arrival_yaw':0.}]
    edges={(1,0):{'estimated_cost_s':40.,'length_m':7.,'arrival_yaw':0.}}
    assert rank_routes([far,tiny],costs,edges)['x']==8.
    tiny['predicted_gain_m2']=1.
    assert rank_routes([far,tiny],costs,edges)['x']==1.
    tiny['predicted_gain_m2']=.1;tiny['visible_gain_m2']=.03
    assert rank_routes([far,tiny],costs,edges)['x']==1.


def test_goal_stopping_disk_keeps_original_arrival_tolerance_inside_safe_space():
    """目标前后0.15米到达偏差仍在0.37米已知安全圆内，不放宽到达或碰撞容差。"""
    g=np.zeros((120,120),dtype=np.int16);g[0,:]=g[-1,:]=g[:,0]=g[:,-1]=100;g[50:60,80:90]=-1
    goals,_=select_frontiers(g,.05,(0.,0.),(3.,3.))
    from frontier_core import dilate
    safe=(g==0)&~dilate(g!=0,.37/.05)
    assert goals
    for goal in goals:
        for angle in np.linspace(0,2*np.pi,48):
            x=int((goal['x']+.15*np.cos(angle))/.05);y=int((goal['y']+.15*np.sin(angle))/.05)
            assert safe[y,x]


def test_open_order_avoids_greedy_return_and_does_not_return_to_start():
    """三个点的真实有向代价表说明，先做略微便宜的小区可避免末段折返。"""
    costs=[{'estimated_cost_s':1.},{'estimated_cost_s':1.},{'estimated_cost_s':10.}]
    edges={(0,1):{'estimated_cost_s':2.},(1,0):{'estimated_cost_s':2.},
           (0,2):{'estimated_cost_s':9.},(2,0):{'estimated_cost_s':9.},
           (1,2):{'estimated_cost_s':11.},(2,1):{'estimated_cost_s':11.}}
    order,total=open_visit_order(costs,edges)
    assert order==[1,0,2] and total==12.
    assert total<costs[0]['estimated_cost_s']+edges[(0,1)]['estimated_cost_s']+edges[(1,2)]['estimated_cost_s']
    assert open_visit_order([None,None],edges)==([],float('inf'))
    order,total=open_visit_order(costs,{(1,0):edges[(1,0)]})
    assert order==[1,0] and total==3.


def test_region_budget_keeps_distant_jobs_and_respects_wall_topology():
    """左侧许多高分小前沿不挤掉远区代表，穿墙近点必须按真实通路分组。"""
    safe=np.ones((80,160),dtype=bool);safe[:65,80]=False
    points=[(20,70),(20,90),(60,140)]
    candidates=[{'cell':[y,x],'x':(x+.5)*.05,'y':(y+.5)*.05,'frontier_group':i,
                 'visible_cells':[i],'visible_gain_m2':.1,'estimated_cost_s':10.,
                 'path_distance_m':float(i+1),'stage':'fine'} for i,(y,x) in enumerate(points)]
    assert assign_observation_regions(candidates,safe,.05,radius=3.)>=2
    assert candidates[0]['observation_region']!=candidates[1]['observation_region']
    crowded=[]
    for i in range(20):
        crowded.append({'x':i*.5,'y':1.,'frontier_group':i,'observation_region':0,
                         'visible_cells':[i],'visible_gain_m2':1.,'estimated_cost_s':5.,
                         'path_distance_m':1.,'stage':'fine'})
    far={'x':20.,'y':3.,'frontier_group':99,'observation_region':1,'visible_cells':[99],
         'visible_gain_m2':.01,'estimated_cost_s':60.,'path_distance_m':12.,'stage':'fine'}
    assert far in shortlist(crowded+[far],8)
