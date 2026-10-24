"""验证漏覆盖、观察记忆和真实路径代价的边界，不连接硬件。"""
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from frontier_core import select_frontiers,visible_unknown_cells,assign_observation_regions,target_still_visible,dilate
from exploration_memory import ViewpointMemory,EmptyConfirmation,navigation_failure
from route_core import path_cost,rank_routes,shortlist,open_visit_order,stopping_approach,information_gain,information_visit_order
from readiness_core import LifecycleReadiness


def test_lifecycle_active_evidence_expires_without_a_fresh_response():
    """服务曾active不能无限沿用，任何关键节点失联都保持无新目标。"""
    state=LifecycleReadiness(['planner','controller'])
    state.received('planner',3,10.);assert not state.ready(10.)
    state.received('controller',3,10.);assert state.ready(12.9)
    assert not state.ready(13.)
    state.received('controller',2,14.);state.received('planner',3,14.)
    assert not state.ready(14.)


def test_lifecycle_lost_query_is_retired_and_can_be_queried_again():
    """暂停时丢失的服务响应有明确期限，后续新查询的active证据可以恢复。"""
    state=LifecycleReadiness(['planner']);state.requested('planner',10.)
    assert state.expired(12.9)==[] and state.expired(13.)==['planner']
    state.unavailable('planner');state.pending.pop('planner');state.requested('planner',14.)
    state.received('planner',3,14.1)
    assert state.ready(14.2) and not state.pending


def test_explicit_resume_discards_old_lifecycle_states_and_pending_queries():
    """显式恢复先清空旧状态，再要求所有关键节点重新确认。"""
    state=LifecycleReadiness(['planner','controller']);state.received('planner',3,10.)
    state.received('controller',3,10.);state.requested('planner',10.1);state.reset()
    assert not state.ready(10.2) and not state.pending
    state.received('planner',3,11.);assert not state.ready(11.)
    state.received('controller',3,11.);assert state.ready(11.)


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


def test_wide_room_retains_wide_and_narrow_legal_observers():
    """宽余量和原合法窄余量均保留，停车安全由实际路径末段独立判定。"""
    g=np.zeros((120,120),dtype=np.int16);g[0,:]=g[-1,:]=g[:,0]=g[:,-1]=100;g[50:60,80:90]=-1
    goals,_=select_frontiers(g,.05,(0.,0.),(3.,3.))
    from frontier_core import dilate
    safe=(g==0)&~dilate(g!=0,.40/.05)
    assert {round(goal['goal_margin_m'],2) for goal in goals}=={.40,.57,.65}
    for goal in goals:
        y,x=goal['cell']
        assert safe[y,x] and goal['target_visible_cells']


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


def test_assigned_frontier_cannot_borrow_visibility_from_other_side_of_wall():
    """墙后指定前沿不可见时，不能用同侧另一片未知的收益冒充它的观察点。"""
    g=np.zeros((100,160),dtype=np.int16);g[:,80:82]=100;g[20:75,110:145]=-1;g[15:25,45:55]=-1
    g[0,:]=g[-1,:]=g[:,0]=g[:,-1]=100
    goals,_=select_frontiers(g,.05,(0.,0.),(1.5,2.5))
    assert goals
    for goal in goals:
        assert goal['target_visible_cells'] and set(goal['target_visible_cells'])<=set(goal['visible_cells'])
        assert all(cell%160<80 for cell in goal['target_visible_cells'])


def test_completed_target_does_not_borrow_neighboring_unknown_on_recheck():
    """指定目标已被新地图观测后，即便仍能看见邻近未知，也不继续发送旧任务。"""
    g=np.zeros((100,100),dtype=np.int16);g[45:55,70:80]=-1;g[20:30,40:50]=-1
    wanted=[(3.525,2.525)]
    assert target_still_visible(g,.05,(0.,0.),(2.,2.5),wanted)
    g[45:55,70:80]=0
    assert visible_unknown_cells(g,.05,(0.,0.),(2.,2.5))
    assert not target_still_visible(g,.05,(0.,0.),(2.,2.5),wanted)


def test_early_stop_path_is_checked_without_discarding_narrow_legal_observer():
    """沿窄通道的安全到达保留，离障过近的提前停点被实际路径检查拒绝。"""
    g=np.full((100,100),100,dtype=np.int16);g[1:99,1:18]=0;safe=(g==0)&~dilate(g!=0,.4/.05)
    path=[(.475,float(y)) for y in np.arange(1.025,2.526,.05)]
    assert stopping_approach(path,(.475,2.525),safe,.05,(0.,0.))
    assert not ((g==0)&~dilate(g!=0,.57/.05))[50,9]
    g=np.zeros((100,100),dtype=np.int16);g[40:60,40:60]=100;safe=(g==0)&~dilate(g!=0,.4/.05)
    departing=[(float(x),2.525) for x in np.arange(3.025,3.476,.025)]
    assert safe[50,int(3.475/.05)]
    assert stopping_approach(departing,(3.475,2.525),safe,.05,(0.,0.)) is None
    approaching=[(float(x),2.525) for x in np.arange(4.475,3.474,-.025)]
    assert stopping_approach(approaching,(3.475,2.525),safe,.05,(0.,0.))


def test_unique_small_target_survives_large_overlap_with_other_information():
    """大部分视野重复时，指定小前沿仍有独立未知格就不能被比例阈值丢掉。"""
    a={'x':1.,'y':1.,'frontier_group':0,'observation_region':0,'stage':'fine','visible_cells':list(range(100)),
       'target_visible_cells':[1],'visible_gain_m2':1.,'estimated_cost_s':5.,'path_distance_m':1.}
    b={'x':2.,'y':1.,'frontier_group':1,'observation_region':1,'stage':'fine','visible_cells':list(range(101)),
       'target_visible_cells':[100],'visible_gain_m2':1.01,'estimated_cost_s':6.,'path_distance_m':2.}
    assert shortlist([a,b],2)==[a,b]


def test_successful_position_does_not_blacklist_new_target_or_new_view_angle():
    """同位置的新目标、以及相隔0.35米的新观察角度，都不被成功位置记忆整体拉黑。"""
    g=np.zeros((100,100),dtype=np.int16);g[50,70:72]=-1;memory=ViewpointMemory()
    old={'x':2.5,'y':2.5,'target_observation_world':[[3.525,2.525]]}
    memory.record(old,4,g,.05,(0.,0.),10.)
    same={'x':2.5,'y':2.5,'target_visible_cells':[50*100+70]}
    new={'x':2.5,'y':2.5,'target_visible_cells':[50*100+71]}
    angle={'x':2.75,'y':2.75,'target_visible_cells':[50*100+70]}
    available,_=memory.filter([same,new,angle],g,.05,(0.,0.),20.)
    assert same not in available and new in available and angle in available


def test_empty_confirmation_restarts_when_candidates_reappear():
    """新候选打断空结果确认，重复旧地图不充数，规划失败保持自己的确认周期。"""
    empty=EmptyConfirmation()
    assert not empty.observe('NO_FRONTIERS',0.,1)
    assert not empty.observe('NO_FRONTIERS',10.,1)
    empty.candidates()
    assert not empty.observe('NO_FRONTIERS',11.,2)
    assert not empty.observe('NO_FRONTIERS',14.,3)
    assert empty.observe('NO_FRONTIERS',17.,4)
    empty.clear();assert not empty.observe('NAV2_UNREACHABLE_FRONTIERS',0.,1)
    empty.candidates();assert not empty.observe('NAV2_UNREACHABLE_FRONTIERS',3.,2)
    assert empty.observe('NAV2_UNREACHABLE_FRONTIERS',6.,3)
    empty.candidates(planned=True);assert not empty.observe('NAV2_UNREACHABLE_FRONTIERS',7.,4)
    assert not empty.observe('VIEWPOINTS_COOLDOWN',100.,5) and empty.reason is None


def test_timeout_is_counted_once_and_manual_cancel_is_not_navigation_failure():
    """超时及后续取消/中止只增加一次失败，暂停和外部取消不消耗八次失败预算。"""
    goal={'timed_out':True}
    assert navigation_failure(goal,5)
    assert not navigation_failure(goal,5) and not navigation_failure(goal,6)
    assert not navigation_failure({},5) and not navigation_failure({},6,paused=True)
    assert navigation_failure({},6)


def test_close_legal_observer_is_retained_alongside_wide_margin_views():
    """大余量仅看到前沿一部分时，合法0.40米观察点仍进入实际Nav2代价比较。"""
    g=np.zeros((100,120),dtype=np.int16);g[:,80:]=-1;g[0,:]=g[-1,:]=g[:,0]=100
    goals,_=select_frontiers(g,.05,(0.,0.),(2.,2.5))
    assert any(x['goal_margin_m']==pytest.approx(.4) for x in goals)
    assert any(x['goal_margin_m']>=.65 for x in goals)
    assert information_gain({'predicted_gain_m2':1.2,'target_visible_gain_m2':.02})==1.2
    assert information_gain({'predicted_gain_m2':0.,'target_visible_gain_m2':.005,'visible_gain_m2':.1})==.005


def test_information_order_prioritizes_large_blind_region_with_near_equal_travel():
    """总行驶差不足一秒时，优先看到大盲区，不能先长途补一个低收益格。"""
    candidates=[{'predicted_gain_m2':.93},{'predicted_gain_m2':.43},{'predicted_gain_m2':.07}]
    costs=[{'estimated_cost_s':57.46},{'estimated_cost_s':84.46},{'estimated_cost_s':54.92}]
    edges={(0,1):{'estimated_cost_s':65.},(1,0):{'estimated_cost_s':65.},
           (1,2):{'estimated_cost_s':62.73},(2,1):{'estimated_cost_s':62.72},
           (0,2):{'estimated_cost_s':100.},(2,0):{'estimated_cost_s':100.}}
    assert open_visit_order(costs,edges)[0]==[2,1,0]
    order,total,details=information_visit_order(candidates,costs,edges)
    assert order==[0,1,2] and total<=details['nav2_visit_cost_budget_s']
    assert details['nav2_information_latency_m2_s']<details['nav2_baseline_information_latency_m2_s']


def test_information_order_does_not_buy_early_gain_with_large_total_detour():
    """高收益优先若明显增大整批路程，则回退实际Nav2最短开放顺序。"""
    candidates=[{'predicted_gain_m2':10.},{'predicted_gain_m2':.01}]
    costs=[{'estimated_cost_s':10.},{'estimated_cost_s':1.}]
    edges={(0,1):{'estimated_cost_s':30.},(1,0):{'estimated_cost_s':10.}}
    order,total,details=information_visit_order(candidates,costs,edges)
    assert order==[1,0] and total==11. and not details['nav2_information_order_used']
    assert information_visit_order(candidates,[None,None],edges)[:2]==([],float('inf'))
