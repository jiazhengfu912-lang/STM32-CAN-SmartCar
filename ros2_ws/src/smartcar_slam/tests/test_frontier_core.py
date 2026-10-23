"""真实形状栅格夹具验证：不穿墙、不进入未知区，不把受阻误报为建图完成。"""
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from frontier_core import select_frontiers,dilate,predicted_gain,path_headings,reachable_distances,tree_route,prioritize_local


def room():
    """房间左侧已知、右侧未知，机器人位于已知自由区。"""
    grid=np.full((60,80),-1,dtype=np.int16);grid[1:59,1:40]=0
    grid[0,:]=grid[-1,:]=100;grid[:,0]=grid[:,-1]=100
    return grid


def test_goal_stands_back_from_unknown_and_has_safe_footprint():
    """目标整个安全圆在已知自由区，面向前沿而非进入未知区。"""
    grid=room();goals,details=select_frontiers(grid,.1,(0.,0.),(1.5,3.))
    assert goals and details['reachable_cells']>0
    for goal in goals:
        x,y=int(goal['x']/.1),int(goal['y']/.1)
        assert grid[y,x]==0 and not dilate(grid!=0,4.)[y,x]
        assert goal['x']<=3.6 and abs(goal['yaw'])<1.0


def test_disconnected_frontier_cannot_be_reached_through_wall():
    """隔离墙把前沿全部隔断时，不能给出穿墙目标或完成结论。"""
    grid=room();grid[:,25:27]=100
    goals,details=select_frontiers(grid,.1,(0.,0.),(1.5,3.))
    assert not goals and details['reason']=='NO_OBSERVABLE_FRONTIERS'


def test_unknown_surrounded_robot_does_not_snap_across_obstacle():
    """机器人落在未知区时，不跳到附近另一连通块虚构可达路径。"""
    goals,details=select_frontiers(room(),.1,(0.,0.),(4.1,3.))
    assert not goals and details['reason']=='ROBOT_NOT_IN_SAFE_SPACE'


def test_excluded_goals_keep_unresolved_frontiers_visible():
    """失败目标全部被排除不等于前沿消失，必须保留部分建图状态。"""
    goals,details=select_frontiers(room(),.1,(0.,0.),(1.5,3.),excluded=[(3.5,3.,10.)])
    assert not goals and details['clusters']>0 and details['reason']=='NO_REACHABLE_FRONTIERS'


def test_map_resize_and_origin_shift_preserve_world_goal():
    """地图左下方扩张后，排除和目标仍按世界坐标解释。"""
    grid=room();first,_=select_frontiers(grid,.1,(0.,0.),(1.5,3.))
    larger=np.pad(grid,((10,0),(20,0)),constant_values=-1)
    second,_=select_frontiers(larger,.1,(-2.,-1.),(1.5,3.))
    assert first and second
    assert second[0]['x']==pytest.approx(first[0]['x'])
    assert second[0]['y']==pytest.approx(first[0]['y'])


def test_fully_observed_room_has_no_frontiers():
    """全部区域已知才允许用前沿消失表示本算法的完成条件。"""
    grid=np.zeros((40,40),dtype=np.int16);grid[0,:]=grid[-1,:]=grid[:,0]=grid[:,-1]=100
    goals,details=select_frontiers(grid,.1,(0.,0.),(2.,2.))
    assert not goals and details['reason']=='NO_FRONTIERS'


def test_transit_corridor_keeps_nav2_clearance_without_requiring_goal_margin():
    """合法通道起点无需满足更严格的观察点余量，仍能去宽阔区域观察。"""
    grid=np.full((100,160),100,dtype=np.int16)
    grid[10:90,5:45]=0;grid[10:90,105:155]=0;grid[42:58,45:105]=0
    grid[10:90,145:155]=-1
    goals,details=select_frontiers(grid,.05,(0.,0.),(3.525,2.525))
    assert goals and details['reason']=='CANDIDATES'
    assert goals[0]['x']>5.5 and goals[0]['x']<6.9
    assert not dilate(grid!=0,8.)[int(goals[0]['y']/.05),int(goals[0]['x']/.05)]


def test_long_frontier_with_unreachable_centroid_uses_reachable_end():
    """一像素连成的前沿质心不可接近时，检查可达端部而非漏掉整组。"""
    grid=np.full((100,80),-1,dtype=np.int16);grid[1:99,1:40]=0
    grid[20:81,:39]=100;grid[0,:]=grid[-1,:]=100;grid[:,0]=100
    goals,details=select_frontiers(grid,.1,(0.,0.),(1.5,1.))
    assert goals and details['clusters']>=1
    assert goals[0]['y']<2.


def test_known_wall_blocks_predicted_information():
    """不能把已知墙后大块未知当作当前观察点可见收益；开门后才出现收益。"""
    grid=np.zeros((100,100),dtype=np.int16);grid[:,55:]=-1;grid[:,50:52]=100
    assert predicted_gain(grid,.1,(0.,0.),(3.,5.))==0.
    grid[43:58,50:52]=0
    assert predicted_gain(grid,.1,(0.,0.),(3.,5.))>0.


def test_one_cell_wall_cannot_be_skipped_by_gain_rays():
    """收益射线的采样细于栅格，单格薄墙也必须截断后方未知。"""
    grid=np.zeros((200,200),dtype=np.int16);grid[:,110:]=-1;grid[:,105]=100
    assert predicted_gain(grid,.05,(0.,0.),(3.013,5.))==0.


def test_turn_cost_selects_forward_frontier_in_symmetric_room():
    """相同观测收益时减少掉头，不靠提高转向速度缩短时间。"""
    grid=np.full((120,120),-1,dtype=np.int16);grid[1:-1,30:90]=0
    grid[0,:]=grid[-1,:]=grid[:,0]=grid[:,-1]=100
    right,_=select_frontiers(grid,.1,(0.,0.),(6.,6.),robot_yaw=0.)
    left,_=select_frontiers(grid,.1,(0.,0.),(6.,6.),robot_yaw=np.pi)
    assert right and left and right[0]['x']>6 and left[0]['x']<6
    assert all(np.isfinite(g['score']) and g['predicted_gain_m2']>=0 for g in right)


def test_large_unseen_region_beats_small_distant_pocket():
    """按预计信息/时间选观察点，优先打开大区域而非先补远处小洞。"""
    grid=np.zeros((120,120),dtype=np.int16);grid[1:-1,80:-1]=-1;grid[10:30,10:20]=-1
    grid[0,:]=grid[-1,:]=grid[:,0]=grid[:,-1]=100
    goals,details=select_frontiers(grid,.1,(0.,0.),(5.,6.),robot_yaw=np.pi)
    assert goals and details['reason']=='CANDIDATES'
    assert goals[0]['x']>6 and goals[0]['stage']=='coarse'


def test_short_observation_is_retained_when_no_large_step_exists():
    """大步探索不可用时仍补小片未知，不能为提速丢掉前沿并虚报完成。"""
    grid=room();goals,_=select_frontiers(grid,.1,(0.,0.),(3.,3.))
    assert goals
    filtered=[(g['x'],g['y'],.65) for g in goals if g['path_distance_m']>=1.2]
    fallback,details=select_frontiers(grid,.1,(0.,0.),(3.,3.),excluded=filtered)
    assert fallback and details['reason']=='CANDIDATES'


def test_small_remaining_unknown_pocket_uses_fine_phase():
    """剩余小片未知时不为大步标签强制远行，仍按收益/时间补盲。"""
    grid=np.zeros((100,100),dtype=np.int16);grid[45:55,65:75]=-1
    grid[0,:]=grid[-1,:]=grid[:,0]=grid[:,-1]=100
    goals,details=select_frontiers(grid,.1,(0.,0.),(5.,5.))
    assert goals and details['reason']=='CANDIDATES'
    assert all(g['stage']=='fine' for g in goals)
    assert goals[0]['path_distance_m']<2.


def test_room_observation_prefers_margin_for_map_updates_and_arrival_error():
    """宽阔已知区域优先选更靠内部的观察点，避免新地图将刚停下的位置判成贴障。"""
    grid=room();goals,_=select_frontiers(grid,.1,(0.,0.),(1.5,3.))
    assert goals
    preferred=[g for g in goals if g['goal_margin_m']>=.65]
    assert preferred
    for goal in preferred:
        x,y=int(goal['x']/.1),int(goal['y']/.1)
        assert not dilate(grid!=0,6.5)[y,x]


def test_tree_connection_never_shortcuts_a_dividing_wall():
    """两个观察点的返回代价沿安全路径算，不能用穿墙的欧氏近路。"""
    safe=np.ones((20,20),dtype=bool);safe[:16,10]=False
    _,parent=reachable_distances(safe,(5,5));route=tree_route(parent,20,(5,5),(5,15))
    assert len(route)-1>10
    assert all(safe[y,x] for y,x in route)
    assert all(abs(a[0]-b[0])+abs(a[1]-b[1])==1 for a,b in zip(route,route[1:]))


def test_local_observation_prevents_expensive_return_after_departure():
    """附近一个有效观察点先做，再去远处大区，减少之后返回同一区域的路程。"""
    safe=np.ones((35,100),dtype=bool);_,parent=reachable_distances(safe,(15,80))
    far={'x':1.,'y':1.5,'cell':[15,10],'frontier_group':0,'path_distance_m':7.,
         'estimated_cost_s':37.,'predicted_gain_m2':6.,'yaw':np.pi}
    near={'x':8.,'y':2.5,'cell':[25,80],'frontier_group':1,'path_distance_m':1.,
          'estimated_cost_s':11.5,'predicted_gain_m2':1.,'yaw':np.pi/2}
    goals=[far,near];prioritize_local(goals,parent,100,.1)
    assert goals[0] is near and near['route_reason']=='finish_local_before_departure'
    before=far['path_distance_m']+near['avoided_return_path_m']
    after=near['path_distance_m']+near['avoided_return_path_m']
    assert after<before
    tiny={**near,'predicted_gain_m2':.1};goals=[far,tiny];prioritize_local(goals,parent,100,.1)
    assert goals[0] is far


def test_arrival_orientation_follows_motion_instead_of_observation_direction():
    """沿向北的安全直线路径抵达时，不因雷达要看东边再强制转90度。"""
    safe=np.ones((20,20),dtype=bool);distance,parent=reachable_distances(safe,(5,5))
    initial,arrival=path_headings(safe,parent,(5,5),(15,5),.1)
    assert initial==pytest.approx(np.pi/2) and arrival==pytest.approx(np.pi/2)
    assert distance[15,5]==10
