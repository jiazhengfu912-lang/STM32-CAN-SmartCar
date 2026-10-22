"""真实形状栅格夹具验证：不穿墙、不进入未知区，不把受阻误报为建图完成。"""
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from frontier_core import select_frontiers,dilate


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
    assert not goals and details['reason']=='NO_REACHABLE_FRONTIERS'


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
