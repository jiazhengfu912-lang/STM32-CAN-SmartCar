"""比较协议的公平性与离线几何指标，不启动仿真或发送运动命令。"""
import copy
from pathlib import Path
import sys
import pytest

WS=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(WS/'tools'))
from benchmark_controllers import profiles
from report_controllers import clearance_trace,summary_rows


def test_only_follow_path_profile_changes():
    """三种控制器共用同一全局规划、地图、恢复、到达检查和平滑配置。"""
    configs=profiles(WS);shared=[]
    for config in configs.values():
        value=copy.deepcopy(config);value['controller_server']['ros__parameters'].pop('FollowPath');shared.append(value)
    assert shared[0]==shared[1]==shared[2]
    rpp=configs['RPP']['controller_server']['ros__parameters']['FollowPath']
    dwb=configs['DWB']['controller_server']['ros__parameters']['FollowPath']
    mppi=configs['MPPI']['controller_server']['ros__parameters']['FollowPath']
    assert rpp['desired_linear_vel']==dwb['max_vel_x']==mppi['vx_max']==.3
    assert rpp['rotate_to_heading_angular_vel']==dwb['max_vel_theta']==mppi['wz_max']==.6
    assert dwb['min_vel_x']==mppi['vx_min']==0.
    for config in configs.values():
        smoother=config['velocity_smoother']['ros__parameters']
        assert smoother['max_velocity'][2]==.6 and smoother['min_velocity'][2]==-.6
    assert mppi['model_dt']==1/configs['MPPI']['controller_server']['ros__parameters']['controller_frequency']


@pytest.mark.parametrize('shape',[{'shape':'box','x':2.,'y':0.,'size':[1.,1.,1.]},
                                {'shape':'circle','x':2.,'y':0.,'radius':.5}])
def test_geometry_distance_and_overlap(shape):
    """矩形/圆障碍到带padding车头的已知间隙为1.23m，车体进入后重叠归零。"""
    rows=[{'wall_s':'0','x':'0','y':'0','yaw':'0'},
          {'wall_s':'1','x':'2','y':'0','yaw':'0'}]
    trace,minimum,overlap=clearance_trace(rows,[shape])
    assert trace[0]['clearance_m']==pytest.approx(1.23)
    assert minimum==0. and overlap==1


def test_inserted_obstacle_not_counted_before_insertion():
    """不能把未生成的箱体计入此前轨迹的碰撞指标。"""
    rows=[{'wall_s':'0','x':'2','y':'0','yaw':'0'},
          {'wall_s':'2','x':'0','y':'0','yaw':'0'}]
    shape={'shape':'box','x':2.,'y':0.,'size':[1.,1.],'inserted':True}
    trace,minimum,overlap=clearance_trace(rows,[shape],inserted=1.)
    assert trace[0]['clearance_m']==float('inf')
    assert minimum==pytest.approx(1.23) and overlap==0


def test_fast_failure_cannot_beat_completed_tasks():
    """选择器必须先通过任务及停车门槛，不能用失败轮次的短耗时获得更高排名。"""
    fixtures=[]
    for name in ['DWB','RPP','MPPI']:
        for case in ['straight','turn','detour','two_doors']:
            fixtures.append({'controller':name,'case':case,'task_correct_and_stopped':not(name=='DWB' and case=='turn'),
              'model_overlap_samples':0,'min_model_clearance_m':.2,'elapsed_sim_s':1. if name=='DWB' else 10.,
              'elapsed_wall_s':20.,'travel_m':4.,'angular_tv_per_sim_s':.2,'controller_process_cpu_percent':10.,
              'controller_process_peak_rss_mib':50.,'position_error_m':.1,'control_loop_overruns':0})
    rows,ranking=summary_rows(fixtures,1)
    dwb=next(r for r in rows if r['controller']=='DWB')
    assert not dwb['eligible'] and dwb['normalized_cost'] is None
    assert ranking[0]['controller']!='DWB'
