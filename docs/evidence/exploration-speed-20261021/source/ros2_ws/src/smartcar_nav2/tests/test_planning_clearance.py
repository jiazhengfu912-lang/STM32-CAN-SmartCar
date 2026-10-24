"""防止 NavFn 重新选择矩形能直走、但车体无法安全转向的窄缝。"""
import math
from pathlib import Path

import yaml


def inscribed_radius(config):
    """计算 NavFn 障碍膨胀采用的内切半径，兼容多边形和圆形配置。"""
    if not config.get('footprint'):
        # 忽略额外 padding 得到保守下界，避免测试高估规划预留空间。
        return config['robot_radius']
    vertices = yaml.safe_load(config['footprint'])
    distances = []
    for p, q in zip(vertices, vertices[1:]+vertices[:1]):
        distances.append(abs(p[0]*q[1]-p[1]*q[0])/math.hypot(q[0]-p[0], q[1]-p[1]))
    return min(distances)


def test_global_planner_reserves_turning_space_for_local_footprint():
    """全局规划安全半径须覆盖局部矩形任意转角，并保留半个地图栅格余量。"""
    params = yaml.safe_load((Path(__file__).parents[1]/'config/nav2.yaml').read_text())
    local = params['local_costmap']['local_costmap']['ros__parameters']
    global_ = params['global_costmap']['global_costmap']['ros__parameters']
    pad = local['footprint_padding']
    vertices = yaml.safe_load(local['footprint'])
    sweep = max(math.hypot(abs(x)+pad, abs(y)+pad) for x, y in vertices)
    assert inscribed_radius(global_) >= sweep+global_['resolution']/2


def test_both_costmaps_observe_new_scan_obstacles():
    """全局重规划与局部碰撞检查均必须接收同一个真实仿真扫描源。"""
    params = yaml.safe_load((Path(__file__).parents[1]/'config/nav2.yaml').read_text())
    for name in ['local_costmap', 'global_costmap']:
        costmap = params[name][name]['ros__parameters']
        assert costmap['plugins'].index('obstacle_layer') < costmap['plugins'].index('inflation_layer')
        layer = costmap['obstacle_layer']
        assert layer['enabled']
        source = layer['scan']
        assert source['topic'] == '/sim/scan' and source['marking'] and source['clearing']
