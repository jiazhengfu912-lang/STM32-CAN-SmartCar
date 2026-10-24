"""防止恢复流程遗忘视野外已发现的堵路障碍，或无限重置重试次数。"""
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml


CONFIG = Path(__file__).parents[1]/'config'


@pytest.mark.parametrize('name', ['navigate_recovery.xml', 'navigate_through_recovery.xml'])
def test_default_goal_modes_preserve_obstacle_memory(name):
    """单目标和连续多目标的自动恢复都不能清空已观测代价图。"""
    tree = ET.parse(CONFIG/name)
    assert not [node for node in tree.iter() if node.tag.startswith('Clear')]


@pytest.mark.parametrize('name', ['navigate_recovery.xml', 'navigate_through_recovery.xml'])
def test_planning_and_control_share_one_bounded_recovery_budget(name):
    """规划失败须上报总恢复预算，不以内层清图/成功重规划无限续试。"""
    tree = ET.parse(CONFIG/name)
    recoveries = list(tree.iter('RecoveryNode'))
    assert len(recoveries) == 1
    assert 0 < int(recoveries[0].get('number_of_retries')) <= 4
    pipeline = recoveries[0].find('PipelineSequence')
    assert pipeline is not None
    assert pipeline.find('.//FollowPath') is not None
    assert any(n.tag in ['ComputePathToPose', 'ComputePathThroughPoses'] for n in pipeline.iter())


def test_removed_obstacles_can_still_clear_from_new_scan_rays():
    """保留正常雷达射线清障；禁止以关闭 clearing 或长期重放旧扫描假装记忆。"""
    config = yaml.safe_load((CONFIG/'nav2.yaml').read_text())
    global_ = config['global_costmap']['global_costmap']['ros__parameters']
    assert global_.get('rolling_window', False) is False
    for name in ['global_costmap', 'local_costmap']:
        scan = config[name][name]['ros__parameters']['obstacle_layer']['scan']
        assert scan['marking'] and scan['clearing']
        assert scan.get('observation_persistence', 0.) == 0.
