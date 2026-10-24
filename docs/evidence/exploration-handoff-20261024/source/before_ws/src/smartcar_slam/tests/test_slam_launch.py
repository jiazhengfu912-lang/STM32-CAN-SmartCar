"""自主建图叠加配置不得改变坐标/时钟，不可因配置缺失静默降级。"""
from pathlib import Path
import importlib.util
import pytest
import yaml
from launch import LaunchContext

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('slam_launch',ROOT/'launch/slam.launch.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_mapping_overlay_keeps_loop_closure_and_tightens_ambiguity_checks():
    """保留闭环和基础定位坐标，增强匹配要求与异常约束处理。"""
    base=yaml.safe_load((ROOT/'config/slam.yaml').read_text())['slam_toolbox']['ros__parameters']
    extra=yaml.safe_load((ROOT/'config/exploration_slam.yaml').read_text())['slam_toolbox']['ros__parameters']
    assert extra['do_loop_closing'] and extra['ceres_loss_function']=='HuberLoss'
    for key in ['loop_match_minimum_chain_size','loop_match_minimum_response_coarse','loop_match_minimum_response_fine']:
        assert extra[key]>base[key]
    assert extra['loop_match_maximum_variance_coarse']<base['loop_match_maximum_variance_coarse']
    assert not set(extra).intersection(['use_sim_time','map_frame','odom_frame','base_frame','scan_topic','resolution'])


def test_missing_overlay_fails_and_empty_overlay_retains_manual_entry():
    """不启动任何ROS进程，验证缺失叠加文件拒绝、手动默认路径仍可构造。"""
    context=LaunchContext();context.launch_configurations.update(params_file=str(ROOT/'config/slam.yaml'),
        override_params_file=str(ROOT/'config/missing.yaml'),map_file='',use_sim_time='true')
    with pytest.raises(RuntimeError,match='override file'):module.start(context)
    context.launch_configurations['override_params_file']=''
    assert len(module.start(context))==1
