import importlib.util
from pathlib import Path
import math

spec = importlib.util.spec_from_file_location('gate_core', Path(__file__).resolve().parents[1]/'scripts/gate_core.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def ready():
    gate = module.NavigationGate()
    gate.localized = True
    gate.receive_clock(10., 1.)
    gate.scan_time = gate.odom_time = 1.
    gate.receive_command(.3, 0., 1.)
    return gate


def test_fresh_command_and_no_localization():
    gate = ready()
    assert gate.output(1.1) == (.3, 0., 'READY')
    gate.localized = False
    assert gate.output(1.1) == (0., 0., 'WAIT_LOCALIZATION')


def test_each_stale_input_stops_motion():
    for attribute, reason in [('scan_time', 'SCAN_STALE'), ('odom_time', 'ODOM_STALE'),
                              ('command_time', 'COMMAND_STALE'), ('clock_time', 'CLOCK_STALE')]:
        gate = ready()
        setattr(gate, attribute, None)
        assert gate.output(1.1) == (0., 0., reason)


def test_paused_clock_not_extended_by_same_clock_messages():
    gate = ready()
    gate.receive_clock(10., 1.4)
    assert gate.output(1.51)[2] == 'CLOCK_STALE'


def test_clock_rollback_requires_new_localization():
    gate = ready()
    gate.receive_clock(0., 1.1)
    assert gate.output(1.1)[2] == 'WAIT_LOCALIZATION'


def test_nonfinite_command_does_not_drive():
    for bad in [math.inf, math.nan, -math.inf]:
        gate = ready()
        gate.receive_command(bad, 0., 1.1)
        assert gate.output(1.1) == (0., 0., 'COMMAND_STALE')


def test_estop_latched_and_limits():
    gate = ready()
    gate.estop = True
    assert gate.output(1.1) == (0., 0., 'STOP_LATCHED')
    gate.estop = False
    gate.receive_command(5., -5., 1.)
    assert gate.output(1.1) == (.3, -.8, 'READY')
    gate.receive_command(-5., 0., 1.)
    assert gate.output(1.1) == (-.15, 0., 'READY')


def test_proximity_is_directional():
    gate = ready()
    gate.points = [(.36, 0.)]
    assert gate.output(1.1)[2] == 'OBSTACLE_AHEAD'
    gate.receive_command(-.1, 0., 1.)
    assert gate.output(1.1)[2] == 'READY'
    gate.points = [(-.25, 0.)]
    assert gate.output(1.1)[2] == 'OBSTACLE_BEHIND'


def test_turn_checks_swept_radius_and_clear_side():
    gate = ready()
    gate.receive_command(0., .4, 1.)
    gate.points = [(.2, .2)]
    assert gate.output(1.1)[2] == 'OBSTACLE_NEAR_TURN'
    gate.points = [(0., .6)]
    assert gate.output(1.1)[2] == 'READY'


def test_map_authority_is_exclusive_for_mapping_and_localization():
    """建图允许SLAM唯一发布，但不能以换来源为由允许AMCL或多图源混入。"""
    check=module.map_authority_ok
    assert check('slam',['slam_toolbox'],['slam_toolbox','planner_server'])
    assert check('amcl',['map_server'],['amcl','map_server'])
    assert not check('slam',['slam_toolbox'],['slam_toolbox','amcl'])
    assert not check('slam',['slam_toolbox','map_server'],['slam_toolbox'])
    assert not check('slam',['map_server'],['map_server'])
    assert not check('amcl',['map_server'],['map_server','slam_toolbox'])


def test_slam_startup_uses_live_map_and_tf_without_one_shot_pose():
    """晚于首个pose订阅时仍可就绪，但缺地图、过期/非法TF必须停车。"""
    check=module.slam_localization_ok
    identity=[0.,0.,0.,0.,0.,0.,1.]
    assert check(20.,19.,12.,11.9,identity)
    assert not check(20.,None,12.,11.9,identity)
    assert not check(20.,9.,12.,11.9,identity)
    assert not check(20.,21.,12.,11.9,identity)
    assert not check(20.,19.,None,11.9,identity)
    assert not check(20.,19.,12.,10.4,identity)
    assert not check(20.,19.,12.,13.1,identity)
    assert not check(20.,19.,12.,11.9,[0.]*7)
    for bad in [math.inf,math.nan,-math.inf]:
        assert not check(20.,19.,12.,11.9,[bad,*identity[1:]])
