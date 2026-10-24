"""Reject broken/empty map data rather than treating any /map as success."""
import importlib.util
from pathlib import Path
import math
import numpy as np

spec = importlib.util.spec_from_file_location(
    'map_validation', Path(__file__).resolve().parents[1] / 'scripts/map_validation.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def inspect(values=None, **kwargs):
    defaults = dict(frame='map', width=20, height=20, resolution=.05,
                    origin=[-1., -1., 0., 0., 0., 0., 1.],
                    values=([100]*20 + [0]*300 + [-1]*80) if values is None else values)
    defaults.update(kwargs)
    return module.inspect_map(**defaults)['checks']


def test_valid_map_with_unknown_free_and_occupied_cells():
    assert all(inspect().values())


def test_missing_data_and_mismatched_dimensions_fail():
    assert not inspect([])['dimensions_match_data']
    assert not inspect(width=21)['dimensions_match_data']


def test_empty_free_space_is_not_a_successful_map():
    checks = inspect([0]*400)
    assert checks['free_space_present']
    assert not checks['obstacles_present']
    assert not inspect([-1]*400)['free_space_present']


def test_invalid_values_frames_resolution_and_origin_fail():
    assert not inspect([101]*400)['valid_occupancy_values']
    assert not inspect([-2]*400)['valid_occupancy_values']
    assert not inspect(frame='odom')['map_frame']
    assert not inspect(resolution=math.nan)['finite_positive_resolution']
    assert not inspect(origin=[0]*7)['unit_origin_quaternion']
    assert not inspect(origin=[0, 0, math.inf, 0, 0, 0, 1])['finite_origin']
