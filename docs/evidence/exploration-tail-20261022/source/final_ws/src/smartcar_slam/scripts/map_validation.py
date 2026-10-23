"""Validation of received OccupancyGrid values; independent of ROS imports."""
import math
import numpy as np


def inspect_map(frame, width, height, resolution, origin, values):
    data = np.asarray(values, dtype=np.int16)
    shape_ok = width > 0 and height > 0 and data.size == width * height
    counts = {'unknown': int(np.count_nonzero(data == -1)),
              'free': int(np.count_nonzero(data == 0)),
              'occupied': int(np.count_nonzero(data >= 65))}
    checks = {
        'map_frame': frame == 'map',
        'dimensions_match_data': shape_ok,
        'finite_positive_resolution': math.isfinite(resolution) and resolution > 0,
        'finite_origin': len(origin) == 7 and all(math.isfinite(v) for v in origin),
        'valid_occupancy_values': bool(data.size) and bool(np.all((data >= -1) & (data <= 100))),
        'free_space_present': counts['free'] >= 100,
        'obstacles_present': counts['occupied'] >= 10,
    }
    if checks['finite_origin']:
        checks['unit_origin_quaternion'] = abs(sum(v*v for v in origin[3:]) - 1) < 1e-3
    else:
        checks['unit_origin_quaternion'] = False
    return {'frame': frame, 'width': width, 'height': height,
            'resolution': resolution, 'origin': list(origin),
            'cells': int(data.size), 'counts': counts, 'checks': checks}
