#!/usr/bin/env python3
"""Save an actual map image and SLAM pose graph into a new directory."""
import argparse
import json
import os
from pathlib import Path
import re
import time
import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SaveMap, SerializePoseGraph
import yaml
from PIL import Image


def call(node, srv_type, service, request, timeout):
    client = node.create_client(srv_type, service)
    if not client.wait_for_service(timeout_sec=15):
        raise RuntimeError(f'Service unavailable: {service}; start SLAM and obtain /map first.')
    future = client.call_async(request)
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not future.done():
        raise TimeoutError(f'{service} did not complete in {timeout}s; inspect saved logs and partial files.')
    response = future.result()
    if response is None or response.result != 0:
        raise RuntimeError(f'{service} failed: {response}')
    return int(response.result)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', type=Path, required=True,
                        help='New directory; never overwrites an earlier mapping session.')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '86' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('First export SMARTCAR_WS and source smartcar_sim/tools/sim_env.bash.')
    folder = args.output_dir.expanduser().resolve()
    prefix = folder / 'map'
    # Humble SLAM Toolbox restricts its map-save service to these characters.
    if not re.fullmatch(r'[A-Za-z0-9_./-]+', str(prefix)):
        raise SystemExit('Use an ASCII output path without spaces for the Humble SLAM map-save service.')
    if folder.exists():
        raise SystemExit(f'Output directory already exists; choose a new directory: {folder}')
    folder.mkdir(parents=True, exist_ok=False)
    report = {'result': 'FAIL', 'scope': 'ACTUAL_SLAM_SERVICE_RESPONSES_AND_FILES',
              'prefix': str(prefix), 'services': {}, 'files': {}}
    rclpy.init()
    node = Node('smartcar_save_map')
    try:
        request = SaveMap.Request()
        request.name.data = str(prefix)
        report['services']['save_map'] = call(node, SaveMap, '/slam_toolbox/save_map', request, 90)
        request = SerializePoseGraph.Request()
        request.filename = str(prefix)
        report['services']['serialize_map'] = call(
            node, SerializePoseGraph, '/slam_toolbox/serialize_map', request, 45)
        for suffix in ['.yaml', '.pgm', '.posegraph', '.data']:
            path = Path(str(prefix) + suffix)
            if not path.is_file() or path.stat().st_size == 0:
                raise RuntimeError(f'Missing or empty saved file: {path}')
            report['files'][path.name] = path.stat().st_size
        metadata = yaml.safe_load(Path(str(prefix) + '.yaml').read_text())
        image_path = Path(metadata['image'])
        if not image_path.is_absolute():
            image_path = folder / image_path
        with Image.open(image_path) as image:
            report['image_size'] = list(image.size)
            report['image_extrema'] = image.getextrema()
        if metadata['resolution'] <= 0 or report['image_extrema'][0] == report['image_extrema'][1]:
            raise RuntimeError('Saved image is uniform or map resolution is invalid.')
        report['result'] = 'PASS'
        report['map_metadata'] = metadata
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = str(exc) or type(exc).__name__
    finally:
        (folder / 'save_result.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps(report, indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if report['result'] == 'PASS' else 1)


if __name__ == '__main__':
    main()
