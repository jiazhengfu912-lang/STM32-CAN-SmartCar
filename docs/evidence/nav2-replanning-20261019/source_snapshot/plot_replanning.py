#!/usr/bin/env python3
"""只读绘制重规划验收的全部实收点，长缺口断线，不平滑或补造样本。"""

import argparse
import csv
import gzip
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
import yaml


TITLE = ('Nav2 obstacle replanning / 20261019 / firmware source v131 / SIMULATION ONLY\n'
         'P: right reverse .45 -> .45, others .30 -> .30; I: steady .010 -> .010, start .030 -> .030 per 10 ms\n'
         'STM32 PI not exercised; firmware algorithm unchanged; NavFn + RPP')


def config_label(folder):
    """从当轮配置快照标明规划轮廓，避免把修改后参数写到基线图上。"""
    params = yaml.safe_load((folder/'config/nav2.yaml').read_text())
    cfg = params['global_costmap']['global_costmap']['ros__parameters']
    if cfg.get('footprint'):
        return 'Global planning: original rectangle; local collision: rectangle'
    return f'Global planning: circle radius {cfg["robot_radius"]} m + padding {cfg["footprint_padding"]} m; local collision: rectangle'


def read_jsonl(path):
    """仅读取既有完整记录，缺失文件明确保留为空。"""
    if path.exists():
        with path.open() as stream:
            return [json.loads(line) for line in stream]
    archive = path.with_suffix(path.suffix+'.gz')
    if archive.exists():
        with gzip.open(archive, 'rt') as stream:
            return [json.loads(line) for line in stream]
    return []


def read_csv(path):
    """读取原始 CSV。"""
    if not path.exists():
        return []
    with path.open() as f:
        return list(csv.DictReader(f))


def line(rows, x, y):
    """只插入 NaN 断开一秒以上缺测，不增加测量点。"""
    xs, ys, previous = [], [], None
    for r in rows:
        now = float(r['wall_s'])
        if previous is not None and now-previous > 1.:
            xs.append(math.nan); ys.append(math.nan)
        xs.append(float(r[x])); ys.append(float(r[y])); previous = now
    return xs, ys


def plot(folder):
    """为每次成功/失败运行分别生成逐点速度、状态、路径和轨迹。"""
    report = json.loads((folder/'result.json').read_text())
    odom = read_csv(folder/'odom.csv'); commands = read_csv(folder/'velocity.csv')
    truth = read_csv(folder/'world_pose.csv'); paths = read_jsonl(folder/'plans.jsonl')
    safety = read_jsonl(folder/'safety.jsonl'); inserted = report.get('inserted_wall_s')
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for ax, key, unit in [(axes[0], 'vx', 'Linear x [m/s]'), (axes[1], 'wz', 'Angular z [rad/s]')]:
        ax.plot(*line(odom, 'wall_s', key), '.-', markersize=1.5, linewidth=.6, label='/sim/odom')
        for topic in ['/nav/cmd_vel_raw', '/nav/safe_cmd_vel']:
            selected = [r for r in commands if r['topic'] == topic]
            ax.plot(*line(selected, 'wall_s', key), '.-', markersize=1, linewidth=.4, label=topic)
        ax.set_ylabel(unit); ax.grid(); ax.legend(loc='upper right')
    names = sorted({json.loads(r['msg']['data'])['state'] for r in safety})
    states = [{'wall_s': r['wall_s'], 'state': names.index(json.loads(r['msg']['data'])['state'])} for r in safety]
    axes[2].plot(*line(states, 'wall_s', 'state'), '.-', markersize=2, linewidth=.5)
    axes[2].set_yticks(range(len(names)), names); axes[2].grid(); axes[2].set_ylabel('Gate state')
    for ax in axes:
        if inserted is not None:
            ax.axvline(inserted, color='red', linestyle='--', label='Obstacle inserted')
    axes[-1].set_xlabel('Monotonic wall time since verifier start [s]')
    axes[0].set_ylim(-.17, .32); axes[1].set_ylim(-.85, .85)
    fig.suptitle(TITLE+'\n'+config_label(folder)+'\n'+folder.name+' / '+report['result']); fig.tight_layout()
    for ext in ['png', 'svg']:
        fig.savefig(folder/('velocity.'+ext), dpi=150)
    plt.close(fig)


def analyze(folder):
    """从只读原始记录计算障碍进入地图、重规划延迟与模型轮廓间隙。"""
    report = json.loads((folder/'result.json').read_text())
    inserted = report.get('inserted_wall_s')
    counts = {}
    for r in read_jsonl(folder/'safety.jsonl'):
        state = json.loads(r['msg']['data'])['state']
        counts[state] = counts.get(state, 0)+1
    summary = {'gate_state_sample_counts': counts, 'scope': 'DERIVED_FROM_SIMULATION_RECORDS',
               'raw_report_result': report['result'], 'obstacle_detected': {}, 'raw_input_modified': False}
    if inserted is not None:
        box = report['obstacle']
        for key in ['global', 'local']:
            found = []
            for sample in read_jsonl(folder/(key+'_costmap_costmap.jsonl')):
                if sample['wall_s'] < inserted:
                    continue
                msg = sample['msg']; info = msg['info']; origin = info['origin']['position']; res = info['resolution']
                # 雷达观测表面而非箱体内部；允许地图/世界 AMCL 近似对齐的 0.1 m 误差。
                x0 = max(0, int((box['x']-box['dx']/2-.1-origin['x'])/res))
                x1 = min(info['width'], int((box['x']+box['dx']/2+.1-origin['x'])/res)+1)
                y0 = max(0, int((box['y']-box['dy']/2-.1-origin['y'])/res))
                y1 = min(info['height'], int((box['y']+box['dy']/2+.1-origin['y'])/res)+1)
                if x1 > x0 and y1 > y0 and any(msg['data'][y*info['width']+x] == 100
                        for y in range(y0, y1) for x in range(x0, x1)):
                    found.append(sample['wall_s'])
            summary['obstacle_detected'][key] = {'occupied_samples': len(found),
                'first_wall_s': found[0] if found else None, 'latency_wall_s': found[0]-inserted if found else None}
        summary['detection_scope'] = 'First published costmap containing lethal cells in box region +/- 0.1 m; publication latency, not internal sensor-processing latency'
        paths = read_jsonl(folder/'plans.jsonl')
        changed = [p for p in paths if p['wall_s'] >= inserted and any(
            abs(v['pose']['position']['x']-box['x']) < .3 and abs(v['pose']['position']['y']) > box['dy']/2+.2
            for v in p['msg']['poses'])]
        summary['first_detour_latency_wall_s'] = changed[0]['wall_s']-inserted if changed else None
        truth = [r for r in read_csv(folder/'world_pose.csv') if float(r['wall_s']) >= inserted]
        # 以局部碰撞检查的带 padding 矩形核对箱体间隙；只是几何模型，非接触传感器。
        bx, by = box['x'], box['y']; hx, hy = box['dx']/2, box['dy']/2
        obstacle = [(bx-hx, by-hy), (bx+hx, by-hy), (bx+hx, by+hy), (bx-hx, by+hy)]

        def point_segment(p, a, b):
            """点到线段的欧氏距离。"""
            dx, dy = b[0]-a[0], b[1]-a[1]
            t = max(0., min(1., ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy)))
            return math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dy)

        def clearance(row):
            """分离轴检测矩形相交，分离时计算顶点到边界的最小距离。"""
            x, y, theta = [float(row[k]) for k in ['x', 'y', 'yaw']]
            c, s = math.cos(theta), math.sin(theta)
            body = [(x+c*u-s*v, y+s*u+c*v) for u, v in [(-.16, -.18), (.27, -.18), (.27, .18), (-.16, .18)]]
            separate = False
            for nx, ny in [(1., 0.), (0., 1.), (c, s), (-s, c)]:
                p = [nx*u+ny*v for u, v in body]; q = [nx*u+ny*v for u, v in obstacle]
                separate |= max(p) < min(q) or max(q) < min(p)
            if not separate:
                return 0.
            distances = []
            for first, second in [(body, obstacle), (obstacle, body)]:
                for p in first:
                    for a, b in zip(second, second[1:]+second[:1]):
                        distances.append(point_segment(p, a, b))
            return min(distances)

        summary['padded_body_to_box_clearance_m'] = min([clearance(r) for r in truth] or [math.nan])
        summary['clearance_scope'] = 'Gazebo world poses + configured padded rectangle vs inserted box; not physical measurement/contact sensor'
    (folder/'analysis.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')


def plot_trajectory(folder):
    """以固定坐标尺度比较原路径、新路径和完整实收世界轨迹。"""
    report = json.loads((folder/'result.json').read_text())
    inserted = report.get('inserted_wall_s')
    paths = read_jsonl(folder/'plans.jsonl')
    truth = read_csv(folder/'world_pose.csv')
    fig, ax = plt.subplots(figsize=(11, 8))
    maps = read_jsonl(folder/'map.jsonl')
    if maps:
        m = maps[0]['msg']; info = m['info']; r = info['resolution']; origin = info['origin']['position']
        data = np.asarray(m['data']).reshape(info['height'], info['width'])
        ax.imshow(np.where(data < 0, .5, 1-data/100), origin='lower', cmap='gray', vmin=0, vmax=1,
                  extent=[origin['x'], origin['x']+info['width']*r, origin['y'], origin['y']+info['height']*r])
    # 全部路径保留在 JSONL；图中选原路线和第一条改变路线便于比较，并不修改原始记录。
    selected = [p for p in paths if inserted is not None and p['wall_s'] < inserted]
    after = [p for p in paths if inserted is not None and p['wall_s'] >= inserted]
    detour = next((p for p in after if any(abs(t['pose']['position']['y']) > .45 for t in p['msg']['poses'])), None)
    for p, label, color in [(selected[0] if selected else None, 'Original path [map]', 'orange'),
                            (detour, 'First detour path [map]', 'green')]:
        if p:
            pts = [v['pose']['position'] for v in p['msg']['poses']]
            ax.plot([v['x'] for v in pts], [v['y'] for v in pts], '--', color=color, label=label)
    ax.plot(*line(truth, 'x', 'y'), '.-', markersize=1.5, linewidth=.7, label='Actual vehicle [Gazebo world]')
    if inserted is not None:
        b = report['obstacle']
        ax.add_patch(Rectangle((b['x']-b['dx']/2, -b['dy']/2), b['dx'], b['dy'], color='red', alpha=.7, label='Inserted obstacle [world]'))
    ax.scatter([4.5], [0.], marker='*', s=100, label='Goal [map]')
    ax.set(xlim=(-6.3, 6.3), ylim=(-5.3, 5.3), xlabel='X [m]', ylabel='Y [m]', aspect='equal')
    ax.grid(); ax.legend(fontsize=8)
    fig.suptitle(TITLE+'\n'+config_label(folder)+'\n'+folder.name+' / '+report['result']+'; map and world approximately aligned by AMCL')
    fig.tight_layout()
    for ext in ['png', 'svg']:
        fig.savefig(folder/('trajectory.'+ext), dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folders', nargs='+', type=Path)
    for output in parser.parse_args().folders:
        plot(output)
        analyze(output)
        plot_trajectory(output)
