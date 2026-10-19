#!/usr/bin/env python3
"""只读比较合成 ROS 故障注入中的真实控制器输出与生命周期响应。"""
import argparse
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

TITLE = ('Nav2 TF/reset fix / 20261019 / SYNTHETIC ROS, NO ACTUATION / firmware source v131\n'
         'P right reverse .45 -> .45, others .30 -> .30; I steady .010 -> .010, start .030 -> .030 per 10 ms\n'
         'STM32 PI not exercised; NavFn/RPP unchanged; checked goal TF + serialized costmap callbacks')


def main():
    """每个点均来自原事件/速度记录；散点图不补齐没有命令的区间。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('before', type=Path); parser.add_argument('after', type=Path)
    parser.add_argument('--output', type=Path, required=True); args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    data = {}
    for label, folder in [('Before (Nav2 1.1.20)', args.before), ('After (SmartCar)', args.after)]:
        events = [json.loads(s) for s in (folder/'events.jsonl').read_text().splitlines()]
        with (folder/'velocity.csv').open() as stream: rows = list(csv.DictReader(stream))
        start = next(e['wall_s'] for e in events if e['event']=='future_tf_goal_sent')
        end = next(e for e in events if e['event']=='future_tf_goal_result')
        data[label] = (events, rows, start, end)
    fig, axes = plt.subplots(2, 1, figsize=(12, 7))
    for label, (events, rows, start, end) in data.items():
        selected = [r for r in rows if start-.15 <= float(r['wall_s']) <= end['wall_s']+.15]
        axes[0].scatter([float(r['wall_s'])-start for r in selected],
                        [float(r['vx']) for r in selected], label=label, marker='x', s=65)
        axes[1].scatter([end['wall_s']-start], [end['status']], label=label, marker='x', s=100)
    axes[0].set(xlabel='Wall time since identical future-TF goal [s]', ylabel='Actual controller command vx [m/s]',
                ylim=(-.03, .35)); axes[0].legend(); axes[0].grid()
    axes[1].set(xlabel='Wall time since goal [s]', ylabel='FollowPath result', ylim=(3.5,6.5))
    axes[1].set_yticks([4,6], ['4 SUCCEEDED (wrong: fixed pose, target 5 m away)', '6 ABORTED (expected TF fault)'])
    axes[1].legend(); axes[1].grid()
    fig.suptitle(TITLE); fig.tight_layout()
    for ext in ['png', 'svg']: fig.savefig(args.output/('tf_fault.'+ext), dpi=150, bbox_inches='tight')
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(12,5)); table=[]
    for variant, (label, (events, _, _, _)) in enumerate(data.items()):
        clean = [e for e in events if e['event'].startswith('cleanup_')]
        for i,e in enumerate(clean):
            ax.scatter([i+1],[e['duration_wall_s']],s=70,marker='o' if e['success'] else 'x',
                       color=['tab:blue', 'tab:orange'][variant], label=label if i==0 else None)
            if not e['success']:
                ax.annotate('TIMEOUT >= 8 s (not completed)', (i+1,e['duration_wall_s']), xytext=(10,-10), textcoords='offset points')
            table.append({'variant':label, 'cycle':i+1, 'duration_wall_s':e['duration_wall_s'], 'success':e['success']})
    ax.set(xlabel='Lifecycle cleanup trial', ylabel='Observed response wall time [s, log scale]', yscale='log',ylim=(.001,12))
    ax.grid(); ax.legend(); fig.suptitle(TITLE); fig.tight_layout()
    for ext in ['png','svg']: fig.savefig(args.output/('lifecycle.'+ext),dpi=150,bbox_inches='tight')
    plt.close(fig)
    with (args.output/'lifecycle_derived.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=['variant','cycle','duration_wall_s','success']);writer.writeheader();writer.writerows(table)


if __name__ == '__main__':
    main()
