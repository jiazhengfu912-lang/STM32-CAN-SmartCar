"""以最终匹配实验指标更新交接入口，不覆盖历史正文或承诺未验证的吞吐改善。"""
from pathlib import Path
import json,re,subprocess
root=Path.cwd();e=root/'docs/evidence/exploration-handoff-20261024';metrics=json.loads((e/'analysis/final_v17/metrics.json').read_text())
protocol=json.loads((e/'source/protocol_v17.json').read_text())


def number(value):
    """缺测不伪装成零秒或完成。"""
    return '未达到/缺测' if value is None else f'{value:.3f}'


def section(prefix):
    """对各文档使用正确相对路径，数字来自同一最终报表。"""
    rows=[]
    for before,after in protocol['formal_pairs']:
        b,c=metrics[before],metrics[after];bh=b['handoff_timing']['handoffs'];ch=c['handoff_timing']['handoffs']
        bi=b['handoff_timing']['intervals'].get('0.95->0.98');ci=c['handoff_timing']['intervals'].get('0.95->0.98')
        scene='实验室' if after.endswith('discovery') else '办公室'
        rows.append(f"{scene}交接中位数 {number(bh['median_gap_sim_s'])}→{number(ch['median_gap_sim_s'])} 仿真秒；95%→98%参考观测区间 {number(bi['simulation_seconds'] if bi else None)}→{number(ci['simulation_seconds'] if ci else None)} 仿真秒。")
    return ('## 2026-10-24 Frontier交接与近静止优化：当前v17\n\n'
        '本轮实现事件驱动交接、完整新鲜观测证据免额外等帧、同批安全/终点几何复用，以及保持选点结果的射线模板和限界区域广搜。v16跨目标预备候选发生开发退化，已移除，原数据和源码保留；当前每次从最新地图与实际位姿重新生成完整候选。NavFn/RPP、SLAM、速度、到达容差和停机保护未改。\n\n'
        +' '.join(rows)+' 交接等待与有效覆盖吞吐分开判断，部分图不称全图完成。正式对照统一RelWithDebInfo编译等级；原构建等级不同的两轮作为开发参考保留，不作最终量化基线。\n\n'
        '105项colcon检查零错误/失败/跳过；12张真实快照的全部候选、收益和排序与原v15字节一致，离线用时改善因快照而异；失联锁存、不自动续行、显式恢复/暂停/保存和小房间NO_FRONTIERS完成回归通过。大场景仍为partial，单场景一对样本不构成统计显著或实车结论。\n\n'
        f'详见[本轮实测结论]({prefix}docs/evidence/exploration-handoff-20261024/final_conclusion.md)、[算法与测量方法]({prefix}docs/EXPLORATION_HANDOFF.md)和[直接嵌入波形的验收正文]({prefix}docs/ACCEPTANCE.md)。\n\n'
        '本轮未执行提交、推送、合并、烧录或实车运动。固件源码仍v131、设备最后确认v127，未连接硬件。基线HEAD为cbc270e；当前Git状态以实时检查为准，以下旧日期的推送授权、HEAD、测试数量与未提交表述均为对应历史阶段。\n')


def prepend(path,body,marker):
    """重复生成时只替换本轮标记块，用户历史段落保持原样。"""
    text=path.read_text();block=f'<!-- {marker}:start -->\n{body}<!-- {marker}:end -->\n\n'
    pattern=re.escape(f'<!-- {marker}:start -->')+r'.*?'+re.escape(f'<!-- {marker}:end -->')+r'\s*'
    if f'<!-- {marker}:start -->' in text:text=re.sub(pattern,lambda _:block,text,flags=re.S)
    else:
        first,rest=text.split('\n',1);text=first+'\n\n'+block+rest
    path.write_text(text)


for name in ['README_HANDOFF.md','HANDOFF.md']:prepend(root/name,section(''),'exploration-handoff-current')
body=section('');body=body.replace('(docs/','(')
prepend(root/'docs/PROGRESS.md',body,'exploration-handoff-current')
path=root/'README.md';text=path.read_text().replace('当前工程完整交接 README（2026-10-22）','当前工程完整交接 README（2026-10-24）')
line='Frontier目标交接与近静止优化见[当前v17说明](docs/EXPLORATION_HANDOFF.md)和[匹配构建的实测结论](docs/evidence/exploration-handoff-20261024/final_conclusion.md)。\n'
if line not in text:
    anchor='ROS 2 当前仍通过 Gazebo 驱动车辆；';text=text.replace(anchor,line+anchor,1)
path.write_text(text)
path=root/'ros2_ws/README.md';text=path.read_text();line='Frontier交接、近静止时间与覆盖吞吐的当前v17验证见[说明](../docs/EXPLORATION_HANDOFF.md)和[本轮结果](../docs/evidence/exploration-handoff-20261024/final_conclusion.md)。\n'
if line not in text:
    first,rest=text.split('\n',1);text=first+'\n\n'+line+rest
path.write_text(text)
print('Updated root handoff, progress and README links with matched metrics')
