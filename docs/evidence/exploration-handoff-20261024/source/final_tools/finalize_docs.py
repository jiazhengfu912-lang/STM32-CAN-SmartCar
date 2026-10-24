"""从最终真实指标生成本轮验收正文；旧章节和失败数据原样保留。"""
from pathlib import Path
import json,re
root=Path.cwd();e=root/'docs/evidence/exploration-handoff-20261024';a=e/'analysis/final_v17'
metrics=json.loads((a/'metrics.json').read_text());protocol=json.loads((e/'source/protocol_v17.json').read_text())
names=['before_v15_office','before_v15_discovery','before_v15_matched_discovery','before_v15_matched_office','final_v16_discovery','final_v16_office','final_v17_discovery','final_v17_office','watchdog_v17','complete_v17']
if not all(name in metrics for name in names):raise RuntimeError('Required run is unfinished')

def number(value,digits=3):
    """缺测保留为缺测，不能把空值格式化为零或通过。"""
    return '缺测' if value is None else f'{value:.{digits}f}'

def percent(before,after):
    """仅在原始指标均存在且基线非零时计算百分比变化。"""
    return None if before is None or after is None or before==0 else (after/before-1)*100

def interval(name,key):
    """统一读取完整原始区间派生值，不使用近似历史估计补缺。"""
    return metrics[name].get('handoff_timing',{}).get('intervals',{}).get(key)

comparisons={};table=['|场景/覆盖区间|用时 v15→v17 s|近静止 v15→v17 s|路程 v15→v17 m|参考面积吞吐变化|',
                      '|---|---:|---:|---:|---:|']
for before,after in protocol['formal_pairs']:
    comparisons[after]={}
    for key in ['0.9->0.95','0.9->0.98','0.95->0.98','0.98->0.99']:
        b,c=interval(before,key),interval(after,key)
        if b is None or c is None:comparisons[after][key]=None;continue
        comparisons[after][key]={field:percent(b.get(field),c.get(field)) for field in
           ['simulation_seconds','near_stationary_sim_s','travel_m','normalized_reference_area_m2_per_sim_s']}
        gain=comparisons[after][key]['normalized_reference_area_m2_per_sim_s']
        table.append(f"|{after.split('_')[-1]} / {key}|{number(b['simulation_seconds'])}→{number(c['simulation_seconds'])}|"
                     f"{number(b['near_stationary_sim_s'])}→{number(c['near_stationary_sim_s'])}|"
                     f"{number(b['travel_m'])}→{number(c['travel_m'])}|{number(gain,1)}%|")
(a/'comparisons.json').write_text(json.dumps(comparisons,indent=2))
handoff=['|场景|交接次数 v15→v17|交接中位数 s|累计交接间隔 s|交接内近静止 s|',
         '|---|---:|---:|---:|---:|']
for before,after in protocol['formal_pairs']:
    b=metrics[before]['handoff_timing']['handoffs'];c=metrics[after]['handoff_timing']['handoffs']
    handoff.append(f"|{after.split('_')[-1]}|{b['count']}→{c['count']}|{number(b['median_gap_sim_s'])}→{number(c['median_gap_sim_s'])}|"
                   f"{number(b['total_gap_sim_s'])}→{number(c['total_gap_sim_s'])}|{number(b['total_near_stationary_sim_s'])}→{number(c['total_near_stationary_sim_s'])}|")
outcomes=['|轮次|结果/终止原因|几何可观测已知/自由 %|参考未知 m²|总路程 m|导出占据表面 P95 m|',
          '|---|---|---:|---:|---:|---:|']
for name in names:
    s=metrics[name];proxy=s.get('observable_proxy') or {};geometry=s.get('exported_map_geometry_proxy') or {}
    known=proxy.get('known_fraction');free=proxy.get('free_fraction')
    outcomes.append(f"|{name}|{s.get('result')} / `{s.get('reason')}`|{number(None if known is None else known*100)} / {number(None if free is None else free*100)}|"
                    f"{number(proxy.get('missing_unknown_area_m2'),4)}|{number(s.get('travel_m'))}|{number(geometry.get('surface_distance_p95_m'))}|")
intro='''## 2026-10-24 探索v17：近静止耗时与Frontier交接

本轮以冻结v15为基线，串行运行相同实验室/办公室及恢复、小房间回归。发现最初隔离基线CMake构建等级为空、新版为RelWithDebInfo后，保留旧轮次为开发参考，并补跑两轮RelWithDebInfo匹配基线；下方正式表仅使用匹配基线。原流程在目标结果后才搜索，并让搜索、查询和派发共享1秒节流，造成空等。v16开发轮出现同覆盖区间退化，保留但未采用；v17移除跨目标预备候选，以最新地图重新选择，等价缓存射线模板并限制区域广搜范围，移除共享交接节流，同批复用几何检查；已有完整新鲜观测证据时免除额外等帧。NavFn/RPP、SLAM、速度、到达容差和停机保护未改。固件源码v131、设备最后确认v127，未连接硬件，PI未在Gazebo中执行。

所有下图P/I均为前→后不变：右反向P 0.45→0.45、其余P 0.30→0.30；稳态I 0.010→0.010、启动I 0.030→0.030，每10 ms积分、电机轴rpm、D=0。图中的算法版本是探索版本，不能当作固件或实车安装版本。

软件构建通过；v17的105项colcon检查0错误、0失败、0跳过，数量包含包装，不是105轮Gazebo。v16曾有108项软件检查通过，但运动吞吐仍退化，软件检查不能替代性能验证。v17移除未采用预备逻辑及对应测试，新增限界广搜回归。初次隔离构建漏拷地图、初次测试误扫描到归档工作区的命令错误也保留日志；最终结果以重新执行成功的日志为准。测试未结束时读取到的中间缺结果日志已明确标注，最终报告在所有测试结束后重新读取。

近静止按原始里程计`|v|<0.02 m/s`且`|w|<0.05 rad/s`判定，超过1仿真秒的缺口断线并单列，不补点、不平滑。阶段与速度的时间关联不能当CPU耗时。区间统计只使用完整落入窗口的原始采样间隔；相较历史工具的末端采样规则可有一个样本的边界差异，本轮新旧同规则。

'''
body=intro+'\n'.join(table)+'\n\n'+'\n'.join(handoff)+'\n\n'+'\n'.join(outcomes)+'\n\n'
body+='目标次数不同，交接累计量需要结合中位数与同覆盖区间判断；吞吐按固定参考面积/实测用时计算，不以更多零碎目标充当有效覆盖。partial仍是部分图，单场景一对样本不构成统计显著性或实车结论。\n\n'
for world in ['discovery','office']:
    body+=f'### {world}：同尺度前后原始波形与覆盖\n\n'
    body+='算法v15→v17；P右反向0.45→0.45、其余0.30→0.30；I稳态0.010→0.010、启动0.030→0.030，每10 ms、电机轴rpm、D=0，PI未在Gazebo执行。目标是减少交接空等，实测变化见上表，覆盖与吞吐分别判断。\n\n'
    for label,file in [('原始里程计；橙色为目标结果到下一次发目标的交接区间','handoff_wave'),('逐次交接与完整采样覆盖尾段','handoff_coverage')]:
        path=f'evidence/exploration-handoff-20261024/analysis/final_v17/final_v17_{file}_{world}.png'
        if not (root/'docs'/path).is_file():raise FileNotFoundError(path)
        body+=f'![{world} {label}]({path})\n\n'
body+='### v16开发退化：保留同覆盖阈值的反例\n\nv16实验室95%→98%相对原开发参考从94.039秒增至231.298秒；原参考的编译等级不同，此处只保留问题记录，不作v17正式量化对照，不能因交接更快就称覆盖吞吐改善。办公室在4个目标后提前失败/中断（已记录运动7.336米）、没有最终地图，不作为路线性能样本；原日志错误和缺测均保留，归因不超出证据。P右反向0.45→0.45、其余0.30→0.30；I稳态0.010→0.010、启动0.030→0.030，每10 ms、电机轴rpm、D=0，PI未在Gazebo执行。\n\n'
for file in ['handoff_wave','handoff_coverage']:
    path=f'evidence/exploration-handoff-20261024/analysis/final_v17/final_v16_{file}_discovery.png'
    if (root/'docs'/path).is_file():body+=f'![v16真实退化 {file}]({path})\n\n'
for name in names:
    s=metrics[name];version=('v15匹配构建基线' if '_matched_' in name else 'v15开发参考，编译等级不同') if name.startswith('before_') else ('v16退化/提前中断，未采用' if '_v16_' in name else 'v17修改后')
    body+=f"### {name}（{version}）\n\n实测：{s.get('result')}，`{s.get('reason')}`，complete={s.get('complete')}；"
    algorithm='v15原交接与v14观测/路线' if name.startswith('before_') else ('v16预备搜索，保留失败' if '_v16_' in name else 'v17最新地图搜索、等价射线模板/限界广搜、事件交接、完整新鲜观测复用')
    body+=f'算法：{algorithm}；P右反向0.45→0.45、其余0.30→0.30；I稳态0.010→0.010、启动0.030→0.030，每10 ms、电机轴rpm、D=0，Gazebo未执行PI。保留启动、行驶、停顿、停车及故障缺测。\n\n'
    for suffix,label in [('_velocity','独立验收器原始速度与安全输出'),('_map','最终地图与真实轨迹；红点为几何代理未知')]:
        path=f'evidence/exploration-handoff-20261024/analysis/final_v17/{name}{suffix}.png'
        if (root/'docs'/path).is_file():body+=f'![{name} {label}]({path})\n\n'
    body+=f'[原始数据与全部检查](evidence/exploration-handoff-20261024/runs/{name}/result.json)；[SVG速度波形](evidence/exploration-handoff-20261024/analysis/final_v17/{name}_velocity.svg)。\n\n'
body+='详见[算法与测量方法](EXPLORATION_HANDOFF.md)、[本轮完整证据](evidence/exploration-handoff-20261024/README.md)、[派生指标](evidence/exploration-handoff-20261024/analysis/final_v17/metrics.json)。\n\n'
accept=root/'docs/ACCEPTANCE.md';old=accept.read_text();marker='<!-- exploration-handoff-v17:start -->'
section=marker+'\n'+body+'<!-- exploration-handoff-v17:end -->\n\n'
if marker in old:old=re.sub(re.escape(marker)+r'.*?<!-- exploration-handoff-v17:end -->\s*',lambda _:section,old,flags=re.S)
else:
    first,rest=old.split('\n',1);old=first+'\n\n'+section+rest
accept.write_text(old)
readme=body.split('### discovery')[0]
readme+='[验收正文与直接嵌入的全部波形](../../ACCEPTANCE.md)；[算法说明](../../EXPLORATION_HANDOFF.md)。\n\n'
readme+='原始文件在各轮`runs/`；源码快照与预设协议在`source/`；软件日志在`software/`；派生PNG/SVG/逐段CSV/指标在`analysis/final_v17/`。基线、修改后、失败、缺测均保留，完整性清单只表示文件未变，不表示运动或覆盖通过。最终源码未提交、未推送，未连接实物。\n'
(e/'README.md').write_text(readme)
(e/'final_conclusion.md').write_text(readme)
print('Generated evidence README, conclusion and directly embedded acceptance waves')
