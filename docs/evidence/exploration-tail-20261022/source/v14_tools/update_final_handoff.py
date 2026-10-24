"""从最终已完成原始记录更新交接和实测结论，不预填通过或改善比例。"""
from pathlib import Path
import json,re,datetime
root=Path.cwd();e=root/'docs/evidence/exploration-tail-20261022';metrics=json.loads((e/'analysis_final_v14/metrics.json').read_text());formal=['final_v14_discovery_r1','final_v14_discovery_r2','final_v14_office']
log=(e/'software/colcon_v14.log').read_text();tests=re.findall(r'Summary: (\d+) tests, (\d+) errors, (\d+) failures, (\d+) skipped',log)
if not tests:raise RuntimeError('Final colcon summary missing')
count,errors,failures,skipped=map(int,tests[-1]);software=f'{count} 项 colcon 检查，{errors} 错误、{failures} 失败、{skipped} 跳过'
nav=json.loads((e/'diagnostics/new_map_navigation_v14/result.json').read_text())
reloads={name:json.loads((e/'diagnostics'/('reload_'+name)/'result.json').read_text()) for name in formal}
summary=['### v14 最终实测判断','',software+'（数量包含测试包装，不是同等数量的Gazebo实验）；软件检查与运动性能分别评价。','']
for name in formal:
    m=metrics[name];base=metrics['before_office' if name.endswith('office') else 'before_discovery'];old=metrics['final_v13_office' if name.endswith('office') else 'final_v13_discovery_r1'];p=m['observable_proxy'];interval=m['observable_intervals']['.95->.98'];b=base['observable_intervals']['.95->.98'];v13=old['observable_intervals']['.95->.98']
    text=f"- {name}（{m.get('run_classification','见原始记录')}）：`{m['reason']}` / complete={m['complete']}；未知参考面积 {base['observable_proxy']['missing_unknown_area_m2']:.4f}→{p['missing_unknown_area_m2']:.4f} m²，几何可观测已知 {p['known_fraction']*100:.3f}%、已知自由 {p['free_fraction']*100:.3f}%；总行程 {m['travel_m']:.3f} m。"
    if interval and b:
        text+=f"95%→98% 实际 {interval['simulation_seconds']:.3f} 仿真秒 / {interval['travel_m']:.3f} m，相对原 v6 的用时/里程变化 {100*(interval['simulation_seconds']/b['simulation_seconds']-1):+.1f}% / {100*(interval['travel_m']/b['travel_m']-1):+.1f}%。"
    if interval and v13:text+=f"v13 同场景首轮该区间为 {v13['simulation_seconds']:.3f} 秒 / {v13['travel_m']:.3f} m。"
    summary.append(text)
summary+=['','95%→98% 阶段的原始速度积分分解如下；近静止不直接等同于规划耗时，原地转向不直接等同于某一控制状态。超过1仿真秒的缺口单列，不补点。','',
    '|轮次|近静止 s|原地转向 s|平移 s|未分类缺口 s|','|---|---:|---:|---:|---:|']
for name in ['before_discovery','final_v13_discovery_r1','final_v13_discovery_r2','final_v14_discovery_r1','final_v14_discovery_r2','before_office','final_v13_office','final_v14_office']:
    item=metrics[name]['observable_intervals']['.95->.98']
    values=['未达到']*4 if item is None else [f'{item[key]:.3f}' for key in ['near_stationary_sim_s','turning_in_place_sim_s','translation_sim_s','unclassified_gap_sim_s']]
    summary.append('|'+name+'|'+'|'.join(values)+'|')
summary+=['','所有大场景若仍为 partial，就不报告全图完成；更多已知、更多自由和更高几何准确度分别判断。同覆盖阈值数据有退化时保留，不以静态矩阵结果替代实际执行。三轮样本不构成统计显著性或实车性能结论。','',
'逐格重载：'+ '；'.join(f"{name} {data['result']}，未知格 {data['unknown_expected']}→{data['unknown_reloaded']}" for name,data in reloads.items())+'。',
f"最终新图短目标导航：{nav['result']}，{nav.get('error','目标/停车检查见实际 checks')}。v13 首次启动未就绪 FAIL 和相同参数重试 PASS 均保留。",'']
(e/'final_conclusion.md').write_text('\n'.join(summary))
p=e/'README.md';s=p.read_text();heading,body=s.split('\n',1);p.write_text(heading+'\n\n'+'\n'.join(summary)+'\n'+body.lstrip('\n'))
p=root/'docs/ACCEPTANCE.md';s=p.read_text();marker='### discovery_lab：同尺度前后比较';s=s.replace(marker,'\n'.join(summary)+'\n'+marker,1);p.write_text(s)
block='''<!-- exploration-tail-current:start -->
## 本地尾段优化 v14：本轮实现与验证已归档

用户已授权按“补盲遗漏 → 实际 Nav2 区域访问 → 终止诊断”修改和运行隔离仿真。五个运行模块按 v14 冻结，保留小前沿、指定前沿可见性、替代视点和按任务访问记忆；尾段用真实 Nav2 路径，在接近最短整批成本的预算内优先完成高收益观测。终止原因分别报告，不以冷却或部分结果冒充全图完成。

'''+software+'。最终运动、地图和保留失败见 [本轮结论](docs/evidence/exploration-tail-20261022/final_conclusion.md)、[完整证据](docs/evidence/exploration-tail-20261022/README.md) 和 [算法说明](docs/EXPLORATION_TAIL.md)。v13 的实际尾段退化用于修正 v14，原记录和原图未覆盖。工作区未提交；未授权提交、推送、合并、烧录或实车运动。旧交接中的“仅整理交接文档”是先前阶段，不能代替本次状态。\n\n'
block+='所有本轮自建仿真结束并检查进程清理后归档；重新启动前仍需核对实时进程，不复用已有输出目录。固件源码 v131、设备最后确认 v127；PI、NavFn/RPP、SLAM 配置、速度及停机保护未改，未连接硬件。Git HEAD 仍为 b77c32cbdaf06085ed7d2c2ff418c7d183bddad0，本轮没有 Git 远端变更。\n<!-- exploration-tail-current:end -->\n'
for name in ['README_HANDOFF.md','HANDOFF.md']:
 p=root/name;s=p.read_text();s=re.sub(r'<!-- exploration-tail-current:start -->.*?<!-- exploration-tail-current:end -->',block.strip(),s,flags=re.S);p.write_text(s)
p=root/'docs/PROGRESS.md';s=p.read_text();heading,body=s.split('\n',1);entry='2026-10-22 后续：[尾段补盲与实际 Nav2 区域访问](EXPLORATION_TAIL.md)已实现 v14，'+software+'。小前沿/替代视角/指定目标复查、有限访问记忆与细分终止已落地；v13 实测尾段退化保留，v14 增加有总成本预算回退的信息完成时延排序。最终各场景的覆盖、同阈值里程、失败及边界见[本轮结论](evidence/exploration-tail-20261022/final_conclusion.md)。部分任务不称全图完成，软件通过不等于效率均改善；本轮未提交、推送、合并或连接硬件。\n\n';p.write_text(heading+'\n\n'+entry+body.lstrip('\n'))
p=root/'README.md';s=p.read_text();needle='自主探索的目标选择、转向和等待优化见[效率优化说明](docs/EXPLORATION_SPEED.md)。';s=s.replace(needle,needle+'\n本地后续的补盲、实际 Nav2 区域访问与终止诊断见[尾段优化说明](docs/EXPLORATION_TAIL.md)及[最新实测结论](docs/evidence/exploration-tail-20261022/final_conclusion.md)。');p.write_text(s)
p=root/'docs/EXPLORATION_TAIL.md';s=p.read_text().replace('v13 的失败结果全部保留，v14 当前正在相同场景/速度/预算下做新的独立仿真，结论待实测。','v13 的失败结果全部保留，v14 已在相同场景/速度/预算下完成新的独立仿真。实际正反结果见[最终实测判断](evidence/exploration-tail-20261022/final_conclusion.md)，不把固定矩阵优化等同于全场景效率保证。');p.write_text(s)
p=e/'environment.json';r=json.loads(p.read_text());r['final_runtime_protocol']='source/v14_serial_protocol.json';r['final_recorded_utc']=datetime.datetime.now(datetime.timezone.utc).isoformat();p.write_text(json.dumps(r,indent=2))
print(software,'and final handoff updated')
