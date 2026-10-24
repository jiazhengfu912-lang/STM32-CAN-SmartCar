"""根据完整实测指标生成证据入口及验收正文，保留旧轮次。"""
from pathlib import Path
import json
root=Path.cwd();e=root/'docs/evidence/exploration-tail-20261022';a=e/'analysis_final_v14'
metrics=json.loads((a/'metrics.json').read_text())
notes={
'before_discovery':'本轮新跑的原 v6 基线；算法前沿消失不等于几何代理全覆盖。',
'before_office':'本轮新跑的原 v6 基线；终止搜索原因为 ROBOT_NOT_IN_SAFE_SPACE，被旧汇总名合并。',
'pilot_v7_discovery':'保留小前沿后过早插入零散补盲，频繁短目标；显式结束保存，未达完整任务结论。',
'after_discovery_r1':'v8 异步搜索/访问记忆；到达后位置退出安全区，后续改为检查实际停车末段。',
'after_office':'v8 开发轮显式结束后替换；保留全过程，不计入最终性能样本。',
'final_v9_discovery_r1':'v9 两段插入前瞻；尾段仍反复跨区，显式结束保存；68.598 m 不作为改善证据。',
'final_v10_discovery_r1':'v10 开放顺序；有限预算偏向同一区域，且观察目标可见性不够精确。自然部分结束；晚到的 finish 请求未发送，审计保留。',
'final_v10_office':'开发队列发生两套自有仿真重叠，原雷达/心跳保护暂停；显式结束保存。环境中断样本，不用于性能对比。',
'final_v11_discovery_r1':'与上一开发轮重叠的启动失败，FAIL 原样保留；保护未放宽。',
'final_v11_discovery_r1b':'v11 串行重试及区域预算；自然部分结束。22 次选中批次中 9 次看不到所分配前沿，后续增加指定目标可见性。',
'final_v11_office':'v11 初始已知区域仅 292 个安全通行格，统一宽观察余量导致 0 候选；v13 同图只读回放得到 72 个原 0.40 m 合法候选，另做新运动验证。',
'final_v11_discovery_r2':'v11 自然视点耗尽，25 目标、66.291 m，未解决尾段跨区。',
'final_v12_discovery_r1':'v12 已修正指定前沿可见性；旧访问抑制与按边界格数收益仍低效。26 目标、92.650 m，自然视点耗尽，完整保留。',
'final_v13_discovery_r1':'v13 开发完整验证，空白图，900 仿真秒预算；未知代理减少，但同阈值尾段成本退化。',
'final_v13_discovery_r2':'v13 开发重复轮，空白图、相同预算与速度；起点安全空间异常导致部分结束。',
'final_v13_office':'v13 开发办公室，95%→98% 路程退化为38.178m；据此修正v14排序目标。',
'final_v14_discovery_r1':'v14 保护中断轮：安全状态消息约2.37秒缺口触发输入陈旧锁存停车；核对会话后显式结束保存，不能当自然任务完成时间。95%→98% 阶段在中断前完成，独立列报。',
'final_v14_discovery_r2':'v14 同参数实验室重复轮，保留真实退化和中断。',
'final_v14_office':'v14 同参数办公室验证，重点比较实际95%→98%尾段成本。',
'watchdog_v14':'v14 失联锁存/显式恢复回归；独立验收器在冻结期间持续记录。',
'complete_v14':'v14 小房间自动完成分支回归，不外推大场景全覆盖。',
'watchdog_v13':'SIGSTOP/SIGCONT 注入：独立验收器继续记录速度，检查失联锁存、不自动续行和显式继续/暂停/结束保存。',
'complete_v13':'独立可观测小房间的自动完成分支回归；不外推到大场景全覆盖。'}
formal=['final_v14_discovery_r1','final_v14_discovery_r2','final_v14_office']
assessment=[]
for name in formal:
    baseline='before_office' if name.endswith('office') else 'before_discovery'
    before=metrics[baseline];after=metrics[name]
    first=before['observable_intervals']['.95->.98'];last=after['observable_intervals']['.95->.98']
    assessment.append({'run':name,'baseline':baseline,
        'unknown_reference_area_before_m2':before['observable_proxy']['missing_unknown_area_m2'],
        'unknown_reference_area_after_m2':after['observable_proxy']['missing_unknown_area_m2'],
        'observable_known_free_before':before['observable_proxy']['free_fraction'],
        'observable_known_free_after':after['observable_proxy']['free_fraction'],
        'tail95_to98_time_change_percent':100*(last['simulation_seconds']/first['simulation_seconds']-1) if first and last else None,
        'tail95_to98_travel_change_percent':100*(last['travel_m']/first['travel_m']-1) if first and last else None})
(a/'performance_assessment.json').write_text(json.dumps({'scope':'Engineering comparisons on measured fixed thresholds; not a preregistered statistical significance claim','cases':assessment},indent=2))
def f(x,d=3):
    """缺失保持缺失，不以零代替未达到阈值。"""
    return '未达到/缺测' if x is None else f'{x:.{d}f}'
def table(names,prefix=''):
    """同一表中保留任务状态和可观测代理，避免用功能PASS冒充全覆盖。"""
    rows=['|轮次|结果 / 终止|complete|中心已观测自由 %|几何可观测已知 %|几何可观测自由 %|未知代理 m²|行程 m|结束仿真秒|',
          '|---|---|---|---:|---:|---:|---:|---:|---:|']
    for name in names:
        m=metrics[name];p=m['observable_proxy']
        rows.append(f"|[{name}]({prefix}runs/{name}/result.json)|{m['result']} / `{m['reason']}`|{m['complete']}|{f(100*m['observed_fraction_final'])}|{f(100*p['known_fraction'])}|{f(100*p['free_fraction'])}|{f(p['missing_unknown_area_m2'])}|{f(m['travel_m'])}|{f(m['end_sim_s'])}|")
    return '\n'.join(rows)
pi='STM32 源码 v131，设备最后确认 v127，本轮未连接。P 右反 0.45→0.45，其余 0.30→0.30；I 稳态 0.010→0.010、启动 0.030→0.030，每 10 ms 按电机轴 rpm 误差累计，D=0。PI 未修改、未在 Gazebo 运行。NavFn/RPP、SLAM 配置、0.20 m/s、±0.35 rad/s、到达容差及原保护保持。波形为真实 ROS 仿真速度消息，全采样逐点、长缺口断线，不平滑、不补点。'
readme=['# 自主探索尾段补盲与访问规划实测证据','',
'本轮执行用户指定顺序：保留小前沿及替代观察点、修正访问/失败记忆；用实际 Nav2 路径优化区域访问；细分终止判定。最终代码为 v14，开发 v7—v13 和全部失败、中断同样保留。未经授权没有提交、推送、合并或操作实车。','',pi,'',
'## 最终独立运动轮次','',table(formal),'',
'功能 `PASS` 仅表示该验收用例中的导航、停车和保存检查通过；`complete` 与地图观测指标独立。大场景仍有未知区域时，不能把部分结束写成全图完成。原 v6 基线只新跑实验室/办公室各一轮；v14 实验室两轮，不构成统计显著性结论。','',
'**v13 开发轮尾段效率失败，完整保留。** 该版实验室首轮 95%→98% 用时 82.471→103.062 仿真秒，路程 11.091→12.995 m，存在退化。该轮未知参考面积 0.8575→0.4375 m²，但可观测自由比例 95.313%→95.099%，不能把更多“已知”直接称作自由区域测绘更准确。[逐场景比较](analysis_final_v14/performance_assessment.json)保留正反结果。静态矩阵的成本改善不能替代实际执行改善。','',
'## 同覆盖阈值的尾段成本','',
'覆盖阈值均由连续三次实际地图消息确认；不插值找更早时刻。下表分别显示 90%→95% 和 95%→98%，无法达到时保留缺测。行程由该区间实际里程计采样累加；提前故障结束不视为更快完成。','',
'|轮次|90→95% 仿真秒 / m|95→98% 仿真秒 / m|95→99% 仿真秒 / m|','|---|---:|---:|---:|']
for name in ['before_discovery','final_v13_discovery_r1','final_v13_discovery_r2','final_v14_discovery_r1','final_v14_discovery_r2','before_office','final_v13_office','final_v14_office']:
    m=metrics[name];parts=[]
    for key in ['.9->.95','.95->.98','.95->.99']:
        item=m['observable_intervals'][key];parts.append('未达到/缺测' if item is None else f(item['simulation_seconds'])+' / '+f(item['travel_m']))
    readme.append('|'+name+'|'+'|'.join(parts)+'|')
readme += ['', '数值分母：中心指标是 SDF 中与出生点连通、0.37 m 安全余量的中心位置被 SLAM 标为自由的比例；几何可观测指标另外使用 5 cm 真实自由采样点、上述安全中心到采样点的 12 m 无遮挡线段。真值只用于离线评价，不输入在线探索。地图外及未知均按未观测；已知占据与已知自由另报。代理并不代表实车精度、有限雷达光束保证或所有环境完整覆盖。最终在线图与实际导出 PGM 的几何误差分别见 metrics.json。','',
'## 全部尝试和失败','',table(list(metrics)),'']
for name in metrics:readme.append(f'- **{name}**：{notes.get(name,"完整原始记录保留。") } [逐点速度](analysis_final_v14/{name}_velocity.png) · [地图与轨迹](analysis_final_v14/{name}_map.png)。')
readme += ['',
'队列重叠发生在开发轮，已在 [queue_overlap_audit.json](source/queue_overlap_audit.json) 记录。最终 v14 使用进程文件锁、源码哈希和前轮完成检查，逐轮隔离 ROS_DOMAIN_ID 与 IGN/GZ_PARTITION，完整命令及退出码见 [serial_v14_results.json](software/serial_v14_results.json)。所有进程只清理自身进程组。','',
'## 实际 Nav2 路径与原因证据','',
'- [静态历史图真实 Nav2 规划](diagnostics/planner_v13/result.json)：只查询路径，不启动运动。6 个可用点、37 次查询，29 条有效候选间有向边；缺失边不当零代价。[独立穷举](diagnostics/planner_v13/order_comparison.json)得到同一顺序：贪心→开放顺序的预计代价 259.874→243.170 秒（低 6.43%），总路径 37.367→32.445 m。这里只比较固定矩阵，**不是实际整场探索节省比例**。',
'- 各运动会话 `mission/nav2_paths.jsonl` 保存实际返回路径、候选、Nav2 报告的规划耗时和估计行驶代价（原始 planning_cpu_s 字段实际是 planning_time，非进程 CPU 计时；派生 CSV 更正列名）；`mission/planning_maps/*.npz` 保存实际代价图及占据图。`analysis_final_v14/*_path_costs.csv` 可比较 BFS 初筛距离与 Nav2 真实路径；`*_execution_costs.csv` 逐目标对照预计长度/成本与实际里程/用时/观测收益，区分路径估计和执行结果。',
'- [旧指定前沿可见性审计](diagnostics/target_visibility_audit.json)：v10 的 18 次选择中 6 次、v11 串行首轮的 22 次中 9 次未看到被分配前沿；最终版增加指定目标可见性和发目标前复查。',
'- [初始余量回放](diagnostics/office_initial_margin_replay.json)：同一失败图从 0 候选到 72 个原合法余量观察点，只读回放不替代最终办公室新仿真。',
'- [v13办公室排序目标诊断](diagnostics/office_weighted_order_replay.json)：原排序为185.101秒、高收益时延225.137m²·s；同一实际Nav2矩阵的高收益优先顺序185.194秒、时延121.073m²·s，据此实施v14。单纯最短整批路程会在几乎同路程时先处理微小任务。v14采用收益时延最优顺序需满足最短成本+max(2秒,5%)预算；不合预算回退原顺序，不宣称约束内全局最优或整场执行必然省时。',
'- [独立分析](analysis_final_v14/metrics.json)：包括按次目标实际观测结果、采样缺口、实际矩阵穷举、几何代理、阈值时间及里程。不同候选子集的贪心顺序不计算“节省百分比”。','',
'## 文件、复现及完整性','',
'- [原代码](source/before_ws/)与[最终 v14 源码](source/final_v14_ws/)包含 src/tools/maps，运行时均从工作空间推导路径。`source/final_v13_ws` 保留 v13 退化版本，`source/final_ws` 和 `final_source_manifest.json` 是更早的 v11 开发冻结，名称保留原样，**不是最终 v14**。准确对应以每轮 source、result.json 的 SHA 和 [v14 冻结协议](source/v14_serial_protocol.json)为准。',
'- [软件日志](software/)保留构建、测试和中间失败；[环境与不变范围](environment.json)明确固件、协议和桌面控制台未改。',
'- 每个 `runs/<name>` 含 result.json、实际消息 CSV、事件、位姿探针、完整日志、每次地图快照、源文件；mission 内含 session、地图四件套和原始导航事件。所有失败和主动结束保留。',
'- [PNG/SVG 和派生表](analysis_final_v14/)；旧 analysis_v1/v2/v3 及 analysis_final（v13）为开发阶段分析，保留原样。当前工具位于 `ros2_ws/tools/report_exploration_tail.py`，只读输入，输出必须使用新的目录。',
'- [SHA256.json](SHA256.json)覆盖本证据目录除自身以外全部文件；[integrity.json](integrity.json)核对每轮原始哈希、最终源代码、保存四件套与进程清理，逐轮真实 map_server 重载结果在 diagnostics/reload_*。','',
'复现入口：先按仓库 `ros2_ws/tools/build.bash` 构建；`source ros2_ws/tools/env.bash`，使用新的输出目录、非 86 的独立 ROS 域运行 `verify_exploration.py`。完整最终串行命令保存在 `source/serial_v14.py`，包含文件锁及未完成轮次检查。不要把旧运行数据目录作为新输出，不并行启动另一套 Gazebo。']
(e/'README.md').write_text('\n'.join(readme)+'\n')
# 验收正文直接嵌入每轮波形，历史正文整体保留。
base='evidence/exploration-tail-20261022/'
section=['## 2026-10-22：可观测补盲、实际 Nav2 区域访问与终止诊断（v14）','',
'用户要求按“漏覆盖 → 尾段折返 → 终止原因”开始优化。算法修改包括小前沿保留、指定前沿可见性、替代观察余量、实际停车末段检查、按任务的有限访问记忆、面积收益、区域候选预算和实际 Nav2 有向开放访问顺序；v14 在接近最短整批行驶成本的预算内优先完成高收益观测。终止需连续确认，冷却不当完成。没有扩大速度、容差或关闭保护。','',pi,'',
'[算法说明](EXPLORATION_TAIL.md) · [全部结果/失败/原始数据](evidence/exploration-tail-20261022/README.md) · [完整指标](evidence/exploration-tail-20261022/analysis_final_v14/metrics.json)','',table(formal,base),'',
'这些结果分别报告任务状态和几何代理，功能 PASS 不等于全部可观测区域已覆盖。尾段时间与里程按同一稳定覆盖阈值比较，具体数值见证据入口；早期异常退出不能作为等价完成时间。','',
'**v13 开发轮不能认定“尾段长途折返已解决”。** 该版实验室首轮 95%→98% 路程 11.091→12.995 m、用时 82.471→103.062 仿真秒，均退化；同时未知参考面积 0.8575→0.4375 m²，可观测自由比例 95.313%→95.099%。实际 Nav2 固定矩阵顺序有收益，整场执行收益仍需继续优化，不能用软件通过或更多已知格替代此结论。','']
for world in ['discovery_lab','office_loop']:
    section += [f'### {world}：同尺度前后比较','',
    'P/I 前后值均沿用本节开头的明确数值，PI 未在仿真运行。原 v6 与最终 v14 使用相同场景、速度/保护和空白图入口。修改原因是小前沿过滤与错误观察任务造成漏覆盖，以及尾段缺少区域访问前瞻。地图黑/白/灰分别为占据/自由/未知，轨迹为被动真值记录，不向探索器反馈。','',
    f'![{world} 同尺度地图与实际轨迹]({base}analysis_final_v14/map_comparison_{world}.png)','',
    f'![{world} 同尺度真实命令与里程计波形]({base}analysis_final_v14/velocity_comparison_{world}.png)','',
    f'![{world} 完整与最后10%观测曲线]({base}analysis_final_v14/formal_coverage_{world}.png)','']
section += ['### v14 回归逐点波形（v13 全部开发失败保留在下节历史正文）','',
'以下各图 P/I 前后均为 P 右反 0.45→0.45、其余 0.30→0.30；I 稳态 0.010→0.010、启动 0.030→0.030，10 ms / 电机轴 rpm，D=0。PI 未修改且未执行；算法版本、原因与实测终止逐图标明，原始异常与缺测保持。','']
for name in metrics:
    if name in formal or 'v14' not in name:continue
    m=metrics[name]
    section += [f'**{name}**：{notes.get(name,"")} 实测 `{m["reason"]}`，complete={m["complete"]}，路程 {f(m["travel_m"])} m；停车验证 {m["stop_verified"]}。P/I 沿用上段明确值。','',f'![{name} 原始全采样速度与命令]({base}analysis_final_v14/{name}_velocity.png)','']
section += ['### 最终新图导航回归','',
'P/I 前后沿用本节明确值，PI 未在 Gazebo 运行。此项加载 v14 导出的新图，使用原普通 AMCL/NavFn/RPP 导航入口和原 0.30 m/s、0.60 rad/s 配置，验证一个短目标及持续停车；它不使用自主探索的 0.20/0.35 覆盖，不代表任意目标导航通过。实际结果以该用例 result.json 为准。','',
f'![v14 新图短目标完整速度与命令]({base}diagnostics/new_map_navigation_v14/velocity.png)','',
f'![v14 新图短目标实际轨迹]({base}diagnostics/new_map_navigation_v14/trajectory.png)','',
'最终构建、测试、重载地图和新图短目标导航的实测结果见证据入口软件/诊断日志；没有把未通过项改写成通过。全部变化留在本地，未提交、未推送、未合并、未连接实车。','']
path=root/'docs/ACCEPTANCE.md';text=path.read_text();heading=text.split('\n',1)[0];marker='## 2026-10-22：可观测补盲、实际 Nav2 区域访问与终止诊断（v14）'
if marker in text:
    previous='## 2026-10-22：可观测补盲、实际 Nav2 区域访问与终止诊断（v13）'
    remainder=text.split(previous,1)[1]
    old=previous+remainder
else:old=text.split('\n',1)[1].lstrip('\n')
path.write_text(heading+'\n\n'+'\n'.join(section)+'\n'+old)
print('Evidence README and ACCEPTANCE generated for',len(metrics),'completed runs')
