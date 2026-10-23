# 自主探索尾段补盲与访问规划实测证据

### v14 路线性能与 v15 恢复验证

102 项 colcon 检查，0 错误、0 失败、0 跳过（数量包含测试包装，不是同等数量的Gazebo实验）；软件检查与运动性能分别评价。

实测判断：补盲范围扩大，办公室未知参考面积降至0.4425m²；但相对原v6，三轮95%→98%里程仍高0.6%—1.6%，用时仍高16.6%—33.4%，不能宣称整体更快或折返问题已完全解决。新增耗时中约56%—69%落在近静止区间，说明减少停靠与等待仍是后续重点；该分类不是对规划CPU耗时的直接测量。

- final_v14_discovery_r1（PROTECTED_INTERRUPTED_FINISHED）：`USER_FINISHED_PARTIAL` / complete=False；未知参考面积 0.8575→0.4875 m²，几何可观测已知 99.316%、已知自由 94.784%；总行程 56.539 m。95%→98% 实际 96.179 仿真秒 / 11.265 m，相对原 v6 的用时/里程变化 +16.6% / +1.6%。v13 同场景首轮该区间为 103.062 秒 / 12.995 m。
- final_v14_discovery_r2（NATURAL_TERMINATION）：`NAV2_UNREACHABLE_FRONTIERS_PARTIAL` / complete=False；未知参考面积 0.8575→0.4725 m²，几何可观测已知 99.337%、已知自由 95.299%；总行程 49.229 m。95%→98% 实际 105.130 仿真秒 / 11.161 m，相对原 v6 的用时/里程变化 +27.5% / +0.6%。v13 同场景首轮该区间为 103.062 秒 / 12.995 m。
- final_v14_office（NATURAL_TERMINATION）：`NAV2_UNREACHABLE_FRONTIERS_PARTIAL` / complete=False；未知参考面积 1.7600→0.4425 m²，几何可观测已知 99.591%、已知自由 95.622%；总行程 87.131 m。95%→98% 实际 144.681 仿真秒 / 14.940 m，相对原 v6 的用时/里程变化 +33.4% / +1.4%。v13 同场景首轮该区间为 336.364 秒 / 38.178 m。

95%→98% 阶段的原始速度积分分解如下；近静止不直接等同于规划耗时，原地转向不直接等同于某一控制状态。超过1仿真秒的缺口单列，不补点。

|轮次|近静止 s|原地转向 s|平移 s|未分类缺口 s|
|---|---:|---:|---:|---:|
|before_discovery|2.281|14.759|65.420|0.000|
|final_v13_discovery_r1|9.111|14.681|79.269|0.000|
|final_v13_discovery_r2|20.043|17.137|81.200|0.000|
|final_v14_discovery_r1|11.663|15.580|68.936|0.000|
|final_v14_discovery_r2|17.111|17.558|70.471|0.000|
|before_office|6.575|11.760|90.125|0.000|
|final_v13_office|36.168|58.656|241.536|0.000|
|final_v14_office|26.904|19.766|98.006|0.000|

v14失联回归的首次FAIL保留：停车锁存和不自动续行通过，显式恢复后超时、缺失最终地图。v15仅修复恢复就绪查询并新增等待诊断；以下是独立新回归，不能替代v14大场景性能数据。

- watchdog_v15_r1: PASS，失联停车锁存通过、心跳恢复不续行通过、显式恢复新目标通过、显式暂停停车通过、停车保存通过。
- watchdog_v15_r2: PASS，失联停车锁存通过、心跳恢复不续行通过、显式恢复新目标通过、显式暂停停车通过、停车保存通过。
- complete_v15: PASS，前沿消失完成通过、停车验证通过、地图保存通过。


所有大场景若仍为 partial，就不报告全图完成；更多已知、更多自由和更高几何准确度分别判断。同覆盖阈值数据有退化时保留，不以静态矩阵结果替代实际执行。三轮样本不构成统计显著性或实车性能结论。

逐格重载：final_v14_discovery_r1 PASS，未知格 1905→1905；final_v14_discovery_r2 PASS，未知格 1866→1866；final_v14_office PASS，未知格 2419→2419。
最终新图短目标导航：PASS，目标到达、实际运动及持续停车通过。v13 首次启动未就绪 FAIL 和相同参数重试 PASS 均保留。

本轮执行用户指定顺序：保留小前沿及替代观察点、修正访问/失败记忆；用实际 Nav2 路径优化区域访问；细分终止判定。当前代码为 v15（生命周期查询时效与显式恢复修复），路线/观测策略保持 v14。完整探索性能来自冻结 v14，其中实验室首轮保护中断；v15只重测恢复、启动与保存分支，未重跑完整大场景。开发 v7—v14 和全部失败、中断同样保留。未经授权没有提交、推送、合并或操作实车。

STM32 源码 v131，设备最后确认 v127，本轮未连接。P 右反 0.45→0.45，其余 0.30→0.30；I 稳态 0.010→0.010、启动 0.030→0.030，每 10 ms 按电机轴 rpm 误差累计，D=0。PI 未修改、未在 Gazebo 运行。NavFn/RPP、SLAM 配置、0.20 m/s、±0.35 rad/s、到达容差及原保护保持。波形为真实 ROS 仿真速度消息，全采样逐点、长缺口断线，不平滑、不补点。

## 最终独立运动轮次

|轮次|结果 / 终止|complete|中心已观测自由 %|几何可观测已知 %|几何可观测自由 %|未知代理 m²|行程 m|结束仿真秒|
|---|---|---|---:|---:|---:|---:|---:|---:|
|[final_v14_discovery_r1](runs/final_v14_discovery_r1/result.json)|PASS / `USER_FINISHED_PARTIAL`|False|100.000|99.316|94.784|0.488|56.539|555.508|
|[final_v14_discovery_r2](runs/final_v14_discovery_r2/result.json)|PASS / `NAV2_UNREACHABLE_FRONTIERS_PARTIAL`|False|100.000|99.337|95.299|0.473|49.229|450.071|
|[final_v14_office](runs/final_v14_office/result.json)|PASS / `NAV2_UNREACHABLE_FRONTIERS_PARTIAL`|False|100.000|99.591|95.622|0.443|87.131|760.603|

功能 `PASS` 仅表示该验收用例中的导航、停车和保存检查通过；`complete` 与地图观测指标独立。大场景仍有未知区域时，不能把部分结束写成全图完成。原 v6 基线只新跑实验室/办公室各一轮；v14 实验室两轮，不构成统计显著性结论。

**v13 开发轮尾段效率失败，完整保留。** 该版实验室首轮 95%→98% 用时 82.471→103.062 仿真秒，路程 11.091→12.995 m，存在退化。该轮未知参考面积 0.8575→0.4375 m²，但可观测自由比例 95.313%→95.099%，不能把更多“已知”直接称作自由区域测绘更准确。[逐场景比较](analysis_final_v15/performance_assessment.json)保留正反结果。静态矩阵的成本改善不能替代实际执行改善。

## 同覆盖阈值的尾段成本

覆盖阈值均由连续三次实际地图消息确认；不插值找更早时刻。下表分别显示 90%→95% 和 95%→98%，无法达到时保留缺测。行程由该区间实际里程计采样累加；提前故障结束不视为更快完成。

|轮次|90→95% 仿真秒 / m|95→98% 仿真秒 / m|95→99% 仿真秒 / m|
|---|---:|---:|---:|
|before_discovery|7.732 / 1.548|82.471 / 11.091|未达到/缺测|
|final_v13_discovery_r1|24.086 / 2.871|103.062 / 12.995|182.574 / 22.673|
|final_v13_discovery_r2|7.141 / 1.428|118.378 / 11.493|279.968 / 28.569|
|final_v14_discovery_r1|19.279 / 1.908|96.179 / 11.265|199.244 / 25.104|
|final_v14_discovery_r2|9.713 / 1.571|105.130 / 11.161|170.207 / 19.599|
|before_office|20.214 / 2.551|108.463 / 14.740|未达到/缺测|
|final_v13_office|19.748 / 2.494|336.364 / 38.178|未达到/缺测|
|final_v14_office|25.916 / 3.190|144.681 / 14.940|262.549 / 28.078|

数值分母：中心指标是 SDF 中与出生点连通、0.37 m 安全余量的中心位置被 SLAM 标为自由的比例；几何可观测指标另外使用 5 cm 真实自由采样点、上述安全中心到采样点的 12 m 无遮挡线段。真值只用于离线评价，不输入在线探索。地图外及未知均按未观测；已知占据与已知自由另报。代理并不代表实车精度、有限雷达光束保证或所有环境完整覆盖。最终在线图与实际导出 PGM 的几何误差分别见 metrics.json。

## 全部尝试和失败

|轮次|结果 / 终止|complete|中心已观测自由 %|几何可观测已知 %|几何可观测自由 %|未知代理 m²|行程 m|结束仿真秒|
|---|---|---|---:|---:|---:|---:|---:|---:|
|[after_discovery_r1](runs/after_discovery_r1/result.json)|PASS / `START_POSE_UNSAFE_PARTIAL`|False|99.735|98.282|95.106|1.225|37.697|342.825|
|[after_office](runs/after_office/result.json)|PASS / `USER_FINISHED_PARTIAL`|False|99.180|94.124|90.333|6.363|16.717|153.121|
|[before_discovery](runs/before_discovery/result.json)|PASS / `NO_FRONTIERS`|True|100.000|98.798|95.313|0.858|43.822|345.095|
|[before_office](runs/before_office/result.json)|PASS / `NO_REACHABLE_FRONTIERS_PARTIAL`|False|100.000|98.375|94.399|1.760|43.524|315.367|
|[complete_v13](runs/complete_v13/result.json)|PASS / `NO_FRONTIERS`|True|100.000|100.000|95.036|0.000|1.657|44.416|
|[complete_v14](runs/complete_v14/result.json)|PASS / `NO_FRONTIERS`|True|100.000|100.000|96.811|0.000|1.404|28.842|
|[complete_v15](runs/complete_v15/result.json)|PASS / `NO_FRONTIERS`|True|100.000|100.000|95.431|0.000|0.363|20.177|
|[final_v10_discovery_r1](runs/final_v10_discovery_r1/result.json)|PASS / `VIEWPOINTS_EXHAUSTED_PARTIAL`|False|100.000|98.931|95.071|0.763|53.144|479.035|
|[final_v10_office](runs/final_v10_office/result.json)|PASS / `USER_FINISHED_PARTIAL`|False|99.973|96.876|92.300|3.383|23.063|303.365|
|[final_v11_discovery_r1](runs/final_v11_discovery_r1/result.json)|FAIL / `USER_FINISHED_PARTIAL`|False|16.859|10.292|10.064|63.978|0.003|91.528|
|[final_v11_discovery_r1b](runs/final_v11_discovery_r1b/result.json)|PASS / `NO_OBSERVABLE_FRONTIERS_PARTIAL`|False|100.000|99.117|95.632|0.630|63.115|591.626|
|[final_v11_discovery_r2](runs/final_v11_discovery_r2/result.json)|PASS / `VIEWPOINTS_EXHAUSTED_PARTIAL`|False|100.000|99.015|94.784|0.703|66.291|665.869|
|[final_v11_office](runs/final_v11_office/result.json)|FAIL / `NO_REACHABLE_FRONTIERS_PARTIAL`|False|5.176|4.719|4.530|103.168|0.000|15.305|
|[final_v12_discovery_r1](runs/final_v12_discovery_r1/result.json)|PASS / `VIEWPOINTS_EXHAUSTED_PARTIAL`|False|100.000|99.723|95.236|0.198|92.650|790.521|
|[final_v13_discovery_r1](runs/final_v13_discovery_r1/result.json)|PASS / `NAV2_UNREACHABLE_FRONTIERS_PARTIAL`|False|100.000|99.387|95.099|0.438|57.657|491.917|
|[final_v13_discovery_r2](runs/final_v13_discovery_r2/result.json)|PASS / `START_POSE_UNSAFE_PARTIAL`|False|100.000|99.327|94.819|0.480|54.032|541.304|
|[final_v13_office](runs/final_v13_office/result.json)|PASS / `NAV2_UNREACHABLE_FRONTIERS_PARTIAL`|False|100.000|98.915|94.246|1.175|72.872|657.596|
|[final_v14_discovery_r1](runs/final_v14_discovery_r1/result.json)|PASS / `USER_FINISHED_PARTIAL`|False|100.000|99.316|94.784|0.488|56.539|555.508|
|[final_v14_discovery_r2](runs/final_v14_discovery_r2/result.json)|PASS / `NAV2_UNREACHABLE_FRONTIERS_PARTIAL`|False|100.000|99.337|95.299|0.473|49.229|450.071|
|[final_v14_office](runs/final_v14_office/result.json)|PASS / `NAV2_UNREACHABLE_FRONTIERS_PARTIAL`|False|100.000|99.591|95.622|0.443|87.131|760.603|
|[final_v9_discovery_r1](runs/final_v9_discovery_r1/result.json)|PASS / `USER_FINISHED_PARTIAL`|False|100.000|99.018|95.113|0.700|68.598|568.185|
|[pilot_v7_discovery](runs/pilot_v7_discovery/result.json)|PASS / `USER_FINISHED_PARTIAL`|False|85.187|78.287|75.977|15.485|11.392|190.011|
|[watchdog_v13](runs/watchdog_v13/result.json)|PASS / `USER_FINISHED_PARTIAL`|False|42.859|31.426|30.490|48.905|0.046|45.106|
|[watchdog_v14](runs/watchdog_v14/result.json)|FAIL / `INTERRUPTED_PARTIAL`|False|未达到/缺测|未达到/缺测|未达到/缺测|未达到/缺测|未达到/缺测|未达到/缺测|
|[watchdog_v15_r1](runs/watchdog_v15_r1/result.json)|PASS / `USER_FINISHED_PARTIAL`|False|42.076|30.841|29.947|49.323|0.106|34.007|
|[watchdog_v15_r2](runs/watchdog_v15_r2/result.json)|PASS / `USER_FINISHED_PARTIAL`|False|42.775|31.584|30.652|48.793|0.114|31.461|

- **after_discovery_r1**：v8 异步搜索/访问记忆；到达后位置退出安全区，后续改为检查实际停车末段。 [逐点速度](analysis_final_v15/after_discovery_r1_velocity.png) · [地图与轨迹](analysis_final_v15/after_discovery_r1_map.png)。
- **after_office**：v8 开发轮显式结束后替换；保留全过程，不计入最终性能样本。 [逐点速度](analysis_final_v15/after_office_velocity.png) · [地图与轨迹](analysis_final_v15/after_office_map.png)。
- **before_discovery**：本轮新跑的原 v6 基线；算法前沿消失不等于几何代理全覆盖。 [逐点速度](analysis_final_v15/before_discovery_velocity.png) · [地图与轨迹](analysis_final_v15/before_discovery_map.png)。
- **before_office**：本轮新跑的原 v6 基线；终止搜索原因为 ROBOT_NOT_IN_SAFE_SPACE，被旧汇总名合并。 [逐点速度](analysis_final_v15/before_office_velocity.png) · [地图与轨迹](analysis_final_v15/before_office_map.png)。
- **complete_v13**：独立可观测小房间的自动完成分支回归；不外推到大场景全覆盖。 [逐点速度](analysis_final_v15/complete_v13_velocity.png) · [地图与轨迹](analysis_final_v15/complete_v13_map.png)。
- **complete_v14**：v14 小房间自动完成分支回归，不外推大场景全覆盖。 [逐点速度](analysis_final_v15/complete_v14_velocity.png) · [地图与轨迹](analysis_final_v15/complete_v14_map.png)。
- **complete_v15**：v15 自动完成/停车保存回归；路线和观测模块与v14相同。 [逐点速度](analysis_final_v15/complete_v15_velocity.png) · [地图与轨迹](analysis_final_v15/complete_v15_map.png)。
- **final_v10_discovery_r1**：v10 开放顺序；有限预算偏向同一区域，且观察目标可见性不够精确。自然部分结束；晚到的 finish 请求未发送，审计保留。 [逐点速度](analysis_final_v15/final_v10_discovery_r1_velocity.png) · [地图与轨迹](analysis_final_v15/final_v10_discovery_r1_map.png)。
- **final_v10_office**：开发队列发生两套自有仿真重叠，原雷达/心跳保护暂停；显式结束保存。环境中断样本，不用于性能对比。 [逐点速度](analysis_final_v15/final_v10_office_velocity.png) · [地图与轨迹](analysis_final_v15/final_v10_office_map.png)。
- **final_v11_discovery_r1**：与上一开发轮重叠的启动失败，FAIL 原样保留；保护未放宽。 [逐点速度](analysis_final_v15/final_v11_discovery_r1_velocity.png) · [地图与轨迹](analysis_final_v15/final_v11_discovery_r1_map.png)。
- **final_v11_discovery_r1b**：v11 串行重试及区域预算；自然部分结束。22 次选中批次中 9 次看不到所分配前沿，后续增加指定目标可见性。 [逐点速度](analysis_final_v15/final_v11_discovery_r1b_velocity.png) · [地图与轨迹](analysis_final_v15/final_v11_discovery_r1b_map.png)。
- **final_v11_discovery_r2**：v11 自然视点耗尽，25 目标、66.291 m，未解决尾段跨区。 [逐点速度](analysis_final_v15/final_v11_discovery_r2_velocity.png) · [地图与轨迹](analysis_final_v15/final_v11_discovery_r2_map.png)。
- **final_v11_office**：v11 初始已知区域仅 292 个安全通行格，统一宽观察余量导致 0 候选；v13 同图只读回放得到 72 个原 0.40 m 合法候选，另做新运动验证。 [逐点速度](analysis_final_v15/final_v11_office_velocity.png) · [地图与轨迹](analysis_final_v15/final_v11_office_map.png)。
- **final_v12_discovery_r1**：v12 已修正指定前沿可见性；旧访问抑制与按边界格数收益仍低效。26 目标、92.650 m，自然视点耗尽，完整保留。 [逐点速度](analysis_final_v15/final_v12_discovery_r1_velocity.png) · [地图与轨迹](analysis_final_v15/final_v12_discovery_r1_map.png)。
- **final_v13_discovery_r1**：v13 开发完整验证，空白图，900 仿真秒预算；未知代理减少，但同阈值尾段成本退化。 [逐点速度](analysis_final_v15/final_v13_discovery_r1_velocity.png) · [地图与轨迹](analysis_final_v15/final_v13_discovery_r1_map.png)。
- **final_v13_discovery_r2**：v13 开发重复轮，空白图、相同预算与速度；起点安全空间异常导致部分结束。 [逐点速度](analysis_final_v15/final_v13_discovery_r2_velocity.png) · [地图与轨迹](analysis_final_v15/final_v13_discovery_r2_map.png)。
- **final_v13_office**：v13 开发办公室，95%→98% 路程退化为38.178m；据此修正v14排序目标。 [逐点速度](analysis_final_v15/final_v13_office_velocity.png) · [地图与轨迹](analysis_final_v15/final_v13_office_map.png)。
- **final_v14_discovery_r1**：v14 保护中断轮：安全状态消息约2.37秒缺口触发输入陈旧锁存停车；核对会话后显式结束保存，不能当自然任务完成时间。95%→98% 阶段在中断前完成，独立列报。 [逐点速度](analysis_final_v15/final_v14_discovery_r1_velocity.png) · [地图与轨迹](analysis_final_v15/final_v14_discovery_r1_map.png)。
- **final_v14_discovery_r2**：v14 同参数实验室重复轮，保留真实退化和中断。 [逐点速度](analysis_final_v15/final_v14_discovery_r2_velocity.png) · [地图与轨迹](analysis_final_v15/final_v14_discovery_r2_map.png)。
- **final_v14_office**：v14 同参数办公室验证，重点比较实际95%→98%尾段成本。 [逐点速度](analysis_final_v15/final_v14_office_velocity.png) · [地图与轨迹](analysis_final_v15/final_v14_office_map.png)。
- **final_v9_discovery_r1**：v9 两段插入前瞻；尾段仍反复跨区，显式结束保存；68.598 m 不作为改善证据。 [逐点速度](analysis_final_v15/final_v9_discovery_r1_velocity.png) · [地图与轨迹](analysis_final_v15/final_v9_discovery_r1_map.png)。
- **pilot_v7_discovery**：保留小前沿后过早插入零散补盲，频繁短目标；显式结束保存，未达完整任务结论。 [逐点速度](analysis_final_v15/pilot_v7_discovery_velocity.png) · [地图与轨迹](analysis_final_v15/pilot_v7_discovery_map.png)。
- **watchdog_v13**：SIGSTOP/SIGCONT 注入：独立验收器继续记录速度，检查失联锁存、不自动续行和显式继续/暂停/结束保存。 [逐点速度](analysis_final_v15/watchdog_v13_velocity.png) · [地图与轨迹](analysis_final_v15/watchdog_v13_map.png)。
- **watchdog_v14**：失败保留：失联停车与心跳恢复不续行通过，显式恢复后40秒未发出新目标，验收超时中断；未保存最终地图，不补造最终覆盖率。 [逐点速度](analysis_final_v15/watchdog_v14_velocity.png) · 无最终地图，不计算最终覆盖率。
- **watchdog_v15_r1**：v15 显式恢复/生命周期新鲜性回归第1轮，实际结果见checks，不与v14性能数据混同。 [逐点速度](analysis_final_v15/watchdog_v15_r1_velocity.png) · [地图与轨迹](analysis_final_v15/watchdog_v15_r1_map.png)。
- **watchdog_v15_r2**：v15 同条件失联回归重复轮，保留实际故障和缺测。 [逐点速度](analysis_final_v15/watchdog_v15_r2_velocity.png) · [地图与轨迹](analysis_final_v15/watchdog_v15_r2_map.png)。

队列重叠发生在开发轮，已在 [queue_overlap_audit.json](source/queue_overlap_audit.json) 记录。最终 v14 使用进程文件锁、源码哈希和前轮完成检查，逐轮隔离 ROS_DOMAIN_ID 与 IGN/GZ_PARTITION，完整命令及退出码见 [serial_v14_results.json](software/serial_v14_results.json)。所有进程只清理自身进程组。

## 实际 Nav2 路径与原因证据

- [静态历史图真实 Nav2 规划](diagnostics/planner_v13/result.json)：只查询路径，不启动运动。6 个可用点、37 次查询，29 条有效候选间有向边；缺失边不当零代价。[独立穷举](diagnostics/planner_v13/order_comparison.json)得到同一顺序：贪心→开放顺序的预计代价 259.874→243.170 秒（低 6.43%），总路径 37.367→32.445 m。这里只比较固定矩阵，**不是实际整场探索节省比例**。
- 各运动会话 `mission/nav2_paths.jsonl` 保存实际返回路径、候选、Nav2 报告的规划耗时和估计行驶代价（原始 planning_cpu_s 字段实际是 planning_time，非进程 CPU 计时；派生 CSV 更正列名）；`mission/planning_maps/*.npz` 保存实际代价图及占据图。`analysis_final_v15/*_path_costs.csv` 可比较 BFS 初筛距离与 Nav2 真实路径；`*_execution_costs.csv` 逐目标对照预计长度/成本与实际里程/用时/观测收益，区分路径估计和执行结果。
- [旧指定前沿可见性审计](diagnostics/target_visibility_audit.json)：v10 的 18 次选择中 6 次、v11 串行首轮的 22 次中 9 次未看到被分配前沿；最终版增加指定目标可见性和发目标前复查。
- [初始余量回放](diagnostics/office_initial_margin_replay.json)：同一失败图从 0 候选到 72 个原合法余量观察点，只读回放不替代最终办公室新仿真。
- [v13办公室排序目标诊断](diagnostics/office_weighted_order_replay.json)：原排序为185.101秒、高收益时延225.137m²·s；同一实际Nav2矩阵的高收益优先顺序185.194秒、时延121.073m²·s，据此实施v14。单纯最短整批路程会在几乎同路程时先处理微小任务。v14采用收益时延最优顺序需满足最短成本+max(2秒,5%)预算；不合预算回退原顺序，不宣称约束内全局最优或整场执行必然省时。
- [独立分析](analysis_final_v15/metrics.json)：包括按次目标实际观测结果、采样缺口、实际矩阵穷举、几何代理、阈值时间及里程。不同候选子集的贪心顺序不计算“节省百分比”。

## 文件、复现及完整性

- [原代码](source/before_ws/)与[性能验证 v14 源码](source/final_v14_ws/)与[当前 v15 源码](source/final_v15_ws/)包含 src/tools/maps，运行时均从工作空间推导路径。`source/final_v13_ws` 保留 v13 退化版本，`source/final_ws` 和 `final_source_manifest.json` 是更早的 v11 开发冻结，名称保留原样，**不是当前 v15**。准确对应以每轮 source、result.json 的 SHA 和 [v14 冻结协议](source/v14_serial_protocol.json)为准。
- [软件日志](software/)保留构建、测试和中间失败；[环境与不变范围](environment.json)明确固件、协议和桌面控制台未改。
- 每个 `runs/<name>` 含 result.json、实际消息 CSV、事件、位姿探针、完整日志、每次地图快照、源文件；正常保存轮次的 mission 内含 session、地图四件套和原始导航事件；watchdog_v14 超时中断缺失最终图及四件套，原始CSV和地图快照保留，最终覆盖率留空。所有失败和主动结束保留。
- [PNG/SVG 和派生表](analysis_final_v15/)；旧 analysis_v1/v2/v3 及 analysis_final（v13）为开发阶段分析，保留原样。当前工具位于 `ros2_ws/tools/report_exploration_tail.py`，只读输入，输出必须使用新的目录。
- [SHA256.json](SHA256.json)覆盖本证据目录除自身以外全部文件；[integrity.json](integrity.json)核对每轮原始哈希、最终源代码、保存四件套与进程清理，逐轮真实 map_server 重载结果在 diagnostics/reload_*。

复现入口：先按仓库 `ros2_ws/tools/build.bash` 构建；`source ros2_ws/tools/env.bash`，使用新的输出目录、非 86 的独立 ROS 域运行 `verify_exploration.py`。完整最终串行命令保存在 `source/serial_v14.py`，包含文件锁及未完成轮次检查。不要把旧运行数据目录作为新输出，不并行启动另一套 Gazebo。
