
## 2026-10-24 探索v17：近静止耗时与Frontier交接

本轮判定：**整体近静止耗时和探索覆盖吞吐目标尚未达成，v17保留为本地待优化实现，未提交或推送。** 办公室交接中位数3.512→2.732仿真秒（降低22.2%），实验室2.652→2.689秒（增加1.4%，基本持平）。不能将办公室的局部交接改善扩大为全任务提速。交接任务构成不同：实验室20→13次、办公室18→29次；目标内容不同，中位数变化也可能受任务构成影响，不能作为独立因果证据。

95%→98%同覆盖区间：实验室用时96.629→128.009秒（+32.5%）、近静止10.275→12.074秒（+17.5%）、路程11.480→16.344米（+42.4%）；办公室用时237.926→248.803秒（+4.6%）、近静止23.129→31.954秒（+38.2%）、路程28.274→27.674米（-2.1%）。固定参考面积吞吐分别下降24.5%和4.4%。

局部等待有可见改善：该区间关联等地图的近静止时间，实验室1.342→0.437秒、办公室3.268→0.420秒；生命周期就绪等待分别3.350→0、8.819→0秒。搜索/查询阶段与行驶/访问顺序的变化抵消了这些收益；这些是事件关联和真实路径证据，不是唯一因果或CPU测量。

覆盖与质量也有取舍：实验室最终几何可观测已知99.821%→99.243%、未知参考面积0.1275→0.5400m²，不能以更早退出/更短总路程称更快完成；办公室98.779%→99.520%、未知1.3225→0.5200m²，但总行程增加。两场景均为partial。导出地图几何P95实验室0.060→0.060m、办公室0.080→0.110m；新地图均通过真实Nav2地图服务器逐格重载，几何代理不代表实车精度。

下一瓶颈的证据已经保留：实验室同覆盖区间的实际行驶增长，办公室搜索/查询阶段的停留增加。后续需要联合评估访问顺序与候选计算负担，不能仅削减交接等待或用更多目标数证明吞吐提高。


本轮以冻结v15为基线，串行运行相同实验室/办公室及恢复、小房间回归。发现最初隔离基线CMake构建等级为空、新版为RelWithDebInfo后，保留旧轮次为开发参考，并补跑两轮RelWithDebInfo匹配基线；下方正式表仅使用匹配基线。原流程在目标结果后才搜索，并让搜索、查询和派发共享1秒节流，造成空等。v16开发轮出现同覆盖区间退化，保留但未采用；v17移除跨目标预备候选，以最新地图重新选择，等价缓存射线模板并限制区域广搜范围，移除共享交接节流，同批复用几何检查；已有完整新鲜观测证据时免除额外等帧。NavFn/RPP、SLAM、速度、到达容差和停机保护未改。固件源码v131、设备最后确认v127，未连接硬件，PI未在Gazebo中执行。

所有下图P/I均为前→后不变：右反向P 0.45→0.45、其余P 0.30→0.30；稳态I 0.010→0.010、启动I 0.030→0.030，每10 ms积分、电机轴rpm、D=0。图中的算法版本是探索版本，不能当作固件或实车安装版本。

软件构建通过；v17的105项colcon检查0错误、0失败、0跳过，数量包含包装，不是105轮Gazebo。v16曾有108项软件检查通过，但运动吞吐仍退化，软件检查不能替代性能验证。v17移除未采用预备逻辑及对应测试，新增限界广搜回归。初次隔离构建漏拷地图、初次测试误扫描到归档工作区的命令错误也保留日志；最终结果以重新执行成功的日志为准。测试未结束时读取到的中间缺结果日志已明确标注，最终报告在所有测试结束后重新读取。

近静止按原始里程计`|v|<0.02 m/s`且`|w|<0.05 rad/s`判定，超过1仿真秒的缺口断线并单列，不补点、不平滑。阶段与速度的时间关联不能当CPU耗时。区间统计只使用完整落入窗口的原始采样间隔；相较历史工具的末端采样规则可有一个样本的边界差异，本轮新旧同规则。

|场景/覆盖区间|用时 v15→v17 s|近静止 v15→v17 s|路程 v15→v17 m|参考面积吞吐变化|
|---|---:|---:|---:|---:|
|discovery / 0.9->0.95|8.842→80.589|0.000→6.429|1.683→10.434|-89.0%|
|discovery / 0.9->0.98|105.471→208.598|10.275→18.503|13.165→26.779|-49.4%|
|discovery / 0.95->0.98|96.629→128.009|10.275→12.074|11.480→16.344|-24.5%|
|discovery / 0.98->0.99|182.372→90.964|18.195→3.958|15.465→12.455|100.5%|
|office / 0.9->0.95|23.659→16.813|4.139→3.080|3.083→1.661|40.7%|
|office / 0.9->0.98|261.585→265.616|27.268→35.034|31.361→29.339|-1.5%|
|office / 0.95->0.98|237.926→248.803|23.129→31.954|28.274→27.674|-4.4%|

|场景|交接次数 v15→v17|交接中位数 s|累计交接间隔 s|交接内近静止 s|
|---|---:|---:|---:|---:|
|discovery|20→13|2.651→2.689|61.043→36.516|60.123→35.629|
|office|18→29|3.512→2.732|66.221→77.551|65.355→76.385|

|轮次|结果/终止原因|几何可观测已知/自由 %|参考未知 m²|总路程 m|导出占据表面 P95 m|
|---|---|---:|---:|---:|---:|
|before_v15_office|PASS / `NAV2_UNREACHABLE_FRONTIERS_PARTIAL`|99.344 / 95.445|0.7100|70.173|0.120|
|before_v15_discovery|PASS / `NAV2_UNREACHABLE_FRONTIERS_PARTIAL`|99.278 / 95.110|0.5150|54.233|0.070|
|before_v15_matched_discovery|PASS / `NAV2_UNREACHABLE_FRONTIERS_PARTIAL`|99.821 / 95.506|0.1275|92.610|0.060|
|before_v15_matched_office|PASS / `NAV2_UNREACHABLE_FRONTIERS_PARTIAL`|98.779 / 94.549|1.3225|57.461|0.080|
|final_v16_discovery|PASS / `NAV2_UNREACHABLE_FRONTIERS_PARTIAL`|99.481 / 95.005|0.3700|76.659|0.060|
|final_v16_office|FAIL / `INTERRUPTED_PARTIAL`|缺测 / 缺测|缺测|缺测|缺测|
|final_v17_discovery|PASS / `NAV2_UNREACHABLE_FRONTIERS_PARTIAL`|99.243 / 94.843|0.5400|54.569|0.060|
|final_v17_office|PASS / `NO_OBSERVABLE_FRONTIERS_PARTIAL`|99.520 / 95.043|0.5200|90.705|0.110|
|watchdog_v17|PASS / `USER_FINISHED_PARTIAL`|31.458 / 30.568|48.8825|0.122|0.050|
|complete_v17|PASS / `NO_FRONTIERS`|100.000 / 96.022|0.0000|1.443|0.045|

目标次数不同，交接累计量需要结合中位数与同覆盖区间判断；吞吐按固定参考面积/实测用时计算，不以更多零碎目标充当有效覆盖。partial仍是部分图，单场景一对样本不构成统计显著性或实车结论。

### discovery：同尺度前后原始波形与覆盖

算法v15→v17；P右反向0.45→0.45、其余0.30→0.30；I稳态0.010→0.010、启动0.030→0.030，每10 ms、电机轴rpm、D=0，PI未在Gazebo执行。目标是减少交接空等，实测变化见上表，覆盖与吞吐分别判断。

![discovery 原始里程计；橙色为目标结果到下一次发目标的交接区间](evidence/exploration-handoff-20261024/analysis/final_v17/final_v17_handoff_wave_discovery.png)

![discovery 逐次交接与完整采样覆盖尾段](evidence/exploration-handoff-20261024/analysis/final_v17/final_v17_handoff_coverage_discovery.png)

### office：同尺度前后原始波形与覆盖

算法v15→v17；P右反向0.45→0.45、其余0.30→0.30；I稳态0.010→0.010、启动0.030→0.030，每10 ms、电机轴rpm、D=0，PI未在Gazebo执行。目标是减少交接空等，实测变化见上表，覆盖与吞吐分别判断。

![office 原始里程计；橙色为目标结果到下一次发目标的交接区间](evidence/exploration-handoff-20261024/analysis/final_v17/final_v17_handoff_wave_office.png)

![office 逐次交接与完整采样覆盖尾段](evidence/exploration-handoff-20261024/analysis/final_v17/final_v17_handoff_coverage_office.png)

### v16开发退化：保留同覆盖阈值的反例

v16实验室95%→98%相对原开发参考从94.039秒增至231.298秒；原参考的编译等级不同，此处只保留问题记录，不作v17正式量化对照，不能因交接更快就称覆盖吞吐改善。办公室在4个目标后提前失败/中断（已记录运动7.336米）、没有最终地图，不作为路线性能样本；原日志错误和缺测均保留，归因不超出证据。P右反向0.45→0.45、其余0.30→0.30；I稳态0.010→0.010、启动0.030→0.030，每10 ms、电机轴rpm、D=0，PI未在Gazebo执行。

![v16真实退化 handoff_wave](evidence/exploration-handoff-20261024/analysis/final_v17/final_v16_handoff_wave_discovery.png)

![v16真实退化 handoff_coverage](evidence/exploration-handoff-20261024/analysis/final_v17/final_v16_handoff_coverage_discovery.png)

### before_v15_office（v15开发参考，编译等级不同）

实测：PASS，`NAV2_UNREACHABLE_FRONTIERS_PARTIAL`，complete=False；算法：v15原交接与v14观测/路线；P右反向0.45→0.45、其余0.30→0.30；I稳态0.010→0.010、启动0.030→0.030，每10 ms、电机轴rpm、D=0，Gazebo未执行PI。保留启动、行驶、停顿、停车及故障缺测。

![before_v15_office 独立验收器原始速度与安全输出](evidence/exploration-handoff-20261024/analysis/final_v17/before_v15_office_velocity.png)

![before_v15_office 最终地图与真实轨迹；红点为几何代理未知](evidence/exploration-handoff-20261024/analysis/final_v17/before_v15_office_map.png)

[原始数据与全部检查](evidence/exploration-handoff-20261024/runs/before_v15_office/result.json)；[SVG速度波形](evidence/exploration-handoff-20261024/analysis/final_v17/before_v15_office_velocity.svg)。

### before_v15_discovery（v15开发参考，编译等级不同）

实测：PASS，`NAV2_UNREACHABLE_FRONTIERS_PARTIAL`，complete=False；算法：v15原交接与v14观测/路线；P右反向0.45→0.45、其余0.30→0.30；I稳态0.010→0.010、启动0.030→0.030，每10 ms、电机轴rpm、D=0，Gazebo未执行PI。保留启动、行驶、停顿、停车及故障缺测。

![before_v15_discovery 独立验收器原始速度与安全输出](evidence/exploration-handoff-20261024/analysis/final_v17/before_v15_discovery_velocity.png)

![before_v15_discovery 最终地图与真实轨迹；红点为几何代理未知](evidence/exploration-handoff-20261024/analysis/final_v17/before_v15_discovery_map.png)

[原始数据与全部检查](evidence/exploration-handoff-20261024/runs/before_v15_discovery/result.json)；[SVG速度波形](evidence/exploration-handoff-20261024/analysis/final_v17/before_v15_discovery_velocity.svg)。

### before_v15_matched_discovery（v15匹配构建基线）

实测：PASS，`NAV2_UNREACHABLE_FRONTIERS_PARTIAL`，complete=False；算法：v15原交接与v14观测/路线；P右反向0.45→0.45、其余0.30→0.30；I稳态0.010→0.010、启动0.030→0.030，每10 ms、电机轴rpm、D=0，Gazebo未执行PI。保留启动、行驶、停顿、停车及故障缺测。

![before_v15_matched_discovery 独立验收器原始速度与安全输出](evidence/exploration-handoff-20261024/analysis/final_v17/before_v15_matched_discovery_velocity.png)

![before_v15_matched_discovery 最终地图与真实轨迹；红点为几何代理未知](evidence/exploration-handoff-20261024/analysis/final_v17/before_v15_matched_discovery_map.png)

[原始数据与全部检查](evidence/exploration-handoff-20261024/runs/before_v15_matched_discovery/result.json)；[SVG速度波形](evidence/exploration-handoff-20261024/analysis/final_v17/before_v15_matched_discovery_velocity.svg)。

### before_v15_matched_office（v15匹配构建基线）

实测：PASS，`NAV2_UNREACHABLE_FRONTIERS_PARTIAL`，complete=False；算法：v15原交接与v14观测/路线；P右反向0.45→0.45、其余0.30→0.30；I稳态0.010→0.010、启动0.030→0.030，每10 ms、电机轴rpm、D=0，Gazebo未执行PI。保留启动、行驶、停顿、停车及故障缺测。

![before_v15_matched_office 独立验收器原始速度与安全输出](evidence/exploration-handoff-20261024/analysis/final_v17/before_v15_matched_office_velocity.png)

![before_v15_matched_office 最终地图与真实轨迹；红点为几何代理未知](evidence/exploration-handoff-20261024/analysis/final_v17/before_v15_matched_office_map.png)

[原始数据与全部检查](evidence/exploration-handoff-20261024/runs/before_v15_matched_office/result.json)；[SVG速度波形](evidence/exploration-handoff-20261024/analysis/final_v17/before_v15_matched_office_velocity.svg)。

### final_v16_discovery（v16退化/提前中断，未采用）

实测：PASS，`NAV2_UNREACHABLE_FRONTIERS_PARTIAL`，complete=False；算法：v16预备搜索，保留失败；P右反向0.45→0.45、其余0.30→0.30；I稳态0.010→0.010、启动0.030→0.030，每10 ms、电机轴rpm、D=0，Gazebo未执行PI。保留启动、行驶、停顿、停车及故障缺测。

![final_v16_discovery 独立验收器原始速度与安全输出](evidence/exploration-handoff-20261024/analysis/final_v17/final_v16_discovery_velocity.png)

![final_v16_discovery 最终地图与真实轨迹；红点为几何代理未知](evidence/exploration-handoff-20261024/analysis/final_v17/final_v16_discovery_map.png)

[原始数据与全部检查](evidence/exploration-handoff-20261024/runs/final_v16_discovery/result.json)；[SVG速度波形](evidence/exploration-handoff-20261024/analysis/final_v17/final_v16_discovery_velocity.svg)。

### final_v16_office（v16退化/提前中断，未采用）

实测：FAIL，`INTERRUPTED_PARTIAL`，complete=False；算法：v16预备搜索，保留失败；P右反向0.45→0.45、其余0.30→0.30；I稳态0.010→0.010、启动0.030→0.030，每10 ms、电机轴rpm、D=0，Gazebo未执行PI。保留启动、行驶、停顿、停车及故障缺测。

![final_v16_office 独立验收器原始速度与安全输出](evidence/exploration-handoff-20261024/analysis/final_v17/final_v16_office_velocity.png)

[原始数据与全部检查](evidence/exploration-handoff-20261024/runs/final_v16_office/result.json)；[SVG速度波形](evidence/exploration-handoff-20261024/analysis/final_v17/final_v16_office_velocity.svg)。

### final_v17_discovery（v17修改后）

实测：PASS，`NAV2_UNREACHABLE_FRONTIERS_PARTIAL`，complete=False；算法：v17最新地图搜索、等价射线模板/限界广搜、事件交接、完整新鲜观测复用；P右反向0.45→0.45、其余0.30→0.30；I稳态0.010→0.010、启动0.030→0.030，每10 ms、电机轴rpm、D=0，Gazebo未执行PI。保留启动、行驶、停顿、停车及故障缺测。

![final_v17_discovery 独立验收器原始速度与安全输出](evidence/exploration-handoff-20261024/analysis/final_v17/final_v17_discovery_velocity.png)

![final_v17_discovery 最终地图与真实轨迹；红点为几何代理未知](evidence/exploration-handoff-20261024/analysis/final_v17/final_v17_discovery_map.png)

[原始数据与全部检查](evidence/exploration-handoff-20261024/runs/final_v17_discovery/result.json)；[SVG速度波形](evidence/exploration-handoff-20261024/analysis/final_v17/final_v17_discovery_velocity.svg)。

### final_v17_office（v17修改后）

实测：PASS，`NO_OBSERVABLE_FRONTIERS_PARTIAL`，complete=False；算法：v17最新地图搜索、等价射线模板/限界广搜、事件交接、完整新鲜观测复用；P右反向0.45→0.45、其余0.30→0.30；I稳态0.010→0.010、启动0.030→0.030，每10 ms、电机轴rpm、D=0，Gazebo未执行PI。保留启动、行驶、停顿、停车及故障缺测。

![final_v17_office 独立验收器原始速度与安全输出](evidence/exploration-handoff-20261024/analysis/final_v17/final_v17_office_velocity.png)

![final_v17_office 最终地图与真实轨迹；红点为几何代理未知](evidence/exploration-handoff-20261024/analysis/final_v17/final_v17_office_map.png)

[原始数据与全部检查](evidence/exploration-handoff-20261024/runs/final_v17_office/result.json)；[SVG速度波形](evidence/exploration-handoff-20261024/analysis/final_v17/final_v17_office_velocity.svg)。

### watchdog_v17（v17修改后）

实测：PASS，`USER_FINISHED_PARTIAL`，complete=False；算法：v17最新地图搜索、等价射线模板/限界广搜、事件交接、完整新鲜观测复用；P右反向0.45→0.45、其余0.30→0.30；I稳态0.010→0.010、启动0.030→0.030，每10 ms、电机轴rpm、D=0，Gazebo未执行PI。保留启动、行驶、停顿、停车及故障缺测。

![watchdog_v17 独立验收器原始速度与安全输出](evidence/exploration-handoff-20261024/analysis/final_v17/watchdog_v17_velocity.png)

![watchdog_v17 最终地图与真实轨迹；红点为几何代理未知](evidence/exploration-handoff-20261024/analysis/final_v17/watchdog_v17_map.png)

[原始数据与全部检查](evidence/exploration-handoff-20261024/runs/watchdog_v17/result.json)；[SVG速度波形](evidence/exploration-handoff-20261024/analysis/final_v17/watchdog_v17_velocity.svg)。

### complete_v17（v17修改后）

实测：PASS，`NO_FRONTIERS`，complete=True；算法：v17最新地图搜索、等价射线模板/限界广搜、事件交接、完整新鲜观测复用；P右反向0.45→0.45、其余0.30→0.30；I稳态0.010→0.010、启动0.030→0.030，每10 ms、电机轴rpm、D=0，Gazebo未执行PI。保留启动、行驶、停顿、停车及故障缺测。

![complete_v17 独立验收器原始速度与安全输出](evidence/exploration-handoff-20261024/analysis/final_v17/complete_v17_velocity.png)

![complete_v17 最终地图与真实轨迹；红点为几何代理未知](evidence/exploration-handoff-20261024/analysis/final_v17/complete_v17_map.png)

[原始数据与全部检查](evidence/exploration-handoff-20261024/runs/complete_v17/result.json)；[SVG速度波形](evidence/exploration-handoff-20261024/analysis/final_v17/complete_v17_velocity.svg)。

详见[算法与测量方法](EXPLORATION_HANDOFF.md)、[本轮完整证据](evidence/exploration-handoff-20261024/README.md)、[派生指标](evidence/exploration-handoff-20261024/analysis/final_v17/metrics.json)。

