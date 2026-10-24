"""从已归档原始数据生成本轮全部尝试表和波形，不改写运行记录。"""
from pathlib import Path
import csv
import json
import math
import statistics
import sys
import numpy as np
import matplotlib.pyplot as plt

ROOT=Path.cwd();ARCHIVE=ROOT/'docs/evidence/exploration-speed-20261021'
sys.path.insert(0,str(ROOT/'ros2_ws/tools'))
from report_exploration_speed import evaluate,plot_velocity,plot_maps,plot_pose,plot_quality_timeline,save


def number(value,digits=3):
    """缺测保留为缺测，不把空字段画为零或纳入均值。"""
    return '缺测' if value is None else f'{value:.{digits}f}'


def main():
    """所有失败、先期停止和部分地图都列出，并单独解释正式速度指标。"""
    attempts=json.loads((ARCHIVE/'attempts.json').read_text());all_metrics={};timelines={}
    for name in attempts:
        world='office_loop' if 'office' in name else ('complete_room' if name=='complete_room' else 'discovery_lab')
        all_metrics[name],timelines[name]=evaluate(ARCHIVE/'runs'/name,ARCHIVE/'source'/f'world_{world}.sdf')
    (ARCHIVE/'all_attempt_metrics.json').write_text(json.dumps(all_metrics,indent=2))
    final=json.loads((ARCHIVE/'summary.json').read_text())
    before=statistics.mean(final[f'before_discovery_r{i}']['milestones']['0.95']['sim_s'] for i in [1,2])
    after=statistics.mean(final[f'after_discovery_r{i}']['milestones']['0.95']['sim_s'] for i in [1,2])
    comparison={'before_mean_sim_seconds_to_95':before,'after_mean_sim_seconds_to_95':after,
                'reduction_percent':100*(before-after)/before,'pairs':2,
                'scope':'Same fixed reachable-center reference observed free on 3 real map updates; not whole-map completion time',
                'gate_file':'source/final_acceptance_gate.json'}
    for key in ['end_sim_s','duration_wall_s','travel_m','angular_travel_rad','idle_sim_s','turn_only_sim_s','goals']:
        old=statistics.mean(final[f'before_discovery_r{i}'][key] for i in [1,2])
        new=statistics.mean(final[f'after_discovery_r{i}'][key] for i in [1,2])
        comparison[key]={'before_mean':old,'after_mean':new,'reduction_percent':100*(old-new)/old}
    (ARCHIVE/'comparison.json').write_text(json.dumps(comparison,indent=2))
    extras=[name for name in attempts if name.startswith(('pilot','interrupted')) or name in ['watchdog','complete_room']]
    for name in extras:
        plot_velocity(ARCHIVE,[name],'velocity_'+name)
        if (ARCHIVE/'runs'/name/'pose_probe.csv').exists():plot_pose(ARCHIVE,name)
    plot_velocity(ARCHIVE,['pilot_v1','pilot_v2','pilot_v3'],'velocity_development_early')
    plot_velocity(ARCHIVE,['interrupted_v4','pilot_v4_r1','pilot_v4_r2','pilot_v5'],'velocity_development_late')
    plot_maps(ARCHIVE,['pilot_v4_r1','pilot_v4_r2','pilot_v5'],'maps_development')
    plot_quality_timeline(ARCHIVE,['pilot_v4_r1','pilot_v4_r2','pilot_v5'],ARCHIVE/'source/world_discovery_lab.sdf','quality_development')
    fig,ax=plt.subplots(figsize=(12,7))
    for name in extras:
        if name in ['watchdog','complete_room']:continue
        xs=[];ys=[];previous=None
        for row in timelines[name]:
            if previous and (row['sim_s']-previous['sim_s']>6 or row['wall_s']-previous['wall_s']>6):xs.append(np.nan);ys.append(np.nan)
            xs.append(row['sim_s']);ys.append(row['observed_fraction']);previous=row
        ax.plot(xs,ys,'.-',markersize=1,linewidth=.6,label=name)
    ax.set(xlabel='Actual simulation time [s]',ylabel='Observed fixed reference free fraction',ylim=(0,1.02));ax.grid();ax.legend(fontsize=8)
    save(fig,ARCHIVE/'plots','development_coverage')
    lines=['# 自主探索效率优化：2026-10-21—22',
        '',f'正式实验室两对重复：同一95%观测指标平均仿真用时 **{before:.3f} → {after:.3f}秒，减少{comparison["reduction_percent"]:.1f}%**。',
        '这不是全图完成时间或地图准确率；完整结束与部分地图逐轮单列。办公室场景另列，不混入实验室平均值。',
        '', '固件源码v131；设备最后确认v127，本轮未连接/运行。P右反0.45→0.45、其余0.30→0.30；I稳态0.010→0.010、启动0.030→0.030，每10ms按电机轴rpm误差累加；D=0。PI未在Gazebo运行。',
        '', '算法：基线为旧前沿策略，最终v6为信息收益/预计时间、附近补盲、到达切线及新地图驱动的目标交接，另有专用SLAM闭环匹配/Huber配置。v1—v6为探索配置编号，非固件版本。NavFn/RPP、0.20m/s与±0.35rad/s上限和停车保护不变。',
        '', '## 正式对比', '', '|运行|95%指标用时/仿真秒|最终参考观测比例|几何P95/m|0.15m内占据格比例|完整结束|结束原因|',
        '|---|---:|---:|---:|---:|---|---|']
    for name,m in final.items():
        point=m['milestones']['0.95'];q=m['map_geometry_proxy'] or {}
        lines.append(f'|[{name}](runs/{name}/result.json)|{number(point["sim_s"] if point else None)}|{number(m["observed_fraction_final"]*100,2)}%|{number(q.get("surface_distance_p95_m"))}|{number(q.get("occupied_support_fraction_within_015m",0)*100,2)}%|{m["complete"]}|{m["reason"]}|')
    lines+=['', '|最终正式导出图|实际PGM几何P95/m|0.15m内占据格比例|', '|---|---:|---:|']
    for name,m in final.items():
        if not name.startswith('after_'):continue
        q=m.get('exported_map_geometry_proxy') or {}
        lines.append(f'|{name}|{number(q.get("surface_distance_p95_m"))}|{number(q.get("occupied_support_fraction_within_015m",0)*100,2)}%|')
    lines+=['', '实际PGM另外核验，避免用保存之后的在线末帧代替交付地图；地图保存器YAML原点的小数位也会造成少量量化差异。所有正式最终导出图仍须满足同一0.15m/95%几何门槛。',
            '', '|实验室两轮平均|优化前|最终v6|', '|---|---:|---:|']
    for key,label in [('end_sim_s','任务结束/仿真秒（均为部分地图）'),('duration_wall_s','任务墙钟秒（受VM负载影响）'),
                      ('travel_m','里程计路程/m'),('angular_travel_rad','累计转角/rad'),('idle_sim_s','低速静止/仿真秒'),
                      ('turn_only_sim_s','近原地转向/仿真秒'),('goals','目标数')]:
        row=comparison[key];lines.append(f'|{label}|{number(row["before_mean"])}|{number(row["after_mean"])}|')
    lines+=['', '最终v6第二轮路程增加，不能宣称所有路线更短或达到全局最优；两轮平均转向、停等及任务结束用时减少。前后均为部分结束，不能把此时间称为全图完成时间。基线地图质量较差，同一观测指标也不等于同等几何质量。',
        '', '![同场景逐帧观测进度，分场景绘制](plots/coverage.png)',
        '', '![实验室第一对真实末图，同一坐标尺度](plots/maps_discovery_r1.png)',
        '', '![实验室第二对真实末图，同一坐标尺度](plots/maps_discovery_r2.png)',
        '', '![办公室真实末图，同一坐标尺度](plots/maps_office.png)',
        '', '![办公室逐帧质量，保留中途退化与回访修正](plots/quality_world_office_loop.png)',
        '', '固定参考由SDF离线计算：与出生点连通、距障碍至少0.37m的机器人中心位置，以5cm采样。实验室分母38.615m²；不代表全部地面、全部障碍召回率或实车精度。连续3次实际地图更新确认阈值，不插值、不拟合坐标。地图质量为占据格中心到SDF表面距离的仿真代理，不能证明完整几何准确率。',
        '', '原始地图在`runs/*/map_snapshots`逐帧保留；完整起停、目标、状态、故障、缺测在各轮CSV/JSONL及`mission/session.json`。所有地图/速度图保留异常和全部点，长缺口断线。',
        '', '## 全部尝试（功能PASS不等于地图质量达标）', '', '|运行|分类|功能结果|95%用时/秒|几何P95/m|结束原因|','|---|---|---|---:|---:|---|']
    for name,a in attempts.items():
        m=all_metrics[name];point=m['milestones']['0.95'];q=m['map_geometry_proxy'] or {}
        lines.append(f'|[{name}](runs/{name}/result.json)|{a["classification"]}|{a["result"]}|{number(point["sim_s"] if point else None)}|{number(q.get("surface_distance_p95_m"))}|{a["reason"]}|')
    lines+=['', 'v1由测试者提前结束，v2/v3暴露到达/定位问题；v4含一次宿主中断、一次完整完成及一次地图质量失败；v5功能通过但地图质量失败。这些轮次都未计为最终v6重复，也未删除原始样本。宿主中断时任务暂停并锁存，恢复后未自动续行；随后显式结束保存。',
        '', '![开发轮次完整观测进度，含中断](plots/development_coverage.png)',
        '', '![开发轮次地图，含质量失败](plots/maps_development.png)',
        '', '## 闭环诊断、导出与保护',
        '', '离线读取v5保存的处理后扫描及原始轮式位姿，未用保存的修正位姿或场景真值作输入。原配置重放的最终栅格逐格等于实测失败栅格；271.5/327.5秒错误闭环细响应0.734/0.724。禁闭环消融与闭环仍启用、细响应门槛0.80重放均使P95从0.426m降至0.061m。离线诊断不代替新仿真。[输入来源](source/offline_replay_provenance.json)、[逐格一致性](source/offline_replay_fidelity.json)、[重放命令](source/offline_replay_commands.json)、[消融结果](source/offline_loop_ablation.json)、[重放源码](diagnostics/replay_probe/replay.cpp)。',
        '', '自动导出free_thresh从0.25改为0.196，避免PGM未知灰度205被Nav2重载为自由。`diagnostics/map_reload_*`保留旧失败与新通过的真实地图服务器逐格结果；`diagnostics/new-map-navigation`为最终新地图重载导航。历史地图未改，手动保存工具未改。',
        '', '任务进程SIGSTOP/SIGCONT、失联锁存、显式继续/暂停/结束及可观测房间自动完成见`runs/watchdog`、`runs/complete_room`；自建进程退出记录在各轮result.json。',
        '', '## 可复核文件', '', '- [逐轮汇总](summary.json)、[所有尝试指标](all_attempt_metrics.json)、[均值计算](comparison.json)',
        '- [预定比较协议](source/protocol.json)、[发布质量门槛](source/final_acceptance_gate.json)、[v6方案记录](source/v6_protocol_update.json)',
        '- [原始与源文件完整性](integrity.json)、[本轮所有文件SHA256](SHA256.json)',
        '- [启动与范围说明](../../EXPLORATION_SPEED.md)',
        '', '基线生产源码、工具和构建所需示例地图位于`source/before_ws/{src,tools,maps}`；复制到全新目录后运行其中`tools/build.bash`可重新构建。生产探索代码冻结，观察器曾同步增加只读位姿记录；各轮实际观察器源码和哈希另在`runs/*/source`，不能把观察器变化当探索算法变化。',
        '', '所有原始记录保持本机实际路径、时间戳和退出值，路径是证据来源，不是新机器运行依赖。结论限于所列仿真场景；实车桥、实车速度/停距/地图精度均待验证。']
    (ARCHIVE/'README.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(comparison,indent=2))


if __name__=='__main__':main()
