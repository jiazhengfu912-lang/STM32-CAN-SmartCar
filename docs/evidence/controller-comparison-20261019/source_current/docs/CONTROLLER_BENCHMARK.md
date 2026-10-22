# DWB、RPP、MPPI 仿真对比

比较对象是 Nav2 局部控制器。全局规划统一使用 NavFn A*，不是比较三种全局寻路算法。当前车模为两轮差速。

从仓库根目录执行，输出目录必须尚不存在。测试主动启动隔离的无界面 Gazebo 并发送目标，不操作已有 RViz/Gazebo 窗口，不连接实车。

```bash
source ros2_ws/tools/env.bash
python3 ros2_ws/tools/benchmark_controllers.py \
  --output "$PWD/ros2_ws/verification_runs/controllers_new" \
  --repeats 3 --goal-timeout 240
python3 ros2_ws/tools/report_controllers.py \
  "$PWD/ros2_ws/verification_runs/controllers_new"
```

首次使用先运行 `tools/build.bash`；依赖由 `tools/install_dependencies.bash` 列出。对比需要已安装 DWB、RPP、MPPI 插件，以及 Python YAML、NumPy、Matplotlib。测试默认顺序使用 ROS 域100—135与每轮独立的随机 Gazebo 分区；开始前避免其他测试占用这些域。测试顺序轮换，所有试验串行执行，避免三种控制器互相争抢仿真资源。

测试输出 `protocol.json`、完整参数副本和源码快照后再启动运动。四个场景分别为直行、转弯过门、行进途中插入箱体绕行，以及两个出口持续封堵。前三种期望到达并持续停车；双出口封堵期望有限恢复后 ABORTED 并持续停车。每种控制器共12轮，正式矩阵36轮。240秒是每次目标的单调墙钟时间预算，不包含启动；仿真时间另外记录。

三种配置共享速度平滑、保护、代价图、定位、到达检查、恢复及全局重规划。为了统一最终角速度，测试副本将共同平滑器角速度上下限从生产默认±0.8改成±0.6rad/s；前进上限0.30m/s。RPP的原地旋转参数本身不能限制全部曲率跟踪指令。生产默认YAML不由测试改写。

选型先要求全部计划任务结果正确、持续停车证据通过，并且二维带padding车体与已知障碍模型没有重叠采样点。只有满足这些条件者才比较效率：按场景内控制器均值做min-max归一化，仿真耗时40%、实际路程25%、角速度总变差/仿真秒20%、控制器进程CPU15%，再将三个可达场景等权平均。分值越低越好；不是成功概率。失败轮次保留在任务正确率分母与原始数据中，不能靠提前失败取得短耗时优势。

`REPORT.html` 内嵌图表，可单独查看；`REPORT.md`、`trials.csv`、`case_summary.csv`、`summary.csv`和`comparison.json`分别提供说明、全轮次、分场景统计、汇总与机器可读结果。每轮目录保留实际CSV/JSON、启动日志、停止及进程清理证据。图表显示原始采样点，不平滑或补造缺测，几何间隙另标为派生数据。

结论只适用于冻结参数、当前地图、模型及计算平台。RPP沿用项目已有配置，DWB/MPPI按同版官方示例适配，没有同等投入的全参数搜索。随机种子未统一，三次重复仅作描述性比较。CPU包含控制进程内代价图和DDS；几何重叠检查不能代替接触传感器或实车碰撞验收。软件测试与仿真均不能证明实车性能。

本轮实测：[2026-10-19—20完整报表与限制](evidence/controller-comparison-20261019/README.md)。严格门槛无全部通过者，工程候选MPPI与正式合格结果分开，默认RPP保持。
