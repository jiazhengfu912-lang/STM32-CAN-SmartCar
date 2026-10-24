# 导航途中遇障后的重规划与执行空间修复

2026-10-19，基于 `62d7e4779532c4eec549dbd22700a02efaa81dd5`，仅 Gazebo Fortress / office_loop / office_01 / AMCL / Nav2，未连接硬件。用户报告“遇到障碍停住，随后 Goal failed”。本轮修改全局规划轮廓，并保存失败与复测原始记录；未提交、推送或合并。

## 原因与修改

原行为树已经每秒重新计算一次路径，局部和全局代价地图也都接收 `/sim/scan`。基线实测在插入障碍后约 1.21 秒产生绕行路径，说明问题不是缺少重规划开关。车辆沿新路径靠近箱体后，门控反复报告 `OBSTACLE_NEAR_TURN`，控制器多次报 `Failed to make progress`。原局部/全局都用同一矩形；NavFn 在二维栅格上使用膨胀后的代价值，不能验证矩形在每个路径姿态下转弯是否可行。

原矩形为 x=[-0.14,0.25]、y=[-0.16,0.16] m，padding=0.02 m，其内切半径约 0.18 m；带 padding 的最远角点半径约 0.3245 m，近障转向保护半径为 0.33 m。规划认为可走的狭窄侧缝因此可能被执行层拒绝。原场景箱体两边净宽各约 0.675 m，几乎没有完整转弯及栅格误差余量。

`global_costmap` 改用 `robot_radius=0.35 m` 的保守圆形轮廓，保留原 0.02 m padding。0.35 m 覆盖局部矩形任意转角的 0.3245 m 半径，并额外覆盖半个 0.05 m 栅格。局部矩形、0.33 m 转向保护、速度、超时、扫描源、碰撞预测、重规划频率和恢复次数均未改。代价是某些只能保持固定姿态通过的窄缝会被规划器排除，可能选择更长路线；这是此次有意采用的保守策略。没有宣称提高了 NavFn 的运动学建模能力。

新增两项配置回归，验证全局规划预留转弯空间、两个代价地图持续接收动态扫描障碍。`geometry.json` 保存几何推导；这不是实车尺寸标定或实测误差。

官方实现依据：[Humble NavFn costmap 转换](https://github.com/ros-navigation/navigation2/blob/humble/nav2_navfn_planner/src/navfn.cpp)、[Humble 膨胀层](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/plugins/inflation_layer.cpp)、[Humble 周期重规划行为树](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml)。

## 工况和全部尝试

两次绕行测试均从出生点向 map(4.5,0,0) 只发一次目标，车辆世界 x 达约 0.45 m 后向本次随机 Gazebo 分区插入静态箱体，箱体中心(2,0)、尺寸0.4×0.5×0.6 m。插入请求期间继续记录 ROS 消息；没有重新发送目标或用世界真值控制车辆。Gazebo world pose 只用于验收。

| 运行 | 实际结果 |
|---|---|
| `baseline_detour_01` | 验收器初版漏开世界位姿记录桥，九个生命周期 active，但就绪检查等不到 world pose；60秒超时，无目标、无运动。保留全部日志和空轨迹图，不能把它当导航失败。 |
| `baseline_detour_02` | 原配置产生绕行路径，但车辆在箱体附近长期被转向门控拦住，记录569条 `OBSTACLE_NEAR_TURN` 状态；多次进展失败。目标执行180秒仍未结束，验收器取消后关闭。**本次没有等到顶层 Goal failed，不能把主动超时取消写成 ABORTED。** |
| `after_detour_01` | 修改后 PASS：原直行路径被替换，改走左侧门口→上方两个房间→右侧门口→目标；原目标返回 SUCCEEDED(4)，停止检查通过，86条低速停车样本。 |
| `after_blocked_01` | PASS：目标 map(4.5,0) 被中心(4.5,0)、1.2×1.85×0.6 m箱体完全占住；原目标经恢复返回 ABORTED(6)，停止检查通过，70条低速停车样本。此工况验证目标被占据，不代表任意完全封路/移动障碍。 |

绕行复测总记录116.28秒，障碍插入后约2.67墙钟秒收到第一条绕行路径（并非性能上限）。最终世界位姿约(4.4532,0.0693,0.0981rad)。原路径与新路径来自 `map`，实际轨迹来自 Gazebo `world`；图中按AMCL近似对齐，不能拿它们的差直接标定定位误差。对配置带 padding 矩形与箱体进行离线几何核对，最小模型间隙约0.625m；这不是接触传感器测量。

局部代价地图宽4m；目标被封堵工况中箱体一直在局部窗口之外，所以该轮局部代价图内没有箱体占据格，**不是局部扫描漏检结论**。全局图实际标记了目标区域，规划器返回无法生成路径。`analysis.json` 的地图观测延迟按已发布代价图计算，还包含发布周期及采样排队，不是内部感知耗时。

基线末段和绕行复测初段曾在不同 ROS 域89/90及不同随机 `IGN_PARTITION`/`GZ_PARTITION` 并行，保留CPU竞争期间的扫描过期/调度告警；没有调整门控新鲜度去消除告警。不能把两轮墙钟延迟当相同计算负载的性能比较。

## 波形与参数前后

**STM32源码v131，P右反0.45→0.45，其余0.30→0.30；I稳态0.010→0.010、低速启动0.030→0.030，10ms按电机轴rpm误差累计，D=0。**固件轮速PI、起步、IMU、相对导航算法未改变、未执行。本轮上层仍为NavFn A*+RPP，改变的是全局规划碰撞轮廓。图为Gazebo/ROS实际消息，未平滑、插值、删异常点，超过1秒缺测断线。

![原配置：新路径贴近障碍，近障转向保护长期拦截](baseline_detour_02/velocity.png)

此图证明原轮廓下控制器仍在请求转向和前进，但保护输出基本为零；不能归因为“完全没有生成路线”。保留超时取消和关闭过程。P/I未改，矩形全局规划轮廓为修改前版本。

![修改后：自动绕行、抵达和停车，全量实收速度与门控状态](after_detour_01/velocity.png)

此图P/I同上未改，改用保守圆形全局轮廓，避免再选择转弯空间不足的侧缝。该轮无 `OBSTACLE_NEAR_TURN`；启动与并行负载中的 `SCAN_STALE` 均保留。到达后要求连续至少1秒、至少10个样本满足|v|<0.02m/s、|ω|<0.03rad/s且新鲜安全命令为零，实际86条样本通过；不表示机械抱闸或实车停距。

![相同坐标尺度下的新路径与实际绕行轨迹](after_detour_01/trajectory.png)

图中P/I未改、规划轮廓改变；原路线橙色，新路线绿色，实收世界轨迹蓝色。只在图上选择原路线和首条绕行路线用于比较，所有规划仍逐条保存在 `plans.jsonl`，实际轨迹全部点均保留。不能从一次通过推断任意障碍可绕行。

![目标被障碍占据：恢复后失败并持续停车](after_blocked_01/velocity.png)

此图P/I、固件和RPP未改，使用修改后的全局轮廓；规划失败后进行原有转向/等待/后退恢复，最终ABORTED并停车。没有自动重发旧目标、解除故障或接入实车。

每轮另有 `velocity.svg`、`trajectory.png/.svg`、`analysis.json`。首次绘图脚本函数边界错误保留于 `plot_baseline.log`，修正后 `plot_final.log`、`plot_blocked.log` 成功；原始采样未改写。

## 构建、软件回归与可复现入口

`build.log`：三包构建通过。`ros_tests.log`：34个pytest用例通过，colcon统计39项（含5个CTest包装），0错误/失败/跳过。新增检查防止全局内切半径再次小于局部转弯包络。不是固件全套回归或远端CI结果；本轮未操作GitHub PR。

```bash
# 从仓库根目录；输出必须是全新目录，会启动隔离仿真并主动导航。
python3 ros2_ws/tools/verify_replanning.py \
  --goal-timeout 300 --output "$PWD/ros2_ws/verification_runs/replan_next_01"
python3 ros2_ws/tools/verify_replanning.py --case blocked \
  --output "$PWD/ros2_ws/verification_runs/blocked_next_01"
# 只读原始采样并生成图和派生分析。
python3 ros2_ws/tools/plot_replanning.py "$PWD/ros2_ws/verification_runs/replan_next_01"
```

默认使用域89，拒绝默认用户域86；分区随机；启动前检查已有发布者，只关闭自己的进程组。每轮 `config/` 保存当轮配置；`result.json` 保存软件来源哈希及原始文件哈希，`SHA256.json` 校验本轮归档文件。扫描中的无穷远值在原始JSONL保留为Python JSON的`Infinity`标记；不得视为有限距离。

归档的JSONL以`.jsonl.gz`无损压缩，`result.json`中的原名/哈希对应**解压后的原字节**，不修改原报告。`compression.json`同时保存压缩前后哈希；全部归档解压核对通过。未压缩原件保留在忽略的`ros2_ws/verification_runs/nav2-replanning-20261019-raw/<运行>/`，没有丢弃消息；CSV与图片直接存放于各运行目录。绘图工具同时支持原JSONL和gzip归档。`source_snapshot/`保存实际运行过的验收器、最终绘图器与新增测试。

全部测试进程已退出；未打开/操作用户RViz窗口，未连接硬件。真实雷达、实车桥、多目标GUI、移动障碍、其它窄门与长期运行仍待各自验收。重新启动导航后新配置才生效。
