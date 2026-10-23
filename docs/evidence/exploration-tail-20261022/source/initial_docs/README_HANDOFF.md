# SmartCar 当前工程交接 README

> 更新日期：2026-10-22，Asia/Shanghai；GitHub 状态核对至 16:06。Git、PR、CI 和运行进程为本次交接时的快照，新对话接手后须重新核对。
> 本文面向下一轮对话，汇总已经实现、实际验证、尚未完成及如何继续。原始数据和历史文档继续保留，本文不替代验收证据。

## 1. 先确认接手的是哪个工程

**实际开发仓库：`/home/jiazhengfu/Projects/STM32-CAN-SmartCar`。ROS 工作空间位于该仓库的 `ros2_ws/`。**

| 路径 | 当前用途 |
|---|---|
| `/home/jiazhengfu/Projects/STM32-CAN-SmartCar` | 本项目 Git 仓库；继续修改代码和文档应在这里进行 |
| 仓库内 `ros2_ws/` | 当前 ROS 2 三包源码、配置、构建和验证工具 |
| `/home/jiazhengfu/Projects/ros2-learning` | 聊天可能默认进入的目录；本次核对不是当前 SmartCar 源码仓库 |
| `/home/jiazhengfu/Projects/smartcar_sim_ws` | 迁入前的独立仿真工作空间；保留历史资料，不作为当前开发入口 |

先读 [AGENTS.md](AGENTS.md) 和本文，再按需要阅读 [历史交接](HANDOFF.md)、[ROS 工作空间 README](ros2_ws/README.md)、[实车集成边界](docs/ROS2_HARDWARE_INTEGRATION.md) 和最新验收。不要因为聊天默认目录不同，就在另一个工作空间重复实现已有功能。

## 2. 当前 Git、PR、作者与工作区

| 项目 | 本次核对结果 |
|---|---|
| 远端 | `https://github.com/jiazhengfu912-lang/STM32-CAN-SmartCar.git` |
| 分支 | `feat/ros2-simulation-navigation`，跟踪同名 origin 分支 |
| 本地与远端 HEAD | `b77c32cbdaf06085ed7d2c2ff418c7d183bddad0` |
| 远端 main | `19d6c23d5f27f123485fc55d72b5523fad34d182`；尚未包含本分支的 ROS 集成成果 |
| PR | [#1：集成ROS2仿真导航、自主探索建图与效率优化](https://github.com/jiazhengfu912-lang/STM32-CAN-SmartCar/pull/1)，open、未合并 |
| 写本文之前 | 工作区干净，本地与远端同步 |
| 本次文档整理后 | 新增 `README_HANDOFF.md`，修改 `README.md`、`HANDOFF.md` 的交接入口；未提交、未推送 |
| 本机构建 | 三个包的 `ros2_ws/install/` 目录存在；不能据此替代下一次代码变更后的构建检查 |
| 运行进程 | 本次按 ROS/Gazebo/RViz 关键进程核对未发现正在运行的本项目仿真；接手时重新检查 |

用户最近要求将作者从 Codex 改成自己。已将本分支全部 4 条提交的作者和提交者改为 `jiazhengfu912-lang`，邮箱使用该账号既有的 GitHub 隐私邮箱：`264059566+jiazhengfu912-lang@users.noreply.github.com`。已设置本仓库本地 Git 身份，未修改全局 Git 身份。

| 当前提交 | 历史旧提交 | 内容 |
|---|---|---|
| `aa2f4cc1f339d61ead9a8253ec2f1a7542f149fb` | `62d7e4779532c4eec549dbd22700a02efaa81dd5` | ROS 2 仿真、SLAM、Nav2 整合及初轮证据 |
| `6ae9583be2bde739a73bf84146d0f0fddae42c20` | `bef40ae9a8cca4eee6004a7495876a11e89495a5` | 障碍记忆、重规划转向空间、TF 失败与 Reset 修复 |
| `89006fe33dc2519b2f596c3dab5967243404a9f8` | `a45dd11196779f9845fe29f07dfa5c3a98eb2ef6` | 自主探索、新场景、控制器比较工具及探索效率优化源码 |
| `b77c32cbdaf06085ed7d2c2ff418c7d183bddad0` | `02c3a2047f7e29ee14d2ea7f48ec28270015dd6a` | 完整证据、报表、进度与验收文档归档 |

本次作者更正已核对每条提交：文件树、提交说明、原始日期全部一致，只有身份及父提交对应变化。原始证据中的旧 SHA、原机器路径、时间戳保留原样，不能为“对齐新 SHA”改写历史数据或重算来掩盖原始记录。旧 SHA 与当前源码的对应关系见上表，不应直接切回旧提交继续开发。

本次交接的 CI 状态见第 6 节。`HANDOFF.md`、`docs/PROGRESS.md` 和旧证据 README 中的“未提交”“作者 Codex”“HEAD bef40ae”等表述是历史阶段快照；当前分支/作者/提交状态以本节及实时 Git/GitHub 为准。

**当前仅授权整理交接文档。** 前一轮功能开发、提交推送及作者更正已完成；不要把旧任务的推送授权扩展成下一轮自动提交、推送、合并、烧录或实车运动。

## 3. 项目目标与整体交付边界

项目将 STM32/ESP32 底盘控制与 Ubuntu 上的 ROS 2 二维感知、建图、定位和导航放在同一仓库管理。近期重点是：陌生环境无需人工遥控或逐个点目标，自行探索、构建可用地图，遇障合理重规划，并在结束或故障时停车。

**当前可运行的是 Gazebo 仿真链路。ROS 实车桥、实车启动入口及实车导航参数仍未实现，不能把仿真成功当成实车通过。**

| 模块 | 已实现/已有记录 | 仍需注意 |
|---|---|---|
| STM32 HAL App、Bootloader | CAN 电机反馈、速度 PI、故障锁存、无线 OTA、相对坐标导航；CubeMX/Keil/EIDE 工程保留 | 源码 v131，设备最后确认 v127；本轮没有连接或安装 |
| ESP32 与网页控制台 | HTTP 管理、UDP 控制、遥测、OTA；本地控制台默认 `127.0.0.1:18765` | 现有控制权、序号与心跳不能被未来 ROS 桥绕过 |
| Gazebo 模型与感知 | 车体、C1 雷达/相机模拟、里程计、TF、场景、键盘控制和速度超时保护 | 摩擦、惯量及部分外参估算；不是实物辨识模型 |
| 已知地图导航 | AMCL + NavFn A* + RPP、RViz/CLI 目标、连续多目标行为树、代价地图与恢复 | 选定场景通过；任意多目标 GUI、移动障碍及长期稳定性未完整验收 |
| 遇障处理 | 周期重规划、保留已观测堵路障碍、有限恢复、不可达时失败停车 | 删除障碍或 Reset 不会自动重发已经失败的目标 |
| 陌生环境自主建图 | 从空白 SLAM 图选择前沿，Nav2 移动，持续观测，暂停/继续/结束及停车保存 | 大场景正式最终轮次仍为部分图，跨启动自主续接尚未实现 |
| 探索效率与地图质量 | 信息收益/耗时选点、附近补盲、减少转向等待、专用 SLAM 配置、未知区导出修复 | 改善限于所测场景；不保证最短路、任意环境全覆盖或实车性能 |
| DWB/RPP/MPPI 比较 | 36 轮正式对比、HTML/CSV 报表及原始数据已归档 | 没有严格全部通过的赢家；默认仍为 RPP |
| ROS 实车接口 | 已有协议核对、单位换算及分阶段实施文档 | 驱动桥、真实 C1 接入、实车 odom/TF/时间同步与控制权整合待做 |

底盘是**两轮差速：左右各一个主动轮，另有两个被动万向轮**。外观共有四个轮子，不是四轮独立驱动/四轮滑移差速。模型主动轮直径 65 mm、宽 27 mm；轮距 252 mm 为模型值，尚未实车标定。

## 4. 当前软件架构与默认算法

目标环境为 Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress；历史实录为 Ubuntu 22.04.5、Fortress 6.18.0、Python 3.10。版本以实际机器为准，接手不应擅自迁移 ROS/Gazebo 发行版。

```text
自主建图：模拟 C1 扫描 + 轮式 odom/TF → SLAM Toolbox → 在线 /map、map→odom
          在线地图 → 前沿探索器 → Nav2 目标 → NavFn A* → RPP
已知地图：保存的 map.yaml/PGM + 扫描 + odom → AMCL → map→odom → Nav2

Nav2 → /nav/cmd_vel_raw → Velocity Smoother → /nav/cmd_vel
     → navigation_gate → /nav/safe_cmd_vel
     → sim_cmd_guard → /sim/drive_cmd_vel → Gazebo

TF：map → odom → base_footprint → base_link → laser_frame 等
```

建图只由 SLAM 发布 `map→odom`；已知地图导航只由 AMCL 发布，不能同时启动两套定位。自主入口不加载旧地图、不启动 AMCL/map_server；自动导出时使用独立保存服务。

- 默认全局规划：NavFn A*；默认局部控制：**RPP**，不是 DWB，也没有因比较结果自动切换为 MPPI。
- 普通导航：RPP 巡航 0.30 m/s，转向目标 0.60 rad/s；门控和平滑/底层仍有各自边界。
- 自主建图覆盖：巡航 0.20 m/s、最终角速度 ±0.35 rad/s、最大前进加速度 0.25 m/s²、角加速度 0.50 rad/s²。
- 全局圆形机器人轮廓半径 0.35 m，另有 0.02 m padding；局部保持矩形。探索连接路径采用 0.37 m 通行圆，观察点至少 0.40 m 已知自由余量。
- 当前导航恢复不自动清空全局/局部代价地图，最多 4 轮；雷达正常射线清障仍保留，避免长期保留已经移走的障碍。
- 导航门控检查时钟、雷达、里程计、TF、定位/地图来源及发布者；自主任务另有单调墙钟心跳和失联锁存。心跳恢复不自动续行，必须显式继续。

## 5. 本阶段已解决的问题及实现位置

### 5.1 遇障停住、Goal failed、两条堵路之间反复切换

问题包含全局规划留给车体转向的空间不足，以及自动清图让规划器遗忘视野外仍存在的障碍。已修改全局保守碰撞轮廓，单目标与连续多目标均保持 1 Hz 周期重规划；规划/跟踪失败进入不清图的有限恢复。实际测试覆盖途中插入箱体后绕行到达、双出口堵死后有限失败停车，以及移走障碍、重新观察并发新目标后恢复通行。

涉及 [nav2.yaml](ros2_ws/src/smartcar_nav2/config/nav2.yaml)、[单目标树](ros2_ws/src/smartcar_nav2/config/navigate_recovery.xml)、[连续目标树](ros2_ws/src/smartcar_nav2/config/navigate_through_recovery.xml)。有限失败停车是不可达场景的预期行为，不能为了“总能走”而无限重试或放宽碰撞保护。

### 5.2 TF 失败误报到达、Reset 卡住

现场曾出现较远目标数秒内误报成功，以及 Reset 卡在清理。新增仓库本地 [safe_controller_server.cpp](ros2_ws/src/smartcar_nav2/src/safe_controller_server.cpp) 和 [safe_planner_server.cpp](ros2_ws/src/smartcar_nav2/src/safe_planner_server.cpp)，检查目标 TF 失败并停车上报，串行处理代价图 TF/扫描过滤相关回调，避免所测版本的锁序反转。不覆盖系统 ROS 库，不扩大到达或 TF 容差。

### 5.3 陌生环境自动建图与新场景

新增 [exploration.launch.py](ros2_ws/src/smartcar_nav2/launch/exploration.launch.py)、[explore.py](ros2_ws/src/smartcar_slam/scripts/explore.py) 和 [frontier_core.py](ros2_ws/src/smartcar_slam/scripts/frontier_core.py)。探索器只发送导航目标，不直接发驾驶速度。使用在线地图中的未知/自由边界及已知自由路径，不能穿越未知或墙体。

`discovery_lab` 是本项目 2026-10-21 新建的 10×8 m 非对称实验室物理场景，供用户亲自验收；不加载该场景的预存导航地图。SDF 几何只作为仿真物理环境和离线对照，不提供给在线探索器作为已知地图。

建图门控还修复了迟加入时漏收静止 SLAM 首帧 pose 而一直 WAITING 的问题；现在联合检查唯一 SLAM 地图源、新鲜地图与有效 TF，原时效保护保持。

### 5.4 探索慢、SLAM 错误闭环、导出未知区丢失

探索选点改为估计观测收益/预计耗时，兼顾行进距离、起步转向、目标交接、附近独立前沿；目标朝向采用到达切线，目标间由新地图驱动，最少等待 0.2 仿真秒，减少原固定 3 秒等待。安全余量、失败记忆和 Nav2 到达容差保持。

自主模式专用 [exploration_slam.yaml](ros2_ws/src/smartcar_slam/config/exploration_slam.yaml) 保留闭环，采用 HuberLoss、候选链 15、粗方差上限 0.5、粗响应 0.5、细响应 0.80；手动建图与常规 AMCL 不加载该覆盖。v5 的错误闭环先用保存扫描/原始 odom 重放定位，再做修改后的独立新仿真；不能以重放替代正式新实验。

自动导出 `free_thresh` 改为 0.196，使 PGM 未知灰度 205 被真实 Nav2 map_server 重载后仍为未知；原 0.25 会误判为自由。历史地图和手动保存工具未自动修改，需要独立验证。详情见 [探索效率说明](docs/EXPLORATION_SPEED.md)。

## 6. 实际验证结果与不得扩大解释的边界

### 6.1 构建、软件测试和 CI

- 最终探索优化代码已完成三个 ROS 包构建；已有记录为 **77 项 colcon 检查，零失败/错误/跳过**。该数包含测试包装，不能写成 77 轮 Gazebo 实验。
- 作者更正前、相同文件内容的 [CI 39723803856](https://github.com/jiazhengfu912-lang/STM32-CAN-SmartCar/actions/runs/39723803856) 为 success。
- 当前 HEAD 的 [push CI 39744068325](https://github.com/jiazhengfu912-lang/STM32-CAN-SmartCar/actions/runs/39744068325) 和 [PR CI 39744086327](https://github.com/jiazhengfu912-lang/STM32-CAN-SmartCar/actions/runs/39744086327) 在本次 16:06 核对时均已 completed / success，二者记录的 head_sha 均为 `b77c32cbdaf06085ed7d2c2ff418c7d183bddad0`。
- CI 只做软件构建/检查，不运行 Gazebo 或实车。本轮交接仅编辑 Markdown，没有重跑运动实验，也不新增硬件结论。

### 6.2 DWB、RPP、MPPI 正式对比

正式矩阵：3 种控制器 × 直线/过门/途中加障碍/双出口封堵 × 3 次重复，共 36 轮。

| 控制器 | 结果符合预期的总轮次 | 封堵后正确退出 | 未发目标的启动失败 |
|---|---:|---:|---:|
| DWB | 8/12 | 3/3 | 1 |
| RPP | 10/12 | 3/3 | 1 |
| MPPI | 10/12 | 3/3 | 1 |

33 次已发目标的任务均验证了持续停车。三种控制器各有一轮绕障墙钟预算超时，启动未就绪不归因为局部算法本身。严格预设协议没有全部通过者；事后工程候选优先继续验证 MPPI，默认仍保留 RPP。MPPI 所测成功绕障较快但 CPU 更高，不是普遍最优或处处更平滑。

报表：[HTML](docs/evidence/controller-comparison-20261019/formal/REPORT.html)、[原始轮次 CSV](docs/evidence/controller-comparison-20261019/formal/trials.csv)、[范围与全部失败](docs/evidence/controller-comparison-20261019/README.md)、[复现说明](docs/CONTROLLER_BENCHMARK.md)。

### 6.3 自主探索效率与地图质量

v1—v6 是探索策略/SLAM 配置编号，不是 STM32 固件版本。正式前后比较使用相同场景、空白地图、900 仿真秒预算和相同运动上限；所有早期失败、受阻和环境中断均保留。

| 指标 | 优化前 | 最终 v6 | 解释 |
|---|---:|---:|---|
| 实验室第 1 轮达到 95% 参考观测指标 | 262.950 s | 123.760 s | 仿真秒 |
| 实验室第 2 轮达到同一指标 | 480.267 s | 105.090 s | 仿真秒 |
| 实验室上述两轮均值 | 371.608 s | 114.425 s | 减少 69.2% |
| 实验室部分任务结束时间均值 | 558.464 s | 346.659 s | 不是全图完成时间 |
| 实验室最终在线地图几何 P95 | — | 0.065 / 0.064 m | 门槛 0.15 m；仿真几何代理 |
| 办公室达到同一 95% 指标 | 442.327 s | 114.972 s | 单独一对，减少约 74.0% |
| 办公室部分任务结束 | 752.538 s | 712.894 s | 仅减少约 5.3%，尾段补盲仍慢 |
| 办公室最终在线地图几何 P95 | 1.071 m | 0.076 m | 不是实车测绘精度 |

参考观测分母是离线 SDF 中与出生点连通、具有 0.37 m 安全余量的机器人中心区域；连续 3 次实际地图更新确认指标。它不是全部地面覆盖率、地图准确率或完整自动建图证明。优化后实验室平均行程反而 32.287→42.462 m，不能声称路线总是更短。

**最终 v6 的实验室两轮和办公室一轮都为 `NO_REACHABLE_FRONTIERS_PARTIAL`、`complete=false`。** 选定质量门槛及停车/保存通过，仍不等于“大场景全图自动构建完成”。已观测参考比例达到 100% 也不能覆盖这一任务状态。

最终交付 PGM 的几何 P95 另验为实验室约 0.070/0.060 m、办公室约 0.080 m。真实 Nav2 地图服务器逐格重载三张图，未知格 2624/2349/2596 完整保留。最终新图还通过一次 AMCL 短目标导航与持续停车；不代表任意目标已通过。

任务进程 SIGSTOP/SIGCONT 后停车锁存、恢复不续行、显式继续/暂停/结束保存，以及可观测小房间自动 `NO_FRONTIERS / complete=true` 的回归已有通过记录。小房间结论不能外推成实验室/办公室全覆盖。

完整证据：[探索效率归档](docs/evidence/exploration-speed-20261021/README.md)、[比较计算](docs/evidence/exploration-speed-20261021/comparison.json)、[原始完整性](docs/evidence/exploration-speed-20261021/SHA256.json)、[验收正文与直接嵌入波形](docs/ACCEPTANCE.md)。正式结果来自新仿真，开发 v1—v5 和错误闭环诊断同样保留。

## 7. 如何启动，供用户自己验收

以下为操作说明，**本次写交接文档没有执行这些启动/运动命令**。切换前先在原启动终端停车并退出旧导航/建图，避免重复时钟、扫描、速度发布者和 `map→odom`；不要批量杀掉用户进程。

### 7.1 环境与构建

本机已有构建。源码改变或换机器后，从正确仓库执行：

```bash
cd /home/jiazhengfu/Projects/STM32-CAN-SmartCar
# 新机器缺依赖时按 ros2_ws/README.md 先安装，密码由用户在本机输入
bash ros2_ws/tools/build.bash
source ros2_ws/tools/env.bash
ros2 pkg prefix smartcar_sim
ros2 pkg prefix smartcar_slam
ros2 pkg prefix smartcar_nav2
# 需要软件回归时运行；不启动运动实验
bash ros2_ws/tools/test.bash
```

包路径须属于本仓库 install。每个新终端均 source 本仓库 `env.bash`；默认 `ROS_DOMAIN_ID=86`、`ROS_LOCALHOST_ONLY=1`。不要叠加旧独立工作空间的环境。若确需并行实验，必须同时隔离 `ROS_DOMAIN_ID` 与 `IGN_PARTITION`，同组终端使用相同值。

### 7.2 全新 discovery_lab 自主探索

在同一个已加载环境的终端：

```bash
export SMARTCAR_ACCEPT_MAP="$HOME/smartcar_maps/discovery_lab_$(date +%Y%m%d_%H%M%S)"
echo "$SMARTCAR_ACCEPT_MAP"
ros2 launch smartcar_nav2 exploration.launch.py \
  world:=discovery_lab rviz:=true headless:=false \
  max_sim_seconds:=1800 output_dir:="$SMARTCAR_ACCEPT_MAP"
```

输出目录必须全新，不能预先创建同名会话目录或覆盖旧图。`headless:=false` 打开 Gazebo 和 RViz；虚拟机负载高时可退出后改为 `true`，保留 RViz。无需键盘、2D Pose Estimate 或 Nav2 Goal，小车会等待就绪后自动选点。

`world` 默认为 `office_loop`，`headless` 默认 true，`rviz` 默认 false，仿真预算默认 900 秒；上面的命令显式选择实验室和 1800 秒。另有运行态 3600 秒墙钟上限；仿真秒不等于墙钟秒。因故障暂停等待用户时不会自动靠预算结束，需显式继续或结束保存。

另开终端，进入同一仓库并 source 相同环境：

```bash
ros2 topic echo /exploration/status
# 暂停并停车
ros2 service call /exploration/pause std_srvs/srv/SetBool '{data: true}'
# 显式继续
ros2 service call /exploration/pause std_srvs/srv/SetBool '{data: false}'
# 提前结束；停车验证后保存部分地图
ros2 service call /exploration/finish std_srvs/srv/Trigger '{}'
```

`WAITING / EXPLORING / PAUSED / STOPPING / SAVING / DONE / ERROR` 分别表示等待、探索、暂停、验证停车、保存、结束、异常。不要在 `SAVING` 时关闭；结束服务返回只代表接受请求，需等待 `DONE` 或 `map_saved` 后再退出启动。

### 7.3 怎样判断成功

完整自动建图至少应同时满足：无人工目标/驾驶、地图布局和障碍合理且无明显重影、可达区域没有大片遗漏、`DONE` 且原因 `NO_FRONTIERS`、`session.json` 中 `complete=true` 和 `stop_verified=true`，并有非空的 `map.yaml / map.pgm / map.posegraph / map.data`。

`DONE` 只表示任务结束并保存；`TIME_LIMIT_PARTIAL`、`NO_REACHABLE_FRONTIERS_PARTIAL`、`USER_FINISHED_PARTIAL` 等都只能称部分地图。`NO_FRONTIERS` 也只是本算法阈值下没有前沿，还须人工核对布局、未知区和质量。不能以车不动、文件存在或白色面积大就认定全图完成。

同一启动终端的 `SMARTCAR_ACCEPT_MAP` 保留着路径；可在正常退出后检查：

```bash
python3 - "$SMARTCAR_ACCEPT_MAP" <<'PY'
import json
import sys
from pathlib import Path
p = Path(sys.argv[1])
s = json.loads((p / 'session.json').read_text())
print({k: s.get(k) for k in ['state', 'reason', 'complete', 'stop_verified', 'prior_map_loaded']})
for name in ['map.yaml', 'map.pgm', 'map.posegraph', 'map.data']:
    f = p / name
    print(name, f.stat().st_size if f.exists() else 'MISSING')
print('记录目录:', p)
PY
```

保留整个会话，包括 `events.jsonl`、各 CSV、`map_initial.npz / map_final.npz`、地图四件套和 `session.json`。直接 Ctrl+C 不保证完成地图保存。更完整的界面说明和布局参考见 [discovery_lab 用户验收](docs/DISCOVERY_LAB_ACCEPTANCE.md)。

### 7.4 使用刚生成的地图导航

等待保存完成，退出探索启动后，在保留路径变量的终端执行：

```bash
ros2 launch smartcar_nav2 navigation.launch.py \
  world:=discovery_lab rviz:=true headless:=false \
  map:="$SMARTCAR_ACCEPT_MAP/map.yaml"
```

此时才进入 AMCL 已知地图导航，在 RViz 使用 Nav2 Goal；定位明显不对时用 2D Pose Estimate。场景必须与地图对应，不要为实验室加载办公室 office_01。

普通办公室导航使用 `ros2 launch smartcar_nav2 navigation.launch.py world:=office_loop rviz:=true`，默认加载安装的 office_01；取消使用 `ros2 run smartcar_nav2 navigate.py --cancel`。两条通道均堵死时，应有限失败停车；移障后先让雷达重新观察，再由用户发送新目标。

## 8. 关键源码、工具与证据索引

| 入口 | 用途 |
|---|---|
| [ros2_ws/tools/env.bash](ros2_ws/tools/env.bash)、[build.bash](ros2_ws/tools/build.bash)、[test.bash](ros2_ws/tools/test.bash) | 正确工作空间环境、构建与软件回归 |
| [smartcar_sim](ros2_ws/src/smartcar_sim/README.md)、[场景](ros2_ws/src/smartcar_sim/worlds/README.md) | 模型、传感器、场景及基础控制 |
| [smartcar_slam](ros2_ws/src/smartcar_slam/README.md) | SLAM 配置和图结构保存；自动入口使用专用覆盖 |
| [navigation.launch.py](ros2_ws/src/smartcar_nav2/launch/navigation.launch.py) | 已知地图/建图模式的导航组件、局部安全服务与参数组合 |
| [navigation_gate.py](ros2_ws/src/smartcar_nav2/scripts/navigation_gate.py)、[gate_core.py](ros2_ws/src/smartcar_nav2/scripts/gate_core.py) | 门控状态、输入时效、任务心跳与锁存 |
| [explore.py](ros2_ws/src/smartcar_slam/scripts/explore.py)、[frontier_core.py](ros2_ws/src/smartcar_slam/scripts/frontier_core.py) | 探索状态机、选点、受阻记忆、停车保存 |
| [verify_exploration.py](ros2_ws/tools/verify_exploration.py) | 主动启动隔离仿真，记录探索/地图/停止等真实结果 |
| [verify_map_export.py](ros2_ws/tools/verify_map_export.py) | 真实 Nav2 地图服务器逐格重载检查 |
| [verify_navigation.py](ros2_ws/tools/verify_navigation.py) | 主动启动隔离仿真及短目标导航/停车验收 |
| [benchmark_controllers.py](ros2_ws/tools/benchmark_controllers.py)、[report_controllers.py](ros2_ws/tools/report_controllers.py) | DWB/RPP/MPPI 正式对比和报表 |
| [report_exploration_speed.py](ros2_ws/tools/report_exploration_speed.py) | 离线生成探索前后比较报表 |
| [firmware](firmware)、[protocol](protocol)、[desktop](desktop) | 底层固件、公共帧协议及现有本机控制台 |
| [GitHub CI](.github/workflows/ros2.yml) | 三包构建和软件检查；不能代替 Gazebo/实车 |

证据按阶段查阅：

- [ROS 迁入](docs/evidence/ros2-integration-20261018/README.md)
- [遇障重规划与转向空间](docs/evidence/nav2-replanning-20261019/README.md)
- [双通道封堵与障碍记忆](docs/evidence/nav2-blocked-passages-20261019/README.md)
- [TF 失败与 Reset](docs/evidence/nav2-tf-reset-20261019/README.md)
- [控制器比较](docs/evidence/controller-comparison-20261019/README.md)
- [最初自主探索](docs/evidence/autonomous-mapping-20261021/README.md)
- [全新实验室场景](docs/evidence/discovery-lab-20261021/README.md)
- [最终探索效率优化及所有开发失败](docs/evidence/exploration-speed-20261021/README.md)
- [完整验收与波形](docs/ACCEPTANCE.md)、[长期进度](docs/PROGRESS.md)、[证据归档规范](docs/ACCEPTANCE_WORKFLOW.md)

可移植验收资料以 `docs/evidence/` 为准。`out/exploration-speed-20261021/` 有本机运行及发布审计，但属于忽略目录，新机器不一定有。`ros2_ws/build / install / log` 是本机产物，不是移植依赖。保留失败、压缩原字节映射和 SHA256；不要删大 SVG/JSONL 或用平滑图替代原始点。部分 SVG 约 75 MB，GitHub 已有文件大小建议提示，不影响本次作者更正和既有证据真实性。

## 9. 尚未完成与下一轮建议

上一轮探索效率优化、完整证据归档和作者更正均已交付，没有待继续的后台实验；当前未提交的只有本轮交接文档。用户本次只要求总结交接，尚未指定下一项实现任务。接手后先核对状态，再依据用户新指令选工作；以下是待办依据，不是自动授权。

1. **用户亲自完整验收陌生环境建图。** 最终大场景存在不可达/受阻前沿导致部分结束，需保留用户新会话，分析剩余灰区、候选位置、失败记忆、地图质量与导航失败，不能只把结束标志改为完整。
2. **探索尾段与稳健性。** 办公室到 95% 很快，但任务部分结束仅快 5.3%；实验室第二轮路程增加。可继续研究尾段补盲、剩余前沿可达性和回访，但必须同场景比较，保留异常且不得降低保护/到达容差。
3. **SLAM 质量泛化。** 更严格闭环门槛减少所测错误闭环，也可能漏掉真实闭环；需要新的陌生场景、长走廊和更长运行验证。不要将细响应 0.80 当所有环境通用最优。
4. **跨启动恢复探索。** 已保存 `.posegraph/.data`，当前自主入口仍每次从空白开始；尚未提供完整的跨启动自主续接流程。
5. **导航/控制器进一步验收。** 更多 GUI 多目标、移动障碍、任意初始位置重定位、长期运行；MPPI 只是后续验证候选，切换默认需要新证据和用户任务。
6. **实车接入。** 先按硬件集成文档确认供电/通信、ICM 真实采样和 C1，再做只读桥、所有权/单位/限幅/时间同步离线测试，之后经明确授权进行架空、落地标定和实车建图导航。

实车特别要保留的事实：

- 设备最后确认 v127；v131 ICM-42688-P 驱动只有软件构建/故障回归记录，未安装。历史主控 3.3 V 轨异常和通信维修状态须重新询问/核实，不能推断已修复。
- 现有接口的左右目标/反馈为**电机轴 rpm**，减速比 36:1；±720 电机轴 rpm 等于 ±20 轮 rpm，按 65 mm 轮径理论直行约 0.0681 m/s，远低于仿真 0.20/0.30 m/s。
- 右轮硬件极性已在底层归一化，未来 ROS 桥不能再次反号；CAN `0x200` 是电流命令，不是 rpm 速度帧。
- ROS 桥应复用同一个本机控制台的所有权/会话/序号入口，不能绕过控制权直接循环发命令；现有 MCU 相对坐标外环也不能与 Nav2 同时控制。
- MCU 300 ms 控制超时、100 ms 反馈超时及锁存保留；已有低速二次停转、启动超调、左右起转差、落地停车距离和长期可靠性仍待实车验收。

具体字段、轮速换算、TF 唯一责任方和阶段验收见 [ROS2_HARDWARE_INTEGRATION.md](docs/ROS2_HARDWARE_INTEGRATION.md)，不要凭猜测实现不存在的遥测字段。

## 10. 下一轮协作规则与最小接手顺序

先做只读核对：

```bash
cd /home/jiazhengfu/Projects/STM32-CAN-SmartCar
git status --short --branch
git log -6 --format='%h | %an <%ae> | %s'
git remote -v
git config --local --get-regexp '^user\.'
git ls-remote origin refs/heads/feat/ros2-simulation-navigation
gh api repos/jiazhengfu912-lang/STM32-CAN-SmartCar/pulls/1 \
  --jq '{state,merged,head:.head.sha,branch:.head.ref}'
gh api 'repos/jiazhengfu912-lang/STM32-CAN-SmartCar/actions/runs?branch=feat/ros2-simulation-navigation&per_page=3' \
  --jq '[.workflow_runs[] | {head_sha,status,conclusion,url:.html_url}]'
```

本机 `gh` 历史版本为 2.4.0，`gh pr edit` 曾因旧 GraphQL projectCards 接口失败；必要且已授权的 PR 更新可使用 `gh api` REST。不读取/打印凭据，不向用户索取密码。没有修改 PR 的新任务时只读检查即可。

接手顺序：

1. 读 AGENTS 与本文，确认实际仓库、分支、未提交交接文档及远端 PR/CI；保留用户修改，不执行 reset/clean。
2. 核对现有仿真/GUI 进程和当前任务范围；用户偏好中文、具体步骤，调试其现有窗口时不要擅自再开第二套 Gazebo/RViz。
3. 阅读本轮相关源码和最近原始证据；当前大场景是部分结果、默认 RPP、实车桥未实现，这三项不能误报。
4. 有明确修复任务后再构建、测试、启动隔离实验；每轮按 AGENTS 和验收规范保存真实 CSV/JSON、SHA256、逐点 PNG/SVG，失败和缺测同样保留，波形直接嵌入 ACCEPTANCE。
5. 普通文档交接不需要新跑运动试验；实现变更的构建/软件/仿真/实车验证分别说明。未运行底层 PI 时要写明，不能生成假实物波形。
6. 保留 HAL、`.ioc`、Keil/EIDE 和厂商许可；自编函数有中文注释，不把其他工作空间的绝对路径写成运行依赖。
7. 后续提交使用用户身份；没有新的明确授权，不提交、推送、合并，不烧录或驱动实车。过去作者更正的历史重写不构成未来重写授权。

## 11. 可直接发给新对话的接手提示词

```text
请接手 /home/jiazhengfu/Projects/STM32-CAN-SmartCar。
先读根目录 AGENTS.md、README_HANDOFF.md 和 HANDOFF.md，再核对实际分支、未提交文档、PR #1/CI 与现有进程。
当前功能分支 feat/ros2-simulation-navigation，交接时 HEAD b77c32cbdaf06085ed7d2c2ff418c7d183bddad0；ROS 工作空间在仓库内 ros2_ws，不要误用 ros2-learning 或旧 smartcar_sim_ws。
已实现仿真导航、保留障碍的有限恢复、自主探索和速度优化。默认局部控制器是 RPP；DWB/RPP/MPPI 比较没有严格全通过的赢家。
最终大场景自主建图仍为 NO_REACHABLE_FRONTIERS_PARTIAL / complete=false，不能称全图完成。实车桥未实现，仿真结论不能当作实车验收。
最近四条提交的作者已更正为我 jiazhengfu912-lang，完整新旧 SHA 对照见 README_HANDOFF.md；保留历史证据旧 SHA。
本轮交接文档尚未提交。保留所有用户修改，不 reset/clean，不擅自启动运动、重开用户窗口、提交、推送、合并或操作实车。
读完后请简要确认当前进度、未完成项和工作区状态，再按我的下一项需求继续。
```

本文为工程交接快照，无法替代新会话对实时状态和原始证据的核对。新需求和用户后续授权优先于历史任务说明。
