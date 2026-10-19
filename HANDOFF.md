# SmartCar 项目上下文交接

## 后续更新：2026-10-19 导航修复与提交授权

下方第1—11节保留14:40交接时的历史快照；其中“仅交接”“未授权提交”等描述已由后续用户指令更新。用户已明确要求修复遇障导航，并在完成验证后要求“提交并推送”；当前授权提交、推送当前功能分支，不包含合并PR或实车操作。

- 分支仍为 `feat/ros2-simulation-navigation`，修复前提交为 `62d7e47`；最新提交和推送状态以 `git log`、远端及PR #1为准，不把历史HEAD当作后续最新版本。
- ROS仍位于 `ros2_ws`，实车桥尚未实现。固件、协议、控制台及安全门控未修改；本轮没有连接、烧录或运行实车。
- 已修复全局规划转向空间不足、自动清图遗忘堵路障碍、TF失败误报到达，以及Reset清理卡住。单目标与连续多目标保持1Hz重规划，自动恢复不清图且最多4轮；正常扫描射线清障保留。
- 新增本包 `safe_controller_server`、`safe_planner_server`。目标TF失败停车并上报，代价图TF/扫描过滤回调串行执行；不覆盖系统ROS库，不放宽速度、碰撞、到达或TF容差。
- 最终三包构建、40个pytest通过（colcon47项含7个包装，零错误/失败/跳过）；实际服务故障注入、重复及执行中Reset、新目标/取消、Gazebo绕行到达停车和双出口封堵有限失败停车通过。全部中间失败保留。
- 最新依据：[TF与Reset修复](docs/evidence/nav2-tf-reset-20261019/README.md)、[双通道障碍记忆](docs/evidence/nav2-blocked-passages-20261019/README.md)、[转向空间与重规划](docs/evidence/nav2-replanning-20261019/README.md)，波形直接嵌入 [ACCEPTANCE](docs/ACCEPTANCE.md)。
- 用户当前窗口未由助手重启或发送目标。旧进程不会随构建热更新；使用新程序须先结束旧启动，再按原场景/定位参数启动。仍由用户在RViz设目标、Gazebo放障碍；不重复打开窗口，不批量杀进程。
- 本次提交包含上述代码、完整证据、验收文档及此前保留的AGENTS/HANDOFF。后续更多GUI路线、移动障碍、长期运行与实车均须独立验收。PR保持未合并；CI结果以本次推送后的实际状态为准。

---

更新时间：2026-10-19，Asia/Shanghai。GitHub/进程状态检查约 14:40；这些状态可能变化，接手后重新核对。

> 当前成果：ROS 2 仿真、SLAM、Nav2 已纳入底层仓库，完成本机编译、单元测试、一次真实 Gazebo 单目标导航与停车验证，已推送分支并创建 PR。两次远端 CI 已成功。**ROS 实车桥尚未实现，未连接或验收实车。PR 尚未合并。**

## 1. 首先找到正确项目与分支

本交接对应的项目根目录是：

```text
/home/jiazhengfu/Projects/STM32-CAN-SmartCar
```

不是当前聊天最初的工作目录 `/home/jiazhengfu/Projects/ros2-learning`。后者仍存在，本次检查不是 Git 仓库，不应在那里修改 SmartCar。

| 路径 | 用途与注意事项 |
|---|---|
| `/home/jiazhengfu/Projects/STM32-CAN-SmartCar` | 继续开发的 Git 仓库，本文所在目录 |
| 仓库内 `ros2_ws/` | 已迁入并验证的 ROS 工作空间；今后优先修改这里 |
| `/home/jiazhengfu/Projects/smartcar_sim_ws` | 原独立仿真工作空间，仍存在；保留原地图图结构与较完整历史日志，不是本轮 Git 工作空间 |
| `/home/jiazhengfu/Projects/ros2-learning` | 此聊天原始默认目录，不作为本任务源码仓库 |

Git 状态（写本文之前核对）：

- 远端：`https://github.com/jiazhengfu912-lang/STM32-CAN-SmartCar.git`。
- 当前分支：`feat/ros2-simulation-navigation`，跟踪 `origin/feat/ros2-simulation-navigation`。
- 本地与远端分支 HEAD：`62d7e4779532c4eec549dbd22700a02efaa81dd5`（短 SHA `62d7e47`）。
- 集成前基线 / 当前远端 main：`19d6c23d5f27f123485fc55d72b5523fad34d182`。
- 集成提交标题：`feat: integrate ROS 2 simulation, SLAM and Nav2 workspace`；作者为 Codex。
- [PR #1](https://github.com/jiazhengfu912-lang/STM32-CAN-SmartCar/pull/1)：open、未合并，检查时 `mergeable=true`。
- 本次交接开始时工作区干净；完成交接后预期只有 `AGENTS.md` 修改、`HANDOFF.md` 未跟踪。**这两份交接文档未提交、未推送，因此新机器直接克隆看不到本文。**
- 用户本次明确要求只交接，不改业务代码、不擅自提交或推送；不要因整理交接而追加提交、推送或合并 PR。
- `main` 尚无本轮 ROS 工作空间。远端新克隆要选择上述功能分支；不能在 main 上误判源码丢失。

登录状态：本机 GitHub CLI 已以 `jiazhengfu912-lang` 登录，Git HTTPS 推送已实际成功。之前工具连接器返回的是另一个只读账号，不能据此继续断言仓库没有写权限。认证只在本机/GitHub 授权页面处理，不读取或打印凭据。

## 2. 用户目标、已确定范围与沟通方式

总体目标：双轮差速小车，上层负责二维感知、建图、定位、路径规划/避障/跟踪；底层负责电机实时闭环、遥测、故障保护及 OTA。

用户已经决定：

- 保留 RPLIDAR C1 和二维 SLAM/定位，暂不增加三维雷达或深度相机。
- 已从简单房间切换到办公室走廊场景 `office_loop`，保存了 office_01 地图。
- 需要目标点设置、自主定位、全局规划、障碍检测/避障、跟踪控制、恢复、到达停车，以及多目标导航测试。
- 最近一轮开发要求是把已有上层工程加入此底层仓库，现已完成软件整合并推送；并未明确授权实车烧录/运动。

用户是 Gazebo/RViz 初学者，偏好中文、具体操作步骤；询问界面配置时要指导其现有窗口，不自行另开 RViz。sudo 密码由用户在本机终端输入，不索取密码。不要把编译/静态检查成功说成仿真或实车运行成功。

主动轮直径 **65 mm**、宽度 **27 mm** 为用户确认值。模型轮距 **252 mm** 沿用原工程，尚非实车有效轮距标定结果。摄像头标称 1080P、2.8 mm，实际视场角未知；质量、惯量、摩擦与部分外参仍是估算。

## 3. 技术栈与运行环境

| 层 | 技术栈 | 状态/证据边界 |
|---|---|---|
| ROS 主机 | Ubuntu 22.04.5、ROS 2 Humble、Gazebo Fortress 6.18.0、Python 3.10、ament_cmake/colcon | 本机已构建并实际运行 |
| 二维建图 | SLAM Toolbox，模拟 C1 LaserScan、轮式里程计、TF | 源码及配置已迁入；迁入后本轮未重新进行整套建图验收 |
| 已知地图导航 | AMCL、NavFn A*、Regulated Pure Pursuit、Velocity Smoother、Nav2 行为树/代价地图 | 本轮实际验证一个短距离目标 |
| STM32 | STM32F103/Cortex-M3、HAL/CMSIS、C、CAN/C610/M2006、10 ms PI、Bootloader | 当前源码 v131；本轮未构建/安装/连接实物 |
| ESP32 | PlatformIO，espressif32 6.12.0 / Arduino 2.0.17；HTTP、UDP、USART 网关 | 原代码保留，本轮未构建/安装 |
| 本机控制台 | Python HTTP 服务 + HTML/JS，默认 `127.0.0.1:18765` | 原代码保留，ROS 硬件桥尚未接入 |
| CI | GitHub Actions，ubuntu-22.04 + `ros:humble-ros-base-jammy` | 远端构建/单元测试已成功；不运行 Gazebo 或实车 |

本机 `gh` 为 **2.4.0**，不支持 `gh auth login --git-protocol`。已成功使用 `gh config set git_protocol https --host github.com`、`gh auth login --hostname github.com --web --scopes workflow`、`gh auth setup-git`。本轮包含 `.github/workflows/ros2.yml`，登录时申请了 workflow scope。

版本实录：[environment.json](docs/evidence/ros2-integration-20261018/environment.json)。其中首次 dpkg 查询包含不存在的包名 `ignition-gazebo6` 并返回非零；实际库 `libignition-gazebo6` 与 `ign gazebo --versions` 给出 6.18.0，不能把查询包名错误当成 Gazebo 未安装。

## 4. 架构与关键文件

所有以下相对路径均相对于本文所在仓库根目录。

```text
STM32-CAN-SmartCar/
├── AGENTS.md                         长期项目约定，接手必读
├── HANDOFF.md                        本文，阶段性交接状态
├── README.md                         总入口与底层历史状态
├── firmware/
│   ├── stm32_app/                    HAL App、CubeMX、Keil/EIDE、BSP、业务控制
│   ├── stm32_bootloader/             Boot Guard、OTA接收及启动
│   └── esp32_gateway/                网关、缓存、HTTP/UDP控制与OTA发送
├── protocol/                         公共串口帧与CRC协议
├── desktop/                          本机网页控制台及IMU遥测解码
├── tests/                            原固件/协议/控制台主机测试
├── tools/                            原固件构建、测试、配置与验收作图
├── ros2_ws/
│   ├── tools/                        新ROS环境、依赖、构建、测试、隔离导航验收
│   ├── src/smartcar_sim/             模型、场景、传感器桥、键盘、速度保护
│   ├── src/smartcar_slam/            建图、保存/恢复图结构、只读SLAM检查
│   ├── src/smartcar_nav2/            AMCL/Nav2、导航门控、CLI与RViz配置
│   └── maps/office_01/               map.yaml + map.pgm，仅栅格地图
├── docs/                             架构、硬件接入、进度、验收与真实证据
└── .github/workflows/ros2.yml         三包构建与软件测试CI
```

### ROS 关键入口

| 文件 | 作用 |
|---|---|
| [ros2_ws/README.md](ros2_ws/README.md) | 运行方式、构建/验收和边界 |
| [env.bash](ros2_ws/tools/env.bash) | 按自身位置设置 SMARTCAR_WS、加载 Humble 和本仓库 install；默认域86、localhost |
| [build.bash](ros2_ws/tools/build.bash)、[test.bash](ros2_ws/tools/test.bash) | 清理旧 overlay 后构建三包；运行软件测试 |
| [verify_navigation.py](ros2_ws/tools/verify_navigation.py) | 主动启动隔离的无界面仿真、发一个目标、验收位移/停车、保存CSV/JSON/PNG/SVG和完整日志 |
| [sim.launch.py](ros2_ws/src/smartcar_sim/launch/sim.launch.py) | Fortress、机器人TF、状态/雷达/相机桥、超时保护；生成临时SDF |
| [robot_parameters.json](ros2_ws/src/smartcar_sim/config/robot_parameters.json) | 模型尺寸、传感器与估算参数 |
| [smartcar.urdf](ros2_ws/src/smartcar_sim/urdf/smartcar.urdf)、[models/smartcar](ros2_ws/src/smartcar_sim/models/smartcar) | RViz模型、不同传感器组合的SDF与网格 |
| [worlds/README.md](ros2_ws/src/smartcar_sim/worlds/README.md) | simple `smartcar` 与复杂 `office_loop` 场景、出生点/通道说明 |
| [cmd_guard.py](ros2_ws/src/smartcar_sim/scripts/cmd_guard.py) / [command_guard_core.py](ros2_ws/src/smartcar_sim/scripts/command_guard_core.py) | 原速度超时与限幅保护 |
| [keyboard_drive.py](ros2_ws/src/smartcar_sim/scripts/keyboard_drive.py) | W/S/A/D、空格停车；仿真前进0.30、后退0.15m/s，转向0.8rad/s |
| [mapping.launch.py](ros2_ws/src/smartcar_slam/launch/mapping.launch.py) | 启动一套仿真+SLAM；`slam.launch.py` 仅附加SLAM到已有仿真 |
| [slam.yaml](ros2_ws/src/smartcar_slam/config/slam.yaml)、[save_map.py](ros2_ws/src/smartcar_slam/scripts/save_map.py) | SLAM参数、栅格图及图结构保存 |
| [navigation.launch.py](ros2_ws/src/smartcar_nav2/launch/navigation.launch.py) | 一套Gazebo + 保存地图 + AMCL + Nav2；不启动SLAM |
| [nav2.yaml](ros2_ws/src/smartcar_nav2/config/nav2.yaml) | AMCL、规划/跟踪/代价地图/平滑/恢复等参数；当前为仿真配置 |
| [navigate_recovery.xml](ros2_ws/src/smartcar_nav2/config/navigate_recovery.xml) | 单目标周期重规划及恢复行为树 |
| [navigation_gate.py](ros2_ws/src/smartcar_nav2/scripts/navigation_gate.py) / [gate_core.py](ros2_ws/src/smartcar_nav2/scripts/gate_core.py) | 指令/扫描/里程计/时钟/TF/发布者检查与软件停车 |
| [navigate.py](ros2_ws/src/smartcar_nav2/scripts/navigate.py)、[check_navigation.py](ros2_ws/src/smartcar_nav2/scripts/check_navigation.py) | 单目标/取消CLI、只读就绪检查 |
| [navigation.rviz](ros2_ws/src/smartcar_nav2/rviz/navigation.rviz) | Nav2目标工具、地图、路径、代价地图等显示 |

### 数据与 TF 链路

```text
建图：模拟 C1 /sim/scan + /sim/odom + TF → SLAM Toolbox → /map、map→odom
导航：保存的栅格地图 + 模拟C1 + 里程计 → AMCL → map→odom

Nav2 controller/behavior → /nav/cmd_vel_raw → Velocity Smoother → /nav/cmd_vel
  → navigation_gate → /nav/safe_cmd_vel → sim_cmd_guard → /sim/drive_cmd_vel
  → Gazebo /model/smartcar/cmd_vel

TF：map --(SLAM或AMCL唯一发布者)--> odom --(Gazebo里程计)--> base_footprint
      → base_link → laser_frame / camera等（robot_state_publisher）
仿真时钟：Gazebo → /clock；仿真节点 use_sim_time=true
```

基础/建图模式键盘发布 `/sim/cmd_vel`；导航模式的 guard 输入重映射为 `/nav/safe_cmd_vel`，旧键盘入口不驱动导航车辆。不要让两套控制源抢同一话题。

门控按墙钟检查：指令0.3s、扫描0.6s、里程计0.5s、时钟推进0.5s；另检查定位、TF和发布者冲突。空闲时 `COMMAND_STALE` 且输出零可以是正常状态。`/navigation/stop` 为 SetBool 软件锁存停车并请求取消导航；解除后应发新目标，不自动恢复旧目标。它不是实车硬件急停。

### 底层关键入口

- `firmware/stm32_app/Core/Inc/drive.h`：DRIVE_VERSION=131、状态/故障等定义。
- `firmware/stm32_app/Core/Src/drive.c`：轮速PI、使能/序号/超时/故障锁存；`smartcar.c`：CAN/UART/遥测业务入口。
- 同目录 `imu.c`、`navigation.c`、`angle_speed.c`、`BSP_*.c`：ICM驱动、MCU相对定位、编码器测速及板级服务。
- `firmware/stm32_app/SmartCar.ioc`、`MDK-ARM/SmartCar.uvprojx`；`firmware/stm32_bootloader/MDK-ARM/Bootloader.uvprojx`：保留原工程布局。
- `firmware/esp32_gateway/src/main.cpp`、`control_datagram.c`：网关及控制数据报。
- `protocol/car_protocol.h/.c`：实际公共帧定义；`desktop/server.py`：本机控制权、认证、遥测和OTA入口；`desktop/imu_protocol.py`：112字节消息解码。
- [硬件集成说明](docs/ROS2_HARDWARE_INTEGRATION.md)为未来桥接最重要入口；[ICM说明](docs/ICM42688_UPGRADE.md)、[进度](docs/PROGRESS.md)、[验收](docs/ACCEPTANCE.md)用于硬件历史核对。
- `docs/ARCHITECTURE.md` 的早期协议正文属于历史版本，不能不对照新说明/源码就作为v131完整接口依据。

## 5. 本轮已完成的修改

1. 导入三个既有ROS包及源网格、两类场景、配置和许可说明；包含办公室YAML/PGM，不包含原27MB级图结构文件。
2. 增加可移植工作空间环境、依赖安装/构建/测试脚本；旧 `sim_env.bash` 兼容入口向上寻找新工作空间，避免继续指向原目录。
3. 导航默认地图从安装包读取；CMake把 `ros2_ws/maps/office_01` 安装到导航包share，不能只拷三个src包而漏掉地图。
4. 把模型17、场景3、SLAM4、门控8个用例纳入colcon测试；增加Apache文本/来源说明。
5. 停用旧单包ZIP工具，避免缺失SLAM/Nav2/环境脚本；静态检查必须指定新输出目录，不覆盖历史证据。修复其对Fortress多顶层GUI配置的误判。
6. 增加 `verify_navigation.py` 和真实本轮证据；更新README、架构、进度、ACCEPTANCE并直接嵌入波形。
7. 增加ROS软件CI与忽略规则，保留build/install/log/out为生成物。
8. 依据真实底层源码整理硬件桥方案和限制；没有修改固件/ESP32/协议/网页业务。
9. 经用户授权，提交、推送 `62d7e47`，创建并附加PR #1；本机GitHub登录已完成。

本次交接只新增本文并补充AGENTS中的长期ROS约定，没有修改业务代码、重跑运动、提交或推送。

## 6. 已验证事实、结果与证据

### 本轮可复核的本机证据

证据根目录：[docs/evidence/ros2-integration-20261018](docs/evidence/ros2-integration-20261018/README.md)。目录名是开始日期；实际导航于2026-10-19执行。原始数据不要改写。

| 检查 | 已验证结果 | 证据 |
|---|---|---|
| 三个ROS包本机构建 | PASS | `build.log`、`build_final.log`、`build_after_tools.log` |
| 软件测试 | 32个pytest用例PASS；colcon显示36项含四个CTest包装，0失败/跳过 | `ros_tests.log` |
| 单包静态检查 | static_01 FAIL：多顶层GUI被误当单根XML；检查器修正后static_02 PASS | 对应 `report.json/tests.log`，首次失败保留 |
| 新路径包解析 | 三包prefix都来自本仓库install | `package_paths.log` |
| 实际Gazebo运行 | 本仓库产物，办公室场景、C1模拟、默认栅格地图、AMCL/Nav2；九个生命周期及扫描/地图/里程计/时钟/TF就绪 | `runtime_01/result.json`、`launch.log`、`events.jsonl` |
| 单目标与实际移动 | map目标(1.5,0,0)，action=SUCCEEDED(4)，odom净位移1.398692m | 同上及 `odom.csv`、`feedback.csv` |
| 到达后持续停车 | 连续1.46754s/68个样本低于0.02m/s与0.03rad/s，位移0.000674m，末次新鲜安全速度为零 | `stop_evidence`、`safe_cmd_vel.csv` |
| 原代码未变 | 576个原固件/协议/控制台文件哈希与基线一致 | `firmware_unchanged.json` |
| 原固件完整主机回归 | **未全通过**：27个协议测试PASS；Boot Guard 11项中1项因没有 `out/app/app.bin` 报错，剩余套件没执行 | `firmware_tests_full.log`、`firmware_host_tests.log` |

实际导航记录约23.33s：718条odom、372条安全速度命令、67条门控状态、14,365条时钟、148条反馈。相应CSV、逐点 `velocity.png/.svg`、`trajectory.png/.svg` 与完整ROS日志均保留，未平滑/补点。Nav2位置容差0.15m，最终odom约(1.39863,0.01285)，不能声称精确到达数值坐标或独立测得定位精度。

运行日志有VMware/EGL警告、第一次唯一goal checker选择提示；模拟扫描实际到达，导航成功。成功/停车之后SIGINT关闭期间有进程 `exit code -2`，没有删掉；本次自建进程组无遗留，launch返回0。不能把关闭时记录与运行期间碰撞失败混淆。

归档内 `summary.json` 的 `remote_ci=NOT_RUN` 是提交前快照；不为交接修改原证据。当前CI结果如下，属于本次只读在线核对的新事实。

### 已验证的远端状态

- [push CI 39207776025](https://github.com/jiazhengfu912-lang/STM32-CAN-SmartCar/actions/runs/39207776025)：completed / **success**。
- [PR CI 39207864527](https://github.com/jiazhengfu912-lang/STM32-CAN-SmartCar/actions/runs/39207864527)：completed / **success**。
- 两次运行均对应 `62d7e4779532c4eec549dbd22700a02efaa81dd5`；这里只证明CI构建/软件测试。
- PR open、未merge；远端功能分支与本地HEAD一致，main仍是基线。
- 本次交接检查未发现运行中的Gazebo/RViz或本项目控制台；启动前仍需重新检查，不能假定此状态持久。

### 原工作空间的历史证据（不替代新目录验收）

仍在 `/home/jiazhengfu/Projects/smartcar_sim_ws/verification_runs/`，可查 `ubuntu_20261016/REPORT.md`、`slam_20261017/REPORT.md`、`nav2_20261018/REPORT.md`、`complex_scene_20261018/REPORT.md`、`speed_20261018/summary.json`。包含原运动、传感器、建图/恢复、选定目标、临时静态障碍和故障注入记录，也有失败/振荡记录；深入引用前必须读取对应工况及原始日志。

原始 `maps/office_01/map.posegraph`、`map.data` 已确认仍在旧工作空间。新仓库只有栅格地图；后续如果要继续原SLAM图，另行复制/归档原文件并明确来源，不从PGM推断图结构已恢复。

## 7. 未验证假设与待解决问题

### 未验证的假设/建议，不能写成实现或通过

- 252mm模型轮距可作为初始几何输入，实际落地有效轮距/轮径仍需标定；物理参数/外参/相机FOV不能当实测。
- MCU有效 `x/y/heading` 可作为未来ROS局部odom候选；仍需核对时间、原点代次、连续性、协方差、方向及误差，当前没有发布硬件odom的ROS节点。
- 通过同一个本机控制台接入ROS控制权是已记录的设计路线，尚无硬件桥、仲裁UI或实车启动文件。
- 单次短直行成功不能推出任意地图/目标、多目标、移动障碍或长期运行都可靠，也不证明未知初始位置下全局重定位通过。
- 当前源码v131与“设备最后确认v127”来自固件历史资料；本轮未在线查询设备，现时维修/供电/版本要重新核对。

### 已知问题、Bug与边界

1. **实车主链路缺失**：真实C1驱动、ROS硬件桥、硬件odom/TF、时钟映射、独立实车Nav2配置尚未实现/验收。
2. **硬件速度不匹配**：±720是电机轴rpm，36:1后车轮20rpm；65mm轮径仅约0.0681m/s。仿真0.30m/s需约3173电机轴rpm；按252mm轮距转0.60rad/s也约需800rpm，不能直接复用。
3. **硬件历史故障未闭环**：资料记录主控3.3V轨异常升至4.6V/通信异常；v125低速右轮起转后再停、启动超调/左右差等仍未解决。不要默认维修完成。
4. **ICM未实测**：v131用ICM-42688-P、I²C2 PB10/PB11，替换旧MPU；安装、0x47身份、真实新样本、轴向、校准/漂移、落地表现待验证。
5. **固件全套回归受环境阻塞**：新目录无原Keil构建镜像，测试不能全绿。不可伪造 `app.bin` 或跳过Boot Guard改成成功。
6. **临时障碍可能让目标失败**：历史出现RPP `detected collision ahead`、Controller patience exceeded、Goal failed；可能是通道被堵、定位/轮廓/代价地图/局部跟踪问题，需结合当轮数据分析。RPP主要跟踪全局路径，绕障依赖障碍层及全局重规划，不保证任意堵塞可通行。
7. **多目标GUI残留未彻底诊断**：用户曾反馈停止后目标标记仍不消失。不要把取消动作、清空待发目标和清除RViz显示缓存混为一谈；本轮未复现/修复该交互。
8. **Reset/Startup不重发旧目标**：恢复生命周期、删障碍不等于重新提交已失败导航动作；需要重新发目标。切场景/重启时避免混用旧SLAM状态。
9. **二维观测局限**：C1只能测其扫描平面，不提供物体完整高度/厚度或语义识别；用户决定暂不加3D传感器。
10. **双工作空间/TF冲突**：两个相同包的overlay、重复仿真时钟/扫描、SLAM与AMCL同时发布map→odom会混淆运行；仅换DDS域不能隔离Gazebo Transport。

## 8. 已确定技术方案及原因

- **同仓库、分层运行**：ROS工作空间放 `ros2_ws/`，不把Nav2编进STM32。保留实时CAN/PI与300ms控制、100ms反馈保护，避免上层集成破坏原底层行为。
- **保留二维C1+SLAM**：满足用户当前范围，后续先完善二维导航与实车接口，不自行扩展3D/深度相机。
- **建图与AMCL导航分开**：各自拥有唯一map→odom发布者；保存的YAML/PGM用于定位导航，继续建图必须有posegraph/data。
- **仿真控制链分离**：专用Nav2速度链、平滑器、门控、原超时保护，避免键盘抢控制；不能为了“能动”旁路保护或增加重复发布者。
- **自定位工作空间路径**：新环境脚本从自身路径推导目录；默认地图随导航包安装，避免依赖旧机器路径。
- **隔离验收**：验证器拒绝默认用户域86，默认域88+随机IGN/GZ分区，只关闭自己创建的进程组；输出新目录，失败也保留。
- **硬件桥复用真实协议和单一控制权**：优先同一个本地控制台，不另建独立UDP运动发送者；默认禁止驱动、人工使能、失效后不自动CLEAR/ARM。
- **实车参数独立**：电机轴rpm转换、共同缩放左右轮保持曲率、真实速度/加减速/停距配置；不放宽±720边界去迁就仿真速度。
- **唯一里程计/时间责任**：MCU已融合的估计不能再作为独立传感器重复融合；实车 `use_sim_time=false`，失效/复位/原点代次变化应停导航，不能给旧样本刷新时间戳。

### 不可忽略的底层接口事实

详见 [ROS2_HARDWARE_INTEGRATION.md](docs/ROS2_HARDWARE_INTEGRATION.md)，实现前再对照生产源码：

- `POST /api/control`：op、owner、Unix毫秒at、整数left/right电机轴rpm，X-Console-Token来自本地csrf；控制端所有者、ARM session、递增sequence不可跨客户端各自生成。
- ESP32 UDP3232为52字节：SCU1 + 32字节令牌 + 16字节小端 `<B3xIIhh>` 载荷；op2 SPEED用于Nav2轮速，op4是MCU内置导航续租，不是Nav2控制。
- 归一化双轮正转表示前进；STM32已处理右轮极性，ROS桥不能再次额外取反。
- 40字节运动遥测和112字节IMU/位置消息**没有暴露每轮累计编码器计数**；不能在ROS解析器里凭空读取。约10Hz HTTP读取也不能冒充连续50Hz采样。
- 指令失联不能无限重发最后非零速度；500ms遥测有效期不等于允许延长MCU300ms控制期限。故障恢复必须保持锁存和显式使能语义。
- 零电流禁止驱动属于滑停，不是机械抱闸；实物停止距离需测量。
- PI未改：P右反0.45、其余0.30；I稳态0.010、低速启动0.030，每10ms按电机轴rpm误差累加；D=0。Gazebo验证未运行这些PI。

## 9. 可复现命令与注意事项

### 第一步：只读检查与加载正确工作空间

```bash
cd /home/jiazhengfu/Projects/STM32-CAN-SmartCar
git status --short --branch
git log -1 --oneline
source ros2_ws/tools/env.bash
ros2 pkg prefix smartcar_sim
ros2 pkg prefix smartcar_slam
ros2 pkg prefix smartcar_nav2
```

三个prefix应指向本仓库 `ros2_ws/install/`。新终端都重新source。源码变化需要时再运行 `bash ros2_ws/tools/build.bash` 和 `bash ros2_ws/tools/test.bash`；交接文档修改本身不需要重跑运动。

### 根据下一项明确目标选择一个模式

以下命令会启动仿真/可能打开窗口；启动前核对现有进程，不重复打开用户窗口，也不按进程名称批量杀死其他会话。

```bash
# 基础仿真（已有RViz时使用rviz:=false）
ros2 launch smartcar_sim sim.launch.py world:=office_loop lidar:=true rviz:=false
# 另一个已source的终端：W/S/A/D，空格停；不要与导航模式混用
ros2 run smartcar_sim keyboard_drive.py

# 建图：与基础模式二选一，不在已运行仿真上再次启动mapping
ros2 launch smartcar_slam mapping.launch.py world:=office_loop rviz:=true
# 保存至尚不存在的目录
ros2 run smartcar_slam save_map.py --output-dir "$SMARTCAR_WS/maps/my_office_02"

# 已保存地图导航：与建图/基础模式二选一，默认安装的office_01图
ros2 launch smartcar_nav2 navigation.launch.py world:=office_loop rviz:=true
# 另一个终端发目标/取消
ros2 run smartcar_nav2 navigate.py --x 1.5 --y 0.0 --yaw 0.0
ros2 run smartcar_nav2 navigate.py --cancel
```

继续旧SLAM图可使用mapping的 `map_file:=/绝对路径/原map前缀`，要求同前缀的 `.posegraph` 和 `.data`、匹配场景和合理初始位姿；不能传PGM/YAML当图结构。旧图的具体恢复条件先读SLAM README，不把“打开保存地图”误当“重新建图”。

主动隔离验收（会发目标并运动），只在需要验证代码变化时执行，输出目录必须全新：

```bash
python3 ros2_ws/tools/verify_navigation.py \
  --output "$PWD/ros2_ws/verification_runs/navigation_next_01"
```

默认域88、随机IGN_PARTITION，拒绝域86，拒绝覆盖证据；失败时查输出目录的result/readiness/events/launch.log，不用静态测试代替。自行并行启动仿真时需同时设置不同ROS_DOMAIN_ID与IGN_PARTITION，每组所有终端一致。

原固件测试入口是根目录 `tools/test.py`，不是ROS的 `ros2_ws/tools/test.bash`。Linux须设置 `HOST_GCC=/usr/bin/gcc` 并有Node；完整镜像用例还依赖实际构建的App/Boot镜像。原构建默认Windows Keil工具链，可用ARM_BIN覆盖，但先验证可用工具链，不能把历史构建结论当本机新构建。该测试会覆盖 `out/tests.log`，运行前先归档本轮需要保留的日志。

GitHub CLI若GraphQL命令偶发 `EOF`，先查询是否已成功创建，避免重复PR；本轮已用REST `gh api` 创建PR成功。不要为绕过问题输出token。

## 10. 下一步具体执行顺序

1. **先读 AGENTS、本HANDOFF、ROS README、硬件集成说明和本轮证据README**，只读核对当前分支/工作区、PR/CI与现有进程。保留这两份未提交交接文档，不reset/clean掉。
2. **核对用户下一阶段意图**：这轮明确任务是交接，暂时没有新功能实施授权；新会话确认是继续仿真、多目标交互，还是启动实车桥开发。不要把文档中的建议当成已经授权烧录/运动/合并。
3. 若继续仿真，优先在新仓库复现用户未闭环的多目标取消/标记残留，再做有绕行空间与完全堵路两类障碍场景。记录待发列表、action状态、RViz显示、代价地图、重规划与最终结果，区分显示残留和真正的动作队列。
4. 若开始硬件桥，先梳理只读状态/协议与离线接口，设计默认禁用、Twist→轮速、共同限幅、所有权、时间与样本失效测试；可考虑新增独立ROS硬件包，但当前不存在该包/驱动，不要写成既有接口。
5. 只有在核对电源维修、真实固件/网关/ICM/C1状态，并得到相应硬件操作授权后，才进入真实只读采集、架空低速符号/停止测试、落地标定、真实建图及导航，按硬件说明分阶段推进。
6. 任何新功能修复按验收流程保留原始CSV/JSON/哈希/逐点PNG/SVG、失败与缺测，更新ACCEPTANCE；软件、Gazebo和实车结论分开。不要改写这轮历史证据。
7. PR #1是否合并、是否提交/推送本交接文档，等待用户新的明确指令。本次不执行这些动作。

## 11. 接手检查清单与证据完整性

- 必读：[AGENTS.md](AGENTS.md)、[ROS快速开始](ros2_ws/README.md)、[硬件集成](docs/ROS2_HARDWARE_INTEGRATION.md)、[本轮证据](docs/evidence/ros2-integration-20261018/README.md)、[验收流程](docs/ACCEPTANCE_WORKFLOW.md)。
- `docs/evidence/ros2-integration-20261018/SHA256.json` 校验52个证据文件；`source_manifest.json` 校验103个ROS源文件；`runtime_01/result.json` 含5份CSV哈希与验证器源码哈希。交接时重新核对，不通过修改原始记录来修正差异。
- `.gitattributes` 对 `docs/evidence/**` 使用 `-text` 保留原字节。CSV的CRLF、原日志空格和生成SVG空白会触发一般whitespace提示，不能为格式整洁改写实测原始数据。
- `ros2_ws/build`、`install`、`log`、`verification_runs`、根目录 `out` 为忽略的本机生成物，当前存在但新克隆不保证存在；从源码重新构建。
- 本次交接仅检查源码/状态和文档、校验证据；未重新安装依赖、运行测试套件、启动Gazebo、连接硬件或发运动命令。编译/32用例/运行通过均来自上面明确的本轮记录；远端两次CI成功来自本次GitHub只读查询。

给新会话的建议首条消息：

> 请接手 `/home/jiazhengfu/Projects/STM32-CAN-SmartCar`。先读根目录 AGENTS.md 和 HANDOFF.md，再核对当前分支、工作区、PR #1及CI。当前功能分支为 feat/ros2-simulation-navigation，ROS源码在 ros2_ws；实车桥尚未实现。保留未提交交接文档，不自动提交/推送/合并或启动实车运动。随后结合我的下一项要求继续。
