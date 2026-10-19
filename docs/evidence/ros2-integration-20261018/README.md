# ROS 2 上层迁入底层仓库：本轮真实证据

本轮于 2026-10-18 开始，2026-10-19 完成隔离 Gazebo 运行；目录名保留开始日期。基线为 `19d6c23d5f27f123485fc55d72b5523fad34d182`，STM32 源码 v131。**结果仅限软件集成与仿真，不代表实车通过。**

## 修改前与本轮改动

底层仓库没有 ROS 工作空间；已运行过的 ROS 工程在另一目录，默认地图及环境入口带有开发机路径。现在 `ros2_ws` 包含 `smartcar_sim`、`smartcar_slam`、`smartcar_nav2`、办公室示例栅格地图，以及安装/构建/测试/运行验收入口。环境按脚本位置寻找工作空间，导航默认从安装包加载地图，不再依赖原目录。

保留模型中的 65 mm 主动轮直径、27 mm 宽度和 252 mm 模型轮距；质量、惯量、摩擦与部分安装参数仍是估算。未修改 STM32/ESP32/协议/控制台代码、PI、速度边界、心跳或故障锁存。新硬件接口说明列出单位、协议、控制权、时间与 TF 的适配责任，未实现或启用硬件桥。

## 结果与原始记录

| 检查 | 结果 | 记录 |
|---|---|---|
| 本机环境 | Ubuntu 22.04.5 / ROS 2 Humble / Gazebo Fortress 6.18.0 | [实际版本命令与输出](environment.json) |
| 新工作空间构建 | 三个包通过 | [首次构建](build.log)、[最终构建](build_final.log) |
| ROS 单元测试 | 32 个 pytest 用例通过；colcon 统计 36 项含四个 CTest 包装项，零失败/跳过 | [完整日志](ros_tests.log) |
| 包路径 | 三个包均来自本仓库 install，未借用原工作空间安装产物 | [路径](package_paths.log) |
| 单包静态检查 | 首轮误将 Fortress 多顶层 GUI 配置当成单根 XML，失败；按其原生格式修正检查器后通过 | [首轮 FAIL](static_01/report.json)、[修正后 PASS](static_02/report.json) |
| 新目录实际导航 | PASS，限下面的单目标工况 | [结果 JSON](runtime_01/result.json)、[启动/运行/关闭日志](runtime_01/launch.log)、[事件时间线](runtime_01/events.jsonl) |
| 原固件完整主机回归尝试 | **未全通过**；27 个协议测试通过，Boot Guard 的真实镜像检查缺少 `out/app/app.bin`，套件中断 | [完整失败日志](firmware_tests_full.log)、[控制台输出](firmware_host_tests.log) |
| 固件等原有代码 | 与基线一致 | [576 个原有文件的校验与变更检查](firmware_unchanged.json) |
| GitHub CI | 已增加构建/测试工作流，远端尚未执行 | [工作流](../../../.github/workflows/ros2.yml) |

环境记录中第一次 dpkg 查询包含不存在的包名 `ignition-gazebo6`，保留其返回码；实际安装库 `libignition-gazebo6` 与 `ign gazebo --versions` 均给出 6.18.0。固件测试没有通过伪造镜像、跳过用例或借用旧报告变成 PASS；本机未配置原 Keil 工具链，本轮没有重新构建、烧录或连接设备。

## 实际导航工况和判据

用本仓库构建产物启动 `office_loop`、默认随包安装的 office_01 地图、模拟 C1 与 AMCL，关闭摄像头、GUI 和 RViz。使用 ROS 域 88、localhost 与随机 `IGN_PARTITION`，没有接入设备。AMCL 初始猜测为出生点 `(0, 0, 0)`；这不验证未知初始位置下的全局重定位。

等待九个生命周期节点 active、单一发布者、地图/新鲜扫描/里程计/推进中的时钟、三段 TF 与门控就绪后，发送 `map` 中 `(1.5 m, 0 m, 0 rad)` 目标。Nav2 返回 `SUCCEEDED=4`。原始里程计净位移 **1.39869 m**，最后位置约 `(1.39863, 0.01285)`；配置的到点位置容差为 0.15 m，不能把动作成功宣传为零误差到达。

到达后检查连续 **1.46754 s、68 个里程计样本**：线速度均低于 0.02 m/s、角速度低于 0.03 rad/s，位移 **0.000674 m**，末次新鲜门控命令为零。窗口开始仍包含减速尾段，线速度峰值 0.01470 m/s，原样保留。整个记录约 23.33 s，含 718 条里程计、372 条安全命令、67 条门控状态、14,365 条时钟、148 条导航反馈。

![新目录实际 Gazebo 导航逐点速度，仅仿真](runtime_01/velocity.png)

**P 右反 0.45→0.45，其余 0.30→0.30；I 稳态 0.010→0.010、启动 0.030→0.030，每 10 ms 按电机轴 rpm 误差累加；D=0。**固件轮速、启动、IMU 和相对坐标导航算法均未改变。本图运行的是 Gazebo DiffDrive 和 ROS/Nav2，未执行 STM32 PI；图中的相近曲线不能证明实车速度跟踪或机械性能改善。新增加的是可移植工程组织及明确的单目标验收入口。

![同一轮原始里程计轨迹，仅仿真](runtime_01/trajectory.png)

全部采样点原样保存，线条只连接实收点，长缺口断线；没有平滑、插值或删除异常。这里的里程计来自 Gazebo，不是独立外部真值。数据和 [速度 SVG](runtime_01/velocity.svg)、[轨迹 SVG](runtime_01/trajectory.svg) 可复核：

- [odom.csv](runtime_01/odom.csv)：全部姿态与六轴速度；
- [safe_cmd_vel.csv](runtime_01/safe_cmd_vel.csv)：完整门控后速度命令；
- [safety_status.csv](runtime_01/safety_status.csv)：原始 JSON 状态与故障；
- [clock.csv](runtime_01/clock.csv)、[feedback.csv](runtime_01/feedback.csv)：原时钟及导航反馈。

启动日志保留 VMware/EGL 3D 警告；模拟扫描仍实际到达并参与就绪检查。控制器第一次提示未指定 goal checker，随后使用唯一配置的 `general_goal_checker`。测试在成功和停车检查后发 SIGINT 关闭自己创建的进程组；关闭期间有 `exit code -2` 记录，均保留，不作为运行期间导航失败。进程组最终无遗留，launch 返回 0。

## 尚待验证

本轮没有重跑临时障碍绕行、恢复/取消、任意多目标、未知位姿重定位、GUI 交互、摄像头和长时运行。原工作空间的历史记录不充当本次测试证据。未完成实车桥、真实 C1 接入、硬件参数与时间/里程计/TF 适配、控制权切换，以及全部实车运动验收。

原硬件 ±720 电机轴 rpm 按 36:1、65 mm 轮径只相当于约 0.0681 m/s，不能直接采用仿真的 0.30 m/s；详见 [硬件集成说明](../../ROS2_HARDWARE_INTEGRATION.md)。

[源工程导入前清单](import_manifest.json)、[最终 ROS 源码清单](source_manifest.json)、[验证摘要](summary.json)、[证据 SHA256](SHA256.json)。
