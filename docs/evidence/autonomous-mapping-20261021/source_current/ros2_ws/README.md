# SmartCar ROS 2 上层工作空间

这里包含小车的 Gazebo 仿真、二维 SLAM 和 Nav2 导航。底层 STM32、ESP32、CAN 与控制台工程保留在仓库原目录；本工作空间中的仿真节点通过 ROS 话题控制 Gazebo，不能据此宣称已经接通实车。

目标环境：Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress。质量、惯量、摩擦、相机视场与部分传感器安装尺寸仍为估算参数；主动轮采用已确认的 65 mm 直径、27 mm 宽度，轮距沿用现有模型 252 mm。

## 目录与数据流

| 目录 | 内容 |
|---|---|
| `src/smartcar_sim` | 小车 CAD 来源、URDF/SDF、场景、C1/摄像头模拟、键盘和速度超时保护 |
| `src/smartcar_slam` | SLAM Toolbox 配置、二维地图生成、栅格图及图结构保存 |
| `src/smartcar_nav2` | AMCL 定位、NavFn 全局规划、RPP 跟踪、障碍层、恢复和目标接口 |
| `maps/office_01` | 已保存的办公室导航示例，仅 YAML/PGM |
| `tools` | 环境、依赖安装、构建、软件测试与隔离导航运行验收入口 |

```text
建图：Gazebo C1 + 里程计 TF → SLAM Toolbox → /map、map→odom
导航：保存地图 + C1 + 里程计 TF → AMCL → map→odom
      导航目标 → Nav2 规划/控制 → 速度平滑 → 导航门控 → 速度超时保护 → Gazebo
```

通常只启动一套仿真，建图与 AMCL 导航分别启动，避免两个节点同时发布 `map→odom`。确需并行时，必须同时设置不同的 `ROS_DOMAIN_ID` 和 `IGN_PARTITION`；仅换 ROS 域不能隔离 Gazebo Transport。每组实验的所有终端使用同一组值。

## 首次准备

先安装 ROS 2 Humble 并配置 Fortress 可用软件源。以下从本仓库根目录执行；sudo 密码只在本机终端输入：

```bash
bash ros2_ws/tools/install_dependencies.bash
bash ros2_ws/tools/build.bash
source ros2_ws/tools/env.bash
bash "$SMARTCAR_WS/tools/test.bash"
```

每个新终端从仓库根目录执行：

```bash
source ros2_ws/tools/env.bash
```

脚本按自身路径自动导出 `SMARTCAR_WS`，无需修改任何个人目录。默认使用 `ROS_DOMAIN_ID=86`、`ROS_LOCALHOST_ONLY=1`；如需另一隔离实验域，在 source 前显式导出 `ROS_DOMAIN_ID`，所有参与该实验的终端须保持一致。部分原有键盘、运动检查和 SLAM 保存脚本要求域 86；本地仿真默认配置无需覆盖。

`build.bash` 在独立 shell 中清除旧工作空间构建覆盖，编译本仓库三个包。构建后重新 source 环境，检查包路径应指向此仓库：

```bash
ros2 pkg prefix smartcar_sim
ros2 pkg prefix smartcar_slam
ros2 pkg prefix smartcar_nav2
```

## 三种启动方式

切换模式之前，先停车并结束旧仿真、旧 SLAM/导航与键盘进程。

**基础仿真和键盘驾驶**：

```bash
ros2 launch smartcar_sim sim.launch.py world:=office_loop lidar:=true
```

另一个已加载环境的终端：

```bash
ros2 run smartcar_sim keyboard_drive.py
```

W 前进、S 后退、A/D 转向、空格停车。前进 0.30 m/s、后退 0.15 m/s、转向 0.8 rad/s。键盘失联后速度保护超时停车。基础启动默认打开 Gazebo/RViz；已有自己的 RViz 时可添加 `rviz:=false`。

**二维 SLAM 建图**：

```bash
ros2 launch smartcar_slam mapping.launch.py world:=office_loop rviz:=true
```

用另一个终端启动键盘，逐渐探索环境。完成后保存到尚不存在的目录：

```bash
ros2 run smartcar_slam save_map.py --output-dir "$SMARTCAR_WS/maps/my_office_01"
```

YAML/PGM 供定位导航使用；`.posegraph/.data` 用于以后恢复 SLAM 图结构继续建图。仓库附带的 office_01 没有原图结构，不能直接用它恢复原来的建图任务。

**陌生环境自主探索建图（无需键盘或人工目标）**：

```bash
ros2 launch smartcar_nav2 exploration.launch.py world:=office_loop rviz:=true
```

先退出旧导航/建图启动，避免重复模拟时钟和定位发布者。该模式从空白地图自动选择前沿目标，采用建图专用慢速RPP，经原门控保护行驶；结束后停车并保存栅格和SLAM图结构。默认新建`~/smartcar_maps/explore_日期_时间`；暂停、显式继续、结束保存、部分地图原因及实际验证范围见[完整说明](../docs/AUTONOMOUS_MAPPING.md)。当前只驱动Gazebo，实车桥尚未实现。

**在保存地图上自主导航**：

```bash
ros2 launch smartcar_nav2 navigation.launch.py world:=office_loop rviz:=true
```

默认使用随导航包安装的 office_01 示例地图；使用新保存的地图时追加 `map:="$SMARTCAR_WS/maps/my_office_01/map.yaml"`。场景必须与地图对应。导航模式启用 AMCL，并不继续 SLAM 建图。

等待 RViz 的 Navigation/Localization 均 active，检查扫描与地图基本重合，然后用 **Nav2 Goal** 在空白可通行区域点击并拖出朝向；定位明显错误时先用 **2D Pose Estimate** 指定合理初始区域。也可发送地图坐标目标或取消：

```bash
ros2 run smartcar_nav2 navigate.py --x 2.0 --y 0.0 --yaw 0.0
# 取消当前导航动作
ros2 run smartcar_nav2 navigate.py --cancel
```

导航模式使用专门的速度链路，旧键盘入口不驱动车辆；需要手动驾驶时先退出导航并切回基础仿真或建图模式。

## 在新目录复现本轮导航验收

从仓库根目录执行，输出目录必须尚不存在：

```bash
python3 ros2_ws/tools/verify_navigation.py \
  --output "$PWD/ros2_ws/verification_runs/navigation_01"
```

该命令会主动启动独立的无界面 Gazebo，使用 ROS 域 88、随机 Gazebo 分区，向 `map` 坐标 `(1.5, 0, 0)` 发送一个目标，并在退出时清理自己创建的进程。原 ROS 域 86 的仿真不参与此次验收。检查内容包括九个生命周期节点、地图/扫描/里程计/TF 就绪、动作成功、实际里程计移动和到达后的持续低速与零命令。它保存完整日志、原始 CSV、结果 JSON、逐点 PNG/SVG；失败也保留结果，不覆盖前一轮。

本轮已在新仓库完成该测试；详细工况与限制见 [证据说明](../docs/evidence/ros2-integration-20261018/README.md)。此脚本不验证临时障碍、多目标或 STM32。

## 验证范围

DWB、RPP、MPPI 的隔离对比入口与统计口径见 [控制器对比说明](../docs/CONTROLLER_BENCHMARK.md)。该测试会主动运行新建的仿真并发目标，生成独立参数副本及CSV/HTML报表，不切换用户窗口中的控制器。

- `tools/test.bash` 验证模型/软件逻辑，不等于实际 Gazebo 运行、完成导航或实车验收。
- 仿真只读观察器为 `check_runtime.py`、`check_slam.py` 和 `check_navigation.py`；各自使用条件见包内 README。带 `--motion`、`--drive-square` 的测试会主动运动，必须按对应场地要求单独运行。
- 原工作空间曾验证基础运动、传感器、建图、选定导航目标、一个临时静态障碍及部分故障注入，完整历史证据留在原工作空间，未全部导入此仓库。
- 本次迁入仓库的构建、测试、隔离运行和真实日志见 [本轮证据目录](../docs/evidence/ros2-integration-20261018/) 与 [验收记录](../docs/ACCEPTANCE.md)。以实际记录为准，本文不预先写入通过结论。
- 历史运行出现过启动失败、控制器振荡、预测碰撞停车及恢复后目标失败。GUI 目标交互、任意多目标路线、移动障碍、未知初始位置重定位与长期稳定性仍需按场景验证。
- 未连接 STM32、实物 C1 或电机进行本轮联调；仿真成功不代表底盘实物控制、里程计标定或制动距离已通过。

旧单包 `smartcar_sim/tools/package_delivery.py` 已停用，避免生成缺少 SLAM/Nav2/环境脚本的不完整包。应通过整个 Git 仓库交付。`verify_delivery.py` 仍只检查仿真包静态内容，现在必须用 `--output` 指定新目录，历史记录不会覆盖。

进一步说明：[基础仿真](src/smartcar_sim/README.md)、[场景切换](src/smartcar_sim/worlds/README.md)、[SLAM](src/smartcar_slam/README.md)、[Nav2](src/smartcar_nav2/README.md)、[示例地图](maps/office_01/README.md)、[来源与许可](THIRD_PARTY_NOTICES.md)。
