# SmartCar Nav2 自主导航

ROS 2 Humble / Gazebo Fortress，使用已保存的二维地图、C1 雷达和轮式里程计。导航模式启动 AMCL，不启动 SLAM Toolbox；AMCL 独占 map→odom，地图固定不再继续建图。

## 安装与构建

在本仓库根目录执行；无需使用原开发机的绝对路径：

```bash
bash ros2_ws/tools/install_dependencies.bash
bash ros2_ws/tools/build.bash
source ros2_ws/tools/env.bash
bash "$SMARTCAR_WS/tools/test.bash"
```

安装脚本要求 Ubuntu 22.04 和已安装的 ROS 2 Humble，sudo 密码只在本机终端输入。每个新终端从仓库根目录执行 `source ros2_ws/tools/env.bash`；它自动导出 `SMARTCAR_WS`，默认本机 ROS 域 86。隔离实验可在 source 前显式设置其他 `ROS_DOMAIN_ID`，相关终端必须使用一致的域。

## 启动

先停车，在原建图/仿真终端 Ctrl+C，退出旧键盘与单独SLAM节点。不要在原建图仿真运行时重复启动；启动器发现当前 ROS 域已有 /clock 或 /map 发布者会在创建仿真前拒绝。

```bash
ros2 launch smartcar_nav2 navigation.launch.py \
  world:=office_loop rviz:=true
```

默认加载本包安装目录 `share/smartcar_nav2/maps/office_01/map.yaml` 的办公室示例地图，不依赖旧工作空间，也不要求另外设置地图路径。使用自己的地图时添加 `map:="$SMARTCAR_WS/maps/my_map/map.yaml"`。

`map` 为已保存的 **YAML 文件**，和恢复建图的无扩展名 `map_file` 参数不同。world 为与地图匹配的 Gazebo 场景。默认 `rviz=false`；明确传 `true` 时才打开自带导航配置。仓库只附带 office_01 的 YAML/PGM，没有其 `.posegraph/.data`，所以可以 AMCL 导航，不能直接恢复原 SLAM 图结构继续建图。详见 [示例地图说明](../../maps/office_01/README.md)。

初始位姿默认 x=0、y=0、yaw=0，来自本场景已知出生点，作为AMCL初始猜测。可以用 initial_x/initial_y/initial_yaw 设置地图坐标中的初始猜测；RViz 的 **2D Pose Estimate** 可在地图上点击并拖出朝向来重新指定。AMCL利用雷达与地图持续估计位置，不使用Gazebo世界真值。任意未知位置的全局重定位不保证一次成功，重复房间尤其需要合理的初始区域。

## 设置目标、导航和结束

等待 Navigation 2 面板显示节点 active、地图及雷达显示正常。用工具栏 **Nav2 Goal** 在可通行的白色区域按住鼠标拖出箭头，松开后发送位置和朝向。地图中的起点/目标均用map坐标。不要把目标点放在黑色障碍物、灰色未知区域或过窄位置。

也可在新终端发送一个目标，例如走廊中的 (2,0)，朝向0弧度：

```bash
ros2 run smartcar_nav2 navigate.py --x 2.0 --y 0.0 --yaw 0.0
```

脚本显示剩余距离及恢复次数，最终返回 SUCCEEDED / CANCELED / ABORTED。默认180墙钟秒超时会请求取消，可用 --timeout 调整。到达判定使用0.15m位置容差、0.2rad朝向容差，成功后停止运动；此容差是Nav2内部判定，不是实物标定精度。

RViz Navigation 2面板可取消当前目标；CLI可用 Ctrl+C 或另一个终端：

```bash
ros2 run smartcar_nav2 navigate.py --cancel
```

需要立即锁定导航停车时：

```bash
ros2 service call /navigation/stop std_srvs/srv/SetBool '{data: true}'
```

它输出零速度并请求取消导航动作。解除后重新设置目标：

```bash
ros2 service call /navigation/stop std_srvs/srv/SetBool '{data: false}'
```

这是仿真软件停车，不是实车硬件急停。

## 七项能力与数据链

1. 目标设置：RViz Nav2 Goal、NavigateToPose动作接口、navigate.py。
2. 自主定位：AMCL + 已保存地图 + /sim/scan + 里程计TF，初始猜测可重新指定。
3. 全局规划：NavFn A*，依据静态地图、实时障碍层及膨胀层，周期重规划。
4. 障碍物检测与避障：C1二维扫描进入局部/全局ObstacleLayer，路径跟踪时的预测碰撞检查使用矩形车体轮廓。这里只检测占据位置，不识别“人/椅子”等语义类别，也无法检测扫描平面之外的全部障碍。
5. 跟踪控制：Regulated Pure Pursuit控制器，Velocity Smoother平滑，最高前进0.30m/s、转速0.8rad/s；没有让键盘节点和导航抢同一速度入口。
6. 异常恢复：进展检查、控制失败检查，行为树最多4轮恢复，依次清理代价地图、碰撞检查下原地转动、等待、短距离后退。不可达时返回ABORTED；取消返回CANCELED。恢复不是保证任何堵塞都能走通。
7. 到达结束：位置/朝向容差判定，动作返回SUCCEEDED并停车；导航基础节点仍保持运行，等待下一个目标。

```text
Nav2控制/恢复 → /nav/cmd_vel_raw → 速度平滑 → /nav/cmd_vel
  → 导航检查门控 → /nav/safe_cmd_vel → 原cmd_guard → /sim/drive_cmd_vel → Gazebo
```

导航仿真将cmd_guard输入改为专用 `/nav/safe_cmd_vel`，旧键盘 `/sim/cmd_vel` 不会驱动导航模式的小车。手动驾驶请关闭导航，重新启动原smartcar_sim/smartcar_slam入口。

门控采用墙钟检查指令、雷达、里程计、仿真时钟是否过期，要求已收到AMCL定位且map→odom变换未过期；重复的地图/雷达/速度接口会阻止输出，近距离障碍会阻止对应方向的运动。原0.3秒指令保护保留。保护节点、导航门控或关键导航服务器退出时，会关闭整个仿真。桥接强制杀死等情况仍应结束仿真；这些措施不构成实车安全认证。

## 观察与检查

专用 RViz 配置包含地图、雷达、全局路径、预测运动轨迹、代价地图和目标设置工具；使用 `rviz:=true` 加载本包配置。已经打开 RViz 时也可手动加载 `src/smartcar_nav2/rviz/navigation.rviz`。

```bash
ros2 run smartcar_nav2 check_navigation.py --output "$SMARTCAR_WS/verification_runs/nav2_observe_01"
ros2 topic echo /navigation/safety_status
```

检查器只读，检查生命周期、接口和 TF，输出 JSON；通过不等于实际完成导航。原工作空间曾执行选定目标、临时静态障碍绕行、取消、雷达中断和不可达恢复验收；也保留了早期启动失败和 DWB 转向振荡等失败记录。那些完整记录留在原工作空间，没有随本仓库导入，不能当作本次目录移植已通过的证据。

本轮迁入仓库的构建、软件测试及隔离运行结果以 [本轮验收目录](../../../docs/evidence/ros2-integration-20261018/) 和 [仓库验收记录](../../../docs/ACCEPTANCE.md) 为准；本文不预先判定通过。GUI 操作、任意多目标路线、移动障碍、更多门口组合和长期稳定性仍需分别验证。此前实际操作中出现过 `detected collision ahead`、恢复后 `Goal failed`，不能承诺任何目标都可达。

保存的 office_01 地图存在误差；质量、惯量、摩擦和传感器安装参数尚有估算值，没有实物标定。当前启动入口与 `/clock`、`/sim/scan`、`/sim/odom` 及 Gazebo 绑定；将此包加入底层仓库不等于已经打通 STM32 实车导航。

配置基于官方Humble分支：https://github.com/ros-navigation/navigation2/tree/humble 。
