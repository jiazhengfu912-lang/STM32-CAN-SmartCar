# SmartCar 二维 SLAM

在既有 Humble/Fortress 小车上使用 SLAM Toolbox，将 `/sim/scan` 与里程计 TF 组合为 `/map`，新增 `map → odom` 坐标变换。当前不启动自主导航。此包从既有仿真工程导入，原 SLAM 验收完整记录保留在原工作空间；本仓库的目录移植验收以 [本轮证据](../../../docs/evidence/ros2-integration-20261018/) 为准，软件测试与实际运行结果分别记录。

每个终端先在本仓库根目录执行，脚本自动设置工作空间路径：

```bash
source ros2_ws/tools/env.bash
```

默认本机 ROS 域 86；可在 source 前显式指定其他域，但参与同一仿真的终端必须一致。

## 依赖与编译

以下命令从本仓库根目录执行，sudo 密码仅在本机终端输入：

```bash
bash ros2_ws/tools/install_dependencies.bash
bash ros2_ws/tools/build.bash
source ros2_ws/tools/env.bash
bash "$SMARTCAR_WS/tools/test.bash"
```

`nav2_map_server` 在建图模式中仅用于地图保存，不会启动 Nav2 导航。

## 已有仿真时，只接入 SLAM

保持已启动的带雷达仿真，只运行：

```bash
ros2 launch smartcar_slam slam.launch.py
```

它不启动另一个 Gazebo 或 RViz，也不发送行驶指令。开始前确认已有 `/clock`、`/sim/scan`、`/sim/odom`，以及 `odom → base_footprint → base_link → laser_frame`；同一 ROS 域内只运行一个 SLAM 实例。

## 全部关闭后，一起启动仿真和 SLAM

```bash
ros2 launch smartcar_slam mapping.launch.py
```

此入口启用雷达，关闭摄像头，默认不打开 RViz。已有仿真时不要重复使用此入口。然后在另一个已加载环境的终端启动键盘：

如需四房间场景，在停止旧仿真与 SLAM 后用 `ros2 launch smartcar_slam mapping.launch.py world:=office_loop`。默认 `world:=smartcar` 保留原房间；两者都支持同一个 `world` 参数传递到仿真。新场景重新建图，不传旧房间的 `map_file`。具体布局与切换步骤见 `../smartcar_sim/worlds/README.md`。

```bash
ros2 run smartcar_sim keyboard_drive.py
```

先缓慢前进、转向，逐渐从多个位置观察障碍物，最后回到附近起点。`map` 会保留已经观测过的自由区域和障碍物；红色 LaserScan 显示当前观测。

## 手动配置你自己的 RViz

1. `Global Options → Fixed Frame` 改为 `map`。
2. 左下角 `Add → By display type → Map`。
3. 新 Map 的 `Topic` 设为 `/map`，展开 Topic，`Reliability Policy = Reliable`，`Durability Policy = Transient Local`。
4. 原 LaserScan 保持 `/sim/scan`、`Best Effort`、`Volatile`、`XYZ`、`FlatColor` 红色。
5. `Panels → Add New Panel → rviz_common → Views` 添加 Views；Orbit 的 Distance 设10、Pitch设1.45、Focal Point设0/0/0，Target Frame保持`<Fixed Frame>`。
6. Map 应显示白色自由区域、黑色障碍物、灰色未知区域；红点应大致贴合黑色边界。用 File → Save Config As 保存你自己的 RViz 配置。

## 检查与真实数据记录

默认只读，不移动小车：

```bash
ros2 run smartcar_slam check_slam.py --output "$SMARTCAR_WS/verification_runs/slam_observe_01"
```

输出目录必须尚不存在。检查真实 OccupancyGrid、有效占据/自由单元、仿真时钟、scan/odom、完整 TF 链、扫描时刻 TF 和地图更新；保存 CSV、JSON、原始 NPZ 与 PNG/SVG。

`--drive-square` 会发送运动指令，仅供关闭键盘后，在本工程原始场地起点及其右前方空旷区域进行0.6m顺时针方形验收，不是避障或探索算法。使用前确认场地空旷。任何失败或中断保留 FAIL 与原始数据，结束发送零速度，保护节点保留0.3秒超时停车。

## 保存地图和可继续建图的图结构

```bash
ros2 run smartcar_slam save_map.py --output-dir "$SMARTCAR_WS/maps/room_01"
```

每次使用新目录；不会覆盖旧地图。成功应得到：

- `map.pgm`、`map.yaml`：栅格图与分辨率/原点等元数据。
- `map.posegraph`、`map.data`：SLAM 图结构与扫描数据，用于之后继续建图。
- `save_result.json`：实际服务结果与文件验证。

## 重新加载并继续建图

先停止旧 SLAM；让机器人回到此次地图创建时的起点附近、朝向一致。已有仿真时：

```bash
ros2 launch smartcar_slam slam.launch.py map_file:="$SMARTCAR_WS/maps/room_01/map"
```

完全重启原始仿真并在原始起点创建的地图，可用：

```bash
ros2 launch smartcar_slam mapping.launch.py map_file:="$SMARTCAR_WS/maps/room_01/map"
```

`map_file` 是无扩展名的绝对前缀，必须同时有 `.posegraph` 和 `.data`，不能把 `.yaml` 传给它。**仓库附带的 office_01 仅有 YAML/PGM 导航栅格图，没有导入原 `.posegraph/.data`。**上述恢复示例适用于你在本工作空间新保存的完整地图，或你自行复制进来的图结构；不能把默认 office_01 导航地图当作可继续建图的存档。这里采用 `map_start_at_dock=true`，表示从保存图的第一节点附近继续；任意地点重定位不是本次接口。

## 参数与边界

`config/slam.yaml`：`scan_topic=/sim/scan`、`base_frame=base_footprint`、`odom_frame=odom`、`map_frame=map`、`use_sim_time=true`、分辨率0.05m、地图更新间隔2秒仿真时间、建图有效距离0.06～12m（传感器自身仍为0.05～12m）；小车每平移0.05m或转动0.1rad即可接受新扫描，支持回环检测。参数基于 Humble SLAM Toolbox 官方异步配置调整。

SLAM Toolbox通过TF获取里程计位姿，不直接订阅 `/sim/odom`。`map → odom`由SLAM发布；`odom → base_footprint`沿用Gazebo差速插件；后续车体/传感器TF由robot_state_publisher提供。

软件渲染可能使仿真慢于墙钟；地图不会按墙钟 2 秒固定刷新。历史运行包含失败与修复记录，本轮结果须查看独立证据。功能验收不代表实物验证、长期稳定性、导航或精度标定完成；当前包使用模拟传感器和仿真时间，尚不代表已接通实车。

来源：https://github.com/SteveMacenski/slam_toolbox/tree/humble

## 使用已保存地图自主导航

工作空间新增 `smartcar_nav2` 包，详见 `../smartcar_nav2/README.md`。先结束本包启动的仿真/SLAM与旧键盘，再用 `ros2 launch smartcar_nav2 navigation.launch.py world:=office_loop rviz:=true`，默认使用随导航包安装的 office_01 栅格图。导航时由AMCL发布map→odom，不与本SLAM节点同时运行；原建图入口和图结构恢复功能保留。
