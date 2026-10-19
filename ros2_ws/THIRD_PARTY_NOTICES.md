# ROS 2 工作空间来源与许可说明

本目录整合既有 SmartCar ROS 2 Humble / Gazebo Fortress 仿真工程中的 `smartcar_sim`、`smartcar_slam`、`smartcar_nav2`。三个包的 `package.xml` 均声明 `Apache-2.0`；各包的 LICENSE 和下列来源说明应随源码保留。本说明限定于 ROS 子目录，不改变仓库原 STM32 HAL、CMSIS、ESP32 或其他第三方文件的许可。

## Navigation2

- 项目：[Navigation2，Humble 分支](https://github.com/ros-navigation/navigation2/tree/humble)
- 许可：Apache License 2.0。
- 本仓库 `smartcar_nav2/config/nav2.yaml` 根据 Humble 默认参数配置调整；`config/navigate_recovery.xml` 根据其重规划/恢复行为树调整；`rviz/navigation.rviz` 根据 `nav2_bringup/rviz/nav2_default_view.rviz` 裁剪。
- SmartCar 的改动包括扫描/里程计话题、车体轮廓、速度/容差、控制器选择、恢复动作、显示项与启动组织。保留这些文件中的来源注释及 Apache 许可文本。
- Nav2 算法本体通过 ROS 软件包安装，未将其源码整体复制进本仓库。

## SLAM Toolbox

- 项目：[SLAM Toolbox，Humble 分支](https://github.com/SteveMacenski/slam_toolbox/tree/humble)
- `smartcar_slam/config/slam.yaml` 以官方异步配置为依据调整坐标系、扫描话题、距离、更新阈值和仿真时钟。
- SLAM Toolbox 算法通过发行版依赖安装，其上游源码和各自许可保持独立；本仓库不把对上游包的依赖重新授权为本项目代码。

## Gazebo Fortress 与 ROS 桥接

- 项目：[Gazebo Sim，Fortress / ign-gazebo6](https://github.com/gazebosim/gz-sim/tree/ign-gazebo6)，[ros_gz，Humble 分支](https://github.com/gazebosim/ros_gz/tree/humble)。
- `smartcar_sim/config/gazebo_gui.config` 源自已安装 Fortress 的 `/usr/share/ignition/ignition-gazebo6/gui/gui.config`，调整了窗口尺寸和初始观察视角。Gazebo Sim 使用 Apache License 2.0。
- SDF 使用 Fortress 的差速驱动、关节状态、传感器、场景广播和交互插件；插件和 ROS 桥接由系统包提供。

## 机械网格、参数与地图

- 机械来源为本项目 V7 装配布局及其引用的 V6、降压盒 V6 打印网格。`smartcar_sim/cad_source/parts` 保存实际使用的原始网格；`placements.json` 和 `mesh_manifest.json` 保存对应变换及 SHA256。具体依据见 [SOURCES.md](src/smartcar_sim/SOURCES.md)。
- `smartcar_sim/models/smartcar/meshes` 为上述网格的仿真坐标/单位转换结果，没有改写原生 CAD 工程。
- RPLIDAR C1 名义扫描规格来自厂商公开产品资料；机械包络、扫描面高度及部分物理参数仍明确标为估算。引用规格不表示厂商对模型精度的认可，也不提供实物性能保证。
- office_01 的 YAML/PGM 是用户在项目仿真场景中保存的地图。原 `.posegraph/.data` 没有随此仓库导入，详见 [地图说明](maps/office_01/README.md)。

## 历史记录与本次验收

包内 `smartcar_sim/verification/` 与 `SOURCES.md` 保留来源工程的历史记录，其中开发机路径和原检测时间不是新仓库的安装要求。原完整运行证据保留在原工作空间。

本轮迁移构建、测试及隔离仿真结论应查看 [本轮证据目录](../docs/evidence/ros2-integration-20261018/) 和 [ACCEPTANCE.md](../docs/ACCEPTANCE.md)。软件配置归档、历史仿真成功与真实硬件验证是不同证据，不能相互替代。
