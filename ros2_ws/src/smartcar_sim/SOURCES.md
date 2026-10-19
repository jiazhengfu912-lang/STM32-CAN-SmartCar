# 官方依据与版本边界

2026-10-16检索。目标组合为Ubuntu22.04、ROS2 Humble、Gazebo Fortress。不同Gazebo主版本的插件名和消息命名空间不可直接混用。

- Gazebo官方ROS配套表：https://gazebosim.org/docs/fortress/ros_installation/
- Fortress差速驱动参考：https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1DiffDrive.html
- Fortress关节状态发布：https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1JointStatePublisher.html
- Fortress传感器：https://gazebosim.org/docs/fortress/sensors/
- Humble官方ROS/Gazebo桥接源码及消息表：https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_bridge
- C1官方参数：https://www.slamtec.com/en/c1/spec
- C1实物ROS2驱动：https://github.com/Slamtec/sllidar_ros2
- ROS坐标约定：https://www.ros.org/reps/rep-0103.html
- Gazebo虚拟机渲染问题：https://gazebosim.org/docs/fortress/troubleshooting/

机械来源：本项目cad/ros_car_v7/assembly_placements.json及其引用的V6、降压盒V6打印STL。复制的原始网格在cad_source/parts，来源哈希在cad_source/placements.json；仿真转换结果在mesh_manifest.json。

用户确认：主动轮65x27mm、轮距沿用模型、雷达C1、摄像头1080P/2.8mm镜头、Ubuntu22.04/Humble。淘宝短链未能读取，不宣称核实其传感器尺寸、真实视场、帧率或USB供电。

C1官方数据手册下载在本次读取中被403/访问限制阻挡，因此外壳和扫描面高度继续作为明示估算项，不将推荐产品网页的扫描规格误当成机械安装图。本次不修改原生SolidWorks文件。


## 本机 GUI 配置来源

`config/gazebo_gui.config` 由已安装的 Gazebo Fortress 6.18.0 默认配置 `/usr/share/ignition/ignition-gazebo6/gui/gui.config` 复制，仅调整窗口尺寸和小车初始观察视角；不修改用户的全局 Gazebo 配置。
