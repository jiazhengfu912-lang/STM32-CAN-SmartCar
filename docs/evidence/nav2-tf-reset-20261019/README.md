# Nav2 TF 到达误判与 Reset 卡住修复 · 2026-10-19

本轮仅仓库 ROS 软件和隔离仿真，无实车、无烧录；实车桥尚未实现。当前用户 Gazebo/RViz 会话未发送目标、未清图、未重启。Git 修改仅留本地，未提交、推送或合并。

## 依据与修复

现场日志在17:22:54收到从(-0.47,2.94)到(3.02,-2.25)的目标，3.752秒后报告成功；前一刻`map→odom`变换失败，数据23.076仿真秒、最新变换22.3秒，落后0.776秒，超过0.5秒容差。17:24:40开始Reset，定位重新激活，但导航清理停在controller，查询planner/bt/behavior/smoother仍为unconfigured。现场快照与日志见[user_before](user_before/analysis.json)。后采集的位置仍距目标约6m，不能把它当作历史连续轨迹；原始任务没有录完整TF/轨迹bag。

本机Nav2 1.1.20、tf2/tf2_ros 0.25.23，详见[环境](environment_scope.json)。固定车辆位姿为零、目标5m外、路径时间比最新TF超前3秒的合成ROS工况，真实系统控制器立即返回SUCCEEDED(4)，输出过0.30m/s；这复现相同失效路径，时间偏移量与现场0.776秒不同。原代码到达检查忽略`transformPose()`失败，继续使用未成功转换的位姿。原始上游源码保存在本目录controller_server.cpp，其版权为Intel等、Apache-2.0；来源[Navigation2 1.1.20](https://github.com/ros-navigation/navigation2/tree/1.1.20)。

同一工况Reset清理8秒无响应，GDB只附加本验证器自己创建的控制器：[原始线程堆栈](baseline/cleanup_backtrace.txt)。一线程在`Buffer::waitForTransform→BufferCore::addTransformableRequest`等待锁，另一线程在`BufferCore::testTransformableRequests`的回调等反向锁；清理线程堵在障碍过滤器析构。和上游[geometry2 #992](https://github.com/ros2/geometry2/issues/992)描述的锁序反转一致，死锁可以早于Reset发生，并冻结TF。

修复随smartcar_nav2构建两个本地可执行文件，不改`/opt/ros`系统库：

- safe_controller_server：使用原Nav2插件和参数初始化，直接绑定带TF检查的FollowPath循环。发速度前验证目标变换，到达检查再次验证；失败停车并ABORTED，交给上层有限恢复。取消、停用和等待代价图时均能退出，不自动续跑旧动作。
- safe_planner_server与控制器共用SafeCostmap：TF监听、MessageFilter和其超时定时器使用同一个代价图专用互斥回调组，避免跨线程锁序反转；清理时先停止该执行线程，再销毁过滤器和TF资源。保留原扫描过滤、marking/clearing和障碍记忆。
- 启动器保持标准controller_server/planner_server节点名及动作接口，替换对应可执行文件；NavFn、RPP、地图轮廓、所有到达/TF容差、速度和平滑/保护配置未修改。

代码快照见[source_snapshot](source_snapshot/)。控制器初始化和执行循环基于Apache-2.0许可的Nav2 1.1.20，保留版权；升级ROS依赖后需重新构建回归。此次串行化针对本包两种代价图，不声称修复所有应用中系统tf2的问题。

## 所有尝试与结果

| 记录 | 实际结果 |
|---|---|
| baseline | FAIL：错误TF却成功；cleanup 8秒超时；堆栈保留，子进程最后需要SIGKILL |
| build.log | FAIL：新C++文件缺少异常类型头文件，完整编译错误保留 |
| after_01 | FAIL：TF故障正确ABORTED且零输出，但第二次配置超时；没有把部分通过算整轮通过 |
| after_02 | PASS：5次清理/配置/激活、有效目标和取消；是中间实现，最终结论以下列最终版本为准 |
| build_final.log | PASS：三包构建；build_02/03日志也原样保存 |
| after_final | PASS：错误TF返回6且零输出；5次Reset/Startup；显式新目标成功；有效远目标持续执行并可取消；执行中Reset终止旧目标，重新激活不续跑 |
| gazebo_detour_reset_01 | PASS：两轮完整管理器Reset/Startup后9节点active；一次目标，中途插箱体，真实绕行、到达、持续停车；两图清空调用0 |
| gazebo_two_doors_01 | PASS：最终二进制，一轮完整Reset/Startup后双门封堵，两个障碍均被确认并保留到ABORTED；持续停车，两图清空调用0 |
| ros_tests.log | PASS：40个pytest，colcon47项含7个CTest包装；0错误/失败/跳过 |

首次封装通过原基类初始化后立即替换后台动作服务，重复配置出现超时；最终直接创建一次安全动作服务。没有通过延迟或重试掩盖该失败。绕行样本在最终构建前运行，随后仅调整构造时释放旧代价图的顺序，消除短暂重名日志告警；最终二进制由after_final、colcon和双门工况回归。各次快照和二进制哈希按当轮报告保留，不混称所有样本为同一构建。

## 图与数据的解释

STM32源码v131：P右反0.45→0.45、其它0.30→0.30，I稳态0.010→0.010、启动0.030→0.030，D0，10ms按电机轴rpm误差累计。本轮PI、起步、IMU和相对导航算法未改、未运行。上层改变目标TF错误处理、动作循环与回调调度；NavFn/RPP算法及参数保持。

![固定原点、远目标与同样故障输入下，真实控制器输出及动作结果的前后比较；仅合成ROS](comparison/tf_fault.png)

前后输入固定原点、目标5m、未来路径时间偏移3秒；点来自真实控制器命令与动作响应，未驱动任何车辆。散点保留全部故障窗口命令，不把无命令区间补成波形，也不把响应时间差当性能保证。完整输入生成程序、参数及速度CSV在baseline/after_final中；原baseline验证器较早版本未保存源快照，使用同一故障设置，最终完整验证器在source_snapshot中。

![清理响应时间前后对照；对数纵轴，超时是未完成下界](comparison/lifecycle.png)

基线第1次清理未完成，8秒是观测超时，不是完成耗时；最终版本第1—5次为空闲重复清理，第6次在有效远目标执行中清理。所有点源于events.jsonl，派生表见[CSV](comparison/lifecycle_derived.csv)。没有平滑或删异常。

![完整Gazebo：两轮Reset后中途插障碍，真实速度、门控、到达停车](gazebo_detour_reset_01/velocity.png)

办公室office_loop，目标map(4.5,0,0)，车x约0.45m时插入中心(2,0)、0.4×0.5×0.6m静态箱体。一次目标，实际绕经相邻房间，最终世界位姿(4.4716,0.0773,0.099rad)。到达后43条新鲜odom样本、约1.459秒连续窗口满足|vx|<0.02m/s、|wz|<0.03rad/s、位移<0.02m且安全命令为零；最大角速度0.02542rad/s。前两轮Reset后无目标时也验证零输出和静止。[结果](gazebo_detour_reset_01/result.json)、[轨迹](gazebo_detour_reset_01/trajectory.png)。

![完整Gazebo：双门障碍保留到不可达失败，没有清图丢失](gazebo_two_doors_01/obstacle_memory.png)

双门模型和前轮验收同源：起点(-2.99,1.91,0)，目标(1.03,-2.92,0)，半径0.5m球中心(-2.9082,0.2833)，1m方块中心(3.0925,0.8227)，均静态且启动前存在。42.54墙钟秒首次两处均有致命格，此后到ABORTED都未归零。计数为中心±0.7m范围、静态地图原本空闲的致命格，不是物体分割或物理尺寸测量。

![同轮双门封堵失败后的真实速度与持续停车](gazebo_two_doors_01/velocity.png)

ABORTED(6)，37条新鲜样本约1.443秒满足相同停车判据，最大线速度0.01858m/s、安全命令为零。全程未重发目标、未清空全局/局部图。[结果](gazebo_two_doors_01/result.json)、[记忆分析](gazebo_two_doors_01/memory_analysis.json)。这是不可达的预期失败停车，不应显示到达成功。

全部图点来自本轮实收ROS/Gazebo记录；无平滑、插值或删异常，长缺口断线。没有实物左右轮速、电机输出、IMU或硬件故障计数，本轮不构造这些字段。Reset属于用户显式清理，会重建代价图，不保证跨Reset保存障碍；正常自动恢复仍不清图。

## 复核与使用

所有自建测试进程组已退出，Gazebo两轮仅SIGINT即退出。当前用户旧窗口不会热更新可执行文件；需在其原启动终端Ctrl+C，再按原场景、定位参数启动，已有独立RViz可用rviz:=false。不要重复启动第二套。此后用户仍可自己在RViz发Nav2 Goal、Gazebo放障碍；本轮没有替用户操作当前窗口。

原始CSV、结果JSON、全量扫描/TF/代价图/路径、服务响应与日志均保留。[summary.json](summary.json)汇总全部轮次；[SHA256.json](SHA256.json)为整个归档校验。较大JSONL使用无损gzip，原始字节哈希与压缩哈希见[compression.json](compression.json)；原未压缩文件仍保存在本机忽略目录，JSON中的原文件名对应解压后的字节。绘图器可直接读取gzip。

本轮仅上述工况通过，不覆盖任意GUI路线、多目标队列、移动障碍、所有地图、长时间压力或实车。ROS实车桥、硬件保护、落地精度均待实现/验收。
