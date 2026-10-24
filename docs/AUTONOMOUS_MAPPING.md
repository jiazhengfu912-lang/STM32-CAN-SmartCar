# 陌生环境自主探索建图（当前仅仿真）

启动后不再用键盘遥控或逐个点击导航目标：SLAM Toolbox从第一帧雷达建立在线地图，前沿探索器寻找“已知自由区与未知区的交界”，在已知自由区内选择有安全余量的观察点，由NavFn A*和RPP负责规划、跟踪与避障，然后继续观察和选择下一个目标。

实车桥尚未实现，这个入口启动Gazebo；不能直接拿来驱动实车。实车还需接入真实扫描、轮式里程计、TF、底盘命令与独立停止保护并完成验收。

要在全新场景亲自验收，使用`world:=discovery_lab`，详见[新场景启动和验收步骤](DISCOVERY_LAB_ACCEPTANCE.md)。该10×8m非对称实验室在本轮重新设计，不加载旧办公室地图。

当前探索器采用信息收益/预计耗时评分、到达切线朝向、附近区域补盲和地图更新驱动的目标交接；自主模式另有SLAM匹配覆盖配置。实现与同场景比较方法见[探索效率优化](EXPLORATION_SPEED.md)。运动上限和失联停车保持。

## 启动

先结束原有导航/建图启动，避免同域存在两套模拟时钟、雷达和map→odom。保留现有用户窗口的规则适用于助手操作；用户自己启动本流程时可以选择需要的GUI。

从仓库根目录：

```bash
bash ros2_ws/tools/build.bash
source ros2_ws/tools/env.bash
ros2 launch smartcar_nav2 exploration.launch.py \
  world:=office_loop rviz:=true \
  output_dir:="$HOME/smartcar_maps/office_auto_01"
```

输出目录必须不存在，使用不含空格的ASCII路径；不会覆盖旧地图。默认未指定目录时，在`~/smartcar_maps/explore_日期_时间`生成独立会话。无需事先加载office_01，无需2D Pose Estimate，无需键盘驾驶或Nav2 Goal。测试代码不会把Gazebo场景几何或预先保存的地图提供给探索器。

默认Gazebo在后台运行，RViz显示实时`/map`、机器人和当前导航路径，减少虚拟机图形负载。需要查看Gazebo场景窗口时可追加`headless:=false`。建图专用RViz界面不依赖AMCL面板，不提供人工目标工具。等待建图、TF和导航生命周期就绪后，探索器自行发第一个目标。屏幕日志及`/exploration/status`包含状态与保存路径：

```bash
ros2 topic echo /exploration/status
```

`WAITING`等待输入/导航，`EXPLORING`自主移动，`PAUSED`暂停，`STOPPING`验证停车，`SAVING`保存，`DONE`停止并已保存，`ERROR`异常需查看日志。状态`DONE`不必然表示看到了每个角落，还要查看终止原因和`session.json`中的`complete`。

## 暂停、继续和结束

在另一个已加载同一环境的终端：

```bash
# 暂停：取消当前目标并锁存停车
ros2 service call /exploration/pause std_srvs/srv/SetBool '{data: true}'
# 继续：显式解除停车，等待新地图后重新选目标
ros2 service call /exploration/pause std_srvs/srv/SetBool '{data: false}'
# 提前结束：先验证停车，再保存已经探索到的部分地图
ros2 service call /exploration/finish std_srvs/srv/Trigger '{}'
```

已有`/navigation/stop`停车同样会使探索器暂停，恢复心跳不会自行续行。希望改为人工操作时先暂停；不要让人工目标与自主任务同时抢控制。任务结束后不能直接“继续”旧会话，重新探索使用新目录。

结束服务返回只表示已接受结束请求，**请等待`DONE`或日志`map_saved`再关闭启动**。直接Ctrl+C会停止进程并保留中断记录，但不保证有完整地图四件套；保存必须在SLAM服务仍运行时执行。

## 保存内容和完成条件

- `map.yaml`与`map.pgm`供后续已知地图AMCL导航使用，由独立地图保存服务写入；图结构由SLAM序列化。
- 自动导出使用`free_thresh: 0.196`保留PGM未知灰度205，避免重载时误判为自由区。旧阈值0.25的历史地图需单独核验；不因保存成功就认定可安全重载。
- `map.posegraph`与`map.data`保存SLAM图结构，后续接续建图需要两者，不能只给PGM。
- `session.json`保存终止原因、目标和结果、停车证据及范围；`events.jsonl`、`odom.csv`、`safe_cmd_vel.csv`、`safety.csv`、`map_updates.csv`保存过程。
- `map_initial.npz`与`map_final.npz`保留原始占据值，便于复核真实地图增长。

只有满足长度阈值的前沿持续消失、多个地图更新均确认时，终止原因为`NO_FRONTIERS`且`complete=true`。这表示本算法未再检测到可探索前沿，不等于所有空间都可见或测绘精度达标。

还存在前沿但已知自由区没有安全路径、接近点全部失败或已尝试时，会保存`NO_REACHABLE_FRONTIERS_PARTIAL`；预算耗尽、连续导航失败或用户提前结束同样保存部分地图并写明原因；输入中断先暂停并锁存停车，数据恢复后仍需显式继续，也可主动结束保存部分图，不冒充完全建图。默认预算900仿真秒，运行态另检查3600秒墙钟上限，可用`max_sim_seconds:=...`指定有限正数。故障暂停/等待显式恢复时不会自动走到超时结束流程，应由用户继续或主动结束保存。

## 保护和实现边界

- 建图模式不启动AMCL和map_server，SLAM独占地图及map→odom；原已知地图导航入口默认仍为AMCL/RPP。支持导航与SLAM配合的依据见[Nav2官方说明](https://github.com/ros-navigation/docs.nav2.org/blob/rolling/docs/tutorials/general_tutorials/navigation2_with_slam/navigation2_with_slam.md)。本项目使用本机Humble接口，非照搬新发行版启动参数。
- 建图门控使用唯一SLAM地图源、持续新鲜地图和有效map→odom判断定位就绪，避免漏收静止时的一次性`/pose`而一直等待；TF原有时间窗口不变，时间回退后等待新地图，仍单独检查odom→base_footprint。AMCL模式继续使用原定位订阅。
- 探索器只调用导航动作，不发布驾驶速度；速度平滑、导航门控、原底层超时保护继续生效。建图专用前进巡航0.20m/s、最终角速度±0.35rad/s、最大前进加速度0.25m/s²和角加速度0.5rad/s²，用较慢运动减少轮滑和扫描匹配退化。原门控前进0.30m/s、后退0.15m/s、角速度0.8rad/s及近障/扫描/里程计/时钟/TF阈值不放宽；已知地图导航默认不变。
- 探索目标至少保留0.40m中心安全圆，圆内必须已知自由；连接路径采用与现有Nav2一致的0.37m通行圆（0.35m+0.02m padding）。未知和障碍都不能被路径搜索穿越。前沿最小长度0.8m，长前沿检查多个代表点，避免只检查质心就漏掉可达观察点。Nav2仍独立检查实际代价图，不能以探索评分代替碰撞检查。
- 任务心跳使用单调墙钟；探索器卡住或失联后，门控停止、取消旧导航并锁存。恢复进程/心跳不自动解除停车，必须显式继续。
- 受阻目标记忆防止在同一点无限重试；已有不自动清图、最多4轮的导航恢复保持。正常雷达射线清障保留。
- 2D雷达存在遮挡；封闭房间、过窄通道、玻璃/反光、楼梯、低于扫描平面的障碍及SLAM漂移都超出“自动探索就一定全覆盖”的保证。地图面积增长不代表地图几何准确，仍需独立检查质量。

## 使用新建地图导航

先等到`DONE`，再退出本次探索启动。重新启动仿真会回到出生点；使用本次输出的新地图，而非office_01：

```bash
ros2 launch smartcar_nav2 navigation.launch.py world:=office_loop rviz:=true \
  map:="$HOME/smartcar_maps/office_auto_01/map.yaml"
```

保持场景与建图时一致；本入口每次从空白地图开始，尚未加入跨启动的自主探索续接。实际测试、失败与原始波形见[本轮验收证据](evidence/autonomous-mapping-20261021/README.md)。
