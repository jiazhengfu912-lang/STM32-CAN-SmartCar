# discovery_lab：用户自主建图验收

这是2026-10-21新建的10×8m非对称实验室场景，墙体、门洞和障碍布局此前未在本项目出现。包括西侧实验室/储物区、中央通道、东侧实验室/设备区，以及工作台、箱体和圆柱。出生点为(0,0)，朝向+X。

自主探索的目标策略及专用SLAM配置已纳入[效率优化说明](EXPLORATION_SPEED.md)；验收时同时观察覆盖进度、路线和地图是否重影，不能只看小车是否快速移动。

![新场景布局，SDF几何参考，非SLAM地图或运行截图](evidence/discovery-lab-20261021/layout.png)

上图只供你对照验收，不被探索器、SLAM或规划器加载。`world:=discovery_lab`只选择物理环境，启动后根据雷达从空白状态新建二维地图，没有这个场景的预存YAML/PGM，也不加载旧办公室地图。当前仅支持Gazebo仿真，实车桥尚未实现。

## 1. 启动一次全新验收

先在原启动终端Ctrl+C结束旧导航/建图和旧Gazebo/RViz，避免重复时钟、扫描和map→odom。不要另外启动键盘驾驶、AMCL或第二套SLAM。

打开终端，复制：

```bash
cd /home/jiazhengfu/Projects/STM32-CAN-SmartCar
source ros2_ws/tools/env.bash
export SMARTCAR_ACCEPT_MAP="$HOME/smartcar_maps/discovery_lab_$(date +%Y%m%d_%H%M%S)"
echo "$SMARTCAR_ACCEPT_MAP"
ros2 launch smartcar_nav2 exploration.launch.py \
  world:=discovery_lab rviz:=true headless:=false \
  max_sim_seconds:=1800 output_dir:="$SMARTCAR_ACCEPT_MAP"
```

本机已构建。以后重新拉取源码或改场景，先在source之前运行`bash ros2_ws/tools/build.bash`。输出目录必须全新；时间戳用于避免覆盖以前的图。

这会打开Gazebo与RViz。**不用2D Pose Estimate、不点Nav2 Goal、不启动键盘控制。** 等待SLAM/导航就绪，小车会自行开始探索。Gazebo观察实际场景，RViz观察实时地图和自动路径；视野太近时滚轮缩小，RViz固定坐标系为`map`。初始只看到雷达当前可见区域是正常的。

虚拟机同时运行两个图形窗口可能较慢。如持续出现时钟/扫描超时，结束本次启动，再把`headless:=false`改为`headless:=true`重新验收；这只关闭Gazebo场景窗口，物理和雷达仍运行，RViz保留。

## 2. 观察任务状态

另开终端，使用相同环境：

```bash
cd /home/jiazhengfu/Projects/STM32-CAN-SmartCar
source ros2_ws/tools/env.bash
ros2 topic echo /exploration/status
```

`WAITING`表示等待输入/就绪，`EXPLORING`为自主探索，`PAUSED`为已暂停，`STOPPING`正在验证停车，`SAVING`正在保存，`DONE`为已结束且已保存；`ERROR`需要查看日志。状态里有目标数、失败数、终止原因及输出目录。

预算为1800仿真秒，不是固定30分钟墙钟；仿真变慢会增加实际耗时，任务另有3600秒墙钟总上限。不要只看“车不动了”就认定完成，也不要在`SAVING`时关进程。

## 3. 判断是否完成

| 检查项 | 你应看到的结果 |
|---|---|
| 自主性 | 不输入人工目标或速度，小车自行选择多个观察点、通过门洞，地图随移动扩展 |
| 布局 | 对照上方场景图，西侧、中央、东侧可达区域被观察，门洞和主要障碍位置合理；无明显重影、双墙、房间扭曲 |
| 未知区 | 可达房间不能仍有大片灰色而被忽略；箱体背后和墙内的小片未知需区分遮挡，不能把白色面积直接当覆盖率 |
| 自动完成 | 最终状态`DONE`，原因`NO_FRONTIERS`；`session.json`中`complete=true`且`stop_verified=true` |
| 保存与停车 | 小车持续停稳，输出目录有非空`map.yaml`、`map.pgm`、`map.posegraph`、`map.data` |

**`DONE`本身只表示已结束并保存。** `TIME_LIMIT_PARTIAL`、`NO_REACHABLE_FRONTIERS_PARTIAL`、`USER_FINISHED_PARTIAL`或其他部分结果，都不能算这次“全图自动构建完成”。`NO_FRONTIERS`也仅表示未再检测到达到算法阈值的前沿，还须结合布局和未知区人工核对。

看到`DONE`后，回到第一个终端Ctrl+C退出。该终端中的`SMARTCAR_ACCEPT_MAP`仍保留，可查看：

```bash
python3 - "$SMARTCAR_ACCEPT_MAP" <<'PY'
import json
import sys
from pathlib import Path
p = Path(sys.argv[1])
s = json.loads((p/'session.json').read_text())
print({k: s.get(k) for k in ['state', 'reason', 'complete', 'stop_verified', 'prior_map_loaded']})
for name in ['map.yaml', 'map.pgm', 'map.posegraph', 'map.data']:
    f = p/name
    print(name, f.stat().st_size if f.exists() else 'MISSING')
print('记录目录:', p)
PY
```

保留整个输出目录，包括`events.jsonl`、CSV、地图与`session.json`。若未完成，把终止原因和目录路径告诉我，可定位受阻区域或失败目标。不要删掉失败记录后只保留最好的一轮。

## 4. 暂停与提前结束

在第二个已source的终端执行：

```bash
# 暂停并停车
ros2 service call /exploration/pause std_srvs/srv/SetBool '{data: true}'
# 显式继续
ros2 service call /exploration/pause std_srvs/srv/SetBool '{data: false}'
# 提前结束，停车后保存部分地图；等待DONE再关闭
ros2 service call /exploration/finish std_srvs/srv/Trigger '{}'
```

不要用直接Ctrl+C代替正常保存。故障/失联暂停后不会因心跳恢复自动移动，需要显式继续。

## 5. 可选：使用自己新建的图导航

完成保存并退出探索启动后，在第一个终端重新运行：

```bash
ros2 launch smartcar_nav2 navigation.launch.py \
  world:=discovery_lab rviz:=true headless:=false \
  map:="$SMARTCAR_ACCEPT_MAP/map.yaml"
```

此时进入已知地图AMCL导航，才可以在RViz设置Nav2 Goal。保持同一个`discovery_lab`场景，不能使用旧`office_01`地图。

本轮助手只做场景安装、静态连通性和短时启动/运动/保存检查，不宣称该新场景已经全图探索完成；完整检查结果由你的这轮验收给出。[助手检查记录](evidence/discovery-lab-20261021/README.md)
