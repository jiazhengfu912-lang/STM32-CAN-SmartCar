# office_01 导航示例地图

这份地图来自既有 SmartCar 仿真工作空间中用户在 `office_loop` 四房间场景构建并保存的二维占据栅格图。它是 SLAM 的观测结果，可能存在边界厚度、局部错位和未知区域，不是从 Gazebo 几何生成的无误差真值地图。

| 文件/参数 | 内容 |
|---|---|
| `map.pgm` | 239 × 200 像素占据栅格 |
| `map.yaml` | 图片相对路径、分辨率、原点和占据阈值 |
| 分辨率 | 0.05 m/像素 |
| 原点 | `[-5.99, -4.98, 0]`，由保存结果保留 |
| 匹配场景 | `smartcar_sim/worlds/office_loop.sdf` |

`smartcar_nav2` 构建时将 YAML/PGM 安装到包的 `share/smartcar_nav2/maps/office_01`。因此导航默认启动不依赖旧开发机路径：

```bash
# 在仓库根目录，完成构建后
source ros2_ws/tools/env.bash
ros2 launch smartcar_nav2 navigation.launch.py world:=office_loop rviz:=true
```

其他地图可用 `map:=/你的地图目录/map.yaml` 覆盖，必须配套对应场景。

**本目录没有导入原来的 `map.posegraph` 和 `map.data`。**YAML/PGM 足够让 map_server/AMCL 在这张地图上定位导航，但不能恢复 SLAM Toolbox 的原图结构。要继续原 SLAM 建图，需自行从原保存目录完整复制这两份同前缀文件；或者重新建图并通过 `save_map.py` 保存新的完整结果。不要把 `.yaml` 传给 SLAM 的 `map_file` 参数。

重新保存地图时使用新的目录，例如 `$SMARTCAR_WS/maps/my_office_01`，避免覆盖此示例。地图与仿真参数未经过实车场地测绘或物理精度标定。
