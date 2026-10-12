# v131 ICM-42688-P 离线验证证据

2026-10-10。范围：生产C驱动+BSP在HAL替身中的运行、主机回归及STM32构建。所有CSV与图均为软件注入/仿真，不是实物采样。未连接、烧录或操作硬件，未下载软件。

- `before.json`：修改前版本/HEAD及源文件SHA256；`before-tests*.log`：修改前完整回归。
- `datasheet-facts.json`：本地手册标识、芯片换算/配置事实与资料边界。
- `tests*.log/json`：修改后主机回归；`build.log/json`：App/Bootloader构建。
- `attempt-01.log`：首次测试夹具编译失败记录；`attempt-02.log`：修复后专用测试。
- `driver-trace.csv`：全部逐点虚拟时基运行记录，包括失败/缺测和回绕场景；相同样本保持值不代表新测量。
- `driver-trace.png/svg`、`driver-failures.png/svg`：无平滑逐点图，缺测断线，标题标注v131/P/I/算法及仅主机注入。
- `plot_driver_trace.py`：使用已有`out/plot_env/Scripts/python.exe`可从原CSV复绘，不访问设备。
- `coordinate-simulation.csv`：生产导航理想模型，逐字节等于已有v128归档，不能作为实车精度证据。
- `verification.json`：参数前后、未改文件校验、构建产物哈希及硬件待验证边界。
- `SHA256.json`：目录内原样文件及相关最终源码校验；不含自身。

波形及结论已直接嵌入`../../ACCEPTANCE.md`本轮正文。接线与驱动说明见`../../ICM42688_UPGRADE.md`。
