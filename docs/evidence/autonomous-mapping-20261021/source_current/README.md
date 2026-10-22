# STM32-CAN-SmartCar

本仓库同时保存 **STM32/ESP32 底层控制** 和 **ROS 2 Humble 上层仿真、二维 SLAM、Nav2 导航**。
新增的 [`ros2_ws/`](ros2_ws/README.md) 可在 Ubuntu 22.04 / Gazebo Fortress 中编译运行，
支持键盘驾驶、C1 雷达仿真、[自主探索建图](docs/AUTONOMOUS_MAPPING.md)、建图保存、AMCL 定位、单点/多点导航、规划、碰撞检查与恢复。
ROS 2 当前仍通过 Gazebo 驱动车辆；**尚未接通实车 ROS 底盘接口**。
上下层使用同一仓库管理，不把 Nav2 编译进 STM32。

| 入口 | 用途 |
|---|---|
| [ROS 2 快速开始](ros2_ws/README.md) | 安装、构建、仿真、建图和导航 |
| [实车接口与分阶段接入](docs/ROS2_HARDWARE_INTEGRATION.md) | 核对现有协议、轮速单位、里程计和控制权 |
| [软件与仿真验收](docs/evidence/ros2-integration-20261018/README.md) | 本次新目录构建、测试和实际仿真记录；不代表实车通过 |

以下是底层固件的现有交付状态。固件版本与 ROS 软件版本分别管理。

**开发中 · 当前源码 v131（ICM-42688-P驱动，未上板） · 整体实物验收未完成。**

[ICM-42688-P驱动与接线](docs/ICM42688_UPGRADE.md) · [BSP分层说明](docs/BSP.md) · [坐标导航说明](docs/COORDINATE_NAVIGATION.md) · [MPU6050历史升级记录](docs/MPU6050_UPGRADE.md) · [当前进度](docs/PROGRESS.md) · [PID 调试记录](docs/PID_TUNING.md) · [完整验收与实测波形](docs/ACCEPTANCE.md) · [每轮归档规范](docs/ACCEPTANCE_WORKFLOW.md)

双轮差速小车：**手动无线驾驶 + 实时遥测 + STM32 OTA**。v128新增单目标相对坐标导航：转向、接近、到位停稳，并增加网页输入、取消及状态显示。v127已完成安装和通信检查，但MPU6050采集仍失败；用户报告主控供电异常，硬件待修。v128坐标导航与v130 BSP分层仅通过离线验证和构建，未烧录或实车验收；不包含避障、路径搜索、巡线、LoRa业务、第三电机或执行器控制。

2026-10-10：用户已改用ICM-42688-P，v131沿用I²C2 PB10/PB11重写驱动，包含身份识别、复位/配置读回、100Hz采样与ICM数据换算，主机回归和构建通过，未安装。STM32最后确认运行v127，当前主控通信、供电维修、ICM真实采样与导航精度仍待验证，旧MPU记录仅用于历史追溯。

源自独立项目 `STM32-CAN-Line-Following-Car` 的 HAL/CMSIS 驱动、硬件映射和已验证 OTA 模块。新项目完全独立，构建不引用原仓库；原项目及未提交修改保留。

## 当前交付边界

- STM32 HAL App、Bootloader、ESP32 网关已构建；两份 Keil 工程可独立构建。
- 通信/缓存/OTA 故障注入、实际 C 运动状态机、电脑端校验和浏览器交互已有离线测试。
- 历史记录已包含 ESP32 SG01、STM32 Bootloader/App 安装、v100–v125 迭代及架空运动试验；v123 的 12 rpm 双向各 20 秒表现较稳。
- **v125 复测仍出现右轮低速起转后再次停转；起步超调、左右启动差、落地载荷与长期可靠性仍待验收。正常停止一次不代表通信问题永久解决。** 具体工况和失败证据见验收记录，本仓库初始化未新增硬件试验。
- 自行编写及复用的业务函数有中文说明；厂商 HAL/CMSIS 保留原注释和许可。

## 工程入口

| 内容 | 路径 |
|---|---|
| CubeMX 配置 | `firmware/stm32_app/SmartCar.ioc` |
| 小车 App Keil 工程 | `firmware/stm32_app/MDK-ARM/SmartCar.uvprojx` |
| Bootloader Keil 工程 | `firmware/stm32_bootloader/MDK-ARM/Bootloader.uvprojx` |
| HAL 业务入口 | `firmware/stm32_app/Core/Src/smartcar.c` |
| 可离线测试的运动控制核心 | `firmware/stm32_app/Core/Src/drive.c` |
| ESP32 工程 | `firmware/esp32_gateway/platformio.ini` |
| 本地电脑控制台 | `desktop/server.py`、`desktop/index.html` |
| 软件验收记录 | `docs/ACCEPTANCE.md` |

## 构建与启动

在项目根目录运行：

```powershell
python -B tools/build.py
python -B tools/test.py
python -B tools/configure.py
uv tool run --with pip --from platformio==6.1.19 platformio run --project-dir firmware/esp32_gateway
python -B desktop/server.py
```

打开 `http://127.0.0.1:18765`。未配置或设备不在线时只能查看离线界面，不显示虚构数据。
`configure.py` 在本机输入 Wi-Fi SSID、密码与 ESP32 局域网地址，生成 `config.local.json`。
修改 Wi-Fi/令牌后必须重新构建 ESP32；历史记录中的本机配置不随仓库分发，克隆后需自行配置。
只修改 DHCP 地址时重启电脑控制台即可。配置文件与生成的私密头文件不进入源码版本管理。

默认 Arm 工具链 `D:\Keil_v5\ARM\ARMCLANG\bin`，可用 `ARM_BIN` 覆盖；主机测试 GCC 默认 `C:\msys64\ucrt64\bin\gcc.exe`，可用 `HOST_GCC` 覆盖。ESP32 使用固定 espressif32 6.12.0 / Arduino 2.0.17。

构建生成：

- `out/SmartCar-v131.ota`：32 字节元数据 + App，供网页 OTA 上传；文件名版本取自 `DRIVE_VERSION`，本版未安装。
- `out/SmartCar-factory.bin`：首次通过 SWD 安装的 Bootloader + App + 有效记录，起始地址 `0x08000000`，不覆盖参数页。**本项目没有自动执行烧录命令。**
- `out/app/app.hex`、`out/boot/boot.hex`：分区镜像。仅下载 App HEX 不会同步有效记录；正式升级使用 OTA 包，首次安装使用 factory 镜像。
- `out/build_report.json`、`out/test_report.json`、`out/tests.log`：构建和测试证据。

## 驾驶流程

1. 首次实物验收前保持电机断电；准备转动验证时架空驱动轮并准备实物断电开关。
2. 设备连接后，确认双轮反馈正常且静止，点击“使能驱动”。
3. 按住方向键行驶，松开则目标回零；界面初始限速 5 rpm，最大轮速 20 rpm。
4. “停止并禁止驱动”、空格或切换窗口会撤销控制。禁止驱动输出零电流，车辆依靠阻力滑停，**不是机械抱闸**。
5. 300 ms 无新控制指令或 100 ms 无任一轮反馈，STM32 锁存故障并清零电流；恢复连接不自动运动。待反馈新鲜且停稳，清故障后重新使能。
6. OTA 要求禁止驱动、双轮反馈新鲜且连续静止 500 ms；完成后仍禁止驱动。Bootloader 恢复模式可直接重传，不要求 App 遥测存在。

**电机/C610 完全断电时没有 CAN 反馈，App 将拒绝进入 OTA。** 这是停止确认门槛；若 App 损坏则 Bootloader 自身仍可接收恢复包。首次安装新 Bootloader 必须走 SWD，不能用旧 OTA 覆盖 Bootloader。

网页控制只用于可互访的局域网。实时闭环和最终超时判断在 STM32；ESP32 与电脑均不会自动循环发送运动命令。当前不提供公网接入、签名固件、旧 App 回滚或断点续传。

## CubeMX / Keil 维护

`.ioc` 保留 CAN1、USART1、HSE 和 SWD，移除巡线 GPIO、TIM3、USART2/3 等旧业务配置。`main.c` 的 `USER CODE` 区挂接 `SmartCar_EarlyInit/Init/Poll`，业务放在独立 `smartcar.c`、`drive.c`。

CubeMX 重新生成后必须运行 `python tools/build.py`，它同步两个 MDK 工程的源列表、头文件路径、C8T6 容量和 App scatter。再次检查 `SmartCar_EarlyInit()` 仍位于 HAL 初始化之前，App 链接地址仍为 `0x08004000`。
本次未实际操作 CubeMX 执行重新生成；已验证的是当前 `.ioc` 配置与源码静态一致性、脚本构建及 Keil 构建。

查看 [架构和协议](docs/ARCHITECTURE.md) 了解字段、状态、接线及尚待实测的边界。

## 仓库记录范围

保留 STM32 HAL/CMSIS、CubeMX `.ioc`、Keil/EIDE 工程、Bootloader、ESP32、电脑控制台、测试和调试证据。`docs/evidence/` 中的真实 CSV/JSON、PNG/SVG 和采样二进制属于验收资料，保留失败及缺测说明。

`config.local.json`、生成的私密头文件、Flash 备份、编译产物和缓存不提交。历史文档中的 `out/` 路径指本机工作记录，不承诺全部随仓库提供；可移植证据以 `docs/evidence/` 为准。厂商代码保留原许可；本次未擅自为整个工程添加新开源许可证。
