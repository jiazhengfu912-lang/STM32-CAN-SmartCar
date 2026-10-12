# ICM-42688-P 驱动与接线（v131）

2026-10-10：用户已决定以 ICM-42688-P 替换 MPU6050，并确认沿用 STM32 I²C2（PB10/PB11）。当前源码和构建为 **v131**；设备最后确认运行 **v127**，本轮未连接、烧录或操作设备。**驱动离线验证通过，真实身份、采样、安装方向、漂移与导航均待实测。**

## 资料核对

资料目录：`D:\BaiduNetdiskDownload\ICM-42688-P`。

- 两份 `ICM42688芯片手册.pdf` 相同：TDK DS-000347，Revision 1.2（2020-04-19），109 页，SHA256 `4d7c04e3e86f37defe27c20a82b9fcd9d4ddbd3b9d68a525c2d49595d52cf97a`。
- 阅读两种 STC 示例的 I²C/SPI 驱动、寄存器枚举及姿态例程。I²C 示例固定地址 0x69；两个示例的默认量程/ODR不同（±4g/±500dps/1kHz 与 ±16g/±2000dps/32kHz），不能直接用于本项目换算。
- 示例身份检测存在无限循环，`unsigned char time` 在循环内反复赋值且 `time<0` 不成立；软件 I²C 没有传播 ACK 失败。`PWR_MGMT0=0` 是关闭传感器，软件复位应写 `DEVICE_CONFIG`。加速度示例输出系数按 mg 定义，不能当作项目使用的 g；`angle.c` 的固定 `halfT` 与项目按真实时间间隔积分不同。本项目依据手册自行编写驱动，未复制 STC 实现。
- 重点核对手册 §3/4（量程、电源与引脚）、§4.13（温度）、§5.5（UI滤波表）、§9（I²C）、§12.8（无效数据）、§14（复位、就绪、配置、寄存器银行）。本地手册 p63/p68/p73–80/p83–87 支持具体寄存器实现。
- 补充在线交叉检查 [TDK 官方 v1.6 手册](https://product.tdk.cn/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000347-icm-42688-p-v1.6.pdf) 的复位/启用时序和 DATA_RDY 定义。[官方产品页](https://www.invensense.tdk.com/en-us/products/6-axis/icm-42688-p) 当前列出 v1.9，未取得该版本全文，不宣称已逐项复核 v1.9。

本机已有 Arm Compiler、GCC、Python、PDF读取和绘图库，本轮**不需要下载软件，未下载或安装软件**。STC示例所用 Keil C251/STC-ISP 不用于本 STM32 工程；本轮也不需要 SmartMotion/MotionLink。

## I²C2 接线

| ICM模块信号 | STM32/连接 | 要求 |
|---|---|---|
| VDD / VCC | 已测量正常的 3.3V | 芯片 VDD/VDDIO 工作范围 1.71–3.6V；按3.3V系统连接，不能默认模块支持5V |
| VDDIO（若单独引出） | 3.3V | 信号上拉也接3.3V |
| GND | GND | 共地 |
| SCL / SCLK | PB10 | I²C2，100kHz，开漏 |
| SDA / SDI | PB11 | 开漏；检查模块是否已有上拉，未有则按电路条件补上拉 |
| CS / AP_CS | 3.3V（VDDIO） | 手册要求 I²C模式时为高；不能悬空或接GND |
| AD0 / SDO | GND→0x68，或3.3V→0x69 | 固定电平；驱动有界尝试两种地址，资料示例默认0x69 |
| INT1 / INT2 | 不接STM32 | 本版读取状态寄存器，未新增EXTI；内部使能DRDY锁存用于查询 |
| FSYNC / CLKIN（若引出） | 未使用 | 本版不启用外部时钟或FSYNC；手册建议未用FSYNC接GND，该脚与INT2/CLKIN复用，须先核对模块电路 |

以上是芯片信号对应关系；资料未包含这块实际模块的引脚照片/原理图，**实际排针顺序、电源处理与上拉仍须核对模块丝印和电路**。过去主控供电异常的排查/修复结论没有因更换传感器而自动成立。

## 驱动行为和换算

`Core/Src/icm42688.c` → `BSP_I2C_*`。设备的寄存器、采样、超时和重试策略仍在驱动层；BSP/HAL、`.ioc`、电机/导航算法没有改动。MDK/EIDE源列表已同步，旧MPU驱动从当前App源列表移除；旧实测记录保留。

初始化：非阻塞等100ms → BANK0 → WHO_AM_I=0x47 → DEVICE_CONFIG软件复位 → 至少等2ms → RESET_DONE校验 → 逐项写入/读回配置 → 启用双传感器低噪声模式 → 非阻塞等50ms → 功率模式读回 → DRDY采样。等待不调用 HAL_Delay，不暂停控制主循环。每笔I²C事务5ms超时，识别失败尝试另一地址，初始化/运行错误后1秒重新初始化。

| 配置/寄存器 | 本版值 | 依据/含义 |
|---|---|---|
| REG_BANK_SEL / WHO_AM_I | 0x76写0 / 0x75读0x47 | 选择BANK0并验证型号 |
| DEVICE_CONFIG / INT_STATUS | 0x11写1 / 0x2D读bit4 | 软件复位、RESET_DONE |
| INTF_CONFIG0 | 0x4C=0x32 | 传感器大端；禁SPI，保留I²C；不复制无效样本 |
| GYRO_CONFIG0 / ACCEL_CONFIG0 | 0x4F=0x48 / 0x50=0x48 | ±500dps、±4g，两个传感器均100Hz |
| GYRO_CONFIG1 / ACCEL_CONFIG1 | 0x51=0x16 / 0x53=0x0D | 二阶UI滤波；保留手册默认DEC2与保留位 |
| GYRO_ACCEL_CONFIG0 | 0x52=0x66 | 两个UI滤波BW码均6；100Hz二阶表实际约19.2Hz，非简单把40Hz当实测带宽 |
| FSYNC_CONFIG | 0x62=0 | 关闭FSYNC对原始数据LSB的标记 |
| INT_CONFIG0 / INT_CONFIG | 0x63=0x30 / 0x14=0x07 | DRDY锁存，状态/传感器读取清除；INT1高有效推挽 |
| INT_CONFIG1 / INT_SOURCE0 | 0x64=0 / 0x65=8 | 清INT_ASYNC_RESET默认位，内部使能DRDY到INT1 |
| PWR_MGMT0 | 0x4E=0x0F | 加速度/陀螺仪低噪声、温度启用；启用后50ms内不访问寄存器 |

每10ms启动一次 INT_STATUS 查询，仅 bit3=1 时从 TEMP_DATA1（0x1D）连续读取14字节。未就绪不刷新样本序号和时间，不把旧值反复积分；状态与数据读取清DRDY，减少两笔事务之间新数据造成的旧就绪标志残留。0x8000未初始化/禁用占位触发失效与重试。寄存器模式不启用FIFO，不补读丢失历史样本；真实总线读数完整性、采样间隔及边界竞争仍需实测。

线序 `temp, ax, ay, az, gx, gy, gz` → 遥测序 `ax, ay, az, temp, gx, gy, gz`（112字节协议不变）。

- 加速度：`raw / 8192` g（±4g）。
- 角速度：`raw × (500 / 32768) × π / 180` rad/s（±500dps，65.5是手册四舍五入的灵敏度）。
- 温度：`raw / 132.48 + 25` °C。电脑端按 identity=0x47 使用此公式；identity=0x68 保留旧MPU记录的换算，未知身份返回无温度。
- 轴向映射、5秒静止校准、roll/pitch互补滤波和yaw积分保持现有算法。驱动重新初始化会失效样本/姿态/校准并清零偏，恢复后需重新校准/设原点，不自动续动。
- 50ms IMU新鲜度、100ms电机反馈、300ms用户心跳保护及原轮速限幅保持；网页/网关不代发心跳。

时间戳仍为 MCU 接收完成的毫秒时刻，**不是芯片硬件时间戳**。六轴ICM没有磁航向或绝对定位，长期偏航漂移仍需真实测量。

## 已验证与下一步

本轮 [离线证据](evidence/icm42688-v131-20261010/verification.json)：真实驱动+BSP的HAL替身故障测试、完整主机回归及STM32构建通过；坐标仿真CSV与v128归档逐字节一致。App 24304/47104字节，静态RAM10896/20480字节（不代表栈高水位），Bootloader3744字节。构建仅有工具链既有A1950W汇编器弃用警告，C编译无错误。`out/SmartCar-v131.ota` 已生成，**未安装**。

在任何烧录/通电运动前，先复核当前主控离线原因、供电、模块信号与架空/电机动力状态。安装后只读确认版本131、WHO_AM_I=0x47、新样本递增、错误/超时、约10ms采样间隔和静止重力约1g，再确认三轴方向、做静止校准和10分钟漂移采集。ICM的安装方向不能沿用未验证的MPU轴映射。完成这些及原停止保护验收后，才分阶段验证直行、转角与坐标导航。

每轮真实测试必须保留全部CSV/JSON、SHA256、逐点PNG/SVG，并将波形直接嵌入 [ACCEPTANCE.md](ACCEPTANCE.md)。本轮软件注入数据和图不能替代真实ICM或小车证据。
