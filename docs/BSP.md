# STM32 当前PCB的BSP分层

版本v130，2026-09-23。仅离线重构、回归和构建；未烧录或操作硬件。范围是当前STM32 App的PCB适配，ESP32独立固件与Bootloader独立启动/Flash驱动未迁入App BSP。

2026-10-10，v131：设备驱动已由MPU6050换为[ICM-42688-P](ICM42688_UPGRADE.md)，下述目录/职责反映当前实现；BSP硬件层未修改。v130验证结果仍为历史记录，v131同样未安装。

## 目录与职责

```text
firmware/stm32_app/
  BSP/
    BSP_can.c       BSP_can.h
    BSP_uart.c      BSP_uart.h
    BSP_i2c.c       BSP_i2c.h
    BSP_system.c   BSP_system.h
  Core/Src/
    smartcar.c     协议、调度、故障决策、电机ID和方向映射
    icm42688.c     设备配置、采样周期、事务超时和重试策略
    imu.c          校准、轴向变换、姿态融合
    navigation.c   里程计、坐标导航与动作状态机
    drive.c        轮速PI、起步和停机保护
```

文件统一使用`BSP_模块名.c/.h`，大小写一致，避免Windows与其他系统路径差异。当前源码无已确认LED引脚，不为示例新增虚构LED模块；以后确认引脚与有效电平后再增加`BSP_led.c/.h`。

| 模块 | 当前板绑定及硬件职责 | 不负责 |
|---|---|---|
| BSP_can | CAN1 PA11/PA12、1Mbit/s初始化；指定ID过滤、原始帧收发、邮箱撤销、总线/FIFO错误 | 电机ID选择、电流编码、左右反向、是否停车 |
| BSP_uart | USART1 PA9/PA10、115200 8N1、IRQ；512字节原始接收环、接收恢复、异步发送/中止 | 帧解析、会话/心跳、ACK优先级、遥测队列、30ms超时决策 |
| BSP_i2c | I2C2 PB10/PB11开漏、100kHz；有界复位、指定地址寄存器事务、完成/错误查询、IRQ | ICM寄存器表、0x68/0x69识别、100Hz采样、5ms超时/1s重试、校准有效性 |
| BSP_system | 8MHz HSE→72MHz、App向量0x08004000、看门狗启动/喂狗、毫秒时基、复位 | 10ms调度、何时喂狗、何时允许升级/复位 |

看门狗没有提供停止接口，启动和喂狗均是硬件操作。是否复位、是否喂狗由上层决定。BSP头文件只暴露标准整数类型和原始帧结构，不向业务公开HAL句柄。I2C句柄是实现内部符号，不在BSP公开头声明。

## 调用方向

业务/设备服务 → BSP硬件接口 → HAL/CMSIS → 外设。

业务决策不得反向放入BSP。BSP不include drive/navigation/imu/icm42688/car_protocol，不调用故障状态机、编码器算法或协议编码。CAN滤波ID由应用传入；BSP只执行配置。UART返回原始字节和错误数，由应用重置解析器及锁存故障。I2C只报告事务状态，ICM设备服务决定何时重试及使哪些数据失效。

CAN读返回1有帧、0无帧、-1硬件错误；发送函数只启动一次，不等待、不缓存旧控制帧。UART发送缓冲由调用方保持有效直到发送完成或中止；四槽帧队列、两个遥测槽限制、30ms超时、停止应答优先策略仍由smartcar.c维护。RX恢复会丢弃未消费字节并返回恢复标志，上层据此丢弃半帧。I2C缓冲同样必须保持有效；完成状态持续到下一事务或复位，不允许多任务并发访问。

## 初始化与工程集成

main.c保留CubeMX入口和用户区：早期BSP向量/看门狗 → HAL_Init → BSP时钟 → BSP CAN/UART初始化 → SmartCar_Init。通用异常处理和SysTick分发留在启动胶水代码中，HAL供应商system文件保持不变。HAL MSP回调按外设放入相应BSP模块；stm32f1xx_it.c只转发IRQ，stm32f1xx_hal_msp.c保留说明占位以兼容当前工程入口。

`tools/build.py`将BSP目录加入App源码及头文件路径，并同步MDK源列表；EIDE同步四个模块及include路径。`.ioc`硬件参数未改。CubeMX重新生成后必须复核MSP回调是否被重复生成、初始化是否仍委托BSP；本轮未运行CubeMX重新生成流程，不能宣称自动生成闭环已验证。

## 验证边界

`python -B tools/test.py`：原主机回归 + BSP依赖边界、CAN帧/邮箱/Bus-Off/溢出 + 生产I2C与MPU状态机HAL替身 + UART持锁窗口/缓冲寿命/超时测试。业务层drive/imu/navigation/angle_speed生产算法没有修改。

`python -B tools/build.py`：v130 App24024/47104字节，静态RAM10896/20480字节，Bootloader3744字节。App/Bootloader地址布局保持。v129为中间构建，v130为最终构建，两者均未安装。实物通信、IMU采样、停止和故障恢复须供电维修确认后重新验收。
