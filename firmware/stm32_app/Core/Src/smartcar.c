/* SmartCar V1：HAL 双轮 CAN 控制，不含巡线、第三电机或 LoRa 自动启动。 */
#include "main.h"
#include "drive.h"
#include "angle_speed.h"
#include "navigation.h"
#include "mpu6050.h"
#include "car_protocol.h"
#include "ota_receiver.h"
#include "update_request.h"
#include <string.h>

extern CAN_HandleTypeDef hcan;
extern UART_HandleTypeDef huart1;
static drive_state drive;
static imu_state imu;
static navigation nav;
static uint32_t control_interval_max,control_late;
static uint8_t tx_frames[4][CAR_MAX_FRAME];
static uint16_t tx_sizes[4];
static unsigned tx_head,tx_tail;
static uint8_t tx_active;
static uint32_t tx_started,tx_dropped;
static angle_speed speed_estimate[2];
static int16_t raw_speed[2];
static car_receiver receiver;
static uint8_t rx_byte;
static volatile uint8_t rx_ring[512];
static volatile uint16_t rx_head,rx_tail;
static volatile uint32_t io_errors;
static uint32_t handled_errors;
static uint16_t telemetry_seq;
static void poll_can(void);

/** 从小端载荷读取有符号转速，不依赖内存对齐。 */
static int16_t get16(const uint8_t *p) { return (int16_t)((uint16_t)p[0]|(uint16_t)p[1]<<8); }

/** 将有符号反馈写为小端格式，供 ESP32 与电脑按统一协议读取。 */
static void put16(uint8_t *p,int16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)((uint16_t)v>>8); }

/** 有界异步串口队列，缓冲区在发送完成前保持有效；不等待硬件。 */
static void poll_tx(void)
{
    uint32_t mask,now=HAL_GetTick(); HAL_StatusTypeDef status;
    if (tx_active) {
        if (huart1.gState!=HAL_UART_STATE_READY) {
            if (now-tx_started<30U) return;
            mask=__get_PRIMASK(); __disable_irq();
            (void)HAL_UART_AbortTransmit(&huart1); __set_PRIMASK(mask); ++io_errors;
        }
        tx_active=0; ++tx_tail;
    }
    if (tx_tail==tx_head) return;
    mask=__get_PRIMASK(); __disable_irq();
    status=HAL_UART_Transmit_IT(&huart1,tx_frames[tx_tail%4],tx_sizes[tx_tail%4]);
    __set_PRIMASK(mask);
    if (status==HAL_OK) { tx_active=1; tx_started=now; }
    else { ++io_errors; ++tx_tail; }
}

/** 排队一帧；遥测最多占两槽，为停止/控制应答预留空间。 */
static void send_frame(uint8_t type,uint16_t seq,const uint8_t *p,uint16_t size)
{
    size_t n; unsigned used=tx_head-tx_tail;
    int optional=type==CAR_MSG_TELEMETRY || type==CAR_MSG_IMU || type==CAR_MSG_DIAGNOSTIC;
    if (used>=4U || (optional && used>=2U)) { if (optional) ++tx_dropped; else ++io_errors; return; }
    n=car_encode(tx_frames[tx_head%4],CAR_MAX_FRAME,type,seq,p,size);
    if (!n) { ++io_errors; return; }
    tx_sizes[tx_head%4]=(uint16_t)n; ++tx_head; poll_tx();
}

/** 升级复位前仅在禁止驱动状态有界排空应答，不用于运动循环。 */
static void flush_reply(void)
{
    uint32_t start=HAL_GetTick();
    while (tx_head!=tx_tail && HAL_GetTick()-start<80U) { poll_can(); poll_tx(); }
}

/** 发送 CAN 0x200 电流帧：右轮 ID1 反向、左轮 ID2 正向，其余通道固定零。 */
static int send_current(void)
{
    CAN_TxHeaderTypeDef header={0};
    uint8_t data[8]={0}; uint32_t mailbox;
    int16_t right=(int16_t)-drive.output[1],left=drive.output[0];
    /* 禁止积压旧运动电流；总线忙时本周期报故障并撤销所有待发送邮箱。 */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan)!=3U) {
        HAL_CAN_AbortTxRequest(&hcan,CAN_TX_MAILBOX0|CAN_TX_MAILBOX1|CAN_TX_MAILBOX2);
        drive_fault(&drive,FAULT_CAN); right=0; left=0;
    }
    header.StdId=0x200; header.IDE=CAN_ID_STD; header.RTR=CAN_RTR_DATA; header.DLC=8;
    data[0]=(uint8_t)((uint16_t)right>>8); data[1]=(uint8_t)right;
    data[2]=(uint8_t)((uint16_t)left>>8); data[3]=(uint8_t)left;
    return HAL_CAN_AddTxMessage(&hcan,&header,data,&mailbox)==HAL_OK;
}

/* 起步诊断只保留最初256拍，不覆盖为末段；运动期间不发送诊断帧。 */
static uint8_t startup_trace[256][28];
static unsigned startup_count,startup_read;
static uint8_t startup_recording;
static uint32_t startup_sent;

/** 保存起步最初约2.56秒的控制状态，新运动段重新采集，满后冻结而不覆盖。 */
static void capture_startup(uint32_t now)
{
    unsigned i; uint8_t *p;
    if (drive.state!=DRIVE_ARMED || (!drive.target[0] && !drive.target[1])) {
        startup_recording=0; return;
    }
    if (!startup_recording) { startup_count=0; startup_read=0; startup_recording=1; }
    if (startup_count>=256U) return;
    p=startup_trace[startup_count++]; memcpy(p,"TRC3",4); ota_put32(p+4,now);
    for (i=0;i<2;i++) {
        put16(p+8+2*i,drive.target[i]); put16(p+12+2*i,drive.rpm[i]);
        put16(p+16+2*i,drive.output[i]); put16(p+20+2*i,(int16_t)drive.integral[i]);
        p[24+i]=drive.starting[i];
    }
    p[26]=drive.state; p[27]=drive.fault;
}

/** 仅禁止驱动或故障且输出为零时导出，一次一帧；不改变控制权或运动目标。 */
static void dump_startup(uint32_t now)
{
    if ((drive.state!=DRIVE_DISARMED && drive.state!=DRIVE_FAULT) ||
        drive.output[0] || drive.output[1] || startup_read>=startup_count || now-startup_sent<20U) return;
    startup_sent=now;
    send_frame(CAR_MSG_DIAGNOSTIC,telemetry_seq++,startup_trace[startup_read++],28);
}

/** 处理主循环中的完整协议帧；只有明确停稳后才接受 Bootloader 请求。 */
static void on_frame(void *context,uint8_t type,uint16_t seq,const uint8_t *p,uint16_t size)
{
    uint8_t reply[16]={0}; (void)context;
    if (type==CAR_MSG_PING && size<=32) { send_frame(CAR_MSG_PONG,seq,p,size); return; }
    if ((type==0x40 && size==16) || (type==CAR_MSG_NAVIGATION && size==CAR_NAV_SIZE)) {
        uint32_t now=HAL_GetTick();
        if (type==CAR_MSG_NAVIGATION) reply[0]=navigation_command(&nav,&imu,&drive,speed_estimate,p,now);
        else if (p[1]||p[2]||p[3]) reply[0]=CONTROL_ARGUMENT;
        else reply[0]=navigation_drive_command(&nav,&drive,p[0],ota_get32(p+4),ota_get32(p+8),get16(p+12),get16(p+14),now);
        reply[1]=drive.state; reply[2]=drive.fault;
        ota_put32(reply+4,drive.session); ota_put32(reply+8,drive.sequence);
        if (drive.state!=DRIVE_ARMED && !send_current()) drive_fault(&drive,FAULT_CAN);
        send_frame(0x71,seq,reply,16); return;
    }
    reply[1]=OTA_APP; reply[2]=type; ota_put32(reply+12,DRIVE_VERSION);
    if (type==CAR_MSG_OTA_QUERY && size==0) send_frame(CAR_MSG_OTA_REPLY,seq,reply,16);
    else if (type==CAR_MSG_ENTER_UPDATE && size==4 && !memcmp(p,"OTA1",4)) {
        if (!drive_can_update(&drive,HAL_GetTick())) reply[0]=OTA_BUSY;
        else {
            drive.state=DRIVE_UPDATING;
            drive_tick(&drive,HAL_GetTick());
            if (!send_current()) reply[0]=OTA_IO_ERROR;
            else {
                uint32_t start=HAL_GetTick();
                while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan)!=3U && HAL_GetTick()-start<10U) {}
                if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan)!=3U || (hcan.Instance->ESR&CAN_ESR_BOFF)) reply[0]=OTA_IO_ERROR;
                else if (!update_request_set()) reply[0]=OTA_IO_ERROR;
            }
        }
        send_frame(CAR_MSG_OTA_REPLY,seq,reply,16);
        if (!reply[0]) { flush_reply(); HAL_CAN_Stop(&hcan); NVIC_SystemReset(); }
        if (drive.state==DRIVE_UPDATING) drive_fault(&drive,FAULT_CAN);
    }
}

/** 从 CAN FIFO0 有界读取两台 M2006 的速度与电流反馈，忽略无关或损坏帧。 */
static void poll_can(void)
{
    unsigned count;
    for (count=0;count<8 && HAL_CAN_GetRxFifoFillLevel(&hcan,CAN_RX_FIFO0);count++) {
        CAN_RxHeaderTypeDef h; uint8_t p[8]; int16_t rpm,current; unsigned wheel;
        if (HAL_CAN_GetRxMessage(&hcan,CAN_RX_FIFO0,&h,p)!=HAL_OK) { drive_fault(&drive,FAULT_CAN); break; }
        if (h.IDE!=CAN_ID_STD || h.RTR!=CAN_RTR_DATA || h.DLC!=8 || (h.StdId!=0x201 && h.StdId!=0x202)) continue;
        rpm=(int16_t)((uint16_t)p[2]<<8|p[3]); current=(int16_t)((uint16_t)p[4]<<8|p[5]);
        wheel=h.StdId==0x202 ? 0U:1U;
        if (rpm==INT16_MIN || current==INT16_MIN) continue;
        if (wheel==1) { rpm=(int16_t)-rpm; current=(int16_t)-current; }
        raw_speed[wheel]=rpm;
        angle_speed_update(&speed_estimate[wheel],(uint16_t)((uint16_t)p[0]<<8|p[1]),wheel==1 ? -1:1,HAL_GetTick());
        drive_feedback(&drive,wheel,speed_estimate[wheel].valid ? speed_estimate[wheel].rpm:rpm,current,HAL_GetTick());
    }
    if ((hcan.Instance->ESR&CAN_ESR_BOFF) || (hcan.Instance->RF0R&CAN_RF0R_FOVR0)) {
        drive_fault(&drive,FAULT_CAN); __HAL_CAN_CLEAR_FLAG(&hcan,CAN_FLAG_FOV0);
    }
}

/** 以 10 Hz 输出 40 字节运动遥测；转速为方向归一化的电机轴 rpm。 */
static void send_telemetry(void)
{
    uint8_t p[40]={0}; unsigned i; uint32_t now=HAL_GetTick();
    ota_put32(p,now); ota_put32(p+4,DRIVE_VERSION);
    p[8]=drive.state; p[9]=drive.fault;
    for (i=0;i<2;i++) {
        if (drive.seen[i] && now-drive.feedback_ms[i]<DRIVE_FEEDBACK_MS) p[10]|=(uint8_t)(1U<<i);
        put16(p+12+2*i,drive.target[i]); put16(p+16+2*i,drive.rpm[i]);
        put16(p+20+2*i,drive.output[i]); put16(p+24+2*i,drive.measured_current[i]);
    }
    ota_put32(p+28,drive.session); ota_put32(p+32,drive.sequence); ota_put32(p+36,io_errors);
    send_frame(CAR_MSG_TELEMETRY,telemetry_seq++,p,40);
}

/** 输出固定身份 SC01 与错误统计；固件版本通过 QUERY 独立确认。 */
static void send_identity(void)
{
    uint8_t p[16]; memcpy(p,"SC01",4); ota_put32(p+4,HAL_GetTick());
    ota_put32(p+8,receiver.crc_errors+receiver.header_errors+receiver.timeouts);
    ota_put32(p+12,io_errors); send_frame(CAR_MSG_DIAGNOSTIC,telemetry_seq++,p,16);
}

/** 单字节接收完成后推进环形队列；溢出计错，由主循环锁存停车故障。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    uint16_t next; if (uart!=&huart1) return;
    next=(uint16_t)((rx_head+1U)%512U);
    if (next==rx_tail) ++io_errors; else { rx_ring[rx_head]=rx_byte; rx_head=next; }
    if (HAL_UART_Receive_IT(&huart1,&rx_byte,1)!=HAL_OK) ++io_errors;
}

/** 串口错误计数并重新挂接收，保证线路恢复后仍可诊断及清故障。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart!=&huart1) return;
    ++io_errors; __HAL_UART_CLEAR_OREFLAG(uart);
    (void)HAL_UART_Receive_IT(uart,&rx_byte,1);
}

/** 接收重挂失败后由主循环恢复；丢弃残帧，保留故障锁定，绝不自动使能。 */
static void recover_uart_rx(void)
{
    uint32_t primask=__get_PRIMASK();
    __disable_irq();
    if (huart1.RxState==HAL_UART_STATE_READY && huart1.Lock==HAL_UNLOCKED) {
        /* Tx 初始化短暂持锁时，接收回调可能返回 HAL_BUSY；此处在发送之外重试。 */
        rx_tail=rx_head;
        car_receiver_init(&receiver,on_frame,0);
        __HAL_UART_CLEAR_OREFLAG(&huart1);
        (void)HAL_UART_Receive_IT(&huart1,&rx_byte,1);
    }
    __set_PRIMASK(primask);
}

/** 启动独立看门狗，典型约 1 秒；仅完整主循环喂狗，Bootloader 也需支持接续。 */
static void watchdog_start(void)
{
    /* 与 HAL_IWDG_Init 相同：先启动 IWDG，硬件才自动开启 LSI 更新时钟。 */
    IWDG->KR=0xCCCC;
    IWDG->KR=0x5555; IWDG->PR=3; IWDG->RLR=1249;
    /* 若更新时钟异常，已运行的看门狗会复位，不在这里持续喂狗。 */
    while (IWDG->SR) {} IWDG->KR=0xAAAA;
}

/** 在 HAL 初始化前设置 App 向量并接续看门狗；复位后的 NVIC 尚未启用外设中断。 */
void SmartCar_EarlyInit(void)
{
    SCB->VTOR=0x08004000U; __DSB(); __ISB(); __enable_irq(); watchdog_start();
}

/** 外设初始化后配置接收过滤器和状态机，不启用任何电机运动。 */
void SmartCar_Init(void)
{
    CAN_FilterTypeDef filter={0};
    drive_init(&drive); imu_init(&imu); navigation_init(&nav); mpu6050_init(); car_receiver_init(&receiver,on_frame,0);
    filter.FilterMode=CAN_FILTERMODE_IDLIST; filter.FilterScale=CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh=0x201U<<5; filter.FilterMaskIdHigh=0x202U<<5;
    filter.FilterFIFOAssignment=CAN_RX_FIFO0; filter.FilterActivation=ENABLE;
    if (HAL_CAN_ConfigFilter(&hcan,&filter)!=HAL_OK || HAL_CAN_Start(&hcan)!=HAL_OK) Error_Handler();
    if (HAL_UART_Receive_IT(&huart1,&rx_byte,1)!=HAL_OK) Error_Handler();
    send_identity();
}

/** 将IEEE754浮点编码为小端，协议字段不使用编译器结构体布局。 */
static void put_float(uint8_t *p,float value)
{
    uint32_t bits; memcpy(&bits,&value,4); ota_put32(p,bits);
}

/** 输出独立版本化IMU/定位快照；诊断模式50Hz，常规10Hz。 */
static void send_imu(uint32_t now)
{
    uint8_t p[CAR_IMU_SIZE]={0}; unsigned i;
    p[0]=1; p[1]=(imu_fresh(&imu,now) ? 1:0)|(nav.valid ? 2:0)|(nav.discontinuous ? 4:0)|
        (nav.configured ? 8:0)|(nav.axes_confirmed ? 16:0)|(imu.attitude_ready ? 32:0);
    p[2]=imu.calibration; p[3]=nav.mode;
    ota_put32(p+4,now); ota_put32(p+8,imu.sample_ms); ota_put32(p+12,imu.sequence);
    for (i=0;i<7;i++) put16(p+16+2*i,imu.raw[i]);
    p[30]=nav.result; p[31]=nav.diagnostic;
    put_float(p+32,imu.roll); put_float(p+36,imu.pitch); put_float(p+40,imu.yaw);
    put_float(p+44,nav.x); put_float(p+48,nav.y); put_float(p+52,nav.heading);
    put_float(p+56,imu.gyro[2]); ota_put32(p+60,mpu_status.errors);
    ota_put32(p+64,mpu_status.timeouts); ota_put32(p+68,imu.gaps);
    ota_put32(p+72,control_interval_max); ota_put32(p+76,imu.max_interval_ms);
    ota_put32(p+80,control_late); ota_put32(p+84,tx_dropped); ota_put32(p+88,nav.origin_id);
    for (i=0;i<2;i++) { put16(p+92+2*i,drive.target[i]); put16(p+96+2*i,drive.rpm[i]); put16(p+100+2*i,drive.output[i]); }
    p[104]=drive.state; p[105]=drive.fault; p[106]=mpu_status.identity;
    ota_put32(p+108,DRIVE_VERSION);
    send_frame(CAR_MSG_IMU,telemetry_seq++,p,sizeof(p));
}

/** 先处理CAN和安全控制，再调度非阻塞采集及低优先级遥测；完整循环才喂狗。 */
void SmartCar_Poll(void)
{
    static uint32_t control,telemetry,identity,imu_sent;
    unsigned budget=128; uint32_t now;
    recover_uart_rx(); poll_can(); poll_tx();
    now=HAL_GetTick();
    if (io_errors!=handled_errors) { handled_errors=io_errors; drive_fault(&drive,FAULT_UART); }
    if (now-control>=10U) {
        uint32_t interval=now-control;
        if (interval>control_interval_max) control_interval_max=interval;
        if (interval>12U) ++control_late;
        control=now;
        drive.rpm[0]=angle_speed_sample(&speed_estimate[0],now,raw_speed[0]);
        drive.rpm[1]=angle_speed_sample(&speed_estimate[1],now,raw_speed[1]);
        navigation_tick(&nav,&imu,&drive,speed_estimate,now);
        drive_tick(&drive,now);
        if (drive.state!=DRIVE_ARMED) navigation_cancel(&nav,NAV_DRIVE_LOST);
        if (!send_current()) drive_fault(&drive,FAULT_CAN);
        capture_startup(now);
    }
    while (rx_tail!=rx_head && budget--) {
        uint8_t byte=rx_ring[rx_tail]; rx_tail=(uint16_t)((rx_tail+1U)%512U);
        car_receiver_feed(&receiver,byte,HAL_GetTick());
    }
    now=HAL_GetTick(); car_receiver_poll(&receiver,now);
    mpu6050_poll(&imu,now,navigation_stationary(&drive,now));
    if (now-telemetry>=100U) { telemetry=now; send_telemetry(); }
    if (now-imu_sent>=(nav.diagnostic ? 20U:100U)) { imu_sent=now; send_imu(now); }
    if (now-identity>=1000U) { identity=now; send_identity(); }
    dump_startup(now); poll_tx(); IWDG->KR=0xAAAA;
}
