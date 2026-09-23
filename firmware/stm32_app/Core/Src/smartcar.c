/* SmartCar V1：HAL 双轮 CAN 控制，不含巡线、第三电机或 LoRa 自动启动。 */
#include "BSP_can.h"
#include "BSP_uart.h"
#include "BSP_system.h"
#include "smartcar.h"
#include "drive.h"
#include "angle_speed.h"
#include "navigation.h"
#include "mpu6050.h"
#include "car_protocol.h"
#include "ota_receiver.h"
#include "update_request.h"
#include <string.h>

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
static uint32_t io_errors;
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
    uint32_t now=BSP_System_Millis(); int status;
    if (tx_active) {
        if (!BSP_UART_TxReady()) {
            if (now-tx_started<30U) return;
            BSP_UART_AbortTx(); ++io_errors;
        }
        tx_active=0; ++tx_tail;
    }
    if (tx_tail==tx_head) return;
    status=BSP_UART_Transmit(tx_frames[tx_tail%4],tx_sizes[tx_tail%4]);
    if (status) { tx_active=1; tx_started=now; }
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
    uint32_t start=BSP_System_Millis();
    while (tx_head!=tx_tail && BSP_System_Millis()-start<80U) { poll_can(); poll_tx(); }
}

/** 发送 CAN 0x200 电流帧：右轮 ID1 反向、左轮 ID2 正向，其余通道固定零。 */
static int send_current(void)
{
    uint8_t data[8]={0};
    int16_t right=(int16_t)-drive.output[1],left=drive.output[0];
    /* 禁止积压旧运动电流；总线忙时本周期报故障并撤销所有待发送邮箱。 */
    if (!BSP_CAN_TxIdle()) {
        BSP_CAN_Abort();
        drive_fault(&drive,FAULT_CAN); right=0; left=0;
    }
    data[0]=(uint8_t)((uint16_t)right>>8); data[1]=(uint8_t)right;
    data[2]=(uint8_t)((uint16_t)left>>8); data[3]=(uint8_t)left;
    return BSP_CAN_Send(0x200,data);
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
        uint32_t now=BSP_System_Millis();
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
        if (!drive_can_update(&drive,BSP_System_Millis())) reply[0]=OTA_BUSY;
        else {
            drive.state=DRIVE_UPDATING;
            drive_tick(&drive,BSP_System_Millis());
            if (!send_current()) reply[0]=OTA_IO_ERROR;
            else {
                uint32_t start=BSP_System_Millis();
                while (!BSP_CAN_TxIdle() && BSP_System_Millis()-start<10U) {}
                if (!BSP_CAN_TxIdle() || BSP_CAN_BusOff()) reply[0]=OTA_IO_ERROR;
                else if (!update_request_set()) reply[0]=OTA_IO_ERROR;
            }
        }
        send_frame(CAR_MSG_OTA_REPLY,seq,reply,16);
        if (!reply[0]) { flush_reply(); BSP_CAN_Stop(); BSP_System_Reset(); }
        if (drive.state==DRIVE_UPDATING) drive_fault(&drive,FAULT_CAN);
    }
}

/** 从 CAN FIFO0 有界读取两台 M2006 的速度与电流反馈，忽略无关或损坏帧。 */
static void poll_can(void)
{
    unsigned count;
    for (count=0;count<8;count++) {
        BSP_CAN_Frame h; const uint8_t *p=h.data; int16_t rpm,current; unsigned wheel;
        int result=BSP_CAN_Read(&h);
        if (!result) break;
        if (result<0) { drive_fault(&drive,FAULT_CAN); break; }
        if (h.extended || h.remote || h.size!=8 || (h.id!=0x201 && h.id!=0x202)) continue;
        rpm=(int16_t)((uint16_t)p[2]<<8|p[3]); current=(int16_t)((uint16_t)p[4]<<8|p[5]);
        wheel=h.id==0x202 ? 0U:1U;
        if (rpm==INT16_MIN || current==INT16_MIN) continue;
        if (wheel==1) { rpm=(int16_t)-rpm; current=(int16_t)-current; }
        raw_speed[wheel]=rpm;
        angle_speed_update(&speed_estimate[wheel],(uint16_t)((uint16_t)p[0]<<8|p[1]),wheel==1 ? -1:1,BSP_System_Millis());
        drive_feedback(&drive,wheel,speed_estimate[wheel].valid ? speed_estimate[wheel].rpm:rpm,current,BSP_System_Millis());
    }
    if (BSP_CAN_Fault()) {
        drive_fault(&drive,FAULT_CAN);
    }
}

/** 以 10 Hz 输出 40 字节运动遥测；转速为方向归一化的电机轴 rpm。 */
static void send_telemetry(void)
{
    uint8_t p[40]={0}; unsigned i; uint32_t now=BSP_System_Millis();
    ota_put32(p,now); ota_put32(p+4,DRIVE_VERSION);
    p[8]=drive.state; p[9]=drive.fault;
    for (i=0;i<2;i++) {
        if (drive.seen[i] && now-drive.feedback_ms[i]<DRIVE_FEEDBACK_MS) p[10]|=(uint8_t)(1U<<i);
        put16(p+12+2*i,drive.target[i]); put16(p+16+2*i,drive.rpm[i]);
        put16(p+20+2*i,drive.output[i]); put16(p+24+2*i,drive.measured_current[i]);
    }
    ota_put32(p+28,drive.session); ota_put32(p+32,drive.sequence); ota_put32(p+36,io_errors+BSP_UART_Errors());
    send_frame(CAR_MSG_TELEMETRY,telemetry_seq++,p,40);
}

/** 输出固定身份 SC01 与错误统计；固件版本通过 QUERY 独立确认。 */
static void send_identity(void)
{
    uint8_t p[16]; memcpy(p,"SC01",4); ota_put32(p+4,BSP_System_Millis());
    ota_put32(p+8,receiver.crc_errors+receiver.header_errors+receiver.timeouts);
    ota_put32(p+12,io_errors+BSP_UART_Errors()); send_frame(CAR_MSG_DIAGNOSTIC,telemetry_seq++,p,16);
}

/** 入口阶段委托BSP建立向量与看门狗。 */
void SmartCar_EarlyInit(void) { BSP_System_EarlyInit(); }

/** 外设初始化后配置接收过滤器和状态机，不启用任何电机运动。 */
void SmartCar_Init(void)
{
    drive_init(&drive); imu_init(&imu); navigation_init(&nav); mpu6050_init(); car_receiver_init(&receiver,on_frame,0);
    /* 电机ID属于应用协议；BSP只执行指定过滤与硬件接收。 */
    if (!BSP_CAN_Start(0x201,0x202) || !BSP_UART_ReceiveStart()) {
        /* 初始化失败不继续业务循环，等待已开启的看门狗复位。 */
        while (1) {}
    }
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
    if (BSP_UART_RecoverRx()) car_receiver_init(&receiver,on_frame,0);
    poll_can(); poll_tx();
    now=BSP_System_Millis();
    if (io_errors+BSP_UART_Errors()!=handled_errors) { handled_errors=io_errors+BSP_UART_Errors(); drive_fault(&drive,FAULT_UART); }
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
    while (budget--) {
        uint8_t byte; if (!BSP_UART_Read(&byte)) break;
        car_receiver_feed(&receiver,byte,BSP_System_Millis());
    }
    now=BSP_System_Millis(); car_receiver_poll(&receiver,now);
    mpu6050_poll(&imu,now,navigation_stationary(&drive,now));
    if (now-telemetry>=100U) { telemetry=now; send_telemetry(); }
    if (now-imu_sent>=(nav.diagnostic ? 20U:100U)) { imu_sent=now; send_imu(now); }
    if (now-identity>=1000U) { identity=now; send_identity(); }
    dump_startup(now); poll_tx(); BSP_System_WatchdogFeed();
}
