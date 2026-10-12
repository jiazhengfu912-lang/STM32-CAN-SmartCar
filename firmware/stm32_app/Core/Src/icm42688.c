#include "BSP_i2c.h"
#include "BSP_system.h"
#include "icm42688.h"
#include <string.h>

icm42688_status icm_status;
enum { BANK, IDENTITY, RESET, RESET_STATUS, CONFIG_WRITE, CONFIG_READ,
       POWER_WRITE, POWER_READ, DATA_STATUS, DATA_SAMPLE };
static uint8_t failed,busy,phase,index,buffer[14];
static uint32_t started,retry_at,last_read;
/* DS-000347：大端/I2C、±500dps/±4g/100Hz、二阶UI低通约19.2Hz。
 * 保留GYRO_CONFIG1的保留位及DEC2设置；关闭FSYNC标记。
 * DRDY采用锁存，状态或数据读取均清除，INT1只用于产生就绪状态。 */
static const uint8_t registers[]={0x4c,0x4f,0x50,0x51,0x52,0x53,0x62,0x63,0x14,0x64,0x65};
static const uint8_t values[]   ={0x32,0x48,0x48,0x16,0x66,0x0d,0x00,0x30,0x07,0x00,0x08};

/** 重置I2C2并从0x68开始识别；上电等待不阻塞控制主循环。 */
void icm42688_init(void)
{
    if (!BSP_I2C_Reset()) ++icm_status.errors;
    busy=failed=index=0; phase=BANK; icm_status.ready=icm_status.identity=0;
    icm_status.address=0x68; retry_at=BSP_System_Millis()+100U; last_read=0;
}

/** 启动一次有界异步寄存器事务，BSP只承担原始收发。 */
static int start_transfer(uint8_t reg,uint8_t size,int write,uint32_t now)
{
    if (!BSP_I2C_Start(icm_status.address,reg,buffer,size,write)) return 0;
    busy=1; started=now; return 1;
}

/** 仅在身份识别期间尝试另一个AD0地址，不给错误型号写配置。 */
static int next_address(uint32_t now)
{
    if (phase>IDENTITY || icm_status.address!=0x68) return 0;
    if (!BSP_I2C_Reset()) ++icm_status.errors;
    busy=failed=0; phase=BANK; icm_status.address=0x69; retry_at=now+1U;
    return 1;
}

/** 失效样本与零偏，保留累计诊断；一秒后重试，不自动恢复导航。 */
static void recover(imu_state *imu,uint32_t now)
{
    ++icm_status.restarts; ++imu->gaps;
    imu->seen=imu->attitude_ready=0; imu->calibration=IMU_UNCALIBRATED;
    memset(imu->bias,0,sizeof(imu->bias));
    icm42688_init(); retry_at=now+1000U;
}

/** 推进初始化/配置读回/DRDY采样；事务5ms超时，陀螺仪启用后等50ms。 */
void icm42688_poll(imu_state *imu,uint32_t now,int stationary)
{
    unsigned i;
    if (imu->calibration==IMU_CALIBRATING && (!stationary || !imu_fresh(imu,now)))
        imu->calibration=IMU_CAL_FAILED;
    if ((int32_t)(now-retry_at)<0) return;
    if (failed || BSP_I2C_Failed() || (busy && !BSP_I2C_Completed() && now-started>=5U)) {
        ++icm_status.errors;
        if (!failed && !BSP_I2C_Failed()) ++icm_status.timeouts;
        if (!next_address(now)) recover(imu,now);
        return;
    }
    if (busy) {
        if (!BSP_I2C_Completed()) return;
        busy=0;
        switch (phase) {
        case BANK: phase=IDENTITY; break;
        case IDENTITY:
            icm_status.identity=buffer[0];
            if (buffer[0]!=0x47) { failed=1; return; }
            phase=RESET; break;
        case RESET:
            phase=RESET_STATUS; retry_at=now+2U; return; /* 手册要求至少1ms。 */
        case RESET_STATUS:
            if (!(buffer[0]&0x10U)) { failed=1; return; }
            phase=CONFIG_WRITE; break;
        case CONFIG_WRITE: phase=CONFIG_READ; break;
        case CONFIG_READ:
            if (buffer[0]!=values[index]) { failed=1; return; }
            phase=(++index<sizeof(registers)) ? CONFIG_WRITE:POWER_WRITE; break;
        case POWER_WRITE:
            phase=POWER_READ; retry_at=now+50U; return; /* >200us禁写、>45ms最短开启。 */
        case POWER_READ:
            if (buffer[0]!=0x0f) { failed=1; return; }
            icm_status.ready=1; phase=DATA_STATUS; last_read=now-10U; break;
        case DATA_STATUS:
            if (!(buffer[0]&0x08U)) return; /* DATA_RDY不是MPU的bit0。 */
            phase=DATA_SAMPLE; break;
        case DATA_SAMPLE:
            /* 复位/未启用传感器的0x8000占位不得作为有效数据积分。 */
            for (i=0;i<7;i++) if (buffer[2*i]==0x80 && buffer[2*i+1]==0) { failed=1; return; }
            imu_sample(imu,buffer,now,stationary); phase=DATA_STATUS; break;
        }
    }
    switch (phase) {
    case BANK: buffer[0]=0; if (!start_transfer(0x76,1,1,now)) failed=1; break;
    case IDENTITY: if (!start_transfer(0x75,1,0,now)) failed=1; break;
    case RESET: buffer[0]=1; if (!start_transfer(0x11,1,1,now)) failed=1; break;
    case RESET_STATUS: if (!start_transfer(0x2d,1,0,now)) failed=1; break;
    case CONFIG_WRITE:
        buffer[0]=values[index]; if (!start_transfer(registers[index],1,1,now)) failed=1; break;
    case CONFIG_READ: if (!start_transfer(registers[index],1,0,now)) failed=1; break;
    case POWER_WRITE: buffer[0]=0x0f; if (!start_transfer(0x4e,1,1,now)) failed=1; break;
    case POWER_READ: if (!start_transfer(0x4e,1,0,now)) failed=1; break;
    case DATA_STATUS:
        if (now-last_read>=10U) { last_read=now; if (!start_transfer(0x2d,1,0,now)) failed=1; }
        break;
    case DATA_SAMPLE: if (!start_transfer(0x1d,14,0,now)) failed=1; break;
    }
}
