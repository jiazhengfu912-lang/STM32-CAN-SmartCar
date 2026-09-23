#include "BSP_i2c.h"
#include "BSP_system.h"
#include "mpu6050.h"
mpu6050_status mpu_status;
static uint8_t failed;
static uint8_t busy,step,buffer[15];
static uint32_t started,retry_at,last_read;
/* 复位、PLL时钟、20Hz低通、100Hz分频、±500dps、±4g。 */
static const uint8_t registers[]={0x6b,0x6b,0x1a,0x19,0x1b,0x1c};
static const uint8_t values[]={0x80,0x01,0x04,0x09,0x08,0x08};

/** 重置总线后重新开始传感器初始化，等待时间属于设备策略。 */
void mpu6050_init(void) {
    if (!BSP_I2C_Reset()) ++mpu_status.errors;
    busy=failed=step=0; mpu_status.ready=0; retry_at=BSP_System_Millis()+100U;
}
/** 启动传感器寄存器事务并记录上层超时起点。 */
static int start_transfer(uint8_t reg,uint8_t size,int write,uint32_t now) {
    if (!BSP_I2C_Start(0x68,reg,buffer,size,write)) return 0;
    busy=1; started=now; return 1;
}

/** 执行一次初始化或新样本读取；每次事务5ms超时，失败后1秒再试。 */
void mpu6050_poll(imu_state *imu,uint32_t now,int stationary)
{
    if ((int32_t)(now-retry_at)<0) return;
    if (failed || BSP_I2C_Failed() || (busy && !BSP_I2C_Completed() && now-started>=5U)) {
        ++mpu_status.errors; if (!failed && !BSP_I2C_Failed()) ++mpu_status.timeouts;
        ++mpu_status.restarts; ++imu->gaps; imu->seen=0; imu->calibration=IMU_UNCALIBRATED;
        mpu6050_init(); retry_at=now+1000U; return;
    }
    if (busy) {
        if (!BSP_I2C_Completed()) return;
        busy=0;
        if (step<6) { ++step; if (step<=2) { retry_at=now+100U; return; } }
        else if (step==6) {
            mpu_status.identity=buffer[0];
            if (buffer[0]!=0x68) { failed=1; return; }
            step=7; mpu_status.ready=1;
        } else if (buffer[0]&1U) imu_sample(imu,buffer+1,now,stationary);
    }
    if (step<6) { buffer[0]=values[step]; if (!start_transfer(registers[step],1,1,now)) failed=1; }
    else if (step==6) { if (!start_transfer(0x75,1,0,now)) failed=1; }
    else if (now-last_read>=10U) {
        last_read=now;
        /* INT_STATUS紧邻14字节数据块；未置DATA_RDY则不积分旧样本。 */
        if (!start_transfer(0x3a,15,0,now)) failed=1;
    }
    if (imu->calibration==IMU_CALIBRATING && (!stationary || !imu_fresh(imu,now))) imu->calibration=IMU_CAL_FAILED;
}
