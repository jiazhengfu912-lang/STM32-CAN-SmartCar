#include "main.h"
#include "mpu6050.h"
#include <string.h>
I2C_HandleTypeDef hi2c2;
mpu6050_status mpu_status;
static volatile uint8_t completed,failed;
static uint8_t busy,step,buffer[15];
static uint32_t started,retry_at,last_read;
/* 复位、PLL时钟、20Hz低通、100Hz分频、±500dps、±4g。 */
static const uint8_t registers[]={0x6b,0x6b,0x1a,0x19,0x1b,0x1c};
static const uint8_t values[]={0x80,0x01,0x04,0x09,0x08,0x08};

/** 仅复位本外设并释放开漏总线，不等待外部线路恢复，不影响CAN/UART。 */
void mpu6050_init(void)
{
    HAL_NVIC_DisableIRQ(I2C2_EV_IRQn); HAL_NVIC_DisableIRQ(I2C2_ER_IRQn);
    __HAL_RCC_I2C2_CLK_ENABLE(); __HAL_RCC_I2C2_FORCE_RESET(); __HAL_RCC_I2C2_RELEASE_RESET();
    memset(&hi2c2,0,sizeof(hi2c2)); hi2c2.Instance=I2C2;
    hi2c2.Init.ClockSpeed=100000; hi2c2.Init.DutyCycle=I2C_DUTYCYCLE_2;
    hi2c2.Init.AddressingMode=I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode=I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.GeneralCallMode=I2C_GENERALCALL_DISABLE; hi2c2.Init.NoStretchMode=I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2)!=HAL_OK) ++mpu_status.errors;
    HAL_NVIC_ClearPendingIRQ(I2C2_EV_IRQn); HAL_NVIC_ClearPendingIRQ(I2C2_ER_IRQn);
    HAL_NVIC_EnableIRQ(I2C2_EV_IRQn); HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
    busy=completed=failed=step=0; mpu_status.ready=0; retry_at=HAL_GetTick()+100U;
}

/** 中断只记录完成状态，浮点解算全部留在主循环。 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *h) { if (h==&hi2c2) completed=1; }
/** 写寄存器完成后由主循环推进初始化步骤。 */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *h) { if (h==&hi2c2) completed=1; }
/** 中断错误只置位，下一主循环执行有界复位。 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *h) { if (h==&hi2c2) failed=1; }

/** 按本仓库HAL内存传输布局直接启动中断，完全跳过其25ms BUSY自旋。 */
static int start_transfer(uint8_t reg,uint8_t size,int write,uint32_t now)
{
    uint32_t mask=__get_PRIMASK();
    __disable_irq();
    if (hi2c2.State!=HAL_I2C_STATE_READY || hi2c2.Lock!=HAL_UNLOCKED || (I2C2->SR2&I2C_SR2_BUSY) ||
        (GPIOB->IDR&(GPIO_PIN_10|GPIO_PIN_11))!=(GPIO_PIN_10|GPIO_PIN_11)) {
        __set_PRIMASK(mask); return 0;
    }
    completed=failed=0; busy=1; started=now;
    /* 等价于当前HAL_I2C_Mem_*_IT的非等待部分；ISR仍使用厂商实现。 */
    hi2c2.State=write ? HAL_I2C_STATE_BUSY_TX:HAL_I2C_STATE_BUSY_RX;
    hi2c2.Mode=HAL_I2C_MODE_MEM; hi2c2.ErrorCode=HAL_I2C_ERROR_NONE;
    hi2c2.pBuffPtr=buffer; hi2c2.XferCount=size; hi2c2.XferSize=size;
    hi2c2.XferOptions=0xFFFF0000U; /* 本仓库HAL私有常量I2C_NO_OPTION_FRAME。 */
    hi2c2.Devaddress=0x68U<<1; hi2c2.Memaddress=reg; hi2c2.MemaddSize=I2C_MEMADD_SIZE_8BIT;
    hi2c2.EventCount=0;
    I2C2->CR1=(I2C2->CR1&~I2C_CR1_POS)|I2C_CR1_PE;
    if (!write) I2C2->CR1|=I2C_CR1_ACK;
    I2C2->CR1|=I2C_CR1_START;
    __HAL_I2C_ENABLE_IT(&hi2c2,I2C_IT_EVT|I2C_IT_BUF|I2C_IT_ERR);
    __set_PRIMASK(mask);
    return 1;
}

/** 执行一次初始化或新样本读取；每次事务5ms超时，失败后1秒再试。 */
void mpu6050_poll(imu_state *imu,uint32_t now,int stationary)
{
    if ((int32_t)(now-retry_at)<0) return;
    if (failed || (busy && !completed && now-started>=5U)) {
        ++mpu_status.errors; if (!failed) ++mpu_status.timeouts;
        ++mpu_status.restarts; ++imu->gaps; imu->seen=0; imu->calibration=IMU_UNCALIBRATED;
        mpu6050_init(); retry_at=now+1000U; return;
    }
    if (busy) {
        if (!completed) return;
        busy=completed=0;
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
