#include "main.h"
#include "BSP_i2c.h"
#include <string.h>
I2C_HandleTypeDef hi2c2;
static volatile uint8_t completed,failed;
/** 复位板载I2C2，不执行传感器配置、重试或采样策略。 */
int BSP_I2C_Reset(void) {
    int ok;
    HAL_NVIC_DisableIRQ(I2C2_EV_IRQn); HAL_NVIC_DisableIRQ(I2C2_ER_IRQn);
    __HAL_RCC_I2C2_CLK_ENABLE(); __HAL_RCC_I2C2_FORCE_RESET(); __HAL_RCC_I2C2_RELEASE_RESET();
    memset(&hi2c2,0,sizeof(hi2c2)); hi2c2.Instance=I2C2;
    hi2c2.Init.ClockSpeed=100000; hi2c2.Init.DutyCycle=I2C_DUTYCYCLE_2;
    hi2c2.Init.AddressingMode=I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode=I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.GeneralCallMode=I2C_GENERALCALL_DISABLE; hi2c2.Init.NoStretchMode=I2C_NOSTRETCH_DISABLE;
    ok=HAL_I2C_Init(&hi2c2)==HAL_OK;
    HAL_NVIC_ClearPendingIRQ(I2C2_EV_IRQn); HAL_NVIC_ClearPendingIRQ(I2C2_ER_IRQn);
    HAL_NVIC_EnableIRQ(I2C2_EV_IRQn); HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
    completed=failed=0; return ok;
}
/** I2C2专用单主机总线，PB10/PB11开漏，外部上拉到3.3V。 */
void HAL_I2C_MspInit(I2C_HandleTypeDef *i2c)
{
    GPIO_InitTypeDef p={0}; if (i2c->Instance!=I2C2) return;
    __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_I2C2_CLK_ENABLE();
    p.Pin=GPIO_PIN_10|GPIO_PIN_11; p.Mode=GPIO_MODE_AF_OD; p.Pull=GPIO_NOPULL;
    p.Speed=GPIO_SPEED_FREQ_HIGH; HAL_GPIO_Init(GPIOB,&p);
    HAL_NVIC_SetPriority(I2C2_EV_IRQn,2,0); HAL_NVIC_SetPriority(I2C2_ER_IRQn,2,0);
}
/** 中断只记录完成状态，浮点解算全部留在主循环。 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *h) { if (h==&hi2c2) completed=1; }
/** 写寄存器完成后由主循环推进初始化步骤。 */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *h) { if (h==&hi2c2) completed=1; }
/** 中断错误只置位，下一主循环执行有界复位。 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *h) { if (h==&hi2c2) failed=1; }

/** 按本仓库HAL内存传输布局直接启动中断，完全跳过其25ms BUSY自旋。 */
int BSP_I2C_Start(uint8_t address,uint8_t reg,uint8_t *buffer,uint8_t size,int write)
{
    uint32_t mask=__get_PRIMASK();
    __disable_irq();
    if (hi2c2.State!=HAL_I2C_STATE_READY || hi2c2.Lock!=HAL_UNLOCKED || (I2C2->SR2&I2C_SR2_BUSY) ||
        (GPIOB->IDR&(GPIO_PIN_10|GPIO_PIN_11))!=(GPIO_PIN_10|GPIO_PIN_11)) {
        __set_PRIMASK(mask); return 0;
    }
    completed=failed=0;
    /* 等价于当前HAL_I2C_Mem_*_IT的非等待部分；ISR仍使用厂商实现。 */
    hi2c2.State=write ? HAL_I2C_STATE_BUSY_TX:HAL_I2C_STATE_BUSY_RX;
    hi2c2.Mode=HAL_I2C_MODE_MEM; hi2c2.ErrorCode=HAL_I2C_ERROR_NONE;
    hi2c2.pBuffPtr=buffer; hi2c2.XferCount=size; hi2c2.XferSize=size;
    hi2c2.XferOptions=0xFFFF0000U; /* 本仓库HAL私有常量I2C_NO_OPTION_FRAME。 */
    hi2c2.Devaddress=(uint16_t)address<<1; hi2c2.Memaddress=reg; hi2c2.MemaddSize=I2C_MEMADD_SIZE_8BIT;
    hi2c2.EventCount=0;
    I2C2->CR1=(I2C2->CR1&~I2C_CR1_POS)|I2C_CR1_PE;
    if (!write) I2C2->CR1|=I2C_CR1_ACK;
    I2C2->CR1|=I2C_CR1_START;
    __HAL_I2C_ENABLE_IT(&hi2c2,I2C_IT_EVT|I2C_IT_BUF|I2C_IT_ERR);
    __set_PRIMASK(mask);
    return 1;
}
/** 查询事务完成，不解读数据内容。 */
int BSP_I2C_Completed(void) { return completed; }
/** 查询事务硬件错误。 */
int BSP_I2C_Failed(void) { return failed; }
/** 分发I2C2事件中断。 */
void BSP_I2C_EventIRQ(void) { HAL_I2C_EV_IRQHandler(&hi2c2); }
/** 分发I2C2错误中断。 */
void BSP_I2C_ErrorIRQ(void) { HAL_I2C_ER_IRQHandler(&hi2c2); }
