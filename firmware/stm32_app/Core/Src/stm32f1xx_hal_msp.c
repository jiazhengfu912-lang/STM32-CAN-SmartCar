#include "main.h"
/** I2C2专用单主机总线，PB10/PB11开漏，外部上拉到3.3V。 */
void HAL_I2C_MspInit(I2C_HandleTypeDef *i2c)
{
    GPIO_InitTypeDef p={0}; if (i2c->Instance!=I2C2) return;
    __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_I2C2_CLK_ENABLE();
    p.Pin=GPIO_PIN_10|GPIO_PIN_11; p.Mode=GPIO_MODE_AF_OD; p.Pull=GPIO_NOPULL;
    p.Speed=GPIO_SPEED_FREQ_HIGH; HAL_GPIO_Init(GPIOB,&p);
    HAL_NVIC_SetPriority(I2C2_EV_IRQn,2,0); HAL_NVIC_SetPriority(I2C2_ER_IRQn,2,0);
}
/** 开启基础时钟并仅保留 SWD 调试端口，释放 JTAG 引脚。 */
void HAL_MspInit(void)
{
    __HAL_RCC_AFIO_CLK_ENABLE(); __HAL_RCC_PWR_CLK_ENABLE();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2); __HAL_AFIO_REMAP_SWJ_NOJTAG();
}
/** 配置 CAN1 的 PA11 接收和 PA12 发送；使用主循环读取 FIFO，不开 CAN IRQ。 */
void HAL_CAN_MspInit(CAN_HandleTypeDef *can)
{
    GPIO_InitTypeDef p={0}; if (can->Instance!=CAN1) return;
    __HAL_RCC_CAN1_CLK_ENABLE(); __HAL_RCC_GPIOA_CLK_ENABLE();
    p.Pin=GPIO_PIN_11; p.Mode=GPIO_MODE_INPUT; p.Pull=GPIO_NOPULL; HAL_GPIO_Init(GPIOA,&p);
    p.Pin=GPIO_PIN_12; p.Mode=GPIO_MODE_AF_PP; p.Speed=GPIO_SPEED_FREQ_HIGH; HAL_GPIO_Init(GPIOA,&p);
}
/** 配置 USART1 与接收中断，PA10 上拉避免 ESP32 断电时输入悬空。 */
void HAL_UART_MspInit(UART_HandleTypeDef *uart)
{
    GPIO_InitTypeDef p={0}; if (uart->Instance!=USART1) return;
    __HAL_RCC_USART1_CLK_ENABLE(); __HAL_RCC_GPIOA_CLK_ENABLE();
    p.Pin=GPIO_PIN_9; p.Mode=GPIO_MODE_AF_PP; p.Speed=GPIO_SPEED_FREQ_HIGH; HAL_GPIO_Init(GPIOA,&p);
    p.Pin=GPIO_PIN_10; p.Mode=GPIO_MODE_INPUT; p.Pull=GPIO_PULLUP; HAL_GPIO_Init(GPIOA,&p);
    HAL_NVIC_SetPriority(USART1_IRQn,1,0); HAL_NVIC_EnableIRQ(USART1_IRQn);
}
