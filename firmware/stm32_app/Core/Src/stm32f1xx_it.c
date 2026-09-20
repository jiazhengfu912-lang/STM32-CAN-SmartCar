#include "main.h"
extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c2;
/** 分发I2C2事件，优先级低于控制串口。 */
void I2C2_EV_IRQHandler(void) { HAL_I2C_EV_IRQHandler(&hi2c2); }
/** 分发I2C2错误，由主循环执行恢复。 */
void I2C2_ER_IRQHandler(void) { HAL_I2C_ER_IRQHandler(&hi2c2); }
/** 提供 HAL 毫秒时基；中断本身不喂狗。 */
void SysTick_Handler(void) { HAL_IncTick(); }
/** 将串口中断分发至 HAL，由接收回调保存字节。 */
void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&huart1); }
/** 硬件故障后等待独立看门狗复位，禁止继续运行不可信状态。 */
void HardFault_Handler(void) { while (1) {} }
