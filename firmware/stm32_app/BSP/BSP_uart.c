#include "main.h"
#include "BSP_uart.h"
static UART_HandleTypeDef huart1;
static uint8_t rx_byte;
static volatile uint8_t rx_ring[512];
static volatile uint16_t rx_head,rx_tail;
static volatile uint32_t io_errors;
/** 初始化板载USART1，收发参数不受业务帧影响。 */
int BSP_UART_Init(void)
{
    huart1.Instance=USART1; huart1.Init.BaudRate=115200; huart1.Init.WordLength=UART_WORDLENGTH_8B;
    huart1.Init.StopBits=UART_STOPBITS_1; huart1.Init.Parity=UART_PARITY_NONE;
    huart1.Init.Mode=UART_MODE_TX_RX; huart1.Init.HwFlowCtl=UART_HWCONTROL_NONE;
    huart1.Init.OverSampling=UART_OVERSAMPLING_16;
    return HAL_UART_Init(&huart1)==HAL_OK;
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
/** 单字节接收完成后推进环形队列；溢出只累计传输错误，不处理业务故障。 */
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

/** 恢复接收硬件并丢弃未消费字节，返回值通知上层重置解析器。 */
int BSP_UART_RecoverRx(void)
{
    int recovered=0; uint32_t primask=__get_PRIMASK();
    __disable_irq();
    if (huart1.RxState==HAL_UART_STATE_READY && huart1.Lock==HAL_UNLOCKED) {
        /* Tx 初始化短暂持锁时，接收回调可能返回 HAL_BUSY；此处在发送之外重试。 */
        rx_tail=rx_head;
        recovered=1;
        __HAL_UART_CLEAR_OREFLAG(&huart1);
        (void)HAL_UART_Receive_IT(&huart1,&rx_byte,1);
    }
    __set_PRIMASK(primask); return recovered;
}
/** 开启单字节中断接收。 */
int BSP_UART_ReceiveStart(void) { return HAL_UART_Receive_IT(&huart1,&rx_byte,1)==HAL_OK; }
/** 从中断接收缓冲读取一个字节，不解析协议。 */
int BSP_UART_Read(uint8_t *byte) {
    if (rx_tail==rx_head) return 0;
    *byte=rx_ring[rx_tail]; rx_tail=(uint16_t)((rx_tail+1U)%512U); return 1;
}
/** 返回接收硬件及缓冲溢出计数，不决定停车策略。 */
uint32_t BSP_UART_Errors(void) { return io_errors; }
/** 查询发送外设就绪状态。 */
int BSP_UART_TxReady(void) { return huart1.gState==HAL_UART_STATE_READY; }
/** 原子启动一次异步发送，调用方须保持缓冲有效直到完成或中止。 */
int BSP_UART_Transmit(uint8_t *data,uint16_t size) {
    uint32_t mask=__get_PRIMASK(); HAL_StatusTypeDef status;
    __disable_irq(); status=HAL_UART_Transmit_IT(&huart1,data,size); __set_PRIMASK(mask);
    return status==HAL_OK;
}
/** 有界中止当前发送，保留接收通道。 */
void BSP_UART_AbortTx(void) {
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    (void)HAL_UART_AbortTransmit(&huart1); __set_PRIMASK(mask);
}
/** 将USART1硬件中断分发给HAL。 */
void BSP_UART_IRQHandler(void) { HAL_UART_IRQHandler(&huart1); }
