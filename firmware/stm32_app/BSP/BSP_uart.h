#ifndef BSP_UART_H
#define BSP_UART_H
#include <stdint.h>
int BSP_UART_Init(void);
int BSP_UART_ReceiveStart(void);
int BSP_UART_Read(uint8_t *byte);
int BSP_UART_RecoverRx(void);
uint32_t BSP_UART_Errors(void);
int BSP_UART_TxReady(void);
int BSP_UART_Transmit(uint8_t *data,uint16_t size);
void BSP_UART_AbortTx(void);
void BSP_UART_IRQHandler(void);
#endif
