#ifndef BSP_CAN_H
#define BSP_CAN_H
#include <stdint.h>
typedef struct { uint32_t id; uint8_t extended,remote,size,data[8]; } BSP_CAN_Frame;
int BSP_CAN_Init(void);
int BSP_CAN_Start(uint16_t id0,uint16_t id1);
/* 返回1有帧、0无帧、-1硬件错误；不解码电机协议。 */
int BSP_CAN_Read(BSP_CAN_Frame *frame);
int BSP_CAN_Send(uint16_t id,const uint8_t data[8]);
int BSP_CAN_TxIdle(void);
void BSP_CAN_Abort(void);
int BSP_CAN_Fault(void);
int BSP_CAN_BusOff(void);
void BSP_CAN_Stop(void);
#endif
