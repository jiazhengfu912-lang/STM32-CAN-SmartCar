#ifndef BSP_I2C_H
#define BSP_I2C_H
#include <stdint.h>
int BSP_I2C_Reset(void);
int BSP_I2C_Start(uint8_t address,uint8_t reg,uint8_t *buffer,uint8_t size,int write);
int BSP_I2C_Completed(void);
int BSP_I2C_Failed(void);
void BSP_I2C_EventIRQ(void);
void BSP_I2C_ErrorIRQ(void);
#endif
