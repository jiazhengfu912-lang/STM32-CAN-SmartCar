#ifndef BSP_SYSTEM_H
#define BSP_SYSTEM_H
#include <stdint.h>
void BSP_System_EarlyInit(void);
void BSP_System_ClockInit(void);
uint32_t BSP_System_Millis(void);
void BSP_System_WatchdogFeed(void);
void BSP_System_Reset(void);
#endif
