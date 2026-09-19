#ifndef SMARTCAR_APP_H
#define SMARTCAR_APP_H
/* 设置向量并启动看门狗，必须放在 main 的 USER CODE BEGIN 1。 */
void SmartCar_EarlyInit(void);
/* 外设初始化完成后初始化业务状态。 */
void SmartCar_Init(void);
/* 在 while(1) 内持续执行一次有界业务循环。 */
void SmartCar_Poll(void);
#endif
