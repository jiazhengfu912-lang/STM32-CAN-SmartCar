/* USER CODE BEGIN Header */
/* SmartCar HAL 入口；业务保存在 smartcar.c / drive.c，CubeMX 用户区保留挂接点。 */
/* USER CODE END Header */
#include "main.h"
/* USER CODE BEGIN Includes */
#include "smartcar.h"
#include "BSP_system.h"
#include "BSP_can.h"
#include "BSP_uart.h"
/* USER CODE END Includes */
void SystemClock_Config(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);

/** 程序入口：初始化 HAL 与外设后执行独立的小车业务主循环。 */
int main(void)
{
    /* USER CODE BEGIN 1 */
    SmartCar_EarlyInit();
    /* USER CODE END 1 */
    HAL_Init();
    SystemClock_Config();
    MX_CAN_Init();
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */
    SmartCar_Init();
    /* USER CODE END 2 */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */
        SmartCar_Poll();
    }
    /* USER CODE END 3 */
}

/** 将当前PCB时钟配置委托给BSP。 */
void SystemClock_Config(void) { BSP_System_ClockInit(); }
/** 将当前PCB CAN初始化委托给BSP。 */
static void MX_CAN_Init(void) { if (!BSP_CAN_Init()) Error_Handler(); }
/** 将网关连接串口初始化委托给BSP。 */
static void MX_USART1_UART_Init(void) { if (!BSP_UART_Init()) Error_Handler(); }

/** 初始化失败时由独立看门狗复位，复位后不会自动使能驱动。 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1) {}
    /* USER CODE END Error_Handler_Debug */
}
