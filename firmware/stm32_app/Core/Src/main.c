/* USER CODE BEGIN Header */
/* SmartCar HAL 入口；业务保存在 smartcar.c / drive.c，CubeMX 用户区保留挂接点。 */
/* USER CODE END Header */
#include "main.h"
/* USER CODE BEGIN Includes */
#include "smartcar.h"
/* USER CODE END Includes */
CAN_HandleTypeDef hcan;
UART_HandleTypeDef huart1;
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

/** 用原板 8 MHz HSE 配置 72 MHz 主时钟和 36 MHz APB1，保持 CAN 1 Mbit/s。 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc={0}; RCC_ClkInitTypeDef clock={0};
    osc.OscillatorType=RCC_OSCILLATORTYPE_HSE; osc.HSEState=RCC_HSE_ON;
    osc.HSEPredivValue=RCC_HSE_PREDIV_DIV1; osc.HSIState=RCC_HSI_ON;
    osc.PLL.PLLState=RCC_PLL_ON; osc.PLL.PLLSource=RCC_PLLSOURCE_HSE; osc.PLL.PLLMUL=RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&osc)!=HAL_OK) Error_Handler();
    clock.ClockType=RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    clock.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK; clock.AHBCLKDivider=RCC_SYSCLK_DIV1;
    clock.APB1CLKDivider=RCC_HCLK_DIV2; clock.APB2CLKDivider=RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clock,FLASH_LATENCY_2)!=HAL_OK) Error_Handler();
}

/** 初始化 HAL CAN 并仅放行标准数据帧 0x201、0x202 至 FIFO0。 */
static void MX_CAN_Init(void)
{
    hcan.Instance=CAN1; hcan.Init.Prescaler=6; hcan.Init.Mode=CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth=CAN_SJW_1TQ; hcan.Init.TimeSeg1=CAN_BS1_3TQ; hcan.Init.TimeSeg2=CAN_BS2_2TQ;
    hcan.Init.AutoBusOff=ENABLE; hcan.Init.AutoRetransmission=DISABLE;
    if (HAL_CAN_Init(&hcan)!=HAL_OK) Error_Handler();
}

/** 初始化 PA9/PA10 的 115200 8N1 通道，接收中断只负责搬运字节。 */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance=USART1; huart1.Init.BaudRate=115200; huart1.Init.WordLength=UART_WORDLENGTH_8B;
    huart1.Init.StopBits=UART_STOPBITS_1; huart1.Init.Parity=UART_PARITY_NONE;
    huart1.Init.Mode=UART_MODE_TX_RX; huart1.Init.HwFlowCtl=UART_HWCONTROL_NONE;
    huart1.Init.OverSampling=UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1)!=HAL_OK) Error_Handler();
}

/** 初始化失败时由独立看门狗复位，复位后不会自动使能驱动。 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1) {}
    /* USER CODE END Error_Handler_Debug */
}
