#include "main.h"
#include "BSP_system.h"
/** 用原板 8 MHz HSE 配置 72 MHz 主时钟和 36 MHz APB1，保持 CAN 1 Mbit/s。 */
void BSP_System_ClockInit(void)
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

/** 启动独立看门狗，典型约 1 秒；仅完整主循环喂狗，Bootloader 也需支持接续。 */
static void watchdog_start(void)
{
    /* 与 HAL_IWDG_Init 相同：先启动 IWDG，硬件才自动开启 LSI 更新时钟。 */
    IWDG->KR=0xCCCC;
    IWDG->KR=0x5555; IWDG->PR=3; IWDG->RLR=1249;
    /* 若更新时钟异常，已运行的看门狗会复位，不在这里持续喂狗。 */
    while (IWDG->SR) {} IWDG->KR=0xAAAA;
}
/** 在 HAL 初始化前设置 App 向量并接续看门狗；复位后的 NVIC 尚未启用外设中断。 */
void BSP_System_EarlyInit(void)
{
    SCB->VTOR=0x08004000U; __DSB(); __ISB(); __enable_irq(); watchdog_start();
}
/** 返回板载HAL毫秒时基，不决定业务调度周期。 */
uint32_t BSP_System_Millis(void) { return HAL_GetTick(); }
/** 喂独立看门狗；何时喂狗由应用决定。 */
void BSP_System_WatchdogFeed(void) { IWDG->KR=0xAAAA; }
/** 触发系统复位；升级条件由上层判断。 */
void BSP_System_Reset(void) { NVIC_SystemReset(); }
/** 开启基础时钟并仅保留 SWD 调试端口，释放 JTAG 引脚。 */
void HAL_MspInit(void)
{
    __HAL_RCC_AFIO_CLK_ENABLE(); __HAL_RCC_PWR_CLK_ENABLE();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2); __HAL_AFIO_REMAP_SWJ_NOJTAG();
}
