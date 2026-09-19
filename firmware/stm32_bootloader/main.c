#include "stm32f1xx.h"
#include "boot_guard.h"
#if defined(M2_UART_UPDATE)
#include "update_request.h"
#endif

int diagnostic_main(void);
/* 在 map 文件中定位后可用 SWD 读取；仅作诊断，不是持久化有效记录。 */
volatile uint32_t boot_reason;

/* 改 MSP 后不能再执行依赖旧栈的 C 尾声，最后两步用无栈汇编完成。 */
__attribute__((naked, noreturn)) static void enter_app(uint32_t stack, uint32_t reset)
{
    __asm volatile (
        "movs r2, #0\n"
        "msr control, r2\n"
        "isb\n"
        "msr msp, r0\n"
        "bx r1\n");
}

/** 完成启动初始化并运行主循环；具体硬件职责见本文件的模块说明。 */
int main(void)
{
    unsigned i;
    int requested = 0;
    __disable_irq();
#if defined(M2_UART_UPDATE)
    requested = update_request_take();
#endif
    boot_reason = (uint32_t)boot_validate((const uint8_t *)BOOT_META_BASE,
                                        (const uint8_t *)BOOT_APP_BASE,
                                        BOOT_APP_CAPACITY);
    if (boot_reason == BOOT_VALID && !requested)
    {
        /* 启动检查发生在启用 UART/CAN/SysTick 之前；交接保留复位后的 HSI。 */
        SysTick->CTRL = 0U;
        SysTick->LOAD = 0U;
        SysTick->VAL = 0U;
        for (i = 0U; i < 8U; ++i)
        {
            NVIC->ICER[i] = 0xFFFFFFFFU;
            NVIC->ICPR[i] = 0xFFFFFFFFU;
        }
        SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
        SCB->VTOR = BOOT_APP_BASE;
        __DSB();
        __ISB();
        enter_app(*(const uint32_t *)BOOT_APP_BASE,
                  *(const uint32_t *)(BOOT_APP_BASE + 4U));
    }
    /* M2A 为只读诊断，M2_UART_UPDATE 构建另启用分块接收器。 */
    return diagnostic_main();
}
