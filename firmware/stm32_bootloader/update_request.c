#include "stm32f1xx.h"
#include "update_request.h"

/* DR1/DR2 用作一次性升级请求；这是启动选择，不是 App 有效记录。 */
static void access_backup(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
    (void)RCC->APB1ENR;
    PWR->CR |= PWR_CR_DBP;
    (void)PWR->CR;
}

/** 写入并读回一次性升级标记，供复位后的 Bootloader 消费。 */
int update_request_set(void)
{
    int ok;
    access_backup();
    BKP->DR1 = 0x4F54U;
    BKP->DR2 = 0xB0ABU;
    ok = BKP->DR1 == 0x4F54U && BKP->DR2 == 0xB0ABU;
    PWR->CR &= ~PWR_CR_DBP;
    return ok;
}

/** 读取并清除一次性升级标记；不把它当作 App 有效证明。 */
int update_request_take(void)
{
    int requested;
    access_backup();
    requested = BKP->DR1 == 0x4F54U && BKP->DR2 == 0xB0ABU;
    if (requested) { BKP->DR1 = 0U; BKP->DR2 = 0U; }
    PWR->CR &= ~PWR_CR_DBP;
    return requested;
}
