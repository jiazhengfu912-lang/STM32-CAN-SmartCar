#include "stm32f1xx.h"
#include "stm32_flash.h"
#include "boot_guard.h"
#include <string.h>

#define PAGE_SIZE 1024U
#define FLASH_ERRORS (FLASH_SR_PGERR | FLASH_SR_WRPRTERR)

/** 有界等待 Flash 忙结束并检查编程与写保护错误。 */
static int ready(void)
{
    uint32_t budget = 1000000U;
    while ((FLASH->SR & FLASH_SR_BSY) != 0U && --budget != 0U) { }
    return budget != 0U && (FLASH->SR & FLASH_ERRORS) == 0U;
}

/** 仅在 Flash 空闲时解锁擦写控制寄存器。 */
static int unlock(void)
{
    if ((FLASH->SR & FLASH_SR_BSY) != 0U) return 0;
    FLASH->SR = FLASH_ERRORS | FLASH_SR_EOP;
    if ((FLASH->CR & FLASH_CR_LOCK) != 0U)
    {
        FLASH->KEYR = 0x45670123U;
        FLASH->KEYR = 0xCDEF89ABU;
    }
    return (FLASH->CR & FLASH_CR_LOCK) == 0U;
}

/** 校验页地址后擦除并读回验证，同时接续 App 启动的看门狗。 */
static int erase_page(uint32_t address)
{
    IWDG->KR = 0xAAAAU;
    unsigned i;
    if ((address & (PAGE_SIZE - 1U)) != 0U || address < BOOT_APP_BASE ||
        address > BOOT_META_BASE) return 0;
    FLASH->SR = FLASH_ERRORS | FLASH_SR_EOP;
    FLASH->CR = FLASH_CR_PER;
    FLASH->AR = address;
    FLASH->CR |= FLASH_CR_STRT;
    if (!ready()) { FLASH->CR = 0U; return 0; }
    FLASH->CR = 0U;
    for (i = 0; i < PAGE_SIZE; ++i)
        if (*(const volatile uint8_t *)(address + i) != 0xFFU) return 0;
    return 1;
}

/** 只在 App 或元数据允许范围内逐半字编程并读回验证。 */
static int program(uint32_t address, const uint8_t *data, uint16_t size)
{
    uint16_t i;
    uint32_t rounded = ((uint32_t)size + 1U) & ~1U;
    /* 只能写 App 或这 32 字节元数据，不能到 Bootloader/参数页。 */
    if ((address & 1U) != 0U || size == 0U ||
        !((address >= BOOT_APP_BASE && address < BOOT_APP_LIMIT && rounded <= BOOT_APP_LIMIT - address) ||
          (address >= BOOT_META_BASE && address < BOOT_META_BASE + BOOT_META_SIZE &&
           rounded <= BOOT_META_BASE + BOOT_META_SIZE - address))) return 0;
    for (i = 0U; i < size; i = (uint16_t)(i + 2U))
    {
        uint16_t value = (uint16_t)data[i] | (uint16_t)((i + 1U < size ? data[i + 1U] : 0xFFU) << 8U);
        if (*(const volatile uint16_t *)(address + i) != 0xFFFFU) return 0;
        FLASH->SR = FLASH_ERRORS | FLASH_SR_EOP;
        FLASH->CR = FLASH_CR_PG;
        *(volatile uint16_t *)(address + i) = value;
        if (!ready()) { FLASH->CR = 0U; return 0; }
        FLASH->CR = 0U;
        if (*(const volatile uint16_t *)(address + i) != value) return 0;
    }
    return 1;
}

/** 先擦除元数据页，防止部分新程序被误认成有效 App。 */
static int invalidate(void)
{
    int ok = unlock() && erase_page(BOOT_META_BASE);
    FLASH->CR = FLASH_CR_LOCK;
    return ok;
}

/** 确认有效记录已撤销后逐页擦除目标 App，保留 Bootloader 和参数页。 */
static int erase_app(uint32_t length)
{
    uint32_t offset;
    int ok = 0;
    if (length < 258U || length > BOOT_APP_CAPACITY) return 0;
    /* 必须先证明确实已撤销记录。仅检查 RAM 会话状态不够。 */
    for (offset = 0U; offset < BOOT_META_SIZE; ++offset)
        if (*(const volatile uint8_t *)(BOOT_META_BASE + offset) != 0xFFU) return 0;
    if (unlock())
    {
        ok = 1;
        for (offset = 0U; offset < length; offset += PAGE_SIZE)
            if (!erase_page(BOOT_APP_BASE + offset)) { ok = 0; break; }
    }
    FLASH->CR = FLASH_CR_LOCK;
    return ok;
}

/** 检查相对偏移并写入指定 App 数据块。 */
static int write_app(uint32_t offset, const uint8_t *data, uint16_t size)
{
    int ok;
    if (offset >= BOOT_APP_CAPACITY || size > BOOT_APP_CAPACITY - offset) return 0;
    ok = unlock() && program(BOOT_APP_BASE + offset, data, size);
    FLASH->CR = FLASH_CR_LOCK;
    return ok;
}

/** 比较已写入块与重传数据，确保重复请求内容一致。 */
static int equal_app(uint32_t offset, const uint8_t *data, uint16_t size)
{
    return offset < BOOT_APP_CAPACITY && size <= BOOT_APP_CAPACITY - offset &&
           memcmp((const void *)(BOOT_APP_BASE + offset), data, size) == 0;
}

/** 完整校验 App，先写元数据头、最后写提交标记并再次验证。 */
static int finish(uint32_t length, uint32_t crc, uint32_t version)
{
    uint8_t record[BOOT_META_SIZE];
    int ok = 0;
    ota_put32(record, BOOT_MAGIC);
    ota_put32(record + 4U, BOOT_FORMAT);
    ota_put32(record + 8U, BOOT_APP_BASE);
    ota_put32(record + 12U, length);
    ota_put32(record + 16U, crc);
    ota_put32(record + 20U, version);
    ota_put32(record + 24U, car_crc32(record, 24U));
    ota_put32(record + 28U, BOOT_COMMITTED);
    if (boot_validate(record, (const uint8_t *)BOOT_APP_BASE, BOOT_APP_CAPACITY) != BOOT_VALID)
        return 0;
    if (unlock() && program(BOOT_META_BASE, record, 28U) &&
        memcmp((const void *)BOOT_META_BASE, record, 28U) == 0)
        ok = program(BOOT_META_BASE + 28U, record + 28U, 4U);
    FLASH->CR = FLASH_CR_LOCK;
    return ok && boot_validate((const uint8_t *)BOOT_META_BASE,
                              (const uint8_t *)BOOT_APP_BASE, BOOT_APP_CAPACITY) == BOOT_VALID;
}

const ota_storage stm32_ota_storage = { invalidate, erase_app, write_app, equal_app, finish };
