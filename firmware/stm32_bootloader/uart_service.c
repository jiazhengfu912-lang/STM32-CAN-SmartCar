#include "stm32f1xx.h"
#if defined(M2_BOOT)
/* Bootloader 的串口服务使用独立入口，避免与启动校验 main 冲突。 */
#define main diagnostic_main
#endif
#include "car_protocol.h"
#include <string.h>
#if defined(M2_UART_UPDATE)
#include "ota_receiver.h"
#include "update_request.h"
#if defined(M2_BOOT)
#include "stm32_flash.h"
static ota_receiver updater;
#endif
#ifndef DIAG_APP_VERSION
#define DIAG_APP_VERSION 2U
#endif
#endif

#if defined(M2_APP)
#include "boot_layout.h"
#define DIAG_VECTOR_BASE BOOT_APP_BASE
#if defined(M2_UART_UPDATE)
#if DIAG_APP_VERSION == 3
#define DIAG_TAG "M2A3"
#else
#define DIAG_TAG "M2A2"
#endif
#else
#define DIAG_TAG "M2A1"
#endif
#elif defined(M2_BOOT)
#define DIAG_VECTOR_BASE FLASH_BASE
#if defined(M2_UART_UPDATE)
#define DIAG_TAG "M2B2"
#else
#define DIAG_TAG "M2B1"
#endif
#else
#define DIAG_VECTOR_BASE FLASH_BASE
#define DIAG_TAG "M1S1"
#endif

/* 独立的通信诊断固件：只启用 SysTick 和 USART1，不启用 CAN 或电机控制。 */
#define RX_CAPACITY 512U
static volatile uint32_t tick_ms;
static volatile uint8_t rx_bytes[RX_CAPACITY];
static volatile uint16_t rx_head, rx_tail;
static volatile uint32_t io_errors;
static uint32_t tx_errors;
static car_receiver receiver;
static uint8_t tx_frame[CAR_MAX_FRAME];

/** 维护 Bootloader 毫秒时基，不在中断里执行擦写。 */
void SysTick_Handler(void)
{
    ++tick_ms;
}

/** 读取串口状态与字节，保存到环形队列并统计溢出错误。 */
void USART1_IRQHandler(void)
{
    uint32_t status = USART1->SR;
    if ((status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE)) != 0U)
    {
        /* 先读 SR 再读 DR，清除 RXNE 和接收错误。 */
        uint8_t byte = (uint8_t)USART1->DR;
        uint16_t next = (uint16_t)((rx_head + 1U) % RX_CAPACITY);
        if ((status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) != 0U)
        {
            ++io_errors;
        }
        else if (next == rx_tail)
        {
            ++io_errors;
        }
        else if ((status & USART_SR_RXNE) != 0U)
        {
            rx_bytes[rx_head] = byte;
            rx_head = next;
        }
    }
}

/** 封装协议帧并发送到指定串口，发送失败计入错误统计。 */
static void send_frame(uint8_t type, uint16_t seq, const uint8_t *data, uint16_t size)
{
    size_t length = car_encode(tx_frame, sizeof(tx_frame), type, seq, data, size);
    size_t i;
    uint32_t start = tick_ms;
    /* 本关没有实时控制；发送时接收 IRQ 仍把数据保存到环形缓冲区。 */
    for (i = 0U; i < length; ++i)
    {
        while ((USART1->SR & USART_SR_TXE) == 0U)
        {
            if ((uint32_t)(tick_ms - start) >= 100U)
            {
                ++tx_errors;
                return;
            }
        }
        USART1->DR = tx_frame[i];
    }
}

/** 处理诊断请求和 OTA 接收命令，成功确认复位请求后进入复位。 */
static void on_frame(void *context, uint8_t type, uint16_t seq,
                     const uint8_t *data, uint16_t size)
{
    (void)context;
    if (type == CAR_MSG_PING)
    {
        send_frame(CAR_MSG_PONG, seq, data, size);
    }
#if defined(M2_UART_UPDATE)
    else
    {
        uint8_t reply[16] = {0};
        int reset = 0;
        if (type == CAR_MSG_ENTER_UPDATE)
        {
            if (size != 4U || memcmp(data, "OTA1", 4U) != 0) return;
#if defined(M2_APP)
            /* 仅限当前无 CAN/运动逻辑的诊断 App；控制 App 后续必须检查停车。 */
            reply[0] = update_request_set() ? OTA_OK : OTA_IO_ERROR;
            reply[1] = OTA_APP;
            reset = reply[0] == OTA_OK;
#else
            reply[1] = updater.state;
#endif
            reply[2] = type;
        }
#if defined(M2_BOOT)
        else if (type >= CAR_MSG_OTA_BEGIN && type <= CAR_MSG_OTA_RESET)
        {
            uint8_t result = ota_handle(&updater, type, seq, data, size, tick_ms, reply);
            reset = type == CAR_MSG_OTA_RESET && result == OTA_OK;
        }
#else
        else if (type == CAR_MSG_OTA_QUERY && size == 0U)
        {
            reply[1] = OTA_APP;
            reply[2] = type;
            ota_put32(reply + 12U, DIAG_APP_VERSION);
        }
#endif
        else return;
        send_frame(CAR_MSG_OTA_REPLY, seq, reply, sizeof(reply));
        if (reset)
        {
            uint32_t started = tick_ms;
            while ((USART1->SR & USART_SR_TC) == 0U)
                if ((uint32_t)(tick_ms - started) >= 100U) { ++tx_errors; return; }
            NVIC_SystemReset();
        }
    }
#endif
}

/** 将 32 位整数编码为小端字节序。 */
static void put32(uint8_t *p, uint32_t value)
{
    unsigned i;
    for (i = 0U; i < 4U; ++i)
    {
        p[i] = (uint8_t)(value >> (8U * i));
    }
}

/** 发送 Bootloader 身份、运行时间与通信错误计数。 */
static void send_status(uint16_t seq)
{
    uint8_t data[16];
    memcpy(data, DIAG_TAG, 4U);
    put32(data + 4U, tick_ms);
    put32(data + 8U, receiver.crc_errors + receiver.header_errors + receiver.timeouts);
    put32(data + 12U, io_errors + tx_errors);
    send_frame(CAR_MSG_DIAGNOSTIC, seq, data, sizeof(data));
}

/** 完成启动初始化并运行主循环；具体硬件职责见本文件的模块说明。 */
int main(void)
{
    uint32_t last_status = 0U;
    uint16_t status_seq = 0U;
    /* 厂商 SystemInit 保持复位后的 HSI 8 MHz，不依赖板上外部晶振。 */
    SystemCoreClockUpdate();
    SCB->VTOR = DIAG_VECTOR_BASE;
#if defined(M2_APP) || defined(M2_BOOT)
    __DSB();
    __ISB();
#endif
    (void)SysTick_Config(SystemCoreClock / 1000U);
    NVIC_SetPriority(SysTick_IRQn, 3U);
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;
    (void)RCC->APB2ENR;
    /* PA9=复用推挽输出；PA10=上拉输入。保留 PA13/PA14 的 SWD 配置。 */
    GPIOA->ODR |= GPIO_ODR_ODR10;
    GPIOA->CRH = (GPIOA->CRH & ~0x00000FF0U) | 0x000008B0U;
    USART1->BRR = (SystemCoreClock + 57600U) / 115200U;
    USART1->CR3 = USART_CR3_EIE;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    car_receiver_init(&receiver, on_frame, NULL);
#if defined(M2_UART_UPDATE) && defined(M2_BOOT)
    ota_init(&updater, &stm32_ota_storage);
#endif
    NVIC_SetPriority(USART1_IRQn, 2U);
    NVIC_EnableIRQ(USART1_IRQn);
#if defined(M2_APP) || defined(M2_BOOT)
    /* Bootloader 跳转时保持 PRIMASK=1，App 完成初始化后才开放中断。 */
    __enable_irq();
#endif
    send_status(status_seq++);
    while (1)
    {
        IWDG->KR = 0xAAAAU; /* 升级期间接续 App 看门狗。 */
        while (rx_tail != rx_head)
        {
            uint8_t byte = rx_bytes[rx_tail];
            rx_tail = (uint16_t)((rx_tail + 1U) % RX_CAPACITY);
            car_receiver_feed(&receiver, byte, tick_ms);
        }
        car_receiver_poll(&receiver, tick_ms);
#if defined(M2_UART_UPDATE) && defined(M2_BOOT)
        ota_poll(&updater, tick_ms);
#endif
        if ((uint32_t)(tick_ms - last_status) >= 1000U)
        {
            last_status = tick_ms;
            send_status(status_seq++);
        }
    }
}
