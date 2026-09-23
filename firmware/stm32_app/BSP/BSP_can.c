#include "main.h"
#include "BSP_can.h"
static CAN_HandleTypeDef hcan;
/** 初始化当前板CAN1时序；不包含电机控制策略。 */
int BSP_CAN_Init(void)
{
    hcan.Instance=CAN1; hcan.Init.Prescaler=6; hcan.Init.Mode=CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth=CAN_SJW_1TQ; hcan.Init.TimeSeg1=CAN_BS1_3TQ; hcan.Init.TimeSeg2=CAN_BS2_2TQ;
    hcan.Init.AutoBusOff=ENABLE; hcan.Init.AutoRetransmission=DISABLE;
    return HAL_CAN_Init(&hcan)==HAL_OK;
}

/** 配置 CAN1 的 PA11 接收和 PA12 发送；使用主循环读取 FIFO，不开 CAN IRQ。 */
void HAL_CAN_MspInit(CAN_HandleTypeDef *can)
{
    GPIO_InitTypeDef p={0}; if (can->Instance!=CAN1) return;
    __HAL_RCC_CAN1_CLK_ENABLE(); __HAL_RCC_GPIOA_CLK_ENABLE();
    p.Pin=GPIO_PIN_11; p.Mode=GPIO_MODE_INPUT; p.Pull=GPIO_NOPULL; HAL_GPIO_Init(GPIOA,&p);
    p.Pin=GPIO_PIN_12; p.Mode=GPIO_MODE_AF_PP; p.Speed=GPIO_SPEED_FREQ_HIGH; HAL_GPIO_Init(GPIOA,&p);
}
/** 配置上层指定的两个标准帧过滤项并启动CAN。 */
int BSP_CAN_Start(uint16_t id0,uint16_t id1) {
    CAN_FilterTypeDef f={0};
    f.FilterMode=CAN_FILTERMODE_IDLIST; f.FilterScale=CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh=(uint32_t)id0<<5; f.FilterMaskIdHigh=(uint32_t)id1<<5;
    f.FilterFIFOAssignment=CAN_RX_FIFO0; f.FilterActivation=ENABLE;
    return HAL_CAN_ConfigFilter(&hcan,&f)==HAL_OK && HAL_CAN_Start(&hcan)==HAL_OK;
}
/** 非阻塞取出一帧，只转换HAL头字段。 */
int BSP_CAN_Read(BSP_CAN_Frame *f) {
    CAN_RxHeaderTypeDef h;
    if (!HAL_CAN_GetRxFifoFillLevel(&hcan,CAN_RX_FIFO0)) return 0;
    if (HAL_CAN_GetRxMessage(&hcan,CAN_RX_FIFO0,&h,f->data)!=HAL_OK) return -1;
    f->extended=h.IDE!=CAN_ID_STD; f->remote=h.RTR!=CAN_RTR_DATA; f->size=(uint8_t)h.DLC;
    f->id=f->extended ? h.ExtId:h.StdId; return 1;
}
/** 发出标准8字节数据帧，邮箱繁忙时不等待也不改变载荷。 */
int BSP_CAN_Send(uint16_t id,const uint8_t data[8]) {
    CAN_TxHeaderTypeDef h={0}; uint32_t mailbox;
    h.StdId=id; h.IDE=CAN_ID_STD; h.RTR=CAN_RTR_DATA; h.DLC=8;
    return HAL_CAN_AddTxMessage(&hcan,&h,(uint8_t *)data,&mailbox)==HAL_OK;
}
/** 查询是否所有发送邮箱空闲。 */
int BSP_CAN_TxIdle(void) { return HAL_CAN_GetTxMailboxesFreeLevel(&hcan)==3U; }
/** 撤销硬件邮箱，不决定是否锁存业务故障。 */
void BSP_CAN_Abort(void) { (void)HAL_CAN_AbortTxRequest(&hcan,CAN_TX_MAILBOX0|CAN_TX_MAILBOX1|CAN_TX_MAILBOX2); }
/** 读取总线关闭及FIFO溢出，清除溢出标志。 */
int BSP_CAN_Fault(void) {
    int fault=(hcan.Instance->ESR&CAN_ESR_BOFF) || (hcan.Instance->RF0R&CAN_RF0R_FOVR0);
    if (fault) __HAL_CAN_CLEAR_FLAG(&hcan,CAN_FLAG_FOV0);
    return fault;
}
/** 停止CAN外设。 */
void BSP_CAN_Stop(void) { (void)HAL_CAN_Stop(&hcan); }

/** 单独查询总线关闭，供调用方判断待发送数据是否完成。 */
int BSP_CAN_BusOff(void) { return (hcan.Instance->ESR&CAN_ESR_BOFF)!=0; }
