"""检查分层边界并对生产CAN适配函数注入邮箱、接收及总线故障；不访问硬件。"""
from pathlib import Path
import re
import subprocess
root=Path(__file__).resolve().parents[1]
bsp=root/'firmware/stm32_app/BSP'
for p in bsp.glob('*.[ch]'):
    text=p.read_text(encoding='utf-8')
    assert not re.search(r'#include "(?:drive|navigation|imu|mpu6050|car_protocol|ota_receiver)\.h"',text),p
    assert not re.search(r'\b(?:drive_fault|drive_command|imu_sample|car_encode)\s*\(',text),p
for name in ('smartcar','mpu6050'):
    text=(root/f'firmware/stm32_app/Core/Src/{name}.c').read_text(encoding='utf-8')
    assert not re.search(r'\b(?:HAL_|__HAL_|GPIO[A-G]|IWDG|SCB|NVIC_|__disable_irq)',text),name
source=(bsp/'BSP_can.c').read_text(encoding='utf-8')
def function(name):
    """提取生产函数体，避免测试中另写CAN处理实现。"""
    m=re.search(r'^(?:int|void) '+name+r'\([^;]*?\)\s*\{',source,re.M)
    start=m.start();i=source.index('{',start)+1;depth=1
    while depth:
        depth+=(source[i]=='{')-(source[i]=='}');i+=1
    return source[start:i]
stub=r'''
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include "BSP_can.h"
typedef struct {uint32_t StdId,ExtId,IDE,RTR,DLC;} CAN_RxHeaderTypeDef;
typedef CAN_RxHeaderTypeDef CAN_TxHeaderTypeDef;
typedef struct {uint32_t ESR,RF0R;} Registers;
static Registers regs;static struct {Registers *Instance;} hcan={&regs};
static unsigned available,free_boxes=3,aborted;static int read_error,send_error;
static CAN_RxHeaderTypeDef incoming,sent;static uint8_t payload[8];
#define CAN_RX_FIFO0 0
#define CAN_ID_STD 0
#define CAN_RTR_DATA 0
#define HAL_OK 0
#define CAN_TX_MAILBOX0 1
#define CAN_TX_MAILBOX1 2
#define CAN_TX_MAILBOX2 4
#define CAN_ESR_BOFF 4
#define CAN_RF0R_FOVR0 16
#define CAN_FLAG_FOV0 16
#define HAL_CAN_GetRxFifoFillLevel(h,f) available
#define HAL_CAN_GetRxMessage(h,f,header,data) (memcpy(header,&incoming,sizeof(incoming)),memset(data,0x5a,8),read_error)
#define HAL_CAN_AddTxMessage(h,header,data,mailbox) (memcpy(&sent,header,sizeof(sent)),memcpy(payload,data,8),*(mailbox)=0,send_error)
#define HAL_CAN_GetTxMailboxesFreeLevel(h) free_boxes
#define HAL_CAN_AbortTxRequest(h,m) (aborted=(m))
#define __HAL_CAN_CLEAR_FLAG(h,f) (regs.RF0R&=~(f))
#define HAL_CAN_Stop(h) 0
'''
test=r'''
int main(void) {
 BSP_CAN_Frame f;uint8_t p[8]={1,2,3,4,5,6,7,8};
 assert(BSP_CAN_Read(&f)==0);available=1;incoming.StdId=0x202;incoming.DLC=8;
 assert(BSP_CAN_Read(&f)==1 && f.id==0x202 && f.size==8 && f.data[7]==0x5a);
 incoming.IDE=1;incoming.ExtId=0x12345;incoming.RTR=1;
 assert(BSP_CAN_Read(&f)==1 && f.extended && f.remote && f.id==0x12345);
 read_error=1;assert(BSP_CAN_Read(&f)==-1);
 assert(BSP_CAN_Send(0x200,p) && sent.StdId==0x200 && !memcmp(p,payload,8));
 send_error=1;assert(!BSP_CAN_Send(0x200,p));
 assert(BSP_CAN_TxIdle());free_boxes=2;assert(!BSP_CAN_TxIdle());
 BSP_CAN_Abort();assert(aborted==7);
 regs.RF0R=16;assert(BSP_CAN_Fault() && regs.RF0R==0 && !BSP_CAN_BusOff());
 regs.ESR=4;assert(BSP_CAN_Fault() && BSP_CAN_BusOff());
 return 0;
}
'''
out=root/'out/bsp_test';out.mkdir(exist_ok=True)
c=out/'test.c';exe=out/'test.exe'
c.write_text(stub+''.join(function(n) for n in ('BSP_CAN_Read','BSP_CAN_Send','BSP_CAN_TxIdle','BSP_CAN_Abort','BSP_CAN_Fault','BSP_CAN_BusOff'))+test,encoding='utf-8')
subprocess.run(['C:/msys64/ucrt64/bin/gcc.exe','-std=c99','-Wall','-Wextra','-Werror','-I'+str(bsp),str(c),'-o',str(exe)],check=True)
subprocess.run([str(exe)],check=True)
print('PASS: BSP dependency boundary, CAN frame transfer, mailboxes, bus-off and overflow (HOST ONLY)')
