"""提取实际收发函数，在HAL发送持锁窗口注入接收中断，验证队列寿命、超时及不丢重挂。"""
from pathlib import Path
import subprocess

root=Path(__file__).resolve().parents[1]
source=(root/'firmware/stm32_app/Core/Src/smartcar.c').read_text(encoding='utf-8')


def function(name):
    """从生产源文件提取完整函数，不维护测试专用算法副本。"""
    start=source.index(name+'(');start=source.rfind('\n',0,start)+1
    opening=source.index('{',start);depth=1;end=opening+1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}');end+=1
    return source[start:end]


stub=r'''
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#define CAR_MAX_FRAME 268
#define CAR_MSG_TELEMETRY 0x10
#define CAR_MSG_DIAGNOSTIC 0x11
#define CAR_MSG_IMU 0x12
#define HAL_OK 0
#define HAL_BUSY 1
#define HAL_UART_STATE_READY 0
#define HAL_UART_STATE_BUSY_TX 1
typedef int HAL_StatusTypeDef;
typedef struct {int Lock,RxState;volatile int gState;} UART_HandleTypeDef;
static UART_HandleTypeDef huart1;
static uint8_t rx_byte=42,rx_ring[512];
static uint16_t rx_head,rx_tail;
static uint32_t io_errors,mask,tick;
static int deferred,stuck,polls,aborts;
static uint8_t tx_frames[4][CAR_MAX_FRAME],tx_active;
static uint16_t tx_sizes[4];
static unsigned tx_head,tx_tail;
static uint32_t tx_started,tx_dropped;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart);
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1;}
static void __set_PRIMASK(uint32_t x){mask=x;if(!x&&deferred){deferred=0;HAL_UART_RxCpltCallback(&huart1);}}
static uint32_t HAL_GetTick(void){return tick++;}
static size_t car_encode(uint8_t*a,size_t cap,uint8_t type,uint16_t seq,const uint8_t*p,uint16_t n){(void)a;(void)cap;(void)type;(void)seq;(void)p;(void)n;return 28;}
static int HAL_UART_Receive_IT(UART_HandleTypeDef*u,uint8_t*p,int n){(void)p;(void)n;if(u->Lock)return HAL_BUSY;u->RxState=1;return HAL_OK;}
static int start_tx(UART_HandleTypeDef*u){u->Lock=1;u->RxState=0;if(mask)deferred=1;else HAL_UART_RxCpltCallback(u);u->gState=1;u->Lock=0;return HAL_OK;}
static int HAL_UART_Transmit(UART_HandleTypeDef*u,uint8_t*p,uint16_t n,int timeout){(void)p;(void)n;(void)timeout;start_tx(u);u->gState=0;return HAL_OK;}
static int HAL_UART_Transmit_IT(UART_HandleTypeDef*u,uint8_t*p,uint16_t n){(void)p;(void)n;return start_tx(u);}
static int HAL_UART_AbortTransmit(UART_HandleTypeDef*u){aborts++;u->gState=0;return HAL_OK;}
static void poll_can(void){polls++;if(!stuck)huart1.gState=0;}
'''
test=r'''
int main(void){
 send_frame(1,1,0,0);
 assert(io_errors==0 && rx_head==1 && rx_ring[0]==42 && huart1.RxState==1);
 assert(mask==0 && polls==0 && huart1.gState==1 && tx_active);
 send_frame(0x12,2,0,0);send_frame(0x12,3,0,0);
 assert(tx_head-tx_tail==2 && tx_dropped==1);
 send_frame(0x71,4,0,0);send_frame(0x71,5,0,0);
 assert(tx_head-tx_tail==4 && io_errors==0);
 huart1.gState=0;poll_tx();assert(tx_tail==1 && tx_active && rx_head==2);
 tick+=31;poll_tx();assert(io_errors==1 && aborts==1 && mask==0);
 while(tx_head!=tx_tail){huart1.gState=0;poll_tx();}
 assert(!tx_active && polls==0);
 return 0;
}
'''
out=root/'out/uart_tx_race';out.mkdir(exist_ok=True)
c=out/'test.c';exe=out/'test.exe'
c.write_text(stub+function('HAL_UART_RxCpltCallback')+function('poll_tx')+function('send_frame')+test,encoding='utf-8')
subprocess.run(['C:/msys64/ucrt64/bin/gcc.exe',str(c),'-o',str(exe)],check=True)
subprocess.run([str(exe)],check=True)
print('PASS: RX during TX lock, async queue, ACK capacity, bounded TX timeout')
