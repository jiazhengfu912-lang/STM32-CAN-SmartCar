"""用真实接收回调及恢复函数复现 HAL 锁冲突；不连接硬件。"""
from pathlib import Path
import re,subprocess
root=Path(__file__).resolve().parents[1]
source=(root/'firmware/stm32_app/Core/Src/smartcar.c').read_text(encoding='utf-8')

def function(name):
    """提取生产函数原文，避免测试复制另一份恢复算法。"""
    start=source.index(name+'(');start=source.rfind('\n',0,start)+1
    opening=source.index('{',start);depth=1;end=opening+1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}');end+=1
    return source[start:end]

p=root/'out/uart_recovery_v102';p.mkdir(exist_ok=True)
stub='''#include <stdint.h>
#include <assert.h>
#define HAL_OK 0
#define HAL_BUSY 1
#define HAL_UART_STATE_READY 0
#define HAL_UART_STATE_BUSY_RX 1
#define HAL_UNLOCKED 0
#define HAL_LOCKED 1
typedef struct {int RxState,Lock;} UART_HandleTypeDef;
static UART_HandleTypeDef huart1;
static uint8_t rx_byte,rx_ring[512];
static uint16_t rx_head,rx_tail;
static uint32_t io_errors,mask;
static int receiver,on_frame,cleared,resets;
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1;}
static void __set_PRIMASK(uint32_t x){mask=x;}
static void car_receiver_init(int*a,int b,int c){(void)a;(void)b;(void)c;resets++;}
#define __HAL_UART_CLEAR_OREFLAG(x) (++cleared)
static int HAL_UART_Receive_IT(UART_HandleTypeDef*u,uint8_t*p,int n){(void)p;(void)n;if(u->Lock||u->RxState)return HAL_BUSY;u->RxState=HAL_UART_STATE_BUSY_RX;return HAL_OK;}
'''
test='''
int main(void){
 huart1.Lock=HAL_LOCKED;rx_byte=42;HAL_UART_RxCpltCallback(&huart1);
 assert(io_errors==1 && huart1.RxState==HAL_UART_STATE_READY);
 recover_uart_rx();assert(huart1.RxState==HAL_UART_STATE_READY && resets==0);
 huart1.Lock=HAL_UNLOCKED;recover_uart_rx();
 assert(huart1.RxState==HAL_UART_STATE_BUSY_RX && rx_head==rx_tail);
 assert(io_errors==1 && resets==1 && cleared==1 && mask==0);
 recover_uart_rx();assert(resets==1);
 huart1.RxState=HAL_UART_STATE_READY;mask=1;recover_uart_rx();assert(mask==1);
 return 0;
}
'''
for label,recovery in [('before','static void recover_uart_rx(void) {}'),('after',function('recover_uart_rx'))]:
    c=p/('recovery_'+label+'.c');exe=c.with_suffix('.exe');c.write_text(stub+function('HAL_UART_RxCpltCallback')+recovery+test,encoding='utf-8')
    subprocess.run(['C:/msys64/ucrt64/bin/gcc.exe',str(c),'-o',str(exe)],check=True)
    result=subprocess.run([str(exe)],capture_output=True)
    assert (result.returncode!=0) if label=='before' else (result.returncode==0)
    print(label, 'fails as expected' if label=='before' else 'PASS')
