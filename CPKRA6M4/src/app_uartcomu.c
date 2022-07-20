#include <rtthread.h>
#include "bsp_api.h"
#include "bsp_can.h"
#include "driver_uart.h"
#include "slcan.h"

#define BUFFER_MAX SLCAN_MTU

uint8_t buffer[BUFFER_MAX];
void uartcomu_thread_entry(void *parameter)
{
    uart_cfg_t cfg;
    int8_t erro;
    cfg.id = UART_ID_6;
    cfg.baud = 115200;
    cfg.datalen = UART_DATA_8;
    cfg.parity = UART_CHECK_NONE;
    cfg.stopb = UART_STOPB_1;
    driver_uart_init(UART_ID_6);
    driver_uart_set(&cfg);
    driver_uart_flush(UART_ID_6);

#if 0
    for(int i=0;i<BUFFER_MAX;i++)
    {
        buffer[i]=i+1;
    }

    /* 发送速率测试 */
    uint32_t pre = rt_tick_get();
    for(int i = 0; i< BUFFER_MAX; i++)
    {
        driver_uart_send(UART_ID_6, buffer, BUFFER_MAX, 1000, &erro);
        //rt_thread_mdelay(100);
    }
    pre = rt_tick_get() - pre;
    buffer[0]=(pre>>24)&0xFF;
    buffer[1]=(pre>>16)&0xFF;
    buffer[2]=(pre>>8)&0xFF;
    buffer[3]=(pre>>0)&0xFF;
    driver_uart_send(UART_ID_6, buffer, 4, 1000, &erro);
#endif
    /* 收发测试  */
    while(1)
    {
        int len = driver_uart_recv(UART_ID_6, buffer, BUFFER_MAX, 100, &erro);
        if(len > 0)
        {
            //driver_uart_send(UART_ID_6, buffer, len, 100, &erro);
            slcan_parse_str(buffer,len);
        }
    }

}

void app_uartcomu_init(void)
{
    rt_thread_t tid = rt_thread_create("uartcomu", uartcomu_thread_entry, RT_NULL,
                           RT_MAIN_THREAD_STACK_SIZE, RT_MAIN_THREAD_PRIORITY, 20);
    RT_ASSERT(tid != RT_NULL);
    rt_thread_startup(tid);
}
