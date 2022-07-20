#include <rtthread.h>
#include "bsp_api.h"
#include "bsp_can.h"
#include "slcan.h"
#include "driver_uart.h"

rt_mq_t tx_queue = NULL;
rt_mq_t rx_queue = NULL;

#define TX_QUEUE_LEN 10
#define RX_QUEUE_LEN 10


int canrx_callback(can_id_e id,can_msg_st* msg)
{
    rt_mq_send(rx_queue,msg,sizeof(can_msg_st));
    return 0;
}

void cantx_thread_entry(void *parameter)
{
    rt_err_t res;
    can_msg_st msg;
    int retry;
    int8_t sendres;
    while(1)
    {
        do
        {
            res = rt_mq_recv(tx_queue,&msg,sizeof(can_msg_st),rt_tick_from_millisecond(1));
            if(RT_EOK == res)
            {
                /* 写消息邮箱 */
                retry = 3;
                do
                {
                    sendres = bsp_can_send(CAN_ID_0,msg);
                    if(sendres != 0)
                    {
                        rt_thread_mdelay(10);
                    }
                }while((sendres != 0) && ( retry-- > 0));
            }
        }while(RT_EOK == res);
        rt_thread_mdelay(10);
    }
}

void canrx_thread_entry(void *parameter)
{
    rt_err_t res;
    can_msg_st msg;
    uint8_t buf[SLCAN_MTU];
    int8_t len;
    int8_t erro;
    while(1)
    {
        do
        {
            res = rt_mq_recv(rx_queue,&msg,sizeof(can_msg_st),rt_tick_from_millisecond(1));
            if(RT_EOK == res)
            {
                /* 收到CAN数据转发到UART */
                if((len = slcan_parse_frame(&msg,buf)) > 0)
                {
                    driver_uart_send(UART_ID_6, buf, len, 1, &erro);
                }
            }
        }while(RT_EOK == res);
        rt_thread_mdelay(10);
    }
}

int can_tx(can_id_e id,can_msg_st* msg,int32_t ms)
{
    return rt_mq_send_wait(tx_queue,msg,sizeof(can_msg_st),ms);
}

static R_CAN0_Type * can0_reg = ((R_CAN0_Type *) R_CAN0_BASE);

void app_cancomu_init(void)
{
    can0_reg = can0_reg;
    bsp_can_setrxcallback(CAN_ID_0,canrx_callback);
    bsp_can_init(CAN_ID_0);

    /* 50000000
     * 5分频 设置值为4  10M 0.1uS
     * tq最多8~25个
     * 500K 2uS  20个tq
     * ss ts1 ts2  sjw
     * 1  14  5    4
     * 注意寄存器值都比设置值小1
     * */
    bsp_can_setbit(CAN_ID_0,4,13,4,3);
    //bsp_can_setfilter();
    bsp_can_settest(CAN_ID_0,1,3);

    tx_queue = rt_mq_create("cantx_queue", sizeof(can_msg_st), TX_QUEUE_LEN, RT_IPC_FLAG_FIFO);
    rx_queue = rt_mq_create("canrx_queue", sizeof(can_msg_st), RX_QUEUE_LEN, RT_IPC_FLAG_FIFO);

    rt_thread_t cantxtid = rt_thread_create("cantx", cantx_thread_entry, RT_NULL,
                           RT_MAIN_THREAD_STACK_SIZE, RT_MAIN_THREAD_PRIORITY, 20);
    RT_ASSERT(cantxtid != RT_NULL);
    rt_thread_startup(cantxtid);

    rt_thread_t canrxtid = rt_thread_create("canrx", canrx_thread_entry, RT_NULL,
                           RT_MAIN_THREAD_STACK_SIZE, RT_MAIN_THREAD_PRIORITY, 20);
    RT_ASSERT(canrxtid != RT_NULL);
    rt_thread_startup(canrxtid);
}
