
/**
 *****************************************************************************
 * \brief       驱动层(DRIVER)串口模块(UART)相关接口实现.
 * \details     Copyright (c) 2019,spacety.
 *              All rights reserved.
 * \file        driver_uart.c
 * \author      lihongjie@spacety.cn
 * \version     1.0
 * \date        2019-08-28
 * \note        使用前参考注释.\n
 *              依赖bsp_uart,OsTimeDelay OSCriticalAlloc OSCriticalEnter OSCriticalExit\n
 * \since       lihongjie@spacety.cn 2019-8-28 1.0  新建
 * \par 修订记录
 * - 2019-08-28 初始版本
 * \par 资源说明
 * - RAM:
 * - ROM:
 *****************************************************************************
 */
#define DRIVER_UART_GLOBALS /**< 包含driver_uart.h中的数据将会编译*/
#include "bsp_uart.h"
#include "driver_uart.h"
#include <rthw.h>
#include <rtthread.h>
#define OsTimeDelay(ms) rt_thread_mdelay(ms)
#define OSCriticalAlloc() int level
#define OSCriticalEnter() level = rt_hw_interrupt_disable()
#define OSCriticalExit()  rt_hw_interrupt_enable(level)

/*****************************************************************************
 *                                                                           *
 *                             内部数据                                      *
 *                                                                           *
 ****************************************************************************/


/*****************************************************************************
 *                                                                           *
 *                             内部接口函数实现                              *
 *                                                                           *
 ****************************************************************************/

/*回调函数*/
#if DRIVER_UART0_ENABLE
static void uart0_txhook(void)
{
/*485设置为发送*/
}

static void uart0_rxhook(void)
{
/*485设置为接收*/
}
#endif
#if DRIVER_UART1_ENABLE
static void uart1_txhook(void)
{
/*485设置为发送*/
}

static void uart1_rxhook(void)
{
/*485设置为接收*/
}
#endif
#if DRIVER_UART2_ENABLE
static void uart2_txhook(void)
{
/*485设置为发送*/
}

static void uart2_rxhook(void)
{
/*485设置为接收*/
}
#endif
#if DRIVER_UART3_ENABLE
static void uart3_txhook(void)
{
/*485设置为发送*/
}

static void uart3_rxhook(void)
{
/*485设置为接收*/
}
#endif
#if DRIVER_UART4_ENABLE
static void uart4_txhook(void)
{
/*485设置为发送*/
}

static void uart4_rxhook(void)
{
/*485设置为接收*/
}
#endif
#if DRIVER_UART5_ENABLE
static void uart5_txhook(void)
{
/*485设置为发送*/
}

static void uart5_rxhook(void)
{
/*485设置为接收*/
}
#endif
#if DRIVER_UART6_ENABLE
static void uart6_txhook(void)
{
/*485设置为发送*/
}

static void uart6_rxhook(void)
{
/*485设置为接收*/
}
#endif

/**
 *****************************************************************************
 * \fn          static int8_t rx_fifo_in(driver_uart_t *p, uint8_t data)
 * \brief       写数据到接收缓冲区.
 * \note        接收中断服务函数调用该函数写数据到接收缓冲区,缓冲区满不写入.
 * \param[in]   p 指向uart结构体实例的指针 \ref driver_uart_t
 * \param[in]   data 指向要写到缓冲区的数据
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
static int8_t rx_fifo_in(driver_uart_t *p, uint8_t data)
{
    if(0 == p)
    {
        return (int8_t)-1;
    }
    /*缓冲区未满写入数据*/
    if (p->rx_len < (p->rxbuf_len/sizeof(p->rx_buf[0])))
    {
        p->rx_buf[p->rx_in++] = data;                   /*写入数据,写入指针递增*/
        p->rx_in %= p->rxbuf_len/sizeof(p->rx_buf[0]);  /*边界处理,实现圆形缓冲区*/
        p->rx_len++;                                    /*数据个数递增*/
        return (int8_t)0;
    }
    else
    {
        return (int8_t)-1;
    }
}

/**
 *****************************************************************************
 * \fn          static uint8_t rx_fifo_out(driver_uart_t *p, uint32_t timeout, int8_t *perro)
 * \brief       从接收缓冲区读数据.
 * \note        应用函数调用该函数从接收缓冲区读数据,可以指定超时时间.
 * \param[in]   p 指向uart结构体实例的指针 \ref driver_uart_t
 * \param[in]   timeout 指定超时时间 0表示一直等待
 * \param[out]  perro 错误码0表示读到了数据 1表示没有读到数据
 * \return      uint8_t 读到的字节数据.
 *****************************************************************************
 */
static uint8_t rx_fifo_out(driver_uart_t *p, uint32_t timeout, int8_t *perro)
{
    uint8_t data = (uint8_t)0;
    if(p==0)
    {
        *perro = (int8_t)1;
        return (uint8_t)0;
    }
    OSCriticalAlloc();
    /*timeout==0 表示一直等待直到有数据*/
    if (timeout == 0)
    {
        while(p->rx_len == 0)
        {
            OsTimeDelay(DRIVER_UART_DELAY_CHECK);  /*延时*/
        }
    }
    else /*timeout!=0 表示等待指定超时时间*/
    {
        while (timeout != 0)
        {
            if (p->rx_len == 0)
            {
                /*如果无数据,延时指定时间间隔*/
                if (timeout >= DRIVER_UART_DELAY_CHECK)
                {
                    timeout -= DRIVER_UART_DELAY_CHECK;
                    OsTimeDelay(DRIVER_UART_DELAY_CHECK);
                }
                else
                {
                     /*不足一个时间间隔延时剩余时间*/
                    OsTimeDelay(timeout);
                    timeout = 0;
                }
            }
            else
            {
                break;
            }
        }
    }
    /*临界段处理 防止此时中断在操作缓冲区*/
    OSCriticalEnter();
    if (p->rx_len != 0)
    {
        /*从缓冲区读出数据*/
        p->rx_len--;
        data = p->rx_buf[p->rx_out++];
        p->rx_out %= p->rxbuf_len/sizeof(p->rx_buf[0]);
        *perro = (int8_t)0;
    }
    else
    {
        *perro = (int8_t)1;
    }
    OSCriticalExit();
    return data;
}

/**
 *****************************************************************************
 * \fn          static int8_t tx_fifo_in(driver_uart_t *p, uint8_t data, uint32_t timeout)
 * \brief       写数据到发送缓冲区.
 * \note        应用调用该函数调写数据到发送缓冲区,缓冲区满等待.
 * \param[in]   p 指向uart结构体实例的指针 \ref driver_uart_t
 * \param[in]   timeout 设定超时时间 0表示一直等待
 * \param[in]   data 指向要写到缓冲区的数据
 * \retval      0 写成功
 * \retval      1 写失败
 *****************************************************************************
 */
static int8_t tx_fifo_in(driver_uart_t *p, uint8_t data, uint32_t timeout)
{
    /*注意确保p缓冲区的读写处于临界段保护 */
    int8_t ret= (int8_t)0;
    if(0 == p)
    {
        return (int8_t)1;
    }
    OSCriticalAlloc();
    if (timeout == (uint32_t)0)
    {
        while(1)
        {
            OSCriticalEnter();
            if(p->tx_len < (p->txbuf_len/sizeof(p->tx_buf[0])))
            {
                /*没有满则写入数据*/
                p->tx_buf[p->tx_in++] = data;
                p->tx_in %= p->txbuf_len/sizeof(p->tx_buf[0]);
                p->tx_len++;
                OSCriticalExit();
                break;
            }
            else
            {
                /*满则等待*/
                OSCriticalExit();
                OsTimeDelay(DRIVER_UART_DELAY_CHECK);  /*延时*/
            }
        }
    }
    while (timeout != 0)
    {
        OSCriticalEnter();
        if (p->tx_len < (p->txbuf_len/sizeof(p->tx_buf[0])))
        {
            /*没有满则写入数据*/
            p->tx_buf[p->tx_in++] = data;
            p->tx_in %= p->txbuf_len/sizeof(p->tx_buf[0]);
            p->tx_len++;
            OSCriticalExit();
            ret = (int8_t)0; /*设置正确返回值*/
            break;
        }
        else
        {
            OSCriticalExit();
            /*如果无数据,延时指定时间间隔*/
            if (timeout >= DRIVER_UART_DELAY_CHECK)
            {
                timeout -= DRIVER_UART_DELAY_CHECK;
                OsTimeDelay(DRIVER_UART_DELAY_CHECK);
                ret = (int8_t)1; /*设置超时错误 之前这里没设置ret=1 在timeout为DRIVER_UART_DELAY_CHECK的整数倍时总是会返回0*/
            }
            else
            {
                /*不足一个时间间隔延时剩余时间*/
                OsTimeDelay(timeout);
                timeout = (uint32_t)0;
                ret = (int8_t)1; /*设置超时错误*/
            }
        }
    }
    return ret;
}

/**
 *****************************************************************************
 * \fn          static uint8_t tx_fifo_out(driver_uart_t *p, int8_t *perro)
 * \brief       从发送缓冲区读数据.
 * \note        发送中断服务函数调用该函数获取发送缓冲区数据,用于发送.
 * \param[in]   p 指向uart结构体实例的指针 \ref driver_uart_t
 * \param[out]  perro 错误码 0表示读到数据 1表示没有数据
 * \return      uint8_t perro=0时表示读到的数据
 *****************************************************************************
 */
static uint8_t tx_fifo_out(driver_uart_t *p, int8_t *perro)
{
    uint8_t data = (uint8_t)0;
    if(0 == p)
    {
        *perro = (int8_t)1;
        return 0;
    }
    if (p->tx_len != (uint16_t)0)
    {
        if(p->tx_hook != 0)
        {
            (*(p->tx_hook))();
        }
        p->tx_len--;
        data = p->tx_buf[p->tx_out++];
        p->tx_out %= p->txbuf_len/sizeof(p->tx_buf[0]);
        *perro = (int8_t)0;
    }
    else
    {
        *perro = (int8_t)1;
    }
    return data;
}

/**
 *****************************************************************************
 * \fn          static int8_t driver_uart0_send_isr(uint8_t *pdata)
 * \brief       发送处理.
 * \note        发送中断服务函数调用该函数,如果缓冲区空则停止发送并调用回调函数.
 * \param[out]  pdata 回写需要发送的数据
 * \retval      int8_t 0 pdata指向的数据有效(需要发送) 1无后续发送数据
 *****************************************************************************
 */
#if DRIVER_UART0_ENABLE
static int8_t driver_uart0_send_isr(uint8_t *pdata)
{
    int8_t erro=0;
    uint8_t data;
    if(driver_uart_tab[0]==0)
    {
        return (int8_t)1;
    }
    data = tx_fifo_out(driver_uart_tab[0], &erro);
    if (erro != 0)
    {
        /*没有获取到数据,停止发送*/
        bsp_uart_enabletx(driver_uart_tab[0]->id,UART_INTERRUPT,UART_DISABLE);
        driver_uart_tab[0]->state = UART_TXIDLE;
        if(driver_uart_tab[0]->rx_hook != 0)
        {
            driver_uart_tab[0]->rx_hook();
        }
        return (int8_t)1;
    }
    else
    {
        *pdata = data;
        return (int8_t)0;
    }
}
#endif
/**
 *****************************************************************************
 * \fn          static int8_t driver_uart1_send_isr(uint8_t *pdata)
 * \brief       发送处理.
 * \note        发送中断服务函数调用该函数,如果缓冲区空则停止发送并调用回调函数.
 * \param[out]  pdata 回写需要发送的数据
 * \retval      int8_t 0 pdata指向的数据有效(需要发送) 1无后续发送数据
 *****************************************************************************
 */
#if DRIVER_UART1_ENABLE
static int8_t driver_uart1_send_isr(uint8_t *pdata)
{
    int8_t erro=0;
    uint8_t data;
    if(driver_uart_tab[1]==0)
    {
        return (int8_t)1;
    }
    data = tx_fifo_out(driver_uart_tab[1], &erro);
    if (erro != 0)
    {
        /*没有获取到数据,停止发送*/
        bsp_uart_enabletx(driver_uart_tab[1]->id,UART_INTERRUPT,UART_DISABLE);
        driver_uart_tab[1]->state = UART_TXIDLE;
        if(driver_uart_tab[1]->rx_hook != 0)
        {
            driver_uart_tab[1]->rx_hook();
        }
        return (int8_t)1;
    }
    else
    {
        *pdata = data;
        return (int8_t)0;
    }
}
#endif
/**
 *****************************************************************************
 * \fn          static int8_t driver_uart2_send_isr(uint8_t *pdata)
 * \brief       发送处理.
 * \note        发送中断服务函数调用该函数,如果缓冲区空则停止发送并调用回调函数.
 * \param[out]  pdata 回写需要发送的数据
 * \retval      int8_t 0 pdata指向的数据有效(需要发送) 1无后续发送数据
 *****************************************************************************
 */
#if DRIVER_UART2_ENABLE
static int8_t driver_uart2_send_isr(uint8_t *pdata)
{
    int8_t erro=0;
    uint8_t data;
    if(driver_uart_tab[2]==0)
    {
        return (int8_t)1;
    }
    data = tx_fifo_out(driver_uart_tab[2], &erro);
    if (erro != 0)
    {
        /*没有获取到数据,停止发送*/
        bsp_uart_enabletx(driver_uart_tab[2]->id,UART_INTERRUPT,UART_DISABLE);
        driver_uart_tab[2]->state = UART_TXIDLE;
        if(driver_uart_tab[2]->rx_hook != 0)
        {
            driver_uart_tab[2]->rx_hook();
        }
        return (int8_t)1;
    }
    else
    {
        *pdata = data;
        return (int8_t)0;
    }
}
#endif
/**
 *****************************************************************************
 * \fn          static int8_t driver_uart3_send_isr(uint8_t *pdata)
 * \brief       发送处理.
 * \note        发送中断服务函数调用该函数,如果缓冲区空则停止发送并调用回调函数.
 * \param[out]  pdata 回写需要发送的数据
 * \retval      int8_t 0 pdata指向的数据有效(需要发送) 1无后续发送数据
 *****************************************************************************
 */
#if DRIVER_UART3_ENABLE
static int8_t driver_uart3_send_isr(uint8_t *pdata)
{
    int8_t erro=0;
    uint8_t data;
    if(driver_uart_tab[3]==0)
    {
        return (int8_t)1;
    }
    data = tx_fifo_out(driver_uart_tab[3], &erro);
    if (erro != 0)
    {
        /*没有获取到数据,停止发送*/
        bsp_uart_enabletx(driver_uart_tab[3]->id,UART_INTERRUPT,UART_DISABLE);
        driver_uart_tab[3]->state = UART_TXIDLE;
        if(driver_uart_tab[3]->rx_hook != 0)
        {
            driver_uart_tab[3]->rx_hook();
        }
        return (int8_t)1;
    }
    else
    {
        *pdata = data;
        return (int8_t)0;
    }
}
#endif
/**
 *****************************************************************************
 * \fn          static int8_t driver_uart4_send_isr(uint8_t *pdata)
 * \brief       发送处理.
 * \note        发送中断服务函数调用该函数,如果缓冲区空则停止发送并调用回调函数.
 * \param[out]  pdata 回写需要发送的数据
 * \retval      int8_t 0 pdata指向的数据有效(需要发送) 1无后续发送数据
 *****************************************************************************
 */
#if DRIVER_UART4_ENABLE
static int8_t driver_uart4_send_isr(uint8_t *pdata)
{
    int8_t erro=0;
    uint8_t data;
    if(driver_uart_tab[4]==0)
    {
        return (int8_t)1;
    }
    data = tx_fifo_out(driver_uart_tab[4], &erro);
    if (erro != 0)
    {
        /*没有获取到数据,停止发送*/
        bsp_uart_enabletx(driver_uart_tab[4]->id,UART_INTERRUPT,UART_DISABLE);
        driver_uart_tab[4]->state = UART_TXIDLE;
        if(driver_uart_tab[4]->rx_hook != 0)
        {
            driver_uart_tab[4]->rx_hook();
        }
        return (int8_t)1;
    }
    else
    {
        *pdata = data;
        return (int8_t)0;
    }
}
#endif
/**
 *****************************************************************************
 * \fn          static int8_t driver_uart5_send_isr(uint8_t *pdata)
 * \brief       发送处理.
 * \note        发送中断服务函数调用该函数,如果缓冲区空则停止发送并调用回调函数.
 * \param[out]  pdata 回写需要发送的数据
 * \retval      int8_t 0 pdata指向的数据有效(需要发送) 1无后续发送数据
 *****************************************************************************
 */
#if DRIVER_UART5_ENABLE
static int8_t driver_uart5_send_isr(uint8_t *pdata)
{
    int8_t erro=0;
    uint8_t data;
    if(driver_uart_tab[5]==0)
    {
        return (int8_t)1;
    }
    data = tx_fifo_out(driver_uart_tab[5], &erro);
    if (erro != 0)
    {
        /*没有获取到数据,停止发送*/
        bsp_uart_enabletx(driver_uart_tab[5]->id,UART_INTERRUPT,UART_DISABLE);
        driver_uart_tab[5]->state = UART_TXIDLE;
        if(driver_uart_tab[5]->rx_hook != 0)
        {
            driver_uart_tab[5]->rx_hook();
        }
        return (int8_t)1;
    }
    else
    {
        *pdata = data;
        return (int8_t)0;
    }
}
#endif
/**
 *****************************************************************************
 * \fn          static int8_t driver_uart6_send_isr(uint8_t *pdata)
 * \brief       发送处理.
 * \note        发送中断服务函数调用该函数,如果缓冲区空则停止发送并调用回调函数.
 * \param[out]  pdata 回写需要发送的数据
 * \retval      int8_t 0 pdata指向的数据有效(需要发送) 1无后续发送数据
 *****************************************************************************
 */
#if DRIVER_UART6_ENABLE
static int8_t driver_uart6_send_isr(uint8_t *pdata)
{
    int8_t erro=0;
    uint8_t data;
    if(driver_uart_tab[6]==0)
    {
        return (int8_t)1;
    }
    data = tx_fifo_out(driver_uart_tab[6], &erro);
    if (erro != 0)
    {
        /*没有获取到数据,停止发送*/
        //bsp_uart_enabletx(driver_uart_tab[6]->id,UART_INTERRUPT,UART_DISABLE);  /* 这里可能还没发送完就停止导致最后一个数据发送不出去 */
        //bsp_uart_enabletxirq(driver_uart_tab[6]->id,UART_INTERRUPT,UART_DISABLE);
        driver_uart_tab[6]->state = UART_TXIDLE;
        if(driver_uart_tab[6]->rx_hook != 0)
        {
            driver_uart_tab[6]->rx_hook();
        }
        return (int8_t)1;
    }
    else
    {
        *pdata = data;
        return (int8_t)0;
    }
}
#endif
/**
 *****************************************************************************
 * \fn          static int8_t driver_uart0_recv_isr(uint8_t* data)
 * \brief       接收处理.
 * \note        接收中断服务函数调用该函数,如果缓冲区满则丢弃新的数据.
 * \param[out]  pdata 新的数据.
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
#if DRIVER_UART0_ENABLE
static int8_t driver_uart0_recv_isr(uint8_t* data)
{
    if(driver_uart_tab[0]==0)
    {
        return (int8_t)1;
    }
    return rx_fifo_in(driver_uart_tab[0], *data);
}
#endif
/**
 *****************************************************************************
 * \fn          static int8_t driver_uart1_recv_isr(uint8_t* data)
 * \brief       接收处理.
 * \note        接收中断服务函数调用该函数,如果缓冲区满则丢弃新的数据.
 * \param[out]  pdata 新的数据.
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
#if DRIVER_UART1_ENABLE
static int8_t driver_uart1_recv_isr(uint8_t* data)
{
    if(driver_uart_tab[1]==0)
    {
        return (int8_t)1;
    }
    return rx_fifo_in(driver_uart_tab[1], *data);
}
#endif
/**
 *****************************************************************************
 * \fn          static int8_t driver_uart2_recv_isr(uint8_t* data)
 * \brief       接收处理.
 * \note        接收中断服务函数调用该函数,如果缓冲区满则丢弃新的数据.
 * \param[out]  pdata 新的数据.
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
#if DRIVER_UART2_ENABLE
static int8_t driver_uart2_recv_isr(uint8_t* data)
{
    if(driver_uart_tab[2]==0)
    {
        return (int8_t)1;
    }
    return rx_fifo_in(driver_uart_tab[2], *data);
}
#endif
/**
 *****************************************************************************
 * \fn          static int8_t driver_uart3_recv_isr(uint8_t* data)
 * \brief       接收处理.
 * \note        接收中断服务函数调用该函数,如果缓冲区满则丢弃新的数据.
 * \param[out]  pdata 新的数据.
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
#if DRIVER_UART3_ENABLE
static int8_t driver_uart3_recv_isr(uint8_t* data)
{
    if(driver_uart_tab[3]==0)
    {
        return (int8_t)1;
    }
    return rx_fifo_in(driver_uart_tab[3], *data);
}
#endif
/**
 *****************************************************************************
 * \fn          static int8_t driver_uart4_recv_isr(uint8_t* data)
 * \brief       接收处理.
 * \note        接收中断服务函数调用该函数,如果缓冲区满则丢弃新的数据.
 * \param[out]  pdata 新的数据.
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
#if DRIVER_UART4_ENABLE
static int8_t driver_uart4_recv_isr(uint8_t* data)
{
    if(driver_uart_tab[4]==0)
    {
        return (int8_t)1;
    }
    return rx_fifo_in(driver_uart_tab[4], *data);
}
#endif
/**
 *****************************************************************************
 * \fn          static int8_t driver_uart5_recv_isr(uint8_t* data)
 * \brief       接收处理.
 * \note        接收中断服务函数调用该函数,如果缓冲区满则丢弃新的数据.
 * \param[out]  pdata 新的数据.
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
#if DRIVER_UART5_ENABLE
static int8_t driver_uart5_recv_isr(uint8_t* data)
{
    if(driver_uart_tab[5]==0)
    {
        return (int8_t)1;
    }
    return rx_fifo_in(driver_uart_tab[5], *data);
}
#endif
/**
 *****************************************************************************
 * \fn          static int8_t driver_uart6_recv_isr(uint8_t* data)
 * \brief       接收处理.
 * \note        接收中断服务函数调用该函数,如果缓冲区满则丢弃新的数据.
 * \param[out]  pdata 新的数据.
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
#if DRIVER_UART6_ENABLE
static int8_t driver_uart6_recv_isr(uint8_t* data)
{
    if(driver_uart_tab[6]==0)
    {
        return (int8_t)1;
    }
    return rx_fifo_in(driver_uart_tab[6], *data);
}
#endif
/**
 *****************************************************************************
 * \fn          static int8_t driver_uart_rxerr_isr(uart_id_e id, uint8_t *pdata)
 * \brief       接收错误处理.
 * \note        接收错误中断服务函数调用该函数.
 * \param[in]   pdata 错误码
 * \return      int8_t 总是返回0
 *****************************************************************************
 */
static int8_t driver_uart_rxerr_isr(uart_id_e id, uint8_t *pdata)
{
    if(driver_uart_tab[id]==0)
    {
        return (int8_t)1;
    }
    if(*pdata & UART_ERR_PER)
    {
        if((driver_uart_tab[id]->err & (uint32_t)0x000000FF) != (uint32_t)0x000000FF)
        {
            driver_uart_tab[id]->err = driver_uart_tab[id]->err + (uint32_t)1;
        }
        else
        {
            bsp_uart_enablerxerr(id,UART_INTERRUPT,UART_DISABLE);
            //bsp_uart_enablerx(id,UART_INTERRUPT,UART_DISABLE);
        }
    }
    if(*pdata & UART_ERR_FER)
    {
        if((driver_uart_tab[id]->err & (uint32_t)0x0000FF00) != (uint32_t)0x0000FF00)
        {
            driver_uart_tab[id]->err = driver_uart_tab[id]->err + (uint32_t)0x100;
        }
        else
        {
            bsp_uart_enablerxerr(id,UART_INTERRUPT,UART_DISABLE);
            //bsp_uart_enablerx(id,UART_INTERRUPT,UART_DISABLE);
        }
    }
    if(*pdata & UART_ERR_ORER)
    {
        if((driver_uart_tab[id]->err & (uint32_t)0x00FF0000) != (uint32_t)0x00FF0000)
        {
            driver_uart_tab[id]->err = driver_uart_tab[id]->err + (uint32_t)0x10000;
        }
        else
        {
            bsp_uart_enablerxerr(id,UART_INTERRUPT,UART_DISABLE);
            //bsp_uart_enablerx(id,UART_INTERRUPT,UART_DISABLE);
        }
    }
    if(*pdata & UART_ERR_OTHER)
    {
        if((driver_uart_tab[id]->err & (uint32_t)0xFF000000) != (uint32_t)0xFF000000)
        {
            driver_uart_tab[id]->err = driver_uart_tab[id]->err + (uint32_t)0x1000000;
        }
        else
        {
            bsp_uart_enablerxerr(id,UART_INTERRUPT,UART_DISABLE);
            //bsp_uart_enablerx(id,UART_INTERRUPT,UART_DISABLE);
        }
    }
    return (int8_t)0;
}

/**
 *****************************************************************************
 * \fn          static int8_t driver_uart0_rxerr_isr(uint8_t *pdata)
 * \brief       接收错误处理.
 * \note        接收错误中断服务函数调用该函数.
 * \param[in]   pdata 错误码
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
#if DRIVER_UART0_ENABLE
static int8_t driver_uart0_rxerr_isr(uint8_t *pdata)
{
    return driver_uart_rxerr_isr(UART_ID_0,pdata);
}
#endif
/**
 *****************************************************************************
 * \fn          static int8_t driver_uart1_rxerr_isr(uint8_t *pdata)
 * \brief       接收错误处理.
 * \note        接收错误中断服务函数调用该函数.
 * \param[in]   pdata 错误码
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
#if DRIVER_UART1_ENABLE
static int8_t driver_uart1_rxerr_isr(uint8_t *pdata)
{
    return driver_uart_rxerr_isr(UART_ID_1,pdata);
}
#endif
/**
 *****************************************************************************
 * \fn          static int8_t driver_uart2_rxerr_isr(uint8_t *pdata)
 * \brief       接收错误处理.
 * \note        接收错误中断服务函数调用该函数.
 * \param[in]   pdata 错误码
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
#if DRIVER_UART2_ENABLE
static int8_t driver_uart2_rxerr_isr(uint8_t *pdata)
{
    return driver_uart_rxerr_isr(UART_ID_2,pdata);
}
#endif
/**
 *****************************************************************************
 * \fn          static int8_t driver_uart3_rxerr_isr(uint8_t *pdata)
 * \brief       接收错误处理.
 * \note        接收错误中断服务函数调用该函数.
 * \param[in]   pdata 错误码
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
#if DRIVER_UART3_ENABLE
static int8_t driver_uart3_rxerr_isr(uint8_t *pdata)
{
    return driver_uart_rxerr_isr(UART_ID_3,pdata);
}
#endif
/**
 *****************************************************************************
 * \fn          static int8_t driver_uart4_rxerr_isr(uint8_t *pdata)
 * \brief       接收错误处理.
 * \note        接收错误中断服务函数调用该函数.
 * \param[in]   pdata 错误码
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
#if DRIVER_UART4_ENABLE
static int8_t driver_uart4_rxerr_isr(uint8_t *pdata)
{
    return driver_uart_rxerr_isr(UART_ID_4,pdata);
}
#endif
/**
 *****************************************************************************
 * \fn          static int8_t driver_uart5_rxerr_isr(uint8_t *pdata)
 * \brief       接收错误处理.
 * \note        接收错误中断服务函数调用该函数.
 * \param[in]   pdata 错误码
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
#if DRIVER_UART5_ENABLE
static int8_t driver_uart5_rxerr_isr(uint8_t *pdata)
{
    return driver_uart_rxerr_isr(UART_ID_5,pdata);
}
#endif
/**
 *****************************************************************************
 * \fn          static int8_t driver_uart6_rxerr_isr(uint8_t *pdata)
 * \brief       接收错误处理.
 * \note        接收错误中断服务函数调用该函数.
 * \param[in]   pdata 错误码
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
#if DRIVER_UART6_ENABLE
static int8_t driver_uart6_rxerr_isr(uint8_t *pdata)
{
    return driver_uart_rxerr_isr(UART_ID_6,pdata);
}
#endif
/*****************************************************************************
 *                                                                           *
 *                             对外接口函数实现                              *
 *                                                                           *
 ****************************************************************************/

/**
 *****************************************************************************
 * \fn          int8_t driver_uart_init(uart_id_e id)
 * \brief       初始化串口.
 * \note        .
 * \param[in]   id 串口id \ref uart_id_e
 * \retval      0      设置成功
 * \retval      其他值 错误
 *****************************************************************************
 */
int8_t driver_uart_init(uart_id_e id)
{
    int8_t res = (int8_t)0;
    bsp_uart_init(id);
    switch(id)
    {
#if DRIVER_UART0_ENABLE
    case UART_ID_0:
        bsp_uart_settxcompletecallback(UART_ID_0,driver_uart0_send_isr);
        bsp_uart_setrxcallback(UART_ID_0,driver_uart0_recv_isr);
        bsp_uart_setrxerrcallback(UART_ID_0,driver_uart0_rxerr_isr);
    break;
#endif
#if DRIVER_UART1_ENABLE
    case UART_ID_1:
        bsp_uart_settxcompletecallback(UART_ID_1,driver_uart1_send_isr);
        bsp_uart_setrxcallback(UART_ID_1,driver_uart1_recv_isr);
        bsp_uart_setrxerrcallback(UART_ID_1,driver_uart1_rxerr_isr);
    break;
#endif
#if DRIVER_UART2_ENABLE
    case UART_ID_2:
        bsp_uart_settxcompletecallback(UART_ID_2,driver_uart2_send_isr);
        bsp_uart_setrxcallback(UART_ID_2,driver_uart2_recv_isr);
        bsp_uart_setrxerrcallback(UART_ID_2,driver_uart2_rxerr_isr);
    break;
#endif
#if DRIVER_UART3_ENABLE
    case UART_ID_3:
        bsp_uart_settxcompletecallback(UART_ID_3,driver_uart3_send_isr);
        bsp_uart_setrxcallback(UART_ID_3,driver_uart3_recv_isr);
        bsp_uart_setrxerrcallback(UART_ID_3,driver_uart3_rxerr_isr);
    break;
#endif
#if DRIVER_UART4_ENABLE
    case UART_ID_4:
        bsp_uart_settxcompletecallback(UART_ID_4,driver_uart4_send_isr);
        bsp_uart_setrxcallback(UART_ID_4,driver_uart4_recv_isr);
        bsp_uart_setrxerrcallback(UART_ID_4,driver_uart4_rxerr_isr);
    break;
#endif
#if DRIVER_UART5_ENABLE
    case UART_ID_5:
        bsp_uart_settxcompletecallback(UART_ID_5,driver_uart5_send_isr);
        bsp_uart_setrxcallback(UART_ID_5,driver_uart5_recv_isr);
        bsp_uart_setrxerrcallback(UART_ID_5,driver_uart5_rxerr_isr);
    break;
#endif
#if DRIVER_UART6_ENABLE
    case UART_ID_6:
        bsp_uart_settxcompletecallback(UART_ID_6,driver_uart6_send_isr);
        bsp_uart_setrxcallback(UART_ID_6,driver_uart6_recv_isr);
        bsp_uart_setrxerrcallback(UART_ID_6,driver_uart6_rxerr_isr);
    break;
#endif
    default:
       res = (int8_t)-1;
    break;
    }
    return res;
}

/**
 *****************************************************************************
 * \fn          int8_t driver_uart_deinit(uart_id_e id)
 * \brief       解除初始化串口.
 * \note        .
 * \param[in]   id 串口id \ref uart_id_e
 * \retval      0      设置成功
 * \retval      其他值 错误
 *****************************************************************************
 */
int8_t driver_uart_deinit(uart_id_e id)
{
    bsp_uart_init(id);
    return 0;
}

/**
 *****************************************************************************
 * \fn          uint16_t driver_uart_getrxlen(uart_id_e id)
 * \brief       获取接收缓冲区有效数据长度.
 * \note        .
 * \param[in]   id 串口id \ref uart_id_e
 * \return      uint16_t 接收缓冲区长度
 *****************************************************************************
 */
uint16_t driver_uart_getrxlen(uart_id_e id)
{
    if(driver_uart_tab[id]==0)
    {
        return (uint16_t)0;
    }
    return driver_uart_tab[id]->rx_len;
}

/**
 *****************************************************************************
 * \fn          int8_t driver_uart_flush(uart_id_e id)
 * \brief       清除缓冲区.
 * \note        .
 * \param[in]   id 串口id \ref uart_id_e
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
int8_t driver_uart_flush(uart_id_e id)
{
    if(driver_uart_tab[id]!=0)
    {
        driver_uart_tab[id]->rx_in  = (uint16_t)0;
        driver_uart_tab[id]->rx_out = (uint16_t)0;
        driver_uart_tab[id]->rx_len = (uint16_t)0;
        driver_uart_tab[id]->tx_in  = (uint16_t)0;
        driver_uart_tab[id]->tx_out = (uint16_t)0;
        driver_uart_tab[id]->tx_len = (uint16_t)0;
        driver_uart_tab[id]->err = (uint32_t)0x00;
    }
    return (int8_t)0;
}

/**
 *****************************************************************************
 * \fn          int8_t driver_uart_set(uart_cfg_t* cfg)
 * \brief       配置串口参数.
 * \note        .
 * \param[in]   cfg \ref uart_cfg_t 配置参数
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
int8_t driver_uart_set(uart_cfg_t* cfg)
{
    int8_t ret;
    if(driver_uart_tab[cfg->id]==0)
    {
        return (int8_t)-1;
    }
    //driver_uart_init(cfg->id);
    ret = bsp_uart_setcfg(cfg->id,cfg);
    driver_uart_flush(cfg->id);
    bsp_uart_enablerx(cfg->id,UART_INTERRUPT,UART_ENABLE);
    bsp_uart_enablerxerr(cfg->id,UART_INTERRUPT,UART_ENABLE);
    return ret;
}

/**
 *****************************************************************************
 * \fn          uint16_t driver_uart_recv(uart_id_e id, uint8_t *pdata, uint16_t len, uint32_t timeout, int8_t *erro);
 * \brief       串口接收指定长度数据,可以设置超时时间.
 * \note        应用调用该函数接收数据,如果缓冲区无数据就等待.
 * \param[in]   id \ref uart_id_e 指定的串口id.
 * \param[out]  pdata 需存储接收数据的缓冲区.
 * \param[in]   len   接收的数据长度.
 * \param[in]   timeout 设置为0表示一直等到接收到指定长度数据,设置为其他值表示等待指定超时时间(单位mS)
 * \param[out]  perro 错误码 0表示成功 -1表示参数错误 1表示超时
 * \return      uint16_t 返回实际读到的数据长度
 *****************************************************************************
 */
uint16_t driver_uart_recv(uart_id_e id, uint8_t *pdata, uint16_t len, uint32_t timeout, int8_t *erro)
{
    uint8_t data = (uint8_t)0;
    uint16_t readlen = (uint16_t)0;
    uint32_t tmptimeout = (uint32_t)0;
    if (id >= UART_ID_MAX)
    {
        *erro = (int8_t)-1;
        return (uint16_t)0;
    }
    if(driver_uart_tab[id]==0)
    {
        *erro = (int8_t)-1;
        return (uint16_t)0;
    }
    if(len ==0)
    {
        *erro = (int8_t)-1;
        return (uint16_t)0;
    }
    tmptimeout = timeout;
    while(len !=(uint16_t)0)
    {
        data = rx_fifo_out(driver_uart_tab[id], tmptimeout, erro);
        if (*erro == (int8_t)0)
        {
            tmptimeout = DRIVER_UART_DELAY_FRAME;
            *pdata++ = data;
            len--;
            readlen++;
        }
        else
        {
            *erro = (uint16_t)1;
            break;
        }
    }
    return readlen;
}

/**
 *****************************************************************************
 * \fn          int16_t driver_uart_send(uart_id_e id, uuint8_t *pdata, uint16_t len,uint32_t timeout,int8_t *erro);
 * \brief       串口发送指定长度数据,可以设置超时时间.
 * \note        应用调用该函数发送数据,如果缓冲区满就等待.
 * \param[in]   id \ref uart_id_e 指定的串口id.
 * \param[in]   pdata 需要发送的数据缓冲区.
 * \param[in]   len   需要发送的数据长度.
 * \param[in]   timeout 设置为0表示一直等到全部写入缓冲区,设置为其他值表示等待指定超时时间(单位mS)
 * \param[out]  perro 错误码 0表示成功 -1表示参数错误 1表示超时
 * \return      int16_t 返回实际发送的数据
 *****************************************************************************
 */
int16_t driver_uart_send(uart_id_e id, uint8_t *pdata, uint16_t len,uint32_t timeout,int8_t *erro)
{
    int8_t full = (int8_t)0;
    driver_uart_t *p=0;
    uint8_t data = (uint8_t)0;
    uint16_t sendlen = (uint16_t)0;
    OSCriticalAlloc();
    if (id >= UART_ID_MAX)
    {
        *erro = (int8_t)-1;
        return (uint16_t)0;
    }
    if(driver_uart_tab[id]==0)
    {
        *erro = (int8_t)-1;
        return (uint16_t)0;
    }
    if(len ==0)
    {
        *erro = (int8_t)-1;
        return (uint16_t)0;
    }
    p = (driver_uart_t*)driver_uart_tab[id];
    if(p->tx_hook !=0)
    {
        p->tx_hook();   /*调用准备发送回调函数*/
    }
    if((uint32_t)0 == timeout)
    {
        while(len != (uint16_t)0)
        {
            full = tx_fifo_in(p,*pdata,DRIVER_UART_DELAY_CHECK);
            if (full == (int8_t)0)
            {
                pdata++;
                len--;
                sendlen++;
            }
            else
            {
                OSCriticalEnter();
                /*如果缓冲区满而且当前串口没有处于发送状态要先发送一个字节触发发送中断流*/
                if(p->state == UART_TXIDLE)
                {
                    data = tx_fifo_out(p, erro);
                    if(*erro == (int8_t)0)
                    {
                        if(p->tx_hook !=0)
                        {
                            p->tx_hook();   /*调用准备发送回调函数*/
                        }
                            p->state = UART_TXBUSY; /*注意:p->state的读写需要临界保护*/
                            bsp_uart_send(p->id,data);  /*注意:必须放在使能TX前,写TDR会清除TEND标志,否则使能TE会马上产生发送完中断*/
                            //bsp_uart_enabletx(id,UART_INTERRUPT,UART_ENABLE);
                    }
                }
                OSCriticalExit();
                OsTimeDelay(DRIVER_UART_DELAY_CHECK);
            }
        }
        *erro = (int8_t)0;
    }
    else
    {
        while ((len != (uint16_t)0) && (timeout != (uint32_t)0))
        {
            full = tx_fifo_in(p,*pdata,DRIVER_UART_DELAY_CHECK);
            if (full == (int8_t)0)
            {
                pdata++;
                len--;
                sendlen++;
            }
            else
            {
                OSCriticalEnter();
                /*如果缓冲区满而且当前串口没有处于发送状态要先发送一个字节触发发送中断流*/
                if(p->state == UART_TXIDLE)
                {
                    data = tx_fifo_out(p, erro);
                    if(*erro == (int8_t)00)
                    {
                        if(p->tx_hook !=0)
                        {
                            p->tx_hook();   /*调用准备发送回调函数*/
                        }
                            p->state = UART_TXBUSY; /*注意:p->state的读写需要临界保护*/
                            //bsp_uart_enabletx(id,UART_INTERRUPT,UART_ENABLE);
                            bsp_uart_send(p->id,data);  /*注意:必须放在使能TX前,写TDR会清除TEND标志,否则使能TE会马上产生发送完中断*/
                            //bsp_uart_enabletxirq(id,UART_INTERRUPT,UART_ENABLE);
                    }
                    OSCriticalExit();
                }
                else /*如果当前处于发送状态*/
                {
                    OSCriticalExit();
                    /*如果缓冲区满而且串口处于发送状态则延时指定时间间隔等待中断发送完*/
                    if (timeout >= DRIVER_UART_DELAY_CHECK)
                    {
                        timeout -= DRIVER_UART_DELAY_CHECK;
                        OsTimeDelay(DRIVER_UART_DELAY_CHECK);
                    }
                    else
                    {
                        /*不足一个时间间隔延时剩余时间*/
                        OsTimeDelay(timeout);
                        timeout = (uint32_t)00;
                    }
                }
            }
        }
        /*如果没发送完设置超时错误码*/
        if(len != (uint16_t)0)
        {
            *erro = (int8_t)1;
        }
    }
    OSCriticalEnter();
    /*如果缓冲区有数据而且当前串口没有处于发送状态要先发送一个字节触发发送中断流*/
    if((p->state == UART_TXIDLE) && (p->tx_len!=0))
    {
        data = tx_fifo_out(p, erro);
        if(*erro == (int8_t)0)
        {
            if(p->tx_hook !=0)
            {
                p->tx_hook();   /*调用准备发送回调函数*/
            }
            p->state = UART_TXBUSY; /*注意:p->state的读写需要临界保护*/
            //bsp_uart_enabletx(id,UART_INTERRUPT,UART_ENABLE);
            bsp_uart_send(p->id,data);  /*注意:必须放在使能TX前,写TDR会清除TEND标志,否则使能TE会马上产生发送完中断*/
            //bsp_uart_enabletxirq(id,UART_INTERRUPT,UART_ENABLE);
        }
    }
    OSCriticalExit();
    return sendlen;
}

/**
 *****************************************************************************
 * \fn          int8_t driver_uart_geterr(uart_id_e id, uint32_t* err);
 * \brief       读串口错误状态字.
 * \note        应用调用该函数用于判断串口是否异常,如果err的低三个字节有任意个字节为FF则表示异常需要处理.
 * \param[in]   id 指定的串口id \ref uart_id_e.
 * \param[out]  err 存储错误字
 *              - 0xxxxxxxFF表示校验错误
 *              - 0xxxxxFFxx表示帧错误
 *              - 0xxxFFxxxx表示溢出错误
 *              - 0xFF00xxxx表示其他错误
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
int8_t driver_uart_geterr(uart_id_e id, uint32_t* err)
{
    if(driver_uart_tab[id]==0)
    {
        return (int8_t)-1;
    }
    *err = driver_uart_tab[id]->err;
    return (int8_t)0;
}

