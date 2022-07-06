
/**
 *****************************************************************************
 * \brief       ������(DRIVER)����ģ��(UART)��ؽӿ�ʵ��.
 * \details     Copyright (c) 2019,spacety.
 *              All rights reserved.
 * \file        driver_uart.c
 * \author      lihongjie@spacety.cn
 * \version     1.0
 * \date        2019-08-28
 * \note        ʹ��ǰ�ο�ע��.\n
 *              ����bsp_uart,OsTimeDelay OSCriticalAlloc OSCriticalEnter OSCriticalExit\n
 * \since       lihongjie@spacety.cn 2019-8-28 1.0  �½�
 * \par �޶���¼
 * - 2019-08-28 ��ʼ�汾
 * \par ��Դ˵��
 * - RAM:
 * - ROM:
 *****************************************************************************
 */
#define DRIVER_UART_GLOBALS /**< ����driver_uart.h�е����ݽ������*/
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
 *                             �ڲ�����                                      *
 *                                                                           *
 ****************************************************************************/


/*****************************************************************************
 *                                                                           *
 *                             �ڲ��ӿں���ʵ��                              *
 *                                                                           *
 ****************************************************************************/

/*�ص�����*/
#if DRIVER_UART0_ENABLE
static void uart0_txhook(void)
{
/*485����Ϊ����*/
}

static void uart0_rxhook(void)
{
/*485����Ϊ����*/
}
#endif
#if DRIVER_UART1_ENABLE
static void uart1_txhook(void)
{
/*485����Ϊ����*/
}

static void uart1_rxhook(void)
{
/*485����Ϊ����*/
}
#endif
#if DRIVER_UART2_ENABLE
static void uart2_txhook(void)
{
/*485����Ϊ����*/
}

static void uart2_rxhook(void)
{
/*485����Ϊ����*/
}
#endif
#if DRIVER_UART3_ENABLE
static void uart3_txhook(void)
{
/*485����Ϊ����*/
}

static void uart3_rxhook(void)
{
/*485����Ϊ����*/
}
#endif
#if DRIVER_UART4_ENABLE
static void uart4_txhook(void)
{
/*485����Ϊ����*/
}

static void uart4_rxhook(void)
{
/*485����Ϊ����*/
}
#endif
#if DRIVER_UART5_ENABLE
static void uart5_txhook(void)
{
/*485����Ϊ����*/
}

static void uart5_rxhook(void)
{
/*485����Ϊ����*/
}
#endif
#if DRIVER_UART6_ENABLE
static void uart6_txhook(void)
{
/*485����Ϊ����*/
}

static void uart6_rxhook(void)
{
/*485����Ϊ����*/
}
#endif

/**
 *****************************************************************************
 * \fn          static int8_t rx_fifo_in(driver_uart_t *p, uint8_t data)
 * \brief       д���ݵ����ջ�����.
 * \note        �����жϷ��������øú���д���ݵ����ջ�����,����������д��.
 * \param[in]   p ָ��uart�ṹ��ʵ����ָ�� \ref driver_uart_t
 * \param[in]   data ָ��Ҫд��������������
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
 *****************************************************************************
 */
static int8_t rx_fifo_in(driver_uart_t *p, uint8_t data)
{
    if(0 == p)
    {
        return (int8_t)-1;
    }
    /*������δ��д������*/
    if (p->rx_len < (p->rxbuf_len/sizeof(p->rx_buf[0])))
    {
        p->rx_buf[p->rx_in++] = data;                   /*д������,д��ָ�����*/
        p->rx_in %= p->rxbuf_len/sizeof(p->rx_buf[0]);  /*�߽紦��,ʵ��Բ�λ�����*/
        p->rx_len++;                                    /*���ݸ�������*/
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
 * \brief       �ӽ��ջ�����������.
 * \note        Ӧ�ú������øú����ӽ��ջ�����������,����ָ����ʱʱ��.
 * \param[in]   p ָ��uart�ṹ��ʵ����ָ�� \ref driver_uart_t
 * \param[in]   timeout ָ����ʱʱ�� 0��ʾһֱ�ȴ�
 * \param[out]  perro ������0��ʾ���������� 1��ʾû�ж�������
 * \return      uint8_t �������ֽ�����.
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
    /*timeout==0 ��ʾһֱ�ȴ�ֱ��������*/
    if (timeout == 0)
    {
        while(p->rx_len == 0)
        {
            OsTimeDelay(DRIVER_UART_DELAY_CHECK);  /*��ʱ*/
        }
    }
    else /*timeout!=0 ��ʾ�ȴ�ָ����ʱʱ��*/
    {
        while (timeout != 0)
        {
            if (p->rx_len == 0)
            {
                /*���������,��ʱָ��ʱ����*/
                if (timeout >= DRIVER_UART_DELAY_CHECK)
                {
                    timeout -= DRIVER_UART_DELAY_CHECK;
                    OsTimeDelay(DRIVER_UART_DELAY_CHECK);
                }
                else
                {
                     /*����һ��ʱ������ʱʣ��ʱ��*/
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
    /*�ٽ�δ��� ��ֹ��ʱ�ж��ڲ���������*/
    OSCriticalEnter();
    if (p->rx_len != 0)
    {
        /*�ӻ�������������*/
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
 * \brief       д���ݵ����ͻ�����.
 * \note        Ӧ�õ��øú�����д���ݵ����ͻ�����,���������ȴ�.
 * \param[in]   p ָ��uart�ṹ��ʵ����ָ�� \ref driver_uart_t
 * \param[in]   timeout �趨��ʱʱ�� 0��ʾһֱ�ȴ�
 * \param[in]   data ָ��Ҫд��������������
 * \retval      0 д�ɹ�
 * \retval      1 дʧ��
 *****************************************************************************
 */
static int8_t tx_fifo_in(driver_uart_t *p, uint8_t data, uint32_t timeout)
{
    /*ע��ȷ��p�������Ķ�д�����ٽ�α��� */
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
                /*û������д������*/
                p->tx_buf[p->tx_in++] = data;
                p->tx_in %= p->txbuf_len/sizeof(p->tx_buf[0]);
                p->tx_len++;
                OSCriticalExit();
                break;
            }
            else
            {
                /*����ȴ�*/
                OSCriticalExit();
                OsTimeDelay(DRIVER_UART_DELAY_CHECK);  /*��ʱ*/
            }
        }
    }
    while (timeout != 0)
    {
        OSCriticalEnter();
        if (p->tx_len < (p->txbuf_len/sizeof(p->tx_buf[0])))
        {
            /*û������д������*/
            p->tx_buf[p->tx_in++] = data;
            p->tx_in %= p->txbuf_len/sizeof(p->tx_buf[0]);
            p->tx_len++;
            OSCriticalExit();
            ret = (int8_t)0; /*������ȷ����ֵ*/
            break;
        }
        else
        {
            OSCriticalExit();
            /*���������,��ʱָ��ʱ����*/
            if (timeout >= DRIVER_UART_DELAY_CHECK)
            {
                timeout -= DRIVER_UART_DELAY_CHECK;
                OsTimeDelay(DRIVER_UART_DELAY_CHECK);
                ret = (int8_t)1; /*���ó�ʱ���� ֮ǰ����û����ret=1 ��timeoutΪDRIVER_UART_DELAY_CHECK��������ʱ���ǻ᷵��0*/
            }
            else
            {
                /*����һ��ʱ������ʱʣ��ʱ��*/
                OsTimeDelay(timeout);
                timeout = (uint32_t)0;
                ret = (int8_t)1; /*���ó�ʱ����*/
            }
        }
    }
    return ret;
}

/**
 *****************************************************************************
 * \fn          static uint8_t tx_fifo_out(driver_uart_t *p, int8_t *perro)
 * \brief       �ӷ��ͻ�����������.
 * \note        �����жϷ��������øú�����ȡ���ͻ���������,���ڷ���.
 * \param[in]   p ָ��uart�ṹ��ʵ����ָ�� \ref driver_uart_t
 * \param[out]  perro ������ 0��ʾ�������� 1��ʾû������
 * \return      uint8_t perro=0ʱ��ʾ����������
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
 * \brief       ���ʹ���.
 * \note        �����жϷ��������øú���,�������������ֹͣ���Ͳ����ûص�����.
 * \param[out]  pdata ��д��Ҫ���͵�����
 * \retval      int8_t 0 pdataָ���������Ч(��Ҫ����) 1�޺�����������
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
        /*û�л�ȡ������,ֹͣ����*/
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
 * \brief       ���ʹ���.
 * \note        �����жϷ��������øú���,�������������ֹͣ���Ͳ����ûص�����.
 * \param[out]  pdata ��д��Ҫ���͵�����
 * \retval      int8_t 0 pdataָ���������Ч(��Ҫ����) 1�޺�����������
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
        /*û�л�ȡ������,ֹͣ����*/
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
 * \brief       ���ʹ���.
 * \note        �����жϷ��������øú���,�������������ֹͣ���Ͳ����ûص�����.
 * \param[out]  pdata ��д��Ҫ���͵�����
 * \retval      int8_t 0 pdataָ���������Ч(��Ҫ����) 1�޺�����������
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
        /*û�л�ȡ������,ֹͣ����*/
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
 * \brief       ���ʹ���.
 * \note        �����жϷ��������øú���,�������������ֹͣ���Ͳ����ûص�����.
 * \param[out]  pdata ��д��Ҫ���͵�����
 * \retval      int8_t 0 pdataָ���������Ч(��Ҫ����) 1�޺�����������
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
        /*û�л�ȡ������,ֹͣ����*/
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
 * \brief       ���ʹ���.
 * \note        �����жϷ��������øú���,�������������ֹͣ���Ͳ����ûص�����.
 * \param[out]  pdata ��д��Ҫ���͵�����
 * \retval      int8_t 0 pdataָ���������Ч(��Ҫ����) 1�޺�����������
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
        /*û�л�ȡ������,ֹͣ����*/
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
 * \brief       ���ʹ���.
 * \note        �����жϷ��������øú���,�������������ֹͣ���Ͳ����ûص�����.
 * \param[out]  pdata ��д��Ҫ���͵�����
 * \retval      int8_t 0 pdataָ���������Ч(��Ҫ����) 1�޺�����������
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
        /*û�л�ȡ������,ֹͣ����*/
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
 * \brief       ���ʹ���.
 * \note        �����жϷ��������øú���,�������������ֹͣ���Ͳ����ûص�����.
 * \param[out]  pdata ��д��Ҫ���͵�����
 * \retval      int8_t 0 pdataָ���������Ч(��Ҫ����) 1�޺�����������
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
        /*û�л�ȡ������,ֹͣ����*/
        //bsp_uart_enabletx(driver_uart_tab[6]->id,UART_INTERRUPT,UART_DISABLE);  /* ������ܻ�û�������ֹͣ�������һ�����ݷ��Ͳ���ȥ */
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
 * \brief       ���մ���.
 * \note        �����жϷ��������øú���,����������������µ�����.
 * \param[out]  pdata �µ�����.
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
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
 * \brief       ���մ���.
 * \note        �����жϷ��������øú���,����������������µ�����.
 * \param[out]  pdata �µ�����.
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
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
 * \brief       ���մ���.
 * \note        �����жϷ��������øú���,����������������µ�����.
 * \param[out]  pdata �µ�����.
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
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
 * \brief       ���մ���.
 * \note        �����жϷ��������øú���,����������������µ�����.
 * \param[out]  pdata �µ�����.
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
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
 * \brief       ���մ���.
 * \note        �����жϷ��������øú���,����������������µ�����.
 * \param[out]  pdata �µ�����.
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
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
 * \brief       ���մ���.
 * \note        �����жϷ��������øú���,����������������µ�����.
 * \param[out]  pdata �µ�����.
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
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
 * \brief       ���մ���.
 * \note        �����жϷ��������øú���,����������������µ�����.
 * \param[out]  pdata �µ�����.
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
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
 * \brief       ���մ�����.
 * \note        ���մ����жϷ��������øú���.
 * \param[in]   pdata ������
 * \return      int8_t ���Ƿ���0
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
 * \brief       ���մ�����.
 * \note        ���մ����жϷ��������øú���.
 * \param[in]   pdata ������
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
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
 * \brief       ���մ�����.
 * \note        ���մ����жϷ��������øú���.
 * \param[in]   pdata ������
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
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
 * \brief       ���մ�����.
 * \note        ���մ����жϷ��������øú���.
 * \param[in]   pdata ������
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
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
 * \brief       ���մ�����.
 * \note        ���մ����жϷ��������øú���.
 * \param[in]   pdata ������
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
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
 * \brief       ���մ�����.
 * \note        ���մ����жϷ��������øú���.
 * \param[in]   pdata ������
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
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
 * \brief       ���մ�����.
 * \note        ���մ����жϷ��������øú���.
 * \param[in]   pdata ������
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
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
 * \brief       ���մ�����.
 * \note        ���մ����жϷ��������øú���.
 * \param[in]   pdata ������
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
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
 *                             ����ӿں���ʵ��                              *
 *                                                                           *
 ****************************************************************************/

/**
 *****************************************************************************
 * \fn          int8_t driver_uart_init(uart_id_e id)
 * \brief       ��ʼ������.
 * \note        .
 * \param[in]   id ����id \ref uart_id_e
 * \retval      0      ���óɹ�
 * \retval      ����ֵ ����
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
 * \brief       �����ʼ������.
 * \note        .
 * \param[in]   id ����id \ref uart_id_e
 * \retval      0      ���óɹ�
 * \retval      ����ֵ ����
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
 * \brief       ��ȡ���ջ�������Ч���ݳ���.
 * \note        .
 * \param[in]   id ����id \ref uart_id_e
 * \return      uint16_t ���ջ���������
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
 * \brief       ���������.
 * \note        .
 * \param[in]   id ����id \ref uart_id_e
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
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
 * \brief       ���ô��ڲ���.
 * \note        .
 * \param[in]   cfg \ref uart_cfg_t ���ò���
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
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
 * \brief       ���ڽ���ָ����������,�������ó�ʱʱ��.
 * \note        Ӧ�õ��øú�����������,��������������ݾ͵ȴ�.
 * \param[in]   id \ref uart_id_e ָ���Ĵ���id.
 * \param[out]  pdata ��洢�������ݵĻ�����.
 * \param[in]   len   ���յ����ݳ���.
 * \param[in]   timeout ����Ϊ0��ʾһֱ�ȵ����յ�ָ����������,����Ϊ����ֵ��ʾ�ȴ�ָ����ʱʱ��(��λmS)
 * \param[out]  perro ������ 0��ʾ�ɹ� -1��ʾ�������� 1��ʾ��ʱ
 * \return      uint16_t ����ʵ�ʶ��������ݳ���
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
 * \brief       ���ڷ���ָ����������,�������ó�ʱʱ��.
 * \note        Ӧ�õ��øú�����������,������������͵ȴ�.
 * \param[in]   id \ref uart_id_e ָ���Ĵ���id.
 * \param[in]   pdata ��Ҫ���͵����ݻ�����.
 * \param[in]   len   ��Ҫ���͵����ݳ���.
 * \param[in]   timeout ����Ϊ0��ʾһֱ�ȵ�ȫ��д�뻺����,����Ϊ����ֵ��ʾ�ȴ�ָ����ʱʱ��(��λmS)
 * \param[out]  perro ������ 0��ʾ�ɹ� -1��ʾ�������� 1��ʾ��ʱ
 * \return      int16_t ����ʵ�ʷ��͵�����
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
        p->tx_hook();   /*����׼�����ͻص�����*/
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
                /*��������������ҵ�ǰ����û�д��ڷ���״̬Ҫ�ȷ���һ���ֽڴ��������ж���*/
                if(p->state == UART_TXIDLE)
                {
                    data = tx_fifo_out(p, erro);
                    if(*erro == (int8_t)0)
                    {
                        if(p->tx_hook !=0)
                        {
                            p->tx_hook();   /*����׼�����ͻص�����*/
                        }
                            p->state = UART_TXBUSY; /*ע��:p->state�Ķ�д��Ҫ�ٽ籣��*/
                            bsp_uart_send(p->id,data);  /*ע��:�������ʹ��TXǰ,дTDR�����TEND��־,����ʹ��TE�����ϲ����������ж�*/
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
                /*��������������ҵ�ǰ����û�д��ڷ���״̬Ҫ�ȷ���һ���ֽڴ��������ж���*/
                if(p->state == UART_TXIDLE)
                {
                    data = tx_fifo_out(p, erro);
                    if(*erro == (int8_t)00)
                    {
                        if(p->tx_hook !=0)
                        {
                            p->tx_hook();   /*����׼�����ͻص�����*/
                        }
                            p->state = UART_TXBUSY; /*ע��:p->state�Ķ�д��Ҫ�ٽ籣��*/
                            //bsp_uart_enabletx(id,UART_INTERRUPT,UART_ENABLE);
                            bsp_uart_send(p->id,data);  /*ע��:�������ʹ��TXǰ,дTDR�����TEND��־,����ʹ��TE�����ϲ����������ж�*/
                            //bsp_uart_enabletxirq(id,UART_INTERRUPT,UART_ENABLE);
                    }
                    OSCriticalExit();
                }
                else /*�����ǰ���ڷ���״̬*/
                {
                    OSCriticalExit();
                    /*��������������Ҵ��ڴ��ڷ���״̬����ʱָ��ʱ�����ȴ��жϷ�����*/
                    if (timeout >= DRIVER_UART_DELAY_CHECK)
                    {
                        timeout -= DRIVER_UART_DELAY_CHECK;
                        OsTimeDelay(DRIVER_UART_DELAY_CHECK);
                    }
                    else
                    {
                        /*����һ��ʱ������ʱʣ��ʱ��*/
                        OsTimeDelay(timeout);
                        timeout = (uint32_t)00;
                    }
                }
            }
        }
        /*���û���������ó�ʱ������*/
        if(len != (uint16_t)0)
        {
            *erro = (int8_t)1;
        }
    }
    OSCriticalEnter();
    /*��������������ݶ��ҵ�ǰ����û�д��ڷ���״̬Ҫ�ȷ���һ���ֽڴ��������ж���*/
    if((p->state == UART_TXIDLE) && (p->tx_len!=0))
    {
        data = tx_fifo_out(p, erro);
        if(*erro == (int8_t)0)
        {
            if(p->tx_hook !=0)
            {
                p->tx_hook();   /*����׼�����ͻص�����*/
            }
            p->state = UART_TXBUSY; /*ע��:p->state�Ķ�д��Ҫ�ٽ籣��*/
            //bsp_uart_enabletx(id,UART_INTERRUPT,UART_ENABLE);
            bsp_uart_send(p->id,data);  /*ע��:�������ʹ��TXǰ,дTDR�����TEND��־,����ʹ��TE�����ϲ����������ж�*/
            //bsp_uart_enabletxirq(id,UART_INTERRUPT,UART_ENABLE);
        }
    }
    OSCriticalExit();
    return sendlen;
}

/**
 *****************************************************************************
 * \fn          int8_t driver_uart_geterr(uart_id_e id, uint32_t* err);
 * \brief       �����ڴ���״̬��.
 * \note        Ӧ�õ��øú��������жϴ����Ƿ��쳣,���err�ĵ������ֽ���������ֽ�ΪFF���ʾ�쳣��Ҫ����.
 * \param[in]   id ָ���Ĵ���id \ref uart_id_e.
 * \param[out]  err �洢������
 *              - 0xxxxxxxFF��ʾУ�����
 *              - 0xxxxxFFxx��ʾ֡����
 *              - 0xxxFFxxxx��ʾ�������
 *              - 0xFF00xxxx��ʾ��������
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
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

