/**
 *****************************************************************************
 * \brief       BSP层(BSP)串口模块(UART)相关数据结构和接口实现.
 * \details     Copyright (c) 2022,qinyunti@hotmail.com.
 *              All rights reserved.
 * \file        bsp_uart.h
 * \author      qinyunti@hotmail.com
 * \version     1.0
 * \date        2022-07-03
 * \note        使用前参考注释.\n
 *              .\n
 * \since       qinyunti@hotmail.com 2022-07-03 1.0  新建
 * \par 修订记录
 * - 2022-07-03 初始版本
 * \par 资源说明
 * - RAM:
 * - ROM:
 *****************************************************************************
 */

#include "bsp_api.h"
#include "bsp_uart.h"
#include <stdint.h>

#define SCI_USE_FIFO 0

/*****************************************************************************
 *                                                                           *
 *                             内部数据                                                                             *
 *                                                                           *
 ****************************************************************************/

/**
 *****************************************************************************
 * \brief       回调函数全局数组 \ref uart_isrcallback_pfun_t.
 * \note        .
 *****************************************************************************
 */
uart_isrcallback_pfun_t s_callbacklists[] =
{
    {UART_ID_0,(uart_isrcallback_pfun)0,(uart_isrcallback_pfun)0,(uart_isrcallback_pfun)0,(uart_isrcallback_pfun)0},
    {UART_ID_1,(uart_isrcallback_pfun)0,(uart_isrcallback_pfun)0,(uart_isrcallback_pfun)0,(uart_isrcallback_pfun)0},
    {UART_ID_2,(uart_isrcallback_pfun)0,(uart_isrcallback_pfun)0,(uart_isrcallback_pfun)0,(uart_isrcallback_pfun)0},
    {UART_ID_3,(uart_isrcallback_pfun)0,(uart_isrcallback_pfun)0,(uart_isrcallback_pfun)0,(uart_isrcallback_pfun)0},
    {UART_ID_4,(uart_isrcallback_pfun)0,(uart_isrcallback_pfun)0,(uart_isrcallback_pfun)0,(uart_isrcallback_pfun)0},
    {UART_ID_5,(uart_isrcallback_pfun)0,(uart_isrcallback_pfun)0,(uart_isrcallback_pfun)0,(uart_isrcallback_pfun)0},
    {UART_ID_6,(uart_isrcallback_pfun)0,(uart_isrcallback_pfun)0,(uart_isrcallback_pfun)0,(uart_isrcallback_pfun)0},
    {UART_ID_7,(uart_isrcallback_pfun)0,(uart_isrcallback_pfun)0,(uart_isrcallback_pfun)0,(uart_isrcallback_pfun)0},
};

/*****************************************************************************
 *                                                                           *
 *                             内部接口函数实现                                                              *
 *                                                                           *
 ****************************************************************************/

/*****************************************************************************
 *                                                                           *
 *                             中断处理函数                                                                     *
 *                                                                           *
 ****************************************************************************/

/**
 *****************************************************************************
 * \fn          void sci_uart6_rxi_isr(void)
 * \brief        USART6 接收中断处理函数.
 * \note        .
 *****************************************************************************
 */
void sci_uart6_rxi_isr(void)
{
    uint8_t param;
    uint8_t status = 0;
    status = R_SCI6->SSR;
    /*接收处理*/
    if(status & (uint8_t)1<<6)
    {
        R_SCI6->SSR = status & (~(uint8_t)1<<6); /* 写0清标志 */
        param  = R_SCI6->RDR;  /* 读  */
        if(s_callbacklists[6].rxfun != 0)
        {
            s_callbacklists[6].rxfun(&param);
        }
    }
    R_ICU->IELSR_b[SCI6_RXI_IRQn].IR = 0;
}

/**
 *****************************************************************************
 * \fn          void sci_uart6_txi_isr(void)
 * \brief        USART6发送空中断处理函数.
 * \note        .
 *****************************************************************************
 */
void sci_uart6_txi_isr(void)
{
    /*发送空处理*/
    uint8_t param;
    uint8_t status = 0;
    status = R_SCI6->SSR;
    if(status & (uint8_t)1<<7)
    {
        R_SCI6->SSR = status & (uint8_t)(~(uint8_t)((uint8_t)1<<7)); /* 写0清标志 */
        if((s_callbacklists[6].txcmpfun != 0) && (s_callbacklists[6].txcmpfun(&param) == 0))
        {
            R_SCI6->TDR = param;  /* 写缓冲区 */
        }
        else
        {
            /* 无数据需要继续发  这里马上TE=0 可能导致最后一个字节发送不出去 */
            //R_SCI6->SCR_b.TE = 0;
            R_SCI6->SCR_b.TIE = 0;
            R_SCI6->SCR_b.TEIE = 1;
        }
     }
    R_ICU->IELSR_b[SCI6_TXI_IRQn].IR = 0;
}

/**
 *****************************************************************************
 * \fn          void sci_uart6_tei_isr(void)
 * \brief        USART6发送完中断处理函数.
 * \note        .
 *****************************************************************************
 */
void sci_uart6_tei_isr(void)
{
    uint8_t param;
    /*发送完处理*/
    if(R_SCI6->SSR & (uint8_t)1<<2)
    {
        if((s_callbacklists[6].txcmpfun != 0) && (s_callbacklists[6].txcmpfun(&param) == 0))
        {
            R_SCI6->TDR = param;  /* 写缓冲区 */
        }
        else
        {
            /* 无数据需要继续发 */
            R_SCI6->SCR_b.TE = 0;
            R_SCI6->SCR_b.TIE = 0;
            R_SCI6->SCR_b.TEIE = 0;
        }
     }
    R_ICU->IELSR_b[SCI6_TEI_IRQn].IR = 0;
}

/**
 *****************************************************************************
 * \fn          void sci_uart6_eri_isr(void)
 * \brief        USART6 接收错误中断处理函数.
 * \note        .
 *****************************************************************************
 */
void sci_uart6_eri_isr(void)
{
    uint8_t param = (uint8_t)0;
    uint8_t status = 0;
    status = R_SCI6->SSR;
    /*1.获取错误状态并清除标志*/
    if(status & (uint8_t)1<<3)
    {
        param |= UART_ERR_PER;
    }
    if(status & (uint8_t)1<<4)
    {
        param |= UART_ERR_FER;
    }
    if(status & (uint8_t)1<<5)
    {
        param |= UART_ERR_ORER;
    }
    /*2.调用相应的回调函数*/
    if(s_callbacklists[6].rxerrfun != 0)
    {
        s_callbacklists[6].rxerrfun(&param);
    }
    /*3.清除中断标志*/
    R_SCI6->SSR = status & (~(uint8_t)1<<3)  & (~(uint8_t)1<<4)  & (~(uint8_t)1<<5);  /* 写0清标志 */
    R_ICU->IELSR_b[SCI6_ERI_IRQn].IR = 0;
}

/**
 *****************************************************************************
 * \fn          void sci_uart6_am_isr(void)
 * \brief        USART6 地址匹配中断处理函数.
 * \note        .
 *****************************************************************************
 */
void sci_uart6_am_isr(void)
{
    R_ICU->IELSR_b[SCI6_AM_IRQn].IR = 0;
}

/*****************************************************************************
 *                                                                           *
 *                             对外接口函数实现                                                              *
 *                                                                           *
 ****************************************************************************/

/**
 *****************************************************************************
 * \fn          int8_t bsp_uart_setcfg(uint8_t id, uart_cfg_t* cfg)
 * \brief       设置串口参数,调用前需要调用bsp_uart_init初始化.
 * \note        .
 * \param[in]   id \ref uart_id_e 指定的串口id.
 * \param[in]   cfg \ref uart_cfg_t 指向配置结构体的指针.
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
int8_t bsp_uart_setcfg(uint8_t id, uart_cfg_t* cfg)
{
    int8_t res = (int8_t)0;
    //uint32_t tmpreg= (uint32_t)0x00;
    //uint32_t apbclock = (uint32_t)0x00;
    //uint32_t integerdivider = (uint32_t)0x00;
    //uint32_t fractionaldivider = (uint32_t)0x00;
    switch(id)
    {
    case UART_ID_6:
        /* 关闭TE RE 再配置 */
        R_SCI6->SCR = 0x00;
#if SCI_USE_FIFO
        R_SCI6->FCR_b.FM =1;
#else
        R_SCI6->FCR_b.FM =0;
#endif
        R_SCI6->FCR_b.TFRST = 1;
        R_SCI6->FCR_b.RFRST = 1;
        R_SCI6->FCR_b.DRES = 0; /* 产生SCIn_RXI中断 */
        R_SCI6->FCR_b.TTRG = 0;
        R_SCI6->FCR_b.RTRG = 8;
        R_SCI6->FCR_b.RSTRG = 15;

        R_SCI6->SCR_b.CKE = 0; /* SCK作为普通IO */

        R_SCI6->SIMR1_b.IICM = 0; /* 非IIC */
        R_SCI6->SPMR_b.CKPH = 0;
        R_SCI6->SPMR_b.CKPH = 0;

        R_SCI6->SMR_b.CKS = 0; /* PCLKA不分频 */
        R_SCI6->SMR_b.MP = 0;  /* 非多机模式  */
        R_SCI6->SMR_b.CM = 0;  /* 异步模式 */

        R_SCI6->SCMR_b.SMIF = 0; /* 异步模式 */
        R_SCI6->SCMR_b.SINV = 0; /* 数据不倒置 */
        R_SCI6->SCMR_b.SDIR = 0; /* 低位在前 */

        R_SCI6->SEMR_b.ABCS = 0;
        R_SCI6->SEMR_b.ABCSE = 0;
        R_SCI6->SEMR_b.BGDM = 0;
        R_SCI6->SEMR_b.BRME = 0;
        R_SCI6->SEMR_b.NFEN = 0;
        R_SCI6->SEMR_b.RXDESEL = 0;

        R_SCI6->SPTR_b.ASEN = 0;
        R_SCI6->SPTR_b.ATEN = 0;
        R_SCI6->SPTR_b.RINV = 0;
        R_SCI6->SPTR_b.SPB2DT = 1;
        R_SCI6->SPTR_b.SPB2IO = 0;

        R_SCI6->ACTR = 0x00;
        /*数据长度*/
        switch(cfg->datalen)
        {
        case UART_DATA_9:
            R_SCI6->SMR_b.CHR = 0;
            R_SCI6->SCMR_b.CHR1 = 0;
        break;
        case UART_DATA_7:
            R_SCI6->SMR_b.CHR = 1;
            R_SCI6->SCMR_b.CHR1 = 1;
        break;
        default:
            R_SCI6->SMR_b.CHR = 0;
            R_SCI6->SCMR_b.CHR1 = 1;
        break;
        }
        /*校验位*/
        switch(cfg->parity)
        {
        case UART_CHECK_EVEN:
            R_SCI6->SMR_b.PE = 1;
            R_SCI6->SMR_b.PM = 0;
        break;
        case UART_CHECK_ODD:
            R_SCI6->SMR_b.PE = 1;
            R_SCI6->SMR_b.PM = 1;
        break;
        default:
            R_SCI6->SMR_b.PE = 0;
        break;
        }
        /*停止位*/
        switch(cfg->stopb)
        {
        case UART_STOPB_2:
            R_SCI6->SMR_b.STOP = 1;
        break;
        default:
            R_SCI6->SMR_b.STOP = 0;
        break;
        }
        /* 波特率 */
        uint8_t div = 0;
        uint8_t n;
        if(R_SCI6->SEMR_b.BGDM == 1)
        {
            div |= 0x04;
        }
        if(R_SCI6->SEMR_b.ABCS == 1)
        {
            div |= 0x02;
        }
        if(R_SCI6->SEMR_b.ABCSE == 1)
        {
            div |= 0x01;
        }
        switch(div)
        {
            case 0:
                div = 64;
            break;
            case 2:
            case 4:
                div = 32;
            break;
            case 6:
                div = 16;
            break;
            default:
                div = 12;
            break;
        }
        n = R_SCI6->SMR_b.CKS;
        uint8_t mul = 1;
        uint8_t N = 0;
        uint32_t B = 0;
        for(int i = 0; i<2*n; i++)
        {
            mul *= 2;
        }
        //N = PCLK x 10^6 / (div x 2^(2n-1) x B) - 1
        SystemCoreClockUpdate();
        N = BSP_STARTUP_PCLKA_HZ / (div * cfg->baud * mul/2) -1;

        B = BSP_STARTUP_PCLKA_HZ / ((N + 1) * div * mul/2); //128<<M<<256

        R_SCI6->BRR = N;
#if 1
        uint8_t mddr = 128;

        R_SCI6->SEMR_b.BRME = 1;
        if(B > cfg->baud)
        {
            mddr = ((uint32_t)256*cfg->baud/B) - 1;
            if(mddr<128)
            {
                mddr = 127;
            }
            R_SCI6->MDDR = mddr;
        }
#endif
        R_SCI6->FCR_b.TFRST = 0;
        R_SCI6->FCR_b.RFRST = 0;

        /* 中断 */
#if 0
        /* 测试用 */
        R_SCI6->SCR_b.RE = 1;
        R_SCI6->SCR_b.TE = 1;
        while(1)
        {
            if(R_SCI6->SSR_b.RDRF == 1)
            {
                R_SCI6->TDR = R_SCI6->RDR;
            }
        }
#endif
    break;
    default:
        res = (int8_t)-1;
    break;
    }
    return res;
}

/**
 *****************************************************************************
 * \fn          int8_t bsp_uart_init(uart_id_e id)
 * \brief       初始化配置时钟,引脚,中断.
 * \note        .
 * \param[in]   id \ref uart_id_e 指定的串口id.
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
int8_t bsp_uart_init(uart_id_e id)
{
    int8_t res = (int8_t)0;
    switch(id)
    {
    case UART_ID_6:  /*UART6*/
        /*使能时钟 Systeminit PCLKA已经设置分频值 */
        // R_MSTP->MSTPCRB_b.MSTPB25 = 0; 需要解锁

        R_BSP_MODULE_START(FSP_IP_SCI, 6);

        /*配置引脚*/

        /* P304作为RXD6,P305作为TXD6.*/
        R_PMISC->PWPR_b.B0WI = 0;   /* 解锁 PmnPFS */
        R_PMISC->PWPR_b.PFSWE = 1;

        /* P305外设输出 */
        R_PFS->PORT[3].PIN[5].PmnPFS_b.PMR = 1;   /* 外设 */
        R_PFS->PORT[3].PIN[5].PmnPFS_b.PDR = 1;   /* 输出 */
        R_PFS->PORT[3].PIN[5].PmnPFS_b.PCR = 1;   /* 使能上拉*/
        R_PFS->PORT[3].PIN[5].PmnPFS_b.NCODR = 0; /* CMOS模式 非开漏*/
        R_PFS->PORT[3].PIN[5].PmnPFS_b.DSCR = 3;  /* 高驱动能力 */
        R_PFS->PORT[3].PIN[5].PmnPFS_b.EOFR = 0;  /* 未使用事件模式所以写默认值 */
        R_PFS->PORT[3].PIN[5].PmnPFS_b.ISEL = 0;  /* 不使能外部中断 */
        R_PFS->PORT[3].PIN[5].PmnPFS_b.ASEL = 0;  /* 非AD */
        R_PFS->PORT[3].PIN[5].PmnPFS_b.PODR = 0;  /* 默认输出0  */
        R_PFS->PORT[3].PIN[5].PmnPFS_b.PSEL = 4;  /* TXD6 */

        /* P304外设输入 */
        R_PFS->PORT[3].PIN[4].PmnPFS_b.PMR = 1;   /* 外设 */
        R_PFS->PORT[3].PIN[4].PmnPFS_b.PDR = 0;   /* 输入 */
        R_PFS->PORT[3].PIN[4].PmnPFS_b.PCR = 1;   /* 使能上拉*/
        R_PFS->PORT[3].PIN[4].PmnPFS_b.NCODR = 0; /* CMOS模式 非开漏*/
        R_PFS->PORT[3].PIN[4].PmnPFS_b.DSCR = 3;  /* 高驱动能力 */
        R_PFS->PORT[3].PIN[4].PmnPFS_b.EOFR = 0;  /* 未使用事件模式所以写默认值 */
        R_PFS->PORT[3].PIN[4].PmnPFS_b.ISEL = 0;  /* 不使能外部中断 */
        R_PFS->PORT[3].PIN[4].PmnPFS_b.ASEL = 0;  /* 非AD */
        R_PFS->PORT[3].PIN[4].PmnPFS_b.PODR = 0;  /* 默认输出0  */
        R_PFS->PORT[3].PIN[4].PmnPFS_b.PSEL = 4;  /* RXD6 */

        R_PMISC->PWPR_b.PFSWE = 0;   /* 锁定 PmnPFS */
        R_PMISC->PWPR_b.B0WI = 1;

        /*配置中断*/
        NVIC_SetPriority(SCI6_RXI_IRQn, 2);
        NVIC_EnableIRQ(SCI6_RXI_IRQn);
        NVIC_SetPriority(SCI6_TEI_IRQn, 2);
        NVIC_EnableIRQ(SCI6_TEI_IRQn);
        NVIC_SetPriority(SCI6_TXI_IRQn, 2);
        NVIC_EnableIRQ(SCI6_TXI_IRQn);
        NVIC_SetPriority(SCI6_ERI_IRQn, 2);
        NVIC_EnableIRQ(SCI6_ERI_IRQn);
        //NVIC_SetPriority(SCI6_AM_IRQn, 2);
        //NVIC_EnableIRQ(SCI6_AM_IRQn);
    break;
    default:
        res = (int8_t)-1;
    break;
    }
    return res;
}

/**
 *****************************************************************************
 * \fn          int8_t bsp_uart_send(uart_id_e id, uint8_t val)
 * \brief       发送字节.
 * \note        .
 * \param[in]   id \ref uart_id_e 指定的串口id.
 * \param[in]   val 发送的字节数据.
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
int8_t bsp_uart_send(uart_id_e id, uint8_t val)
{
    int8_t res = (int8_t)0;
    switch(id)
    {
    case UART_ID_6:
#if SCI_USE_FIFO
        R_SCI6->FTDRL = val;
#else
        R_SCI6->SSR_b.TDRE = 0;  /* 清除标志避免马上就进入发送(完)中断 */
        R_SCI6->SCR_b.TEIE = 1;
        if(R_SCI6->SCR_b.TE == 0)
        {
            R_SCI6->SCR_b.TE = 1;
        }
        R_SCI6->TDR = val;
        //while(R_SCI6->SSR_b.TDRE == 0);
#endif
    break;
    default:
        res = (int8_t)-1;
    break;
    }
    return res;
}

/**
 *****************************************************************************
 * \fn          int8_t bsp_uart_settxcallback(uart_id_e id, uart_isrcallback_pfun callbackfun)
 * \brief        设置发送空中断回调函数.
 * \note        .
 * \param[in]   id \ref uart_id_e 指定的串口id.
 * \param[in]   callbackfun \ref  uart_isrcallback_pfun 中断回调函数.
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
int8_t bsp_uart_settxcallback(uart_id_e id, uart_isrcallback_pfun callbackfun)
{
    if(id >= sizeof(s_callbacklists)/sizeof(uart_isrcallback_pfun_t))
    {
        return (int8_t)-1;
    }
    s_callbacklists[id].txfun = callbackfun;
    return (int8_t)0;
}

/**
 *****************************************************************************
 * \fn          int8_t bsp_uart_settxcompletecallback(uart_id_e id, uart_isrcallback_pfun callbackfun)
 * \brief        设置发送完中断回调函数.
 * \note        .
 * \param[in]   id \ref uart_id_e 指定的串口id.
 * \param[in]   callbackfun \ref  uart_isrcallback_pfun 中断回调函数.
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
int8_t bsp_uart_settxcompletecallback(uart_id_e id, uart_isrcallback_pfun callbackfun)
{
    if(id >= sizeof(s_callbacklists)/sizeof(uart_isrcallback_pfun_t))
    {
        return (int8_t)-1;
    }
    s_callbacklists[id].txcmpfun = callbackfun;
    return (int8_t)0;
}

/**
 *****************************************************************************
 * \fn          int8_t bsp_uart_setrxcallback(uart_id_e id, uart_isrcallback_pfun callbackfun)
 * \brief       设置接收中断回调函数.
 * \note        .
 * \param[in]   id \ref uart_id_e 指定的串口id.
 * \param[in]   callbackfun \ref  uart_isrcallback_pfun 中断回调函数.
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
int8_t bsp_uart_setrxcallback(uart_id_e id, uart_isrcallback_pfun callbackfun)
{
    if(id >= sizeof(s_callbacklists)/sizeof(uart_isrcallback_pfun_t))
    {
        return (int8_t)-1;
    }
    s_callbacklists[id].rxfun = callbackfun;
    return (int8_t)0;
}

/**
 *****************************************************************************
 * \fn          int8_t bsp_uart_setrxerrcallback(uart_id_e id, uart_isrcallback_pfun callbackfun)
 * \brief       设置接收错误中断回调函数.
 * \note        .
 * \param[in]   id \ref uart_id_e 指定的串口id.
 * \param[in]   callbackfun \ref  uart_isrcallback_pfun 中断回调函数.
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
int8_t bsp_uart_setrxerrcallback(uart_id_e id, uart_isrcallback_pfun callbackfun)
{
    if(id >= sizeof(s_callbacklists)/sizeof(uart_isrcallback_pfun_t))
    {
        return (int8_t)-1;
    }
    s_callbacklists[id].rxerrfun = callbackfun;
    return (int8_t)0;
}

/**
 *****************************************************************************
 * \fn          int8_t bsp_uart_enabletx(uart_id_e id, uart_type_e type, uart_enable_e enable)
 * \brief       配置使能、禁能串口发送.
 * \note        .
 * \param[in]   id \ref uart_id_e 指定的串口id.
 * \param[in]   type \ref uart_type_e 模式.
 * \param[in]   enable \ref  uart_enable_e 使能、禁能.
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
int8_t bsp_uart_enabletx(uart_id_e id, uart_type_e type, uart_enable_e enable)
{
    int8_t res = (int8_t)0;
    switch(id)
    {
        case UART_ID_6:
        if(UART_ENABLE == enable)
        {
            switch(type)
            {
            case UART_INTERRUPT:
                R_SCI6->SCR_b.TEIE = 1;
            break;
            case UART_CHECK:
                R_SCI6->SCR_b.TEIE = 0;
            break;
            default:
                res = (int8_t)-1;
            break;
            }
        }
        else
        {
            R_SCI6->SCR_b.TE = 0;
            R_SCI6->SCR_b.TIE = 0;
            R_SCI6->SCR_b.TEIE = 0;
        }
        break;
        default:
            res = (int8_t)-1;
        break;
    }
    return res;
}

/**
 *****************************************************************************
 * \fn          int8_t bsp_uart_enabletxirq(uart_id_e id, uart_type_e type, uart_enable_e enable)
 * \brief       配置使能、禁能串口发送中断.
 * \note        .
 * \param[in]   id \ref uart_id_e 指定的串口id.
 * \param[in]   type \ref uart_type_e 模式.
 * \param[in]   enable \ref  uart_enable_e 使能、禁能.
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
int8_t bsp_uart_enabletxirq(uart_id_e id, uart_type_e type, uart_enable_e enable)
{
    int8_t res = (int8_t)0;
    switch(id)
    {
        case UART_ID_6:
        if(UART_ENABLE == enable)
        {
            R_SCI6->SCR_b.TE = 1;
        }
        else
        {
            R_SCI6->SCR_b.TE = 0;
        }
        break;
        default:
            res = (int8_t)-1;
        break;
    }
    return res;
}

/**
 *****************************************************************************
 * \fn          int8_t bsp_uart_enablerx(uart_id_e id, uart_type_e type, uart_enable_e enable)
 * \brief       配置使能、禁能串口接收.
 * \note        .
 * \param[in]   id \ref uart_id_e 指定的串口id.
 * \param[in]   type \ref uart_type_e 模式.
 * \param[in]   enable \ref  uart_enable_e 使能、禁能.
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
int8_t bsp_uart_enablerx(uart_id_e id, uart_type_e type, uart_enable_e enable)
{
    int8_t res = (int8_t)0;
    switch(id)
    {
        case UART_ID_6:
        if(UART_ENABLE == enable)
        {
            switch(type)
            {
            case UART_INTERRUPT:
                R_SCI6->SCR_b.RE = 1;
                R_SCI6->SCR_b.RIE = 1;
            break;
            case UART_CHECK:
                R_SCI6->SCR_b.RE = 1;
                R_SCI6->SCR_b.RIE = 0;
            break;
            default:
                res = (int8_t)-1;
            break;
            }
        }
        else
        {
            R_SCI6->SCR_b.RE = 0;
            R_SCI6->SCR_b.RIE = 0;
        }
        break;
        default:
            res = (int8_t)-1;
        break;
    }
    return res;
}


/**
 *****************************************************************************
 * \fn          int8_t bsp_uart_enablerxerr(uart_id_e id, uart_type_e type, uart_enable_e enable)
 * \brief       配置使能、禁能串口接收错误.
 * \note        .
 * \param[in]   id \ref uart_id_e 指定的串口id.
 * \param[in]   type \ref uart_type_e 模式.
 * \param[in]   enable \ref  uart_enable_e 使能、禁能.
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
int8_t bsp_uart_enablerxerr(uart_id_e id, uart_type_e type, uart_enable_e enable)
{
    int8_t res = (int8_t)0;
    switch(id)
    {
        case UART_ID_6:
        if(UART_ENABLE == enable)
        {
            //R_SCI6->SCR_b.RIE = 1;
        }
        else
        {
            //R_SCI6->SCR_b.RIE = 0;
        }
        break;
        default:
            res = (int8_t)-1;
        break;
    }
    return res;
}


/**
 *****************************************************************************
 * \fn          int8_t bsp_uart_deinit(uart_id_e id)
 * \brief       解除初始化串口.
 * \note        .
 * \param[in]   id \ref uart_id_e 指定的串口id.
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
int8_t bsp_uart_deinit(uart_id_e id)
{
    int8_t res = (int8_t)0;
    bsp_uart_enabletx(id, UART_CHECK, UART_DISABLE);
    bsp_uart_enablerx(id, UART_CHECK, UART_DISABLE);
    switch(id)
    {
        case UART_ID_6:

        break;
        default:
            res = (int8_t)-1;
        break;
    }
    return res;
}
