/**
 *****************************************************************************
 * \brief       BSP层(BSP)CAN模块(CAN)相关数据结构和接口实现.
 * \details     Copyright (c) 2022,qinyunti@hotmail.com.
 *              All rights reserved.
 * \file        bsp_can.c
 * \author      qinyunti@hotmail.com
 * \version     1.0
 * \date        2022-07-18
 * \note        使用前参考注释.\n
 *              .\n
 * \since       qinyunti@hotmail.com 2022-07-18 1.0  新建
 * \par 修订记录
 * - 2022-07-18 初始版本
 * \par 资源说明
 * - RAM:
 * - ROM:
 *****************************************************************************
 */

#include "bsp_api.h"
#include "bsp_can.h"
#include <stdint.h>



/*****************************************************************************
 *                                                                           *
 *                             内部数据                                                                             *
 *                                                                           *
 ****************************************************************************/

can_rxcallback s_rx_callback = NULL;

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

void can0_rxerr_isr(void)
{
    R_CAN0->ECSR &= 0x80;
    R_CAN0->EIFR = 0;
    R_ICU->IELSR_b[CAN0_RXERR_IRQn].IR = 0;
}
void can0_rxfifo_isr(void)
{
    R_ICU->IELSR_b[CAN0_RXFIFO_IRQn].IR = 0;
}
void can0_txfifo_isr(void)
{
    R_ICU->IELSR_b[CAN0_TXFIFO_IRQn].IR = 0;
}

void can0_rx_isr(void)
{
    can_msg_st msg;
    for(int i=0;i<16;i++)
    {
        if(R_CAN0->MCTL_RX_b[i].NEWDATA == 1)
        {
            R_CAN0->MCTL_RX_b[i].NEWDATA = 0;
            msg.dlc = R_CAN0->MB[i].DL;
            msg.id = R_CAN0->MB[i].ID;
            msg.ts = R_CAN0->MB[i].TS;
            for(int j=0;j<msg.dlc;j++)
            {
                msg.data[j]=R_CAN0->MB[i].D[j];
            }
            if(s_rx_callback != NULL)
            {
                s_rx_callback(CAN_ID_0,&msg);
            }
        }
    }
    R_ICU->IELSR_b[CAN0_RX_IRQn].IR = 0;
}

void can0_tx_isr(void)
{
    for(int i=16;i<32;i++)
    {
        /* SENTDATA
         * To set this flag to 0, first set the TRMREQ bit to 0. The SENTDATA flag and the TRMREQ bit
         * cannot be set to 0 simultaneously To transmit a new message from the associated mailbox, set the SENTDATA flag to 0.
         * */
        if(R_CAN0->MCTL_TX_b[i].SENTDATA == 1)
        {
            R_CAN0->MCTL_TX_b[i].TRMREQ = 0;
            R_CAN0->MCTL_TX_b[i].SENTDATA = 0;
        }
    }
    R_ICU->IELSR_b[CAN0_TX_IRQn].IR = 0;
}

/*****************************************************************************
 *                                                                           *
 *                             对外接口函数实现                                                              *
 *                                                                           *
 ****************************************************************************/

/**
 *****************************************************************************
 * \fn          int8_t bsp_can_setrxcallback(can_id_e id,can_rxcallback callback)
 * \brief       设置接收回调函数.
 * \note        .
 * \param[in]   id \ref can_id_e 指定的CAN id.
 * \param[in]   callback \ref callback.
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
int8_t bsp_can_setrxcallback(can_id_e id,can_rxcallback callback)
{
    s_rx_callback = callback;
    return 0;
}

/**
 *****************************************************************************
 * \fn          int8_t bsp_can_setbit(can_id_e id,uint16_t brp,uint8_t ts1, uint8_t ts2, uint8_t sjw)
 * \brief       设置bit时序.
 * \note        .
 * \param[in]   id \ref can_id_e 指定的CAN id.
 * \param[in]   brp 分频值.
 * \param[in]   ts1 TSEG1值.
 * \param[in]   ts2 TSEG2值.
 * \param[in]   sjw SJW值.
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
int8_t bsp_can_setbit(can_id_e id,uint16_t brp,uint8_t ts1, uint8_t ts2, uint8_t sjw)
{
    uint8_t mode;
    switch(id)
    {
    case CAN_ID_0:  /*CAN0*/
        mode = R_CAN0->CTLR_b.CANM;
        /*设置为halt模式再进行配置 */
        R_CAN0->CTLR_b.CANM = 2;  /* halt 模式 */
        while(R_CAN0->STR_b.HLTST != 1); /* 必须等切换完成 */
        R_CAN0->BCR_b.BRP = brp;
        R_CAN0->BCR_b.SJW = sjw;
        R_CAN0->BCR_b.TSEG1 = ts1;
        R_CAN0->BCR_b.TSEG2 = ts2;
        R_CAN0->CTLR_b.CANM = mode;  /* 恢复 模式 */
        while(R_CAN0->CTLR_b.CANM != mode);
    break;
    default:
    break;
    }
    return 0;
}

/**
 *****************************************************************************
 * \fn          int8_t bsp_can_setfilter(can_id_e id,uint8_t enable,uint32_t mask)
 * \brief       设计接收过滤.
 * \note        .
 * \param[in]   id \ref can_id_e 指定的CAN id.
 * \param[in]   enable 0禁用 其他值使能.
 * \param[in]   index 过滤器索引 0~7.
 * \param[in]   mask 过滤ID.
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
int8_t bsp_can_setfilter(can_id_e id,uint8_t enable,uint8_t index,uint32_t mask)
{
    uint8_t mode;
    if(index > 7)
    {
        return -1;
    }
    switch(id)
    {
    case CAN_ID_0:  /*CAN0*/
        mode = R_CAN0->CTLR_b.CANM;
        /*设置为halt模式再进行配置 */
        R_CAN0->CTLR_b.CANM = 2;  /* halt 模式 */
        while(R_CAN0->STR_b.HLTST != 1); /* 必须等切换完成 */

        if(enable)
        {
            R_CAN0->MKIVLR = R_CAN0->MKIVLR & (~(0xFF << index*4));
            R_CAN0->MKR[index] = mask;
        }
        else
        {
            R_CAN0->MKIVLR = R_CAN0->MKIVLR | (0xFF << index*4);
            R_CAN0->MKR[index] = 0;
        }
        R_CAN0->CTLR_b.CANM = mode;  /* 恢复 模式 */
        while(R_CAN0->CTLR_b.CANM != mode);
    break;
    default:
    break;
    }
    return 0;
}


/**
 *****************************************************************************
 * \fn          int8_t bsp_can_settest(can_id_e id,uint8_t enable,uint8_t mode)
 * \brief       设置测试模式.
 * \note        .
 * \param[in]   id \ref can_id_e 指定的CAN id.
 * \param[in]   enable 0禁用 其他值使能.
 * \param[in]   mode 0无测试模式 1监听模式 2外部回环 3内部回环.
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
int8_t bsp_can_settest(can_id_e id,uint8_t enable,uint8_t mode)
{
    uint8_t workmode;
    switch(id)
    {
    case CAN_ID_0:  /*CAN0*/
        workmode = R_CAN0->CTLR_b.CANM;
        /*设置为halt模式再进行配置 */
        R_CAN0->CTLR_b.CANM = 2;  /* halt 模式 */
        while(R_CAN0->STR_b.HLTST != 1); /* 必须等切换完成 */

        if(enable)
        {
            R_CAN0->TCR_b.TSTE = 1;
            R_CAN0->TCR_b.TSTM = mode;
        }
        else
        {
            R_CAN0->TCR_b.TSTE = 0;
            R_CAN0->TCR_b.TSTM = 0;
        }

        R_CAN0->CTLR_b.CANM = workmode;  /* 恢复 模式 */
        while(R_CAN0->CTLR_b.CANM != workmode);
    break;
    default:
    break;
    }
    return 0;
}


/**
 *****************************************************************************
 * \fn          int8_t bsp_can_send(can_id_e id,can_msg_st msg)
 * \brief       发送.
 * \note        .
 * \param[in]   id \ref can_id_e 指定的CAN id.
 * \param[in]   msg \ref can_msg_st.
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
int8_t bsp_can_send(can_id_e id,can_msg_st msg)
{
    uint8_t index = 0;
    /* 使用了邮箱ID优先级
     * 所以后来的数据添加在后面的邮箱
     * 比如邮箱17满 其他的空 则新的数据写入18而不是16
     */
    for(int i=31;i>15;i--)
    {
        if(R_CAN0->MCTL_TX_b[i].TRMACTIVE == 1)
        {
            index = i;
            break;
        }
    }
    if(index == 31)
    {
        /* 最后一个邮箱未发出去不能写入 否则写入前面的邮箱会乱序 */
        return -1;
    }
    else if(index == 0)
    {
        /* 所有的邮箱都未在用  使用第16个*/
        index = 16;
    }
    else
    {
        index++;  /* 使用占用的邮箱的后一个 */
    }

    R_CAN0->MB[index].DL = msg.dlc;
    R_CAN0->MB[index].ID = msg.id;
    for(int i=0;i<msg.dlc;i++)
    {
        R_CAN0->MB[index].D[i] = msg.data[i];
    }
    R_CAN0->MCTL_TX_b[index].TRMREQ = 1; /* 启动发送 */
    return 0;
}


/**
 *****************************************************************************
 * \fn          int8_t bsp_can_init(can_id_e id)
 * \brief       初始化配置时钟,引脚,中断.
 * \note        .
 * \param[in]   id \ref can_id_e 指定的CAN id.
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
int8_t bsp_can_init(can_id_e id)
{
    int8_t res = (int8_t)0;
    switch(id)
    {
    case CAN_ID_0:  /*CAN0*/
        /*使能时钟 Systeminit PCLKB已经设置分频值 */
        //R_MSTP->MSTPCRB_b.MSTPB2 = 0; 需要解锁
        R_BSP_MODULE_START(FSP_IP_CAN, 0);

        /*配置引脚*/

        /* P401作为CTX0,P402作为CRX0.*/
        R_PMISC->PWPR_b.B0WI = 0;   /* 解锁 PmnPFS */
        R_PMISC->PWPR_b.PFSWE = 1;

        /* P401外设输出 */
        R_PFS->PORT[4].PIN[1].PmnPFS_b.PMR = 1;   /* 外设 */
        R_PFS->PORT[4].PIN[1].PmnPFS_b.PDR = 1;   /* 输出 */
        R_PFS->PORT[4].PIN[1].PmnPFS_b.PCR = 1;   /* 使能上拉*/
        R_PFS->PORT[4].PIN[1].PmnPFS_b.NCODR = 0; /* CMOS模式 非开漏*/
        R_PFS->PORT[4].PIN[1].PmnPFS_b.DSCR = 3;  /* 高驱动能力 */
        R_PFS->PORT[4].PIN[1].PmnPFS_b.EOFR = 0;  /* 未使用事件模式所以写默认值 */
        R_PFS->PORT[4].PIN[1].PmnPFS_b.ISEL = 0;  /* 不使能外部中断 */
        R_PFS->PORT[4].PIN[1].PmnPFS_b.ASEL = 0;  /* 非AD */
        R_PFS->PORT[4].PIN[1].PmnPFS_b.PODR = 0;  /* 默认输出0  */
        R_PFS->PORT[4].PIN[1].PmnPFS_b.PSEL = 0x10;  /* CAN0 */

        /* P402外设输入 */
        R_PFS->PORT[4].PIN[2].PmnPFS_b.PMR = 1;   /* 外设 */
        R_PFS->PORT[4].PIN[2].PmnPFS_b.PDR = 0;   /* 输入 */
        R_PFS->PORT[4].PIN[2].PmnPFS_b.PCR = 1;   /* 使能上拉*/
        R_PFS->PORT[4].PIN[2].PmnPFS_b.NCODR = 0; /* CMOS模式 非开漏*/
        R_PFS->PORT[4].PIN[2].PmnPFS_b.DSCR = 3;  /* 高驱动能力 */
        R_PFS->PORT[4].PIN[2].PmnPFS_b.EOFR = 0;  /* 未使用事件模式所以写默认值 */
        R_PFS->PORT[4].PIN[2].PmnPFS_b.ISEL = 0;  /* 不使能外部中断 */
        R_PFS->PORT[4].PIN[2].PmnPFS_b.ASEL = 0;  /* 非AD */
        R_PFS->PORT[4].PIN[2].PmnPFS_b.PODR = 0;  /* 默认输出0  */
        R_PFS->PORT[4].PIN[2].PmnPFS_b.PSEL = 0x10;  /* RXD6 */

        R_PMISC->PWPR_b.PFSWE = 0;   /* 锁定 PmnPFS */
        R_PMISC->PWPR_b.B0WI = 1;

        /*配置中断*/
        NVIC_SetPriority(CAN0_RXERR_IRQn, 2);
        NVIC_EnableIRQ(CAN0_RXERR_IRQn);
        NVIC_SetPriority(CAN0_RXFIFO_IRQn, 2);
        NVIC_EnableIRQ(CAN0_RXFIFO_IRQn);
        NVIC_SetPriority(CAN0_TXFIFO_IRQn, 2);
        NVIC_EnableIRQ(CAN0_TXFIFO_IRQn);
        NVIC_SetPriority(CAN0_RX_IRQn, 2);
        NVIC_EnableIRQ(CAN0_RX_IRQn);
        NVIC_SetPriority(CAN0_TX_IRQn, 2);
        NVIC_EnableIRQ(CAN0_TX_IRQn);

        /*设置为reset模式再进行配置 */
        R_CAN0->CTLR_b.CANM = 1;  /* reset 模式 */
        while(R_CAN0->STR_b.RSTST != 1);  /* 必须等待切换完毕 */
        R_CAN0->CTLR_b.SLPM = 0;   /*不sleep  reset模式才能配置 */

        R_CAN0->CTLR_b.MBM  = 0;   /* normal */
        R_CAN0->CTLR_b.IDFM = 2;   /* Mixed ID mode */
        R_CAN0->CTLR_b.MLM = 1;    /* 来不及处理接收则溢出 */
        R_CAN0->CTLR_b.TPM = 0;     /* 优先级按邮箱序号      */
        //R_CAN0->CTLR_b.TSRC 必须操作模式才能配置
        R_CAN0->CTLR_b.TSPS = 3;    /* 时间戳精度是8bit */
        R_CAN0->CTLR_b.BOM = 0;     /* bus off自动恢复 */

        /* 时序配置
         * reset模式下配置  在进入halt和工作模式前一定要线配置一次有效的BCR 之后就可以在halt模式下配置了，而不需要再在reset模式下配置了
         */
        R_CAN0->BCR_b.CCLKS = 0;    /* 选择PCLK作为时钟源头 */
        R_CAN0->BCR_b.BRP = 4;
        R_CAN0->BCR_b.SJW = 3;
        R_CAN0->BCR_b.TSEG1 = 13;
        R_CAN0->BCR_b.TSEG2 = 4;

        /*
                  * 接收过滤设置
         */
        R_CAN0->MKIVLR = 0x00000000;
        for(int i=0;i<8;i++)
        {
            R_CAN0->MKR[i] = 0x00000000;
        }

        /* 中断配置 */
        R_CAN0->MIER = 0xFFFFFFFF;
        R_CAN0->EIER = 0xFF;

        R_CAN0->CTLR_b.CANM = 0;  /* 工作 模式 */
        while(R_CAN0->STR_b.RSTST != 0);

        /* 邮箱配置  前16个接收 后16个发送 必须工作模式或者halt模式配置
        MCTL_TX[j].TRMREQ   MCTL_TX[j].RECREQ   MCTL_TX[j].ONESHOT
        MCTL_RX[j].TRMREQ   MCTL_RX[j].RECREQ   MCTL_RX[j].ONESHOT
        0                   1                   0                  接收
        1                   0                   0                  发送
        */
        for(int i=0;i<16;i++)
        {
            R_CAN0->MCTL_RX_b[i].TRMREQ = 0;
            R_CAN0->MCTL_RX_b[i].RECREQ = 1;
            R_CAN0->MCTL_RX_b[i].ONESHOT = 0;
        }
        for(int i=16;i<32;i++)
        {
            //R_CAN0->MCTL_TX_b[i].TRMREQ = 1; 写完邮箱后 置1开始发送
            //R_CAN0->MCTL_RX_b[i].TRMREQ = 1;
            R_CAN0->MCTL_TX_b[i].RECREQ = 0;
            R_CAN0->MCTL_TX_b[i].ONESHOT = 0;
        }
    break;
    default:
        res = (int8_t)-1;
    break;
    }
    return res;
}
