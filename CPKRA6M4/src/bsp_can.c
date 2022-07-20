/**
 *****************************************************************************
 * \brief       BSP��(BSP)CANģ��(CAN)������ݽṹ�ͽӿ�ʵ��.
 * \details     Copyright (c) 2022,qinyunti@hotmail.com.
 *              All rights reserved.
 * \file        bsp_can.c
 * \author      qinyunti@hotmail.com
 * \version     1.0
 * \date        2022-07-18
 * \note        ʹ��ǰ�ο�ע��.\n
 *              .\n
 * \since       qinyunti@hotmail.com 2022-07-18 1.0  �½�
 * \par �޶���¼
 * - 2022-07-18 ��ʼ�汾
 * \par ��Դ˵��
 * - RAM:
 * - ROM:
 *****************************************************************************
 */

#include "bsp_api.h"
#include "bsp_can.h"
#include <stdint.h>



/*****************************************************************************
 *                                                                           *
 *                             �ڲ�����                                                                             *
 *                                                                           *
 ****************************************************************************/

can_rxcallback s_rx_callback = NULL;

/*****************************************************************************
 *                                                                           *
 *                             �ڲ��ӿں���ʵ��                                                              *
 *                                                                           *
 ****************************************************************************/

/*****************************************************************************
 *                                                                           *
 *                             �жϴ�����                                                                     *
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
 *                             ����ӿں���ʵ��                                                              *
 *                                                                           *
 ****************************************************************************/

/**
 *****************************************************************************
 * \fn          int8_t bsp_can_setrxcallback(can_id_e id,can_rxcallback callback)
 * \brief       ���ý��ջص�����.
 * \note        .
 * \param[in]   id \ref can_id_e ָ����CAN id.
 * \param[in]   callback \ref callback.
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
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
 * \brief       ����bitʱ��.
 * \note        .
 * \param[in]   id \ref can_id_e ָ����CAN id.
 * \param[in]   brp ��Ƶֵ.
 * \param[in]   ts1 TSEG1ֵ.
 * \param[in]   ts2 TSEG2ֵ.
 * \param[in]   sjw SJWֵ.
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
 *****************************************************************************
 */
int8_t bsp_can_setbit(can_id_e id,uint16_t brp,uint8_t ts1, uint8_t ts2, uint8_t sjw)
{
    uint8_t mode;
    switch(id)
    {
    case CAN_ID_0:  /*CAN0*/
        mode = R_CAN0->CTLR_b.CANM;
        /*����Ϊhaltģʽ�ٽ������� */
        R_CAN0->CTLR_b.CANM = 2;  /* halt ģʽ */
        while(R_CAN0->STR_b.HLTST != 1); /* ������л���� */
        R_CAN0->BCR_b.BRP = brp;
        R_CAN0->BCR_b.SJW = sjw;
        R_CAN0->BCR_b.TSEG1 = ts1;
        R_CAN0->BCR_b.TSEG2 = ts2;
        R_CAN0->CTLR_b.CANM = mode;  /* �ָ� ģʽ */
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
 * \brief       ��ƽ��չ���.
 * \note        .
 * \param[in]   id \ref can_id_e ָ����CAN id.
 * \param[in]   enable 0���� ����ֵʹ��.
 * \param[in]   index ���������� 0~7.
 * \param[in]   mask ����ID.
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
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
        /*����Ϊhaltģʽ�ٽ������� */
        R_CAN0->CTLR_b.CANM = 2;  /* halt ģʽ */
        while(R_CAN0->STR_b.HLTST != 1); /* ������л���� */

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
        R_CAN0->CTLR_b.CANM = mode;  /* �ָ� ģʽ */
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
 * \brief       ���ò���ģʽ.
 * \note        .
 * \param[in]   id \ref can_id_e ָ����CAN id.
 * \param[in]   enable 0���� ����ֵʹ��.
 * \param[in]   mode 0�޲���ģʽ 1����ģʽ 2�ⲿ�ػ� 3�ڲ��ػ�.
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
 *****************************************************************************
 */
int8_t bsp_can_settest(can_id_e id,uint8_t enable,uint8_t mode)
{
    uint8_t workmode;
    switch(id)
    {
    case CAN_ID_0:  /*CAN0*/
        workmode = R_CAN0->CTLR_b.CANM;
        /*����Ϊhaltģʽ�ٽ������� */
        R_CAN0->CTLR_b.CANM = 2;  /* halt ģʽ */
        while(R_CAN0->STR_b.HLTST != 1); /* ������л���� */

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

        R_CAN0->CTLR_b.CANM = workmode;  /* �ָ� ģʽ */
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
 * \brief       ����.
 * \note        .
 * \param[in]   id \ref can_id_e ָ����CAN id.
 * \param[in]   msg \ref can_msg_st.
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
 *****************************************************************************
 */
int8_t bsp_can_send(can_id_e id,can_msg_st msg)
{
    uint8_t index = 0;
    /* ʹ��������ID���ȼ�
     * ���Ժ�������������ں��������
     * ��������17�� �����Ŀ� ���µ�����д��18������16
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
        /* ���һ������δ����ȥ����д�� ����д��ǰ������������ */
        return -1;
    }
    else if(index == 0)
    {
        /* ���е����䶼δ����  ʹ�õ�16��*/
        index = 16;
    }
    else
    {
        index++;  /* ʹ��ռ�õ�����ĺ�һ�� */
    }

    R_CAN0->MB[index].DL = msg.dlc;
    R_CAN0->MB[index].ID = msg.id;
    for(int i=0;i<msg.dlc;i++)
    {
        R_CAN0->MB[index].D[i] = msg.data[i];
    }
    R_CAN0->MCTL_TX_b[index].TRMREQ = 1; /* �������� */
    return 0;
}


/**
 *****************************************************************************
 * \fn          int8_t bsp_can_init(can_id_e id)
 * \brief       ��ʼ������ʱ��,����,�ж�.
 * \note        .
 * \param[in]   id \ref can_id_e ָ����CAN id.
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
 *****************************************************************************
 */
int8_t bsp_can_init(can_id_e id)
{
    int8_t res = (int8_t)0;
    switch(id)
    {
    case CAN_ID_0:  /*CAN0*/
        /*ʹ��ʱ�� Systeminit PCLKB�Ѿ����÷�Ƶֵ */
        //R_MSTP->MSTPCRB_b.MSTPB2 = 0; ��Ҫ����
        R_BSP_MODULE_START(FSP_IP_CAN, 0);

        /*��������*/

        /* P401��ΪCTX0,P402��ΪCRX0.*/
        R_PMISC->PWPR_b.B0WI = 0;   /* ���� PmnPFS */
        R_PMISC->PWPR_b.PFSWE = 1;

        /* P401������� */
        R_PFS->PORT[4].PIN[1].PmnPFS_b.PMR = 1;   /* ���� */
        R_PFS->PORT[4].PIN[1].PmnPFS_b.PDR = 1;   /* ��� */
        R_PFS->PORT[4].PIN[1].PmnPFS_b.PCR = 1;   /* ʹ������*/
        R_PFS->PORT[4].PIN[1].PmnPFS_b.NCODR = 0; /* CMOSģʽ �ǿ�©*/
        R_PFS->PORT[4].PIN[1].PmnPFS_b.DSCR = 3;  /* ���������� */
        R_PFS->PORT[4].PIN[1].PmnPFS_b.EOFR = 0;  /* δʹ���¼�ģʽ����дĬ��ֵ */
        R_PFS->PORT[4].PIN[1].PmnPFS_b.ISEL = 0;  /* ��ʹ���ⲿ�ж� */
        R_PFS->PORT[4].PIN[1].PmnPFS_b.ASEL = 0;  /* ��AD */
        R_PFS->PORT[4].PIN[1].PmnPFS_b.PODR = 0;  /* Ĭ�����0  */
        R_PFS->PORT[4].PIN[1].PmnPFS_b.PSEL = 0x10;  /* CAN0 */

        /* P402�������� */
        R_PFS->PORT[4].PIN[2].PmnPFS_b.PMR = 1;   /* ���� */
        R_PFS->PORT[4].PIN[2].PmnPFS_b.PDR = 0;   /* ���� */
        R_PFS->PORT[4].PIN[2].PmnPFS_b.PCR = 1;   /* ʹ������*/
        R_PFS->PORT[4].PIN[2].PmnPFS_b.NCODR = 0; /* CMOSģʽ �ǿ�©*/
        R_PFS->PORT[4].PIN[2].PmnPFS_b.DSCR = 3;  /* ���������� */
        R_PFS->PORT[4].PIN[2].PmnPFS_b.EOFR = 0;  /* δʹ���¼�ģʽ����дĬ��ֵ */
        R_PFS->PORT[4].PIN[2].PmnPFS_b.ISEL = 0;  /* ��ʹ���ⲿ�ж� */
        R_PFS->PORT[4].PIN[2].PmnPFS_b.ASEL = 0;  /* ��AD */
        R_PFS->PORT[4].PIN[2].PmnPFS_b.PODR = 0;  /* Ĭ�����0  */
        R_PFS->PORT[4].PIN[2].PmnPFS_b.PSEL = 0x10;  /* RXD6 */

        R_PMISC->PWPR_b.PFSWE = 0;   /* ���� PmnPFS */
        R_PMISC->PWPR_b.B0WI = 1;

        /*�����ж�*/
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

        /*����Ϊresetģʽ�ٽ������� */
        R_CAN0->CTLR_b.CANM = 1;  /* reset ģʽ */
        while(R_CAN0->STR_b.RSTST != 1);  /* ����ȴ��л���� */
        R_CAN0->CTLR_b.SLPM = 0;   /*��sleep  resetģʽ�������� */

        R_CAN0->CTLR_b.MBM  = 0;   /* normal */
        R_CAN0->CTLR_b.IDFM = 2;   /* Mixed ID mode */
        R_CAN0->CTLR_b.MLM = 1;    /* ������������������ */
        R_CAN0->CTLR_b.TPM = 0;     /* ���ȼ����������      */
        //R_CAN0->CTLR_b.TSRC �������ģʽ��������
        R_CAN0->CTLR_b.TSPS = 3;    /* ʱ���������8bit */
        R_CAN0->CTLR_b.BOM = 0;     /* bus off�Զ��ָ� */

        /* ʱ������
         * resetģʽ������  �ڽ���halt�͹���ģʽǰһ��Ҫ������һ����Ч��BCR ֮��Ϳ�����haltģʽ�������ˣ�������Ҫ����resetģʽ��������
         */
        R_CAN0->BCR_b.CCLKS = 0;    /* ѡ��PCLK��Ϊʱ��Դͷ */
        R_CAN0->BCR_b.BRP = 4;
        R_CAN0->BCR_b.SJW = 3;
        R_CAN0->BCR_b.TSEG1 = 13;
        R_CAN0->BCR_b.TSEG2 = 4;

        /*
                  * ���չ�������
         */
        R_CAN0->MKIVLR = 0x00000000;
        for(int i=0;i<8;i++)
        {
            R_CAN0->MKR[i] = 0x00000000;
        }

        /* �ж����� */
        R_CAN0->MIER = 0xFFFFFFFF;
        R_CAN0->EIER = 0xFF;

        R_CAN0->CTLR_b.CANM = 0;  /* ���� ģʽ */
        while(R_CAN0->STR_b.RSTST != 0);

        /* ��������  ǰ16������ ��16������ ���빤��ģʽ����haltģʽ����
        MCTL_TX[j].TRMREQ   MCTL_TX[j].RECREQ   MCTL_TX[j].ONESHOT
        MCTL_RX[j].TRMREQ   MCTL_RX[j].RECREQ   MCTL_RX[j].ONESHOT
        0                   1                   0                  ����
        1                   0                   0                  ����
        */
        for(int i=0;i<16;i++)
        {
            R_CAN0->MCTL_RX_b[i].TRMREQ = 0;
            R_CAN0->MCTL_RX_b[i].RECREQ = 1;
            R_CAN0->MCTL_RX_b[i].ONESHOT = 0;
        }
        for(int i=16;i<32;i++)
        {
            //R_CAN0->MCTL_TX_b[i].TRMREQ = 1; д������� ��1��ʼ����
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
