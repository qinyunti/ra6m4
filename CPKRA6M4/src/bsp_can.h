/**
 *****************************************************************************
 * \brief       BSP��(BSP)CANģ��(CAN)������ݽṹ�ͽӿ�����.
 * \details     Copyright (c) 2022,qinyunti@hotmail.com.
 *              All rights reserved.
 * \file        bsp_can.h
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

#ifndef BSP_CAN_H
#define BSP_CAN_H
#ifdef __cplusplus
 extern "C" {
#endif


/** \addtogroup BSP BSP��(BSP)
 *  \{
 */

/** \addtogroup BSP_UART BSP��(BSP)CANģ��(CAN)
 *  \{
 */

/*****************************************************************************
 *                                                                           *
 *                             ���ݽṹ����                                  *
 *                                                                           *
 ****************************************************************************/

/** \defgroup BSP_CAN_DATA BSP��(BSP)CANģ��(CAN)���ݽṹ
  * \{
  */

 typedef enum  can_id can_id_e; /**< CAN IDö�ٶ���  */

 /**
  * \enum can_id
  * CAN IDö�ٶ���.
  * \note id��0��ʼ��������,Ԥ���㹻���id��\n
  *       ʵ���õ���id����BSPӲ����Դʵ�־���.
  */
 enum can_id
 {
     CAN_ID_0=0,           /**< CAN ID0 */
     CAN_ID_1=1,           /**< CAN ID1 */
     CAN_ID_MAX=8,         /**< CAN ���� */
     CAN_ID_INV=0xFF,      /**< ��ЧID */
 };

 typedef struct can_msg can_msg_st;  /**< CAN��Ϣ���ṹ��  */

 /**
  * \struct can_msg
  * CAN��Ϣ.
  */
 struct can_msg
 {
     uint8_t dlc;
     uint8_t data[8];
     uint32_t id;
     uint32_t ts;
 };


typedef int (*can_rxcallback)(can_id_e,can_msg_st*);

/**
  * \}
  */

/*****************************************************************************
 *                                                                           *
 *                             �ӿں�������                                  *
 *                                                                           *
 ****************************************************************************/

/** \defgroup BSP_CAN_DATA BSP��(BSP)CANģ��(CAN)�ӿ�
  * \{
  */

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
 int8_t bsp_can_setbit(can_id_e id,uint16_t brp,uint8_t ts1, uint8_t ts2, uint8_t sjw);

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
 int8_t bsp_can_setfilter(can_id_e id,uint8_t enable,uint8_t index,uint32_t mask);


 /**
  *****************************************************************************
  * \fn          int8_t bsp_can_settest(can_id_e id,uint8_t enable,uint8_t mode)
  * \brief       ���ò���ģʽ.
  * \note        .
  * \param[in]   id \ref can_id_e ָ����CAN id.
  * \param[in]   enable 0���� ����ֵʹ��.
  * \param[in]   mode 0�޲���ģʽ 1����ģʽ 2�ڲ��ػ� 3�ⲿ�ػ�.
  * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
  *****************************************************************************
  */
 int8_t bsp_can_settest(can_id_e id,uint8_t enable,uint8_t mode);

 /**
  *****************************************************************************
  * \fn          int8_t bsp_can_init(can_id_e id)
  * \brief       ��ʼ������ʱ��,����,�ж�.
  * \note        .
  * \param[in]   id \ref can_id_e ָ����CAN id.
  * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
  *****************************************************************************
  */
 int8_t bsp_can_init(can_id_e id);


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
 int8_t bsp_can_send(can_id_e id,can_msg_st msg);

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
 int8_t bsp_can_setrxcallback(can_id_e id,can_rxcallback callback);

/**
  * \}
  */

/**
  * \}
  */

/**
  * \}
  */

#ifdef __cplusplus
}
#endif
#endif
