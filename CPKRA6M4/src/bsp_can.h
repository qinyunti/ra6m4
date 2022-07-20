/**
 *****************************************************************************
 * \brief       BSP层(BSP)CAN模块(CAN)相关数据结构和接口描述.
 * \details     Copyright (c) 2022,qinyunti@hotmail.com.
 *              All rights reserved.
 * \file        bsp_can.h
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

#ifndef BSP_CAN_H
#define BSP_CAN_H
#ifdef __cplusplus
 extern "C" {
#endif


/** \addtogroup BSP BSP层(BSP)
 *  \{
 */

/** \addtogroup BSP_UART BSP层(BSP)CAN模块(CAN)
 *  \{
 */

/*****************************************************************************
 *                                                                           *
 *                             数据结构描述                                  *
 *                                                                           *
 ****************************************************************************/

/** \defgroup BSP_CAN_DATA BSP层(BSP)CAN模块(CAN)数据结构
  * \{
  */

 typedef enum  can_id can_id_e; /**< CAN ID枚举定义  */

 /**
  * \enum can_id
  * CAN ID枚举定义.
  * \note id从0开始连续递增,预留足够多的id号\n
  *       实际用到的id号由BSP硬件资源实现决定.
  */
 enum can_id
 {
     CAN_ID_0=0,           /**< CAN ID0 */
     CAN_ID_1=1,           /**< CAN ID1 */
     CAN_ID_MAX=8,         /**< CAN 个数 */
     CAN_ID_INV=0xFF,      /**< 无效ID */
 };

 typedef struct can_msg can_msg_st;  /**< CAN消息数结构体  */

 /**
  * \struct can_msg
  * CAN消息.
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
 *                             接口函数描述                                  *
 *                                                                           *
 ****************************************************************************/

/** \defgroup BSP_CAN_DATA BSP层(BSP)CAN模块(CAN)接口
  * \{
  */

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
 int8_t bsp_can_setbit(can_id_e id,uint16_t brp,uint8_t ts1, uint8_t ts2, uint8_t sjw);

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
 int8_t bsp_can_setfilter(can_id_e id,uint8_t enable,uint8_t index,uint32_t mask);


 /**
  *****************************************************************************
  * \fn          int8_t bsp_can_settest(can_id_e id,uint8_t enable,uint8_t mode)
  * \brief       设置测试模式.
  * \note        .
  * \param[in]   id \ref can_id_e 指定的CAN id.
  * \param[in]   enable 0禁用 其他值使能.
  * \param[in]   mode 0无测试模式 1监听模式 2内部回环 3外部回环.
  * \return      int8_t 0:成功 其他值:失败
  *****************************************************************************
  */
 int8_t bsp_can_settest(can_id_e id,uint8_t enable,uint8_t mode);

 /**
  *****************************************************************************
  * \fn          int8_t bsp_can_init(can_id_e id)
  * \brief       初始化配置时钟,引脚,中断.
  * \note        .
  * \param[in]   id \ref can_id_e 指定的CAN id.
  * \return      int8_t 0:成功 其他值:失败
  *****************************************************************************
  */
 int8_t bsp_can_init(can_id_e id);


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
 int8_t bsp_can_send(can_id_e id,can_msg_st msg);

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
