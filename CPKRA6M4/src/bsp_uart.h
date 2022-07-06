/**
 *****************************************************************************
 * \brief       BSP层(BSP)串口模块(UART)相关数据结构和接口描述.
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

#ifndef BSP_UART_H
#define BSP_UART_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "bsp_typedef.h"
/** \addtogroup BSP BSP层(BSP)
 *  \{
 */

/** \addtogroup BSP_UART BSP层(BSP)串口模块(UART)
 *  \{
 */

/*****************************************************************************
 *                                                                           *
 *                             数据结构描述                                  *
 *                                                                           *
 ****************************************************************************/

/** \defgroup BSP_UART_DATA BSP层(BSP)串口模块(UART)数据结构
  * \{
  */

#define UART_TIMEOUT   (int32_t)10000         /**< 串口操作超时时间 单位uS*/
#define CR1_OVER8_Set  ((uint16_t)0x8000) /**< USART OVER8 mode Enable Mask */
#define UART1_PRI      14                  /**< UART1中断优先级 */
#define UART2_PRI      14                  /**< UART2中断优先级 */
#define UART3_PRI      14                  /**< UART3中断优先级 */
#define UART4_PRI      14                  /**< UART4中断优先级 */
#define UART5_PRI      14                  /**< UART5中断优先级 */
#define LPUART1_PRI    14                  /**< LPUART1中断优先级 */
/**
  * \}
  */

/*****************************************************************************
 *                                                                           *
 *                             接口函数描述                                  *
 *                                                                           *
 ****************************************************************************/

/** \defgroup DRIVER_UART_if 驱动层(DRIVER)串口模块(UART)接口
  * \{
  */

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
int8_t bsp_uart_setcfg(uint8_t id, uart_cfg_t* cfg);

/**
 *****************************************************************************
 * \fn          int8_t bsp_uart_init(uart_id_e id)
 * \brief       初始化配置始终,引脚,中断.
 * \note        .
 * \param[in]   id \ref uart_id_e 指定的串口id.
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
int8_t bsp_uart_init(uart_id_e id);

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
int8_t bsp_uart_send(uart_id_e id, uint8_t val);

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
int8_t bsp_uart_settxcallback(uart_id_e id, uart_isrcallback_pfun callbackfun);

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
int8_t bsp_uart_settxcompletecallback(uart_id_e id, uart_isrcallback_pfun callbackfun);

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
int8_t bsp_uart_setrxcallback(uart_id_e id, uart_isrcallback_pfun callbackfun);

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
int8_t bsp_uart_setrxerrcallback(uart_id_e id, uart_isrcallback_pfun callbackfun);

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
int8_t bsp_uart_enabletx(uart_id_e id, uart_type_e type, uart_enable_e enable);

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
int8_t bsp_uart_enabletxirq(uart_id_e id, uart_type_e type, uart_enable_e enable);

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
int8_t bsp_uart_enablerx(uart_id_e id, uart_type_e type, uart_enable_e enable);

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
int8_t bsp_uart_enablerxerr(uart_id_e id, uart_type_e type, uart_enable_e enable);

/**
 *****************************************************************************
 * \fn          int8_t bsp_uart_deinit(uart_id_e id)
 * \brief       解除初始化串口.
 * \note        .
 * \param[in]   id \ref uart_id_e 指定的串口id.
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
int8_t bsp_uart_deinit(uart_id_e id);


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
