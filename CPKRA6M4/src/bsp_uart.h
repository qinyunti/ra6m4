/**
 *****************************************************************************
 * \brief       BSP��(BSP)����ģ��(UART)������ݽṹ�ͽӿ�����.
 * \details     Copyright (c) 2022,qinyunti@hotmail.com.
 *              All rights reserved.
 * \file        bsp_uart.h
 * \author      qinyunti@hotmail.com
 * \version     1.0
 * \date        2022-07-03
 * \note        ʹ��ǰ�ο�ע��.\n
 *              .\n
 * \since       qinyunti@hotmail.com 2022-07-03 1.0  �½�
 * \par �޶���¼
 * - 2022-07-03 ��ʼ�汾
 * \par ��Դ˵��
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
/** \addtogroup BSP BSP��(BSP)
 *  \{
 */

/** \addtogroup BSP_UART BSP��(BSP)����ģ��(UART)
 *  \{
 */

/*****************************************************************************
 *                                                                           *
 *                             ���ݽṹ����                                  *
 *                                                                           *
 ****************************************************************************/

/** \defgroup BSP_UART_DATA BSP��(BSP)����ģ��(UART)���ݽṹ
  * \{
  */

#define UART_TIMEOUT   (int32_t)10000         /**< ���ڲ�����ʱʱ�� ��λuS*/
#define CR1_OVER8_Set  ((uint16_t)0x8000) /**< USART OVER8 mode Enable Mask */
#define UART1_PRI      14                  /**< UART1�ж����ȼ� */
#define UART2_PRI      14                  /**< UART2�ж����ȼ� */
#define UART3_PRI      14                  /**< UART3�ж����ȼ� */
#define UART4_PRI      14                  /**< UART4�ж����ȼ� */
#define UART5_PRI      14                  /**< UART5�ж����ȼ� */
#define LPUART1_PRI    14                  /**< LPUART1�ж����ȼ� */
/**
  * \}
  */

/*****************************************************************************
 *                                                                           *
 *                             �ӿں�������                                  *
 *                                                                           *
 ****************************************************************************/

/** \defgroup DRIVER_UART_if ������(DRIVER)����ģ��(UART)�ӿ�
  * \{
  */

/**
 *****************************************************************************
 * \fn          int8_t bsp_uart_setcfg(uint8_t id, uart_cfg_t* cfg)
 * \brief       ���ô��ڲ���,����ǰ��Ҫ����bsp_uart_init��ʼ��.
 * \note        .
 * \param[in]   id \ref uart_id_e ָ���Ĵ���id.
 * \param[in]   cfg \ref uart_cfg_t ָ�����ýṹ���ָ��.
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
 *****************************************************************************
 */
int8_t bsp_uart_setcfg(uint8_t id, uart_cfg_t* cfg);

/**
 *****************************************************************************
 * \fn          int8_t bsp_uart_init(uart_id_e id)
 * \brief       ��ʼ������ʼ��,����,�ж�.
 * \note        .
 * \param[in]   id \ref uart_id_e ָ���Ĵ���id.
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
 *****************************************************************************
 */
int8_t bsp_uart_init(uart_id_e id);

/**
 *****************************************************************************
 * \fn          int8_t bsp_uart_send(uart_id_e id, uint8_t val)
 * \brief       �����ֽ�.
 * \note        .
 * \param[in]   id \ref uart_id_e ָ���Ĵ���id.
 * \param[in]   val ���͵��ֽ�����.
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
 *****************************************************************************
 */
int8_t bsp_uart_send(uart_id_e id, uint8_t val);

/**
 *****************************************************************************
 * \fn          int8_t bsp_uart_settxcallback(uart_id_e id, uart_isrcallback_pfun callbackfun)
 * \brief        ���÷��Ϳ��жϻص�����.
 * \note        .
 * \param[in]   id \ref uart_id_e ָ���Ĵ���id.
 * \param[in]   callbackfun \ref  uart_isrcallback_pfun �жϻص�����.
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
 *****************************************************************************
 */
int8_t bsp_uart_settxcallback(uart_id_e id, uart_isrcallback_pfun callbackfun);

/**
 *****************************************************************************
 * \fn          int8_t bsp_uart_settxcompletecallback(uart_id_e id, uart_isrcallback_pfun callbackfun)
 * \brief        ���÷������жϻص�����.
 * \note        .
 * \param[in]   id \ref uart_id_e ָ���Ĵ���id.
 * \param[in]   callbackfun \ref  uart_isrcallback_pfun �жϻص�����.
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
 *****************************************************************************
 */
int8_t bsp_uart_settxcompletecallback(uart_id_e id, uart_isrcallback_pfun callbackfun);

/**
 *****************************************************************************
 * \fn          int8_t bsp_uart_setrxcallback(uart_id_e id, uart_isrcallback_pfun callbackfun)
 * \brief       ���ý����жϻص�����.
 * \note        .
 * \param[in]   id \ref uart_id_e ָ���Ĵ���id.
 * \param[in]   callbackfun \ref  uart_isrcallback_pfun �жϻص�����.
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
 *****************************************************************************
 */
int8_t bsp_uart_setrxcallback(uart_id_e id, uart_isrcallback_pfun callbackfun);

/**
 *****************************************************************************
 * \fn          int8_t bsp_uart_setrxerrcallback(uart_id_e id, uart_isrcallback_pfun callbackfun)
 * \brief       ���ý��մ����жϻص�����.
 * \note        .
 * \param[in]   id \ref uart_id_e ָ���Ĵ���id.
 * \param[in]   callbackfun \ref  uart_isrcallback_pfun �жϻص�����.
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
 *****************************************************************************
 */
int8_t bsp_uart_setrxerrcallback(uart_id_e id, uart_isrcallback_pfun callbackfun);

/**
 *****************************************************************************
 * \fn          int8_t bsp_uart_enabletx(uart_id_e id, uart_type_e type, uart_enable_e enable)
 * \brief       ����ʹ�ܡ����ܴ��ڷ���.
 * \note        .
 * \param[in]   id \ref uart_id_e ָ���Ĵ���id.
 * \param[in]   type \ref uart_type_e ģʽ.
 * \param[in]   enable \ref  uart_enable_e ʹ�ܡ�����.
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
 *****************************************************************************
 */
int8_t bsp_uart_enabletx(uart_id_e id, uart_type_e type, uart_enable_e enable);

/**
 *****************************************************************************
 * \fn          int8_t bsp_uart_enabletxirq(uart_id_e id, uart_type_e type, uart_enable_e enable)
 * \brief       ����ʹ�ܡ����ܴ��ڷ����ж�.
 * \note        .
 * \param[in]   id \ref uart_id_e ָ���Ĵ���id.
 * \param[in]   type \ref uart_type_e ģʽ.
 * \param[in]   enable \ref  uart_enable_e ʹ�ܡ�����.
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
 *****************************************************************************
 */
int8_t bsp_uart_enabletxirq(uart_id_e id, uart_type_e type, uart_enable_e enable);

/**
 *****************************************************************************
 * \fn          int8_t bsp_uart_enablerx(uart_id_e id, uart_type_e type, uart_enable_e enable)
 * \brief       ����ʹ�ܡ����ܴ��ڽ���.
 * \note        .
 * \param[in]   id \ref uart_id_e ָ���Ĵ���id.
 * \param[in]   type \ref uart_type_e ģʽ.
 * \param[in]   enable \ref  uart_enable_e ʹ�ܡ�����.
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
 *****************************************************************************
 */
int8_t bsp_uart_enablerx(uart_id_e id, uart_type_e type, uart_enable_e enable);

/**
 *****************************************************************************
 * \fn          int8_t bsp_uart_enablerxerr(uart_id_e id, uart_type_e type, uart_enable_e enable)
 * \brief       ����ʹ�ܡ����ܴ��ڽ��մ���.
 * \note        .
 * \param[in]   id \ref uart_id_e ָ���Ĵ���id.
 * \param[in]   type \ref uart_type_e ģʽ.
 * \param[in]   enable \ref  uart_enable_e ʹ�ܡ�����.
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
 *****************************************************************************
 */
int8_t bsp_uart_enablerxerr(uart_id_e id, uart_type_e type, uart_enable_e enable);

/**
 *****************************************************************************
 * \fn          int8_t bsp_uart_deinit(uart_id_e id)
 * \brief       �����ʼ������.
 * \note        .
 * \param[in]   id \ref uart_id_e ָ���Ĵ���id.
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
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
