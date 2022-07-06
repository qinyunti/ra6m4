/**
 *****************************************************************************
 * \brief       ������(DRIVER)����ģ��(UART)������ݽṹ�ͽӿ�����.
 * \details     Copyright (c) 2022,qinyunti@hotmail.com.
 *              All rights reserved.
 * \file        driver_uart.h
 * \author      qinyunti@hotmail.com
 * \version     1.0
 * \date        2022-07-03
 * \note        ʹ��ǰ�ο�ע��.\n
 *              ����bsp_uart.\n
 * \since       qinyunti@hotmail.com 2022-07-03 1.0  �½�
 * \par �޶���¼
 * - 2022-07-03 ��ʼ�汾
 * \par ��Դ˵��
 * - RAM:
 * - ROM:
 *****************************************************************************
 */

#ifndef DRIVER_UART_H
#define DRIVER_UART_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "bsp_uart.h"
/** \addtogroup DRIVER ������(DRIVER)
 *  \{
 */

/** \addtogroup DRIVER_UART ������(DRIVER)����ģ��(UART)
 *  \{
 */
/*****************************************************************************
 *                                                                           *
 *                             ���ݽṹ����                                  *
 *                                                                           *
 ****************************************************************************/
/** \defgroup DRIVER_UART_DATA ������(DRIVER)����ģ��(UART)���ݽṹ
  * \{
  */

#define DRIVER_UART0_ENABLE          1  /**< ͨ��0 ʹ�� */
#define DRIVER_UART1_ENABLE          1  /**< ͨ��1 ʹ�� */
#define DRIVER_UART2_ENABLE          1  /**< ͨ��2 ʹ�� */
#define DRIVER_UART3_ENABLE          1  /**< ͨ��3 ʹ�� */
#define DRIVER_UART4_ENABLE          1  /**< ͨ��4 ʹ�� */
#define DRIVER_UART5_ENABLE          1  /**< ͨ��5 ʹ�� */
#define DRIVER_UART6_ENABLE          1  /**< ͨ��6 ʹ�� */
#define DRIVER_UART0_TXBUF_SIZE 1      /**< UART0���ͻ�������С  */
#define DRIVER_UART0_RXBUF_SIZE 128     /**< UART0���ջ�������С  */
#define DRIVER_UART1_TXBUF_SIZE 256      /**< UART1���ͻ�������С  */
#define DRIVER_UART1_RXBUF_SIZE 256     /**< UART1���ջ�������С  */
#define DRIVER_UART2_TXBUF_SIZE 64      /**< UART2���ͻ�������С  */
#define DRIVER_UART2_RXBUF_SIZE 128     /**< UART2���ջ�������С  */
#define DRIVER_UART3_TXBUF_SIZE 384      /**< UART3���ͻ�������С  */
#define DRIVER_UART3_RXBUF_SIZE 128     /**< UART3���ջ�������С  */
#define DRIVER_UART4_TXBUF_SIZE 64      /**< UART4���ͻ�������С  */
#define DRIVER_UART4_RXBUF_SIZE 128     /**< UART4���ջ�������С  */
#define DRIVER_UART5_TXBUF_SIZE 64      /**< UART5���ͻ�������С  */
#define DRIVER_UART5_RXBUF_SIZE 128     /**< UART5���ջ�������С  */
#define DRIVER_UART6_TXBUF_SIZE 64      /**< UART5���ͻ�������С  */
#define DRIVER_UART6_RXBUF_SIZE 128     /**< UART5���ջ�������С  */
#define DRIVER_UART_DELAY_FIFO    (20)    /**< ������������ʱʱ��       */
#define DRIVER_UART_DELAY_CHECK   (1)     /**< ��������ʱ��ѯʱ����   */
#define DRIVER_UART_DELAY_FRAME   (20)    /**< �ֽڼ����ڸ�ֵ��Ϊ֡����   */

typedef void (*driver_uart_hook)(void);   /**< ����/���ջص�����   */

/**
 * \struct driver_uart_t
 * UART�������ýṹ��.
 */
typedef struct
{
    uart_id_e id;              /**< UART id   */
    uint16_t rx_in;            /**< ���ջ�����д��ָ��        */
    uint16_t rx_out;           /**< ���ջ���������ָ��        */
    uint16_t rx_len;           /**< ���ջ�������Ч���ݴ�С    */
    uint16_t tx_in;            /**< ���ͻ�����д��ָ��        */
    uint16_t tx_out;           /**< ���ͻ���������ָ��        */
    uint16_t tx_len;           /**< ���ͻ�������Ч���ݴ�С    */
    uint16_t rxbuf_len;        /**< ���ջ�������С            */
    uint16_t txbuf_len;        /**< ���ͻ�������С            */
    driver_uart_hook tx_hook;  /**< ׼�����ͻص�����          */
    driver_uart_hook rx_hook;  /**< ������ת���ջص�����      */
    uint8_t *rx_buf;           /**< ���ջ�����                */
    uint8_t *tx_buf;           /**< ���ͻ�����                */
    uart_txstate_e state;      /**< ����״̬��־�Ƿ��ڷ���    */
    uint32_t err;              /**< �����������              */
}driver_uart_t;


/*****************************************************************************
 *                                                                           *
 *                             ģ�����ݶ���                                  *
 *                                                                           *
 ****************************************************************************/

#ifdef  DRIVER_UART_GLOBALS
/*�ص���������*/
void static uart6_txhook(void);
void static uart6_rxhook(void);
void static uart5_txhook(void);
void static uart5_rxhook(void);
void static uart4_txhook(void);
void static uart4_rxhook(void);
void static uart3_txhook(void);
void static uart3_rxhook(void);
void static uart2_txhook(void);
void static uart2_rxhook(void);
void static uart1_txhook(void);
void static uart1_rxhook(void);
void static uart0_txhook(void);
void static uart0_rxhook(void);

/*����������*/
static uint8_t driver_uart0_rxbuf[DRIVER_UART0_RXBUF_SIZE] ;
static uint8_t driver_uart0_txbuf[DRIVER_UART0_TXBUF_SIZE] ;
static uint8_t driver_uart1_rxbuf[DRIVER_UART1_RXBUF_SIZE] ;
static uint8_t driver_uart1_txbuf[DRIVER_UART1_TXBUF_SIZE] ;
static uint8_t driver_uart2_rxbuf[DRIVER_UART2_RXBUF_SIZE] ;
static uint8_t driver_uart2_txbuf[DRIVER_UART2_TXBUF_SIZE] ;
static uint8_t driver_uart3_rxbuf[DRIVER_UART3_RXBUF_SIZE] ;
static uint8_t driver_uart3_txbuf[DRIVER_UART3_TXBUF_SIZE] ;
static uint8_t driver_uart4_rxbuf[DRIVER_UART4_RXBUF_SIZE] ;
static uint8_t driver_uart4_txbuf[DRIVER_UART4_TXBUF_SIZE] ;
static uint8_t driver_uart5_rxbuf[DRIVER_UART5_RXBUF_SIZE] ;
static uint8_t driver_uart5_txbuf[DRIVER_UART5_TXBUF_SIZE] ;
static uint8_t driver_uart6_rxbuf[DRIVER_UART6_RXBUF_SIZE] ;
static uint8_t driver_uart6_txbuf[DRIVER_UART6_TXBUF_SIZE] ;

/*���ƽṹ�嶨��*/
static driver_uart_t driver_uart0  = {UART_ID_0, 0,0,0, 0,0,0, sizeof(driver_uart0_rxbuf), sizeof(driver_uart0_txbuf),uart0_txhook,uart0_rxhook,driver_uart0_rxbuf, driver_uart0_txbuf, UART_TXIDLE,0};
static driver_uart_t driver_uart1  = {UART_ID_1, 0,0,0, 0,0,0, sizeof(driver_uart1_rxbuf), sizeof(driver_uart1_txbuf),uart1_txhook,uart1_rxhook,driver_uart1_rxbuf, driver_uart1_txbuf, UART_TXIDLE,0};
static driver_uart_t driver_uart2  = {UART_ID_2, 0,0,0, 0,0,0, sizeof(driver_uart2_rxbuf), sizeof(driver_uart2_txbuf),uart2_txhook,uart2_rxhook,driver_uart2_rxbuf, driver_uart2_txbuf, UART_TXIDLE,0};
static driver_uart_t driver_uart3  = {UART_ID_3, 0,0,0, 0,0,0, sizeof(driver_uart3_rxbuf), sizeof(driver_uart3_txbuf),uart3_txhook,uart3_rxhook,driver_uart3_rxbuf, driver_uart3_txbuf, UART_TXIDLE,0};
static driver_uart_t driver_uart4  = {UART_ID_4, 0,0,0, 0,0,0, sizeof(driver_uart4_rxbuf), sizeof(driver_uart4_txbuf),uart4_txhook,uart4_rxhook,driver_uart4_rxbuf, driver_uart4_txbuf, UART_TXIDLE,0};
static driver_uart_t driver_uart5  = {UART_ID_5, 0,0,0, 0,0,0, sizeof(driver_uart5_rxbuf), sizeof(driver_uart5_txbuf),uart5_txhook,uart5_rxhook,driver_uart5_rxbuf, driver_uart5_txbuf, UART_TXIDLE,0};
static driver_uart_t driver_uart6  = {UART_ID_6, 0,0,0, 0,0,0, sizeof(driver_uart6_rxbuf), sizeof(driver_uart6_txbuf),uart6_txhook,uart6_rxhook,driver_uart6_rxbuf, driver_uart6_txbuf, UART_TXIDLE,0};

/*�ܵĿ��ƽṹ������*/
static driver_uart_t *driver_uart_tab[]  = {&driver_uart0, &driver_uart1, &driver_uart2,&driver_uart3, &driver_uart4, &driver_uart5, &driver_uart6};

#endif
/**
  * \}
  */
/*****************************************************************************
 *                                                                           *
 *                             �ӿں�������                                  *
 *                                                                           *
 ****************************************************************************/

/** \defgroup DRIVER_UART_IF ������(DRIVER)����ģ��(UART)�ӿ�
  * \{
  */

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
int8_t driver_uart_init(uart_id_e id);

/**
 *****************************************************************************
 * \fn          uint16_t driver_uart_getrxlen(uart_id_e id);
 * \brief       ��ȡ���ջ�������Ч���ݳ���.
 * \note        .
 * \param[in]   id ����id \ref uart_id_e
 * \return      uint16_t ���ջ���������
 *****************************************************************************
 */
uint16_t driver_uart_getrxlen(uart_id_e id);

/**
 *****************************************************************************
 * \fn          int8_t driver_uart_flush(uart_id_e id);
 * \brief       ���������.
 * \note        .
 * \param[in]   id ����id \ref uart_id_e
 * \return      int8_t ���Ƿ���0
 *****************************************************************************
 */
int8_t driver_uart_flush(uart_id_e id);

/**
 *****************************************************************************
 * \fn          int8_t driver_uart_set(uart_cfg_t* cfg);
 * \brief       ���ô��ڲ���.
 * \note        .
 * \param[in]   cfg ���ò���
 * \return      int8_t 0:�ɹ� ����ֵ:ʧ��
 *****************************************************************************
 */
int8_t driver_uart_set(uart_cfg_t* cfg);

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
uint16_t driver_uart_recv(uart_id_e id, uint8_t *pdata, uint16_t len, uint32_t timeout, int8_t *erro);

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
int16_t driver_uart_send(uart_id_e id, uint8_t *pdata, uint16_t len,uint32_t timeout,int8_t *erro);

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
 * \return      int8_t ���Ƿ���0
 *****************************************************************************
 */
int8_t driver_uart_geterr(uart_id_e id, uint32_t* err);

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
int8_t driver_uart_deinit(uart_id_e id);

/**
  * \}
  */

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

