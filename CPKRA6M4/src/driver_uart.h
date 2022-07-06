/**
 *****************************************************************************
 * \brief       驱动层(DRIVER)串口模块(UART)相关数据结构和接口描述.
 * \details     Copyright (c) 2022,qinyunti@hotmail.com.
 *              All rights reserved.
 * \file        driver_uart.h
 * \author      qinyunti@hotmail.com
 * \version     1.0
 * \date        2022-07-03
 * \note        使用前参考注释.\n
 *              依赖bsp_uart.\n
 * \since       qinyunti@hotmail.com 2022-07-03 1.0  新建
 * \par 修订记录
 * - 2022-07-03 初始版本
 * \par 资源说明
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
/** \addtogroup DRIVER 驱动层(DRIVER)
 *  \{
 */

/** \addtogroup DRIVER_UART 驱动层(DRIVER)串口模块(UART)
 *  \{
 */
/*****************************************************************************
 *                                                                           *
 *                             数据结构描述                                  *
 *                                                                           *
 ****************************************************************************/
/** \defgroup DRIVER_UART_DATA 驱动层(DRIVER)串口模块(UART)数据结构
  * \{
  */

#define DRIVER_UART0_ENABLE          1  /**< 通道0 使能 */
#define DRIVER_UART1_ENABLE          1  /**< 通道1 使能 */
#define DRIVER_UART2_ENABLE          1  /**< 通道2 使能 */
#define DRIVER_UART3_ENABLE          1  /**< 通道3 使能 */
#define DRIVER_UART4_ENABLE          1  /**< 通道4 使能 */
#define DRIVER_UART5_ENABLE          1  /**< 通道5 使能 */
#define DRIVER_UART6_ENABLE          1  /**< 通道6 使能 */
#define DRIVER_UART0_TXBUF_SIZE 1      /**< UART0发送缓冲区大小  */
#define DRIVER_UART0_RXBUF_SIZE 128     /**< UART0接收缓冲区大小  */
#define DRIVER_UART1_TXBUF_SIZE 256      /**< UART1发送缓冲区大小  */
#define DRIVER_UART1_RXBUF_SIZE 256     /**< UART1接收缓冲区大小  */
#define DRIVER_UART2_TXBUF_SIZE 64      /**< UART2发送缓冲区大小  */
#define DRIVER_UART2_RXBUF_SIZE 128     /**< UART2接收缓冲区大小  */
#define DRIVER_UART3_TXBUF_SIZE 384      /**< UART3发送缓冲区大小  */
#define DRIVER_UART3_RXBUF_SIZE 128     /**< UART3接收缓冲区大小  */
#define DRIVER_UART4_TXBUF_SIZE 64      /**< UART4发送缓冲区大小  */
#define DRIVER_UART4_RXBUF_SIZE 128     /**< UART4接收缓冲区大小  */
#define DRIVER_UART5_TXBUF_SIZE 64      /**< UART5发送缓冲区大小  */
#define DRIVER_UART5_RXBUF_SIZE 128     /**< UART5接收缓冲区大小  */
#define DRIVER_UART6_TXBUF_SIZE 64      /**< UART5发送缓冲区大小  */
#define DRIVER_UART6_RXBUF_SIZE 128     /**< UART5接收缓冲区大小  */
#define DRIVER_UART_DELAY_FIFO    (20)    /**< 缓冲区操作延时时间       */
#define DRIVER_UART_DELAY_CHECK   (1)     /**< 缓冲区延时查询时间间隔   */
#define DRIVER_UART_DELAY_FRAME   (20)    /**< 字节间距大于该值认为帧结束   */

typedef void (*driver_uart_hook)(void);   /**< 发送/接收回调函数   */

/**
 * \struct driver_uart_t
 * UART驱动配置结构体.
 */
typedef struct
{
    uart_id_e id;              /**< UART id   */
    uint16_t rx_in;            /**< 接收缓冲区写入指针        */
    uint16_t rx_out;           /**< 接收缓冲区读出指针        */
    uint16_t rx_len;           /**< 接收缓冲区有效数据大小    */
    uint16_t tx_in;            /**< 发送缓冲区写入指针        */
    uint16_t tx_out;           /**< 发送缓冲区读出指针        */
    uint16_t tx_len;           /**< 发送缓冲区有效数据大小    */
    uint16_t rxbuf_len;        /**< 接收缓冲区大小            */
    uint16_t txbuf_len;        /**< 发送缓冲区大小            */
    driver_uart_hook tx_hook;  /**< 准备发送回调函数          */
    driver_uart_hook rx_hook;  /**< 发送完转接收回调函数      */
    uint8_t *rx_buf;           /**< 接收缓冲区                */
    uint8_t *tx_buf;           /**< 发送缓冲区                */
    uart_txstate_e state;      /**< 发送状态标志是否在发送    */
    uint32_t err;              /**< 错误码计数器              */
}driver_uart_t;


/*****************************************************************************
 *                                                                           *
 *                             模块数据定义                                  *
 *                                                                           *
 ****************************************************************************/

#ifdef  DRIVER_UART_GLOBALS
/*回调函数申明*/
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

/*缓冲区定义*/
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

/*控制结构体定义*/
static driver_uart_t driver_uart0  = {UART_ID_0, 0,0,0, 0,0,0, sizeof(driver_uart0_rxbuf), sizeof(driver_uart0_txbuf),uart0_txhook,uart0_rxhook,driver_uart0_rxbuf, driver_uart0_txbuf, UART_TXIDLE,0};
static driver_uart_t driver_uart1  = {UART_ID_1, 0,0,0, 0,0,0, sizeof(driver_uart1_rxbuf), sizeof(driver_uart1_txbuf),uart1_txhook,uart1_rxhook,driver_uart1_rxbuf, driver_uart1_txbuf, UART_TXIDLE,0};
static driver_uart_t driver_uart2  = {UART_ID_2, 0,0,0, 0,0,0, sizeof(driver_uart2_rxbuf), sizeof(driver_uart2_txbuf),uart2_txhook,uart2_rxhook,driver_uart2_rxbuf, driver_uart2_txbuf, UART_TXIDLE,0};
static driver_uart_t driver_uart3  = {UART_ID_3, 0,0,0, 0,0,0, sizeof(driver_uart3_rxbuf), sizeof(driver_uart3_txbuf),uart3_txhook,uart3_rxhook,driver_uart3_rxbuf, driver_uart3_txbuf, UART_TXIDLE,0};
static driver_uart_t driver_uart4  = {UART_ID_4, 0,0,0, 0,0,0, sizeof(driver_uart4_rxbuf), sizeof(driver_uart4_txbuf),uart4_txhook,uart4_rxhook,driver_uart4_rxbuf, driver_uart4_txbuf, UART_TXIDLE,0};
static driver_uart_t driver_uart5  = {UART_ID_5, 0,0,0, 0,0,0, sizeof(driver_uart5_rxbuf), sizeof(driver_uart5_txbuf),uart5_txhook,uart5_rxhook,driver_uart5_rxbuf, driver_uart5_txbuf, UART_TXIDLE,0};
static driver_uart_t driver_uart6  = {UART_ID_6, 0,0,0, 0,0,0, sizeof(driver_uart6_rxbuf), sizeof(driver_uart6_txbuf),uart6_txhook,uart6_rxhook,driver_uart6_rxbuf, driver_uart6_txbuf, UART_TXIDLE,0};

/*总的控制结构体数组*/
static driver_uart_t *driver_uart_tab[]  = {&driver_uart0, &driver_uart1, &driver_uart2,&driver_uart3, &driver_uart4, &driver_uart5, &driver_uart6};

#endif
/**
  * \}
  */
/*****************************************************************************
 *                                                                           *
 *                             接口函数描述                                  *
 *                                                                           *
 ****************************************************************************/

/** \defgroup DRIVER_UART_IF 驱动层(DRIVER)串口模块(UART)接口
  * \{
  */

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
int8_t driver_uart_init(uart_id_e id);

/**
 *****************************************************************************
 * \fn          uint16_t driver_uart_getrxlen(uart_id_e id);
 * \brief       获取接收缓冲区有效数据长度.
 * \note        .
 * \param[in]   id 串口id \ref uart_id_e
 * \return      uint16_t 接收缓冲区长度
 *****************************************************************************
 */
uint16_t driver_uart_getrxlen(uart_id_e id);

/**
 *****************************************************************************
 * \fn          int8_t driver_uart_flush(uart_id_e id);
 * \brief       清除缓冲区.
 * \note        .
 * \param[in]   id 串口id \ref uart_id_e
 * \return      int8_t 总是返回0
 *****************************************************************************
 */
int8_t driver_uart_flush(uart_id_e id);

/**
 *****************************************************************************
 * \fn          int8_t driver_uart_set(uart_cfg_t* cfg);
 * \brief       配置串口参数.
 * \note        .
 * \param[in]   cfg 配置参数
 * \return      int8_t 0:成功 其他值:失败
 *****************************************************************************
 */
int8_t driver_uart_set(uart_cfg_t* cfg);

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
uint16_t driver_uart_recv(uart_id_e id, uint8_t *pdata, uint16_t len, uint32_t timeout, int8_t *erro);

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
int16_t driver_uart_send(uart_id_e id, uint8_t *pdata, uint16_t len,uint32_t timeout,int8_t *erro);

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
 * \return      int8_t 总是返回0
 *****************************************************************************
 */
int8_t driver_uart_geterr(uart_id_e id, uint32_t* err);

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

