/**
 *****************************************************************************
 * \brief       BSP层(BSP)抽象数据定义模块(TYPEDEF)相关数据结构描述.
 * \details     Copyright (c) 2022,qinyunti@hotmail.com.
 *              All rights reserved.
 * \file        bsp_typedef.h
 * \author      qinyunti@hotmail.com
 * \version     1.0
 * \date        2022-07-03
 * \note        使用前参考注释.\n
 *              .\n
 * \since       qinyunti@hotmail.com 2022-07-03 1.0  新建
 * \par 修订记录
 * - 2017-07-24 初始版本
 * \par 资源说明
 * - RAM:
 * - ROM:
 *****************************************************************************
 */

#ifndef BSP_TYPEDEF_H
#define BSP_TYPEDEF_H
#ifdef __cplusplus
 extern "C" {
#endif

/** \addtogroup BSP BSP层(BSP)
 *  \{
 */

/** \addtogroup BSP_TYPEDEF BSP层(BSP)抽象数据定义模块(TYPEDEF)
 *  \{
 */

#include <stdint.h>
/*****************************************************************************
 *                                                                           *
 *                             数据结构描述                                                                     *
 *                                                                           *
 ****************************************************************************/

/** \defgroup BSP_TYPEDEF_DATA BSP层(BSP)抽象数据定义模块(TYPEDEF)数据结构
  * \{
  */

/*****************************************************************************
 *                                                                           *
 *                             UART数据结构描述                                                             *
 *                                                                           *
 ****************************************************************************/

typedef enum  uart_txstate uart_txstate_e; /**< UART发送状态枚举定义  */

/**
 * \enum uart_txstate
 * UART发送状态.
 */
enum uart_txstate
{
    UART_TXIDLE=0,      /**< UART发送空闲 */
    UART_TXBUSY=1,      /**< UART正在发送 */
};


typedef enum  uart_enable uart_enable_e; /**< UART使能枚举定义  */

/**
 * \enum uart_enable
 * UART使能枚举定义.
 */
enum uart_enable
{
    UART_DISABLE=0,     /**< UART不使能 */
    UART_ENABLE=1,      /**< UART使能 */
};

typedef enum  uart_type uart_type_e; /**< UART处理方式枚举定义  */

/**
 * \enum hal_uart_type
 * UART处理方式枚举定义.
 */
enum uart_type
{
    UART_INTERRUPT=0,     /**< UART中断方式 */
    UART_CHECK=1,         /**< UART查询方式 */
};

typedef enum  uart_id uart_id_e; /**< UART ID枚举定义  */

/**
 * \enum uart_id
 * UART ID枚举定义.
 * \note id从0开始连续递增,预留足够多的id号\n
 *       实际用到的id号由BSP硬件资源实现决定.
 */
enum uart_id
{
    UART_ID_0=0,           /**< UART ID0 */
    UART_ID_1=1,           /**< UART ID1 */
    UART_ID_2=2,           /**< UART ID2 */
    UART_ID_3=3,           /**< UART ID3 */
    UART_ID_4=4,           /**< UART ID4 */
    UART_ID_5=5,           /**< UART ID5 */
    UART_ID_6=6,           /**< UART ID6 */
    UART_ID_7=7,           /**< UART ID7 */
    UART_ID_MAX=8,         /**< UART 个数 */
    UART_ID_INV=0xFF,      /**< 无效ID */
};


/**
 * \enum uart_parity
 * UART数据奇偶校验枚举定义.
 * \note 描述了所有标准校验方式,\n
 *       实际用到的校验由BSP硬件资源实现决定.
 */
enum uart_parity
{
    UART_CHECK_NONE=0,       /**< UART无检验 */
    UART_CHECK_EVEN=1,       /**< UART偶校验 */
    UART_CHECK_ODD=2,        /**< UART奇校验 */
    UART_CHECK_HIG=3,        /**< UART校验固定1  */
    UART_CHECK_LOW=4,        /**< UART校验固定0  */
};
typedef enum  uart_parity uart_parity_e; /**< UART数据奇偶校验枚举定义  */

/**
 * \enum uart_datalen
 * UART数据长度枚举定义.
 * \note 描述了所有标准数据长度,\n
 *       实际用到的数据长度由BSP硬件资源实现决定.
 */
enum uart_datalen
{
    UART_DATA_9=9,       /**< UART 9位数据 */
    UART_DATA_8=8,       /**< UART 8位数据 */
    UART_DATA_7=7,       /**< UART 7位数据 */
    UART_DATA_6=6,       /**< UART 6位数据 */
    UART_DATA_5=5,       /**< UART 5位数据 */
};
typedef enum  uart_datalen uart_datalen_e; /**< UART数据长度枚举定义  */

/**
 * \enum uart_stopb
 * UART停止位枚举定义.
 * \note 描述了所有标准停止位长度,\n
 *       实际用到的停止位长度由BSP硬件资源实现决定.
 */
enum uart_stopb
{
    UART_STOPB_1P5=0,       /**< UART 1.5位停止位 */
    UART_STOPB_1=1,         /**< UART 1位停止位 */
    UART_STOPB_2=2,         /**< UART 2位停止位 */
};
typedef enum  uart_stopb uart_stopb_e; /**< UART停止位枚举定义  */

/**
 * \enum uart_baud
 * UART波特率枚举定义.
 * \note 描述了常见波特率,\n
 *       实际支持的波特率由BSP硬件资源实现决定.
 */
typedef enum  uart_baud uart_baud_e; /**< UART波特率枚举定义  */

enum uart_baud
{
    UART_BAUD_600=(uint32_t)600,        /**< UART波特率600 */
    UART_BAUD_1200=(uint32_t)1200,      /**< UART波特率1200 */
    UART_BAUD_2400=(uint32_t)2400,      /**< UART波特率2400 */
    UART_BAUD_4800=(uint32_t)4800,      /**< UART波特率4800 */
    UART_BAUD_9600=(uint32_t)9600,      /**< UART波特率9600 */
    UART_BAUD_19200=(uint32_t)19200,    /**< UART波特率19200 */
    UART_BAUD_38400=(uint32_t)38400,    /**< UART波特率38400 */
    UART_BAUD_57600=(uint32_t)57600,    /**< UART波特率38400 */
    UART_BAUD_115200=(uint32_t)115200,  /**< UART波特率115200 */
    UART_BAUD_500000=(uint32_t)500000,  /**< UART波特率500000 */
};

typedef struct uart_cfg uart_cfg_t;  /**< UART配置结构体  */

/**
 * \struct uart_cfg
 * UART配置结构体.
 */
struct uart_cfg
{
    uart_id_e       id;             /**< UART id     */
    uart_datalen_e  datalen;        /**< UART数据长度*/
    uart_parity_e   parity;         /**< UART校验    */
    uart_stopb_e    stopb;          /**< UART停止位  */
    uart_baud_e     baud;           /**< UART波特率  */
};

typedef enum  uart_err uart_err_e; /**< UART错误类型枚举定义  */

/**
 * \enum uart_err
 * UART错误类型枚举定义.
 */
enum uart_err
{
    UART_ERR_NONE=0x00,           /**< UART无错误 */
    UART_ERR_PER=0x02,            /**< UART校验错误 */
    UART_ERR_FER=0x04,            /**< UART帧错误 */
    UART_ERR_ORER=0x08,           /**< UART溢出错误 */
    UART_ERR_OTHER=0x10,          /**< UART其他错误 */
};

/**
 * \typedef uart_isrcallback_pfun
 * 中断服务回调函数.
 * - rxerrfun输入错误回调函数时,param指向错误码字节数据,不需要返回值.
 * - rxfun输入缓冲满回调函数时,param指向读到的字节数据,不需要返回值.
 * - txfun发送缓冲空回调函数,需要往param写需要发送的字节,返回值0表示还有数据需要发送 返回其他值标志本次为最后一字节数据.
 * - txcmpfun发送完回调函数,需要往param写需要发送的字节,返回值0表示还有数据需要发送 返回其他值标志本次为最后一字节数据.
 */
typedef int8_t (*uart_isrcallback_pfun)(uint8_t* param) ;   /**< 中断服务回调函数        */

typedef struct uart_isrcallback_st uart_isrcallback_pfun_t;  /**< 中断服务回调函数结构体  */

/**
 * \struct uart_isrcallback_st
 * UART回调函数结构体.
 */
struct uart_isrcallback_st
{
    uart_id_e        id;              /**< UART ID              */
    uart_isrcallback_pfun rxerrfun;   /**< 输入错误回调函数     */
    uart_isrcallback_pfun rxfun;      /**< 输入缓冲满回调函数   */
    uart_isrcallback_pfun txfun;      /**< 发送缓冲空回调函数   */
    uart_isrcallback_pfun txcmpfun;   /**< 发送完回调函数       */
};


/*****************************************************************************
 *                                                                           *
 *                             定时器数据结构描述                            *
 *                                                                           *
 ****************************************************************************/

typedef enum  timer_id timer_id_e; /**< TIMER ID枚举定义  */

/**
 * \enum timer_id
 * TIMER ID枚举定义.
 * \note id从0开始连续递增,预留足够多的id号\n
 *       实际用到的id号由BSP硬件资源实现决定.
 */
enum timer_id
{
    TIMER_ID_0=0,         /**< TIMER ID0 */
    TIMER_ID_1,           /**< TIMER ID1 */
    TIMER_ID_2,           /**< TIMER ID2 */
    TIMER_ID_3,           /**< TIMER ID3 */
    TIMER_ID_4,           /**< TIMER ID4 */
    TIMER_ID_5,           /**< TIMER ID5 */
    TIMER_ID_6,           /**< TIMER ID6 */
    TIMER_ID_7,           /**< TIMER ID7 */
    TIMER_ID_8,           /**< TIMER ID8 */
    TIMER_ID_9,           /**< TIMER ID9 */
    TIMER_ID_10,          /**< TIMER ID10 */
    TIMER_ID_11,          /**< TIMER ID11 */
    TIMER_ID_12,          /**< TIMER ID12 */
    TIMER_ID_13,          /**< TIMER ID13 */
    TIMER_ID_14,          /**< TIMER ID14 */
    TIMER_ID_15,          /**< TIMER ID15 */
    TIMER_ID_16,          /**< TIMER ID16 */
    TIMER_ID_17,          /**< TIMER ID17 */
    TIMER_ID_MAX,         /**< UART 个数 */
    TIMER_ID_INV=0xFF,    /**< 无效ID */
};

typedef enum  timer_enable timer_enable_e; /**< TIMER使能枚举定义  */

/**
 * \enum timer_enable
 * TIMER使能枚举定义.
 */
enum timer_enable
{
    TIMER_DISABLE=0,     /**< TIMER不使能 */
    TIMER_ENABLE=1,      /**< TIMER使能 */
};

typedef void (*timer_callback_pfun)(void);   /**< 定时器中断服务回调函数        */

/*****************************************************************************
 *                                                                           *
 *                             SPI数据结构描述                            *
 *                                                                           *
 ****************************************************************************/
/**
 * \enum spi_id
 * SPI ID枚举定义.
 * \note id从0开始连续递增,预留足够多的id号\n
 *       实际用到的id号由BSP硬件资源实现决定.
 */
enum spi_id
{
    SPI_ID_1 = 0,
    SPI_ID_2,
    SPI_ID_3,
    SPI_ID_MAX
};

typedef enum spi_id spi_id_t;

enum spi_mode
{
    SPI_MODE_0 = 0,
    SPI_MODE_1,
    SPI_MODE_2,
    SPI_MODE_3,
    SPI_MODE_MAX
};

typedef enum spi_mode spi_mode_t;


enum spi_msb_lsb
{
    SPI_MSB = 0,
    SPI_LSB_1,
};

typedef enum spi_msb_lsb spi_msb_lsb_e;

enum spi_speed
{
    SPI_SPEED_APB2CLK_DIV2 = 0,
    SPI_SPEED_APB2CLK_DIV4,
    SPI_SPEED_APB2CLK_DIV8,
    SPI_SPEED_APB2CLK_DIV16,
    SPI_SPEED_APB2CLK_DIV32,
    SPI_SPEED_APB2CLK_DIV64,
    SPI_SPEED_APB2CLK_DIV128,
    SPI_SPEED_APB2CLK_DIV256,
    SPI_SPEED_MAX
};

typedef enum spi_speed spi_speed_t;


typedef struct spi_cfg_st spi_cfg_t;  /**< SPI配置数结构体  */

/**
 * \struct spi_cfg_st
 * SPI配置.
 */
struct spi_cfg_st
{
    spi_id_t spi_id_e;
    uint32_t speed;
    spi_mode_t spi_mode_e;
    spi_msb_lsb_e msblsb;
    uint8_t dat_bit_len_u8;
};

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
