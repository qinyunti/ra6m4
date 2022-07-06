/**
 *****************************************************************************
 * \brief       BSP��(BSP)�������ݶ���ģ��(TYPEDEF)������ݽṹ����.
 * \details     Copyright (c) 2022,qinyunti@hotmail.com.
 *              All rights reserved.
 * \file        bsp_typedef.h
 * \author      qinyunti@hotmail.com
 * \version     1.0
 * \date        2022-07-03
 * \note        ʹ��ǰ�ο�ע��.\n
 *              .\n
 * \since       qinyunti@hotmail.com 2022-07-03 1.0  �½�
 * \par �޶���¼
 * - 2017-07-24 ��ʼ�汾
 * \par ��Դ˵��
 * - RAM:
 * - ROM:
 *****************************************************************************
 */

#ifndef BSP_TYPEDEF_H
#define BSP_TYPEDEF_H
#ifdef __cplusplus
 extern "C" {
#endif

/** \addtogroup BSP BSP��(BSP)
 *  \{
 */

/** \addtogroup BSP_TYPEDEF BSP��(BSP)�������ݶ���ģ��(TYPEDEF)
 *  \{
 */

#include <stdint.h>
/*****************************************************************************
 *                                                                           *
 *                             ���ݽṹ����                                                                     *
 *                                                                           *
 ****************************************************************************/

/** \defgroup BSP_TYPEDEF_DATA BSP��(BSP)�������ݶ���ģ��(TYPEDEF)���ݽṹ
  * \{
  */

/*****************************************************************************
 *                                                                           *
 *                             UART���ݽṹ����                                                             *
 *                                                                           *
 ****************************************************************************/

typedef enum  uart_txstate uart_txstate_e; /**< UART����״̬ö�ٶ���  */

/**
 * \enum uart_txstate
 * UART����״̬.
 */
enum uart_txstate
{
    UART_TXIDLE=0,      /**< UART���Ϳ��� */
    UART_TXBUSY=1,      /**< UART���ڷ��� */
};


typedef enum  uart_enable uart_enable_e; /**< UARTʹ��ö�ٶ���  */

/**
 * \enum uart_enable
 * UARTʹ��ö�ٶ���.
 */
enum uart_enable
{
    UART_DISABLE=0,     /**< UART��ʹ�� */
    UART_ENABLE=1,      /**< UARTʹ�� */
};

typedef enum  uart_type uart_type_e; /**< UART����ʽö�ٶ���  */

/**
 * \enum hal_uart_type
 * UART����ʽö�ٶ���.
 */
enum uart_type
{
    UART_INTERRUPT=0,     /**< UART�жϷ�ʽ */
    UART_CHECK=1,         /**< UART��ѯ��ʽ */
};

typedef enum  uart_id uart_id_e; /**< UART IDö�ٶ���  */

/**
 * \enum uart_id
 * UART IDö�ٶ���.
 * \note id��0��ʼ��������,Ԥ���㹻���id��\n
 *       ʵ���õ���id����BSPӲ����Դʵ�־���.
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
    UART_ID_MAX=8,         /**< UART ���� */
    UART_ID_INV=0xFF,      /**< ��ЧID */
};


/**
 * \enum uart_parity
 * UART������żУ��ö�ٶ���.
 * \note ���������б�׼У�鷽ʽ,\n
 *       ʵ���õ���У����BSPӲ����Դʵ�־���.
 */
enum uart_parity
{
    UART_CHECK_NONE=0,       /**< UART�޼��� */
    UART_CHECK_EVEN=1,       /**< UARTżУ�� */
    UART_CHECK_ODD=2,        /**< UART��У�� */
    UART_CHECK_HIG=3,        /**< UARTУ��̶�1  */
    UART_CHECK_LOW=4,        /**< UARTУ��̶�0  */
};
typedef enum  uart_parity uart_parity_e; /**< UART������żУ��ö�ٶ���  */

/**
 * \enum uart_datalen
 * UART���ݳ���ö�ٶ���.
 * \note ���������б�׼���ݳ���,\n
 *       ʵ���õ������ݳ�����BSPӲ����Դʵ�־���.
 */
enum uart_datalen
{
    UART_DATA_9=9,       /**< UART 9λ���� */
    UART_DATA_8=8,       /**< UART 8λ���� */
    UART_DATA_7=7,       /**< UART 7λ���� */
    UART_DATA_6=6,       /**< UART 6λ���� */
    UART_DATA_5=5,       /**< UART 5λ���� */
};
typedef enum  uart_datalen uart_datalen_e; /**< UART���ݳ���ö�ٶ���  */

/**
 * \enum uart_stopb
 * UARTֹͣλö�ٶ���.
 * \note ���������б�׼ֹͣλ����,\n
 *       ʵ���õ���ֹͣλ������BSPӲ����Դʵ�־���.
 */
enum uart_stopb
{
    UART_STOPB_1P5=0,       /**< UART 1.5λֹͣλ */
    UART_STOPB_1=1,         /**< UART 1λֹͣλ */
    UART_STOPB_2=2,         /**< UART 2λֹͣλ */
};
typedef enum  uart_stopb uart_stopb_e; /**< UARTֹͣλö�ٶ���  */

/**
 * \enum uart_baud
 * UART������ö�ٶ���.
 * \note �����˳���������,\n
 *       ʵ��֧�ֵĲ�������BSPӲ����Դʵ�־���.
 */
typedef enum  uart_baud uart_baud_e; /**< UART������ö�ٶ���  */

enum uart_baud
{
    UART_BAUD_600=(uint32_t)600,        /**< UART������600 */
    UART_BAUD_1200=(uint32_t)1200,      /**< UART������1200 */
    UART_BAUD_2400=(uint32_t)2400,      /**< UART������2400 */
    UART_BAUD_4800=(uint32_t)4800,      /**< UART������4800 */
    UART_BAUD_9600=(uint32_t)9600,      /**< UART������9600 */
    UART_BAUD_19200=(uint32_t)19200,    /**< UART������19200 */
    UART_BAUD_38400=(uint32_t)38400,    /**< UART������38400 */
    UART_BAUD_57600=(uint32_t)57600,    /**< UART������38400 */
    UART_BAUD_115200=(uint32_t)115200,  /**< UART������115200 */
    UART_BAUD_500000=(uint32_t)500000,  /**< UART������500000 */
};

typedef struct uart_cfg uart_cfg_t;  /**< UART���ýṹ��  */

/**
 * \struct uart_cfg
 * UART���ýṹ��.
 */
struct uart_cfg
{
    uart_id_e       id;             /**< UART id     */
    uart_datalen_e  datalen;        /**< UART���ݳ���*/
    uart_parity_e   parity;         /**< UARTУ��    */
    uart_stopb_e    stopb;          /**< UARTֹͣλ  */
    uart_baud_e     baud;           /**< UART������  */
};

typedef enum  uart_err uart_err_e; /**< UART��������ö�ٶ���  */

/**
 * \enum uart_err
 * UART��������ö�ٶ���.
 */
enum uart_err
{
    UART_ERR_NONE=0x00,           /**< UART�޴��� */
    UART_ERR_PER=0x02,            /**< UARTУ����� */
    UART_ERR_FER=0x04,            /**< UART֡���� */
    UART_ERR_ORER=0x08,           /**< UART������� */
    UART_ERR_OTHER=0x10,          /**< UART�������� */
};

/**
 * \typedef uart_isrcallback_pfun
 * �жϷ���ص�����.
 * - rxerrfun�������ص�����ʱ,paramָ��������ֽ�����,����Ҫ����ֵ.
 * - rxfun���뻺�����ص�����ʱ,paramָ��������ֽ�����,����Ҫ����ֵ.
 * - txfun���ͻ���ջص�����,��Ҫ��paramд��Ҫ���͵��ֽ�,����ֵ0��ʾ����������Ҫ���� ��������ֵ��־����Ϊ���һ�ֽ�����.
 * - txcmpfun������ص�����,��Ҫ��paramд��Ҫ���͵��ֽ�,����ֵ0��ʾ����������Ҫ���� ��������ֵ��־����Ϊ���һ�ֽ�����.
 */
typedef int8_t (*uart_isrcallback_pfun)(uint8_t* param) ;   /**< �жϷ���ص�����        */

typedef struct uart_isrcallback_st uart_isrcallback_pfun_t;  /**< �жϷ���ص������ṹ��  */

/**
 * \struct uart_isrcallback_st
 * UART�ص������ṹ��.
 */
struct uart_isrcallback_st
{
    uart_id_e        id;              /**< UART ID              */
    uart_isrcallback_pfun rxerrfun;   /**< �������ص�����     */
    uart_isrcallback_pfun rxfun;      /**< ���뻺�����ص�����   */
    uart_isrcallback_pfun txfun;      /**< ���ͻ���ջص�����   */
    uart_isrcallback_pfun txcmpfun;   /**< ������ص�����       */
};


/*****************************************************************************
 *                                                                           *
 *                             ��ʱ�����ݽṹ����                            *
 *                                                                           *
 ****************************************************************************/

typedef enum  timer_id timer_id_e; /**< TIMER IDö�ٶ���  */

/**
 * \enum timer_id
 * TIMER IDö�ٶ���.
 * \note id��0��ʼ��������,Ԥ���㹻���id��\n
 *       ʵ���õ���id����BSPӲ����Դʵ�־���.
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
    TIMER_ID_MAX,         /**< UART ���� */
    TIMER_ID_INV=0xFF,    /**< ��ЧID */
};

typedef enum  timer_enable timer_enable_e; /**< TIMERʹ��ö�ٶ���  */

/**
 * \enum timer_enable
 * TIMERʹ��ö�ٶ���.
 */
enum timer_enable
{
    TIMER_DISABLE=0,     /**< TIMER��ʹ�� */
    TIMER_ENABLE=1,      /**< TIMERʹ�� */
};

typedef void (*timer_callback_pfun)(void);   /**< ��ʱ���жϷ���ص�����        */

/*****************************************************************************
 *                                                                           *
 *                             SPI���ݽṹ����                            *
 *                                                                           *
 ****************************************************************************/
/**
 * \enum spi_id
 * SPI IDö�ٶ���.
 * \note id��0��ʼ��������,Ԥ���㹻���id��\n
 *       ʵ���õ���id����BSPӲ����Դʵ�־���.
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


typedef struct spi_cfg_st spi_cfg_t;  /**< SPI�������ṹ��  */

/**
 * \struct spi_cfg_st
 * SPI����.
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
