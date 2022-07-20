#include <stdint.h>
#include <string.h>
#include "bsp_can.h"
#include "slcan.h"
#include "app_cancomu.h"
#include "driver_uart.h"
/*
    O - Open channel
    C - Close channel
    S0 - Set bitrate to 10k
    S1 - Set bitrate to 20k
    S2 - Set bitrate to 50k
    S3 - Set bitrate to 100k
    S4 - Set bitrate to 125k
    S5 - Set bitrate to 250k
    S6 - Set bitrate to 500k
    S7 - Set bitrate to 750k
    S8 - Set bitrate to 1M
    M0 - Set mode to normal mode (default)
    M1 - Set mode to silent mode
    A0 - Disable automatic retransmission
    A1 - Enable automatic retransmission (default)
    TIIIIIIIILDD... - Transmit data frame (Extended ID) [ID, length, data]
    tIIILDD... - Transmit data frame (Standard ID) [ID, length, data]
    RIIIIIIIIL - Transmit remote frame (Extended ID) [ID, length]
    rIIIL - Transmit remote frame (Standard ID) [ID, length]
    V - Returns firmware version and remote path as a string
*/

/* CAN�յ�������תΪSLCAN��ʽ�ַ���  */
int8_t slcan_parse_frame(can_msg_st *msg, uint8_t* buf)
{
    uint8_t msg_position = 0;
    uint8_t id_len = 3;
    uint32_t can_id = msg->id & 0x1FFFFFFF;
    /* ���һ���ַ� */
    if(msg->id & (1u<<30))
    {
        if(msg->id & (1u<<31))
        {
            /* Զ����չ֡ */
            buf[msg_position] = 'R';
            id_len = 8;
        }
        else
        {
            /* Զ�̱�׼֡ */
            buf[msg_position] = 'r';
        }
    }
    else
    {
        if(msg->id & (1u<<31))
        {
            /* ��չ����֡ */
            buf[msg_position] = 'T';
            id_len = 8;
        }
        else
        {
            /* ��׼����֡ */
            buf[msg_position] = 'T';
        }
    }

    /* ��֡ID */
    msg_position++;
    for(uint8_t j = id_len; j > 0; j--)
    {
        /* ���ұ� ��λ��ʼ */
        buf[j] = (can_id & 0xF);
        can_id = can_id >> 4;
        msg_position++;
    }

    /* ��DLC */
    buf[msg_position++] = msg->dlc;

    /* ������ */
    for (uint8_t j = 0; j < msg->dlc; j++)
    {
        buf[msg_position++] = (msg->data[j] >> 4);
        buf[msg_position++] = (msg->data[j] & 0x0F);
    }

    /* �ӵڶ����ַ���ʼ 16������תΪ�ַ� */
    for (uint8_t j = 1; j < msg_position; j++)
    {
        if (buf[j] < 0xA)
        {
            buf[j] += 0x30;
        }
        else
        {
            buf[j] += 0x37;
        }
    }

    /* ���ӻس� */
    buf[msg_position++] = '\r';

    /* ������֡�󳤶� */
    return msg_position;
}


/* ������λ�����͹���������ת����CAN */
int8_t slcan_parse_str(uint8_t *buf, uint8_t len)
{
    can_msg_st msg;
    memset(&msg,0,sizeof(msg));

    /* CANID ��ʽ
     * 31  30  29 28-18     17-0
     * IDE RTR -  SID[10:0] EID[17:0]
     */

    /* �ӵڶ����ַ���ʼ���ַ�תΪ���� */
    for (uint8_t i = 1; i < len; i++)
    {
        if(buf[i] >= 'a')
        {
            buf[i] = buf[i] - 'a' + 10;
        }
        else if(buf[i] >= 'A')
        {
            buf[i] = buf[i] - 'A' + 10;
        }
        else
        {
            buf[i] = buf[i] - '0';
        }
    }


    /* �������� */
    switch(buf[0])
    {
        case 'O':
            /* ��CAN */
            return 0;

        case 'C':
            /* �ر�CAN */
            return 0;

        case 'S':
            /* �������� */
            if(buf[1] >= 9)
            {
                /* S0~S8 */
                return -1;
            }
            return 0;

        case 'm':
        case 'M':
            /* ����ģʽ */
            if (buf[1] == 1)
            {
                /* ��Ĭ */
            }
            else
            {
                /* �������� */
            }
            return 0;

        case 'a':
        case 'A':
            /* �Զ��ط����� */
            if (buf[1] == 1)
            {
                /* ʹ���Զ��ط� */
            }
            else
            {
                /* ��ʹ���Զ��ط� */
            }
            return 0;

        case 'V':
        case 'v':
        {
            /* ��ȡ�̼��汾 */
            int8_t erro;
            driver_uart_send(UART_ID_6, "V1.0", 5, 1000, &erro);
            return 0;
        }

        case 'T':
            /* ��չ����֡ */
            msg.id |= 1u<<31;
            msg.id &= ~(1u<<30);
            break;
        case 't':
            /* ��׼����֡ */
            msg.id &= ~(1u<<31);
            msg.id &= ~(1u<<30);
            break;
        case 'R':
            /* ��չԶ��֡ */
            msg.id |= 1u<<31;
            msg.id |= 1u<<30;
            break;
        case 'r':
            /* ��׼Զ��֡ */
            msg.id &= ~(1u<<31);
            msg.id |= 1u<<30;
            break;
        default:
            /* ����ָ�� */
            return -1;
    }


    /* ��ȡ֡ID */
    uint32_t id = 0;
    uint8_t msg_position = 1;
    if (msg.id & (1<<31))
    {
        /* ��չ֡ 8���ַ���֡ID */
        while (msg_position <= 8)
        {
            id *= 16;
            id += buf[msg_position++];
        }
    }
    else
    {
        /* ��׼֡ 3���ַ���֡ID */
        while (msg_position <= 3)
        {
            id *= 16;
            id += buf[msg_position++];
        }
    }
    msg.id |= id;
    /* ��ȡDLC */
    msg.dlc = buf[msg_position++];
    if(msg.dlc > 8)
    {
        return -1;
    }

    /* ��ȡ���� */
    for(uint8_t j = 0; j < msg.dlc; j++)
    {
        msg.data[j] = (buf[msg_position] << 4) + buf[msg_position+1];
        msg_position += 2;
    }

    /* �������� */
    can_tx(CAN_ID_0,&msg,1);

    return 0;
}
