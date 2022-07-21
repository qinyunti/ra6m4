/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2021-10-10     Sherman       first version
 * 2021-11-03     Sherman       Add icu_sample
 */

#include <rtthread.h>
#include "hal_data.h"
#include <rtdevice.h>
#include "bsp_api.h"
#include "app_uartcomu.h"
#include "app_cancomu.h"

#define LED3_PIN    BSP_IO_PORT_01_PIN_06
#define USER_INPUT  "P105"

void hal_entry(void)
{
    rt_kprintf("\nHello RT-Thread!\n");

    SystemCoreClockUpdate();
    rt_kprintf("\nCore:%dHz!\n",SystemCoreClock);
    rt_kprintf("\nPCLKA:%dHz!\n",BSP_STARTUP_PCLKA_HZ);
    int pclkb = BSP_STARTUP_PCLKB_HZ;
    rt_kprintf("\nPCLKB:%dHz!\n",pclkb);

    R_PMISC->PWPR_b.B0WI = 0;   /* 解锁 PmnPFS */
    R_PMISC->PWPR_b.PFSWE = 1;

    R_PFS->PORT[1].PIN[6].PmnPFS_b.PMR = 0;   /* 普通IO */
    R_PFS->PORT[1].PIN[6].PmnPFS_b.PDR = 1;   /* 输出 */
    R_PFS->PORT[1].PIN[6].PmnPFS_b.PCR = 1;   /* 使能上拉*/
    R_PFS->PORT[1].PIN[6].PmnPFS_b.NCODR = 0; /* CMOS模式 非开漏*/
    R_PFS->PORT[1].PIN[6].PmnPFS_b.DSCR = 3;  /* 高驱动能力 */
    R_PFS->PORT[1].PIN[6].PmnPFS_b.EOFR = 0;  /* 未使用事件模式所以写默认值 */
    R_PFS->PORT[1].PIN[6].PmnPFS_b.ISEL = 0;  /* 不使能外部中断 */
    R_PFS->PORT[1].PIN[6].PmnPFS_b.ASEL = 0;  /* 非AD */
    R_PFS->PORT[1].PIN[6].PmnPFS_b.PODR = 0;  /* 默认输出0 熄灯*/
    R_PFS->PORT[1].PIN[6].PmnPFS_b.PSEL = 0;  /* PMR=1时才有效 */

    R_PMISC->PWPR_b.PFSWE = 0;   /* 锁定 PmnPFS */
    R_PMISC->PWPR_b.B0WI = 1;

    app_uartcomu_init();
    app_cancomu_init();
    uint32_t id = 0;
    while (1)
    {
        //R_PORT1->PCNTR3_b.POSR = (uint16_t)1<<6; /* 亮*/
        //R_PORT1->PCNTR1_b.PODR |= (uint16_t)1<<6;
        R_PMISC->PWPR_b.B0WI = 0;   /* 解锁 PmnPFS */
        R_PMISC->PWPR_b.PFSWE = 1;
        R_PFS->PORT[1].PIN[6].PmnPFS_b.PODR =1;
        R_PMISC->PWPR_b.PFSWE = 0;   /* 锁定 PmnPFS */
        R_PMISC->PWPR_b.B0WI = 1;
        rt_thread_mdelay(500);
        //R_PORT1->PCNTR3_b.PORR = (uint16_t)1<<6; /* 灭 */
        //R_PORT1->PCNTR1_b.PODR &= ~(uint16_t)1<<6;
        R_PMISC->PWPR_b.B0WI = 0;   /* 解锁 PmnPFS */
        R_PMISC->PWPR_b.PFSWE = 1;
        R_PFS->PORT[1].PIN[6].PmnPFS_b.PODR = 0;
        R_PMISC->PWPR_b.PFSWE = 0;   /* 锁定 PmnPFS */
        R_PMISC->PWPR_b.B0WI = 1;
        rt_thread_mdelay(500);

        can_msg_st msg;
        msg.id = 0x80000000;
        msg.dlc = 8;
        msg.id |= id++;
        for(int i=0;i<8;i++)
        {
            msg.data[i] = msg.id*8+i;
        }
        can_tx(CAN_ID_0,&msg,10);
    }
}

void irq_callback_test(void *args)
{
    rt_kprintf("\n IRQ00 triggered \n");
}

void icu_sample(void)
{
    /* init */
    rt_uint32_t pin = rt_pin_get(USER_INPUT);
    rt_kprintf("\n pin number : 0x%04X \n", pin);
    rt_err_t err = rt_pin_attach_irq(pin, PIN_IRQ_MODE_RISING, irq_callback_test, RT_NULL);
    if(RT_EOK != err)
    {
        rt_kprintf("\n attach irq failed. \n");
    }
    err = rt_pin_irq_enable(pin, PIN_IRQ_ENABLE);
    if(RT_EOK != err)
    {
        rt_kprintf("\n enable irq failed. \n");
    }
}
MSH_CMD_EXPORT(icu_sample, icu sample);
