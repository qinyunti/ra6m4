/* generated vector source file - do not edit */
        #include "bsp_api.h"
        /* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
        #if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_MAX_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = sci_uart_rxi_isr, /* SCI7 RXI (Received data full) */
            [1] = sci_uart_txi_isr, /* SCI7 TXI (Transmit data empty) */
            [2] = sci_uart_tei_isr, /* SCI7 TEI (Transmit end) */
            [3] = sci_uart_eri_isr, /* SCI7 ERI (Receive error) */
            [4] = r_icu_isr, /* ICU IRQ0 (External pin interrupt 0) */

            [5] = sci_uart6_rxi_isr, /* SCI6 RXI (Received data full) */
            [6] = sci_uart6_txi_isr, /* SCI6 TXI (Transmit data empty) */
            [7] = sci_uart6_tei_isr, /* SCI6 TEI (Transmit end) */
            [8] = sci_uart6_eri_isr, /* SCI6 ERI (Receive error) */
            [9] = sci_uart6_am_isr,  /* SCI6 AM (Address match) */

            [10] = can0_rxerr_isr,  /*  Error interrupt */
            [11] = can0_rxfifo_isr,  /*  Receive FIFO interrupt */
            [12] = can0_txfifo_isr,  /*  Transmit FIFO interrupt */
            [13] = can0_rx_isr,  /*  Reception complete interrupt */
            [14] = can0_tx_isr,   /*  Transmission complete interrupt */
        };
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_MAX_ENTRIES] =
        {
            [0] = BSP_PRV_IELS_ENUM(EVENT_SCI7_RXI), /* SCI7 RXI (Received data full) */
            [1] = BSP_PRV_IELS_ENUM(EVENT_SCI7_TXI), /* SCI7 TXI (Transmit data empty) */
            [2] = BSP_PRV_IELS_ENUM(EVENT_SCI7_TEI), /* SCI7 TEI (Transmit end) */
            [3] = BSP_PRV_IELS_ENUM(EVENT_SCI7_ERI), /* SCI7 ERI (Receive error) */
            [4] = BSP_PRV_IELS_ENUM(EVENT_ICU_IRQ0), /* ICU IRQ0 (External pin interrupt 0) */

            [5] = BSP_PRV_IELS_ENUM(EVENT_SCI6_RXI), /* SCI6 RXI (Received data full) */
            [6] = BSP_PRV_IELS_ENUM(EVENT_SCI6_TXI), /* SCI6 TXI (Transmit data empty) */
            [7] = BSP_PRV_IELS_ENUM(EVENT_SCI6_TEI), /* SCI6 TEI (Transmit end) */
            [8] = BSP_PRV_IELS_ENUM(EVENT_SCI6_ERI), /* SCI6 ERI (Receive error) */
            [9] = BSP_PRV_IELS_ENUM(EVENT_SCI6_AM),  /* SCI6 AM (Address match) */

            [10] = BSP_PRV_IELS_ENUM(EVENT_CAN0_ERROR),       /*  Error interrupt */
            [11] = BSP_PRV_IELS_ENUM(EVENT_CAN0_FIFO_RX),     /*  Receive FIFO interrupt */
            [12] = BSP_PRV_IELS_ENUM(EVENT_CAN0_FIFO_TX),     /*  Transmit FIFO interrupt */
            [13] = BSP_PRV_IELS_ENUM(EVENT_CAN0_MAILBOX_RX),  /*  Reception complete interrupt */
            [14] = BSP_PRV_IELS_ENUM(EVENT_CAN0_MAILBOX_TX),  /*  Transmission complete interrupt */
        };
        #endif
