/* generated vector header file - do not edit */
        #ifndef VECTOR_DATA_H
        #define VECTOR_DATA_H
                /* Number of interrupts allocated */
        #ifndef VECTOR_DATA_IRQ_COUNT
        #define VECTOR_DATA_IRQ_COUNT    (10)
        #endif
        /* ISR prototypes */
        void sci_uart_rxi_isr(void);
        void sci_uart_txi_isr(void);
        void sci_uart_tei_isr(void);
        void sci_uart_eri_isr(void);
        void r_icu_isr(void);
        void sci_uart6_rxi_isr(void);
        void sci_uart6_txi_isr(void);
        void sci_uart6_tei_isr(void);
        void sci_uart6_eri_isr(void);
        void sci_uart6_am_isr(void);

        /* Vector table allocations */
        #endif /* VECTOR_DATA_H */
