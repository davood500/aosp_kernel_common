/*
 * include/linux/jz_uart.h
 *
 */

#ifndef __JZ_UART_H__
#define __JZ_UART_H__

/* Register Offset */
#define UART_RDR		OFF_RDR
#define UART_TDR		OFF_TDR
#define UART_DLLR	    OFF_DLLR
#define UART_DLHR	    OFF_DLHR
#define UART_IER		OFF_IER
#define UART_IIR		OFF_ISR
#define UART_FCR		OFF_FCR
#define UART_LCR		OFF_LCR
#define UART_MCR		OFF_MCR
#define UART_LSR		OFF_LSR
#define UART_MSR		OFF_MSR
#define UART_SPR		OFF_SPR
#define UART_ISR	    OFF_SIRCR
#define UART_UMR		OFF_UMR
#define UART_UACR       OFF_UACR

#define UARTIIR_IP      UARTISR_IP

#define UART_LSR_BRK_ERROR_BITS	0x1E /* BI, FE, PE, OE bits */

#define PORT_47XX   83

#endif /* __JZ_UART_H__ */

