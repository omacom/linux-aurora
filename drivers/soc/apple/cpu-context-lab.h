/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_CPU_CONTEXT_LAB_H
#define APPLE_CPU_CONTEXT_LAB_H

/* J314s serial0: Apple S5L/S3C2410 UART, MMIO32 and 16-byte FIFO. */
#define LAB_UART_PA 0x39b200000
#define LAB_UART_SIZE 0x1000
#define LAB_UART_UCON 0x04
#define LAB_UART_UFCON 0x08
#define LAB_UART_UFSTAT 0x18
#define LAB_UART_UTXH 0x20
#define LAB_UART_FIFO_ENABLED 1
#define LAB_UART_TX_FULL 0x200
#define LAB_UART_POLLS 8192

#ifndef __ASSEMBLY__
#include <linux/types.h>

void secondary_holding_pen(void);
void __noreturn apple_context_lab_poweroff(void __iomem *stop, u32 bit,
					 void __iomem *uart);
void apple_context_lab_resume(void);

#endif
#endif
