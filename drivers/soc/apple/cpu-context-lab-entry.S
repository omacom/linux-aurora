/* SPDX-License-Identifier: GPL-2.0-only */
#include <linux/linkage.h>
#include <asm/assembler.h>
#include "cpu-context-lab.h"

/* x11 is the UART base; only scratch x12/x13 are changed. Never wait forever. */
	.macro lab_uart_char value
	mov	w13, #LAB_UART_POLLS
.Luart_poll\@:
	ldr	w12, [x11, #LAB_UART_UFSTAT]
	tbz	w12, #9, .Luart_write\@
	subs	w13, w13, #1
	b.ne	.Luart_poll\@
	b	.Luart_done\@
.Luart_write\@:
	mov	w12, #\value
	str	w12, [x11, #LAB_UART_UTXH]
.Luart_done\@:
	.endm

	.macro lab_uart_mark value
	lab_uart_char 0x5e /* ^ */
	lab_uart_char \value
	lab_uart_char 0x0a
	dsb	sy
	.endm

/* x14 holds the read-only snapshot; x9/x10/x12/x13/x16 are scratch. */
	.macro lab_uart_hex tag
	lab_uart_char 0x5e
	lab_uart_char \tag
	lab_uart_char 0x3d
	mov	w16, #60
.Lhex_digit\@:
	lsrv	x9, x14, x16
	and	w9, w9, #0xf
	cmp	w9, #10
	add	w9, w9, #0x30
	add	w10, w9, #0x27
	csel	w9, w9, w10, lo
	mov	w13, #LAB_UART_POLLS
.Lhex_poll\@:
	ldr	w12, [x11, #LAB_UART_UFSTAT]
	tbz	w12, #9, .Lhex_write\@
	subs	w13, w13, #1
	b.ne	.Lhex_poll\@
	b	.Lhex_next\@
.Lhex_write\@:
	str	w9, [x11, #LAB_UART_UTXH]
.Lhex_next\@:
	subs	w16, w16, #4
	b.ge	.Lhex_digit\@
	lab_uart_char 0x0a
	dsb	sy
	.endm

	.text
/*
 * x0: CPU stop register, x1: target bit, x2: UART VA. CPU0 holds shared L2 awake.
 * Sweep only the local L1. No stack access or C call follows this sweep.
 * M1 disables set/way L2 operations in EHID4, not L1 operations.
 */
SYM_FUNC_START(apple_context_lab_poweroff)
	mov	x11, x2
	dsb	sy
	msr	csselr_el1, xzr
	isb
	mrs	x2, ccsidr_el1
	and	x3, x2, #7
	add	x3, x3, #4
	mrs	x4, id_aa64mmfr2_el1
	ubfx	x4, x4, #20, #4
	cbnz	x4, 1f
	ubfx	x4, x2, #3, #10
	ubfx	x5, x2, #13, #15
	b	2f
1:
	ubfx	x4, x2, #3, #21
	ubfx	x5, x2, #32, #24
2:
	clz	w6, w4
	mov	x7, x4
3:
	mov	x8, x5
4:
	lsl	x9, x7, x6
	lsl	x10, x8, x3
	orr	x9, x9, x10
	dc	cisw, x9
	subs	x8, x8, #1
	b.ge	4b
	subs	x7, x7, #1
	b.ge	3b
	dsb	sy
	lab_uart_mark 0x6c /* l: final L1 sweep completed */
	str	w1, [x0]
	dsb	sy
	lab_uart_mark 0x77 /* w: stop request written, about to enter WFI */
	isb
	wfi
	/* Report only the first WFI return; never acknowledge or clear its cause. */
	mrs	x14, isr_el1
	mrs	x15, S3_5_C15_C1_1
	lab_uart_mark 0x71 /* q: WFI returned without entering the resume vector */
	lab_uart_hex 0x48 /* H: ISR_EL1 captured at first return */
	mov	x14, x15
	lab_uart_hex 0x51 /* Q: fast IPI status captured at first return */
	mrs	x14, cntp_ctl_el0
	lab_uart_hex 0x4e /* N: host physical timer control */
	mrs	x14, cntv_ctl_el0
	lab_uart_hex 0x5a /* Z: host virtual timer control */
	mrs	x14, S3_1_C15_C0_0
	lab_uart_hex 0x4d /* M: PMCR0, as read by the AIC FIQ handler */
	mrs	x14, S3_7_C15_C0_4
	lab_uart_hex 0x55 /* U: uncore PMC interrupt mode */
	mrs	x14, S3_7_C15_C6_4
	lab_uart_hex 0x57 /* W: uncore PMC interrupt status */
	mrs	x14, CurrentEL
	lab_uart_hex 0x4c /* L: gate EL2-only timer reads below */
	cmp	x14, #8
	b.ne	6f
	mrs	x14, S3_5_C14_C2_1
	lab_uart_hex 0x70 /* p: guest physical timer control, EL02 */
	mrs	x14, S3_5_C14_C3_1
	lab_uart_hex 0x76 /* v: guest virtual timer control, EL02 */
	mrs	x14, S3_5_C15_C1_3
	lab_uart_hex 0x68 /* h: guest timer FIQ enable, EL2 */
6:
	isb
	wfi
	b	6b
SYM_FUNC_END(apple_context_lab_poweroff)

	.pushsection ".idmap.text", "ax"
/* AURA calls this PA using its normal secondary entry contract; no return. */
SYM_CODE_START(apple_context_lab_resume)
	mov_q	x11, LAB_UART_PA
	lab_uart_mark 0x72 /* r: physical resume trampoline reached */
	b	cpu_resume
SYM_CODE_END(apple_context_lab_resume)
	.popsection
