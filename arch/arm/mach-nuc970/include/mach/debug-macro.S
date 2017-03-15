/*
 * arch/arm/mach-nuc970/include/mach/debug-macro.S
 *
 *  Copyright (C) 2017 Nuvoton Technology Corp.
 *
 * Debugging macro include header
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <mach/hardware.h>

#define CLK_PCLKEN0	0xB0000218
#define SYS_GPE_MFPL	0xB0000090
#define UART_REG_THR	0x00
#define UART_REG_FSR	0x18
#define TX_EMPTY	0x00400000
#define TE_FLAG		0x10000000

#define TX_IDLE		(TX_EMPTY | TE_FLAG)

	.macro	addruart, rp, rv, tmp
	adr	\rp, 8f					@ get effective addr of 8f
	ldr	\rv, [\rp]				@ get absolute addr of 8f
	sub	\rv, \rv, \rp				@ offset between the two
	ldr	\rp, [\rp, #4]				@ abs addr of uart_init
	sub	\tmp, \rp, \rv				@ make it effective
	ldr	\rp, [\tmp, #0]				@ uart_init val
	cmp	\rp, #1					@ CLK/MFP configured?
	bne	6f
	mov	\rp, #0
	str	\rp, [\tmp, #0]		
	ldr	\tmp, =CLK_PCLKEN0			@ Enable UART0 CLK
	ldr	\rp, [\tmp, #0]
	orr	\rp, \rp, #0x10000
	str	\rp, [\tmp, #0]
	ldr	\tmp, =SYS_GPE_MFPL			@ Enable UART0 MFP
	mov	\rp, #0x99
	str	\rp, [\tmp, #0]

6:	ldr	\rp, =NUC970_PA_UART			@ Physical address
	ldr	\rv, =NUC970_VA_UART			@ Virtual address
	b	168f
	.align
8:	.word	.
	.word	nuc970_uart_init
	.ltorg
168:
	.endm

	.macro	senduart,rd,rx	
	str	\rd, [\rx, #(UART_REG_THR)]		@ Write to Tx FIFO
	.endm

	.macro	waituart,rd,rx				@ Do nothing
	.endm

	.macro	busyuart,rd,rx
1688:	ldr	\rd, [\rx, #(UART_REG_FSR)]		@ Read FIFO Status Register
	and	\rd, \rd, #TX_IDLE
	tst	\rd, #TX_IDLE
	beq	1688b
	.endm

