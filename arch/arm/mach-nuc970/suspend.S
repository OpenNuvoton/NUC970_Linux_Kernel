 /* arch/arm/mach-nuc970/suspend.S
 *
 * Copyright (c) 2016 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#include <linux/linkage.h>
#include <asm/assembler.h>
#include <mach/map.h>

	.text
	.align  10
ENTRY(nuc970_sys_suspend)

	@ Enable DDR low frequency
	mov       r1,#0xF0000000
	add       r0,r1,#0x00001800             @ r0 = NUC970_VA_SDIC
	ldr       r0,[r0,#0x0000]
	orr       r0,r0,#0x00040000
	add       r1,r1,#0x00001800
	str       r0,[r1,#0x0000]

	@ Save UPLL setting	
	mov       r1,#0xF8000000
	add       r0,r1,#0x00002000             @ r0 = NUC970_VA_GCR
	ldr       r5,[r0,#0x0264]

	@ Set UPLL to 60MHz
	mov       r0,#0xC0000005
	mov       r1,#0xF8000000
	add       r1,r1,#0x00002000             @ r1 = NUC970_VA_GCR
	str       r0,[r1,#0x0264]

	@ Enter CPU Idle mode#1
	mov     r0, #0
	mrc     p15, 0, r1, c1, c0, 0           @ Read control register
	mcr     p15, 0, r0, c7, c10, 4          @ Drain write buffer
	bic     r2, r1, #1 << 12
	mrs     r3, cpsr                        @ Disable FIQs while Icache
	orr     ip, r3, #PSR_F_BIT              @ is disabled
	msr     cpsr_c, ip
	mcr     p15, 0, r2, c1, c0, 0           @ Disable I cache
	mcr     p15, 0, r0, c7, c0, 4           @ Wait for interrupt
	mcr     p15, 0, r1, c1, c0, 0           @ Restore ICache enable
	msr     cpsr_c, r3                      @ Restore FIQ state

	@ Restore UPLL setting
	mov       r1,#0xF8000000
	add       r1,r1,#0x00002000             @ r1 = NUC970_VA_GCR
	str       r5,[r1,#0x0264]

	@ Disable DDR low frequency
	mov       r1,#0xF0000000
	add       r0,r1,#0x00001800             @ r0 = NUC970_VA_SDIC
	ldr       r0,[r0,#0x0000]
	bic       r0,#0x00040000
	add       r1,r1,#0x00001800
	str       r0,[r1,#0x0000]

	@ Return nuc970_sys_suspend function 
	mov    	pc, lr
ENTRY(nuc970_sys_suspend_sz)
	.word	. - nuc970_sys_suspend
