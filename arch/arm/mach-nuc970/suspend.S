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

	@ Set PLL Stable Counter to 0xFFFF
	mov       r1,#0xF8000000
	add       r0,r1,#0x00002000             @ r0 = NUC970_VA_GCR
	ldr 			r2,=0x0000FFFF
	str       r2,[r0,#0x0280]

	@ Enable Reset DLL(bit[8]) of DDR2
	mov       r1,#0xF0000000
	add       r0,r1,#0x00001800             @ r0 = NUC970_VA_SDIC
	ldr       r0,[r0,#0x0018]
	orr       r0,r0,#0x00000100
	add       r1,r1,#0x00001800
	str       r0,[r1,#0x0018]

	@ delay
	mov r2, #1000
	mov r1, #0
	mov r0, #1
	loop1:  add r1, r1, r0
	cmp r1, r2
	bne loop1

	@ Disable Reset DLL(bit[8]) of DDR2
	mov       r1,#0xF0000000
	add       r0,r1,#0x00001800             @ r0 = NUC970_VA_SDIC
	ldr       r0,[r0,#0x0018]
	bic       r0,r0,#0x00000100
	add       r1,r1,#0x00001800
	str       r0,[r1,#0x0018]

	@ delay
	mov r2, #1000
	mov r1, #0
	mov r0, #1
	loop2:  add r1, r1, r0
	cmp r1, r2
	bne loop2

	@ Set SDIC_OPMCTL[16] low to disable auto power down mode
	mov       r1,#0xF0000000
	add       r0,r1,#0x00001800             @ r0 = NUC970_VA_SDIC
	ldr       r0,[r0,#0x0000]
	bic       r0,r0,#0x00010000
	add       r1,r1,#0x00001800
	str       r0,[r1,#0x0000]

	mov       r1,#0xF0000000
	add       r0,r1,#0x00001800             @ r0 = NUC970_VA_SDIC
	ldr       r0,[r0,#0x0004]
	bic       r0,r0,#0x00000020
	add       r1,r1,#0x00001800
	str       r0,[r1,#0x0004]

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

	@ Set SDIC_OPMCTL[16] hight to disable auto power down mode
	mov       r1,#0xF0000000
	add       r0,r1,#0x00001800             @ r0 = NUC970_VA_SDIC
	ldr       r0,[r0,#0x0004]
	orr       r0,r0,#0x00000020
	add       r1,r1,#0x00001800
	str       r0,[r1,#0x0004]

	mov       r1,#0xF0000000
	add       r0,r1,#0x00001800             @ r0 = NUC970_VA_SDIC
	ldr       r0,[r0,#0x0000]
	orr       r0,r0,#0x00010000
	add       r1,r1,#0x00001800
	str       r0,[r1,#0x0000]

	@ Return nuc970_sys_suspend function
	mov    	pc, lr
ENTRY(nuc970_sys_suspend_sz)
	.word	. - nuc970_sys_suspend
