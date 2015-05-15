/*
 * Timer FIQ handler
 *
 * Copyright (C) 2015 Nuvoton
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/linkage.h>
#include <asm/assembler.h>

	.text
	.global fiq_handler_end

ENTRY(fiq_handler)
    stmdb	sp!, {r0-r12, lr}
    
    @ branch to c handler
    blx     r8
    
    @ read REG_AIC_IPER
    ldr     r12, [r10]
   
    @ write 0x1 to REG_AIC_EOSCR
    mov     r12, #0x1
    add     r11, r10, #0x38
    str     r12, [r11]
        
	/* return from FIQ */
	ldmia	sp!, {r0-r12, lr}
    subs	pc, lr, #4   
    
fiq_handler_end:
