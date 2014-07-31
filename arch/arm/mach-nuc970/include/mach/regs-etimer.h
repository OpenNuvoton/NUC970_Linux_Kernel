/*
 * arch/arm/mach-nuc970/include/mach/regs-etimer.h
 *
 * Copyright (c) 2014 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __ASM_ARCH_REGS_ETIMER_H
#define __ASM_ARCH_REGS_ETIMER_H

/* ETimer Registers */


#define REG_ETMR_CTL(x)		(NUC970_VA_ETIMER + 0x100 * (x) + 0x00)
#define REG_ETMR_PRECNT(x)	(NUC970_VA_ETIMER + 0x100 * (x) + 0x04)
#define REG_ETMR_CMPR(x)	(NUC970_VA_ETIMER + 0x100 * (x) + 0x08)
#define REG_ETMR_IER(x)		(NUC970_VA_ETIMER + 0x100 * (x) + 0x0C)
#define REG_ETMR_ISR(x)		(NUC970_VA_ETIMER + 0x100 * (x) + 0x10)
#define REG_ETMR_DR(x)		(NUC970_VA_ETIMER + 0x100 * (x) + 0x14)
#define REG_ETMR_TCAP(x)	(NUC970_VA_ETIMER + 0x100 * (x) + 0x18)


#endif /*  __ASM_ARCH_REGS_ETIMER_H */
