/*
 * arch/arm/mach-nuc970/include/mach/regs-irq.h
 *
 * Copyright (c) 2012 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef ___ASM_ARCH_REGS_IRQ_H
#define ___ASM_ARCH_REGS_IRQ_H

/* Advance Interrupt Controller (AIC) Registers */

#define AIC_BA    		NUC970_VA_IRQ

#define REG_AIC_MECR		(AIC_BA+0x130)
#define REG_AIC_MECRH		(AIC_BA+0x134)
#define REG_AIC_MDCR		(AIC_BA+0x138)
#define REG_AIC_MDCRH		(AIC_BA+0x13C)
//#define REG_AIC_SSCR		(AIC_BA+0x128)
//#define REG_AIC_SCCR		(AIC_BA+0x12C)
#define REG_AIC_EOSCR		(AIC_BA+0x150)

#define AIC_ISNR		(0x120)
#define AIC_IPER		(0x118)

/*16-18 bits of REG_AIC_GEN define irq(2-4) group*/


#endif /* ___ASM_ARCH_REGS_IRQ_H */
