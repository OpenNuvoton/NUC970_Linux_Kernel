/*
 * arch/arm/mach-nuc970/include/mach/regs-clock.h
 *
 * Copyright (c) 2014 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

#ifndef __ASM_ARCH_REGS_AIC_H
#define __ASM_ARCH_REGS_AIC_H

/* Advance Interrupt Controller (AIC) Registers */
#define AIC_BA    NUC970_VA_IRQ /* Interrupt Controller */
#define REG_AIC_SCR1    (AIC_BA+0x04)    /* Source control register 1 */
#define REG_AIC_SCR2    (AIC_BA+0x08)    /* Source control register 2 */
#define REG_AIC_SCR3    (AIC_BA+0x0C)    /* Source control register 3 */
#define REG_AIC_SCR4    (AIC_BA+0x10)    /* Source control register 4 */
#define REG_AIC_SCR5    (AIC_BA+0x14)    /* Source control register 5 */
#define REG_AIC_SCR6    (AIC_BA+0x18)    /* Source control register 6 */
#define REG_AIC_SCR7    (AIC_BA+0x1C)    /* Source control register 7 */
#define REG_AIC_SCR8    (AIC_BA+0x20)    /* Source control register 8 */
#define REG_AIC_SCR9    (AIC_BA+0x24)    /* Source control register 9 */
#define REG_AIC_SCR10   (AIC_BA+0x28)    /* Source control register 10 */
#define REG_AIC_SCR11   (AIC_BA+0x2C)    /* Source control register 11 */
#define REG_AIC_SCR12   (AIC_BA+0x30)    /* Source control register 12 */
#define REG_AIC_SCR13   (AIC_BA+0x34)    /* Source control register 13 */
#define REG_AIC_SCR14   (AIC_BA+0x38)    /* Source control register 14 */
#define REG_AIC_SCR15   (AIC_BA+0x3C)    /* Source control register 15 */
#define REG_AIC_SCR16   (AIC_BA+0x40)    /* Source control register 16 */
#define REG_AIC_SCR17   (AIC_BA+0x44)    /* Source control register 17 */
#define REG_AIC_SCR18   (AIC_BA+0x48)    /* Source control register 18 */
#define REG_AIC_SCR19   (AIC_BA+0x4C)    /* Source control register 19 */
#define REG_AIC_SCR20   (AIC_BA+0x50)    /* Source control register 20 */
#define REG_AIC_SCR21   (AIC_BA+0x54)    /* Source control register 21 */
#define REG_AIC_SCR22   (AIC_BA+0x58)    /* Source control register 22 */
#define REG_AIC_SCR23   (AIC_BA+0x5C)    /* Source control register 23 */
#define REG_AIC_SCR24   (AIC_BA+0x60)    /* Source control register 24 */
#define REG_AIC_SCR25   (AIC_BA+0x64)    /* Source control register 25 */
#define REG_AIC_SCR26   (AIC_BA+0x68)    /* Source control register 26 */
#define REG_AIC_SCR27   (AIC_BA+0x6C)    /* Source control register 27 */
#define REG_AIC_SCR28   (AIC_BA+0x70)    /* Source control register 28 */
#define REG_AIC_SCR29   (AIC_BA+0x74)    /* Source control register 29 */
#define REG_AIC_SCR30   (AIC_BA+0x78)    /* Source control register 30 */
#define REG_AIC_SCR31   (AIC_BA+0x7C)    /* Source control register 31 */
#define REG_AIC_IRQSC   (AIC_BA+0x80)    /* External Interrupt Control Register */
#define REG_AIC_GEN     (AIC_BA+0x84)    /* Interrupt Group Enable Control Register */
#define REG_AIC_GASR    (AIC_BA+0x88)    /* Interrupt Group Active Status Register */
#define REG_AIC_GSCR    (AIC_BA+0x8C)    /* Interrupt Group Status Clear Register */
#define REG_AIC_IRSR    (AIC_BA+0x100)   /* Interrupt raw status register */
#define REG_AIC_IASR    (AIC_BA+0x104)   /* Interrupt active status register */
#define REG_AIC_ISR     (AIC_BA+0x108)   /* Interrupt status register */
#define REG_AIC_IPER    (AIC_BA+0x10C)   /* Interrupt priority encoding register */
#define REG_AIC_ISNR    (AIC_BA+0x110)   /* Interrupt source number register */
#define REG_AIC_IMR     (AIC_BA+0x114)   /* Interrupt mask register */
#define REG_AIC_OISR    (AIC_BA+0x118)   /* Output interrupt status register */
#define REG_AIC_MECR    (AIC_BA+0x120)   /* Mask enable command register */
#define REG_AIC_MDCR    (AIC_BA+0x124)   /* Mask disable command register */
#define REG_AIC_SSCR	(AIC_BA+0x128)	 /* Source set command register */
#define REG_AIC_SCCR	(AIC_BA+0x12C)   //zswan add it
#define REG_AIC_EOSCR   (AIC_BA+0x130)   /* End of service command register */

#endif /*  __ASM_ARCH_REGS_AIC_H */
