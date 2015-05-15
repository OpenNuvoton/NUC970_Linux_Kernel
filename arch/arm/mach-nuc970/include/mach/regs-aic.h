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
#define     REG_AIC_SCR1    (AIC_BA+0x00)    /* Source control register 1 */
#define     REG_AIC_SCR2    (AIC_BA+0x04)    /* Source control register 2 */
#define     REG_AIC_SCR3    (AIC_BA+0x08)    /* Source control register 3 */
#define     REG_AIC_SCR4    (AIC_BA+0x0C)    /* Source control register 4 */
#define     REG_AIC_SCR5    (AIC_BA+0x10)    /* Source control register 5 */
#define     REG_AIC_SCR6    (AIC_BA+0x14)    /* Source control register 6 */
#define     REG_AIC_SCR7    (AIC_BA+0x18)    /* Source control register 7 */
#define     REG_AIC_SCR8    (AIC_BA+0x1C)    /* Source control register 8 */
#define     REG_AIC_SCR9    (AIC_BA+0x20)    /* Source control register 9 */
#define     REG_AIC_SCR10   (AIC_BA+0x24)    /* Source control register 10 */
#define     REG_AIC_SCR11   (AIC_BA+0x28)    /* Source control register 11 */
#define     REG_AIC_SCR12   (AIC_BA+0x2C)    /* Source control register 12 */
#define     REG_AIC_SCR13   (AIC_BA+0x30)    /* Source control register 13 */
#define     REG_AIC_SCR14   (AIC_BA+0x34)    /* Source control register 14 */
#define     REG_AIC_SCR15   (AIC_BA+0x38)    /* Source control register 15 */
#define     REG_AIC_IRSR    (AIC_BA+0x100)   /* Interrupt raw status register */
#define     REG_AIC_IRSRH   (AIC_BA+0x104)   /* Interrupt raw status register (Hign) */
#define     REG_AIC_IASR    (AIC_BA+0x108)   /* Interrupt active status register */
#define     REG_AIC_IASRH   (AIC_BA+0x10C)   /* Interrupt active status register (Hign) */
#define     REG_AIC_ISR     (AIC_BA+0x110)   /* Interrupt status register */
#define     REG_AIC_ISRH    (AIC_BA+0x114)   /* Interrupt status register (High) */
#define     REG_AIC_IPER    (AIC_BA+0x118)   /* Interrupt priority encoding register */
#define     REG_AIC_ISNR    (AIC_BA+0x120)   /* Interrupt source number register */
#define     REG_AIC_OISR    (AIC_BA+0x124)   /* Output interrupt status register */
#define     REG_AIC_IMR     (AIC_BA+0x128)   /* Interrupt mask register */
#define     REG_AIC_IMRH    (AIC_BA+0x12C)   /* Interrupt mask register (High) */
#define     REG_AIC_MECR    (AIC_BA+0x130)   /* Mask enable command register */
#define     REG_AIC_MECRH   (AIC_BA+0x134)   /* Mask enable command register (High) */
#define     REG_AIC_MDCR    (AIC_BA+0x138)   /* Mask disable command register */
#define     REG_AIC_MDCRH   (AIC_BA+0x13C)   /* Mask disable command register (High) */
#define     REG_AIC_SSCR    (AIC_BA+0x140)   /* Source Set Command Register */
#define     REG_AIC_SSCRH   (AIC_BA+0x144)   /* Source Set Command Register (High) */
#define     REG_AIC_SCCR    (AIC_BA+0x148)   /* Source Clear Command Register */
#define     REG_AIC_SCCRH   (AIC_BA+0x14C)   /* Source Clear Command Register (High) */
#define     REG_AIC_EOSCR   (AIC_BA+0x150)   /* End of service command register */

#define AIC_ISNR		(0x120)
#define AIC_IPER		(0x118)

#endif /*  __ASM_ARCH_REGS_AIC_H */
