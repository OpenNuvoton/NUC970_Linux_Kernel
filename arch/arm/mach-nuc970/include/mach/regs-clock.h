/*
 * arch/arm/mach-nuc970/include/mach/regs-clock.h
 *
 * Copyright (c) 2014 Nuvoton technology corporation
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

#ifndef __ASM_ARCH_REGS_CLOCK_H
#define __ASM_ARCH_REGS_CLOCK_H

/* Clock Control Registers  */
#define CLK_BA		NUC970_VA_CLK

#define REG_CLK_PMCON		(CLK_BA+0x000)  /*  Power Management Control Register */
#define REG_CLK_HCLKEN		(CLK_BA+0x010)  /*  AHB IP Clock Enable Control Register */
#define REG_CLK_PCLKEN0		(CLK_BA+0x018)  /*  APB IP Clock Enable Control Register 0 */
#define REG_CLK_PCLKEN1		(CLK_BA+0x01C)  /*  APB IP Clock Enable Control Register 1 */
#define REG_CLK_DIV0		(CLK_BA+0x020)  /*  Clock Divider Control Register 0 */
#define REG_CLK_DIV1		(CLK_BA+0x024)  /*  Clock Divider Control Register 1 */
#define REG_CLK_DIV2		(CLK_BA+0x028)  /*  Clock Divider Control Register 2 */
#define REG_CLK_DIV3		(CLK_BA+0x02C)  /*  Clock Divider Control Register 3 */
#define REG_CLK_DIV4		(CLK_BA+0x030)  /*  Clock Divider Control Register 4 */
#define REG_CLK_DIV5		(CLK_BA+0x034)  /*  Clock Divider Control Register 5 */
#define REG_CLK_DIV6		(CLK_BA+0x038)  /*  Clock Divider Control Register 6 */
#define REG_CLK_DIV7		(CLK_BA+0x03C)  /*  Clock Divider Control Register 7 */
#define REG_CLK_DIV8		(CLK_BA+0x040)  /*  Clock Divider Control Register 8 */
#define REG_CLK_DIV9		(CLK_BA+0x044)  /*  Clock Divider Control Register 9 */
#define REG_CLK_APLLCON		(CLK_BA+0x060)  /*  APLL Control Register */
#define REG_CLK_UPLLCON		(CLK_BA+0x064)  /*  UPLL Control Register */

#endif /*  __ASM_ARCH_REGS_CLOCK_H */
