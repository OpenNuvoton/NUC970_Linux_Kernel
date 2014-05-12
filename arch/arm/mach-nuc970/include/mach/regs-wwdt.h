/*
 * arch/arm/mach-nuc970/include/mach/regs-wwdt.h
 *
 * Copyright (c) 2014 Nuvoton Technology Corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __ASM_ARCH_REGS_WWDT_H
#define __ASM_ARCH_REGS_WWDT_H

/* WWDT Registers */

#define WWDT_BA			NUC970_VA_WWDT
#define REG_WWDT_RLD		(WWDT_BA+0x00)
#define REG_WWDT_CR		(WWDT_BA+0x04)
#define REG_WWDT_SR		(WWDT_BA+0x08)
#define REG_WWDT_CVR		(WWDT_BA+0x0C)


#endif /*  __ASM_ARCH_REGS_WWDT_H */
