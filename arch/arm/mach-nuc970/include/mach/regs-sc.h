/*
 * arch/arm/mach-nuc970/include/mach/regs-sc.h
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

#ifndef __ASM_ARCH_REGS_SC_H
#define __ASM_ARCH_REGS_SC_H

/* SC Registers */

#define SC0_BA			NUC970_VA_SC
#define SC1_BA			(NUC970_VA_SC + 0x400)
#define REG_SC_DAT		0x00
#define REG_SC_CTL		0x04
#define REG_SC_ALTCTL		0x08
#define REG_SC_EGT		0x0C
#define REG_SC_RXTOUT		0x10
#define REG_SC_ETUCTL		0x14
#define REG_SC_INTEN		0x18
#define REG_SC_INTSTS		0x1C
#define REG_SC_STATUS		0x20
#define REG_SC_PINCTL		0x24
#define REG_SC_TMRCTL0		0x28
#define REG_SC_TMRCTL1		0x2C
#define REG_SC_TMRCTL2		0x30
#define REG_SC_UARTCTL		0x34

#define SC_CTL_NSB		0x00008000

#define SC_ALTCTL_RXRST		0x00000002
#define SC_ALTCTL_TXRST		0x00000001

#define SC_INTEN_RXTOIEN	0x00000200
#define SC_INTEN_TBEIEN		0x00000002
#define SC_INTEN_RDAIEN		0x00000001

#define SC_INTSTS_RXTOIF	0x00000200
#define SC_INTSTS_TBEIF		0x00000002
#define SC_INTSTS_RDAIF		0x00000001

#define SC_STATUS_TXFULL	0x00000400
#define SC_STATUS_TXEMPTY	0x00000200
#define SC_STATUS_BEF		0x00000040
#define SC_STATUS_FEF		0x00000020
#define SC_STATUS_PEF		0x00000010
#define SC_STATUS_RXEMPTY	0x00000002
#define SC_STATUS_RXOV		0x00000001


#endif /*  __ASM_ARCH_REGS_SC_H */
