/*
 * arch/arm/mach-nuc970/include/mach/regs-gdma.h
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

#ifndef __ASM_ARCH_REGS_GDMA_H
#define __ASM_ARCH_REGS_GDMA_H

/* GDMA Registers */

#define		 	GDMA_BA							NUC970_VA_GDMA

#define     GDMA_CTL				(0x000)  /* Channel 0/1 Control Register */
#define     GDMA_SRCB				(0x004)  /* Channel 0/1 Source Base Address Register */
#define     GDMA_DSTB				(0x008)  /* Channel 0/1 Destination Base Address Register */
#define     GDMA_TCNT				(0x00C)  /* Channel 0/1 Transfer Count Register */
#define     GDMA_CSRC				(0x010)  /* Channel 0/1 Current Source Address Register */
#define     GDMA_CDST				(0x014)  /* Channel 0/1 Current Destination Address Register */
#define     GDMA_CTCNT			(0x018)  /* Channel 0/1 Current Transfer Count Register */
#define     GDMA_DADR				(0x01C)  /* Channel 0/1 Descriptor Address Register */

#define     REG_GDMA_CTL0				(GDMA_BA+0x000)  /* Channel 0 Control Register */
#define     REG_GDMA_SRCB0			(GDMA_BA+0x004)  /* Channel 0 Source Base Address Register */
#define     REG_GDMA_DSTB0			(GDMA_BA+0x008)  /* Channel 0 Destination Base Address Register */
#define     REG_GDMA_TCNT0			(GDMA_BA+0x00C)  /* Channel 0 Transfer Count Register */
#define     REG_GDMA_CSRC0			(GDMA_BA+0x010)  /* Channel 0 Current Source Address Register */
#define     REG_GDMA_CDST0			(GDMA_BA+0x014)  /* Channel 0 Current Destination Address Register */
#define     REG_GDMA_CTCNT0			(GDMA_BA+0x018)  /* Channel 0 Current Transfer Count Register */
#define     REG_GDMA_DADR0			(GDMA_BA+0x01C)  /* Channel 0 Descriptor Address Register */

#define     REG_GDMA_CTL1				(GDMA_BA+0x020)  /* Channel 1 Control Register */
#define     REG_GDMA_SRCB1			(GDMA_BA+0x024)  /* Channel 1 Source Base Address Register */
#define     REG_GDMA_DSTB1			(GDMA_BA+0x028)  /* Channel 1 Destination Base Address Register */
#define     REG_GDMA_TCNT1			(GDMA_BA+0x02C)  /* Channel 1 Transfer Count Register */
#define     REG_GDMA_CSRC1			(GDMA_BA+0x030)  /* Channel 1 Current Source Address Register */
#define     REG_GDMA_CDST1			(GDMA_BA+0x034)  /* Channel 1 Current Destination Address Register */
#define     REG_GDMA_CTCNT1			(GDMA_BA+0x038)  /* Channel 1 Current Transfer Count Register */
#define     REG_GDMA_DADR1			(GDMA_BA+0x03C)  /* Channel 1 Descriptor Address Register */

#define     REG_GDMA_INTBUF0    (GDMA_BA+0x080)  /* GDMA Internal Buffer Word 0 */
#define     REG_GDMA_INTBUF1    (GDMA_BA+0x084)  /* GDMA Internal Buffer Word 1 */
#define     REG_GDMA_INTBUF2    (GDMA_BA+0x088)  /* GDMA Internal Buffer Word 2 */
#define     REG_GDMA_INTBUF3    (GDMA_BA+0x08C)  /* GDMA Internal Buffer Word 3 */
#define     REG_GDMA_INTBUF4    (GDMA_BA+0x090)  /* GDMA Internal Buffer Word 4 */
#define     REG_GDMA_INTBUF5    (GDMA_BA+0x094)  /* GDMA Internal Buffer Word 5 */
#define     REG_GDMA_INTBUF6    (GDMA_BA+0x098)  /* GDMA Internal Buffer Word 6 */
#define     REG_GDMA_INTBUF7    (GDMA_BA+0x09C)  /* GDMA Internal Buffer Word 7 */
#define     REG_GDMA_INTCS			(GDMA_BA+0x0A0)  /* Interrupt Control and Status Register */


#define TWS_Msk			(0x3<<12)  //GDMA_CTL : Transfer Width Select
#define TWS_8BIT		(0x0<<12)
#define TWS_16BIT		(0x1<<12)
#define TWS_32BIT		(0x2<<12)
#define SOFTREQ		(0x1<<16) 

#define BME			(0x1<<1)   //GDMA_CTL : Burst Mode Enable

#define TERR1EN 	(0x1<<3)
#define TC1EN			(0x1<<2)
#define TERR0EN 	(0x1<<1)
#define TC0EN			(0x1)

#endif /*  __ASM_ARCH_REGS_GDMA_H */