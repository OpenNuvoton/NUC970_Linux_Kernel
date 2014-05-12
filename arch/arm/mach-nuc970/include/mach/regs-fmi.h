/* linux/include/asm-arm/arch-nuc900/nuc900_reg.h
 *
 * Copyright (c) 2008 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Changelog:
 *
 *   2006/08/26     vincen.zswan add this file for nuvoton nuc900 MCU ip REG.
 */

#ifndef __ASM_ARCH_REGS_FMI_H
#define __ASM_ARCH_REGS_FMI_H

#define FMI_BA   			NUC970_VA_FMI /* Flash Memory Card Interface */

#define REG_NAND_FB0		(FMI_BA+0x000)  /* DMAC Control and Status Register */
#define REG_NAND_DMACCSR	(FMI_BA+0x400)  /* DMAC Control and Status Register */
#define REG_NAND_DMACSAR	(FMI_BA+0x408)  /* DMAC Transfer Starting Address Register */
#define REG_NAND_DMACBCR	(FMI_BA+0x40C)  /* DMAC Transfer Byte Count Register */
#define REG_NAND_DMACIER	(FMI_BA+0x410)  /* DMAC Interrupt Enable Register */
#define REG_NAND_DMACISR	(FMI_BA+0x414)  /* DMAC Interrupt Status Register */

#define REG_NAND_FMICSR		(FMI_BA+0x800)   /* Global Control and Status Register */
#define REG_NAND_FMIIER	    (FMI_BA+0x804)   /* Global Interrupt Control Register */
#define REG_NAND_FMIISR	    (FMI_BA+0x808)   /* Global Interrupt Status Register */

/* eMMC Registers */
#define REG_NAND_SDCSR	    (FMI_BA+0x820)   /* SD control and status register */
#define REG_NAND_SDARG	    (FMI_BA+0x824)   /* SD command argument register */
#define REG_NAND_SDIER		(FMI_BA+0x828)   /* SD interrupt enable register */
#define REG_NAND_SDISR		(FMI_BA+0x82C)   /* SD interrupt status register */
#define REG_NAND_SDRSP0		(FMI_BA+0x830)   /* SD receive response token register 0 */
#define REG_NAND_SDRSP1		(FMI_BA+0x834)   /* SD receive response token register 1 */
#define REG_NAND_SDBLEN		(FMI_BA+0x838)   /* SD block length register */
#define REG_NAND_SDTMOUT	(FMI_BA+0x83C)   /* SD block length register */

/* NAND-type Flash Registers */
#define REG_SMCSR	        (FMI_BA+0x8A0)   /* NAND Flash Control and Status Register */
#define REG_SMTCR	        (FMI_BA+0x8A4)   /* NAND Flash Timing Control Register */
#define REG_SMIER	        (FMI_BA+0x8A8)   /* NAND Flash Interrupt Control Register */
#define REG_SMISR	        (FMI_BA+0x8AC)   /* NAND Flash Interrupt Status Register */
#define REG_SMCMD	        (FMI_BA+0x8B0)   /* NAND Flash Command Port Register */
#define REG_SMADDR	        (FMI_BA+0x8B4)   /* NAND Flash Address Port Register */
#define REG_SMDATA		    (FMI_BA+0x8B8)   /* NAND Flash Data Port Register */
#define REG_SMREACTL        (FMI_BA+0x8BC)   /* NAND Flash Smart-Media Redundant Area Control Register */
#define REG_NFECR           (FMI_BA+0x8C0)   /* NAND Flash Extend Control Regsiter */
#define REG_SMECC_ST0	    (FMI_BA+0x8D0)	 /* Smart-Media ECC Error Status 0 */
#define REG_SMECC_ST1	    (FMI_BA+0x8D4)	 /* Smart-Media ECC Error Status 1 */
#define REG_SMECC_ST2	    (FMI_BA+0x8D8)	 /* Smart-Media ECC Error Status 2 */
#define REG_SMECC_ST3	    (FMI_BA+0x8DC)	 /* Smart-Media ECC Error Status 3 */
#define REG_SMPROT_ADDR0    (FMI_BA+0x8E0)  /* Smart-Media Protect region end address 0 */
#define REG_SMPROT_ADDR1    (FMI_BA+0x8E4)  /* Smart-Media Protect region end address 1 */

/* NAND-type Flash BCH Error Address Registers */
#define REG_BCH_ECC_ADDR0	(FMI_BA+0x900)  /* BCH error byte address 0 */
#define REG_BCH_ECC_ADDR1	(FMI_BA+0x904)  /* BCH error byte address 1 */
#define REG_BCH_ECC_ADDR2	(FMI_BA+0x908)  /* BCH error byte address 2 */
#define REG_BCH_ECC_ADDR3	(FMI_BA+0x90C)  /* BCH error byte address 3 */
#define REG_BCH_ECC_ADDR4	(FMI_BA+0x910)  /* BCH error byte address 4 */
#define REG_BCH_ECC_ADDR5	(FMI_BA+0x914)  /* BCH error byte address 5 */
#define REG_BCH_ECC_ADDR6	(FMI_BA+0x918)  /* BCH error byte address 6 */
#define REG_BCH_ECC_ADDR7	(FMI_BA+0x91C)  /* BCH error byte address 7 */
#define REG_BCH_ECC_ADDR8	(FMI_BA+0x920)  /* BCH error byte address 8 */
#define REG_BCH_ECC_ADDR9	(FMI_BA+0x924)  /* BCH error byte address 9 */
#define REG_BCH_ECC_ADDR10	(FMI_BA+0x928)  /* BCH error byte address 10 */
#define REG_BCH_ECC_ADDR11	(FMI_BA+0x92C)  /* BCH error byte address 11 */

/* NAND-type Flash BCH Error Data Registers */
#define REG_BCH_ECC_DATA0	(FMI_BA+0x960)  /* BCH error byte data 0 */
#define REG_BCH_ECC_DATA1	(FMI_BA+0x964)  /* BCH error byte data 1 */
#define REG_BCH_ECC_DATA2	(FMI_BA+0x968)  /* BCH error byte data 2 */
#define REG_BCH_ECC_DATA3	(FMI_BA+0x96C)  /* BCH error byte data 3 */
#define REG_BCH_ECC_DATA4	(FMI_BA+0x970)  /* BCH error byte data 4 */
#define REG_BCH_ECC_DATA5	(FMI_BA+0x974)  /* BCH error byte data 5 */

/* NAND-type Flash Redundant Area Registers */
#define REG_SMRA0			(FMI_BA+0xA00)  /* Smart-Media Redundant Area Register */
#define REG_SMRA1			(FMI_BA+0xA04)  /* Smart-Media Redundant Area Register */


#endif
