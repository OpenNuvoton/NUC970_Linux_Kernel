/*
 * arch/arm/mach-nuc970/include/mach/regs-gcr.h
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

#ifndef __ASM_ARCH_REGS_GCR_H
#define __ASM_ARCH_REGS_GCR_H

/* Global control registers */

#define GCR_BA		NUC970_VA_GCR

#define REG_PDID     	(GCR_BA+0x000)  /* Product Identifier Register */
#define REG_PWRON	(GCR_BA+0x004)  /* Power-On Setting Register */
#define REG_ARBCON	(GCR_BA+0x008)  /* Arbitration Control Register */
#define REG_LVRDCR 	(GCR_BA+0x020)  /* Low Voltage Reset & Detect Control Register */
#define REG_MISCFCR  	(GCR_BA+0x030)  /* Miscellaneous Function Control Register */
#define REG_MISCIER 	(GCR_BA+0x040)  /* Miscellaneous Interrupt Enable Register */
#define REG_MISCISR 	(GCR_BA+0x044)  /* Miscellaneous Interrupt Status Register */
#define REG_ROMSUM0 	(GCR_BA+0x048)  /* Internal ROM BIST Checksum Register 0 */
#define REG_ROMSUM1 	(GCR_BA+0x04C)  /* Internal ROM BIST Checksum Register 1 */
#define REG_WKUPSER	(GCR_BA+0x058)  /* System Wakeup Source Enable Register */
#define REG_WKUPSSR	(GCR_BA+0x05C)  /* System Wakeup Source Status Register */
#define REG_AHBIPRST	(GCR_BA+0x060)  /* AHB IP Reset Control Register */
#define REG_APBIPRST0	(GCR_BA+0x064)  /* APB IP Reset Control Register 0 */
#define REG_APBIPRST1	(GCR_BA+0x068)  /* APB IP Reset Control Register 1 */
#define REG_RSTSTS	(GCR_BA+0x06C)  /* Reset Source Active Status Register */

#define REG_MFP_GPA_L	(GCR_BA+0x070)  /* GPIOA Low Byte Multiple Function Control Register */
#define REG_MFP_GPA_H	(GCR_BA+0x074)  /* GPIOA High Byte Multiple Function Control Register */
#define REG_MFP_GPB_L	(GCR_BA+0x078)  /* GPIOB Low Byte Multiple Function Control Register */
#define REG_MFP_GPB_H	(GCR_BA+0x07C)  /* GPIOB High Byte Multiple Function Control Register */
#define REG_MFP_GPC_L	(GCR_BA+0x080)  /* GPIOC Low Byte Multiple Function Control Register */
#define REG_MFP_GPC_H	(GCR_BA+0x084)  /* GPIOC High Byte Multiple Function Control Register */
#define REG_MFP_GPD_L	(GCR_BA+0x088)  /* GPIOD Low Byte Multiple Function Control Register */
#define REG_MFP_GPD_H	(GCR_BA+0x08C)  /* GPIOD High Byte Multiple Function Control Register */
#define REG_MFP_GPE_L	(GCR_BA+0x090)  /* GPIOE Low Byte Multiple Function Control Register */
#define REG_MFP_GPE_H	(GCR_BA+0x094)  /* GPIOE High Byte Multiple Function Control Register */
#define REG_MFP_GPF_L	(GCR_BA+0x098)  /* GPIOF Low Byte Multiple Function Control Register */
#define REG_MFP_GPF_H	(GCR_BA+0x09C)  /* GPIOF High Byte Multiple Function Control Register */
#define REG_MFP_GPG_L	(GCR_BA+0x0A0)  /* GPIOG Low Byte Multiple Function Control Register */
#define REG_MFP_GPG_H	(GCR_BA+0x0A4)  /* GPIOG High Byte Multiple Function Control Register */
#define REG_MFP_GPH_L	(GCR_BA+0x0A8)  /* GPIOH Low Byte Multiple Function Control Register */
#define REG_MFP_GPH_H	(GCR_BA+0x0AC)  /* GPIOH High Byte Multiple Function Control Register */
#define REG_MFP_GPI_L	(GCR_BA+0x0B0)  /* GPIOI Low Byte Multiple Function Control Register */
#define REG_MFP_GPI_H	(GCR_BA+0x0B4)  /* GPIOI High Byte Multiple Function Control Register */
#define REG_MFP_GPJ_L	(GCR_BA+0x0B8)  /* GPIOJ Low Byte Multiple Function Control Register */
#define REG_DDR_DS_CR	(GCR_BA+0x0E0)  /* DDR I/O Driving Strength Control Register */
#define REG_PORDISCR    (GCR_BA+0x100)  /* Power-On-Reset Disable Control Register */
#define REG_ICEDBGCR    (GCR_BA+0x104)  /* ICE Debug Interface Control Register */
#define REG_WRPRTR	(GCR_BA+0x1FC)  /* Register Write-Protection Control Register */


#endif /*  __ASM_ARCH_REGS_GCR_H */
