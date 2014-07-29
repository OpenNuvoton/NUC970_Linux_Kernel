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

#ifndef __ASM_ARCH_REGS_SDH_H
#define __ASM_ARCH_REGS_SDH_H

#define SDH_BA              NUC970_VA_SDH /* SD/SDIO Host Controller */

/* DMAC Control Registers*/
#define REG_SDH_FB0         (SDH_BA+0x000)  /* DMAC Control and Status Register */
#define REG_DMACCSR         (SDH_BA+0x400)  /* DMAC Control and Status Register */
#define REG_DMACSAR2        (SDH_BA+0x408)  /* DMAC Transfer Starting Address Register */
#define REG_DMACBCR         (SDH_BA+0x40C)  /* DMAC Transfer Byte Count Register */
#define REG_DMACIER         (SDH_BA+0x410)  /* DMAC Interrupt Enable Register */
#define REG_DMACISR         (SDH_BA+0x414)  /* DMAC Interrupt Status Register */

#define REG_FMICSR          (SDH_BA+0x800)  /* Global Control and Status Register */
#define REG_FMIIER          (SDH_BA+0x804)  /* Global Interrupt Control Register */
#define REG_FMIISR          (SDH_BA+0x808)  /* Global Interrupt Status Register */

/* Secure Digit Registers */
#define REG_SDCSR           (SDH_BA+0x820)  /* SD control and status register */
#define REG_SDARG           (SDH_BA+0x824)  /* SD command argument register */
#define REG_SDIER           (SDH_BA+0x828)  /* SD interrupt enable register */
#define REG_SDISR           (SDH_BA+0x82C)  /* SD interrupt status register */
#define REG_SDRSP0          (SDH_BA+0x830)  /* SD receive response token register 0 */
#define REG_SDRSP1          (SDH_BA+0x834)  /* SD receive response token register 1 */
#define REG_SDBLEN          (SDH_BA+0x838)  /* SD block length register */
#define REG_SDTMOUT         (SDH_BA+0x83C)  /* SD block length register */
#define REG_SDECR           (SDH_BA+0x840)  /* SD extended control register */


/* Flash buffer 0 registers */
#define FB0_BASE_ADDR       (SDH_BA+0x000)
#define FB0_SIZE            0x80

#define DMA_BLOCK_SIZE      0x200
#define SD_BLOCK_SIZE       0x200

/* FMI Global Control and Status Register(FMICSR) */
#define FMICSR_SW_RST       (1)
#define FMICSR_SD_EN        (1<<1)

/* FMI Global Interrupt Control Register(FMIIER) */
#define FMIIER_DTA_IE       (1)

/* FMI Global Interrupt Status Register (FMIISR) */
#define FMIISR_DTA_IF       (1)

/* SD Control and Status Register (SDCSR) */
#define SDCSR_CO_EN         (1)
#define SDCSR_RI_EN         (1<<1)
#define SDCSR_DI_EN         (1<<2)
#define SDCSR_DO_EN         (1<<3)
#define SDCSR_R2_EN         (1<<4)
#define SDCSR_CLK74_OE      (1<<5)
#define SDCSR_CLK8_OE       (1<<6)
#define SDCSR_CLK_KEEP0     (1<<7)
#define SDCSR_SW_RST        (1<<14)
#define SDCSR_DBW           (1<<15)
#define SDCSR_CLK_KEEP1     (1<<31)

/* SD Interrupt Control Register (SDIER) */
#define SDIER_BLKD_IE       (1)
#define SDIER_CRC_IE        (1<<1)
#define SDIER_CD0_IE        (1<<8)
#define SDIER_CD1_IE        (1<<9)
#define SDIER_SDIO0_IE      (1<<10)
#define SDIER_SDIO1_IE      (1<<11)
#define SDIER_RITO_IE       (1<<12)
#define SDIER_DITO_IE       (1<<13)
#define SDIER_WKUP_EN       (1<<14)
#define SDIER_CD0SRC        (1<<30)
#define SDIER_CD1SRC        (1<<31)

/* SD Interrupt Status Register (SDISR) */
#define SDISR_BLKD_IF       (1)
#define SDISR_CRC_IF        (1<<1)
#define SDISR_CRC_7         (1<<2)
#define SDISR_CRC_16        (1<<3)
#define SDISR_SDDAT0        (1<<7)
#define SDISR_CD0_IF        (1<<8)
#define SDISR_CD1_IF        (1<<9)
#define SDISR_SDIO0_IF      (1<<10)
#define SDISR_SDIO1_IF      (1<<11)
#define SDISR_RITO_IF       (1<<12)
#define SDISR_DITO_IF       (1<<13)
#define SDISR_CDPS0         (1<<16)
#define SDISR_CDPS1         (1<<17)
#define SDISR_SD0DAT1       (1<<18)
#define SDISR_SD1DAT1       (1<<19)

/* DMAC Control and Status Register (DMACCSR) */
#define DMACCSR_DMACEN      (1)
#define DMACCSR_SW_RST      (1<<1)
#define DMACCSR_SG_EN1      (1<<2)
#define DMACCSR_SG_EN2      (1<<3)
#define DMACCSR_ATA_BUSY    (1<<8)
#define DMACCSR_FMI_BUSY    (1<<9)

/* DMAC Interrupt Enable Register (DMACIER) */
#define DMACIER_TABORT_IE   (1)
#define DMACIER_WEOT_IE     (1<<1)

/* DMAC Interrupt Status Register (DMACISR) */
#define DMACISR_TABORT_IF   (1)
#define DMACISR_WEOT_IF     (1<<1)

/* DMAC BIST Control and Status Register (DMACBIST) */
#define DMACBIST_BIST_EN    (1)
#define DMACBIST_FINISH     (1<<1)
#define DMACBIST_FAILED     (1<<2)


#endif
