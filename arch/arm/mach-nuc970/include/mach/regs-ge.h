/*
 * arch/arm/mach-nuc970/include/mach/regs-ge.h
 *
 * Copyright (c) 2013 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#ifndef __ASM_ARCH_REGS_GE_H
#define __ASM_ARCH_REGS_GE_H

/* Clock Control Registers  */
#define GE_BA		NUC970_VA_GE

#define REG_GE2D_TRG		(GE_BA+0x000)  	/* Graphic Engine Trigger Control Register       */
#define REG_GE2D_XYSORG		(GE_BA+0x004)  	/* Graphic Engine XY Mode Source Origin Starting Register */
#define REG_GE2D_TCNTVHSF	(GE_BA+0x008)  	/* Graphic Engine Tile Width/Height or V/H Scale Factor N/M */
#define REG_GE2D_XYRRP	    (GE_BA+0x00C)  	/* Graphic Engine Rotate Reference Point XY Address       */
#define REG_GE2D_INTSTS		(GE_BA+0x010)  	/* Graphic Engine Interrupt Status Register      */
#define REG_GE2D_PATSA		(GE_BA+0x014)  	/* Graphic Engine Pattern Location Starting Address Register  */
#define REG_GE2D_BETSC		(GE_BA+0x018)  	/* GE Bresenham Error Term Stepping Constant Register */
#define REG_GE2D_BIEPC		(GE_BA+0x01C)  	/* GE Bresenham Initial Error, Pixel Count Major M Register */
#define REG_GE2D_CTL      	(GE_BA+0x020)  	/* Graphic Engine Control Register               */
#define REG_GE2D_BGCOLR		(GE_BA+0x024)  	/* Graphic Engine Background Color Register      */
#define REG_GE2D_FGCOLR		(GE_BA+0x028)  	/* Graphic Engine Foreground Color Register      */
#define REG_GE2D_TRNSCOLR	(GE_BA+0x02C)  	/* Graphic Engine Transparency Color Register    */
#define REG_GE2D_TCMSK		(GE_BA+0x030)  	/* Graphic Engine Transparency Color Mask Register */
#define REG_GE2D_XYDORG		(GE_BA+0x034)  	/* Graphic Engine XY Mode Display Origin Starting Register */
#define REG_GE2D_SDPITCH	(GE_BA+0x038)  	/* Graphic Engine Source/Destination Pitch Register */
#define REG_GE2D_SRCSPA		(GE_BA+0x03C)  	/* Graphic Engine Source Start XY/Linear Address Register */
#define REG_GE2D_DSTSPA		(GE_BA+0x040)  	/* Graphic Engine Destination Start XY/Linear Register  */
#define REG_GE2D_RTGLSZ		(GE_BA+0x044)  	/* Graphic Engine Dimension XY/Linear Register   */
#define REG_GE2D_CLPBTL		(GE_BA+0x048)  	/* Graphic Engine Clipping Boundary Top/Left Register */
#define REG_GE2D_CLPBBR		(GE_BA+0x04C)  	/* Graphic Engine Clipping Boundary Bottom/Right Register */
#define REG_GE2D_PTNA 		(GE_BA+0x050)  	/* Graphic Engine Pattern A Register             */
#define REG_GE2D_PTN  B		(GE_BA+0x054)  	/* Graphic Engine Pattern B Register             */
#define REG_GE2D_WRPLNMSK	(GE_BA+0x058)  	/* Graphic Engine Write Plane Mask Register      */
#define REG_GE2D_MISCTL   	(GE_BA+0x05C)  	/* Graphic Engine Miscellaneous Control Register */
#define REG_GE2D_HSTBLTDP0	(GE_BA+0x060)  	/* Graphic Engine HostBLT Double Word Data Port0 Register */
#define REG_GE2D_HSTBLTDP1	(GE_BA+0x064)  	/* Graphic Engine HostBLT Double Word Data Port1 Register */
#define REG_GE2D_HSTBLTDP2	(GE_BA+0x068)  	/* Graphic Engine HostBLT Double Word Data Port2 Register */
#define REG_GE2D_HSTBLTDP3	(GE_BA+0x06C)  	/* Graphic Engine HostBLT Double Word Data Port3 Register */
#define REG_GE2D_HSTBLTDP4	(GE_BA+0x070)  	/* Graphic Engine HostBLT Double Word Data Port4 Register */
#define REG_GE2D_HSTBLTDP5	(GE_BA+0x074)  	/* Graphic Engine HostBLT Double Word Data Port5 Register */
#define REG_GE2D_HSTBLTDP6	(GE_BA+0x078)  	/* Graphic Engine HostBLT Double Word Data Port6 Register */
#define REG_GE2D_HSTBLTDP7	(GE_BA+0x07C)  	/* Graphic Engine HostBLT Double Word Data Port7 Register */


#endif /*  __ASM_ARCH_REGS_GE_H */
