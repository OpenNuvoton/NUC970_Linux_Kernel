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

#define REG_2D_GETG			(GE_BA+0x000)  	/* Graphic Engine Trigger Control Register       */
#define REG_2D_GEXYSORG		(GE_BA+0x004)  	/* Graphic Engine XY Mode Source Origin Starting Register */
#define REG_2D_TileXY_VHSF	(GE_BA+0x008)  	/* Graphic Engine Tile Width/Height or V/H Scale Factor N/M */
#define REG_2D_GERRXY		(GE_BA+0x00C)  	/* Graphic Engine Rotate Reference Point XY Address       */
#define REG_2D_GEINTS		(GE_BA+0x010)  	/* Graphic Engine Interrupt Status Register      */
#define REG_2D_GEPLS		(GE_BA+0x014)  	/* Graphic Engine Pattern Location Starting Address Register  */
#define REG_2D_GEBER		(GE_BA+0x018)  	/* GE Bresenham Error Term Stepping Constant Register */
#define REG_2D_GEBIR		(GE_BA+0x01C)  	/* GE Bresenham Initial Error, Pixel Count Major M Register */
#define REG_2D_GEC			(GE_BA+0x020)  	/* Graphic Engine Control Register               */
#define REG_2D_GEBC			(GE_BA+0x024)  	/* Graphic Engine Background Color Register      */
#define REG_2D_GEFC			(GE_BA+0x028)  	/* Graphic Engine Foreground Color Register      */
#define REG_2D_GETC			(GE_BA+0x02C)  	/* Graphic Engine Transparency Color Register    */
#define REG_2D_GETCM		(GE_BA+0x030)  	/* Graphic Engine Transparency Color Mask Register */
#define REG_2D_GEXYDORG		(GE_BA+0x034)  	/* Graphic Engine XY Mode Display Origin Starting Register */
#define REG_2D_GESDP		(GE_BA+0x038)  	/* Graphic Engine Source/Destination Pitch Register */
#define REG_2D_GESSXYL		(GE_BA+0x03C)  	/* Graphic Engine Source Start XY/Linear Address Register */
#define REG_2D_GEDSXYL		(GE_BA+0x040)  	/* Graphic Engine Destination Start XY/Linear Register  */
#define REG_2D_GEDIXYL		(GE_BA+0x044)  	/* Graphic Engine Dimension XY/Linear Register   */
#define REG_2D_GECBTL		(GE_BA+0x048)  	/* Graphic Engine Clipping Boundary Top/Left Register */
#define REG_2D_GECBBR		(GE_BA+0x04C)  	/* Graphic Engine Clipping Boundary Bottom/Right Register */
#define REG_2D_GEPTNA		(GE_BA+0x050)  	/* Graphic Engine Pattern A Register             */
#define REG_2D_GEPTNB		(GE_BA+0x054)  	/* Graphic Engine Pattern B Register             */
#define REG_2D_GEWPM		(GE_BA+0x058)  	/* Graphic Engine Write Plane Mask Register      */
#define REG_2D_GEMC			(GE_BA+0x05C)  	/* Graphic Engine Miscellaneous Control Register */
#define REG_2D_GEHBDW0		(GE_BA+0x060)  	/* Graphic Engine HostBLT Double Word Data Port0 Register */
#define REG_2D_GEHBDW1		(GE_BA+0x064)  	/* Graphic Engine HostBLT Double Word Data Port1 Register */
#define REG_2D_GEHBDW2		(GE_BA+0x068)  	/* Graphic Engine HostBLT Double Word Data Port2 Register */
#define REG_2D_GEHBDW3		(GE_BA+0x06C)  	/* Graphic Engine HostBLT Double Word Data Port3 Register */
#define REG_2D_GEHBDW4		(GE_BA+0x070)  	/* Graphic Engine HostBLT Double Word Data Port4 Register */
#define REG_2D_GEHBDW5		(GE_BA+0x074)  	/* Graphic Engine HostBLT Double Word Data Port5 Register */
#define REG_2D_GEHBDW6		(GE_BA+0x078)  	/* Graphic Engine HostBLT Double Word Data Port6 Register */
#define REG_2D_GEHBDW7		(GE_BA+0x07C)  	/* Graphic Engine HostBLT Double Word Data Port7 Register */


#endif /*  __ASM_ARCH_REGS_GE_H */
