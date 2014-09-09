/*
 * arch/arm/mach-nuc970/include/mach/regs-vcap.h
 *
 * Copyright (c) 2012 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#ifndef __ASM_ARCH_REGS_CAP_H
#define __ASM_ARCH_REGS_CAP_H

#define BIT0	0x00000001
#define BIT1	0x00000002
#define BIT2	0x00000004
#define BIT3	0x00000008
#define BIT4	0x00000010
#define BIT5	0x00000020
#define BIT6	0x00000040
#define BIT7	0x00000080
#define BIT8	0x00000100
#define BIT9	0x00000200
#define BIT10	0x00000400
#define BIT11	0x00000800
#define BIT12	0x00001000
#define BIT13	0x00002000
#define BIT14	0x00004000
#define BIT15	0x00008000
#define BIT16	0x00010000
#define BIT17	0x00020000
#define BIT18	0x00040000
#define BIT19	0x00080000
#define BIT20	0x00100000
#define BIT21	0x00200000
#define BIT22	0x00400000
#define BIT23	0x00800000
#define BIT24	0x01000000
#define BIT25	0x02000000
#define BIT26	0x04000000
#define BIT27	0x08000000
#define BIT28	0x10000000
#define BIT29	0x20000000
#define BIT30	0x40000000
#define BIT31	0x80000000

#include <mach/map.h>
 
#define VIDEOIN_BASE	NUC970_VA_CAP	/* Videoin Control */

#define CAP_BA	NUC970_VA_CAP	/* Videoin Control */
/*
 VideoIn Control Registers
*/
#define REG_CAP_CTL					(CAP_BA + 0x00)	/* Image Capture Interface Control Register */	
#define REG_CAP_PAR					(CAP_BA + 0x04)	/* Image Capture Interface Parameter Register */
#define REG_CAP_INT					(CAP_BA + 0x08)	/* Image Capture Interface Interrupt Register */
#define REG_CAP_POSTERIZE		(CAP_BA + 0x0C)	/* YUV Component Posterizing Factor Register */
#define REG_CAP_MD					(CAP_BA + 0x10)	/* Motion Detection Register */
#define REG_CAP_MDADDR			(CAP_BA + 0x14)	/* Motion Detection Output Address Register */
#define REG_CAP_MDYADDR			(CAP_BA + 0x18)	/* Motion Detection Temp Y Output Address Register */
#define REG_CAP_SEPIA				(CAP_BA + 0X1C)	/* Sepia Effect Control Register */
#define REG_CAP_CWSP        (CAP_BA + 0X20)	/* Cropping Window Starting Address Register */
#define REG_CAP_CWS         (CAP_BA + 0X24)	/* Cropping Window Size Register */
#define REG_CAP_PKTSL       (CAP_BA + 0X28)	/* Packet Scaling Vertical/Horizontal Factor Register (LSB) */
#define REG_CAP_PLNSL       (CAP_BA + 0X2C)	/* Planar Scaling Vertical/Horizontal Factor Register (LSB) */
#define REG_CAP_FRCTL       (CAP_BA + 0X30)	/* Scaling Frame Rate Factor Register */
#define REG_CAP_STRIDE      (CAP_BA + 0X34)	/* Frame Output Pixel Stride Width Register */
#define REG_CAP_FIFOTH      (CAP_BA + 0X3C)	/* FIFO Threshold Register */
#define REG_CAP_CMPADDR     (CAP_BA + 0X40)	/* Compare Memory Base Address Register */
#define REG_CAP_PKTSM       (CAP_BA + 0X48)	/* Packet Scaling Vertical/Horizontal Factor Register (MSB) */
#define REG_CAP_PLNSM       (CAP_BA + 0X4C)	/* Planar Scaling Vertical/Horizontal Factor Register (MSB) */
#define REG_CAP_CURADDRP    (CAP_BA + 0X50)	/* Current Packet System Memory Address Register */
#define REG_CAP_CURADDRY    (CAP_BA + 0X54)	/* Current Planar Y System Memory Address Register */
#define REG_CAP_CURADDRU    (CAP_BA + 0X58)	/* Current Planar U System Memory Address Register */
#define REG_CAP_CURADDRV    (CAP_BA + 0X5C)	/* Current Planar V System Memory Address Register */
#define REG_CAP_PKTBA0      (CAP_BA + 0X60)	/* System Memory Packet Base Address 0 Register */
#define REG_CAP_PKTBA1			(CAP_BA + 0X64)	/* System Memory Packet Base Address 1 Register */
#define REG_CAP_YBA					(CAP_BA + 0X80)	/* System Memory Planar Y Base Address Register */
#define REG_CAP_UBA					(CAP_BA + 0X84)	/* System Memory Planar U Base Address Register */
#define REG_CAP_VBA					(CAP_BA + 0X88)	/* System Memory Planar V Base Address Register */    

#define CAP_CTL_CAPEN 	(1<<0 )
#define CAP_CTL_PLNEN		(1<<5 )
#define CAP_CTL_PKTEN		(1<<6 )
#define CAP_CTL_UPDATE	(1<<20)

#define CAP_CWSP_CWSADDRV (0x3FF<<16)
#define CAP_CWSP_CWSADDRH	(0x3FF<<0 )  

#define CAP_PKTSL_PKTSVNL (0xFF<<24)
#define CAP_PKTSL_PKTSVML (0xFF<<16)
#define CAP_PKTSL_PKTSHNL (0xFF<<8 )
#define CAP_PKTSL_PKTSHML (0xFF<<0 )

#define CAP_PKTSM_PKTSVNH (0xFF<<24)
#define CAP_PKTSM_PKTSVMH (0xFF<<16)
#define CAP_PKTSM_PKTSHNH (0xFF<<8 )
#define CAP_PKTSM_PKTSHMH (0xFF<<0 )

#define CAP_STRIDE_PLNSTRIDE	(0x3FFF<<16)
#define CAP_STRIDE_PKTSTRIDE	(0x3FFF<<0 )


#define CAP_PLNSL_PLNSVNL (0xFF<<24)
#define CAP_PLNSL_PLNSVML (0xFF<<16)
#define CAP_PLNSL_PLNSHNL (0xFF<<8 )
#define CAP_PLNSL_PLNSHML (0xFF<<0 )

#define CAP_PLNSM_PLNSVNH (0xFF<<24)
#define CAP_PLNSM_PLNSVMH (0xFF<<16)
#define CAP_PLNSM_PLNSHNH (0xFF<<8 )
#define CAP_PLNSM_PLNSHMH (0xFF<<0 )


#if 0
#define REG_VPECTL  		(VIDEOIN_BASE + 0x00)	// R/W: Video Pre-processor Control Register
			#define VPRST		BIT24										// Video Pre-processor Reset.
			#define UPDATE 		BIT20									// Video-In Update Register at New Frame	
			#define CAPONE		BIT16									// Video-In One Shutter
			#define VPRBIST		BIT8									// Video-In One Shutter
			#define PKEN		BIT6										// Packet Output Enable
			#define PNEN		BIT5										// Planar Output Enable
			#define ADDRSW		BIT3									// Packet Buffer Address select
			#define FBMODE		BIT2									// Packet Frame Buffer Control by FSC
			#define VPEEN		BIT0										// Planar Output Enable	

#define REG_VPEPAR		(VIDEOIN_BASE + 0x04)		// R/W: Video Pre-processor Parameter Register
			#define VPEBFIN		BIT28									// BIST Finish [Read Only]
			#define BFAIL		NVTBIT(27, 24)					// BIST Fail Flag [Read Only]
			#define FLDID		BIT20										// Field ID [Read Only]
			#define FLD1EN		BIT17									// Field 1 Input Enable
			#define FLD0EN		BIT16					// Field 0 Input Enable
			#define FLDDETP		BIT15					// Field Detect Position
			#define FLDDETM 	BIT14					// Field Detect Mode (By HSYNC or input FIELD PIN)
			#define FLDSWAP	BIT13					// Swap Input Field
			#define VSP			BIT10					// Sensor Vsync Polarity.
			#define HSP			BIT9					// Sensor Hsync Polarity
			#define PCLKP		BIT8					// Sensor Pixel Clock Polarity	
			#define PNFMT		BIT7					// Planar Output Format
			#define RANGE		BIT6					// Scale Input YUV CCIR601 color range to full range
			#define OUTFMT 	NVTBIT(5, 4)				// Image Data Format Output to System Memory.
			#define PDORD		NVTBIT(3, 2)				// Sensor Output Type
			#define SNRTYPE 	BIT1					// device is CCIR601 or CCIR656
			#define INFMT 		BIT0					// Sensor Output Format


#define REG_VPEINT  		(VIDEOIN_BASE + 0x08)	// R/W: Video Pre-processor Interrupt  Register
			#define MDINTEN	BIT20					// Motion Detection Interrupt Enable
			#define ADDRMEN  	BIT19					// Address Match Interrupt Enable.	
			#define MEINTEN	BIT17					// System Memory Error Interrupt Enable.
			#define VINTEN		BIT16					// Video Frame End Interrupt Enable.
			#define MDINT		BIT4					// Motion Detection Output Finsish Interrupt		
			#define ADDRMINT	BIT3					// Memory Address Match Interrupt Flag.
			#define MEINT		BIT1					// System Memory Error Interrupt. If read this bit shows 1, 
														// Memory Error occurs. Write 0 to clear it.
			#define VINT		BIT0					// Video Frame End Interrupt. If read this bit shows 1, 
														// received a frame complete. Write 0 to clear it.


#define REG_VPEMD  		(VIDEOIN_BASE + 0x10)	// R/W: Motion Detection  Register
			#define MDTHR	  	NVTBIT(20, 16)			// MD Differential Threshold	
			#define MDDF		NVTBIT(11, 10)			// MD Detect Frequence
			#define MDSM		BIT9					// MD Save Mode
			#define MDBS		BIT8					// MD Block Size
			#define MDEN		BIT0					// MD Enable
			
#define REG_MDADDR  		(VIDEOIN_BASE + 0x14)	// R/W: Motion Detection Output Address Register
#define REG_MDYADDR  	(VIDEOIN_BASE + 0x18)	// R/W: Motion Detection Output Address Register

#define REG_VPECWSP  	(VIDEOIN_BASE + 0x20)	// R/W:  Cropping Window Starting Address Register
			#define CWSPV		NVTBIT(26, 16)			// Cropping Window Vertical Starting Address
			#define CWSPH		NVTBIT(11, 0)			// Cropping Window Horizontal  Starting Address

#define REG_VPECWS	 	(VIDEOIN_BASE + 0x24)	// R/W:  Cropping Window Size Register
			#define CWSH			NVTBIT(26, 16)		// Cropping Image Window Height
			#define CWSW		NVTBIT(11, 0)			// Cropping Image Window Width

#define REG_VPEPKDS  		(VIDEOIN_BASE + 0x28)	// R/W  : Packet Scaling Vertical/Horizontal Factor Register
#define REG_VPEPNDS 		(VIDEOIN_BASE + 0x2C)	// R?W  : Planar Scaling Vertical/Horizontal Factor Register	
			#define DSVN		NVTBIT(31, 24)			// Scaling Vertical Factor N
			#define DSVM		NVTBIT(23, 16)			// Scaling Vertical Factor M
			#define DSHN		NVTBIT(15, 8)			// Scaling Horizontal Factor N
			#define DSHM		NVTBIT(7, 0)				// Scaling Horizontal Factor M

#define REG_VPEFRC  		(VIDEOIN_BASE + 0x30)	// R/W  : Scaling Frame Rate Factor Register
			#define FRCN		NVTBIT(13, 8)			// Scaling Frame Rate Factor N
			#define FRCM		NVTBIT(5, 0)				// Scaling Frame Rate Factor M
		
/*
#define REG_VWIDTH  		(VIDEOIN_BASE + 0x34)	// R/W  : Frame Output Pixel Straight Width Register
			#define PNOW		BIT(27, 16)				// Planar Frame Output Pixel Straight Width
			#define PKOW		BIT(11, 0)				// Packet Frame Output Pixel Straight Width
*/
#define REG_VSTRIDE 		(VIDEOIN_BASE + 0x34)	// R/W  : Frame Stride Register
			#define PNSTRIDE	NVTBIT(27, 16)			// Planar Frame Stride
			#define PKSTRIDE	NVTBIT(11, 0)			// Packet Frame Stride

#define REG_VFIFO 		(VIDEOIN_BASE + 0x3C)		// R/W  : FIFO threshold Register
			#define FTHP		NVTBIT(27, 24)			// Packet FIFO Threshold 
			#define PTHY		NVTBIT(19, 16)			// Planar Y FIFO Threshold 
			#define PTHU		NVTBIT(10, 8)			// Planar U FIFO Threshold 
			#define PTHV		NVTBIT(2, 0)				// Planar V FIFO Threshold 

#define REG_CMPADDR 	(VIDEOIN_BASE + 0x40)		// R/W  : Current Packet System Memory Address Register
#define REG_CURADDRP 	(VIDEOIN_BASE + 0x50)		// R/W  : FIFO threshold Register
#define REG_CURADDRY 	(VIDEOIN_BASE + 0x54)		// R/W  : Current Planar Y System Memory Address Register
#define REG_CURADDRU 	(VIDEOIN_BASE + 0x58)		// R/W  : Current Planar U System Memory Address Register
#define REG_CURADDRV 	(VIDEOIN_BASE + 0x5C)		// R/W  : Current Planar V System Memory Address Register
#define REG_PACBA0 	(VIDEOIN_BASE + 0x60)		// R/W  : System Memory Packet 0 Base Address Register
#define REG_PACBA1 	(VIDEOIN_BASE + 0x64)		// R/W  : System Memory Packet 1 Base Address Register
#define REG_YBA0 		(VIDEOIN_BASE + 0x80)		// R/W  : System Memory Planar Y Base Address Register
#define REG_UBA0 		(VIDEOIN_BASE + 0x84)		// R/W  : System Memory Planar U Base Address Register
#define REG_VBA0 		(VIDEOIN_BASE + 0x88)		// R/W  : System Memory Planar V Base Address Register
#endif

#endif /*  __ASM_ARCH_REGS_CAP_H */
