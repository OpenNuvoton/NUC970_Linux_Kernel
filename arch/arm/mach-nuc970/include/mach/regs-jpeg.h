/* linux/include/asm/arch-nuc970/regs-jpeg.h
 *
 * Copyright (c) 2015 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Changelog:
 *
 *   2015/09/04     Add this file for nuvoton nuc970 JPEG engine.
 */



#ifndef ___ASM_ARCH_REGS_JPEG_H
#define ___ASM_ARCH_REGS_JPEG_H "$Id: lcd.h,v 1.3 2003/06/26 13:25:06 ben Exp $"

#include "map.h"

#define JPEG_BA		NUC970_VA_JPEG

#define REG_JMCR		(JPEG_BA+0x000)
#define REG_JHEADER		(JPEG_BA+0x004)
#define REG_JITCR		(JPEG_BA+0x008)
#define REG_JPRIQC		(JPEG_BA+0x010)
#define REG_JTHBQC		(JPEG_BA+0x014)
#define REG_JPRIWH		(JPEG_BA+0x018)
#define REG_JTHBWH		(JPEG_BA+0x01C)
#define REG_JPRST		(JPEG_BA+0x020)
#define REG_JTRST		(JPEG_BA+0x024)
#define REG_JDECWH		(JPEG_BA+0x028)
#define REG_JINTCR		(JPEG_BA+0x02C)
#define REG_JDOWFBS		(JPEG_BA+0x03C)
#define REG_JTEST		(JPEG_BA+0x040)
#define REG_JWINDEC0	(JPEG_BA+0x044)
#define REG_JWINDEC1	(JPEG_BA+0x048)
#define REG_JWINDEC2	(JPEG_BA+0x04C)
#define REG_JMACR		(JPEG_BA+0x050)
#define REG_JPSCALU		(JPEG_BA+0x054)
#define REG_JPSCALD		(JPEG_BA+0x058)
#define REG_JTSCALD		(JPEG_BA+0x05C)
#define REG_JDBCR		(JPEG_BA+0x060)
#define REG_JRESERVE	(JPEG_BA+0x070)
#define REG_JOFFSET		(JPEG_BA+0x074)
#define REG_JFSTRIDE	(JPEG_BA+0x078)
#define REG_JYADDR0		(JPEG_BA+0x07C)
#define REG_JUADDR0		(JPEG_BA+0x080)
#define REG_JVADDR0		(JPEG_BA+0x084)
#define REG_JYADDR1		(JPEG_BA+0x088)
#define REG_JUADDR1		(JPEG_BA+0x08C)
#define REG_JVADDR1		(JPEG_BA+0x090)
#define REG_JYSTRIDE	(JPEG_BA+0x094)
#define REG_JUSTRIDE	(JPEG_BA+0x098)
#define REG_JVSTRIDE	(JPEG_BA+0x09C)
#define REG_JIOADDR0	(JPEG_BA+0x0A0)
#define REG_JIOADDR1	(JPEG_BA+0x0A4)
#define REG_JPRI_SIZE	(JPEG_BA+0x0A8)
#define REG_JTHB_SIZE	(JPEG_BA+0x0AC)
#define REG_JUPRAT		(JPEG_BA+0x0B0)
#define REG_JBSFIFO		(JPEG_BA+0x0B4)
#define REG_JSRCH		(JPEG_BA+0x0B8)
#define REG_JQTAB0		(JPEG_BA+0x100)
#define REG_JQTAB1		(JPEG_BA+0x140)
#define REG_JQTAB2		(JPEG_BA+0x180)


/********************* Bit definition of REG_JMCR  ************************/
#define JPG_EN			((u32)0x00000001)			/*!<JPEG Engine Operation Control              */
#define ENG_RST			((u32)0x00000002)			/*!<Soft Reset JPEG Engine (Except JPEG Control Registers) */
#define QT_BUSY			((u32)0x00000004)			/*!<Quantization-Table Busy Status (Read-Only) */
#define EY422			((u32)0x00000008)			/*!<Encode Image Format                        */
#define THB				((u32)0x00000010)			/*!<Encode Thumbnail Image                     */
#define PRI				((u32)0x00000020)			/*!<Encode Primary Image                       */
#define WIN_DEC			((u32)0x00000040)			/*!<JPEG Window Decode Mode                    */
#define ENC_DEC			((u32)0x00000080)			/*!<JPEG Encode/Decode Mode                    */
#define RESUMEO			((u32)0x00000100)			/*!<Resume JPEG Operation for Output On-the-Fly Mode */
#define RESUMEI			((u32)0x00000200)			/*!<Resume JPEG Operation for Input On-the-Fly Mode  */

/********************* Bit definition of REG_JHEADER  ************************/
#define P_QTAB			((u32)0x00000020)			/*!<Primary JPEG Bit-stream Include Quantization-Table */
#define P_HTAB			((u32)0x00000040)			/*!<Primary JPEG Bit-stream Include Huffman-Table */

/********************* Bit definition of REG_JITCR  ************************/
#define ERR_DIS			((u32)0x00000001)			/*!<Decode Error Engine Abort                  */
#define EY_ONLY			((u32)0x00000020)			/*!<Encode Gray-level (Y-component Only) Image */
#define DYUV_MODE		((u32)0x00000700)			/*!<Decoded Image YUV Color Format (Read-Only) */
#define PLANAR_ON		((u32)0x00008000)			
#define BIT18			(1<<18)

/********************* Bit definition of REG_JINTCR  ************************/
#define ERR_INTS		((u32)0x00000001)			/*!<Encode (On-The-Fly) Error Interrupt Status */
#define DER_INTS		((u32)0x00000002)			/*!<Decode Error Interrupt Status              */
#define DEC_INTS		((u32)0x00000004)			/*!<Decode Complete Interrupt Status           */
#define ENC_INTS		((u32)0x00000008)			/*!<Encode Complete Interrupt Status           */
#define OPW_INTS		((u32)0x00000010)			/*!<Output Wait Interrupt Status               */
#define IPW_INTS		((u32)0x00000020)			/*!<Input Wait Interrupt Status                */
#define DHE_INTS		((u32)0x00000040)			/*!<JPEG Header Decode End Wait Interrupt Status */
#define EER_INTE		((u32)0x00000100)			/*!<Encode (On-The-Fly) Error Interrupt Enable */
#define DER_INTE		((u32)0x00000200)			/*!<Decode Error Interrupt Enable              */
#define DEC_INTE		((u32)0x00000400)			/*!<Decode Complete Interrupt Enable           */
#define ENC_INTE		((u32)0x00000800)			/*!<Encode Complete Interrupt Enable           */
#define OPW_INTE		((u32)0x00001000)			/*!<Output Wait Interrupt Enable               */
#define IPW_INTE		((u32)0x00002000)			/*!<Input Wait Interrupt Enable                */
#define DHE_INTE		((u32)0x00004000)			/*!<JPEG Header Decode End Wait Interrupt Enable */
#define DOW_INTS		((u32)0x01000000)			/*!<decoding output Wait Interrupt Status      */
#define DOW_INTE		((u32)0x10000000)			/*!<decoding output Wait Interrupt Enable      */

/********************* Bit definition of REG_JPSCALU  ************************/
#define A_JUMP			((u32)0x00000004)			/*!<Reserve Buffer Size In JPEG Bit-stream For Software Application */
#define JPSCALU_8X		((u32)0x00000040)

/********************* Bit definition of REG_JPSCALD  ************************/
#define PSCALY_F		((u32)0x0000003F)
#define PSCALX_F		((u32)0x00001F00)
#define PS_LPF_ON		((u32)0x00004000)
#define PSX_ON			((u32)0x00008000)


#endif /* ___ASM_ARCH_REGS_JPEG_H */



