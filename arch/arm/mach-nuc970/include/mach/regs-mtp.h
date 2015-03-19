/*
 * arch/arm/mach-nuc970/include/mach/regs-clock.h
 *
 * Copyright (c) 2014 Nuvoton technology corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

#ifndef __ASM_ARCH_REGS_MTP_H
#define __ASM_ARCH_REGS_MTP_H

typedef struct
{
	u32  MTP_KEYEN;				/*!< Offset: 0x0000   MTP Key Enable                        */ 
	u32  Reserved1[2];			/*!< Offset: 0x0004 ~ 0x0008  Reserved                      */
	u32  MTP_USERDATA;			/*!< Offset: 0x000C   MTP user data (IBR option)            */ 
	u32  MTP_KEY[8];			/*!< Offset: 0x0010 ~ 0x002C  MTP Key Value 0~7             */ 
	u32  MTP_PCYCLE;			/*!< Offset: 0x0030   MTP Program Cycle Control register    */ 
	u32  MTP_CTL;				/*!< Offset: 0x0034   MTP Control register                  */ 
	u32  MTP_PSTART;			/*!< Offset: 0x0038   MTP Program Start register            */ 
	u32  Reserved2;				/*!< Offset: 0x003C   Reserved                              */
	u32  MTP_STATUS;			/*!< Offset: 0x0040   MTP Status register                   */ 
	u32  Reserved3[3];			/*!< Offset: 0x0044 ~ 0x004C   Reserved                     */
	u32  MTP_REGLCTL;			/*!< Offset: 0x0050   MTP Register Write-Protection Control Register  */ 
} 	MTP_TypeDef;

#define MTP           		        ((MTP_TypeDef *) (NUC970_VA_MTP))

#define MTP_KEY_PROG_COUNT		    ((MTP->MTP_STATUS >> 16) & 0xf)


/********************* Bit definition of MTP_KEYEN  ************************/
#define MTP_KEYEN_KEYEN				((u32)0x00000001)			/*!<MTP Key Enable */

/********************* Bit definition of MTP_CTL  *************************/
#define MTP_CTL_MODE_MASK			((u32)0x00000003)			/*!<MTP_CTL MODE control bits mask */
#define MTP_CLT_MODE_IDLE			((u32)0x00000000)			/*!<MTP_CTL MTP idle mode          */
#define MTP_CLT_MODE_PROG			((u32)0x00000002)			/*!<MTP_CTL program key mode       */
#define MTP_CTL_MODE_LOCK			((u32)0x00000003)			/*!<MTP lock key mode              */

/********************* Bit definition of MTP_PSTART  *************************/
#define MTP_PSTART_PSTART			((u32)0x00000001)			/*!<MTP start program	 */

/********************* Bit definition of MTP_STATUS  ************************/
#define MTP_STATUS_MTPEN			((u32)0x00000001)			/*!<MTP enable status	        */
#define MTP_STATUS_KEYVALID		    ((u32)0x00000002)			/*!<MTP key valid status        */
#define MTP_STATUS_NONPRG			((u32)0x00000004)			/*!<MTP non-program status      */
#define MTP_STATUS_LOCKED			((u32)0x00000008)			/*!<MTP key lock status	        */
#define MTP_STATUS_PRGFAIL		    ((u32)0x00000010)			/*!<MTP program fail status	    */
#define MTP_STATUS_BUSY				((u32)0x01000000)			/*!<MTP busy fail	            */

#endif /*  __ASM_ARCH_REGS_MTP_H */

