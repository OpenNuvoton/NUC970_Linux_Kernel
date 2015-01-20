/*
 * arch/arm/mach-nuc970/include/mach/regs-gpio.h
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

#ifndef __ASM_ARCH_REGS_GPIO_H
#define __ASM_ARCH_REGS_GPIO_H

/* Global control registers */
#define GPIO_BA		NUC970_VA_GPIO

#define REG_GPIOA_DIR 		(GPIO_BA+0x000)
#define REG_GPIOA_DATAOUT 	(GPIO_BA+0x004)
#define REG_GPIOA_DATAIN 	(GPIO_BA+0x008)
#define REG_GPIOA_IMD 		(GPIO_BA+0x00C)
#define REG_GPIOA_IREN 		(GPIO_BA+0x010)
#define REG_GPIOA_IFEN 		(GPIO_BA+0x014)
#define REG_GPIOA_ISR 		(GPIO_BA+0x018)
#define REG_GPIOA_DBEN 		(GPIO_BA+0x01C)
#define REG_GPIOA_PUEN 		(GPIO_BA+0x020)
#define REG_GPIOA_PDEN 		(GPIO_BA+0x024)
#define REG_GPIOA_ICEN 		(GPIO_BA+0x028)
#define REG_GPIOA_ISEN 		(GPIO_BA+0x02C)
	
#define REG_GPIOB_DIR 		(GPIO_BA+0x040)              
#define REG_GPIOB_DATAOUT 	(GPIO_BA+0x044)            
#define REG_GPIOB_DATAIN 	(GPIO_BA+0x048)              
#define REG_GPIOB_IMD 		(GPIO_BA+0x04C)              
#define REG_GPIOB_IREN 		(GPIO_BA+0x050)              
#define REG_GPIOB_IFEN 		(GPIO_BA+0x054)              
#define REG_GPIOB_ISR 		(GPIO_BA+0x058)              
#define REG_GPIOB_DBEN 		(GPIO_BA+0x05C)              
#define REG_GPIOB_PUEN 		(GPIO_BA+0x060)              
#define REG_GPIOB_PDEN 		(GPIO_BA+0x064)              
#define REG_GPIOB_ICEN 		(GPIO_BA+0x068)              
#define REG_GPIOB_ISEN 		(GPIO_BA+0x06C)              
                                                     
#define REG_GPIOC_DIR 		(GPIO_BA+0x080)              
#define REG_GPIOC_DATAOUT 	(GPIO_BA+0x084)            
#define REG_GPIOC_DATAIN 	(GPIO_BA+0x088)              
#define REG_GPIOC_IMD 		(GPIO_BA+0x08C)              
#define REG_GPIOC_IREN 		(GPIO_BA+0x090)              
#define REG_GPIOC_IFEN 		(GPIO_BA+0x094)              
#define REG_GPIOC_ISR 		(GPIO_BA+0x098)              
#define REG_GPIOC_DBEN 		(GPIO_BA+0x09C)              
#define REG_GPIOC_PUEN 		(GPIO_BA+0x0A0)              
#define REG_GPIOC_PDEN 		(GPIO_BA+0x0A4)              
#define REG_GPIOC_ICEN 		(GPIO_BA+0x0A8)              
#define REG_GPIOC_ISEN 		(GPIO_BA+0x0AC)              
                                                     
#define REG_GPIOD_DIR 		(GPIO_BA+0x0C0)              
#define REG_GPIOD_DATAOUT 	(GPIO_BA+0x0C4)            
#define REG_GPIOD_DATAIN 	(GPIO_BA+0x0C8)              
#define REG_GPIOD_IMD 		(GPIO_BA+0x0CC)              
#define REG_GPIOD_IREN 		(GPIO_BA+0x0D0)              
#define REG_GPIOD_IFEN 		(GPIO_BA+0x0D4)              
#define REG_GPIOD_ISR 		(GPIO_BA+0x0D8)              
#define REG_GPIOD_DBEN 		(GPIO_BA+0x0DC)              
#define REG_GPIOD_PUEN 		(GPIO_BA+0x0E0)              
#define REG_GPIOD_PDEN 		(GPIO_BA+0x0E4)              
#define REG_GPIOD_ICEN 		(GPIO_BA+0x0E8)              
#define REG_GPIOD_ISEN 		(GPIO_BA+0x0EC)              
                                                     
#define REG_GPIOE_DIR 		(GPIO_BA+0x100)              
#define REG_GPIOE_DATAOUT 	(GPIO_BA+0x104)            
#define REG_GPIOE_DATAIN 	(GPIO_BA+0x108)              
#define REG_GPIOE_IMD 		(GPIO_BA+0x10C)              
#define REG_GPIOE_IREN 		(GPIO_BA+0x110)              
#define REG_GPIOE_IFEN 		(GPIO_BA+0x114)              
#define REG_GPIOE_ISR 		(GPIO_BA+0x118)              
#define REG_GPIOE_DBEN 		(GPIO_BA+0x11C)              
#define REG_GPIOE_PUEN 		(GPIO_BA+0x120)              
#define REG_GPIOE_PDEN 		(GPIO_BA+0x124)              
#define REG_GPIOE_ICEN 		(GPIO_BA+0x128)              
#define REG_GPIOE_ISEN 		(GPIO_BA+0x12C)              
                                                     
#define REG_GPIOF_DIR 		(GPIO_BA+0x140)              
#define REG_GPIOF_DATAOUT 	(GPIO_BA+0x144)
#define REG_GPIOF_DATAIN 	(GPIO_BA+0x148)              
#define REG_GPIOF_IMD 		(GPIO_BA+0x14C)              
#define REG_GPIOF_IREN 		(GPIO_BA+0x150)              
#define REG_GPIOF_IFEN 		(GPIO_BA+0x154)              
#define REG_GPIOF_ISR 		(GPIO_BA+0x158)              
#define REG_GPIOF_DBEN 		(GPIO_BA+0x15C)              
#define REG_GPIOF_PUEN 		(GPIO_BA+0x160)             
#define REG_GPIOF_PDEN 		(GPIO_BA+0x164)              
#define REG_GPIOF_ICEN 		(GPIO_BA+0x168)              
#define REG_GPIOF_ISEN 		(GPIO_BA+0x16C)              
	                                                     
#define REG_GPIOG_DIR 		(GPIO_BA+0x180)              
#define REG_GPIOG_DATAOUT 	(GPIO_BA+0x184)
#define REG_GPIOG_DATAIN 	(GPIO_BA+0x188)              
#define REG_GPIOG_IMD 		(GPIO_BA+0x18C)              
#define REG_GPIOG_IREN 		(GPIO_BA+0x190)              
#define REG_GPIOG_IFEN 		(GPIO_BA+0x194)              
#define REG_GPIOG_ISR 		(GPIO_BA+0x198)              
#define REG_GPIOG_DBEN 		(GPIO_BA+0x19C)              
#define REG_GPIOG_PUEN 		(GPIO_BA+0x1A0)              
#define REG_GPIOG_PDEN 		(GPIO_BA+0x1A4)             
#define REG_GPIOG_ICEN 		(GPIO_BA+0x1A8)              
#define REG_GPIOG_ISEN 		(GPIO_BA+0x1AC)
                                                     
#define REG_GPIOH_DIR 		(GPIO_BA+0x1C0)              
#define REG_GPIOH_DATAOUT 	(GPIO_BA+0x1C4)            
#define REG_GPIOH_DATAIN 	(GPIO_BA+0x1C8)              
#define REG_GPIOH_IMD 		(GPIO_BA+0x1CC)              
#define REG_GPIOH_IREN 		(GPIO_BA+0x1D0)              
#define REG_GPIOH_IFEN 		(GPIO_BA+0x1D4)
#define REG_GPIOH_ISR 		(GPIO_BA+0x1D8)              
#define REG_GPIOH_DBEN 		(GPIO_BA+0x1DC)              
#define REG_GPIOH_PUEN 		(GPIO_BA+0x1E0)              
#define REG_GPIOH_PDEN 		(GPIO_BA+0x1E4)              
#define REG_GPIOH_ICEN 		(GPIO_BA+0x1E8)              
#define REG_GPIOH_ISEN 		(GPIO_BA+0x1EC)              
	                                                     
#define REG_GPIOI_DIR 		(GPIO_BA+0x200)              
#define REG_GPIOI_DATAOUT 	(GPIO_BA+0x204)            
#define REG_GPIOI_DATAIN 	(GPIO_BA+0x208)              
#define REG_GPIOI_IMD 		(GPIO_BA+0x20C)              
#define REG_GPIOI_IREN 		(GPIO_BA+0x210)              
#define REG_GPIOI_IFEN 		(GPIO_BA+0x214)              
#define REG_GPIOI_ISR 		(GPIO_BA+0x218)              
#define REG_GPIOI_DBEN 		(GPIO_BA+0x21C)              
#define REG_GPIOI_PUEN 		(GPIO_BA+0x220)              
#define REG_GPIOI_PDEN 		(GPIO_BA+0x224)              
#define REG_GPIOI_ICEN 		(GPIO_BA+0x228)              
#define REG_GPIOI_ISEN 		(GPIO_BA+0x22C)  


#define REG_GPIOJ_DIR 		(GPIO_BA+0x240)              
#define REG_GPIOJ_DATAOUT (GPIO_BA+0x244)            
#define REG_GPIOJ_DATAIN 	(GPIO_BA+0x248)              
#define REG_GPIOJ_IMD 		(GPIO_BA+0x24C)              
#define REG_GPIOJ_IREN 		(GPIO_BA+0x250)              
#define REG_GPIOJ_IFEN 		(GPIO_BA+0x254)              
#define REG_GPIOJ_ISR 		(GPIO_BA+0x258)              
#define REG_GPIOJ_DBEN 		(GPIO_BA+0x25C)              
#define REG_GPIOJ_PUEN 		(GPIO_BA+0x260)              
#define REG_GPIOJ_PDEN 		(GPIO_BA+0x264)              
#define REG_GPIOJ_ICEN 		(GPIO_BA+0x268)              
#define REG_GPIOJ_ISEN 		(GPIO_BA+0x26C) 
                                               
#define REG_GPIO_DBNCECON	(GPIO_BA+0x3F0)              
#define REG_GPIO_ISR		  (GPIO_BA+0x3FC)

#define GPIO_OFFSET 0x20
#define	DRIVER_NAME "nuc970-gpio"
#define NUMGPIO 0x20 * 10	//(PortA~Portj)

#endif /*  __ASM_ARCH_REGS_GPIO_H */
