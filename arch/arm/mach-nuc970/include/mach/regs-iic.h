/* linux/include/asm/arch-nuc900/regs-iic.h
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
 *   2006/08/26     vincen.zswan add this file for nuvoton nuc900 evb.
 */



#ifndef ___ASM_ARCH_REGS_IIC_H
#define ___ASM_ARCH_REGS_IIC_H "$Id: lcd.h,v 1.3 2003/06/26 13:25:06 ben Exp $"

/* I2C control registers */
#define I2C0_BA                         NUC970_VA_I2C
#define I2C1_BA                         (NUC970_VA_I2C+0x100)

#define NUC970_I2CCSR                   0x00
#define NUC970_I2CDIVIDER               0x04
#define NUC970_I2CCMDR                  0x08
#define NUC970_I2CSWR                   0x0C
#define NUC970_I2CRXR                   0x10
#define NUC970_I2CTXR                   0x14


#endif /* ___ASM_ARCH_REGS_IIC_H */



