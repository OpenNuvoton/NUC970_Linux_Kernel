/* linux/include/asm/arch-nuc900/regs-audio.h
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
 */

#ifndef __ASM_ARM_NUC970_REGS_AUDIO_H
#define __ASM_ARM_NUC970_REGS_AUDIO_H

/* Audio Control Registers */
#define ACTL_CON			0x00
#define ACTL_RESET			0x04
#define ACTL_RDESB			0x08
#define ACTL_RDES_LENGTH	0x0C
#define ACTL_RDESC			0x10
#define ACTL_PDESB			0x14
#define ACTL_PDES_LENGTH	0x18
#define ACTL_PDESC			0x1C
#define ACTL_RSR			0x20
#define ACTL_PSR			0x24
#define ACTL_I2SCON			0x28
#define ACTL_COUNTER		0x2C
#define ACTL_PCMCON			0x30
#define ACTL_PCM1ST			0x34
#define ACTL_PCM2ST			0x38
#define ACTL_RDESB2			0x40
#define ACTL_PDESB2			0x44

#endif
