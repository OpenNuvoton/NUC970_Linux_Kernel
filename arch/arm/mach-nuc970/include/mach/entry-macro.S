/*
 * arch/arm/mach-nuc970/include/mach/entry-macro.S
 *
 * Copyright (c) 2014 Nuvoton Technology Corp. 
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
#include <mach/map.h>
#include <mach/regs-aic.h>

	.macro  get_irqnr_preamble, base, tmp
	.endm

	.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp

		mov	\base, #AIC_BA

		ldr	\irqnr, [\base, #AIC_IPER]
		ldr	\irqnr, [\base, #AIC_ISNR]
		cmp	\irqnr, #0

	.endm
