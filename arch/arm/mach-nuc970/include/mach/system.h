/*
 * arch/arm/mach-nuc970/include/mach/system.h
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

#include <linux/io.h>
#include <asm/proc-fns.h>
#include <mach/map.h>


static void arch_idle(void)
{
}

static void arch_reset(char mode, const char *cmd)
{
	if (mode == 's') {
		/* Jump into ROM at address 0 */
		//cpu_reset(0);
		while(1);
	} else {
		//__raw_writel(WTE | WTRE | WTCLK, WTCR);
		while(1);
	}
}

