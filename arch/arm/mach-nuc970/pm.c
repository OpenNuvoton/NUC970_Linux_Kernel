/* linux/arch/arm/mach-nuc970/pm.c
 *
 * Copyright (c) 2016 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/cpu_pm.h>
#include <linux/suspend.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <mach/regs-clock.h>
#include <mach/regs-aic.h>
#include <mach/regs-gcr.h>
#include <mach/map.h>

#ifdef CONFIG_PM_SLEEP


static int nuc970_suspend_enter(suspend_state_t state)
{

	if(state != PM_SUSPEND_MEM)
		return -EINVAL;

	__raw_writel(__raw_readl(REG_CLK_PMCON) & ~1, REG_CLK_PMCON);	// clear bit 0 so NUC970 enter pd mode instead of idle in next function call
	cpu_do_idle();
	
	__raw_writel(__raw_readl(REG_WKUPSSR), REG_WKUPSER);	// clear wake source flag


	return 0;
}

static const struct platform_suspend_ops nuc970_suspend_ops = {
	.valid		= suspend_valid_only_mem,
	.enter		= nuc970_suspend_enter,
};



void __init nuc970_init_suspend(void)
{
	__raw_writel(__raw_readl(REG_CLK_PMCON) & ~0xFF000000, REG_CLK_PMCON);	// reduce wake up delay time waiting for HXT stable
	suspend_set_ops(&nuc970_suspend_ops);
}
#endif

