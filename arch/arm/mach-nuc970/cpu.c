/*
 * linux/arch/arm/mach-nuc970/cpu.c
 *
 * Copyright (c) 2014~2016 Nuvoton Technology Corporation.
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

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/serial_core.h>
#include <linux/delay.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <asm/irq.h>

#include <mach/hardware.h>
#include <mach/regs-gcr.h>
#include <mach/regs-serial.h>
#include <mach/regs-clock.h>

#include "cpu.h"
#include "clock.h"

#ifdef CONFIG_DEBUG_LL
u32 nuc970_uart_init[1] = {
	/* Debug UART initialization required */
	1,
};
#endif

void nuc970_restart(char mode, const char *cmd)
{
	unsigned long flags;

	local_irq_save(flags);
	//UnlockReg
	while(__raw_readl(NUC970_VA_GCR + 0x1fc) != 1) {
		__raw_writel(0x59, NUC970_VA_GCR + 0x1fc);
		__raw_writel(0x16, NUC970_VA_GCR + 0x1fc);
		__raw_writel(0x88, NUC970_VA_GCR + 0x1fc);
	}

	__raw_writel(1, REG_AHBIPRST);      // System reset...
	local_irq_restore(flags);
}

EXPORT_SYMBOL(nuc970_restart);

