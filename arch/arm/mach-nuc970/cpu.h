/*
 * arch/arm/mach-nuc970/cpu.h
 *
 *
 * Copyright (c) 2014 Nuvoton technology corporation
 * All rights reserved.
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

#include <linux/serial_core.h>

#ifndef __CPU_H__
#define __CPU_H__



#define IODESC_ENT(y)                                  \
{                                                      \
       .virtual = (unsigned long)NUC970_VA_##y,       \
       .pfn     = __phys_to_pfn(NUC970_PA_##y),       \
       .length  = NUC970_SZ_##y,                      \
       .type    = MT_DEVICE,                           \
}

#define NUC970SERIAL_PORT(name)					\
{								\
	.membase	= name##_BA,				\
	.mapbase	= name##_PA,				\
	.irq		= IRQ_##name,				\
	.uartclk	= 12000000,				\
}

#define NUC970PID	NUC970_VA_GCR


extern struct platform_device nuc970_device_sdh;
extern struct platform_device nuc970_device_jpeg;


extern void nuc970_init_irq(void);
extern struct sys_timer nuc970_timer;

extern void nuc970_clock_source(struct device *dev, unsigned char *src);
extern void nuc970_add_clocks(void);
extern void nuc970_init_clocks(void);
extern void nuc970_platform_init(struct platform_device **device, int size);

#endif //__CPU_H__

