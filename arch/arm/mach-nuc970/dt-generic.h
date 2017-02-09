/*
 * arch/arm/mach-nuc970/dt-generic.h
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

#include <linux/clkdev.h>
#include <linux/of.h>

#ifndef __GENERIC_H__
#define __GENERIC_H__

extern void __init nuc970_map_io_dt(void);

extern int  __init nuc970_aic_of_init(struct device_node *node,
				    struct device_node *parent);

extern void __init nuc970_dt_initialize(void);

extern int __init nuc970_dt_clock_init(void);

extern void nuc970_restart(char mode, const char *cmd);

extern void __init nuc970_timer_init_dt(void);


#endif //__GENERIC_H__

