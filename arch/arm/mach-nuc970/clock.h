/*
 * linux/arch/arm/mach-nuc970/clock.h
 *
 * Copyright (c) 2014 Nuvoton technology corporation
 *
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

#include <asm/clkdev.h>

struct clk {
	unsigned int	ctlbit;
	unsigned int	count;
	unsigned int	src;	/* available clock source */
	void		(*control)(struct clk *clk, int enable);
	u32		(*set_rate)(struct clk *clk, unsigned int src, unsigned int rate);
	u32		(*get_rate)(struct clk *clk);
};


#define DEF_CLKLOOK(_clk, _devname, _conname)		\
	{						\
		.clk		= _clk,			\
		.dev_id		= _devname,		\
		.con_id		= _conname,		\
	}

