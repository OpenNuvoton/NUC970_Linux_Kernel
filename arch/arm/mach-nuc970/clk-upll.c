/*
 * linux/arch/arm/mach-nuc970/clk-upll.c
 *
 * Copyright (c) 2014 Nuvoton Technology Corporation.
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
 
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/err.h>

#include "clk-ccf.h"

/**
 * upll 
 *
 * @clk_hw	clock source
 * @parent	the parent clock name
 * @base	base address of pll registers
 *
 */
struct clk_upll {
	struct clk_hw	hw;
	void __iomem	*base;
};

#define to_clk_upll(clk) (container_of(clk, struct clk_upll, clk))

static unsigned long clk_upll_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct clk_upll *pll = to_clk_upll(hw);
	long long ll;
	u32 reg = readl(pll->base) & 0x0FFFFFFF;
	
	if(parent_rate != 12000000)
		return 0;
	
	switch(reg)
	{
		case 0x15:
			ll = 264000000;
			break;
		
		case 0x18:
			ll = 300000000;
			break;
		
		default:
			ll = 264000000;
			break;
	}

	return ll;
}

static struct clk_ops clk_upll_ops = {
	.recalc_rate = clk_upll_recalc_rate,
};

struct clk *nuc970_clk_upll(const char *name, const char *parent,
		void __iomem *base)
{
	struct clk_upll *pll;
	struct clk *clk;
	struct clk_init_data init;

	pll = kmalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	pll->base = base;

	init.name = name;
	init.ops = &clk_upll_ops;
	init.flags = 0;
	init.parent_names = &parent;
	init.num_parents = 1;

	pll->hw.init = &init;

	clk = clk_register(NULL, &pll->hw);
	if (IS_ERR(clk))
		kfree(pll);

	return clk;
}
