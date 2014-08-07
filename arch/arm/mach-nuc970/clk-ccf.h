#ifndef __MACH_NUC970_CLK_H
#define __MACH_NUC970_CLK_H

#include <linux/spinlock.h>
#include <linux/clk-provider.h>

extern struct clk *nuc970_clk_apll(const char *name, const char *parent,
		void __iomem *base);
extern struct clk *nuc970_clk_upll(const char *name, const char *parent,
		void __iomem *base);		
		
extern spinlock_t nuc970_lock;

static inline struct clk *nuc970_clk_fixed(const char *name, int rate)
{
	return clk_register_fixed_rate(NULL, name, NULL, CLK_IS_ROOT, rate);
}

static inline struct clk *nuc970_clk_mux(const char *name, void __iomem *reg,
		u8 shift, u8 width, const char **parents, int num_parents)
{
	return clk_register_mux(NULL, name, parents, num_parents, 0, reg, shift,
			width, 0, &nuc970_lock);
}

static inline struct clk *nuc970_clk_divider(const char *name, const char *parent,
		void __iomem *reg, u8 shift, u8 width)
{
	return clk_register_divider(NULL, name, parent, 0,
			reg, shift, width, 0, &nuc970_lock);
}

static inline struct clk *nuc970_clk_fixed_factor(const char *name,
		const char *parent, unsigned int mult, unsigned int div)
{
	return clk_register_fixed_factor(NULL, name, parent,
			CLK_SET_RATE_PARENT, mult, div);
}

static inline struct clk *nuc970_clk_gate(const char *name, const char *parent,
		void __iomem *reg, u8 shift)
{
	return clk_register_gate(NULL, name, parent, CLK_SET_RATE_PARENT, reg,
			shift, 0, &nuc970_lock);
}

#endif
