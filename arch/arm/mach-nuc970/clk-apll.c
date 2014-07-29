#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/err.h>

#include "clk-ccf.h"

/**
 * apll 
 *
 * @clk_hw	clock source
 * @parent	the parent clock name
 * @base	base address of pll registers
 *
 */
struct clk_apll {
	struct clk_hw	hw;
	void __iomem	*base;
};

#define to_clk_apll(clk) (container_of(clk, struct clk_apll, clk))

static int clk_apll_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk_apll *pll = to_clk_apll(hw);
	u32 reg;
	
	reg = readl(pll->base) & ~0x0FFFFFFF;
	
	printk("clk_apll_set_rate-->%ld\n", rate);
	
	switch(rate)
	{
		case 98400000:			//i2s
			reg |= 0x8028;
			break;
		
		case 16950000:			//i2s
			reg |= 0x21f0;
			break;
	}
	
	writel(reg, pll->base);
	
	return 0;
}

static unsigned long clk_apll_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct clk_apll *pll = to_clk_apll(hw);
	long long ll;
	u32 reg = readl(pll->base) & 0x0FFFFFFF;
	
	if(parent_rate != 12000000)
		return 0;
	
	switch(reg)
	{
		case 0x0015:
			ll = 264000000;		//system default, 264MHz
			break;
			
		case 0x8028:
			ll = 98400000;		//i2s
			break;
		
		case 0x21f0:
			ll = 16950000;		//i2s
			break;
			
		default:
			ll = 264000000;
			break;
	}
	
	printk("clk_apll_recalc_rate-->%lld\n", ll);
	
	return ll;
}

static long clk_apll_round_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long *prate)
{
	printk("clk_apll_round_rate-->%ld\n", rate);
	
	return rate;
}

static int clk_apll_enable(struct clk_hw *hw)
{
	struct clk_apll *pll = to_clk_apll(hw);
	u32 val;
	
	printk("clk_apll_enable-->\n");
	val = readl(pll->base);
	val &= ~0x10000000;			// PD = 0, power down mode disable
	val |= 0x40000000;			// RESETN = 1
	writel(val, pll->base);
	
	return 0;
}

static void clk_apll_disable(struct clk_hw *hw)
{
	struct clk_apll *pll = to_clk_apll(hw);
	u32 val;
	
	printk("clk_apll_disable-->\n");
	val = readl(pll->base);
	val |= 0x10000000;			// PD = 1, power down mode enable
	val &= ~0x40000000;			// RESETN = 1
	writel(val, pll->base);
}

static struct clk_ops clk_apll_ops = {
	.recalc_rate = clk_apll_recalc_rate,
	.enable = clk_apll_enable,
	.disable = clk_apll_disable,
	.set_rate = clk_apll_set_rate,
	.round_rate = clk_apll_round_rate,
};

struct clk *nuc970_clk_apll(const char *name, const char *parent,
		void __iomem *base)
{
	struct clk_apll *pll;
	struct clk *clk;
	struct clk_init_data init;

	pll = kmalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	pll->base = base;

	init.name = name;
	init.ops = &clk_apll_ops;
	init.flags = 0;
	init.parent_names = &parent;
	init.num_parents = 1;

	pll->hw.init = &init;

	clk = clk_register(NULL, &pll->hw);
	if (IS_ERR(clk))
		kfree(pll);

	return clk;
}
