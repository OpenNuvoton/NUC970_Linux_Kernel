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
	void __iomem *pllbase;
	
	double fTemp, Fvco, cal_out_pll;
	int out_dv, in_dv, fb_dv;
	int P, M, N, pll_ctl;
	unsigned long input_clk = parent_rate, output_pll = rate;

	pllbase = pll->base;	
#if 0	
	deviation = 0;
	nItem = 0;
	for (out_dv = 0; out_dv <= 0x7; out_dv++)
	{
		P = out_dv + 1;
	
		for (in_dv = 0; in_dv <= 0x3f; in_dv++)
		{
			M = in_dv + 1;
		
			for (fb_dv = 0; fb_dv <= 0x7f; fb_dv++)
			{
				N = fb_dv + 1;

				Fvco = (input_clk * N) / M;

				if ((Fvco <= 200) || (Fvco >= 500))
					continue;
					
				fTemp = input_clk / M;

				if (fTemp > 80)
					continue;
				
				if ((N == 1) && (fTemp < 11))
					continue;
				if ((N == 2) && (fTemp < 7))
					continue;
				if ((N == 3) && (fTemp < 5))
					continue;
				if ((N == 4) && (fTemp < 4))
					continue;
				if ((N == 5) && (fTemp < 3.5))
					continue;
				if ((N == 6) && (fTemp < 3))
					continue;
				if (((N == 7) || (N == 8)) && (fTemp < 2.5))
					continue;
				if (((N == 9) || (N == 10)) && (fTemp < 3.5))
					continue;
				if (((N >= 11) && (N <= 40)) && (fTemp < 3))
					continue;
				if (((N >= 40) && (N <= 128)) && (fTemp < 2.5))
					continue;

				cal_out_pll = (input_clk * N) / (M * P);
			
				cal_dev = (cal_out_pll / output_pll) * 100;
				if ((cal_dev <= 100 + deviation) && (cal_dev >= 100 - deviation))
				{
					pll_ctl = (out_dv << 13) + (in_dv << 7) + fb_dv;

					if (cal_dev > 100)
						cal_dev -= 100;
					else
						cal_dev = 100 - cal_dev;
				}
			}
		}
	}
#endif		
	return 0;
}

static unsigned long clk_apll_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct clk_apll *pll = to_clk_apll(hw);
	long long ll = 66000000;
	u32 reg;
	

	
	reg = readl(pll->base);
	

	return ll;
}

static int clk_apll_enable(struct clk_hw *hw)
{
	struct clk_apll *pll = to_clk_apll(hw);
	u32 val;
	
	val = readl(pll->base);
	val &= ~0x10000000;			// power down mode disable
	writel(val, pll->base);
	
	return 0;
}

static void clk_apll_disable(struct clk_hw *hw)
{
	struct clk_apll *pll = to_clk_apll(hw);
	u32 val;

	val = readl(pll->base);
	val |= 0x10000000;			// power down mode enable
	writel(val, pll->base);
}

static struct clk_ops clk_apll_ops = {
	.recalc_rate = clk_apll_recalc_rate,
	.enable = clk_apll_enable,
	.disable = clk_apll_disable,
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
