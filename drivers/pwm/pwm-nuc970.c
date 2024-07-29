/* drivers/pwm/pwm-nuc970.c
 *
 * Copyright (c) 2014 Nuvoton Technilogy Corp.
 *
 * NUC970 Series PWM driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
*/


#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/pwm.h>

#include <mach/map.h>
#include <mach/regs-pwm.h>
#include <mach/regs-clock.h>

//#define DEBIG_PWM

struct nuc970_chip {
	struct platform_device	*pdev;
	struct clk		*clk;
	struct pwm_chip		 chip;
};

#define to_nuc970_chip(chip)	container_of(chip, struct nuc970_chip, chip)

#ifdef DEBUG_PWM
static void pwm_dbg(void)
{

	printk("%08x\n", __raw_readl(REG_PWM_PPR));
	printk("%08x\n", __raw_readl(REG_PWM_CSR));
	printk("%08x\n", __raw_readl(REG_PWM_PCR));
	printk("%08x\n", __raw_readl(REG_PWM_CNR0));
	printk("%08x\n", __raw_readl(REG_PWM_CMR0));
	printk("%08x\n", __raw_readl(REG_PWM_CNR1));
	printk("%08x\n", __raw_readl(REG_PWM_CMR1));
	printk("%08x\n", __raw_readl(REG_PWM_CNR2));
	printk("%08x\n", __raw_readl(REG_PWM_CMR2));
	printk("%08x\n", __raw_readl(REG_PWM_CNR3));
	printk("%08x\n", __raw_readl(REG_PWM_CMR3));
	printk("%08x\n", __raw_readl(REG_PWM_PIER));
	printk("%08x\n", __raw_readl(REG_PWM_PIIR));

}
#endif

static int nuc970_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	//struct nuc970_chip *nuc970 = to_nuc970_chip(chip);
	int ch = pwm->hwpwm + chip->base;
	unsigned long flags, cnr, cmr;

	local_irq_save(flags);

	if(ch == 0) {
		cnr = __raw_readl(REG_PWM_CNR0);
		cmr = __raw_readl(REG_PWM_CMR0);
		__raw_writel(__raw_readl(REG_PWM_PCR) | (9), REG_PWM_PCR);
		__raw_writel(cnr, REG_PWM_CNR0);
		__raw_writel(cmr, REG_PWM_CMR0);
	} else if(ch == 1) {
		cnr = __raw_readl(REG_PWM_CNR1);
		cmr = __raw_readl(REG_PWM_CMR1);
		__raw_writel(__raw_readl(REG_PWM_PCR) | (9 << 8), REG_PWM_PCR);
		__raw_writel(cnr, REG_PWM_CNR1);
		__raw_writel(cmr, REG_PWM_CMR1);
	} else if (ch == 2) {
		cnr = __raw_readl(REG_PWM_CNR2);
		cmr = __raw_readl(REG_PWM_CMR2);
		__raw_writel(__raw_readl(REG_PWM_PCR) | (9 << 12), REG_PWM_PCR);
		__raw_writel(cnr, REG_PWM_CNR2);
		__raw_writel(cmr, REG_PWM_CMR2);
	} else {	/* ch 3 */
		cnr = __raw_readl(REG_PWM_CNR3);
		cmr = __raw_readl(REG_PWM_CMR3);
		__raw_writel(__raw_readl(REG_PWM_PCR) | (9 << 16), REG_PWM_PCR);
		__raw_writel(cnr, REG_PWM_CNR3);
		__raw_writel(cmr, REG_PWM_CMR3);
	}

	local_irq_restore(flags);
#ifdef DEBUG_PWM
	pwm_dbg();
#endif
	return 0;
}

static void nuc970_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	//struct nuc970_chip *nuc970 = to_nuc970_chip(chip);
	int ch = pwm->hwpwm + chip->base;
	unsigned long flags;

	local_irq_save(flags);
	if(ch == 0)
		__raw_writel(__raw_readl(REG_PWM_PCR) & ~(1), REG_PWM_PCR);
	else if(ch == 1)
		__raw_writel(__raw_readl(REG_PWM_PCR) & ~(1 << 8), REG_PWM_PCR);
	else if (ch == 2)
		__raw_writel(__raw_readl(REG_PWM_PCR) & ~(1 << 12), REG_PWM_PCR);
	else	/* ch 3 */
		__raw_writel(__raw_readl(REG_PWM_PCR) & ~(1 << 16), REG_PWM_PCR);

	local_irq_restore(flags);
#ifdef DEBUG_PWM
	pwm_dbg();
#endif
}

static int nuc970_pwm_set_polarity(struct pwm_chip *chip, struct pwm_device *pwm, enum pwm_polarity polarity)
{
	//struct nuc980_chip *nuc980 = to_nuc980_chip(chip);
	int ch = pwm->hwpwm + chip->base;
	unsigned long flags;

	local_irq_save(flags);

	if(ch == 0) {
		if (polarity == PWM_POLARITY_NORMAL)
			__raw_writel(__raw_readl(REG_PWM_PCR) & ~(4), REG_PWM_PCR);
		else
			__raw_writel(__raw_readl(REG_PWM_PCR) | (4), REG_PWM_PCR);
	} else if(ch == 1) {
		if (polarity == PWM_POLARITY_NORMAL)
			__raw_writel(__raw_readl(REG_PWM_PCR) & ~(4 << 8), REG_PWM_PCR);
		else
			__raw_writel(__raw_readl(REG_PWM_PCR) | (4 << 8), REG_PWM_PCR);
	} else if (ch == 2) {
		if (polarity == PWM_POLARITY_NORMAL)
			__raw_writel(__raw_readl(REG_PWM_PCR) & ~(4 << 12), REG_PWM_PCR);
		else
			__raw_writel(__raw_readl(REG_PWM_PCR) | (4 << 12), REG_PWM_PCR);
	} else {        /* ch 3 */
		if (polarity == PWM_POLARITY_NORMAL)
			__raw_writel(__raw_readl(REG_PWM_PCR) & ~(4 << 16), REG_PWM_PCR);
		else
			__raw_writel(__raw_readl(REG_PWM_PCR) | (4 << 16), REG_PWM_PCR);
	}

	local_irq_restore(flags);
#ifdef DEBUG_PWM
	pwm_dbg();
#endif
	return 0;
}

static int nuc970_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
                             int duty_ns, int period_ns)
{
	struct nuc970_chip *nuc970 = to_nuc970_chip(chip);
	unsigned long period, duty, prescale;
	unsigned long flags;
	int ch = pwm->hwpwm + chip->base;

	// Get PCLK, calculate valid parameter range.
	prescale = clk_get_rate(nuc970->clk) / 1000000 - 1;

	// now pwm time unit is 1000ns.
	period = (period_ns + 500) / 1000;
	duty = (duty_ns + 500) / 1000;

	// don't want the minus 1 below change the value to -1 (0xFFFF)
	if(period == 0)
		period = 1;
	if(duty == 0)
		duty = 1;

	local_irq_save(flags);
	// Set prescale for all pwm channels
	__raw_writel(prescale | (prescale << 8), REG_PWM_PPR);

	if(ch == 0) {
		__raw_writel(period - 1, REG_PWM_CNR0);
		__raw_writel(duty - 1, REG_PWM_CMR0);
	} else if(ch == 1) {
		__raw_writel(period - 1, REG_PWM_CNR1);
		__raw_writel(duty - 1, REG_PWM_CMR1);
	} else if (ch == 2) {
		__raw_writel(period - 1, REG_PWM_CNR2);
		__raw_writel(duty - 1, REG_PWM_CMR2);
	} else {/* ch 3 */
		__raw_writel(period - 1, REG_PWM_CNR3);
		__raw_writel(duty - 1, REG_PWM_CMR3);
	}

	local_irq_restore(flags);

#ifdef DEBUG_PWM
	pwm_dbg();
#endif

	return 0;
}

static struct pwm_ops nuc970_pwm_ops = {
	.enable = nuc970_pwm_enable,
	.disable = nuc970_pwm_disable,
	.config = nuc970_pwm_config,
	.set_polarity = nuc970_pwm_set_polarity,
	.owner = THIS_MODULE,
};

static int nuc970_pwm_probe(struct platform_device *pdev)
{

	struct nuc970_chip *nuc970;
	struct pinctrl *p;
	int ret;
#ifdef CONFIG_OF
	int pwm_id;
#endif

	nuc970 = devm_kzalloc(&pdev->dev, sizeof(*nuc970), GFP_KERNEL);
	if (nuc970 == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory for pwm_device\n");
		return -ENOMEM;
	}
	/* calculate base of control bits in TCON */

	nuc970->chip.dev = &pdev->dev;
	nuc970->chip.ops = &nuc970_pwm_ops;
	//nuc970->chip.of_xlate = of_pwm_xlate_with_flags;
	//nuc970->chip.of_pwm_n_cells = 3;
#ifdef CONFIG_OF
	of_property_read_u32_array(pdev->dev.of_node,"id", &pwm_id,1);
	nuc970->chip.base = pwm_id;
#else
	nuc970->chip.base = pdev->id;
#endif
	nuc970->chip.npwm = 1;

	nuc970->clk = clk_get(NULL, "pwm");
	if (IS_ERR(nuc970->clk)) {
		dev_err(&pdev->dev, "failed to get pwm clock\n");
		ret = PTR_ERR(nuc970->clk);
		return ret;
	}

	clk_prepare(nuc970->clk);
	clk_enable(nuc970->clk);
	// all channel prescale output div by 1
	__raw_writel(0x4444, REG_PWM_CSR);

	if(pdev->id == 0) {
#if defined(CONFIG_OF)
		p = devm_pinctrl_get_select_default(&pdev->dev);
#else
#if defined (CONFIG_NUC970_PWM0_PA12)
		p = devm_pinctrl_get_select(&pdev->dev, "pwm0-PA");
#elif defined (CONFIG_NUC970_PWM0_PB2)
		p = devm_pinctrl_get_select(&pdev->dev, "pwm0-PB");
#elif defined (CONFIG_NUC970_PWM0_PC14)
		p = devm_pinctrl_get_select(&pdev->dev, "pwm0-PC");
#elif defined (CONFIG_NUC970_PWM0_PD12)
		p = devm_pinctrl_get_select(&pdev->dev, "pwm0-PD");
#endif

#ifndef CONFIG_NUC970_PWM0_NONE
		if(IS_ERR(p)) {
			dev_err(&pdev->dev, "unable to reserve output pin\n");

		}
#endif
#endif
	}
	if(pdev->id == 1) {
#if defined(CONFIG_OF)
		p = devm_pinctrl_get_select_default(&pdev->dev);
#else
#if defined (CONFIG_NUC970_PWM1_PA13)
		p = devm_pinctrl_get_select(&pdev->dev, "pwm1-PA");
#elif defined (CONFIG_NUC970_PWM1_PB3)
		p = devm_pinctrl_get_select(&pdev->dev, "pwm1-PB");
#elif defined (CONFIG_NUC970_PWM1_PD13)
		p = devm_pinctrl_get_select(&pdev->dev, "pwm1-PD");
#endif

#ifndef CONFIG_NUC970_PWM1_NONE
		if(IS_ERR(p)) {
			dev_err(&pdev->dev, "unable to reserve output pin\n");
		}
#endif
#endif
	}
	if(pdev->id == 2) {
#if defined(CONFIG_OF)
		p = devm_pinctrl_get_select_default(&pdev->dev);
#else
#if defined (CONFIG_NUC970_PWM2_PA14)
		p = devm_pinctrl_get_select(&pdev->dev, "pwm2-PA");
#elif defined (CONFIG_NUC970_PWM2_PH2)
		p = devm_pinctrl_get_select(&pdev->dev, "pwm2-PH");
#elif defined (CONFIG_NUC970_PWM2_PD14)
		p = devm_pinctrl_get_select(&pdev->dev, "pwm2-PD");
#endif

#ifndef CONFIG_NUC970_PWM2_NONE
		if(IS_ERR(p)) {
			dev_err(&pdev->dev, "unable to reserve output pin\n");
		}
#endif
#endif
	}
	if(pdev->id == 3) {
#if defined(CONFIG_OF)
		p = devm_pinctrl_get_select_default(&pdev->dev);
#else
#if defined (CONFIG_NUC970_PWM3_PA15)
		p = devm_pinctrl_get_select(&pdev->dev, "pwm3-PA");
#elif defined (CONFIG_NUC970_PWM3_PH3)
		p = devm_pinctrl_get_select(&pdev->dev, "pwm3-PH");
#elif defined (CONFIG_NUC970_PWM3_PD15)
		p = devm_pinctrl_get_select(&pdev->dev, "pwm3-PD");
#endif

#ifndef CONFIG_NUC970_PWM3_NONE
		if(IS_ERR(p)) {
			dev_err(&pdev->dev, "unable to reserve output pin\n");
		}
#endif
#endif
	}

	ret = pwmchip_add(&nuc970->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register pwm\n");
		goto err;
	}

	platform_set_drvdata(pdev, nuc970);

	return 0;

err:
	//clk_disable(nuc970->clk);
	return ret;
}

static int nuc970_pwm_remove(struct platform_device *pdev)
{
	struct nuc970_chip *nuc970 = platform_get_drvdata(pdev);

	clk_disable(nuc970->clk);
	return pwmchip_remove(&nuc970->chip);
}

#ifdef CONFIG_PM
static u32 pcr_save, cnr0_save, cnr1_save, cnr2_save, cnr3_save;
static int nuc970_pwm_suspend(struct platform_device *pdev, pm_message_t state)
{
	//struct nuc970_chip *chip = dev_get_drvdata(dev);

	if (pdev->id == 3) {
		pcr_save = __raw_readl(REG_PWM_PCR);

		cnr3_save = __raw_readl(REG_PWM_CNR3);
		__raw_writel(0, REG_PWM_CNR3);
		while(__raw_readl(REG_PWM_PDR3));
	} else if (pdev->id == 2) {
		cnr2_save = __raw_readl(REG_PWM_CNR2);
		__raw_writel(0, REG_PWM_CNR2);
		while(__raw_readl(REG_PWM_PDR2));
	} else if (pdev->id == 1) {
		cnr1_save = __raw_readl(REG_PWM_CNR1);
		__raw_writel(0, REG_PWM_CNR1);
		while(__raw_readl(REG_PWM_PDR1));
	}
	if (pdev->id == 0) {
		cnr0_save = __raw_readl(REG_PWM_CNR0);
		__raw_writel(0, REG_PWM_CNR0);
		while(__raw_readl(REG_PWM_PDR0));

		__raw_writel( __raw_readl(REG_PWM_PCR) & ~0x11101, REG_PWM_PCR);
	}


	return 0;
}

static int nuc970_pwm_resume(struct platform_device *pdev)
{
	//struct nuc970_chip *chip = dev_get_drvdata(dev);

	if (pdev->id == 0) {
		__raw_writel(cnr0_save, REG_PWM_CNR0);
	} else if (pdev->id == 1) {
		__raw_writel(cnr1_save, REG_PWM_CNR1);
	} else if (pdev->id == 2) {
		__raw_writel(cnr2_save, REG_PWM_CNR2);
	} else if (pdev->id == 3) {
		__raw_writel(cnr3_save, REG_PWM_CNR3);

		__raw_writel(pcr_save, REG_PWM_PCR);
	}

	return 0;
}

//static SIMPLE_DEV_PM_OPS(nuc970_pwm_pm_ops, nuc970_pwm_suspend, nuc970_pwm_resume);
#else
#define nuc970_pwm_suspend NULL
#define nuc970_pwm_resume  NULL
#endif


#if defined(CONFIG_OF)
static const struct of_device_id nuc970_pwm0_of_match[] = {
	{   .compatible = "nuvoton,nuc970-pwm" },
	{	},
};
MODULE_DEVICE_TABLE(of, nuc970_pwm0_of_match);
#endif

static struct platform_driver nuc970_pwm_driver = {
	.driver		= {
		.name	= "nuc970-pwm",
		.owner	= THIS_MODULE,
#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(nuc970_pwm0_of_match),
#endif
//#ifdef CONFIG_PM
//		.pm	= &nuc970_pwm_pm_ops,
//#endif
	},
	.probe		= nuc970_pwm_probe,
	.remove		= nuc970_pwm_remove,
	.suspend        = nuc970_pwm_suspend,
	.resume         = nuc970_pwm_resume,
};



module_platform_driver(nuc970_pwm_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nuvoton Technology Corp.");
MODULE_ALIAS("platform:nuc970-pwm");
