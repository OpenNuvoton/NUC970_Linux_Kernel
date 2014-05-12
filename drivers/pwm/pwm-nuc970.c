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
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/pwm.h>

#include <mach/map.h>
#include <mach/regs-pwm.h>

struct nuc970_chip {
	struct platform_device	*pdev;
	struct clk		*clk;
	struct pwm_chip		 chip;
};

#define to_nuc970_chip(chip)	container_of(chip, struct nuc970_chip, chip)


static int nuc970_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	//struct nuc970_chip *nuc970 = to_nuc970_chip(chip);
	int ch = pwm->hwpwm;
	unsigned long flags;

	local_irq_save(flags);

	if(ch == 0)
		__raw_writel(__raw_readl(REG_PWM_PCR) | (9), REG_PWM_PCR);
	else if(ch == 1)
		__raw_writel(__raw_readl(REG_PWM_PCR) | (9 << 8), REG_PWM_PCR);
	else if (ch == 2)
		__raw_writel(__raw_readl(REG_PWM_PCR) | (9 << 12), REG_PWM_PCR);
	else	/* ch 3 */
		__raw_writel(__raw_readl(REG_PWM_PCR) | (9 << 16), REG_PWM_PCR);


	local_irq_restore(flags);

	return 0;
}

static void nuc970_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	//struct nuc970_chip *nuc970 = to_nuc970_chip(chip);
	int ch = pwm->hwpwm;
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
}



static int nuc970_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
		int duty_ns, int period_ns)
{
	//struct nuc970_chip *nuc9790 = to_nuc970_chip(chip);
	unsigned long period, duty, prescale;
	unsigned long flags;
	int ch = pwm->hwpwm;

	/* PWM clock comes from PCLK (75MHz) */
	// TODO: get PCLK, calculate valid parameter range.
	prescale = 75 - 1;
	// now pwm time unit is 1000ns.
	period = (period_ns + 500) / 1000;
	duty = (duty_ns + 500) / 1000;

	local_irq_save(flags);
	// Set prescale for all pwm channels
	__raw_writel(prescale | (prescale << 8), REG_PWM_PPR);

	if(ch == 0) {
		__raw_writel(period, REG_PWM_CNR0);
		__raw_writel(duty, REG_PWM_CMR0);
	} else if(ch == 1) {
		__raw_writel(period, REG_PWM_CNR1);
		__raw_writel(duty, REG_PWM_CMR1);
	} else if (ch == 2) {
		__raw_writel(period, REG_PWM_CNR2);
		__raw_writel(duty, REG_PWM_CMR2);
	} else {/* ch 3 */
		__raw_writel(period, REG_PWM_CNR3);
		__raw_writel(duty, REG_PWM_CMR3);
	}

	local_irq_restore(flags);

	return 0;
}

static struct pwm_ops nuc970_pwm_ops = {
	.enable = nuc970_pwm_enable,
	.disable = nuc970_pwm_disable,
	.config = nuc970_pwm_config,
	.owner = THIS_MODULE,
};

static int nuc970_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nuc970_chip *nuc970;
	unsigned long flags;
	//unsigned int id = pdev->id;
	int ret;

	nuc970 = devm_kzalloc(&pdev->dev, sizeof(*nuc970), GFP_KERNEL);
	if (nuc970 == NULL) {
		dev_err(dev, "failed to allocate memory for pwm_device\n");
		return -ENOMEM;
	}

	/* calculate base of control bits in TCON */
	//nuc970->pwm_id = id; //?
	nuc970->chip.dev = &pdev->dev;
	nuc970->chip.ops = &nuc970_pwm_ops;
	nuc970->chip.base = -1;  //?
	nuc970->chip.npwm = 4;

	nuc970->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(nuc970->clk)) {
		dev_err(dev, "failed to get pwm clk\n");
		return PTR_ERR(nuc970->clk);
	}

	clk_enable(nuc970->clk);

	local_irq_save(flags);

	local_irq_restore(flags);

	ret = pwmchip_add(&nuc970->chip);
	if (ret < 0) {
		dev_err(dev, "failed to register pwm\n");
		goto err;
	}

	platform_set_drvdata(pdev, nuc970);
	return 0;

err:
	clk_disable(nuc970->clk);
	return ret;
}

static int nuc970_pwm_remove(struct platform_device *pdev)
{
	struct nuc970_chip *nuc970 = platform_get_drvdata(pdev);

	// TODO: Disable clock
	return pwmchip_remove(&nuc970->chip);
}

#ifdef CONFIG_PM_SLEEP
static int nuc970_pwm_suspend(struct device *dev)
{
	struct nuc970_chip *chip = dev_get_drvdata(dev);

	__raw_writel(0, REG_PWM_CNR0);
	__raw_writel(0, REG_PWM_CNR1);
	__raw_writel(0, REG_PWM_CNR2);
	__raw_writel(0, REG_PWM_CNR3);

	return 0;
}

static int nu970_pwm_resume(struct device *dev)
{
	struct nuc970_chip *chip = dev_get_drvdata(dev);


	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(nuc970_pwm_pm_ops, nuc970_pwm_suspend,
			nuc970_pwm_resume);

static struct platform_driver nuc970_pwm_driver = {
	.driver		= {
		.name	= "nuc970-pwm",
		.owner	= THIS_MODULE,
		.pm	= &nuc970_pwm_pm_ops,
	},
	.probe		= nuc970_pwm_probe,
	.remove		= nuc970_pwm_remove,
};


static int __init pwm_init(void)
{
	return platform_driver_register(&nuc970_pwm_driver);
}
arch_initcall(pwm_init);

static void __exit pwm_exit(void)
{
	platform_driver_unregister(&nuc970_pwm_driver);
}
module_exit(pwm_exit);


