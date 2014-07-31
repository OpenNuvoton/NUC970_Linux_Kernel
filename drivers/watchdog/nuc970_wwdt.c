/*
 * Copyright (c) 2014 Nuvoton technology corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/watchdog.h>
#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-wwdt.h>

#define RELOAD_WORD	0x00005AA5

/*
 *  Select WWDT clock source from external 32k crystal.
 *  Here we set compare window to 32, and prescale to 1024 after init.
 *
 *  So WWDT time out every 2048 * 32 * (1/32768) = 2 second,
 *  And software has another 1 second window period to reload
 *  WWDT counter by writing RELOAD_WORD to REG_WWDT_RLD register.
 */
#define WWDT_CONFIG	0x00200D01

struct nuc970_wwdt {
	struct resource		*res;
	struct clk		*clock;
	struct platform_device	*pdev;
};

static struct nuc970_wwdt *nuc970_wwdt;

static int nuc970wwdt_ping(struct watchdog_device *wdd)
{
	__raw_writel(RELOAD_WORD, REG_WWDT_RLD);
	return 0;
}

static int nuc970wwdt_start(struct watchdog_device *wdd)
{
	__raw_writel(WWDT_CONFIG, REG_WWDT_CR);
	return 0;
}

/*
 *  This function always return error, we define it here simply because stop() is mandatory operation.
 *  Due to the fact that WWDT register can only be programmed once, so there is NO WAY OUT!!!
 */
static int nuc970wwdt_stop(struct watchdog_device *wdd)
{

	return -EBUSY;
}

static unsigned int nuc970wwdt_get_timeleft(struct watchdog_device *wdd)
{
	unsigned int time_left;

	time_left = __raw_readl(REG_WWDT_CVR) / 32;

	return time_left;
}

static const struct watchdog_info nuc970wwdt_info = {
	.identity	= "nuc970 window watchdog",
	.options	= WDIOF_KEEPALIVEPING,
};

static struct watchdog_ops nuc970wwdt_ops = {
	.owner 	= THIS_MODULE,
	.start 	= nuc970wwdt_start,
	.stop 	= nuc970wwdt_stop,
	.ping 	= nuc970wwdt_ping,
	.get_timeleft = nuc970wwdt_get_timeleft,
};

static struct watchdog_device nuc970_wdd = {
	.status	= WATCHDOG_NOWAYOUT_INIT_STATUS,
	.info 	= &nuc970wwdt_info,
	.ops 	= &nuc970wwdt_ops,
	.timeout = 2,
};


static int nuc970wwdt_probe(struct platform_device *pdev)
{
	int ret = 0;

	nuc970_wwdt = devm_kzalloc(&pdev->dev, sizeof(struct nuc970_wwdt), GFP_KERNEL);
	if (!nuc970_wwdt)
		return -ENOMEM;

	nuc970_wwdt->pdev = pdev;

	nuc970_wwdt->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (nuc970_wwdt->res == NULL) {
		dev_err(&pdev->dev, "no memory resource specified\n");
		return -ENOENT;
	}

	if (!devm_request_mem_region(&pdev->dev, nuc970_wwdt->res->start,
				resource_size(nuc970_wwdt->res), pdev->name)) {
		dev_err(&pdev->dev, "failed to get memory region\n");
		return -ENOENT;
	}

	nuc970_wwdt->clock = clk_get(&pdev->dev, NULL);
	if (IS_ERR(nuc970_wwdt->clock)) {
		dev_err(&pdev->dev, "failed to find window watchdog clock source\n");
		ret = PTR_ERR(nuc970_wwdt->clock);
		return ret;
	}

	clk_enable(nuc970_wwdt->clock);

	__raw_writel(__raw_readl(REG_CLK_DIV8) | 0xC00, REG_CLK_DIV8);

	ret = watchdog_register_device(&nuc970_wdd);
	if (ret) {
		clk_disable(nuc970_wwdt->clock);
		clk_put(nuc970_wwdt->clock);
		dev_err(&pdev->dev, "err register window watchdog device\n");
		return ret;
	}

	return 0;

}

static int nuc970wwdt_remove(struct platform_device *pdev)
{

	watchdog_unregister_device(&nuc970_wdd);

	clk_disable(nuc970_wwdt->clock);
	clk_put(nuc970_wwdt->clock);


	return 0;
}


static struct platform_driver nuc970wwdt_driver = {
	.probe		= nuc970wwdt_probe,
	.remove		= nuc970wwdt_remove,
	.driver		= {
		.name	= "nuc970-wwdt",
		.owner	= THIS_MODULE,
	},
};

module_platform_driver(nuc970wwdt_driver);

MODULE_DESCRIPTION("NUC970 Window Watchdog Timer Driver");
MODULE_LICENSE("GPL");

