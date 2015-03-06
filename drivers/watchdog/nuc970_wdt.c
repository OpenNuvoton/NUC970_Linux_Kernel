/*
 * Copyright (c) 2014 Nuvoton Technology corporation.
 *
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
#include <mach/regs-gcr.h>
#include <mach/regs-wdt.h>

#define WTIS			(0x07 << 8)     /* wdt interval selection */
#define WTE			(0x01 << 7)	/* wdt enable*/
#define WTIE			(0x01 << 6)
#define WTWKF			(0x01 << 5)
#define WTWKE			(0x01 << 4)
#define WTIF			(0x01 << 3)
#define WTRF			(0x01 << 2)	/* wdt reset flag */
#define WTRE			(0x01 << 1)	/* wdt reset enable */
#define WTR			(0x01 << 0)	/* wdt reset */
/*
 * Assumming 32k crytal is configured as the watchdog clock source,
 * the time out interval can be calculated via following formula:
 * WTIS		real time interval (formula)
 * 0x05		((2^ 14 + 1024) * (32k crystal freq))seconds = 0.53 sec
 * 0x06		((2^ 16 + 1024) * (32k crystal freq))seconds = 2.03 sec
 * 0x07		((2^ 18 + 1024) * (32k crystal freq))seconds = 8.03 sec
 */
#define WDT_HW_TIMEOUT		0x05

static int heartbeat = 2;	// default 2 second
module_param(heartbeat, int, 0);
MODULE_PARM_DESC(heartbeat, "Watchdog heartbeats in seconds. "
	"(default = " __MODULE_STRING(WDT_HEARTBEAT) ")");

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started "
	"(default=" __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

struct nuc970_wdt {
	struct resource		*res;
	struct clk		*clk;
	struct clk		*eclk;
	struct platform_device	*pdev;
};

static struct nuc970_wdt *nuc970_wdt;

// UnLock register write protect
static void Unlock_RegWriteProtect(void)
{
    do {
        __raw_writel(0x59, REG_WRPRTR);
        __raw_writel(0x16, REG_WRPRTR);
        __raw_writel(0x88, REG_WRPRTR);
    //wait for write-protection disabled indicator raised 
    } while(!(__raw_readl(REG_WRPRTR) & 1));
}

// Lock register write protect
static void Lock_RegWriteProtect(void)
{
    __raw_writel(0x0, REG_WRPRTR);
}

static int nuc970wdt_ping(struct watchdog_device *wdd)
{
	unsigned int val;
	
	Unlock_RegWriteProtect();
	val = __raw_readl(REG_WDT_CR);
	val |= WTR;
	__raw_writel(val, REG_WDT_CR);
	Lock_RegWriteProtect();
	return 0;
}

static int nuc970wdt_start(struct watchdog_device *wdd)
{
	unsigned int val = 0;

	val |= (WTRE | WTE | WTR);

	if(wdd->timeout < 2) {
		val |= 0x5 << 8;
	} else if (wdd->timeout < 8) {
		val |= 0x6 << 8;
	} else {
		val |= 0x7 << 8;
	}
	Unlock_RegWriteProtect();
	__raw_writel(val, REG_WDT_CR);
	Lock_RegWriteProtect();
	return 0;
}

static int nuc970wdt_stop(struct watchdog_device *wdd)
{
	Unlock_RegWriteProtect();
	__raw_writel(0, REG_WDT_CR);
	Lock_RegWriteProtect();
	return 0;
}


static int nuc970wdt_set_timeout(struct watchdog_device *wdd, unsigned int timeout)
{
	unsigned int val;

	Unlock_RegWriteProtect();
	val = __raw_readl(REG_WDT_CR);
	val &= ~WTIS;
	if(timeout < 2) {
		val |= 0x5 << 8;
	} else if (timeout < 8) {
		val |= 0x6 << 8;
	} else {
		val |= 0x7 << 8;
	}

	__raw_writel(val, REG_WDT_CR);
	Lock_RegWriteProtect();
	return 0;
}

static const struct watchdog_info nuc970wdt_info = {
	.identity	= "nuc970 watchdog",
	.options	= WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING,
};

static struct watchdog_ops nuc970wdt_ops = {
	.owner = THIS_MODULE,
	.start = nuc970wdt_start,
	.stop = nuc970wdt_stop,
	.ping = nuc970wdt_ping,
	.set_timeout = nuc970wdt_set_timeout,
};

static struct watchdog_device nuc970_wdd = {
	.status = WATCHDOG_NOWAYOUT_INIT_STATUS,
	.info = &nuc970wdt_info,
	.ops = &nuc970wdt_ops,
};


static int nuc970wdt_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct clk *clkmux, *clklxt;

	nuc970_wdt = devm_kzalloc(&pdev->dev, sizeof(struct nuc970_wdt), GFP_KERNEL);
	if (!nuc970_wdt)
		return -ENOMEM;

	nuc970_wdt->pdev = pdev;

	nuc970_wdt->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (nuc970_wdt->res == NULL) {
		dev_err(&pdev->dev, "no memory resource specified\n");
		return -ENOENT;
	}

	if (!devm_request_mem_region(&pdev->dev, nuc970_wdt->res->start,
				resource_size(nuc970_wdt->res), pdev->name)) {
		dev_err(&pdev->dev, "failed to get memory region\n");
		return -ENOENT;
	}

	clkmux = clk_get(NULL, "wdt_eclk_mux");
	if (IS_ERR(clkmux)) {
		dev_err(&pdev->dev, "failed to get watchdog clock mux\n");
		ret = PTR_ERR(clkmux);
		return ret;
	}

	clklxt = clk_get(NULL, "xin32k");
	if (IS_ERR(clklxt)) {
		dev_err(&pdev->dev, "failed to get 32k clk\n");
		ret = PTR_ERR(clklxt);
		return ret;
	}


	clk_set_parent(clkmux, clklxt);

	nuc970_wdt->clk = clk_get(NULL, "wdt");
	if (IS_ERR(nuc970_wdt->clk)) {
		dev_err(&pdev->dev, "failed to get watchdog clock\n");
		ret = PTR_ERR(nuc970_wdt->clk);
		return ret;
	}

	clk_prepare(nuc970_wdt->clk);
	clk_enable(nuc970_wdt->clk);

	nuc970_wdt->eclk = clk_get(NULL, "wdt_eclk");
	if (IS_ERR(nuc970_wdt->eclk)) {
		dev_err(&pdev->dev, "failed to get watchdog eclock\n");
		ret = PTR_ERR(nuc970_wdt->eclk);
		return ret;
	}

	clk_prepare(nuc970_wdt->eclk);
	clk_enable(nuc970_wdt->eclk);

	nuc970_wdd.timeout = 2;		// default time out = 2 sec (2.03)
	nuc970_wdd.min_timeout = 1;	// min time out = 1 sec (0.53)
	nuc970_wdd.max_timeout = 9;	// max time out = 9 sec (8.03)
	watchdog_init_timeout(&nuc970_wdd, heartbeat, &pdev->dev);
	watchdog_set_nowayout(&nuc970_wdd, nowayout);

	ret = watchdog_register_device(&nuc970_wdd);
	if (ret) {
		dev_err(&pdev->dev, "err register watchdog device\n");
		clk_disable(nuc970_wdt->clk);
		clk_put(nuc970_wdt->clk);
		return ret;
	}

	return 0;
}

static int nuc970wdt_remove(struct platform_device *pdev)
{
	watchdog_unregister_device(&nuc970_wdd);

	clk_disable(nuc970_wdt->eclk);
	clk_put(nuc970_wdt->eclk);
	clk_disable(nuc970_wdt->clk);
	clk_put(nuc970_wdt->clk);

	return 0;
}

static void nuc970wdt_shutdown(struct platform_device *pdev)
{
	nuc970wdt_stop(&nuc970_wdd);
}

#ifdef CONFIG_PM
static u32 reg_save;
static int nuc970wdt_suspend(struct platform_device *dev, pm_message_t state){

	reg_save = __raw_readl(REG_WDT_CR);
	nuc970wdt_stop(&nuc970_wdd);

	return 0;
}

static int nuc970wdt_resume(struct platform_device *dev)
{
	Unlock_RegWriteProtect();
	__raw_writel(reg_save | WTR, REG_WDT_CR);
	Lock_RegWriteProtect();	
	return 0;
}

#else
#define nuc970wdt_suspend NULL
#define nuc970wdt_resume  NULL
#endif /* CONFIG_PM */

static struct platform_driver nuc970wdt_driver = {
	.probe		= nuc970wdt_probe,
	.remove		= nuc970wdt_remove,
	.shutdown	= nuc970wdt_shutdown,
	.suspend	= nuc970wdt_suspend,
	.resume		= nuc970wdt_resume,
	.driver		= {
		.name	= "nuc970-wdt",
		.owner	= THIS_MODULE,
	},
};

module_platform_driver(nuc970wdt_driver);

MODULE_DESCRIPTION("Watchdog driver for NUC970");
MODULE_LICENSE("GPL");

