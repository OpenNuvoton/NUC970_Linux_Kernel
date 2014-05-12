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
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/watchdog.h>
#include <linux/uaccess.h>

#include <mach/map.h>
#include <mach/regs-wdt.h>

#define WTIS			(0x07 << 8)
#define WTE			(0x01 << 7)	/*wdt enable*/
#define WTIE			(0x01 << 6)
#define WTWKF			(0x01 << 5)
#define WTWKE			(0x01 << 4)
#define WTIF			(0x01 << 3)
#define WTRF			(0x01 << 2)
#define WTRE			(0x01 << 1)
#define WTR			(0x01 << 0)
/*
 * The watchdog time interval can be calculated via following formula:
 * WTIS		real time interval (formula)
 * 0x05		((2^ 14 ) * ((external crystal freq) / 256))seconds
 * 0x06		((2^ 16 ) * ((external crystal freq) / 256))seconds
 * 0x07		((2^ 18 ) * ((external crystal freq) / 256))seconds
 *
 * The external crystal freq is 15Mhz in the nuc970 evaluation board.	// FIXME:
 * So 0x00 = +-0.28 seconds, 0x01 = +-1.12 seconds, 0x02 = +-4.48 seconds,
 * 0x03 = +- 16.92 seconds..
 */
#define WDT_HW_TIMEOUT		0x02
#define WDT_TIMEOUT		(HZ/2)
#define WDT_HEARTBEAT		15

static int heartbeat = WDT_HEARTBEAT;
module_param(heartbeat, int, 0);
MODULE_PARM_DESC(heartbeat, "Watchdog heartbeats in seconds. "
	"(default = " __MODULE_STRING(WDT_HEARTBEAT) ")");

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started "
	"(default=" __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

struct nuc970_wdt {
	struct resource		*res;
	struct clk		*wdt_clock;
	struct platform_device	*pdev;
	char			expect_close;
	struct timer_list	timer;
	spinlock_t		wdt_lock;
	unsigned long		next_heartbeat;
};

static unsigned long nuc970wdt_busy;
static struct nuc970_wdt *nuc970_wdt;

static inline void nuc970_wdt_keepalive(void)
{
	unsigned int val;

	spin_lock(&nuc970_wdt->wdt_lock);

	val = __raw_readl(REG_WDT_CR);
	val |= (WTR | WTIF);
	__raw_writel(val, REG_WDT_CR);

	spin_unlock(&nuc970_wdt->wdt_lock);
}

static inline void nuc970_wdt_start(void)
{
	unsigned int val;

	spin_lock(&nuc970_wdt->wdt_lock);

	val = __raw_readl(REG_WDT_CR);
	val |= (WTRE | WTE | WTR| WTIF);
	val &= ~WTIS;
	val |= (WDT_HW_TIMEOUT << 0x08);	// FIXME:
	__raw_writel(val, REG_WDT_CR);

	spin_unlock(&nuc970_wdt->wdt_lock);

	nuc970_wdt->next_heartbeat = jiffies + heartbeat * HZ;
	mod_timer(&nuc970_wdt->timer, jiffies + WDT_TIMEOUT);
}

static inline void nuc970_wdt_stop(void)
{
	unsigned int val;

	del_timer(&nuc970_wdt->timer);

	spin_lock(&nuc970_wdt->wdt_lock);

	val = __raw_readl(REG_WDT_CR);
	val &= ~WTE;
	__raw_writel(val, REG_WDT_CR);

	spin_unlock(&nuc970_wdt->wdt_lock);
}

static inline void nuc970_wdt_ping(void)
{
	nuc970_wdt->next_heartbeat = jiffies + heartbeat * HZ;
}

static int nuc970_wdt_open(struct inode *inode, struct file *file)
{

	if (test_and_set_bit(0, &nuc970wdt_busy))
		return -EBUSY;

	nuc970_wdt_start();

	return nonseekable_open(inode, file);
}

static int nuc970_wdt_close(struct inode *inode, struct file *file)
{
	if (nuc970_wdt->expect_close == 42)
		nuc970_wdt_stop();
	else {
		dev_crit(&nuc970_wdt->pdev->dev,
			"Unexpected close, not stopping watchdog!\n");
		nuc970_wdt_ping();
	}

	nuc970_wdt->expect_close = 0;
	clear_bit(0, &nuc970wdt_busy);
	return 0;
}

static const struct watchdog_info nuc970_wdt_info = {
	.identity	= "nuc970 watchdog",
	.options	= WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING |
						WDIOF_MAGICCLOSE,
};

static long nuc970_wdt_ioctl(struct file *file,
					unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	int new_value;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		return copy_to_user(argp, &nuc970_wdt_info,
				sizeof(nuc970_wdt_info)) ? -EFAULT : 0;
	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		return put_user(0, p);

	case WDIOC_KEEPALIVE:
		nuc970_wdt_ping();
		return 0;

	case WDIOC_SETTIMEOUT:
		if (get_user(new_value, p))
			return -EFAULT;

		heartbeat = new_value;
		nuc970_wdt_ping();

		return put_user(new_value, p);
	case WDIOC_GETTIMEOUT:
		return put_user(heartbeat, p);
	default:
		return -ENOTTY;
	}
}

static ssize_t nuc970_wdt_write(struct file *file, const char __user *data,
						size_t len, loff_t *ppos)
{
	if (!len)
		return 0;

	/* Scan for magic character */
	if (!nowayout) {
		size_t i;

		nuc970_wdt->expect_close = 0;

		for (i = 0; i < len; i++) {
			char c;
			if (get_user(c, data + i))
				return -EFAULT;
			if (c == 'V') {
				nuc970_wdt->expect_close = 42;
				break;
			}
		}
	}

	nuc970_wdt_ping();
	return len;
}

static void nuc970_wdt_timer_ping(unsigned long data)
{
	if (time_before(jiffies, nuc970_wdt->next_heartbeat)) {
		nuc970_wdt_keepalive();
		mod_timer(&nuc970_wdt->timer, jiffies + WDT_TIMEOUT);
	} else
		dev_warn(&nuc970_wdt->pdev->dev, "Will reset the machine !\n");
}

static const struct file_operations nuc970wdt_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.unlocked_ioctl	= nuc970_wdt_ioctl,
	.open		= nuc970_wdt_open,
	.release	= nuc970_wdt_close,
	.write		= nuc970_wdt_write,
};

static struct miscdevice nuc970wdt_miscdev = {
	.minor		= WATCHDOG_MINOR,
	.name		= "watchdog",
	.fops		= &nuc970wdt_fops,
};

static int nuc970wdt_probe(struct platform_device *pdev)
{
	int ret = 0;

	nuc970_wdt = kzalloc(sizeof(struct nuc970_wdt), GFP_KERNEL);
	if (!nuc970_wdt)
		return -ENOMEM;

	nuc970_wdt->pdev = pdev;

	spin_lock_init(&nuc970_wdt->wdt_lock);

	nuc970_wdt->wdt_clock = clk_get(&pdev->dev, NULL);
	if (IS_ERR(nuc970_wdt->wdt_clock)) {
		dev_err(&pdev->dev, "failed to find watchdog clock source\n");
		ret = PTR_ERR(nuc970_wdt->wdt_clock);
		goto err_get;
	}

	clk_enable(nuc970_wdt->wdt_clock);

	setup_timer(&nuc970_wdt->timer, nuc970_wdt_timer_ping, 0);

	ret = misc_register(&nuc970wdt_miscdev);
	if (ret) {
		dev_err(&pdev->dev, "err register miscdev on minor=%d (%d)\n",
			WATCHDOG_MINOR, ret);
		goto err_clk;
	}

	return 0;

err_clk:
	clk_disable(nuc970_wdt->wdt_clock);
	clk_put(nuc970_wdt->wdt_clock);
err_get:
	kfree(nuc970_wdt);
	return ret;
}

static int nuc970wdt_remove(struct platform_device *pdev)
{
	misc_deregister(&nuc970wdt_miscdev);

	clk_disable(nuc970_wdt->wdt_clock);
	clk_put(nuc970_wdt->wdt_clock);;

	kfree(nuc970_wdt);

	return 0;
}

static struct platform_driver nuc970wdt_driver = {
	.probe		= nuc970wdt_probe,
	.remove		= nuc970wdt_remove,
	.driver		= {
		.name	= "nuc970-wdt",
		.owner	= THIS_MODULE,
	},
};

module_platform_driver(nuc970wdt_driver);

MODULE_DESCRIPTION("Watchdog driver for NUC970");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_ALIAS("platform:nuc970-wdt");
