/*
 *  linux/drivers/serial/nuc970_can0_platform.c
 *
 *  NUC970 CAN driver
 *
 *
 *  Copyright (C) 2014 Nuvoton Technology Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>

#include <linux/can/dev.h>

#include "nuc970_can.h"

#define CAN_RAMINIT_START_MASK(i)	(1 << (i))


static u16 c_can_plat_read_reg_aligned_to_32bit(struct c_can_priv *priv,
						enum reg index)
{
	return readw(priv->base + 2 * priv->regs[index]);
}

static void c_can_plat_write_reg_aligned_to_32bit(struct c_can_priv *priv,
						enum reg index, u16 val)
{
	writew(val, priv->base + 2 * priv->regs[index]);
}

static struct platform_device_id nuc970_can0_driver_ids[] = {
	{ "nuc970-can0", 0 },
	{ },
};
MODULE_DEVICE_TABLE(platform, nuc970_can0_driver_ids);

static const struct of_device_id nuc970_can0_of_table[] = {
	{ .compatible = "nuc970-can0", .data = &nuc970_can0_driver_ids[0] },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, nuc970_can0_of_table);

static int c_can_plat_probe(struct platform_device *pdev)
{
	int ret;
	void __iomem *addr;
	struct net_device *dev;
	struct c_can_priv *priv;
	const struct of_device_id *match;
	const struct platform_device_id *id;
	struct pinctrl *pinctrl;
	struct resource *mem;
	int irq;
	struct clk *clk;

	int retval = 0; 

	if (pdev->dev.of_node) {
		match = of_match_device(nuc970_can0_of_table, &pdev->dev);
		if (!match) {
			dev_err(&pdev->dev, "Failed to find matching dt id\n");
			ret = -EINVAL;
			goto exit;
		}
		id = match->data;
	} else {
		id = platform_get_device_id(pdev);
	}

	#if defined (CONFIG_NUC970_CAN0_PB)
		pinctrl = devm_pinctrl_get_select(&pdev->dev, "can0-PB");
	#elif defined (CONFIG_NUC970_CAN0_PH)
		pinctrl = devm_pinctrl_get_select(&pdev->dev, "can0-PH");
	#elif defined (CONFIG_NUC970_CAN0_PI)
		pinctrl = devm_pinctrl_get_select(&pdev->dev, "can0-PH");
	#endif
	
	if (IS_ERR(pinctrl))
	{
		dev_err(&pdev->dev, "unable to reserve pin\n");
		retval = PTR_ERR(pinctrl);
	}

	clk = clk_get(NULL, "can0");
	clk_prepare(clk);
	clk_enable(clk);

	/* get the platform data */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (!mem || irq <= 0) {
		ret = -ENODEV;
		goto exit_free_clk;
	}

	if (!request_mem_region(mem->start, resource_size(mem),
				KBUILD_MODNAME)) {
		dev_err(&pdev->dev, "resource unavailable\n");
		ret = -ENODEV;
		goto exit_free_clk;
	}

	addr = ioremap(mem->start, resource_size(mem));
	if (!addr) {
		dev_err(&pdev->dev, "failed to map can port\n");
		ret = -ENOMEM;
		goto exit_release_mem;
	}

	/* allocate the c_can device */
	dev = alloc_c_can_dev();
	if (!dev) {
		ret = -ENOMEM;
		goto exit_iounmap;
	}

	priv = netdev_priv(dev);

	priv->regs = reg_map_c_can;
	priv->read_reg = c_can_plat_read_reg_aligned_to_32bit;
	priv->write_reg = c_can_plat_write_reg_aligned_to_32bit;
	
	dev->irq = irq;
	priv->base = addr;
	priv->device = &pdev->dev;
	priv->can.clock.freq = clk_get_rate(clk);
	priv->priv = clk;
	priv->type = id->driver_data;

	platform_set_drvdata(pdev, dev);
	SET_NETDEV_DEV(dev, &pdev->dev);

	ret = register_c_can_dev(dev);
	if (ret) {
		dev_err(&pdev->dev, "registering %s failed (err=%d)\n",
			KBUILD_MODNAME, ret);
		goto exit_free_device;
	}

	dev_info(&pdev->dev, "%s device registered (regs=%p, irq=%d)\n",
		 KBUILD_MODNAME, priv->base, dev->irq);
	return 0;

exit_free_device:
	platform_set_drvdata(pdev, NULL);
	free_c_can_dev(dev);
exit_iounmap:
	iounmap(addr);
exit_release_mem:
	release_mem_region(mem->start, resource_size(mem));
exit_free_clk:
	clk_put(clk);
exit:
	dev_err(&pdev->dev, "probe failed\n");

	return ret;
}

static int c_can_plat_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct c_can_priv *priv = netdev_priv(dev);
	struct resource *mem;

	unregister_c_can_dev(dev);
	platform_set_drvdata(pdev, NULL);

	free_c_can_dev(dev);
	iounmap(priv->base);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(mem->start, resource_size(mem));

	clk_put(priv->priv);

	return 0;
}

#ifdef CONFIG_PM
static int c_can_suspend(struct platform_device *pdev, pm_message_t state)
{
	int ret;
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct c_can_priv *priv = netdev_priv(ndev);

	if (netif_running(ndev)) {
		netif_stop_queue(ndev);
		netif_device_detach(ndev);
	}

	ret = c_can_power_down(ndev);
	if (ret) {
		netdev_err(ndev, "failed to enter power down mode\n");
		return ret;
	}

	priv->can.state = CAN_STATE_SLEEPING;

	return 0;
}

static int c_can_resume(struct platform_device *pdev)
{
	int ret;
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct c_can_priv *priv = netdev_priv(ndev);

	ret = c_can_power_up(ndev);
	if (ret) {
		netdev_err(ndev, "Still in power down mode\n");
		return ret;
	}

	priv->can.state = CAN_STATE_ERROR_ACTIVE;

	if (netif_running(ndev)) {
		netif_device_attach(ndev);
		netif_start_queue(ndev);
	}

	return 0;
}
#else
#define c_can_suspend NULL
#define c_can_resume NULL
#endif

static struct platform_driver nuc970_can0_driver = {
		.driver 	= {
			.name	= "nuc970-can0",
			.owner	= THIS_MODULE,
		},
	.probe = c_can_plat_probe,
	.remove = c_can_plat_remove,
	.suspend = c_can_suspend,
	.resume = c_can_resume,
	.id_table = nuc970_can0_driver_ids,
};


module_platform_driver(nuc970_can0_driver);

MODULE_AUTHOR("nuvoton");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Platform CAN bus driver for NUC970 controller");
