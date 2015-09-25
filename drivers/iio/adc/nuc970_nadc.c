/*
 * NUC970 normal ADC driver
 *
 * Copyright (c) 2015 Nuvoton Technology Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/iio/iio.h>
#include <linux/clk.h>

/* nuc970 adc registers offset */
#define CTL     0x00
#define CONF    0x04
#define IER     0x08
#define ISR     0x0C
#define DATA    0x28

#define NUC970_ADC_TIMEOUT	(msecs_to_jiffies(1000))

#define ADC_CHANNEL(_index, _id) {			\
	.type = IIO_VOLTAGE,				\
	.indexed = 1,					\
	.channel = _index,				\
	.address = _index,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),   \
	.datasheet_name = _id,				\
}

struct nuc970_adc_device {
	struct clk  *clk;
    struct clk  *eclk;
	unsigned int    irq;
	void __iomem    *regs;
	struct completion   completion;
};

static const struct iio_chan_spec nuc970_adc_iio_channels[] = {
	ADC_CHANNEL(0, "adc0"),
	ADC_CHANNEL(1, "adc1"),
	ADC_CHANNEL(2, "adc2"),
	ADC_CHANNEL(3, "adc3"),
	ADC_CHANNEL(4, "adc4"),
	ADC_CHANNEL(5, "adc5"),
	ADC_CHANNEL(6, "adc6"),
	ADC_CHANNEL(7, "adc7"),
};

static irqreturn_t nuc970_adc_isr(int irq, void *dev_id)
{
	struct nuc970_adc_device *info = (struct nuc970_adc_device *)dev_id;

    if(readl(info->regs+ISR) & 1)    //check M_F bit
    {
        writel(1, info->regs + ISR); //clear flag
    	complete(&info->completion);
    }

	return IRQ_HANDLED;
}

static void nuc970_adc_channels_remove(struct iio_dev *indio_dev)
{
	kfree(indio_dev->channels);
}

static int nuc970_adc_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan,
		int *val, int *val2, long mask)
{
	struct nuc970_adc_device *info = iio_priv(indio_dev);
    unsigned long timeout;

    if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;
    
    mutex_lock(&indio_dev->mlock);
    
    // enable channel
    writel((readl(info->regs + CONF) & ~(0x7 << 3)) | (chan->channel << 3), info->regs + CONF);
    
    // enable MST
    writel(readl(info->regs + CTL) | 0x100, info->regs + CTL);
    
    timeout = wait_for_completion_interruptible_timeout
			(&info->completion, NUC970_ADC_TIMEOUT);
            
    *val = readl(info->regs + DATA);
    
    mutex_unlock(&indio_dev->mlock);
    
    if (timeout == 0)
		return -ETIMEDOUT;
    
	return IIO_VAL_INT;
}

static const struct iio_info nuc970_adc_info = {
	.read_raw = &nuc970_adc_read_raw,
};

static int nuc970_adc_probe(struct platform_device *pdev)
{
	struct iio_dev	*indio_dev;
    struct nuc970_adc_device *info = NULL;
    int ret = -ENODEV;
	struct resource *res;    
	int irq;
    
    indio_dev = iio_device_alloc(sizeof(struct nuc970_adc_device));
	if (indio_dev == NULL) {
		dev_err(&pdev->dev, "failed to allocate iio device\n");
		ret = -ENOMEM;
		goto err_ret;
	}
    
    info = iio_priv(indio_dev);
    
    /* map the registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "cannot find IO resource\n");
		ret = -ENOENT;
		goto err_ret;
	}
  
  	info->regs = ioremap(res->start, resource_size(res));
	if (info->regs == NULL) {
		dev_err(&pdev->dev, "cannot map IO\n");
		ret = -ENXIO;
		goto err_ret;
	}

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &nuc970_adc_info;
    indio_dev->num_channels = 8;
    indio_dev->channels = nuc970_adc_iio_channels;
    
    /* find the clock and enable it */	
	info->eclk=clk_get(NULL, "adc_eclk");
    clk_prepare(info->eclk);
    clk_enable(info->eclk);
    info->clk=clk_get(NULL, "adc");
    clk_prepare(info->clk);
    clk_enable(info->clk);

    clk_set_rate(info->eclk, 1000000);
    
    irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		ret = irq;
		goto err_ret;
	}

	info->irq = irq;

	init_completion(&info->completion);

	ret = request_irq(info->irq, nuc970_adc_isr,
					0, dev_name(&pdev->dev), info);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed requesting irq, irq = %d\n",
							info->irq);
		goto err_ret;
	}

#ifdef CONFIG_NUC970_NADC_BANDGAP
    writel(3, info->regs + CTL); //enable AD_EN
#endif
#ifdef CONFIG_NUC970_NADC_VREF
    writel(1, info->regs + CTL); //enable AD_EN
#endif

    writel(1, info->regs + IER); //enable M_IEN
    
	ret = iio_device_register(indio_dev);
	if (ret)
		goto err_free_channels;

	platform_set_drvdata(pdev, indio_dev);
    
    writel(1<<2, info->regs + CONF); //enable NACEN
    
    printk("%s: nuc970 Normal ADC adapter\n",
						indio_dev->name);
                        
	return 0;

err_free_channels:
	nuc970_adc_channels_remove(indio_dev);
	iio_device_free(indio_dev);
err_ret:
	return ret;
}

static int nuc970_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
    struct nuc970_adc_device *info = iio_priv(indio_dev);
    
	iio_device_unregister(indio_dev);
	nuc970_adc_channels_remove(indio_dev);

	iio_device_free(indio_dev);
    
    clk_disable_unprepare(info->clk);
    clk_disable_unprepare(info->eclk);
    
    free_irq(info->irq, info);
    
    writel(0, info->regs + CONF); //disable NACEN
    writel(0, info->regs + CTL); //enable AD_EN

	return 0;
}

#ifdef CONFIG_PM
static int nuc970_adc_suspend(struct device *dev)
{	
	return 0;
}

static int nuc970_adc_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops nuc970_adc_pm_ops = {
	.suspend = nuc970_adc_suspend,
	.resume = nuc970_adc_resume,
};
#define NUC970_ADC_PM_OPS (&nuc970_adc_pm_ops)
#else
#define NUC970_ADC_PM_OPS NULL
#endif

static struct platform_driver nuc970_adc_driver = {
	.driver = {
		.name   = "nuc970-nadc",
		.owner	= THIS_MODULE,
		.pm	= NUC970_ADC_PM_OPS,
	},
	.probe	= nuc970_adc_probe,
	.remove	= nuc970_adc_remove,
};

module_platform_driver(nuc970_adc_driver);

MODULE_DESCRIPTION("NUC970 ADC controller driver");
MODULE_AUTHOR("Nuvoton Technology Corp.");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nuc970-nadc");
