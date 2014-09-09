/*
 * Copyright (c) 2014 Nuvoton technology corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <mach/map.h>
#include <mach/regs-gcr.h>
#include <mach/mfp.h>
#include <linux/platform_data/keypad-nuc970.h>

/* Keypad Interface Control Registers */
#define KPI_CONF		0x00
#define KPI_3KCONF		0x04
#define KPI_STATUS		0x08
#define KPI_RSTC		0x0C
#define KPI_KEST		0x10
#define KPI_KPE			0x18
#define KPI_KRE			0x20
#define KPI_PRESCALDIV	0x28

// KPI_CONF
#define KROW		(0x70000000)	// Keypad Matrix ROW number
#define KCOL		(0x07000000)	// Keypad Matrix COL Number
#define DB_EN		(0x00200000)	// Scan In Signal De-bounce Enable	
#define DB_CLKSEL	(0x000F0000)	// Scan In De-bounce sampling cycle selection
#define PRESCALE	(0x0000FF00)	// Row Scan Cycle Pre-scale Value
#define INPU		(0x00000040)	// key Scan In Pull-UP Enable Register
#define WAKEUP		(0x00000020)	// Lower Power Wakeup Enable	
#define ODEN		(0x00000010)	// Open Drain Enable
#define INTEN		(0x00000008)	// Key Interrupt Enable Control 
#define RKINTEN 	(0x00000004)	// Release Key Interrupt Enable Cintrol
#define PKINTEN 	(0x00000002)	// Press Key Interrupt Enable Control
#define ENKP		(0x00000001)	// Keypad Scan Enable

// KPI_STATUS
#define RROW7		(0x00800000)	// Release key row coordinate
#define RROW6		(0x00400000)
#define RROW5		(0x00200000)
#define RROW4		(0x00100000)
#define RROW3		(0x00080000)
#define RROW2		(0x00040000)
#define RROW1		(0x00020000)
#define RROW0		(0x00010000)
#define PROW7		(0x00008000)	// Press key row coordinate
#define PROW6		(0x00004000)
#define PROW5		(0x00002000)
#define PROW4		(0x00001000)
#define PROW3		(0x00000800)
#define PROW2		(0x00000400)
#define PROW1		(0x00000200)
#define PROW0		(0x00000100)
#define PKEY_INT	(0x00000010)	// Press key interrupt
#define RKEY_INT	(0x00000008)	// Release key interrupt
#define KEY_INT		(0x00000004)	// Key Interrupt
#define RST_3KEY	(0x00000002)	// 3-Keys Reset Flag 
#define PDWAKE		(0x00000001)	// Power Down Wakeup Flag	
 
#define PROW 		(0x00000f00)	// Press Key Row Coordinate

#define KPI_PRESCALE	(8)
#define DEBOUNCE_BIT	(16)

#define NUC970_NUM_ROWS		4
#define NUC970_NUM_COLS		8
#define NUC970_ROW_SHIFT	3

struct nuc970_keypad {
	const struct nuc970_keypad_platform_data *pdata;
	struct clk *clk;
	struct input_dev *input_dev;
	void __iomem *mmio_base;
	int irq;
	unsigned short keymap[NUC970_NUM_ROWS * NUC970_NUM_COLS];
};


void nuc970_keypad_mfp_set(struct platform_device *pdev)
{
	struct pinctrl *p = NULL;
	int retval = 0; 

	#if defined (CONFIG_NUC970_KEYPAD_PA_4x2)
	p = devm_pinctrl_get_select(&pdev->dev, "kpi_4x2-PA");
	#elif defined (CONFIG_NUC970_KEYPAD_PA_4x4)
	p = devm_pinctrl_get_select(&pdev->dev, "kpi_4x4-PA");
	#elif defined (CONFIG_NUC970_KEYPAD_PA_4x8)
	p = devm_pinctrl_get_select(&pdev->dev, "kpi_4x8-PA");
	#elif defined (CONFIG_NUC970_KEYPAD_PH_4x2)
	p = devm_pinctrl_get_select(&pdev->dev, "kpi_4x2-PH");
	#elif defined (CONFIG_NUC970_KEYPAD_PH_4x4)
	p = devm_pinctrl_get_select(&pdev->dev, "kpi_4x4-PH");
	#elif defined (CONFIG_NUC970_KEYPAD_PH_4x8)
	p = devm_pinctrl_get_select(&pdev->dev, "kpi_4x8-PH");
	#endif

	if (IS_ERR(p))
	{
		dev_err(&pdev->dev, "unable to reserve pin\n");
		retval = PTR_ERR(p);
	}

}

static void nuc970_keypad_scan_matrix(struct nuc970_keypad *keypad,
							unsigned int status)
{
	struct input_dev *input_dev = keypad->input_dev;
	unsigned int i;
	unsigned int row;
	unsigned int col;
	unsigned int code;
	unsigned int key;
	unsigned long u32KeyEvent;

	if(status & PKEY_INT)
		u32KeyEvent = __raw_readl(keypad->mmio_base + KPI_KPE);
	else if(status & RKEY_INT)
		u32KeyEvent = __raw_readl(keypad->mmio_base + KPI_KRE);

	for (i=0; i<32; i++)
	{
		if(u32KeyEvent & 1<<i)
		{	
			row = i/8;
			col = i%8;			
		}
	}

	code = MATRIX_SCAN_CODE(row, col, NUC970_ROW_SHIFT);
	
	key = keypad->keymap[code];

	if(status & PKEY_INT)
	{
		__raw_writel(__raw_readl(keypad->mmio_base + KPI_KPE), (keypad->mmio_base + KPI_KPE));

		input_event(input_dev, EV_MSC, MSC_SCAN, code);
		input_report_key(input_dev, key, 1);
		input_sync(input_dev);
	}
	else if(status & RKEY_INT)
	{
		__raw_writel(__raw_readl(keypad->mmio_base + KPI_KRE), (keypad->mmio_base + KPI_KRE));

		input_event(input_dev, EV_MSC, MSC_SCAN, code);
		input_report_key(input_dev, key, 0);
		input_sync(input_dev);
	}

	
}

static irqreturn_t nuc970_keypad_irq_handler(int irq, void *dev_id)
{
	struct nuc970_keypad *keypad = dev_id;
	unsigned int  kstatus;

	kstatus = __raw_readl(keypad->mmio_base + KPI_STATUS);

	if (kstatus & (PKEY_INT|RKEY_INT))
	{
		nuc970_keypad_scan_matrix(keypad, kstatus);
	}
	else
	{
		if(kstatus & PDWAKE)
			__raw_writel(PDWAKE, (keypad->mmio_base + KPI_KRE));
	}


	return IRQ_HANDLED;
}

static int nuc970_keypad_open(struct input_dev *dev)
{
	struct nuc970_keypad *keypad = input_get_drvdata(dev);
	const struct nuc970_keypad_platform_data *pdata = keypad->pdata;
	unsigned int val, config;

	val = INPU | RKINTEN | PKINTEN | INTEN | ENKP;

	#if defined (CONFIG_NUC970_KEYPAD_PA_4x2)
	val |= ((4 - 1) << 28) | ((2 - 1) << 24);
	#elif defined (CONFIG_NUC970_KEYPAD_PA_4x4)
	val |= ((4 - 1) << 28) | ((4 - 1) << 24);
	#elif defined (CONFIG_NUC970_KEYPAD_PA_4x8)
	val |= ((4 - 1) << 28) | ((8 - 1) << 24);
	#elif defined (CONFIG_NUC970_KEYPAD_PH_4x2)
	val |= ((4 - 1) << 28) | ((2 - 1) << 24);
	#elif defined (CONFIG_NUC970_KEYPAD_PH_4x4)
	val |= ((4 - 1) << 28) | ((4 - 1) << 24);
	#elif defined (CONFIG_NUC970_KEYPAD_PH_4x8)
	val |= ((4 - 1) << 28) | ((8 - 1) << 24);
	#endif

	config = (pdata->prescale << KPI_PRESCALE) | (pdata->debounce << DEBOUNCE_BIT) | DB_EN;

	val |= config;

	__raw_writel(val, keypad->mmio_base + KPI_CONF);
	__raw_writel(0x1f, keypad->mmio_base + KPI_PRESCALDIV);

	return 0;
}

static void nuc970_keypad_close(struct input_dev *dev)
{
	struct nuc970_keypad *keypad = input_get_drvdata(dev);

	/* Disable clock unit */
	clk_disable(keypad->clk);
}

static int nuc970_keypad_probe(struct platform_device *pdev)
{
	const struct nuc970_keypad_platform_data *pdata =
						pdev->dev.platform_data;
	const struct matrix_keymap_data *keymap_data;
	struct nuc970_keypad *keypad;
	struct input_dev *input_dev;
	struct resource *res;
	int irq;
	int error;

	if (!pdata) {
		dev_err(&pdev->dev, "no platform data defined\n");
		return -EINVAL;
	}

	keymap_data = pdata->keymap_data;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "failed to get keypad irq\n");
		return -ENXIO;
	}

	keypad = kzalloc(sizeof(struct nuc970_keypad), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!keypad || !input_dev) {
		dev_err(&pdev->dev, "failed to allocate driver data\n");
		error = -ENOMEM;
		goto failed_free;
	}

	keypad->pdata = pdata;
	keypad->input_dev = input_dev;
	keypad->irq = irq;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get I/O memory\n");
		error = -ENXIO;
		goto failed_free;
	}

	res = request_mem_region(res->start, resource_size(res), pdev->name);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to request I/O memory\n");
		error = -EBUSY;
		goto failed_free;
	}

	keypad->mmio_base = ioremap(res->start, resource_size(res));
	if (keypad->mmio_base == NULL) {
		dev_err(&pdev->dev, "failed to remap I/O memory\n");
		error = -ENXIO;
		goto failed_free_res;
	}

	keypad->clk = clk_get(NULL, "kpi");	
	clk_prepare(keypad->clk);
	clk_enable(keypad->clk);
	keypad->clk = clk_get(NULL, "kpi_eclk");	
	clk_prepare(keypad->clk);
	clk_enable(keypad->clk);
	if (IS_ERR(keypad->clk)) {
		dev_err(&pdev->dev, "failed to get keypad clock\n");
		error = PTR_ERR(keypad->clk);
		goto failed_free_io;
	}


	/* set multi-function pin for nuc970 kpi. */
	nuc970_keypad_mfp_set(pdev);

	input_dev->name = pdev->name;
	input_dev->id.bustype = BUS_HOST;
	input_dev->open = nuc970_keypad_open;
	input_dev->close = nuc970_keypad_close;
	input_dev->dev.parent = &pdev->dev;

	error = matrix_keypad_build_keymap(keymap_data, NULL,
					   NUC970_NUM_ROWS, NUC970_NUM_COLS,
					   keypad->keymap, input_dev);
	
	if (error) {
		dev_err(&pdev->dev, "failed to build keymap\n");
		goto failed_put_clk;
	}

	error = request_irq(keypad->irq, nuc970_keypad_irq_handler,
			    0, pdev->name, keypad);
	if (error) {
		dev_err(&pdev->dev, "failed to request IRQ\n");
		goto failed_put_clk;
	}

	__set_bit(EV_REP, input_dev->evbit);
	input_set_capability(input_dev, EV_MSC, MSC_SCAN);
	input_set_drvdata(input_dev, keypad);

	/* Register the input device */
	error = input_register_device(input_dev);
	if (error) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto failed_free_irq;
	}

	platform_set_drvdata(pdev, keypad);
	return 0;

failed_free_irq:
	free_irq(irq, pdev);
failed_put_clk:
	clk_put(keypad->clk);
failed_free_io:
	iounmap(keypad->mmio_base);
failed_free_res:
	release_mem_region(res->start, resource_size(res));
failed_free:
	input_free_device(input_dev);
	kfree(keypad);
	return error;
}

static int nuc970_keypad_remove(struct platform_device *pdev)
{
	struct nuc970_keypad *keypad = platform_get_drvdata(pdev);
	struct resource *res;

	free_irq(keypad->irq, pdev);

	clk_put(keypad->clk);

	input_unregister_device(keypad->input_dev);

	iounmap(keypad->mmio_base);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));

	platform_set_drvdata(pdev, NULL);
	kfree(keypad);

	return 0;
}

static struct platform_driver nuc970_keypad_driver = {
	.probe		= nuc970_keypad_probe,
	.remove		= nuc970_keypad_remove,
	.driver		= {
		.name	= "nuc970-kpi",
		.owner	= THIS_MODULE,
	},
};
module_platform_driver(nuc970_keypad_driver);

MODULE_AUTHOR("nuvoton");
MODULE_DESCRIPTION("nuc970 keypad driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nuc970-keypad");
