/*
 *  linux/drivers/drivers/gpio/nuc970-gpio.c - Nuvoton NUC970 GPIO Drive
 *
 *  Copyright (c) 2010 CompuLab Ltd
 *  Author: shanchun 
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License 2 as published
 *  by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; see the file COPYING.  If not, 	write to
 *  the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.  /gpio-tc3589x.c/
 */



#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/acpi.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <linux/clk.h>

#include <mach/map.h>
#include <mach/regs-gpio.h>

#define GPIO_BASE NUC970_VA_GPIO
#define GPIO_OFFSET 0x20

#define	DRIVER_NAME "nuc970-gpio"

#define NUMGPIO 0x20 * 10	//(PortA~Portj)

static DEFINE_SPINLOCK(gpio_lock);

static unsigned short gpio_ba;

struct gpio_port {
	volatile unsigned int * dir;
	volatile unsigned int * out;
	volatile unsigned int * in;
};

static const struct gpio_port port_class[] = {
	{(volatile unsigned int *)REG_GPIOA_DIR, (volatile unsigned int *)REG_GPIOA_DATAOUT,
	 (volatile unsigned int *)REG_GPIOA_DATAIN},	
	{(volatile unsigned int *)REG_GPIOB_DIR, (volatile unsigned int *)REG_GPIOB_DATAOUT,
	 (volatile unsigned int *)REG_GPIOB_DATAIN},	
	{(volatile unsigned int *)REG_GPIOC_DIR, (volatile unsigned int *)REG_GPIOC_DATAOUT,
	 (volatile unsigned int *)REG_GPIOC_DATAIN},
	{(volatile unsigned int *)REG_GPIOD_DIR, (volatile unsigned int *)REG_GPIOD_DATAOUT,
	 (volatile unsigned int *)REG_GPIOD_DATAIN},	
	{(volatile unsigned int *)REG_GPIOE_DIR, (volatile unsigned int *)REG_GPIOE_DATAOUT,
	 (volatile unsigned int *)REG_GPIOE_DATAIN},	
	{(volatile unsigned int *)REG_GPIOF_DIR, (volatile unsigned int *)REG_GPIOF_DATAOUT,
	 (volatile unsigned int *)REG_GPIOF_DATAIN},	
	{(volatile unsigned int *)REG_GPIOG_DIR, (volatile unsigned int *)REG_GPIOG_DATAOUT,
	 (volatile unsigned int *)REG_GPIOG_DATAIN},	
	{(volatile unsigned int *)REG_GPIOH_DIR, (volatile unsigned int *)REG_GPIOH_DATAOUT,
	 (volatile unsigned int *)REG_GPIOH_DATAIN},	
	{(volatile unsigned int *)REG_GPIOI_DIR, (volatile unsigned int *)REG_GPIOI_DATAOUT,
	 (volatile unsigned int *)REG_GPIOI_DATAIN},		 	 	 	 	 
	{},
};

static const struct gpio_port *nuc970_gpio_cla_port(unsigned gpio_num,
						    unsigned *num)
{
	int group;
	group = gpio_num / GPIO_OFFSET;
	*num = gpio_num % GPIO_OFFSET;
	return &port_class[group];
}

static int nuc970_gpio_core_direction_in(struct gpio_chip *gc,
					 unsigned gpio_num)
{
	int port_num;
	unsigned long value;
	const struct gpio_port *port =
	    nuc970_gpio_cla_port(gpio_num, &port_num);

	spin_lock(&gpio_lock);
	value = __raw_readl(port->dir);
	value &= ~(1 << port_num);
	__raw_writel(value, port->dir);
	spin_unlock(&gpio_lock);

	return 0;
}

static int nuc970_gpio_core_get(struct gpio_chip *gc, unsigned gpio_num)
{
	int port_num, value;
	const struct gpio_port *port;
	port = nuc970_gpio_cla_port(gpio_num, &port_num);
	value = 0;

	if ((__raw_readl(port->dir) & (1 << port_num))) {	//GPIO OUT
		value = (__raw_readl(port->out) >> port_num) & 0x1;

	} else {		//GPIO IN
		value = (__raw_readl(port->in) >> port_num) & 0x1;
		__raw_writel(value, port->in);
	}
	return value;
}

static void nuc970_gpio_core_set(struct gpio_chip *gc, unsigned gpio_num,
				 int val)
{
	int port_num, value;
	const struct gpio_port *port =
	    nuc970_gpio_cla_port(gpio_num, &port_num);
	spin_lock(&gpio_lock);

	if ((__raw_readl(port->dir) & (1 << port_num))) {	//GPIO OUT    
		value = __raw_readl(port->out);
		if (val)
			value |= (1 << port_num);
		else
			value &= ~(1 << port_num);
		__raw_writel(value, port->out);

	} else {		//GPIO IN
		value = __raw_readl(port->in);
		if (val)
			value |= (1 << port_num);
		else
			value &= ~(1 << port_num);
		__raw_writel(value, port->in);;
	}

	spin_unlock(&gpio_lock);
}

static int nuc970_gpio_core_direction_out(struct gpio_chip *gc,
					  unsigned gpio_num, int val)
{
	int port_num;
	unsigned long value;
	const struct gpio_port *port =
	    nuc970_gpio_cla_port(gpio_num, &port_num);

	spin_lock(&gpio_lock);
	value = __raw_readl(port->dir);
	value |= (1 << port_num);
	__raw_writel(value, port->dir);
	nuc970_gpio_core_set(gc, gpio_num, val);
	spin_unlock(&gpio_lock);

	return 0;
}

static int nuc970_gpio_core_to_request(struct gpio_chip *chip, unsigned offset)
{
	return 0;
}

static void nuc970_gpio_core_to_free(struct gpio_chip *chip, unsigned offset)
{
}

static struct gpio_chip nuc970_gpio_port = {
	.label = "nuc970_gpio_port",
	.owner = THIS_MODULE,
	.direction_input = nuc970_gpio_core_direction_in,
	.get = nuc970_gpio_core_get,
	.direction_output = nuc970_gpio_core_direction_out,
	.set = nuc970_gpio_core_set,
	.request = nuc970_gpio_core_to_request,
	.free = nuc970_gpio_core_to_free,
	.base = 0,
	.ngpio = NUMGPIO,
};

static int nuc970_gpio_probe(struct platform_device *pdev)
{
	int err;

	struct clk *clk;

	/* Enable GPIO clock */
	clk = clk_get(NULL, "gpio");
        if (IS_ERR(clk)) {
		printk(KERN_ERR "nuc970-gpio:failed to get gpio clock source\n");
		err = PTR_ERR(clk);
		return err;
	}

	clk_prepare(clk);
	clk_enable(clk);

	nuc970_gpio_port.dev = &pdev->dev;
	err = gpiochip_add(&nuc970_gpio_port);
	if (err < 0) {
		goto err_nuc970_gpio_port;
	}

	return 0;

 err_nuc970_gpio_port:
	gpio_ba = 0;
	return err;
}

static int nuc970_gpio_remove(struct platform_device *pdev)
{
	struct resource *res;

        struct clk *clk;

        /* Disable GPIO clock */
        clk = clk_get(NULL, "gpio");
        if (IS_ERR(clk)) {
                int err;

		printk(KERN_ERR "nuc970-gpio:failed to get gpio clock source\n");
                err = PTR_ERR(clk);
                return err;
        }

        clk_disable(clk);

	if (gpio_ba) {
		int err;

		err = gpiochip_remove(&nuc970_gpio_port);
		if (err)
			dev_err(&pdev->dev, "%s failed, %d\n",
				"gpiochip_remove()", err);
		res = platform_get_resource(pdev, IORESOURCE_IO, 0);
		release_region(res->start, resource_size(res));
		gpio_ba = 0;
		return err;
	}

	return 0;
}

static struct platform_driver nuc970_gpio_driver = {
	.probe		= nuc970_gpio_probe,
	.remove		= nuc970_gpio_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};
module_platform_driver(nuc970_gpio_driver);

MODULE_AUTHOR("shan chun <SCChung@nuvoton.com>");
MODULE_DESCRIPTION("GPIO interface for Nuvoton NUC970 GPIO Drive");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nuc970_gpio");
