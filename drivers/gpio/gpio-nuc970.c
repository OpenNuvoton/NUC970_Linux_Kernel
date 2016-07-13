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
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <mach/map.h>
#include <mach/regs-gpio.h>
#include <mach/regs-clock.h>
#include <mach/regs-gcr.h>

#include <mach/irqs.h>

//#define GPIO_DEBUG_ENABLE_ENTER_LEAVE
#ifdef GPIO_DEBUG_ENABLE_ENTER_LEAVE
#define ENTRY()					printk("[%-20s] : Enter...\n", __FUNCTION__)
#define LEAVE()					printk("[%-20s] : Leave...\n", __FUNCTION__)
#else
#define ENTRY()
#define LEAVE()
#endif


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
	{(volatile unsigned int *)REG_GPIOJ_DIR, (volatile unsigned int *)REG_GPIOJ_DATAOUT,
         (volatile unsigned int *)REG_GPIOJ_DATAIN},
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
	unsigned int group,num1,num,reg,value;
	group = offset / GPIO_OFFSET;
	num1  = num = offset % GPIO_OFFSET;
	reg   = (unsigned int)REG_MFP_GPA_L+(group* 0x08);
	if(num>7)
	{
		num -= 8;
		reg = reg + 0x04 ;
	}

	value =	( __raw_readl((volatile unsigned int *)reg) & (0xf<<(num*4)))>>(num*4);
	if(value>0 && value<0xf)
	{
			printk(KERN_ERR "Please Check GPIO%c%02d's multi-function = 0x%x \n",(char)(65+group),num1,value);
			return -EINVAL;
	}
	return 0;
}

static void nuc970_gpio_core_to_free(struct gpio_chip *chip, unsigned offset)
{
}

static int nuc970_gpio_core_to_irq(struct gpio_chip *chip, unsigned offset)
{
	unsigned int irqno= IRQ_GPIO_START+offset;
	switch(offset)
	{
		case NUC970_PH0:
			if((__raw_readl(REG_MFP_GPH_L) & (0xf<<0))==(0xf<<0))
				irqno = IRQ_EXT0_H0;
		break;

		case NUC970_PH1:
			if((__raw_readl(REG_MFP_GPH_L) & (0xf<<4))==(0xf<<4))
				irqno = IRQ_EXT1_H1;
		break;
		case NUC970_PH2:
			if((__raw_readl(REG_MFP_GPH_L) & (0xf<<8))==(0xf<<8))
				irqno = IRQ_EXT2_H2;
		break;
		case NUC970_PH3:
			if((__raw_readl(REG_MFP_GPH_L) & (0xf<<12))==(0xf<<12))
				irqno = IRQ_EXT3_H3;
		break;
		case NUC970_PH4:
			if((__raw_readl(REG_MFP_GPH_L) & (0xf<<16))==(0xf<<16))
				irqno = IRQ_EXT4_H4;
		break;
		case NUC970_PH5:
			if((__raw_readl(REG_MFP_GPH_L) & (0xf<<20))==(0xf<<20))
				irqno = IRQ_EXT5_H5;
		break;
		case NUC970_PH6:
			if((__raw_readl(REG_MFP_GPH_L) & (0xf<<24))==(0xf<<24))
				irqno = IRQ_EXT6_H6;
		break;
		case NUC970_PH7:
			if((__raw_readl(REG_MFP_GPH_L) & (0xf<<28))==(0xf<<28))
				irqno = IRQ_EXT7_H7;
		break;

		case NUC970_PF11:
			if((__raw_readl(REG_MFP_GPF_H) & (0xf<<12))==(0xf<<12))
				irqno = IRQ_EXT0_F11;
		break;
		case NUC970_PF12:
			if((__raw_readl(REG_MFP_GPF_H) & (0xf<<16))==(0xf<<16))
				irqno = IRQ_EXT1_F12;
		break;
		case NUC970_PF13:
			if((__raw_readl(REG_MFP_GPF_H) & (0xf<<20))==(0xf<<20))
				irqno = IRQ_EXT2_F13;
		break;
		case NUC970_PF14:
			if((__raw_readl(REG_MFP_GPF_H) & (0xf<<24))==(0xf<<24))
				irqno = IRQ_EXT3_F14;
		break;
		case NUC970_PF15:
			if((__raw_readl(REG_MFP_GPF_H) & (0xf<<28))==(0xf<<28))
				irqno = IRQ_EXT4_F15;
		break;
		case NUC970_PG15:
			if((__raw_readl(REG_MFP_GPG_H) & (0xf<<28))==(0xf<<28))
				irqno = IRQ_EXT5_G15;
		break;
		case NUC970_PI1:
			if((__raw_readl(REG_MFP_GPI_L) & (0xf<<4))==(0xf<<4))
				irqno = IRQ_EXT6_I1;
		break;
		case NUC970_PI2:
			if((__raw_readl(REG_MFP_GPI_L) & (0xf<<8))==(0xf<<8))
				irqno = IRQ_EXT7_I2;
		break;
		default:
			irqno = IRQ_GPIO_START+offset;
		break;
	}
	return irqno;
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
	.to_irq = nuc970_gpio_core_to_irq,
	.base = 0,
	.ngpio = NUMGPIO,
};

/* example :
 * {IRQ_EXT0_H0,nuc970_eint0_interrupt,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND,"eint0"},
 * 
 * Set external interrup pin as bellow:
 * IRQ_EXT0_H0  / IRQ_EXT1_H1  / IRQ_EXT2_H2  / IRQ_EXT3_H3
 * IRQ_EXT4_H4  / IRQ_EXT5_H5  / IRQ_EXT6_H6  / IRQ_EXT7_H7
 * IRQ_EXT0_F11 / IRQ_EXT1_F12 / IRQ_EXT2_F13 / IRQ_EXT3_F14
 * IRQ_EXT4_F15 / IRQ_EXT5_G15 / IRQ_EXT6_I1  / IRQ_EXT7_I2
 *
 * Set callback function as below:
 * static irqreturn_t nuc970_eint0_interrupt(int irq, void *dev_id){
 *       return IRQ_HANDLED;
 * }
 *
 * Set trigger type as below:
 * IRQF_TRIGGER_FALLING / IRQF_TRIGGER_RISING / IRQF_TRIGGER_HIGH / IRQF_TRIGGER_LOW
 *
 */ 
struct nuc970_eint_pins eint[]={
//ex.{IRQ_EXT0_H0,nuc970_eint0_interrupt,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND,"eint0"}
//Add your external interrup pin here.


{0,0,0,0}
};

/*
 * flag=1 : Enable external interrupt and wakeup functions.
 * flag=0 : Enable external interrupt function only.
 */
void nuc970_enable_eint_wakeup(uint32_t flag){
	int err;
	struct nuc970_eint_pins *peint=eint;
	while(peint->pin!=(u32)0){
		if ((err = request_irq(peint->pin,peint->handler, peint->trigger, peint->name, 0)) != 0) {
			printk("register %s irq failed %d\n",peint->name ,err);
		}
		switch(peint->pin){
			case IRQ_EXT0_H0:
				if(flag==1){
					__raw_writel((1<<0) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(IRQ_EXT0_H0);
				}
				__raw_writel(0xF|__raw_readl(REG_MFP_GPH_L) ,REG_MFP_GPH_L);
			break;
			case IRQ_EXT0_F11:
				if(flag==1){
					__raw_writel((1<<0) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(IRQ_EXT0_F11);
				}
				__raw_writel(0xF000|__raw_readl(REG_MFP_GPF_H) ,REG_MFP_GPF_H);
			break;
			case IRQ_EXT1_H1:
				if(flag==1){
					__raw_writel((1<<1) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(IRQ_EXT1_H1);
				}
				__raw_writel(0xF0|__raw_readl(REG_MFP_GPH_L) ,REG_MFP_GPH_L);
			break;
			case IRQ_EXT1_F12 :
				if(flag==1){
					__raw_writel((1<<1) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(IRQ_EXT1_F12);
				}
				__raw_writel(0xF0000|__raw_readl(REG_MFP_GPF_H) ,REG_MFP_GPF_H);
			break;
			case IRQ_EXT2_H2:
				if(flag==1){
					__raw_writel((1<<2) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(IRQ_EXT2_H2);
				}
				__raw_writel(0xF00|__raw_readl(REG_MFP_GPH_L) ,REG_MFP_GPH_L);
			break;
			case IRQ_EXT2_F13: 
				if(flag==1){
					__raw_writel((1<<2) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(IRQ_EXT2_F13);
				}
				__raw_writel(0xF00000|__raw_readl(REG_MFP_GPF_H) ,REG_MFP_GPF_H);
			break;
			case IRQ_EXT3_H3:
				if(flag==1){
					__raw_writel((1<<3) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(IRQ_EXT3_H3);
				}
				__raw_writel(0xF000|__raw_readl(REG_MFP_GPH_L) ,REG_MFP_GPH_L);
			break;
			case IRQ_EXT3_F14:
				if(flag==1){
					__raw_writel((1<<3) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(IRQ_EXT3_F14);
				}
				__raw_writel(0xF000000|__raw_readl(REG_MFP_GPF_H) ,REG_MFP_GPF_H);
			break;
			case IRQ_EXT4_H4:
				if(flag==1){
					__raw_writel((1<<4) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(IRQ_EXT4_H4);
				}
				__raw_writel(0xF0000|__raw_readl(REG_MFP_GPH_L) ,REG_MFP_GPH_L);
			break;
			case IRQ_EXT4_F15:
				if(flag==1){
					__raw_writel((1<<4) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(IRQ_EXT4_F15);
				}
				__raw_writel(0xF0000000|__raw_readl(REG_MFP_GPF_H) ,REG_MFP_GPF_H);
			break;
			case IRQ_EXT5_H5:
				if(flag==1){
					__raw_writel((1<<5) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(IRQ_EXT5_H5);
				}
				__raw_writel(0xF00000|__raw_readl(REG_MFP_GPH_L) ,REG_MFP_GPH_L);
			break;
			case IRQ_EXT5_G15:
				if(flag==1){
					__raw_writel((1<<5) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(IRQ_EXT5_G15);
				}
				__raw_writel(0xF0000000|__raw_readl(REG_MFP_GPG_H) ,REG_MFP_GPG_H);
			break;
			case IRQ_EXT6_H6:
				if(flag==1){
					__raw_writel((1<<6) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(IRQ_EXT6_H6);
				}
				__raw_writel(0xF000000|__raw_readl(REG_MFP_GPH_L) ,REG_MFP_GPH_L);
			break;
			case IRQ_EXT6_I1:
				if(flag==1){
					__raw_writel((1<<6) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(IRQ_EXT6_I1);
				}
				__raw_writel(0xF0|__raw_readl(REG_MFP_GPI_L) ,REG_MFP_GPI_L);
			break;
			case IRQ_EXT7_H7:
				if(flag==1){
					__raw_writel((1<<7) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(IRQ_EXT7_H7);
				}
				__raw_writel(0xF0000000|__raw_readl(REG_MFP_GPH_L) ,REG_MFP_GPH_L);
			break;
			case IRQ_EXT7_I2:
				if(flag==1){
					__raw_writel((1<<7) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(IRQ_EXT7_I2);
				}
				 __raw_writel(0xF00|__raw_readl(REG_MFP_GPI_L) ,REG_MFP_GPI_L);
			break;
		}
	peint++;
	}
}

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

	clk = clk_get(NULL, "gpio_eclk");
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

	#if defined (CONFIG_PULL_UP_MATRIX_KEYPAD_PIN)
	#if defined (CONFIG_NUC970_KEYPAD_PA_3x2)
                __raw_writel((__raw_readl((void *)(NUC970_VA_GPIO+0x24)) &~ 0xfffffc1f), (void *)(NUC970_VA_GPIO+0x24));
		__raw_writel((__raw_readl((void *)(NUC970_VA_GPIO+0x20)) | 0x370), (void *)(NUC970_VA_GPIO+0x20));
		//printk("\n select CONFIG_NUC970_KEYPAD_PA_3x2 \n");
	#elif defined (CONFIG_NUC970_KEYPAD_PA_4x2)
		__raw_writel((__raw_readl((void *)(NUC970_VA_GPIO+0x24)) &~ 0xfffffc0f), (void *)(NUC970_VA_GPIO+0x24));
		__raw_writel((__raw_readl((void *)(NUC970_VA_GPIO+0x20)) | 0x3f0), (void *)(NUC970_VA_GPIO+0x20));
		//printk("\n select CONFIG_NUC970_KEYPAD_PA_4x2 \n");
	#elif defined (CONFIG_NUC970_KEYPAD_PA_4x4)
		__raw_writel((__raw_readl((void *)(NUC970_VA_GPIO+0x24)) &~ 0xfffff00f), (void *)(NUC970_VA_GPIO+0x24));
		__raw_writel((__raw_readl((void *)(NUC970_VA_GPIO+0x20)) | 0xff0), (void *)(NUC970_VA_GPIO+0x20));
		//printk("\n select CONFIG_NUC970_KEYPAD_PA_4x4 \n");
	#elif defined (CONFIG_NUC970_KEYPAD_PA_4x8)
		__raw_writel((__raw_readl((void *)(NUC970_VA_GPIO+0x24)) &~ 0xffff000f), (void *)(NUC970_VA_GPIO+0x24));
		__raw_writel((__raw_readl((void *)(NUC970_VA_GPIO+0x20)) | 0xfff0), (void *)(NUC970_VA_GPIO+0x20));
		//printk("\n select CONFIG_NUC970_KEYPAD_PA_4x8 \n");
	#elif defined (CONFIG_NUC970_KEYPAD_PH_4x2)
		__raw_writel((__raw_readl((void *)(NUC970_VA_GPIO+0x1e4)) &~ 0xfffffc0f), (void *)(NUC970_VA_GPIO+0x1e4));
		__raw_writel((__raw_readl((void *)(NUC970_VA_GPIO+0x1e0)) | 0x3f0), (void *)(NUC970_VA_GPIO+0x1e0));
		//printk("\n select CONFIG_NUC970_KEYPAD_PH_4x2 \n");
	#elif defined (CONFIG_NUC970_KEYPAD_PH_4x4)
		__raw_writel((__raw_readl((void *)(NUC970_VA_GPIO+0x1e4)) &~ 0xfffff00f), (void *)(NUC970_VA_GPIO+0x1e4));
		__raw_writel((__raw_readl((void *)(NUC970_VA_GPIO+0x1e0)) | 0xff0), (void *)(NUC970_VA_GPIO+0x1e0));
		//printk("\n select CONFIG_NUC970_KEYPAD_PH_4x4 \n");
	#elif defined (CONFIG_NUC970_KEYPAD_PH_4x8)
		__raw_writel((__raw_readl((void *)(NUC970_VA_GPIO+0x1e4)) &~ 0xffff000f), (void *)(NUC970_VA_GPIO+0x1e4));
		__raw_writel((__raw_readl((void *)(NUC970_VA_GPIO+0x1e0)) | 0xfff0), (void *)(NUC970_VA_GPIO+0x1e0));
		//printk("\n select CONFIG_NUC970_KEYPAD_PH_4x8 \n");
	#endif
	#endif

#ifdef CONFIG_GPIO_NUC970_EINT_WKUP
	nuc970_enable_eint_wakeup(1);
#else
	nuc970_enable_eint_wakeup(0);
#endif

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

static int nuc970_gpio_resume(struct platform_device *pdev){
ENTRY();
LEAVE();
return 0;
}

static int nuc970_gpio_suspend(struct platform_device *pdev,pm_message_t state){
ENTRY();
LEAVE();
return 0;
}

static struct platform_driver nuc970_gpio_driver = {
	.probe		= nuc970_gpio_probe,
	.remove		= nuc970_gpio_remove,
	.resume		= nuc970_gpio_resume,
	.suspend	= nuc970_gpio_suspend,
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
