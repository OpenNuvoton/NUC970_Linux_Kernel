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

#include <mach/gpio.h>
#include <linux/gpio.h>

#include <linux/platform_data/keypad-nuc970.h>

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


	value = __raw_readl(port->out);
	if (val)
		value |= (1 << port_num);
	else
		value &= ~(1 << port_num);
	__raw_writel(value, port->out);


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
	spin_unlock(&gpio_lock);
	nuc970_gpio_core_set(gc, gpio_num, val);

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
	if(value != 0)
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


#ifndef CONFIG_OF
/*
 * @brief       External Interrupt 0 Handler
 * @details     This function will be used by EINT0,
 *              when enable IRQ_EXT0_H0 or IRQ_EXT0_F11 in eint0
 */
/*
static irqreturn_t nuc970_eint0_interrupt(int irq, void *dev_id){
	printk("@0\n");
	return IRQ_HANDLED;
}
*/

/* If enable IRQ_EXT0_H0 or IRQ_EXT0_F11 , linux will enable EINT0
 * User can modify trigger tiypes as below :
 * IRQF_TRIGGER_FALLING / IRQF_TRIGGER_RISING / IRQF_TRIGGER_HIGH / IRQF_TRIGGER_LOW
 */
struct nuc970_eint_pins eint0[]={
//{IRQ_EXT0_H0, nuc970_eint0_interrupt,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,"eint0"},
//{IRQ_EXT0_F11,nuc970_eint0_interrupt,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,"eint0"},
{0,0,0,0}
};

/*
 * @brief       External Interrupt 0 Handler
 * @details     This function will be used by EINT1,
 *              when enable IRQ_EXT1_H1 or IRQ_EXT1_F12 in eint1
 */
/*
static irqreturn_t nuc970_eint1_interrupt(int irq, void *dev_id){
	printk("@1\n");
	return IRQ_HANDLED;
}
*/

/* If enable IRQ_EXT1_H1 or IRQ_EXT1_F12 , linux will enable EINT1
 * User can modify trigger tiypes as below :
 * IRQF_TRIGGER_FALLING / IRQF_TRIGGER_RISING / IRQF_TRIGGER_HIGH / IRQF_TRIGGER_LOW
 */
struct nuc970_eint_pins eint1[]={
//{IRQ_EXT1_H1, nuc970_eint1_interrupt,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,"eint1"},
//{IRQ_EXT1_F12,nuc970_eint1_interrupt,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,"eint1"},
{0,0,0,0}
};

/*
 * @brief       External Interrupt 2 Handler
 * @details     This function will be used by EINT2,
 *              when enable IRQ_EXT2_H2 or IRQ_EXT2_F13 in eint2
 */
/*
static irqreturn_t nuc970_eint2_interrupt(int irq, void *dev_id){
	printk("@2\n");
	return IRQ_HANDLED;
}
*/

/* If enable IRQ_EXT2_H2 or IRQ_EXT2_F13 , linux will enable EINT2
 * User can modify trigger tiypes as below :
 * IRQF_TRIGGER_FALLING / IRQF_TRIGGER_RISING / IRQF_TRIGGER_HIGH / IRQF_TRIGGER_LOW
 */
struct nuc970_eint_pins eint2[]={
//{IRQ_EXT2_H2, nuc970_eint2_interrupt,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,"eint2"},
//{IRQ_EXT2_F13,nuc970_eint2_interrupt,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,"eint2"},
{0,0,0,0}
};

/*
 * @brief       External Interrupt 3 Handler
 * @details     This function will be used by EINT3,
 *              when enable IRQ_EXT3_H3 or IRQ_EXT3_F14 in eint3
 */
/*
static irqreturn_t nuc970_eint3_interrupt(int irq, void *dev_id){
	printk("@3\n");
	return IRQ_HANDLED;
}
*/

/* If enable IRQ_EXT3_H3 or IRQ_EXT3_F14 , linux will enable EINT31
 * User can modify trigger tiypes as below :
 * IRQF_TRIGGER_FALLING / IRQF_TRIGGER_RISING / IRQF_TRIGGER_HIGH / IRQF_TRIGGER_LOW
 */
struct nuc970_eint_pins eint3[]={
//{IRQ_EXT3_H3, nuc970_eint3_interrupt,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,"eint3"},
//{IRQ_EXT3_F14,nuc970_eint3_interrupt,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,"eint3"},
{0,0,0,0}
};

/*
 * @brief       External Interrupt 3 Handler
 * @details     This function will be used by EINT3,
 *              when enable IRQ_EXT4_H4 or IRQ_EXT4_F15 in eint4
 */
/*
static irqreturn_t nuc970_eint4_interrupt(int irq, void *dev_id){
	printk("@4\n");
	return IRQ_HANDLED;
}
*/

/* If enable IRQ_EXT4_H4 or IRQ_EXT4_F15 , linux will enable EINT4
 * User can modify trigger tiypes as below :
 * IRQF_TRIGGER_FALLING / IRQF_TRIGGER_RISING / IRQF_TRIGGER_HIGH / IRQF_TRIGGER_LOW
 */
struct nuc970_eint_pins eint4[]={
//{IRQ_EXT4_H4, nuc970_eint4_interrupt,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,"eint4"},
//{IRQ_EXT4_F15,nuc970_eint4_interrupt,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,"eint4"},
{0,0,0,0}
};

/*
 * @brief       External Interrupt 4 Handler
 * @details     This function will be used by EINT4,
 *              when enable IRQ_EXT5_H5 or IRQ_EXT5_G15 in eint5
 */
/*
static irqreturn_t nuc970_eint5_interrupt(int irq, void *dev_id){
	printk("@5\n");
	return IRQ_HANDLED;
}
*/

/* If enable IRQ_EXT5_H5 or IRQ_EXT5_G15 , linux will enable EINT5
 * User can modify trigger tiypes as below :
 * IRQF_TRIGGER_FALLING / IRQF_TRIGGER_RISING / IRQF_TRIGGER_HIGH / IRQF_TRIGGER_LOW
 */
struct nuc970_eint_pins eint5[]={
//{IRQ_EXT5_H5, nuc970_eint5_interrupt,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,"eint5"},
//{IRQ_EXT5_G15,nuc970_eint5_interrupt,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,"eint5"},
{0,0,0,0}
};

/*
 * @brief       External Interrupt 6 Handler
 * @details     This function will be used by EINT5,
 *              when enable IRQ_EXT6_H6 or IRQ_EXT6_I1 in eint6
 */
/*
static irqreturn_t nuc970_eint6_interrupt(int irq, void *dev_id){
	printk("@6\n");
	return IRQ_HANDLED;
}

*/
/* If enable IRQ_EXT6_H6 or IRQ_EXT6_I1 , linux will enable EINT6
 * User can modify trigger tiypes as below :
 * IRQF_TRIGGER_FALLING / IRQF_TRIGGER_RISING / IRQF_TRIGGER_HIGH / IRQF_TRIGGER_LOW
 */
struct nuc970_eint_pins eint6[]={
//{IRQ_EXT6_H6,nuc970_eint6_interrupt,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,"eint6"},
//{IRQ_EXT6_I1,nuc970_eint6_interrupt,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,"eint6"},
{0,0,0,0}
};

/*
 * @brief       External Interrupt 7 Handler
 * @details     This function will be used by EINT1,
 *              when enable IRQ_EXT7_H7 or IRQ_EXT7_I2 in eint7
 */
/*
static irqreturn_t nuc970_eint7_interrupt(int irq, void *dev_id){
	printk("@7\n");
	return IRQ_HANDLED;
}

*/
/* If enable IRQ_EXT7_H7 or IRQ_EXT7_I2 , linux will enable EINT7
 * User can modify trigger tiypes as below :
 * IRQF_TRIGGER_FALLING / IRQF_TRIGGER_RISING / IRQF_TRIGGER_HIGH / IRQF_TRIGGER_LOW
 */
struct nuc970_eint_pins eint7[]={
//{IRQ_EXT7_H7,nuc970_eint7_interrupt,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,"eint7"},
//{IRQ_EXT7_I2,nuc970_eint7_interrupt,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,"eint7"},
{0,0,0,0}
};

static int nuc970_enable_eint(uint32_t flag,struct platform_device *pdev){
	int err;
	struct nuc970_eint_pins *peint;
	struct pinctrl *p = NULL;
	switch(pdev->id)
	{
		case 1:
			peint=eint0;
			while(peint->pin!=(u32)0){
				switch(peint->pin){
					case IRQ_EXT0_H0:
						p = devm_pinctrl_get_select(&pdev->dev, "eint0-PH");
					break;
					case IRQ_EXT0_F11:
						p = devm_pinctrl_get_select(&pdev->dev, "eint0-PF");
					break;
				}
				if (IS_ERR(p))
				{
					dev_err(&pdev->dev, "unable to reserve pin\n");
					return PTR_ERR(p);
				}
				if ((err = request_irq(peint->pin,peint->handler, peint->trigger|IRQF_NO_SUSPEND, peint->name, 0)) != 0) {
					printk("register %s irq failed %d\n",peint->name ,err);
				}
				if(flag==1){
					__raw_writel((1<<0) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(peint->pin);
				}
				peint++;
		}
		break;
		case 2:
			peint=eint1;
			while(peint->pin!=(u32)0){
				switch(peint->pin){
					case IRQ_EXT1_H1:
						p = devm_pinctrl_get_select(&pdev->dev, "eint1-PH");
					break;
					case IRQ_EXT1_F12:
						p = devm_pinctrl_get_select(&pdev->dev, "eint1-PF");
					break;
				}
				if (IS_ERR(p))
				{
					dev_err(&pdev->dev, "unable to reserve pin\n");
					return PTR_ERR(p);
				}
				if ((err = request_irq(peint->pin,peint->handler, peint->trigger|IRQF_NO_SUSPEND, peint->name, 0)) != 0) {
					printk("register %s irq failed %d\n",peint->name ,err);
				}
				if(flag==1){
					__raw_writel((1<<1) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(peint->pin);
				}
				peint++;
		}
		break;
		case 3:
			peint=eint2;
			while(peint->pin!=(u32)0){
				switch(peint->pin){
					case IRQ_EXT2_H2:
						p = devm_pinctrl_get_select(&pdev->dev, "eint2-PH");
					break;
					case IRQ_EXT2_F13:
						p = devm_pinctrl_get_select(&pdev->dev, "eint2-PF");
					break;
				}
				if (IS_ERR(p))
				{
					dev_err(&pdev->dev, "unable to reserve pin\n");
					return PTR_ERR(p);
				}
				if ((err = request_irq(peint->pin,peint->handler, peint->trigger|IRQF_NO_SUSPEND, peint->name, 0)) != 0) {
					printk("register %s irq failed %d\n",peint->name ,err);
				}
				if(flag==1){
					__raw_writel((1<<2) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(peint->pin);
				}
				peint++;
		}
		break;
		case 4:
			peint=eint3;
			while(peint->pin!=(u32)0){
				switch(peint->pin){
					case IRQ_EXT3_H3:
						p = devm_pinctrl_get_select(&pdev->dev, "eint3-PH");
					break;
					case IRQ_EXT3_F14:
						p = devm_pinctrl_get_select(&pdev->dev, "eint3-PF");
					break;
				}
				if (IS_ERR(p))
				{
					dev_err(&pdev->dev, "unable to reserve pin\n");
					return PTR_ERR(p);
				}
				if ((err = request_irq(peint->pin,peint->handler, peint->trigger|IRQF_NO_SUSPEND, peint->name, 0)) != 0) {
					printk("register %s irq failed %d\n",peint->name ,err);
				}
				if(flag==1){
					__raw_writel((1<<3) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(peint->pin);
				}
				peint++;
		}
		break;
		case 5:
			peint=eint4;
			while(peint->pin!=(u32)0){
				switch(peint->pin){
					case IRQ_EXT4_H4:
						p = devm_pinctrl_get_select(&pdev->dev, "eint4-PH");
					break;
					case IRQ_EXT4_F15:
						p = devm_pinctrl_get_select(&pdev->dev, "eint4-PF");
					break;
				}
				if (IS_ERR(p))
				{
					dev_err(&pdev->dev, "unable to reserve pin\n");
					return PTR_ERR(p);
				}
				if ((err = request_irq(peint->pin,peint->handler, peint->trigger|IRQF_NO_SUSPEND, peint->name, 0)) != 0) {
					printk("register %s irq failed %d\n",peint->name ,err);
				}
				if(flag==1){
					__raw_writel((1<<4) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(peint->pin);
				}
				peint++;
		}
		break;
		case 6:
			peint=eint5;
			while(peint->pin!=(u32)0){
				switch(peint->pin){
					case IRQ_EXT5_H5:
						p = devm_pinctrl_get_select(&pdev->dev, "eint5-PH");
					break;
					case IRQ_EXT5_G15:
						p = devm_pinctrl_get_select(&pdev->dev, "eint5-PG");
					break;
				}
				if (IS_ERR(p))
				{
					dev_err(&pdev->dev, "unable to reserve pin\n");
					return PTR_ERR(p);
				}
				if ((err = request_irq(peint->pin,peint->handler, peint->trigger|IRQF_NO_SUSPEND, peint->name, 0)) != 0) {
					printk("register %s irq failed %d\n",peint->name ,err);
				}
				if(flag==1){
					__raw_writel((1<<5) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(peint->pin);
				}
				peint++;
		}
		break;
		case 7:
			peint=eint6;
			while(peint->pin!=(u32)0){
				switch(peint->pin){
					case IRQ_EXT6_H6:
						p = devm_pinctrl_get_select(&pdev->dev, "eint6-PH");
					break;
					case IRQ_EXT6_I1:
						p = devm_pinctrl_get_select(&pdev->dev, "eint6-PI");
					break;
				}
				if (IS_ERR(p))
				{
					dev_err(&pdev->dev, "unable to reserve pin\n");
					return PTR_ERR(p);
				}
				if ((err = request_irq(peint->pin,peint->handler, peint->trigger|IRQF_NO_SUSPEND, peint->name, 0)) != 0) {
					printk("register %s irq failed %d\n",peint->name ,err);
				}
				if(flag==1){
					__raw_writel((1<<6) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(peint->pin);
				}
				peint++;
		}
		break;
		case 8:
			peint=eint7;
			while(peint->pin!=(u32)0){
				switch(peint->pin){
					case IRQ_EXT7_H7:
						p = devm_pinctrl_get_select(&pdev->dev, "eint7-PH");
					break;
					case IRQ_EXT7_I2:
						p = devm_pinctrl_get_select(&pdev->dev, "eint7-PI");
					break;
				}
				if (IS_ERR(p))
				{
					dev_err(&pdev->dev, "unable to reserve pin\n");
					return PTR_ERR(p);
				}
				if ((err = request_irq(peint->pin,peint->handler, peint->trigger|IRQF_NO_SUSPEND, peint->name, 0)) != 0) {
					printk("register %s irq failed %d\n",peint->name ,err);
				}
				if(flag==1){
					__raw_writel((1<<7) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(IRQ_EXT7_H7);
				}
				peint++;
		}
		break;
	}
	return 0;
}
#else

static irqreturn_t nuc970_eint0_interrupt(int irq, void *dev_id){
	printk("@0\n");
	return IRQ_HANDLED;
}
__attribute__ ((unused)) static irqreturn_t nuc970_eint1_interrupt(int irq, void *dev_id){
	printk("@1\n");
	return IRQ_HANDLED;
}
__attribute__ ((unused)) static irqreturn_t nuc970_eint2_interrupt(int irq, void *dev_id){
	printk("@2\n");
	return IRQ_HANDLED;
}
__attribute__ ((unused)) static irqreturn_t nuc970_eint3_interrupt(int irq, void *dev_id){
	printk("@3\n");
	return IRQ_HANDLED;
}
__attribute__ ((unused)) static irqreturn_t nuc970_eint4_interrupt(int irq, void *dev_id){
	printk("@4\n");
	return IRQ_HANDLED;
}
__attribute__ ((unused)) static irqreturn_t nuc970_eint5_interrupt(int irq, void *dev_id){
	printk("@5\n");
	return IRQ_HANDLED;
}
__attribute__ ((unused)) static irqreturn_t nuc970_eint6_interrupt(int irq, void *dev_id){
	printk("@6\n");
	return IRQ_HANDLED;
}
__attribute__ ((unused)) static irqreturn_t nuc970_eint7_interrupt(int irq, void *dev_id){
	printk("@7\n");
	return IRQ_HANDLED;
}

u32 trigger_type[5]={	(IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING),
												IRQF_TRIGGER_RISING,
												IRQF_TRIGGER_FALLING,
												IRQF_TRIGGER_HIGH,
												IRQF_TRIGGER_LOW};

static int nuc970_enable_eint(uint32_t flag,struct platform_device *pdev){
	int err;
	u32	val32[3];
	u32 irqnum,irqflag;

	//eint 0
	if (of_property_read_u32_array(pdev->dev.of_node, "eint0-config", val32, 3) != 0){
		printk("%s - eint0 can not get port-number!\n", __func__);
		return -EINVAL;
	}
	if(val32[0]==1)
	{
		irqnum=(val32[1]==0)?(IRQ_EXT0_H0):(IRQ_EXT0_F11);
		irqflag=trigger_type[val32[2]]|IRQF_NO_SUSPEND;
		if(flag==1){
					__raw_writel((1<<0) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(irqnum);
		}
		if ((err = request_irq(irqnum,nuc970_eint0_interrupt,irqflag, "eint0", 0)) != 0) {
			printk("%s - eint0 can not get irq!\n", __func__);
			return -EINVAL;
		}
	}

	//eint 1
	if (of_property_read_u32_array(pdev->dev.of_node, "eint1-config", val32, 3) != 0){
		printk("%s - eint1 can not get port-number!\n", __func__);
		return -EINVAL;
	}
	if(val32[0]==1)
	{
		irqnum=(val32[1]==0)?(IRQ_EXT1_H1):(IRQ_EXT1_F12);
		irqflag=trigger_type[val32[2]]|IRQF_NO_SUSPEND;
		if(flag==1){
					__raw_writel((1<<1) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(irqnum);
		}
		if ((err = request_irq(irqnum,nuc970_eint1_interrupt,irqflag, "eint1", 0)) != 0) {
			printk("%s - eint1 can not get irq!\n", __func__);
			return -EINVAL;
		}
	}

	//eint 2
	if (of_property_read_u32_array(pdev->dev.of_node, "eint2-config", val32, 3) != 0){
		printk("%s - eint2 can not get port-number!\n", __func__);
		return -EINVAL;
	}
	if(val32[0]==1)
	{
		irqnum=(val32[1]==0)?(IRQ_EXT2_H2):(IRQ_EXT2_F13);
		irqflag=trigger_type[val32[2]]|IRQF_NO_SUSPEND;
		if(flag==1){
					__raw_writel((1<<2) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(irqnum);
		}
		if ((err = request_irq(irqnum,nuc970_eint2_interrupt,irqflag, "eint2", 0)) != 0) {
			printk("%s - eint2 can not get irq!\n", __func__);
			return -EINVAL;
		}
	}

	//eint 3
	if (of_property_read_u32_array(pdev->dev.of_node, "eint3-config", val32, 3) != 0){
		printk("%s - eint3 can not get port-number!\n", __func__);
		return -EINVAL;
	}
	if(val32[0]==1)
	{
		irqnum=(val32[1]==0)?(IRQ_EXT3_H3):(IRQ_EXT3_F14);
		irqflag=trigger_type[val32[2]]|IRQF_NO_SUSPEND;
		if(flag==3){
					__raw_writel((1<<30) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(irqnum);
		}
		if ((err = request_irq(irqnum,nuc970_eint3_interrupt,irqflag, "eint3", 0)) != 0) {
			printk("%s - eint3 can not get irq!\n", __func__);
			return -EINVAL;
		}
	}

	//eint 4
	if (of_property_read_u32_array(pdev->dev.of_node, "eint4-config", val32, 3) != 0){
		printk("%s - eint4 can not get port-number!\n", __func__);
		return -EINVAL;
	}
	if(val32[0]==1)
	{
		irqnum=(val32[1]==0)?(IRQ_EXT4_H4):(IRQ_EXT4_F15);
		irqflag=trigger_type[val32[2]]|IRQF_NO_SUSPEND;
		if(flag==1){
					__raw_writel((1<<4) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(irqnum);
		}
		if(flag==1){
					__raw_writel((1<<4) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(irqnum);
		}
		if ((err = request_irq(irqnum,nuc970_eint4_interrupt,irqflag, "eint4", 0)) != 0) {
			printk("%s - eint4 can not get irq!\n", __func__);
			return -EINVAL;
		}
	}

	//eint 5
	if (of_property_read_u32_array(pdev->dev.of_node, "eint5-config", val32, 3) != 0){
		printk("%s - eint5 can not get port-number!\n", __func__);
		return -EINVAL;
	}
	if(val32[0]==1)
	{
		irqnum=(val32[1]==0)?(IRQ_EXT5_H5):(IRQ_EXT5_G15);
		irqflag=trigger_type[val32[2]]|IRQF_NO_SUSPEND;
		if(flag==1){
					__raw_writel((1<<5) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(irqnum);
		}
		if ((err = request_irq(irqnum,nuc970_eint5_interrupt,irqflag, "eint5", 0)) != 0) {
			printk("%s - eint5 can not get irq!\n", __func__);
			return -EINVAL;
		}
	}

	//eint 6
	if (of_property_read_u32_array(pdev->dev.of_node, "eint6-config", val32, 3) != 0){
		printk("%s - eint6 can not get port-number!\n", __func__);
		return -EINVAL;
	}
	if(val32[0]==1)
	{
		irqnum=(val32[1]==0)?(IRQ_EXT6_H6):(IRQ_EXT6_I1);
		irqflag=trigger_type[val32[2]]|IRQF_NO_SUSPEND;
		if(flag==1){
					__raw_writel((1<<6) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(irqnum);
		}
		if ((err = request_irq(irqnum,nuc970_eint6_interrupt,irqflag, "eint6", 0)) != 0) {
			printk("%s - eint6 can not get irq!\n", __func__);
			return -EINVAL;
		}
	}

	//eint 7
	if (of_property_read_u32_array(pdev->dev.of_node, "eint7-config", val32, 3) != 0){
		printk("%s - eint7 can not get port-number!\n", __func__);
		return -EINVAL;
	}
	if(val32[0]==1)
	{
		irqnum=(val32[1]==0)?(IRQ_EXT7_H7):(IRQ_EXT7_I2);
		irqflag=trigger_type[val32[2]]|IRQF_NO_SUSPEND;
		if(flag==1){
					__raw_writel((1<<7) | __raw_readl(REG_WKUPSER),REG_WKUPSER);
					enable_irq_wake(irqnum);
		}
		if ((err = request_irq(irqnum,nuc970_eint7_interrupt,irqflag, "eint7", 0)) != 0) {
			printk("%s - eint7 can not get irq!\n", __func__);
			return -EINVAL;
		}
	}
	return 0;
}
#endif

static int nuc970_gpio_probe(struct platform_device *pdev)
{
	int err;
	struct clk *clk;

	//printk("%s - pdev = %s\n", __func__, pdev->name);
#ifndef CONFIG_OF
	if(pdev->id == 0)
#endif
	{

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

	}

#ifdef CONFIG_OF
	{
		struct pinctrl *pinctrl;
		pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
		if (IS_ERR(pinctrl)) {
			return PTR_ERR(pinctrl);
		}
	}
#endif
	#ifdef CONFIG_GPIO_NUC970_EINT_WKUP
		nuc970_enable_eint(1,pdev);
	#else
		nuc970_enable_eint(0,pdev);
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

static int kpi_suspend_flag = 0;
static int kpi_resume_flag = 0;

static int nuc970_gpio_resume(struct platform_device *pdev){
#if defined CONFIG_NUC970_KEYPAD_PH
	int i;
#endif

	ENTRY();

#if defined CONFIG_NUC970_KEYPAD_PH
	if(kpi_resume_flag == 0)
	{
		__raw_writel(__raw_readl(REG_WKUPSER)& ~(1 << 25),REG_WKUPSER);

		for(i = 0; i < NUC970_KPD_ROW_NUMBER; i++)
		{
			disable_irq_nosync(gpio_to_irq(NUC970_PH4+i));
		}

		// Set Column
		writel(readl(REG_GPIOH_DIR) & ~( ((1 << NUC970_KPD_COL_NUMBER) - 1) << 8), REG_GPIOH_DIR); // input
		writel(readl(REG_GPIOH_PUEN) | ( ((1 << NUC970_KPD_COL_NUMBER) - 1) << 8), REG_GPIOH_PUEN); // pull-up

		// Set Row
		writel(readl(REG_GPIOH_DIR) | ( ((1 << NUC970_KPD_ROW_NUMBER) - 1) << 4), REG_GPIOH_DIR);  // output
		writel(readl(REG_GPIOH_PUEN) | ( ((1 << NUC970_KPD_ROW_NUMBER) - 1) << 4), REG_GPIOH_PUEN); // pull up
		writel(readl(REG_GPIOH_DATAOUT) & ~( ((1 << NUC970_KPD_ROW_NUMBER) - 1) << 4), REG_GPIOH_DATAOUT); // low

		// clear ISR
		writel(readl(REG_GPIOH_ISR), REG_GPIOH_ISR);

		for(i = 0; i < NUC970_KPD_COL_NUMBER; i++)
		{
			enable_irq(gpio_to_irq(NUC970_PH8+i));
		}

		kpi_suspend_flag = 0;
		kpi_resume_flag = 1;
	}

#endif
	LEAVE();
	return 0;
}

static int nuc970_gpio_suspend(struct platform_device *pdev,pm_message_t state){
#if defined CONFIG_NUC970_KEYPAD_PH
	int i;
#endif
	ENTRY();
#if defined CONFIG_NUC970_KEYPAD_PH
	if(kpi_suspend_flag == 0)
	{
		for(i = 0; i < NUC970_KPD_COL_NUMBER; i++)
		{
			disable_irq_nosync(gpio_to_irq(NUC970_PH8+i));
		}

		// Set Row
		writel(readl(REG_GPIOH_DIR) & ~( ((1 << NUC970_KPD_COL_NUMBER) - 1) << 4), REG_GPIOH_DIR); // input
		writel(readl(REG_GPIOH_PUEN) | ( ((1 << NUC970_KPD_COL_NUMBER) - 1) << 4), REG_GPIOH_PUEN); // pull-up

		// Set Column
		writel(readl(REG_GPIOH_DIR) | ( ((1 << NUC970_KPD_ROW_NUMBER) - 1) << 8), REG_GPIOH_DIR);  // output
		writel(readl(REG_GPIOH_PUEN) | ( ((1 << NUC970_KPD_ROW_NUMBER) - 1) << 8), REG_GPIOH_PUEN); // pull up
		writel(readl(REG_GPIOH_DATAOUT) & ~( ((1 << NUC970_KPD_ROW_NUMBER) - 1) << 8), REG_GPIOH_DATAOUT); // low

		// clear ISR
		writel(readl(REG_GPIOH_ISR), REG_GPIOH_ISR);

		__raw_writel(__raw_readl(REG_WKUPSER)| (1 << 25),REG_WKUPSER);

		for(i = 0; i < NUC970_KPD_ROW_NUMBER; i++)
		{
			enable_irq(gpio_to_irq(NUC970_PH4+i));
		}

		kpi_suspend_flag = 1;
		kpi_resume_flag = 0;
	}
#endif
	LEAVE();
	return 0;
}

static const struct of_device_id nuc970_gpio_of_match[] = {
	{ .compatible = "nuvoton,nuc970-gpio" },
	{},
};
MODULE_DEVICE_TABLE(of, nuc970_gpio_of_match);

static struct platform_driver nuc970_gpio_driver = {
	.probe		= nuc970_gpio_probe,
	.remove		= nuc970_gpio_remove,
	.resume		= nuc970_gpio_resume,
	.suspend	= nuc970_gpio_suspend,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(nuc970_gpio_of_match),
	},
};
module_platform_driver(nuc970_gpio_driver);

MODULE_AUTHOR("shan chun <SCChung@nuvoton.com>");
MODULE_DESCRIPTION("GPIO interface for Nuvoton NUC970/N9H30 GPIO Drive");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nuc970_gpio");
