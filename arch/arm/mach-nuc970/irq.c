/*
 * linux/arch/arm/mach-nuc970/irq.c
 *
 * based on linux/arch/arm/sa1100/irq.c
 *
 * Copyright (c) 2014 Nuvoton technology corporation
 * All rights reserved.
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/ptrace.h>
#include <linux/device.h>
#include <linux/io.h>

#include <asm/irq.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/regs-aic.h>
#include <mach/regs-gpio.h>
#include <asm/gpio.h>

static void nuc970_irq_mask(struct irq_data *d)
{
	if(d->irq < 32)
		__raw_writel(1 << (d->irq), REG_AIC_MDCR);
	else
		__raw_writel(1 << (d->irq - 32), REG_AIC_MDCRH);	
}


static void nuc970_irq_ack(struct irq_data *d)
{
	__raw_writel(0x01, REG_AIC_EOSCR);
}

static void nuc970_irq_unmask(struct irq_data *d)
{
	if(d->irq < 32)
		__raw_writel(1 << (d->irq), REG_AIC_MECR);
	else
		__raw_writel(1 << (d->irq - 32), REG_AIC_MECRH);	
}

static struct irq_chip nuc970_irq_chip = {
	.irq_ack	= nuc970_irq_ack,
	.irq_mask	= nuc970_irq_mask,
	.irq_unmask	= nuc970_irq_unmask,
};

#ifdef CONFIG_GPIO_NUC970
static const unsigned int Port[10]={
				(unsigned int)REG_GPIOA_DIR,
				(unsigned int)REG_GPIOB_DIR,
				(unsigned int)REG_GPIOC_DIR,
				(unsigned int)REG_GPIOD_DIR,
				(unsigned int)REG_GPIOE_DIR,
				(unsigned int)REG_GPIOF_DIR,
				(unsigned int)REG_GPIOG_DIR,
				(unsigned int)REG_GPIOH_DIR,
				(unsigned int)REG_GPIOI_DIR,
				(unsigned int)REG_GPIOJ_DIR};

static void nuc970_irq_gpio_mask(struct irq_data *d)
{
	__raw_writel(1 << (IRQ_GPIO - 32), REG_AIC_MDCRH);
}


static void nuc970_irq_gpio_ack(struct irq_data *d)
{
	__raw_writel(0x01, REG_AIC_EOSCR);
}

static void nuc970_irq_gpio_unmask(struct irq_data *d)
{
	unsigned int port,num;
	port =(d->irq-IRQ_GPIO_START)/GPIO_OFFSET;
	num  =(d->irq-IRQ_GPIO_START)%GPIO_OFFSET;
	__raw_writel(0x1<<num,(volatile unsigned int *)(Port[port]+0x18));
	__raw_writel(1 << (IRQ_GPIO- 32), REG_AIC_MECRH);
}

static int nuc970_irq_gpio_type(struct irq_data *d, unsigned int type)
{
	unsigned int port,num;
	port =(d->irq-IRQ_GPIO_START)/GPIO_OFFSET;
	num  =(d->irq-IRQ_GPIO_START)%GPIO_OFFSET;

	if (type == IRQ_TYPE_PROBE) {
		__raw_writel(__raw_readl((volatile unsigned int *)(Port[port]+0x10)) |(0x1<<num),(volatile unsigned int *)(Port[port]+0x10));
		__raw_writel(__raw_readl((volatile unsigned int *)(Port[port]+0x14)) |(0x1<<num),(volatile unsigned int *)(Port[port]+0x14));
		return 0;
	}

	if (type & IRQ_TYPE_EDGE_RISING)
	{
		__raw_writel(__raw_readl((volatile unsigned int *)(Port[port]+0x10)) | (0x1<<num),(volatile unsigned int *)(Port[port]+0x10));
	}else
		__raw_writel(__raw_readl((volatile unsigned int *)(Port[port]+0x10)) &~(0x1<<num),(volatile unsigned int *)(Port[port]+0x10));

	if (type & IRQ_TYPE_EDGE_FALLING){
		__raw_writel(__raw_readl((volatile unsigned int *)(Port[port]+0x14)) | (0x1<<num),(volatile unsigned int *)(Port[port]+0x14));
	}else
		__raw_writel(__raw_readl((volatile unsigned int *)(Port[port]+0x14)) &~(0x1<<num),(volatile unsigned int *)(Port[port]+0x14));

	return 0;
}

static struct irq_chip nuc970_irq_gpio = {
	.name		= "GPIO-IRQ",
	.irq_ack	= nuc970_irq_gpio_ack,
	.irq_mask	= nuc970_irq_gpio_mask,
	.irq_unmask	= nuc970_irq_gpio_unmask,
	.irq_set_type	= nuc970_irq_gpio_type,
};

static void nuc970_irq_demux_intgroup(unsigned int irq,
			struct irq_desc *desc)
{
	unsigned int i,j,isr,sub_isr;
	isr=__raw_readl(REG_GPIO_ISR);
	for(i=0;i<10;i++)
	{
		if(isr & 0x1)
		{
			sub_isr=__raw_readl((volatile unsigned int *)(Port[i]+0x18));
			for(j=0;j<16;j++)
			{
				if(sub_isr & 0x1)
				{
					generic_handle_irq(IRQ_GPIO_START+i*0x20+j);
				}
				sub_isr=sub_isr>>1;
			}
		}
		isr=isr>>1;
	}
}
//------------------------------------------------------------------------------

static const unsigned int EXT[16]={
				(unsigned int)NUC970_PH0,
				(unsigned int)NUC970_PH1,
				(unsigned int)NUC970_PH2,
				(unsigned int)NUC970_PH3,
				(unsigned int)NUC970_PH4,
				(unsigned int)NUC970_PH5,
				(unsigned int)NUC970_PH6,
				(unsigned int)NUC970_PH7,
				(unsigned int)NUC970_PF11,
				(unsigned int)NUC970_PF12,
				(unsigned int)NUC970_PF13,
				(unsigned int)NUC970_PF14,
				(unsigned int)NUC970_PF15,
				(unsigned int)NUC970_PG15,
				(unsigned int)NUC970_PI1,
				(unsigned int)NUC970_PI2,
};

static void nuc970_irq_ext_mask(struct irq_data *d)
{
	if(d->irq==IRQ_EXT0_H0 || d->irq==IRQ_EXT0_F11)
		__raw_writel(1<<IRQ_EXT0, REG_AIC_MDCR);
	else if(d->irq==IRQ_EXT1_H1 || d->irq==IRQ_EXT1_F12)
		__raw_writel(1 <<IRQ_EXT1, REG_AIC_MDCR);
	else if(d->irq==IRQ_EXT2_H2 || d->irq==IRQ_EXT2_F13)
		__raw_writel(1 <<IRQ_EXT2, REG_AIC_MDCR);
	else if(d->irq==IRQ_EXT3_H3 || d->irq==IRQ_EXT3_F14)
		__raw_writel(1 <<IRQ_EXT3, REG_AIC_MDCR);
	else if(d->irq==IRQ_EXT4_H4 || d->irq==IRQ_EXT4_F15)
		__raw_writel(1 <<IRQ_EXT4, REG_AIC_MDCR);
	else if(d->irq==IRQ_EXT5_H5 || d->irq==IRQ_EXT5_G15)
		__raw_writel(1 <<IRQ_EXT5, REG_AIC_MDCR);
	else if(d->irq==IRQ_EXT6_H6 || d->irq==IRQ_EXT6_I1)
		__raw_writel(1 <<IRQ_EXT6, REG_AIC_MDCR);
	else if(d->irq==IRQ_EXT7_H7 || d->irq==IRQ_EXT7_I2)
		__raw_writel(1 <<IRQ_EXT7, REG_AIC_MDCR);
}

static void nuc970_irq_ext_ack(struct irq_data *d)
{
	__raw_writel(0x01, REG_AIC_EOSCR);
}

static void nuc970_irq_ext_unmask(struct irq_data *d)
{
	if(d->irq==IRQ_EXT0_H0 || d->irq==IRQ_EXT0_F11)
		__raw_writel(1 <<IRQ_EXT0, REG_AIC_MECR);
	else if(d->irq==IRQ_EXT1_H1 || d->irq==IRQ_EXT1_F12)
		__raw_writel(1 <<IRQ_EXT1, REG_AIC_MECR);
	else if(d->irq==IRQ_EXT2_H2 || d->irq==IRQ_EXT2_F13)
		__raw_writel(1 <<IRQ_EXT2, REG_AIC_MECR);
	else if(d->irq==IRQ_EXT3_H3 || d->irq==IRQ_EXT3_F14)
		__raw_writel(1 <<IRQ_EXT3, REG_AIC_MECR);
	else if(d->irq==IRQ_EXT4_H4 || d->irq==IRQ_EXT4_F15)
		__raw_writel(1 <<IRQ_EXT4, REG_AIC_MECR);
	else if(d->irq==IRQ_EXT5_H5 || d->irq==IRQ_EXT5_G15)
		__raw_writel(1 <<IRQ_EXT5, REG_AIC_MECR);
	else if(d->irq==IRQ_EXT6_H6 || d->irq==IRQ_EXT6_I1)
		__raw_writel(1 <<IRQ_EXT6, REG_AIC_MECR);
	else if(d->irq==IRQ_EXT7_H7 || d->irq==IRQ_EXT7_I2)
		__raw_writel(1 <<IRQ_EXT7, REG_AIC_MECR);
}

static int nuc970_irq_ext_type(struct irq_data *d, unsigned int type)
{
	unsigned int port,num;
	port =(EXT[d->irq-EXT0_BASE])/GPIO_OFFSET;
	num  =(EXT[d->irq-EXT0_BASE])%GPIO_OFFSET;
	if (type == IRQ_TYPE_PROBE) {
		__raw_writel(__raw_readl((volatile unsigned int *)(Port[port]+0x10)) |(0x1<<num),(volatile unsigned int *)(Port[port]+0x10));
		__raw_writel(__raw_readl((volatile unsigned int *)(Port[port]+0x14)) |(0x1<<num),(volatile unsigned int *)(Port[port]+0x14));
		return 0;
	}

	if (type & IRQ_TYPE_EDGE_RISING)
	{
		__raw_writel(__raw_readl((volatile unsigned int *)(Port[port]+0x10)) | (0x1<<num),(volatile unsigned int *)(Port[port]+0x10));
	}else
		__raw_writel(__raw_readl((volatile unsigned int *)(Port[port]+0x10)) & ~(0x1<<num),(volatile unsigned int *)(Port[port]+0x10));

	if (type & IRQ_TYPE_EDGE_FALLING){
		__raw_writel(__raw_readl((volatile unsigned int *)(Port[port]+0x14)) | (0x1<<num),(volatile unsigned int *)(Port[port]+0x14));
	}else
		__raw_writel(__raw_readl((volatile unsigned int *)(Port[port]+0x14)) & ~(0x1<<num),(volatile unsigned int *)(Port[port]+0x14));

	return 0;
}

static struct irq_chip nuc970_irq_ext = {
	.name		= "EXT-IRQ",
	.irq_ack	= nuc970_irq_ext_ack,
	.irq_mask	= nuc970_irq_ext_mask,
	.irq_unmask	= nuc970_irq_ext_unmask,
	.irq_set_type	= nuc970_irq_ext_type,
};

static void nuc970_irq_demux_intgroup2(unsigned int irq,
			struct irq_desc *desc)
{
	unsigned int port0,num0,port1,num1;
	port0= EXT[irq-4]/GPIO_OFFSET;
	num0 = EXT[irq-4]%GPIO_OFFSET;
	port1= EXT[irq-4+8]/GPIO_OFFSET;
	num1 = EXT[irq-4+8]%GPIO_OFFSET;
	switch(irq)
	{
		case IRQ_EXT0:
			if(__raw_readl((volatile unsigned int *)(Port[port0]+0x18)) & (1<<num0))
			{
				generic_handle_irq(IRQ_EXT0_H0);
				__raw_writel(0x1<<num0,(volatile unsigned int *)(Port[port0]+0x18));
			}
			else if(__raw_readl((volatile unsigned int *)(Port[port1]+0x18)) & (1<<num1))
			{
				generic_handle_irq(IRQ_EXT0_F11);
				__raw_writel(0x1<<num1,(volatile unsigned int *)(Port[port1]+0x18));
			}
			break;
		case IRQ_EXT1:
			if(__raw_readl((volatile unsigned int *)(Port[port0]+0x18)) & (1<<num0))
			{
				generic_handle_irq(IRQ_EXT1_H1);
				__raw_writel(0x1<<num0,(volatile unsigned int *)(Port[port0]+0x18));
			}
			else if(__raw_readl((volatile unsigned int *)(Port[port1]+0x18)) & (1<<num1))
			{
				generic_handle_irq(IRQ_EXT1_F12);
				__raw_writel(0x1<<num1,(volatile unsigned int *)(Port[port1]+0x18));
			}
			break;
		case IRQ_EXT2:
			if(__raw_readl((volatile unsigned int *)(Port[port0]+0x18)) & (1<<num0))
			{
				generic_handle_irq(IRQ_EXT2_H2);
				__raw_writel(0x1<<num0,(volatile unsigned int *)(Port[port0]+0x18));
			}
			else if(__raw_readl((volatile unsigned int *)(Port[port1]+0x18)) & (1<<num1))
			{
				generic_handle_irq(IRQ_EXT2_F13);
				__raw_writel(0x1<<num1,(volatile unsigned int *)(Port[port1]+0x18));
			}
			break;
		case IRQ_EXT3:
			if(__raw_readl((volatile unsigned int *)(Port[port0]+0x18)) & (1<<num0))
			{
				generic_handle_irq(IRQ_EXT3_H3);
				__raw_writel(0x1<<num0,(volatile unsigned int *)(Port[port0]+0x18));
			}
			else if(__raw_readl((volatile unsigned int *)(Port[port1]+0x18)) & (1<<num1))
			{
				generic_handle_irq(IRQ_EXT3_F14);
				__raw_writel(0x1<<num1,(volatile unsigned int *)(Port[port1]+0x18));
			}
			break;
		case IRQ_EXT4:
			if(__raw_readl((volatile unsigned int *)(Port[port0]+0x18)) & (1<<num0))
			{
				generic_handle_irq(IRQ_EXT4_H4);
				__raw_writel(0x1<<num0,(volatile unsigned int *)(Port[port0]+0x18));
			}
			else if(__raw_readl((volatile unsigned int *)(Port[port1]+0x18)) & (1<<num1))
			{
				generic_handle_irq(IRQ_EXT4_F15);
				__raw_writel(0x1<<num1,(volatile unsigned int *)(Port[port1]+0x18));
			}
			break;
		case IRQ_EXT5:
			if(__raw_readl((volatile unsigned int *)(Port[port0]+0x18)) & (1<<num0))
			{
				generic_handle_irq(IRQ_EXT5_H5);
				__raw_writel(0x1<<num0,(volatile unsigned int *)(Port[port0]+0x18));
			}
			else if(__raw_readl((volatile unsigned int *)(Port[port1]+0x18)) & (1<<num1))
			{
				generic_handle_irq(IRQ_EXT5_G15);
				__raw_writel(0x1<<num1,(volatile unsigned int *)(Port[port1]+0x18));
			}
			break;
		case IRQ_EXT6:
			if(__raw_readl((volatile unsigned int *)(Port[port0]+0x18)) & (1<<num0))
			{
				generic_handle_irq(IRQ_EXT6_H6);
				__raw_writel(0x1<<num0,(volatile unsigned int *)(Port[port0]+0x18));
			}
			else if(__raw_readl((volatile unsigned int *)(Port[port1]+0x18)) & (1<<num1))
			{
				generic_handle_irq(IRQ_EXT6_I1);
				__raw_writel(0x1<<num1,(volatile unsigned int *)(Port[port1]+0x18));
			}
			break;
		case IRQ_EXT7:
			if(__raw_readl((volatile unsigned int *)(Port[port0]+0x18)) & (1<<num0))
			{
				generic_handle_irq(IRQ_EXT7_H7);
				__raw_writel(0x1<<num0,(volatile unsigned int *)(Port[port0]+0x18));
			}
			else if(__raw_readl((volatile unsigned int *)(Port[port1]+0x18)) & (1<<num1))
			{
				generic_handle_irq(IRQ_EXT7_I2);
				__raw_writel(0x1<<num1,(volatile unsigned int *)(Port[port1]+0x18));
			}
			break;
	}
}
#endif

void __init nuc970_init_irq(void)
{
	int irqno;

	__raw_writel(0xFFFFFFFC, REG_AIC_MDCR);
	__raw_writel(0xFFFFFFFF, REG_AIC_MDCRH);

	for (irqno = IRQ_WDT; irqno < NR_IRQS; irqno++) {
		irq_set_chip_and_handler(irqno, &nuc970_irq_chip, handle_level_irq);
		set_irq_flags(irqno, IRQF_VALID);
	}

	#ifdef CONFIG_GPIO_NUC970
	/*
	 * Install handler for GPIO edge detect interrupts
	 */
		irq_set_chip(IRQ_GPIO, &nuc970_irq_chip);
		irq_set_chained_handler(IRQ_GPIO, nuc970_irq_demux_intgroup);

		for (irqno = IRQ_GPIO_START; irqno < IRQ_GPIO_END; irqno++) {
			irq_set_chip_and_handler(irqno, &nuc970_irq_gpio, handle_level_irq);
			set_irq_flags(irqno, IRQF_VALID);
		}

	/*
	 * Install handler for GPIO external interrupts
	 */
	for (irqno = IRQ_EXT0; irqno <= IRQ_EXT7; irqno++) {
		//printk("registering irq %d (extended nuc970 irq)\n", irqno);
		irq_set_chip(irqno, &nuc970_irq_chip);
		irq_set_chained_handler(irqno, nuc970_irq_demux_intgroup2);
	}

	for (irqno = IRQ_EXT0_H0; irqno <= IRQ_EXT7_I2; irqno++) {
			irq_set_chip_and_handler(irqno, &nuc970_irq_ext, handle_level_irq);
			set_irq_flags(irqno, IRQF_VALID);
	}
	#endif
}
