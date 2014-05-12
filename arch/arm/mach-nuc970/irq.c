/*
 * linux/arch/arm/mach-nuc970/irq.c
 *
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
#include <mach/regs-irq.h>



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

void __init nuc970_init_irq(void)
{
	int irqno;

	__raw_writel(0xFFFFFFFC, REG_AIC_MDCR);
	__raw_writel(0xFFFFFFFF, REG_AIC_MDCRH);

	for (irqno = IRQ_WDT; irqno < NR_IRQS; irqno++) {
		irq_set_chip_and_handler(irqno, &nuc970_irq_chip, handle_level_irq);
		set_irq_flags(irqno, IRQF_VALID);
	}
}
