/*
 * linux/arch/arm/mach-nuc970/mfp.c
 *
 * Copyright (c) 2014 Nuvoton technology corporation
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
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/clk.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <mach/map.h>
#include <mach/regs-gcr.h>

static DEFINE_SPINLOCK(mfp_lock);

// TODO: error checking...?

void nuc970_mfp_set_port_a(u32 pin, u32 func)
{
        unsigned long flags;
	spin_lock_irqsave(&mfp_lock, flags);
	if(pin < 8) {
		pin *= 4;
		__raw_writel((__raw_readl(REG_MFP_GPA_L) & ~(0xF << pin)) | (func << pin), REG_MFP_GPA_L);
	} else {
		pin = (pin - 8) * 4;
		__raw_writel((__raw_readl(REG_MFP_GPA_H) & ~(0xF << pin)) | (func << pin), REG_MFP_GPA_H);
	}
	spin_unlock_irqrestore(&mfp_lock, flags);
}
EXPORT_SYMBOL(nuc970_mfp_set_port_a);

void nuc970_mfp_set_port_b(u32 pin, u32 func)
{
        unsigned long flags;
	spin_lock_irqsave(&mfp_lock, flags);
	if(pin < 8) {
		pin *= 4;
		__raw_writel((__raw_readl(REG_MFP_GPB_L) & ~(0xF << pin)) | (func << pin), REG_MFP_GPB_L);
	} else {
		pin = (pin - 8) * 4;
		__raw_writel((__raw_readl(REG_MFP_GPB_H) & ~(0xF << pin)) | (func << pin), REG_MFP_GPB_H);
	}
	spin_unlock_irqrestore(&mfp_lock, flags);
}
EXPORT_SYMBOL(nuc970_mfp_set_port_b);

void nuc970_mfp_set_port_c(u32 pin, u32 func)
{
        unsigned long flags;
	spin_lock_irqsave(&mfp_lock, flags);
	if(pin < 8) {
		pin *= 4;
		__raw_writel((__raw_readl(REG_MFP_GPC_L) & ~(0xF << pin)) | (func << pin), REG_MFP_GPC_L);
	} else {
		pin = (pin - 8) * 4;
		__raw_writel((__raw_readl(REG_MFP_GPC_H) & ~(0xF << pin)) | (func << pin), REG_MFP_GPC_H);
	}
	spin_unlock_irqrestore(&mfp_lock, flags);
}
EXPORT_SYMBOL(nuc970_mfp_set_port_c);

void nuc970_mfp_set_port_d(u32 pin, u32 func)
{
        unsigned long flags;
	spin_lock_irqsave(&mfp_lock, flags);
	if(pin < 8) {
		pin *= 4;
		__raw_writel((__raw_readl(REG_MFP_GPD_L) & ~(0xF << pin)) | (func << pin), REG_MFP_GPD_L);
	} else {
		pin = (pin - 8) * 4;
		__raw_writel((__raw_readl(REG_MFP_GPD_H) & ~(0xF << pin)) | (func << pin), REG_MFP_GPD_H);
	}
	spin_unlock_irqrestore(&mfp_lock, flags);
}
EXPORT_SYMBOL(nuc970_mfp_set_port_d);

void nuc970_mfp_set_port_e(u32 pin, u32 func)
{
        unsigned long flags;
	spin_lock_irqsave(&mfp_lock, flags);
	if(pin < 8) {
		pin *= 4;
		__raw_writel((__raw_readl(REG_MFP_GPE_L) & ~(0xF << pin)) | (func << pin), REG_MFP_GPE_L);
	} else {
		pin = (pin - 8) * 4;
		__raw_writel((__raw_readl(REG_MFP_GPE_H) & ~(0xF << pin)) | (func << pin), REG_MFP_GPE_H);
	}
	spin_unlock_irqrestore(&mfp_lock, flags);
}
EXPORT_SYMBOL(nuc970_mfp_set_port_e);

void nuc970_mfp_set_port_f(u32 pin, u32 func)
{
        unsigned long flags;
	spin_lock_irqsave(&mfp_lock, flags);
	if(pin < 8) {
		pin *= 4;
		__raw_writel((__raw_readl(REG_MFP_GPF_L) & ~(0xF << pin)) | (func << pin), REG_MFP_GPF_L);
	} else {
		pin = (pin - 8) * 4;
		__raw_writel((__raw_readl(REG_MFP_GPF_H) & ~(0xF << pin)) | (func << pin), REG_MFP_GPF_H);
	}
	spin_unlock_irqrestore(&mfp_lock, flags);
}
EXPORT_SYMBOL(nuc970_mfp_set_port_f);


void nuc970_mfp_set_port_g(u32 pin, u32 func)
{
        unsigned long flags;
	spin_lock_irqsave(&mfp_lock, flags);
	if(pin < 8) {
		pin *= 4;
		__raw_writel((__raw_readl(REG_MFP_GPG_L) & ~(0xF << pin)) | (func << pin), REG_MFP_GPG_L);
	} else {
		pin = (pin - 8) * 4;
		__raw_writel((__raw_readl(REG_MFP_GPG_H) & ~(0xF << pin)) | (func << pin), REG_MFP_GPG_H);
	}
	spin_unlock_irqrestore(&mfp_lock, flags);
}
EXPORT_SYMBOL(nuc970_mfp_set_port_g);

void nuc970_mfp_set_port_h(u32 pin, u32 func)
{
        unsigned long flags;
	spin_lock_irqsave(&mfp_lock, flags);
	if(pin < 8) {
		pin *= 4;
		__raw_writel((__raw_readl(REG_MFP_GPH_L) & ~(0xF << pin)) | (func << pin), REG_MFP_GPH_L);
	} else {
		pin = (pin - 8) * 4;
		__raw_writel((__raw_readl(REG_MFP_GPH_H) & ~(0xF << pin)) | (func << pin), REG_MFP_GPH_H);
	}
	spin_unlock_irqrestore(&mfp_lock, flags);
}
EXPORT_SYMBOL(nuc970_mfp_set_port_h);

void nuc970_mfp_set_port_i(u32 pin, u32 func)
{
        unsigned long flags;
	spin_lock_irqsave(&mfp_lock, flags);
	if(pin < 8) {
		pin *= 4;
		__raw_writel((__raw_readl(REG_MFP_GPI_L) & ~(0xF << pin)) | (func << pin), REG_MFP_GPI_L);
	} else {
		pin = (pin - 8) * 4;
		__raw_writel((__raw_readl(REG_MFP_GPI_H) & ~(0xF << pin)) | (func << pin), REG_MFP_GPI_H);
	}
	spin_unlock_irqrestore(&mfp_lock, flags);
}
EXPORT_SYMBOL(nuc970_mfp_set_port_i);


