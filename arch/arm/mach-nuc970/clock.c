/*
 * linux/arch/arm/mach-nuc970/clock.c
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
#include <linux/clkdev.h>

#include <mach/hardware.h>
#include <mach/regs-clock.h>
#include "clock.h"

// TODO: select IP clock source and set rate

void nuc970_hclken(struct clk *clk, int enable);
void nuc970_pclk0en(struct clk *clk, int enable);
void nuc970_pclk1en(struct clk *clk, int enable);
unsigned int nuc970_get_apll(void);
unsigned int nuc970_get_upll(void);
unsigned int nuc970_get_hclk(void);
unsigned int nuc970_get_hclk234(void);
unsigned int nuc970_get_pclk(void);

static DEFINE_SPINLOCK(clk_lock);

/* Initial clock declarations. */
struct clk clk_sram = 	{(1 << 8), 0, 0, nuc970_hclken, NULL, NULL};
struct clk clk_ebi = 	{(1 << 9), 0, 0, nuc970_hclken, NULL, NULL};
struct clk clk_gdma = 	{(1 << 12), 0, 0, nuc970_hclken, NULL, NULL};
struct clk clk_cko = 	{(1 << 15), 0, 0, nuc970_hclken, NULL, NULL};
struct clk clk_emac0 = 	{(1 << 16), 0, 0, nuc970_hclken, NULL, NULL};
struct clk clk_emac1 = 	{(1 << 17), 0, 0, nuc970_hclken, NULL, NULL};
struct clk clk_usbh = 	{(1 << 18), 0, 0, nuc970_hclken, NULL, NULL};
struct clk clk_usbd = 	{(1 << 19), 0, 0, nuc970_hclken, NULL, NULL};
struct clk clk_fmi = 	{(1 << 20), 0, 0, nuc970_hclken, NULL, NULL};
struct clk clk_nand = 	{(1 << 21), 0, 0, nuc970_hclken, NULL, NULL};
struct clk clk_emmc = 	{(1 << 22), 0, 0, nuc970_hclken, NULL, NULL};
struct clk clk_crypto = {(1 << 23), 0, 0, nuc970_hclken, NULL, NULL};
struct clk clk_i2s = 	{(1 << 24), 0, 0, nuc970_hclken, NULL, NULL};
struct clk clk_lcd = 	{(1 << 25), 0, 0, nuc970_hclken, NULL, NULL};
struct clk clk_vcap = 	{(1 << 26), 0, 0, nuc970_hclken, NULL, NULL};
struct clk clk_sensor = {(1 << 27), 0, 0, nuc970_hclken, NULL, NULL};
struct clk clk_ge = 	{(1 << 28), 0, 0, nuc970_hclken, NULL, NULL};
struct clk clk_jpeg = 	{(1 << 29), 0, 0, nuc970_hclken, NULL, NULL};
struct clk clk_sdh = 	{(1 << 30), 0, 0, nuc970_hclken, NULL, NULL};

struct clk clk_wdt = 	{(1 << 0), 0, 0, nuc970_pclk0en, NULL, NULL};
struct clk clk_wwdt = 	{(1 << 1), 0, 0, nuc970_pclk0en, NULL, NULL};
struct clk clk_rtc = 	{(1 << 2), 0, 0, nuc970_pclk0en, NULL, NULL};
struct clk clk_gpio = 	{(1 << 3), 0, 0, nuc970_pclk0en, NULL, NULL};
struct clk clk_etimer0 = {(1 << 4), 0, 0 , nuc970_pclk0en, NULL, NULL};
struct clk clk_etimer1 = {(1 << 5), 0, 0, nuc970_pclk0en, NULL, NULL};
struct clk clk_etimer2 = {(1 << 6), 0, 0, nuc970_pclk0en, NULL, NULL};
struct clk clk_etimer3 = {(1 << 7), 0, 0, nuc970_pclk0en, NULL, NULL};
struct clk clk_timer0 = {(1 << 8), 0, 0, nuc970_pclk0en, NULL, NULL};
struct clk clk_timer1 = {(1 << 9), 0, 0, nuc970_pclk0en, NULL, NULL};
struct clk clk_timer2 = {(1 << 10), 0, 0, nuc970_pclk0en, NULL, NULL};
struct clk clk_timer3 = {(1 << 11), 0, 0, nuc970_pclk0en, NULL, NULL};
struct clk clk_timer4 = {(1 << 12), 0, 0, nuc970_pclk0en, NULL, NULL};
struct clk clk_uart0 = 	{(1 << 16), 0, 0, nuc970_pclk0en, NULL, NULL};
struct clk clk_uart1 = 	{(1 << 17), 0, 0, nuc970_pclk0en, NULL, NULL};
struct clk clk_uart2 = 	{(1 << 18), 0, 0, nuc970_pclk0en, NULL, NULL};
struct clk clk_uart3 = 	{(1 << 19), 0, 0, nuc970_pclk0en, NULL, NULL};
struct clk clk_uart4 = 	{(1 << 20), 0, 0, nuc970_pclk0en, NULL, NULL};
struct clk clk_uart5 = 	{(1 << 21), 0, 0, nuc970_pclk0en, NULL, NULL};
struct clk clk_uart6 = 	{(1 << 22), 0, 0, nuc970_pclk0en, NULL, NULL};
struct clk clk_uart7 = 	{(1 << 23), 0, 0, nuc970_pclk0en, NULL, NULL};
struct clk clk_uart8 = 	{(1 << 24), 0, 0, nuc970_pclk0en, NULL, NULL};
struct clk clk_uart9 = 	{(1 << 25), 0, 0, nuc970_pclk0en, NULL, NULL};
struct clk clk_uart10 = {(1 << 26), 0, 0, nuc970_pclk0en, NULL, NULL};


struct clk clk_i2c0 = 	{(1 << 0), 0, 0, nuc970_pclk1en, NULL, NULL};
struct clk clk_i2c1 = 	{(1 << 1), 0, 0, nuc970_pclk1en, NULL, NULL};
struct clk clk_spi0 = 	{(1 << 4), 0, 0, nuc970_pclk1en, NULL, NULL};
struct clk clk_spi1 = 	{(1 << 5), 0, 0, nuc970_pclk1en, NULL, NULL};
struct clk clk_can0 = 	{(1 << 8), 0, 0, nuc970_pclk1en, NULL, NULL};
struct clk clk_can1 = 	{(1 << 9), 0, 0, nuc970_pclk1en, NULL, NULL};
struct clk clk_sc0 = 	{(1 << 12), 0, 0, nuc970_pclk1en, NULL, NULL};
struct clk clk_sc1 = 	{(1 << 13), 0, 0, nuc970_pclk1en, NULL, NULL};
struct clk clk_adc = 	{(1 << 24), 0, 0, nuc970_pclk1en, NULL, NULL};
struct clk clk_kpi = 	{(1 << 25), 0, 0, nuc970_pclk1en, NULL, NULL};
struct clk clk_mtp = 	{(1 << 26), 0, 0, nuc970_pclk1en, NULL, NULL};
struct clk clk_pwm = 	{(1 << 27), 0, 0, nuc970_pclk1en, NULL, NULL};




static struct clk_lookup nuc970_clkregs[] = {
	// AHB
	DEF_CLKLOOK(&clk_sram, "nuc970-sram", NULL),
	DEF_CLKLOOK(&clk_ebi, "nuc970-ebi", NULL),
	DEF_CLKLOOK(&clk_gdma, "nuc970-gdma", NULL),
	DEF_CLKLOOK(&clk_cko, "nuc97-cko", NULL),
	DEF_CLKLOOK(&clk_emac0, "nuc970-emac0", NULL),
	DEF_CLKLOOK(&clk_emac1, "nuc970-emac1", NULL),
	DEF_CLKLOOK(&clk_usbh, "nuc970-ohci", NULL),
	DEF_CLKLOOK(&clk_usbh, "nuc970-ehci", NULL),
	DEF_CLKLOOK(&clk_usbd, "nuc970-usbdev", NULL),
	DEF_CLKLOOK(&clk_fmi, "nuc970-fmi", NULL),
	DEF_CLKLOOK(&clk_nand, NULL, "nand"),
	DEF_CLKLOOK(&clk_emmc, NULL, "emmc"),
	DEF_CLKLOOK(&clk_crypto, "nuc970-crypto", NULL),
	DEF_CLKLOOK(&clk_i2s, "nuc970-actl", NULL),
	DEF_CLKLOOK(&clk_lcd, "nuc970-lcd", NULL),
	DEF_CLKLOOK(&clk_vcap, "nuc970-vcap", NULL),
	DEF_CLKLOOK(&clk_sensor, "nuc970-sensor", NULL),
	DEF_CLKLOOK(&clk_ge, "nuc970-ge", NULL),
	DEF_CLKLOOK(&clk_jpeg, "nuc970-jpeg", NULL),
	DEF_CLKLOOK(&clk_sdh, "nuc970-sdh", NULL),
	// APB
	DEF_CLKLOOK(&clk_wdt, "nuc970-wdt", NULL),
	DEF_CLKLOOK(&clk_wwdt, "nuc970-wwdt", NULL),
	DEF_CLKLOOK(&clk_rtc, "nuc970-rtc", NULL),
	DEF_CLKLOOK(&clk_gpio, "nuc970-gpio", NULL),
	DEF_CLKLOOK(&clk_etimer0, "nuc970-etimer0", NULL),
	DEF_CLKLOOK(&clk_etimer1, "nuc970-etimer1", NULL),
	DEF_CLKLOOK(&clk_etimer2, "nuc970-etimer2", NULL),
	DEF_CLKLOOK(&clk_etimer3, "nuc970-etimer3", NULL),

	DEF_CLKLOOK(&clk_timer0, NULL, "nuc970-timer0"),	//cc
	DEF_CLKLOOK(&clk_timer1, NULL, "nuc970-timer1"),	//cc
	DEF_CLKLOOK(&clk_timer2, "nuc970-timer2", NULL),
	DEF_CLKLOOK(&clk_timer3, "nuc970-timer3", NULL),

	DEF_CLKLOOK(&clk_uart0, NULL, "nuc970-uart0"),		//cc
	DEF_CLKLOOK(&clk_uart1, "nuc970-uart1", NULL),
	DEF_CLKLOOK(&clk_uart2, "nuc970-uart2", NULL),
	DEF_CLKLOOK(&clk_uart3, "nuc970-uart3", NULL),
	DEF_CLKLOOK(&clk_uart4, "nuc970-uart4", NULL),
	DEF_CLKLOOK(&clk_uart5, "nuc970-uart5", NULL),
	DEF_CLKLOOK(&clk_uart6, "nuc970-uart6", NULL),
	DEF_CLKLOOK(&clk_uart7, "nuc970-uart7", NULL),
	DEF_CLKLOOK(&clk_uart8, "nuc970-uart8", NULL),
	DEF_CLKLOOK(&clk_uart9, "nuc970-uart9", NULL),
	DEF_CLKLOOK(&clk_uart10, "nuc970-uart10", NULL),

	DEF_CLKLOOK(&clk_i2c0, "nuc970-i2c0", NULL),
	DEF_CLKLOOK(&clk_i2c1, "nuc970-i2c1", NULL),
	DEF_CLKLOOK(&clk_spi0, "nuc970-spi0", NULL),
	DEF_CLKLOOK(&clk_spi1, "nuc970-spi1", NULL),
	DEF_CLKLOOK(&clk_can0, "nuc970-can0", NULL),
	DEF_CLKLOOK(&clk_can1, "nuc970-can1", NULL),
	DEF_CLKLOOK(&clk_sc0, "nuc970-sc0", NULL),
	DEF_CLKLOOK(&clk_sc1, "nuc970-sc1", NULL),
//	DEF_CLKLOOK(&clk_adc, "nuc970-uart0", NULL),	//cc
//	DEF_CLKLOOK(&clk_kpi, "nuc970-uart0", NULL),	//cc
	DEF_CLKLOOK(&clk_adc, "nuc970-adc", NULL),
	DEF_CLKLOOK(&clk_kpi, "nuc970-kpi", NULL),
	DEF_CLKLOOK(&clk_mtp, "nuc970-mtp", NULL),
	DEF_CLKLOOK(&clk_pwm, "nuc970-pwm", NULL),
};

int clk_enable(struct clk *clk)
{
        unsigned long flags;
	spin_lock_irqsave(&clk_lock, flags);
	if(clk->count++ == 0)
		clk->control(clk, 1);
	spin_unlock_irqrestore(&clk_lock, flags);

	return 0;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
        unsigned long flags;
	spin_lock_irqsave(&clk_lock, flags);
	if(--clk->count == 0)
		clk->control(clk, 0);
	spin_unlock_irqrestore(&clk_lock, flags);
}
EXPORT_SYMBOL(clk_disable);

unsigned long clk_get_rate(struct clk *clk)
{
        unsigned long flags;
	spin_lock_irqsave(&clk_lock, flags);

	spin_unlock_irqrestore(&clk_lock, flags);
	return 0;
}
EXPORT_SYMBOL(clk_get_rate);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
        unsigned long flags;
	spin_lock_irqsave(&clk_lock, flags);

	spin_unlock_irqrestore(&clk_lock, flags);

	return 0;
}
EXPORT_SYMBOL(clk_set_rate);
#if 0
int clk_set_parent(struct clk *clk, struct clk *parent)
{
	return 0;
}
EXPORT_SYMBOL(clk_set_parent);

struct clk *clk_get_parent(struct clk *clk)
{

	return NULL;
}
EXPORT_SYMBOL(clk_get_parent);
#endif
void nuc970_hclken(struct clk *clk, int enable)
{
	if(enable)
		__raw_writel(clk->ctlbit | __raw_readl(REG_CLK_HCLKEN), REG_CLK_HCLKEN);
	else
		__raw_writel(~clk->ctlbit & __raw_readl(REG_CLK_HCLKEN), REG_CLK_HCLKEN);

	return;
}

void nuc970_pclk0en(struct clk *clk, int enable)
{
	if(enable)
		__raw_writel(clk->ctlbit | __raw_readl(REG_CLK_PCLKEN0), REG_CLK_PCLKEN0);
	else
		__raw_writel(~clk->ctlbit & __raw_readl(REG_CLK_PCLKEN0), REG_CLK_PCLKEN0);
	return;
}


void nuc970_pclk1en(struct clk *clk, int enable)
{
	if(enable)
		__raw_writel(clk->ctlbit | __raw_readl(REG_CLK_PCLKEN1), REG_CLK_PCLKEN1);
	else
		__raw_writel(~clk->ctlbit & __raw_readl(REG_CLK_PCLKEN1), REG_CLK_PCLKEN1);
	return;
}

unsigned int nuc970_get_apll(void)
{


	return 0;
}

unsigned int nuc970_get_upll(void)
{


	return 0;
}

unsigned int nuc970_get_hclk(void)
{
	return 0;
}


unsigned int nuc970_get_hclk234(void)
{
	return 0;
}

unsigned int nuc970_get_pclk(void)
{
	return 0;
}


void __init nuc970_init_clocks(void)
{
        clkdev_add_table(nuc970_clkregs, ARRAY_SIZE(nuc970_clkregs));
}


