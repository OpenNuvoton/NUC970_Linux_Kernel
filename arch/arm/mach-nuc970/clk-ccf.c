/*
 *  Copyright (C) 2008 Sascha Hauer <s.hauer@pengutronix.de>, Pengutronix
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/clkdev.h>
#include <linux/err.h>

#include <mach/hardware.h>
#include <mach/regs-clock.h>
#include <linux/spinlock.h>

#include "clk-ccf.h"

DEFINE_SPINLOCK(nuc970_lock);

static const char *sys_sel_clks[] = { "xin", "dummy", "apll", "upll", };
static const char *lcd_sel_clks[] = { "xin", "dummy", "lcd_aplldiv", "lcd_uplldiv", };
static const char *audio_sel_clks[] = { "xin", "dummy", "audio_aplldiv", "audio_uplldiv", };

enum nuc970_clks {
	xin, apll, upll, 
	
	lcd_aplldiv, lcd_uplldiv, lcd_eclk_mux, lcd_eclk_div, lcd_eclk_gate,
	audio_aplldiv, audio_uplldiv, audio_eclk_mux, audio_eclk_div, audio_eclk_gate,
	
	sys_mux, sys_div, cpu_div, 
	
	hclk1_div, hclk1_gate, pclk_div, pclk_gate, 
	
	i2c0_gate, i2c1_gate,
	spi0_gate, spi1_gate,
	uart0_gate,
	timer0_gate,
	timer1_gate,
	clk_max
};

static struct clk *clk[clk_max];

int __init nuc970_init_clocks()
{
	int i;
	
	// source
	clk[xin] 		= nuc970_clk_fixed("xin", 12000000);
	clk[apll] 		= nuc970_clk_apll("apll", "xin", REG_CLK_APLLCON);
	clk[upll] 		= nuc970_clk_upll("upll", "xin", REG_CLK_UPLLCON);
	
	clk[sys_mux] 	= nuc970_clk_mux("sys_mux", REG_CLK_DIV0, 3, 2, sys_sel_clks, ARRAY_SIZE(sys_sel_clks));
	clk[sys_div]	= nuc970_clk_divider("sys_div", "sys_mux", REG_CLK_DIV0, 0, 2);
	
	clk[cpu_div]	= nuc970_clk_divider("cpu_div", "sys_div", REG_CLK_DIV0, 16, 1);
	clk[hclk1_div]  = nuc970_clk_fixed_factor("hclk1_div", "cpu_div", 1, 2);			//CPU_N=0 (default), so hclk1_div= /2
	clk[hclk1_gate] = nuc970_clk_gate("hclk1_gate", "hclk1_div", REG_CLK_HCLKEN, 2);
	
	// ECLK
	// -LCD
	clk[lcd_aplldiv]	= nuc970_clk_divider("lcd_aplldiv", "apll", REG_CLK_DIV1, 0, 3);
	clk[lcd_uplldiv]	= nuc970_clk_divider("lcd_uplldiv", "upll", REG_CLK_DIV1, 0, 3);
	clk[lcd_eclk_mux] 	= nuc970_clk_mux("lcd_eclk_mux", REG_CLK_DIV1, 3, 2, lcd_sel_clks, ARRAY_SIZE(lcd_sel_clks));
	clk[lcd_eclk_div]	= nuc970_clk_divider("lcd_eclk_div", "lcd_eclk_mux", REG_CLK_DIV1, 8, 8);
	clk[lcd_eclk_gate]  = nuc970_clk_gate("lcd_eclk_gate", "lcd_eclk_div", REG_CLK_HCLKEN, 25);
	
	// -AUDIO
	clk[audio_aplldiv]	= nuc970_clk_divider("audio_aplldiv", "apll", REG_CLK_DIV1, 16, 3);
	clk[audio_uplldiv]	= nuc970_clk_divider("audio_uplldiv", "upll", REG_CLK_DIV1, 16, 3);
	clk[audio_eclk_mux] = nuc970_clk_mux("audio_eclk_mux", REG_CLK_DIV1, 19, 2, lcd_sel_clks, ARRAY_SIZE(lcd_sel_clks));
	clk[audio_eclk_div]	= nuc970_clk_divider("audio_eclk_div", "audio_eclk_mux", REG_CLK_DIV1, 14, 8);
	clk[audio_eclk_gate]  = nuc970_clk_gate("audio_eclk_gate", "audio_eclk_div", REG_CLK_HCLKEN, 24);
	
	// PCLK
	clk[pclk_div]	= nuc970_clk_divider("pclk_div", "hclk1_gate", REG_CLK_DIV0, 24, 4);
	clk[pclk_gate]  = nuc970_clk_gate("pclk_gate", "pclk_div", REG_CLK_HCLKEN, 5);
	clk[i2c0_gate]  = nuc970_clk_gate("i2c0_gate", "pclk_gate", REG_CLK_PCLKEN1, 0);
	clk[i2c1_gate]  = nuc970_clk_gate("i2c1_gate", "pclk_gate", REG_CLK_PCLKEN1, 1);
	
	clk[spi0_gate]  = nuc970_clk_gate("spi0_gate", "pclk_gate", REG_CLK_PCLKEN1, 4);
	clk[spi1_gate]  = nuc970_clk_gate("spi1_gate", "pclk_gate", REG_CLK_PCLKEN1, 5);
	
	clk[uart0_gate] = nuc970_clk_gate("uart0_gate", "pclk_gate", REG_CLK_PCLKEN0, 16);
	
	clk[timer0_gate]= nuc970_clk_gate("timer0_gate", "pclk_gate", REG_CLK_PCLKEN0, 8);
	clk[timer1_gate]= nuc970_clk_gate("timer1_gate", "pclk_gate", REG_CLK_PCLKEN0, 9);
	
	for (i = 0; i < ARRAY_SIZE(clk); i++)
		if (IS_ERR(clk[i]))
			pr_err("nuc970 clk %d: register failed with %ld\n",
				i, PTR_ERR(clk[i]));

	clk_register_clkdev(clk[timer0_gate], "timer0", NULL);		// limitation of name size is xxxxxxxxxxxxxxxx
	clk_register_clkdev(clk[timer1_gate], "timer1", NULL);	
	
	clk_register_clkdev(clk[xin], "xin", NULL);
	clk_register_clkdev(clk[apll], "apll", NULL);
	clk_register_clkdev(clk[upll], "upll", NULL);
	
	clk_register_clkdev(clk[sys_mux], "sysmux", NULL);
	clk_register_clkdev(clk[sys_div], "sysdiv", NULL);
	clk_register_clkdev(clk[cpu_div], "cpudiv", NULL);
	clk_register_clkdev(clk[hclk1_div], "hclk1div", NULL);
	clk_register_clkdev(clk[hclk1_gate], "hclk1", NULL);
	
	// ECLK
	clk_register_clkdev(clk[lcd_aplldiv], "lcd_aplldiv", NULL);
	clk_register_clkdev(clk[lcd_uplldiv], "lcd_uplldiv", NULL);
	clk_register_clkdev(clk[lcd_eclk_mux], "lcd_eclk_mux", NULL);
	clk_register_clkdev(clk[lcd_eclk_div], "lcd_eclk_div", NULL);
	clk_register_clkdev(clk[lcd_eclk_gate], "lcd_eclk", NULL);
	
	clk_register_clkdev(clk[audio_aplldiv], "audio_aplldiv", NULL);
	clk_register_clkdev(clk[audio_uplldiv], "audio_uplldiv", NULL);
	clk_register_clkdev(clk[audio_eclk_mux], "audio_eclk_mux", NULL);
	clk_register_clkdev(clk[audio_eclk_div], "audio_eclk_div", NULL);
	clk_register_clkdev(clk[audio_eclk_gate], "audio_eclk", NULL);
	
	
	//PCLK	
	clk_register_clkdev(clk[pclk_div], "pclkdiv", NULL);
	clk_register_clkdev(clk[pclk_gate], "pclk", NULL);
	clk_register_clkdev(clk[i2c0_gate], "i2c0", NULL);
	clk_register_clkdev(clk[i2c1_gate], "i2c1", NULL);
	clk_register_clkdev(clk[spi0_gate], "spi0", NULL);
	clk_register_clkdev(clk[spi1_gate], "spi1", NULL);
	clk_register_clkdev(clk[uart0_gate], "uart0", NULL);

	return 0;
}

