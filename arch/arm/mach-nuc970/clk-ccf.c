/*
 * linux/arch/arm/mach-nuc970/clk-ccf.c
 *
 * Copyright (c) 2014 Nuvoton Technology Corporation.
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
static const char *usb_sel_clks[] = { "xin", "dummy", "usb_aplldiv", "usb_uplldiv", };
static const char *adc_sel_clks[] = { "xin", "dummy", "adc_aplldiv", "adc_uplldiv", };
static const char *cap_sel_clks[] = { "xin", "dummy", "cap_aplldiv", "cap_uplldiv", };
static const char *sdh_sel_clks[] = { "xin", "dummy", "sdh_aplldiv", "sdh_uplldiv", };
static const char *emmc_sel_clks[] = { "xin", "dummy", "emmc_aplldiv", "emmc_uplldiv", };
static const char *uart0_sel_clks[] = { "xin", "dummy", "uart0_aplldiv", "uart0_uplldiv", };
static const char *uart1_sel_clks[] = { "xin", "dummy", "uart1_aplldiv", "uart1_uplldiv", };
static const char *uart2_sel_clks[] = { "xin", "dummy", "uart2_aplldiv", "uart2_uplldiv", };
static const char *uart3_sel_clks[] = { "xin", "dummy", "uart3_aplldiv", "uart3_uplldiv", };
static const char *uart4_sel_clks[] = { "xin", "dummy", "uart4_aplldiv", "uart4_uplldiv", };
static const char *uart5_sel_clks[] = { "xin", "dummy", "uart5_aplldiv", "uart5_uplldiv", };
static const char *uart6_sel_clks[] = { "xin", "dummy", "uart6_aplldiv", "uart6_uplldiv", };
static const char *uart7_sel_clks[] = { "xin", "dummy", "uart7_aplldiv", "uart7_uplldiv", };
static const char *uart8_sel_clks[] = { "xin", "dummy", "uart8_aplldiv", "uart8_uplldiv", };
static const char *uart9_sel_clks[] = { "xin", "dummy", "uart9_aplldiv", "uart9_uplldiv", };
static const char *uart10_sel_clks[] = { "xin", "dummy", "uart10_aplldiv", "uart10_uplldiv", };
static const char *system_sel_clks[] = { "xin", "dummy", "system_aplldiv", "system_uplldiv", };
static const char *gpio_sel_clks[] = { "xin", "xin32k"};
static const char *kpi_sel_clks[] = { "xin", "xin32k"};
static const char *etimer_sel_clks[] = { "xin", "pclk_div", "pclk4096_div", "xin32k",};
static const char *wwdt_sel_clks[] = { "xin", "xin128_div", "pclk4096_div", "xin32k",};

enum nuc970_clks {
	// source
	xin, apll, upll, xin32k, xin128_div, 
	
	// eclk
	usb_aplldiv, usb_uplldiv, usb_eclk_mux, usb_eclk_div, usb_eclk_gate,
	sd_aplldiv, sd_uplldiv, sd_eclk_mux, sd_eclk_div, sd_eclk_gate,
	lcd_aplldiv, lcd_uplldiv, lcd_eclk_mux, lcd_eclk_div, lcd_eclk_gate,
	adc_aplldiv, adc_uplldiv, adc_eclk_mux, adc_eclk_div, adc_eclk_gate,
	audio_aplldiv, audio_uplldiv, audio_eclk_mux, audio_eclk_div, audio_eclk_gate,
	cap_aplldiv, cap_uplldiv, cap_eclk_mux, cap_eclk_div, cap_eclk_gate,
	sdh_aplldiv, sdh_uplldiv, sdh_eclk_mux, sdh_eclk_div, sdh_eclk_gate,
	emmc_aplldiv, emmc_uplldiv, emmc_eclk_mux, emmc_eclk_div, emmc_eclk_gate,
	uart0_aplldiv, uart0_uplldiv, uart0_eclk_mux, uart0_eclk_div, uart0_eclk_gate,
	uart1_aplldiv, uart1_uplldiv, uart1_eclk_mux, uart1_eclk_div, uart1_eclk_gate,
	uart2_aplldiv, uart2_uplldiv, uart2_eclk_mux, uart2_eclk_div, uart2_eclk_gate,
	uart3_aplldiv, uart3_uplldiv, uart3_eclk_mux, uart3_eclk_div, uart3_eclk_gate,
	uart4_aplldiv, uart4_uplldiv, uart4_eclk_mux, uart4_eclk_div, uart4_eclk_gate,
	uart5_aplldiv, uart5_uplldiv, uart5_eclk_mux, uart5_eclk_div, uart5_eclk_gate,
	uart6_aplldiv, uart6_uplldiv, uart6_eclk_mux, uart6_eclk_div, uart6_eclk_gate,
	uart7_aplldiv, uart7_uplldiv, uart7_eclk_mux, uart7_eclk_div, uart7_eclk_gate,
	uart8_aplldiv, uart8_uplldiv, uart8_eclk_mux, uart8_eclk_div, uart8_eclk_gate,
	uart9_aplldiv, uart9_uplldiv, uart9_eclk_mux, uart9_eclk_div, uart9_eclk_gate,
	uart10_aplldiv, uart10_uplldiv, uart10_eclk_mux, uart10_eclk_div, uart10_eclk_gate,
	system_aplldiv, system_uplldiv, system_eclk_mux, system_eclk_div, system_eclk_gate,	
	gpio_eclk_mux, gpio_eclk_div, gpio_eclk_gate,
	kpi_eclk_mux, kpi_eclk_div, kpi_eclk_gate,
	etimer0_eclk_mux, etimer0_eclk_gate,
	etimer1_eclk_mux, etimer1_eclk_gate,
	etimer2_eclk_mux, etimer2_eclk_gate,
	etimer3_eclk_mux, etimer3_eclk_gate,
	wwdt_eclk_mux, wwdt_eclk_gate,
	wdt_eclk_mux, wdt_eclk_gate,
	smc0_eclk_div, smc0_eclk_gate, smc0_gate, 
	smc1_eclk_div, smc1_eclk_gate, smc1_gate, 
	
	// sys
	sys_mux, sys_div, cpu_div, cpu_gate, ddr_gate, 
	
	// hclk
	hclk_gate, hclk1_div, gdma_gate, ebi_gate, tic_gate, sram_gate, 
	hclkn_div, dram_gate, hclk234_div, 
	usbh_gate, emac1_gate, emac1_eclk_div, emac1_eclk_gate, usbd_gate, fmi_gate, nand_gate, emmc_gate, crypto_gate, jpeg_gate, jpeg_eclk_div, jpeg_eclk_gate, ge2d_gate, ge2d_eclk_div, ge2d_eclk_gate,
	emac0_gate, emac0_eclk_div, emac0_eclk_gate, sdh_gate, audio_gate, lcd_gate, cap_gate, sensor_gate,
	
	
	// pclk
	pclk_div, pclk4096_div, 
	i2c0_gate, i2c1_gate,
	spi0_gate, spi1_gate,
	uart0_gate, uart1_gate, uart2_gate, uart3_gate, uart4_gate, uart5_gate, uart6_gate, uart7_gate, uart8_gate, uart9_gate, uart10_gate,
	timer0_gate, timer1_gate, timer2_gate, timer3_gate, timer4_gate, 
	wdt_gate,
	rtc_gate,
	wwdt_gate, 
	gpio_gate,	
	adc_gate, 
	kpi_gate, 
	mtpc_gate,
	pwm_gate,
	etimer0_gate, etimer1_gate, etimer2_gate, etimer3_gate,
	can0_gate, can1_gate, 
	
	clk_max
};

static struct clk *clk[clk_max];

int __init nuc970_init_clocks(void)
{
	int i;
	
	// source
	clk[xin] 		= nuc970_clk_fixed("xin", 12000000);
	clk[xin32k] 	= nuc970_clk_fixed("xin32k", 32768);
	clk[apll] 		= nuc970_clk_apll("apll", "xin", REG_CLK_APLLCON);
	clk[upll] 		= nuc970_clk_upll("upll", "xin", REG_CLK_UPLLCON);
	
	clk[xin128_div]  = nuc970_clk_fixed_factor("xin128_div", "xin", 1, 128);		//  xin/128
	
	clk[sys_mux] 	= nuc970_clk_mux("sys_mux", REG_CLK_DIV0, 3, 2, sys_sel_clks, ARRAY_SIZE(sys_sel_clks));
	clk[sys_div]	= nuc970_clk_divider("sys_div", "sys_mux", REG_CLK_DIV0, 0, 2);
	clk[ddr_gate] = nuc970_clk_gate("ddr_gate", "sys_div", REG_CLK_HCLKEN, 10);
		
	// CPU
	clk[cpu_div]  = nuc970_clk_divider("cpu_div", "sys_div", REG_CLK_DIV0, 16, 1);
	clk[cpu_gate] = nuc970_clk_gate("cpu_gate", "cpu_div", REG_CLK_HCLKEN, 0);
	
	// HCLK1 & PCLK
	clk[hclk1_div]  = nuc970_clk_fixed_factor("hclk1_div", "cpu_div", 1, 2);	
	clk[gdma_gate] = nuc970_clk_gate("gdma_hclk_gate", "hclk1_div", REG_CLK_HCLKEN, 12);
	clk[ebi_gate] = nuc970_clk_gate("ebi_hclk_gate", "hclk1_div", REG_CLK_HCLKEN, 9);
	clk[tic_gate] = nuc970_clk_gate("tic_hclk_gate", "hclk1_div", REG_CLK_HCLKEN, 7);
	
	// HCLK & HCLK234
	clk[hclkn_div]  = nuc970_clk_fixed_factor("hclkn_div", "sys_div", 1, 2);			//  /2		
	clk[dram_gate] = nuc970_clk_gate("dram_gate", "hclkn_div", REG_CLK_HCLKEN, 10);
	clk[hclk_gate] = nuc970_clk_gate("hclk_gate", "hclkn_div", REG_CLK_HCLKEN, 1);
	clk[sram_gate] = nuc970_clk_gate("sram_gate", "hclk_gate", REG_CLK_HCLKEN, 8);
	clk[hclk234_div] = nuc970_clk_divider("hclk234_div", "hclkn_div", REG_CLK_DIV0, 20, 4);
	
	// HCLK3
	clk[usbh_gate] = nuc970_clk_gate("usbh_hclk_gate", "hclk234_div", REG_CLK_HCLKEN, 18);	
	clk[usbd_gate] = nuc970_clk_gate("usbd_hclk_gate", "hclk234_div", REG_CLK_HCLKEN, 19);
	clk[fmi_gate] = nuc970_clk_gate("fmi_hclk_gate", "hclk234_div", REG_CLK_HCLKEN, 20);
	clk[nand_gate] = nuc970_clk_gate("nand_hclk_gate", "hclk234_div", REG_CLK_HCLKEN, 21);
	clk[emmc_gate] = nuc970_clk_gate("emmc_hclk_gate", "hclk234_div", REG_CLK_HCLKEN, 22);
	clk[crypto_gate] = nuc970_clk_gate("crypto_hclk_gate", "hclk234_div", REG_CLK_HCLKEN, 23);
	
	clk[emac1_gate] = nuc970_clk_gate("emac1_hclk_gate", "hclk234_div", REG_CLK_HCLKEN, 17);
	clk[emac1_eclk_div] = nuc970_clk_divider("emac1_eclk_div", "hclk234_div", REG_CLK_DIV8, 0, 8);
	clk[emac1_eclk_gate] = nuc970_clk_gate("emac1_eclk_gate", "emac1_eclk_div", REG_CLK_HCLKEN, 17);
	
	clk[jpeg_gate] = nuc970_clk_gate("jpeg_hclk_gate", "hclk234_div", REG_CLK_HCLKEN, 29);
	clk[jpeg_eclk_div] = nuc970_clk_divider("jpeg_eclk_div", "hclk234_div", REG_CLK_DIV3, 28, 3);
	clk[jpeg_eclk_gate] = nuc970_clk_gate("jpeg_eclk_gate", "jpeg_eclk_div", REG_CLK_HCLKEN, 29);
    
    clk[ge2d_gate] = nuc970_clk_gate("ge2d_hclk_gate", "hclk234_div", REG_CLK_HCLKEN, 28);
	clk[ge2d_eclk_div] = nuc970_clk_divider("ge2d_eclk_div", "hclk234_div", REG_CLK_DIV2, 28, 2);
	clk[ge2d_eclk_gate] = nuc970_clk_gate("ge2d_eclk_gate", "ge2d_eclk_div", REG_CLK_HCLKEN, 28);
	
	// HCLK4
	clk[sdh_gate] = nuc970_clk_gate("sdh_hclk_gate", "hclk234_div", REG_CLK_HCLKEN, 30);
	clk[audio_gate] = nuc970_clk_gate("audio_hclk_gate", "hclk234_div", REG_CLK_HCLKEN, 24);
	clk[lcd_gate] = nuc970_clk_gate("lcd_hclk_gate", "hclk234_div", REG_CLK_HCLKEN, 25);
	clk[cap_gate] = nuc970_clk_gate("cap_hclk_gate", "hclk234_div", REG_CLK_HCLKEN, 26);
	clk[sensor_gate] = nuc970_clk_gate("sensor_hclk_gate", "hclk234_div", REG_CLK_HCLKEN, 27);
	
	clk[emac0_gate] = nuc970_clk_gate("emac0_hclk_gate", "hclk234_div", REG_CLK_HCLKEN, 16);
	clk[emac0_eclk_div] = nuc970_clk_divider("emac0_eclk_div", "hclk234_div", REG_CLK_DIV8, 0, 8);
	clk[emac0_eclk_gate] = nuc970_clk_gate("emac0_eclk_gate", "emac0_eclk_div", REG_CLK_HCLKEN, 16);
				
	// ECLK
	// -USB
	clk[usb_aplldiv]	= nuc970_clk_divider("usb_aplldiv", "apll", REG_CLK_DIV2, 0, 3);
	clk[usb_uplldiv]	= nuc970_clk_divider("usb_uplldiv", "upll", REG_CLK_DIV2, 0, 3);
	clk[usb_eclk_mux]	= nuc970_clk_mux("usb_eclk_mux", REG_CLK_DIV2, 3, 2, usb_sel_clks, ARRAY_SIZE(usb_sel_clks));
	clk[usb_eclk_div]	= nuc970_clk_divider("usb_eclk_div", "usb_eclk_mux", REG_CLK_DIV2, 8, 4);
	clk[usb_eclk_gate]  = nuc970_clk_gate("usb_eclk_gate", "usb_eclk_div", REG_CLK_HCLKEN, 18);
	
	// -SDH
	clk[sdh_aplldiv]	= nuc970_clk_divider("sdh_aplldiv", "apll", REG_CLK_DIV9, 0, 3);
	clk[sdh_uplldiv]	= nuc970_clk_divider("sdh_uplldiv", "upll", REG_CLK_DIV9, 0, 3);
	clk[sdh_eclk_mux]	= nuc970_clk_mux("sdh_eclk_mux", REG_CLK_DIV9, 3, 2, sdh_sel_clks, ARRAY_SIZE(sdh_sel_clks));
	clk[sdh_eclk_div]	= nuc970_clk_divider("sdh_eclk_div", "sdh_eclk_mux", REG_CLK_DIV9, 8, 8);
	clk[sdh_eclk_gate]  = nuc970_clk_gate("sdh_eclk_gate", "sdh_eclk_div", REG_CLK_HCLKEN, 30);
	
	// -EMMC
	clk[emmc_aplldiv]	= nuc970_clk_divider("emmc_aplldiv", "apll", REG_CLK_DIV3, 0, 3);
	clk[emmc_uplldiv]	= nuc970_clk_divider("emmc_uplldiv", "upll", REG_CLK_DIV3, 0, 3);
	clk[emmc_eclk_mux]	= nuc970_clk_mux("emmc_eclk_mux", REG_CLK_DIV3, 3, 2, emmc_sel_clks, ARRAY_SIZE(emmc_sel_clks));
	clk[emmc_eclk_div]	= nuc970_clk_divider("emmc_eclk_div", "emmc_eclk_mux", REG_CLK_DIV3, 8, 8);
	clk[emmc_eclk_gate]  = nuc970_clk_gate("emmc_eclk_gate", "emmc_eclk_div", REG_CLK_HCLKEN, 22);
	
	// -ADC
	clk[adc_aplldiv]	= nuc970_clk_divider("adc_aplldiv", "apll", REG_CLK_DIV7, 16, 3);
	clk[adc_uplldiv]	= nuc970_clk_divider("adc_uplldiv", "upll", REG_CLK_DIV7, 16, 3);
	clk[adc_eclk_mux]	= nuc970_clk_mux("adc_eclk_mux", REG_CLK_DIV7, 19, 2, adc_sel_clks, ARRAY_SIZE(adc_sel_clks));
	clk[adc_eclk_div]	= nuc970_clk_divider("adc_eclk_div", "adc_eclk_mux", REG_CLK_DIV7, 24, 8);
	clk[adc_eclk_gate]  = nuc970_clk_gate("adc_eclk_gate", "adc_eclk_div", REG_CLK_PCLKEN1, 24);
	
	// -LCD
	clk[lcd_aplldiv]	= nuc970_clk_divider("lcd_aplldiv", "apll", REG_CLK_DIV1, 0, 3);
	clk[lcd_uplldiv]	= nuc970_clk_divider("lcd_uplldiv", "upll", REG_CLK_DIV1, 0, 3);
	clk[lcd_eclk_mux] 	= nuc970_clk_mux("lcd_eclk_mux", REG_CLK_DIV1, 3, 2, lcd_sel_clks, ARRAY_SIZE(lcd_sel_clks));
	clk[lcd_eclk_div]	= nuc970_clk_divider("lcd_eclk_div", "lcd_eclk_mux", REG_CLK_DIV1, 8, 8);
	clk[lcd_eclk_gate]  = nuc970_clk_gate("lcd_eclk_gate", "lcd_eclk_div", REG_CLK_HCLKEN, 25);

	// -AUDIO
	clk[audio_aplldiv]	= nuc970_clk_divider("audio_aplldiv", "apll", REG_CLK_DIV1, 16, 3);
	clk[audio_uplldiv]	= nuc970_clk_divider("audio_uplldiv", "upll", REG_CLK_DIV1, 16, 3);
	clk[audio_eclk_mux] = nuc970_clk_mux("audio_eclk_mux", REG_CLK_DIV1, 19, 2, audio_sel_clks, ARRAY_SIZE(audio_sel_clks));
	clk[audio_eclk_div]	= nuc970_clk_divider("audio_eclk_div", "audio_eclk_mux", REG_CLK_DIV1, 24, 8);
	clk[audio_eclk_gate]  = nuc970_clk_gate("audio_eclk_gate", "audio_eclk_div", REG_CLK_HCLKEN, 24);

	// -CAP
	clk[cap_aplldiv]	= nuc970_clk_divider("cap_aplldiv", "apll", REG_CLK_DIV3, 16, 3);
	clk[cap_uplldiv]	= nuc970_clk_divider("cap_uplldiv", "upll", REG_CLK_DIV3, 16, 3);
	clk[cap_eclk_mux]   = nuc970_clk_mux("cap_eclk_mux", REG_CLK_DIV3, 19, 2, cap_sel_clks, ARRAY_SIZE(cap_sel_clks));
	clk[cap_eclk_div]	= nuc970_clk_divider("cap_eclk_div", "cap_eclk_mux", REG_CLK_DIV3, 24, 4);
	clk[cap_eclk_gate]  = nuc970_clk_gate("cap_eclk_gate", "cap_eclk_div", REG_CLK_HCLKEN, 26);

	// -UART0
	clk[uart0_aplldiv]	= nuc970_clk_divider("uart0_aplldiv", "apll", REG_CLK_DIV4, 0, 3);
	clk[uart0_uplldiv]	= nuc970_clk_divider("uart0_uplldiv", "upll", REG_CLK_DIV4, 0, 3);
	clk[uart0_eclk_mux] = nuc970_clk_mux("uart0_eclk_mux", REG_CLK_DIV4, 3, 2, uart0_sel_clks, ARRAY_SIZE(uart0_sel_clks));
	clk[uart0_eclk_div]	= nuc970_clk_divider("uart0_eclk_div", "uart0_eclk_mux", REG_CLK_DIV4, 5, 3);
	clk[uart0_eclk_gate]  = nuc970_clk_gate("uart0_eclk_gate", "uart0_eclk_div", REG_CLK_PCLKEN0, 16);
	
	// -UART1
	clk[uart1_aplldiv]	= nuc970_clk_divider("uart1_aplldiv", "apll", REG_CLK_DIV4, 8, 3);
	clk[uart1_uplldiv]	= nuc970_clk_divider("uart1_uplldiv", "upll", REG_CLK_DIV4, 8, 3);
	clk[uart1_eclk_mux] = nuc970_clk_mux("uart1_eclk_mux", REG_CLK_DIV4, 11, 2, uart1_sel_clks, ARRAY_SIZE(uart1_sel_clks));
	clk[uart1_eclk_div]	= nuc970_clk_divider("uart1_eclk_div", "uart1_eclk_mux", REG_CLK_DIV4, 13, 3);
	clk[uart1_eclk_gate]  = nuc970_clk_gate("uart1_eclk_gate", "uart1_eclk_div", REG_CLK_PCLKEN0, 17);
	
	// -UART2
	clk[uart2_aplldiv]	= nuc970_clk_divider("uart2_aplldiv", "apll", REG_CLK_DIV4, 16, 3);
	clk[uart2_uplldiv]	= nuc970_clk_divider("uart2_uplldiv", "upll", REG_CLK_DIV4, 16, 3);
	clk[uart2_eclk_mux] = nuc970_clk_mux("uart2_eclk_mux", REG_CLK_DIV4, 19, 2, uart2_sel_clks, ARRAY_SIZE(uart2_sel_clks));
	clk[uart2_eclk_div]	= nuc970_clk_divider("uart2_eclk_div", "uart2_eclk_mux", REG_CLK_DIV4, 21, 3);
	clk[uart2_eclk_gate]  = nuc970_clk_gate("uart2_eclk_gate", "uart2_eclk_div", REG_CLK_PCLKEN0, 18);
	
	// -UART3
	clk[uart3_aplldiv]	= nuc970_clk_divider("uart3_aplldiv", "apll", REG_CLK_DIV4, 24, 3);
	clk[uart3_uplldiv]	= nuc970_clk_divider("uart3_uplldiv", "upll", REG_CLK_DIV4, 24, 3);
	clk[uart3_eclk_mux] = nuc970_clk_mux("uart3_eclk_mux", REG_CLK_DIV4, 27, 2, uart3_sel_clks, ARRAY_SIZE(uart3_sel_clks));
	clk[uart3_eclk_div]	= nuc970_clk_divider("uart3_eclk_div", "uart3_eclk_mux", REG_CLK_DIV4, 29, 3);
	clk[uart3_eclk_gate]  = nuc970_clk_gate("uart3_eclk_gate", "uart3_eclk_div", REG_CLK_PCLKEN0, 19);
	
	// -UART4
	clk[uart4_aplldiv]	= nuc970_clk_divider("uart4_aplldiv", "apll", REG_CLK_DIV5, 0, 3);
	clk[uart4_uplldiv]	= nuc970_clk_divider("uart4_uplldiv", "upll", REG_CLK_DIV5, 0, 3);
	clk[uart4_eclk_mux] = nuc970_clk_mux("uart4_eclk_mux", REG_CLK_DIV5, 3, 2, uart4_sel_clks, ARRAY_SIZE(uart4_sel_clks));
	clk[uart4_eclk_div]	= nuc970_clk_divider("uart4_eclk_div", "uart4_eclk_mux", REG_CLK_DIV5, 5, 3);
	clk[uart4_eclk_gate]  = nuc970_clk_gate("uart4_eclk_gate", "uart4_eclk_div", REG_CLK_PCLKEN0, 20);
	
	// -UART5
	clk[uart5_aplldiv]	= nuc970_clk_divider("uart5_aplldiv", "apll", REG_CLK_DIV5, 8, 3);
	clk[uart5_uplldiv]	= nuc970_clk_divider("uart5_uplldiv", "upll", REG_CLK_DIV5, 8, 3);
	clk[uart5_eclk_mux] = nuc970_clk_mux("uart5_eclk_mux", REG_CLK_DIV5, 11, 2, uart5_sel_clks, ARRAY_SIZE(uart5_sel_clks));
	clk[uart5_eclk_div]	= nuc970_clk_divider("uart5_eclk_div", "uart5_eclk_mux", REG_CLK_DIV5, 13, 3);
	clk[uart5_eclk_gate]  = nuc970_clk_gate("uart5_eclk_gate", "uart5_eclk_div", REG_CLK_PCLKEN0, 21);
	
	// -UART6
	clk[uart6_aplldiv]	= nuc970_clk_divider("uart6_aplldiv", "apll", REG_CLK_DIV5, 16, 3);
	clk[uart6_uplldiv]	= nuc970_clk_divider("uart6_uplldiv", "upll", REG_CLK_DIV5, 16, 3);
	clk[uart6_eclk_mux] = nuc970_clk_mux("uart6_eclk_mux", REG_CLK_DIV5, 19, 2, uart6_sel_clks, ARRAY_SIZE(uart6_sel_clks));
	clk[uart6_eclk_div]	= nuc970_clk_divider("uart6_eclk_div", "uart6_eclk_mux", REG_CLK_DIV5, 21, 3);
	clk[uart6_eclk_gate]  = nuc970_clk_gate("uart6_eclk_gate", "uart6_eclk_div", REG_CLK_PCLKEN0, 22);
	
	// -UART7
	clk[uart7_aplldiv]	= nuc970_clk_divider("uart7_aplldiv", "apll", REG_CLK_DIV5, 24, 3);
	clk[uart7_uplldiv]	= nuc970_clk_divider("uart7_uplldiv", "upll", REG_CLK_DIV5, 24, 3);
	clk[uart7_eclk_mux] = nuc970_clk_mux("uart7_eclk_mux", REG_CLK_DIV5, 27, 2, uart7_sel_clks, ARRAY_SIZE(uart7_sel_clks));
	clk[uart7_eclk_div]	= nuc970_clk_divider("uart7_eclk_div", "uart7_eclk_mux", REG_CLK_DIV5, 29, 3);
	clk[uart7_eclk_gate]  = nuc970_clk_gate("uart7_eclk_gate", "uart7_eclk_div", REG_CLK_PCLKEN0, 23);
	
	// -UART8
	clk[uart8_aplldiv]	= nuc970_clk_divider("uart8_aplldiv", "apll", REG_CLK_DIV6, 0, 3);
	clk[uart8_uplldiv]	= nuc970_clk_divider("uart8_uplldiv", "upll", REG_CLK_DIV6, 0, 3);
	clk[uart8_eclk_mux] = nuc970_clk_mux("uart8_eclk_mux", REG_CLK_DIV6, 3, 2, uart8_sel_clks, ARRAY_SIZE(uart8_sel_clks));
	clk[uart8_eclk_div]	= nuc970_clk_divider("uart8_eclk_div", "uart8_eclk_mux", REG_CLK_DIV6, 5, 3);
	clk[uart8_eclk_gate]  = nuc970_clk_gate("uart8_eclk_gate", "uart8_eclk_div", REG_CLK_PCLKEN0, 24);
	
	// -UART9
	clk[uart9_aplldiv]	= nuc970_clk_divider("uart9_aplldiv", "apll", REG_CLK_DIV6, 8, 3);
	clk[uart9_uplldiv]	= nuc970_clk_divider("uart9_uplldiv", "upll", REG_CLK_DIV6, 8, 3);
	clk[uart9_eclk_mux] = nuc970_clk_mux("uart9_eclk_mux", REG_CLK_DIV6, 11, 2, uart9_sel_clks, ARRAY_SIZE(uart9_sel_clks));
	clk[uart9_eclk_div]	= nuc970_clk_divider("uart9_eclk_div", "uart9_eclk_mux", REG_CLK_DIV6, 13, 3);
	clk[uart9_eclk_gate]  = nuc970_clk_gate("uart9_eclk_gate", "uart9_eclk_div", REG_CLK_PCLKEN0, 25);
	
	// -UART10
	clk[uart10_aplldiv]	 = nuc970_clk_divider("uart10_aplldiv", "apll", REG_CLK_DIV6, 16, 3);
	clk[uart10_uplldiv]	 = nuc970_clk_divider("uart10_uplldiv", "upll", REG_CLK_DIV6, 16, 3);
	clk[uart10_eclk_mux] = nuc970_clk_mux("uart10_eclk_mux", REG_CLK_DIV6, 19, 2, uart10_sel_clks, ARRAY_SIZE(uart10_sel_clks));
	clk[uart10_eclk_div] = nuc970_clk_divider("uart10_eclk_div", "uart10_eclk_mux", REG_CLK_DIV6, 21, 3);
	clk[uart10_eclk_gate]= nuc970_clk_gate("uart10_eclk_gate", "uart10_eclk_div", REG_CLK_PCLKEN0, 26);
	
	// -SYSTEM
	clk[system_aplldiv]	 = nuc970_clk_divider("system_aplldiv", "apll", REG_CLK_DIV0, 0, 3);
	clk[system_uplldiv]	 = nuc970_clk_divider("system_uplldiv", "upll", REG_CLK_DIV0, 0, 3);
	clk[system_eclk_mux] = nuc970_clk_mux("system_eclk_mux", REG_CLK_DIV0, 3, 2, system_sel_clks, ARRAY_SIZE(system_sel_clks));
	clk[system_eclk_div] = nuc970_clk_divider("system_eclk_div", "system_eclk_mux", REG_CLK_DIV0, 8, 4);
	
	// -GPIO
	clk[gpio_eclk_mux]  = nuc970_clk_mux("gpio_eclk_mux", REG_CLK_DIV7, 7, 1, gpio_sel_clks, ARRAY_SIZE(gpio_sel_clks));
	clk[gpio_eclk_div]  = nuc970_clk_divider("gpio_eclk_div", "gpio_eclk_mux", REG_CLK_DIV7, 0, 7);
	clk[gpio_eclk_gate] = nuc970_clk_gate("gpio_eclk_gate", "gpio_eclk_div", REG_CLK_PCLKEN0, 3);
	
	// -KPI
	clk[kpi_eclk_mux]  = nuc970_clk_mux("kpi_eclk_mux", REG_CLK_DIV7, 15, 1, kpi_sel_clks, ARRAY_SIZE(kpi_sel_clks));
	clk[kpi_eclk_div]  = nuc970_clk_divider("kpi_eclk_div", "kpi_eclk_mux", REG_CLK_DIV7, 8, 7);
	clk[kpi_eclk_gate] = nuc970_clk_gate("kpi_eclk_gate", "kpi_eclk_div", REG_CLK_PCLKEN1, 25);
	
	// -ETIMER0
	clk[etimer0_eclk_mux] = nuc970_clk_mux("etimer0_eclk_mux", REG_CLK_DIV8, 16, 2, etimer_sel_clks, ARRAY_SIZE(etimer_sel_clks));
	clk[etimer0_eclk_gate]  = nuc970_clk_gate("etimer0_eclk_gate", "etimer0_eclk_mux", REG_CLK_PCLKEN0, 4);
	
	// -ETIMER1
	clk[etimer1_eclk_mux] = nuc970_clk_mux("etimer1_eclk_mux", REG_CLK_DIV8, 18, 2, etimer_sel_clks, ARRAY_SIZE(etimer_sel_clks));
	clk[etimer1_eclk_gate]  = nuc970_clk_gate("etimer1_eclk_gate", "etimer1_eclk_mux", REG_CLK_PCLKEN0, 5);
	
	// -ETIMER2
	clk[etimer2_eclk_mux] = nuc970_clk_mux("etimer2_eclk_mux", REG_CLK_DIV8, 20, 2, etimer_sel_clks, ARRAY_SIZE(etimer_sel_clks));
	clk[etimer2_eclk_gate]  = nuc970_clk_gate("etimer2_eclk_gate", "etimer2_eclk_mux", REG_CLK_PCLKEN0, 6);
	
	// -ETIMER3
	clk[etimer3_eclk_mux] = nuc970_clk_mux("etimer3_eclk_mux", REG_CLK_DIV8, 22, 2, etimer_sel_clks, ARRAY_SIZE(etimer_sel_clks));
	clk[etimer3_eclk_gate]  = nuc970_clk_gate("etimer3_eclk_gate", "etimer3_eclk_mux", REG_CLK_PCLKEN0, 7);
	
	// -WWDT
	clk[wwdt_eclk_mux] = nuc970_clk_mux("wwdt_eclk_mux", REG_CLK_DIV8, 10, 2, wwdt_sel_clks, ARRAY_SIZE(wwdt_sel_clks));
	clk[wwdt_eclk_gate]  = nuc970_clk_gate("wwdt_eclk_gate", "wwdt_eclk_mux", REG_CLK_PCLKEN0, 1);
	
	// -WDT
	clk[wdt_eclk_mux] = nuc970_clk_mux("wdt_eclk_mux", REG_CLK_DIV8, 8, 2, wwdt_sel_clks, ARRAY_SIZE(wwdt_sel_clks));
	clk[wdt_eclk_gate]  = nuc970_clk_gate("wdt_eclk_gate", "wdt_eclk_mux", REG_CLK_PCLKEN0, 0);
	
	// -SMARTCARD
	clk[smc0_eclk_div]  = nuc970_clk_divider("smc0_eclk_div", "xin", REG_CLK_DIV6, 24, 4);
	clk[smc0_eclk_gate] = nuc970_clk_gate("smc0_eclk_gate", "smc0_eclk_div", REG_CLK_PCLKEN1, 12);
		
	clk[smc1_eclk_div]  = nuc970_clk_divider("smc1_eclk_div", "xin", REG_CLK_DIV6, 28, 4);
	clk[smc1_eclk_gate] = nuc970_clk_gate("smc1_eclk_gate", "smc1_eclk_div", REG_CLK_PCLKEN1, 13);
	
	// PCLK
	clk[pclk_div]	= nuc970_clk_divider("pclk_div", "hclk1_div", REG_CLK_DIV0, 24, 4);
	clk[pclk4096_div]  = nuc970_clk_fixed_factor("pclk4096_div", "pclk_div", 1, 4096);		//  pclk/4096
	clk[i2c0_gate]  = nuc970_clk_gate("i2c0_gate", "pclk_div", REG_CLK_PCLKEN1, 0);
	clk[i2c1_gate]  = nuc970_clk_gate("i2c1_gate", "pclk_div", REG_CLK_PCLKEN1, 1);
	
	clk[spi0_gate]  = nuc970_clk_gate("spi0_gate", "pclk_div", REG_CLK_PCLKEN1, 4);
	clk[spi1_gate]  = nuc970_clk_gate("spi1_gate", "pclk_div", REG_CLK_PCLKEN1, 5);
	
	clk[uart0_gate] = nuc970_clk_gate("uart0_gate", "pclk_div", REG_CLK_PCLKEN0, 16);
	clk[uart1_gate] = nuc970_clk_gate("uart1_gate", "pclk_div", REG_CLK_PCLKEN0, 17);
	clk[uart2_gate] = nuc970_clk_gate("uart2_gate", "pclk_div", REG_CLK_PCLKEN0, 18);
	clk[uart3_gate] = nuc970_clk_gate("uart3_gate", "pclk_div", REG_CLK_PCLKEN0, 19);
	clk[uart4_gate] = nuc970_clk_gate("uart4_gate", "pclk_div", REG_CLK_PCLKEN0, 20);
	clk[uart5_gate] = nuc970_clk_gate("uart5_gate", "pclk_div", REG_CLK_PCLKEN0, 21);
	clk[uart6_gate] = nuc970_clk_gate("uart6_gate", "pclk_div", REG_CLK_PCLKEN0, 22);
	clk[uart7_gate] = nuc970_clk_gate("uart7_gate", "pclk_div", REG_CLK_PCLKEN0, 23);
	clk[uart8_gate] = nuc970_clk_gate("uart8_gate", "pclk_div", REG_CLK_PCLKEN0, 24);
	clk[uart9_gate] = nuc970_clk_gate("uart9_gate", "pclk_div", REG_CLK_PCLKEN0, 25);
	clk[uart10_gate] = nuc970_clk_gate("uart10_gate", "pclk_div", REG_CLK_PCLKEN0, 26);
	
	clk[wdt_gate] = nuc970_clk_gate("wdt_gate", "pclk_div", REG_CLK_PCLKEN0, 0);
	clk[wwdt_gate] = nuc970_clk_gate("wwdt_gate", "pclk_div", REG_CLK_PCLKEN0, 1);
	
	clk[rtc_gate] = nuc970_clk_gate("rtc_gate", "pclk_div", REG_CLK_PCLKEN0, 2);
	
	clk[gpio_gate] = nuc970_clk_gate("gpio_gate", "pclk_div", REG_CLK_PCLKEN0, 3);
	
	clk[adc_gate] = nuc970_clk_gate("adc_gate", "pclk_div", REG_CLK_PCLKEN1, 24);
	
	clk[kpi_gate] = nuc970_clk_gate("kpi_gate", "pclk_div", REG_CLK_PCLKEN1, 25);
	
	clk[mtpc_gate] = nuc970_clk_gate("mtpc_gate", "pclk_div", REG_CLK_PCLKEN1, 26);
	
	clk[pwm_gate] = nuc970_clk_gate("pwm_gate", "pclk_div", REG_CLK_PCLKEN1, 27);
	
	clk[etimer0_gate] = nuc970_clk_gate("etimer0_gate", "pclk_div", REG_CLK_PCLKEN0, 4);
	clk[etimer1_gate] = nuc970_clk_gate("etimer1_gate", "pclk_div", REG_CLK_PCLKEN0, 5);
	clk[etimer2_gate] = nuc970_clk_gate("etimer2_gate", "pclk_div", REG_CLK_PCLKEN0, 6);
	clk[etimer3_gate] = nuc970_clk_gate("etimer3_gate", "pclk_div", REG_CLK_PCLKEN0, 7);
	
	clk[can0_gate] = nuc970_clk_gate("can0_gate", "pclk_div", REG_CLK_PCLKEN1, 8);
	clk[can1_gate] = nuc970_clk_gate("can1_gate", "pclk_div", REG_CLK_PCLKEN1, 9);

	clk[timer0_gate] = nuc970_clk_gate("timer0_gate", "xin", REG_CLK_PCLKEN0, 8);
	clk[timer1_gate] = nuc970_clk_gate("timer1_gate", "xin", REG_CLK_PCLKEN0, 9);
	clk[timer2_gate] = nuc970_clk_gate("timer2_gate", "xin", REG_CLK_PCLKEN0, 10);
	clk[timer3_gate] = nuc970_clk_gate("timer3_gate", "xin", REG_CLK_PCLKEN0, 11);
	clk[timer4_gate] = nuc970_clk_gate("timer4_gate", "xin", REG_CLK_PCLKEN0, 12);	
	
	clk[smc0_gate] = nuc970_clk_gate("smc0_gate", "pclk_div", REG_CLK_PCLKEN1, 12);
	clk[smc1_gate] = nuc970_clk_gate("smc1_gate", "pclk_div", REG_CLK_PCLKEN1, 13);
	
	for (i = 0; i < ARRAY_SIZE(clk); i++)
		if (IS_ERR(clk[i]))
			pr_err("nuc970 clk %d: register failed with %ld\n",
				i, PTR_ERR(clk[i]));
	
	
	// register clock device	
	clk_register_clkdev(clk[timer0_gate], "timer0", NULL);		// limitation of name size is xxxxxxxxxxxxxxxx
	clk_register_clkdev(clk[timer1_gate], "timer1", NULL);	
	
	clk_register_clkdev(clk[pclk4096_div], "pclk4096_div", NULL);
	
	clk_register_clkdev(clk[xin], "xin", NULL);
	clk_register_clkdev(clk[xin32k], "xin32k", NULL);
	clk_register_clkdev(clk[apll], "apll", NULL);
	clk_register_clkdev(clk[upll], "upll", NULL);
	
	clk_register_clkdev(clk[sys_mux], "sysmux", NULL);
	clk_register_clkdev(clk[sys_div], "sysdiv", NULL);
	
	clk_register_clkdev(clk[xin128_div], "xin128div", NULL);
	
	// CPU
	clk_register_clkdev(clk[cpu_div], "cpudiv", NULL);
	clk_register_clkdev(clk[cpu_gate], "cpu", NULL);

	// HCLK1
	clk_register_clkdev(clk[hclk_gate], "hclk", NULL);
	clk_register_clkdev(clk[sram_gate], "sram", NULL);
	clk_register_clkdev(clk[hclk1_div], "hclk1div", NULL);
	clk_register_clkdev(clk[ddr_gate], "ddr_hclk", NULL);
	clk_register_clkdev(clk[gdma_gate], "gdma_hclk", NULL);
	clk_register_clkdev(clk[ebi_gate], "ebi_hclk", NULL);
	clk_register_clkdev(clk[tic_gate], "tic_hclk", NULL);
	
	// HCLK234
	clk_register_clkdev(clk[hclkn_div], "hclkndiv", NULL);
	clk_register_clkdev(clk[dram_gate], "dram", NULL);
	clk_register_clkdev(clk[hclk234_div], "hclk234div", NULL);
		
	//HCLK3
	clk_register_clkdev(clk[usbh_gate], "usbh_hclk", NULL);
	clk_register_clkdev(clk[emac1_gate], "emac1_hclk", NULL);
	clk_register_clkdev(clk[emac1_eclk_div], "emac1_eclk_div", NULL);
	clk_register_clkdev(clk[emac1_eclk_gate], "emac1_eclk", NULL);
	clk_register_clkdev(clk[usbd_gate], "usbd_hclk", NULL);
	clk_register_clkdev(clk[fmi_gate], "fmi_hclk", NULL);
	clk_register_clkdev(clk[nand_gate], "nand_hclk", NULL);
    clk_register_clkdev(clk[emmc_gate], "emmc_hclk", NULL);
	clk_register_clkdev(clk[crypto_gate], "crypto_hclk", NULL);
	clk_register_clkdev(clk[jpeg_gate], "jpeg_hclk", NULL);
	clk_register_clkdev(clk[jpeg_eclk_div], "jpeg_eclk_div", NULL);
	clk_register_clkdev(clk[jpeg_eclk_gate], "jpeg_eclk", NULL);
    clk_register_clkdev(clk[ge2d_gate], "ge2d_hclk", NULL);
	clk_register_clkdev(clk[ge2d_eclk_div], "ge2d_eclk_div", NULL);
	clk_register_clkdev(clk[ge2d_eclk_gate], "ge2d_eclk", NULL);
	
	//HCLK4
	clk_register_clkdev(clk[emac0_gate], "emac0_hclk", NULL);
	clk_register_clkdev(clk[emac0_eclk_div], "emac0_eclk_div", NULL);
	clk_register_clkdev(clk[emac0_eclk_gate], "emac0_eclk", NULL);
	clk_register_clkdev(clk[sdh_gate], "sdh_hclk", NULL);
	clk_register_clkdev(clk[audio_gate], "audio_hclk", NULL);
	clk_register_clkdev(clk[lcd_gate], "lcd_hclk", NULL);
	clk_register_clkdev(clk[sensor_gate], "sensor_hclk", NULL);
	clk_register_clkdev(clk[cap_gate], "cap_hclk", NULL);
	
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
	
	clk_register_clkdev(clk[usb_aplldiv], "usb_aplldiv", NULL);
	clk_register_clkdev(clk[usb_uplldiv], "usb_uplldiv", NULL);
	clk_register_clkdev(clk[usb_eclk_mux], "usb_eclk_mux", NULL);
	clk_register_clkdev(clk[usb_eclk_div], "usb_eclk_div", NULL);
	clk_register_clkdev(clk[usb_eclk_gate], "usb_eclk", NULL);
	
	clk_register_clkdev(clk[sdh_aplldiv], "sdh_aplldiv", NULL);
	clk_register_clkdev(clk[sdh_uplldiv], "sdh_uplldiv", NULL);
	clk_register_clkdev(clk[sdh_eclk_mux], "sdh_eclk_mux", NULL);
	clk_register_clkdev(clk[sdh_eclk_div], "sdh_eclk_div", NULL);
	clk_register_clkdev(clk[sdh_eclk_gate], "sdh_eclk", NULL);
    
    clk_register_clkdev(clk[emmc_aplldiv], "emmc_aplldiv", NULL);
	clk_register_clkdev(clk[emmc_uplldiv], "emmc_uplldiv", NULL);
	clk_register_clkdev(clk[emmc_eclk_mux], "emmc_eclk_mux", NULL);
	clk_register_clkdev(clk[emmc_eclk_div], "emmc_eclk_div", NULL);
	clk_register_clkdev(clk[emmc_eclk_gate], "emmc_eclk", NULL);
	
	clk_register_clkdev(clk[adc_aplldiv], "adc_aplldiv", NULL);
	clk_register_clkdev(clk[adc_uplldiv], "adc_uplldiv", NULL);
	clk_register_clkdev(clk[adc_eclk_mux], "adc_eclk_mux", NULL);
	clk_register_clkdev(clk[adc_eclk_div], "adc_eclk_div", NULL);
	clk_register_clkdev(clk[adc_eclk_gate], "adc_eclk", NULL);
	
	clk_register_clkdev(clk[cap_aplldiv], "cap_aplldiv", NULL);
	clk_register_clkdev(clk[cap_uplldiv], "cap_uplldiv", NULL);
	clk_register_clkdev(clk[cap_eclk_mux], "cap_eclk_mux", NULL);
	clk_register_clkdev(clk[cap_eclk_div], "cap_eclk_div", NULL);
	clk_register_clkdev(clk[cap_eclk_gate], "cap_eclk", NULL);
	
	clk_register_clkdev(clk[uart0_aplldiv], "uart0_aplldiv", NULL);
	clk_register_clkdev(clk[uart0_uplldiv], "uart0_uplldiv", NULL);
	clk_register_clkdev(clk[uart0_eclk_mux], "uart0_eclk_mux", NULL);
	clk_register_clkdev(clk[uart0_eclk_div], "uart0_eclk_div", NULL);
	clk_register_clkdev(clk[uart0_eclk_gate], "uart0_eclk", NULL);
	
	clk_register_clkdev(clk[uart1_aplldiv], "uart1_aplldiv", NULL);
	clk_register_clkdev(clk[uart1_uplldiv], "uart1_uplldiv", NULL);
	clk_register_clkdev(clk[uart1_eclk_mux], "uart1_eclk_mux", NULL);
	clk_register_clkdev(clk[uart1_eclk_div], "uart1_eclk_div", NULL);
	clk_register_clkdev(clk[uart1_eclk_gate], "uart1_eclk", NULL);
	
	clk_register_clkdev(clk[uart2_aplldiv], "uart2_aplldiv", NULL);
	clk_register_clkdev(clk[uart2_uplldiv], "uart2_uplldiv", NULL);
	clk_register_clkdev(clk[uart2_eclk_mux], "uart2_eclk_mux", NULL);
	clk_register_clkdev(clk[uart2_eclk_div], "uart2_eclk_div", NULL);
	clk_register_clkdev(clk[uart2_eclk_gate], "uart2_eclk", NULL);
	
	clk_register_clkdev(clk[uart3_aplldiv], "uart3_aplldiv", NULL);
	clk_register_clkdev(clk[uart3_uplldiv], "uart3_uplldiv", NULL);
	clk_register_clkdev(clk[uart3_eclk_mux], "uart3_eclk_mux", NULL);
	clk_register_clkdev(clk[uart3_eclk_div], "uart3_eclk_div", NULL);
	clk_register_clkdev(clk[uart3_eclk_gate], "uart3_eclk", NULL);
	
	clk_register_clkdev(clk[uart4_aplldiv], "uart4_aplldiv", NULL);
	clk_register_clkdev(clk[uart4_uplldiv], "uart4_uplldiv", NULL);
	clk_register_clkdev(clk[uart4_eclk_mux], "uart4_eclk_mux", NULL);
	clk_register_clkdev(clk[uart4_eclk_div], "uart4_eclk_div", NULL);
	clk_register_clkdev(clk[uart4_eclk_gate], "uart4_eclk", NULL);
	
	clk_register_clkdev(clk[uart5_aplldiv], "uart5_aplldiv", NULL);
	clk_register_clkdev(clk[uart5_uplldiv], "uart5_uplldiv", NULL);
	clk_register_clkdev(clk[uart5_eclk_mux], "uart5_eclk_mux", NULL);
	clk_register_clkdev(clk[uart5_eclk_div], "uart5_eclk_div", NULL);
	clk_register_clkdev(clk[uart5_eclk_gate], "uart5_eclk", NULL);
	
	clk_register_clkdev(clk[uart6_aplldiv], "uart6_aplldiv", NULL);
	clk_register_clkdev(clk[uart6_uplldiv], "uart6_uplldiv", NULL);
	clk_register_clkdev(clk[uart6_eclk_mux], "uart6_eclk_mux", NULL);
	clk_register_clkdev(clk[uart6_eclk_div], "uart6_eclk_div", NULL);
	clk_register_clkdev(clk[uart6_eclk_gate], "uart6_eclk", NULL);
	
	clk_register_clkdev(clk[uart7_aplldiv], "uart7_aplldiv", NULL);
	clk_register_clkdev(clk[uart7_uplldiv], "uart7_uplldiv", NULL);
	clk_register_clkdev(clk[uart7_eclk_mux], "uart7_eclk_mux", NULL);
	clk_register_clkdev(clk[uart7_eclk_div], "uart7_eclk_div", NULL);
	clk_register_clkdev(clk[uart7_eclk_gate], "uart7_eclk", NULL);
	
	clk_register_clkdev(clk[uart8_aplldiv], "uart8_aplldiv", NULL);
	clk_register_clkdev(clk[uart8_uplldiv], "uart8_uplldiv", NULL);
	clk_register_clkdev(clk[uart8_eclk_mux], "uart8_eclk_mux", NULL);
	clk_register_clkdev(clk[uart8_eclk_div], "uart8_eclk_div", NULL);
	clk_register_clkdev(clk[uart8_eclk_gate], "uart8_eclk", NULL);
	
	clk_register_clkdev(clk[uart9_aplldiv], "uart9_aplldiv", NULL);
	clk_register_clkdev(clk[uart9_uplldiv], "uart9_uplldiv", NULL);
	clk_register_clkdev(clk[uart9_eclk_mux], "uart9_eclk_mux", NULL);
	clk_register_clkdev(clk[uart9_eclk_div], "uart9_eclk_div", NULL);
	clk_register_clkdev(clk[uart9_eclk_gate], "uart9_eclk", NULL);
	
	clk_register_clkdev(clk[uart10_aplldiv], "uart10_aplldiv", NULL);
	clk_register_clkdev(clk[uart10_uplldiv], "uart10_uplldiv", NULL);
	clk_register_clkdev(clk[uart10_eclk_mux], "uart10_eclk_mux", NULL);
	clk_register_clkdev(clk[uart10_eclk_div], "uart10_eclk_div", NULL);
	clk_register_clkdev(clk[uart10_eclk_gate], "uart10_eclk", NULL);
	
	clk_register_clkdev(clk[system_aplldiv], "system_aplldiv", NULL);
	clk_register_clkdev(clk[system_uplldiv], "system_uplldiv", NULL);
	clk_register_clkdev(clk[system_eclk_mux], "system_eclk_mux", NULL);
	clk_register_clkdev(clk[system_eclk_div], "system_eclk_div", NULL);
	clk_register_clkdev(clk[system_eclk_gate], "system_eclk", NULL);
	
	clk_register_clkdev(clk[gpio_eclk_mux], "gpio_eclk_mux", NULL);
	clk_register_clkdev(clk[gpio_eclk_div], "gpio_eclk_div", NULL);
	clk_register_clkdev(clk[gpio_eclk_gate], "gpio_eclk", NULL);
	
	clk_register_clkdev(clk[kpi_eclk_mux], "kpi_eclk_mux", NULL);
	clk_register_clkdev(clk[kpi_eclk_div], "kpi_eclk_div", NULL);
	clk_register_clkdev(clk[kpi_eclk_gate], "kpi_eclk", NULL);
	
	clk_register_clkdev(clk[etimer0_eclk_mux], "etmr0_eclk_mux", NULL);
	clk_register_clkdev(clk[etimer0_eclk_gate], "etmr0_eclk", NULL);
	clk_register_clkdev(clk[etimer1_eclk_mux], "etmr1_eclk_mux", NULL);
	clk_register_clkdev(clk[etimer1_eclk_gate], "etmr1_eclk", NULL);
	clk_register_clkdev(clk[etimer2_eclk_mux], "etmr2_eclk_mux", NULL);
	clk_register_clkdev(clk[etimer2_eclk_gate], "etmr2_eclk", NULL);
	clk_register_clkdev(clk[etimer3_eclk_mux], "etmr3_eclk_mux", NULL);
	clk_register_clkdev(clk[etimer3_eclk_gate], "etmr3_eclk", NULL);
	
	clk_register_clkdev(clk[wwdt_eclk_mux], "wwdt_eclk_mux", NULL);
	clk_register_clkdev(clk[wwdt_eclk_gate], "wwdt_eclk", NULL);
	clk_register_clkdev(clk[wdt_eclk_mux], "wdt_eclk_mux", NULL);
	clk_register_clkdev(clk[wdt_eclk_gate], "wdt_eclk", NULL);
	
	clk_register_clkdev(clk[smc0_eclk_div], "smc0_eclk_div", NULL);
	clk_register_clkdev(clk[smc0_eclk_gate], "smc0_eclk", NULL);
	clk_register_clkdev(clk[smc1_eclk_div], "smc1_eclk_div", NULL);
	clk_register_clkdev(clk[smc1_eclk_gate], "smc1_eclk", NULL);

	//PCLK	
	clk_register_clkdev(clk[pclk_div], "pclkdiv", NULL);
	clk_register_clkdev(clk[rtc_gate], "rtc", NULL);
	clk_register_clkdev(clk[i2c0_gate], "i2c0", NULL);
	clk_register_clkdev(clk[i2c1_gate], "i2c1", NULL);
	clk_register_clkdev(clk[spi0_gate], "spi0", NULL);
	clk_register_clkdev(clk[spi1_gate], "spi1", NULL);
	clk_register_clkdev(clk[uart0_gate], "uart0", NULL);
	clk_register_clkdev(clk[uart1_gate], "uart1", NULL);
	clk_register_clkdev(clk[uart2_gate], "uart2", NULL);
	clk_register_clkdev(clk[uart3_gate], "uart3", NULL);
	clk_register_clkdev(clk[uart4_gate], "uart4", NULL);
	clk_register_clkdev(clk[uart5_gate], "uart5", NULL);
	clk_register_clkdev(clk[uart6_gate], "uart6", NULL);
	clk_register_clkdev(clk[uart7_gate], "uart7", NULL);
	clk_register_clkdev(clk[uart8_gate], "uart8", NULL);
	clk_register_clkdev(clk[uart9_gate], "uart9", NULL);
	clk_register_clkdev(clk[uart10_gate], "uart10", NULL);
	clk_register_clkdev(clk[wdt_gate], "wdt", NULL);
	clk_register_clkdev(clk[wwdt_gate], "wwdt", NULL);
	clk_register_clkdev(clk[gpio_gate], "gpio", NULL);
	clk_register_clkdev(clk[smc0_gate], "smc0", NULL);
	clk_register_clkdev(clk[smc1_gate], "smc1", NULL);
	clk_register_clkdev(clk[adc_gate], "adc", NULL);
	clk_register_clkdev(clk[kpi_gate], "kpi", NULL);
	clk_register_clkdev(clk[mtpc_gate], "mtpc", NULL);
	clk_register_clkdev(clk[pwm_gate], "pwm", NULL);
	clk_register_clkdev(clk[etimer0_gate], "etimer0", NULL);
	clk_register_clkdev(clk[etimer1_gate], "etimer1", NULL);
	clk_register_clkdev(clk[etimer2_gate], "etimer2", NULL);
	clk_register_clkdev(clk[etimer3_gate], "etimer3", NULL);
	clk_register_clkdev(clk[timer2_gate], "timer2", NULL);
	clk_register_clkdev(clk[timer3_gate], "timer3", NULL);
	clk_register_clkdev(clk[timer4_gate], "timer4", NULL);
	clk_register_clkdev(clk[can0_gate], "can0", NULL);
	clk_register_clkdev(clk[can1_gate], "can1", NULL);
	
	// enable some important clocks
	clk_prepare(clk_get(NULL, "cpu"));
	clk_enable(clk_get(NULL, "cpu"));
	
	clk_prepare(clk_get(NULL, "hclk"));
	clk_enable(clk_get(NULL, "hclk"));
	
	clk_prepare(clk_get(NULL, "sram"));
	clk_enable(clk_get(NULL, "sram"));
	
	clk_prepare(clk_get(NULL, "dram"));
	clk_enable(clk_get(NULL, "dram"));
	
	clk_prepare(clk_get(NULL, "ddr_hclk"));
	clk_enable(clk_get(NULL, "ddr_hclk"));		
	

	return 0;
}
