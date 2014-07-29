 /*
 * linux/arch/arm/mach-nuc970/mach-nuc970.c
 *
 * Copyright (C) 2014 Nuvoton technology corporation.
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

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>
#include <mach/map.h>
#include <mach/mfp.h>

#include <mach/irqs.h>
#include <mach/regs-gcr.h>
#include <mach/regs-aic.h>
#include "cpu.h"


/* Initial IO mappings */	//TODO: MAP ENABLED IP ONLY
static struct map_desc nuc970_iodesc[] __initdata = {
	IODESC_ENT(IRQ),
	IODESC_ENT(GCR_CLK),
	IODESC_ENT(EBI_SDIC),
        IODESC_ENT(EMAC0),
        IODESC_ENT(EMAC1),
        IODESC_ENT(GDMA),
        IODESC_ENT(EHCI),
        IODESC_ENT(OHCI),
        IODESC_ENT(USBDEV),
        IODESC_ENT(LCD),
        IODESC_ENT(ACTL),
        IODESC_ENT(JPEG),
        IODESC_ENT(GE),
        IODESC_ENT(SDH),
        IODESC_ENT(FMI),
        IODESC_ENT(CAP),
        IODESC_ENT(CRYPTO),
        IODESC_ENT(UART),
	IODESC_ENT(TIMER_ETIMER_WDT_WWDT),
        IODESC_ENT(GPIO),
        IODESC_ENT(RTC),
        IODESC_ENT(SC),
	IODESC_ENT(I2C_SPI),
        IODESC_ENT(PWM),
        IODESC_ENT(KPI),
        IODESC_ENT(ADC),
        IODESC_ENT(CAN),
        IODESC_ENT(MTP),
};

extern void nuc970_restart(char mode, const char *cmd);
extern void nuc970_timer_init(void);
static struct platform_device *nuc970_dev[] __initdata = {

};

static void __init nuc970_map_io(void)
{
	iotable_init(nuc970_iodesc, ARRAY_SIZE(nuc970_iodesc));
}

static void __init nuc970_init(void)
{
	nuc970_platform_init(nuc970_dev, ARRAY_SIZE(nuc970_dev));
}

MACHINE_START(NUC970, "NUC970")
	.atag_offset	= 0x100,
	.map_io		= nuc970_map_io,
	.init_irq	= nuc970_init_irq,
	.init_machine	= nuc970_init,
	.init_time	= nuc970_timer_init,
	.restart	= nuc970_restart,
MACHINE_END

