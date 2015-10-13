/*
 * Copyright (c) 2014 Nuvoton Technology Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <mach/map.h>
#include <mach/regs-gcr.h>

// The numbering is not related to actual layout.
const struct pinctrl_pin_desc nuc970_pins[] = {
	PINCTRL_PIN(0x00, "PA0"),
	PINCTRL_PIN(0x01, "PA1"),
	PINCTRL_PIN(0x02, "PA2"),
	PINCTRL_PIN(0x03, "PA3"),
	PINCTRL_PIN(0x04, "PA4"),
	PINCTRL_PIN(0x05, "PA5"),
	PINCTRL_PIN(0x06, "PA6"),
	PINCTRL_PIN(0x07, "PA7"),
	PINCTRL_PIN(0x08, "PA8"),
	PINCTRL_PIN(0x09, "PA9"),
	PINCTRL_PIN(0x0A, "PA10"),
	PINCTRL_PIN(0x0B, "PA11"),
	PINCTRL_PIN(0x0C, "PA12"),
	PINCTRL_PIN(0x0D, "PA13"),
	PINCTRL_PIN(0x0E, "PA14"),
	PINCTRL_PIN(0x0F, "PA15"),
	PINCTRL_PIN(0x10, "PB0"),
	PINCTRL_PIN(0x11, "PB1"),
	PINCTRL_PIN(0x12, "PB2"),
	PINCTRL_PIN(0x13, "PB3"),
	PINCTRL_PIN(0x14, "PB4"),
	PINCTRL_PIN(0x15, "PB5"),
	PINCTRL_PIN(0x16, "PB6"),
	PINCTRL_PIN(0x17, "PB7"),
	PINCTRL_PIN(0x18, "PB8"),
	PINCTRL_PIN(0x19, "PB9"),
	PINCTRL_PIN(0x1A, "PB10"),
	PINCTRL_PIN(0x1B, "PB11"),
	PINCTRL_PIN(0x1C, "PB12"),
	PINCTRL_PIN(0x1D, "PB13"),
	PINCTRL_PIN(0x1E, "PB14"),
	PINCTRL_PIN(0x1F, "PB15"),
	PINCTRL_PIN(0x20, "PC0"),
	PINCTRL_PIN(0x21, "PC1"),
	PINCTRL_PIN(0x22, "PC2"),
	PINCTRL_PIN(0x23, "PC3"),
	PINCTRL_PIN(0x24, "PC4"),
	PINCTRL_PIN(0x25, "PC5"),
	PINCTRL_PIN(0x26, "PC6"),
	PINCTRL_PIN(0x27, "PC7"),
	PINCTRL_PIN(0x28, "PC8"),
	PINCTRL_PIN(0x29, "PC9"),
	PINCTRL_PIN(0x2A, "PC10"),
	PINCTRL_PIN(0x2B, "PC11"),
	PINCTRL_PIN(0x2C, "PC12"),
	PINCTRL_PIN(0x2D, "PC13"),
	PINCTRL_PIN(0x2E, "PC14"),
	//PINCTRL_PIN(0x2F, "PC15"),
	PINCTRL_PIN(0x30, "PD0"),
	PINCTRL_PIN(0x31, "PD1"),
	PINCTRL_PIN(0x32, "PD2"),
	PINCTRL_PIN(0x33, "PD3"),
	PINCTRL_PIN(0x34, "PD4"),
	PINCTRL_PIN(0x35, "PD5"),
	PINCTRL_PIN(0x36, "PD6"),
	PINCTRL_PIN(0x37, "PD7"),
	PINCTRL_PIN(0x38, "PD8"),
	PINCTRL_PIN(0x39, "PD9"),
	PINCTRL_PIN(0x3A, "PD10"),
	PINCTRL_PIN(0x3B, "PD11"),
	PINCTRL_PIN(0x3C, "PD12"),
	PINCTRL_PIN(0x3D, "PD13"),
	PINCTRL_PIN(0x3E, "PD14"),
	PINCTRL_PIN(0x3F, "PD15"),
	PINCTRL_PIN(0x40, "PE0"),
	PINCTRL_PIN(0x41, "PE1"),
	PINCTRL_PIN(0x42, "PE2"),
	PINCTRL_PIN(0x43, "PE3"),
	PINCTRL_PIN(0x44, "PE4"),
	PINCTRL_PIN(0x45, "PE5"),
	PINCTRL_PIN(0x46, "PE6"),
	PINCTRL_PIN(0x47, "PE7"),
	PINCTRL_PIN(0x48, "PE8"),
	PINCTRL_PIN(0x49, "PE9"),
	PINCTRL_PIN(0x4A, "PE10"),
	PINCTRL_PIN(0x4B, "PE11"),
	PINCTRL_PIN(0x4C, "PE12"),
	PINCTRL_PIN(0x4D, "PE13"),
	PINCTRL_PIN(0x4E, "PE14"),
	PINCTRL_PIN(0x4F, "PE15"),
	PINCTRL_PIN(0x50, "PF0"),
	PINCTRL_PIN(0x51, "PF1"),
	PINCTRL_PIN(0x52, "PF2"),
	PINCTRL_PIN(0x53, "PF3"),
	PINCTRL_PIN(0x54, "PF4"),
	PINCTRL_PIN(0x55, "PF5"),
	PINCTRL_PIN(0x56, "PF6"),
	PINCTRL_PIN(0x57, "PF7"),
	PINCTRL_PIN(0x58, "PF8"),
	PINCTRL_PIN(0x59, "PF9"),
	PINCTRL_PIN(0x5A, "PF10"),
	PINCTRL_PIN(0x5B, "PF11"),
	PINCTRL_PIN(0x5C, "PF12"),
	PINCTRL_PIN(0x5D, "PF13"),
	PINCTRL_PIN(0x5E, "PF14"),
	PINCTRL_PIN(0x5F, "PF15"),
	PINCTRL_PIN(0x60, "PG0"),
	PINCTRL_PIN(0x61, "PG1"),
	PINCTRL_PIN(0x62, "PG2"),
	PINCTRL_PIN(0x63, "PG3"),
	PINCTRL_PIN(0x64, "PG4"),
	PINCTRL_PIN(0x65, "PG5"),
	PINCTRL_PIN(0x66, "PG6"),
	PINCTRL_PIN(0x67, "PG7"),
	PINCTRL_PIN(0x68, "PG8"),
	PINCTRL_PIN(0x69, "PG9"),
	PINCTRL_PIN(0x6A, "PG10"),
	PINCTRL_PIN(0x6B, "PG11"),
	PINCTRL_PIN(0x6C, "PG12"),
	PINCTRL_PIN(0x6D, "PG13"),
	PINCTRL_PIN(0x6E, "PG14"),
	PINCTRL_PIN(0x6F, "PG15"),
	PINCTRL_PIN(0x70, "PH0"),
	PINCTRL_PIN(0x71, "PH1"),
	PINCTRL_PIN(0x72, "PH2"),
	PINCTRL_PIN(0x73, "PH3"),
	PINCTRL_PIN(0x74, "PH4"),
	PINCTRL_PIN(0x75, "PH5"),
	PINCTRL_PIN(0x76, "PH6"),
	PINCTRL_PIN(0x77, "PH7"),
	PINCTRL_PIN(0x78, "PH8"),
	PINCTRL_PIN(0x79, "PH9"),
	PINCTRL_PIN(0x7A, "PH10"),
	PINCTRL_PIN(0x7B, "PH11"),
	PINCTRL_PIN(0x7C, "PH12"),
	PINCTRL_PIN(0x7D, "PH13"),
	PINCTRL_PIN(0x7E, "PH14"),
	PINCTRL_PIN(0x7F, "PH15"),
	PINCTRL_PIN(0x80, "PI0"),
	PINCTRL_PIN(0x81, "PI1"),
	PINCTRL_PIN(0x82, "PI2"),
	PINCTRL_PIN(0x83, "PI3"),
	PINCTRL_PIN(0x84, "PI4"),
	PINCTRL_PIN(0x85, "PI5"),
	PINCTRL_PIN(0x86, "PI6"),
	PINCTRL_PIN(0x87, "PI7"),
	PINCTRL_PIN(0x88, "PI8"),
	PINCTRL_PIN(0x89, "PI9"),
	PINCTRL_PIN(0x8A, "PI10"),
	PINCTRL_PIN(0x8B, "PI11"),
	PINCTRL_PIN(0x8C, "PI12"),
	PINCTRL_PIN(0x8D, "PI13"),
	PINCTRL_PIN(0x8E, "PI14"),
	PINCTRL_PIN(0x8F, "PI15"),
	PINCTRL_PIN(0x90, "PJ0"),
	PINCTRL_PIN(0x91, "PJ1"),
	PINCTRL_PIN(0x92, "PJ2"),
	PINCTRL_PIN(0x93, "PJ3"),
	PINCTRL_PIN(0x94, "PJ4"),
};



struct nuc970_pinctrl_group {
	const char *name;
	const unsigned int *pins;
	const unsigned num_pins;
	const unsigned func;
};

static const unsigned emac0_pins[] = {0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59}; // Port F
static const unsigned emac1_pins[] = {0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B}; // Port E
static const unsigned pps0_pin[] = {0x5E};
static const unsigned pps1_pin[] = {0x4D};

static const unsigned lcd_0_pins[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
								0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
								0x66, 0x67, 0x68, 0x69}; // 16 bit mode
static const unsigned lcd_1_pins[] = {0x38, 0x39}; // 18 bit mode
static const unsigned lcd_2_pins[] = {0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F}; // 24 bit mode

static const unsigned vcap_pins[] = {0x83, 0x84, 0x85, 0x86, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F};

static const unsigned kpi_0_pins[] = {0x04, 0x05, 0x06, 0x07}; // row
static const unsigned kpi_1_pins[] = {0x08, 0x09}; // 2 col
static const unsigned kpi_2_pins[] = {0x0A, 0x0B}; // 4 col
static const unsigned kpi_3_pins[] = {0x0C, 0x0D, 0x0E, 0x0F}; // 8 col
static const unsigned kpi_4_pins[] = {0x74, 0x75, 0x76, 0x77}; // row
static const unsigned kpi_5_pins[] = {0x78, 0x79}; // 2 col
static const unsigned kpi_6_pins[] = {0x7A, 0x7B}; // 4 col
static const unsigned kpi_7_pins[] = {0x7C, 0x7D, 0x7E, 0x7F}; // 8 col

static const unsigned sd0_pins[] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37};
static const unsigned sd1_0_pins[] = {0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8C, 0x8D}; // Port I
static const unsigned sd1_1_pins[] = {0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49}; // Port E
static const unsigned sd1_2_pins[] = {0x76, 0x77, 0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D}; // Port H
static const unsigned sd01_0_pins[] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
					0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8C, 0x8D}; // Port I
static const unsigned sd01_1_pins[] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
					0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49}; // Port E
static const unsigned sd01_2_pins[] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
					0x76, 0x77, 0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D}; // Port H

static const unsigned nand_0_pins[] = {0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
								0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E};  // Port C
static const unsigned nand_1_pins[] = {0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
								0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F};  // Port I
static const unsigned nand_2_pins[] = {0x10, 0x11};  // ncs1, rdy1
static const unsigned nand_3_pins[] = {0x64, 0x65};  // ncs1, rdy1

static const unsigned emmc_0_pins[] = {0x20, 0x21, 0x22, 0x23, 0x24, 0x25};  // Port C
static const unsigned emmc_1_pins[] = {0x85, 0x86, 0x87, 0x88, 0x89, 0x8A};  // Port I

static const unsigned usbh_pe_pins[] = {0x4E, 0x4F, 0x71}; // ppwr0 & ppwr1, over-current
static const unsigned usbh_pf_pin[] = {0x5A, 0x71};        // ppwr, over-current
static const unsigned usbh_oc_pin[] = {0x71};            // over-current
static const unsigned usbd_pin[] = {0x70};  // vbvld

static const unsigned i2c0_pins[] = {0x60, 0x61};
static const unsigned i2c1_0_pins[] = {0x10, 0x11};
static const unsigned i2c1_1_pins[] = {0x62, 0x63};
static const unsigned i2c1_2_pins[] = {0x72, 0x73};
static const unsigned i2c1_3_pins[] = {0x83, 0x84};

static const unsigned i2s_pins[] = {0x6A, 0x6B, 0x6C, 0x6D, 0x6E};

static const unsigned uart0_pins[] = {0x40, 0x41};
static const unsigned uart1_0_pins[] = {0x42, 0x43}; // tx, rx
static const unsigned uart1_1_pins[] = {0x44, 0x45}; // rts, cts
static const unsigned uart1_2_pins[] = {0x46, 0x47, 0x48, 0x49}; // full func
static const unsigned uart1_3_pins[] = {0x74, 0x75}; // tx, rx
static const unsigned uart1_4_pins[] = {0x76, 0x77}; // rts, cts
static const unsigned uart1_5_pins[] = {0x85, 0x86}; // tx, rx
static const unsigned uart1_6_pins[] = {0x87, 0x88}; // rts, cts
static const unsigned uart2_0_pins[] = {0x5B, 0x5C}; // tx, rx
static const unsigned uart2_1_pins[] = {0x5D, 0x5E}; // rts, cts
static const unsigned uart3_pins[] = {0x4C, 0x4D}; // tx, rx
static const unsigned uart4_0_pins[] = {0x2A, 0x2B}; // tx, rx
static const unsigned uart4_1_pins[] = {0x2C, 0x2D}; // rts, cts
static const unsigned uart4_2_pins[] = {0x78, 0x79}; // tx, rx
static const unsigned uart4_3_pins[] = {0x7A, 0x7B}; // rts, cts
static const unsigned uart4_4_pins[] = {0x89, 0x8A}; // tx, rx
static const unsigned uart5_pins[] = {0x10, 0x11};
static const unsigned uart6_0_pins[] = {0x12, 0x13}; // tx, rx
static const unsigned uart6_1_pins[] = {0x14, 0x15}; // rts, cts
static const unsigned uart6_2_pins[] = {0x6B, 0x6C}; // tx, rx
static const unsigned uart6_3_pins[] = {0x6D, 0x6E}; // rts, cts
static const unsigned uart7_0_pins[] = {0x64, 0x65}; // tx, rx
static const unsigned uart7_1_pins[] = {0x81, 0x82}; // tx, rx
static const unsigned uart8_0_pins[] = {0x4A, 0x4B}; // tx, rx
static const unsigned uart8_1_pins[] = {0x4C, 0x4D}; // rts, cts
static const unsigned uart8_2_pins[] = {0x7C, 0x7D}; // tx, rx
static const unsigned uart8_3_pins[] = {0x7E, 0x7F}; // rts, cts
static const unsigned uart8_4_pins[] = {0x8C, 0x8D}; // tx, rx
static const unsigned uart8_5_pins[] = {0x8E, 0x8F}; // rts, cts
static const unsigned uart9_0_pins[] = {0x3B, 0x3C}; // tx, rx
static const unsigned uart9_1_pins[] = {0x3E, 0x3F}; // tx, rx
static const unsigned uart9_2_pins[] = {0x72, 0x73}; // tx, rx
static const unsigned uart10_0_pins[] = {0x1A, 0x1B}; // tx, rx
static const unsigned uart10_1_pins[] = {0x1C, 0x1D}; // tx, rx
static const unsigned uart10_2_pins[] = {0x1E, 0x1F}; // rts, cts
static const unsigned uart10_3_pins[] = {0x26, 0x27}; // tx, rx
static const unsigned uart10_4_pins[] = {0x28, 0x29}; // rts, cts

static const unsigned sc0_0_pins[] = {0x6A, 0x6B, 0x6C, 0x6D, 0x6E};
static const unsigned sc0_1_pins[] = {0x8B, 0x8C, 0x8D, 0x8E, 0x8F};
static const unsigned sc0_2_pins[] = {0x6B, 0x6C};  // scuart
static const unsigned sc0_3_pins[] = {0x8C, 0x8D};  // scuart
static const unsigned sc1_0_pins[] = {0x86, 0x87, 0x88, 0x89, 0x8A};
static const unsigned sc1_1_pins[] = {0x87, 0x88};  // scuart

static const unsigned spi0_0_pins[] = {0x16, 0x17, 0x18, 0x19};
static const unsigned spi0_1_pins[] = {0x1A, 0x1B}; // quad
static const unsigned spi0_2_pins[] = {0x10}; // ss1
static const unsigned spi0_3_pins[] = {0x7C}; // ss1
static const unsigned spi1_0_pins[] = {0x1C, 0x1D, 0x1E, 0x1F};
static const unsigned spi1_1_pins[] = {0x11}; // ss1
static const unsigned spi1_2_pins[] = {0x64, 0x65}; // quad
static const unsigned spi1_3_pins[] = {0x7D}; // ss1
static const unsigned spi1_4_pins[] = {0x85, 0x86, 0x87, 0x88};

static const unsigned can0_0_pins[] = {0x1A, 0x1B}; // Port B
static const unsigned can0_1_pins[] = {0x72, 0x73}; // Port H
static const unsigned can0_2_pins[] = {0x83, 0x84}; // Port I
static const unsigned can1_pins[] = {0x7E, 0x7F}; // Port H

static const unsigned pwm0_0_pin[] = {0x0C};
static const unsigned pwm0_1_pin[] = {0x12};
static const unsigned pwm0_2_pin[] = {0x2E};
static const unsigned pwm0_3_pin[] = {0x3C};
static const unsigned pwm1_0_pin[] = {0x0D};
static const unsigned pwm1_1_pin[] = {0x13};
static const unsigned pwm1_2_pin[] = {0x3D};
static const unsigned pwm2_0_pin[] = {0x0E};
static const unsigned pwm2_1_pin[] = {0x72};
static const unsigned pwm2_2_pin[] = {0x3E};
static const unsigned pwm3_0_pin[] = {0x0F};
static const unsigned pwm3_1_pin[] = {0x73};
static const unsigned pwm3_2_pin[] = {0x3F};

static const unsigned wdt_pin[] = {0x5A};  // nrst

static const unsigned rtc_0_pin[] = {74};
static const unsigned rtc_1_pin[] = {83};

static const unsigned etimer0_0_pin[] = {0x26}; // tgl
static const unsigned etimer0_1_pin[] = {0x27}; // cap
static const unsigned etimer0_2_pin[] = {0x12}; // tgl
static const unsigned etimer0_3_pin[] = {0x13}; // cap
static const unsigned etimer1_0_pin[] = {0x10}; // tgl
static const unsigned etimer1_1_pin[] = {0x11}; // cap
static const unsigned etimer1_2_pin[] = {0x28}; // tgl
static const unsigned etimer1_3_pin[] = {0x29}; // cap
static const unsigned etimer2_0_pin[] = {0x2A}; // tgl
static const unsigned etimer2_1_pin[] = {0x2B}; // cap
static const unsigned etimer2_2_pin[] = {0x5B}; // tgl
static const unsigned etimer2_3_pin[] = {0x5C}; // cap
static const unsigned etimer3_0_pin[] = {0x2C}; // tgl
static const unsigned etimer3_1_pin[] = {0x2D}; // cap
static const unsigned etimer3_2_pin[] = {0x5D}; // tgl
static const unsigned etimer3_3_pin[] = {0x5E}; // cap

static const unsigned jtag_pins[] = {0x90, 0x91, 0x92, 0x93, 0x94};

static const unsigned eint0_0_pin[] = {0x5B};
static const unsigned eint0_1_pin[] = {0x70};
static const unsigned eint1_0_pin[] = {0x5C};
static const unsigned eint1_1_pin[] = {0x71};
static const unsigned eint2_0_pin[] = {0x5D};
static const unsigned eint2_1_pin[] = {0x72};
static const unsigned eint3_0_pin[] = {0x5E};
static const unsigned eint3_1_pin[] = {0x73};
static const unsigned eint4_0_pin[] = {0x5F};
static const unsigned eint4_1_pin[] = {0x74};
static const unsigned eint5_0_pin[] = {0x6F};
static const unsigned eint5_1_pin[] = {0x75};
static const unsigned eint6_0_pin[] = {0x76};
static const unsigned eint6_1_pin[] = {0x81};
static const unsigned eint7_0_pin[] = {0x77};
static const unsigned eint7_1_pin[] = {0x82};

static const unsigned ebi0_pin[] = {0x38};  // nCS0
static const unsigned ebi1_pin[] = {0x39};  // nCS1
static const unsigned ebi2_pin[] = {0x3A};  // nCS2
static const unsigned ebi3_pin[] = {0x3B};  // nCS3
static const unsigned ebi4_pin[] = {0x3C};  // nCS4
static const unsigned ebi_0_pin[] = {0x3D, 0x3E, 0x3F, 0x74, 0x75, 0x76, 0x77, 0x78,
				     0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x80, 0x81, 0x82,
				     0x83, 0x84, 0x85, 0x86, 0x87};  // 8-bit

static const unsigned ebi_1_pin[] = {0x7E, 0x7F, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F};
// TODO: CKO

static const struct nuc970_pinctrl_group nuc970_pinctrl_groups[] = {
	{
		.name = "emac0_grp",
		.pins = emac0_pins,
		.num_pins = ARRAY_SIZE(emac0_pins),
		.func = 0x1,
	},
	{
		.name = "emac1_grp",
		.pins = emac1_pins,
		.num_pins = ARRAY_SIZE(emac1_pins),
		.func = 0x1,
	},
	{
		.name = "pps0_grp",
		.pins = pps0_pin,
		.num_pins = ARRAY_SIZE(pps0_pin),
		.func = 0x1,
	},
	{
		.name = "pps1_grp",
		.pins = pps1_pin,
		.num_pins = ARRAY_SIZE(pps1_pin),
		.func = 0x1,
	},
	{
		.name = "lcd0_grp",
		.pins = lcd_0_pins,
		.num_pins = ARRAY_SIZE(lcd_0_pins),
		.func = 0x2,
	},
	{
		.name = "lcd1_grp",
		.pins = lcd_1_pins,
		.num_pins = ARRAY_SIZE(lcd_1_pins),
		.func = 0x2,
	},
	{
		.name = "lcd2_grp",
		.pins = lcd_2_pins,
		.num_pins = ARRAY_SIZE(lcd_2_pins),
		.func = 0x2,
	},
	{
		.name = "vcap_grp",
		.pins = vcap_pins,
		.num_pins = ARRAY_SIZE(vcap_pins),
		.func = 0x3,
	},
	{
		.name = "kpi_0_grp",
		.pins = kpi_0_pins,
		.num_pins = ARRAY_SIZE(kpi_0_pins),
		.func = 0x4,
	},
	{
		.name = "kpi_1_grp",
		.pins = kpi_1_pins,
		.num_pins = ARRAY_SIZE(kpi_1_pins),
		.func = 0x4,
	},
	{
		.name = "kpi_2_grp",
		.pins = kpi_2_pins,
		.num_pins = ARRAY_SIZE(kpi_2_pins),
		.func = 0x4,
	},
	{
		.name = "kpi_3_grp",
		.pins = kpi_3_pins,
		.num_pins = ARRAY_SIZE(kpi_3_pins),
		.func = 0x4,
	},
	{
		.name = "kpi_4_grp",
		.pins = kpi_4_pins,
		.num_pins = ARRAY_SIZE(kpi_4_pins),
		.func = 0x4,
	},
	{
		.name = "kpi_5_grp",
		.pins = kpi_5_pins,
		.num_pins = ARRAY_SIZE(kpi_5_pins),
		.func = 0x4,
	},
	{
		.name = "kpi_6_grp",
		.pins = kpi_6_pins,
		.num_pins = ARRAY_SIZE(kpi_6_pins),
		.func = 0x4,
	},
	{
		.name = "kpi_7_grp",
		.pins = kpi_7_pins,
		.num_pins = ARRAY_SIZE(kpi_7_pins),
		.func = 0x4,
	},
	{
		.name = "sd0_grp",
		.pins = sd0_pins,
		.num_pins = ARRAY_SIZE(sd0_pins),
		.func = 0x6,
	},
	{
		.name = "sd1_0_grp",
		.pins = sd1_0_pins,
		.num_pins = ARRAY_SIZE(sd1_0_pins),
		.func = 0x4,
	},
	{
		.name = "sd1_1_grp",
		.pins = sd1_1_pins,
		.num_pins = ARRAY_SIZE(sd1_1_pins),
		.func = 0x6,
	},
	{
		.name = "sd1_2_grp",
		.pins = sd1_2_pins,
		.num_pins = ARRAY_SIZE(sd1_2_pins),
		.func = 0x6,
	},
	{
		.name = "sd01_0_grp",
		.pins = sd01_0_pins,
		.num_pins = ARRAY_SIZE(sd01_0_pins),
		.func = 0x6,
	},
	{
		.name = "sd01_1_grp",
		.pins = sd01_1_pins,
		.num_pins = ARRAY_SIZE(sd01_1_pins),
		.func = 0x6,
	},
	{
		.name = "sd01_2_grp",
		.pins = sd01_2_pins,
		.num_pins = ARRAY_SIZE(sd01_2_pins),
		.func = 0x6,
	},
	{
		.name = "nand_0_grp",
		.pins = nand_0_pins,
		.num_pins = ARRAY_SIZE(nand_0_pins),
		.func = 0x5,
	},
	{
		.name = "nand_1_grp",
		.pins = nand_1_pins,
		.num_pins = ARRAY_SIZE(nand_1_pins),
		.func = 0x5,
	},
	{
		.name = "nand_2_grp",
		.pins = nand_2_pins,
		.num_pins = ARRAY_SIZE(nand_2_pins),
		.func = 0x5,
	},
	{
		.name = "nand_3_grp",
		.pins = nand_3_pins,
		.num_pins = ARRAY_SIZE(nand_3_pins),
		.func = 0x5,
	},
	{
		.name = "emmc_0_grp",
		.pins = emmc_0_pins,
		.num_pins = ARRAY_SIZE(emmc_0_pins),
		.func = 0x6,
	},
	{
		.name = "emmc_1_grp",
		.pins = emmc_1_pins,
		.num_pins = ARRAY_SIZE(emmc_1_pins),
		.func = 0x6,
	},
	{
		.name = "usbh_pe_grp",
		.pins = usbh_pe_pins,
		.num_pins = ARRAY_SIZE(usbh_pe_pins),
		.func = 0x7,
	},
	{
		.name = "usbh_pf_grp",
		.pins = usbh_pf_pin,
		.num_pins = ARRAY_SIZE(usbh_pf_pin),
		.func = 0x7,
	},
	{
		.name = "usbh_oc_grp",
		.pins = usbh_oc_pin,
		.num_pins = ARRAY_SIZE(usbh_oc_pin),
		.func = 0x7,
	},
	{
		.name = "usbd_grp",
		.pins = usbd_pin,
		.num_pins = ARRAY_SIZE(usbd_pin),
		.func = 0x7,
	},
	{
		.name = "i2c0_grp",
		.pins = i2c0_pins,
		.num_pins = ARRAY_SIZE(i2c0_pins),
		.func = 0x8,
	},
	{
		.name = "i2c1_0_grp",
		.pins = i2c1_0_pins,
		.num_pins = ARRAY_SIZE(i2c1_0_pins),
		.func = 0x8,
	},
	{
		.name = "i2c1_1_grp",
		.pins = i2c1_1_pins,
		.num_pins = ARRAY_SIZE(i2c1_1_pins),
		.func = 0x8,
	},
	{
		.name = "i2c1_2_grp",
		.pins = i2c1_2_pins,
		.num_pins = ARRAY_SIZE(i2c1_2_pins),
		.func = 0x8,
	},
	{
		.name = "i2c1_3_grp",
		.pins = i2c1_3_pins,
		.num_pins = ARRAY_SIZE(i2c1_3_pins),
		.func = 0x8,
	},
	{
		.name = "i2s_grp",
		.pins = i2s_pins,
		.num_pins = ARRAY_SIZE(i2s_pins),
		.func = 0x8,
	},
	{
		.name = "uart0_grp",
		.pins = uart0_pins,
		.num_pins = ARRAY_SIZE(uart0_pins),
		.func = 0x9,
	},
	{
		.name = "uart1_0_grp",
		.pins = uart1_0_pins,
		.num_pins = ARRAY_SIZE(uart1_0_pins),
		.func = 0x9,
	},
	{
		.name = "uart1_1_grp",
		.pins = uart1_1_pins,
		.num_pins = ARRAY_SIZE(uart1_1_pins),
		.func = 0x9,
	},
	{
		.name = "uart1_2_grp",
		.pins = uart1_2_pins,
		.num_pins = ARRAY_SIZE(uart1_2_pins),
		.func = 0x9,
	},
	{
		.name = "uart1_3_grp",
		.pins = uart1_3_pins,
		.num_pins = ARRAY_SIZE(uart1_3_pins),
		.func = 0x9,
	},
	{
		.name = "uart1_4_grp",
		.pins = uart1_4_pins,
		.num_pins = ARRAY_SIZE(uart1_4_pins),
		.func = 0x9,
	},
	{
		.name = "uart1_5_grp",
		.pins = uart1_5_pins,
		.num_pins = ARRAY_SIZE(uart1_5_pins),
		.func = 0x9,
	},
	{
		.name = "uart1_6_grp",
		.pins = uart1_6_pins,
		.num_pins = ARRAY_SIZE(uart1_6_pins),
		.func = 0x9,
	},
	{
		.name = "uart2_0_grp",
		.pins = uart2_0_pins,
		.num_pins = ARRAY_SIZE(uart2_0_pins),
		.func = 0x9,
	},
	{
		.name = "uart2_1_grp",
		.pins = uart2_1_pins,
		.num_pins = ARRAY_SIZE(uart2_1_pins),
		.func = 0x9,
	},
	{
		.name = "uart3_grp",
		.pins = uart3_pins,
		.num_pins = ARRAY_SIZE(uart3_pins),
		.func = 0xA,
	},
	{
		.name = "uart4_0_grp",
		.pins = uart4_0_pins,
		.num_pins = ARRAY_SIZE(uart4_0_pins),
		.func = 0x9,
	},
	{
		.name = "uart4_1_grp",
		.pins = uart4_1_pins,
		.num_pins = ARRAY_SIZE(uart4_1_pins),
		.func = 0x9,
	},
	{
		.name = "uart4_2_grp",
		.pins = uart4_2_pins,
		.num_pins = ARRAY_SIZE(uart4_2_pins),
		.func = 0x9,
	},
	{
		.name = "uart4_3_grp",
		.pins = uart4_3_pins,
		.num_pins = ARRAY_SIZE(uart4_3_pins),
		.func = 0x9,
	},
	{
		.name = "uart4_4_grp",
		.pins = uart4_4_pins,
		.num_pins = ARRAY_SIZE(uart4_4_pins),
		.func = 0x9,
	},
	{
		.name = "uart5_grp",
		.pins = uart5_pins,
		.num_pins = ARRAY_SIZE(uart5_pins),
		.func = 0x9,
	},
	{
		.name = "uart6_0_grp",
		.pins = uart6_0_pins,
		.num_pins = ARRAY_SIZE(uart6_0_pins),
		.func = 0x9,
	},
	{
		.name = "uart6_1_grp",
		.pins = uart6_1_pins,
		.num_pins = ARRAY_SIZE(uart6_1_pins),
		.func = 0x9,
	},
	{
		.name = "uart6_2_grp",
		.pins = uart6_2_pins,
		.num_pins = ARRAY_SIZE(uart6_2_pins),
		.func = 0x9,
	},
	{
		.name = "uart6_3_grp",
		.pins = uart6_3_pins,
		.num_pins = ARRAY_SIZE(uart6_3_pins),
		.func = 0x9,
	},
	{
		.name = "uart7_0_grp",
		.pins = uart7_0_pins,
		.num_pins = ARRAY_SIZE(uart7_0_pins),
		.func = 0x9,
	},
	{
		.name = "uart7_1_grp",
		.pins = uart7_1_pins,
		.num_pins = ARRAY_SIZE(uart7_1_pins),
		.func = 0x9,
	},
	{
		.name = "uart8_0_grp",
		.pins = uart8_0_pins,
		.num_pins = ARRAY_SIZE(uart8_0_pins),
		.func = 0x9,
	},
	{
		.name = "uart8_1_grp",
		.pins = uart8_1_pins,
		.num_pins = ARRAY_SIZE(uart8_1_pins),
		.func = 0x9,
	},
	{
		.name = "uart8_2_grp",
		.pins = uart8_2_pins,
		.num_pins = ARRAY_SIZE(uart8_2_pins),
		.func = 0x9,
	},
	{
		.name = "uart8_3_grp",
		.pins = uart8_3_pins,
		.num_pins = ARRAY_SIZE(uart8_3_pins),
		.func = 0x9,
	},
	{
		.name = "uart8_4_grp",
		.pins = uart8_4_pins,
		.num_pins = ARRAY_SIZE(uart8_4_pins),
		.func = 0x9,
	},
	{
		.name = "uart8_5_grp",
		.pins = uart8_5_pins,
		.num_pins = ARRAY_SIZE(uart8_5_pins),
		.func = 0x9,
	},
	{
		.name = "uart9_0_grp",
		.pins = uart9_0_pins,
		.num_pins = ARRAY_SIZE(uart9_0_pins),
		.func = 0x9,
	},
	{
		.name = "uart9_1_grp",
		.pins = uart9_1_pins,
		.num_pins = ARRAY_SIZE(uart9_1_pins),
		.func = 0x9,
	},
	{
		.name = "uart9_2_grp",
		.pins = uart9_2_pins,
		.num_pins = ARRAY_SIZE(uart9_2_pins),
		.func = 0x9,
	},
	{
		.name = "uart10_0_grp",
		.pins = uart10_0_pins,
		.num_pins = ARRAY_SIZE(uart10_0_pins),
		.func = 0x9,
	},
	{
		.name = "uart10_1_grp",
		.pins = uart10_1_pins,
		.num_pins = ARRAY_SIZE(uart10_1_pins),
		.func = 0x9,
	},
	{
		.name = "uart10_2_grp",
		.pins = uart10_2_pins,
		.num_pins = ARRAY_SIZE(uart10_2_pins),
		.func = 0x9,
	},
	{
		.name = "uart10_3_grp",
		.pins = uart10_3_pins,
		.num_pins = ARRAY_SIZE(uart10_3_pins),
		.func = 0x9,
	},
	{
		.name = "uart10_4_grp",
		.pins = uart10_4_pins,
		.num_pins = ARRAY_SIZE(uart10_4_pins),
		.func = 0x9,
	},
	{
		.name = "sc0_0_grp",
		.pins = sc0_0_pins,
		.num_pins = ARRAY_SIZE(sc0_0_pins),
		.func = 0xA,
	},
	{
		.name = "sc0_1_grp",
		.pins = sc0_1_pins,
		.num_pins = ARRAY_SIZE(sc0_1_pins),
		.func = 0xA,
	},
	{
		.name = "sc0_2_grp",
		.pins = sc0_2_pins,
		.num_pins = ARRAY_SIZE(sc0_2_pins),
		.func = 0xA,
	},
	{
		.name = "sc0_3_grp",
		.pins = sc0_3_pins,
		.num_pins = ARRAY_SIZE(sc0_3_pins),
		.func = 0xA,
	},
	{
		.name = "sc1_0_grp",
		.pins = sc1_0_pins,
		.num_pins = ARRAY_SIZE(sc1_0_pins),
		.func = 0xA,
	},
	{
		.name = "sc1_1_grp",
		.pins = sc1_1_pins,
		.num_pins = ARRAY_SIZE(sc1_1_pins),
		.func = 0xA,
	},
	{
		.name = "spi0_0_grp",
		.pins = spi0_0_pins,
		.num_pins = ARRAY_SIZE(spi0_0_pins),
		.func = 0xB,
	},
	{
		.name = "spi0_1_grp",
		.pins = spi0_1_pins,
		.num_pins = ARRAY_SIZE(spi0_1_pins),
		.func = 0xB,

	},
	{
		.name = "spi0_2_grp",
		.pins = spi0_2_pins,
		.num_pins = ARRAY_SIZE(spi0_2_pins),
		.func = 0xB,
	},
	{
		.name = "spi0_3_grp",
		.pins = spi0_3_pins,
		.num_pins = ARRAY_SIZE(spi0_3_pins),
		.func = 0xB,
	},
	{
		.name = "spi1_0_grp",
		.pins = spi1_0_pins,
		.num_pins = ARRAY_SIZE(spi1_0_pins),
		.func = 0xB,
	},
	{
		.name = "spi1_1_grp",
		.pins = spi1_1_pins,
		.num_pins = ARRAY_SIZE(spi1_1_pins),
		.func = 0xB,
	},
	{
		.name = "spi1_2_grp",
		.pins = spi1_2_pins,
		.num_pins = ARRAY_SIZE(spi1_2_pins),
		.func = 0xB,
	},
	{
		.name = "spi1_3_grp",
		.pins = spi1_3_pins,
		.num_pins = ARRAY_SIZE(spi1_3_pins),
		.func = 0xB,
	},
	{
		.name = "spi1_4_grp",
		.pins = spi1_4_pins,
		.num_pins = ARRAY_SIZE(spi1_4_pins),
		.func = 0xB,
	},
	{
		.name = "can0_0_grp",
		.pins = can0_0_pins,
		.num_pins = ARRAY_SIZE(can0_0_pins),
		.func = 0xC,
	},
	{
		.name = "can0_1_grp",
		.pins = can0_1_pins,
		.num_pins = ARRAY_SIZE(can0_1_pins),
		.func = 0xC,
	},
	{
		.name = "can0_2_grp",
		.pins = can0_2_pins,
		.num_pins = ARRAY_SIZE(can0_2_pins),
		.func = 0xC,
	},
	{
		.name = "can1_grp",
		.pins = can1_pins,
		.num_pins = ARRAY_SIZE(can1_pins),
		.func = 0xC,
	},
	{
		.name = "pwm0_0_grp",
		.pins = pwm0_0_pin,
		.num_pins = ARRAY_SIZE(pwm0_0_pin),
		.func = 0xD,
	},
	{
		.name = "pwm0_1_grp",
		.pins = pwm0_1_pin,
		.num_pins = ARRAY_SIZE(pwm0_1_pin),
		.func = 0xD,
	},
	{
		.name = "pwm0_2_grp",
		.pins = pwm0_2_pin,
		.num_pins = ARRAY_SIZE(pwm0_2_pin),
		.func = 0xD,
	},
	{
		.name = "pwm0_3_grp",
		.pins = pwm0_3_pin,
		.num_pins = ARRAY_SIZE(pwm0_3_pin),
		.func = 0xD,
	},
	{
		.name = "pwm1_0_grp",
		.pins = pwm1_0_pin,
		.num_pins = ARRAY_SIZE(pwm1_0_pin),
		.func = 0xD,
	},
	{
		.name = "pwm1_1_grp",
		.pins = pwm1_1_pin,
		.num_pins = ARRAY_SIZE(pwm1_1_pin),
		.func = 0xD,
	},
	{
		.name = "pwm1_2_grp",
		.pins = pwm1_2_pin,
		.num_pins = ARRAY_SIZE(pwm1_2_pin),
		.func = 0xD,
	},
	{
		.name = "pwm2_0_grp",
		.pins = pwm2_0_pin,
		.num_pins = ARRAY_SIZE(pwm2_0_pin),
		.func = 0xD,
	},
	{
		.name = "pwm2_1_grp",
		.pins = pwm2_1_pin,
		.num_pins = ARRAY_SIZE(pwm2_1_pin),
		.func = 0xD,
	},
	{
		.name = "pwm2_2_grp",
		.pins = pwm2_2_pin,
		.num_pins = ARRAY_SIZE(pwm2_2_pin),
		.func = 0xD,
	},
	{
		.name = "pwm3_0_grp",
		.pins = pwm3_0_pin,
		.num_pins = ARRAY_SIZE(pwm3_0_pin),
		.func = 0xD,
	},
	{
		.name = "pwm3_1_grp",
		.pins = pwm3_1_pin,
		.num_pins = ARRAY_SIZE(pwm3_1_pin),
		.func = 0xD,
	},
	{
		.name = "pwm3_2_grp",
		.pins = pwm3_2_pin,
		.num_pins = ARRAY_SIZE(pwm3_2_pin),
		.func = 0xD,
	},
	{
		.name = "wdt_grp",
		.pins = wdt_pin,
		.num_pins = ARRAY_SIZE(wdt_pin),
		.func = 0xD,
	},
	{
		.name = "rtc_0_grp",
		.pins = rtc_0_pin,
		.num_pins = ARRAY_SIZE(rtc_0_pin),
		.func = 0xD,
	},
	{
		.name = "rtc_1_grp",
		.pins = rtc_1_pin,
		.num_pins = ARRAY_SIZE(rtc_1_pin),
		.func = 0xD,
	},
	{
		.name = "etimer0_0_grp",
		.pins = etimer0_0_pin,
		.num_pins = ARRAY_SIZE(etimer0_0_pin),
		.func = 0xD,
	},
	{
		.name = "etimer0_1_grp",
		.pins = etimer0_1_pin,
		.num_pins = ARRAY_SIZE(etimer0_1_pin),
		.func = 0xD,
	},
	{
		.name = "etimer0_2_grp",
		.pins = etimer0_2_pin,
		.num_pins = ARRAY_SIZE(etimer0_2_pin),
		.func = 0xF,
	},
	{
		.name = "etimer0_3_grp",
		.pins = etimer0_3_pin,
		.num_pins = ARRAY_SIZE(etimer0_3_pin),
		.func = 0xF,
	},
	{
		.name = "etimer1_0_grp",
		.pins = etimer1_0_pin,
		.num_pins = ARRAY_SIZE(etimer1_0_pin),
		.func = 0xD,
	},
	{
		.name = "etimer1_1_grp",
		.pins = etimer1_1_pin,
		.num_pins = ARRAY_SIZE(etimer1_1_pin),
		.func = 0xD,
	},
	{
		.name = "etimer1_2_grp",
		.pins = etimer1_2_pin,
		.num_pins = ARRAY_SIZE(etimer1_2_pin),
		.func = 0xD,
	},
	{
		.name = "etimer1_3_grp",
		.pins = etimer1_3_pin,
		.num_pins = ARRAY_SIZE(etimer1_3_pin),
		.func = 0xD,
	},
	{
		.name = "etimer2_0_grp",
		.pins = etimer2_0_pin,
		.num_pins = ARRAY_SIZE(etimer2_0_pin),
		.func = 0xD,
	},
	{
		.name = "etimer2_1_grp",
		.pins = etimer2_1_pin,
		.num_pins = ARRAY_SIZE(etimer2_1_pin),
		.func = 0xD,
	},
	{
		.name = "etimer2_2_grp",
		.pins = etimer2_2_pin,
		.num_pins = ARRAY_SIZE(etimer2_2_pin),
		.func = 0xD,
	},
	{
		.name = "etimer2_3_grp",
		.pins = etimer2_3_pin,
		.num_pins = ARRAY_SIZE(etimer2_3_pin),
		.func = 0xD,
	},
	{
		.name = "etimer3_0_grp",
		.pins = etimer3_0_pin,
		.num_pins = ARRAY_SIZE(etimer3_0_pin),
		.func = 0xD,
	},
	{
		.name = "etimer3_1_grp",
		.pins = etimer3_1_pin,
		.num_pins = ARRAY_SIZE(etimer3_1_pin),
		.func = 0xD,
	},
	{
		.name = "etimer3_2_grp",
		.pins = etimer3_2_pin,
		.num_pins = ARRAY_SIZE(etimer3_2_pin),
		.func = 0xD,
	},
	{
		.name = "etimer3_3_grp",
		.pins = etimer3_3_pin,
		.num_pins = ARRAY_SIZE(etimer3_3_pin),
		.func = 0xD,
	},
	{
		.name = "jtag_grp",
		.pins = jtag_pins,
		.num_pins = ARRAY_SIZE(jtag_pins),
		.func = 0xF,
	},
	{
		.name = "eint0_0_grp",
		.pins = eint0_0_pin,
		.num_pins = ARRAY_SIZE(eint0_0_pin),
		.func = 0xF,
	},
	{
		.name = "eint0_1_grp",
		.pins = eint0_1_pin,
		.num_pins = ARRAY_SIZE(eint0_1_pin),
		.func = 0xF,
	},
	{
		.name = "eint1_0_grp",
		.pins = eint1_0_pin,
		.num_pins = ARRAY_SIZE(eint1_0_pin),
		.func = 0xF,
	},
	{
		.name = "eint1_1_grp",
		.pins = eint1_1_pin,
		.num_pins = ARRAY_SIZE(eint1_1_pin),
		.func = 0xF,
	},
	{
		.name = "eint2_0_grp",
		.pins = eint2_0_pin,
		.num_pins = ARRAY_SIZE(eint2_0_pin),
		.func = 0xF,
	},
	{
		.name = "eint2_1_grp",
		.pins = eint2_1_pin,
		.num_pins = ARRAY_SIZE(eint2_1_pin),
		.func = 0xF,
	},
	{
		.name = "eint3_0_grp",
		.pins = eint3_0_pin,
		.num_pins = ARRAY_SIZE(eint3_0_pin),
		.func = 0xF,
	},
	{
		.name = "eint3_1_grp",
		.pins = eint3_1_pin,
		.num_pins = ARRAY_SIZE(eint3_1_pin),
		.func = 0xF,
	},
	{
		.name = "eint4_0_grp",
		.pins = eint4_0_pin,
		.num_pins = ARRAY_SIZE(eint4_0_pin),
		.func = 0xF,
	},
	{
		.name = "eint4_1_grp",
		.pins = eint4_1_pin,
		.num_pins = ARRAY_SIZE(eint4_1_pin),
		.func = 0xF,
	},
	{
		.name = "eint5_0_grp",
		.pins = eint5_0_pin,
		.num_pins = ARRAY_SIZE(eint5_0_pin),
		.func = 0xF,
	},
	{
		.name = "eint5_1_grp",
		.pins = eint5_1_pin,
		.num_pins = ARRAY_SIZE(eint5_1_pin),
		.func = 0xF,
	},
	{
		.name = "eint6_0_grp",
		.pins = eint6_0_pin,
		.num_pins = ARRAY_SIZE(eint6_0_pin),
		.func = 0xF,
	},
	{
		.name = "eint6_1_grp",
		.pins = eint6_1_pin,
		.num_pins = ARRAY_SIZE(eint6_1_pin),
		.func = 0xF,
	},
	{
		.name = "eint7_0_grp",
		.pins = eint7_0_pin,
		.num_pins = ARRAY_SIZE(eint7_0_pin),
		.func = 0xF,
	},
	{
		.name = "eint7_1_grp",
		.pins = eint7_1_pin,
		.num_pins = ARRAY_SIZE(eint7_1_pin),
		.func = 0xF,
	},
	{
		.name = "ebi0_grp",
		.pins = ebi0_pin,
		.num_pins = ARRAY_SIZE(ebi0_pin),
		.func = 0xE,
	},
	{
		.name = "ebi1_grp",
		.pins = ebi1_pin,
		.num_pins = ARRAY_SIZE(ebi1_pin),
		.func = 0xE,
	},
	{
		.name = "ebi2_grp",
		.pins = ebi2_pin,
		.num_pins = ARRAY_SIZE(ebi2_pin),
		.func = 0xE,
	},
	{
		.name = "ebi3_grp",
		.pins = ebi3_pin,
		.num_pins = ARRAY_SIZE(ebi3_pin),
		.func = 0xE,
	},
	{
		.name = "ebi4_grp",
		.pins = ebi4_pin,
		.num_pins = ARRAY_SIZE(ebi4_pin),
		.func = 0xF,
	},
	{
		.name = "ebi_0_grp",
		.pins = ebi_0_pin,
		.num_pins = ARRAY_SIZE(ebi0_pin),
		.func = 0xE,
	},
	{
		.name = "ebi_1_grp",
		.pins = ebi_1_pin,
		.num_pins = ARRAY_SIZE(ebi_1_pin),
		.func = 0xE,
	},
};

static int nuc970_get_groups_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(nuc970_pinctrl_groups);
}

static const char *nuc970_get_group_name(struct pinctrl_dev *pctldev,
				       unsigned selector)
{;
	return nuc970_pinctrl_groups[selector].name;
}

static int nuc970_get_group_pins(struct pinctrl_dev *pctldev, unsigned selector,
			       const unsigned ** pins,
			       unsigned * num_pins)
{
	*pins = (unsigned *) nuc970_pinctrl_groups[selector].pins;
	*num_pins = nuc970_pinctrl_groups[selector].num_pins;
	return 0;
}

static struct pinctrl_ops nuc970_pctrl_ops = {
	.get_groups_count = nuc970_get_groups_count,
	.get_group_name = nuc970_get_group_name,
	.get_group_pins = nuc970_get_group_pins,
};

struct nuc970_pmx_func {
	const char *name;
	const char * const *groups;
	const unsigned num_groups;
};

static const char * const emac0_groups[] = {"emac0_grp"};
static const char * const emac1_groups[] = {"emac1_grp"};
static const char * const pps0_groups[] = {"pps0_grp"};
static const char * const pps1_groups[] = {"pps1_grp"};
static const char * const lcd0_groups[] = {"lcd0_grp"};
static const char * const lcd1_groups[] = {"lcd1_grp"};
static const char * const lcd2_groups[] = {"lcd2_grp"};
static const char * const vcap_groups[] = {"vcap_grp"};
static const char * const kpi_row_groups[] = {"kpi_0_grp", "kpi_4_grp"};
static const char * const kpi_2col_groups[] = {"kpi_1_grp", "kpi_5_grp"};
static const char * const kpi_4col_groups[] = {"kpi_2_grp", "kpi_6_grp"};
static const char * const kpi_8col_groups[] = {"kpi_3_grp", "kpi_7_grp"};
static const char * const sd0_groups[] = {"sd0_grp"};
static const char * const sd1_groups[] = {"sd1_0_grp", "sd1_1_grp", "sd1_2_grp"};
static const char * const sd01_groups[] = {"sd01_0_grp", "sd01_1_grp", "sd01_2_grp"};
static const char * const nand_groups[] = {"nand_0_grp", "nand_1_grp"};
static const char * const nand_ctl1_groups[] = {"nand_2_grp", "nand_3_grp"};
static const char * const emmc_groups[] = {"emmc_0_grp", "emmc_1_grp"};
static const char * const usbh_ppwr_groups[] = {"usbh_pe_grp", "usbh_pf_grp", "usbh_oc_grp" };
static const char * const usbd_groups[] = {"usbd_grp"};
static const char * const i2c0_groups[] = {"i2c0_grp"};
static const char * const i2c1_groups[] = {"i2c1_0_grp", "i2c1_1_grp", "i2c1_2_grp", "i2c1_3_grp"};
static const char * const i2s_groups[] = {"i2s_grp"};
static const char * const uart0_groups[] = {"uart0_grp"};
static const char * const uart1_groups[] = {"uart1_0_grp", "uart1_3_grp", "uart1_5_grp"};
static const char * const uart1_fc_groups[] = {"uart1_1_grp", "uart1_4_grp", "uart1_6_grp"};
static const char * const uart1_ff_groups[] = {"uart1_2_grp"};
static const char * const uart2_groups[] = {"uart2_0_grp"};
static const char * const uart2_fc_groups[] = {"uart2_1_grp"};
static const char * const uart3_groups[] = {"uart3_grp"};
static const char * const uart4_groups[] = {"uart4_0_grp", "uart4_2_grp", "uart4_4_grp"};
static const char * const uart4_fc_groups[] = {"uart4_1_grp", "uart4_3_grp"};
static const char * const uart5_groups[] = {"uart5_grp"};
static const char * const uart6_groups[] = {"uart6_0_grp", "uart6_2_grp"};
static const char * const uart6_fc_groups[] = {"uart6_1_grp", "uart6_3_grp"};
static const char * const uart7_groups[] = {"uart7_0_grp", "uart7_1_grp"};
static const char * const uart8_groups[] = {"uart8_0_grp", "uart8_2_grp", "uart8_4_grp"};
static const char * const uart8_fc_groups[] = {"uart8_1_grp", "uart8_3_grp", "uart8_5_grp"};
static const char * const uart9_groups[] = {"uart9_0_grp", "uart9_1_grp", "uart9_2_grp"};
static const char * const uart10_groups[] = {"uart10_0_grp", "uart10_1_grp", "uart10_3_grp"};
static const char * const uart10_fc_groups[] = {"uart10_2_grp", "uart10_4_grp"};
static const char * const sc0_groups[] = {"sc0_0_grp", "sc0_1_grp"};
static const char * const sc1_groups[] = {"sc1_0_grp"};
static const char * const scuart0_groups[] = {"sc0_2_grp", "sc0_3_grp"};
static const char * const scuart1_groups[] = {"sc1_1_grp"};
static const char * const spi0_groups[] = {"spi0_0_grp"};
static const char * const spi0_quad_groups[] = {"spi0_1_grp"};
static const char * const spi0_ss1_groups[] = {"spi0_2_grp", "spi0_3_grp"};
static const char * const spi1_groups[] = {"spi1_0_grp", "spi1_4_grp"};
static const char * const spi1_quad_groups[] = {"spi1_2_grp"};
static const char * const spi1_ss1_groups[] = {"spi1_1_grp", "spi1_3_grp"};
static const char * const can0_groups[] = {"can0_0_grp", "can0_1_grp", "can0_2_grp"};
static const char * const can1_groups[] = {"can1_grp"};
static const char * const pwm0_groups[] = {"pwm0_0_grp", "pwm0_1_grp", "pwm0_2_grp", "pwm0_3_grp"};
static const char * const pwm1_groups[] = {"pwm1_0_grp", "pwm1_1_grp", "pwm1_2_grp"};
static const char * const pwm2_groups[] = {"pwm2_0_grp", "pwm2_1_grp", "pwm2_2_grp"};
static const char * const pwm3_groups[] = {"pwm3_0_grp", "pwm3_1_grp", "pwm3_2_grp"};
static const char * const wdt_groups[] = {"wdt_grp"};
static const char * const rtc_groups[] = {"rtc_0_grp", "rtc_1_grp"};
static const char * const etimer0_tgl_groups[] = {"etimer0_0_grp", "etimer0_2_grp"};
static const char * const etimer0_cap_groups[] = {"etimer0_1_grp", "etimer0_3_grp"};
static const char * const etimer1_tgl_groups[] = {"etimer1_0_grp", "etimer1_2_grp"};
static const char * const etimer1_cap_groups[] = {"etimer1_1_grp", "etimer1_3_grp"};
static const char * const etimer2_tgl_groups[] = {"etimer2_0_grp", "etimer2_2_grp"};
static const char * const etimer2_cap_groups[] = {"etimer2_1_grp", "etimer2_3_grp"};
static const char * const etimer3_tgl_groups[] = {"etimer3_0_grp", "etimer3_2_grp"};
static const char * const etimer3_cap_groups[] = {"etimer3_1_grp", "etimer3_3_grp"};
static const char * const jtag_groups[] = {"jtag_grp"};
static const char * const eint0_groups[] = {"eint0_0_grp", "eint0_1_grp"};
static const char * const eint1_groups[] = {"eint1_0_grp", "eint1_1_grp"};
static const char * const eint2_groups[] = {"eint2_0_grp", "eint2_1_grp"};
static const char * const eint3_groups[] = {"eint3_0_grp", "eint3_1_grp"};
static const char * const eint4_groups[] = {"eint4_0_grp", "eint4_1_grp"};
static const char * const eint5_groups[] = {"eint5_0_grp", "eint5_1_grp"};
static const char * const eint6_groups[] = {"eint6_0_grp", "eint6_1_grp"};
static const char * const eint7_groups[] = {"eint7_0_grp", "eint7_1_grp"};
static const char * const ebi0_group[] = {"ebi0_grp"};
static const char * const ebi1_group[] = {"ebi1_grp"};
static const char * const ebi2_group[] = {"ebi2_grp"};
static const char * const ebi3_group[] = {"ebi3_grp"};
static const char * const ebi4_group[] = {"ebi4_grp"};
static const char * const ebi_8_group[] = {"ebi_0_grp"};
static const char * const ebi_16_group[] = {"ebi_1_grp"};


static const struct nuc970_pmx_func nuc970_functions[] = {
	{
		.name = "emac0",
		.groups = emac0_groups,
		.num_groups = ARRAY_SIZE(emac0_groups),
	},
	{
		.name = "emac1",
		.groups = emac1_groups,
		.num_groups = ARRAY_SIZE(emac1_groups),
	},
	{
		.name = "pps0",
		.groups = pps0_groups,
		.num_groups = ARRAY_SIZE(pps0_groups),
	},
	{
		.name = "pps1",
		.groups = pps1_groups,
		.num_groups = ARRAY_SIZE(pps1_groups),
	},
	{
		.name = "lcd0",
		.groups = lcd0_groups,
		.num_groups = ARRAY_SIZE(lcd0_groups),
	},
	{
		.name = "lcd1",
		.groups = lcd1_groups,
		.num_groups = ARRAY_SIZE(lcd1_groups),
	},
	{
		.name = "lcd2",
		.groups = lcd2_groups,
		.num_groups = ARRAY_SIZE(lcd2_groups),
	},
	{
		.name = "vcap",
		.groups = vcap_groups,
		.num_groups = ARRAY_SIZE(vcap_groups),
	},
	{
		.name = "kpi_row",
		.groups = kpi_row_groups,
		.num_groups = ARRAY_SIZE(kpi_row_groups),
	},
	{
		.name = "kpi_2col",
		.groups = kpi_2col_groups,
		.num_groups = ARRAY_SIZE(kpi_2col_groups),
	},
	{
		.name = "kpi_4col",
		.groups = kpi_4col_groups,
		.num_groups = ARRAY_SIZE(kpi_4col_groups),
	},
	{
		.name = "kpi_8col",
		.groups = kpi_8col_groups,
		.num_groups = ARRAY_SIZE(kpi_8col_groups),
	},
	{
		.name = "sd0",
		.groups = sd0_groups,
		.num_groups = ARRAY_SIZE(sd0_groups),
	},
	{
		.name = "sd1",
		.groups = sd1_groups,
		.num_groups = ARRAY_SIZE(sd1_groups),
	},
	{
		.name = "sd01",
		.groups = sd01_groups,
		.num_groups = ARRAY_SIZE(sd01_groups),
	},
	{
		.name = "nand",
		.groups = nand_groups,
		.num_groups = ARRAY_SIZE(nand_groups),
	},
	{
		.name = "nand_ctl1",
		.groups = nand_ctl1_groups,
		.num_groups = ARRAY_SIZE(nand_ctl1_groups),
	},
	{
		.name = "emmc",
		.groups = emmc_groups,
		.num_groups = ARRAY_SIZE(emmc_groups),
	},
	{
		.name = "usbh_ppwr",
		.groups = usbh_ppwr_groups,
		.num_groups = ARRAY_SIZE(usbh_ppwr_groups),
	},
	{
		.name = "usbd",
		.groups = usbd_groups,
		.num_groups = ARRAY_SIZE(usbd_groups),
	},
	{
		.name = "i2c0",
		.groups = i2c0_groups,
		.num_groups = ARRAY_SIZE(i2c0_groups),
	},
	{
		.name = "i2c1",
		.groups = i2c1_groups,
		.num_groups = ARRAY_SIZE(i2c1_groups),
	},
	{
		.name = "i2s",
		.groups = i2s_groups,
		.num_groups = ARRAY_SIZE(i2s_groups),
	},
	{
		.name = "uart0",
		.groups = uart0_groups,
		.num_groups = ARRAY_SIZE(uart0_groups),
	},
	{
		.name = "uart1",
		.groups = uart1_groups,
		.num_groups = ARRAY_SIZE(uart1_groups),
	},
	{
		.name = "uart1_fc",
		.groups = uart1_fc_groups,
		.num_groups = ARRAY_SIZE(uart1_fc_groups),
	},
	{
		.name = "uart1_ff",
		.groups = uart1_ff_groups,
		.num_groups = ARRAY_SIZE(uart1_ff_groups),
	},
	{
		.name = "uart2",
		.groups = uart2_groups,
		.num_groups = ARRAY_SIZE(uart2_groups),
	},
	{
		.name = "uart2_fc",
		.groups = uart2_fc_groups,
		.num_groups = ARRAY_SIZE(uart2_fc_groups),
	},
	{
		.name = "uart3",
		.groups = uart3_groups,
		.num_groups = ARRAY_SIZE(uart3_groups),
	},
	{
		.name = "uart4",
		.groups = uart4_groups,
		.num_groups = ARRAY_SIZE(uart4_groups),
	},
	{
		.name = "uart4_fc",
		.groups = uart4_fc_groups,
		.num_groups = ARRAY_SIZE(uart4_fc_groups),
	},
	{
		.name = "uart5",
		.groups = uart5_groups,
		.num_groups = ARRAY_SIZE(uart5_groups),
	},
	{
		.name = "uart6",
		.groups = uart6_groups,
		.num_groups = ARRAY_SIZE(uart6_groups),
	},
	{
		.name = "uart6_fc",
		.groups = uart6_fc_groups,
		.num_groups = ARRAY_SIZE(uart6_fc_groups),
	},
	{
		.name = "uart7",
		.groups = uart7_groups,
		.num_groups = ARRAY_SIZE(uart7_groups),
	},
	{
		.name = "uart8",
		.groups = uart8_groups,
		.num_groups = ARRAY_SIZE(uart8_groups),
	},
	{
		.name = "uart8_fc",
		.groups = uart8_fc_groups,
		.num_groups = ARRAY_SIZE(uart8_fc_groups),
	},
	{
		.name = "uart9",
		.groups = uart9_groups,
		.num_groups = ARRAY_SIZE(uart9_groups),
	},
	{
		.name = "uart10",
		.groups = uart10_groups,
		.num_groups = ARRAY_SIZE(uart10_groups),
	},
	{
		.name = "uart10_fc",
		.groups = uart10_fc_groups,
		.num_groups = ARRAY_SIZE(uart10_fc_groups),
	},
	{
		.name = "sc0",
		.groups = sc0_groups,
		.num_groups = ARRAY_SIZE(sc0_groups),
	},
	{
		.name = "sc1",
		.groups = sc1_groups,
		.num_groups = ARRAY_SIZE(sc1_groups),
	},
	{
		.name = "scuart0",
		.groups = scuart0_groups,
		.num_groups = ARRAY_SIZE(scuart0_groups),
	},
	{
		.name = "scuart1",
		.groups = scuart1_groups,
		.num_groups = ARRAY_SIZE(scuart1_groups),
	},
	{
		.name = "spi0",
		.groups = spi0_groups,
		.num_groups = ARRAY_SIZE(spi0_groups),
	},
	{
		.name = "spi0_quad",
		.groups = spi0_quad_groups,
		.num_groups = ARRAY_SIZE(spi0_quad_groups),
	},
	{
		.name = "spi0_ss1",
		.groups = spi0_ss1_groups,
		.num_groups = ARRAY_SIZE(spi0_ss1_groups),
	},
	{
		.name = "spi1",
		.groups = spi1_groups,
		.num_groups = ARRAY_SIZE(spi1_groups),
	},
	{
		.name = "spi1_quad",
		.groups = spi1_quad_groups,
		.num_groups = ARRAY_SIZE(spi1_quad_groups),
	},
	{
		.name = "spi1_ss1",
		.groups = spi1_ss1_groups,
		.num_groups = ARRAY_SIZE(spi1_ss1_groups),
	},
	{
		.name = "can0",
		.groups = can0_groups,
		.num_groups = ARRAY_SIZE(can0_groups),
	},
	{
		.name = "can1",
		.groups = can1_groups,
		.num_groups = ARRAY_SIZE(can1_groups),
	},
	{
		.name = "pwm0",
		.groups = pwm0_groups,
		.num_groups = ARRAY_SIZE(pwm0_groups),
	},
	{
		.name = "pwm1",
		.groups = pwm1_groups,
		.num_groups = ARRAY_SIZE(pwm1_groups),
	},
	{
		.name = "pwm2",
		.groups = pwm2_groups,
		.num_groups = ARRAY_SIZE(pwm2_groups),
	},
	{
		.name = "pwm3",
		.groups = pwm3_groups,
		.num_groups = ARRAY_SIZE(pwm3_groups),
	},
	{
		.name = "wdt",
		.groups = wdt_groups,
		.num_groups = ARRAY_SIZE(wdt_groups),
	},
	{
		.name = "rtc",
		.groups = rtc_groups,
		.num_groups = ARRAY_SIZE(rtc_groups),
	},
	{
		.name = "etimer0_tgl",
		.groups = etimer0_tgl_groups,
		.num_groups = ARRAY_SIZE(etimer0_tgl_groups),
	},
	{
		.name = "etimer0_cap",
		.groups = etimer0_cap_groups,
		.num_groups = ARRAY_SIZE(etimer0_cap_groups),
	},
	{
		.name = "etimer1_tgl",
		.groups = etimer1_tgl_groups,
		.num_groups = ARRAY_SIZE(etimer1_tgl_groups),
	},
	{
		.name = "etimer1_cap",
		.groups = etimer1_cap_groups,
		.num_groups = ARRAY_SIZE(etimer1_cap_groups),
	},
	{
		.name = "etimer2_tgl",
		.groups = etimer2_tgl_groups,
		.num_groups = ARRAY_SIZE(etimer2_tgl_groups),
	},
	{
		.name = "etimer2_cap",
		.groups = etimer2_cap_groups,
		.num_groups = ARRAY_SIZE(etimer2_cap_groups),
	},
	{
		.name = "etimer3_tgl",
		.groups = etimer3_tgl_groups,
		.num_groups = ARRAY_SIZE(etimer3_tgl_groups),
	},
	{
		.name = "etimer3_cap",
		.groups = etimer3_cap_groups,
		.num_groups = ARRAY_SIZE(etimer3_cap_groups),
	},
	{
		.name = "jtag",
		.groups = jtag_groups,
		.num_groups = ARRAY_SIZE(jtag_groups),
	},
	{
		.name = "eint0",
		.groups = eint0_groups,
		.num_groups = ARRAY_SIZE(eint0_groups),
	},
	{
		.name = "eint1",
		.groups = eint1_groups,
		.num_groups = ARRAY_SIZE(eint1_groups),
	},
	{
		.name = "eint2",
		.groups = eint2_groups,
		.num_groups = ARRAY_SIZE(eint2_groups),
	},
	{
		.name = "eint3",
		.groups = eint3_groups,
		.num_groups = ARRAY_SIZE(eint3_groups),
	},
	{
		.name = "eint4",
		.groups = eint4_groups,
		.num_groups = ARRAY_SIZE(eint4_groups),
	},
	{
		.name = "eint5",
		.groups = eint5_groups,
		.num_groups = ARRAY_SIZE(eint5_groups),
	},
	{
		.name = "eint6",
		.groups = eint6_groups,
		.num_groups = ARRAY_SIZE(eint6_groups),
	},
	{
		.name = "eint7",
		.groups = eint7_groups,
		.num_groups = ARRAY_SIZE(eint7_groups),
	},
	{
		.name = "ebi0",
		.groups = ebi0_group,
		.num_groups = ARRAY_SIZE(ebi0_group),
	},
	{
		.name = "ebi1",
		.groups = ebi1_group,
		.num_groups = ARRAY_SIZE(ebi1_group),
	},
	{
		.name = "ebi2",
		.groups = ebi2_group,
		.num_groups = ARRAY_SIZE(ebi2_group),
	},
	{
		.name = "ebi3",
		.groups = ebi3_group,
		.num_groups = ARRAY_SIZE(ebi3_group),
	},
	{
		.name = "ebi4",
		.groups = ebi4_group,
		.num_groups = ARRAY_SIZE(ebi4_group),
	},
	{
		.name = "ebi_8",
		.groups = ebi_8_group,
		.num_groups = ARRAY_SIZE(ebi_8_group),
	},
	{
		.name = "ebi_16",
		.groups = ebi_16_group,
		.num_groups = ARRAY_SIZE(ebi_16_group),
	},
};


int nuc970_get_functions_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(nuc970_functions);
}

const char *nuc970_get_fname(struct pinctrl_dev *pctldev, unsigned selector)
{
	return nuc970_functions[selector].name;
}

static int nuc970_get_groups(struct pinctrl_dev *pctldev, unsigned selector,
			  const char * const **groups,
			  unsigned * const num_groups)
{
	*groups = nuc970_functions[selector].groups;
	*num_groups = nuc970_functions[selector].num_groups;
	return 0;
}

/*
 * selector = data.nux.func, which is entry number in nuc970_functions,
 * and group = data.mux.group, which is entry number in nuc970_pmx_func
 * group is not used since some function use different setting between
 * different ports. for example UART....
 */
int nuc970_enable(struct pinctrl_dev *pctldev, unsigned selector,
		unsigned group)
{
	unsigned int i, j;
	unsigned int reg, offset;

	//printk("enable =>%x %x\n", selector, group);
	for(i = 0; i < nuc970_pinctrl_groups[group].num_pins; i++) {
		j = nuc970_pinctrl_groups[group].pins[i];
		offset = (j >> 4) * 8 + ((j & 0x8) ? 4 : 0);

		reg = __raw_readl(REG_MFP_GPA_L + offset);
		reg = (reg & ~(0xF << ((j & 0x7) * 4))) | (nuc970_pinctrl_groups[group].func << ((j & 0x7) * 4));

		__raw_writel(reg, REG_MFP_GPA_L + offset);
	}

	/* SD0 pin value is 0x6, SD1 PI pin value is 0x4, should set the correct value */
	if (strcmp(nuc970_pinctrl_groups[group].name, "sd01_0_grp") == 0)
	{
		for(i = 8; i < nuc970_pinctrl_groups[group].num_pins; i++) {
			j = nuc970_pinctrl_groups[group].pins[i];
			offset = (j >> 4) * 8 + ((j & 0x8) ? 4 : 0);

			reg = __raw_readl(REG_MFP_GPA_L + offset);
			reg = (reg & ~(0xF << ((j & 0x7) * 4))) | (0x4 << ((j & 0x7) * 4));

			__raw_writel(reg, REG_MFP_GPA_L + offset);
		}
	}
	return 0;
}

/*
 * By disable a function, we'll switch it back to GPIO
 */
void nuc970_disable(struct pinctrl_dev *pctldev, unsigned selector,
		unsigned group)
{

	unsigned int i, j;
	unsigned int reg, offset;

	//printk("disable =>%x %x\n", selector, group);
	for(i = 0; i < nuc970_pinctrl_groups[group].num_pins; i++) {
		j = nuc970_pinctrl_groups[group].pins[i];
		offset = (j >> 4) * 8 + ((j & 0x8) ? 4 : 0);

		reg = __raw_readl(REG_MFP_GPA_L + offset);
		reg &= ~(0xF << ((j & 0x7) * 4));
		__raw_writel(reg, REG_MFP_GPA_L + offset);
	}

	return;
}


struct pinmux_ops nuc970_pmxops = {
	.get_functions_count = nuc970_get_functions_count,
	.get_function_name = nuc970_get_fname,
	.get_function_groups = nuc970_get_groups,
	.enable = nuc970_enable,
	.disable = nuc970_disable,
};

static struct pinctrl_desc nuc970_pinctrl_desc = {
	.name = "nuc970-pinctrl_desc",
	.pins = nuc970_pins,
	.npins = ARRAY_SIZE(nuc970_pins),
	.pctlops = &nuc970_pctrl_ops,
	.pmxops = &nuc970_pmxops,
	.owner = THIS_MODULE,
};

static const struct pinctrl_map nuc970_pinmap[] = {
	{
		.dev_name = "nuc970-emac0",
		.name = PINCTRL_STATE_DEFAULT,
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "emac0",
		.data.mux.group = "emac0_grp",
	},
	{
		.dev_name = "nuc970-emac1",
		.name = PINCTRL_STATE_DEFAULT,
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "emac1",
		.data.mux.group = "emac1_grp",
	},
	{
		.dev_name = "nuc970-emac0",
		.name = "pps0",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "pps0",
		.data.mux.group = "pps0_grp",
	},
	{
		.dev_name = "nuc970-emac1",
		.name = "pps1",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "pps1",
		.data.mux.group = "pps1_grp",
	},
	{
		.dev_name = "nuc970-lcd",
		.name = "lcd-16bit",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "lcd0",
		.data.mux.group = "lcd0_grp",
	},
	{
		.dev_name = "nuc970-lcd",
		.name = "lcd-18bit",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "lcd0",
		.data.mux.group = "lcd0_grp",
	},
	{
		.dev_name = "nuc970-lcd",
		.name = "lcd-18bit",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "lcd1",
		.data.mux.group = "lcd1_grp",
	},
		{
		.dev_name = "nuc970-lcd",
		.name = "lcd-24bit",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "lcd0",
		.data.mux.group = "lcd0_grp",
	},
	{
		.dev_name = "nuc970-lcd",
		.name = "lcd-24bit",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "lcd1",
		.data.mux.group = "lcd1_grp",
	},
	{
		.dev_name = "nuc970-lcd",
		.name = "lcd-24bit",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "lcd2",
		.data.mux.group = "lcd2_grp",
	},
	{
		.dev_name = "nuc970-videoin",
		.name = PINCTRL_STATE_DEFAULT,
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "vcap",
		.data.mux.group = "vcap_grp",
	},
	{
		.dev_name = "nuc970-kpi",
		.name = "kpi_4x2-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "kpi_row",
		.data.mux.group = "kpi_0_grp",
	},
	{
		.dev_name = "nuc970-kpi",
		.name = "kpi_4x2-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "kpi_2col",
		.data.mux.group = "kpi_1_grp",
	},
	{
		.dev_name = "nuc970-kpi",
		.name = "kpi_4x4-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "kpi_row",
		.data.mux.group = "kpi_0_grp",
	},
	{
		.dev_name = "nuc970-kpi",
		.name = "kpi_4x4-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "kpi_2col",
		.data.mux.group = "kpi_1_grp",
	},
	{
		.dev_name = "nuc970-kpi",
		.name = "kpi_4x4-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "kpi_4col",
		.data.mux.group = "kpi_2_grp",
	},
	{
		.dev_name = "nuc970-kpi",
		.name = "kpi_4x8-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "kpi_row",
		.data.mux.group = "kpi_0_grp",
	},
	{
		.dev_name = "nuc970-kpi",
		.name = "kpi_4x8-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "kpi_2col",
		.data.mux.group = "kpi_1_grp",
	},
	{
		.dev_name = "nuc970-kpi",
		.name = "kpi_4x8-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "kpi_4col",
		.data.mux.group = "kpi_2_grp",
	},
	{
		.dev_name = "nuc970-kpi",
		.name = "kpi_4x8-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "kpi_8col",
		.data.mux.group = "kpi_3_grp",
	},
	{
		.dev_name = "nuc970-kpi",
		.name = "kpi_4x2-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "kpi_row",
		.data.mux.group = "kpi_4_grp",
	},
	{
		.dev_name = "nuc970-kpi",
		.name = "kpi_4x2-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "kpi_2col",
		.data.mux.group = "kpi_5_grp",
	},
	{
		.dev_name = "nuc970-kpi",
		.name = "kpi_4x4-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "kpi_row",
		.data.mux.group = "kpi_4_grp",
	},
	{
		.dev_name = "nuc970-kpi",
		.name = "kpi_4x4-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "kpi_2col",
		.data.mux.group = "kpi_5_grp",
	},
	{
		.dev_name = "nuc970-kpi",
		.name = "kpi_4x4-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "kpi_4col",
		.data.mux.group = "kpi_6_grp",
	},
	{
		.dev_name = "nuc970-kpi",
		.name = "kpi_4x8-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "kpi_row",
		.data.mux.group = "kpi_4_grp",
	},
	{
		.dev_name = "nuc970-kpi",
		.name = "kpi_4x8-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "kpi_2col",
		.data.mux.group = "kpi_5_grp",
	},
	{
		.dev_name = "nuc970-kpi",
		.name = "kpi_4x8-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "kpi_4col",
		.data.mux.group = "kpi_6_grp",
	},
	{
		.dev_name = "nuc970-kpi",
		.name = "kpi_4x8-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "kpi_8col",
		.data.mux.group = "kpi_7_grp",
	},
	{
		.dev_name = "nuc970-sdh",
		.name = "sd0", //PINCTRL_STATE_DEFAULT,
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "sd0",
		.data.mux.group = "sd0_grp",
	},
	{
		.dev_name = "nuc970-sdh",
		.name = "sd1-PI",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "sd1",
		.data.mux.group = "sd1_0_grp",
	},
	{
		.dev_name = "nuc970-sdh",
		.name = "sd1-PE",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "sd1",
		.data.mux.group = "sd1_1_grp",
	},
	{
		.dev_name = "nuc970-sdh",
		.name = "sd1-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "sd1",
		.data.mux.group = "sd1_2_grp",
	},
	{
		.dev_name = "nuc970-sdh",
		.name = "sd01-PI",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "sd01",
		.data.mux.group = "sd01_0_grp",
	},
	{
		.dev_name = "nuc970-sdh",
		.name = "sd01-PE",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "sd01",
		.data.mux.group = "sd01_1_grp",
	},
	{
		.dev_name = "nuc970-sdh",
		.name = "sd01-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "sd01",
		.data.mux.group = "sd01_2_grp",
	},
	{
		.dev_name = "nuc970-fmi",
		.name = "nand-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "nand",
		.data.mux.group = "nand_0_grp",
	},
	{
		.dev_name = "nuc970-fmi",
		.name = "nand-PI",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "nand",
		.data.mux.group = "nand_1_grp",
	},
	{
		.dev_name = "nuc970-fmi",
		.name = "emmc-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "emmc",
		.data.mux.group = "emmc_0_grp",
	},
	{
		.dev_name = "nuc970-fmi",
		.name = "emmc-PI",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "emmc",
		.data.mux.group = "emmc_1_grp",
	},
	{
		.dev_name = "nuc970-ehci",
		.name = "usbh-ppwr-pe",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "usbh_ppwr",
		.data.mux.group = "usbh_pe_grp",
	},
	{
		.dev_name = "nuc970-ehci",
		.name = "usbh-ppwr-pf",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "usbh_ppwr",
		.data.mux.group = "usbh_pf_grp",
	},
	{
		.dev_name = "nuc970-ehci",
		.name = "usbh-ppwr-oc",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "usbh_ppwr",
		.data.mux.group = "usbh_oc_grp",
	},
	{
		.dev_name = "nuc970-usbdev",
		.name = PINCTRL_STATE_DEFAULT,
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "usbd",
		.data.mux.group = "usbd_grp",
	},
	{
		.dev_name = "nuc970-i2c0",
		.name = PINCTRL_STATE_DEFAULT,
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "i2c0",
		.data.mux.group = "i2c0_grp",
	},
	{
		.dev_name = "nuc970-i2c1",
		.name = "i2c1-PB",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "i2c1",
		.data.mux.group = "i2c1_0_grp",
	},
	{
		.dev_name = "nuc970-i2c1",
		.name = "i2c1-PG",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "i2c1",
		.data.mux.group = "i2c1_1_grp",
	},
	{
		.dev_name = "nuc970-i2c1",
		.name = "i2c1-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "i2c1",
		.data.mux.group = "i2c1_2_grp",
	},
	{
		.dev_name = "nuc970-i2c1",
		.name = "i2c1-PI",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "i2c1",
		.data.mux.group = "i2c1_3_grp",
	},
	{
		.dev_name = "nuc970-audio-i2s",
		.name = PINCTRL_STATE_DEFAULT,
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "i2s",
		.data.mux.group = "i2s_grp",
	},
	// {
		// .dev_name = "nuc970-uart0",
		// .name = PINCTRL_STATE_DEFAULT,
		// .type = PIN_MAP_TYPE_MUX_GROUP,
		// .ctrl_dev_name = "pinctrl-nuc970",
		// .data.mux.function = "uart0",
		// .data.mux.group = "uart0_grp",
	// },
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("nuc970-uart.0", "uart0_grp", "uart0"),  // hog
	{
		.dev_name = "nuc970-uart.1",
		.name = "uart1-PE",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart1",
		.data.mux.group = "uart1_0_grp",
	},
	{
		.dev_name = "nuc970-uart.1",
		.name = "uart1-fc-PE",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart1",
		.data.mux.group = "uart1_0_grp",
	},
	{
		.dev_name = "nuc970-uart.1",
		.name = "uart1-fc-PE",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart1_fc",
		.data.mux.group = "uart1_1_grp",
	},
	{
		.dev_name = "nuc970-uart.1",
		.name = "uart1-ff-PE",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart1",
		.data.mux.group = "uart1_0_grp",
	},
	{
		.dev_name = "nuc970-uart.1",
		.name = "uart1-ff-PE",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart1_fc",
		.data.mux.group = "uart1_1_grp",
	},
	{
		.dev_name = "nuc970-uart.1",
		.name = "uart1-ff-PE",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart1_ff",
		.data.mux.group = "uart1_2_grp",
	},
	{
		.dev_name = "nuc970-uart.1",
		.name = "uart1-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart1",
		.data.mux.group = "uart1_3_grp",
	},
	{
		.dev_name = "nuc970-uart.1",
		.name = "uart1-fc-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart1",
		.data.mux.group = "uart1_3_grp",
	},
	{
		.dev_name = "nuc970-uart.1",
		.name = "uart1-fc-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart1_fc",
		.data.mux.group = "uart1_4_grp",
	},
	{
		.dev_name = "nuc970-uart.1",
		.name = "uart1-PI",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart1",
		.data.mux.group = "uart1_5_grp",
	},
	{
		.dev_name = "nuc970-uart.1",
		.name = "uart1-fc-PI",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart1",
		.data.mux.group = "uart1_5_grp",
	},
	{
		.dev_name = "nuc970-uart.1",
		.name = "uart1-fc-PI",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart1_fc",
		.data.mux.group = "uart1_6_grp",
	},
	{
		.dev_name = "nuc970-uart.2",
		.name = "uart2",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart2",
		.data.mux.group = "uart2_0_grp",
	},
	{
		.dev_name = "nuc970-uart.2",
		.name = "uart2_fc",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart2",
		.data.mux.group = "uart2_0_grp",
	},
	{
		.dev_name = "nuc970-uart.2",
		.name = "uart2_fc",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart2_fc",
		.data.mux.group = "uart2_1_grp",
	},
	{
		.dev_name = "nuc970-uart.3",
		.name = PINCTRL_STATE_DEFAULT,
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart3",
		.data.mux.group = "uart3_grp",
	},
	{
		.dev_name = "nuc970-uart.4",
		.name = "uart4-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart4",
		.data.mux.group = "uart4_0_grp",
	},
	{
		.dev_name = "nuc970-uart.4",
		.name = "uart4-fc-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart4",
		.data.mux.group = "uart4_0_grp",
	},
	{
		.dev_name = "nuc970-uart.4",
		.name = "uart4-fc-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart4_fc",
		.data.mux.group = "uart4_1_grp",
	},
	{
		.dev_name = "nuc970-uart.4",
		.name = "uart4-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart4",
		.data.mux.group = "uart4_2_grp",
	},
	{
		.dev_name = "nuc970-uart.4",
		.name = "uart4-fc-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart4",
		.data.mux.group = "uart4_2_grp",
	},
	{
		.dev_name = "nuc970-uart.4",
		.name = "uart4-fc-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart4_fc",
		.data.mux.group = "uart4_3_grp",
	},
	{
		.dev_name = "nuc970-uart.4",
		.name = "uart4-PI",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart4",
		.data.mux.group = "uart4_4_grp",
	},
	{
		.dev_name = "nuc970-uart.5",
		.name = PINCTRL_STATE_DEFAULT,
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart5",
		.data.mux.group = "uart5_grp",
	},
	{
		.dev_name = "nuc970-uart.6",
		.name = "uart6-PB",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart6",
		.data.mux.group = "uart6_0_grp",
	},
	{
		.dev_name = "nuc970-uart.6",
		.name = "uart6-fc-PB",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart6",
		.data.mux.group = "uart6_0_grp",
	},
	{
		.dev_name = "nuc970-uart.6",
		.name = "uart6-fc-PB",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart6_fc",
		.data.mux.group = "uart6_1_grp",
	},
	{
		.dev_name = "nuc970-uart.6",
		.name = "uart6-PG",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart6",
		.data.mux.group = "uart6_2_grp",
	},
	{
		.dev_name = "nuc970-uart.6",
		.name = "uart6-fc-PG",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart6",
		.data.mux.group = "uart6_2_grp",
	},
	{
		.dev_name = "nuc970-uart.6",
		.name = "uart6-fc-PG",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart6_fc",
		.data.mux.group = "uart6_3_grp",
	},
	{
		.dev_name = "nuc970-uart.7",
		.name = "uart7-PG",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart7",
		.data.mux.group = "uart7_0_grp",
	},
	{
		.dev_name = "nuc970-uart.7",
		.name = "uart7-PI",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart7",
		.data.mux.group = "uart7_1_grp",
	},
	{
		.dev_name = "nuc970-uart.8",
		.name = "uart8-PE",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart8",
		.data.mux.group = "uart8_0_grp",
	},
	{
		.dev_name = "nuc970-uart.8",
		.name = "uart8-fc-PE",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart8",
		.data.mux.group = "uart8_0_grp",
	},
	{
		.dev_name = "nuc970-uart.8",
		.name = "uart8-fc-PE",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart8_fc",
		.data.mux.group = "uart8_1_grp",
	},
	{
		.dev_name = "nuc970-uart.8",
		.name = "uart8-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart8",
		.data.mux.group = "uart8_2_grp",
	},
	{
		.dev_name = "nuc970-uart.8",
		.name = "uart8-fc-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart8",
		.data.mux.group = "uart8_2_grp",
	},
	{
		.dev_name = "nuc970-uart.8",
		.name = "uart8-fc-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart8_fc",
		.data.mux.group = "uart8_3_grp",
	},
	{
		.dev_name = "nuc970-uart.8",
		.name = "uart8-PI",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart8",
		.data.mux.group = "uart8_4_grp",
	},
	{
		.dev_name = "nuc970-uart.8",
		.name = "uart8-fc-PI",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart8",
		.data.mux.group = "uart8_4_grp",
	},
	{
		.dev_name = "nuc970-uart.8",
		.name = "uart8-fc-PI",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart8_fc",
		.data.mux.group = "uart8_5_grp",
	},
	{
		.dev_name = "nuc970-uart.9",
		.name = "uart9-PD0",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart9",
		.data.mux.group = "uart9_0_grp",
	},
	{
		.dev_name = "nuc970-uart.9",
		.name = "uart9-PD1",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart9",
		.data.mux.group = "uart9_1_grp",
	},
	{
		.dev_name = "nuc970-uart.9",
		.name = "uart9-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart9",
		.data.mux.group = "uart9_2_grp",
	},
	{
		.dev_name = "nuc970-uart.10",
		.name = "uart10-PB0",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart10",
		.data.mux.group = "uart10_0_grp",
	},
	{
		.dev_name = "nuc970-uart.10",
		.name = "uart10-PB1",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart10",
		.data.mux.group = "uart10_1_grp",
	},
	{
		.dev_name = "nuc970-uart.10",
		.name = "uart10-fc-PB1",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart10",
		.data.mux.group = "uart10_1_grp",
	},
	{
		.dev_name = "nuc970-uart.10",
		.name = "uart10-fc-PB1",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart10_fc",
		.data.mux.group = "uart10_2_grp",
	},
	{
		.dev_name = "nuc970-uart",
		.name = "uart10-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart10",
		.data.mux.group = "uart10_3_grp",
	},
	{
		.dev_name = "nuc970-uart.10",
		.name = "uart10-fc-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart10",
		.data.mux.group = "uart10_3_grp",
	},
	{
		.dev_name = "nuc970-uart.10",
		.name = "uart10-fc-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "uart10_fc",
		.data.mux.group = "uart10_4_grp",
	},
	{
		.dev_name = "nuc970-sc.0",
		.name = "sc0-PG",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "sc0",
		.data.mux.group = "sc0_0_grp",
	},
	{
		.dev_name = "nuc970-sc.0",
		.name = "sc0-PI",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "sc0",
		.data.mux.group = "sc0_1_grp",
	},
	{
		.dev_name = "nuc970-sc.0",
		.name = "scuart0-PG",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "scuart0",
		.data.mux.group = "sc0_2_grp",
	},
	{
		.dev_name = "nuc970-sc.0",
		.name = "scuart0-PI",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "scuart0",
		.data.mux.group = "sc0_3_grp",
	},
	{
		.dev_name = "nuc970-sc.1",
		.name = "sc1",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "sc1",
		.data.mux.group = "sc1_0_grp",
	},
	{
		.dev_name = "nuc970-sc.1",
		.name = "scuart1",//PINCTRL_STATE_DEFAULT,
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "scuart1",
		.data.mux.group = "sc1_1_grp",
	},
	{
		.dev_name = "nuc970-spi0",
		.name = "spi0",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "spi0",
		.data.mux.group = "spi0_0_grp",
	},
	{
		.dev_name = "nuc970-spi0",
		.name = "spi0-quad",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "spi0",
		.data.mux.group = "spi0_0_grp",
	},
	{
		.dev_name = "nuc970-spi0",
		.name = "spi0-quad",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "spi0_quad",
		.data.mux.group = "spi0_1_grp",
	},
	{
		.dev_name = "nuc970-spi1",
		.name = "spi1-PB",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "spi1",
		.data.mux.group = "spi1_0_grp",
	},
	{
		.dev_name = "nuc970-spi1",
		.name = "spi1-quad-PB",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "spi1",
		.data.mux.group = "spi1_0_grp",
	},
	{
		.dev_name = "nuc970-spi1",
		.name = "spi1-quad-PB",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "spi1_quad",
		.data.mux.group = "spi1_2_grp",
	},
	{
		.dev_name = "nuc970-spi1",
		.name = "spi1-PI",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "spi1",
		.data.mux.group = "spi1_4_grp",
	},
	{
		.dev_name = "nuc970-spi1",
		.name = "spi1-quad-PI",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "spi1",
		.data.mux.group = "spi1_4_grp",
	},
	{
		.dev_name = "nuc970-spi1",
		.name = "spi1-quad-PI",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "spi1_quad",
		.data.mux.group = "spi1_2_grp",
	},
	{
		.dev_name = "nuc970-can0",
		.name = "can0-PB",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "can0",
		.data.mux.group = "can0_0_grp",
	},
	{
		.dev_name = "nuc970-can0",
		.name = "can0-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "can0",
		.data.mux.group = "can0_1_grp",
	},
	{
		.dev_name = "nuc970-can0",
		.name = "can0-PI",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "can0",
		.data.mux.group = "can0_2_grp",
	},
	{
		.dev_name = "nuc970-can1",
		.name = PINCTRL_STATE_DEFAULT,
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "can1",
		.data.mux.group = "can1_grp",
	},
	{
		.dev_name = "nuc970-pwm.0",
		.name = "pwm0-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "pwm0",
		.data.mux.group = "pwm0_0_grp",
	},
	{
		.dev_name = "nuc970-pwm.0",
		.name = "pwm0-PB",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "pwm0",
		.data.mux.group = "pwm0_1_grp",
	},
	{
		.dev_name = "nuc970-pwm.0",
		.name = "pwm0-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "pwm0",
		.data.mux.group = "pwm0_2_grp",
	},
	{
		.dev_name = "nuc970-pwm.0",
		.name = "pwm0-PD",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "pwm0",
		.data.mux.group = "pwm0_3_grp",
	},
	{
		.dev_name = "nuc970-pwm.1",
		.name = "pwm1-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "pwm1",
		.data.mux.group = "pwm1_0_grp",
	},
	{
		.dev_name = "nuc970-pwm.1",
		.name = "pwm1-PB",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "pwm1",
		.data.mux.group = "pwm1_1_grp",
	},
	{
		.dev_name = "nuc970-pwm.1",
		.name = "pwm-PD",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "pwm1",
		.data.mux.group = "pwm1_2_grp",
	},
	{
		.dev_name = "nuc970-pwm.2",
		.name = "pwm2-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "pwm2",
		.data.mux.group = "pwm2_0_grp",
	},
	{
		.dev_name = "nuc970-pwm.2",
		.name = "pwm2-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "pwm2",
		.data.mux.group = "pwm2_1_grp",
	},
	{
		.dev_name = "nuc970-pwm.2",
		.name = "pwm2-PD",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "pwm2",
		.data.mux.group = "pwm2_2_grp",
	},
	{
		.dev_name = "nuc970-pwm.3",
		.name = "pwm3-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "pwm3",
		.data.mux.group = "pwm3_0_grp",
	},
	{
		.dev_name = "nuc970-pwm.3",
		.name = "pwm3-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "pwm3",
		.data.mux.group = "pwm3_1_grp",
	},
	{
		.dev_name = "nuc970-pwm.3",
		.name = "pwm3-PD",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "pwm3",
		.data.mux.group = "pwm3_2_grp",
	},
	{
		.dev_name = "nuc970-wdt",
		.name = PINCTRL_STATE_DEFAULT,
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "wdt",
		.data.mux.group = "wdt_grp",
	},
	{
		.dev_name = "nuc970-rtc",
		.name = "rtc-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "rtc",
		.data.mux.group = "rtc_0_grp",
	},
	{
		.dev_name = "nuc970-rtc",
		.name = "rtc-PI",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "rtc",
		.data.mux.group = "rtc_1_grp",
	},
	{
		.dev_name = "nuc970-etimer.0",
		.name = "etimer0-tgl-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "etimer0_tgl",
		.data.mux.group = "etimer0_0_grp",
	},
	{
		.dev_name = "nuc970-etimer.0",
		.name = "etimer0-tgl-PB",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "etimer0_tgl",
		.data.mux.group = "etimer0_2_grp",
	},
	{
		.dev_name = "nuc970-etimer.0",
		.name = "etimer0-cap-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "etimer0_cap",
		.data.mux.group = "etimer0_1_grp",
	},
	{
		.dev_name = "nuc970-etimer.0",
		.name = "etimer0-cap-PB",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "etimer0_cap",
		.data.mux.group = "etimer0_3_grp",
	},
	{
		.dev_name = "nuc970-etimer.1",
		.name = "etimer1-tgl-PB",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "etimer1_tgl",
		.data.mux.group = "etimer1_0_grp",
	},
	{
		.dev_name = "nuc970-etimer.1",
		.name = "etimer1-tgl-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "etimer1_tgl",
		.data.mux.group = "etimer1_2_grp",
	},
	{
		.dev_name = "nuc970-etimer.1",
		.name = "etimer1-cap-PB",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "etimer1_cap",
		.data.mux.group = "etimer1_1_grp",
	},
	{
		.dev_name = "nuc970-etimer.1",
		.name = "etimer1-cap-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "etimer1_cap",
		.data.mux.group = "etimer1_3_grp",
	},
	{
		.dev_name = "nuc970-etimer.2",
		.name = "etimer2-tgl-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "etimer2_tgl",
		.data.mux.group = "etimer2_0_grp",
	},
	{
		.dev_name = "nuc970-etimer.2",
		.name = "etimer2-tgl-PF",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "etimer2_tgl",
		.data.mux.group = "etimer2_2_grp",
	},
	{
		.dev_name = "nuc970-etimer.2",
		.name = "etimer2-cap-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "etimer2_cap",
		.data.mux.group = "etimer2_1_grp",
	},
	{
		.dev_name = "nuc970-etimer.2",
		.name = "etimer2-cap-PF",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "etimer2_cap",
		.data.mux.group = "etimer2_3_grp",
	},
	{
		.dev_name = "nuc970-etimer.3",
		.name = "etimer3-tgl-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "etimer3_tgl",
		.data.mux.group = "etimer3_0_grp",
	},
	{
		.dev_name = "nuc970-etimer.3",
		.name = "etimer3-tgl-PF",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "etimer3_tgl",
		.data.mux.group = "etimer3_2_grp",
	},
	{
		.dev_name = "nuc970-etimer.3",
		.name = "etimer3-cap-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "etimer3_cap",
		.data.mux.group = "etimer3_1_grp",
	},
	{
		.dev_name = "nuc970-etimer.3",
		.name = "etimer3-cap-PF",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "etimer3_cap",
		.data.mux.group = "etimer3_3_grp",
	},
	{
		.dev_name = "nuc970-jtag",
		.name = PINCTRL_STATE_DEFAULT,
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "jtag",
		.data.mux.group = "jtag_grp",
	},
	{
		.dev_name = "nuc970-gpio",
		.name = "eint0-PF",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "eint0",
		.data.mux.group = "eint0_0_grp",
	},
	{
		.dev_name = "nuc970-gpio",
		.name = "eint0-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "eint0",
		.data.mux.group = "eint0_1_grp",
	},
	{
		.dev_name = "nuc970-gpio",
		.name = "eint1-PF",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "eint1",
		.data.mux.group = "eint1_0_grp",
	},
	{
		.dev_name = "nuc970-gpio",
		.name = "eint1-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "eint1",
		.data.mux.group = "eint1_1_grp",
	},
	{
		.dev_name = "nuc970-gpio",
		.name = "eint2-PF",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "eint2",
		.data.mux.group = "eint2_0_grp",
	},
	{
		.dev_name = "nuc970-gpio",
		.name = "eint2-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "eint2",
		.data.mux.group = "eint2_1_grp",
	},
	{
		.dev_name = "nuc970-gpio",
		.name = "eint3-PF",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "eint3",
		.data.mux.group = "eint3_0_grp",
	},
	{
		.dev_name = "nuc970-gpio",
		.name = "eint3-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "eint3",
		.data.mux.group = "eint3_1_grp",
	},
	{
		.dev_name = "nuc970-gpio",
		.name = "eint4-PF",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "eint4",
		.data.mux.group = "eint4_0_grp",
	},
	{
		.dev_name = "nuc970-gpio",
		.name = "eint4-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "eint4",
		.data.mux.group = "eint4_1_grp",
	},
	{
		.dev_name = "nuc970-gpio",
		.name = "eint5-PF",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "eint5",
		.data.mux.group = "eint5_0_grp",
	},
	{
		.dev_name = "nuc970-gpio",
		.name = "eint5-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "eint5",
		.data.mux.group = "eint5_1_grp",
	},
	{
		.dev_name = "nuc970-gpio",
		.name = "eint6-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "eint6",
		.data.mux.group = "eint6_0_grp",
	},
	{
		.dev_name = "nuc970-gpio",
		.name = "eint6-PI",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "eint6",
		.data.mux.group = "eint6_1_grp",
	},
	{
		.dev_name = "nuc970-gpio",
		.name = "eint7-PH",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "eint7",
		.data.mux.group = "eint7_0_grp",
	},
	{
		.dev_name = "nuc970-gpio",
		.name = "eint7-PI",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "eint7",
		.data.mux.group = "eint7_1_grp",
	},
	{
		.dev_name = "nuc970-ebi",
		.name = "ebi0",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "ebi0",
		.data.mux.group = "ebi0_grp",
	},
	{
		.dev_name = "nuc970-ebi",
		.name = "ebi1",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "ebi1",
		.data.mux.group = "ebi1_grp",
	},
	{
		.dev_name = "nuc970-ebi",
		.name = "ebi2",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "ebi2",
		.data.mux.group = "ebi2_grp",
	},
	{
		.dev_name = "nuc970-ebi",
		.name = "ebi3",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "ebi3",
		.data.mux.group = "ebi3_grp",
	},
	{
		.dev_name = "nuc970-ebi",
		.name = "ebi4",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "ebi4",
		.data.mux.group = "ebi4_grp",
	},
	{
		.dev_name = "nuc970-ebi",
		.name = "ebi-8bit",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "ebi_8",
		.data.mux.group = "ebi_0_grp",
	},
	{
		.dev_name = "nuc970-ebi",
		.name = "ebi-16bit",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "ebi_8",
		.data.mux.group = "ebi_0_grp",
	},
	{
		.dev_name = "nuc970-ebi",
		.name = "ebi-16bit",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc970",
		.data.mux.function = "ebi_16",
		.data.mux.group = "ebi_1_grp",
	},
};


static int nuc970_pinctrl_probe(struct platform_device *pdev)
{
	struct pinctrl_dev *pctl;

	pctl = pinctrl_register(&nuc970_pinctrl_desc, &pdev->dev, NULL);
	if (IS_ERR(pctl))
		pr_err("could not register NUC970 pin driver\n");

	platform_set_drvdata(pdev, pctl);

	return pinctrl_register_mappings(nuc970_pinmap, ARRAY_SIZE(nuc970_pinmap));

}

static int nuc970_pinctrl_remove(struct platform_device *pdev)
{
	struct pinctrl_dev *pctl = platform_get_drvdata(pdev);

	pinctrl_unregister(pctl);

	return 0;
}


static struct platform_driver nuc970_pinctrl_driver = {
	.driver = {
		.name = "pinctrl-nuc970",
		.owner = THIS_MODULE,
	},
	.probe = nuc970_pinctrl_probe,
	.remove = nuc970_pinctrl_remove,
};


static int __init nuc970_pinctrl_init(void)
{
	return platform_driver_register(&nuc970_pinctrl_driver);
}
arch_initcall(nuc970_pinctrl_init);

static void __exit nuc970_pinctrl_exit(void)
{
	platform_driver_unregister(&nuc970_pinctrl_driver);
}

module_exit(nuc970_pinctrl_exit);

MODULE_AUTHOR("Nuvoton Technology Corp.");
MODULE_LICENSE("GPL");
