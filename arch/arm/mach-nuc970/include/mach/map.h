/*
 * arch/arm/mach-nuc970/include/mach/map.h
 *
 * Copyright (c) 2014 Nuvoton technology corporation.
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

#ifndef __ASM_ARCH_MAP_H
#define __ASM_ARCH_MAP_H

#ifndef __ASSEMBLY__
#define NUC970_ADDR(x)		((void __iomem *)(0xF0000000 + (x)))
#else
#define NUC970_ADDR(x)		(0xF0000000 + (x))
#endif

#define AHB_IO_BASE		0xB0000000
#define APB_IO_BASE		0xB8000000
#define CLOCKPW_BASE		(APB_IO_BASE+0x200)
#define AIC_IO_BASE		(APB_IO_BASE+0x2000)
#define TIMER_IO_BASE		(APB_IO_BASE+0x1000)

/*
 * interrupt controller is the first thing we put in, to make
 * the assembly code for the irq detection easier
 */
#define NUC970_VA_IRQ		NUC970_ADDR(0x00000000)
#define NUC970_PA_IRQ		(0xB8002000)
#define NUC970_SZ_IRQ		SZ_4K

/* GCR, Clock management */
#define NUC970_VA_GCR_CLK	NUC970_ADDR(0x08002000)		//cc
#define NUC970_PA_GCR_CLK	(0xB0000000)
#define NUC970_SZ_GCR_CLK	SZ_4K

#define NUC970_VA_GCR		NUC970_ADDR(0x08002000)
#define NUC970_PA_GCR		(0xB0000000)
#define NUC970_SZ_GCR		SZ_512

/* Clock management */
#define NUC970_VA_CLK		(NUC970_VA_GCR+0x200)
#define NUC970_PA_CLK		(0xB0000200)
#define NUC970_SZ_CLK		SZ_512

/* EBI, SDIC management */
#define NUC970_VA_EBI_SDIC	NUC970_ADDR(0x00001000)		//cc
#define NUC970_PA_EBI_SDIC	(0xB0001000)
#define NUC970_SZ_EBI_SDIC	SZ_4K

/* EBI management */
#define NUC970_VA_EBI		NUC970_ADDR(0x00001000)
#define NUC970_PA_EBI		(0xB0001000)
#define NUC970_SZ_EBI		SZ_2K

/* SDIC management */
#define NUC970_VA_SDIC		NUC970_ADDR(0x00001800)
#define NUC970_PA_SDIC		(0xB0001800)
#define NUC970_SZ_SDIC		SZ_2K

/* External MAC0 control*/
#define NUC970_VA_EMAC0		NUC970_ADDR(0x00002000)
#define NUC970_PA_EMAC0		(0xB0002000)
#define NUC970_SZ_EMAC0		SZ_4K

/* External MAC1 control*/
#define NUC970_VA_EMAC1		NUC970_ADDR(0x00003000)
#define NUC970_PA_EMAC1		(0xB0003000)
#define NUC970_SZ_EMAC1		SZ_4K

/* GDMA control */
#define NUC970_VA_GDMA		NUC970_ADDR(0x00004000)
#define NUC970_PA_GDMA		(0xB0004000)
#define NUC970_SZ_GDMA		SZ_4K

/* USB host controller*/
#define NUC970_VA_EHCI		NUC970_ADDR(0x00005000)
#define NUC970_PA_EHCI		(0xB0005000)
#define NUC970_SZ_EHCI		SZ_4K

#define NUC970_VA_OHCI		NUC970_ADDR(0x00007000)
#define NUC970_PA_OHCI		(0xB0007000)
#define NUC970_SZ_OHCI		SZ_4K

/* USB Device port */
#define NUC970_VA_USBDEV	NUC970_ADDR(0x00006000)
#define NUC970_PA_USBDEV	(0xB0006000)
#define NUC970_SZ_USBDEV	SZ_4K


/* LCD controller*/
#define NUC970_VA_LCD		NUC970_ADDR(0x00008000)
#define NUC970_PA_LCD		(0xB0008000)
#define NUC970_SZ_LCD		SZ_4K

/* Audio Controller controller */
#define NUC970_VA_ACTL		NUC970_ADDR(0x00009000)
#define NUC970_PA_ACTL		(0xB0009000)
#define NUC970_SZ_ACTL		SZ_4K

/* JPEG codec */
#define NUC970_VA_JPEG 	   	NUC970_ADDR(0x0000A000)
#define NUC970_PA_JPEG 	   	(0xB000A000)
#define NUC970_SZ_JPEG 	   	SZ_4K

/* 2D controller*/
#define NUC970_VA_GE		NUC970_ADDR(0x0000B000)
#define NUC970_PA_GE		(0xB000B000)
#define NUC970_SZ_GE		SZ_4K

/* SDIO Controller */
#define NUC970_VA_SDH	 	NUC970_ADDR(0x0000C000)
#define NUC970_PA_SDH 		(0xB000C000)
#define NUC970_SZ_SDH 		SZ_4K

/* FMI Controller */
#define NUC970_VA_FMI 	   	NUC970_ADDR(0x0000D000)
#define NUC970_PA_FMI 	   	(0xB000D000)
#define NUC970_SZ_FMI 	   	SZ_4K

/* VCAP Interface */
#define NUC970_VA_CAP   	NUC970_ADDR(0x0000E000)
#define NUC970_PA_CAP   	(0xB000E000)
#define NUC970_SZ_CAP   	SZ_4K

/* Crypto Engine */
#define NUC970_VA_CRYPTO  	NUC970_ADDR(0x0000F000)
#define NUC970_PA_CRYPTO   	(0xB000F000)
#define NUC970_SZ_CRYPTO   	SZ_4K

/* UARTs */
#define NUC970_VA_UART		NUC970_ADDR(0x08000000)
#define NUC970_PA_UART		(0xB8000000)
#define NUC970_SZ_UART		SZ_4K

/* Timers, ETimers, Watchdog Timer (WDT), Window Watchdog Timer (WWDT) management */
#define NUC970_VA_TIMER_ETIMER_WDT_WWDT		NUC970_ADDR(0x08001000)		//cc
#define NUC970_PA_TIMER_ETIMER_WDT_WWDT		(0xB8001000)
#define NUC970_SZ_TIMER_ETIMER_WDT_WWDT		SZ_4K

/* Timers */
#define NUC970_VA_TIMER		NUC970_ADDR(0x08001000)
#define NUC970_PA_TIMER		(0xB8001000)
#define NUC970_SZ_TIMER		SZ_1K

/* ETimers */
#define NUC970_VA_ETIMER	NUC970_ADDR(0x08001400)
#define NUC970_PA_ETIMER	(0xB8001400)
#define NUC970_SZ_ETIMER	SZ_1K

/* Watchdog Timer (WDT) */
#define NUC970_VA_WDT		NUC970_ADDR(0x08001800)
#define NUC970_PA_WDT		(0xB8001800)
#define NUC970_SZ_WDT		SZ_256

/* Window Watchdog Timer (WWDT) */
#define NUC970_VA_WWDT		NUC970_ADDR(0x08001900)
#define NUC970_PA_WWDT		(0xB8001900)
#define NUC970_SZ_WWDT		SZ_256

/* GPIO ports */
#define NUC970_VA_GPIO		NUC970_ADDR(0x08003000)
#define NUC970_PA_GPIO		(0xB8003000)
#define NUC970_SZ_GPIO		SZ_4K

/* RTC */
#define NUC970_VA_RTC		NUC970_ADDR(0x08004000)
#define NUC970_PA_RTC		(0xB8004000)
#define NUC970_SZ_RTC		SZ_4K

/* Smart card host*/
#define NUC970_VA_SC		NUC970_ADDR(0x08005000)
#define NUC970_PA_SC		(0xB8005000)
#define NUC970_SZ_SC		SZ_4K

/* I2C, Smart card host*/
#define NUC970_VA_I2C_SPI	NUC970_ADDR(0x08006000)		//cc
#define NUC970_PA_I2C_SPI	(0xB8006000)
#define NUC970_SZ_I2C_SPI	SZ_1K

/* I2C hardware controller */
#define NUC970_VA_I2C0		NUC970_ADDR(0x08006000)
#define NUC970_PA_I2C0		(0xB8006000)
#define NUC970_SZ_I2C0		SZ_256

#define NUC970_VA_I2C1		NUC970_ADDR(0x08006100)
#define NUC970_PA_I2C1		(0xB8006100)
#define NUC970_SZ_I2C1		SZ_256

/* SPI0 Controller */
#define NUC970_VA_SPI0 		NUC970_ADDR(0x08006200)
#define NUC970_PA_SPI0 		(0xB8006200)
#define NUC970_SZ_SPI0		SZ_256

/* SPI1 Controller */
#define NUC970_VA_SPI1 		NUC970_ADDR(0x08006300)
#define NUC970_PA_SPI1 		(0xB8006300)
#define NUC970_SZ_SPI1		SZ_256

/* Pulse Width Modulation(PWM) Registers */
#define NUC970_VA_PWM		NUC970_ADDR(0x08007000)
#define NUC970_PA_PWM		(0xB8007000)
#define NUC970_SZ_PWM		SZ_4K

/* Keypad Interface*/
#define NUC970_VA_KPI		NUC970_ADDR(0x08008000)
#define NUC970_PA_KPI		(0xB8008000)
#define NUC970_SZ_KPI		SZ_4K

/* ADC */
#define NUC970_VA_ADC		NUC970_ADDR(0x0800A000)
#define NUC970_PA_ADC		(0xB800A000)
#define NUC970_SZ_ADC		SZ_4K

/* CAN Controller */
#define NUC970_VA_CAN		NUC970_ADDR(0x0800B000)
#define NUC970_PA_CAN		(0xB800B000)
#define NUC970_SZ_CAN		SZ_1K

#define NUC970_VA_CAN1		NUC970_ADDR(0x0800B400)
#define NUC970_PA_CAN1		(0xB800B400)
#define NUC970_SZ_CAN1		SZ_1K


/* MTP */
#define NUC970_VA_MTP		NUC970_ADDR(0x0800C000)
#define NUC970_PA_MTP		(0xB800C000)
#define NUC970_SZ_MTP		SZ_4K



#endif /* __ASM_ARCH_MAP_H */
