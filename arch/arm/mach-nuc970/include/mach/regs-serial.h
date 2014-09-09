/*
 * arch/arm/mach-nuc970/include/mach/regs-serial.h
 *
 * Copyright (c) 2012 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __ASM_ARM_REGS_SERIAL_H
#define __ASM_ARM_REGS_SERIAL_H

#define UART0_BA	NUC970_VA_UART
#define UART1_BA	(NUC970_VA_UART+0x100)
#define UART2_BA	(NUC970_VA_UART+0x200)
#define UART3_BA	(NUC970_VA_UART+0x300)
#define UART4_BA	(NUC970_VA_UART+0x400)
#define UART5_BA	(NUC970_VA_UART+0x500)
#define UART6_BA	(NUC970_VA_UART+0x600)
#define UART7_BA	(NUC970_VA_UART+0x700)
#define UART8_BA	(NUC970_VA_UART+0x800)
#define UART9_BA	(NUC970_VA_UART+0x900)
#define UART10_BA	(NUC970_VA_UART+0xA00)

#define UART0_PA	NUC970_PA_UART
#define UART1_PA	(NUC970_PA_UART+0x100)
#define UART2_PA	(NUC970_PA_UART+0x200)
#define UART3_PA	(NUC970_PA_UART+0x300)
#define UART4_PA	(NUC970_PA_UART+0x400)
#define UART5_PA	(NUC970_PA_UART+0x500)
#define UART6_PA	(NUC970_PA_UART+0x600)
#define UART7_PA	(NUC970_PA_UART+0x700)
#define UART8_PA	(NUC970_PA_UART+0x800)
#define UART9_PA	(NUC970_PA_UART+0x900)
#define UART10_PA	(NUC970_PA_UART+0xA00)

struct uart_port;
struct plat_nuc970serial_port {
	unsigned long	iobase;		/* io base address */
	void __iomem	*membase;	/* ioremap cookie or NULL */
	resource_size_t	mapbase;	/* resource base */
	unsigned int	irq;		/* interrupt number */
	unsigned int	uartclk;	/* UART clock rate */
	void            *private_data;
	unsigned int	(*serial_in)(struct uart_port *, int);
	void		(*serial_out)(struct uart_port *, int, int);
};

#ifndef __ASSEMBLY__

struct nuc970_uart_clksrc {
	const char	*name;
	unsigned int	divisor;
	unsigned int	min_baud;
	unsigned int	max_baud;
};

struct nuc970_uartcfg {
	unsigned char	hwport;
	unsigned char	unused;
	unsigned short	flags;
	unsigned long	uart_flags;

	unsigned long	ucon;
	unsigned long	ulcon;
	unsigned long	ufcon;

	struct nuc970_uart_clksrc *clocks;
	unsigned int	clocks_size;
};

#endif /* __ASSEMBLY__ */

#endif /* __ASM_ARM_REGS_SERIAL_H */

