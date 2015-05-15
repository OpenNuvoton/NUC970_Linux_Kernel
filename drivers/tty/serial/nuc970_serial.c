/*
 *  linux/drivers/serial/nuc970_serial.c
 *
 *  NUC970 serial driver
 *
 *
 *  Copyright (C) 2014 Nuvoton Technology Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#if defined(CONFIG_SERIAL_NUC970_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/clk.h>
#include <linux/serial_reg.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/nmi.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/serial.h>

#include <mach/map.h>
#include <mach/regs-serial.h>
#include <mach/regs-gcr.h>
#include <mach/mfp.h>


#include "nuc970_serial.h"

#define UART_NR 11
static struct uart_driver nuc970serial_reg;

struct clk		*clk;

struct uart_nuc970_port {
	struct uart_port	port;

	unsigned short		capabilities;	/* port capabilities */
	unsigned char		ier;
	unsigned char		lcr;
	unsigned char		mcr;
	unsigned char		mcr_mask;	/* mask of user bits */
	unsigned char		mcr_force;	/* mask of forced bits */

	struct serial_rs485     rs485;          /* rs485 settings */
	/*
	 * We provide a per-port pm hook.
	 */
	void			(*pm)(struct uart_port *port,
				      unsigned int state, unsigned int old);
};

static struct uart_nuc970_port nuc970serial_ports[UART_NR];

static inline struct uart_nuc970_port *
to_nuc970_uart_port(struct uart_port *uart)
{
	return container_of(uart, struct uart_nuc970_port, port);
}

static inline unsigned int serial_in(struct uart_nuc970_port *p, int offset)
{
	return(__raw_readl(p->port.membase + offset));
}

static inline void serial_out(struct uart_nuc970_port *p, int offset, int value)
{
	__raw_writel(value, p->port.membase + offset);
}

static void rs485_start_rx(struct uart_nuc970_port *port)
{
	#if 0
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)port;

	if(port->rs485.flags & SER_RS485_RTS_AFTER_SEND)
	{
		// Set logical level for RTS pin equal to high 
		serial_out(port, UART_REG_MCR, (serial_in(port, UART_REG_MCR) & ~0x200) );
}
	else
{
		// Set logical level for RTS pin equal to low 
		serial_out(port, UART_REG_MCR, (serial_in(port, UART_REG_MCR) | 0x200) );
	}
	#endif
}

static void rs485_stop_rx(struct uart_nuc970_port *port)
{
	#if 0
	if(port->rs485.flags & SER_RS485_RTS_ON_SEND)
	{
		// Set logical level for RTS pin equal to high 
		serial_out(port, UART_REG_MCR, (serial_in(port, UART_REG_MCR) & ~0x200) );
	}
	else
	{
		// Set logical level for RTS pin equal to low 
		serial_out(port, UART_REG_MCR, (serial_in(port, UART_REG_MCR) | 0x200) );
	}
	#endif

}

static inline void __stop_tx(struct uart_nuc970_port *p)
{
	unsigned int ier;
	struct tty_struct *tty = p->port.state->port.tty;

	if ((ier = serial_in(p, UART_REG_IER)) & THRE_IEN) {
		serial_out(p, UART_REG_IER, ier & ~THRE_IEN);
	}
	if (p->rs485.flags & SER_RS485_ENABLED)
		rs485_start_rx(p);

	if (tty->termios.c_line == N_IRDA)
	{
		while(!(serial_in(p, UART_REG_FSR) & TX_EMPTY));
		while(!(serial_in(p, UART_REG_FSR) & TE_FLAG));
	
		serial_out(p, UART_REG_IRCR, (serial_in(p, UART_REG_IRCR) & ~0x2) ); // Tx disable (select Rx)
	}
}

static void nuc970serial_stop_tx(struct uart_port *port)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)port;

	__stop_tx(up);

}

static void transmit_chars(struct uart_nuc970_port *up);

static void nuc970serial_start_tx(struct uart_port *port)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)port;
	unsigned int ier;	
	struct tty_struct *tty = up->port.state->port.tty;

	if (tty->termios.c_line == N_IRDA)
	{
		serial_out(up, UART_REG_IRCR, (serial_in(up, UART_REG_IRCR) | 0x2) ); // Tx enable
	}

	if (up->rs485.flags & SER_RS485_ENABLED)
		rs485_stop_rx(up);

	if (!((ier = serial_in(up, UART_REG_IER)) & THRE_IEN)) {
		ier |= THRE_IEN;
		serial_out(up, UART_REG_IER, ier);
	}

}

static void nuc970serial_stop_rx(struct uart_port *port)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)port;

	serial_out(up, UART_REG_IER, serial_in(up, UART_REG_IER) & ~RDA_IEN);
}

static void nuc970serial_enable_ms(struct uart_port *port)
{

}

static void
receive_chars(struct uart_nuc970_port *up)
{
	unsigned char ch;
	unsigned int fsr;
	int max_count = 256;
	char flag;

	do {
		ch = (unsigned char)serial_in(up, UART_REG_RBR);
		fsr = serial_in(up, UART_REG_FSR);
		flag = TTY_NORMAL;
		up->port.icount.rx++;

		if (unlikely(fsr & (BIF | FEF | PEF | RX_OVER_IF))) {
			if (fsr & BIF) {
				serial_out(up, UART_REG_FSR, BIF);
				up->port.icount.brk++;
				if (uart_handle_break(&up->port))
					continue;
			}

			if (fsr & FEF) {
				serial_out(up, UART_REG_FSR, FEF);
				up->port.icount.parity++;
			}

			if (fsr & PEF) {
				serial_out(up, UART_REG_FSR, PEF);
				up->port.icount.frame++;
			}

			if (fsr & RX_OVER_IF) {
				serial_out(up, UART_REG_FSR, RX_OVER_IF);
				up->port.icount.overrun++;
			}
			// FIXME: check port->read_status_mask to determin report flags
			if (fsr & BIF)
				flag = TTY_BREAK;
			if (fsr & PEF)
				flag = TTY_PARITY;
			if (fsr & FEF)
				flag = TTY_FRAME;
		}
		
		if (uart_handle_sysrq_char(&up->port, ch))
			continue;

		uart_insert_char(&up->port, fsr, RX_OVER_IF, ch, flag);

	} while (!(fsr & RX_EMPTY) && (max_count-- > 0));

	spin_lock(&up->port.lock);
	tty_flip_buffer_push(&up->port.state->port);	
	spin_unlock(&up->port.lock);
}

static void transmit_chars(struct uart_nuc970_port *up)
{
	struct circ_buf *xmit = &up->port.state->xmit;
	int count = 12;

	if (up->port.x_char) {
		while(serial_in(up, UART_REG_FSR) & TX_FULL);
		serial_out(up, UART_REG_THR, up->port.x_char);
		up->port.icount.tx++;
		up->port.x_char = 0;
		return;
	}
	if (uart_tx_stopped(&up->port)) {
		nuc970serial_stop_tx(&up->port);
		return;
	}

	if (uart_circ_empty(xmit)) {
		__stop_tx(up);
		return;
	}

	do {
		//while(serial_in(up, UART_REG_FSR) & TX_FULL);
		serial_out(up, UART_REG_THR, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		up->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);

	if (uart_circ_empty(xmit))
		__stop_tx(up);
}

static unsigned int check_modem_status(struct uart_nuc970_port *up)
{
	unsigned int status = 0;

	if (0) {

		wake_up_interruptible(&up->port.state->port.delta_msr_wait);
	}

	return status;
}

static irqreturn_t nuc970serial_interrupt(int irq, void *dev_id)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)dev_id;
	unsigned int isr;

	isr = serial_in(up, UART_REG_ISR);

	if (isr & (RDA_IF | TOUT_IF))
		receive_chars(up);
	
	check_modem_status(up);
	
	if (isr & THRE_IF)
		transmit_chars(up);

	return IRQ_HANDLED;
}

static unsigned int nuc970serial_tx_empty(struct uart_port *port)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)port;
	//unsigned long flags;
	unsigned int fsr;

	//spin_lock_irqsave(&up->port.lock, flags);
	fsr = serial_in(up, UART_REG_FSR);
	//spin_unlock_irqrestore(&up->port.lock, flags);

	return (fsr & (TE_FLAG | TX_EMPTY)) == (TE_FLAG | TX_EMPTY) ? TIOCSER_TEMT : 0;
}

static unsigned int nuc970serial_get_mctrl(struct uart_port *port)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)port;
	unsigned int status;
	unsigned int ret = 0;

	//status = check_modem_status(up);

	status = serial_in(up, UART_REG_MSR);;
	
	if(status & 0x10)
		ret |= TIOCM_CTS;

	return ret;
}

static void nuc970serial_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)port;
	unsigned char mcr = 0;

	if (mctrl & TIOCM_RTS)
	{
		// set RTS high level trigger
		mcr = serial_in(up, UART_REG_MCR);
		mcr |= 0x200;
		mcr &= ~(0x2);
	}

	if (up->mcr & UART_MCR_AFE)
	{
		// set RTS high level trigger
		mcr = serial_in(up, UART_REG_MCR);
		mcr |= 0x200;
		mcr &= ~(0x2);

		// enable CTS/RTS auto-flow control
		serial_out(up, UART_REG_IER, (serial_in(up, UART_REG_IER) | (0x3000)));
	}

	// set CTS high level trigger
	serial_out(up, UART_REG_MSR, (serial_in(up, UART_REG_MSR) | (0x100)));
	serial_out(up, UART_REG_MCR, mcr);
}

static void nuc970serial_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)port;
	unsigned long flags;
	unsigned int lcr;

	spin_lock_irqsave(&up->port.lock, flags);
	lcr = serial_in(up, UART_REG_LCR);
	if (break_state != 0)
		lcr |= BCB;	// set break
	else
		lcr &= ~BCB;	// clr break
	serial_out(up, UART_REG_LCR, lcr);
	spin_unlock_irqrestore(&up->port.lock, flags);
}

static int nuc970serial_startup(struct uart_port *port)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)port;
	struct tty_struct *tty = port->state->port.tty;
	int retval;
	
	//  TODO: configure pin function and enable engine clock
	//nuc970serial_pinctrl();

	/* Reset FIFO */
	serial_out(up, UART_REG_FCR, TFR | RFR /* | RX_DIS */);

	/* Clear pending interrupts (not every bit are write 1 clear though...) */
	serial_out(up, UART_REG_ISR, 0xFFFFFFFF);

	retval = request_irq(port->irq, nuc970serial_interrupt, 0,
			tty ? tty->name : "nuc970_serial", port);

	if (retval) {
		printk("request irq failed...\n");
		return retval;
	}

	/*
	 * Now, initialize the UART
	 */
	serial_out(up, UART_REG_FCR, serial_in(up, UART_REG_FCR) | 0x10);	// Trigger level 4 byte
	serial_out(up, UART_REG_LCR, 0x7);						// 8 bit
	serial_out(up, UART_REG_TOR, 0x40);
	serial_out(up, UART_REG_IER, RTO_IEN | RDA_IEN | TIME_OUT_EN);
	
	/* 12MHz reference clock input, 115200 */
	serial_out(up, UART_REG_BAUD, 0x30000066);
	
	return 0;
}

static void nuc970serial_shutdown(struct uart_port *port)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)port;
	//unsigned long flags;
	free_irq(port->irq, port);

	/*
	 * Disable interrupts from this port
	 */
	serial_out(up, UART_REG_IER, 0);
	serial_out(up, UART_REG_FCR, TFR | RFR /* | RX_DIS */);

}

static unsigned int nuc970serial_get_divisor(struct uart_port *port, unsigned int baud)
{
	unsigned int quot;

	quot = (port->uartclk / baud) - 2;

	return quot;
}

static void
nuc970serial_set_termios(struct uart_port *port, struct ktermios *termios,
		       struct ktermios *old)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)port;
	unsigned int lcr = 0;
	unsigned long flags;
	unsigned int baud, quot;

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		lcr = 0;
		break;
	case CS6:
		lcr |= 1;
		break;
	case CS7:
		lcr |= 2;
		break;
	default:
	case CS8:
		lcr |= 3;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		lcr |= NSB;
	if (termios->c_cflag & PARENB)
		lcr |= PBE;
	if (!(termios->c_cflag & PARODD))
		lcr |= EPE;
	if (termios->c_cflag & CMSPAR)
		lcr |= SPE;

	baud = uart_get_baud_rate(port, termios, old,
				  port->uartclk / 16 / 0xffff,
				  port->uartclk / 16);

	quot = nuc970serial_get_divisor(port, baud);

	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&up->port.lock, flags);

	up->port.read_status_mask = RX_OVER_IF /*| UART_LSR_THRE | UART_LSR_DR*/;
	if (termios->c_iflag & INPCK)
		up->port.read_status_mask |= FEF | PEF;
	if (termios->c_iflag & (BRKINT | PARMRK))
		up->port.read_status_mask |= BIF;

	/*
	 * Characteres to ignore
	 */
	up->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		up->port.ignore_status_mask |= FEF | PEF;
	if (termios->c_iflag & IGNBRK) {
		up->port.ignore_status_mask |= BIF;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			up->port.ignore_status_mask |= RX_OVER_IF;
	}

	if (termios->c_cflag & CRTSCTS)
		up->mcr |= UART_MCR_AFE;
	else
		up->mcr &= ~UART_MCR_AFE;

	nuc970serial_set_mctrl(&up->port, up->port.mctrl);

	serial_out(up, UART_REG_BAUD, quot | 0x30000000);
	serial_out(up, UART_REG_LCR, lcr);		

	spin_unlock_irqrestore(&up->port.lock, flags);

}

static void
nuc970serial_set_ldisc(struct uart_port *port, int ld)
{
	struct uart_nuc970_port *uart = (struct uart_nuc970_port *)port;
	unsigned int baud;

	switch (ld) {
	case N_IRDA:

		baud = serial_in(uart, UART_REG_BAUD);
		baud = baud & (0x0000ffff);
		baud = baud + 2;
		baud = baud / 16;
		baud = baud - 2;

		serial_out(uart, UART_REG_BAUD, baud);
		serial_out(uart, UART_REG_IRCR, (serial_in(uart, UART_REG_IRCR) & ~0x40) );  // Rx inverse
			
		serial_out(uart, UART_FUN_SEL, (serial_in(uart, UART_FUN_SEL) & ~FUN_SEL_Msk) );
		serial_out(uart, UART_FUN_SEL, (serial_in(uart, UART_FUN_SEL) | FUN_SEL_IrDA) );

		break;
	default:
		serial_out(uart, UART_FUN_SEL, (serial_in(uart, UART_FUN_SEL) & ~FUN_SEL_Msk) );
	}

}

static void
nuc970serial_pm(struct uart_port *port, unsigned int state,
	      unsigned int oldstate)
{
	struct uart_nuc970_port *p = (struct uart_nuc970_port *)port;


	if (p->pm)
		p->pm(port, state, oldstate);
}

static void nuc970serial_release_port(struct uart_port *port)
{
	struct platform_device *pdev = to_platform_device(port->dev);
	int size = pdev->resource[0].end - pdev->resource[0].start + 1;

	release_mem_region(port->mapbase, size);

	iounmap(port->membase);
	port->membase = NULL;


}

static int nuc970serial_request_port(struct uart_port *port)
{
	return 0;
}

static void nuc970serial_config_port(struct uart_port *port, int flags)
{
	int ret;

	/*
	 * Find the region that we can probe for.  This in turn
	 * tells us whether we can probe for the type of port.
	 */
	ret = nuc970serial_request_port(port);
	if (ret < 0)
		return;
	port->type = PORT_NUC970;


}

static int
nuc970serial_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_NUC970)
		return -EINVAL;
	return 0;
}

static const char *
nuc970serial_type(struct uart_port *port)
{

	return (port->type == PORT_NUC970) ? "NUC970" : NULL;
}

/* Enable or disable the rs485 support */
void nuc970serial_config_rs485(struct uart_port *port, struct serial_rs485 *rs485conf)
{
	struct uart_nuc970_port *p = to_nuc970_uart_port(port);

	spin_lock(&port->lock);

	p->rs485 = *rs485conf;

	if (p->rs485.delay_rts_before_send >= 1000)
		p->rs485.delay_rts_before_send = 1000;

	serial_out(p, UART_FUN_SEL, (serial_in(p, UART_FUN_SEL) & ~FUN_SEL_Msk) );
		
	if(rs485conf->flags & SER_RS485_ENABLED)
	{
		serial_out(p, UART_FUN_SEL, (serial_in(p, UART_FUN_SEL) | FUN_SEL_RS485) );
	
		//rs485_start_rx(p);	// stay in Rx mode 	

		if(rs485conf->flags & SER_RS485_RTS_ON_SEND)
		{
			serial_out(p, UART_REG_MCR, (serial_in(p, UART_REG_MCR) & ~0x200) );
		}
		else
		{
			serial_out(p, UART_REG_MCR, (serial_in(p, UART_REG_MCR) | 0x200) );
		}
		
		// set auto direction mode
		serial_out(p,UART_REG_ALT_CSR,(serial_in(p, UART_REG_ALT_CSR) | (1 << 10)) );
	}

	spin_unlock(&port->lock);
}

static int
nuc970serial_rs485_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg)
{
	struct serial_rs485 rs485conf;

	switch (cmd) {
	case TIOCSRS485:
		if (copy_from_user(&rs485conf, (struct serial_rs485 *) arg,
					sizeof(rs485conf)))
			return -EFAULT;

		nuc970serial_config_rs485(port, &rs485conf);
		break;

	case TIOCGRS485:
		if (copy_to_user((struct serial_rs485 *) arg,
					&(to_nuc970_uart_port(port)->rs485),
					sizeof(rs485conf)))
			return -EFAULT;
		break;

	default:
		return -ENOIOCTLCMD;
	}
	return 0;
}

static struct uart_ops nuc970serial_ops = {
	.tx_empty	= nuc970serial_tx_empty,
	.set_mctrl	= nuc970serial_set_mctrl,
	.get_mctrl	= nuc970serial_get_mctrl,
	.stop_tx	= nuc970serial_stop_tx,
	.start_tx	= nuc970serial_start_tx,
	.stop_rx	= nuc970serial_stop_rx,
	.enable_ms	= nuc970serial_enable_ms,
	.break_ctl	= nuc970serial_break_ctl,
	.startup	= nuc970serial_startup,
	.shutdown	= nuc970serial_shutdown,
	.set_termios	= nuc970serial_set_termios,
	.set_ldisc	= nuc970serial_set_ldisc,
	.pm		= nuc970serial_pm,
	.type		= nuc970serial_type,
	.release_port	= nuc970serial_release_port,
	.request_port	= nuc970serial_request_port,
	.config_port	= nuc970serial_config_port,
	.verify_port	= nuc970serial_verify_port,
	.ioctl		= nuc970serial_rs485_ioctl,

};

static void __init nuc970serial_init_ports(void)
{
	static int first = 1;
	int i;
	
	// enable clock
	clk = clk_get(NULL, "uart0");	
	clk_prepare(clk);
	clk_enable(clk);

	
	if (!first)
		return;
	first = 0;

	for (i = 0; i < UART_NR; i++) {
		struct uart_nuc970_port *up = &nuc970serial_ports[i];

		up->port.line = i;
		spin_lock_init(&up->port.lock);

		up->port.ops = &nuc970serial_ops;
		up->port.iobase = (long)(NUC970_VA_UART + (i*0x100));
		up->port.membase = NUC970_VA_UART + (i*0x100);
		up->port.uartclk = 12000000;
	
	}
}

#ifdef CONFIG_SERIAL_NUC970_CONSOLE
static void nuc970serial_console_putchar(struct uart_port *port, int ch)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)port;

	while(!(serial_in(up, UART_REG_FSR) & TX_EMPTY));
	serial_out(up, UART_REG_THR, ch);
}

/*
 *	Print a string to the serial port trying not to disturb
 *	any possible real use of the port...
 *
 *	The console_lock must be held when we get here.
 */
static void
nuc970serial_console_write(struct console *co, const char *s, unsigned int count)
{
	struct uart_nuc970_port *up = &nuc970serial_ports[co->index];
	unsigned long flags;
	unsigned int ier;

	local_irq_save(flags);

	/*
	 *	First save the IER then disable the interrupts
	 */
	ier = serial_in(up, UART_REG_IER);
	serial_out(up, UART_REG_IER, 0);

	uart_console_write(&up->port, s, count, nuc970serial_console_putchar);

	/*
	 *	Finally, wait for transmitter to become empty
	 *	and restore the IER
	 */
	while(!(serial_in(up, UART_REG_FSR) & TX_EMPTY));
	serial_out(up, UART_REG_IER, ier);


	local_irq_restore(flags);
}

static int __init nuc970serial_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (co->index >= UART_NR)
		co->index = 0;
	port = &nuc970serial_ports[co->index].port;

	if (!port->iobase && !port->membase)
		return -ENODEV;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(port, co, baud, parity, bits, flow);
}


static struct console nuc970serial_console = {
	.name		= "ttyS",
	.write		= nuc970serial_console_write,
	.device		= uart_console_device,
	.setup		= nuc970serial_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &nuc970serial_reg,
};

static int __init nuc970serial_console_init(void)
{
	nuc970serial_init_ports();		
	register_console(&nuc970serial_console);
	
	return 0;
}
console_initcall(nuc970serial_console_init);

#define NUC970SERIAL_CONSOLE	&nuc970serial_console
#else
#define NUC970SERIAL_CONSOLE	NULL
#endif

static struct uart_driver nuc970serial_reg = {
	.owner			= THIS_MODULE,
	.driver_name		= "serial",
	.dev_name		= "ttyS",
	.major			= TTY_MAJOR,
	.minor			= 64,
	.cons			= NUC970SERIAL_CONSOLE,
	.nr				= UART_NR,
};


/**
 *
 *	Suspend one serial port.
 */
void nuc970serial_suspend_port(int line)
{
	uart_suspend_port(&nuc970serial_reg, &nuc970serial_ports[line].port);
}

/**
 *
 *	Resume one serial port.
 */
void nuc970serial_resume_port(int line)
{
	struct uart_nuc970_port *up = &nuc970serial_ports[line];

	uart_resume_port(&nuc970serial_reg, &up->port);
}

static int nuc970serial_pinctrl(struct platform_device *pdev)
{
	struct pinctrl *p = NULL;
	int retval = 0;  

if(pdev->id == 1)
{
#if defined (CONFIG_NUC970_UART1_PE)
    p = devm_pinctrl_get_select(&pdev->dev, "uart1-PE");
#elif defined (CONFIG_NUC970_UART1_FC_PE)
    p = devm_pinctrl_get_select(&pdev->dev, "uart1-fc-PE");
#elif defined (CONFIG_NUC970_UART1_FF_PE)
	p = devm_pinctrl_get_select(&pdev->dev, "uart1-ff-PE");
#elif defined (CONFIG_NUC970_UART1_PH)
    p = devm_pinctrl_get_select(&pdev->dev, "uart1-PH");
#elif defined (CONFIG_NUC970_UART1_FC_PH)
    p = devm_pinctrl_get_select(&pdev->dev, "uart1-fc-PH");
#elif defined (CONFIG_NUC970_UART1_PI)
    p = devm_pinctrl_get_select(&pdev->dev, "uart1-PI");
#elif defined (CONFIG_NUC970_UART1_FC_PI)
    p = devm_pinctrl_get_select(&pdev->dev, "uart1-fc-PI");
#endif

	if (IS_ERR(p))
	{
		dev_err(&pdev->dev, "unable to reserve pin\n");
		retval = PTR_ERR(p);
	}
}
else if(pdev->id == 2)
{
#if defined (CONFIG_NUC970_UART2)
	p = devm_pinctrl_get_select(&pdev->dev, "uart2");
#elif defined (CONFIG_NUC970_UART2_FC)
	p = devm_pinctrl_get_select(&pdev->dev, "uart2_fc");
#endif

	if (IS_ERR(p))
	{
		dev_err(&pdev->dev, "unable to reserve pin\n");
		retval = PTR_ERR(p);
	}
}
else if(pdev->id == 4)
{
#if defined (CONFIG_NUC970_UART4_PC)
	p = devm_pinctrl_get_select(&pdev->dev, "uart4-PC");
#elif defined (CONFIG_NUC970_UART4_FC_PC)
	p = devm_pinctrl_get_select(&pdev->dev, "uart4-fc-PC");
#elif defined (CONFIG_NUC970_UART4_PH)
	p = devm_pinctrl_get_select(&pdev->dev, "uart4-PH");
#elif defined (CONFIG_NUC970_UART4_FC_PH)
	p = devm_pinctrl_get_select(&pdev->dev, "uart4-fc-PH");
#elif defined (CONFIG_NUC970_UART4_PI)
	p = devm_pinctrl_get_select(&pdev->dev, "uart4-PI");
#endif

	if (IS_ERR(p))
	{
		dev_err(&pdev->dev, "unable to reserve pin\n");
		retval = PTR_ERR(p);
	}
}
else if(pdev->id == 6)
{
#if defined (CONFIG_NUC970_UART6_PB)
	p = devm_pinctrl_get_select(&pdev->dev, "uart6-PB");
#elif defined (CONFIG_NUC970_UART6_FC_PB)
	p = devm_pinctrl_get_select(&pdev->dev, "uart6-fc-PB");
#elif defined (CONFIG_NUC970_UART6_PG)
	p = devm_pinctrl_get_select(&pdev->dev, "uart6-PG");
#elif defined (CONFIG_NUC970_UART6_FC_PG)
	p = devm_pinctrl_get_select(&pdev->dev, "uart6-fc-PG");
#elif defined (CONFIG_NUC970_UART6_FC_PB)
	p = devm_pinctrl_get_select(&pdev->dev, "uart6-fc-PB");
#endif

	if (IS_ERR(p))
	{
		dev_err(&pdev->dev, "unable to reserve pin\n");
		retval = PTR_ERR(p);
	}
}
else if(pdev->id == 7)
{
#if defined (CONFIG_NUC970_UART7_PG)
	p = devm_pinctrl_get_select(&pdev->dev, "uart7-PG");
#elif defined (CONFIG_NUC970_UART7_PI)
	p = devm_pinctrl_get_select(&pdev->dev, "uart7-PI");
#endif

	if (IS_ERR(p))
	{
		dev_err(&pdev->dev, "unable to reserve pin\n");
		retval = PTR_ERR(p);
	}
}
else if(pdev->id == 8)
{
#if defined (CONFIG_NUC970_UART8_PE)
	p = devm_pinctrl_get_select(&pdev->dev, "uart8-PE");
#elif defined (CONFIG_NUC970_UART8_FC_PE)
	p = devm_pinctrl_get_select(&pdev->dev, "uart8-fc-PE");
#elif defined (CONFIG_NUC970_UART8_PH)
	p = devm_pinctrl_get_select(&pdev->dev, "uart8-PH");
#elif defined (CONFIG_NUC970_UART8_FC_PH)
	p = devm_pinctrl_get_select(&pdev->dev, "uart8-fc-PH");
#elif defined (CONFIG_NUC970_UART8_PI)
	p = devm_pinctrl_get_select(&pdev->dev, "uart8-PI");
#elif defined (CONFIG_NUC970_UART8_FC_PI)
	p = devm_pinctrl_get_select(&pdev->dev, "uart8-fc-PI");
#endif

	if (IS_ERR(p))
	{
		dev_err(&pdev->dev, "unable to reserve pin\n");
		retval = PTR_ERR(p);
	}
}
else if(pdev->id == 9)
{
#if defined (CONFIG_NUC970_UART9_PD0)
	p = devm_pinctrl_get_select(&pdev->dev, "uart9-PD0");
#elif defined (CONFIG_NUC970_UART9_PD1)
	p = devm_pinctrl_get_select(&pdev->dev, "uart9-PD1");
#elif defined (CONFIG_NUC970_UART9_PH)
	p = devm_pinctrl_get_select(&pdev->dev, "uart9-PH");
#endif

	if (IS_ERR(p))
	{
		dev_err(&pdev->dev, "unable to reserve pin\n");
		retval = PTR_ERR(p);
	}
}
else if(pdev->id == 10)
{
#if defined (CONFIG_NUC970_UART10_PB0)
	p = devm_pinctrl_get_select(&pdev->dev, "uart10-PB0");
#elif defined (CONFIG_NUC970_UART10_PB1)
	p = devm_pinctrl_get_select(&pdev->dev, "uart10-PB1");
#elif defined (CONFIG_NUC970_UART10_FC_PB1)
	p = devm_pinctrl_get_select(&pdev->dev, "uart10-fc-PB1");
#elif defined (CONFIG_NUC970_UART10_PC)
	p = devm_pinctrl_get_select(&pdev->dev, "uart10-PC");
#elif defined (CONFIG_NUC970_UART10_FC_PC)
	p = devm_pinctrl_get_select(&pdev->dev, "uart10-fc-PC");
#endif

    if (IS_ERR(p))
    {
        dev_err(&pdev->dev, "unable to reserve pin\n");
        retval = PTR_ERR(p);
    }
}
	return retval;
}

void nuc970serial_set_clock(void)
{
	clk = clk_get(NULL, "uart0");	
	clk_prepare(clk);
	clk_enable(clk);

	clk = clk_get(NULL, "uart0_eclk");	
	clk_prepare(clk);
	clk_enable(clk);


	#ifdef CONFIG_NUC970_UART1
	clk = clk_get(NULL, "uart1");	
	clk_prepare(clk);
	clk_enable(clk);

	clk = clk_get(NULL, "uart1_eclk");	
	clk_prepare(clk);
	clk_enable(clk);
	#endif

	#ifdef CONFIG_NUC970_UART2
	clk = clk_get(NULL, "uart2");	
	clk_prepare(clk);
	clk_enable(clk);

	clk = clk_get(NULL, "uart2_eclk");	
	clk_prepare(clk);
	clk_enable(clk);
	#endif

	#ifdef CONFIG_NUC970_UART3
	clk = clk_get(NULL, "uart3");	
	clk_prepare(clk);
	clk_enable(clk);

	clk = clk_get(NULL, "uart3_eclk");	
	clk_prepare(clk);
	clk_enable(clk);
	#endif

	#ifdef CONFIG_NUC970_UART4
	clk = clk_get(NULL, "uart4");	
	clk_prepare(clk);
	clk_enable(clk);

	clk = clk_get(NULL, "uart4_eclk");	
	clk_prepare(clk);
	clk_enable(clk);
	#endif

	#ifdef CONFIG_NUC970_UART5
	clk = clk_get(NULL, "uart5");	
	clk_prepare(clk);
	clk_enable(clk);

	clk = clk_get(NULL, "uart5_eclk");	
	clk_prepare(clk);
	clk_enable(clk);
	#endif

	#ifdef CONFIG_NUC970_UART6
	clk = clk_get(NULL, "uart6");	
	clk_prepare(clk);
	clk_enable(clk);

	clk = clk_get(NULL, "uart6_eclk");	
	clk_prepare(clk);
	clk_enable(clk);
	#endif

	#ifdef CONFIG_NUC970_UART7
	clk = clk_get(NULL, "uart7");	
	clk_prepare(clk);
	clk_enable(clk);

	clk = clk_get(NULL, "uart7_eclk");	
	clk_prepare(clk);
	clk_enable(clk);
	#endif

	#ifdef CONFIG_NUC970_UART8
	clk = clk_get(NULL, "uart8");	
	clk_prepare(clk);
	clk_enable(clk);

	clk = clk_get(NULL, "uart8_eclk");	
	clk_prepare(clk);
	clk_enable(clk);
	#endif

	#ifdef CONFIG_NUC970_UART9
	clk = clk_get(NULL, "uart9");	
	clk_prepare(clk);
	clk_enable(clk);

	clk = clk_get(NULL, "uart9_eclk");	
	clk_prepare(clk);
	clk_enable(clk);
	#endif

	#ifdef CONFIG_NUC970_UART10
	clk = clk_get(NULL, "uart10");	
	clk_prepare(clk);
	clk_enable(clk);

	clk = clk_get(NULL, "uart10_eclk");	
	clk_prepare(clk);
	clk_enable(clk);
	#endif
	
}

/*
 * Register a set of serial devices attached to a platform device.  The
 * list is terminated with a zero flags entry, which means we expect
 * all entries to have at least UPF_BOOT_AUTOCONF set.
 */
static int nuc970serial_probe(struct platform_device *pdev)
{
	struct plat_nuc970serial_port *p = pdev->dev.platform_data;
	struct uart_nuc970_port *up;

	int ret, i, retval;

//	memset(&port, 0, sizeof(struct uart_port));

	retval = nuc970serial_pinctrl(pdev);
	if(retval != 0)
		return retval;

	nuc970serial_set_clock();

	i = pdev->id;

	//for (i = 0; i < UART_NR && p; p++, i++) {
		up = &nuc970serial_ports[i];
		
		up->port.line 			= i;
		up->port.iobase       	= p->iobase;
		up->port.membase      	= p->membase;
		up->port.irq          	= p->irq;
		up->port.uartclk      	= p->uartclk;
		up->port.mapbase     	= p->mapbase;
		up->port.private_data 	= p->private_data;
		up->port.dev 			= &pdev->dev;
		up->port.flags 			= ASYNC_BOOT_AUTOCONF;
		
		/* Possibly override default I/O functions.  */
		if (p->serial_in)
			up->port.serial_in = p->serial_in;
		if (p->serial_out)
			up->port.serial_out = p->serial_out;

		ret = uart_add_one_port(&nuc970serial_reg, &up->port);


	//}



	
	return 0;
}

/*
 * Remove serial ports registered against a platform device.
 */
static int nuc970serial_remove(struct platform_device *dev)
{
	int i;
	struct uart_port *port = platform_get_drvdata(dev);

	free_irq(port->irq, port);

	for (i = 0; i < UART_NR; i++) {
		struct uart_nuc970_port *up = &nuc970serial_ports[i];

		if (up->port.dev == &dev->dev)
			uart_remove_one_port(&nuc970serial_reg, &up->port);
	}
	return 0;
}

static int nuc970serial_suspend(struct platform_device *dev, pm_message_t state)
{
	int i;

	for (i = 0; i < UART_NR; i++) {
		struct uart_nuc970_port *up = &nuc970serial_ports[i];

		if (up->port.type != PORT_UNKNOWN && up->port.dev == &dev->dev)
			uart_suspend_port(&nuc970serial_reg, &up->port);
	}

	return 0;
}

static int nuc970serial_resume(struct platform_device *dev)
{
	int i;

	for (i = 0; i < UART_NR; i++) {
		struct uart_nuc970_port *up = &nuc970serial_ports[i];

		if (up->port.type != PORT_UNKNOWN && up->port.dev == &dev->dev)
			nuc970serial_resume_port(i);
	}

	return 0;
}

static struct platform_driver nuc970serial_driver = {
	.probe		= nuc970serial_probe,
	.remove		= nuc970serial_remove,
	.suspend	= nuc970serial_suspend,
	.resume		= nuc970serial_resume,
	.driver		= {
		.name	= "nuc970-uart",
		.owner	= THIS_MODULE,
	},
};

static int __init nuc970serial_init(void)
{
	int ret;

	ret = uart_register_driver(&nuc970serial_reg);
	if (ret)
		return ret;

	ret = platform_driver_register(&nuc970serial_driver);
	if (ret)
		uart_unregister_driver(&nuc970serial_reg);

	return ret;
}

static void __exit nuc970serial_exit(void)
{
	platform_driver_unregister(&nuc970serial_driver);
	uart_unregister_driver(&nuc970serial_reg);
}

module_init(nuc970serial_init);
module_exit(nuc970serial_exit);

EXPORT_SYMBOL(nuc970serial_suspend_port);
EXPORT_SYMBOL(nuc970serial_resume_port);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NUC970 serial driver");

MODULE_ALIAS_CHARDEV_MAJOR(TTY_MAJOR);
