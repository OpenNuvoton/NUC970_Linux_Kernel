/*
 *  linux/drivers/serial/nuc970_scuart.c
 *
 *  NUC970 Smartcard UART mode driver
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


#include <linux/export.h>
#include <linux/kernel.h>
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
#include <mach/regs-sc.h>
#include <mach/regs-gcr.h>
#include <mach/mfp.h>


#define SCUART_NR 2
static struct uart_driver nuc970serial_reg;
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


struct uart_nuc970_port {
	struct uart_port	port;
	/*
	 * We provide a per-port pm hook.
	 */
	void			(*pm)(struct uart_port *port,
				      unsigned int state, unsigned int old);
};

static struct uart_nuc970_port nuc970serial_ports[SCUART_NR];

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


static inline void __stop_tx(struct uart_nuc970_port *p)
{
	unsigned int ier;

	if ((ier = serial_in(p, REG_SC_INTEN)) & SC_INTEN_TBEIEN) {
		serial_out(p, REG_SC_INTEN, ier & ~SC_INTEN_TBEIEN);
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


	if (!((ier = serial_in(up, REG_SC_INTEN)) & SC_INTEN_TBEIEN)) {
		ier |= SC_INTEN_TBEIEN;
		serial_out(up, REG_SC_INTEN, ier);
	}

}

static void nuc970serial_stop_rx(struct uart_port *port)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)port;

	serial_out(up, REG_SC_INTEN, serial_in(up, REG_SC_INTEN) & ~SC_INTEN_RDAIEN);
}

static void nuc970serial_enable_ms(struct uart_port *port)
{

}

static void receive_chars(struct uart_nuc970_port *up)
{
	unsigned char ch;
	unsigned int status;
	int max_count = 256;
	char flag;

	do {
		ch = (unsigned char)serial_in(up, REG_SC_DAT);
		status = serial_in(up, REG_SC_STATUS);
		flag = TTY_NORMAL;
		up->port.icount.rx++;

		if (unlikely(status & (SC_STATUS_BEF | SC_STATUS_FEF | SC_STATUS_PEF | SC_STATUS_RXOV))) {
			if (status & SC_STATUS_BEF) {
				serial_out(up, REG_SC_STATUS, SC_STATUS_BEF);
				up->port.icount.brk++;
				if (uart_handle_break(&up->port))
					continue;
			}

			if (status & SC_STATUS_FEF) {
				serial_out(up, REG_SC_STATUS, SC_STATUS_FEF);
				up->port.icount.parity++;
			}

			if (status & SC_STATUS_PEF) {
				serial_out(up, REG_SC_STATUS, SC_STATUS_PEF);
				up->port.icount.frame++;
			}

			if (status & SC_STATUS_RXOV) {
				serial_out(up, REG_SC_STATUS, SC_STATUS_RXOV);
				up->port.icount.overrun++;
			}
			// FIXME: check port->read_status_mask to determin report flags
			if (status & SC_STATUS_BEF)
				flag = TTY_BREAK;
			if (status & SC_STATUS_PEF)
				flag = TTY_PARITY;
			if (status & SC_STATUS_FEF)
				flag = TTY_FRAME;
		}

		if (uart_handle_sysrq_char(&up->port, ch))
			continue;

		uart_insert_char(&up->port, status, SC_STATUS_RXOV, ch, flag);

	} while (!(status & SC_STATUS_RXEMPTY) && (max_count-- > 0));

	tty_flip_buffer_push(&up->port.state->port);
}

static void transmit_chars(struct uart_nuc970_port *up)
{
	struct circ_buf *xmit = &up->port.state->xmit;
	int count = 4;	// SCUART FIFO depath is 4 bytes

	if (up->port.x_char) {
		while(serial_in(up, REG_SC_STATUS) & SC_STATUS_TXFULL);
		serial_out(up, REG_SC_DAT, up->port.x_char);
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
		serial_out(up, REG_SC_DAT, xmit->buf[xmit->tail]);
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


static irqreturn_t nuc970serial_interrupt(int irq, void *dev_id)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)dev_id;
	unsigned int isr, ier;

	isr = serial_in(up, REG_SC_INTSTS);
	ier = serial_in(up, REG_SC_INTEN);

	if (isr & (SC_INTSTS_RXTOIF | SC_INTSTS_RDAIF))
		receive_chars(up);

	if ((isr & SC_INTSTS_TBEIF) && (ier & SC_INTEN_TBEIEN))
		transmit_chars(up);

	return IRQ_HANDLED;
}

static unsigned int nuc970serial_tx_empty(struct uart_port *port)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)port;
	unsigned long flags;
	unsigned int status;

	spin_lock_irqsave(&up->port.lock, flags);
	status = serial_in(up, REG_SC_STATUS);
	spin_unlock_irqrestore(&up->port.lock, flags);

	return (status & SC_STATUS_TXEMPTY) ? TIOCSER_TEMT : 0;
}

static unsigned int nuc970serial_get_mctrl(struct uart_port *port)
{

	return 0;
}

static void nuc970serial_set_mctrl(struct uart_port *port, unsigned int mctrl)
{

}

static void nuc970serial_break_ctl(struct uart_port *port, int break_state)
{

}

static int nuc970serial_startup(struct uart_port *port)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)port;
	struct tty_struct *tty = port->state->port.tty;
	int retval;

	/* Reset FIFO */
	serial_out(up, REG_SC_ALTCTL, SC_ALTCTL_RXRST| SC_ALTCTL_TXRST);

	/* Clear pending interrupts (not every bit are write 1 clear though...) */
	serial_out(up, REG_SC_INTSTS, 0xFFFFFFFF);

	retval = request_irq(port->irq, nuc970serial_interrupt, 0,
			tty ? tty->name : "nuc970_scuart", port);

	if (retval) {
		printk("request irq failed...\n");
		return retval;
	}

	/*
	 * Now, initialize the UART
	 */
	serial_out(up, REG_SC_CTL, 0x1041);	// 1 stop bit, trigger level 2 bytes
	serial_out(up, REG_SC_UARTCTL, 1);	// 8 bit, uart mode enable
	serial_out(up, REG_SC_RXTOUT, 0x40);
	serial_out(up, REG_SC_INTEN, SC_INTEN_RXTOIEN | SC_INTEN_RDAIEN);

	/* 12MHz reference clock input, 115200bps */
	serial_out(up, REG_SC_ETUCTL, 0x67);

	return 0;
}

static void nuc970serial_shutdown(struct uart_port *port)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)port;
	//unsigned long flags;

	/*
	 * Disable interrupts from this port
	 */
	serial_out(up, REG_SC_INTEN, 0);
	serial_out(up, REG_SC_ALTCTL, SC_ALTCTL_RXRST| SC_ALTCTL_TXRST);
	free_irq(port->irq, port);

}

static unsigned int nuc970serial_get_divisor(struct uart_port *port, unsigned int baud)
{
	unsigned int quot;

	quot = (port->uartclk / baud) - 1;

	return quot;
}

static void nuc970serial_set_termios(struct uart_port *port, struct ktermios *termios,
		       struct ktermios *old)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)port;
	unsigned int uartctl = 1, ctl;	// 1 is for enable UART mode
	unsigned long flags;
	unsigned int baud, quot;


	switch (termios->c_cflag & CSIZE) {
	case CS5:
		uartctl |= 0x30;
		break;
	case CS6:
		uartctl |= 0x20;
		break;
	case CS7:
		uartctl |= 0x10;
		break;
	default:
	case CS8:
		uartctl |= 0;
		break;
	}

	if (termios->c_cflag & PARENB)
		uartctl &= ~0x40;
	else
		uartctl |= 0x40;
	if (termios->c_cflag & PARODD)
		uartctl |= 0x80;


	baud = uart_get_baud_rate(port, termios, old,
				  port->uartclk / 0xFFF,
				  port->uartclk / 5);	// 4 < bauddate divider <= 0xFFF

	quot = nuc970serial_get_divisor(port, baud);

	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&up->port.lock, flags);

	up->port.read_status_mask = SC_STATUS_RXOV;
	if (termios->c_iflag & INPCK)
		up->port.read_status_mask |= SC_STATUS_FEF | SC_STATUS_PEF;
	if (termios->c_iflag & (BRKINT | PARMRK))
		up->port.read_status_mask |= SC_STATUS_BEF;

	/*
	 * Characteres to ignore
	 */
	up->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		up->port.ignore_status_mask |= SC_STATUS_FEF | SC_STATUS_PEF;
	if (termios->c_iflag & IGNBRK) {
		up->port.ignore_status_mask |= SC_STATUS_BEF;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			up->port.ignore_status_mask |= SC_STATUS_RXOV;
	}

	serial_out(up, REG_SC_ETUCTL, quot);
	serial_out(up, REG_SC_UARTCTL, uartctl);

	ctl = serial_in(up, REG_SC_CTL);

	if (termios->c_cflag & CSTOPB)
		ctl &= ~SC_CTL_NSB;
	else
		ctl |= SC_CTL_NSB;
	serial_out(up, REG_SC_CTL, ctl);


	spin_unlock_irqrestore(&up->port.lock, flags);

}

static void nuc970serial_set_ldisc(struct uart_port *port, int ld)
{
	return;

}

static void nuc970serial_pm(struct uart_port *port, unsigned int state,
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

};

static void __init nuc970serial_init_ports(void)
{
	int i;

	for (i = 0; i < SCUART_NR; i++) {
		struct uart_nuc970_port *up = &nuc970serial_ports[i];
		up->port.line = i;
		spin_lock_init(&up->port.lock);

		up->port.ops = &nuc970serial_ops;
		up->port.iobase = (long)(NUC970_VA_SC + (i * 0x400));
		up->port.membase = NUC970_VA_SC + (i * 0x400);
		up->port.uartclk = 12000000;

	}
}

static struct uart_driver nuc970serial_reg = {
	.owner			= THIS_MODULE,
	.driver_name		= "sc_serial",
	.dev_name		= "ttySCU",
	.major			= TTY_MAJOR,
	.minor			= 80,//64,  reerve at least 11 for real UART
	.nr			= SCUART_NR,
};


/**
 *
 *	Suspend one serial port.
 */
static void nuc970scuart_suspend_port(int line)
{
	uart_suspend_port(&nuc970serial_reg, &nuc970serial_ports[line].port);
}

/**
 *
 *	Resume one serial port.
 */
static void nuc970scuart_resume_port(int line)
{
	struct uart_nuc970_port *up = &nuc970serial_ports[line];

	uart_resume_port(&nuc970serial_reg, &up->port);
}

static int nuc970serial_pinctrl(struct platform_device *pdev)
{
	struct pinctrl *p = NULL;
	int retval = 0;

	if(pdev->id == 0) {
#if defined (CONFIG_NUC970_SCUART0_PG)
		p = devm_pinctrl_get_select(&pdev->dev, "scuart0-PG");
#elif defined (CONFIG_NUC970_SCUART0_PI)
		p = devm_pinctrl_get_select(&pdev->dev, "scuart0-PI");
#endif

		if (IS_ERR(p)) {
			dev_err(&pdev->dev, "Unable to reserve pin\n");
			retval = PTR_ERR(p);
		}

	} else { // if(pdev->id == 1)
		p = devm_pinctrl_get_select(&pdev->dev, "scuart1");
		if (IS_ERR(p)) {
			dev_err(&pdev->dev, "Unable to reserve pin\n");
			retval = PTR_ERR(p);
		}
	}
	return retval;
}

static void nuc970serial_set_clock(void)
{
	struct clk *clk;
#ifdef CONFIG_NUC970_SCUART0
	clk = clk_get(NULL, "smc0");
	clk_prepare(clk);
	clk_enable(clk);

	clk = clk_get(NULL, "smc0_eclk");
	clk_prepare(clk);
	clk_enable(clk);
#endif

#ifdef CONFIG_NUC970_SCUART1
	clk = clk_get(NULL, "smc1");
	clk_prepare(clk);
	clk_enable(clk);

	clk = clk_get(NULL, "smc1_eclk");
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

	retval = nuc970serial_pinctrl(pdev);
	if(retval != 0)
		return retval;

	nuc970serial_set_clock();

	i = pdev->id;

	up = &nuc970serial_ports[i];
	up->port.line 		= i;
	up->port.iobase       	= (long)p->membase;
	up->port.membase      	= p->membase;
	up->port.irq          	= p->irq;
	up->port.uartclk      	= p->uartclk;
	up->port.mapbase     	= p->mapbase;
	up->port.private_data 	= p->private_data;
	up->port.dev 		= &pdev->dev;
	up->port.flags 		= ASYNC_BOOT_AUTOCONF;
	up->port.ops = &nuc970serial_ops;

	spin_lock_init(&up->port.lock);


	ret = uart_add_one_port(&nuc970serial_reg, &up->port);


	return 0;
}

/*
 * Remove serial ports registered against a platform device.
 */
static int nuc970serial_remove(struct platform_device *dev)
{
	int i;

	// FIXME: remove all or one-by-one???
	for (i = 0; i < SCUART_NR; i++) {
		struct uart_nuc970_port *up = &nuc970serial_ports[i];

		if (up->port.dev == &dev->dev)
			uart_remove_one_port(&nuc970serial_reg, &up->port);
	}
	return 0;
}

static int nuc970serial_suspend(struct platform_device *dev, pm_message_t state)
{
	int i;

	for (i = 0; i < SCUART_NR; i++) {
		struct uart_nuc970_port *up = &nuc970serial_ports[i];

		if (up->port.type != PORT_UNKNOWN && up->port.dev == &dev->dev)
			uart_suspend_port(&nuc970serial_reg, &up->port);
	}

	return 0;
}

static int nuc970serial_resume(struct platform_device *dev)
{
	int i;

	for (i = 0; i < SCUART_NR; i++) {
		struct uart_nuc970_port *up = &nuc970serial_ports[i];

		if (up->port.type != PORT_UNKNOWN && up->port.dev == &dev->dev)
			nuc970scuart_resume_port(i);
	}

	return 0;
}

static struct platform_driver nuc970serial_driver = {
	.probe		= nuc970serial_probe,
	.remove		= nuc970serial_remove,
	.suspend	= nuc970serial_suspend,
	.resume		= nuc970serial_resume,
	.driver		= {
		.name	= "nuc970-sc",
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

	nuc970serial_init_ports();

	return ret;
}

static void __exit nuc970serial_exit(void)
{
	platform_driver_unregister(&nuc970serial_driver);
	uart_unregister_driver(&nuc970serial_reg);
}

module_init(nuc970serial_init);
module_exit(nuc970serial_exit);

EXPORT_SYMBOL(nuc970scuart_suspend_port);
EXPORT_SYMBOL(nuc970scuart_resume_port);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NUC970 scuart driver");

MODULE_ALIAS_CHARDEV_MAJOR(TTY_MAJOR);

