/*
 * 
 *
 *
 */


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/mutex.h>

#include <linux/serial_core.h>
#include <linux/serial.h>


#include <net/irda/irda.h>
#include <net/irda/irda_device.h>

#include <asm/serial.h>

#include <mach/map.h>
#include <mach/regs-serial.h>
#include <mach/regs-gcr.h>
#include <mach/regs-clock.h>


#include "sir-dev.h"


#include "nuc970_sir.h"

/*
 * This defines the low- and high-watermarks for throttling and
 * unthrottling the TTY driver.  These watermarks are used for
 * controlling the space in the read buffer.
 */
#define TTY_THRESHOLD_THROTTLE		128 /* now based on remaining room */
#define TTY_THRESHOLD_UNTHROTTLE	128


static int qos_mtt_bits = 0x03;      /* 5 ms or more */

module_param(qos_mtt_bits, int, 0);
MODULE_PARM_DESC(qos_mtt_bits, "Minimum Turn Time");

/* serialize ldisc open/close with sir_dev */
static DEFINE_MUTEX(nuc970irda_mutex);

struct n_irda_data {
	unsigned char lnext:1, erasing:1, raw:1, real_raw:1, icanon:1;

	DECLARE_BITMAP(read_flags, N_TTY_BUF_SIZE);

	char *read_buf;
	int read_head;
	int read_tail;
	int read_cnt;

	int canon_data;
	unsigned long canon_head;
	unsigned int canon_column;

	struct mutex atomic_read_lock;
	raw_spinlock_t read_lock;
};



struct uart_nuc970sir_port {
	struct uart_port	port;

	unsigned short		capabilities;	/* port capabilities */
	unsigned char		ier;
	unsigned char		lcr;
	unsigned char		mcr;
	unsigned char		mcr_mask;	/* mask of user bits */
	unsigned char		mcr_force;	/* mask of forced bits */

	/*
	 * We provide a per-port pm hook.
	 */
	void			(*pm)(struct uart_port *port,
				      unsigned int state, unsigned int old);
};

static void n_irda_set_room(struct tty_struct *tty);


/* called from sir_dev when there is more data to send
 * context is either netdev->hard_xmit or some transmit-completion bh
 * i.e. we are under spinlock here and must not sleep.
 */
static int nuc970irda_do_write(struct sir_dev *dev, const unsigned char *ptr, size_t len)
{
	struct sirtty_cb *priv = dev->priv;
	struct tty_struct *tty;
	int writelen;

	IRDA_ASSERT(priv != NULL, return -1;);
	IRDA_ASSERT(priv->magic == IRTTY_MAGIC, return -1;);

	printk("\n nuc970irda_do_write \n");

	tty = priv->tty;
	if (!tty->ops->write)
		return 0;
	set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
	writelen = tty_write_room(tty);
	if (writelen > len)
		writelen = len;
	return tty->ops->write(tty, ptr, writelen);
}

/* notifier from sir_dev when irda% device gets opened (ifup) */
static int nuc970irda_start_dev(struct sir_dev *dev)
{
	struct sirtty_cb *priv;
	struct tty_struct *tty;

	/* serialize with ldisc open/close */
	mutex_lock(&nuc970irda_mutex);

	printk("\n nuc970irda_start_dev \n");

	priv = dev->priv;
	if (unlikely(!priv || priv->magic!=IRTTY_MAGIC)) {
		mutex_unlock(&nuc970irda_mutex);
		return -ESTALE;
	}

	tty = priv->tty;

	if (tty->ops->start)
		tty->ops->start(tty);

	mutex_unlock(&nuc970irda_mutex);
	return 0;
}


static struct sir_driver sir_nuc970_drv = {
	.owner			= THIS_MODULE,
	.driver_name		= "sir_nuc970",
	.start_dev		= nuc970irda_start_dev,
	//.stop_dev		= NULL,
	.do_write		= nuc970irda_do_write,
	//.chars_in_buffer	= NULL,
	//.wait_until_sent	= NULL,
	//.set_speed		= NULL,
	//.set_dtr_rts		= NULL,
};


/*
 * Function nuc970irda_ioctl (tty, file, cmd, arg)
 *
 *     The Swiss army knife of system calls :-)
 *
 */
static int nuc970irda_ioctl(struct tty_struct *tty, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct irtty_info { char name[6]; } ;
	int err = 0;

	switch (cmd) {
	case IRTTY_IOCTDONGLE:

		break;

	case IRTTY_IOCGET:

		break;
	default:
		err = tty_mode_ioctl(tty, file, cmd, arg);
		break;
	}
	return err;
}

static inline void nuc970_stop_receiver(struct tty_struct *tty, int stop)
{
	struct ktermios old_termios;
	int cflag;

	mutex_lock(&tty->termios_mutex);
	old_termios = tty->termios;
	cflag = tty->termios.c_cflag;
	
	if (stop)
		cflag &= ~CREAD;
	else
		cflag |= CREAD;

	tty->termios.c_cflag = cflag;
	if (tty->ops->set_termios)
		tty->ops->set_termios(tty, &old_termios);
	mutex_unlock(&tty->termios_mutex);
}


static void nuc970_reset_buffer_flags(struct n_irda_data *ldata)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&ldata->read_lock, flags);
	ldata->read_head = ldata->read_tail = ldata->read_cnt = 0;
	raw_spin_unlock_irqrestore(&ldata->read_lock, flags);

	ldata->canon_head = ldata->canon_data = ldata->erasing = 0;
	bitmap_zero(ldata->read_flags, N_TTY_BUF_SIZE);
}

/* 
 *  Function nuc970irda_open
 *
 *    This function is called by the TTY module when the IrDA line
 *    discipline is called for.  Because we are sure the tty line exists,
 *    we only have to link it to a free IrDA channel.  
 */
static int nuc970irda_open(struct tty_struct *tty) 
{
	struct sir_dev *dev;
	struct sirtty_cb *priv = NULL;
	int ret = 0;
	struct n_irda_data *ldata = NULL;

	ldata = kzalloc(sizeof(*ldata), GFP_KERNEL);
	if (!ldata){
		ret = -ENOMEM;
		goto err_free_bufs;
	}

	ldata->read_buf = kzalloc(N_TTY_BUF_SIZE, GFP_KERNEL);
	if (!ldata->read_buf){
		ret = -ENOMEM;
		goto err_free_bufs;
	}

	/* stop the underlying  driver */
	nuc970_stop_receiver(tty, TRUE);
	if (tty->ops->stop)
		tty->ops->stop(tty);

	tty_driver_flush_buffer(tty);
	
	/* get a sir device instance for this driver */
	dev = sirdev_get_instance(&sir_nuc970_drv, tty->name);
	if (!dev) {
		ret = -ENODEV;
		goto out;
	}

	/* allocate private device info block */
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto err_free_bufs;
	}

	priv->magic = IRTTY_MAGIC;
	priv->tty = tty;
	priv->dev = dev;

	/* serialize with start_dev - in case we were racing with ifup */
	mutex_lock(&nuc970irda_mutex);

	dev->priv = priv;

	tty->disc_data = ldata;
		nuc970_reset_buffer_flags(tty->disc_data);
	tty->receive_room = 65536;

	mutex_unlock(&nuc970irda_mutex);

	IRDA_DEBUG(0, "%s - %s: irda line discipline opened\n", __func__, tty->name);

	return 0;

out:

err_free_bufs:
		kfree(ldata->read_buf);
		kfree(ldata);
		kfree(priv);
	
	return ret;
}

/* 
 *  Function nuc970irda_close (tty)
 *
 *    Close down a IrDA channel. This means flushing out any pending queues,
 *    and then restoring the TTY line discipline to what it was before it got
 *    hooked to IrDA (which usually is TTY again).  
 */
static void nuc970irda_close(struct tty_struct *tty) 
{
	/* we are dead now */
	tty->disc_data = NULL;

	/* Stop tty */
	if (tty->ops->stop)
		tty->ops->stop(tty);


	IRDA_DEBUG(0, "%s - %s: irda line discipline closed\n", __func__, tty->name);
}


/**
 *	nuc970irda_copy_from_read_buf	-	copy read data directly
 *	@tty: terminal device
 *	@b: user data
 *	@nr: size of data
 *
 *	Helper function to speed up n_tty_read.  It is only called when
 *	ICANON is off; it copies characters straight from the tty queue to
 *	user space directly.  It can be profitably called twice; once to
 *	drain the space from the tail pointer to the (physical) end of the
 *	buffer, and once to drain the space from the (physical) beginning of
 *	the buffer to head pointer.
 *
 *	Called under the ldata->atomic_read_lock sem
 *
 */
static int nuc970irda_copy_from_read_buf(struct tty_struct *tty,
				      unsigned char __user **b,
				      size_t *nr)

{
	struct n_irda_data *ldata = tty->disc_data;
	int retval;
	size_t n;
	unsigned long flags;
	bool is_eof;

	retval = 0;
	raw_spin_lock_irqsave(&ldata->read_lock, flags);
	n = min(ldata->read_cnt, N_TTY_BUF_SIZE - ldata->read_tail);
	n = min(*nr, n);
	raw_spin_unlock_irqrestore(&ldata->read_lock, flags);
	if (n) {
		retval = copy_to_user(*b, &ldata->read_buf[ldata->read_tail], n);
		n -= retval;
		is_eof = n == 1 &&
			ldata->read_buf[ldata->read_tail] == EOF_CHAR(tty);
		tty_audit_add_data(tty, &ldata->read_buf[ldata->read_tail], n,
				ldata->icanon);
		raw_spin_lock_irqsave(&ldata->read_lock, flags);
		ldata->read_tail = (ldata->read_tail + n) & (N_TTY_BUF_SIZE-1);
		ldata->read_cnt -= n;
		/* Turn single EOF into zero-length read */
		if (L_EXTPROC(tty) && ldata->icanon && is_eof && !ldata->read_cnt)
			n = 0;
		raw_spin_unlock_irqrestore(&ldata->read_lock, flags);
		*b += n;
		*nr -= n;
	}
	return retval;
}

ssize_t	nuc970irda_read(struct tty_struct *tty, struct file *file,
			unsigned char __user *buf, size_t nr)
{
	unsigned char __user *b = buf;
	DECLARE_WAITQUEUE(wait, current);
	int minimum, time;
	ssize_t retval = 0;
	ssize_t size;
	//long timeout;
	int uncopied;

do_it_again:

	minimum = time = 0;
	//timeout = MAX_SCHEDULE_TIMEOUT;

	add_wait_queue(&tty->read_wait, &wait);
	while (nr) {
		uncopied = nuc970irda_copy_from_read_buf(tty, &b, &nr);

		if (uncopied) {
			retval = -EFAULT;
			//break;
			continue;
		}

		if (b - buf >= minimum)
			break;
	}

	remove_wait_queue(&tty->read_wait, &wait);

	size = b - buf;
	if (size) {
		retval = size;
		if (nr)
			clear_bit(TTY_PUSH, &tty->flags);
	} else if (test_and_clear_bit(TTY_PUSH, &tty->flags))
		goto do_it_again;

	n_irda_set_room(tty);
	return retval;

}


ssize_t	nuc970irda_write(struct tty_struct *tty, struct file *file,
			 const unsigned char *buf, size_t nr)
{
	int writelen;

	writelen = tty_write_room(tty);
	if (writelen > nr)
		writelen = nr;
	return tty->ops->write(tty, buf, writelen);	
}

/**
 *	n_irda_set_room	-	receive space
 *	@tty: terminal
 *
 *	Called by the driver to find out how much data it is
 *	permitted to feed to the line discipline without any being lost
 *	and thus to manage flow control. Not serialized. Answers for the
 *	"instant".
 */

static void n_irda_set_room(struct tty_struct *tty)
{
	struct n_irda_data *ldata = tty->disc_data;
	int left;
	int old_left;

	/* ldata->read_cnt is not read locked ? */
	if (I_PARMRK(tty)) {
		/* Multiply read_cnt by 3, since each byte might take up to
		 * three times as many spaces when PARMRK is set (depending on
		 * its flags, e.g. parity error). */
		left = N_TTY_BUF_SIZE - ldata->read_cnt * 3 - 1;
	} else
		left = N_TTY_BUF_SIZE - ldata->read_cnt - 1;

	/*
	 * If we are doing input canonicalization, and there are no
	 * pending newlines, let characters through without limit, so
	 * that erase characters will be handled.  Other excess
	 * characters will be beeped.
	 */
	if (left <= 0)
		left = ldata->icanon && !ldata->canon_data;
	old_left = tty->receive_room;
	tty->receive_room = left;

	/* Did this open up the receive buffer? We may need to flip */
	if (left && !old_left) {
		WARN_RATELIMIT(tty->port->itty == NULL,
				"scheduling with invalid itty\n");
		/* see if ldisc has been killed - if so, this means that
		 * even though the ldisc has been halted and ->buf.work
		 * cancelled, ->buf.work is about to be rescheduled
		 */
		WARN_RATELIMIT(test_bit(TTY_LDISC_HALTED, &tty->flags),
			       "scheduling buffer work for halted ldisc\n");
		schedule_work(&tty->port->buf.work);
	}
}


/* 
 *  Function nuc970irda_receive_buf( tty, cp, count)
 *
 *    Handle the 'receiver data ready' interrupt.  This function is called
 *    by the 'tty_io' module in the kernel when a block of IrDA data has
 *    been received, which can now be decapsulated and delivered for
 *    further processing 
 *
 * calling context depends on underlying driver and tty->port->low_latency!
 * for example (low_latency: 1 / 0):
 * serial.c:	uart-interrupt / softint
 * usbserial:	urb-complete-interrupt / softint
 */

static void nuc970irda_receive_buf(struct tty_struct *tty, const unsigned char *cp,
			      char *fp, int count) 
{
	struct n_irda_data *ldata = tty->disc_data;
	int i;
	unsigned long cpuflags;

	raw_spin_lock_irqsave(&ldata->read_lock, cpuflags);
	i = min(N_TTY_BUF_SIZE - ldata->read_cnt,
		N_TTY_BUF_SIZE - ldata->read_head);
	i = min(count, i);
	memcpy(ldata->read_buf + ldata->read_head, cp, i);
	ldata->read_head = (ldata->read_head + i) & (N_TTY_BUF_SIZE-1);
	ldata->read_cnt += i;
	cp += i;
	count -= i;

	raw_spin_unlock_irqrestore(&ldata->read_lock, cpuflags);

	n_irda_set_room(tty);

	/*
	 * Check the remaining room for the input canonicalization
	 * mode.  We don't want to throttle the driver if we're in
	 * canonical mode and don't have a newline yet!
	 */
	while (1) {
		tty_set_flow_change(tty, TTY_THROTTLE_SAFE);
		if (tty->receive_room >= TTY_THRESHOLD_THROTTLE)
			break;
		if (!tty_throttle_safe(tty))
			break;
	}
	__tty_set_flow_change(tty, 0);
	
}

/*
 * Function nuc970irda_write_wakeup (tty)
 *
 *    Called by the driver when there's room for more data.  If we have
 *    more packets to send, we send them here.
 *
 */
static void nuc970irda_write_wakeup(struct tty_struct *tty) 
{

}


static struct tty_ldisc_ops nuc970irda_ldisc = {
	.magic		= TTY_LDISC_MAGIC,
 	.name		= "nuc970-sir",
	.flags		= 0,
	.open		= nuc970irda_open,
	.close		= nuc970irda_close,
	.read		= nuc970irda_read,
	.write		= nuc970irda_write,
	.ioctl		= nuc970irda_ioctl,
 	.poll		= NULL,
	.receive_buf	= nuc970irda_receive_buf,
	.write_wakeup	= nuc970irda_write_wakeup,
	.owner		= THIS_MODULE,
};


static int __init nuc970_sir_init(void)
{
	int err;

	if ((err = tty_register_ldisc(N_IRDA, &nuc970irda_ldisc)) != 0)
		IRDA_ERROR("IrDA: can't register line discipline (err = %d)\n",
			   err);
	return err;
}


static void __exit nuc970_sir_cleanup(void) 
{
	int err;

	if ((err = tty_unregister_ldisc(N_IRDA))) {
		IRDA_ERROR("%s(), can't unregister line discipline (err = %d)\n",
			   __func__, err);
	}
}


module_init(nuc970_sir_init);
module_exit(nuc970_sir_cleanup);

MODULE_AUTHOR("D>");
MODULE_DESCRIPTION("IrDA device driver");
MODULE_ALIAS_LDISC(N_IRDA);
MODULE_LICENSE("GPL");



