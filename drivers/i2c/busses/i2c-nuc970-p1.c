/*
 * linux/drivers/i2c/busses/i2c-nuc970-p1.c
 *
 * Copyright (c) 2014 Nuvoton technology corporation.
 *
 * This driver based on S3C2410 I2C driver of Ben Dooks <ben-Y5A6D6n0/KfQXOPxS62xeg@public.gmane.org>.
 * Written by Wan ZongShun <mcuos.com-Re5JQEeQqe8AvxtiuMwx3w@public.gmane.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of_i2c.h>

#include <mach/mfp.h>
#include <linux/platform_data/i2c-nuc970.h>

/* nuc970 i2c registers offset */

#define CSR     0x00
#define DIVIDER     0x04
#define CMDR        0x08
#define SWR     0x0C
#define RXR     0x10
#define TXR     0x14

/* nuc970 i2c CSR register bits */

#define IRQEN       0x003
#define I2CBUSY     0x400
#define I2CSTART    0x018
#define IRQFLAG     0x004
#define ARBIT_LOST  0x200
#define SLAVE_ACK   0x800

/* nuc970 i2c CMDR register bits */

#define I2C_CMD_START   0x10
#define I2C_CMD_STOP    0x08
#define I2C_CMD_READ    0x04
#define I2C_CMD_WRITE   0x02
#define I2C_CMD_NACK    0x01

/* i2c controller state */

enum nuc970_i2c_state {
	STATE_IDLE,
	STATE_START,
	STATE_READ,
	STATE_WRITE,
	STATE_STOP
};

/* i2c controller private data */

struct nuc970_i2c {
	spinlock_t      lock;
	wait_queue_head_t   wait;

	struct i2c_msg      *msg;
	unsigned int        msg_num;
	unsigned int        msg_idx;
	unsigned int        msg_ptr;
	unsigned int        irq;
	unsigned int        arblost;
	
	enum nuc970_i2c_state   state;

	void __iomem        *regs;
	struct clk      *clk;
	struct device       *dev;
	struct resource     *ioarea;
	struct i2c_adapter  adap;
};

/* nuc970_i2c1_master_complete
 *
 * complete the message and wake up the caller, using the given return code,
 * or zero to mean ok.
*/

static inline void nuc970_i2c1_master_complete(struct nuc970_i2c *i2c, int ret)
{
	dev_dbg(i2c->dev, "master_complete %d\n", ret);

	i2c->msg_ptr = 0;
	i2c->msg = NULL;
	i2c->msg_idx++;
	i2c->msg_num = 0;
	if (ret)
		i2c->msg_idx = ret;

	wake_up(&i2c->wait);
}

/* irq enable/disable functions */

static inline void nuc970_i2c1_disable_irq(struct nuc970_i2c *i2c)
{
	unsigned long tmp;

	tmp = readl(i2c->regs + CSR);
	writel(tmp & ~IRQEN, i2c->regs + CSR);
}

static inline void nuc970_i2c1_enable_irq(struct nuc970_i2c *i2c)
{
	unsigned long tmp;

	tmp = readl(i2c->regs + CSR);
	writel(tmp | IRQEN, i2c->regs + CSR);
}


/* nuc970_i2c1_message_start
 *
 * put the start of a message onto the bus
*/

static void nuc970_i2c1_message_start(struct nuc970_i2c *i2c,
					  struct i2c_msg *msg)
{
	unsigned int addr = (msg->addr & 0x7f) << 1;

	if (msg->flags & I2C_M_RD)
		addr |= 0x1;
	writel(addr & 0xff, i2c->regs + TXR);
	writel(I2C_CMD_START | I2C_CMD_WRITE, i2c->regs + CMDR);
}

static inline void nuc970_i2c1_stop(struct nuc970_i2c *i2c, int ret)
{

	dev_dbg(i2c->dev, "STOP\n");

	/* stop the transfer */
	i2c->state = STATE_STOP;
	writel(I2C_CMD_STOP, i2c->regs + CMDR);

	nuc970_i2c1_master_complete(i2c, ret);
}

/* helper functions to determine the current state in the set of
 * messages we are sending
*/

/* is_lastmsg()
 *
 * returns TRUE if the current message is the last in the set
*/

static inline int is_lastmsg(struct nuc970_i2c *i2c)
{
	return i2c->msg_idx >= (i2c->msg_num - 1);
}

/* is_msglast
 *
 * returns TRUE if we this is the last byte in the current message
*/

static inline int is_msglast(struct nuc970_i2c *i2c)
{
	return i2c->msg_ptr == i2c->msg->len-1;
}

/* is_msgend
 *
 * returns TRUE if we reached the end of the current message
*/

static inline int is_msgend(struct nuc970_i2c *i2c)
{
	return i2c->msg_ptr >= i2c->msg->len;
}

/* i2c_nuc970_irq_nextbyte
 *
 * process an interrupt and work out what to do
 */

static void i2c_nuc970_irq_nextbyte(struct nuc970_i2c *i2c,
							unsigned long iicstat)
{
	unsigned char byte;

	switch (i2c->state) {

	case STATE_IDLE:
		dev_err(i2c->dev, "%s: called in STATE_IDLE\n", __func__);
		break;

	case STATE_STOP:
		nuc970_i2c1_disable_irq(i2c);
		i2c->state = STATE_IDLE;
		break;

	case STATE_START:
		/* last thing we did was send a start condition on the
		 * bus, or started a new i2c message
		 */

		if (iicstat & SLAVE_ACK &&
			!(i2c->msg->flags & I2C_M_IGNORE_NAK)) {
			/* ack was not received... */

			dev_dbg(i2c->dev, "ack was not received\n");
			nuc970_i2c1_stop(i2c, -ENXIO);
			break;
		}

		if (i2c->msg->flags & I2C_M_RD)
			i2c->state = STATE_READ;
		else
			i2c->state = STATE_WRITE;

		/* terminate the transfer if there is nothing to do
		 * as this is used by the i2c probe to find devices.
		*/

		if (is_lastmsg(i2c) && i2c->msg->len == 0) {
			nuc970_i2c1_stop(i2c, 0);
			break;
		}

		if (i2c->state == STATE_READ)
			goto prepare_read;

		/* fall through to the write state, as we will need to
		 * send a byte as well
		*/

	case STATE_WRITE:
		/* we are writing data to the device... check for the
		 * end of the message, and if so, work out what to do
		 */

		if (!(i2c->msg->flags & I2C_M_IGNORE_NAK)) {
			if (iicstat & SLAVE_ACK) {
				dev_dbg(i2c->dev, "WRITE: No Ack\n");

				nuc970_i2c1_stop(i2c, -ECONNREFUSED);
				break;
			}
		}

retry_write:

		if (!is_msgend(i2c)) {
			byte = i2c->msg->buf[i2c->msg_ptr++];
			writeb(byte, i2c->regs + TXR);
			writel(I2C_CMD_WRITE, i2c->regs + CMDR);

		} else if (!is_lastmsg(i2c)) {
			/* we need to go to the next i2c message */

			dev_dbg(i2c->dev, "WRITE: Next Message\n");

			i2c->msg_ptr = 0;
			i2c->msg_idx++;
			i2c->msg++;

			/* check to see if we need to do another message */
			if (i2c->msg->flags & I2C_M_NOSTART) {

				if (i2c->msg->flags & I2C_M_RD) {
					/* cannot do this, the controller
					 * forces us to send a new START
					 * when we change direction
					*/

					nuc970_i2c1_stop(i2c, -EINVAL);
				}

				goto retry_write;
			} else {
				/* send the new start */
				nuc970_i2c1_message_start(i2c, i2c->msg);
				i2c->state = STATE_START;
			}

		} else {
			/* send stop */

			nuc970_i2c1_stop(i2c, 0);
		}
		break;

	case STATE_READ:
		/* we have a byte of data in the data register, do
		 * something with it, and then work out whether we are
		 * going to do any more read/write
		 */

		byte = readb(i2c->regs + RXR);
		i2c->msg->buf[i2c->msg_ptr++] = byte;

prepare_read:
		if (is_msglast(i2c)) {
			/* last byte of buffer */

			if (is_lastmsg(i2c))
				writel(I2C_CMD_READ | I2C_CMD_NACK,
							i2c->regs + CMDR);

		} else if (is_msgend(i2c)) {
			/* ok, we've read the entire buffer, see if there
			 * is anything else we need to do
			*/

			if (is_lastmsg(i2c)) {
				/* last message, send stop and complete */
				dev_dbg(i2c->dev, "READ: Send Stop\n");

				nuc970_i2c1_stop(i2c, 0);
			} else {
				/* go to the next transfer */
				dev_dbg(i2c->dev, "READ: Next Transfer\n");

				i2c->msg_ptr = 0;
				i2c->msg_idx++;
				i2c->msg++;

				writel(I2C_CMD_READ, i2c->regs + CMDR);
			}

		} else {
			writel(I2C_CMD_READ, i2c->regs + CMDR);
		}

		break;
	}
}

/* nuc970_i2c1_irq
 *
 * top level IRQ servicing routine
*/

static irqreturn_t nuc970_i2c_irq(int irqno, void *dev_id)
{
	struct nuc970_i2c *i2c = dev_id;
	unsigned long status;

	status = readl(i2c->regs + CSR);
	writel(status | IRQFLAG, i2c->regs + CSR);

	if (status & ARBIT_LOST) {
		/* deal with arbitration loss */
		dev_err(i2c->dev, "deal with arbitration loss\n");
		i2c->arblost = 1;
		goto out;
	}

	if (i2c->state == STATE_IDLE) {
		dev_dbg(i2c->dev, "IRQ: error i2c->state == IDLE\n");
		goto out;
	}

	/* pretty much this leaves us with the fact that we've
	 * transmitted or received whatever byte we last sent
	*/

	i2c_nuc970_irq_nextbyte(i2c, status);

 out:
	return IRQ_HANDLED;
}

/* nuc970_i2c1_hangup
 *
 * send out some dummy clocks to let SDA free
*/
static void nuc970_i2c1_hangup(struct nuc970_i2c *i2c)
{
	int i;

	for(i=0;i<2;i++) {        
		writel(0x6, i2c->regs + SWR);       //CLK Low
		ndelay(2);
		writel(0x7, i2c->regs + SWR);       //CLK High
		ndelay(2);
	}
}

/* nuc970_i2c1_set_master
 *
 * get the i2c bus for a master transaction
*/

static int nuc970_i2c1_set_master(struct nuc970_i2c *i2c)
{
	int timeout = 400;

	while (timeout-- > 0) {
		if (((readl(i2c->regs + SWR) & I2CSTART) == I2CSTART) &&
				((readl(i2c->regs + CSR) & I2CBUSY) == 0)) {
			return 0;
		}

		msleep(1);
	}

	return -ETIMEDOUT;
}

/* nuc970_i2c1_doxfer
 *
 * this starts an i2c transfer
*/

static int nuc970_i2c1_doxfer(struct nuc970_i2c *i2c,
				  struct i2c_msg *msgs, int num)
{
	unsigned long iicstat, timeout;
	int spins = 20;
	int ret;

	ret = nuc970_i2c1_set_master(i2c);
	if (ret != 0) {
		dev_err(i2c->dev, "cannot get bus (error %d)\n", ret);
		ret = -EAGAIN;
		goto out;
	}

	spin_lock_irq(&i2c->lock);

	i2c->msg     = msgs;
	i2c->msg_num = num;
	i2c->msg_ptr = 0;
	i2c->msg_idx = 0;
	i2c->state   = STATE_START;
//printk("[%x] %x %x\n", (i2c->msg->addr & 0x7f), i2c->msg->buf[0]>>1, (i2c->msg->buf[1] | ((i2c->msg->buf[0] & 1) << 8)));
	nuc970_i2c1_message_start(i2c, msgs);
	spin_unlock_irq(&i2c->lock);

	timeout = wait_event_timeout(i2c->wait, i2c->msg_num == 0, HZ * 5);
	ret = i2c->msg_idx;

	/* having these next two as dev_err() makes life very
	 * noisy when doing an i2cdetect
	*/

	if (timeout == 0)
		dev_dbg(i2c->dev, "timeout\n");
	else if (ret != num)
		dev_dbg(i2c->dev, "incomplete xfer (%d)\n", ret);

	/* ensure the stop has been through the bus */
	dev_dbg(i2c->dev, "waiting for bus idle\n");

	/* first, try busy waiting briefly */
	do {
		iicstat = readl(i2c->regs + CSR);
	} while ((iicstat & I2CBUSY) && --spins);

	/* if that timed out sleep */
	if (!spins) {
		msleep(1);
		iicstat = readl(i2c->regs + CSR);
	}

	if (iicstat & I2CBUSY)
		dev_warn(i2c->dev, "timeout waiting for bus idle\n");
	
	if(i2c->arblost) {
		dev_dbg(i2c->dev, "arb lost, stop\n");
		i2c->arblost = 0;
		nuc970_i2c1_stop(i2c, 0);
		msleep(1);
		nuc970_i2c1_disable_irq(i2c);
		nuc970_i2c1_hangup(i2c);
		ret = -EAGAIN;
	}
	
 out:
	return ret;
}

/* nuc970_i2c1_xfer
 *
 * first port of call from the i2c bus code when an message needs
 * transferring across the i2c bus.
*/

static int nuc970_i2c1_xfer(struct i2c_adapter *adap,
			struct i2c_msg *msgs, int num)
{
	struct nuc970_i2c *i2c = (struct nuc970_i2c *)adap->algo_data;
	int retry;
	int ret;

	nuc970_i2c1_enable_irq(i2c);

	for (retry = 0; retry < adap->retries; retry++) {

		ret = nuc970_i2c1_doxfer(i2c, msgs, num);

		if (ret != -EAGAIN)
			return ret;

		dev_dbg(i2c->dev, "Retrying transmission (%d)\n", retry);

		udelay(100);
	}

	return -EREMOTEIO;
}

/* declare our i2c functionality */
static u32 nuc970_i2c1_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_NOSTART |
		I2C_FUNC_PROTOCOL_MANGLING;
}

/* i2c bus registration info */

static const struct i2c_algorithm nuc970_i2c1_algorithm = {
	.master_xfer        = nuc970_i2c1_xfer,
	.functionality      = nuc970_i2c1_func,
};

/* nuc970_i2c1_probe
 *
 * called by the bus driver when a suitable device is found
*/

static int nuc970_i2c1_probe(struct platform_device *pdev)
{
	struct nuc970_i2c *i2c;
	struct nuc970_platform_i2c *pdata=NULL;
	struct resource *res;
	int ret;
	int busnum = 0, busfreq = 0;
	struct pinctrl *pinctrl;

	if (!pdev->dev.of_node) {
		pdata = pdev->dev.platform_data;
		if (!pdata) {
			dev_err(&pdev->dev, "no platform data\n");
			return -EINVAL;
		}
	}
	
	i2c = kzalloc(sizeof(struct nuc970_i2c), GFP_KERNEL);
	if (!i2c) {
		dev_err(&pdev->dev, "no memory for state\n");
		return -ENOMEM;
	}

	strlcpy(i2c->adap.name, "nuc970-i2c1", sizeof(i2c->adap.name));
	i2c->adap.owner   = THIS_MODULE;
	i2c->adap.algo    = &nuc970_i2c1_algorithm;
	i2c->adap.retries = 2;
	i2c->adap.class   = I2C_CLASS_HWMON | I2C_CLASS_SPD;

	spin_lock_init(&i2c->lock);
	init_waitqueue_head(&i2c->wait);

	/* find the clock and enable it */
	
	i2c->dev = &pdev->dev;
	i2c->clk = clk_get(NULL, "i2c1");
	if (IS_ERR(i2c->clk)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		ret = -ENOENT;
		goto err_noclk;
	}

	dev_dbg(&pdev->dev, "clock source %p\n", i2c->clk);
	
	clk_prepare(i2c->clk);
	clk_enable(i2c->clk);

	/* map the registers */

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "cannot find IO resource\n");
		ret = -ENOENT;
		goto err_clk;
	}

#if defined(CONFIG_OF)
	i2c->regs = devm_ioremap_resource(&pdev->dev, res);
#else
	i2c->ioarea = request_mem_region(res->start, resource_size(res), pdev->name);
	if (i2c->ioarea == NULL) {
		dev_err(&pdev->dev, "cannot request IO\n");
		ret = -ENXIO;
		goto err_clk;
	}

	i2c->regs = ioremap(res->start, resource_size(res));

	if (i2c->regs == NULL) {
		dev_err(&pdev->dev, "cannot map IO\n");
		ret = -ENXIO;
		goto err_ioarea;
	}

	dev_dbg(&pdev->dev, "registers %p (%p, %p)\n",
		i2c->regs, i2c->ioarea, res);
#endif

	/* setup info block for the i2c core */

	i2c->adap.algo_data = i2c;
	i2c->adap.dev.parent = &pdev->dev;

#if defined(CONFIG_OF)
	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
#else
 #ifdef CONFIG_NUC970_I2C1_PB   
	pinctrl = devm_pinctrl_get_select(&pdev->dev, "i2c1-PB");
 #elif defined(CONFIG_NUC970_I2C1_PG)
	pinctrl = devm_pinctrl_get_select(&pdev->dev, "i2c1-PG");
 #elif defined(CONFIG_NUC970_I2C1_PH)
	pinctrl = devm_pinctrl_get_select(&pdev->dev, "i2c1-PH");
 #else
	pinctrl = devm_pinctrl_get_select(&pdev->dev, "i2c1-PI");
 #endif
#endif
	if(IS_ERR(pinctrl)) { 
		dev_err(&pdev->dev, "unable to reserve pin\n");
		ret = PTR_ERR(pinctrl);
	}

	if (pdata) {
		busfreq = pdata->bus_freq;
		busnum = pdata->bus_num;
	} else {
		of_property_read_u32(pdev->dev.of_node, "bus_freq", &busfreq);
		of_property_read_u32(pdev->dev.of_node, "bus_num", &busnum);
	}
	
	ret = clk_get_rate(i2c->clk)/(busfreq * 5) - 1;
	writel(ret & 0xffff, i2c->regs + DIVIDER);

	/* find the IRQ for this unit (note, this relies on the init call to
	 * ensure no current IRQs pending
	 */

	i2c->irq = ret = platform_get_irq(pdev, 0);
	if (ret <= 0) {
		dev_err(&pdev->dev, "cannot find IRQ\n");
		goto err_iomap;
	}

	ret = request_irq(i2c->irq, nuc970_i2c_irq, IRQF_SHARED,
			  dev_name(&pdev->dev), i2c);

	if (ret != 0) {
		dev_err(&pdev->dev, "cannot claim IRQ %d\n", i2c->irq);
		goto err_iomap;
	}

	/* Note, previous versions of the driver used i2c_add_adapter()
	 * to add the bus at any number. We now pass the bus number via
	 * the platform data, so if unset it will now default to always
	 * being bus 0.
	 */

	i2c->adap.nr = busnum;
	i2c->adap.dev.of_node = pdev->dev.of_node;

	ret = i2c_add_numbered_adapter(&i2c->adap);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add bus to i2c core\n");
		goto err_irq;
	}
	of_i2c_register_devices(&i2c->adap); 

	platform_set_drvdata(pdev, i2c);

	dev_info(&pdev->dev, "%s: nuc970 I2C adapter\n",
						dev_name(&i2c->adap.dev));

	return 0;

 err_irq:
	free_irq(i2c->irq, i2c);

 err_iomap:
	iounmap(i2c->regs);

#ifndef CONFIG_OF
 err_ioarea:
	release_resource(i2c->ioarea);
	kfree(i2c->ioarea);
#endif

 err_clk:
	clk_disable(i2c->clk);
	clk_put(i2c->clk);

 err_noclk:
	kfree(i2c);
	return ret;
}

/* nuc970_i2c1_remove
 *
 * called when device is removed from the bus
*/

static int nuc970_i2c1_remove(struct platform_device *pdev)
{
	struct nuc970_i2c *i2c = platform_get_drvdata(pdev);

	i2c_del_adapter(&i2c->adap);
	free_irq(i2c->irq, i2c);

	clk_disable(i2c->clk);
	clk_put(i2c->clk);

	iounmap(i2c->regs);

	release_resource(i2c->ioarea);
	kfree(i2c->ioarea);
	kfree(i2c);

	return 0;
}

#ifdef CONFIG_PM
static int nuc970_i2c1_suspend(struct device *dev)
{
	struct nuc970_i2c *i2c = dev_get_drvdata(dev);
	
	while(i2c->state != STATE_IDLE)
		msleep(1);
	
	//disable i2c
	writel(0x0, i2c->regs + CSR);
	
	//free SCL,SDA
	writel(0x1, i2c->regs + SWR);
	writel(0x7, i2c->regs + SWR);
	
	return 0;
}

static int nuc970_i2c1_resume(struct device *dev)
{
		
	return 0;
}

static const struct dev_pm_ops nuc970_i2c1_pmops = {
	.suspend    = nuc970_i2c1_suspend,
	.resume     = nuc970_i2c1_resume,
};

#define NUC970_I2C1_PMOPS (&nuc970_i2c1_pmops)

#else
#define NUC970_I2C1_PMOPS NULL
#endif

#if defined(CONFIG_OF)
static const struct of_device_id nuc970_i2c1_of_match[] = {
	{   .compatible = "nuvoton,nuc970-i2c1" },
	{   },
};
MODULE_DEVICE_TABLE(of, nuc970_i2c1_of_match);
#endif

static struct platform_driver nuc970_i2c1_driver = {
	.probe      = nuc970_i2c1_probe,
	.remove     = nuc970_i2c1_remove,
	.driver     = {
		.name   = "nuc970-i2c1",
		.owner  = THIS_MODULE,
#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(nuc970_i2c1_of_match),
#endif
		.pm = NUC970_I2C1_PMOPS,
	},
};
module_platform_driver(nuc970_i2c1_driver);

MODULE_DESCRIPTION("nuc970 I2C Bus driver");
MODULE_AUTHOR("Wan ZongShun, <mcuos.com-Re5JQEeQqe8AvxtiuMwx3w@public.gmane.org>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nuc970-i2c1");
