/* linux/driver/char/nuc970-etimer.c
 *
 * Copyright (c) 2014 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <mach/map.h>
#include <mach/regs-gcr.h>
#include <mach/regs-clock.h>
#include <mach/regs-etimer.h>
#include <mach/nuc970-etimer.h>

#define ETIMER_CH	4
#define ETIMER_OPMODE_NONE		0
//#define ETIMER_OPMODE_ONESHOT		1
//#define ETIMER_OPMODE_PERIODIC	2
//#define ETIMER_OPMODE_CONTINUOUS	3
#define ETIMER_OPMODE_TOGGLE		4
#define ETIMER_OPMODE_TRIGGER_COUNTING	5
#define ETIMER_OPMODE_FREE_COUNTING	6

#define ETIMER_CTL_ETMR_EN		0x00000001
#define ETIMER_CTL_ONESHOT		0x00000000
#define ETIMER_CTL_PERIODIC		0x00000010
#define ETIMER_CTL_TOGGLE		0x00000020
#define ETIMER_CTL_CONTINUOUS		0x00000030
#define ETIMER_CTL_TCAP_EN		0x00010000
#define ETIMER_CTL_FREE_COUNTING	0x00000000
#define ETIMER_CTL_TRIGGER_COUNTING	0x00100000
#define ETIMER_IER_TCAP_IE		0x00000002

#define ETIMER_TRIGGER_COUNTING 	(ETIMER_CTL_TRIGGER_COUNTING |\
					ETIMER_CTL_TCAP_EN |\
					ETIMER_CTL_PERIODIC |\
					ETIMER_CTL_ETMR_EN)

#define ETIMER_FREE_COUNTING 		(ETIMER_CTL_FREE_COUNTING |\
					ETIMER_CTL_TCAP_EN |\
					ETIMER_CTL_PERIODIC |\
					ETIMER_CTL_ETMR_EN)

#define ETIMER_TOGGLE			(ETIMER_CTL_TOGGLE | ETIMER_CTL_ETMR_EN)

struct nuc970_etimer {
	spinlock_t lock;
	struct pinctrl *pinctrl;
	struct clk *clk;
	struct clk *eclk;
	wait_queue_head_t wq;
	int minor;	// dynamic minor num, so we need this to distinguish between channels
	u32 cap;	// latest capture data
	int irq;	// interrupt number
	u8 ch;		// Etimer channel. 0~3
	u8 mode;	// Current OP mode. Counter, free counting, trigger counting...
	u8 occupied;	// device opened
	u8 update;	// new capture data available

};


static struct nuc970_etimer *etmr[ETIMER_CH];

static irqreturn_t nuc970_etimer_interrupt(int irq, void *dev_id)
{
	struct nuc970_etimer *t = (struct nuc970_etimer *)dev_id;
	int ch = t->ch;

	__raw_writel(__raw_readl(REG_ETMR_ISR(ch)), REG_ETMR_ISR(ch));
	spin_lock(&t->lock);
	if(t->mode == ETIMER_OPMODE_TRIGGER_COUNTING ||
	   t->mode == ETIMER_OPMODE_FREE_COUNTING) {
		t->update = 1;
		t->cap = __raw_readl(REG_ETMR_TCAP(ch));
		wake_up_interruptible(&t->wq);
	}
	spin_unlock(&t->lock);
	return IRQ_HANDLED;
}

static void stop_timer(struct nuc970_etimer *t)
{
	unsigned long flag;

	spin_lock_irqsave(&t->lock, flag);
	// stop timer
	__raw_writel(0, REG_ETMR_CTL(t->ch));
	// disable interrupt
	__raw_writel(0, REG_ETMR_IER(t->ch));
	// clear interrupt flag if any
	__raw_writel(0xFFFFFFFF, REG_ETMR_ISR(t->ch));
	t->mode = ETIMER_OPMODE_NONE;
	spin_unlock_irqrestore(&t->lock, flag);


}

static ssize_t etimer_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	unsigned long flag;
	struct nuc970_etimer *t = (struct nuc970_etimer *)filp->private_data;
	int ret = 0;

	spin_lock_irqsave(&t->lock, flag);
	if(t->mode != ETIMER_OPMODE_TRIGGER_COUNTING &&
	   t->mode != ETIMER_OPMODE_FREE_COUNTING) {
		ret = -EPERM;
		goto out;
	}

	if(t->update) {
		if(copy_to_user(buf, &t->cap, sizeof(unsigned int)))
			ret = -EFAULT;
		else
			ret = 4;	// size of int.
		t->update = 0;
		goto out;
	} else {
		spin_unlock_irqrestore(&t->lock, flag);
		wait_event_interruptible(t->wq, t->update != 0);
		if(copy_to_user(buf, &t->cap, sizeof(unsigned int)))
			ret = -EFAULT;
		else
			ret = 4;
		t->update = 0;
		return ret;
	}

out:
	spin_unlock_irqrestore(&t->lock, flag);
	return ret;
}


static int etimer_release(struct inode *inode, struct file *filp)
{
	struct nuc970_etimer *t = (struct nuc970_etimer *)filp->private_data;
	int ch = t->ch;
	unsigned long flag;

	stop_timer(t);

	// free irq
	free_irq(etmr[ch]->irq, etmr[ch]);
	// disable clk
	clk_disable(etmr[ch]->clk);
	clk_disable(etmr[ch]->eclk);
	clk_put(etmr[ch]->clk);
	clk_put(etmr[ch]->eclk);

	spin_lock_irqsave(&etmr[ch]->lock, flag);
	etmr[ch]->occupied = 0;
	spin_unlock_irqrestore(&etmr[ch]->lock, flag);
	filp->private_data = NULL;

	return(0);
}

static int etimer_open(struct inode *inode, struct file *filp)
{
	int i, ret, ch = 0;
	unsigned long flag;
	struct clk *clkmux, *clkhxt;

	for(i = 0; i < ETIMER_CH; i++)
		if(MINOR(inode->i_rdev) == etmr[i]->minor) {
			ch = i;
			break;
		}

	spin_lock_irqsave(&etmr[ch]->lock, flag);
	if(etmr[ch]->occupied) {
		spin_unlock_irqrestore(&etmr[ch]->lock, flag);
		return -EBUSY;
	}

	etmr[ch]->occupied = 1;
	spin_unlock_irqrestore(&etmr[ch]->lock, flag);

	if (request_irq(etmr[ch]->irq, nuc970_etimer_interrupt,
						0x0, "nuc970-etimer", etmr[ch])) {
		pr_debug("register irq failed %d\n", etmr[ch]->irq);
		ret = -EAGAIN;
		goto out2;
	}

	filp->private_data = etmr[ch];

	// configure engine clock
	clkhxt = clk_get(NULL, "xin");
	if (IS_ERR(clkhxt)) {
		pr_debug("failed to get xin clk\n");
		ret = PTR_ERR(clkhxt);
		goto out1;
	}
	if(ch == 0) {
		clkmux = clk_get(NULL, "etmr0_eclk_mux");
	} else if (ch == 1) {
		clkmux = clk_get(NULL, "etmr1_eclk_mux");
	} else if (ch == 2) {
		clkmux = clk_get(NULL, "etmr2_eclk_mux");
	} else {
		clkmux = clk_get(NULL, "etmr3_eclk_mux");
	}
	if (IS_ERR(clkmux)) {
		pr_debug("failed to get etimer clock mux\n");
		ret = PTR_ERR(clkmux);
		goto out1;
	}
	clk_set_parent(clkmux, clkhxt);

	if(ch == 0) {
		etmr[ch]->clk = clk_get(NULL, "etimer0");
		etmr[ch]->eclk = clk_get(NULL, "etmr0_eclk");
	} else if(ch == 1) {
		etmr[ch]->clk = clk_get(NULL, "etimer1");
		etmr[ch]->eclk = clk_get(NULL, "etmr1_eclk");
	} else if(ch == 2) {
		etmr[ch]->clk = clk_get(NULL, "etimer2");
		etmr[ch]->eclk = clk_get(NULL, "etmr2_eclk");
	} else {
		etmr[ch]->clk = clk_get(NULL, "etimer3");
		etmr[ch]->eclk = clk_get(NULL, "etmr3_eclk");
	}


	if (IS_ERR(etmr[ch]->clk)) {
		pr_debug("failed to get etmr clock\n");
		ret = PTR_ERR(etmr[ch]->clk);
		goto out1;
	}


	if (IS_ERR(etmr[ch]->eclk)) {
		pr_debug("failed to get etmr eclock\n");
		ret = PTR_ERR(etmr[ch]->eclk);
		goto out1;
	}

	clk_prepare(etmr[ch]->clk);
	clk_enable(etmr[ch]->clk);
	clk_prepare(etmr[ch]->eclk);
	clk_enable(etmr[ch]->eclk);

	return 0;


out1:

	free_irq(etmr[ch]->irq, etmr[ch]);
out2:
	spin_lock_irqsave(&etmr[ch]->lock, flag);
	etmr[ch]->occupied = 0;
	spin_unlock_irqrestore(&etmr[ch]->lock, flag);

	return ret;

}

static long etimer_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned long flag;
	struct nuc970_etimer *t = (struct nuc970_etimer *)filp->private_data;
	int ch = t->ch;
	int unsigned param;
	struct pinctrl_state *s;
	int ret;

	// stop timer before we do any change
	stop_timer(t);

	switch(cmd) {
		case ETMR_IOC_STOP:
			// timer stopped
			break;
		case ETMR_IOC_TOGGLE:
			// get output duty in us
			if(copy_from_user((void *)&param, (const void *)arg, sizeof(unsigned int)))
				return -EFAULT;
			// divide by 2 because a duty cycle is high + low
			param >>= 1;
			// compare register is 24-bit width
			if(param > 0xFFFFFF)
				return -EPERM;

			// set pin function
			if(t->ch == 0) {
#if defined (CONFIG_NUC970_ETIMER0_TGL_PB2)
				s = pinctrl_lookup_state(t->pinctrl, "etimer0-tgl-PB");
#elif defined (CONFIG_NUC970_ETIMER0_TGL_PC6)
				s = pinctrl_lookup_state(t->pinctrl, "etimer0-tgl-PC");
#elif defined (CONFIG_NUC970_ETIMER0_TGL_NONE)
				return -EPERM;
#endif
			} else if(t->ch == 1) {
#if defined (CONFIG_NUC970_ETIMER1_TGL_PB0)
				s = pinctrl_lookup_state(t->pinctrl, "etimer1-tgl-PB");
#elif defined (CONFIG_NUC970_ETIMER1_TGL_PC8)
				s = pinctrl_lookup_state(t->pinctrl, "etimer1-tgl-PC");
#elif defined (CONFIG_NUC970_ETIMER1_TGL_NONE)
				return -EPERM;
#endif
			} else if(t->ch == 2) {
#if defined (CONFIG_NUC970_ETIMER2_TGL_PF11)
				s = pinctrl_lookup_state(t->pinctrl, "etimer2-tgl-PF");
#elif defined (CONFIG_NUC970_ETIMER2_TGL_PC10)
				s = pinctrl_lookup_state(t->pinctrl, "etimer2-tgl-PC");
#elif defined (CONFIG_NUC970_ETIMER2_TGL_NONE)
				return -EPERM;
#endif
			} else { // ch 3
#if defined (CONFIG_NUC970_ETIMER3_TGL_PF13)
				s = pinctrl_lookup_state(t->pinctrl, "etimer3-tgl-PF");
#elif defined (CONFIG_NUC970_ETIMER3_TGL_PC12)
				s = pinctrl_lookup_state(t->pinctrl, "etimer3-tgl-PC");
#elif defined (CONFIG_NUC970_ETIMER3_TGL_NONE)
				return -EPERM;
#endif
			}

			if (IS_ERR(s)) {
				pr_debug("pinctrl_lookup_state err\n");
				return -EPERM;
			}

			if((ret = pinctrl_select_state(t->pinctrl, s)) < 0) {
				pr_debug("pinctrl_select_state err\n");
				return ret;
			}

			spin_lock_irqsave(&t->lock, flag);
			// timer clock is 12MHz, set prescaler to 12 - 1.
			__raw_writel(11, REG_ETMR_PRECNT(ch));
			__raw_writel(param, REG_ETMR_CMPR(ch));
			__raw_writel(ETIMER_TOGGLE, REG_ETMR_CTL(ch));
			t->mode = ETIMER_OPMODE_TOGGLE;

			spin_unlock_irqrestore(&t->lock, flag);

			break;

		case ETMR_IOC_FREE_COUNTING:
			// get capture setting
			if(copy_from_user((void *)&param, (const void *)arg, sizeof(unsigned int)))
				return -EFAULT;

			// set pin function
			if(t->ch == 0) {
#if defined (CONFIG_NUC970_ETIMER0_CAP_PB3)
				s = pinctrl_lookup_state(t->pinctrl, "etimer0-cap-PB");
#elif defined (CONFIG_NUC970_ETIMER0_CAP_PC7)
				s = pinctrl_lookup_state(t->pinctrl, "etimer0-cap-PC");
#elif defined (CONFIG_NUC970_ETIMER0_CAP_NONE)
				return -EPERM;
#endif
			} else if(t->ch == 1) {
#if defined (CONFIG_NUC970_ETIMER1_CAP_PB1)
				s = pinctrl_lookup_state(t->pinctrl, "etimer1-cap-PB");
#elif defined (CONFIG_NUC970_ETIMER1_CAP_PC9)
				s = pinctrl_lookup_state(t->pinctrl, "etimer1-cap-PC");
#elif defined (CONFIG_NUC970_ETIMER1_CAP_NONE)
				return -EPERM;
#endif
			} else if(t->ch == 2) {
#if defined (CONFIG_NUC970_ETIMER2_CAP_PF12)
				s = pinctrl_lookup_state(t->pinctrl, "etimer2-cap-PF");
#elif defined (CONFIG_NUC970_ETIMER2_CAP_PC11)
				s = pinctrl_lookup_state(t->pinctrl, "etimer2-cap-PC");
#elif defined (CONFIG_NUC970_ETIMER2_CAP_NONE)
				return -EPERM;
#endif
			} else { // ch 3
#if defined (CONFIG_NUC970_ETIMER3_CAP_PF14)
				s = pinctrl_lookup_state(t->pinctrl, "etimer3-cap-PF");
#elif defined (CONFIG_NUC970_ETIMER3_CAP_PC13)
				s = pinctrl_lookup_state(t->pinctrl, "etimer3-cap-PC");
#elif defined (CONFIG_NUC970_ETIMER3_CAP_NONE)
				return -EPERM;
#endif
			}

			if (IS_ERR(s)) {
				pr_debug("pinctrl_lookup_state err\n");
				return -EPERM;
			}
			if((ret = pinctrl_select_state(t->pinctrl, s)) < 0) {
				pr_debug("pinctrl_select_state err\n");
				return ret;
			}

			spin_lock_irqsave(&t->lock, flag);
			// timer clock is 12MHz, set prescaler to 12 - 1.
			__raw_writel(11, REG_ETMR_PRECNT(ch));
			__raw_writel(0xFFFFFF, REG_ETMR_CMPR(ch));
			// enable capture interrupt
			__raw_writel(ETIMER_IER_TCAP_IE, REG_ETMR_IER(ch));
			__raw_writel(param | ETIMER_FREE_COUNTING, REG_ETMR_CTL(ch));
			t->mode = ETIMER_OPMODE_FREE_COUNTING;
			spin_unlock_irqrestore(&t->lock, flag);

			break;

		case ETMR_IOC_TRIGGER_COUNTING:
			// get capture setting
			if(copy_from_user((void *)&param, (const void *)arg, sizeof(unsigned int)))
				return -EFAULT;

			// set pin function
			if(t->ch == 0) {
#if defined (CONFIG_NUC970_ETIMER0_CAP_PB3)
				s = pinctrl_lookup_state(t->pinctrl, "etimer0-cap-PB");
#elif defined (CONFIG_NUC970_ETIMER0_CAP_PC7)
				s = pinctrl_lookup_state(t->pinctrl, "etimer0-cap-PC");
#elif defined (CONFIG_NUC970_ETIMER0_CAP_NONE)
				return -EPERM;
#endif
			} else if(t->ch == 1) {
#if defined (CONFIG_NUC970_ETIMER1_CAP_PB1)
				s = pinctrl_lookup_state(t->pinctrl, "etimer1-cap-PB");
#elif defined (CONFIG_NUC970_ETIMER1_CAP_PC9)
				s = pinctrl_lookup_state(t->pinctrl, "etimer1-cap-PC");
#elif defined (CONFIG_NUC970_ETIMER1_CAP_NONE)
				return -EPERM;
#endif
			} else if(t->ch == 2) {
#if defined (CONFIG_NUC970_ETIMER2_CAP_PF12)
				s = pinctrl_lookup_state(t->pinctrl, "etimer2-cap-PF");
#elif defined (CONFIG_NUC970_ETIMER2_CAP_PC11)
				s = pinctrl_lookup_state(t->pinctrl, "etimer2-cap-PC");
#elif defined (CONFIG_NUC970_ETIMER2_CAP_NONE)
				return -EPERM;
#endif
			} else { // ch 3
#if defined (CONFIG_NUC970_ETIMER3_CAP_PF14)
				s = pinctrl_lookup_state(t->pinctrl, "etimer3-cap-PF");
#elif defined (CONFIG_NUC970_ETIMER3_CAP_PC13)
				s = pinctrl_lookup_state(t->pinctrl, "etimer3-cap-PC");
#elif defined (CONFIG_NUC970_ETIMER3_CAP_NONE)
				return -EPERM;
#endif
			}

			if (IS_ERR(s)) {
				pr_debug("pinctrl_lookup_state err\n");
				return -EPERM;
			}
			if((ret = pinctrl_select_state(t->pinctrl, s)) < 0) {
				pr_debug("pinctrl_select_state err\n");
				return ret;
			}

			spin_lock_irqsave(&t->lock, flag);
			// timer clock is 12MHz, set prescaler to 12 - 1.
			__raw_writel(11, REG_ETMR_PRECNT(ch));
			__raw_writel(0xFFFFFF, REG_ETMR_CMPR(ch));
			// enable capture interrupt
			__raw_writel(ETIMER_IER_TCAP_IE, REG_ETMR_IER(ch));
			__raw_writel(param | ETIMER_TRIGGER_COUNTING, REG_ETMR_CTL(ch));
			t->mode = ETIMER_OPMODE_TRIGGER_COUNTING;
			spin_unlock_irqrestore(&t->lock, flag);
			break;

		default:
			return -ENOTTY;


	}
	return 0;
}

static unsigned int etimer_poll(struct file *filp, poll_table *wait)
{
	struct nuc970_etimer *t = (struct nuc970_etimer *)filp->private_data;
	unsigned int mask = 0;

	poll_wait(filp, &t->wq, wait);
	if(t->update)
		mask |= POLLIN | POLLRDNORM;
	return mask;
}

struct file_operations etimer_fops =
{
	.owner		= THIS_MODULE,
	.open		= etimer_open,
	.release	= etimer_release,
	.read		= etimer_read,
	.unlocked_ioctl	= etimer_ioctl,
	.poll		= etimer_poll,
};

static struct miscdevice etimer_dev[] = {
	[0] = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "etimer0",
		.fops = &etimer_fops,
	},
	[1] = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "etimer1",
		.fops = &etimer_fops,
	},
	[2] = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "etimer2",
		.fops = &etimer_fops,
	},
	[3] = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "etimer3",
		.fops = &etimer_fops,
	},
};

static int nuc970_etimer_probe(struct platform_device *pdev)
{
	int ch = pdev->id;


	etmr[ch] = devm_kzalloc(&pdev->dev, sizeof(struct nuc970_etimer), GFP_KERNEL);
	if (etmr[ch] == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory for etimer device\n");
		return -ENOMEM;
	}


	misc_register(&etimer_dev[ch]);

	etmr[ch]->pinctrl = devm_pinctrl_get(&pdev->dev);
	etmr[ch]->minor = MINOR(etimer_dev[ch].minor);
	etmr[ch]->ch = ch;
	spin_lock_init(&etmr[ch]->lock);

	etmr[ch]->irq = platform_get_irq(pdev, ch);

	init_waitqueue_head(&etmr[ch]->wq);


	platform_set_drvdata(pdev, etmr[ch]);


	return(0);
}

static int nuc970_etimer_remove(struct platform_device *pdev)
{
	//struct nuc970_etimer *etmr = platform_get_drvdata(pdev);
	int ch = pdev->id;

	misc_deregister(&etimer_dev[ch]);


	return 0;
}


#ifdef CONFIG_PM
static int nuc970_etimer_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int nuc970_etimer_resume(struct platform_device *pdev)
{

	return 0;
}

#else
#define nuc970_etimer_suspend 	NULL
#define nuc970_etimer_resume	NULL
#endif

static struct platform_driver nuc970_etimer_driver = {
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "nuc970-etimer",
	},
	.probe		= nuc970_etimer_probe,
	.remove		= nuc970_etimer_remove,
	.suspend	= nuc970_etimer_suspend,
	.resume		= nuc970_etimer_resume,
};


module_platform_driver(nuc970_etimer_driver);



MODULE_AUTHOR("Nuvoton Technology Corp.");
MODULE_ALIAS("platform:nuc970-etimer");
MODULE_LICENSE("GPL");
