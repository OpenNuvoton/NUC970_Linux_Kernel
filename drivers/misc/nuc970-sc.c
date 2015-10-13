/* linux/driver/misc/nuc970-sc.c
 *
 * Copyright (c) 2015 Nuvoton technology corporation
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
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <mach/map.h>
#include <mach/regs-gcr.h>
#include <mach/regs-clock.h>
#include <mach/regs-sc.h>
#include <mach/nuc970-sc.h>

extern int ATR_InitFromArray (ATR_t * atr, const char atr_buffer[ATR_MAX_SIZE], unsigned int length);
extern int ATR_CheckIntegrity(ATR_t * atr, int type);
extern int ATR_Parse(int intf);
extern void CmdXfrT0(unsigned int reader_index,
	unsigned int snd_len, unsigned char snd_buf[], /*unsigned int *rcv_len,*/
	unsigned char rcv_buf[]);
extern void CmdXfrT1(unsigned int reader_index,
	unsigned int snd_len, unsigned char snd_buf[], /*unsigned int *rcv_len,*/
	unsigned char rcv_buf[]);

extern int t1_init(unsigned int intf);
extern int t1_negotiate_ifsd(/*t1_state_t * t1*/unsigned int intf, unsigned int dad, int ifsd);

struct nuc970_sc sc[SC_INTF];
static int atr_remain_t, atr_remain, ifbyte_flag, tck; // variables for ATR processing


static void reset_reader(struct nuc970_sc *sc)
{
	// stop all timers
	__raw_writel((__raw_readl(sc->base + REG_SC_ALTCTL) & ~(SC_ALTCTL_CNTEN2 | SC_ALTCTL_CNTEN1 | SC_ALTCTL_CNTEN0)) |
			(SC_ALTCTL_TXRST | SC_ALTCTL_RXRST | SC_ALTCTL_ADACEN),
			sc->base + REG_SC_ALTCTL);

	// Set Rx trigger level to 1 character, longest card detect debounce period, disable error retry (EMV ATR does not use error retry)
	// Enable auto convention, and all three smartcard internal timers
	__raw_writel((__raw_readl(sc->base + REG_SC_CTL) & ~(SC_CTL_RXTRGLV | SC_CTL_TXRTY | SC_CTL_RXRTY)) | (SC_CTL_AUTOCEN | SC_CTL_TMRSEL),
			sc->base + REG_SC_CTL);
	__raw_writel(0, sc->base + REG_SC_TMRCTL0);
	__raw_writel(0, sc->base + REG_SC_TMRCTL1);
	__raw_writel(0, sc->base + REG_SC_TMRCTL2);
	// Disable Rx timeout
	__raw_writel(0, sc->base + REG_SC_RXTOUT);
	// 372 clocks per ETU by default
	__raw_writel(371, sc->base + REG_SC_ETUCTL);


    /* Enable necessary interrupt for smartcard operation */
    if(sc->ignorecd == 1) // Do not enable card detect interrupt if card present state ignore
	__raw_writel(SC_INTEN_RDAIEN |
                     SC_INTEN_TERRIEN |
                     SC_INTEN_TMR0IEN |
                     SC_INTEN_TMR1IEN |
                     SC_INTEN_TMR2IEN |
                     SC_INTEN_INITIEN |
                     SC_INTEN_ACERRIEN, sc->base + REG_SC_INTEN);
    else
    	__raw_writel(SC_INTEN_RDAIEN |
                     SC_INTEN_TERRIEN |
                     SC_INTEN_TMR0IEN |
                     SC_INTEN_TMR1IEN |
                     SC_INTEN_TMR2IEN |
                     SC_INTEN_CDIEN |
                     SC_INTEN_INITIEN |
                     SC_INTEN_ACERRIEN, sc->base + REG_SC_INTEN);
#ifdef CONFIG_EMV_CHECK
	__raw_writel(__raw_readl(sc->base + REG_SC_ALTCTL) | SC_ALTCTL_INITSEL, sc->base + REG_SC_ALTCTL);
#endif

    return;
}

static void config_reader(struct nuc970_sc *sc)
{
	int GT = 12;
	// Set ETU
	__raw_writel(sc->F / sc->D, sc->base + REG_SC_ETUCTL);
	// Calculate GT
	if(sc->N == 255) {
		if(sc->protocol == 1)
			GT = 11;
	} else if(sc->N != 0) {
		GT = sc->N + 12;
	}
	// Set EGT and BGT
	if(sc->protocol == 0) {
		__raw_writel((__raw_readl(sc->base + REG_SC_CTL) & ~(SC_CTL_NSB |SC_CTL_BGT)) | ((16 - 1) << 8),
			sc->base + REG_SC_CTL);
		__raw_writel(GT - 12, sc->base + REG_SC_EGT);
	} else {
		printk("set EGT to %d\n", GT - 11);
		__raw_writel((__raw_readl(sc->base + REG_SC_CTL) & ~SC_CTL_BGT) | SC_CTL_NSB | ((22 - 1) << 8),
			sc->base + REG_SC_CTL);
		__raw_writel(GT - 11, sc->base + REG_SC_EGT);
	}
	// Set retry count
	while(__raw_readl(sc->base + REG_SC_CTL) & SC_CTL_SYNC);
	if(sc->protocol == 0) { // retry 4 times max for T0
		printk("set retry count %d\n", __raw_readl(sc->base + REG_SC_ETUCTL));
		__raw_writel((__raw_readl(sc->base + REG_SC_CTL) & ~(SC_CTL_TXRTY | SC_CTL_RXRTY)) | SC_CTL_TXRTYEN | SC_CTL_RXRTYEN | 0x00330000,
			sc->base + REG_SC_CTL);
		__raw_writel(SC_ALTCTL_RXRST | SC_ALTCTL_TXRST, sc->base + REG_SC_ALTCTL);

	} else { // disable retry if T1
		__raw_writel(__raw_readl(sc->base + REG_SC_CTL) & ~(SC_CTL_TXRTYEN | SC_CTL_RXRTYEN),
			sc->base + REG_SC_CTL);

	}

}

// type 0: cold reset, 1: warm reset
int parse_atr(struct nuc970_sc *sc, int type)
{
	int i;

	//printk("parse\n");
	if(ATR_InitFromArray(&sc->atr, sc->atrbuf, sc->atrlen) != 0)
		return SC_ERR_ATR;
#ifdef CONFIG_EMV_CHECK
	if((i = ATR_CheckIntegrity(&sc->atr, type)) != 0)
		return i;
#endif
	if((i = ATR_Parse(sc->intf)) != 0)
		return i;

	config_reader(sc);


	return 0;

}


//XXX: make sure Tx/Rx pointer does not excees buffer size
static irqreturn_t nuc970_sc_interrupt(int irq, void *dev_id)
{
	struct nuc970_sc *sc = (struct nuc970_sc *)dev_id;

	u32 intsts = __raw_readl(sc->base + REG_SC_INTSTS);
	u32 status = __raw_readl(sc->base + REG_SC_STATUS);
	u32 inten = __raw_readl(sc->base + REG_SC_INTEN);

	// Activate, warm reset, deactivate
	if(intsts & SC_INTSTS_INITIF) {
		__raw_writel(SC_INTSTS_INITIF, sc->base + REG_SC_INTSTS);
		if(__raw_readl(sc->base + REG_SC_PINCTL) & SC_PINCTL_RSTSTS) {  // cold reset or warm reset
			//printk("activate done\n");
			sc->state = SC_OP_READ_ATR;
		} else { // deactivate
			sc->state = SC_OP_IDLE;
			sc->act = -1;
			//printk("deact done %d\n", sc->act);
		}

	}

	// Check CD event
	if(intsts & SC_INTSTS_CDIF) {

		// don't care about latest CD status. As long as remove event trigger, stop everything...
		if(status & SC_STATUS_CREMOVE) {
			//printk("SC%d: Card Removed\n", intf);
			/* Stop write operation */
			__raw_writel(__raw_readl(sc->base + REG_SC_INTEN) & ~SC_INTEN_TBEIEN, sc->base + REG_SC_INTEN);
			__raw_writel(SC_STATUS_CREMOVE, sc->base + REG_SC_STATUS);

			sc->err = SC_ERR_CARD_REMOVED;
			sc->state = SC_OP_IDLE;
			sc->act = -1;
			sc->atrlen = sc->protocol = 0;
			memset(&sc->T0, 0, sizeof(sc->T0));
			memset(&sc->T1, 0, sizeof(sc->T1));
			memset(&sc->T0_dat, 0, sizeof(sc->T0_dat));
			memset(&sc->T1_dat, 0, sizeof(sc->T1_dat));
			memset(&sc->atr, 0, sizeof(sc->atr));

			__raw_writel(__raw_readl(sc->base + REG_SC_ALTCTL) & ~(SC_ALTCTL_CNTEN2 | SC_ALTCTL_CNTEN1 | SC_ALTCTL_CNTEN0),
						sc->base + REG_SC_ALTCTL);
			__raw_writel(SC_ALTCTL_RXRST | SC_ALTCTL_TXRST, sc->base + REG_SC_ALTCTL);

		} else if(status & SC_STATUS_CINSERT) {
			__raw_writel(SC_STATUS_CINSERT, sc->base + REG_SC_STATUS);

		}
		// clear CD_IS bit
		__raw_writel(SC_INTSTS_CDIF, sc->base + REG_SC_INTSTS);

	}

	// Check Timeout event
	if(intsts & SC_INTSTS_TMR0IF) {
		__raw_writel(SC_INTSTS_TMR0IF, sc->base + REG_SC_INTSTS);
		sc->err = SC_ERR_TIME0OUT;
		sc->state = SC_OP_IDLE;
		__raw_writel(__raw_readl(sc->base + REG_SC_ALTCTL) & ~SC_ALTCTL_CNTEN0, sc->base + REG_SC_ALTCTL);

	}

	if(intsts & SC_INTSTS_TMR1IF) {
		__raw_writel(SC_INTSTS_TMR1IF, sc->base + REG_SC_INTSTS);
		sc->err = SC_ERR_TIME1OUT;
		sc->state = SC_OP_IDLE;
		__raw_writel(__raw_readl(sc->base + REG_SC_ALTCTL) & ~SC_ALTCTL_CNTEN1, sc->base + REG_SC_ALTCTL);
	}

	if(intsts & SC_INTSTS_TMR2IF) {
		__raw_writel(SC_INTSTS_TMR2IF, sc->base + REG_SC_INTSTS);

		if(atr_remain_t < 0) {
			__raw_writel(__raw_readl(sc->base + REG_SC_ALTCTL) & ~SC_ALTCTL_CNTEN2, sc->base + REG_SC_ALTCTL);
			sc->err = SC_ERR_TIME2OUT;
		} else {
			atr_remain_t -= 256;
		}

	}

	// Check Tx/Rx event
	/* transmit buffer empty interrupt */
	if(intsts & SC_INTSTS_TBEIF) {
		if( sc->state == SC_OP_WRITE ) {
			int len, i;
			// We can push 4 bytes into FIFO at most due to FIFO depth limitation
			if(sc->tcnt - sc->toffset > 3)
				len = 4;
			else
				len = sc->tcnt - sc->toffset;
			for(i = 0; i < len; i++) {
				__raw_writel(sc->tbuf[sc->toffset++], sc->base + REG_SC_DAT);
			}
			if(sc->toffset == sc->tcnt) {
				__raw_writel(__raw_readl(sc->base + REG_SC_INTEN) & ~SC_INTEN_TBEIEN, sc->base + REG_SC_INTEN);
				sc->state = SC_OP_IDLE;
			}
		}
	}

	/* RDR data ready or Rx time out*/
	if(intsts & (SC_INTSTS_RDAIF | SC_INTSTS_RXTOIF)) {

		if(sc->state == SC_OP_READ) {
			// [2011.11.25]
			/* EMV Certification */
			if(sc->protocol == SC_PROTOCOL_T1) {
				// ISO 7816-3 11.4.3, CWT = (11 + 2^CWI)ETU
				__raw_writel((sc->T1.CWT + 50) | SC_TMR_MODE_7, sc->base + REG_SC_TMRCTL0);
				__raw_writel(__raw_readl(sc->base + REG_SC_ALTCTL) | SC_ALTCTL_CNTEN0, sc->base + REG_SC_ALTCTL);
			}
			if(intsts & SC_INTSTS_RDAIF) {
				sc->rbuf[sc->rtail++] = __raw_readl(sc->base + REG_SC_DAT);

			}
		} else if(sc->state == SC_OP_READ_ATR) {  /* Read ATR ISR */
			int volatile ii;
			// stop checking timer & start to check waiting time 9600
			__raw_writel(__raw_readl(sc->base + REG_SC_ALTCTL) & ~SC_ALTCTL_CNTEN0, sc->base + REG_SC_ALTCTL);
			for(ii = 0; ii < 10; ii++);
			__raw_writel((9600+480) | SC_TMR_MODE_0, sc->base + REG_SC_TMRCTL0);
			__raw_writel(__raw_readl(sc->base + REG_SC_ALTCTL) | SC_ALTCTL_CNTEN0, sc->base + REG_SC_ALTCTL);

			if(sc->atrlen == 0) { // first byte received, start ticking...
				/* start counting total time for ATR session */
				__raw_writel(__raw_readl(sc->base + REG_SC_ALTCTL) & ~SC_ALTCTL_CNTEN2, sc->base + REG_SC_ALTCTL);
				for(ii = 0; ii < 10; ii++);
				__raw_writel(255 | SC_TMR_MODE_4, sc->base + REG_SC_TMRCTL2);
				__raw_writel(__raw_readl(sc->base + REG_SC_ALTCTL) | SC_ALTCTL_CNTEN2, sc->base + REG_SC_ALTCTL);

				atr_remain_t = 20050;
				atr_remain = 2;
				ifbyte_flag = -1;
				tck = 0;
			}

			if( (intsts & SC_INTSTS_RDAIF) && atr_remain) {
				/*
				* atr_len==0 : TS
				* atr_len==1 : T0
				*/
				sc->atrbuf[sc->atrlen] = __raw_readl(sc->base + REG_SC_DAT);

				atr_remain--;
				ifbyte_flag--;

				if(sc->atrlen == 1) {
					atr_remain += (sc->atrbuf[sc->atrlen] & 0xf); // Historical byte
					ifbyte_flag = 0; // T0 contains Y(x) as well\n
				}

				if( ifbyte_flag == 0 ) {
					if(sc->atrbuf[sc->atrlen] & 0x10) {
						++atr_remain;
						++ifbyte_flag;
					}
					if(sc->atrbuf[sc->atrlen] & 0x20) {
						++atr_remain;
						++ifbyte_flag;
					}
					if(sc->atrbuf[sc->atrlen] & 0x40) {
						++atr_remain;
						++ifbyte_flag;
					}
					if(sc->atrbuf[sc->atrlen] & 0x80) {
						++atr_remain;
						++ifbyte_flag;
						if((tck == 0) && (sc->atrlen != 1) && ((sc->atrbuf[sc->atrlen] & 0xf) != 0)) {
							++atr_remain; //tck exist
							tck = 1;
						}
					} else {
						/* Here, it's special case for APDU test card */
						if((tck == 0) && (sc->atrlen != 1) && ((sc->atrbuf[sc->atrlen] & 0xf) != 0)) {
							++atr_remain; //tck exist
							tck = 1;
						}
						ifbyte_flag = -1;
					}
				}

				sc->atrlen++;   /* increase the length of ATR */

			}

			if(atr_remain == 0) {   /* receive ATR done */
				// Stop timer
				__raw_writel(__raw_readl(sc->base + REG_SC_ALTCTL) & ~(SC_ALTCTL_CNTEN2 | SC_ALTCTL_CNTEN0),
						sc->base + REG_SC_ALTCTL);
				sc->state = SC_OP_IDLE;

			}
		} else {
			// also goes here if we cannot consume previously received data in time, not necessary a bug
			sc->rbuf[sc->rtail++] = __raw_readl(sc->base + REG_SC_DAT);
		}
	}
	// Check Transmit error
	if(intsts & SC_INTSTS_ACERRIF){
		__raw_writel(SC_INTSTS_ACERRIF, sc->base + REG_SC_INTSTS);
		sc->err = SC_ERR_AUTOCONVENTION;
		sc->state = SC_OP_IDLE;
	}

	/* Transmit Error: break error, frame error, Rx/Tx over flow, parity error, invalid stop */
	if((intsts & SC_INTSTS_TERRIF) && (inten & SC_INTEN_TERRIEN)) {
		if(status & SC_STATUS_RXOV) {
			__raw_writel(SC_STATUS_RXOV, sc->base + REG_SC_STATUS);
			sc->err = SC_ERR_READ;
			sc->state = SC_OP_IDLE;
		}

		if(status & SC_STATUS_TXOV) {
			__raw_writel(SC_STATUS_TXOV, sc->base + REG_SC_STATUS);
			sc->err = SC_ERR_WRITE;
			sc->state = SC_OP_IDLE;
		}

		if(status & (SC_STATUS_PEF | SC_STATUS_BEF | SC_STATUS_FEF)) {

			__raw_writel(SC_STATUS_PEF | SC_STATUS_BEF | SC_STATUS_FEF, sc->base + REG_SC_STATUS);
			sc->err = SC_ERR_PARITY;
			__raw_writel(SC_ALTCTL_RXRST, sc->base + REG_SC_ALTCTL);
			if(sc->protocol == SC_PROTOCOL_T0) {
				sc->state = SC_OP_IDLE;
			}

		}

		if(status & SC_STATUS_TXOVERR) {
			__raw_writel(__raw_readl(sc->base + REG_SC_INTEN) & ~SC_INTEN_TBEIEN, sc->base + REG_SC_INTEN);
			__raw_writel(SC_STATUS_TXRERR | SC_STATUS_TXOVERR, sc->base + REG_SC_STATUS);
			__raw_writel(SC_ALTCTL_TXRST, sc->base + REG_SC_ALTCTL);
			sc->err = SC_ERR_WRITE;
			sc->state = SC_OP_IDLE;
		}

		if(status & SC_STATUS_RXOVERR) {
			__raw_writel(SC_STATUS_RXRERR | SC_STATUS_RXOVERR, sc->base + REG_SC_STATUS);
			__raw_writel(SC_ALTCTL_RXRST, sc->base + REG_SC_ALTCTL);
			sc->err = SC_ERR_READ;
			sc->state = SC_OP_IDLE;
		}
	}
	wake_up_interruptible(&sc->wq);  // wake on any event
	return IRQ_HANDLED;
}

// This function does not block, transaction complete in write(), this API is only for user to read back card response.
static ssize_t sc_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{

	struct nuc970_sc *sc = (struct nuc970_sc *)filp->private_data;

	int ret = 0;

	if(unlikely((count == 0) || (sc->rhead == sc->rtail)))
		goto out;

	if(unlikely(count > MAX_LEN))
		count = MAX_LEN;

	if(count > sc->rtail - sc->rhead)
		count = sc->rtail - sc->rhead;


	mutex_lock(&sc->lock);
	if(sc->err != 0)
		ret = -EFAULT;
	else {
		if(copy_to_user(buf, &sc->rbuf[0], count))
			ret = -EFAULT;
		else {
			ret = count;
			sc->rhead += count;
		}
	}
	mutex_unlock(&sc->lock);
out:

	return ret;
}


static ssize_t sc_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{

	struct nuc970_sc *sc = (struct nuc970_sc *)filp->private_data;
	int intf = sc->intf;
	int ret = 0;

	if(unlikely(count == 0))
		goto out;
	if(unlikely(sc->act != 1))
		goto out;

	mutex_lock(&sc->lock);

	if(unlikely(count > MAX_LEN))
		sc->tcnt = MAX_LEN;
	else
		sc->tcnt = count;

	sc->toffset = 0;
	sc->err = 0;
	sc->rhead = sc->rtail = 0; // this is a new transaction, drop old data
	if (copy_from_user(sc->tbuf, buf, count)) {
		ret = -EFAULT;
		mutex_unlock(&sc->lock);
		goto out;
	}
	if(sc->protocol == 0) {
		CmdXfrT0(intf, sc->tcnt, sc->tbuf, sc->rbuf);
	} else {  //T1
		CmdXfrT1(intf, sc->tcnt, sc->tbuf, sc->rbuf);
	}

	if(sc->err != 0)
		ret = -EFAULT;
	else
		ret = sc->tcnt; // transfer complete...
	mutex_unlock(&sc->lock);
out:
	return ret;
}

static int sc_release(struct inode *inode, struct file *filp)
{
	struct nuc970_sc *sc = (struct nuc970_sc *)filp->private_data;
	// free irq
	free_irq(sc->irq, (void *)sc);

	// deactivate
	__raw_writel(SC_INTSTS_INITIF, sc->base + REG_SC_INTSTS);
	__raw_writel(__raw_readl(sc->base + REG_SC_ALTCTL) | SC_ALTCTL_DACTEN, sc->base + REG_SC_ALTCTL);
	while(!(__raw_readl(sc->base + REG_SC_INTSTS) & SC_INTSTS_INITIF));

	__raw_writel(0, sc->base + REG_SC_INTEN);
	__raw_writel(0, sc->base + REG_SC_ALTCTL);
	__raw_writel(0, sc->base + REG_SC_CTL);

	sc->atrlen = 0;
	sc->rtail = sc->rhead = 0;
	sc->act = 0;
	memset(&sc->T0, 0, sizeof(sc->T0));
	memset(&sc->T1, 0, sizeof(sc->T1));
	memset(&sc->T0_dat, 0, sizeof(sc->T0_dat));
	memset(&sc->T1_dat, 0, sizeof(sc->T1_dat));
	memset(&sc->atr, 0, sizeof(sc->atr));
	// disable clk
	clk_disable(sc->clk);
	clk_disable(sc->eclk);
	clk_put(sc->clk);
	clk_put(sc->eclk);

	filp->private_data = NULL;

	return(0);
}

static int sc_open(struct inode *inode, struct file *filp)
{

	int ret, intf;

	for(intf = 0; intf < SC_INTF; intf++)
		if(MINOR(inode->i_rdev) == sc[intf].minor) {
			break;
		}
	filp->private_data = (void *)&sc[intf];

	if(intf == 0) {
		sc[intf].clk = clk_get(NULL, "smc0");
		sc[intf].eclk = clk_get(NULL, "smc0_eclk");
	} else {
		sc[intf].clk = clk_get(NULL, "smc1");
		sc[intf].eclk = clk_get(NULL, "smc1_eclk");
	}


	if (IS_ERR(sc[intf].clk)) {
		printk("failed to get sc clock\n");
		ret = PTR_ERR(sc[intf].clk);
		goto out2;
	}
	if (IS_ERR(sc[intf].eclk)) {
		printk("failed to get sc eclock\n");
		ret = PTR_ERR(sc[intf].eclk);
		goto out2;
	}
	clk_prepare(sc[intf].clk);
	clk_enable(sc[intf].clk);
	clk_prepare(sc[intf].eclk);
	clk_enable(sc[intf].eclk);
	clk_set_rate(sc[intf].eclk, 4000000);	// Set SC clock to 4MHz

	if(intf == 0) {
#ifdef CONFIG_NUC970_SC0_PWRINV
	__raw_writel(__raw_readl(sc[intf].base + REG_SC_PINCTL) | SC_PINCTL_PWRINV, sc[intf].base + REG_SC_PINCTL);
#endif

#ifdef CONFIG_NUC970_SC0_CDLV_H
	__raw_writel(__raw_readl(sc[intf].base + REG_SC_CTL) | SC_CTL_CDLV, sc[intf].base + REG_SC_CTL);
#elif defined(CONFIG_NUC970_SC0_CD_IGNORE)
	sc[intf].ignorecd = 1;
#endif
	} else {
#ifdef CONFIG_NUC970_SC1_PWRINV
	__raw_writel(__raw_readl(sc[intf].base + REG_SC_PINCTL) | SC_PINCTL_PWRINV, sc[intf].base + REG_SC_PINCTL);
#endif

#ifdef CONFIG_NUC970_SC1_CDLV_H
	__raw_writel(__raw_readl(sc[intf].base + REG_SC_CTL) | SC_CTL_CDLV, sc[intf].base + REG_SC_CTL);
#elif defined(CONFIG_NUC970_SC1_CD_IGNORE)
	sc[intf].ignorecd = 1;
#endif
	}
	// enable SC engine
	__raw_writel(__raw_readl(sc[intf].base + REG_SC_CTL) | SC_CTL_SCEN, sc[intf].base + REG_SC_CTL);
	if (request_irq(sc[intf].irq, nuc970_sc_interrupt,
						0x0, "nuc970-sc", (void *)&sc[intf])) {
		printk("register irq failed %d\n", sc[intf].irq);
		ret = -EAGAIN;
		goto out1;
	}

	return 0;


out1:

	free_irq(sc[intf].irq, (void *)&sc[intf]);
out2:
	return ret;

}

static long sc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct nuc970_sc *sc = (struct nuc970_sc *)filp->private_data;
	int intf = sc->intf;
	struct sc_transact *sc_t;
	unsigned int param;


	switch(cmd) {
		case SC_IOC_ACTIVATE:
			reset_reader(sc);

			if(sc->act == 1) {

				sc->act = 0;
				sc->state = SC_OP_DEACTIVATE;
				__raw_writel(__raw_readl(sc->base + REG_SC_ALTCTL) | SC_ALTCTL_DACTEN, sc->base + REG_SC_ALTCTL);
				wait_event_interruptible(sc->wq, sc->state != SC_OP_DEACTIVATE);
				memset(&sc->T0, 0, sizeof(sc->T0));
				memset(&sc->T1, 0, sizeof(sc->T1));
				memset(&sc->T0_dat, 0, sizeof(sc->T0_dat));
				memset(&sc->T1_dat, 0, sizeof(sc->T1_dat));
				// Delay 10ms before we cold reset the card. ISO 7816-3 6.2.4
				schedule_timeout_interruptible(HZ/100);
			}
			sc->act = sc->atrlen = sc->rhead = sc->rtail = sc->err = 0;
			sc->state = SC_OP_COLD_RESET;
			sc->N = sc->D = sc->F = 0;
			__raw_writel(((42000/372) + 1) | SC_TMR_MODE_3, sc->base + REG_SC_TMRCTL0);
			__raw_writel(__raw_readl(sc->base + REG_SC_ALTCTL) | SC_ALTCTL_CNTEN0 | SC_ALTCTL_ACTEN, sc->base + REG_SC_ALTCTL);

			wait_event_interruptible(sc->wq, (sc->state != SC_OP_READ_ATR) && (sc->state != SC_OP_COLD_RESET));

			if(sc->err == 0)
				sc->err = parse_atr(sc, 0);

			if(sc->err == SC_ERR_PARAM) {

				sc->err = sc->protocol =  0;
				sc->atrlen = sc->act = 0;
				sc->state = SC_OP_WARM_RESET;
				sc->N = sc->D = sc->F = 0;
				reset_reader(sc);
				__raw_writel(((42000/372) + 1) | SC_TMR_MODE_3, sc->base + REG_SC_TMRCTL0);
				__raw_writel(__raw_readl(sc->base + REG_SC_ALTCTL) | SC_ALTCTL_CNTEN0 | SC_ALTCTL_WARSTEN, sc->base + REG_SC_ALTCTL);

				wait_event_interruptible(sc->wq, (sc->state != SC_OP_READ_ATR) && (sc->state != SC_OP_WARM_RESET));
				if(sc->err == 0)
					sc->err = parse_atr(sc, 1);
			}
			if(sc->err != 0) { // no matter what error, deactivate...
				schedule_timeout_interruptible(HZ/100);
				sc->act = 0;
				__raw_writel(__raw_readl(sc->base + REG_SC_ALTCTL) | SC_ALTCTL_DACTEN, sc->base + REG_SC_ALTCTL);
				wait_event_interruptible(sc->wq, (sc->act == -1) || (sc->err != 0));
				memset(&sc->T0, 0, sizeof(sc->T0));
				memset(&sc->T1, 0, sizeof(sc->T1));
				memset(&sc->T0_dat, 0, sizeof(sc->T0_dat));
				memset(&sc->T1_dat, 0, sizeof(sc->T1_dat));
				return -EFAULT;
			} else {
				sc->act = 1;
				return sc->atrlen;
			}
		case SC_IOC_READATR:
			if(sc->act != 1) {
				return -EFAULT;
			}
			if(copy_to_user((void *)arg, (const void *)sc->atrbuf, sc->atrlen))
				return -EFAULT;
			break;

		case SC_IOC_DEACTIVATE:
			if(__raw_readl(sc->base + REG_SC_PINCTL) & SC_PINCTL_RSTSTS) {
				// only deactivate if card is activated
				sc->state = SC_OP_DEACTIVATE;
				__raw_writel(__raw_readl(sc->base + REG_SC_ALTCTL) | SC_ALTCTL_DACTEN, sc->base + REG_SC_ALTCTL);
				wait_event_interruptible(sc->wq, sc->state != SC_OP_DEACTIVATE);
			}
			sc->atrlen = sc->protocol = 0;
			sc->rtail = sc->rhead = 0;
			sc->act = 0;
			sc->err = 0;
			memset(&sc->T0, 0, sizeof(sc->T0));
			memset(&sc->T1, 0, sizeof(sc->T1));
			memset(&sc->T0_dat, 0, sizeof(sc->T0_dat));
			memset(&sc->T1_dat, 0, sizeof(sc->T1_dat));
			memset(&sc->atr, 0, sizeof(sc->atr));
			break;

		case SC_IOC_GETSTATUS:
			if(sc->intf == 0) {
#if defined(CONFIG_NUC970_SC0_CD_IGNORE)
				if(sc->act == 1) {
					param = ICC_PRESENT_ACTIVE;
				} else {
					param =ICC_PRESENT_INACTIVE;
				}
#else
				int status = __raw_readl(sc->base + REG_SC_STATUS);
				int ctl = __raw_readl(sc->base + REG_SC_CTL);
				if(sc->act == 1) {
					param = ICC_PRESENT_ACTIVE;
				}else if(sc->err == SC_ERR_CARD_REMOVED) { // card once removed, maybe insert now, but, must report this event to user app.
					param = ICC_ABSENT;
					sc->err = 0;
				} else if(((status & SC_STATUS_CDPINSTS) >> 13) != ((ctl & SC_CTL_CDLV) >> 26)) { // card is currently removed
					param = ICC_ABSENT;
				} else {
					param =ICC_PRESENT_INACTIVE;
				}
#endif
			} else { // sc1
#if defined(CONFIG_NUC970_SC1_CD_IGNORE)
				if(sc->act == 1) {
					param = ICC_PRESENT_ACTIVE;
				} else {
					param =ICC_PRESENT_INACTIVE;
				}
#else
				int status = __raw_readl(sc->base + REG_SC_STATUS);
				int ctl = __raw_readl(sc->base + REG_SC_CTL);
				if(sc->act == 1) {
					param = ICC_PRESENT_ACTIVE;
				} else if(sc->err == SC_ERR_CARD_REMOVED) { // card once removed, maybe insert now, but, must report this event to user app.
					param = ICC_ABSENT;
					sc->err = 0;
				} else if(((status & SC_STATUS_CDPINSTS) >> 13) != ((ctl & SC_CTL_CDLV) >> 26)) { // card is currently removed
					param = ICC_ABSENT;
				} else {
					param =ICC_PRESENT_INACTIVE;
				}

#endif
			}

			if(copy_to_user((void *)arg, (const void *)&param, sizeof(unsigned int)))
				return -EFAULT;
			break;

		case SC_IOC_SETPARAM:
			if(sc->protocol == 1) {
				t1_init(sc->intf);
				t1_negotiate_ifsd(sc->intf, 0, 0xFE); // EMV 9.2.4.3, this must be 0xFE
			} else {// do nothing if T0
				//printk("this is a T0 card\n");
			}
			break;
		case SC_IOC_TRANSACT:
			sc_t = (struct sc_transact *)arg;

			if(unlikely(sc_t == NULL))
				return -EFAULT;
			if(unlikely(sc_t->tx_len == 0 || sc_t->rx_len == 0))
				return -EFAULT;
			if(unlikely(sc_t->tx_buf == NULL || sc_t->rx_buf == NULL))
				return -EFAULT;
			if(unlikely(sc->act != 1))
				return -EIO;

			// write
			mutex_lock(&sc->lock);

			if(unlikely(sc_t->tx_len > MAX_LEN))
				sc->tcnt = MAX_LEN;
			else
				sc->tcnt = sc_t->tx_len;

			sc->toffset = 0;
			sc->err = 0;
			sc->rhead = sc->rtail = 0; // this is a new transaction, drop old data
			if (copy_from_user(sc->tbuf, sc_t->tx_buf, sc_t->tx_len)) {
				mutex_unlock(&sc->lock);
				return -EFAULT;
			}
			if(sc->protocol == 0) {
				CmdXfrT0(intf, sc->tcnt, sc->tbuf, sc->rbuf);
			} else {  //T1
				CmdXfrT1(intf, sc->tcnt, sc->tbuf, sc->rbuf);
			}

			if(sc->err != 0) {
				mutex_unlock(&sc->lock);
				return -EFAULT;
			}

			//read
			sc_t->rx_len = sc->rtail - sc->rhead;

			if(copy_to_user(sc_t->rx_buf, &sc->rbuf[0], sc_t->rx_len)) {
				mutex_unlock(&sc->lock);
				return -EFAULT;
			}
			mutex_unlock(&sc->lock);

			break;
		default:
			return -ENOTTY;


	}
	return 0;
}


struct file_operations sc_fops =
{
	.owner		= THIS_MODULE,
	.open		= sc_open,
	.release	= sc_release,
	.read		= sc_read,
	.write		= sc_write,
	.unlocked_ioctl	= sc_ioctl,
};

static struct miscdevice sc_dev[] = {
	[0] = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "sc0",
		.fops = &sc_fops,
	},
	[1] = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "sc1",
		.fops = &sc_fops,
	},
};

static int nuc970_sc_probe(struct platform_device *pdev)
{
	int intf = pdev->id;
	struct resource *res;

	memset(&sc[intf], 0, sizeof(struct nuc970_sc));

	if(intf == 0) {
#ifdef CONFIG_NUC970_SC0
#if defined (CONFIG_NUC970_SC_PG)
		sc[intf].pinctrl = devm_pinctrl_get_select(&pdev->dev, "sc0-PG");
#elif defined (CONFIG_NUC970_SC_PI)
		sc[intf].pinctrl = devm_pinctrl_get_select(&pdev->dev, "sc0-PI");
#endif
		if(IS_ERR(sc[intf].pinctrl)) {
			dev_err(&pdev->dev, "Unable to reserve SC pin");
			return PTR_ERR(sc[intf].pinctrl);
		}
#endif
	} else {
#ifdef CONFIG_NUC970_SC1
		sc[intf].pinctrl = devm_pinctrl_get_select(&pdev->dev, "sc1");
		if(IS_ERR(sc[intf].pinctrl)) {
			dev_err(&pdev->dev, "Unable to reserve SC pin");
			return PTR_ERR(sc[intf].pinctrl);
		}
#endif
	}



	misc_register(&sc_dev[intf]);

	sc[intf].minor = MINOR(sc_dev[intf].minor);
	sc[intf].intf = intf;

	mutex_init(&sc[intf].lock);

	sc[intf].irq = platform_get_irq(pdev, 0);
	res = (void __iomem *)platform_get_resource(pdev, IORESOURCE_MEM, 0);
	sc[intf].res = request_mem_region(res->start, resource_size(res), pdev->name);
	sc[intf].base = ioremap(res->start, resource_size(res));
	if (sc[intf].base == NULL) {
		dev_err(&pdev->dev, "cannot request IO\n");
		return -ENXIO;
	}

	init_waitqueue_head(&sc[intf].wq);

	platform_set_drvdata(pdev, (void *)&sc[intf]);


	return 0;
}

static int nuc970_sc_remove(struct platform_device *pdev)
{
	int intf = pdev->id;

	misc_deregister(&sc_dev[intf]);

	iounmap(sc[intf].base);
	release_resource(sc[intf].res);

	return 0;
}


#ifdef CONFIG_PM
static int nuc970_sc_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int nuc970_sc_resume(struct platform_device *pdev)
{

	return 0;
}

#else
#define nuc970_sc_suspend 	NULL
#define nuc970_sc_resume	NULL
#endif

static struct platform_driver nuc970_sc_driver = {
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "nuc970-sc",
	},
	.probe		= nuc970_sc_probe,
	.remove		= nuc970_sc_remove,
	.suspend	= nuc970_sc_suspend,
	.resume		= nuc970_sc_resume,
};


module_platform_driver(nuc970_sc_driver);



MODULE_AUTHOR("Nuvoton Technology Corp.");
MODULE_ALIAS("platform:nuc970-sc");
MODULE_LICENSE("GPL");
