/* linux/driver/misc/sc-t0.c
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
// Based on:
/*
    commands.c: Commands sent to the card
    Copyright (C) 2003-2010   Ludovic Rousseau
    Copyright (C) 2005 Martin Paljak

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public License
	along with this library; if not, write to the Free Software Foundation,
	Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
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
#include <linux/wait.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <mach/map.h>
#include <mach/regs-gcr.h>
#include <mach/regs-clock.h>
#include <mach/regs-sc.h>
#include <mach/nuc970-sc.h>


extern struct nuc970_sc sc[SC_INTF];
static char gRx[SC_INTF][300];
static unsigned gRx_offset[SC_INTF];

/*****************************************************************************
 *
 *					T0CmdParsing
 *
 ****************************************************************************/
static int T0CmdParsing(unsigned char *cmd, unsigned int cmd_len, unsigned int *exp_len)
{
	int ret = 0;
	*exp_len = 0;

	if(cmd_len < 4)
		return -1;
	/* Ref: 7816-4 Annex A */
	switch (cmd_len)
	{
		case 4:	/* Case 1 */
			//printk("Case 1\n");
			*exp_len = 2; /* SW1 and SW2 only */
			break;

		case 5: /* Case 2 */
			if (cmd[4] != 0) {
				//printk("Case 2, recv %d\n", cmd[4] + 2);
				*exp_len = cmd[4] + 2;
			} else {
				//printk("Case 2, recv 258\n");
				*exp_len = 256 + 2;
			}
			break;

		default:
			if (cmd_len == (unsigned int)(cmd[4] + 5)) {/* Case 3 */
				//printk("Case 3\n");
				*exp_len = 2; /* SW1 and SW2 only */
			} else if (cmd_len == (unsigned int)(cmd[4] + 6)) {/* Case 4 */
				//printk("Case 3\n");
				*exp_len = 2; /* SW1 and SW2 only */
				ret = 4;
			}else
				return -1;	/* situation not supported */
			break;
	}

	return ret;
} /* T0CmdParsing */

unsigned char tmp_buf[512]; // move out from function. we have limited stack depth
/*****************************************************************************
 *
 *					T0ProcACK
 *
 ****************************************************************************/
static void T0ProcACK(unsigned int intf, unsigned int *in_len,
	unsigned int proc_len, int is_rcv)
{

	unsigned int ret_len;

	if (is_rcv == 1)
	{	/* Receiving mode */
		unsigned int remain_len;

		/* There is no data in our tmp_buf,
		* we have to read all data we needed */
		remain_len = proc_len;

		{
			ret_len = remain_len;

			sc[intf].rlen = ret_len;
			sc[intf].state = SC_OP_READ;
			wait_event_interruptible(sc[intf].wq, ((sc[intf].rtail - sc[intf].rhead) >= sc[intf].rlen) || sc[intf].err);
			if(sc[intf].err)
				return;
			sc[intf].state = SC_OP_IDLE;

			*in_len = ret_len;
			memcpy(&gRx[intf][gRx_offset[intf]], &sc[intf].rbuf[sc[intf].rhead], ret_len);
			gRx_offset[intf] += ret_len;
			sc[intf].rhead += ret_len;

		}

	}
	else
	{	/* Sending mode */

		sc[intf].tcnt += proc_len;
		sc[intf].state = SC_OP_WRITE;
		// enable tx interrupt
		__raw_writel(__raw_readl(sc[intf].base + REG_SC_INTEN) | SC_INTEN_TBEIEN, sc[intf].base + REG_SC_INTEN);
		wait_event_interruptible(sc[intf].wq, sc[intf].state == SC_OP_IDLE);
	}

	return;
} /* T0ProcACK */


/*****************************************************************************
 *
 *					T0ProcSW1
 *
 ****************************************************************************/
static void T0ProcSW1(unsigned int intf,
	unsigned char *rcv_buf,
	unsigned char *in_buf, unsigned int in_len)
{

	rcv_buf++;
	in_buf++;
	/* store the SW2 */
	if(1) {
		sc[intf].rlen = 1;
		sc[intf].state = SC_OP_READ;
		wait_event_interruptible(sc[intf].wq, ((sc[intf].rtail - sc[intf].rhead) >= sc[intf].rlen) || sc[intf].err);

		if(sc[intf].err) {
			__raw_writel(__raw_readl(sc[intf].base + REG_SC_ALTCTL) & ~SC_ALTCTL_CNTEN0, sc[intf].base + REG_SC_ALTCTL);
			return;
		}

		in_buf = &sc[intf].rbuf[sc[intf].rtail - 1];

	}
	// all done, stop timer.
	__raw_writel(__raw_readl(sc[intf].base + REG_SC_ALTCTL) & ~SC_ALTCTL_CNTEN0, sc[intf].base + REG_SC_ALTCTL);
	sc[intf].state = SC_OP_IDLE;

	gRx[intf][gRx_offset[intf]++] = sc[intf].rbuf[sc[intf].rtail - 2];
	gRx[intf][gRx_offset[intf]++] = sc[intf].rbuf[sc[intf].rtail - 1];
	memcpy(sc[intf].rbuf, &gRx[intf][0], gRx_offset[intf]);
	sc[intf].rtail = gRx_offset[intf];
	sc[intf].rhead = 0;

	return;
} /* T0ProcSW1 */

void CmdXfrT0(unsigned int intf,
	unsigned int snd_len, unsigned char snd_buf[],
	unsigned char rcv_buf[])
{
	int is_rcv;

	unsigned int exp_len, in_len;
	unsigned char ins, *in_buf = &sc[intf].rbuf[0];  // just give a address avoid un-init warning

	int return_value = 0;

	gRx_offset[intf] = 0;

	if(0){
		int ii;
		for(ii = 0; ii < snd_len; ii ++) {
			printk("%02x ", snd_buf[ii]);
			if(ii % 0x10 == 15)
				printk("\n");
		}
		printk("\n");
	}

	in_len = 0;
	return_value = T0CmdParsing(snd_buf, snd_len, &exp_len);

	if (return_value < 0)
	{
		sc[intf].err = SC_ERR_T0;

		return ;
	}

	if(return_value == 4) {  // case 4, no need to send Le
		snd_len--;
	}

	if (snd_len == 5 || snd_len == 4)
		is_rcv = 1;
	else
		is_rcv = 0;

	/* Command to send to the smart card (must be 5 bytes, from 7816 p.15) */
	if (snd_len == 4)
	{
		snd_len -= 4;
	}
	else
	{
		snd_len -= 5;
	}

	/* Make sure this is a valid command by checking the INS field */
	ins = snd_buf[1];
	if ((ins & 0xF0) == 0x60 ||	/* 7816-3 8.3.2 */
		(ins & 0xF0) == 0x90)
	{
		sc[intf].err = SC_ERR_T0;
		return;
	}

	__raw_writel((sc[intf].T0.WT) | SC_TMR_MODE_7, sc[intf].base + REG_SC_TMRCTL0);
	__raw_writel(__raw_readl(sc[intf].base + REG_SC_ALTCTL) | SC_ALTCTL_CNTEN0, sc[intf].base + REG_SC_ALTCTL);

	sc[intf].tcnt = 5;
	sc[intf].state = SC_OP_WRITE;
	// enable tx interrupt
	__raw_writel(__raw_readl(sc[intf].base + REG_SC_INTEN) | SC_INTEN_TBEIEN, sc[intf].base + REG_SC_INTEN);
	wait_event_interruptible(sc[intf].wq, sc[intf].state == SC_OP_IDLE);

	if (sc[intf].err)
		return;

	while (1)
	{
		if (in_len == 0)
		{
			in_len = 1;

			sc[intf].rlen = 1;
			sc[intf].state = SC_OP_READ;
			wait_event_interruptible(sc[intf].wq, ((sc[intf].rtail - sc[intf].rhead) >= sc[intf].rlen) || sc[intf].err);
			if(sc[intf].err)
			{
				__raw_writel(__raw_readl(sc[intf].base + REG_SC_ALTCTL) & ~SC_ALTCTL_CNTEN0, sc[intf].base + REG_SC_ALTCTL);
				return;
			}

			in_buf = &sc[intf].rbuf[sc[intf].rhead];
			in_len = return_value;
			sc[intf].rhead++;  // this byte is will be process immediately and not pass to user
		}

		/* Start to process the procedure bytes */
		if (*in_buf == 0x60)
		{
			in_len = 0;
			continue;
		}
		else if (*in_buf == ins || *in_buf == (ins ^ 0x01))
		{
			/* ACK => To transfer all remaining data bytes */
			in_len--;
			if (is_rcv) {
				T0ProcACK(intf, &in_len, exp_len - gRx_offset[intf], 1);
			} else {
				T0ProcACK(intf, &in_len, snd_len, 0);
				in_len = 0;
			}
			if ((gRx_offset[intf] == exp_len) || (sc[intf].err != 0)) {
				__raw_writel(__raw_readl(sc[intf].base + REG_SC_ALTCTL) & ~SC_ALTCTL_CNTEN0, sc[intf].base + REG_SC_ALTCTL);
				if(sc[intf].err == 0) {
					if ((gRx[intf][gRx_offset[intf] - 2] & 0xF0) != 0x60 && (gRx[intf][gRx_offset[intf] - 2] & 0xF0) != 0x90)
						sc[intf].err = SC_ERR_T0;
					else {
						sc[intf].rtail = gRx_offset[intf];
						sc[intf].rhead = 0;
						memcpy(sc[intf].rbuf, &gRx[intf][0], gRx_offset[intf]);
					}
				}
				return;
			}

			continue;
		}
		else if (*in_buf == (ins ^ 0xFF) || *in_buf == (ins ^ 0xFE))
		{
			if(!is_rcv)
				snd_len--;
			T0ProcACK(intf, &in_len, 1, is_rcv);
			if(sc[intf].err) {
				__raw_writel(__raw_readl(sc[intf].base + REG_SC_ALTCTL) & ~SC_ALTCTL_CNTEN0, sc[intf].base + REG_SC_ALTCTL);
				return;
			}
			in_len = 0;
			continue;
		} else if ((*in_buf & 0xF0) == 0x60 || (*in_buf & 0xF0) == 0x90) {
			/* SW1 */
			return T0ProcSW1(intf, rcv_buf, in_buf, in_len);
		} else {
			sc[intf].err = SC_ERR_T0; // protocol error
			/* Error, unrecognized situation found */
			__raw_writel(__raw_readl(sc[intf].base + REG_SC_ALTCTL) & ~SC_ALTCTL_CNTEN0, sc[intf].base + REG_SC_ALTCTL);

			return;
		}
	}

	return;
} /* CmdXfrT0 */
