/* linux/driver/misc/sc-t1.c
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
// Original from
/*
 * Implementation of T=1
 *
 * Copyright (C) 2003, Olaf Kirch <okir@suse.de>
 *
 * improvements by:
 * Copyright (C) 2004 Ludovic Rousseau <ludovic.rousseau@free.fr>
 */
/*
 * Buffer handling functions
 *
 * Copyright (C) 2003, Olaf Kirch <okir@suse.de>
 */
 /*
 * Checksum handling
 *
 * Copyright Matthias Bruestle 1999-2002
 * For licensing, see the file LICENCE
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

/* T=1 protocol constants */
#define T1_I_BLOCK		0x00
#define T1_R_BLOCK		0x80
#define T1_S_BLOCK		0xC0
#define T1_MORE_BLOCKS		0x20

enum {
	IFD_PROTOCOL_RECV_TIMEOUT = 0x0000,
	IFD_PROTOCOL_T1_BLOCKSIZE,
	IFD_PROTOCOL_T1_CHECKSUM_CRC,
	IFD_PROTOCOL_T1_CHECKSUM_LRC,
	IFD_PROTOCOL_T1_IFSC,
	IFD_PROTOCOL_T1_IFSD,
	IFD_PROTOCOL_T1_STATE,
	IFD_PROTOCOL_T1_MORE
};

#define T1_BUFFER_SIZE		(3 + 254 + 2)

static char gTx[SC_INTF][300];
static char gRx[SC_INTF][300];

#define T1_I_SEQ_SHIFT		6

#define T1_IS_ERROR(pcb)	((pcb) & 0x0F)
#define T1_EDC_ERROR		0x01
#define T1_OTHER_ERROR		0x02
#define T1_R_SEQ_SHIFT		4

#define T1_S_TYPE(pcb)		((pcb) & 0x0F)
#define T1_S_RESPONSE		0x20
#define T1_S_RESYNC		0x00
#define T1_S_IFS		0x01
#define T1_S_ABORT		0x02
#define T1_S_WTX		0x03
#define T1_S_IS_RESPONSE	((pcb) & T1_S_RESPONSE)

#define NAD 0
#define PCB 1
#define LEN 2
#define DATA 3

#define swap_nibbles(x)	((x >> 4) | ((x & 0xF) << 4))

/* see /usr/include/PCSC/ifdhandler.h for other values
 * this one is for internal use only */
#define IFD_PARITY_ERROR 699

typedef struct {
//	int		lun;
	int		state;

	unsigned char	ns;	/* reader side */
	unsigned char	nr;	/* card side */
	unsigned int	ifsc;
	unsigned int	ifsd;

	unsigned char	wtx;
	unsigned int	retries;
	unsigned int	rc_bytes;
	unsigned int 	icc_has_more;

	unsigned int	(*checksum)(const uint8_t *, size_t, unsigned char *);

	char			more;	/* more data bit */
	unsigned char	previous_block[4];	/* to store the last R-block */
	unsigned char	previous_i_block[MAX_LEN];	/* to store the last I-block */
} t1_state_t;

t1_state_t t1_state[SC_INTF];

/* ISO STD 3309 */
/* From: medin@catbyte.b30.ingr.com (Dave Medin)
 * Subject: CCITT checksums
 * Newsgroups: sci.electronics
 * Date: Mon, 7 Dec 1992 17:33:39 GMT
 */

/* Correct Table? */

static unsigned short crctab[256] = {
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/*
 * Returns LRC of data.
 */
unsigned int
csum_lrc_compute(const uint8_t *in, size_t len, unsigned char *rc)
{
	unsigned char	lrc = 0;

	while (len--)
		lrc ^= *in++;

	if (rc)
		*rc = lrc;
	return 1;
}

/*
 * Compute CRC of data.
 */
unsigned int
csum_crc_compute(const uint8_t * data, size_t len, unsigned char *rc)
{
	unsigned short v = 0xFFFF;

	while (len--) {
		v = ((v >> 8) & 0xFF) ^ crctab[(v ^ *data++) & 0xFF];
	}

	if (rc) {
		rc[0] = (v >> 8) & 0xFF;
		rc[1] = v & 0xFF;
	}

	return 2;
}

typedef struct ct_buf {
	unsigned char *base;
	unsigned int head, tail, size;
	unsigned int overrun;
} ct_buf_t;

void
ct_buf_init(ct_buf_t *bp, void *mem, size_t len)
{
	memset(bp, 0, sizeof(*bp));
	bp->base = (unsigned char *) mem;
	bp->size = len;
}

void
ct_buf_set(ct_buf_t *bp, void *mem, size_t len)
{
	ct_buf_init(bp, mem, len);
	bp->tail = len;
}

int
ct_buf_get(ct_buf_t *bp, void *mem, size_t len)
{
	if (len > bp->tail - bp->head)
		return -1;
	if (mem)
		memcpy(mem, bp->base + bp->head, len);
	bp->head += len;
	return len;
}

int
ct_buf_put(ct_buf_t *bp, const void *mem, size_t len)
{
	if (len > bp->size - bp->tail) {
		bp->overrun = 1;
		return -1;
	}
	if (mem)
		memcpy(bp->base + bp->tail, mem, len);
	bp->tail += len;
	return len;
}

int
ct_buf_putc(ct_buf_t *bp, int byte)
{
	unsigned char	c = byte;

	return ct_buf_put(bp, &c, 1);
}

unsigned int
ct_buf_avail(ct_buf_t *bp)
{
	return bp->tail - bp->head;
}

void *
ct_buf_head(ct_buf_t *bp)
{
	return bp->base + bp->head;
}



/* internal state, do not mess with it. */
/* should be != DEAD after reset/init */
enum {
	SENDING, RECEIVING, RESYNCH, DEAD
};

static void t1_set_checksum(t1_state_t *, int);
static unsigned int t1_block_type(unsigned char);
static unsigned int t1_seq(unsigned char);
static unsigned int t1_rebuild(unsigned int intf, unsigned char *block);
static unsigned int t1_compute_checksum(t1_state_t *, unsigned char *, size_t);
static int t1_verify_checksum(t1_state_t *, unsigned char *, size_t);
static int t1_xcv(unsigned int, unsigned char *, size_t, size_t);
unsigned int t1_build(unsigned int intf, unsigned char *block,
	unsigned char dad, unsigned char pcb,
	ct_buf_t *bp, size_t *lenp);
/*
 * Set default T=1 protocol parameters
 */
static void t1_set_defaults(t1_state_t * t1)
{
	t1->retries = 3;
	/* This timeout is rather insane, but we need this right now
	 * to support cryptoflex keygen */
	t1->ifsc = 32;
	t1->ifsd = 32;
	t1->nr = 0;
	t1->ns = 0;
	t1->wtx = 0;
}

static void t1_set_checksum(t1_state_t * t1, int csum)
{
	switch (csum) {
	case IFD_PROTOCOL_T1_CHECKSUM_LRC:
		t1->rc_bytes = 1;
		t1->checksum = csum_lrc_compute;
		break;
	case IFD_PROTOCOL_T1_CHECKSUM_CRC:
		t1->rc_bytes = 2;
		t1->checksum = csum_crc_compute;
		break;
	}
}

/*
 * Get/set parmaters for T1 protocol
 */
int t1_set_param(t1_state_t * t1, int type, long value)
{
	switch (type) {
	case IFD_PROTOCOL_T1_CHECKSUM_LRC:
	case IFD_PROTOCOL_T1_CHECKSUM_CRC:
		t1_set_checksum(t1, type);
		break;
	case IFD_PROTOCOL_T1_IFSC:
		t1->ifsc = value;
		break;
	case IFD_PROTOCOL_T1_IFSD:
		t1->ifsd = value;
		break;
	case IFD_PROTOCOL_T1_STATE:
		t1->state = value;
		break;
	case IFD_PROTOCOL_T1_MORE:
		t1->more = value;
		break;
	default:
		//printk("Unsupported parameter %d", type);
		return -1;
	}

	return 0;
}


/*
 * Attach t1 protocol
 */
int t1_init(unsigned int intf)
{
	t1_state_t * t1 = &t1_state[intf];
	t1_set_defaults(t1);
	t1_set_param(t1, IFD_PROTOCOL_T1_CHECKSUM_LRC, 0);
	t1_set_param(t1, IFD_PROTOCOL_T1_STATE, SENDING);
	t1_set_param(t1, IFD_PROTOCOL_T1_MORE, 0);
	return 0;
}


/*
 * Send an APDU through T=1
 */
unsigned char sdata[T1_BUFFER_SIZE];
int t1_transceive(unsigned int intf, unsigned int dad,
		const void *snd_buf, size_t snd_len,
		void *rcv_buf)
{
	ct_buf_t sbuf, rbuf, tbuf;
	unsigned char sblk[5];
	unsigned int slen, retries, resyncs;
	size_t last_send = 0;
	t1_state_t * t1 = &t1_state[intf];

	if (snd_len == 0)
		return -1;

	/* we can't talk to a dead card / reader. Reset it! */
	if (t1->state == DEAD)
	{
		printk("T=1 state machine is DEAD. Reset the card first.\n");
		return -1;
	}

	t1->state = SENDING;
	retries = t1->retries;
	resyncs = 3;

	t1->previous_block[0] = t1->previous_block[1] = t1->previous_block[2] = t1->previous_block[3] = 0;
	t1->icc_has_more = 0;
	/* Initialize send/recv buffer */
	ct_buf_set(&sbuf, (void *)snd_buf, snd_len);
	ct_buf_init(&rbuf, rcv_buf, T1_BUFFER_SIZE);

	/* Send the first block */
	slen = t1_build(intf, sdata, dad, T1_I_BLOCK, &sbuf, &last_send);

	while (1) {
		unsigned char pcb;
		int n;


		retries--;

		n = t1_xcv(intf, sdata, slen, sizeof(sdata));

		if (-2 == n || sc[intf].err == SC_ERR_PARITY || sc[intf].err == SC_ERR_CARD_REMOVED)
		{
			printk("Parity error\n");

			/* ISO 7816-3 Rule 7.4.2 */
			if (retries <= 0)
				goto resync;

			sc[intf].err = 0;

			if(t1->icc_has_more)
				slen = t1_build(intf, sdata,
					dad, T1_R_BLOCK,
					NULL, NULL);
			else
				slen = t1_build(intf, sdata,
					dad, T1_R_BLOCK | T1_EDC_ERROR,
					NULL, NULL);
			continue;
		}


		if ((n < 0)) {
			int err;

			if(t1_block_type(t1->previous_block[PCB]) == T1_S_BLOCK) {
				if(T1_S_TYPE(t1->previous_block[PCB]) == T1_S_ABORT)
					goto error;

			}

			if (retries <= 0)
				goto resync;

			if(t1->icc_has_more)
				slen = t1_build(intf, sdata,
					dad, T1_R_BLOCK,
					NULL, NULL);
			else
				slen = t1_build(intf, sdata,
					dad, T1_R_BLOCK | T1_OTHER_ERROR,
					NULL, NULL);
			err = sc[intf].err;
			sc[intf].err = 0;

			continue;

		}

		if (!t1_verify_checksum(t1, sdata, n)) {

			/* ISO 7816-3 Rule 7.4.2 */
			if (retries <= 0)
				goto resync;

			if(t1->icc_has_more)
				slen = t1_build(intf, sdata,
					dad, T1_R_BLOCK,
					NULL, NULL);
			else
				slen = t1_build(intf, sdata,
					dad, T1_R_BLOCK | T1_EDC_ERROR,
					NULL, NULL);
			continue;
		}


		if ((sdata[NAD] != swap_nibbles(dad)) /* wrong NAD */
			|| (sdata[LEN] == 0xFF))	/* length == 0xFF (illegal) */
		{

			/* ISO 7816-3 Rule 7.4.2 */
			if (retries <= 0)
				goto resync;

			if(t1->icc_has_more)
				slen = t1_build(intf, sdata,
					dad, T1_R_BLOCK,
					NULL, NULL);
			else
				slen = t1_build(intf, sdata,
					dad, T1_R_BLOCK | T1_OTHER_ERROR,
					NULL, NULL);
			continue;
		}

		pcb = sdata[PCB];
		switch (t1_block_type(pcb)) {
		case T1_R_BLOCK:
			if ((sdata[LEN] != 0x00)	/* length != 0x00 (illegal) */
				|| (pcb & 0x20)			/* b6 of pcb is set */
			   )
			{
				/* ISO 7816-3 Rule 7.4.2 */
				if (retries <= 0)
					goto resync;

				if(t1->icc_has_more)
					slen = t1_build(intf, sdata,
						dad, T1_R_BLOCK,
						NULL, NULL);
				else
					slen = t1_build(intf, sdata,
							dad, T1_R_BLOCK | T1_OTHER_ERROR,
							NULL, NULL);
				continue;
			}

			if (((t1_seq(pcb) != t1->ns)	/* wrong sequence number & no bit more */
					&& ! t1->more)
			   )
			{

				/* ISO 7816-3 Rule 7.4.2 */
				if (retries <= 0)
					goto resync;


				if(t1->icc_has_more)
					slen = t1_build(intf, sdata,
						dad, T1_R_BLOCK,
						NULL, NULL);
				else
					slen = t1_build(intf, sdata,
							dad, T1_R_BLOCK | T1_OTHER_ERROR,
							NULL, NULL);
				continue;
			}

			if (t1->state == RECEIVING) {
				/* ISO 7816-3 Rule 7.2 */
				if (T1_R_BLOCK == t1_block_type(t1->previous_block[1]))
				{
#ifndef CONFIG_EMV_CHECK
					/* ISO 7816-3 Rule 7.4.2 */
					if (retries <= 0)
						goto resync;
#endif
					//printk("Rule 7.2 f\n");
					slen = t1_rebuild(intf, sdata);
					continue;
				}

				slen = t1_build(intf, sdata,
						dad, T1_R_BLOCK,
						NULL, NULL);
				break;
			}

			/* If the card terminal requests the next
			 * sequence number, it received the previous
			 * block successfully */
			if (t1_seq(pcb) != t1->ns) {
				ct_buf_get(&sbuf, NULL, last_send);
				last_send = 0;
				t1->ns ^= 1;
			}

#ifdef CONFIG_EMV_CHECK
			/* If there's no data available, the ICC
			 * shouldn't be asking for more */
			if (ct_buf_avail(&sbuf) == 0)
				goto resync;
#endif
			if(T1_IS_ERROR(pcb)) {// this R block indicates an error
				slen = t1_build(intf, sdata, dad, T1_I_BLOCK,
						&sbuf, &last_send);

				if (retries <= 0)
					goto resync;

				continue;

			} else {
				slen = t1_build(intf, sdata, dad, T1_I_BLOCK,
						&sbuf, &last_send);

			}
			break;

		case T1_I_BLOCK:
			/* The first I-block sent by the ICC indicates
			 * the last block we sent was received successfully. */
			if (t1->state == SENDING) {
				ct_buf_get(&sbuf, NULL, last_send);
				last_send = 0;
				t1->ns ^= 1;
			}


			if(t1->more) {  // while we're still sending chaining I blocks shouldn't receive I block from ICC

				slen = t1_build(intf, sdata, dad,
						T1_R_BLOCK | T1_OTHER_ERROR,
						NULL, NULL);
				continue;

			}

			t1->state = RECEIVING;

			/* If the block sent by the card doesn't match
			 * what we expected it to send, reply with
			 * an R block */
			if (t1_seq(pcb) != t1->nr) {
				/* ISO 7816-3 Rule 7.4.2 */
				if (retries <= 0)
					goto resync;
				if(t1->icc_has_more)
					slen = t1_build(intf, sdata,
						dad, T1_R_BLOCK,
						NULL, NULL);
				else
					slen = t1_build(intf, sdata, dad,
							T1_R_BLOCK | T1_OTHER_ERROR,
							NULL, NULL);
				continue;
			}




			t1->nr ^= 1;

			if (ct_buf_put(&rbuf, sdata + 3, sdata[LEN]) < 0)
			{
				goto error;
			}

			if ((pcb & T1_MORE_BLOCKS) == 0) {
				goto done;
			}
			// clear previous error stored in previous_block if any
			if(T1_IS_ERROR(t1->previous_block[PCB]) != 0 && t1_block_type(t1->previous_block[PCB]) ==  T1_R_BLOCK) { // use first error code
				t1->previous_block[PCB] = 0;

			}
			t1->icc_has_more = 1;
			slen = t1_build(intf, sdata, dad, T1_R_BLOCK, NULL, NULL);
			break;

		case T1_S_BLOCK:
			if (/*T1_S_IS_RESPONSE(pcb)*/(pcb & 0x20) && t1->state == RESYNCH) {
				/* ISO 7816-3 Rule 6.2 */
				if(T1_S_TYPE(pcb) != T1_S_RESYNC) {
					goto resync;
				}

				/* ISO 7816-3 Rule 6.3 */
				t1->state = SENDING;
				last_send = 0;
				resyncs = 3;
				retries = t1->retries;
				ct_buf_init(&rbuf, rcv_buf, T1_BUFFER_SIZE);
				sbuf.head = 0;//==>
				slen = t1_build(intf, sdata, dad, T1_I_BLOCK,
						&sbuf, &last_send);

				continue;
			}

			if (/*T1_S_IS_RESPONSE(pcb)*/ (pcb & 0x20))
			{
				/* ISO 7816-3 Rule 7.4.2 */
				if (retries <= 0)
					goto resync;

				if(t1->icc_has_more)
					slen = t1_build(intf, sdata,
						dad, T1_R_BLOCK,
						NULL, NULL);
				else
					slen = t1_build(intf, sdata,
							dad, T1_R_BLOCK | T1_OTHER_ERROR,
							NULL, NULL);
				continue;
			}

			ct_buf_init(&tbuf, sblk, sizeof(sblk));

			switch (T1_S_TYPE(pcb)) {
			case T1_S_RESYNC:
				if (sdata[LEN] != 0)
				{

					slen = t1_build(intf, sdata, dad,
						T1_R_BLOCK | T1_OTHER_ERROR,
						NULL, NULL);
					continue;
				}

				/* the card is not allowed to send a resync. */
				goto resync;

			case T1_S_ABORT:
				if (sdata[LEN] != 0)
				{
					slen = t1_build(intf, sdata, dad,
						T1_R_BLOCK | T1_OTHER_ERROR,
						NULL, NULL);
					continue;
				}

				break;

			case T1_S_IFS:
				if (sdata[LEN] != 1)
				{

					slen = t1_build(intf, sdata, dad,
						T1_R_BLOCK | T1_OTHER_ERROR,
						NULL, NULL);
					continue;
				}
				if(sdata[DATA] < 0x10 || sdata[DATA] == 0xFF) {

					if (retries <= 0)
						goto resync;

					if(t1->icc_has_more)
						slen = t1_build(intf, sdata,
							dad, T1_R_BLOCK,
							NULL, NULL);
					else
						slen = t1_build(intf, sdata, dad,
							T1_R_BLOCK | T1_OTHER_ERROR,
							NULL, NULL);
					continue;
				} //else
					//printk("CT sent S-block with ifs=%u\n", sdata[DATA]);
				if (sdata[DATA] == 0)
					goto resync;
				t1->ifsc = sdata[DATA];
				ct_buf_putc(&tbuf, sdata[DATA]);
				break;

			case T1_S_WTX:
				if (sdata[LEN] != 1)
				{
					if (retries <= 0)
						goto resync;


					if(t1->icc_has_more)
						slen = t1_build(intf, sdata,
							dad, T1_R_BLOCK,
							NULL, NULL);
					else
						slen = t1_build(intf, sdata, dad,
							T1_R_BLOCK | T1_OTHER_ERROR,
							NULL, NULL);
					continue;
				}

				t1->wtx = sdata[DATA];
				sc[intf].T1.WTX = sdata[DATA];
				ct_buf_putc(&tbuf, sdata[DATA]);
				break;

			default:

				if (retries <= 0)
					goto resync;
				if(t1->icc_has_more)
					slen = t1_build(intf, sdata,
						dad, T1_R_BLOCK,
						NULL, NULL);
				else
					slen = t1_build(intf, sdata, dad,
						T1_R_BLOCK | T1_OTHER_ERROR,
						NULL, NULL);
				continue;
				//goto resync;
			}

			slen = t1_build(intf, sdata, dad,
				T1_S_BLOCK | T1_S_RESPONSE | T1_S_TYPE(pcb),
				&tbuf, NULL);
		}

		/* Everything went just splendid */
		retries = t1->retries;
		continue;

resync:
		/* the number or resyncs is limited, too */
		/* ISO 7816-3 Rule 6.4 */
		if (resyncs == 0)
			goto error;

		/* ISO 7816-3 Rule 6 */
		resyncs--;
		t1->ns = 0;
		t1->nr = 0;
		sc[intf].err = 0;
		slen = t1_build(intf, sdata, dad, T1_S_BLOCK | T1_S_RESYNC, NULL,
				NULL);
		t1->state = RESYNCH;
		t1->more = 0;
		retries = 1;
		continue;
	}

done:
	return ct_buf_avail(&rbuf);

error:
	t1->state = DEAD;
	return -1;
}

static unsigned t1_block_type(unsigned char pcb)
{
	switch (pcb & 0xC0) {
	case T1_R_BLOCK:
		return T1_R_BLOCK;
	case T1_S_BLOCK:
		return T1_S_BLOCK;
	default:
		return T1_I_BLOCK;
	}
}

static unsigned int t1_seq(unsigned char pcb)
{
	switch (pcb & 0xC0) {
	case T1_R_BLOCK:
		return (pcb >> T1_R_SEQ_SHIFT) & 1;
	case T1_S_BLOCK:
		return 0;
	default:
		return (pcb >> T1_I_SEQ_SHIFT) & 1;
	}
}

unsigned int t1_build(unsigned int intf, unsigned char *block,
	unsigned char dad, unsigned char pcb,
	ct_buf_t *bp, size_t *lenp)
{
	unsigned int len;
	char more = 0;
	t1_state_t * t1 = &t1_state[intf];
	len = bp ? ct_buf_avail(bp) : 0;

	if (len > t1->ifsc) {
		pcb |= T1_MORE_BLOCKS;
		len = t1->ifsc;
		more = 1;
	}


	/* Add the sequence number */
	switch (t1_block_type(pcb)) {
	case T1_R_BLOCK:
		if(T1_IS_ERROR(t1->previous_block[PCB]) != 0 && t1_block_type(t1->previous_block[PCB]) ==  T1_R_BLOCK) { // use first error code
			pcb = T1_R_BLOCK | T1_IS_ERROR(t1->previous_block[PCB]);
		}
		pcb |= t1->nr << T1_R_SEQ_SHIFT;
		break;
	case T1_I_BLOCK:
		pcb |= t1->ns << T1_I_SEQ_SHIFT;
		t1->more = more;
		break;
	}

	block[0] = dad;
	block[1] = pcb;
	block[2] = len;

	if (len)
		memcpy(block + 3, ct_buf_head(bp), len);
	if (lenp)
		*lenp = len;

	len = t1_compute_checksum(t1, block, len + 3);

	/* memorize the last sent block */
	/* only 4 bytes since we are only interesed in R-blocks */  //==> this is probably wrong, and makes erro handling difficult
	memcpy(t1->previous_block, block, 4);
	if(t1_block_type(pcb) == T1_I_BLOCK) {
		memcpy(t1->previous_i_block, block, len + 4);
	}
	return len;
}

static unsigned int
t1_rebuild(unsigned int intf, unsigned char *block)
{
	t1_state_t * t1 = &t1_state[intf];
	unsigned char pcb = t1 -> previous_block[1];
	unsigned char pcb1 = t1 -> previous_i_block[1];

	if(T1_I_BLOCK == t1_block_type(pcb1)) {
		if(t1_seq(pcb1) == t1_seq(sdata[PCB]) /*&& (T1_R_BLOCK == t1_block_type(sdata[PCB]))*/) {
			memcpy(block, t1 -> previous_i_block, t1 -> previous_i_block[2] + 4);
			return (t1 -> previous_i_block[2] + 4);
		} else {
			memcpy(block, t1 -> previous_block, 4);
			return 4;
		}
	} else
	/* copy the last sent block */
	if (T1_R_BLOCK == t1_block_type(pcb)) {
		memcpy(block, t1 -> previous_block, 4);
		return 4;
	} else
	{
		return 0;
	}
}

/*
 * Build/verify checksum
 */
static unsigned int t1_compute_checksum(t1_state_t * t1,
	unsigned char *data, size_t len)
{
	return len + t1->checksum(data, len, data + len);
}

static int t1_verify_checksum(t1_state_t * t1, unsigned char *rbuf,
	size_t len)
{
	unsigned char csum[2];
	int m, n;

	m = len - t1->rc_bytes;
	n = t1->rc_bytes;

	if (m < 0)
		return 0;

	t1->checksum(rbuf, m, csum);
	if (!memcmp(rbuf + m, csum, n))
		return 1;

	return 0;
}

/*
 * Send/receive block
 */
static int t1_xcv(unsigned int intf, unsigned char *block, size_t slen,
	size_t rmax)
{
#if 0
	printk("TX:\n");
	{
		int i;
		for(i = 0; i < slen; i++)
			printk("%02x ", block[i]);
		printk("\n");
	}
#endif
	sc[intf].rhead = sc[intf].rtail = 0;
	sc[intf].state = SC_OP_WRITE;
	memcpy(sc[intf].tbuf, block, slen);
	sc[intf].toffset = 0;
	sc[intf].tcnt = slen;


	if(!sc[intf].T1.WTX)  {
		__raw_writel((sc[intf].T1.BWT + 2400 * sc[intf].D) | SC_TMR_MODE_7, sc[intf].base + REG_SC_TMRCTL0);
	} else {
		__raw_writel((sc[intf].T1.BWT * sc[intf].T1.WTX + 2400 * sc[intf].D) | SC_TMR_MODE_7, sc[intf].base + REG_SC_TMRCTL0);
	}
	__raw_writel(__raw_readl(sc[intf].base + REG_SC_ALTCTL) | SC_ALTCTL_CNTEN0, sc[intf].base + REG_SC_ALTCTL);
	sc[intf].T1.WTX = 0;  // WTX only valid for 1 packet.


	// enable tx interrupt
	__raw_writel(__raw_readl(sc[intf].base + REG_SC_INTEN) | SC_INTEN_TBEIEN, sc[intf].base + REG_SC_INTEN);
	wait_event_interruptible(sc[intf].wq, sc[intf].state == SC_OP_IDLE);
	if (sc[intf].err) {
		return -1;
	}


	// 1. read prologue, get the block length
	sc[intf].rlen = 3;
	sc[intf].state = SC_OP_READ;
	wait_event_interruptible(sc[intf].wq, /*sc[intf].state == SC_OP_IDLE*/(sc[intf].rtail > 3) || (sc[intf].err != 0));
	if((sc[intf].err != 0) && (sc[intf].err != SC_ERR_PARITY)) {
		goto out;
	}
	// 2. read rest of the block
	sc[intf].rlen += sc[intf].rbuf[2] + (sc[intf].T1.EDC == SC_CHKSUM_LRC ? 1 : 2);
	sc[intf].state = SC_OP_READ;
	wait_event_interruptible(sc[intf].wq, (sc[intf].rtail == sc[intf].rlen) || (sc[intf].err != 0));
	if(sc[intf].err == SC_ERR_PARITY) {
		sc[intf].state = SC_OP_READ;
		sc[intf].err = 0;
		wait_event_interruptible(sc[intf].wq, (sc[intf].rtail == sc[intf].rlen) || (sc[intf].err != 0));
		sc[intf].err = SC_ERR_PARITY;  // to see if there's remaining data...
		__raw_writel(SC_ALTCTL_RXRST, sc[intf].base + REG_SC_ALTCTL);
	}

out:
	__raw_writel(__raw_readl(sc[intf].base + REG_SC_ALTCTL) & ~SC_ALTCTL_CNTEN0, sc[intf].base + REG_SC_ALTCTL);
	sc[intf].state = SC_OP_IDLE;

	if(!sc[intf].err) {
#if 0
		printk("RX:\n");
		{
			int i;
			for(i = 0; i < sc[intf].rlen; i++)
				printk("%02x ", sc[intf].rbuf[i]);
			printk("\n");
		}
#endif
		memcpy(block, sc[intf].rbuf, sc[intf].rlen);
		return sc[intf].rlen;
	} else if(sc[intf].err == SC_ERR_PARITY) {
		//printk("parity err\n");
		return -2;
	} else {
		//printk("other err\n");
		return -1;
	}
}

int t1_negotiate_ifsd(unsigned int intf, unsigned int dad, int ifsd)
{
	ct_buf_t sbuf;
	unsigned char sdata[T1_BUFFER_SIZE];
	unsigned int slen;
	unsigned int retries;
	size_t snd_len;
	int n;
	unsigned char snd_buf[1];
	t1_state_t * t1 = &t1_state[intf];

	retries = t1->retries;

	if(sc[intf].T1.EDC == SC_CHKSUM_LRC)
		t1_set_param(t1, IFD_PROTOCOL_T1_CHECKSUM_LRC, 0);
	else
		t1_set_param(t1, IFD_PROTOCOL_T1_CHECKSUM_CRC, 0);

	t1_set_param(t1, IFD_PROTOCOL_T1_IFSC, sc[intf].T1.IFSC);
	/* S-block IFSD request */
	snd_buf[0] = ifsd;
	snd_len = 1;

	/* Initialize send/recv buffer */
	ct_buf_set(&sbuf, (void *)snd_buf, snd_len);

	while (1)
	{
		/* Build the block */
		slen = t1_build(intf, sdata, 0, T1_S_BLOCK | T1_S_IFS, &sbuf, NULL);

		/* Send the block */
		n = t1_xcv(intf, sdata, slen, sizeof(sdata));

		retries--;
		/* ISO 7816-3 Rule 7.4.2 */
		if (retries <= 0)
			goto error;

		if (-1 == n)
		{
			//printk("fatal: transmit/receive failed\n");
			if(sc[intf].err == SC_ERR_TIME0OUT) {
				sc[intf].err = 0;
				continue;
			}
			goto error;
		}

		if ((-2 == n)								/* Parity error */
			|| (sdata[DATA] != ifsd)				/* Wrong ifsd received */
			|| (sdata[NAD] != swap_nibbles(dad))	/* wrong NAD */
			|| (!t1_verify_checksum(t1, sdata, n))	/* checksum failed */
			|| (n != 4 + (int)t1->rc_bytes)				/* wrong frame length */
			|| (sdata[LEN] != 1)					/* wrong data length */
			|| (sdata[PCB] != (T1_S_BLOCK | T1_S_RESPONSE | T1_S_IFS))) {/* wrong PCB */
			sc[intf].err = 0;
			continue;
		}
		/* no more error */
		goto done;
	}

done:
	return n;

error:
	t1->state = DEAD;
	return -1;
}


int CmdXfrT1(unsigned int intf,
	unsigned int tx_length, unsigned char tx_buffer[], unsigned int *rx_length,
	unsigned char rx_buffer[])
{
	int return_value = 0;
	int ret;

	memcpy(&gTx[intf][0], tx_buffer, tx_length);
	ret = t1_transceive(intf, 0,
		&gTx[intf][0], tx_length, &gRx[intf][0]);

	printk("out %d\n", ret);
	if (ret < 0)
		return_value = SC_ERR_T1;

	//copy data from temp buffer to rbuf
	sc[intf].rhead = 0;
	sc[intf].rtail = ret;
	memcpy(rx_buffer, &gRx[intf][0], ret);

	return return_value;
} /* CmdXfrT1 */
