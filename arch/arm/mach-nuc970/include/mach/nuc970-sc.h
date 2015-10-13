/* linux/arch/arm/mach-nuc970/include/mach/nuc970-sc.h
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

#ifndef _NUC970_SC_H_
#define _NUC970_SC_H_

#include <linux/types.h>
#include <linux/ioctl.h>

// parameter for SC_IOC_TRANSACT command
struct sc_transact{
	unsigned char *tx_buf;
	unsigned int tx_len;
	unsigned char *rx_buf;
	unsigned int rx_len;
};

#define SC_IOC_MAGIC		's'
#define SC_IOC_MAXNR		5

#define SC_IOC_ACTIVATE			_IO(SC_IOC_MAGIC, 0)
#define SC_IOC_READATR			_IOR(SC_IOC_MAGIC, 1, unsigned int *)
#define SC_IOC_DEACTIVATE		_IO(SC_IOC_MAGIC, 2)
#define SC_IOC_GETSTATUS		_IOR(SC_IOC_MAGIC, 3, unsigned int *)
#define SC_IOC_SETPARAM			_IO(SC_IOC_MAGIC, 4)
#define SC_IOC_TRANSACT			_IOWR(SC_IOC_MAGIC, 5, unsigned int *)

// Status
#define SC_OP_IDLE			0x0000
#define SC_OP_READ			0x0001
#define SC_OP_WRITE			0x0002
#define SC_OP_READ_ATR			0x0003
#define SC_OP_COLD_RESET		0x0004
#define SC_OP_WARM_RESET		0x0005
#define SC_OP_DEACTIVATE		0x0006

// Error code
#define SC_ERR_CARD_REMOVED		0x0001
#define SC_ERR_TIME0OUT			0x0002
#define SC_ERR_TIME1OUT			0x0003
#define SC_ERR_TIME2OUT			0x0004
#define SC_ERR_AUTOCONVENTION		0x0005
#define SC_ERR_READ			0x0006
#define SC_ERR_WRITE			0x0007
#define SC_ERR_PARAM			0x0008
#define SC_ERR_ATR			0x0009
#define SC_ERR_PPS			0x000A
#define SC_ERR_T0			0x000B
#define SC_ERR_T1			0x000C
#define SC_ERR_PARITY			0x000D

// CD/ACT state for user level query
#define ICC_PRESENT_ACTIVE		0x0000
#define ICC_PRESENT_INACTIVE		0x0001
#define ICC_ABSENT			0x0002

#define SC_TMR_MODE_0			0x00000000
#define SC_TMR_MODE_1			0x01000000
#define SC_TMR_MODE_2			0x02000000
#define SC_TMR_MODE_3			0x03000000
#define SC_TMR_MODE_4			0x04000000
#define SC_TMR_MODE_5			0x05000000
#define SC_TMR_MODE_6			0x06000000
#define SC_TMR_MODE_7			0x07000000
#define SC_TMR_MODE_8			0x08000000
#define SC_TMR_MODE_F			0x0F000000

#define SC_PROTOCOL_T0			0
#define SC_PROTOCOL_T1			1

#define SC_CHKSUM_CRC			0
#define SC_CHKSUM_LRC			1

#endif
