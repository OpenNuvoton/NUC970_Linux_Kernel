/*
 * arch/arm/mach-nuc970/include/mach/regs-sc.h
 *
 * Copyright (c) 2015 Nuvoton Technology Corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __ASM_ARCH_REGS_SC_H
#define __ASM_ARCH_REGS_SC_H

/* SC Registers */

#define SC0_BA			NUC970_VA_SC
#define SC1_BA			(NUC970_VA_SC + 0x400)
#define REG_SC_DAT		0x00
#define REG_SC_CTL		0x04
#define REG_SC_ALTCTL		0x08
#define REG_SC_EGT		0x0C
#define REG_SC_RXTOUT		0x10
#define REG_SC_ETUCTL		0x14
#define REG_SC_INTEN		0x18
#define REG_SC_INTSTS		0x1C
#define REG_SC_STATUS		0x20
#define REG_SC_PINCTL		0x24
#define REG_SC_TMRCTL0		0x28
#define REG_SC_TMRCTL1		0x2C
#define REG_SC_TMRCTL2		0x30
#define REG_SC_UARTCTL		0x34

#define SC_CTL_SYNC		0x40000000
#define SC_CTL_CDLV		0x04000000
#define SC_CTL_TXRTYEN		0x00800000
#define SC_CTL_TXRTY		0x00700000
#define SC_CTL_RXRTYEN		0x00080000
#define SC_CTL_RXRTY		0x00070000
#define SC_CTL_NSB		0x00008000
#define SC_CTL_TMRSEL		0x00006000
#define SC_CTL_BGT		0x00001F00
#define SC_CTL_RXTRGLV		0x000000C0
#define SC_CTL_AUTOCEN		0x00000008
#define SC_CTL_TXOFF		0x00000004
#define SC_CTL_RXOFF		0x00000002
#define SC_CTL_SCEN		0x00000001

#define SC_ALTCTL_ADACEN	0x00000800
#define SC_ALTCTL_INITSEL	0x00000300
#define SC_ALTCTL_CNTEN2	0x00000080
#define SC_ALTCTL_CNTEN1	0x00000040
#define SC_ALTCTL_CNTEN0	0x00000020
#define SC_ALTCTL_WARSTEN	0x00000010
#define SC_ALTCTL_ACTEN		0x00000008
#define SC_ALTCTL_DACTEN	0x00000004
#define SC_ALTCTL_RXRST		0x00000002
#define SC_ALTCTL_TXRST		0x00000001

#define SC_INTEN_ACERRIEN	0x00000400
#define SC_INTEN_RXTOIEN	0x00000200
#define SC_INTEN_INITIEN	0x00000100
#define SC_INTEN_CDIEN		0x00000080
#define SC_INTEN_TMR2IEN	0x00000020
#define SC_INTEN_TMR1IEN	0x00000010
#define SC_INTEN_TMR0IEN	0x00000008
#define SC_INTEN_TERRIEN	0x00000004
#define SC_INTEN_TBEIEN		0x00000002
#define SC_INTEN_RDAIEN		0x00000001

#define SC_INTSTS_ACERRIF	0x00000400
#define SC_INTSTS_RXTOIF	0x00000200
#define SC_INTSTS_INITIF	0x00000100
#define SC_INTSTS_CDIF		0x00000080
#define SC_INTSTS_TMR2IF	0x00000020
#define SC_INTSTS_TMR1IF	0x00000010
#define SC_INTSTS_TMR0IF	0x00000008
#define SC_INTSTS_TERRIF	0x00000004
#define SC_INTSTS_TBEIF		0x00000002
#define SC_INTSTS_RDAIF		0x00000001

#define SC_STATUS_TXOVERR	0x40000000
#define SC_STATUS_TXRERR	0x20000000
#define SC_STATUS_RXOVERR	0x00400000
#define SC_STATUS_RXRERR	0x00200000
#define SC_STATUS_CDPINSTS	0x00002000
#define SC_STATUS_CINSERT	0x00001000
#define SC_STATUS_CREMOVE	0x00000800
#define SC_STATUS_TXFULL	0x00000400
#define SC_STATUS_TXEMPTY	0x00000200
#define SC_STATUS_TXOV		0x00000100
#define SC_STATUS_BEF		0x00000040
#define SC_STATUS_FEF		0x00000020
#define SC_STATUS_PEF		0x00000010
#define SC_STATUS_RXFULL	0x00000004
#define SC_STATUS_RXEMPTY	0x00000002
#define SC_STATUS_RXOV		0x00000001

#define SC_PINCTL_SYNC		0x40000000
#define SC_PINCTL_RSTSTS	0x00040000
#define SC_PINCTL_PWRINV	0x00000800
#define SC_PINCTL_SCDOUT	0x00000200
#define SC_PINCTL_CLKKEEP	0x00000040
#define SC_PINCTL_SCRST		0x00000002
#define SC_PINCTL_PWREN		0x00000001



/* Return values */
#define ATR_OK		0	/* ATR could be parsed and data returned */
#define ATR_NOT_FOUND	1	/* Data not present in ATR */
#define ATR_MALFORMED	2	/* ATR could not be parsed */
#define ATR_IO_ERROR	3	/* I/O stream error */

/* Paramenters */
#define ATR_MAX_SIZE 		33	/* Maximum size of ATR byte array */
#define ATR_MAX_HISTORICAL	15	/* Maximum number of historical bytes */
#define ATR_MAX_PROTOCOLS	7	/* Maximun number of protocols */
#define ATR_MAX_IB		4	/* Maximum number of interface bytes per protocol */
#define ATR_CONVENTION_DIRECT	0	/* Direct convention */
#define ATR_CONVENTION_INVERSE	1	/* Inverse convention */
#define ATR_PROTOCOL_TYPE_T0	0	/* Protocol type T=0 */
#define ATR_PROTOCOL_TYPE_T1	1	/* Protocol type T=1 */
#define ATR_PROTOCOL_TYPE_T2	2	/* Protocol type T=2 */
#define ATR_PROTOCOL_TYPE_T3	3	/* Protocol type T=3 */
#define ATR_PROTOCOL_TYPE_T14	14	/* Protocol type T=14 */
#define ATR_INTERFACE_BYTE_TA	0	/* Interface byte TAi */
#define ATR_INTERFACE_BYTE_TB	1	/* Interface byte TBi */
#define ATR_INTERFACE_BYTE_TC	2	/* Interface byte TCi */
#define ATR_INTERFACE_BYTE_TD	3	/* Interface byte TDi */
#define ATR_PARAMETER_F		0	/* Parameter F */
#define ATR_PARAMETER_D		1	/* Parameter D */
#define ATR_PARAMETER_I		2	/* Parameter I */
#define ATR_PARAMETER_P		3	/* Parameter P */
#define ATR_PARAMETER_N		4	/* Parameter N */
#define ATR_INTEGER_VALUE_FI	0	/* Integer value FI */
#define ATR_INTEGER_VALUE_DI	1	/* Integer value DI */
#define ATR_INTEGER_VALUE_II	2	/* Integer value II */
#define ATR_INTEGER_VALUE_PI1	3	/* Integer value PI1 */
#define ATR_INTEGER_VALUE_N	4	/* Integer value N */
#define ATR_INTEGER_VALUE_PI2	5	/* Integer value PI2 */

/* Default values for paramenters */
#define ATR_DEFAULT_F	372
#define ATR_DEFAULT_D	1
#define ATR_DEFAULT_I 	50
#define ATR_DEFAULT_N	0
#define ATR_DEFAULT_P	5

#define SC_INTF		2
#define MAX_LEN		300 // This is larger than MAC T1 block size 271

typedef struct {

    unsigned char   IFSC;  ///< Current information field size that can be transmitted
    unsigned char   IFSD;  ///< Current information field size we can receive
    unsigned char   IBLOCK_REC; ///< Record if received I-block was sent correctly from ICC
    unsigned char   RSN;  ///< The 'number' of received I-Blocks
    unsigned char   SSN;  ///< The 'number' of sent I-Blocks as defined in ISO 7816-3
    unsigned char   WTX;   ///< Waiting time extension requested by the smart card, This value should be used by the driver to extend block waiting time
} T1_DATA;

typedef struct {
    unsigned int   Lc;    ///< Number of data bytes in this request
    unsigned int   Le;    ///< Number of expected bytes from the card

} T0_DATA;


typedef struct
{
	unsigned int length;
	char TS;
	char T0;
	struct
	{
		char value;
		char present;
	} ib[ATR_MAX_PROTOCOLS][ATR_MAX_IB], TCK;
	unsigned int pn;
	char hb[ATR_MAX_HISTORICAL];
	unsigned int hbn;
}ATR_t;

struct nuc970_sc {
	//spinlock_t lock;
	struct mutex lock;
	struct pinctrl *pinctrl;
	struct clk *clk;
	struct clk *eclk;
	wait_queue_head_t wq;
	int protocol;			// T0, T1
	int intf;			// 0, 1
	int minor;			// dynamic allocate device minor number
	struct resource *res;
	void __iomem * base;			// reg base
	char rbuf[MAX_LEN];		// recv data from card
	int rhead;			// last rx data
	int rtail;			// first rx data
	int rlen;			// expect receive length (from user application layer)
	char tbuf[MAX_LEN];		// tx data to card
	int tcnt;			// total tx data len
	int toffset;			// current tx data offset
	char atrbuf[ATR_MAX_SIZE];	// ATR from card
	ATR_t atr;
	int atrlen;			// ATR length
	int act;			// Card activated
	int err;			// Last error code. 0 means no error
	int ignorecd;			// ignore CD pin
	int state;			// idle, read, write, atr, (removed?), (acted?)
	int F;
	int D;
	int N;
	int irq;			// IRQ number
	/** T=0 specific data */
	struct {
		u8 WI;         ///< Waiting integer
		u32 WT;        ///< Waiting time in ETU
	} T0;

	/** T=1 specific data */
	struct {
		u8 IFSC;       ///< Information field size of card
		u8 CWI;        ///< Character waiting integer
		u8 BWI;        ///< Block waiting integer
		u8 EDC;        ///< Error detection code
		u8 ChkSum;
		u32 CWT;       ///< Character and block waiting time in ETU
		u32 BWT;       ///< Character and block waiting time in ETU
		u32 WTX;       ///< Block guarding time in micro seconds
	} T1;

	T0_DATA T0_dat;   ///< Data for T=0
	T1_DATA T1_dat;   ///< Data for T=1
};

#endif /*  __ASM_ARCH_REGS_SC_H */
