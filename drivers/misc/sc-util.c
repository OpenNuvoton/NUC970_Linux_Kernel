/* linux/driver/misc/sc-util.c
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
// This file is based on atr.c and pps.c from a project start by Towitoko
/*
    atr.c
    ISO 7816 ICC's answer to reset abstract data type implementation

    This file is part of the Unix driver for Towitoko smartcard readers
    Copyright (C) 2000 Carlos Prados <cprados@yahoo.com>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2 of the License, or (at your option) any later version.

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

/*
 * Not exported variables definition
 */

static unsigned int atr_num_ib_table[16] = {0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4};

/*
 * Exported variables definition
 */

static unsigned int atr_f_table[16] = {
	372, 372, 558, 744, 1116, 1488, 1860, 0, 0, 512, 768, 1024, 1536, 2048, 0, 0
};

static unsigned int atr_d_table[16] = {0, 1, 2, 4, 8, 16, 32, 64, 12, 20, 0, 0, 0, 0, 0, 0};

//static unsigned int atr_i_table[4] = {25, 50, 100, 0};

/*
 * Exported functions definition
 */

int ATR_InitFromArray (ATR_t * atr, const char atr_buffer[ATR_MAX_SIZE], unsigned int length)
{
	char TDi, tck = 0;
	unsigned int pointer = 0, pn = 0;


	/* Check size of buffer */
	if (length < 2)
		return (SC_ERR_ATR);

	/* Store T0 and TS */
	atr->TS = atr_buffer[0];

	atr->T0 = TDi = atr_buffer[1];
	pointer = 1;

	/* Store number of historical bytes */
	atr->hbn = TDi & 0x0F;

	/* TCK is not present by default */
	(atr->TCK).present = 0;

	/* Extract interface bytes */
	while (pointer < length) {
		/* Check buffer is long enought */
		if (pointer + atr_num_ib_table[(0xF0 & TDi) >> 4] >= length) {
			return (SC_ERR_ATR);
		}
		/* Check TAi is present */
		if ((TDi | 0xEF) == 0xFF) {
			pointer++;
			atr->ib[pn][ATR_INTERFACE_BYTE_TA].value = atr_buffer[pointer];
			atr->ib[pn][ATR_INTERFACE_BYTE_TA].present = 1;
		} else
			atr->ib[pn][ATR_INTERFACE_BYTE_TA].present = 0;
		/* Check TBi is present */
		if ((TDi | 0xDF) == 0xFF) {
			pointer++;
			atr->ib[pn][ATR_INTERFACE_BYTE_TB].value = atr_buffer[pointer];
			atr->ib[pn][ATR_INTERFACE_BYTE_TB].present = 1;
		} else
			atr->ib[pn][ATR_INTERFACE_BYTE_TB].present = 0;

		/* Check TCi is present */
		if ((TDi | 0xBF) == 0xFF) {
			pointer++;
			atr->ib[pn][ATR_INTERFACE_BYTE_TC].value = atr_buffer[pointer];
			atr->ib[pn][ATR_INTERFACE_BYTE_TC].present = 1;
		} else
			atr->ib[pn][ATR_INTERFACE_BYTE_TC].present = 0;

		/* Read TDi if present */
		if ((TDi | 0x7F) == 0xFF) {
			pointer++;
			TDi = atr->ib[pn][ATR_INTERFACE_BYTE_TD].value = atr_buffer[pointer];
			atr->ib[pn][ATR_INTERFACE_BYTE_TD].present = 1;
			(atr->TCK).present = ((TDi & 0x0F) != ATR_PROTOCOL_TYPE_T0);
			pn++;
			if (pn >= ATR_MAX_PROTOCOLS)
				return (SC_ERR_ATR);
		} else {
			atr->ib[pn][ATR_INTERFACE_BYTE_TD].present = 0;
			break;
		}
	}

	/* Store number of protocols */
	atr->pn = pn + 1;

	/* Store historical bytes */
	if (pointer + atr->hbn >= length)
		return (SC_ERR_ATR);

	memcpy (atr->hb, atr_buffer + pointer + 1, atr->hbn);
	pointer += (atr->hbn);

	/* Store TCK  */
	if ((atr->TCK).present) {
		int i;
		for(i = 1; i < length; i++)
			tck ^= atr_buffer[i];

		if (pointer + 1 >= length)
			return (SC_ERR_ATR);
		if(tck != 0)
			return (SC_ERR_ATR);

		pointer++;

		(atr->TCK).value = atr_buffer[pointer];

	}

	atr->length = pointer + 1;
	return (0);
}

int ATR_GetIntegerValue (ATR_t * atr, int name, char * value)
{
	int ret;

	if (name == ATR_INTEGER_VALUE_FI) {
		if (atr->ib[0][ATR_INTERFACE_BYTE_TA].present) {
			(*value) = (atr->ib[0][ATR_INTERFACE_BYTE_TA].value & 0xF0) >> 4;
			ret = 0;
		} else
			ret = ATR_NOT_FOUND;
	}

	else if (name == ATR_INTEGER_VALUE_DI) {
		if (atr->ib[0][ATR_INTERFACE_BYTE_TA].present) {
			(*value) = (atr->ib[0][ATR_INTERFACE_BYTE_TA].value & 0x0F);
			ret = 0;
		} else
			ret = ATR_NOT_FOUND;
	}

	else if (name == ATR_INTEGER_VALUE_II) {
		if (atr->ib[0][ATR_INTERFACE_BYTE_TB].present) {
			(*value) = (atr->ib[0][ATR_INTERFACE_BYTE_TB].value & 0x60) >> 5;
			ret = 0;
		} else
			ret = ATR_NOT_FOUND;
	}

	else if (name == ATR_INTEGER_VALUE_PI1) {
		if (atr->ib[0][ATR_INTERFACE_BYTE_TB].present) {
			(*value) = (atr->ib[0][ATR_INTERFACE_BYTE_TB].value & 0x1F);
			ret = 0;
		} else
			ret = ATR_NOT_FOUND;
	}

	else if (name == ATR_INTEGER_VALUE_PI2) {
		if (atr->ib[1][ATR_INTERFACE_BYTE_TB].present) {
			(*value) = atr->ib[1][ATR_INTERFACE_BYTE_TB].value;
			ret = 0;
		} else
			ret = ATR_NOT_FOUND;
	}

	else if (name == ATR_INTEGER_VALUE_N) {
		if (atr->ib[0][ATR_INTERFACE_BYTE_TC].present) {
			(*value) = atr->ib[0][ATR_INTERFACE_BYTE_TC].value;
			ret = 0;
		} else
			ret = ATR_NOT_FOUND;
	} else
		ret = ATR_NOT_FOUND;

	return ret;
}

int ATR_GetParameter (ATR_t * atr, int name, int *parameter)
{
	unsigned char FI, DI, /*II, PI1, PI2,*/ N;

	if (name == ATR_PARAMETER_F) {
		if (ATR_GetIntegerValue (atr, ATR_INTEGER_VALUE_FI, &FI) == 0)
			*parameter = (int)atr_f_table[FI];
		else
			(*parameter) = ATR_DEFAULT_F;
		return (0);
	}

	else if (name == ATR_PARAMETER_D) {
		if (ATR_GetIntegerValue (atr, ATR_INTEGER_VALUE_DI, &DI) == 0)
			*parameter = (int)atr_d_table[DI];
		else
			(*parameter) = ATR_DEFAULT_D;
		return (0);
	}
	else if (name == ATR_PARAMETER_N) {
		if (ATR_GetIntegerValue (atr, ATR_INTEGER_VALUE_N, &N) == 0)
			(*parameter) = N;
		else
			(*parameter) = ATR_DEFAULT_N;
		return (0);
	}

	return (ATR_NOT_FOUND);
}

/*
 * This function was greatly inspired by ATRDecodeAtr() and
 * PHGetDefaultProtocol() from pcsc-lite
 *
 * It was rewritten by Ludovic Rousseau, 2004
 */
#define PROTOCOL_UNSET -1
int ATR_GetDefaultProtocol(ATR_t * atr, int *protocol, int *availableProtocols)
{
	int i;

	/* default value */
	*protocol = PROTOCOL_UNSET;
	if (availableProtocols)
		*availableProtocols = 0;

	for (i=0; i<ATR_MAX_PROTOCOLS; i++)
		if (atr->ib[i][ATR_INTERFACE_BYTE_TD].present) {
			int T = atr->ib[i][ATR_INTERFACE_BYTE_TD].value & 0x0F;

			//printk("T=%d Protocol Found\n", T);
			if (availableProtocols)
				*availableProtocols |= 1 << T;

			if (PROTOCOL_UNSET == *protocol) {
				/* set to the first protocol byte found */
				*protocol = T;
				//printk("default protocol: T=%d\n", *protocol);
			}
		}

	/* specific mode if TA2 present */
	if (atr->ib[1][ATR_INTERFACE_BYTE_TA].present) {
		*protocol = atr->ib[1][ATR_INTERFACE_BYTE_TA].value & 0x0F;
		if (availableProtocols)
			*availableProtocols = 1 << *protocol;
		//printk("specific mode found: T=%d\n", *protocol);
	}

	if (PROTOCOL_UNSET == *protocol) {
		//printk("no default protocol found in ATR. Using T=0\n");
		*protocol = ATR_PROTOCOL_TYPE_T0;
		if (availableProtocols)
			*availableProtocols = 1 << *protocol;
	}

	return 0;
}

#ifdef CONFIG_EMV_CHECK
// For EMV
int ATR_CheckIntegrity(ATR_t * atr, int type)
{
	int i, j, proto, availableproto;

        if(atr->ib[0][ATR_INTERFACE_BYTE_TA].present == 1) {
            if(atr->ib[0][ATR_INTERFACE_BYTE_TA].value < 0x11 || atr->ib[0][ATR_INTERFACE_BYTE_TA].value > 0x13)
                return SC_ERR_PARAM;
        }

        /* In response to the cold-reset, TB1 only could be 0x00 */
        if(type == 0) { // cold reset
            if(atr->ib[0][ATR_INTERFACE_BYTE_TB].present == 1)
                if(atr->ib[0][ATR_INTERFACE_BYTE_TB].value != 0x00)
                    return SC_ERR_PARAM;
            if(atr->ib[0][ATR_INTERFACE_BYTE_TB].present == 0)
                return SC_ERR_PARAM;
        }

        /* Reject ATR containing TB2 */
        if(atr->ib[1][ATR_INTERFACE_BYTE_TB].present == 1)
            return SC_ERR_PARAM;

        /* Bit [5] of TA2 must be equal to 0x0 */
        if(atr->ib[1][ATR_INTERFACE_BYTE_TA].present == 1) {
            if((atr->ib[1][ATR_INTERFACE_BYTE_TA].value & 0x10) == 0x10)
                return SC_ERR_PARAM;
        }

        /* Reject an ATR that TC2 is equal to 0x00 */
        if(atr->ib[1][ATR_INTERFACE_BYTE_TC].present == 1 && atr->ib[1][ATR_INTERFACE_BYTE_TC].value == 0x00)
            return SC_ERR_PARAM;

        /* TD1's l.s. nibble must be 0x0 or 0x1 */
        if(atr->ib[0][ATR_INTERFACE_BYTE_TD].present == 1) {
            if((atr->ib[0][ATR_INTERFACE_BYTE_TD].value & 0xF) > 0x1) {
                return SC_ERR_PARAM;
            }
        }

        /* TD2's l.s. nibble must be 0x1 or 0xE if TD1's l.s. nibble is 0x0 */
        if(atr->ib[1][ATR_INTERFACE_BYTE_TD].present == 1) {
            if((atr->ib[1][ATR_INTERFACE_BYTE_TD].value & 0xF)!=0x1 && (atr->ib[1][ATR_INTERFACE_BYTE_TD].value & 0xF) != 0xE)
                return SC_ERR_PARAM;

            if((atr->ib[1][ATR_INTERFACE_BYTE_TD].value & 0xF) == 0xE) {
                if((atr->ib[0][ATR_INTERFACE_BYTE_TD].value & 0xF) != 0x0)
                    return SC_ERR_PARAM;
            }
        }

        /* Reject TA3 having a value in the range 0x0~0xF or 0xFF */
        if(atr->ib[2][ATR_INTERFACE_BYTE_TA].present == 1) {
            if(atr->ib[2][ATR_INTERFACE_BYTE_TA].value < 0x10 || atr->ib[2][ATR_INTERFACE_BYTE_TA].value == 0xFF) {
                return SC_ERR_PARAM;
            }

        }

	// Get default protocol first...
	ATR_GetDefaultProtocol(atr, &proto, &availableproto);

        /* Reject ATR not containing TB3 or BWI greater than 4 or CWI greater than 5 */
        /* And reject ATR if fitting the formula : 2 to the power of CWI is equal or less than (N+1) */
        if(proto == 1) {
            if(atr->ib[2][ATR_INTERFACE_BYTE_TB].present == 1) {
                if(((atr->ib[2][ATR_INTERFACE_BYTE_TB].value & 0xF0) >> 4) > 0x4)
                    return SC_ERR_PARAM;

                if((atr->ib[2][ATR_INTERFACE_BYTE_TB].value & 0xF) > 0x5)
                    return SC_ERR_PARAM;

                i = 1;
                j = (atr->ib[2][ATR_INTERFACE_BYTE_TB].value & 0xF);
                while(j--)
                    i *= 2;
                /* if TC1 is equal to 0xFF, N as -1 that is always valid */
                if(atr->ib[0][ATR_INTERFACE_BYTE_TC].value != 0xFF)
                    if( i <= (atr->ib[0][ATR_INTERFACE_BYTE_TC].value + 1))
                        return SC_ERR_PARAM;

            } else
                return SC_ERR_PARAM;

		/* ATR must contain TB3 in T=1 */
		if(atr->ib[2][ATR_INTERFACE_BYTE_TB].present == 0)
			return SC_ERR_PARAM;

        }

        /* Reject ATR if TC3 is not equal to 0x00 */
        if(atr->ib[2][ATR_INTERFACE_BYTE_TC].present == 1) {
            if(atr->ib[2][ATR_INTERFACE_BYTE_TC].value != 0x00) {
                return SC_ERR_PARAM;
            }
        }

        return 0;
}
#endif//ifdef CONFIG_EMV_CHECK

// including PPS exchange
int ATR_Parse(int intf)
{
	char pps[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
	ATR_t atr = sc[intf].atr;
	char PCK;
	int protocol, availableproto;


	/* Set to zero buffer */
	memset(pps, 0, sizeof(pps));

	// Get default protocol first...
	ATR_GetDefaultProtocol(&atr, &protocol, &availableproto);


	if (0 == protocol)
		pps[1] |= ATR_PROTOCOL_TYPE_T0;
	else
		if (1 == protocol)
			pps[1] |= ATR_PROTOCOL_TYPE_T1;
		else
			return SC_ERR_ATR;

	/* TA1 present */
	if (atr.ib[0][ATR_INTERFACE_BYTE_TA].present)
	{
		unsigned int card_baudrate;
		unsigned int default_baudrate;


		(void)ATR_GetParameter(&atr, ATR_PARAMETER_D, &sc[intf].D);
		(void)ATR_GetParameter(&atr, ATR_PARAMETER_F, &sc[intf].F);

		/* may happen with non ISO cards */
		if ((0 == sc[intf].F) || (0 == sc[intf].D))
		{
			/* values for TA1=11 */
			sc[intf].F = 372;
			sc[intf].D = 1;
		}

		/* Baudrate = f x D/F */
		card_baudrate = (unsigned int) (4000000
			* sc[intf].D / sc[intf].F);

		default_baudrate = (unsigned int) (4000000
			* ATR_DEFAULT_D / ATR_DEFAULT_F);

		/* if the card does not try to lower the default speed */
		if (card_baudrate > default_baudrate)
		{
			pps[1] |= 0x10; /* PTS1 presence */
			pps[2] = atr.ib[0][ATR_INTERFACE_BYTE_TA].value;

			printk("Set speed to %d bauds\n", card_baudrate);
		}
	} else { // assign default value
		sc[intf].F = 372;
		sc[intf].D = 1;

	}

#ifndef CONFIG_EMV_CHECK	// PPS not specified in EMV
	/* Generate PPS */
	pps[0] = 0xFF;
	pps[3] = pps[0] ^ pps[1] ^ pps[2];

	/* TA2 absent: negociable mode */
	if ( ! atr.ib[1][ATR_INTERFACE_BYTE_TA].present)
	{
		int default_protocol;
		//printk("prepare for pps\n");

		ATR_GetDefaultProtocol(&atr, &default_protocol, NULL);

		/* if the requested protocol is not the default one
		 * or a TA1/PPS1 is present */
		if (((pps[1] & 0x0F) != default_protocol) || (pps[2] != 0x00))
		{
			sc[intf].tbuf[0] = pps[0];
			sc[intf].tbuf[1] = pps[1];
			sc[intf].tbuf[2] = pps[2];
			sc[intf].tbuf[3] = pps[3];
			sc[intf].toffset = 0;
			sc[intf].tcnt = 4;
			sc[intf].state = SC_OP_WRITE;
			sc[intf].rlen = 4;
			sc[intf].rhead = sc[intf].rtail = 0;

			__raw_writel(__raw_readl(sc[intf].base + REG_SC_ALTCTL) & ~(SC_ALTCTL_CNTEN2 | SC_ALTCTL_CNTEN0 | SC_ALTCTL_CNTEN0),
						sc[intf].base + REG_SC_ALTCTL);
			// Waiting time check, the waiting time of PPS is fixed at 9600 ETUs
			__raw_writel((9600 - 1) | SC_TMR_MODE_7, sc[intf].base + REG_SC_TMRCTL0);
			__raw_writel(__raw_readl(sc[intf].base + REG_SC_ALTCTL) | SC_ALTCTL_CNTEN0, sc[intf].base + REG_SC_ALTCTL);

			__raw_writel(__raw_readl(sc[intf].base + REG_SC_INTEN) | SC_INTEN_TBEIEN, sc[intf].base + REG_SC_INTEN);
			wait_event_interruptible(sc[intf].wq, (sc[intf].state == SC_OP_IDLE) || (sc[intf].err != 0));
			if(sc[intf].err != 0)
				return SC_ERR_PPS;

			sc[intf].state = SC_OP_READ;
			wait_event_interruptible(sc[intf].wq, ((sc[intf].rtail == 4) || (sc[intf].err != 0)));
			__raw_writel(__raw_readl(sc[intf].base + REG_SC_ALTCTL) & ~SC_ALTCTL_CNTEN0, sc[intf].base + REG_SC_ALTCTL);
			if(sc[intf].err != 0)
				return SC_ERR_PPS;

			if(sc[intf].rbuf[0] != sc[intf].tbuf[0])   /* PPSS */
				return SC_ERR_PPS;

			PCK = sc[intf].rbuf[0];
			if((sc[intf].rbuf[1]&0x0f) == (sc[intf].tbuf[1] &0x0f) &&
				((sc[intf].rbuf[1] & 0xf0) == 0x10 ||(sc[intf].rbuf[1] & 0xf0) == 0x00)) {
				PCK ^= sc[intf].rbuf[1];

			} else
				return SC_ERR_PPS;

			if (sc[intf].rbuf[2] == sc[intf].tbuf[2])
				PCK ^= sc[intf].rbuf[2];
			else
				return SC_ERR_PPS;

			if (sc[intf].rbuf[3] != PCK)  /* PCK */
				return SC_ERR_PPS;

		}
	}
#else

	if ( ! atr.ib[1][ATR_INTERFACE_BYTE_TA].present) {  // TA2 absent

		if(atr.ib[0][ATR_INTERFACE_BYTE_TA].present) {	// TA1 present
			sc[intf].F = 372;
			sc[intf].D = 1;
		}
	}

#endif
	__raw_writel((sc[intf].F / sc[intf].D) - 1, sc[intf].base + REG_SC_ETUCTL);
	printk("set etu to %d\n", (sc[intf].F / sc[intf].D));


	/* TCi (i>2) indicates CRC instead of LRC */
	if (SC_PROTOCOL_T1 == protocol)
	{
		//t1_state_t *t1 = &(reader -> t1);
		int i;

		/* TCi (i>2) present? */
		for (i=2; i<ATR_MAX_PROTOCOLS; i++)
			if (atr.ib[i][ATR_INTERFACE_BYTE_TC].present)
			{
				if (0 == atr.ib[i][ATR_INTERFACE_BYTE_TC].value)
				{
					//printk("Use LRC\n");
					sc[intf].T1.EDC = SC_CHKSUM_LRC;
				} else if (1 == atr.ib[i][ATR_INTERFACE_BYTE_TC].value) {
					//printk("Use CRC\n");
					sc[intf].T1.EDC = SC_CHKSUM_CRC;
				}
				/* only the first TCi (i>2) must be used */
				break;
			} else
				sc[intf].T1.EDC = SC_CHKSUM_LRC;

		for (i=2; i<ATR_MAX_PROTOCOLS; i++)
			if (atr.ib[i][ATR_INTERFACE_BYTE_TB].present)
			{
				int j, k = 1;
				sc[intf].T1.CWI = atr.ib[i][ATR_INTERFACE_BYTE_TB].value & 0xF;
				for(j = 0; j < sc[intf].T1.CWI; j++)
					k *= 2;
				sc[intf].T1.CWT = k + 11;  // CWT = (11 + 2^CWI)
				sc[intf].T1.BWI = atr.ib[i][ATR_INTERFACE_BYTE_TB].value >> 4;
				k = 1;
				for(j = 0; j < sc[intf].T1.BWI; j++)
					k *= 2;
				// ISO 7816-3 11.4.3
				sc[intf].T1.BWT = 11 + (k * 960 * 372) / (__raw_readl(sc[intf].base + REG_SC_ETUCTL) + 1);
				break;
			}
		if(i == ATR_MAX_PROTOCOLS) {  // set default
			int i, j = 1;
			sc[intf].T1.CWI = 13;
			for(i = 0; i < 13; i++)
				j *= 2;
			sc[intf].T1.CWT = j + 11;
			sc[intf].T1.BWI = 4;
			j = 1;
			for(i = 0; i < sc[intf].T1.BWI; i++)
				j *= 2;
			// ISO 7816-3 11.4.3
			sc[intf].T1.BWT = 11 + (j * 960 * 372) / (__raw_readl(sc[intf].base + REG_SC_ETUCTL) + 1);
		}

		//printk("Set BWT to %d, CWT to %d\n", sc[intf].T1.BWT, sc[intf].T1.CWT);

		for (i=2; i<ATR_MAX_PROTOCOLS; i++)
			if (atr.ib[i][ATR_INTERFACE_BYTE_TA].present)
			{
				sc[intf].T1.IFSC = atr.ib[i][ATR_INTERFACE_BYTE_TA].value;
				//printk("set IFSC to %d\n", sc[intf].T1.IFSC);
				break;
			}
		if(i == ATR_MAX_PROTOCOLS) {
			sc[intf].T1.IFSC = 32; // use default value
		}
	}



#if 0
	/* specific mode and implicit parameters? (b5 of TA2) */
	if (atr.ib[1][ATR_INTERFACE_BYTE_TA].present
		&& (atr.ib[1][ATR_INTERFACE_BYTE_TA].value & 0x10))
		return SC_ERR_ATR;
#endif

	sc[intf].protocol = protocol;
	(void)ATR_GetParameter(&atr, ATR_PARAMETER_N, &sc[intf].N);

	if(atr.ib[1][ATR_INTERFACE_BYTE_TC].present == 1 ) { // TC2 present
		//if(atr.ib[0][ATR_INTERFACE_BYTE_TD].value == 0)
			sc[intf].T0.WI = atr.ib[1][ATR_INTERFACE_BYTE_TC].value;
	} else {
		sc[intf].T0.WI = 10;  // set default
	}
	sc[intf].T0.WT = (sc[intf].T0.WI * 960 * sc[intf].F) / (__raw_readl(sc[intf].base + REG_SC_ETUCTL) + 1) + 500 * sc[intf].D; // +500 for INQ

	//printk("WT = %d\n", sc[intf].T0.WT);

	return 0;
}


