/*
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
#ifndef __ASM_NUC970ADC_H
#define __ASM_NUC970ADC_H

#define ADC_CTL_ADEN 			0x00000001
#define ADC_CTL_VBGEN 		0x00000002
#define ADC_CTL_PKWPEN		0x00000004
#define ADC_CTL_MST 			0x00000100
#define ADC_CTL_PEDEEN 		0x00000200
#define ADC_CTL_WKPEN 		0x00000400
#define ADC_CTL_WKTEN 		0x00000800
#define ADC_CTL_WMSWCH 		0x00010000

#define ADC_CONF_TEN 			0x00000001
#define ADC_CONF_ZEN 			0x00000002
#define ADC_CONF_NACEN		0x00000004
#define ADC_CONF_VBATEN 	0x00000100
#define ADC_CONF_KPCEN 		0x00000200
#define ADC_CONF_SELFTEN 	0x00000400
#define ADC_CONF_DISTMAVEN 	(1<<20)
#define ADC_CONF_DISZMAVEN 	(1<<21)
#define ADC_CONF_HSPEED 		(1<<22)

#define ADC_CONF_CHSEL_Pos 	3
#define ADC_CONF_CHSEL_Msk 	(7<<3)
#define ADC_CONF_CHSEL_VBT 	(0<<3)
#define ADC_CONF_CHSEL_A2		(2<<3)

#define ADC_IER_MIEN			0x00000001
#define ADC_IER_KPEIEN		0x00000002
#define ADC_IER_PEDEIEN		0x00000004
#define ADC_IER_WKTIEN		0x00000008
#define ADC_IER_WKPIEN		0x00000010
#define ADC_IER_KPUEIEN		0x00000020
#define ADC_IER_PEUEIEN		0x00000040

#define ADC_ISR_MF			0x00000001
#define ADC_ISR_KPEF		0x00000002
#define ADC_ISR_PEDEF		0x00000004
#define ADC_ISR_KPUEF		0x00000008
#define ADC_ISR_PEUEF		0x00000010
#define ADC_ISR_TF			0x00000100
#define ADC_ISR_ZF			0x00000200
#define ADC_ISR_NACF		0x00000400
#define ADC_ISR_VBF			0x00000800
#define ADC_ISR_KPCF		0x00001000
#define ADC_ISR_SELFTF	0x00002000
#define ADC_ISR_INTKP	  0x00010000
#define ADC_ISR_INTTC	  0x00020000

#define ADC_WKISR_WKPEF		0x00000001
#define ADC_WKISR_WPEDEF	0x00000002

#endif