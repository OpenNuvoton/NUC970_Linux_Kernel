/*
 * arch/arm/mach-nuc970/include/mach/regs-adc.h
 *
 * Copyright (c) 2012 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#ifndef __ASM_ARCH_REGS_ADC_H
#define __ASM_ARCH_REGS_ADC_H

#define ADC_BA	NUC970_VA_ADC	/* ADC Control */

#define REG_ADC_CTL					(ADC_BA + 0x000)	/* ADC Control  */	
#define REG_ADC_CONF				(ADC_BA + 0x004)	/* ADC Configure  */	
#define REG_ADC_IER					(ADC_BA + 0x008)	/* ADC Interrupt Enable Register */	
#define REG_ADC_ISR					(ADC_BA + 0x00C) /* ADC Interrupt Status Register */
#define REG_ADC_WKISR       (ADC_BA + 0x010) /* ADC Wake Up Interrupt Status Register */
#define REG_ADC_XYDATA      (ADC_BA + 0x020) /* ADC Touch X,Y Position Data  */
#define REG_ADC_ZDATA       (ADC_BA + 0x024) /* ADC Touch Z Pressure Data  */
#define REG_ADC_DATA        (ADC_BA + 0x028) /* ADC Normal Conversion Data  */
#define REG_ADC_VBADATA     (ADC_BA + 0x02C) /* ADC Battery Detection Data  */
#define REG_ADC_KPDATA      (ADC_BA + 0x030) /* ADC Battery Detection Data  */
#define REG_ADC_SELFDATA    (ADC_BA + 0x034) /* ADC Self-Test Data  */
#define REG_ADC_XYSORT0     (ADC_BA + 0x1F4) /* ADC Touch XY Position Mean Value Sort 0 */
#define REG_ADC_XYSORT1     (ADC_BA + 0x1F8) /* ADC Touch XY Position Mean Value Sort 1 */
#define REG_ADC_XYSORT2     (ADC_BA + 0x1FC) /* ADC Touch XY Position Mean Value Sort 2 */
#define REG_ADC_XYSORT3     (ADC_BA + 0x200) /* ADC Touch XY Position Mean Value Sort 3 */
#define REG_ADC_ZSORT0      (ADC_BA + 0x204) /* ADC Touch Z Position Mean Value Sort 0 */
#define REG_ADC_ZSORT1      (ADC_BA + 0x208) /* ADC Touch Z Position Mean Value Sort 1 */
#define REG_ADC_ZSORT2      (ADC_BA + 0x20C) /* ADC Touch Z Pressure Mean Value Sort 2 */
#define REG_ADC_ZSORT3      (ADC_BA + 0x210) /* ADC Touch Z Pressure Mean Value Sort 3 */
#define REG_ADC_MTMULCK     (ADC_BA + 0x220) /* ADC Manual Test Mode Unlock */
#define REG_ADC_MTCONF      (ADC_BA + 0x224) /* ADC Manual Test Mode Configure  */
#define REG_ADC_MTCTL       (ADC_BA + 0x228) /* ADC Manual Test Mode Control */
#define REG_ADC_ADCAII      (ADC_BA + 0x22C) /* ADC Analog Interface Information */
#define REG_ADC_ADCAIIRLT   (ADC_BA + 0x230) /* ADC Analog Interface Information Result */

#if 0
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

#endif /*  __ASM_ARCH_REGS_CAP_H */