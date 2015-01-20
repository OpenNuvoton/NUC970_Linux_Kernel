/*
 * arch/arm/mach-nuc970/include/mach/irqs.h
 *
 * Copyright (c) 2014 Nuvoton technology corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

#ifndef __ASM_ARCH_IRQS_H
#define __ASM_ARCH_IRQS_H

/*
 * we keep the first set of CPU IRQs out of the range of
 * the ISA space, so that the PC104 has them to itself
 * and we don't end up having to do horrible things to the
 * standard ISA drivers....
 *
 */

#define NUC970_IRQ(x)	(x)

/* Main cpu interrupts */

#define IRQ_WDT		NUC970_IRQ(1)
#define IRQ_WWDT	NUC970_IRQ(2)
#define IRQ_LVD		NUC970_IRQ(3)
#define IRQ_EXT0	NUC970_IRQ(4)
#define IRQ_EXT1	NUC970_IRQ(5)
#define IRQ_EXT2	NUC970_IRQ(6)
#define IRQ_EXT3	NUC970_IRQ(7)
#define IRQ_EXT4	NUC970_IRQ(8)
#define IRQ_EXT5	NUC970_IRQ(9)
#define IRQ_EXT6	NUC970_IRQ(10)
#define IRQ_EXT7	NUC970_IRQ(11)
#define IRQ_ACTL	NUC970_IRQ(12)
#define IRQ_LCD		NUC970_IRQ(13)
#define IRQ_CAP		NUC970_IRQ(14)
#define IRQ_RTC		NUC970_IRQ(15)
#define IRQ_TMR0	NUC970_IRQ(16)
#define IRQ_TMR1	NUC970_IRQ(17)
#define IRQ_ADC		NUC970_IRQ(18)
#define IRQ_EMC0RX	NUC970_IRQ(19)
#define IRQ_EMC1RX	NUC970_IRQ(20)
#define IRQ_EMC0TX	NUC970_IRQ(21)
#define IRQ_EMC1TX	NUC970_IRQ(22)
#define IRQ_EHCI	NUC970_IRQ(23)
#define IRQ_OHCI	NUC970_IRQ(24)
#define IRQ_GDMA0	NUC970_IRQ(25)
#define IRQ_GDMA1	NUC970_IRQ(26)
#define IRQ_SDH		NUC970_IRQ(27)
#define IRQ_FMI		NUC970_IRQ(28)
#define IRQ_UDC		NUC970_IRQ(29)
#define IRQ_TMR2	NUC970_IRQ(30)
#define IRQ_TMR3	NUC970_IRQ(31)
#define IRQ_TMR4	NUC970_IRQ(32)
#define IRQ_JPEG	NUC970_IRQ(33)
#define IRQ_GE2D	NUC970_IRQ(34)
#define IRQ_CRYPTO	NUC970_IRQ(35)
#define IRQ_UART0	NUC970_IRQ(36)
#define IRQ_UART1	NUC970_IRQ(37)
#define IRQ_UART2	NUC970_IRQ(38)
#define IRQ_UART4	NUC970_IRQ(39)
#define IRQ_UART6	NUC970_IRQ(40)
#define IRQ_UART8	NUC970_IRQ(41)
#define IRQ_UART10	NUC970_IRQ(42)
#define IRQ_UART3	NUC970_IRQ(43)
#define IRQ_UART5	NUC970_IRQ(44)
#define IRQ_UART7	NUC970_IRQ(45)
#define IRQ_UART9	NUC970_IRQ(46)
#define IRQ_ETIMER0	NUC970_IRQ(47)
#define IRQ_ETIMER1	NUC970_IRQ(48)
#define IRQ_ETIMER2	NUC970_IRQ(49)
#define IRQ_ETIMER3	NUC970_IRQ(50)
#define IRQ_SPI0	NUC970_IRQ(51)
#define IRQ_SPI1	NUC970_IRQ(52)
#define IRQ_I2C0	NUC970_IRQ(53)
#define IRQ_I2C1	NUC970_IRQ(54)
#define IRQ_SMC0	NUC970_IRQ(55)
#define IRQ_SMC1	NUC970_IRQ(56)
#define IRQ_GPIO	NUC970_IRQ(57)
#define IRQ_CAN0	NUC970_IRQ(58)
#define IRQ_CAN1	NUC970_IRQ(59)
#define IRQ_PWM		NUC970_IRQ(60)
#define IRQ_KPI		NUC970_IRQ(61)

#ifndef CONFIG_GPIO_NUC970
#define NR_IRQS		(IRQ_KPI + 1)
#else
#define EXT0_BASE 100
#define IRQ_EXT0_H0		NUC970_IRQ(EXT0_BASE+0)
#define IRQ_EXT1_H1		NUC970_IRQ(EXT0_BASE+1)
#define IRQ_EXT2_H2		NUC970_IRQ(EXT0_BASE+2)
#define IRQ_EXT3_H3		NUC970_IRQ(EXT0_BASE+3)
#define IRQ_EXT4_H4		NUC970_IRQ(EXT0_BASE+4)
#define IRQ_EXT5_H5		NUC970_IRQ(EXT0_BASE+5)
#define IRQ_EXT6_H6		NUC970_IRQ(EXT0_BASE+6)
#define IRQ_EXT7_H7		NUC970_IRQ(EXT0_BASE+7)

#define IRQ_EXT0_F11	NUC970_IRQ(EXT0_BASE+8)
#define IRQ_EXT1_F12	NUC970_IRQ(EXT0_BASE+9)
#define IRQ_EXT2_F13	NUC970_IRQ(EXT0_BASE+10)
#define IRQ_EXT3_F14	NUC970_IRQ(EXT0_BASE+11)
#define IRQ_EXT4_F15	NUC970_IRQ(EXT0_BASE+12)
#define IRQ_EXT5_G15	NUC970_IRQ(EXT0_BASE+13)
#define IRQ_EXT6_I1	  NUC970_IRQ(EXT0_BASE+14)
#define IRQ_EXT7_I2 	NUC970_IRQ(EXT0_BASE+15)

#define IRQ_GPIO_START 	NUC970_IRQ(NUC970_IRQ(0x100))
#define IRQ_GPIO_END 		NUC970_IRQ(NUC970_IRQ(0x100+0x130))
#define NR_IRQS		(IRQ_GPIO_END + 1)
#endif




#endif /* __ASM_ARCH_IRQ_H */
