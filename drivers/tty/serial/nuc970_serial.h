/*
 *  linux/drivers/serial/nuc970_serial.h
 *
 *  NUC970 serial driver header file
 *
 *
 *  Copyright (C) 2012 Nuvoton Technology Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __NUC970_SERIAL_H__
#define __NUC970_SERIAL_H__

#define UART_REG_RBR	0x00
#define UART_REG_THR	0x00

#define UART_REG_IER	0x04
#define RDA_IEN			0x00000001
#define THRE_IEN		0x00000002
#define RTO_IEN			0x00000010
#define TIME_OUT_EN		0x00000800

#define UART_REG_FCR	0x08
#define RFR			0x00000002
#define TFR			0x00000004

#define UART_REG_LCR	0x0C
#define	NSB			0x00000004
#define PBE			0x00000008
#define EPE			0x00000010
#define SPE			0x00000020
#define BCB			0x00000040

#define UART_REG_MCR	0x10
#define UART_REG_MSR	0x14

#define UART_REG_FSR	0x18
#define RX_OVER_IF		0x00000001
#define PEF			0x00000010
#define FEF			0x00000020
#define BIF			0x00000040
#define RX_EMPTY		0x00004000
#define TX_EMPTY		0x00400000
#define TX_FULL			0x00800000
#define RX_FULL			0x00008000
#define TE_FLAG			0x10000000

#define UART_REG_ISR	0x1C
#define RDA_IF			0x00000001
#define THRE_IF			0x00000002
#define TOUT_IF			0x00000010

#define UART_REG_TOR	0x20
#define UART_REG_BAUD	0x24

#define UART_REG_IRCR	0x28

#define UART_REG_ALT_CSR 0x2C

#define UART_FUN_SEL    0x30
#define FUN_SEL_UART    0x00000000
#define FUN_SEL_LIN     0x00000001
#define FUN_SEL_IrDA    0x00000002
#define FUN_SEL_RS485	0x00000003
#define FUN_SEL_Msk		0x00000007

#endif // __NUC970_SERIAL_H__
