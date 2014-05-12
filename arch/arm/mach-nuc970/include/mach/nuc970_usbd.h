/* linux/arch/arm/mach-nuc970/include/mach/regs_usbd.h
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
 */

 
#ifndef __NUC970_USBD_H
#define __NUC970_USBD_H


#define USBD_DMA_LEN		0x10000
#define USB_HIGHSPEED		2
#define USB_FULLSPEED		1
#define EPSTADDR        	0x400
#define CBW_SIZE			64

#define DMA_READ			1
#define DMA_WRITE			2

#define REG_USBD_IRQ_STAT_L             (0x00)
#define REG_USBD_IRQ_ENB_L              (0x08)
#define REG_USBD_IRQ_STAT               (0x10)
#define REG_USBD_IRQ_ENB                (0x14)
#define REG_USBD_OPER                   (0x18)
#define REG_USBD_FRAME_CNT              (0x1c)
#define REG_USBD_ADDR                   (0x20)
#define REG_USBD_TEST                   (0x24)
#define REG_USBD_CEP_DATA_BUF           (0x28)
#define REG_USBD_CEP_CTRL_STAT          (0x2c)
#define REG_USBD_CEP_IRQ_ENB            (0x30)
#define REG_USBD_CEP_IRQ_STAT           (0x34)
#define REG_USBD_IN_TRNSFR_CNT          (0x38)
#define REG_USBD_OUT_TRNSFR_CNT         (0x3c)
#define REG_USBD_CEP_CNT                (0x40)
#define REG_USBD_SETUP1_0               (0x44)
#define REG_USBD_SETUP3_2               (0x48)
#define REG_USBD_SETUP5_4               (0x4c)
#define REG_USBD_SETUP7_6               (0x50)
#define REG_USBD_CEP_START_ADDR         (0x54)
#define REG_USBD_CEP_END_ADDR           (0x58)
#define REG_USBD_DMA_CTRL_STS           (0x5c)
#define REG_USBD_DMA_CNT                (0x60)

#define REG_USBD_EPA_DATA_BUF           (0x64)
#define REG_USBD_EPA_IRQ_STAT           (0x68)
#define REG_USBD_EPA_IRQ_ENB            (0x6c)
#define REG_USBD_EPA_DATA_CNT           (0x70)
#define REG_USBD_EPA_RSP_SC             (0x74)
#define REG_USBD_EPA_MPS                (0x78)
#define REG_USBD_EPA_TRF_CNT            (0x7c)
#define REG_USBD_EPA_CFG                (0x80)
#define REG_USBD_EPA_START_ADDR         (0x84)
#define REG_USBD_EPA_END_ADDR           (0x88)

#define REG_USBD_EPB_DATA_BUF           (0x8c)
#define REG_USBD_EPB_IRQ_STAT           (0x90)
#define REG_USBD_EPB_IRQ_ENB            (0x94)
#define REG_USBD_EPB_DATA_CNT           (0x98)
#define REG_USBD_EPB_RSP_SC             (0x9c)
#define REG_USBD_EPB_MPS                (0xa0)
#define REG_USBD_EPB_TRF_CNT            (0xa4)
#define REG_USBD_EPB_CFG                (0xa8)
#define REG_USBD_EPB_START_ADDR         (0xac)
#define REG_USBD_EPB_END_ADDR           (0xb0)

#define REG_USBD_EPC_DATA_BUF           (0xb4)
#define REG_USBD_EPC_IRQ_STAT           (0xb8)
#define REG_USBD_EPC_IRQ_ENB            (0xbc)
#define REG_USBD_EPC_DATA_CNT           (0xc0)
#define REG_USBD_EPC_RSP_SC             (0xc4)
#define REG_USBD_EPC_MPS                (0xc8)
#define REG_USBD_EPC_TRF_CNT            (0xcc)
#define REG_USBD_EPC_CFG                (0xd0)
#define REG_USBD_EPC_START_ADDR         (0xd4)
#define REG_USBD_EPC_END_ADDR           (0xd8)

#define REG_USBD_EPD_DATA_BUF           (0xdc)
#define REG_USBD_EPD_IRQ_STAT           (0xe0)
#define REG_USBD_EPD_IRQ_ENB            (0xe4)
#define REG_USBD_EPD_DATA_CNT           (0xe8)
#define REG_USBD_EPD_RSP_SC             (0xec)
#define REG_USBD_EPD_MPS                (0xf0)
#define REG_USBD_EPD_TRF_CNT            (0xf4)
#define REG_USBD_EPD_CFG                (0xf8)
#define REG_USBD_EPD_START_ADDR         (0xfc)
#define REG_USBD_EPD_END_ADDR           (0x100)

#define REG_USBD_EPE_DATA_BUF           (0x104)
#define REG_USBD_EPE_IRQ_STAT           (0x108)
#define REG_USBD_EPE_IRQ_ENB            (0x10c)
#define REG_USBD_EPE_DATA_CNT           (0x110)
#define REG_USBD_EPE_RSP_SC             (0x114)
#define REG_USBD_EPE_MPS                (0x118)
#define REG_USBD_EPE_TRF_CNT            (0x11c)
#define REG_USBD_EPE_CFG                (0x120)
#define REG_USBD_EPE_START_ADDR         (0x124)
#define REG_USBD_EPE_END_ADDR           (0x128)

#define REG_USBD_EPF_DATA_BUF           (0x12c)
#define REG_USBD_EPF_IRQ_STAT           (0x130)
#define REG_USBD_EPF_IRQ_ENB            (0x134)
#define REG_USBD_EPF_DATA_CNT           (0x138)
#define REG_USBD_EPF_RSP_SC             (0x13c)
#define REG_USBD_EPF_MPS                (0x140)
#define REG_USBD_EPF_TRF_CNT            (0x144)
#define REG_USBD_EPF_CFG                (0x148)
#define REG_USBD_EPF_START_ADDR         (0x14c)
#define REG_USBD_EPF_END_ADDR           (0x150)
#define REG_USBD_AHB_DMA_ADDR           (0x700)
#define REG_USBD_PHY_CTL                (0x704) 


/*
 * Standard requests
 */
#define USBR_GET_STATUS			0x00
#define USBR_CLEAR_FEATURE		0x01
#define USBR_SET_FEATURE		0x03
#define USBR_SET_ADDRESS		0x05
#define USBR_GET_DESCRIPTOR		0x06
#define USBR_SET_DESCRIPTOR		0x07
#define USBR_GET_CONFIGURATION  0x08
#define USBR_SET_CONFIGURATION	0x09
#define USBR_GET_INTERFACE		0x0A
#define USBR_SET_INTERFACE		0x0B
#define USBR_SYNCH_FRAME		0x0C


//Bit Definitions of IRQ_ENB/STAT register
#define	IRQ_USB_STAT		    0x01
#define IRQ_CEP					0x02
#define IRQ_NCEP				0xfc   

//Definition of Bits in USB_IRQ_STS register
#define USB_SOF					0x01	
#define USB_RST_STS				0x02
#define	USB_RESUME				0x04
#define	USB_SUS_REQ				0x08
#define	USB_HS_SETTLE			0x10
#define	USB_DMA_REQ				0x20
#define USABLE_CLK				0x40
#define USB_FLT_DET				0x100


//Definition of Bits in USB_OPER register
#define USB_GEN_RES             0x1
#define USB_HS		        	0x2
#define USB_CUR_SPD_HS          0x4

//Definition of Bits in CEP_IRQ_STS register
#define CEP_SUPTOK	 			0x0001
#define CEP_SUPPKT				0x0002
#define CEP_OUT_TOK				0x0004
#define CEP_IN_TOK				0x0008
#define CEP_PING_TOK	        0x0010
#define CEP_DATA_TXD	        0x0020
#define CEP_DATA_RXD	        0x0040
#define CEP_NAK_SENT	        0x0080
#define CEP_STALL_SENT	        0x0100
#define CEP_USB_ERR				0x0200
#define CEP_STS_END				0x0400
#define CEP_BUFF_FULL	        0x0800
#define CEP_BUFF_EMPTY	        0x1000

//Definition of Bits in CEP_CTRL_STS register
#define CEP_NAK_CLEAR			0x00  //writing zero clears the nak bit
#define CEP_SEND_STALL			0x02
#define CEP_ZEROLEN				0x04
#define CEP_FLUSH				0x08

//Definition of Bits in EP_IRQ_STS register
#define EP_BUFF_FULL	        0x001
#define EP_BUFF_EMPTY	        0x002
#define EP_SHORT_PKT	        0x004
#define EP_DATA_TXD				0x008
#define EP_DATA_RXD				0x010
#define EP_OUT_TOK				0x020
#define EP_IN_TOK				0x040
#define EP_PING_TOK				0x080
#define EP_NAK_SENT				0x100
#define EP_STALL_SENT	        0x200
#define EP_USB_ERR				0x800
#define EP_BO_SHORT_PKT			0x1000

//Bit Definitons of EP_RSP_SC Register
#define EP_BUFF_FLUSH           0x01
#define EP_MODE                 0x06
#define EP_MODE_AUTO	        0x01
#define EP_MODE_MAN 	        0x02
#define EP_MODE_FLY				0x03
#define EP_TOGGLE				0x8
#define EP_HALT					0x10
#define EP_ZERO_IN              0x20
#define EP_PKT_END              0x40

//Bit Definitons of EP_CFG Register
#define EP_VALID				0x01
#define EP_TYPE					0x06 //2-bit size	
#define EP_TYPE_BLK				0x01
#define EP_TYPE_INT				0x02
#define EP_TYPE_ISO				0x03
#define EP_DIR					0x08
#define EP_NO					0xf0 //4-bit size

/* Define Endpoint feature */
#define Ep_In                   0x01
#define Ep_Out                  0x00


#endif /* __NUC970_USBD_H */
