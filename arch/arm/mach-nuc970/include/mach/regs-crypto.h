/*
 * arch/arm/mach-nuc970/include/mach/regs-crypto.h
 *
 * Copyright (c) 2014 Nuvoton technology corporation
 * All rights reserved.
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

#ifndef __ASM_ARM_REGS_IPSEC_H
#define __ASM_ARM_REGS_IPSEC_H

struct nuc970_crypto_regs
{
	u32 CRPT_INTEN;				/*!< Offset: 0x0000   Crypto Interrupt Enable Control Register           */ 
	u32 CRPT_INTSTS;			/*!< Offset: 0x0004   Crypto Interrupt Flag                              */ 
	u32 CRPT_PRNG_CTL;          /*!< Offset: 0x0008   PRNG Control Register                              */
	u32 CRPT_PRNG_SEED;			/*!< Offset: 0x000C   Seed for PRNG                                      */
	u32 CRPT_PRNG_KEY0;			/*!< Offset: 0x0010   PRNG Generated Key0                                */
	u32 CRPT_PRNG_KEY1;			/*!< Offset: 0x0014   PRNG Generated Key1                                */
	u32 CRPT_PRNG_KEY2;			/*!< Offset: 0x0018   PRNG Generated Key2                                */
	u32 CRPT_PRNG_KEY3;			/*!< Offset: 0x001C   PRNG Generated Key3                                */
	u32 CRPT_PRNG_KEY4;			/*!< Offset: 0x0020   PRNG Generated Key4                                */
	u32 CRPT_PRNG_KEY5;			/*!< Offset: 0x0024   PRNG Generated Key5                                */
	u32 CRPT_PRNG_KEY6;			/*!< Offset: 0x0028   PRNG Generated Key6                                */
	u32 CRPT_PRNG_KEY7;			/*!< Offset: 0x002C   PRNG Generated Key7                                */
	u32 RESERVED1[8];		    /*!< Offset: 0x0030 ~ 0x004C   Reserved                                  */
	u32 CRPT_AES_FDBCK0;		/*!< Offset: 0x0050   AES Engine Output Feedback Data after Cryptographic Operation */
	u32 CRPT_AES_FDBCK1;		/*!< Offset: 0x0054   AES Engine Output Feedback Data after Cryptographic Operation */
	u32 CRPT_AES_FDBCK2;		/*!< Offset: 0x0058   AES Engine Output Feedback Data after Cryptographic Operation */
	u32 CRPT_AES_FDBCK3;		/*!< Offset: 0x005C   AES Engine Output Feedback Data after Cryptographic Operation */
	u32 CRPT_TDES_FDBCKH;		/*!< Offset: 0x0060   Triple DES engine Output higher feedback data after cryptographic operation */
	u32 CRPT_TDES_FDBCKL;		/*!< Offset: 0x0064   Triple DES engine Output lower feedback data after cryptographic operation */
	u32 RESERVED2[38];		    /*!< Offset: 0x0068 ~ 0x00FC   Reserved                                  */
	u32 CRPT_AES_CTL;			/*!< Offset: 0x0100   AES engine setting                                 */
	u32 CRPT_AES_STS;			/*!< Offset: 0x0104   AES engine flag                                    */
	u32 CRPT_AES_DATIN;			/*!< Offset: 0x0108   AES engine data input port                         */
	u32 CRPT_AES_DATOUT;		/*!< Offset: 0x010C   AES engine data output port                        */
	u32 CRPT_AES0_KEY0;			/*!< Offset: 0x0110   AES0 Key Word 0 Register                           */
	u32 CRPT_AES0_KEY1;			/*!< Offset: 0x0114   AES0 Key Word 1 Register                           */
	u32 CRPT_AES0_KEY2;			/*!< Offset: 0x0118   AES0 Key Word 2 Register                           */
	u32 CRPT_AES0_KEY3;			/*!< Offset: 0x011C   AES0 Key Word 3 Register                           */
	u32 CRPT_AES0_KEY4;			/*!< Offset: 0x0120   AES0 Key Word 4 Register                           */
	u32 CRPT_AES0_KEY5;			/*!< Offset: 0x0124   AES0 Key Word 5 Register                           */
	u32 CRPT_AES0_KEY6;			/*!< Offset: 0x0128   AES0 Key Word 6 Register                           */
	u32 CRPT_AES0_KEY7;			/*!< Offset: 0x012C   AES0 Key Word 7 Register                           */
	u32 CRPT_AES0_IV0;			/*!< Offset: 0x0130   AES0 Initial Vector Word 0 Register                */
	u32 CRPT_AES0_IV1;			/*!< Offset: 0x0134   AES0 Initial Vector Word 1 Register                */
	u32 CRPT_AES0_IV2;			/*!< Offset: 0x0138   AES0 Initial Vector Word 2 Register                */
	u32 CRPT_AES0_IV3;			/*!< Offset: 0x013C   AES0 Initial Vector Word 3 Register                */
	u32 CRPT_AES0_SADDR;		/*!< Offset: 0x0140   AES0 Source Address Register                       */
	u32 CRPT_AES0_DADDR;		/*!< Offset: 0x0144   AES0 Destination Address Register                  */
	u32 CRPT_AES0_CNT;			/*!< Offset: 0x0148   AES0 Byte Count Register                           */
	u32 CRPT_AES1_KEY0;			/*!< Offset: 0x014C   AES1 Key Word 0 Register                           */
	u32 CRPT_AES1_KEY1;			/*!< Offset: 0x0150   AES1 Key Word 1 Register                           */
	u32 CRPT_AES1_KEY2;			/*!< Offset: 0x0154   AES1 Key Word 2 Register                           */
	u32 CRPT_AES1_KEY3;			/*!< Offset: 0x0158   AES1 Key Word 3 Register                           */
	u32 CRPT_AES1_KEY4;			/*!< Offset: 0x015C   AES1 Key Word 4 Register                           */
	u32 CRPT_AES1_KEY5;			/*!< Offset: 0x0160   AES1 Key Word 5 Register                           */
	u32 CRPT_AES1_KEY6;			/*!< Offset: 0x0164   AES1 Key Word 6 Register                           */
	u32 CRPT_AES1_KEY7;			/*!< Offset: 0x0168   AES1 Key Word 7 Register                           */
	u32 CRPT_AES1_IV0;			/*!< Offset: 0x016C   AES1 Initial Vector Word 0 Register                */
	u32 CRPT_AES1_IV1;			/*!< Offset: 0x0170   AES1 Initial Vector Word 1 Register                */
	u32 CRPT_AES1_IV2;			/*!< Offset: 0x0174   AES1 Initial Vector Word 2 Register                */
	u32 CRPT_AES1_IV3;			/*!< Offset: 0x0178   AES1 Initial Vector Word 3 Register                */
	u32 CRPT_AES1_SADDR;		/*!< Offset: 0x017C   AES1 Source Address Register                       */
	u32 CRPT_AES1_DADDR;		/*!< Offset: 0x0180   AES1 Destination Address Register                  */
	u32 CRPT_AES1_CNT;			/*!< Offset: 0x0184   AES1 Byte Count Register                           */
	u32 CRPT_AES2_KEY0;			/*!< Offset: 0x0188   AES2 Key Word 0 Register                           */
	u32 CRPT_AES2_KEY1;			/*!< Offset: 0x018C   AES2 Key Word 1 Register                           */
	u32 CRPT_AES2_KEY2;			/*!< Offset: 0x0190   AES2 Key Word 2 Register                           */
	u32 CRPT_AES2_KEY3;			/*!< Offset: 0x0194   AES2 Key Word 3 Register                           */
	u32 CRPT_AES2_KEY4;			/*!< Offset: 0x0198   AES2 Key Word 4 Register                           */
	u32 CRPT_AES2_KEY5;			/*!< Offset: 0x019C   AES2 Key Word 5 Register                           */
	u32 CRPT_AES2_KEY6;			/*!< Offset: 0x01A0   AES2 Key Word 6 Register                           */
	u32 CRPT_AES2_KEY7;			/*!< Offset: 0x01A4   AES2 Key Word 7 Register                           */
	u32 CRPT_AES2_IV0;			/*!< Offset: 0x01A8   AES2 Initial Vector Word 0 Register                */
	u32 CRPT_AES2_IV1;			/*!< Offset: 0x01AC   AES2 Initial Vector Word 1 Register                */
	u32 CRPT_AES2_IV2;			/*!< Offset: 0x01B0   AES2 Initial Vector Word 2 Register                */
	u32 CRPT_AES2_IV3;			/*!< Offset: 0x01B4   AES2 Initial Vector Word 3 Register                */
	u32 CRPT_AES2_SADDR;		/*!< Offset: 0x01B8   AES2 Source Address Register                       */
	u32 CRPT_AES2_DADDR;		/*!< Offset: 0x01BC   AES2 Destination Address Register                  */
	u32 CRPT_AES2_CNT;			/*!< Offset: 0x01C0   AES2 Byte Count Register                           */
	u32 CRPT_AES3_KEY0;			/*!< Offset: 0x01C4   AES3 Key Word 0 Register                           */
	u32 CRPT_AES3_KEY1;			/*!< Offset: 0x01C8   AES3 Key Word 1 Register                           */
	u32 CRPT_AES3_KEY2;			/*!< Offset: 0x01CC   AES3 Key Word 2 Register                           */
	u32 CRPT_AES3_KEY3;			/*!< Offset: 0x01D0   AES3 Key Word 3 Register                           */
	u32 CRPT_AES3_KEY4;			/*!< Offset: 0x01D4   AES3 Key Word 4 Register                           */
	u32 CRPT_AES3_KEY5;			/*!< Offset: 0x01D8   AES3 Key Word 5 Register                           */
	u32 CRPT_AES3_KEY6;			/*!< Offset: 0x01DC   AES3 Key Word 6 Register                           */
	u32 CRPT_AES3_KEY7;			/*!< Offset: 0x01E0   AES3 Key Word 7 Register                           */
	u32 CRPT_AES3_IV0;			/*!< Offset: 0x01E4   AES3 Initial Vector Word 0 Register                */
	u32 CRPT_AES3_IV1;			/*!< Offset: 0x01E8   AES3 Initial Vector Word 1 Register                */
	u32 CRPT_AES3_IV2;			/*!< Offset: 0x01EC   AES3 Initial Vector Word 2 Register                */
	u32 CRPT_AES3_IV3;			/*!< Offset: 0x01F0   AES3 Initial Vector Word 3 Register                */
	u32 CRPT_AES3_SADDR;		/*!< Offset: 0x01F4   AES3 Source Address Register                       */
	u32 CRPT_AES3_DADDR;		/*!< Offset: 0x01F8   AES3 Destination Address Register                  */
	u32 CRPT_AES3_CNT;			/*!< Offset: 0x01FC   AES3 Byte Count Register                           */
	u32 CRPT_TDES_CTL;			/*!< Offset: 0x0200   3DES engine parameter  setting                     */
	u32 CRPT_TDES_STS;		    /*!< Offset: 0x0204   3DES flag setting                                  */
	u32 CRPT_TDES0_KEY1H;	    /*!< Offset: 0x0208   DES/3DES Key 1 Higher Word Register                */
	u32 CRPT_TDES0_KEY1L;	    /*!< Offset: 0x020C   DES/3DES Key 1 Lower Word Register                 */
	u32 CRPT_TDES0_KEY2H;	    /*!< Offset: 0x0210   3DES Key 2 Higher Word Register                    */
	u32 CRPT_TDES0_KEY2L;	    /*!< Offset: 0x0214   3DES Key 2 Lower Word Register                     */
	u32 CRPT_TDES0_KEY3H;	    /*!< Offset: 0x0218   3DES Key 3 Higher Word Register                    */
	u32 CRPT_TDES0_KEY3L;	    /*!< Offset: 0x021C   3DES Key 3 Lower Word Register                     */
	u32 CRPT_TDES0_IVH;		    /*!< Offset: 0x0220   3DES Initial Vector Higher Word Register           */
	u32 CRPT_TDES0_IVL;			/*!< Offset: 0x0224   3DES Initial Vector Lower Word Register            */
	u32 CRPT_TDES0_SADDR;		/*!< Offset: 0x0228   DES/3DES Source Address Register                   */
	u32 CRPT_TDES0_DADDR;		/*!< Offset: 0x022C   DES/3DES Destination Address Register              */
	u32 CRPT_TDES0_CNT;			/*!< Offset: 0x0230   DES/3DES Block Count Register                      */
	u32 CRPT_TDES_DATIN;		/*!< Offset: 0x0234   DES/3DES engine Input Word data Register           */
	u32 CRPT_TDES_DATOUT;		/*!< Offset: 0x0238   DES/3DES engine Output Word data Register          */
	u32 RESERVED3[3];		    /*!< Offset: 0x023C ~ 0x0244   Reserved                                  */
	u32 CRPT_TDES1_KEY1H;		/*!< Offset: 0x0248   DES/3DES Key 1 Higher Word Register                */
	u32 CRPT_TDES1_KEY1L; 		/*!< Offset: 0x024C   DES/3DES Key 1 Lower Word Register                 */
	u32 CRPT_TDES1_KEY2H;		/*!< Offset: 0x0250   3DES Key 2 Higher Word Register                    */
	u32 CRPT_TDES1_KEY2L;		/*!< Offset: 0x0254   3DES Key 2 Lower Word Register                     */
	u32 CRPT_TDES1_KEY3H;		/*!< Offset: 0x0258   3DES Key 3 Higher Word Register                    */
	u32 CRPT_TDES1_KEY3L;		/*!< Offset: 0x025C   3DES Key 3 Lower Word Register                     */
	u32 CRPT_TDES1_IVH;		    /*!< Offset: 0x0260   3DES Initial Vector Higher Word Register           */
	u32 CRPT_TDES1_IVL;		    /*!< Offset: 0x0264   3DES Initial Vector Lower Word Register            */
	u32 CRPT_TDES1_SADDR;	    /*!< Offset: 0x0268   DES/3DES Source Address Register                   */
	u32 CRPT_TDES1_DADDR;	    /*!< Offset: 0x026C   DES/3DES Destination Address Register              */
	u32 CRPT_TDES1_CNT;		    /*!< Offset: 0x0270   DES/3DES Block Count Register                      */
	u32 RESERVED4[5];		    /*!< Offset: 0x0274 ~ 0x0284   Reserved                                  */
	u32 CRPT_TDES2_KEY1H;		/*!< Offset: 0x0288   DES/3DES Key 1 Higher Word Register                */
	u32 CRPT_TDES2_KEY1L;		/*!< Offset: 0x028C   DES/3DES Key 1 Lower Word Register                 */
	u32 CRPT_TDES2_KEY2H;		/*!< Offset: 0x0290   3DES Key 2 Higher Word Register                    */
	u32 CRPT_TDES2_KEY2L;		/*!< Offset: 0x0294   3DES Key 2 Lower Word Register                     */
	u32 CRPT_TDES2_KEY3H;		/*!< Offset: 0x0298   3DES Key 3 Higher Word Register                    */
	u32 CRPT_TDES2_KEY3L;		/*!< Offset: 0x029C   3DES Key 3 Lower Word Register                     */
	u32 CRPT_TDES2_IVH;		    /*!< Offset: 0x02A0   3DES Initial Vector Higher Word Register           */
	u32 CRPT_TDES2_IVL;		    /*!< Offset: 0x02A4   3DES Initial Vector Lower Word Register            */
	u32 CRPT_TDES2_SADDR;	    /*!< Offset: 0x02A8   DES/3DES Source Address Register                   */
	u32 CRPT_TDES2_DADDR;	    /*!< Offset: 0x02AC   DES/3DES Destination Address Register              */
	u32 CRPT_TDES2_CNT;		    /*!< Offset: 0x02B0   DES/3DES Block Count Register                      */
	u32 RESERVED5[5];		    /*!< Offset: 0x02B4 ~ 0x02C4   Reserved                                  */
	u32 CRPT_TDES3_KEY1H;		/*!< Offset: 0x02C8   DES/3DES Key 1 Higher Word Register                */
	u32 CRPT_TDES3_KEY1L;		/*!< Offset: 0x02CC   DES/3DES Key 1 Lower Word Register                 */
	u32 CRPT_TDES3_KEY2H;		/*!< Offset: 0x02D0   3DES Key 2 Higher Word Register                    */
	u32 CRPT_TDES3_KEY2L;		/*!< Offset: 0x02D4   3DES Key 2 Lower Word Register                     */
	u32 CRPT_TDES3_KEY3H;		/*!< Offset: 0x02D8   3DES Key 3 Higher Word Register                    */
	u32 CRPT_TDES3_KEY3L;		/*!< Offset: 0x02DC   3DES Key 3 Lower Word Register                     */
	u32 CRPT_TDES3_IVH;		    /*!< Offset: 0x02E0   3DES Initial Vector Higher Word Register           */
	u32 CRPT_TDES3_IVL;		    /*!< Offset: 0x02E4   3DES Initial Vector Lower Word Register            */
	u32 CRPT_TDES3_SADDR;	    /*!< Offset: 0x02E8   DES/3DES Source Address Register                   */
	u32 CRPT_TDES3_DADDR;	    /*!< Offset: 0x02EC   DES/3DES Destination Address Register              */
	u32 CRPT_TDES3_CNT;		    /*!< Offset: 0x02F0   DES/3DES Block Count Register                      */
	u32 RESERVED6[3];		    /*!< Offset: 0x02F4 ~ 0x02FC   Reserved                                  */
	u32 CRPT_HMAC_CTL;		    /*!< Offset: 0x0300   HMAC/SHA engine parameter setting                  */
	u32 CRPT_HMAC_STS;		    /*!< Offset: 0x0304   HMAC/SHA status flag                               */
	u32 CRPT_HMAC_DGST[16];		/*!< Offset: 0x0308   HMAC/SHA digest message word 0~15                  */
	u32 CRPT_HMAC_KEYCNT;		/*!< Offset: 0x0348   HMAC/SHA key byte length                           */
	u32 CRPT_HMAC_SADDR;	    /*!< Offset: 0x034C   HMAC DMA Source Address Register                   */
	u32 CRPT_HMAC_DMACNT;		/*!< Offset: 0x0350   HMAC DMA count                                     */
	u32 CRPT_HMAC_DATIN;		/*!< Offset: 0x0354   HMAC software write to engine port                 */
};

struct nuc970_aes_regs
{
	u32 key[8];
	u32 iv[4];
	u32 src_addr;
	u32 dst_addr;
	u32 count;
};

struct nuc970_tdes_regs
{
	u32 key[6];
	u32 iv[2];
	u32 src_addr;
	u32 dst_addr;
	u32 count;
};


/********************* Bit definition of CRPT_INTEN  ************************/
#define AESIEN						((u32)0x00000001)			/*!<AES DMA finish interrupt flag enable */
#define AESERRIEN					((u32)0x00000002)			/*!<AES error flag enable */
#define TDESIEN						((u32)0x00000100)			/*!<TDES interrupt source enable */
#define TDESERRIEN					((u32)0x00000200)			/*!<TDES error flag enable */
#define PRNGIEN						((u32)0x00010000)			/*!<PRNG finish interrupt enable */
#define HMACIEN						((u32)0x01000000)			/*!<HMAC interrupt enable */
#define HMACERRIEN					((u32)0x02000000)			/*!<HMAC error interrupt enable */

/********************* Bit definition of CRPT_INTSTS *******************/
#define AESIF						((u32)0x00000001)			/*!<AES finish interrupt flag */
#define AESERRIF					((u32)0x00000002)			/*!<AES error flag */
#define TDESIF						((u32)0x00000100)			/*!<TDES finish interrupt flag */
#define TDESERRIF					((u32)0x00000200)			/*!<TDES error flag */
#define PRNGIF						((u32)0x00010000)			/*!<PRNG finish interrupt flag */
#define HMACIF						((u32)0x01000000)			/*!<HMAC finish interrupt flag */
#define HMACERRIF					((u32)0x02000000)		/*!<HMAC error flag */

/********************* Bit definition of CRPT_PRNG_CTL *******************/
#define PRNG_START					((u32)0x00000001)			/*!<Write 1 start PRNG engine generate new KEY and store KEYx register. */
#define SEEDRLD						((u32)0x00000002)			/*!1: reload new seed; 0: generate key base original seed. */
#define PRNG_KEYSZ_MASK				((u32)0x0000000C)			/*!<PRNG KEY SIZE bit mask */
#define PRNG_KEYSZ_64			  	((u32)0x00000000)			/*!<PRNG 64 bits key */
#define PRNG_KEYSZ_128  			((u32)0x00000004)			/*!<PRNG 128 bits key */
#define PRNG_KEYSZ_192  			((u32)0x00000008)			/*!<PRNG 192 bits key */
#define PRNG_KEYSZ_256  			((u32)0x0000000C)			/*!<PRNG 256 bits key */
#define PRNG_BUSY					((u32)0x00000100)			/*!<read only, 1:  indicate the PRNG engine under generating KEY */


/********************* Bit definition of CRPT_AES_CTL *******************/
#define AES_START					((u32)0x00000001)			/*!<Write 1, AES engine starting. AES_BUSY flag will be set. */
#define AES_STOP					((u32)0x00000002)			/*!<Write 1,  stop  AES engine instantly. */
#define AES_KEYSZ_MASK 				((u32)0x0000000C)			/*!<AES KEY_SIZE bit mask */
#define AES_KEYSZ_128				((u32)0x00000000)			/*!<AES 128 bits key */
#define AES_KEYSZ_192				((u32)0x00000004)			/*!<AES 192 bits key */
#define AES_KEYSZ_256				((u32)0x00000008)			/*!<AES 256 bits key */
#define AES_EXTERNAL_KEY			((u32)0x00000010)			/*!<1: AES KEY use extend key. 0:  AES KEY source form  AES_KEYx registers. */
#define AES_DMALAST					((u32)0x00000020)			/*!<Index current operation is  last one. */
#define AES_DMACSCAD				((u32)0x00000040)			/*!<Cascade AES encrypt/decrypt result */
#define AES_DMAEN					((u32)0x00000080)			/*!<enable AES DMA engine */
#define AES_OPMODE_MASK		 		((u32)0x0000FF00)			/*!<AES engine operation mode bit mask */
#define AES_ECB_MODE				((u32)0x00000000)			/*!<AES ECB mode */
#define AES_CBC_MODE				((u32)0x00000100)			/*!<AES CBC mode */
#define AES_CFB_MODE				((u32)0x00000200)			/*!<AES CFB mode */
#define AES_OFB_MODE				((u32)0x00000300)			/*!<AES OFB mode */
#define AES_CTR_MODE				((u32)0x00000400)			/*!<AES CTR mode */
#define AES_CBCCS1_MODE				((u32)0x00001000)			/*!<AES CBC CS1 mode */
#define AES_CBCCS2_MODE				((u32)0x00001100)			/*!<AES CBC CS2 mode */
#define AES_CBCCS3_MODE				((u32)0x00001200)			/*!<AES CBC CS3 mode */
#define AES_ENCRYPT					((u32)0x00010000)			/*!<AES engine execute encryption */
#define AES_DECRYPT					((u32)0x00000000)			/*!<AES engine execute decryption */
#define AES_OUTSWAP					((u32)0x00400000)			/*!<AES engine output data transform */
#define AES_INSWAP					((u32)0x00800000)			/*!<AES engine input data transform */
#define AES_CHANNEL_MASK			((u32)0x03000000)			/*!<AES working channel bit mask */
#define AES_CHANNEL0				((u32)0x00000000)			/*!<AES Current  control register  setting on channel 0 */
#define AES_CHANNEL1				((u32)0x01000000)			/*!<AES Current  control register  setting on channel 0 */
#define AES_CHANNEL2				((u32)0x02000000)			/*!<AES Current  control register  setting on channel 0 */
#define AES_CHANNEL3				((u32)0x03000000)			/*!<AES Current  control register  setting on channel 0 */
#define AES_KEYPRT_MASK				((u32)0xFC000000)			/*!<AES unprotect key bit mask */
#define AES_KEY_UNPRT        		((u32)0x58000000)			/*!<Set AES unprotect key */
#define AES_KEY_PRT					((u32)0x80000000)			/*!<AES key protect */

/********************* Bit definition of CRPT_AES_STS *******************/
#define AES_BUSY					((u32)0x00000001)			/*!<AES engine under archiving */
#define AES_INBUFEMPTY				((u32)0x00000100)			/*!<AES input buffer empty */
#define AES_INBUFFULL				((u32)0x00000200)			/*!<AES input buffer full */
#define AES_INBUFERR				((u32)0x00000400)			/*!<ERROR during feed AES engine data. */
#define AES_CNTERR					((u32)0x00001000)			/*!<AES_CNT setting error */
#define AES_OUTBUFEMPTY				((u32)0x00010000)			/*!<AES output buffer empty */
#define AES_OUTBUFFULL				((u32)0x00020000)			/*!<AES output buffer full */
#define AES_OUTBUFERR				((u32)0x00040000)			/*!<ERROR during get AES engine result */
#define AES_BUSERR					((u32)0x00100000)			/*!<AES bus error */

/********************* Bit definition of CRPT_TDES_CTL *******************/
#define TDES_START					((u32)0x00000001)			/*!<Write 1, TDES engine starting. AES_BUSY flag will be set. */
#define TDES_STOP					((u32)0x00000002)			/*!<Write 1 , stop TDES engine instantly. */
#define TDES_TMODE			 		((u32)0x00000004)			/*!<Tripple DES mode */
#define TDES_3KEYS			 		((u32)0x00000008)			/*!<enable triple KEY in TDES engine */
#define TDES_EXTERNAL_KEY			((u32)0x00000010)			/*!<TDES KEY use extend key */
#define TDES_DMALAST				((u32)0x00000020)			/*!<TDES last DMA cascade round */
#define TDES_DMACSCAD				((u32)0x00000040)			/*!<TDES DMA cascade mode */
#define TDES_DMAEN					((u32)0x00000080)			/*!<enable TDES_DMA engine */
#define TDES_OPMODE_MASK			((u32)0x00000700)			/*!<TDES engine operation mode bit mask */
#define TDES_ECB_MODE				((u32)0x00000000)			/*!<ECB mode */
#define TDES_CBC_MODE				((u32)0x00000100)			/*!<CBC mode */
#define TDES_CFB_MODE				((u32)0x00000200)			/*!<CFB mode */
#define TDES_OFB_MODE				((u32)0x00000300)			/*!<OFB mode */
#define TDES_CTR_MODE				((u32)0x00000400)			/*!<CTR mode */
#define TDES_ENCRYPT				((u32)0x00010000)			/*!<TDES engine execute encryption */
#define TDES_DECRYPT				((u32)0x00000000)			/*!<TDES engine execute decryption */
#define TDES_BLKSWAP				((u32)0x00200000)			/*!<TDES engine Swap High/Low word */
#define TDES_OUTSWAP				((u32)0x00400000)			/*!<TDES engine output data transform */
#define TDES_INSWAP					((u32)0x00800000)			/*!<TDES engine input data transform */
#define TDES_CHANNEL_MASK			((u32)0x03000000)			/*!<TDES working channel bit mask */
#define TDES_CHANNEL_0				((u32)0x00000000)			/*!<TDES Current  control register  setting on channel 0 */
#define TDES_CHANNEL_1				((u32)0x01000000)			/*!<TDES Current  control register  setting on channel 0 */
#define TDES_CHANNEL_2				((u32)0x02000000)			/*!<TDES Current  control register  setting on channel 0 */
#define TDES_CHANNEL_3				((u32)0x03000000)			/*!<TDES Current  control register  setting on channel 0 */
#define TDES_KEYPRT_MASK			((u32)0xFC000000)			/*!<TDES unprotect key bit mask */
#define TDES_KEYUNPRT		       	((u32)0x58000000)			/*!<Set TDES unprotect key */
#define TDES_KEYPRT					((u32)0x80000000)			/*!<TDES key protect */

/********************* Bit definition of CRPT_TDES_STS *******************/
#define TDES_BUSY					((u32)0x00000001)			/*!<TDES engine under archiving */
#define TDES_INBUFEMPTY				((u32)0x00000100)	/*!<TDES input buffer empty */
#define TDES_INBUFFULL				((u32)0x00000200)	/*!<TDES input buffer full */
#define TDES_INBUFERR				((u32)0x00000400)			/*!<ERROR during feed TDES engine data. */
#define TDES_OUTBUFEMPTY			((u32)0x00010000)	/*!<TDES output buffer empty */
#define TDES_OUTBUFFULL				((u32)0x00020000)	/*!<TDES output buffer full */
#define TDES_OUTBUFERR				((u32)0x00040000)	/*!<ERROR during get TDES engine result */
#define TDES_BUSERR					((u32)0x00100000)			/*!<TDES bus error */

/********************* Bit definition of HMAC_CTL ********************/
#define HMAC_START					((u32)0x00000001)			/*!<HMAC engine starting */
#define HMAC_STOP					((u32)0x00000002)			/*!<HMAC engine stop */
#define HMAC_EN						((u32)0x00000010)			/*!<execute HMAC function */
#define HMAC_DMALAST				((u32)0x00000020)			/*!<last DMA cascade round */
#define HMAC_DMAEN					((u32)0x00000080)			/*!<enable HMAC_DMA engine */
#define HMAC_OPMODE_MASK			((u32)0x00000700)			/*!<HMAC engine operation modes mask */
#define HMAC_SHA1					((u32)0x00000000)			/*!<SHA1 */
#define HMAC_SHA224					((u32)0x00000500)			/*!<SHA224 */
#define HMAC_SHA256					((u32)0x00000400)			/*!<SHA256 */
#define HMAC_SHA384					((u32)0x00000700)			/*!<SHA384 */
#define HMAC_SHA512					((u32)0x00000600)			/*!<SHA512 */
#define HMAC_OUTSWAP				((u32)0x00400000)			/*!<HMAC engine output data transform */
#define HMAC_INSWAP					((u32)0x00800000)			/*!<HMAC engine input data transform */

/********************* Bit definition of HMAC_FLAG *******************/
#define HMAC_BUSY					((u32)0x00000001)			/*!<HMAC engine busy */
#define HMAC_DMABUSY				((u32)0x00000002)			/*!<HMAC engine is under active */
#define HMAC_DMAERR					((u32)0x00000100)			/*!<HMAC DMA error */
#define HMAC_DINREQ					((u32)0x00010000)			/*!<HMAC_SOFTWARE mode Data input  request */


#endif /* __ASM_ARM_REGS_IPSEC_H */

