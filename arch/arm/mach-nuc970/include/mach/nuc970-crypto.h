/*
 * arch/arm/mach-nuc970/include/mach/nuc970-crypto.h
 *
 * Copyright (c) 2018 Nuvoton Technology Corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef _NUC970_CRYPTO_H_
#define _NUC970_CRYPTO_H_

#include <linux/types.h>
#include <linux/ioctl.h>


struct nuc970_crypto_dev {
	struct device  *dev;
	struct nuc970_crypto_regs  *regs;
	struct mutex  aes_lock;
	struct mutex  des_lock;
	struct mutex  sha_lock;
	struct mutex  mtp_lock;

	u8           aes_channels;
	u8           des_channels;

	u32          *aes_inbuf;                /* AES input buffer                          */
	u32          aes_inbuf_size;            /* AES input buffer size                     */
	dma_addr_t   aes_inbuf_dma_addr;        /* physical address of AES input buffer      */
	u32          *aes_outbuf;               /* AES output buffer                         */
	u32          aes_outbuf_size;           /* AES output buffer size                    */
	dma_addr_t   aes_outbuf_dma_addr;       /* physical address of AES output buffer     */

	u32          *des_inbuf;
	dma_addr_t   des_inbuf_dma_addr;
	u32          *des_outbuf;
	dma_addr_t   des_outbuf_dma_addr;

	u32          *hmac_inbuf;               /* SHA/HAMC input buffer                     */
	u32          hmac_inbuf_size;           /* SHA/HMAC input buffer size                */
	dma_addr_t   hmac_inbuf_dma_addr;       /* physical address of SHA/HMAC input buffer */
	u32          sha_len;
};


#define CRYPTO_IOC_MAGIC        'C'

#define AES_IOC_SET_MODE        _IOW(CRYPTO_IOC_MAGIC,  1, unsigned long)
#define AES_IOC_SET_LEN         _IOW(CRYPTO_IOC_MAGIC,  2, unsigned long)
#define AES_IOC_GET_BUFSIZE     _IOW(CRYPTO_IOC_MAGIC,  3, unsigned long *)
#define AES_IOC_SET_IV          _IOW(CRYPTO_IOC_MAGIC,  5, unsigned long *)
#define AES_IOC_SET_KEY         _IOW(CRYPTO_IOC_MAGIC,  6, unsigned long *)
#define AES_IOC_START           _IOW(CRYPTO_IOC_MAGIC,  8, unsigned long)
#define AES_IOC_C_START         _IOW(CRYPTO_IOC_MAGIC,  9, unsigned long)
#define AES_IOC_UPDATE_IV       _IOW(CRYPTO_IOC_MAGIC, 11, unsigned long *)

#define SHA_IOC_INIT            _IOW(CRYPTO_IOC_MAGIC, 21, unsigned long)
#define SHA_IOC_FINISH          _IOW(CRYPTO_IOC_MAGIC, 25, unsigned long)

#endif
