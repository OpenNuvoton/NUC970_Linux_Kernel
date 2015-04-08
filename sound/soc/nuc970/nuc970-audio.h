/*
 * Copyright (c) 2014 Nuvoton technology corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#ifndef _NUC970_AUDIO_H
#define _NUC970_AUDIO_H

#include <linux/io.h>
#include <mach/regs-audio.h>

/* bit definition of REG_ACTL_CON register */
#define R_DMA_IRQ_EN			0x200000
#define P_DMA_IRQ_EN			0x100000

#define R_DMA_IRQ_SEL_EIGHTH	0xC000
#define R_DMA_IRQ_SEL_QUARTER	0x8000
#define R_DMA_IRQ_SEL_HALF		0x4000
#define R_DMA_IRQ_SEL_END		0x0000

#define P_DMA_IRQ_SEL_EIGHTH	0x3000
#define P_DMA_IRQ_SEL_QUARTER	0x2000
#define P_DMA_IRQ_SEL_HALF		0x1000
#define P_DMA_IRQ_SEL_END		0x0000

#define R_DMA_IRQ				0x0800
#define P_DMA_IRQ				0x0400
#define BITS_SELECT_24			0x0200
#define BITS_SELECT_16			0x0100
#define BITS_SELECT_8			0x0000
#define FIFO_TH					0x0080
#define IRQ_DMA_CNTER_EN		0x0010
#define IRQ_DMA_DATA_ZERO_EN	0x0008
#define PCM_EN					0x0002
#define I2S_EN					0x0001


/* bit definition of REG_ACTL_RESET register */
#define SPLIT_DATA			0x100000
#define ACTL_RESET_BIT		0x10000
#define RECORD_RIGHT_CHNNEL	0x08000
#define RECORD_LEFT_CHNNEL	0x04000
#define PLAY_RIGHT_CHNNEL	0x02000
#define PLAY_LEFT_CHNNEL	0x01000
#define AUDIO_RECORD		0x00040
#define AUDIO_PLAY			0x00020
#define I2S_PCM_RESET		0x00001

/* bit definition of ACTL_RSR register */
#define R_FIFO_FULL			0x04
#define R_FIFO_EMPTY		0x02
#define R_DMA_RIA_IRQ		0x01

/* bit definition of ACTL_PSR register */
#define P_FIFO_FULL			0x04
#define P_FIFO_EMPTY		0x02
#define P_DMA_RIA_IRQ		0x01

/*----- bit definition of REG_ACTL_I2SCON register -----*/
#define I2S_SLAVE			0x100000
#define FORMAT_I2S			0x0000
#define FORMAT_MSB			0x0008
#define MCLK_SEL			0x0010
#define SCALE_1				0x00000
#define SCALE_2				0x10000
#define SCALE_3				0x20000
#define SCALE_4				0x30000
#define SCALE_5				0x40000
#define SCALE_6				0x50000
#define SCALE_7				0x60000
#define SCALE_8				0x70000
#define SCALE_10			0x90000
#define SCALE_12			0xB0000
#define SCALE_14			0xD0000
#define SCALE_16			0xF0000
#define FS_384				0x20
#define FS_256				0x0
#define BCLK_32				0x00
#define BCLK_48				0x40

#define NUC970_AUDIO_SAMPLECLK	0x00
#define NUC970_AUDIO_CLKDIV		0x01

#define CODEC_READY		0x10
#define RESET_PRSR		0x00
#define AUDIO_WRITE(addr, val)	__raw_writel(val, addr)
#define AUDIO_READ(addr)	__raw_readl(addr)

struct nuc970_audio {
	void __iomem *mmio;
	spinlock_t irqlock, lock;
	dma_addr_t dma_addr[2];
	unsigned long buffersize[2];
	unsigned long irq_num;
	struct snd_pcm_substream *substream[2];
	struct resource *res;
	struct clk *clk;
	struct device *dev;

};

extern struct nuc970_audio *nuc970_i2s_data;

int nuc970_dma_create(struct nuc970_audio *nuc970_audio);
int nuc970_dma_destroy(struct nuc970_audio *nuc970_audio);
#endif /*end _NUC970_AUDIO_H */
