/*
 * Copyright (c) 2014 Nuvoton technology corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <mach/hardware.h>

#include "nuc970-audio.h"

static const struct snd_pcm_hardware nuc970_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_INTERLEAVED |
					SNDRV_PCM_INFO_BLOCK_TRANSFER |
					SNDRV_PCM_INFO_MMAP |
					SNDRV_PCM_INFO_MMAP_VALID |
					SNDRV_PCM_INFO_PAUSE |
					SNDRV_PCM_INFO_RESUME,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.channels_min		= 1,
	.channels_max		= 2,
	.buffer_bytes_max	= 4*1024,
	.period_bytes_min	= 1*1024,
	.period_bytes_max	= 4*1024,
	.periods_min		= 1,
	.periods_max		= 1024,
};

static int nuc970_dma_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct nuc970_audio *nuc970_audio = runtime->private_data;
	unsigned long flags;
	int ret = 0;

	ret = snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
	if (ret < 0)
		return ret;

	spin_lock_irqsave(&nuc970_audio->lock, flags);

	nuc970_audio->substream = substream;
	nuc970_audio->dma_addr[substream->stream] = runtime->dma_addr | 0x80000000;
	nuc970_audio->buffersize[substream->stream] =
						params_buffer_bytes(params);

	spin_unlock_irqrestore(&nuc970_audio->lock, flags);

	return ret;
}

static void nuc970_update_dma_register(struct snd_pcm_substream *substream,
				dma_addr_t dma_addr, size_t count)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct nuc970_audio *nuc970_audio = runtime->private_data;
	void __iomem *mmio_addr, *mmio_len;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		mmio_addr = nuc970_audio->mmio + ACTL_PDESB;
		mmio_len = nuc970_audio->mmio + ACTL_PDES_LENGTH;
	} else {
		mmio_addr = nuc970_audio->mmio + ACTL_RDESB;
		mmio_len = nuc970_audio->mmio + ACTL_RDES_LENGTH;
	}

	AUDIO_WRITE(mmio_addr, dma_addr);
	AUDIO_WRITE(mmio_len, count);
}

static void nuc970_dma_start(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct nuc970_audio *nuc970_audio = runtime->private_data;
	unsigned long val;

	val = AUDIO_READ(nuc970_audio->mmio + ACTL_CON);
	val |= (P_DMA_IRQ_EN | R_DMA_IRQ_EN);
	AUDIO_WRITE(nuc970_audio->mmio + ACTL_CON, val);
}

static void nuc970_dma_stop(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct nuc970_audio *nuc970_audio = runtime->private_data;
	unsigned long val;

	val = AUDIO_READ(nuc970_audio->mmio + ACTL_CON);
	val &= ~(P_DMA_IRQ_EN | R_DMA_IRQ_EN);
	AUDIO_WRITE(nuc970_audio->mmio + ACTL_CON, val);
}

static irqreturn_t nuc970_dma_interrupt(int irq, void *dev_id)
{
	struct snd_pcm_substream *substream = dev_id;
	struct nuc970_audio *nuc970_audio = substream->runtime->private_data;
	unsigned long val;

	spin_lock(&nuc970_audio->lock);

	val = AUDIO_READ(nuc970_audio->mmio + ACTL_CON);

	if (val & R_DMA_IRQ) {
		AUDIO_WRITE(nuc970_audio->mmio + ACTL_CON, val | R_DMA_IRQ);

		val = AUDIO_READ(nuc970_audio->mmio + ACTL_RSR);

		if (val & R_DMA_RIA_IRQ) {
			val = R_DMA_RIA_IRQ;
			AUDIO_WRITE(nuc970_audio->mmio + ACTL_RSR, val);
		}

	} else if (val & P_DMA_IRQ) {
		AUDIO_WRITE(nuc970_audio->mmio + ACTL_CON, val | P_DMA_IRQ);

		val = AUDIO_READ(nuc970_audio->mmio + ACTL_PSR);

		if (val & P_DMA_RIA_IRQ) {
			val = P_DMA_RIA_IRQ;
			AUDIO_WRITE(nuc970_audio->mmio + ACTL_PSR, val);
		}

	} else {
		dev_err(nuc970_audio->dev, "Wrong DMA interrupt status!\n");
		spin_unlock(&nuc970_audio->lock);
		return IRQ_HANDLED;
	}

	spin_unlock(&nuc970_audio->lock);

	snd_pcm_period_elapsed(substream);

	return IRQ_HANDLED;
}

static int nuc970_dma_hw_free(struct snd_pcm_substream *substream)
{
	snd_pcm_lib_free_pages(substream);
	return 0;
}

static int nuc970_dma_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct nuc970_audio *nuc970_audio = runtime->private_data;
	unsigned long flags, val;
	int ret = 0;

	spin_lock_irqsave(&nuc970_audio->lock, flags);

	nuc970_update_dma_register(substream,
				nuc970_audio->dma_addr[substream->stream],
				nuc970_audio->buffersize[substream->stream]);

	val = AUDIO_READ(nuc970_audio->mmio + ACTL_RESET);

	switch (runtime->channels) {
	case 1:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			val &= ~(PLAY_LEFT_CHNNEL | PLAY_RIGHT_CHNNEL);
			val |= PLAY_RIGHT_CHNNEL;
		} else {
			val &= ~(RECORD_LEFT_CHNNEL | RECORD_RIGHT_CHNNEL);
			val |= RECORD_RIGHT_CHNNEL;
		}
		AUDIO_WRITE(nuc970_audio->mmio + ACTL_RESET, val);
		break;
	case 2:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			val |= (PLAY_LEFT_CHNNEL | PLAY_RIGHT_CHNNEL);
		else
			val |= (RECORD_LEFT_CHNNEL | RECORD_RIGHT_CHNNEL);
		AUDIO_WRITE(nuc970_audio->mmio + ACTL_RESET, val);
		break;
	default:
		ret = -EINVAL;
	}
	
	/* set DMA IRQ to half */
	val = AUDIO_READ(nuc970_audio->mmio + ACTL_CON);
	val &= ~0xf000;
	val |= (R_DMA_IRQ_SEL_HALF | P_DMA_IRQ_SEL_HALF);
	AUDIO_WRITE(nuc970_audio->mmio + ACTL_CON, val);
	
	spin_unlock_irqrestore(&nuc970_audio->lock, flags);
	return ret;
}

static int nuc970_dma_trigger(struct snd_pcm_substream *substream, int cmd)
{
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		nuc970_dma_start(substream);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		nuc970_dma_stop(substream);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int nuc970_dma_getposition(struct snd_pcm_substream *substream,
					dma_addr_t *src, dma_addr_t *dst)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct nuc970_audio *nuc970_audio = runtime->private_data;

	if (src != NULL)
		*src = AUDIO_READ(nuc970_audio->mmio + ACTL_PDESC);

	if (dst != NULL)
		*dst = AUDIO_READ(nuc970_audio->mmio + ACTL_RDESC);

	return 0;
}

static snd_pcm_uframes_t nuc970_dma_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	dma_addr_t src, dst;
	unsigned long res;

	nuc970_dma_getposition(substream, &src, &dst);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		res = dst - runtime->dma_addr;
	else
		res = src - runtime->dma_addr;

	return bytes_to_frames(substream->runtime, res);
}

static int nuc970_dma_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct nuc970_audio *nuc970_audio;

	snd_soc_set_runtime_hwparams(substream, &nuc970_pcm_hardware);

	nuc970_audio = nuc970_i2s_data;

	if (request_irq(nuc970_audio->irq_num, nuc970_dma_interrupt,
			0, "nuc970-dma", substream))
		return -EBUSY;

	runtime->private_data = nuc970_audio;

	return 0;
}

static int nuc970_dma_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct nuc970_audio *nuc970_audio = runtime->private_data;

	free_irq(nuc970_audio->irq_num, substream);

	return 0;
}

static int nuc970_dma_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	return dma_mmap_writecombine(substream->pcm->card->dev, vma,
					runtime->dma_area,
					runtime->dma_addr,
					runtime->dma_bytes);
}

static struct snd_pcm_ops nuc970_dma_ops = {
	.open		= nuc970_dma_open,
	.close		= nuc970_dma_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= nuc970_dma_hw_params,
	.hw_free	= nuc970_dma_hw_free,
	.prepare	= nuc970_dma_prepare,
	.trigger	= nuc970_dma_trigger,
	.pointer	= nuc970_dma_pointer,
	.mmap		= nuc970_dma_mmap,
};

static void nuc970_dma_free_dma_buffers(struct snd_pcm *pcm)
{
	snd_pcm_lib_preallocate_free_for_all(pcm);
}

static u64 nuc970_pcm_dmamask = DMA_BIT_MASK(32);
static int nuc970_dma_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	struct snd_pcm *pcm = rtd->pcm;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &nuc970_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV,
		card->dev, 4 * 1024, (4 * 1024) - 1);

	return 0;
}

static struct snd_soc_platform_driver nuc970_soc_platform = {
	.ops		= &nuc970_dma_ops,
	.pcm_new	= nuc970_dma_new,
	.pcm_free	= nuc970_dma_free_dma_buffers,
};

static int nuc970_soc_platform_probe(struct platform_device *pdev)
{
	return snd_soc_register_platform(&pdev->dev, &nuc970_soc_platform);
}

static int nuc970_soc_platform_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static struct platform_driver nuc970_pcm_driver = {
	.driver = {
			.name = "nuc970-audio-pcm",
			.owner = THIS_MODULE,
	},

	.probe = nuc970_soc_platform_probe,
	.remove = nuc970_soc_platform_remove,
};

module_platform_driver(nuc970_pcm_driver);

MODULE_DESCRIPTION("nuc970 Audio DMA module");
MODULE_LICENSE("GPL");
