/*
 * Copyright (c) 2014 Nuvoton technology corporation.
 *
 * Wan ZongShun <mcuos.com@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/suspend.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <linux/device.h>
#include <linux/clk.h>

#include <mach/mfp.h>
#include <mach/map.h>
#include <mach/regs-clock.h>

#include "nuc970-audio.h"

static DEFINE_MUTEX(i2s_mutex);
struct nuc970_audio *nuc970_i2s_data;

static int nuc970_i2s_set_fmt(struct snd_soc_dai *cpu_dai,
                              unsigned int fmt)
{
        struct nuc970_audio *nuc970_audio = nuc970_i2s_data;
        unsigned long val = AUDIO_READ(nuc970_audio->mmio + ACTL_I2SCON);

        switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
        case SND_SOC_DAIFMT_MSB:
                val |= FORMAT_MSB;
                break;
        case SND_SOC_DAIFMT_I2S:
                val &= ~FORMAT_MSB;
                break;
        default:
                return -EINVAL;
        }

		switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
		case SND_SOC_DAIFMT_CBM_CFM:
			val &= ~I2S_SLAVE;
			break;
		case SND_SOC_DAIFMT_CBS_CFS:
			val |= I2S_SLAVE;
			break;
		default:
			return -EINVAL;
		}
			
        AUDIO_WRITE(nuc970_audio->mmio + ACTL_I2SCON, val);

        return 0;
}

static int nuc970_i2s_set_sysclk(struct snd_soc_dai *cpu_dai,
                                 int clk_id, unsigned int freq, int dir)
{
        unsigned int val;
        struct nuc970_audio *nuc970_audio = nuc970_i2s_data;
		struct clk *clkmux, *clkaplldiv, *clkapll, *clkaudio;
		int ret;
		unsigned int mclkdiv, bclkdiv, mclk;
		
		clkmux = clk_get(NULL, "audio_eclk_mux");
        if (IS_ERR(clkmux)) {
			printk(KERN_ERR "nuc970-audio:failed to get audio clock source\n");
			ret = PTR_ERR(clkmux);
			return ret;
		}
		
		clkaplldiv = clk_get(NULL, "audio_aplldiv");
        if (IS_ERR(clkaplldiv)) {
			printk(KERN_ERR "nuc970-audio:failed to get audio clock source\n");
			ret = PTR_ERR(clkaplldiv);
			return ret;
		}
		
		clkapll = clk_get(NULL, "apll");
        if (IS_ERR(clkapll)) {
			printk(KERN_ERR "nuc970-audio:failed to get audio clock source\n");
			ret = PTR_ERR(clkapll);
			return ret;
		}
		
		clkaudio = clk_get(NULL, "audio_eclk");
        if (IS_ERR(clkaudio)) {
			printk(KERN_ERR "nuc970-audio:failed to get audio clock source\n");
			ret = PTR_ERR(clkaudio);
			return ret;
		}
				
        if (clk_id == NUC970_AUDIO_SAMPLECLK) {
                val = AUDIO_READ(nuc970_audio->mmio + ACTL_I2SCON);
		
				mclk = (freq*256);		// 256fs
				mclkdiv = clk_get_rate(clkaudio) / mclk;
				val &= ~0x000F0000;
				val |= (mclkdiv-1) << 16;
				
				bclkdiv = mclk / (freq*16*2);
				bclkdiv = bclkdiv/2 - 1;
				val &= ~0xf0;
				val |= (bclkdiv << 5);
 
                AUDIO_WRITE(nuc970_audio->mmio + ACTL_I2SCON, val);
        }

        if (clk_id == NUC970_AUDIO_CLKDIV) {
                //use APLL to generate 12.288MHz ,16.934MHz or 11.285Mhz for I2S
                //input source clock is XIN=12Mhz             
				
				clk_set_parent(clkmux, clkaplldiv);
				
				if ((freq % 8000 == 0)  && (freq != 32000)) {
					//12.288MHz ==> APLL=98.4MHz / 8 = 12.3MHz
					clk_set_rate(clkapll, 98400000);
					clk_set_rate(clkaudio, 12300000);
                } else if(freq == 44100) {
                	//16.934MHz ==> APLL=169.5MHz / 15 = 11.30MHz                	
                	clk_set_rate(clkapll, 169500000);
					clk_set_rate(clkaudio, 11300000);
            	} else {
                	//16.934MHz ==> APLL=169.5MHz / 10 = 16.95MHz                	
                	clk_set_rate(clkapll, 169500000);
					clk_set_rate(clkaudio, 16950000);
            	}
        }

        return 0;
}

static int nuc970_i2s_trigger(struct snd_pcm_substream *substream,
                              int cmd, struct snd_soc_dai *dai)
{
        struct nuc970_audio *nuc970_audio = nuc970_i2s_data;
        int ret = 0;
        unsigned long val, con;

        con = AUDIO_READ(nuc970_audio->mmio + ACTL_CON);

        switch (cmd) {
        case SNDRV_PCM_TRIGGER_START:
        case SNDRV_PCM_TRIGGER_RESUME:
                val = AUDIO_READ(nuc970_audio->mmio + ACTL_RESET);
                con |= I2S_EN;
                if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
                        con |= P_DMA_IRQ_EN;
                        AUDIO_WRITE(nuc970_audio->mmio + ACTL_PSR, P_DMA_RIA_IRQ);
						
                        val |= AUDIO_PLAY;                        
                } else {
                        con |= R_DMA_IRQ_EN;
                        AUDIO_WRITE(nuc970_audio->mmio + ACTL_RSR, R_DMA_RIA_IRQ);
						
                        val |= AUDIO_RECORD;
                }
                AUDIO_WRITE(nuc970_audio->mmio + ACTL_RESET, val);
                AUDIO_WRITE(nuc970_audio->mmio + ACTL_CON, con);

                break;
        case SNDRV_PCM_TRIGGER_STOP:
        case SNDRV_PCM_TRIGGER_SUSPEND:
                val = AUDIO_READ(nuc970_audio->mmio + ACTL_RESET);
                con &= ~I2S_EN;
                if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
                        con &= ~P_DMA_IRQ_EN;
                        AUDIO_WRITE(nuc970_audio->mmio + ACTL_PSR, RESET_PRSR);
                        val &= ~AUDIO_PLAY;
                } else {
                        con &= ~R_DMA_IRQ_EN;
                        AUDIO_WRITE(nuc970_audio->mmio + ACTL_RSR, RESET_PRSR);
                        val &= ~AUDIO_RECORD;
                }

                AUDIO_WRITE(nuc970_audio->mmio + ACTL_RESET, val);
                AUDIO_WRITE(nuc970_audio->mmio + ACTL_CON, con);

                break;
        default:
                ret = -EINVAL;
        }

        return ret;
}

static int nuc970_i2s_probe(struct snd_soc_dai *dai)
{
        struct nuc970_audio *nuc970_audio = nuc970_i2s_data;
        unsigned long val;

        mutex_lock(&i2s_mutex);

        clk_prepare(clk_get(NULL, "audio_hclk"));
		clk_enable(clk_get(NULL, "audio_hclk"));
	
        /* enable unit clock */
        clk_prepare(nuc970_audio->clk);
        clk_enable(nuc970_audio->clk);

        /* Select I2S pins */
        val = AUDIO_READ(nuc970_audio->mmio + ACTL_CON);
		val = (val & ~0x300) | BITS_SELECT_16;			//set default data bit to 16-bit
        AUDIO_WRITE(nuc970_audio->mmio + ACTL_CON, val);
				
        mutex_unlock(&i2s_mutex);

        return 0;
}

static int nuc970_i2s_remove(struct snd_soc_dai *dai)
{
        struct nuc970_audio *nuc970_audio = nuc970_i2s_data;
		clk_disable(nuc970_audio->clk);
		
		return 0;
}

static struct snd_soc_dai_ops nuc970_i2s_dai_ops = {
        .trigger	= nuc970_i2s_trigger,
        .set_fmt	= nuc970_i2s_set_fmt,
        .set_sysclk	= nuc970_i2s_set_sysclk,
};

struct snd_soc_dai_driver nuc970_i2s_dai = {
        .probe			= nuc970_i2s_probe,
        .remove			= nuc970_i2s_remove,
        .playback = {
			.rates		= SNDRV_PCM_RATE_8000_48000,
			.formats	= SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min	= 1,
			.channels_max	= 2,
        },
        .capture = {
			.rates		= SNDRV_PCM_RATE_8000_48000,
			.formats	= SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min	= 1,
			.channels_max	= 2,
        },
        .ops = &nuc970_i2s_dai_ops,
};

static const struct snd_soc_component_driver nuc970_i2s_component = {
	.name		= "nuc970-i2s",
};

static int nuc970_i2s_drvprobe(struct platform_device *pdev)
{
        struct nuc970_audio *nuc970_audio;
        int ret;

	if (nuc970_i2s_data)
		return -EBUSY;

	nuc970_audio = kzalloc(sizeof(struct nuc970_audio), GFP_KERNEL);
	if (!nuc970_audio)
		return -ENOMEM;

	spin_lock_init(&nuc970_audio->lock);
    spin_lock_init(&nuc970_audio->irqlock);

	nuc970_audio->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!nuc970_audio->res) {
		ret = -ENODEV;
		goto out0;
	}

	if (!request_mem_region(nuc970_audio->res->start,
			resource_size(nuc970_audio->res), pdev->name)) {
		ret = -EBUSY;
		goto out0;
	}

	nuc970_audio->mmio = ioremap(nuc970_audio->res->start,
					resource_size(nuc970_audio->res));
	if (!nuc970_audio->mmio) {
		ret = -ENOMEM;
		goto out1;
	}

	nuc970_audio->clk = clk_get(NULL, "audio_eclk");
	if (IS_ERR(nuc970_audio->clk)) {
		ret = PTR_ERR(nuc970_audio->clk);
		goto out2;
	}

	nuc970_audio->irq_num = platform_get_irq(pdev, 0);
	if (!nuc970_audio->irq_num) {
		ret = -EBUSY;
		goto out3;
	}
    
    ret = nuc970_dma_create(nuc970_audio);
	if (ret != 0)
		return ret;

	nuc970_i2s_data = nuc970_audio;

	ret = snd_soc_register_component(&pdev->dev, &nuc970_i2s_component,
					 &nuc970_i2s_dai, 1);
	if (ret)
		goto out3;

	return 0;

out3:
	clk_put(nuc970_audio->clk);
out2:
	iounmap(nuc970_audio->mmio);
out1:
	release_mem_region(nuc970_audio->res->start,
					resource_size(nuc970_audio->res));
out0:
	kfree(nuc970_audio);
	
	return ret;
}

static int nuc970_i2s_drvremove(struct platform_device *pdev)
{
	nuc970_dma_destroy(nuc970_i2s_data);
    snd_soc_unregister_component(&pdev->dev);

	clk_put(nuc970_i2s_data->clk);
	iounmap(nuc970_i2s_data->mmio);
	release_mem_region(nuc970_i2s_data->res->start,
				resource_size(nuc970_i2s_data->res));

	kfree(nuc970_i2s_data);
	nuc970_i2s_data = NULL;

        return 0;
}

static struct platform_driver nuc970_i2s_driver = {
        .driver	= {
			.name	= "nuc970-audio-i2s",
			.owner	= THIS_MODULE,
        },
        .probe		= nuc970_i2s_drvprobe,
        .remove		= nuc970_i2s_drvremove,
};

static int __init nuc970_i2s_init(void)
{
        return platform_driver_register(&nuc970_i2s_driver);
}

static void __exit nuc970_i2s_exit(void)
{
        platform_driver_unregister(&nuc970_i2s_driver);
}

module_init(nuc970_i2s_init);
module_exit(nuc970_i2s_exit);

MODULE_DESCRIPTION("NUC970 IIS SoC driver!");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nuc970-i2s");
