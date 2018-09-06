/*
 * Copyright (c) 2014 Nuvoton technology corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#include "nuc970-audio.h"
#include "../codecs/nau8822.h"

static int nuc970_audio_hw_params(struct snd_pcm_substream *substream,
								  struct snd_pcm_hw_params *params)
{
		struct snd_soc_pcm_runtime *rtd = substream->private_data;
		struct snd_soc_dai *codec_dai = rtd->codec_dai;
		struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
		int ret;

//slave mode
#ifdef CONFIG_NUC970_I2S_SLAVE_MODE
		 /* set codec DAI configuration */
		ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
								  SND_SOC_DAIFMT_NB_NF |
								  SND_SOC_DAIFMT_CBM_CFM);
		if (ret < 0)
			return ret;

		/* set cpu DAI configuration */
		ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
								  SND_SOC_DAIFMT_NB_NF |
								  SND_SOC_DAIFMT_CBS_CFS);
		if (ret < 0)
			return ret;

		ret = snd_soc_dai_set_sysclk(codec_dai, NAU8822_PLL, 12000000, SND_SOC_CLOCK_OUT);
		if (ret < 0)
			return ret;

		ret = snd_soc_dai_set_clkdiv(codec_dai, NAU8822_BCLKDIV, 0xC);      // divide 8 form MCLK to BCLK
		if (ret < 0)
			return ret;
#else
//master mode
		unsigned int clk = 0;
		unsigned int sample_rate = params_rate(params);

		/* set codec DAI configuration */
		ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
								  SND_SOC_DAIFMT_NB_NF |
								  SND_SOC_DAIFMT_CBS_CFS);
		if (ret < 0)
				return ret;

		/* set cpu DAI configuration */
		ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
								  SND_SOC_DAIFMT_NB_NF |
								  SND_SOC_DAIFMT_CBM_CFM);
		if (ret < 0)
				return ret;

		clk = 256 * sample_rate;

		/* set the codec system clock for DAC and ADC */
		ret = snd_soc_dai_set_sysclk(codec_dai, NAU8822_MCLK, clk, SND_SOC_CLOCK_OUT);
		if (ret < 0)
				return ret;

		/* set prescaler division for sample rate */
		ret = snd_soc_dai_set_sysclk(cpu_dai, NUC970_AUDIO_CLKDIV, sample_rate, SND_SOC_CLOCK_OUT);
		if (ret < 0)
				return ret;

		/* set MCLK division for sample rate */
		ret = snd_soc_dai_set_sysclk(cpu_dai, NUC970_AUDIO_SAMPLECLK, sample_rate, SND_SOC_CLOCK_OUT);
		if (ret < 0)
				return ret;
#endif
		return 0;
}

static struct snd_soc_ops nuc970_audio_ops = {
		.hw_params = nuc970_audio_hw_params,
};

static struct snd_soc_dai_link nuc970evb_i2s_dai = {
		.name       = "IIS",
		.stream_name    = "IIS HiFi",
		.cpu_dai_name   = "nuc970-audio-i2s",
		.codec_dai_name = "nau8822-hifi",
		.codec_name     = "nau8822.0-001a",
		.ops        = &nuc970_audio_ops,
		.platform_name = "nuc970-audio-pcm.0",
};

static struct snd_soc_card nuc970evb_audio_machine = {
		.name       = "nuc970_IIS",
		.owner      = THIS_MODULE,
		.dai_link   = &nuc970evb_i2s_dai,
		.num_links  = 1,
};

static int nuc970_audio_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card = &nuc970evb_audio_machine;
	int ret;

	card->dev = &pdev->dev;

	if (np) {
		nuc970evb_i2s_dai.cpu_dai_name = NULL;
		nuc970evb_i2s_dai.cpu_of_node = of_parse_phandle(np,
				"i2s-controller", 0);
		if (!nuc970evb_i2s_dai.cpu_of_node) {
			dev_err(&pdev->dev,
			   "Property 'i2s-controller' missing or invalid\n");
			ret = -EINVAL;
		}

		nuc970evb_i2s_dai.platform_name = NULL;
		nuc970evb_i2s_dai.platform_of_node  = of_parse_phandle(np,
				"i2s-platform", 0);
		if (!nuc970evb_i2s_dai.platform_of_node) {
			dev_err(&pdev->dev,
			   "Property 'i2s-platform' missing or invalid\n");
			ret = -EINVAL;
		}

	}

	ret = snd_soc_register_card(card);
	if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card() failed: %d\n", ret);

	return ret;
}

static int nuc970_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);
	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id nuc970_audio_of_match[] = {
	{   .compatible = "nuvoton,nuc970-audio"    },
	{   },
};
MODULE_DEVICE_TABLE(of, nuc970_audio_of_match);
#endif

static struct platform_driver nuc970_audio_driver = {
	.driver     = {
		.name   = "nuc970-audio",
		.owner  = THIS_MODULE,
#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(nuc970_audio_of_match),
#endif
	},
	.probe      = nuc970_audio_probe,
	.remove     = nuc970_audio_remove,
};

module_platform_driver(nuc970_audio_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NUC970/N9H30 Series SoC audio support");
