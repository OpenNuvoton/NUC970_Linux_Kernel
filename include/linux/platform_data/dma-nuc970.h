#ifndef __ASM_ARCH_DMA_H
#define __ASM_ARCH_DMA_H

#include <linux/types.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>

/* M2M channels */
#define NUC970_DMA_MEM		10

/**
 * struct nuc970_dma_data - configuration data for the NUC970 dmaengine
 * @port: peripheral which is requesting the channel
 * @direction: TX/RX channel
 * @name: optional name for the channel, this is displayed in /proc/interrupts
 *
 * This information is passed as private channel parameter in a filter
 * function. Note that this is only needed for slave/cyclic channels.  For
 * memcpy channels %NULL data should be passed.
 */
struct nuc970_dma_data {
	int				port;
	enum dma_transfer_direction	direction;
	const char			*name;
};

/**
 * struct nuc970_dma_chan_data - platform specific data for a DMA channel
 * @name: name of the channel, used for getting the right clock for the channel
 * @base: mapped registers
 * @irq: interrupt number used by this channel
 */
struct nuc970_dma_chan_data {
	const char			*name;
	void __iomem			*base;
	int				irq;
};

/**
 * struct nuc970_dma_platform_data - platform data for the dmaengine driver
 * @channels: array of channels which are passed to the driver
 * @num_channels: number of channels in the array
 *
 * This structure is passed to the DMA engine driver via platform data. For
 * M2P channels, contract is that even channels are for TX and odd for RX.
 * There is no requirement for the M2M channels.
 */
struct nuc970_dma_platform_data {
	struct nuc970_dma_chan_data	*channels;
	size_t				num_channels;
};

#if 0
static inline bool nuc970_dma_chan_is_m2p(struct dma_chan *chan)
{
	return !strcmp(dev_name(chan->device->dev), "nuc970-dma-m2p");
}
#endif

#if 0
/**
 * nuc970_dma_chan_direction - returns direction the channel can be used
 * @chan: channel
 *
 * This function can be used in filter functions to find out whether the
 * channel supports given DMA direction. Only M2P channels have such
 * limitation, for M2M channels the direction is configurable.
 */
static inline enum dma_transfer_direction
nuc970_dma_chan_direction(struct dma_chan *chan)
{
	if (!nuc970_dma_chan_is_m2p(chan))
		return DMA_NONE;

	/* even channels are for TX, odd for RX */
	return (chan->chan_id % 2 == 0) ? DMA_MEM_TO_DEV : DMA_DEV_TO_MEM;
}
#endif
#endif /* __ASM_ARCH_DMA_H */
