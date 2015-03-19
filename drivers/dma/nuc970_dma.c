/*
 * Driver for the NUC970 DMA Controller
 *
 * Copyright (C) 2011 Mika Westerberg
 *
 * DMA M2M implementation is based on the original
 * arch/arm/nuc970/dma.c which has following copyrights:
 *
 * This driver is based on xxx and xxx drivers.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/dmaengine.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/platform_data/dma-nuc970.h>
#include <asm/irq.h>
#include <mach/map.h>
#include <mach/regs-gdma.h>
#include <mach/regs-clock.h>


#include "dmaengine.h"

#if 0
#define ENTRY()					printk("[%-20s] : Enter...\n", __FUNCTION__)
#define LEAVE()					printk("[%-20s] : Leave...\n", __FUNCTION__)
#else
#define ENTRY()
#define LEAVE()
#endif

#if 0
#define DMA_DEBUG printk
#else
#define DMA_DEBUG(fmt,args...)
#endif


#define DMA_MAX_CHAN_DESCRIPTORS	32
#define DMA_MAX_CHAN_BYTES		0xffffff

struct nuc970_dma_engine;

/**
 * struct ep93xx_dma_desc - EP93xx specific transaction descriptor
 * @src_addr: source address of the transaction
 * @dst_addr: destination address of the transaction
 * @size: size of the transaction (in bytes)
 * @complete: this descriptor is completed
 * @txd: dmaengine API descriptor
 * @tx_list: list of linked descriptors
 * @node: link used for putting this into a channel queue
 */
struct nuc970_dma_desc {
	u32				src_addr;
	u32				dst_addr;
	size_t				size;
	bool				complete;
	struct dma_async_tx_descriptor	txd;
	struct list_head		tx_list;
	struct list_head		node;
};

/**
 * struct nuc970_dma_chan - an NUC970 DMA M2M channel
 * @chan: dmaengine API channel
 * @edma: pointer to to the engine device
 * @regs: memory mapped registers
 * @irq: interrupt number of the channel
 * @clk: clock used by this channel
 * @tasklet: channel specific tasklet used for callbacks
 * @lock: lock protecting the fields following
 * @flags: flags for the channel
 * @buffer: which buffer to use next (0/1)
 * @active: flattened chain of descriptors currently being processed
 * @queue: pending descriptors which are handled next
 * @free_list: list of free descriptors which can be used
 * @runtime_addr: physical address currently used as dest/src (M2M only). This
 *                is set via %DMA_SLAVE_CONFIG before slave operation is
 *                prepared
 * @runtime_ctrl: M2M runtime values for the control register.
 *
 * As NUC970 DMA controller doesn't support real chained DMA descriptors we
 * will have slightly different scheme here: @active points to a head of
 * flattened DMA descriptor chain.
 *
 * @queue holds pending transactions. These are linked through the first
 * descriptor in the chain. When a descriptor is moved to the @active queue,
 * the first and chained descriptors are flattened into a single list.
 *
 * @chan.private holds pointer to &struct nuc970_dma_data which contains
 * necessary channel configuration information. For memcpy channels this must
 * be %NULL.
 */
struct nuc970_dma_chan {
	struct dma_chan			chan;
	const struct nuc970_dma_engine	*edma;
	void __iomem			*regs;
	int				irq;
	struct tasklet_struct		tasklet;
	/* protects the fields following */
	spinlock_t			lock;
	unsigned long			flags;
/* Channel is configured for cyclic transfers */
#define NUC970_DMA_IS_CYCLIC		0

	int				buffer;
	struct list_head		active;
	struct list_head		queue;
	struct list_head		free_list;
	u32				runtime_addr;
	u32				runtime_ctrl;
};

/**
 * struct nuc970_dma_engine - the NUC970 DMA engine instance
 * @dma_dev: holds the dmaengine device
 * @hw_setup: method which sets the channel up for operation
 * @hw_shutdown: shuts the channel down and flushes whatever is left
 * @hw_submit: pushes active descriptor(s) to the hardware
 * @hw_interrupt: handle the interrupt
 * @num_channels: number of channels for this instance
 * @channels: array of channels
 *
 * There is one instance of this struct for the M2M channels. 
 * hw_xxx() methods are used to perform operations which are
 * different on M2M and M2P channels. These methods are called with channel
 * lock held and interrupts disabled so they cannot sleep.
 */
struct nuc970_dma_engine {
	struct dma_device	dma_dev;
	int			(*hw_setup)(struct nuc970_dma_chan *);
	void		(*hw_shutdown)(struct nuc970_dma_chan *);
	void		(*hw_submit)(struct nuc970_dma_chan *);
	int			(*hw_interrupt)(struct nuc970_dma_chan *);
#define INTERRUPT_UNKNOWN	0
#define INTERRUPT_DONE		1
#define INTERRUPT_NEXT_BUFFER	2
	size_t			num_channels;
	struct nuc970_dma_chan	channels[];
};

static inline struct device *chan2dev(struct nuc970_dma_chan *edmac)
{
	ENTRY();
	return &edmac->chan.dev->device;
}

static struct nuc970_dma_chan *to_nuc970_dma_chan(struct dma_chan *chan)
{
	ENTRY();
	return container_of(chan, struct nuc970_dma_chan, chan);
}

/**
 * nuc970_dma_set_active - set new active descriptor chain
 * @edmac: channel
 * @desc: head of the new active descriptor chain
 *
 * Sets @desc to be the head of the new active descriptor chain. This is the
 * chain which is processed next. The active list must be empty before calling
 * this function.
 *
 * Called with @edmac->lock held and interrupts disabled.
 */
static void nuc970_dma_set_active(struct nuc970_dma_chan *edmac,
				  struct nuc970_dma_desc *desc)
{
	ENTRY();
	BUG_ON(!list_empty(&edmac->active));

	list_add_tail(&desc->node, &edmac->active);

	/* Flatten the @desc->tx_list chain into @edmac->active list */
	while (!list_empty(&desc->tx_list)) {
		struct nuc970_dma_desc *d = list_first_entry(&desc->tx_list,
			struct nuc970_dma_desc, node);

		/*
		 * We copy the callback parameters from the first descriptor
		 * to all the chained descriptors. This way we can call the
		 * callback without having to find out the first descriptor in
		 * the chain. Useful for cyclic transfers.
		 */
		d->txd.callback = desc->txd.callback;
		d->txd.callback_param = desc->txd.callback_param;

		list_move_tail(&d->node, &edmac->active);
	}
	LEAVE();
}

/* Called with @edmac->lock held and interrupts disabled */
static struct nuc970_dma_desc *
nuc970_dma_get_active(struct nuc970_dma_chan *edmac)
{
	DMA_DEBUG("NUC970 GDMA %s\n", __FUNCTION__ );
	if (list_empty(&edmac->active))
		return NULL;

	return list_first_entry(&edmac->active, struct nuc970_dma_desc, node);
}

/**
 * nuc970_dma_advance_active - advances to the next active descriptor
 * @edmac: channel
 *
 * Function advances active descriptor to the next in the @edmac->active and
 * returns %true if we still have descriptors in the chain to process.
 * Otherwise returns %false.
 *
 * When the channel is in cyclic mode always returns %true.
 *
 * Called with @edmac->lock held and interrupts disabled.
 */
static bool nuc970_dma_advance_active(struct nuc970_dma_chan *edmac)
{	
	struct nuc970_dma_desc *desc;
	DMA_DEBUG("NUC970 GDMA %s\n", __FUNCTION__ );
	list_rotate_left(&edmac->active);

	if (test_bit(NUC970_DMA_IS_CYCLIC, &edmac->flags))
		return true;

	desc = nuc970_dma_get_active(edmac);
	if (!desc)
		return false;

	/*
	 * If txd.cookie is set it means that we are back in the first
	 * descriptor in the chain and hence done with it.
	 */
	return !desc->txd.cookie;
}

/*
 * M2M DMA implementation
 */

static int m2m_hw_setup(struct nuc970_dma_chan *edmac)
{	
	const struct nuc970_dma_data *data = edmac->chan.private;
	//u32 control = 0;
	DMA_DEBUG("NUC970 GDMA %s\n", __FUNCTION__ );
	if (!data) {
		/* This is memcpy channel, nothing to configure */
		return 0;
	}

	switch (data->port) {
	case NUC970_DMA_MEM:
		break;

	default:
		return -EINVAL;
	}
	//writel(control, edmac->regs + M2M_CONTROL);
	return 0;
}

static void m2m_hw_shutdown(struct nuc970_dma_chan *edmac)
{
	ENTRY();
	/* Just disable the channel */
	LEAVE();
}

static void m2m_fill_desc(struct nuc970_dma_chan *edmac)
{
	struct nuc970_dma_desc *desc;
	u32 tcnt,config;
	
	ENTRY();	
	desc = nuc970_dma_get_active(edmac);
	if (!desc) {
		dev_warn(chan2dev(edmac), "M2M: empty descriptor list\n");
		return;
	}
	__raw_writel(desc->src_addr, edmac->regs + GDMA_SRCB);
	__raw_writel(desc->dst_addr, edmac->regs + GDMA_DSTB);	
	
	//Transfer Width Select for GDMA
	config = __raw_readl(edmac->regs+GDMA_CTL);
    config = (config & ~TWS_Msk) | edmac->runtime_ctrl;	
	__raw_writel(config,edmac->regs + GDMA_CTL);

	switch((__raw_readl(edmac->regs + GDMA_CTL) & TWS_Msk)>>12)
	{
		case 0:		tcnt=(desc->size)>>0; break;			
		case 1:		tcnt=(desc->size)>>1; break;
		case 2:		tcnt=(desc->size)>>2; break;
		default:	tcnt=(desc->size)>>0; break;
	}		
	if( __raw_readl(edmac->regs + GDMA_CTL) & BME)
		tcnt=tcnt>>3;
		
	__raw_writel(tcnt,edmac->regs + GDMA_TCNT);		
	DMA_DEBUG("%s GDMA_CTL  0x%08x=0x%08x\n", __FUNCTION__,edmac->regs,__raw_readl(edmac->regs));
	DMA_DEBUG("%s GDMA_SRCB 0x%08x=0x%08x\n", __FUNCTION__,edmac->regs+ GDMA_SRCB,__raw_readl(edmac->regs+ GDMA_SRCB));
	DMA_DEBUG("%s GDMA_DSTB 0x%08x=0x%08x\n", __FUNCTION__,edmac->regs+ GDMA_DSTB,__raw_readl(edmac->regs+ GDMA_DSTB));
	DMA_DEBUG("%s GDMA_TCNT 0x%08x=0x%08x\n", __FUNCTION__,edmac->regs+ GDMA_TCNT,__raw_readl(edmac->regs+ GDMA_TCNT));
	edmac->buffer ^= 1;
	LEAVE();
}

static void m2m_hw_submit(struct nuc970_dma_chan *edmac)
{
	//struct nuc970_dma_data *data = edmac->chan.private;
	u32 control,intctrl;
	ENTRY();
	/*
	 * Since we allow clients to configure PW (peripheral width) we always
	 * clear PW bits here and then set them according what is given in
	 * the runtime configuration.
	 */
	m2m_fill_desc(edmac);
	if (nuc970_dma_advance_active(edmac)) {
		DMA_DEBUG("%s nuc970_dma_advance_active(edmac) OK\n", __FUNCTION__);
		m2m_fill_desc(edmac);		
	}

	intctrl = __raw_readl(REG_GDMA_INTCS);
	intctrl |= (TC0EN | TC1EN);
	__raw_writel(intctrl,REG_GDMA_INTCS);
	DMA_DEBUG("%s GDMA_INTCS  0x%08x=0x%08x\n", __FUNCTION__,REG_GDMA_INTCS,__raw_readl(REG_GDMA_INTCS));
		
	control = __raw_readl(edmac->regs + GDMA_CTL);
	control |= (1 | SOFTREQ);
	__raw_writel(control,edmac->regs + GDMA_CTL);
//	while(1)
	DMA_DEBUG("%s GDMA_CTL  0x%08x=0x%08x, 0x%08x=0x%08x\n", __FUNCTION__,edmac->regs,__raw_readl(edmac->regs),REG_GDMA_INTCS,__raw_readl(REG_GDMA_INTCS));
	LEAVE();
}

/*
 * According to NUC970 User's Guide, we should receive DONE interrupt when all
 * M2M DMA controller transactions complete normally. This is not always the
 * case - sometimes NUC970 M2M DMA asserts DONE interrupt when the DMA channel
 * is still running (channel Buffer FSM in DMA_BUF_ON state, and channel
 * Control FSM in DMA_MEM_RD state, observed at least in IDE-DMA operation).
 * In effect, disabling the channel when only DONE bit is set could stop
 * currently running DMA transfer. To avoid this, we use Buffer FSM and
 * Control FSM to check current state of DMA channel.
 */
static int m2m_hw_interrupt(struct nuc970_dma_chan *edmac)
{
	u32 status = __raw_readl(REG_GDMA_INTCS);		
	ENTRY();	
	DMA_DEBUG("REG_GDMA_INTCS=0x%08x\n",status);
	status &= 0x0F;
	if(status & 1<<(9+(edmac->irq-25)*2))  // test TERR0F/TERR0F
	{
		printk("GDMA :Channel %d Transfer Error\n",(edmac->irq-25));
		__raw_writel((1<<(9+(edmac->irq-25)*2))|status, REG_GDMA_INTCS);
	}	
	__raw_writel((1<<(8+(edmac->irq-25)*2))|status, REG_GDMA_INTCS);	
	LEAVE();
	return INTERRUPT_DONE;
}

/*
 * DMA engine API implementation
 */

static struct nuc970_dma_desc *
nuc970_dma_desc_get(struct nuc970_dma_chan *edmac)
{
	struct nuc970_dma_desc *desc, *_desc;
	struct nuc970_dma_desc *ret = NULL;
	unsigned long flags;
	ENTRY();
	spin_lock_irqsave(&edmac->lock, flags);
	list_for_each_entry_safe(desc, _desc, &edmac->free_list, node) {
		if (async_tx_test_ack(&desc->txd)) {
			list_del_init(&desc->node);

			/* Re-initialize the descriptor */
			desc->src_addr = 0;
			desc->dst_addr = 0;
			desc->size = 0;
			desc->complete = false;
			desc->txd.cookie = 0;
			desc->txd.callback = NULL;
			desc->txd.callback_param = NULL;

			ret = desc;
			break;
		}
	}
	spin_unlock_irqrestore(&edmac->lock, flags);
	LEAVE();	
	return ret;
}

static void nuc970_dma_desc_put(struct nuc970_dma_chan *edmac,
				struct nuc970_dma_desc *desc)
{
	ENTRY();
	if (desc) {
		unsigned long flags;
	
		spin_lock_irqsave(&edmac->lock, flags);
		list_splice_init(&desc->tx_list, &edmac->free_list);
		list_add(&desc->node, &edmac->free_list);
		spin_unlock_irqrestore(&edmac->lock, flags);
	}
}

/**
 * nuc970_dma_advance_work - start processing the next pending transaction
 * @edmac: channel
 *
 * If we have pending transactions queued and we are currently idling, this
 * function takes the next queued transaction from the @edmac->queue and
 * pushes it to the hardware for execution.
 */
static void nuc970_dma_advance_work(struct nuc970_dma_chan *edmac)
{
	struct nuc970_dma_desc *new;
	unsigned long flags;
	ENTRY();
	spin_lock_irqsave(&edmac->lock, flags);
	if (!list_empty(&edmac->active) || list_empty(&edmac->queue)) {
		spin_unlock_irqrestore(&edmac->lock, flags);
		return;
	}

	/* Take the next descriptor from the pending queue */
	new = list_first_entry(&edmac->queue, struct nuc970_dma_desc, node);
	list_del_init(&new->node);

	nuc970_dma_set_active(edmac, new);

	/* Push it to the hardware */
	edmac->edma->hw_submit(edmac);
	spin_unlock_irqrestore(&edmac->lock, flags);
	LEAVE();	
}

static void nuc970_dma_unmap_buffers(struct nuc970_dma_desc *desc)
{
	struct device *dev = desc->txd.chan->device->dev;
	ENTRY();
	if (!(desc->txd.flags & DMA_COMPL_SKIP_SRC_UNMAP)) {
		if (desc->txd.flags & DMA_COMPL_SRC_UNMAP_SINGLE)
			dma_unmap_single(dev, desc->src_addr, desc->size,
					 DMA_TO_DEVICE);
		else
			dma_unmap_page(dev, desc->src_addr, desc->size,
				       DMA_TO_DEVICE);
	}
	if (!(desc->txd.flags & DMA_COMPL_SKIP_DEST_UNMAP)) {
		if (desc->txd.flags & DMA_COMPL_DEST_UNMAP_SINGLE)
			dma_unmap_single(dev, desc->dst_addr, desc->size,
					 DMA_FROM_DEVICE);
		else
			dma_unmap_page(dev, desc->dst_addr, desc->size,
				       DMA_FROM_DEVICE);
	}
	LEAVE();	
}

static void nuc970_dma_tasklet(unsigned long data)
{
	struct nuc970_dma_chan *edmac = (struct nuc970_dma_chan *)data;
	struct nuc970_dma_desc *desc, *d;
	dma_async_tx_callback callback = NULL;
	void *callback_param = NULL;
	LIST_HEAD(list);
	ENTRY();
	spin_lock_irq(&edmac->lock);
	/*
	 * If dma_terminate_all() was called before we get to run, the active
	 * list has become empty. If that happens we aren't supposed to do
	 * anything more than call nuc970_dma_advance_work().
	 */
	desc = nuc970_dma_get_active(edmac);
	if (desc) {
		if (desc->complete) {
			/* mark descriptor complete for non cyclic case only */
			if (!test_bit(NUC970_DMA_IS_CYCLIC, &edmac->flags))
				dma_cookie_complete(&desc->txd);
			list_splice_init(&edmac->active, &list);
		}
		callback = desc->txd.callback;
		callback_param = desc->txd.callback_param;
	}
	spin_unlock_irq(&edmac->lock);

	/* Pick up the next descriptor from the queue */
	nuc970_dma_advance_work(edmac);

	/* Now we can release all the chained descriptors */
	list_for_each_entry_safe(desc, d, &list, node) {
		/*
		 * For the memcpy channels the API requires us to unmap the
		 * buffers unless requested otherwise.
		 */
		if (!edmac->chan.private)
			nuc970_dma_unmap_buffers(desc);

		nuc970_dma_desc_put(edmac, desc);
	}

	if (callback)
		callback(callback_param);
	LEAVE();	
}

static irqreturn_t nuc970_dma_interrupt(int irq, void *dev_id)
{
	struct nuc970_dma_chan *edmac = dev_id;
	struct nuc970_dma_desc *desc;
	irqreturn_t ret = IRQ_HANDLED;
	ENTRY();	
	spin_lock(&edmac->lock);

	desc = nuc970_dma_get_active(edmac);
	if (!desc) {
		dev_warn(chan2dev(edmac),
			 "got interrupt while active list is empty\n");
		spin_unlock(&edmac->lock);
		return IRQ_NONE;
	}

	switch (edmac->edma->hw_interrupt(edmac)) {
	case INTERRUPT_DONE:
		desc->complete = true;
		tasklet_schedule(&edmac->tasklet);
		break;

	case INTERRUPT_NEXT_BUFFER:
		if (test_bit(NUC970_DMA_IS_CYCLIC, &edmac->flags))
			tasklet_schedule(&edmac->tasklet);
		break;

	default:
		dev_warn(chan2dev(edmac), "unknown interrupt!\n");
		ret = IRQ_NONE;
		break;
	}

	spin_unlock(&edmac->lock);
	LEAVE();	
	return ret;
}

/**
 * nuc970_dma_tx_submit - set the prepared descriptor(s) to be executed
 * @tx: descriptor to be executed
 *
 * Function will execute given descriptor on the hardware or if the hardware
 * is busy, queue the descriptor to be executed later on. Returns cookie which
 * can be used to poll the status of the descriptor.
 */
static dma_cookie_t nuc970_dma_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct nuc970_dma_chan *edmac = to_nuc970_dma_chan(tx->chan);
	struct nuc970_dma_desc *desc;
	dma_cookie_t cookie;
	unsigned long flags;
	ENTRY();
	spin_lock_irqsave(&edmac->lock, flags);
	cookie = dma_cookie_assign(tx);

	desc = container_of(tx, struct nuc970_dma_desc, txd);

	/*
	 * If nothing is currently prosessed, we push this descriptor
	 * directly to the hardware. Otherwise we put the descriptor
	 * to the pending queue.
	 */
	if (list_empty(&edmac->active)) {
		nuc970_dma_set_active(edmac, desc);
		edmac->edma->hw_submit(edmac);
	} else {
		list_add_tail(&desc->node, &edmac->queue);
	}

	spin_unlock_irqrestore(&edmac->lock, flags);
	LEAVE();
	return cookie;
}

/**
 * nuc970_dma_alloc_chan_resources - allocate resources for the channel
 * @chan: channel to allocate resources
 *
 * Function allocates necessary resources for the given DMA channel and
 * returns number of allocated descriptors for the channel. Negative errno
 * is returned in case of failure.
 */
static int nuc970_dma_alloc_chan_resources(struct dma_chan *chan)
{
	struct nuc970_dma_chan *edmac = to_nuc970_dma_chan(chan);
	struct nuc970_dma_data *data = chan->private;
	const char *name = dma_chan_name(chan);
	int ret, i;
	ENTRY();
    DMA_DEBUG("name =%s\n", name);	
	/* Sanity check the channel parameters */
	if (data) {
		switch (data->port) {
		case NUC970_DMA_MEM:
			if (!is_slave_direction(data->direction))
				return -EINVAL;
			break;
		default:
				return -EINVAL;
			}
		}
	
	if (data && data->name)
		name = data->name;

	#if 0
	ret = clk_enable(edmac->clk);
	if (ret)
		return ret;
	#endif

	ret = request_irq(edmac->irq, nuc970_dma_interrupt, IRQF_DISABLED | IRQF_SHARED, name, edmac);
	if (ret)
		goto fail_clk_disable;

	spin_lock_irq(&edmac->lock);
	dma_cookie_init(&edmac->chan);
	ret = edmac->edma->hw_setup(edmac);
	spin_unlock_irq(&edmac->lock);

	if (ret)
		goto fail_free_irq;

	for (i = 0; i < DMA_MAX_CHAN_DESCRIPTORS; i++) {
		struct nuc970_dma_desc *desc;

		desc = kzalloc(sizeof(*desc), GFP_KERNEL);
		if (!desc) {
			dev_warn(chan2dev(edmac), "not enough descriptors\n");
			break;
		}

		INIT_LIST_HEAD(&desc->tx_list);

		dma_async_tx_descriptor_init(&desc->txd, chan);
		desc->txd.flags = DMA_CTRL_ACK;
		desc->txd.tx_submit = nuc970_dma_tx_submit;

		nuc970_dma_desc_put(edmac, desc);
	}
	DMA_DEBUG("return %d\n",i);
	LEAVE();
	return i;

fail_free_irq:
	free_irq(edmac->irq, edmac);
fail_clk_disable:
	//clk_disable(edmac->clk);
	LEAVE();
	return ret;
}

/**
 * nuc970_dma_free_chan_resources - release resources for the channel
 * @chan: channel
 *
 * Function releases all the resources allocated for the given channel.
 * The channel must be idle when this is called.
 */
static void nuc970_dma_free_chan_resources(struct dma_chan *chan)
{
	struct nuc970_dma_chan *edmac = to_nuc970_dma_chan(chan);
	struct nuc970_dma_desc *desc, *d;
	unsigned long flags;
	LIST_HEAD(list);
	ENTRY();
	BUG_ON(!list_empty(&edmac->active));
	BUG_ON(!list_empty(&edmac->queue));

	spin_lock_irqsave(&edmac->lock, flags);
	edmac->edma->hw_shutdown(edmac);
	edmac->runtime_addr = 0;
	edmac->runtime_ctrl = 0;
	edmac->buffer = 0;
	list_splice_init(&edmac->free_list, &list);
	spin_unlock_irqrestore(&edmac->lock, flags);

	list_for_each_entry_safe(desc, d, &list, node)
		kfree(desc);

	//clk_disable(edmac->clk);	
	free_irq(edmac->irq, edmac);
	LEAVE();	
}

/**
 * nuc970_dma_prep_dma_memcpy - prepare a memcpy DMA operation
 * @chan: channel
 * @dest: destination bus address
 * @src: source bus address
 * @len: size of the transaction
 * @flags: flags for the descriptor
 *
 * Returns a valid DMA descriptor or %NULL in case of failure.
 */
static struct dma_async_tx_descriptor *
nuc970_dma_prep_dma_memcpy(struct dma_chan *chan, dma_addr_t dest,
			   dma_addr_t src, size_t len, unsigned long flags)
{
	struct nuc970_dma_chan *edmac = to_nuc970_dma_chan(chan);
	struct nuc970_dma_desc *desc, *first;
	size_t bytes, offset;
	ENTRY();
	first = NULL;
	for (offset = 0; offset < len; offset += bytes) {
		desc = nuc970_dma_desc_get(edmac);
		if (!desc) {
			dev_warn(chan2dev(edmac), "couln't get descriptor\n");
			goto fail;
		}

		bytes = min_t(size_t, len - offset, DMA_MAX_CHAN_BYTES);

		desc->src_addr = src + offset;
		desc->dst_addr = dest + offset;
		desc->size = bytes;

		DMA_DEBUG("src_addr=0x%08x\n",desc->src_addr);
		DMA_DEBUG("dst_addr=0x%08x\n",desc->dst_addr);
		DMA_DEBUG("size=0x%08x\n",desc->size);
		DMA_DEBUG("offset=0x%08x\n",offset);

		if (!first)
			first = desc;
		else
			list_add_tail(&desc->node, &first->tx_list);
	}

	first->txd.cookie = -EBUSY;
	first->txd.flags = flags;
	LEAVE();

	return &first->txd;
fail:
	DMA_DEBUG("%s fail =>\n", __FUNCTION__);
	nuc970_dma_desc_put(edmac, first);
	LEAVE();	
	return NULL;
}

/**
 * nuc970_dma_prep_slave_sg - prepare a slave DMA operation
 * @chan: channel
 * @sgl: list of buffers to transfer
 * @sg_len: number of entries in @sgl
 * @dir: direction of tha DMA transfer
 * @flags: flags for the descriptor
 * @context: operation context (ignored)
 *
 * Returns a valid DMA descriptor or %NULL in case of failure.
 */
static struct dma_async_tx_descriptor *
nuc970_dma_prep_slave_sg(struct dma_chan *chan, struct scatterlist *sgl,
			 unsigned int sg_len, enum dma_transfer_direction dir,
			 unsigned long flags, void *context)
{
	struct nuc970_dma_chan *edmac = to_nuc970_dma_chan(chan);
	struct nuc970_dma_desc *desc, *first;
	struct scatterlist *sg;
	int i;
	ENTRY();
#if 0
	if (!edmac->edma->m2m && dir != nuc970_dma_chan_direction(chan)) {
		dev_warn(chan2dev(edmac),
			 "channel was configured with different direction\n");
		return NULL;
	}
#endif

	if (test_bit(NUC970_DMA_IS_CYCLIC, &edmac->flags)) {
		dev_warn(chan2dev(edmac),
			 "channel is already used for cyclic transfers\n");
		return NULL;
	}

	first = NULL;
	for_each_sg(sgl, sg, sg_len, i) {
		size_t sg_len = sg_dma_len(sg);

		if (sg_len > DMA_MAX_CHAN_BYTES) {
			dev_warn(chan2dev(edmac), "too big transfer size %d\n",
				 sg_len);
			goto fail;
		}

		desc = nuc970_dma_desc_get(edmac);
		if (!desc) {
			dev_warn(chan2dev(edmac), "couln't get descriptor\n");
			goto fail;
		}

		if (dir == DMA_MEM_TO_DEV) {
			desc->src_addr = sg_dma_address(sg);
			desc->dst_addr = edmac->runtime_addr;
		} else {
			desc->src_addr = edmac->runtime_addr;
			desc->dst_addr = sg_dma_address(sg);
		}
		desc->size = sg_len;

		if (!first)
			first = desc;
		else
			list_add_tail(&desc->node, &first->tx_list);
	}

	first->txd.cookie = -EBUSY;
	first->txd.flags = flags;
	LEAVE();
	return &first->txd;

fail:
	nuc970_dma_desc_put(edmac, first);
	LEAVE();	
	return NULL;
}

/**
 * nuc970_dma_prep_dma_cyclic - prepare a cyclic DMA operation
 * @chan: channel
 * @dma_addr: DMA mapped address of the buffer
 * @buf_len: length of the buffer (in bytes)
 * @period_len: length of a single period
 * @dir: direction of the operation
 * @flags: tx descriptor status flags
 * @context: operation context (ignored)
 *
 * Prepares a descriptor for cyclic DMA operation. This means that once the
 * descriptor is submitted, we will be submitting in a @period_len sized
 * buffers and calling callback once the period has been elapsed. Transfer
 * terminates only when client calls dmaengine_terminate_all() for this
 * channel.
 *
 * Returns a valid DMA descriptor or %NULL in case of failure.
 */
static struct dma_async_tx_descriptor *
nuc970_dma_prep_dma_cyclic(struct dma_chan *chan, dma_addr_t dma_addr,
			   size_t buf_len, size_t period_len,
			   enum dma_transfer_direction dir, unsigned long flags,
			   void *context)
{
	struct nuc970_dma_chan *edmac = to_nuc970_dma_chan(chan);
	struct nuc970_dma_desc *desc, *first;
	size_t offset = 0;
	ENTRY();
	#if 0
	if (dir != nuc970_dma_chan_direction(chan)) {
		dev_warn(chan2dev(edmac),
			 "channel was configured with different direction\n");
		return NULL;
	}
	#endif
	if (test_and_set_bit(NUC970_DMA_IS_CYCLIC, &edmac->flags)) {
		dev_warn(chan2dev(edmac),
			 "channel is already used for cyclic transfers\n");
		return NULL;
	}

	if (period_len > DMA_MAX_CHAN_BYTES) {
		dev_warn(chan2dev(edmac), "too big period length %d\n",
			 period_len);
		return NULL;
	}

	/* Split the buffer into period size chunks */
	first = NULL;
	for (offset = 0; offset < buf_len; offset += period_len) {
		desc = nuc970_dma_desc_get(edmac);
		if (!desc) {
			dev_warn(chan2dev(edmac), "couln't get descriptor\n");
			goto fail;
		}

		if (dir == DMA_MEM_TO_MEM) {
			desc->src_addr = edmac->runtime_addr;
			desc->dst_addr = dma_addr + offset;
		}

		desc->size = period_len;

		if (!first)
			first = desc;
		else
			list_add_tail(&desc->node, &first->tx_list);
	}

	first->txd.cookie = -EBUSY;
	LEAVE();
	return &first->txd;

fail:
	nuc970_dma_desc_put(edmac, first);
	LEAVE();	
	return NULL;
}

/**
 * nuc970_dma_terminate_all - terminate all transactions
 * @edmac: channel
 *
 * Stops all DMA transactions. All descriptors are put back to the
 * @edmac->free_list and callbacks are _not_ called.
 */
static int nuc970_dma_terminate_all(struct nuc970_dma_chan *edmac)
{
	struct nuc970_dma_desc *desc, *_d;
	unsigned long flags;
	LIST_HEAD(list);
	ENTRY();
	spin_lock_irqsave(&edmac->lock, flags);
	/* First we disable and flush the DMA channel */
	edmac->edma->hw_shutdown(edmac);
	clear_bit(NUC970_DMA_IS_CYCLIC, &edmac->flags);
	list_splice_init(&edmac->active, &list);
	list_splice_init(&edmac->queue, &list);
	/*
	 * We then re-enable the channel. This way we can continue submitting
	 * the descriptors by just calling ->hw_submit() again.
	 */
	edmac->edma->hw_setup(edmac);
	spin_unlock_irqrestore(&edmac->lock, flags);

	list_for_each_entry_safe(desc, _d, &list, node)
		nuc970_dma_desc_put(edmac, desc);
	LEAVE();
	return 0;
}

static int nuc970_dma_slave_config(struct nuc970_dma_chan *edmac,
				   struct dma_slave_config *config)
{
	enum dma_slave_buswidth width;
		unsigned long flags;
		//u32 addr;
		u32 ctrl;
      
	ENTRY();
	width = config->src_addr_width;
	switch (width) {
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		ctrl = TWS_8BIT;
		break;
	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		ctrl = TWS_16BIT;
		break;
	case DMA_SLAVE_BUSWIDTH_4_BYTES:
		ctrl = TWS_32BIT;
		break;
	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&edmac->lock, flags);
	edmac->runtime_ctrl = ctrl;
	spin_unlock_irqrestore(&edmac->lock, flags);	
	LEAVE();	
	return 0;
}

/**
 * nuc970_dma_control - manipulate all pending operations on a channel
 * @chan: channel
 * @cmd: control command to perform
 * @arg: optional argument
 *
 * Controls the channel. Function returns %0 in case of success or negative
 * error in case of failure.
 */
static int nuc970_dma_control(struct dma_chan *chan, enum dma_ctrl_cmd cmd,
			      unsigned long arg)
{
	struct nuc970_dma_chan *edmac = to_nuc970_dma_chan(chan);
	struct dma_slave_config *config;
	ENTRY();
	switch (cmd) {
	case DMA_TERMINATE_ALL:
		return nuc970_dma_terminate_all(edmac);

	case DMA_SLAVE_CONFIG:
		config = (struct dma_slave_config *)arg;
		return nuc970_dma_slave_config(edmac, config);

	default:
		break;
	}
	LEAVE();
	return -ENOSYS;
}

/**
 * nuc970_dma_tx_status - check if a transaction is completed
 * @chan: channel
 * @cookie: transaction specific cookie
 * @state: state of the transaction is stored here if given
 *
 * This function can be used to query state of a given transaction.
 */
static enum dma_status nuc970_dma_tx_status(struct dma_chan *chan,
					    dma_cookie_t cookie,
					    struct dma_tx_state *state)
{
	struct nuc970_dma_chan *edmac = to_nuc970_dma_chan(chan);
	enum dma_status ret;
	unsigned long flags;
	ENTRY();
	spin_lock_irqsave(&edmac->lock, flags);
	ret = dma_cookie_status(chan, cookie, state);
	spin_unlock_irqrestore(&edmac->lock, flags);
	LEAVE();
	return ret;
}

/**
 * nuc970_dma_issue_pending - push pending transactions to the hardware
 * @chan: channel
 *
 * When this function is called, all pending transactions are pushed to the
 * hardware and executed.
 */
static void nuc970_dma_issue_pending(struct dma_chan *chan)
{
	ENTRY();
	nuc970_dma_advance_work(to_nuc970_dma_chan(chan));
	LEAVE();	
}

#define CLK_HCLKEN_GDMA (1<<12)

static int __init nuc970_dma_probe(struct platform_device *pdev)
{
	struct nuc970_dma_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct nuc970_dma_engine *edma;
	struct dma_device *dma_dev;
	size_t edma_size;
	struct clk *clk;
	int ret, i;
	ENTRY();
	//DMA_DEBUG("%s pdev->dev=%s\n", __func__,pdev->dev);
	edma_size = pdata->num_channels * sizeof(struct nuc970_dma_chan);
	edma = kzalloc(sizeof(*edma) + edma_size, GFP_KERNEL);
	if (!edma)
	{
		DMA_DEBUG("NUC970 GDMA -ENOMEM\n");
		return -ENOMEM;
	}
	DMA_DEBUG("NUC970 GDMA !!!\n");
	
  	/* enable gdma clock */ 
	#if 0
	__raw_writel(__raw_readl(REG_CLK_HCLKEN)| CLK_HCLKEN_GDMA ,REG_CLK_HCLKEN);
	#else
	clk = clk_get(NULL, "gdma_hclk");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "cannot get clock\n");		
		return -ENOENT;
	}
	dev_dbg(&pdev->dev, "clock source %p\n", clk);	
	clk_prepare(clk);
	clk_enable(clk);
	#endif	
	dma_dev = &edma->dma_dev;
	edma->num_channels = pdata->num_channels;

	INIT_LIST_HEAD(&dma_dev->channels);
	for (i = 0; i < pdata->num_channels; i++) {
		const struct nuc970_dma_chan_data *cdata = &pdata->channels[i];
		struct nuc970_dma_chan *edmac = &edma->channels[i];

		edmac->chan.device = dma_dev;
		edmac->chan.private = 0;
		edmac->regs = cdata->base;
		edmac->irq = cdata->irq;
		edmac->edma = edma;

		spin_lock_init(&edmac->lock);
		INIT_LIST_HEAD(&edmac->active);
		INIT_LIST_HEAD(&edmac->queue);
		INIT_LIST_HEAD(&edmac->free_list);
		tasklet_init(&edmac->tasklet, nuc970_dma_tasklet,
			     (unsigned long)edmac);

		list_add_tail(&edmac->chan.device_node,
			      &dma_dev->channels);
	}

	dma_cap_zero(dma_dev->cap_mask);
	dma_cap_set(DMA_SLAVE, dma_dev->cap_mask);
	dma_cap_set(DMA_CYCLIC, dma_dev->cap_mask);

	dma_dev->dev = &pdev->dev;
	dma_dev->device_alloc_chan_resources = nuc970_dma_alloc_chan_resources;
	dma_dev->device_free_chan_resources = nuc970_dma_free_chan_resources;
	dma_dev->device_prep_slave_sg = nuc970_dma_prep_slave_sg;
	dma_dev->device_prep_dma_cyclic = nuc970_dma_prep_dma_cyclic;
	dma_dev->device_control = nuc970_dma_control;
	dma_dev->device_issue_pending = nuc970_dma_issue_pending;
	dma_dev->device_tx_status = nuc970_dma_tx_status;

	dma_set_max_seg_size(dma_dev->dev, DMA_MAX_CHAN_BYTES);

	dma_cap_set(DMA_MEMCPY, dma_dev->cap_mask);
	dma_dev->device_prep_dma_memcpy = nuc970_dma_prep_dma_memcpy;

	edma->hw_setup = m2m_hw_setup;
	edma->hw_shutdown = m2m_hw_shutdown;
	edma->hw_submit = m2m_hw_submit;
	edma->hw_interrupt = m2m_hw_interrupt;

	ret = dma_async_device_register(dma_dev);
	if (unlikely(ret)) {
		#if 0
		for (i = 0; i < edma->num_channels; i++) {
			struct nuc970_dma_chan *edmac = &edma->channels[i];
			if (!IS_ERR_OR_NULL(edmac->clk))
				clk_put(edmac->clk);
		}
		#endif
		kfree(edma);
	} else {
		dev_info(dma_dev->dev, "NUC970 M2M DMA ready\n");
	}
	LEAVE();	
	return ret;
}

static struct platform_device_id nuc970_dma_driver_ids[] = {	
	{ "nuc970-dma-m2m", 0 },
	{ },
};

static struct platform_driver nuc970_dma_driver = {
	.driver		= {
		.name	= "nuc970-dma",
		.owner	= THIS_MODULE,
	},
	.probe		= nuc970_dma_probe,
	.id_table	= nuc970_dma_driver_ids,
};
module_platform_driver(nuc970_dma_driver);


MODULE_AUTHOR("SChung <schung@nuvoton.com>");
MODULE_DESCRIPTION("NUC970 DMA driver");
MODULE_LICENSE("GPL");
