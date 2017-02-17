/* drivers/char/nuc970_dma_proxy.c
 *
 * Copyright (c) 2015 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
*/

#include <linux/dmaengine.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>
#include <mach/map.h>
#include <mach/regs-clock.h>
#include "nuc970_dma_proxy.h"

#define CLK_HCLKEN_GDMA (1<<12)

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

MODULE_LICENSE("GPL");

#define DRIVER_NAME 		"nuc970_dma"
#define CHANNEL_COUNT 		1
#define ERROR 			-1
#define NOT_LAST_CHANNEL 	0
#define LAST_CHANNEL 		1

/* The following data structure represents a single channel of DMA, transmit or receive in the case
 * when using DMA.  It contains all the data to be maintained for the channel.
 */
struct dma_proxy_channel {
	struct device *proxy_device_p;				/* character device support */
	struct device *dma_device_p;
	dev_t dev_node;
	struct cdev cdev;
	struct class *class_p;

	struct dma_chan *channel_p;				/* dma support */
	struct completion cmp;
	dma_cookie_t cookie;
	dma_addr_t dma_handle;
	u32 direction;						/* DMA_MEM_TO_MEM */
};

/* Allocate the channels for this example statically rather than dynamically for simplicity.
 */
static struct dma_proxy_channel channels;



static int flag;
static unsigned int physical_address;
static unsigned int virtual_address;


/* Map the memory for the channel interface into user space such that user space can
 * access it taking into account if the memory is not cached.
 */
static int nuc970_dma_mmap(struct file *file_p, struct vm_area_struct *vma)
{
	//struct dma_proxy_channel *pchannel_p = (struct dma_proxy_channel *)file_p->private_data;
		unsigned long virt_addr,phys_addr;
		virt_addr = (unsigned long)dma_alloc_writecombine(NULL, vma->vm_end - vma->vm_start,
				(unsigned int *) &phys_addr,
				GFP_KERNEL);	 
		physical_address = (unsigned int)phys_addr;			
		virtual_address = virt_addr;
		
		return dma_mmap_writecombine(NULL, vma,
				     (void *)virt_addr,
				     (dma_addr_t)phys_addr,
				     (size_t)(vma->vm_end - vma->vm_start));
}

/* Open the device file and set up the data pointer to the proxy channel data for the
 * proxy channel such that the ioctl function can access the data structure later.
 */
static int nuc970_dma_open(struct inode *ino, struct file *file)
{
	ENTRY();
	file->private_data = container_of(ino->i_cdev, struct dma_proxy_channel, cdev);
	LEAVE();
	return 0;
}

/* Close the file and there's nothing to do for it
 */
static int nuc970_dma_release(struct inode *ino, struct file *file)
{
	ENTRY();
	LEAVE();
	return 0;
}

struct nuc970_dma_done {
	bool			done;
	wait_queue_head_t	*wait;
};

static void nuc970_dma_callback(void *arg)
{
	struct nuc970_dma_done *done = arg;
	done->done = true;
}

#define NUC970_DMA_ALLOC 		1
#define NCU970_DMA_TRANSFER 	2
/* Perform I/O control to start a DMA transfer.
 */
 
 static long nuc970_dma_ioctl(struct file *file, unsigned int command , unsigned long arg)
{

	unsigned int cmd=*((unsigned int *)command);
	struct dma_proxy_channel *pchannel_p = (struct dma_proxy_channel *)file->private_data;
	ENTRY();
	
	switch(cmd)
	{
		case NUC970_DMA_ALLOC:
		{
			struct nuc970_mem_alloc param;
			if (copy_from_user(&param, (struct nuc970_mem_alloc *)arg,
						sizeof(struct nuc970_mem_alloc))) {
				return -EFAULT;
			}
			flag = MEM_ALLOC;
			param.phy_addr = physical_address;
			param.vir_addr = virtual_address;
			if (copy_to_user((struct nuc970_mem_alloc *)arg, &param,
						sizeof(struct nuc970_mem_alloc))) {
				flag = 0;
				return -EFAULT;
			}
		}
		break;
		case NCU970_DMA_TRANSFER:
		{
			int i;
			struct nuc970_dma_done	done;// = { .wait = &done_wait };
			struct nuc970_mem_dma_param *dma_param;
			struct dma_async_tx_descriptor *tx = NULL;
			dma_cookie_t		cookie;
									
			dma_param = kzalloc(sizeof(struct nuc970_mem_dma_param), GFP_KERNEL);
			if(copy_from_user(dma_param,(struct nuc970_mem_dma_param *)arg,
				                sizeof(struct nuc970_mem_dma_param))){
				 return -EFAULT;
			}	  
			
			for(i=0;i<dma_param->size;i++)
			 *((unsigned char *)virtual_address+i)=i;
			 
			tx = pchannel_p->channel_p->device->device_prep_dma_memcpy(pchannel_p->channel_p,
							 dma_param->dst_addr,
							 dma_param->src_addr, 
							 dma_param->size,
							 dma_param->cfg);
			if (!tx) {
				return -ENOMEM;				 
			}				
			done.done = false;
			tx->callback = nuc970_dma_callback;
			tx->callback_param = &done;
			cookie = tx->tx_submit(tx);
			while(dma_async_is_tx_complete(pchannel_p->channel_p, pchannel_p->cookie, NULL, NULL)!=0);
  	}
  	break;
	}

	LEAVE();
	return 0;
}

static struct file_operations dm_fops = {
	.owner    = THIS_MODULE,
	.open     = nuc970_dma_open,
	.mmap   = nuc970_dma_mmap,
	.release  = nuc970_dma_release,
	.unlocked_ioctl = nuc970_dma_ioctl,
};


/* Initialize the driver to be a character device such that is responds to
 * file operations.
 */
static int cdevice_init(struct dma_proxy_channel *pchannel_p)
{
	int rc;
	char device_name[32] = "dma_proxy";
	static struct class *local_class_p = NULL;
	ENTRY();
	/* Allocate a character device from the kernel for this
	 * driver
	 */
	rc = alloc_chrdev_region(&pchannel_p->dev_node, 0, 1, "dma_proxy");

	if (rc) {
		dev_err(pchannel_p->dma_device_p, "unable to get a char device number\n");
		return rc;
	}

	/* Initialize the ter device data structure before
	 * registering the character device with the kernel
	 */
	cdev_init(&pchannel_p->cdev, &dm_fops);
	pchannel_p->cdev.owner = THIS_MODULE;
	rc = cdev_add(&pchannel_p->cdev, pchannel_p->dev_node, 1);

	if (rc) {
		dev_err(pchannel_p->dma_device_p, "unable to add char device\n");
		goto init_error1;
	}

	/* Only one class in sysfs is to be created for multiple channels,
	 * create the device in sysfs which will allow the device node
	 * in /dev to be created
	 */
	if (!local_class_p) {
		local_class_p = class_create(THIS_MODULE, DRIVER_NAME);

		if (IS_ERR(pchannel_p->dma_device_p->class)) {
			dev_err(pchannel_p->dma_device_p, "unable to create class\n");
			rc = ERROR;
			goto init_error2;
		}
	}
	pchannel_p->class_p = local_class_p;

	/* Create the device node in /dev so the device is accessible
	 * as a character device
	 */
	pchannel_p->proxy_device_p = device_create(pchannel_p->class_p, NULL,
											   pchannel_p->dev_node, NULL, device_name);

	if (IS_ERR(pchannel_p->proxy_device_p)) {
		dev_err(pchannel_p->dma_device_p, "unable to create the device\n");
		goto init_error3;
	}
	LEAVE();
	return 0;

init_error3:
	class_destroy(pchannel_p->class_p);

init_error2:
	cdev_del(&pchannel_p->cdev);

init_error1:
	unregister_chrdev_region(pchannel_p->dev_node, 1);
	LEAVE();	
	return rc;
}

/* Exit the character device by freeing up the resources that it created and
 * disconnecting itself from the kernel.
 */
static void cdevice_exit(struct dma_proxy_channel *pchannel_p, int last_channel)
{
	/* Take everything down in the reverse order
	 * from how it was created for the char device
	 */
	ENTRY();	 
	if (pchannel_p->proxy_device_p) {
		device_destroy(pchannel_p->class_p, pchannel_p->dev_node);
		if (last_channel)
			class_destroy(pchannel_p->class_p);

		cdev_del(&pchannel_p->cdev);
		unregister_chrdev_region(pchannel_p->dev_node, 1);
	}
	LEAVE();	
}

/* Create a DMA channel by getting a DMA channel from the DMA Engine and then setting
 * up the channel as a character device to allow user space control.
 */
static int create_channel(struct dma_proxy_channel *pchannel_p, u32 direction)
{
	int rc;
	dma_cap_mask_t mask;
	ENTRY();
	/* Zero out the capability mask then initialize it for a slave channel that is
	 * private.
	 */
	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);

	/* Request the DMA channel from the DMA engine and then use the device from
	 * the channel for the proxy channel also.
	 */
	pchannel_p->channel_p = dma_request_channel(mask, NULL, NULL);
	if (!pchannel_p->channel_p) {
		dev_err(pchannel_p->dma_device_p, "DMA channel request error\n");
		return ERROR;
	}
	//pchannel_p->channel_p->private=1;
	pchannel_p->dma_device_p = &pchannel_p->channel_p->dev->device;

	/* Initialize the character device for the dma proxy channel
	 */
	rc = cdevice_init(pchannel_p);
	if (rc) {
		return rc;
	}

	pchannel_p->direction = direction;
	dma_set_coherent_mask(pchannel_p->proxy_device_p, 0xFFFFFFFF);

	LEAVE();	
	return 0;
}

/* Initialize the dma proxy device driver module.
 */
static int __init nuc970_dma_init(void)
{
	int rc =0;
	ENTRY();
	printk(KERN_INFO "nuc970_dma_proxy module initialized\n");

	if((__raw_readl(REG_CLK_HCLKEN)& CLK_HCLKEN_GDMA)==CLK_HCLKEN_GDMA)
	{
		rc = create_channel(&channels, DMA_MEM_TO_MEM);
		if (rc) {
			return rc;
		}
	}
	LEAVE();
	return 0;
}

/* Exit the dma proxy device driver module.
 */
static void __exit nuc970_dma_exit(void)
{
	//int i;
	ENTRY();
	printk(KERN_INFO "nuc970_dma_proxy module exited\n");

	/* Take care of the char device infrastructure for each
	 * channel except for the last channel. Handle the last
	 * channel seperately.
	 */
		if (channels.proxy_device_p)
			cdevice_exit(&channels, NOT_LAST_CHANNEL);
	cdevice_exit(&channels, LAST_CHANNEL);

	/* Take care of the DMA channels and the any buffers allocated
	 * for the DMA transfers. The DMA buffers are using managed
	 * memory such that it's automatically done.
	 */
		if (channels.channel_p)
			dma_release_channel(channels.channel_p);
	LEAVE();	
}

module_init(nuc970_dma_init);
module_exit(nuc970_dma_exit);
MODULE_LICENSE("GPL");
