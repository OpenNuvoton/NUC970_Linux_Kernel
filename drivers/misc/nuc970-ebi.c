/* linux/driver/misc/nuc970-ebi.c
 *
 * Copyright (c) 2015 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#if 0
#define ENTRY()                                 printk("[%-20s] : Enter...\n", __FUNCTION__)
#define LEAVE()                                 printk("[%-20s] : Leave...\n", __FUNCTION__)
#else
#define ENTRY()
#define LEAVE()
#endif

#if 0
#define DEBUG(fmt, args...)     printk(fmt, ##args)
#else
#define DEBUG(fmt, args...)     do {} while (0)
#endif



#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <mach/map.h>
#include <mach/regs-gcr.h>
#include <mach/regs-clock.h>
#include <mach/regs-ebi.h>
#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/of.h>


#define SRAM_GRANULARITY	32

struct ebi_dev {
	int minor;	// dynamic minor num, so we need this to distinguish between channels
	struct pinctrl *pinctrl;
	struct clk *clk;
	struct clk *hclk;
	unsigned long base_addr[4];
	int bank;
};

static struct ebi_dev *ebi;

static long ebi_ioctl(struct file *filp, unsigned int cmd, unsigned long arg){
	struct nuc970_set_ebi *pebi,sebi;
	struct ebi_dev *nuc970_ebi = (struct ebi_dev *)filp->private_data;
	pebi=&sebi;
	switch(cmd) {
		case EBI_IOC_SET:
			if(copy_from_user((void *)&sebi, (const void *)arg, sizeof(struct nuc970_set_ebi)))
				return -EFAULT;
			DEBUG("set bank=%d\n",pebi->bank);
			pebi=(struct nuc970_set_ebi *)(arg);
			nuc970_set_ebi_mode(pebi->bank,pebi->mode);
			nuc970_set_ebi_attrib(pebi->bank,pebi->base,pebi->size,pebi->width);
			nuc970_set_ebi_timing(pebi->bank,pebi->tACC,pebi->tCOH,pebi->tACS,pebi->tCOS);
			nuc970_ebi->bank = pebi->bank;
			nuc970_ebi->base_addr[nuc970_ebi->bank] = pebi->base;
		break;
	}
	return 0;
}
static int ebi_open(struct inode *inode, struct file *filp){
	#ifndef CONFIG_OF
	struct pinctrl_state *s=NULL;
	int ret;
	#endif
	
	ENTRY();
	filp->private_data = ebi;
	ebi->hclk = clk_get(NULL, "ebi_hclk");	
	if (IS_ERR(ebi->hclk)) {
		printk("failed to get ebi clock mux\n");
		return -EAGAIN;
	}
	clk_prepare(ebi->hclk);
	clk_enable(ebi->hclk);
	
	#ifndef CONFIG_OF
	s = pinctrl_lookup_state(ebi->pinctrl, "ebi-16bit-0");	//ebi 16bit cs0
       if (IS_ERR(s)) {
                printk("pinctrl_lookup_state err\n");
                return -EPERM;
        }

	if((ret = pinctrl_select_state(ebi->pinctrl, s)) < 0) {
		printk("pinctrl_select_state err\n");
		return ret;
	}
	#endif
	LEAVE();
	return 0;
}
//static unsigned int ebi_poll(struct file *filp, poll_table *wait){return 0;}
static ssize_t ebi_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos){return 0;}
static int ebi_release(struct inode *inode, struct file *filp){
	ENTRY();
	clk_disable(ebi->hclk);
	clk_put(ebi->hclk);
	filp->private_data = NULL;
	LEAVE();
	return 0;
}
static int ebi_mmap(struct file *filp, struct vm_area_struct * vma);
struct file_operations ebi_fops =
{
	.owner		= THIS_MODULE,
	.open		= ebi_open,
	.release	= ebi_release,
	.read		= ebi_read,
	.mmap = ebi_mmap,
	.unlocked_ioctl	= ebi_ioctl,
};

static struct miscdevice ebi_dev[] = {
	[0] = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "ebi",
		.fops = &ebi_fops,
	},
};

;
static int ebi_mmap(struct file *filp, struct vm_area_struct * vma){
	struct ebi_dev *nuc970_ebi = (struct ebi_dev *)filp->private_data;
	unsigned long pageFrameNo = 0, size;
#if 0
	unsigned long virt_addr,phys_addr;
	static unsigned int physical_address;
	static unsigned int virtual_address;
	size = vma->vm_end - vma->vm_start;
	ENTRY();
	virt_addr = (unsigned long)dma_alloc_writecombine(NULL, size,
                                (unsigned int *) &phys_addr,
                                GFP_KERNEL);
	pageFrameNo = __phys_to_pfn(phys_addr);
	if (!virt_addr) {
		printk(KERN_INFO "kmalloc() failed !\n");
		return -EINVAL;
	}
	DEBUG("MMAP_KMALLOC : virt addr = 0x%08x, size = %d, %d\n",virt_addr, size, __LINE__);
#else
	DEBUG("mmap: nuc970_ebi->bank=0x%08x\n",nuc970_ebi->bank);
	DEBUG("nuc970_ebi->base_addr=0x%08x\n",(unsigned int)nuc970_ebi->base_addr[nuc970_ebi->bank]);
	pageFrameNo = __phys_to_pfn(nuc970_ebi->base_addr[nuc970_ebi->bank]);
	ENTRY();
#endif

	size = vma->vm_end - vma->vm_start;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_flags |= (VM_DONTEXPAND | VM_DONTDUMP);
	if (remap_pfn_range(vma, vma->vm_start, pageFrameNo,size, vma->vm_page_prot)) {
		printk(KERN_INFO "nuc970_mem_mmap() : remap_pfn_range() failed !\n");
		return -EINVAL;
	}

  DEBUG("REG_EBI_CTL=0x%08x\n",__raw_readl(REG_EBI_CTL));
  DEBUG("REG_EBI_BNKCTL0=0x%08x\n",__raw_readl(REG_EBI_BNKCTL(0)));
  DEBUG("REG_EBI_BNKCTL1=0x%08x\n",__raw_readl(REG_EBI_BNKCTL(1)));
  DEBUG("REG_EBI_BNKCTL2=0x%08x\n",__raw_readl(REG_EBI_BNKCTL(2)));
  DEBUG("REG_EBI_BNKCTL3=0x%08x\n",__raw_readl(REG_EBI_BNKCTL(3)));
  DEBUG("REG_MFP_GPD_H=0x%08x\n",__raw_readl(REG_MFP_GPD_H));
  DEBUG("REG_MFP_GPH_L=0x%08x\n",__raw_readl(REG_MFP_GPH_L));
  DEBUG("REG_MFP_GPH_H=0x%08x\n",__raw_readl(REG_MFP_GPH_H));
  DEBUG("REG_MFP_GPI_L=0x%08x\n",__raw_readl(REG_MFP_GPI_L));
  DEBUG("REG_MFP_GPI_H=0x%08x\n",__raw_readl(REG_MFP_GPI_H));
 	LEAVE();
	return 0;
}
	
static int nuc970_ebi_probe(struct platform_device *pdev)
{
	struct ebi_dev *nuc970_ebi;
	ENTRY();
	printk("%s - pdev = %s\n", __func__, pdev->name);
	nuc970_ebi = devm_kzalloc(&pdev->dev, sizeof(*nuc970_ebi), GFP_KERNEL);
	if (!nuc970_ebi)
		return -ENOMEM;

	nuc970_ebi->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(nuc970_ebi->clk))
		nuc970_ebi->clk = NULL;
	else
		clk_prepare_enable(nuc970_ebi->clk);
		
	misc_register(&ebi_dev[0]);
	nuc970_ebi->pinctrl = devm_pinctrl_get(&pdev->dev);
	nuc970_ebi->minor = MINOR(ebi_dev[0].minor);
	
	#ifdef CONFIG_OF
	{
		struct pinctrl *pinctrl;
		pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
		if (IS_ERR(pinctrl)) {
			return PTR_ERR(pinctrl);
		}
	}
	#endif
	
	DEBUG("nuc970_ebi->minor=%d\n",nuc970_ebi->minor);
	ebi=nuc970_ebi;
	platform_set_drvdata(pdev, nuc970_ebi);
	
	LEAVE();
	return 0;
}

static int nuc970_ebi_remove(struct platform_device *pdev)
{
	//struct ebi_dev *nuc970_ebi = platform_get_drvdata(pdev);
	ENTRY();
	misc_deregister(&ebi_dev[0]);
	LEAVE();
	return 0;
}

static const struct of_device_id nuc970_ebi_of_match[] = {
	{ .compatible = "nuvoton,nuc970-ebi" },
	{},
};

static struct platform_driver nuc970_ebi_driver = {
	.driver = {
		.name = "nuc970-ebi",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(nuc970_ebi_of_match),
	},
	.probe = nuc970_ebi_probe,
	.remove = nuc970_ebi_remove,
};

module_platform_driver(nuc970_ebi_driver);
MODULE_AUTHOR("Nuvoton Technology Corp.");
MODULE_ALIAS("platform:nuc970-ebi");
MODULE_LICENSE("GPL");

