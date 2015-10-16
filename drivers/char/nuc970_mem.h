/* drivers/char/nuc970_mem.h
 *
 * Copyright (c) 2015 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
*/

#define DEBUG_NUC970_MEM
#undef	DEBUG_NUC970_MEM

#if 0
#define DEBUG(fmt, args...)	printk(fmt, ##args)
#else
#define DEBUG(fmt, args...)	do {} while (0)
#endif

#define MEM_IOCTL_MAGIC		'M'

#define NUC970_MEM_ALLOC		_IOWR(MEM_IOCTL_MAGIC, 310,\
						struct nuc970_mem_alloc)
#define NUC970_MEM_FREE		_IOWR(MEM_IOCTL_MAGIC, 311,\
						struct nuc970_mem_alloc)

#define NUC970_MEM_SHARE_ALLOC	_IOWR(MEM_IOCTL_MAGIC, 314,\
						struct nuc970_mem_alloc)
#define NUC970_MEM_SHARE_FREE	_IOWR(MEM_IOCTL_MAGIC, 315,\
						struct nuc970_mem_alloc)

#define NUC970_MEM_CACHEABLE_ALLOC	_IOWR(MEM_IOCTL_MAGIC, 316,\
						struct nuc970_mem_alloc)
#define NUC970_MEM_CACHEABLE_SHARE_ALLOC	_IOWR(MEM_IOCTL_MAGIC, 317,\
						struct nuc970_mem_alloc)

#define MEM_ALLOC			1
#define MEM_ALLOC_SHARE			2
#define MEM_ALLOC_CACHEABLE		3
#define MEM_ALLOC_CACHEABLE_SHARE	4

#define NUC970_MEM_MINOR			13
#define USE_DMA_ALLOC

static DEFINE_MUTEX(mem_alloc_lock);
static DEFINE_MUTEX(mem_free_lock);

static DEFINE_MUTEX(mem_share_alloc_lock);
static DEFINE_MUTEX(mem_share_free_lock);

static DEFINE_MUTEX(mem_cacheable_alloc_lock);
static DEFINE_MUTEX(mem_cacheable_share_alloc_lock);

struct nuc970_mem_alloc {
	int		size;
	unsigned int	vir_addr;
	unsigned int	phy_addr;

#ifdef USE_DMA_ALLOC
	unsigned int	kvir_addr;
#endif
};

struct nuc970_mem_dma_param {
	int		size;
	unsigned int	src_addr;
	unsigned int	dst_addr;
	int		cfg;
};

int nuc970_mem_mmap(struct file *filp, struct vm_area_struct *vma);
long nuc970_mem_ioctl(struct file *file, unsigned int cmd, unsigned long arg);