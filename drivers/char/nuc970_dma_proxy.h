/* drivers/char/nuc970_dma_proxy.h
 *
 * Copyright (c) 2015 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
*/

 
#define USE_DMA_ALLOC 1
#define NUC970_DMA_ALLOC 		1
#define NCU970_DMA_TRANSFER 2

#define MEM_ALLOC			1
#define MEM_ALLOC_SHARE			2
#define MEM_ALLOC_CACHEABLE		3
#define MEM_ALLOC_CACHEABLE_SHARE	4
						
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
