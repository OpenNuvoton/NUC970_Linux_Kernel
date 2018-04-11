/* linux/arch/arm/mach-nuc970/include/mach/nuc970-ebi.h
 *
 * Copyright (c) 2014 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#ifndef _NUC970_EBI_H_
#define _NUC970_EBI_H_

#include <linux/types.h>
#include <linux/ioctl.h>

#define EBI_IOC_MAGIC		'e'
#define EBI_IOC_SET			_IOW(EBI_IOC_MAGIC, 0, unsigned int *)

#define NUC970_EBI_8BIT         0x00000001
#define NUC970_EBI_16BIT        0x00000002

#define NUC970_EBI_80TYPE_nBE_WRITE     0x00000000
#define NUC970_EBI_80TYPE_nWE_WRITE     0x01000000
#define NUC970_EBI_68TYPE               0x00080000


struct nuc970_set_ebi {
        unsigned int bank;
        unsigned int mode;
        unsigned int base;
        unsigned int size;
        unsigned int width;
        unsigned int tACC;
        unsigned int tCOH;
        unsigned int tACS;
        unsigned int tCOS;
};

#endif

