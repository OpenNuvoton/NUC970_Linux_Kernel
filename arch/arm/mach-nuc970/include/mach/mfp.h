/*
 * linux/arch/arm/mach-nuc970/include/mach/clock.c
 *
 * Copyright (c) 2014 Nuvoton technology corporation
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */


#ifndef __ASM_ARCH_MFP_H
#define __ASM_ARCH_MFP_H

extern void nuc970_mfp_set_port_a(u32 pin, u32 func);
extern void nuc970_mfp_set_port_b(u32 pin, u32 func);
extern void nuc970_mfp_set_port_c(u32 pin, u32 func);
extern void nuc970_mfp_set_port_d(u32 pin, u32 func);
extern void nuc970_mfp_set_port_e(u32 pin, u32 func);
extern void nuc970_mfp_set_port_f(u32 pin, u32 func);
extern void nuc970_mfp_set_port_g(u32 pin, u32 func);
extern void nuc970_mfp_set_port_h(u32 pin, u32 func);
extern void nuc970_mfp_set_port_i(u32 pin, u32 func);


#endif /* __ASM_ARCH_MFP_H */
