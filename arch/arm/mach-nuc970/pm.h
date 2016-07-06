/* linux/arch/arm/mach-nuc970/pm.c
 *
 * Copyright (c) 2016 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef _MACH_NUC970_PM_H_
#define _MACH_NUC970_PM_H_

#ifdef CONFIG_PM_SLEEP
void nuc970_init_suspend(void);
#else
static inline void nuc970_init_suspend(void) {}
#endif

#endif /* _MACH_NUC970_PM_H_ */
