/*
 * /linux/platform_data/keypad-nuc970.h
 *
 * Copyright (c) 2014 Nuvoton technology corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */


#ifndef __ASM_ARCH_NUC970_KEYPAD_H
#define __ASM_ARCH_NUC970_KEYPAD_H

#include <linux/input/matrix_keypad.h>

struct nuc970_keypad_platform_data {
	const struct matrix_keymap_data *keymap_data;

	unsigned int	prescale;
	unsigned int	debounce;
};

#endif /* __ASM_ARCH_W90P910_KEYPAD_H */
