#ifndef __ASM_ARCH_NUC970_KEYPAD_H
#define __ASM_ARCH_NUC970_KEYPAD_H

#include <linux/input/matrix_keypad.h>

struct nuc970_keypad_platform_data {
	const struct matrix_keymap_data *keymap_data;

	unsigned int	prescale;
	unsigned int	debounce;
};

#endif /* __ASM_ARCH_W90P910_KEYPAD_H */
