#ifndef __ASM_ARCH_NUC970_I2C_H
#define __ASM_ARCH_NUC970_I2C_H

extern void nuc970_mfp_set_port_g(u32 pin, u32 func);

struct nuc970_platform_i2c {
	int		bus_num;
	unsigned long   bus_freq;
};

#endif /* __ASM_ARCH_NUC970_I2C_H */
