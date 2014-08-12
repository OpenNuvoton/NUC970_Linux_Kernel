/*
 * Blackfin IrDA Driver
 *
 *
 */

#ifndef IRTTYSIR_H
#define IRTTYSIR_H

#include <net/irda/irda.h>
#include <net/irda/irda_device.h>	

#define IRTTY_IOC_MAGIC 'e'
#define IRTTY_IOCTDONGLE  _IO(IRTTY_IOC_MAGIC, 1)
#define IRTTY_IOCGET     _IOR(IRTTY_IOC_MAGIC, 2, struct irtty_info)
#define IRTTY_IOC_MAXNR   2

struct sirtty_cb {
	magic_t magic;

	struct sir_dev *dev;
	struct tty_struct  *tty;

	chipio_t io;               /* IrDA controller information */
};

#endif


