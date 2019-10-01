/* linux/driver/crypto/nuc970-crypto-raw.c
 *
 * Copyright (c) 2018 Nuvoton Technology Corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>


#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <asm/io.h>
#include <asm/uaccess.h>

#include <mach/map.h>
#include <mach/regs-crypto.h>
#include <mach/nuc970-crypto.h>


extern struct nuc970_crypto_dev  nuc970_crdev;    /* declared in nuc970-crypto.c */


/*-----------------------------------------------------------------------------------------------*/
/*                                                                                               */
/*    AES                                                                                        */
/*                                                                                               */
/*-----------------------------------------------------------------------------------------------*/


// This function does not block, transaction complete in write(), this API is only for user to read back card response.
static ssize_t nvt_aes_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	volatile struct nuc970_crypto_regs  *crpt_regs = nuc970_crdev.regs;
	u32    t0;
	int    ret = 0;

	mutex_lock(&nuc970_crdev.aes_lock);

	t0 = jiffies;
	while (crpt_regs->CRPT_AES_STS & AES_BUSY) {
		if (jiffies - t0 >= 100) { /* 1s time-out */
			ret = -EFAULT;
			goto out;
		}
	}

	if (copy_to_user(buf, (u8 *)nuc970_crdev.aes_outbuf, count)) {
		ret = -EFAULT;
		goto out;
	}

out:
	mutex_unlock(&nuc970_crdev.aes_lock);
	return ret;
}


static ssize_t nvt_aes_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	volatile struct nuc970_crypto_regs  *crpt_regs = nuc970_crdev.regs;
	int   ret = 0;

	mutex_lock(&nuc970_crdev.aes_lock);

	if (copy_from_user((u8 *)nuc970_crdev.aes_inbuf, buf, count)) {
		ret = -EFAULT;
	} else {
		crpt_regs->CRPT_AES_CTL |= AES_START;
	}

	mutex_unlock(&nuc970_crdev.aes_lock);
	return ret;
}

static int nvt_aes_mmap(struct file *filp, struct vm_area_struct * vma)
{
	unsigned long pageFrameNo = 0, size;

	pageFrameNo = __phys_to_pfn(nuc970_crdev.aes_inbuf_dma_addr);

	size = vma->vm_end - vma->vm_start;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_flags |= (VM_DONTEXPAND | VM_DONTDUMP);
	if (remap_pfn_range(vma, vma->vm_start, pageFrameNo,size, vma->vm_page_prot)) {
		printk(KERN_INFO "nvt_aes_mmap() : remap_pfn_range() failed !\n");
		return -EINVAL;
	}
	return 0;
}


static long nvt_aes_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	volatile struct nuc970_crypto_regs  *crpt_regs = nuc970_crdev.regs;
	u32   t0, param;

	mutex_lock(&nuc970_crdev.aes_lock);

	switch(cmd) {
	case AES_IOC_SET_MODE:
		//printk("Set AES mode\n");
		crpt_regs->CRPT_AES_CTL = arg | AES_DMAEN;
		crpt_regs->CRPT_AES0_SADDR = nuc970_crdev.aes_inbuf_dma_addr;
		crpt_regs->CRPT_AES0_DADDR = nuc970_crdev.aes_outbuf_dma_addr;
		crpt_regs->CRPT_AES0_CNT = 16;   /* one AES block */
		break;

	case AES_IOC_SET_LEN:
		crpt_regs->CRPT_AES0_CNT = arg;
		break;

	case AES_IOC_SET_IV:
		copy_from_user((void *)&(crpt_regs->CRPT_AES0_IV[0]), (const void *)arg, 16);
		//printk("AES_IOC_SET_IV: 0x%x-0x%x-0x%x-0x%x\n", crpt_regs->CRPT_AES0_IV[0], crpt_regs->CRPT_AES0_IV[1], crpt_regs->CRPT_AES0_IV[2], crpt_regs->CRPT_AES0_IV[3]);
		break;

	case AES_IOC_SET_KEY:
		copy_from_user((void *)&(crpt_regs->CRPT_AES0_KEY[0]), (const void *)arg, 32);
		//printk("AES_IOC_SET_KEY: 0x%x-0x%x-0x%x-0x%x\n", crpt_regs->CRPT_AES0_KEY[0], crpt_regs->CRPT_AES0_KEY[1], crpt_regs->CRPT_AES0_KEY[2], crpt_regs->CRPT_AES0_KEY[3]);
		break;

	case AES_IOC_GET_BUFSIZE:
		param = nuc970_crdev.aes_inbuf_size * 2;    /* input buffer plus output buffer, they are contiguous */
		copy_to_user((void __user *)arg, (u8 *)&param, 4);
		break;

	case AES_IOC_START:
		crpt_regs->CRPT_AES_CTL = (crpt_regs->CRPT_AES_CTL & ~AES_DMACSCAD) | AES_START;
		t0 = jiffies;
		while (crpt_regs->CRPT_AES_STS & AES_BUSY) {
			if (jiffies - t0 >= 100) { /* 1s time-out */
				mutex_unlock(&nuc970_crdev.aes_lock);
				return -EFAULT;
			}
		}
		break;

	case AES_IOC_C_START:
		crpt_regs->CRPT_AES_CTL |= (AES_DMACSCAD | AES_START);
		t0 = jiffies;
		while (crpt_regs->CRPT_AES_STS & AES_BUSY) {
			if (jiffies - t0 >= 100) { /* 1s time-out */
				mutex_unlock(&nuc970_crdev.aes_lock);
				return -EFAULT;
			}
		}
		break;

	case AES_IOC_UPDATE_IV:
		copy_to_user((unsigned char *)arg, (unsigned char *)&(crpt_regs->CRPT_AES_FDBCK[0]), 32);
		break;

	default:
		mutex_unlock(&nuc970_crdev.aes_lock);
		return -ENOTTY;
	}
	mutex_unlock(&nuc970_crdev.aes_lock);
	return 0;
}


struct file_operations nvt_aes_fops = {
	.owner      = THIS_MODULE,
	.read       = nvt_aes_read,
	.write      = nvt_aes_write,
	.mmap       = nvt_aes_mmap,
	.unlocked_ioctl = nvt_aes_ioctl,
};

static struct miscdevice nvt_aes_dev = {
	.minor      = MISC_DYNAMIC_MINOR,
	.name       = "nuvoton-aes",
	.fops       = &nvt_aes_fops,
};


/*-----------------------------------------------------------------------------------------------*/
/*                                                                                               */
/*    SHA                                                                                        */
/*                                                                                               */
/*-----------------------------------------------------------------------------------------------*/

#define SHA_BUFF_SIZE       256
static unsigned char  sha_buffer[SHA_BUFF_SIZE];
static int   sha_remaining_cnt = 0;

static ssize_t nvt_sha_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	volatile struct nuc970_crypto_regs  *crpt_regs = nuc970_crdev.regs;
	u32    t0;
	int    ret = 0;

	if (sha_remaining_cnt != 0) {
		printk("SHA_IOC_FINISH missed!\n");
		return -EFAULT;
	}

	mutex_lock(&nuc970_crdev.sha_lock);

	t0 = jiffies;
	while (crpt_regs->CRPT_HMAC_STS & HMAC_BUSY) {
		if (jiffies - t0 >= 100) { /* 1s time-out */
			ret = -EFAULT;
			goto out;
		}
	}

	if (copy_to_user(buf, (u8 *)&crpt_regs->CRPT_HMAC_DGST[0], count)) {
		ret = -EFAULT;
		goto out;
	}

out:
	mutex_unlock(&nuc970_crdev.sha_lock);
	return ret;
}


static ssize_t nvt_sha_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	volatile struct nuc970_crypto_regs  *crpt_regs = nuc970_crdev.regs;
	u32   *data_ptr;
	int   rcnt, ret = 0;

	mutex_lock(&nuc970_crdev.sha_lock);

	while (count > 0) {
		rcnt = SHA_BUFF_SIZE - sha_remaining_cnt;

		if (count < rcnt)
			rcnt = count;

		if (copy_from_user(&sha_buffer[sha_remaining_cnt], buf, rcnt)) {
			ret = -EFAULT;
		}

		buf += rcnt;
		count -= rcnt;
		sha_remaining_cnt += rcnt;

		if ((sha_remaining_cnt == SHA_BUFF_SIZE) && (count > 0)) {
			/*
			 * If SHA buffer full and still have input data, flush the buffer to SHA engine.
			 */
			data_ptr = (u32 *)&sha_buffer[0];
			while (sha_remaining_cnt > 0) {
				if (crpt_regs->CRPT_HMAC_STS & HMAC_DINREQ) {
					crpt_regs->CRPT_HMAC_DATIN = *data_ptr++;
					sha_remaining_cnt -= 4;
				}
			}
			sha_remaining_cnt = 0;
		}
	}
	mutex_unlock(&nuc970_crdev.sha_lock);
	return ret;
}

static long nvt_sha_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	volatile struct nuc970_crypto_regs  *crpt_regs = nuc970_crdev.regs;
	u32   *data_ptr;

	mutex_lock(&nuc970_crdev.sha_lock);

	switch(cmd) {
	case SHA_IOC_INIT:
		sha_remaining_cnt = 0;
		crpt_regs->CRPT_HMAC_CTL = arg | HMAC_START;
		crpt_regs->CRPT_HMAC_DMACNT = 0x10000000;
		break;

	case SHA_IOC_FINISH:
		if (sha_remaining_cnt) {
			crpt_regs->CRPT_HMAC_DMACNT = sha_remaining_cnt;
			data_ptr = (u32 *)&sha_buffer[0];
			while (sha_remaining_cnt > 0) {
				if (crpt_regs->CRPT_HMAC_STS & HMAC_DINREQ) {
					if (sha_remaining_cnt <= 4) {
						crpt_regs->CRPT_HMAC_CTL |= HMAC_START | HMAC_DMALAST;
					}
					crpt_regs->CRPT_HMAC_DATIN = *data_ptr++;
					sha_remaining_cnt -= 4;
				}
			}
			sha_remaining_cnt = 0;
		} else {
			/* SHA was started, but no data pushed! */
			return -EFAULT;
		}
		break;


	default:
		mutex_unlock(&nuc970_crdev.sha_lock);
		return -ENOTTY;
	}

	mutex_unlock(&nuc970_crdev.sha_lock);
	return 0;
}

struct file_operations nvt_sha_fops = {
	.owner      = THIS_MODULE,
	.read       = nvt_sha_read,        /* used to read SHA output digest            */
	.write      = nvt_sha_write,       /* used to push SHA input data               */
	.unlocked_ioctl = nvt_sha_ioctl,   /* used to start and finish a SHA operation  */
};

static struct miscdevice nvt_sha_dev = {
	.minor      = MISC_DYNAMIC_MINOR,
	.name       = "nuvoton-sha",
	.fops       = &nvt_sha_fops,
};


static int nuc970_crypto_raw_probe(struct platform_device *pdev)
{
	misc_register(&nvt_aes_dev);
	misc_register(&nvt_sha_dev);
	return 0;
}

static int nuc970_crypto_raw_remove(struct platform_device *pdev)
{
	misc_deregister(&nvt_aes_dev);
	misc_deregister(&nvt_sha_dev);
	return 0;
}


static const struct of_device_id nuc970_crypto_raw_of_match[] = {
	{ .compatible = "nuvoton,nuc970-crypto-raw" },
	{},
};
MODULE_DEVICE_TABLE(of, nuc970_crypto_raw_of_match);


static struct platform_driver nuc970_crypto_raw_driver = {
	.probe      = nuc970_crypto_raw_probe,
	.remove     = nuc970_crypto_raw_remove,
	.driver     = {
		.name   = "nuc970-crypto-raw",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(nuc970_crypto_raw_of_match),
	},
};

module_platform_driver(nuc970_crypto_raw_driver);

MODULE_AUTHOR("Nuvoton Technology Corporation");
MODULE_DESCRIPTION("NUC970 Cryptographic Accerlerator Raw");
MODULE_LICENSE("GPL");
