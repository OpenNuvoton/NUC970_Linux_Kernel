/*
 * Copyright © 2014 Nuvoton technology corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/blkdev.h>

#include <linux/freezer.h>
#include <linux/of.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-gcr.h>
#include <mach/regs-timer.h>
#include <mach/regs-fmi.h>
#include <linux/dma-mapping.h>
#include "../mtdcore.h"


#define RESET_FMI   0x01
#define NAND_EN     0x08
#define READYBUSY   (0x01 << 18)

#define SWRST       0x01
#define PSIZE       (0x01 << 3)
#define DMARWEN     (0x03 << 1)
#define BUSWID      (0x01 << 4)
#define ECC4EN      (0x01 << 5)
#define WP          (0x01 << 24)
#define NANDCS      (0x01 << 25)
#define ENDADDR     (0x01 << 31)

#define BCH_T15     0x00400000
#define BCH_T12     0x00200000
#define BCH_T8      0x00100000
#define BCH_T4      0x00080000
#define BCH_T24     0x00040000

#define NUC970_DRV_VERSION "20160331"
#define DEF_RESERVER_OOB_SIZE_FOR_MARKER 4

//#define NUC970_NAND_DEBUG
#ifndef NUC970_NAND_DEBUG
#define DBG(fmt, arg...)
#define ENTER()
#define LEAVE()
#else
#define DBG(fmt, arg...)    printk(fmt, ##arg)
#define ENTER()
#define LEAVE()
#endif

#define read_data_reg(dev)        readl(REG_SMDATA)

#define write_data_reg(dev, val)  writel((val), REG_SMDATA)

#define write_cmd_reg(dev, val)   writel((val), REG_SMCMD)

#define write_addr_reg(dev, val)  writel((val), REG_SMADDR)

struct nuc970_nand_info {
	struct nand_hw_control  controller;
	struct mtd_info         mtd;
	struct nand_chip        chip;
	struct mtd_partition    *parts;     // mtd partition
	int                     nr_parts;   // mtd partition number
	struct platform_device  *pdev;
	struct clk              *clk;
	struct clk              *fmi_clk;

	void __iomem            *reg;
	int                     eBCHAlgo;
	int                     m_i32SMRASize;
	int                     m_ePageSize;

	unsigned char *         pnand_vaddr;
	unsigned char *         pnand_phyaddr;

	spinlock_t              lock;
};

typedef enum  {
	eBCH_T4,
	eBCH_T8,
	eBCH_T12,
	eBCH_T15,
	eBCH_T24,
	eBCH_CNT
} E_BCHALGORITHM;

typedef enum {
	ePageSize_512,
	ePageSize_2048,
	ePageSize_4096,
	ePageSize_8192,
	ePageSize_CNT
} E_PAGESIZE;

static const int g_i32BCHAlgoIdx[eBCH_CNT] = { BCH_T4, BCH_T8, BCH_T12, BCH_T15, BCH_T24 };
static struct nand_ecclayout nuc970_nand_oob;
static const int g_i32ParityNum[ePageSize_CNT][eBCH_CNT] = {
	{ 8,    15,     23,     29,     -1  },  // For 512
	{ 32,   60,     92,     116,    90  },  // For 2K
	{ 64,   120,    184,    232,    180 },  // For 4K
	{ 128,  240,    368,    464,    360 },  // For 8K
};


#ifndef CONFIG_MTD_CMDLINE_PARTS
#ifndef CONFIG_OF
static struct mtd_partition partitions[] = {
	{
		.name = "u-boot",
		.offset = 0,
		.size = 2 * 1024 * 1024,
		.ecclayout = (struct nand_ecclayout*)&nuc970_nand_oob
	},
	{
		.name = "Kernel",
		.size = 20 * 1024 * 1024,
		.offset = MTDPART_OFS_APPEND,
		.ecclayout = (struct nand_ecclayout*)&nuc970_nand_oob
	},
	{
		.name = "user",
		.offset = MTDPART_OFS_APPEND,
		.size = MTDPART_SIZ_FULL
	}
};
#endif
#endif

static void dump_chip_info( struct nand_chip * chip )
{
#ifndef NUC970_NAND_DEBUG
	return;
#endif

	printk( "==========================\n" );

	printk("chip_delay: %d\n", chip->chip_delay );
	printk("chip->options: 0x%08X\n", chip->options );
	printk("page size: %d Byte\n", 1<<chip->page_shift );

	printk("chip->phys_erase_shift: %d\n", 1<<chip->phys_erase_shift );
	printk("chip->bbt_erase_shift: %d\n", 1<<chip->bbt_erase_shift );

	printk("chip->chip_shift: 0x%08X\n", chip->chip_shift );
	printk("chip->numchips: %d\n", chip->numchips );

	printk("chip->subpagesize: %d\n", 1<<chip->subpagesize );
	printk("chip->cellinfo: 0x%08X\n", chip->cellinfo );

	printk( "==========================\n" );
}


static void dump_regs(int i32Line)
{
#ifndef NUC970_NAND_DEBUG
	return;
#endif

	printk("============[%d]==============\n", i32Line);

	printk("REG_NAND_FMICR[00] : 0x%08X\n",  readl(REG_NAND_FMICSR));

	printk("REG_SMCSR[A0] : 0x%08X\n",  readl(REG_SMCSR));
	printk("REG_SMISR[AC] : 0x%08X\n",  readl(REG_SMISR));
	printk("REG_SMIER[A8] : 0x%08X\n",  readl(REG_SMIER));

	printk("REG_SMCMD[B0] : 0x%08X\n",  readl(REG_SMCMD));
	printk("REG_SMADDR[B4] : 0x%08X\n", readl(REG_SMADDR));
	printk("REG_SMDATA[B8] : 0x%08X\n", readl(REG_SMDATA));

	printk("REG_SMTCR[A4] : 0x%08X\n",  readl(REG_SMTCR));

	printk("REG_NAND_DMACSAR[08] : 0x%08X\n", readl(REG_NAND_DMACSAR));

	printk("============[%d]==============\n", i32Line);
}


/*
 * nuc970_nand_hwecc_init - Initialize hardware ECC IP
 */
static void nuc970_nand_hwecc_init (struct mtd_info *mtd)
{
	struct nuc970_nand_info *nand = container_of(mtd, struct nuc970_nand_info, mtd);

	writel ( readl(REG_SMCSR)|0x1, REG_SMCSR);    // reset SM controller

	// Redundant area size
	writel( nand->m_i32SMRASize , REG_SMREACTL );

	// Protect redundant 3 bytes
	// because we need to implement write_oob function to partial data to oob available area.
	// Please note we skip 4 bytes
	writel( readl(REG_SMCSR) | 0x100, REG_SMCSR);

	// To read/write the ECC parity codes automatically from/to NAND Flash after data area field written.
	writel( readl(REG_SMCSR) | 0x10, REG_SMCSR);

	if ( nand->eBCHAlgo >= 0 ) {
		// Set BCH algorithm
		writel( (readl(REG_SMCSR) & (~0x007C0000)) | g_i32BCHAlgoIdx[nand->eBCHAlgo], REG_SMCSR);

		// Enable H/W ECC, ECC parity check enable bit during read page
		writel( readl(REG_SMCSR) | 0x00800080, REG_SMCSR);

	} else  {
		// Disable H/W ECC / ECC parity check enable bit during read page
		writel( readl(REG_SMCSR) & (~0x00800000) &(~0x80), REG_SMCSR);
	}
}

/*
 * nuc970_nand_hwecc_fini - Finalize hardware ECC IP
 */
static void nuc970_nand_hwecc_fini (struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	if ( chip->ecc.mode == NAND_ECC_HW_OOB_FIRST )
		writel(readl(REG_SMCSR)&(~0x00800000), REG_SMCSR); // ECC disable
}

static void nuc970_nand_initialize ( void )
{
	ENTER() ;

	// Enable SM_EN
	writel( NAND_EN, REG_NAND_FMICSR );

	// Timing control
	// writel(0x3050b, REG_SMTCR);
	// tCLS= (2+1)TAHB,
	// tCLH= (2*2+2)TAHB,
	// tALS= (2*1+1)TAHB,
	// tALH= (2*2+2)TAHB,
	writel(0x20305, REG_SMTCR);

	// Enable SM_CS0
	writel((readl(REG_SMCSR)&(~0x06000000))|0x04000000, REG_SMCSR);
	writel(0x1, REG_NFECR); /* un-lock write protect */

	// NAND Reset
	writel(readl(REG_SMCSR) | 0x1, REG_SMCSR);    // software reset

	LEAVE();
}

/*-----------------------------------------------------------------------------
 * Define some constants for BCH
 *---------------------------------------------------------------------------*/
// define the total padding bytes for 512/1024 data segment
#define BCH_PADDING_LEN_512     32
#define BCH_PADDING_LEN_1024    64
// define the BCH parity code lenght for 512 bytes data pattern
#define BCH_PARITY_LEN_T4  8
#define BCH_PARITY_LEN_T8  15
#define BCH_PARITY_LEN_T12 23
#define BCH_PARITY_LEN_T15 29
// define the BCH parity code lenght for 1024 bytes data pattern
#define BCH_PARITY_LEN_T24 45


/*-----------------------------------------------------------------------------
 * Correct data by BCH alrogithm.
 *      Support 8K page size NAND and BCH T4/8/12/15/24.
 *---------------------------------------------------------------------------*/
void fmiSM_CorrectData_BCH(u8 ucFieidIndex, u8 ucErrorCnt, u8* pDAddr)
{
	u32 uaData[24], uaAddr[24];
	u32 uaErrorData[6];
	u8  ii, jj;
	u32 uPageSize;
	u32 field_len, padding_len, parity_len;
	u32 total_field_num;
	u8  *smra_index;

	ENTER();

	//--- assign some parameters for different BCH and page size
	switch (readl(REG_SMCSR) & 0x007C0000)
	{
		case BCH_T24:
			field_len   = 1024;
			padding_len = BCH_PADDING_LEN_1024;
			parity_len  = BCH_PARITY_LEN_T24;
			break;
		case BCH_T15:
			field_len   = 512;
			padding_len = BCH_PADDING_LEN_512;
			parity_len  = BCH_PARITY_LEN_T15;
			break;
		case BCH_T12:
			field_len   = 512;
			padding_len = BCH_PADDING_LEN_512;
			parity_len  = BCH_PARITY_LEN_T12;
			break;
		case BCH_T8:
			field_len   = 512;
			padding_len = BCH_PADDING_LEN_512;
			parity_len  = BCH_PARITY_LEN_T8;
			break;
		case BCH_T4:
			field_len   = 512;
			padding_len = BCH_PADDING_LEN_512;
			parity_len  = BCH_PARITY_LEN_T4;
			break;
		default:
			printk("NAND ERROR: %s(): invalid SMCR_BCH_TSEL = 0x%08X\n", __FUNCTION__, (u32)(readl(REG_SMCSR) & 0x7C0000));
			LEAVE();
			return;
	}

	uPageSize = readl(REG_SMCSR) & 0x00030000;
	switch (uPageSize)
	{
		case 0x30000:  total_field_num = 8192 / field_len; break;
		case 0x20000:  total_field_num = 4096 / field_len; break;
		case 0x10000:  total_field_num = 2048 / field_len; break;
		case 0x00000:  total_field_num =  512 / field_len; break;
		default:
			printk("NAND ERROR: %s(): invalid SMCR_PSIZE = 0x%08X\n", __FUNCTION__, uPageSize);
			LEAVE();
			return;
	}

	//--- got valid BCH_ECC_DATAx and parse them to uaData[]
	// got the valid register number of BCH_ECC_DATAx since one register include 4 error bytes
	jj = ucErrorCnt/4;
	jj ++;
	if (jj > 6)
		jj = 6;     // there are 6 BCH_ECC_DATAx registers to support BCH T24

	for(ii=0; ii<jj; ii++)
	{
		uaErrorData[ii] = readl(REG_BCH_ECC_DATA0 + ii*4);
	}

	for(ii=0; ii<jj; ii++)
	{
		uaData[ii*4+0] = uaErrorData[ii] & 0xff;
		uaData[ii*4+1] = (uaErrorData[ii]>>8) & 0xff;
		uaData[ii*4+2] = (uaErrorData[ii]>>16) & 0xff;
		uaData[ii*4+3] = (uaErrorData[ii]>>24) & 0xff;
	}

	//--- got valid REG_BCH_ECC_ADDRx and parse them to uaAddr[]
	// got the valid register number of REG_BCH_ECC_ADDRx since one register include 2 error addresses
	jj = ucErrorCnt/2;
	jj ++;
	if (jj > 12)
		jj = 12;    // there are 12 REG_BCH_ECC_ADDRx registers to support BCH T24

	for(ii=0; ii<jj; ii++)
	{
		uaAddr[ii*2+0] = readl(REG_BCH_ECC_ADDR0 + ii*4) & 0x07ff;   // 11 bits for error address
		uaAddr[ii*2+1] = (readl(REG_BCH_ECC_ADDR0 + ii*4)>>16) & 0x07ff;
	}

	//--- pointer to begin address of field that with data error
	pDAddr += (ucFieidIndex-1) * field_len;

	//--- correct each error bytes
	for(ii=0; ii<ucErrorCnt; ii++)
	{
		// for wrong data in field
		if (uaAddr[ii] < field_len)
		{
#ifdef NUC970_NAND_DEBUG
			printk("BCH error corrected for data: address 0x%08X, data [0x%02X] --> ",
				(unsigned int)(pDAddr+uaAddr[ii]), (unsigned int)(*(pDAddr+uaAddr[ii])));
#endif
			*(pDAddr+uaAddr[ii]) ^= uaData[ii];

#ifdef NUC970_NAND_DEBUG
			printk("[0x%02X]\n", *(pDAddr+uaAddr[ii]));
#endif
		}
		// for wrong first-3-bytes in redundancy area
		else if (uaAddr[ii] < (field_len+3))
		{
			uaAddr[ii] -= field_len;
			uaAddr[ii] += (parity_len*(ucFieidIndex-1));    // field offset

#ifdef NUC970_NAND_DEBUG
			printk("BCH error corrected for 3 bytes: address 0x%08X, data [0x%02X] --> ",
				(unsigned int)((u8 *)REG_SMRA0 + uaAddr[ii]), (unsigned int)(*((u8 *)REG_SMRA0 + uaAddr[ii])));
#endif
			*((u8 *)REG_SMRA0 + uaAddr[ii]) ^= uaData[ii];

#ifdef NUC970_NAND_DEBUG
			printk("[0x%02X]\n", *((u8 *)REG_SMRA0+uaAddr[ii]));
#endif
		}
		// for wrong parity code in redundancy area
		else
		{
			// BCH_ERR_ADDRx = [data in field] + [3 bytes] + [xx] + [parity code]
			//                                   |<--     padding bytes      -->|
			// The BCH_ERR_ADDRx for last parity code always = field size + padding size.
			// So, the first parity code = field size + padding size - parity code length.
			// For example, for BCH T12, the first parity code = 512 + 32 - 23 = 521.
			// That is, error byte address offset within field is
			uaAddr[ii] = uaAddr[ii] - (field_len + padding_len - parity_len);

			// smra_index point to the first parity code of first field in register SMRA0~n
			smra_index = (u8 *)
						 (REG_SMRA0 + (readl(REG_SMREACTL) & 0x1ff) - // bottom of all parity code -
						  (parity_len * total_field_num)                             // byte count of all parity code
						 );

			// final address = first parity code of first field +
			//                 offset of fields +
			//                 offset within field

#ifdef NUC970_NAND_DEBUG
			printk("BCH error corrected for parity: address 0x%08X, data [0x%02X] --> ",
				(unsigned int)(smra_index + (parity_len * (ucFieidIndex-1)) + uaAddr[ii]),
				(unsigned int)(*(smra_index + (parity_len * (ucFieidIndex-1)) + uaAddr[ii])));
#endif
			*((u8 *)smra_index + (parity_len * (ucFieidIndex-1)) + uaAddr[ii]) ^= uaData[ii];

#ifdef NUC970_NAND_DEBUG
			printk("[0x%02X]\n",
				*((u8 *)smra_index + (parity_len * (ucFieidIndex-1)) + uaAddr[ii]));
#endif
		}
	}   // end of for (ii<ucErrorCnt)
	LEAVE();
}

int fmiSMCorrectData (struct mtd_info *mtd, unsigned long uDAddr )
{
	int uStatus, ii, jj, i32FieldNum=0;
	volatile int uErrorCnt = 0;
	volatile int uReportErrCnt = 0;

	ENTER();

	if ( readl ( REG_SMISR ) & 0x4 )
	{
		if ( ( readl(REG_SMCSR) & 0x7C0000) == BCH_T24 )
			i32FieldNum = mtd->writesize / 1024;    // Block=1024 for BCH
		else
			i32FieldNum = mtd->writesize / 512;

		if ( i32FieldNum < 4 )
			i32FieldNum  = 1;
		else
			i32FieldNum /= 4;

		for ( jj=0; jj<i32FieldNum; jj++ )
		{
			uStatus = readl ( REG_SMECC_ST0+jj*4 );
			if ( !uStatus )
				continue;

			for ( ii=1; ii<5; ii++ )
			{
				if ( !(uStatus & 0x03) ) { // No error
					uStatus >>= 8;
					continue;
				} else if ( (uStatus & 0x03)==0x01 ) { // Correctable error
					uErrorCnt = (uStatus >> 2) & 0x1F;
#ifdef NUC970_NAND_DEBUG
					printk("Field (%d, %d) have %d error!!\n", jj, ii, uErrorCnt);
#endif
					fmiSM_CorrectData_BCH(jj*4+ii, uErrorCnt, (char*)uDAddr);
					uReportErrCnt += uErrorCnt;
					uStatus >>= 8;
					continue;
				} else // uncorrectable error or ECC error
				{
#ifdef NUC970_NAND_DEBUG
					printk("SM uncorrectable error is encountered, 0x%4x !!\n", uStatus);
#endif
					LEAVE();
					return -1;
				}
				uStatus >>= 8;
			}
		} //jj
	}
	LEAVE();
	return uReportErrCnt;
}

/*
 * HW ECC Correction
 * function called after a read
 * mtd:        MTD block structure
 * dat:        raw data read from the chip
 * read_ecc:   ECC from the chip (unused)
 * isnull:     unused
 */
static int nuc970_nand_correct_data(struct mtd_info *mtd, u_char *dat, u_char *read_ecc, u_char *calc_ecc)
{
	return 0;
}


/*
 * Enable HW ECC : unused on most chips
 */
void nuc970_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
	ENTER();
#ifdef NUC970_NAND_DEBUG
	{
		char * ptr=REG_SMRA0;
		int i=0;
		if( mode == NAND_ECC_READ )
			printk("[R]=\n");
		else
			printk("[W]=\n");

		for(i=0; i<mtd->oobsize; i++)
		{
			printk("%X ",  *(ptr+i) );
			if ( i % 32 == 31)
				printk("\n");
		}
		printk("\n");
	}
#endif
	LEAVE();
}

/*
 * nuc970_nand_dmac_init - Initialize dma controller
 */
static void nuc970_nand_dmac_init( void )
{
	// DMAC enable
	writel( readl(REG_NAND_DMACCSR) | 0x3, REG_NAND_DMACCSR);
	writel( readl(REG_NAND_DMACCSR) & (~0x2), REG_NAND_DMACCSR);

	// Clear DMA finished flag
	//writel( readl(REG_SMISR) | 0x1, REG_SMISR);
	writel( 0x1, REG_SMISR);

	// Disable Interrupt
	writel(readl(REG_SMIER) & ~(0x1), REG_SMIER);
}

/*
 * nuc970_nand_dmac_fini - Finalize dma controller
 */
static void nuc970_nand_dmac_fini(void)
{
	// Clear DMA finished flag
	//writel(readl(REG_SMISR) | 0x1, REG_SMISR);
	writel(0x1, REG_SMISR);
}

/*
 * nuc970_nand_read_byte - read a byte from NAND controller into buffer
 * @mtd: MTD device structure
 */
static unsigned char nuc970_nand_read_byte(struct mtd_info *mtd)
{
	unsigned char ret;
	struct nuc970_nand_info *nand;

	ENTER() ;

	nand = container_of(mtd, struct nuc970_nand_info, mtd);
	ret = (unsigned char)read_data_reg(nand);

	LEAVE();
	return ret;
}


/*
 * nuc970_nand_read_buf - read data from NAND controller into buffer
 * @mtd: MTD device structure
 * @buf: virtual address in RAM of source
 * @len: number of data bytes to be transferred
 */
static void nuc970_nand_read_buf(struct mtd_info *mtd, unsigned char *buf, int len)
{
	int i;
	struct nuc970_nand_info *nand;
	nand = container_of(mtd, struct nuc970_nand_info, mtd);

	ENTER() ;

	for (i = 0; i < len; i++)
		buf[i] = (unsigned char)read_data_reg(nand);

	LEAVE();
}
/*
 * nuc970_nand_write_buf - write data from buffer into NAND controller
 * @mtd: MTD device structure
 * @buf: virtual address in RAM of source
 * @len: number of data bytes to be transferred
 */

static void nuc970_nand_write_buf(struct mtd_info *mtd, const unsigned char *buf, int len)
{
	int i;
	struct nuc970_nand_info *nand;
	nand = container_of(mtd, struct nuc970_nand_info, mtd);

	ENTER() ;

	for (i = 0; i < len; i++)
		write_data_reg(nand, buf[i]);

	LEAVE();
}

/*
 * _nuc970_nand_dma_transfer: configer and start dma transfer
 * @mtd: MTD device structure
 * @addr: virtual address in RAM of source/destination
 * @len: number of data bytes to be transferred
 * @is_write: flag for read/write operation
 */
static inline int _nuc970_nand_dma_transfer(struct mtd_info *mtd, const u_char *addr, unsigned int len, int is_write)
{
	struct nuc970_nand_info *nand = container_of(mtd, struct nuc970_nand_info, mtd);
	dma_addr_t dma_addr = (dma_addr_t)nand->pnand_phyaddr;
	int stat = 0;

	ENTER() ;

	// For save, wait DMAC to ready
	while ( readl(REG_NAND_DMACCSR) & 0x200 );

	// Reinitial dmac
	nuc970_nand_dmac_init();

	// Fill dma_addr
	writel((unsigned long)dma_addr, REG_NAND_DMACSAR);

	// Enable target abort interrupt generation during DMA transfer.
	writel( 0x1, REG_NAND_DMACIER);

	// Clear Ready/Busy 0 Rising edge detect flag
	writel(0x400, REG_SMISR);

	// Set which BCH algorithm
	if ( nand->eBCHAlgo >= 0 ) {
		// Set BCH algorithm
		writel( (readl(REG_SMCSR) & (~0x7C0000)) | g_i32BCHAlgoIdx[nand->eBCHAlgo], REG_SMCSR);
		// Enable H/W ECC, ECC parity check enable bit during read page
		writel( readl(REG_SMCSR) | 0x00800000 | 0x80, REG_SMCSR);

	} else  {
		// Disable H/W ECC / ECC parity check enable bit during read page
		writel( readl(REG_SMCSR) & (~0x00800080), REG_SMCSR);
	}

	writel( nand->m_i32SMRASize , REG_SMREACTL );

	writel( readl(REG_SMIER) & (~0x4), REG_SMIER );
	writel ( 0x4, REG_SMISR );

	// Enable SM_CS0
	writel((readl(REG_SMCSR)&(~0x06000000))|0x04000000, REG_SMCSR);
	/* setup and start DMA using dma_addr */

	if ( is_write ) {
		register char * ptr=REG_SMRA0;
		// To mark this page as dirty.
		if ( ptr[3] == 0xFF )
			ptr[3] = 0;
		if ( ptr[2] == 0xFF )
			ptr[2] = 0;

		if ( addr )
			memcpy( (void*)nand->pnand_vaddr, (void*)addr, len);

		writel ( readl(REG_SMCSR) | 0x4, REG_SMCSR );
		while ( !(readl(REG_SMISR) & 0x1) );

	} else {
		// Blocking for reading
		// Enable DMA Read

		writel ( readl(REG_SMCSR) | 0x2, REG_SMCSR);
		if ( readl(REG_SMCSR) & 0x80 ) {
			do {
				if ( (stat=fmiSMCorrectData ( mtd,  (unsigned long)nand->pnand_vaddr)) < 0 )
				{
					mtd->ecc_stats.failed++;
					writel ( 0x4, REG_SMISR );
					writel ( 0x3, REG_NAND_DMACCSR);          // reset DMAC
					writel ( readl(REG_SMCSR)|0x1, REG_SMCSR);    // reset SM controller
					stat = -EIO;
					break;
				}
				else if ( stat > 0 ) {
					mtd->ecc_stats.corrected += stat;   // Add corrected bit count
					writel ( 0x4, REG_SMISR );
				}

			} while ( !(readl(REG_SMISR) & 0x1) || (readl(REG_SMISR) & 0x4) );
		} else
			while ( !(readl(REG_SMISR) & 0x1) );

		if ( addr )
			memcpy( (void*)addr, (void*)nand->pnand_vaddr,  len );
	}

	nuc970_nand_dmac_fini();
	LEAVE();
	return stat;
}

/**
 * nuc970_read_buf_dma_pref - read data from NAND controller into buffer
 * @mtd: MTD device structure
 * @buf: buffer to store date
 * @len: number of bytes to read
 */
static void nuc970_read_buf_dma(struct mtd_info *mtd, u_char *buf, int len)
{
	ENTER();

	if ( len == mtd->writesize ) /* start transfer in DMA mode */
		_nuc970_nand_dma_transfer ( mtd, buf, len, 0x0);
	else {
		nuc970_nand_read_buf(mtd, buf, len);

#ifdef NUC970_NAND_DEBUG
		{
		int i;
		printk("R OOB %d\n", len );
		for ( i=0; i<len; i++ )
		{
			printk("%02X ", buf[i] );
			if ( i%32 == 31 )   printk("\n");
		}
		printk("\n");
		}
#endif
	}
	LEAVE();
}

/**
 * nuc970_write_buf_dma_pref - write buffer to NAND controller
 * @mtd: MTD device structure
 * @buf: data buffer
 * @len: number of bytes to write
 */
static void nuc970_write_buf_dma(struct mtd_info *mtd, const u_char *buf, int len)
{
	ENTER();

	if ( len == mtd->writesize ) /* start transfer in DMA mode */
		_nuc970_nand_dma_transfer(mtd, (u_char *)buf, len, 0x1);
	else
	{
#ifdef NUC970_NAND_DEBUG
		int i;
		printk("W OOB %d\n", len);
		for ( i=0; i<len; i++ )
		{
			printk("%02X ", buf[i] );
			if ( i%32 == 31 )   printk("\n");
		}
#endif
		nuc970_nand_write_buf(mtd, buf, len);
	}

	LEAVE();
}


/**
 * nuc970_check_rb - check ready/busy pin
 * @mtd: MTD device structure
 */
static int nuc970_check_rb(struct nuc970_nand_info *nand)
{
	unsigned int val;

	ENTER();
	spin_lock(&nand->lock);
	val = readl(REG_SMISR) & READYBUSY;
	spin_unlock(&nand->lock);
	LEAVE();

	return val;
}

static int nuc970_nand_devready(struct mtd_info *mtd)
{
	struct nuc970_nand_info *nand;
	int ready;

	ENTER() ;

	nand = container_of(mtd, struct nuc970_nand_info, mtd);
	ready = (nuc970_check_rb(nand)) ? 1 : 0;

	LEAVE();
	return ready;
}

static void nuc970_nand_command_lp(struct mtd_info *mtd, unsigned int command, int column, int page_addr)
{
	register struct nand_chip *chip = mtd->priv;
	struct nuc970_nand_info *nand;

	ENTER() ;

	nand = container_of(mtd, struct nuc970_nand_info, mtd);

	writel(0x400, REG_SMISR);
	if (command == NAND_CMD_READOOB) {
		column += mtd->writesize;
		command = NAND_CMD_READ0;
	}

	write_cmd_reg(nand, command & 0xff);

	if (command == NAND_CMD_READID)
	{
		write_addr_reg(nand, ENDADDR);
	}
	else
	{
		if (column != -1 || page_addr != -1) {
			if (column != -1) {
				write_addr_reg(nand, (column&0xFF) );
				if ( page_addr != -1 )
					write_addr_reg(nand, (column >> 8) );
				else
					write_addr_reg(nand, (column >> 8) | ENDADDR);

			}

			if (page_addr != -1) {
				write_addr_reg(nand, (page_addr&0xFF) );

				if ( chip->chipsize > (128 << 20) ) {
					write_addr_reg(nand, (page_addr >> 8)&0xFF );
					write_addr_reg(nand, ((page_addr >> 16)&0xFF)|ENDADDR );
				} else {
					write_addr_reg(nand, ((page_addr >> 8)&0xFF)|ENDADDR );
				}
			}
		}
	}

	switch (command) {
	case NAND_CMD_ERASE1:
	case NAND_CMD_ERASE2:
	case NAND_CMD_CACHEDPROG:
	case NAND_CMD_PAGEPROG:
	case NAND_CMD_SEQIN:
	case NAND_CMD_RNDIN:
	case NAND_CMD_STATUS:
	case NAND_CMD_READID:
		LEAVE();
		return;

	case NAND_CMD_RESET:
		if (chip->dev_ready)
			break;

		if ( chip->chip_delay )
			udelay(chip->chip_delay);

		write_cmd_reg(nand, NAND_CMD_STATUS);
		write_cmd_reg(nand, command);

		while (!nuc970_check_rb(nand)) ;

		LEAVE();
		return;

	case NAND_CMD_RNDOUT:
		write_cmd_reg(nand, NAND_CMD_RNDOUTSTART);
		LEAVE();
		return;

	case NAND_CMD_READ0:
		write_cmd_reg(nand, NAND_CMD_READSTART);
		break;
	default:
		if (!chip->dev_ready) {
			if ( chip->chip_delay )
				udelay(chip->chip_delay);
			LEAVE();
			return;
		}
	}

	//while (!nuc970_check_rb(nand)) ;
	//printk("cmd: 0x%x, 0x%x\n", command, readl(REG_SMISR));
	while(1)
	{
		if (readl(REG_SMISR) & 0x400)
		{
			writel(0x400, REG_SMISR);
			break;
		}
	}

	LEAVE();
}

/* select chip */
static void nuc970_nand_select_chip(struct mtd_info *mtd, int chip)
{
	writel((readl(REG_SMCSR)&(~0x06000000))|0x04000000, REG_SMCSR);
	return;
}

/*
 * Calculate HW ECC
 * function called after a write
 * mtd:        MTD block structure
 * dat:        raw data (unused)
 * ecc_code:   buffer for ECC
 */
static int nuc970_nand_calculate_ecc(struct mtd_info *mtd, const u_char *dat, u_char *ecc_code)
{
	return 0;
}

/**
 * nand_write_page_hwecc - [REPLACABLE] hardware ecc based page write function
 * @mtd:        mtd info structure
 * @chip:       nand chip info structure
 * @buf:        data buffer
 */
static int nuc970_nand_write_page_hwecc(struct mtd_info *mtd, struct nand_chip *chip, const uint8_t *buf, int oob_required)
{
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	uint32_t hweccbytes=chip->ecc.layout->eccbytes;
	register char * ptr=REG_SMRA0;

	ENTER();

	memset ( (void*)ptr, 0xFF, mtd->oobsize );
	memcpy ( (void*)ptr, (void*)chip->oob_poi,  mtd->oobsize - chip->ecc.total );

	_nuc970_nand_dma_transfer( mtd, buf, mtd->writesize , 0x1);

	// Copy parity code in SMRA to calc
	memcpy ( (void*)ecc_calc,  (void*)( REG_SMRA0 + ( mtd->oobsize - chip->ecc.total ) ), chip->ecc.total );

	// Copy parity code in calc to oob_poi
	memcpy ( (void*)(chip->oob_poi+hweccbytes), (void*)ecc_calc, chip->ecc.total);

	LEAVE();
	return 0;
}

/**
 * nuc970_nand_read_page_hwecc_oob_first - hardware ecc based page write function
 * @mtd:        mtd info structure
 * @chip:       nand chip info structure
 * @buf:        buffer to store read data
 * @page:       page number to read
 */
static int nuc970_nand_read_page_hwecc_oob_first(struct mtd_info *mtd, struct nand_chip *chip, uint8_t *buf, int oob_required, int page)
{
	int eccsize = chip->ecc.size;
	uint8_t *p = buf;
	char * ptr=REG_SMRA0;
	int stat = 0;

	ENTER();

	/* At first, read the OOB area  */
	nuc970_nand_command_lp(mtd, NAND_CMD_READOOB, 0, page);
	nuc970_nand_read_buf(mtd, chip->oob_poi, mtd->oobsize);

	// Second, copy OOB data to SMRA for page read
	memcpy ( (void*)ptr, (void*)chip->oob_poi, mtd->oobsize );

	if ((*(ptr+2) != 0) && (*(ptr+3) != 0))
	{
		memset((void*)p, 0xff, eccsize);
	}
	else
	{
		// Third, read data from nand
		nuc970_nand_command_lp(mtd, NAND_CMD_READ0, 0, page);
		stat = _nuc970_nand_dma_transfer(mtd, p, eccsize, 0x0);

		// Fouth, restore OOB data from SMRA
		memcpy ( (void*)chip->oob_poi, (void*)ptr, mtd->oobsize );
	}

	LEAVE();

	return stat;
}

static void nuc970_layout_oob_table ( struct nand_ecclayout* pNandOOBTbl, int oobsize , int eccbytes )
{
	pNandOOBTbl->eccbytes = eccbytes;

	pNandOOBTbl->oobavail = oobsize - DEF_RESERVER_OOB_SIZE_FOR_MARKER - eccbytes ;

	pNandOOBTbl->oobfree[0].offset = DEF_RESERVER_OOB_SIZE_FOR_MARKER;  // Bad block marker size

	pNandOOBTbl->oobfree[0].length = oobsize - eccbytes - pNandOOBTbl->oobfree[0].offset ;
}

/**
 * nand_read_oob_std - [REPLACABLE] the most common OOB data read function
 * @mtd:        mtd info structure
 * @chip:       nand chip info structure
 * @page:       page number to read
 * @sndcmd:     flag whether to issue read command or not
 */
static int nuc970_nand_read_oob_hwecc(struct mtd_info *mtd, struct nand_chip *chip, int page)
{
	char * ptr=REG_SMRA0;

	ENTER();

	nuc970_nand_command_lp(mtd, NAND_CMD_READOOB, 0, page);

	nuc970_nand_read_buf(mtd, &chip->oob_poi[0], mtd->oobsize);

	// Second, copy OOB data to SMRA for page read
	memcpy ( (void*)ptr, (void*)chip->oob_poi, mtd->oobsize );

#if 0
	if ((*(ptr+2) == 0) && (*(ptr+3) == 0))
	{
		// Third, read data from nand
		nuc970_nand_command_lp(mtd, NAND_CMD_READ0, 0, page);
		_nuc970_nand_dma_transfer(mtd, NULL, mtd->writesize, 0x0);

		// Fouth, recovery OOB data for SMRA
		memcpy ( (void*)chip->oob_poi, (void*)ptr, mtd->oobsize );
	}
#endif
	return 0;
}

/**
 * nand_write_page - [REPLACEABLE] write one page
 * @mtd:        MTD device structure
 * @chip:       NAND chip descriptor
 * @buf:        the data to write
 * @page:       page number to write
 * @cached:     cached programming
 * @raw:        use _raw version of write_page
 */


static int nuc970_nand_write_page(struct mtd_info *mtd, struct nand_chip *chip, uint32_t offset, int data_len,
								  const uint8_t *buf, int oob_required, int page, int cached, int raw)
{
	int status;

	nuc970_nand_command_lp(mtd, NAND_CMD_SEQIN, 0x00, page);

	if (unlikely(raw))
		chip->ecc.write_page_raw(mtd, chip, buf, 0);
	else
	{
		if ( page >= 0 && page < (1<<(chip->phys_erase_shift+2)) / mtd->writesize ) // four blocks
		{
			// Special pattern
			char * ptr=REG_SMRA0;
			memset ( (void*)ptr, 0xFF, mtd->oobsize );
			ptr[3] = 0x00;
			ptr[2] = 0x00;
			ptr[1] = 0xFF;
			_nuc970_nand_dma_transfer( mtd, buf, mtd->writesize , 0x1 );
		}
		else
		{
			nuc970_nand_write_page_hwecc ( mtd, chip, buf, oob_required );
		}
	}

	/*
	 * Cached progamming disabled for now, Not sure if its worth the
	 * trouble. The speed gain is not very impressive. (2.3->2.6Mib/s)
	 */
	cached = 0;

	if (!cached || !(chip->options & NAND_CACHEPRG)) {
		nuc970_nand_command_lp(mtd, NAND_CMD_PAGEPROG, -1, -1);
		status = chip->waitfunc(mtd, chip);
		/*
		 * See if operation failed and additional status checks are
		 * available
		 */
		if ((status & NAND_STATUS_FAIL) && (chip->errstat))
			status = chip->errstat(mtd, chip, FL_WRITING, status, page);

		if (status & NAND_STATUS_FAIL)
		{
			return -EIO;
		}
	}
	else {
		nuc970_nand_command_lp(mtd, NAND_CMD_CACHEDPROG, -1, -1);
		status = chip->waitfunc(mtd, chip);
	}

	return 0;
}

static int nuc970_nand_probe(struct platform_device *pdev)
{
	struct nand_chip *chip;
	struct nuc970_nand_info *nuc970_nand;
	struct mtd_part_parser_data ppdata = {};
	struct mtd_info *mtd;
	struct pinctrl *p;

	int retval=0;

	ENTER() ;

	nuc970_nand = devm_kzalloc(&pdev->dev, sizeof(struct nuc970_nand_info), GFP_KERNEL);
	if (!nuc970_nand)
		return -ENOMEM;

	if (pdev->dev.of_node)
	{
		pdev->dev.platform_data = nuc970_nand;
		nuc970_nand = dev_get_platdata(&pdev->dev);
	}

	nuc970_nand->pnand_vaddr = (unsigned char *) dma_alloc_writecombine(NULL, 512*16, (dma_addr_t *)&nuc970_nand->pnand_phyaddr, GFP_KERNEL);
	if(nuc970_nand->pnand_vaddr == NULL){
		printk(KERN_ERR "nuc970_nand: failed to allocate ram for nand data.\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, nuc970_nand);

	spin_lock_init(&nuc970_nand->controller.lock);
	init_waitqueue_head(&nuc970_nand->controller.wq);

	nuc970_nand->pdev = pdev;
	mtd = &nuc970_nand->mtd;
	chip = &(nuc970_nand->chip);

	nuc970_nand->mtd.priv   = chip;
	nuc970_nand->mtd.owner  = THIS_MODULE;
	spin_lock_init(&nuc970_nand->lock);

	/*
	 * Get Clock
	 */
	nuc970_nand->fmi_clk = clk_get(NULL, "fmi_hclk");
	if (IS_ERR(nuc970_nand->fmi_clk)) {
		printk("no fmi_clk?\n");
		retval = -ENXIO;
		goto fail1;
	}

	nuc970_nand->clk = clk_get(NULL, "nand_hclk");
	if (IS_ERR(nuc970_nand->clk)) {
		printk("no nand_clk?\n");
		goto fail2;
	}

	clk_prepare(nuc970_nand->fmi_clk);
	clk_enable(nuc970_nand->fmi_clk);
	clk_prepare(nuc970_nand->clk);
	clk_enable(nuc970_nand->clk);

	nuc970_nand->chip.controller = &nuc970_nand->controller;

	chip->cmdfunc     = nuc970_nand_command_lp;
	chip->read_byte   = nuc970_nand_read_byte;
	chip->select_chip = nuc970_nand_select_chip;
	chip->read_buf  = nuc970_read_buf_dma;
	chip->write_buf = nuc970_write_buf_dma;

	// Check NAND device NBUSY0 pin
	chip->dev_ready     = nuc970_nand_devready;
	/* set up nand options */
	chip->bbt_options = NAND_BBT_USE_FLASH | NAND_BBT_NO_OOB;
	//chip->options     |= NAND_SKIP_BBTSCAN;

	nuc970_nand->reg    = 0x00;

	// Read OOB data first, then HW read page
	chip->write_page    = nuc970_nand_write_page;
	chip->ecc.mode      = NAND_ECC_HW_OOB_FIRST;
	chip->ecc.hwctl     = nuc970_nand_enable_hwecc;
	chip->ecc.calculate = nuc970_nand_calculate_ecc;
	chip->ecc.correct   = nuc970_nand_correct_data;
	chip->ecc.write_page= nuc970_nand_write_page_hwecc;
	chip->ecc.read_page = nuc970_nand_read_page_hwecc_oob_first;
	chip->ecc.read_oob  = nuc970_nand_read_oob_hwecc;
	chip->ecc.layout    = &nuc970_nand_oob;

#ifdef CONFIG_OF

	p = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(p)) {
		return PTR_ERR(p);
	}

	/*
	 * Right now device-tree probed devices don't get dma_mask set.
	 * Since shared usb code relies on it, set it here for now.
	 * Once we have dma capability bindings this can go away.
	 */
	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
	if (!pdev->dev.coherent_dma_mask)
		pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

#else

#if defined (CONFIG_NUC970_NAND_PC)
	p = devm_pinctrl_get_select(&pdev->dev, "nand-PC");

#elif defined (CONFIG_NUC970_NAND_PI)
	p = devm_pinctrl_get_select(&pdev->dev, "nand-PI");
#endif
	if (IS_ERR(p))
	{
		dev_err(&pdev->dev, "unable to reserve pin\n");
		retval = PTR_ERR(p);
	}
#endif

	nuc970_nand_initialize( );

	/* first scan to find the device and get the page size */
	if (nand_scan_ident(&(nuc970_nand->mtd), 1, NULL)) {
		retval = -ENXIO;
		goto fail3;
	}

	//Set PSize bits of SMCSR register to select NAND card page size
	switch (mtd->writesize) {
		case 2048:
			writel( (readl(REG_SMCSR)&(~0x30000)) + 0x10000, REG_SMCSR);
			nuc970_nand->eBCHAlgo = 0; /* T4 */
			nuc970_nand->m_ePageSize = ePageSize_2048;
			break;

		case 4096:
			writel( (readl(REG_SMCSR)&(~0x30000)) + 0x20000, REG_SMCSR);
			nuc970_nand->eBCHAlgo = 1; /* T8 */
			nuc970_nand->m_ePageSize = ePageSize_4096;
			break;

		case 8192:
			writel( (readl(REG_SMCSR)&(~0x30000)) + 0x30000, REG_SMCSR);
			nuc970_nand->eBCHAlgo = 2; /* T12 */
			nuc970_nand->m_ePageSize = ePageSize_8192;
			break;

		/* Not support now. */
		case 512:
			//writel( (readl(REG_SMCSR)&(~0x30000)) + 0, REG_SMCSR);
			//nuc970_nand->m_ePageSize = ePageSize_512;
			//break;

		default:
			printk("NUC970 NAND CONTROLLER IS NOT SUPPORT THE PAGE SIZE. (%d, %d)\n", mtd->writesize, mtd->oobsize );
			goto fail3;
	}
	nuc970_layout_oob_table(&nuc970_nand_oob, mtd->oobsize, g_i32ParityNum[nuc970_nand->m_ePageSize][nuc970_nand->eBCHAlgo]);

	/* check power on setting */
	if ((readl(REG_PWRON) & 0xc0) != 0xc0) { /* page size */
		switch ((readl(REG_PWRON) & 0xc0)) {
			case 0x00: // 2KB
				mtd->writesize = 2048;
				writel( (readl(REG_SMCSR)&(~0x30000)) + 0x10000, REG_SMCSR);
				nuc970_nand->eBCHAlgo = 0; /* T4 */
				nuc970_nand->m_ePageSize = ePageSize_2048;
				mtd->oobsize = 64;
				break;

			case 0x40: // 4KB
				mtd->writesize = 4096;
				writel( (readl(REG_SMCSR)&(~0x30000)) + 0x20000, REG_SMCSR);
				nuc970_nand->eBCHAlgo = 1; /* T8 */
				nuc970_nand->m_ePageSize = ePageSize_4096;
				mtd->oobsize = 128;
				break;

			case 0x80: // 8KB
				mtd->writesize = 8192;
				writel( (readl(REG_SMCSR)&(~0x30000)) + 0x30000, REG_SMCSR);
				nuc970_nand->eBCHAlgo = 2; /* T12 */
				nuc970_nand->m_ePageSize = ePageSize_8192;
				mtd->oobsize = 376;
				break;

			default:
				printk("WRONG NAND page Power-On-Setting (0x%x)\n", readl(REG_PWRON));
		}
	}
	if ((readl(REG_PWRON) & 0x300) != 0x300) { /* ECC */
		switch ((readl(REG_PWRON) & 0x300)) {
			case 0x000: // T12
				nuc970_nand->eBCHAlgo = 2;
				break;

			case 0x100: // T15
				nuc970_nand->eBCHAlgo = 3;
				break;

			case 0x200: // T24
				nuc970_nand->eBCHAlgo = 4;
				break;

			default:
				printk("WRONG ECC Power-On-Setting (0x%x)\n", readl(REG_PWRON));
		}
		mtd->oobsize = g_i32ParityNum[nuc970_nand->m_ePageSize][nuc970_nand->eBCHAlgo] + 8;
	}
	nuc970_layout_oob_table(&nuc970_nand_oob,mtd->oobsize,g_i32ParityNum[nuc970_nand->m_ePageSize][nuc970_nand->eBCHAlgo]);
	printk("nand: SMRA size %d, %d\n", mtd->oobsize, nuc970_nand_oob.eccbytes);

#ifndef CONFIG_MTD_CMDLINE_PARTS
#ifndef CONFIG_OF
	nuc970_nand->parts = (struct mtd_partition*)partitions;
	nuc970_nand->nr_parts = ARRAY_SIZE(partitions);
#endif
#endif

	nuc970_nand->m_i32SMRASize = mtd->oobsize;
	chip->ecc.bytes = nuc970_nand_oob.eccbytes;
	chip->ecc.size  = mtd->writesize;

	/* set BCH Tn */
	switch (nuc970_nand->eBCHAlgo)
	{
		case eBCH_T4:
			chip->ecc.strength  = 4;
			mtd->bitflip_threshold = 3;
			break;
		case eBCH_T8:
			chip->ecc.strength  = 8;
			mtd->bitflip_threshold = 6;
			break;
		case eBCH_T12:
			chip->ecc.strength  = 12;
			mtd->bitflip_threshold = 9;
			break;
		case eBCH_T15:
			chip->ecc.strength  = 15;
			mtd->bitflip_threshold = 11;
			break;
		case eBCH_T24:
			chip->ecc.strength  = 24;
			mtd->bitflip_threshold = 18;
			break;
		default:
			;
	}

	/* add mtd-id. The string should same as uboot definition */
	mtd->name = "nand0";
	ppdata.of_node = pdev->dev.of_node;

	/* second phase scan */
	if ( nand_scan_tail( &(nuc970_nand->mtd) ) ) {
		retval = -ENXIO;
		goto fail3;
	}

	if ( nuc970_nand->eBCHAlgo >= 0 )
		nuc970_nand_hwecc_init (&(nuc970_nand->mtd));
	else
		nuc970_nand_hwecc_fini (&(nuc970_nand->mtd));

	/* Doesn't handle subpage write */
	mtd->subpage_sft = 0;
	chip->subpagesize = mtd->writesize;

	dump_chip_info( chip );

	/* First look for RedBoot table or partitions on the command
	 * line, these take precedence over device tree information */
	mtd_device_parse_register(&(nuc970_nand->mtd), NULL, &ppdata, nuc970_nand->parts, nuc970_nand->nr_parts);

	LEAVE();

	dump_regs(__LINE__);

	printk("fmi-sm: registered successfully! mtdid=%s\n", mtd->name);
	return retval;

fail3:
fail2:
fail1:
	kfree(nuc970_nand);
	return retval;
}

static int nuc970_nand_remove(struct platform_device *pdev)
{
	struct nuc970_nand_info *nuc970_nand = platform_get_drvdata(pdev);

	struct mtd_info *mtd=&nuc970_nand->mtd;

	nuc970_nand_hwecc_fini(mtd);

	clk_disable(nuc970_nand->clk);
	clk_put(nuc970_nand->clk);

	dma_free_coherent(NULL, 512*16, nuc970_nand->pnand_vaddr, (dma_addr_t )nuc970_nand->pnand_phyaddr);

	kfree(nuc970_nand);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

/* PM Support */
#ifdef CONFIG_PM
static int nuc970_nand_suspend(struct platform_device *pdev, pm_message_t pm)
{
	struct nuc970_nand_info *nuc970_nand = platform_get_drvdata(pdev);

	// For save, wait DMAC to ready
	while ( readl(REG_NAND_DMACCSR) & 0x200 );
	writel(0x0, REG_NFECR); /* write protect */
	nuc970_nand_hwecc_fini(&nuc970_nand->mtd);
	clk_disable(nuc970_nand->clk);

	return 0;
}

static int nuc970_nand_resume(struct platform_device *pdev)
{
	struct nuc970_nand_info *nuc970_nand = platform_get_drvdata(pdev);

	clk_enable(nuc970_nand->clk);
	nuc970_nand_hwecc_init(&nuc970_nand->mtd);
	writel(0x1, REG_NFECR); /* un-lock write protect */
	nuc970_nand_dmac_init();


	return 0;
}

#else
#define nuc970_nand_suspend NULL
#define nuc970_nand_resume NULL
#endif

static const struct of_device_id nuc970_fmi_of_match[] = {
	{ .compatible = "nuvoton,nuc970-fmi" },
	{},
};
MODULE_DEVICE_TABLE(of, nuc970_fmi_of_match);

static struct platform_driver nuc970_nand_driver = {
		.driver = {
		.name   = "nuc970-fmi",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(nuc970_fmi_of_match),
		},
		.probe      = nuc970_nand_probe,
		.remove     = nuc970_nand_remove,
		.suspend    = nuc970_nand_suspend,
		.resume     = nuc970_nand_resume,
};

static int __init nuc970_nand_init(void)
{
	int ret;
	printk("nuc970 mtd nand driver version: %s\n", NUC970_DRV_VERSION );

	ret = platform_driver_register(&nuc970_nand_driver);
	if (ret) {
		printk("nand: failed to add device driver %s \n", nuc970_nand_driver.driver.name);
		return ret;
	}

	return ret;
}

static void __exit nuc970_nand_exit(void)
{
	platform_driver_unregister(&nuc970_nand_driver);
	printk("nand: unregistered successfully! \n");
}

module_init(nuc970_nand_init);
module_exit(nuc970_nand_exit);

MODULE_AUTHOR("nuvoton");
MODULE_DESCRIPTION("nuc970 nand driver!");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nuc970-fmi");
