/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/spi/spi.h>
#include "giga_spinand.h"


void gigadevice_set_defaults(struct spi_device *spi_nand)
{
	struct mtd_info *mtd = (struct mtd_info *)dev_get_drvdata
						(&spi_nand->dev);
	struct nand_chip *chip = (struct nand_chip *)mtd->priv;

	chip->ecc.size	= 0x800;
	chip->ecc.bytes	= 0x0;
	chip->ecc.steps	= 0x0;

	chip->ecc.strength = 1;
	chip->ecc.total	= 0;
	chip->ecc.layout = NULL;
}

void gigadevice_read_cmd(struct spinand_cmd *cmd, u32 page_id)
{
	cmd->addr[0] = (u8) (page_id >> 16);
	cmd->addr[1] = (u8) (page_id >> 8);
	cmd->addr[2] = (u8) (page_id);

}

void gigadevice_read_data(struct spinand_cmd *cmd, u16 column, u32 page_id)
{
	cmd->addr[1] = (u8)(column >> 8);
	cmd->addr[2] = (u8)(column);
}

void macronix_read_data(struct spinand_cmd *cmd, u16 column, u32 page_id)
{
	cmd->addr[0] = ((u8)(column >> 8) & MACRONIX_NORM_RW_MASK);
	cmd->addr[1] = (u8)(column);
}

void winbond_read_data(struct spinand_cmd *cmd, u16 column, u32 page_id)
{
	cmd->addr[0] = (u8)(column >> 8);
	cmd->addr[1] = (u8)(column);
}

void gigadevice_write_cmd(struct spinand_cmd *cmd, u32 page_id)
{
	cmd->addr[0] = (u8)(page_id >> 16);
	cmd->addr[1] = (u8)(page_id >> 8);
	cmd->addr[2] = (u8)(page_id);
}

void gigadevice_write_data(struct spinand_cmd *cmd, u16 column, u32 page_id)
{
	cmd->addr[1] = (u8)(column >> 8);
	cmd->addr[2] = (u8)(column);
}

void macronix_write_data(struct spinand_cmd *cmd, u16 column, u32 page_id)
{
	cmd->addr[0] = ((u8)(column >> 8) & MACRONIX_NORM_RW_MASK);
	cmd->addr[1] = (u8)(column);
}

void winbond_write_data(struct spinand_cmd *cmd, u16 column, u32 page_id)
{
	cmd->addr[0] = (u8)(column >> 8);
	cmd->addr[1] = (u8)(column);
}

void gigadevice_erase_blk(struct spinand_cmd *cmd, u32 page_id)
{
	cmd->addr[0] = (u8)(page_id >> 16);
	cmd->addr[1] = (u8)(page_id >> 8);
	cmd->addr[2] = (u8)(page_id);
}

int gigadevice_verify_ecc(u8 status)
{
	int ecc_status = (status & STATUS_ECC_MASK_GIGA);

	if (ecc_status == STATUS_ECC_ERROR_GIGA)
		return SPINAND_ECC_ERROR;
	else if (ecc_status)
		return SPINAND_ECC_CORRECTED;
	else
		return 0;
}

int macronix_verify_ecc(u8 status)
{
	int ecc_status = (status & STATUS_ECC_MASK_MACRONIX);

	if ((ecc_status == STATUS_ECC_ERROR_MACRONIX) ||
	    (ecc_status == STATUS_ECC_MASK_MACRONIX))
		return SPINAND_ECC_ERROR;
	else if (ecc_status)
		return SPINAND_ECC_CORRECTED;
	else
		return 0;
}

int dummy_verify_ecc(u8 status)
{
	return 0;
}

int gigadevice_parse_id(struct spi_device *spi_nand, u8 *nand_id, u8 *id)
{
	if (nand_id[0] != NAND_MFR_GIGA && nand_id[0] != NAND_MFR_ATO)
		return -EINVAL;

	if (nand_id[0] == NAND_MFR_GIGA) {
		id[0] = nand_id[0];
		id[1] = nand_id[1];
	}

	return 0;
}

int macronix_parse_id(struct spi_device *spi_nand, u8 *nand_id, u8 *id)
{
	if (nand_id[1] != NAND_MFR_MACRONIX)
		return -EINVAL;

	return 0;
}

int winbond_parse_id(struct spi_device *spi_nand, u8 *nand_id, u8 *id)
{
	if (nand_id[1] != NAND_MFR_WINBOND)
		return -EINVAL;

	return 0;
}


MODULE_DESCRIPTION("SPI NAND driver for Gigadevice and Macronix");
