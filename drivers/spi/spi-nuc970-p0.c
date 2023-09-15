/*
 * Copyright (c) 2014 Nuvoton technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/slab.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#include <mach/mfp.h>
#include <linux/platform_data/spi-nuc970.h>

/* spi registers offset */
#define REG_CNTRL		0x00
#define REG_DIVIDER		0x04
#define REG_SSR		0x08
#define REG_RX0		0x10
#define REG_TX0		0x10

/* spi register bit */
#define ENINT		(0x01 << 17)
#define ENFLG		(0x01 << 16)
#define TXNUM		(0x03 << 8)
#define TXNEG		(0x01 << 2)
#define RXNEG		(0x01 << 1)
#define LSB			(0x01 << 10)
#define SELECTLEV	(0x01 << 2)
#define SELECTPOL	(0x01 << 31)
#define SELECTSLAVE0	0x01
#define SELECTSLAVE1	0x02
#define GOBUSY		0x01

struct nuc970_spi {
	struct spi_bitbang	 bitbang;
	struct completion	 done;
	void __iomem		*regs;
	int			 irq;
	unsigned int len;
	unsigned int count;
	const void	*tx;
	void *rx;
	struct clk		*clk;
	struct resource		*ioarea;
	struct spi_master	*master;
	struct spi_device	*curdev;
	struct device		*dev;
	struct nuc970_spi_info *pdata;
	spinlock_t		lock;
	struct resource		*res;
};

static inline struct nuc970_spi0 *to_hw(struct spi_device *sdev)
{
	return spi_master_get_devdata(sdev->master);
}

static inline void nuc970_slave_select(struct spi_device *spi, unsigned int ssr)
{
	struct nuc970_spi *hw = (struct nuc970_spi *)to_hw(spi);
	unsigned int val;
	unsigned int cs = spi->mode & SPI_CS_HIGH ? 1 : 0;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_SSR);

	if (!cs)
		val &= ~SELECTLEV;
	else
		val |= SELECTLEV;

	if(spi->chip_select == 0) {
		if (!ssr)
			val &= ~SELECTSLAVE0;
		else
			val |= SELECTSLAVE0;
	} else {
		if (!ssr)
			val &= ~SELECTSLAVE1;
		else
			val |= SELECTSLAVE1;
	}

	__raw_writel(val, hw->regs + REG_SSR);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static inline void nuc970_spi0_chipsel(struct spi_device *spi, int value)
{
	switch (value) {
	case BITBANG_CS_INACTIVE:
		nuc970_slave_select(spi, 0);
		break;

	case BITBANG_CS_ACTIVE:
		nuc970_slave_select(spi, 1);
		break;
	}
}

static inline void nuc970_spi0_setup_txnum(struct nuc970_spi *hw,
							unsigned int txnum)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_CNTRL);
	val &= ~TXNUM;

	if (txnum)
		val |= (txnum << 0x08);

	__raw_writel(val, hw->regs + REG_CNTRL);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static inline void nuc970_spi0_setup_txbitlen(struct nuc970_spi *hw,
							unsigned int txbitlen)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_CNTRL);
	val &= ~0xf8;
	if(txbitlen != 32)
		val |= (txbitlen << 3);

	__raw_writel(val, hw->regs + REG_CNTRL);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static inline void nuc970_spi0_gobusy(struct nuc970_spi *hw)
{
	unsigned int val;

	val = __raw_readl(hw->regs + REG_CNTRL);
	val |= GOBUSY;
	__raw_writel(val, hw->regs + REG_CNTRL);
}

static inline unsigned int hw_tx(struct nuc970_spi *hw, unsigned int count)
{
	const unsigned char *tx_byte = hw->tx;
	const unsigned short *tx_short = hw->tx;
	const unsigned int *tx_int = hw->tx;
	int bwp = hw->pdata->txbitlen;

	if(bwp <= 8)
		return tx_byte ? tx_byte[count] : 0xffffffff;
	else if(bwp <= 16)
		return tx_short ? tx_short[count] : 0xffffffff;
	else
		return tx_int ? tx_int[count] : 0xffffffff;
}

static inline void hw_rx(struct nuc970_spi *hw, unsigned int data, int count)
{
	unsigned char *rx_byte = hw->rx;
	unsigned short *rx_short = hw->rx;
	unsigned int *rx_int = hw->rx;
	int bwp = hw->pdata->txbitlen;

	if(bwp <= 8)
		rx_byte[count] = data;
	else if(bwp <= 16)
		rx_short[count] = data;
	else
		rx_int[count] = data;
}

static int nuc970_spi0_txrx(struct spi_device *spi, struct spi_transfer *t)
{
	struct nuc970_spi *hw = (struct nuc970_spi *)to_hw(spi);
	int i, offset;

	hw->tx = t->tx_buf;
	hw->rx = t->rx_buf;
	hw->len = t->len;
	hw->count = 0;

	if(t->tx_nbits & SPI_NBITS_DUAL) {
		__raw_writel(__raw_readl(hw->regs + REG_CNTRL) | (0x5 << 20), hw->regs + REG_CNTRL);
	} else if(t->tx_nbits & SPI_NBITS_QUAD) {
		__raw_writel(__raw_readl(hw->regs + REG_CNTRL) | (0x3 << 20), hw->regs + REG_CNTRL);
	}

	if(t->rx_nbits & SPI_NBITS_DUAL) {
		__raw_writel(__raw_readl(hw->regs + REG_CNTRL) | (0x4 << 20), hw->regs + REG_CNTRL);
	} else if(t->rx_nbits & SPI_NBITS_QUAD) {
		__raw_writel(__raw_readl(hw->regs + REG_CNTRL) | (0x2 << 20), hw->regs + REG_CNTRL);
	}

	if(hw->len >= 4) {
		nuc970_spi0_setup_txnum(hw, 3);
		hw->pdata->txnum = 3;
	} else {
		nuc970_spi0_setup_txnum(hw, hw->len-1);
		hw->pdata->txnum = hw->len-1;
	}

	for(i=0, offset=0; i<hw->pdata->txnum+1 ;i++,offset+=4)
		__raw_writel(hw_tx(hw, i), hw->regs + REG_TX0 + offset);
	nuc970_spi0_gobusy(hw);

	wait_for_completion(&hw->done);

	if(spi->mode & (SPI_TX_DUAL | SPI_TX_QUAD | SPI_RX_DUAL | SPI_RX_QUAD))
		__raw_writel(__raw_readl(hw->regs + REG_CNTRL) & ~(0x7 << 20), hw->regs + REG_CNTRL);

	return hw->count;
}

static irqreturn_t nuc970_spi0_irq(int irq, void *dev)
{
	struct nuc970_spi *hw = dev;
	unsigned int status, i, offset;
	unsigned int count = hw->count, last;

	status = __raw_readl(hw->regs + REG_CNTRL);
	__raw_writel(status, hw->regs + REG_CNTRL);

	hw->count += hw->pdata->txnum+1;

	if (hw->rx) {
		for(i=0, offset=0; i<hw->pdata->txnum+1 ;i++,offset+=4)
			hw_rx(hw, __raw_readl(hw->regs + REG_RX0 + offset), count++);
	}

	count = hw->count;
	last = hw->len - count;

	/* Use 4 times transfer automatically */
	if (last > 0) {
		 if ((last > 0) && (last <= 4)) {
			 nuc970_spi0_setup_txnum(hw, last-1);
			 hw->pdata->txnum = last-1;
		 }

		 for(i=0, offset=0; i<hw->pdata->txnum+1 ;i++,offset+=4) {
			__raw_writel(hw_tx(hw, count++), hw->regs + REG_TX0 + offset);
		}
		nuc970_spi0_gobusy(hw);

	} else {
		complete(&hw->done);
	}


	return IRQ_HANDLED;
}

static inline void nuc970_set_clock_polarity(struct nuc970_spi *hw, unsigned int polarity)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_CNTRL);

	if (polarity)
		val |= SELECTPOL;
	else
		val &= ~SELECTPOL;
	__raw_writel(val, hw->regs + REG_CNTRL);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static inline void nuc970_tx_edge(struct nuc970_spi *hw, unsigned int edge)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_CNTRL);

	if (edge)
		val |= TXNEG;
	else
		val &= ~TXNEG;
	__raw_writel(val, hw->regs + REG_CNTRL);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static inline void nuc970_rx_edge(struct nuc970_spi *hw, unsigned int edge)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_CNTRL);

	if (edge)
		val |= RXNEG;
	else
		val &= ~RXNEG;
	__raw_writel(val, hw->regs + REG_CNTRL);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static inline void nuc970_send_first(struct nuc970_spi *hw, unsigned int lsb)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_CNTRL);

	if (lsb)
		val |= LSB;
	else
		val &= ~LSB;
	__raw_writel(val, hw->regs + REG_CNTRL);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static inline void nuc970_set_sleep(struct nuc970_spi *hw, unsigned int sleep)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_CNTRL);

	if (sleep)
		val |= (sleep << 12);
	else
		val &= ~(0x0f << 12);
	__raw_writel(val, hw->regs + REG_CNTRL);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static inline void nuc970_enable_int(struct nuc970_spi *hw)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_CNTRL);

	val |= ENINT;

	__raw_writel(val, hw->regs + REG_CNTRL);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static inline void nuc970_set_divider(struct nuc970_spi *hw)
{
	__raw_writel(hw->pdata->divider, hw->regs + REG_DIVIDER);
}

static int nuc970_spi0_update_state(struct spi_device *spi,
					struct spi_transfer *t)
{
	struct nuc970_spi *hw = (struct nuc970_spi *)to_hw(spi);
	unsigned int clk;
	unsigned int div;
	unsigned int bpw;
	unsigned int hz;
	unsigned char spimode;

	bpw = t ? t->bits_per_word : spi->bits_per_word;
	hz  = t ? t->speed_hz : spi->max_speed_hz;

	if(hw->pdata->txbitlen != bpw)
		hw->pdata->txbitlen = bpw;

	if(hw->pdata->hz != hz) {
		clk = clk_get_rate(hw->clk);
		div = DIV_ROUND_UP(clk, hz * 2) - 1;
		hw->pdata->hz = hz;
		hw->pdata->divider = div;
	}

#if defined(CONFIG_OF)
	if(hw->pdata->quad)
		spi->mode |= (SPI_TX_QUAD | SPI_RX_QUAD);
	else
		spi->mode |= (SPI_RX_DUAL | SPI_TX_DUAL);
#else
#if defined(CONFIG_SPI_NUC970_P0_QUAD)
		spi->mode |= (SPI_TX_QUAD | SPI_RX_QUAD);
#elif defined(CONFIG_SPI_NUC970_P0_NORMAL)
		spi->mode |= (SPI_RX_DUAL | SPI_TX_DUAL);
#endif
#endif

	//Mode 0: CPOL=0, CPHA=0; active high
	//Mode 1: CPOL=0, CPHA=1 ;active low
	//Mode 2: CPOL=1, CPHA=0 ;active low
	//Mode 3: POL=1, CPHA=1;active high
	if (spi->mode & SPI_CPOL)
		hw->pdata->clkpol = 1;
	else
		hw->pdata->clkpol = 0;

	spimode = spi->mode & 0xff; //remove dual/quad bit

	if ((spimode == SPI_MODE_0) || (spimode == SPI_MODE_2)) {
		hw->pdata->txneg = 1;
		hw->pdata->rxneg = 0;
	} else {
		hw->pdata->txneg = 0;
		hw->pdata->rxneg = 1;
	}

	if (spi->mode & SPI_LSB_FIRST)
		hw->pdata->lsb = 1;
	else
		hw->pdata->lsb = 0;

	return 0;
}

static int nuc970_spi0_setupxfer(struct spi_device *spi,
					struct spi_transfer *t)
{
	struct nuc970_spi *hw = (struct nuc970_spi *)to_hw(spi);
	int ret;

	ret = nuc970_spi0_update_state(spi, t);
	if (ret)
		return ret;

	nuc970_set_divider(hw);
	nuc970_spi0_setup_txbitlen(hw, hw->pdata->txbitlen);
	nuc970_tx_edge(hw, hw->pdata->txneg);
	nuc970_rx_edge(hw, hw->pdata->rxneg);
	nuc970_set_clock_polarity(hw, hw->pdata->clkpol);
	nuc970_send_first(hw, hw->pdata->lsb);

	return 0;
}

static int nuc970_spi0_setup(struct spi_device *spi)
{
	struct nuc970_spi *hw = (struct nuc970_spi *)to_hw(spi);
	int ret;

	ret = nuc970_spi0_update_state(spi, NULL);
	if (ret)
		return ret;

	spin_lock(&hw->bitbang.lock);
	if (!hw->bitbang.busy) {
		nuc970_set_divider(hw);
		nuc970_slave_select(spi, 0);
	}
	spin_unlock(&hw->bitbang.lock);

	return 0;
}

static void nuc970_init_spi(struct nuc970_spi *hw)
{
	clk_prepare(hw->clk);
	clk_enable(hw->clk);

	spin_lock_init(&hw->lock);

	nuc970_tx_edge(hw, hw->pdata->txneg);
	nuc970_rx_edge(hw, hw->pdata->rxneg);
	nuc970_send_first(hw, hw->pdata->lsb);
	nuc970_set_sleep(hw, hw->pdata->sleep);
	nuc970_spi0_setup_txbitlen(hw, hw->pdata->txbitlen);
	nuc970_spi0_setup_txnum(hw, hw->pdata->txnum);
	nuc970_set_clock_polarity(hw, hw->pdata->clkpol);
	nuc970_set_divider(hw);
	nuc970_enable_int(hw);
}

#ifdef CONFIG_OF
static struct nuc970_spi_info *nuc970_spi0_parse_dt(struct device *dev)
{
	struct nuc970_spi_info *sci;
	u32 temp;

	sci = devm_kzalloc(dev, sizeof(*sci), GFP_KERNEL);
	if (!sci) {
		dev_err(dev, "memory allocation for spi_info failed\n");
		return ERR_PTR(-ENOMEM);
	}

	if (of_property_read_u32(dev->of_node, "num_cs", &temp)) {
		dev_warn(dev, "can't get num_cs from dt\n");
		sci->num_cs = 2;
	} else {
		sci->num_cs = temp;
	}

	if (of_property_read_u32(dev->of_node, "lsb", &temp)) {
		dev_warn(dev, "can't get lsb from dt\n");
		sci->lsb = 0;
	} else {
		sci->lsb = temp;
	}

	if (of_property_read_u32(dev->of_node, "txneg", &temp)) {
		dev_warn(dev, "can't get txneg from dt\n");
		sci->txneg = 1;
	} else {
		sci->txneg = temp;
	}

	if (of_property_read_u32(dev->of_node, "txneg", &temp)) {
		dev_warn(dev, "can't get txneg from dt\n");
		sci->txneg = 0;
	} else {
		sci->txneg = temp;
	}

	if (of_property_read_u32(dev->of_node, "rxneg", &temp)) {
		dev_warn(dev, "can't get rxneg from dt\n");
		sci->rxneg = 1;
	} else {
		sci->rxneg = temp;
	}

	if (of_property_read_u32(dev->of_node, "divider", &temp)) {
		dev_warn(dev, "can't get divider from dt\n");
		sci->divider = 4;
	} else {
		sci->divider = temp;
	}

	if (of_property_read_u32(dev->of_node, "sleep", &temp)) {
		dev_warn(dev, "can't get sleep from dt\n");
		sci->sleep = 0;
	} else {
		sci->sleep = temp;
	}

	if (of_property_read_u32(dev->of_node, "txnum", &temp)) {
		dev_warn(dev, "can't get txnum from dt\n");
		sci->txnum = 0;
	} else {
		sci->txnum = temp;
	}

	if (of_property_read_u32(dev->of_node, "txbitlen", &temp)) {
		dev_warn(dev, "can't get txbitlen from dt\n");
		sci->txbitlen = 8;
	} else {
		sci->txbitlen = temp;
	}

	if (of_property_read_u32(dev->of_node, "bus_num", &temp)) {
		dev_warn(dev, "can't get bus_num from dt\n");
		sci->bus_num = 0;
	} else {
		sci->bus_num = temp;
	}

	if (of_property_read_u32(dev->of_node, "quad", &temp)) {
		dev_warn(dev, "can't get quad from dt\n");
		sci->quad = 0;
	} else {
		sci->quad = temp;
	}
	return sci;
}
#else
static struct nuc970_spi_info *nuc970_spi0_parse_dt(struct device *dev)
{
	return dev->platform_data;
}
#endif

static int nuc970_spi0_probe(struct platform_device *pdev)
{
	struct nuc970_spi *hw;
	struct spi_master *master;
	int err = 0;
	struct pinctrl *p;

	master = spi_alloc_master(&pdev->dev, sizeof(struct nuc970_spi));
	if (master == NULL) {
		dev_err(&pdev->dev, "No memory for spi_master\n");
		err = -ENOMEM;
		goto err_nomem;
	}

	hw = spi_master_get_devdata(master);
	hw->master = spi_master_get(master);
	hw->pdata = nuc970_spi0_parse_dt(&pdev->dev);
	hw->dev = &pdev->dev;

	if (hw->pdata == NULL) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		err = -ENOENT;
		goto err_pdata;
	}

	platform_set_drvdata(pdev, hw);
	init_completion(&hw->done);
#if defined(CONFIG_OF)
	if (hw->pdata->quad)
		master->mode_bits = (SPI_MODE_0 | SPI_TX_DUAL | SPI_RX_DUAL | SPI_TX_QUAD | SPI_RX_QUAD | SPI_CS_HIGH | SPI_LSB_FIRST | SPI_CPHA | SPI_CPOL);
	else
		master->mode_bits = (SPI_MODE_0 | SPI_TX_DUAL | SPI_RX_DUAL | SPI_CS_HIGH | SPI_LSB_FIRST | SPI_CPHA | SPI_CPOL);
#else
#if defined(CONFIG_SPI_NUC970_P0_NORMAL)
	master->mode_bits          = (SPI_MODE_0 | SPI_TX_DUAL | SPI_RX_DUAL | SPI_CS_HIGH | SPI_LSB_FIRST | SPI_CPHA | SPI_CPOL);
#elif defined(CONFIG_SPI_NUC970_P0_QUAD)
	master->mode_bits          = (SPI_MODE_0 | SPI_TX_DUAL | SPI_RX_DUAL | SPI_TX_QUAD | SPI_RX_QUAD | SPI_CS_HIGH | SPI_LSB_FIRST | SPI_CPHA | SPI_CPOL);
#endif
#endif
	master->dev.of_node        = pdev->dev.of_node;
	master->num_chipselect     = hw->pdata->num_cs;
	master->bus_num            = hw->pdata->bus_num;
	hw->bitbang.master         = hw->master;
	hw->bitbang.setup_transfer = nuc970_spi0_setupxfer;
	hw->bitbang.chipselect     = nuc970_spi0_chipsel;
	hw->bitbang.txrx_bufs      = nuc970_spi0_txrx;
	hw->bitbang.master->setup  = nuc970_spi0_setup;

	hw->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (hw->res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		err = -ENOENT;
		goto err_pdata;
	}

#if defined(CONFIG_OF)
	hw->regs = devm_ioremap_resource(&pdev->dev, hw->res);
#else
	hw->ioarea = request_mem_region(hw->res->start,
					resource_size(hw->res), pdev->name);

	if (hw->ioarea == NULL) {
		dev_err(&pdev->dev, "Cannot reserve region\n");
		err = -ENXIO;
		goto err_pdata;
	}

	hw->regs = ioremap(hw->res->start, resource_size(hw->res));
	if (hw->regs == NULL) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		err = -ENXIO;
		goto err_iomap;
	}
#endif

	hw->irq = platform_get_irq(pdev, 0);
	if (hw->irq < 0) {
		dev_err(&pdev->dev, "No IRQ specified\n");
		err = -ENOENT;
		goto err_irq;
	}

	err = request_irq(hw->irq, nuc970_spi0_irq, 0, pdev->name, hw);
	if (err) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		goto err_irq;
	}

	hw->clk = clk_get(NULL, "spi0");
	if (IS_ERR(hw->clk)) {
		dev_err(&pdev->dev, "No clock for device\n");
		err = PTR_ERR(hw->clk);
		goto err_clk;
	}

#if defined(CONFIG_OF)
	p = devm_pinctrl_get_select_default(&pdev->dev);
#else
 #if defined(CONFIG_SPI_NUC970_P0_NORMAL) && !defined(CONFIG_SPI_NUC970_P0_SS1)
	p = devm_pinctrl_get_select(&pdev->dev, "spi0");
 #elif defined(CONFIG_SPI_NUC970_P0_NORMAL) && defined(CONFIG_SPI_NUC970_P0_SS1_PB0)
	p = devm_pinctrl_get_select(&pdev->dev, "spi0-ss1-PB");
 #elif defined(CONFIG_SPI_NUC970_P0_NORMAL) && defined(CONFIG_SPI_NUC970_P0_SS1_PH12)
	p = devm_pinctrl_get_select(&pdev->dev, "spi0-ss1-PH");
 #elif defined(CONFIG_SPI_NUC970_P0_QUAD) && !defined(CONFIG_SPI_NUC970_P0_SS1)
	p = devm_pinctrl_get_select(&pdev->dev, "spi0-quad");
 #elif defined(CONFIG_SPI_NUC970_P0_QUAD) && defined(CONFIG_SPI_NUC970_P0_SS1_PB0)
	p = devm_pinctrl_get_select(&pdev->dev, "spi0-quad-ss1-PB");
 #elif defined(CONFIG_SPI_NUC970_P0_QUAD) && defined(CONFIG_SPI_NUC970_P0_SS1_PH12)
	p = devm_pinctrl_get_select(&pdev->dev, "spi0-quad-ss1-PH");
 #endif
#endif
	if(IS_ERR(p)) {
		dev_err(&pdev->dev, "unable to reserve spi pin by mode\n");
		err = PTR_ERR(p);
	}

	nuc970_init_spi(hw);

	err = spi_bitbang_start(&hw->bitbang);
	if (err) {
		dev_err(&pdev->dev, "Failed to register SPI master\n");
		goto err_register;
	}

	return 0;

err_register:
	clk_disable(hw->clk);
	clk_put(hw->clk);
err_clk:
	free_irq(hw->irq, hw);
err_irq:
	iounmap(hw->regs);
#ifndef CONFIG_OF
err_iomap:
	release_mem_region(hw->res->start, resource_size(hw->res));
	kfree(hw->ioarea);
#endif
err_pdata:
	spi_master_put(hw->master);

err_nomem:
	return err;
}

static int nuc970_spi0_remove(struct platform_device *dev)
{
	struct nuc970_spi *hw = platform_get_drvdata(dev);

	free_irq(hw->irq, hw);
	platform_set_drvdata(dev, NULL);
	spi_bitbang_stop(&hw->bitbang);

	clk_disable(hw->clk);
	clk_put(hw->clk);

	iounmap(hw->regs);

#ifndef CONFIG_OF
	release_mem_region(hw->res->start, resource_size(hw->res));
	kfree(hw->ioarea);
#else
	kfree(hw->pdata);
#endif

	spi_master_put(hw->master);
	return 0;
}

#ifdef CONFIG_PM
static int nuc970_spi0_suspend(struct device *dev)
{
	struct nuc970_spi *hw = dev_get_drvdata(dev);

	while(__raw_readl(hw->regs + REG_CNTRL) & 0x1)
		msleep(1);

	// disable interrupt
	__raw_writel((__raw_readl(hw->regs + REG_CNTRL) & ~0x20000), hw->regs + REG_CNTRL);

	return 0;
}

static int nuc970_spi0_resume(struct device *dev)
{
	struct nuc970_spi *hw = dev_get_drvdata(dev);

	// enable interrupt
	__raw_writel((__raw_readl(hw->regs + REG_CNTRL) | 0x20000), hw->regs + REG_CNTRL);

	return 0;
}

static const struct dev_pm_ops nuc970_spi0_pmops = {
	.suspend    = nuc970_spi0_suspend,
	.resume     = nuc970_spi0_resume,
};

#define NUC970_SPI0_PMOPS (&nuc970_spi0_pmops)

#else
#define NUC970_SPI0_PMOPS NULL
#endif

#if defined(CONFIG_OF)
static const struct of_device_id nuc970_spi0_of_match[] = {
	{   .compatible = "nuvoton,nuc970-spi0" } ,
	{	},
};
MODULE_DEVICE_TABLE(of, nuc970_spi0_of_match);
#endif

static struct platform_driver nuc970_spi0_driver = {
	.probe      = nuc970_spi0_probe,
	.remove     = nuc970_spi0_remove,
	.driver     = {
		.name   = "nuc970-spi0",
		.owner  = THIS_MODULE,
		.pm	= NUC970_SPI0_PMOPS,
#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(nuc970_spi0_of_match),
#endif
	},
};
module_platform_driver(nuc970_spi0_driver);

MODULE_AUTHOR("Wan ZongShun <mcuos.com@gmail.com>");
MODULE_DESCRIPTION("NUC970/N9H30 SPI0 driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nuc970-spi0");
