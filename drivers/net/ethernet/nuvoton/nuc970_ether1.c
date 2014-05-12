/*
 * Copyright (c) 2014 Nuvoton Technology Corporation.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/mii.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/gfp.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>

#define DRV_MODULE_NAME		"nuc970-emc1"
#define DRV_MODULE_VERSION	"1.0"

/* Ethernet MAC1 Registers */
#define REG_CAMCMR		(void __iomem *)0xF0003000
#define REG_CAMEN		(void __iomem *)0xF0003004
#define REG_CAMM_BASE		(void __iomem *)0xF0003008
#define REG_CAML_BASE		(void __iomem *)0xF000300c
#define REG_TXDLSA		(void __iomem *)0xF0003088
#define REG_RXDLSA		(void __iomem *)0xF000308C
#define REG_MCMDR		(void __iomem *)0xF0003090
#define REG_MIID		(void __iomem *)0xF0003094
#define REG_MIIDA		(void __iomem *)0xF0003098
#define REG_FFTCR		(void __iomem *)0xF000309C
#define REG_TSDR		(void __iomem *)0xF00030a0
#define REG_RSDR		(void __iomem *)0xF00030a4
#define REG_DMARFC		(void __iomem *)0xF00030a8
#define REG_MIEN		(void __iomem *)0xF00030ac
#define REG_MISTA		(void __iomem *)0xF00030b0
#define REG_CTXDSA		(void __iomem *)0xF00030cc
#define REG_CTXBSA		(void __iomem *)0xF00030d0
#define REG_CRXDSA		(void __iomem *)0xF00030d4
#define REG_CRXBSA		(void __iomem *)0xF00030d8

/* mac controller bit */
#define MCMDR_RXON		0x01
#define MCMDR_ACP		(0x01 << 3)
#define MCMDR_SPCRC		(0x01 << 5)
#define MCMDR_TXON		(0x01 << 8)
#define MCMDR_FDUP		(0x01 << 18)
#define MCMDR_ENMDC		(0x01 << 19)
#define MCMDR_OPMOD		(0x01 << 20)
#define SWR				(0x01 << 24)

/* cam command regiser */
#define CAMCMR_AUP		0x01
#define CAMCMR_AMP		(0x01 << 1)
#define CAMCMR_ABP		(0x01 << 2)
#define CAMCMR_CCAM		(0x01 << 3)
#define CAMCMR_ECMP		(0x01 << 4)
#define CAM0EN			0x01

/* mac mii controller bit */
#define MDCCR			(0x0a << 20)
#define PHYAD			(0x01 << 8)
#define PHYWR			(0x01 << 16)
#define PHYBUSY			(0x01 << 17)
#define PHYPRESP		(0x01 << 18)
#define CAM_ENTRY_SIZE		0x08

/* rx and tx status */
#define TXDS_TXCP		(0x01 << 19)
#define RXDS_CRCE		(0x01 << 17)
#define RXDS_PTLE		(0x01 << 19)
#define RXDS_RXGD		(0x01 << 20)
#define RXDS_ALIE		(0x01 << 21)
#define RXDS_RP			(0x01 << 22)

/* mac interrupt status*/
#define MISTA_EXDEF		(0x01 << 19)
#define MISTA_TXBERR	(0x01 << 24)
#define MISTA_TDU		(0x01 << 23)
#define MISTA_RDU		(0x01 << 10)
#define MISTA_RXBERR	(0x01 << 11)

#define ENSTART			0x01
#define ENRXINTR		0x01
#define ENRXGD			(0x01 << 4)
#define ENRXBERR		(0x01 << 11)
#define ENTXINTR		(0x01 << 16)
#define ENTXCP			(0x01 << 18)
#define ENTXABT			(0x01 << 21)
#define ENTXBERR		(0x01 << 24)
#define ENMDC			(0x01 << 19)
#define PHYBUSY			(0x01 << 17)
#define MDCCR_VAL		0xa00000

/* rx and tx owner bit */
#define RX_OWEN_DMA		(0x01 << 31)
#define RX_OWEN_CPU		(~(0x03 << 30))
#define TX_OWEN_DMA		(0x01 << 31)
#define TX_OWEN_CPU		(~(0x01 << 31))

/* tx frame desc controller bit */
#define MACTXINTEN		0x04
#define CRCMODE			0x02
#define PADDINGMODE		0x01

/* fftcr controller bit */
#define TXTHD 			(0x03 << 8)
#define BLENGTH			(0x01 << 20)

/* global setting for driver */
#define RX_DESC_SIZE	32
#define TX_DESC_SIZE	16
#define MAX_RBUFF_SZ	0x600
#define MAX_TBUFF_SZ	0x600
#define TX_TIMEOUT		50
#define DELAY			1000
#define CAM0			0x0

static int nuc970_mdio_read(struct net_device *dev, int phy_id, int reg);
extern void mfp_set_groupf(struct device *dev);

struct nuc970_rxbd {
	unsigned int sl;
	unsigned int buffer;
	unsigned int reserved;
	unsigned int next;
};

struct nuc970_txbd {
	unsigned int mode;
	unsigned int buffer;
	unsigned int sl;
	unsigned int next;
};

u8 nuc970_mac1[6] = { 0x08, 0x00, 0x27, 0x00, 0x01, 0x93 };

struct recv_pdesc {
	struct nuc970_rxbd desclist[RX_DESC_SIZE];
	char recv_buf[RX_DESC_SIZE][MAX_RBUFF_SZ];
};

struct tran_pdesc {
	struct nuc970_txbd desclist[TX_DESC_SIZE];
	char tran_buf[TX_DESC_SIZE][MAX_TBUFF_SZ];
};

struct  nuc970_ether {
	struct recv_pdesc *rdesc;
	struct tran_pdesc *tdesc;
	dma_addr_t rdesc_phys;
	dma_addr_t tdesc_phys;
	struct net_device_stats stats;
	struct platform_device *pdev;
	struct resource *res;
	struct sk_buff *skb;
	struct clk *clk;
	struct clk *rmiiclk;
	struct mii_if_info mii;
	struct timer_list check_timer;
	int rxirq;
	int txirq;
	unsigned int cur_tx;
	unsigned int cur_rx;
	unsigned int finish_tx;
	unsigned int rx_packets;
	unsigned int rx_bytes;
	unsigned int start_tx_ptr;
	unsigned int start_rx_ptr;
	unsigned int linkflag;
};


static void update_linkspeed_register(struct net_device *dev,
				unsigned int speed, unsigned int duplex)
{
	unsigned int val;

	val = __raw_readl( REG_MCMDR);

	if (speed == SPEED_100) {
		/* 100 full/half duplex */
		if (duplex == DUPLEX_FULL) {
			val |= (MCMDR_OPMOD | MCMDR_FDUP);
		} else {
			val |= MCMDR_OPMOD;
			val &= ~MCMDR_FDUP;
		}
	} else {
		/* 10 full/half duplex */
		if (duplex == DUPLEX_FULL) {
			val |= MCMDR_FDUP;
			val &= ~MCMDR_OPMOD;
		} else {
			val &= ~(MCMDR_FDUP | MCMDR_OPMOD);
		}
	}

	__raw_writel(val,  REG_MCMDR);
}

static void update_linkspeed(struct net_device *dev)
{
	struct nuc970_ether *ether = netdev_priv(dev);
	struct platform_device *pdev;
	unsigned int bmsr, bmcr, lpa, speed, duplex;

	pdev = ether->pdev;

	if (!mii_link_ok(&ether->mii)) {
		ether->linkflag = 0x0;
		netif_carrier_off(dev);
		dev_dbg(&pdev->dev, "%s: Link down.\n", dev->name);
		return;
	}

	if (ether->linkflag == 1)
		return;

	bmsr = nuc970_mdio_read(dev, ether->mii.phy_id, MII_BMSR);
	bmcr = nuc970_mdio_read(dev, ether->mii.phy_id, MII_BMCR);

	if (bmcr & BMCR_ANENABLE) {
		if (!(bmsr & BMSR_ANEGCOMPLETE))
			return;

		lpa = nuc970_mdio_read(dev, ether->mii.phy_id, MII_LPA);

		if ((lpa & LPA_100FULL) || (lpa & LPA_100HALF))
			speed = SPEED_100;
		else
			speed = SPEED_10;

		if ((lpa & LPA_100FULL) || (lpa & LPA_10FULL))
			duplex = DUPLEX_FULL;
		else
			duplex = DUPLEX_HALF;

	} else {
		speed = (bmcr & BMCR_SPEED100) ? SPEED_100 : SPEED_10;
		duplex = (bmcr & BMCR_FULLDPLX) ? DUPLEX_FULL : DUPLEX_HALF;
	}

	update_linkspeed_register(dev, speed, duplex);

	dev_info(&pdev->dev, "%s: Link now %i-%s\n", dev->name, speed,
			(duplex == DUPLEX_FULL) ? "FullDuplex" : "HalfDuplex");
			
			
	ether->linkflag = 0x01;

	netif_carrier_on(dev);
}

static void nuc970_check_link(unsigned long dev_id)
{
	struct net_device *dev = (struct net_device *) dev_id;
	struct nuc970_ether *ether = netdev_priv(dev);

	update_linkspeed(dev);
	mod_timer(&ether->check_timer, jiffies + msecs_to_jiffies(1000));
}

static void nuc970_write_cam(struct net_device *dev,
				unsigned int x, unsigned char *pval)
{
	unsigned int msw, lsw;

	msw = (pval[0] << 24) | (pval[1] << 16) | (pval[2] << 8) | pval[3];

	lsw = (pval[4] << 24) | (pval[5] << 16);

	__raw_writel(lsw,  REG_CAML_BASE + x * CAM_ENTRY_SIZE);
	__raw_writel(msw,  REG_CAMM_BASE + x * CAM_ENTRY_SIZE);
}

static int nuc970_init_desc(struct net_device *dev)
{
	struct nuc970_ether *ether;
	struct nuc970_txbd  *tdesc;
	struct nuc970_rxbd  *rdesc;
	struct platform_device *pdev;
	unsigned int i;

	ether = netdev_priv(dev);
	pdev = ether->pdev;

	ether->tdesc = (struct tran_pdesc *)
		dma_alloc_coherent(&pdev->dev, sizeof(struct tran_pdesc),
					&ether->tdesc_phys, GFP_KERNEL);

	if (!ether->tdesc) {
		dev_err(&pdev->dev, "Failed to allocate memory for tx desc\n");
		return -ENOMEM;
	}

	ether->rdesc = (struct recv_pdesc *)
		dma_alloc_coherent(&pdev->dev, sizeof(struct recv_pdesc),
					&ether->rdesc_phys, GFP_KERNEL);

	if (!ether->rdesc) {
		dev_err(&pdev->dev, "Failed to allocate memory for rx desc\n");
		dma_free_coherent(&pdev->dev, sizeof(struct tran_pdesc),
					ether->tdesc, ether->tdesc_phys);
		return -ENOMEM;
	}

	for (i = 0; i < TX_DESC_SIZE; i++) {
		unsigned int offset;

		tdesc = &(ether->tdesc->desclist[i]);

		if (i == TX_DESC_SIZE - 1)
			offset = offsetof(struct tran_pdesc, desclist[0]);
		else
			offset = offsetof(struct tran_pdesc, desclist[i + 1]);

		tdesc->next = ether->tdesc_phys + offset;
		tdesc->buffer = ether->tdesc_phys +
			offsetof(struct tran_pdesc, tran_buf[i]);
		tdesc->sl = 0;
		tdesc->mode = 0;
	}

	ether->start_tx_ptr = ether->tdesc_phys;

	for (i = 0; i < RX_DESC_SIZE; i++) {
		unsigned int offset;

		rdesc = &(ether->rdesc->desclist[i]);

		if (i == RX_DESC_SIZE - 1)
			offset = offsetof(struct recv_pdesc, desclist[0]);
		else
			offset = offsetof(struct recv_pdesc, desclist[i + 1]);

		rdesc->next = ether->rdesc_phys + offset;
		rdesc->sl = RX_OWEN_DMA;
		rdesc->buffer = ether->rdesc_phys +
			offsetof(struct recv_pdesc, recv_buf[i]);
	  }

	ether->start_rx_ptr = ether->rdesc_phys;

	return 0;
}

static void nuc970_set_fifo_threshold(struct net_device *dev)
{
	unsigned int val;

	val = TXTHD | BLENGTH;
	__raw_writel(val,  REG_FFTCR);
}

static void nuc970_return_default_idle(struct net_device *dev)
{
	unsigned int val;

	val = __raw_readl( REG_MCMDR);
	val |= SWR;
	__raw_writel(val,  REG_MCMDR);
}

static void nuc970_trigger_rx(struct net_device *dev)
{

	__raw_writel(ENSTART,  REG_RSDR);
}

static void nuc970_trigger_tx(struct net_device *dev)
{

	__raw_writel(ENSTART,  REG_TSDR);
}

static void nuc970_enable_mac_interrupt(struct net_device *dev)
{
	unsigned int val;

	val = ENTXINTR | ENRXINTR | ENRXGD | ENTXCP;
	val |= ENTXBERR | ENRXBERR | ENTXABT;

	__raw_writel(val,  REG_MIEN);
}

static void nuc970_get_and_clear_int(struct net_device *dev,
							unsigned int *val, unsigned int mask)
{
	*val = __raw_readl( REG_MISTA) & mask;
	__raw_writel(*val,  REG_MISTA);
}

static void nuc970_set_global_maccmd(struct net_device *dev)
{
	unsigned int val;

	val = __raw_readl( REG_MCMDR);
	val |= MCMDR_SPCRC | MCMDR_ENMDC | MCMDR_ACP | ENMDC;
	__raw_writel(val,  REG_MCMDR);
}

static void nuc970_enable_cam(struct net_device *dev)
{
	unsigned int val;

	nuc970_write_cam(dev, CAM0, dev->dev_addr);

	val = __raw_readl( REG_CAMEN);
	val |= CAM0EN;
	__raw_writel(val,  REG_CAMEN);
}

static void nuc970_enable_cam_command(struct net_device *dev)
{
	unsigned int val;

	val = CAMCMR_ECMP | CAMCMR_ABP | CAMCMR_AMP;
	__raw_writel(val,  REG_CAMCMR);
}

static void nuc970_enable_tx(struct net_device *dev, unsigned int enable)
{
	unsigned int val;

	val = __raw_readl( REG_MCMDR);

	if (enable)
		val |= MCMDR_TXON;
	else
		val &= ~MCMDR_TXON;

	__raw_writel(val,  REG_MCMDR);
}

static void nuc970_enable_rx(struct net_device *dev, unsigned int enable)
{
	unsigned int val;

	val = __raw_readl(REG_MCMDR);

	if (enable)
		val |= MCMDR_RXON;
	else
		val &= ~MCMDR_RXON;

	__raw_writel(val,  REG_MCMDR);
}

static void nuc970_set_curdest(struct net_device *dev)
{
	struct nuc970_ether *ether = netdev_priv(dev);

	__raw_writel(ether->start_rx_ptr,  REG_RXDLSA);
	__raw_writel(ether->start_tx_ptr,  REG_TXDLSA);
}

static void nuc970_reset_mac(struct net_device *dev)
{
	struct nuc970_ether *ether = netdev_priv(dev);

	nuc970_enable_tx(dev, 0);
	nuc970_enable_rx(dev, 0);

	nuc970_return_default_idle(dev);
	nuc970_set_fifo_threshold(dev);

	if (!netif_queue_stopped(dev))
		netif_stop_queue(dev);

	nuc970_init_desc(dev);

	dev->trans_start = jiffies; /* prevent tx timeout */
	ether->cur_tx = 0x0;
	ether->finish_tx = 0x0;
	ether->cur_rx = 0x0;

	nuc970_set_curdest(dev);
	nuc970_enable_cam(dev);
	nuc970_enable_cam_command(dev);
	nuc970_enable_mac_interrupt(dev);
	nuc970_enable_tx(dev, 1);
	nuc970_enable_rx(dev, 1);
	nuc970_trigger_tx(dev);
	nuc970_trigger_rx(dev);

	dev->trans_start = jiffies; /* prevent tx timeout */

	if (netif_queue_stopped(dev))
		netif_wake_queue(dev);
}

static void nuc970_mdio_write(struct net_device *dev,
					int phy_id, int reg, int data)
{
	struct nuc970_ether *ether = netdev_priv(dev);
	struct platform_device *pdev;
	unsigned int val, i;

	pdev = ether->pdev;
	
	__raw_writel(data,  REG_MIID);

	val = (phy_id << 0x08) | reg;
	val |= PHYBUSY | PHYWR | MDCCR_VAL;
	__raw_writel(val,  REG_MIIDA);

	for (i = 0; i < DELAY; i++) {
		if ((__raw_readl( REG_MIIDA) & PHYBUSY) == 0)
			break;
	}

	if (i == DELAY)
		dev_warn(&pdev->dev, "mdio write timed out\n");
}

static int nuc970_mdio_read(struct net_device *dev, int phy_id, int reg)
{
	struct nuc970_ether *ether = netdev_priv(dev);
	struct platform_device *pdev;
	unsigned int val, i, data;

	pdev = ether->pdev;

	val = (phy_id << 0x08) | reg;
	val |= PHYBUSY | MDCCR_VAL;
	__raw_writel(val,  REG_MIIDA);

	for (i = 0; i < DELAY; i++) {
		if ((__raw_readl( REG_MIIDA) & PHYBUSY) == 0)
			break;
	}

	if (i == DELAY) {
		dev_warn(&pdev->dev, "mdio read timed out\n");
		data = 0xffff;
	} else {
		data = __raw_readl( REG_MIID);
	}

	return data;
}

static int nuc970_set_mac_address(struct net_device *dev, void *addr)
{
	struct sockaddr *address = addr;

	if (!is_valid_ether_addr(address->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(dev->dev_addr, address->sa_data, dev->addr_len);
	nuc970_write_cam(dev, CAM0, dev->dev_addr);

	return 0;
}

static int nuc970_ether_close(struct net_device *dev)
{
	struct nuc970_ether *ether = netdev_priv(dev);
	struct platform_device *pdev;

	pdev = ether->pdev;

	dma_free_coherent(&pdev->dev, sizeof(struct recv_pdesc),
					ether->rdesc, ether->rdesc_phys);
	dma_free_coherent(&pdev->dev, sizeof(struct tran_pdesc),
					ether->tdesc, ether->tdesc_phys);

	netif_stop_queue(dev);

	del_timer_sync(&ether->check_timer);
	clk_disable(ether->rmiiclk);
	clk_disable(ether->clk);

	free_irq(ether->txirq, dev);
	free_irq(ether->rxirq, dev);

	return 0;
}

static struct net_device_stats *nuc970_ether_stats(struct net_device *dev)
{
	struct nuc970_ether *ether;

	ether = netdev_priv(dev);

	return &ether->stats;
}

static int nuc970_send_frame(struct net_device *dev,
					unsigned char *data, int length)
{
	struct nuc970_ether *ether;
	struct nuc970_txbd *txbd;
	struct platform_device *pdev;
	unsigned char *buffer;

	ether = netdev_priv(dev);
	pdev = ether->pdev;

	txbd = &ether->tdesc->desclist[ether->cur_tx];
	buffer = ether->tdesc->tran_buf[ether->cur_tx];

	if (length > 1514) {
		dev_err(&pdev->dev, "send data %d bytes, check it\n", length);
		length = 1514;
	}

	txbd->sl = length /*& 0xFFFF*/;

	memcpy(buffer, data, length);

	txbd->mode = TX_OWEN_DMA | PADDINGMODE | CRCMODE | MACTXINTEN;

	nuc970_enable_tx(dev, 1);

	nuc970_trigger_tx(dev);

	if (++ether->cur_tx >= TX_DESC_SIZE)
		ether->cur_tx = 0;

	txbd = &ether->tdesc->desclist[ether->cur_tx];

	if (txbd->mode & TX_OWEN_DMA)
		netif_stop_queue(dev);

	return 0;
}

static int nuc970_ether_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	if (!(nuc970_send_frame(dev, skb->data, skb->len))) {
		//ether->skb = skb;
		dev_kfree_skb_irq(skb);
		return 0;
	}
	return -EAGAIN;
}

static irqreturn_t nuc970_tx_interrupt(int irq, void *dev_id)
{
	struct nuc970_ether *ether;
	struct nuc970_txbd  *txbd;
	struct platform_device *pdev;
	struct net_device *dev;
	unsigned int cur_entry, entry, status;

printk("[tx]\n");
	dev = dev_id;
	ether = netdev_priv(dev);
	pdev = ether->pdev;

	nuc970_get_and_clear_int(dev, &status, 0xFFFF0000);

	cur_entry = __raw_readl( REG_CTXDSA);

	entry = ether->tdesc_phys +
		offsetof(struct tran_pdesc, desclist[ether->finish_tx]);

	while (entry != cur_entry) {
		txbd = &ether->tdesc->desclist[ether->finish_tx];

		if (++ether->finish_tx >= TX_DESC_SIZE)
			ether->finish_tx = 0;

		if (txbd->sl & TXDS_TXCP) {
			ether->stats.tx_packets++;
			ether->stats.tx_bytes += txbd->sl & 0xFFFF;
		} else {
			ether->stats.tx_errors++;
		}

		txbd->sl = 0x0;
		txbd->mode = 0x0;

		//if (netif_queue_stopped(dev))
			netif_wake_queue(dev);

		entry = ether->tdesc_phys +
			offsetof(struct tran_pdesc, desclist[ether->finish_tx]);
	}

	if (status & MISTA_EXDEF) {
		dev_err(&pdev->dev, "emc defer exceed interrupt\n");
	} else if (status & MISTA_TXBERR) {
		dev_err(&pdev->dev, "emc bus error interrupt\n");
		nuc970_reset_mac(dev);
	} else if (status & MISTA_TDU) {
		//if (netif_queue_stopped(dev))
			netif_wake_queue(dev);
	}

	return IRQ_HANDLED;
}

static void netdev_rx(struct net_device *dev)
{
	struct nuc970_ether *ether;
	struct nuc970_rxbd *rxbd;
	struct platform_device *pdev;
	struct sk_buff *skb;
	unsigned char *data;
	unsigned int length, status, val, entry;

	ether = netdev_priv(dev);
	pdev = ether->pdev;

	rxbd = &ether->rdesc->desclist[ether->cur_rx];

	do {

                val = __raw_readl(REG_CRXDSA);

                entry = ether->rdesc_phys +
                        offsetof(struct recv_pdesc, desclist[ether->cur_rx]);

                if (val == entry)
                        break;

		status = rxbd->sl;
		length = status & 0xFFFF;

		if (status & RXDS_RXGD) {
			data = ether->rdesc->recv_buf[ether->cur_rx];
			skb = dev_alloc_skb(length+2);
			if (!skb) {
				dev_err(&pdev->dev, "get skb buffer error\n");
				ether->stats.rx_dropped++;
				return;
			}

			skb_reserve(skb, 2);
			skb_put(skb, length);
			skb_copy_to_linear_data(skb, data, length);
			skb->protocol = eth_type_trans(skb, dev);
			ether->stats.rx_packets++;
			ether->stats.rx_bytes += length;
			netif_rx(skb);
		} else {
			ether->stats.rx_errors++;

			if (status & RXDS_RP) {
				dev_err(&pdev->dev, "rx runt err\n");
				ether->stats.rx_length_errors++;
			} else if (status & RXDS_CRCE) {
				dev_err(&pdev->dev, "rx crc err\n");
				ether->stats.rx_crc_errors++;
			} else if (status & RXDS_ALIE) {
				dev_err(&pdev->dev, "rx aligment err\n");
				ether->stats.rx_frame_errors++;
			} else if (status & RXDS_PTLE) {
				dev_err(&pdev->dev, "rx longer err\n");
				ether->stats.rx_over_errors++;
			}
		}

		rxbd->sl = RX_OWEN_DMA;
		rxbd->reserved = 0x0;

		if (++ether->cur_rx >= RX_DESC_SIZE)
			ether->cur_rx = 0;

		rxbd = &ether->rdesc->desclist[ether->cur_rx];

	} while (1);
}

static irqreturn_t nuc970_rx_interrupt(int irq, void *dev_id)
{
	struct net_device *dev;

	unsigned int status;
	dev = dev_id;

printk("[rx]\n");
	nuc970_get_and_clear_int(dev, &status, 0xFFFF);

	if (status & MISTA_RXBERR) {
		struct nuc970_ether  *ether = netdev_priv(dev);
		struct platform_device *pdev = ether->pdev;
		dev_err(&pdev->dev, "emc rx bus error\n");
		nuc970_reset_mac(dev);
	}

	netdev_rx(dev);
	nuc970_trigger_rx(dev);
	return IRQ_HANDLED;
}


static int nuc970_ether_open(struct net_device *dev)
{
	struct nuc970_ether *ether;
	struct platform_device *pdev;

	ether = netdev_priv(dev);
	pdev = ether->pdev;

	//clk_enable(ether->rmiiclk);
	//clk_enable(ether->clk);
	
	nuc970_reset_mac(dev);
	nuc970_set_fifo_threshold(dev);
	nuc970_set_curdest(dev);
	nuc970_enable_cam(dev);
	nuc970_enable_cam_command(dev);
	nuc970_enable_mac_interrupt(dev);
	nuc970_set_global_maccmd(dev);
	nuc970_enable_rx(dev, 1);

	ether->rx_packets = 0x0;
	ether->rx_bytes = 0x0;

	if (request_irq(ether->txirq, nuc970_tx_interrupt,
						0x0, pdev->name, dev)) {
		dev_err(&pdev->dev, "register irq tx failed\n");
		return -EAGAIN;
	}

	if (request_irq(ether->rxirq, nuc970_rx_interrupt,
						0x0, pdev->name, dev)) {
		dev_err(&pdev->dev, "register irq rx failed\n");
		free_irq(ether->txirq, dev);
		return -EAGAIN;
	}

	mod_timer(&ether->check_timer, jiffies + msecs_to_jiffies(1000));
	netif_start_queue(dev);
	nuc970_trigger_rx(dev);

	dev_info(&pdev->dev, "%s is OPENED\n", dev->name);

	return 0;
}

static void nuc970_ether_set_multicast_list(struct net_device *dev)
{
	struct nuc970_ether *ether;
	unsigned int rx_mode;

	ether = netdev_priv(dev);

	if (dev->flags & IFF_PROMISC)
		rx_mode = CAMCMR_AUP | CAMCMR_AMP | CAMCMR_ABP | CAMCMR_ECMP;
	else if ((dev->flags & IFF_ALLMULTI) || !netdev_mc_empty(dev))
		rx_mode = CAMCMR_AMP | CAMCMR_ABP | CAMCMR_ECMP;
	else
		rx_mode = CAMCMR_ECMP | CAMCMR_ABP;
	__raw_writel(rx_mode,  REG_CAMCMR);
}

static int nuc970_ether_ioctl(struct net_device *dev,
						struct ifreq *ifr, int cmd)
{
	struct nuc970_ether *ether = netdev_priv(dev);
	struct mii_ioctl_data *data = if_mii(ifr);

	return generic_mii_ioctl(&ether->mii, data, cmd, NULL);
}

static void nuc970_get_drvinfo(struct net_device *dev,
					struct ethtool_drvinfo *info)
{
	strcpy(info->driver, DRV_MODULE_NAME);
	strcpy(info->version, DRV_MODULE_VERSION);
}

static int nuc970_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct nuc970_ether *ether = netdev_priv(dev);
	return mii_ethtool_gset(&ether->mii, cmd);
}

static int nuc970_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct nuc970_ether *ether = netdev_priv(dev);
	return mii_ethtool_sset(&ether->mii, cmd);
}

static int nuc970_nway_reset(struct net_device *dev)
{
	struct nuc970_ether *ether = netdev_priv(dev);
	return mii_nway_restart(&ether->mii);
}

static u32 nuc970_get_link(struct net_device *dev)
{
	struct nuc970_ether *ether = netdev_priv(dev);
	return mii_link_ok(&ether->mii);
}

static const struct ethtool_ops nuc970_ether_ethtool_ops = {
	.get_settings	= nuc970_get_settings,
	.set_settings	= nuc970_set_settings,
	.get_drvinfo	= nuc970_get_drvinfo,
	.nway_reset	= nuc970_nway_reset,
	.get_link	= nuc970_get_link,
};

static const struct net_device_ops nuc970_ether_netdev_ops = {
	.ndo_open		= nuc970_ether_open,
	.ndo_stop		= nuc970_ether_close,
	.ndo_start_xmit		= nuc970_ether_start_xmit,
	.ndo_get_stats		= nuc970_ether_stats,
	.ndo_set_rx_mode	= nuc970_ether_set_multicast_list,
	.ndo_set_mac_address	= nuc970_set_mac_address,
	.ndo_do_ioctl		= nuc970_ether_ioctl,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_change_mtu		= eth_change_mtu,
};

static void __init get_mac_address(struct net_device *dev)
{
	struct nuc970_ether *ether = netdev_priv(dev);
	struct platform_device *pdev;

	pdev = ether->pdev;

	if (is_valid_ether_addr(nuc970_mac1))
		memcpy(dev->dev_addr, &nuc970_mac1[0], 0x06);
	else
		dev_err(&pdev->dev, "invalid mac address\n");
}

static int nuc970_ether_setup(struct net_device *dev)
{
	struct nuc970_ether *ether = netdev_priv(dev);

	ether_setup(dev);
	dev->netdev_ops = &nuc970_ether_netdev_ops;
	dev->ethtool_ops = &nuc970_ether_ethtool_ops;

	dev->tx_queue_len = 16;
	dev->dma = 0x0;
	dev->watchdog_timeo = TX_TIMEOUT;

	get_mac_address(dev);

	ether->cur_tx = 0x0;
	ether->cur_rx = 0x0;
	ether->finish_tx = 0x0;
	ether->linkflag = 0x0;
	ether->mii.phy_id = 0x01;
	ether->mii.phy_id_mask = 0x1f;
	ether->mii.reg_num_mask = 0x1f;
	ether->mii.dev = dev;
	ether->mii.mdio_read = nuc970_mdio_read;
	ether->mii.mdio_write = nuc970_mdio_write;
	

	setup_timer(&ether->check_timer, nuc970_check_link,
						(unsigned long)dev);

	return 0;
}

static int nuc970_ether_probe(struct platform_device *pdev)
{
	struct nuc970_ether *ether;
	struct net_device *dev;
	int error;

	dev = alloc_etherdev(sizeof(struct nuc970_ether));
	if (!dev)
		return -ENOMEM;

	ether = netdev_priv(dev);

	ether->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (ether->res == NULL) {
		dev_err(&pdev->dev, "failed to get I/O memory\n");
		error = -ENXIO;
		goto failed_free;
	}

	ether->txirq = platform_get_irq(pdev, 0);
	if (ether->txirq < 0) {
		dev_err(&pdev->dev, "failed to get ether tx irq\n");
		error = -ENXIO;
		goto failed_free;
	}
	printk("MAC1 ether->txirq is %d\n", ether->txirq);

	ether->rxirq = platform_get_irq(pdev, 1);
	if (ether->rxirq < 0) {
		dev_err(&pdev->dev, "failed to get ether rx irq\n");
		error = -ENXIO;
		goto failed_free_txirq;
	}
	printk("MAC1 ether->rxirq is %d\n", ether->txirq);

	platform_set_drvdata(pdev, dev);

//	__raw_writel((__raw_readl(REG_CLK_HCLKEN) | (1 << 17)),  REG_CLK_HCLKEN);


//	__raw_writel(0x11111100, REG_MFP_GPE_L);	// PE 2 ~ 11
//	__raw_writel(0x1111, REG_MFP_GPE_H);   

//	__raw_writel((__raw_readl(REG_CLK_DIV8) | 0x80),  REG_CLK_DIV8);
//	__raw_writel((__raw_readl(REG_MCMDR) | 0x00800000),  REG_MCMDR);

	ether->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(ether->clk)) {
		dev_err(&pdev->dev, "failed to get ether clock\n");
		error = PTR_ERR(ether->clk);
		goto failed_free_rxirq;
	}

	ether->rmiiclk = clk_get(&pdev->dev, "RMII");
	if (IS_ERR(ether->rmiiclk)) {
		dev_err(&pdev->dev, "failed to get ether clock\n");
		error = PTR_ERR(ether->rmiiclk);
		goto failed_put_clk;
	}

	ether->pdev = pdev;

	nuc970_ether_setup(dev);

	error = register_netdev(dev);
	if (error != 0) {
		dev_err(&pdev->dev, "Regiter EMC nuc970 FAILED\n");
		error = -ENODEV;
		goto failed_put_rmiiclk;
	}

	return 0;
failed_put_rmiiclk:
	clk_put(ether->rmiiclk);
failed_put_clk:
	clk_put(ether->clk);
failed_free_rxirq:
	free_irq(ether->rxirq, pdev);
	platform_set_drvdata(pdev, NULL);
failed_free_txirq:
	free_irq(ether->txirq, pdev);
failed_free:
	free_netdev(dev);
	return error;
}

static int nuc970_ether_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct nuc970_ether *ether = netdev_priv(dev);

	unregister_netdev(dev);

	clk_put(ether->rmiiclk);
	clk_put(ether->clk);

	free_irq(ether->txirq, dev);
	free_irq(ether->rxirq, dev);

	del_timer_sync(&ether->check_timer);
	platform_set_drvdata(pdev, NULL);

	free_netdev(dev);
	return 0;
}

static struct platform_driver nuc970_ether_driver = {
	.probe		= nuc970_ether_probe,
	.remove		= nuc970_ether_remove,
	.driver		= {
		.name	= "nuc970-emac1",
		.owner	= THIS_MODULE,
	},
};

static int __init nuc970_ether_init(void)
{

	return platform_driver_register(&nuc970_ether_driver);
}

static void __exit nuc970_ether_exit(void)
{
	platform_driver_unregister(&nuc970_ether_driver);
}

module_init(nuc970_ether_init);
module_exit(nuc970_ether_exit);

MODULE_AUTHOR("Nuvoton Technology Corp.");
MODULE_DESCRIPTION("NUC970 MAC1 driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nuc970-emac1");

