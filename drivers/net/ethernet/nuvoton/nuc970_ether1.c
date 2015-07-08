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
#include <linux/phy.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/gfp.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/ctype.h>

#include <mach/map.h>
#include <mach/regs-clock.h>

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
//#define MCMDR_ENMDC		(0x01 << 19)
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
#define MDCON			(0x01 << 19)
//#define PHYAD			(0x01 << 8)
#define PHYWR			(0x01 << 16)
#define PHYBUSY			(0x01 << 17)
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
#define ENRDU			(0x01 << 10)
#define ENRXBERR		(0x01 << 11)
#define ENTXINTR		(0x01 << 16)
#define ENTXCP			(0x01 << 18)
#define ENTXABT			(0x01 << 21)
#define ENTXBERR		(0x01 << 24)
//#define ENMDC			(0x01 << 19)
#define PHYBUSY			(0x01 << 17)
//#define MDCCR_VAL		0xa00000

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
#define TX_TIMEOUT	50
#define DELAY		1000
#define CAM0		0x0

#define MII_TIMEOUT	100

#define ETH_TRIGGER_RX	do{__raw_writel(ENSTART, REG_RSDR);}while(0)
#define ETH_TRIGGER_TX	do{__raw_writel(ENSTART, REG_TSDR);}while(0)
#define ETH_ENABLE_TX	do{__raw_writel(__raw_readl( REG_MCMDR) | MCMDR_TXON, REG_MCMDR);}while(0)
#define ETH_ENABLE_RX	do{__raw_writel(__raw_readl( REG_MCMDR) | MCMDR_RXON, REG_MCMDR);}while(0)
#define ETH_DISABLE_TX	do{__raw_writel(__raw_readl( REG_MCMDR) & ~MCMDR_TXON, REG_MCMDR);}while(0)
#define ETH_DISABLE_RX	do{__raw_writel(__raw_readl( REG_MCMDR) & ~MCMDR_RXON, REG_MCMDR);}while(0)

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

static struct sk_buff *rx_skb[RX_DESC_SIZE];
static struct sk_buff *tx_skb[TX_DESC_SIZE];

struct  nuc970_ether {
	spinlock_t lock;
	struct nuc970_rxbd *rdesc;
	struct nuc970_txbd *tdesc;
	dma_addr_t rdesc_phys;
	dma_addr_t tdesc_phys;
	struct net_device_stats stats;
	struct platform_device *pdev;
	struct net_device *ndev;
	struct resource *res;
	//struct sk_buff *skb;
	struct clk *clk;
	struct clk *eclk;
	unsigned int msg_enable;
	struct mii_bus *mii_bus;
	struct phy_device *phy_dev;
	struct napi_struct napi;
	int rxirq;
	int txirq;
	unsigned int cur_tx;
	unsigned int cur_rx;
	unsigned int finish_tx;
	//unsigned int rx_packets;
	//unsigned int rx_bytes;
	unsigned int start_tx_ptr;
	unsigned int start_rx_ptr;
	int link;
	int speed;
	int duplex;
};


static __init int setup_macaddr(char *str)
{
	u8 mac[6] = {0, 0, 0, 0, 0, 0};
	char *c = str;
	int i, j;

	if (!str)
		goto err;

	for(i = 0; i < 6; i++) {
		for(j = 0; j < 2; j++) {
			mac[i] <<= 4;
			if(isdigit(*c))
				mac[i] += *c - '0';
			else if(isxdigit(*c))
				mac[i] += toupper(*c) - 'A' + 10;
			else {
				goto err;
			}
			c++;
		}

		if(i != 5)
			if(*c != ':') {
				goto err;
			}

		c++;
	}

	// all good
	for(i = 0; i < 6; i++) {
		nuc970_mac1[i] = mac[i];

	}
	return 0;

err:
	return -EINVAL;
}
early_param("ethaddr1", setup_macaddr);

static void adjust_link(struct net_device *dev)
{
	struct nuc970_ether *ether = netdev_priv(dev);
	struct phy_device *phydev = ether->phy_dev;
	unsigned int val;
	bool status_change = false;
	unsigned long flags;

	// clear GPIO interrupt status whihc indicates PHY statu change?

	spin_lock_irqsave(&ether->lock, flags);

	if (phydev->link) {
		if ((ether->speed != phydev->speed) ||
		    (ether->duplex != phydev->duplex)) {
			ether->speed = phydev->speed;
			ether->duplex = phydev->duplex;
			status_change = true;
		}
	} else {
		ether->speed = 0;
		ether->duplex = -1;
	}

	if (phydev->link != ether->link) {

		ether->link = phydev->link;

		status_change = true;
	}

	spin_unlock_irqrestore(&ether->lock, flags);

	if (status_change) {

		val = __raw_readl( REG_MCMDR);

		if (ether->speed == 100) {
			val |= MCMDR_OPMOD;
		} else {
			val &= ~MCMDR_OPMOD;
		}

		if(ether->duplex == DUPLEX_FULL) {
			val |= MCMDR_FDUP;
		} else {
			val &= ~MCMDR_FDUP;
		}

		__raw_writel(val,  REG_MCMDR);
	}
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


static struct sk_buff * get_new_skb(struct net_device *dev, u32 i) {
	struct nuc970_ether *ether = netdev_priv(dev);
	struct sk_buff *skb = dev_alloc_skb(1518 + 2);

	if (skb == NULL)
		return NULL;

	//skb_reserve(skb, 2);    // reserve 2 bytes to align IP header
	skb->dev = dev;

	(ether->rdesc + i)->buffer = dma_map_single(&dev->dev, skb->data,
							1518, DMA_FROM_DEVICE);
	rx_skb[i] = skb;

	return skb;
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

	ether->tdesc = (struct nuc970_txbd *)
			dma_alloc_coherent(&pdev->dev, sizeof(struct nuc970_txbd) * TX_DESC_SIZE,
						&ether->tdesc_phys, GFP_KERNEL);

	if (!ether->tdesc) {
		dev_err(&pdev->dev, "Failed to allocate memory for tx desc\n");
		return -ENOMEM;
	}

	ether->rdesc = (struct nuc970_rxbd *)
			dma_alloc_coherent(&pdev->dev, sizeof(struct nuc970_rxbd) * RX_DESC_SIZE,
						&ether->rdesc_phys, GFP_KERNEL);

	if (!ether->rdesc) {
		dev_err(&pdev->dev, "Failed to allocate memory for rx desc\n");
		dma_free_coherent(&pdev->dev, sizeof(struct nuc970_txbd) * TX_DESC_SIZE,
						ether->tdesc, ether->tdesc_phys);
		return -ENOMEM;
	}

	for (i = 0; i < TX_DESC_SIZE; i++) {
		unsigned int offset;

		tdesc = (ether->tdesc + i);

		if (i == TX_DESC_SIZE - 1)
			offset = 0;
		else
			offset = sizeof(struct nuc970_txbd) * (i + 1);

		tdesc->next = ether->tdesc_phys + offset;
		tdesc->buffer = (unsigned int)NULL;
		tdesc->sl = 0;
		if(i % 4 == 0)	// Trigger TX interrupt every 4 packets
			tdesc->mode = PADDINGMODE | CRCMODE | MACTXINTEN;
		else
			tdesc->mode = PADDINGMODE | CRCMODE;
	}

	ether->start_tx_ptr = ether->tdesc_phys;

	for (i = 0; i < RX_DESC_SIZE; i++) {
		unsigned int offset;

		rdesc = (ether->rdesc + i);

		if (i == RX_DESC_SIZE - 1)
			offset = 0;
		else
			offset = sizeof(struct nuc970_rxbd) * (i + 1);

		rdesc->next = ether->rdesc_phys + offset;
		rdesc->sl = RX_OWEN_DMA;
		if(get_new_skb(dev, i) == NULL) {
			dma_free_coherent(&pdev->dev, sizeof(struct nuc970_txbd) * TX_DESC_SIZE,
						ether->tdesc, ether->tdesc_phys);
			dma_free_coherent(&pdev->dev, sizeof(struct nuc970_rxbd) * RX_DESC_SIZE,
						ether->rdesc, ether->rdesc_phys);

			for(; i != 0; i--) {
				dma_unmap_single(&dev->dev, (dma_addr_t)((ether->rdesc + i)->buffer),
							1518, DMA_FROM_DEVICE);
				dev_kfree_skb_any(rx_skb[i]);
			}
			return -ENOMEM;
		}
	}

	ether->start_rx_ptr = ether->rdesc_phys;

	return 0;
}

// This API must call with Tx/Rx stopped
static void nuc970_free_desc(struct net_device *dev)
{
	struct sk_buff *skb;
	u32 i;
	struct nuc970_ether *ether = netdev_priv(dev);
	struct platform_device *pdev = ether->pdev;

	for (i = 0; i < TX_DESC_SIZE; i++) {
		skb = tx_skb[i];
		if(skb != NULL) {
			dma_unmap_single(&dev->dev, (dma_addr_t)((ether->tdesc + i)->buffer), skb->len, DMA_TO_DEVICE);
			dev_kfree_skb_any(skb);
		}
	}

	for (i = 0; i < RX_DESC_SIZE; i++) {
		skb = rx_skb[i];
		if(skb != NULL) {
			dma_unmap_single(&dev->dev, (dma_addr_t)((ether->rdesc + i)->buffer), 1518, DMA_FROM_DEVICE);
			dev_kfree_skb_any(skb);
		}
	}

	dma_free_coherent(&pdev->dev, sizeof(struct nuc970_txbd) * TX_DESC_SIZE,
				ether->tdesc, ether->tdesc_phys);
	dma_free_coherent(&pdev->dev, sizeof(struct nuc970_rxbd) * RX_DESC_SIZE,
				ether->rdesc, ether->rdesc_phys);

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


static void nuc970_enable_mac_interrupt(struct net_device *dev)
{
	unsigned int val;

	val = ENTXINTR | ENRXINTR | ENRXGD | ENTXCP | ENRDU;
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
	val |= MCMDR_SPCRC | /*MCMDR_ENMDC |*/ MCMDR_ACP /*| ENMDC*/;
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


static void nuc970_set_curdest(struct net_device *dev)
{
	struct nuc970_ether *ether = netdev_priv(dev);

	__raw_writel(ether->start_rx_ptr,  REG_RXDLSA);
	__raw_writel(ether->start_tx_ptr,  REG_TXDLSA);
}

static void nuc970_reset_mac(struct net_device *dev, int need_free)
{
	struct nuc970_ether *ether = netdev_priv(dev);

	ETH_DISABLE_TX;
	ETH_DISABLE_RX;;

	nuc970_return_default_idle(dev);
	nuc970_set_fifo_threshold(dev);

	if (!netif_queue_stopped(dev))
		netif_stop_queue(dev);

	if(need_free)
		nuc970_free_desc(dev);
	nuc970_init_desc(dev);

	//dev->trans_start = jiffies; /* prevent tx timeout */
	ether->cur_tx = 0x0;
	ether->finish_tx = 0x0;
	ether->cur_rx = 0x0;

	nuc970_set_curdest(dev);
	nuc970_enable_cam(dev);
	nuc970_enable_cam_command(dev);
	nuc970_enable_mac_interrupt(dev);
	ETH_ENABLE_TX;
	ETH_ENABLE_RX;

	ETH_TRIGGER_RX;

	dev->trans_start = jiffies; /* prevent tx timeout */

	if (netif_queue_stopped(dev))
		netif_wake_queue(dev);
}

static int nuc970_mdio_write(struct mii_bus *bus, int phy_id, int regnum,
		u16 value)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(MII_TIMEOUT * 100);

	__raw_writel(value,  REG_MIID);
	__raw_writel((phy_id << 0x08) | regnum | PHYBUSY | MDCON | PHYWR,  REG_MIIDA);


	/* Wait for completion */
	while (__raw_readl( REG_MIIDA) & PHYBUSY) {
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
		cpu_relax();
	}

	return 0;

}

static int nuc970_mdio_read(struct mii_bus *bus, int phy_id, int regnum)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(MII_TIMEOUT * 100);


	__raw_writel((phy_id << 0x08) | regnum | PHYBUSY | MDCON,  REG_MIIDA);

	/* Wait for completion */
	while (__raw_readl( REG_MIIDA) & PHYBUSY) {
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
		cpu_relax();
	}

	return __raw_readl(REG_MIID);
}

static int nuc970_mdio_reset(struct mii_bus *bus)
{

	// reser ENAC engine??
	return 0;
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

	netif_stop_queue(dev);
	napi_disable(&ether->napi);
	free_irq(ether->txirq, dev);
	free_irq(ether->rxirq, dev);

	nuc970_free_desc(dev);

	if (ether->phy_dev)
		phy_stop(ether->phy_dev);

	return 0;
}

static struct net_device_stats *nuc970_ether_stats(struct net_device *dev)
{
	struct nuc970_ether *ether;

	ether = netdev_priv(dev);

	return &ether->stats;
}


static int nuc970_ether_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct nuc970_ether *ether = netdev_priv(dev);
	struct nuc970_txbd *txbd;
	unsigned int cur_entry, entry;
	struct sk_buff *s;

	cur_entry = __raw_readl( REG_CTXDSA);

	entry = ether->tdesc_phys + sizeof(struct nuc970_txbd) * (ether->finish_tx);

	while (entry != cur_entry) {
		txbd = (ether->tdesc + ether->finish_tx);
		s = tx_skb[ether->finish_tx];
		dma_unmap_single(&dev->dev, txbd->buffer, s->len, DMA_TO_DEVICE);
		dev_kfree_skb(s);
		tx_skb[ether->finish_tx] = NULL;

		if (++ether->finish_tx >= TX_DESC_SIZE)
			ether->finish_tx = 0;

		if (txbd->sl & TXDS_TXCP) {
			ether->stats.tx_packets++;
			ether->stats.tx_bytes += (txbd->sl & 0xFFFF);
		} else {
			ether->stats.tx_errors++;
		}

		txbd->sl = 0x0;
		txbd->buffer = (unsigned int)NULL;

		entry = ether->tdesc_phys + sizeof(struct nuc970_txbd) * (ether->finish_tx);
	}

	txbd = ether->tdesc + ether->cur_tx;
	txbd->buffer = dma_map_single(&dev->dev, skb->data,
					skb->len, DMA_TO_DEVICE);

	tx_skb[ether->cur_tx]  = skb;
	txbd->sl = skb->len > 1514 ? 1514 : skb->len;
	wmb();	// This is dummy function for ARM9
	txbd->mode |= TX_OWEN_DMA;
	wmb();	// This is dummy function for ARM9

	ETH_TRIGGER_TX;

	if (++ether->cur_tx >= TX_DESC_SIZE)
		ether->cur_tx = 0;

	txbd = (ether->tdesc + ether->cur_tx);

	if (txbd->mode & TX_OWEN_DMA) {
		netif_stop_queue(dev);
	}

	return(0);
}

static irqreturn_t nuc970_tx_interrupt(int irq, void *dev_id)
{
	struct nuc970_ether *ether;
	struct platform_device *pdev;
	struct net_device *dev;
	unsigned int status;

	dev = dev_id;
	ether = netdev_priv(dev);
	pdev = ether->pdev;

	nuc970_get_and_clear_int(dev, &status, 0xFFFF0000);

	if (status & MISTA_EXDEF) {
		dev_err(&pdev->dev, "emc defer exceed interrupt\n");
	} else if (status & MISTA_TXBERR) {
		dev_err(&pdev->dev, "emc bus error interrupt\n");
		nuc970_reset_mac(dev, 1);
	}

	if (netif_queue_stopped(dev)) {
		netif_wake_queue(dev);
	}

	return IRQ_HANDLED;
}

static int nuc970_poll(struct napi_struct *napi, int budget)
{
	struct nuc970_ether *ether = container_of(napi, struct nuc970_ether, napi);
	struct nuc970_rxbd *rxbd;
	struct net_device *dev = ether->ndev;
	struct sk_buff *skb, *s;
	unsigned int length, status;
	int rx_cnt = 0;
	int complete = 0;

	rxbd = (ether->rdesc + ether->cur_rx);

	while(rx_cnt < budget) {

		if((rxbd->sl & RX_OWEN_DMA) == RX_OWEN_DMA) {
			complete = 1;
			break;
		}

		s = rx_skb[ether->cur_rx];
		status = rxbd->sl;
		length = status & 0xFFFF;

		if (likely(status & RXDS_RXGD)) {

			skb = dev_alloc_skb(1518 + 2);

			if (!skb) {
				struct platform_device *pdev = ether->pdev;
				dev_err(&pdev->dev, "get skb buffer error\n");
				ether->stats.rx_dropped++;
				goto rx_out;
			}
			dma_unmap_single(&dev->dev, (dma_addr_t)rxbd->buffer, 1518, DMA_FROM_DEVICE);

			skb_put(s, length);
			s->protocol = eth_type_trans(s, dev);
			netif_receive_skb(s);
			ether->stats.rx_packets++;
			ether->stats.rx_bytes += length;

			//skb_reserve(skb, 2);
			skb->dev = dev;

			rxbd->buffer = dma_map_single(&dev->dev, skb->data,
							1518, DMA_FROM_DEVICE);

			rx_skb[ether->cur_rx] = skb;
			rx_cnt++;

		} else {
			ether->stats.rx_errors++;

			if (status & RXDS_RP) {
				ether->stats.rx_length_errors++;
			} else if (status & RXDS_CRCE) {
				ether->stats.rx_crc_errors++;
			} else if (status & RXDS_ALIE) {
				ether->stats.rx_frame_errors++;
			} else if (status & RXDS_PTLE) {
				ether->stats.rx_over_errors++;
			}
		}

		wmb();	// This is dummy function for ARM9
		rxbd->sl = RX_OWEN_DMA;

		if (++ether->cur_rx >= RX_DESC_SIZE)
			ether->cur_rx = 0;

		rxbd = (ether->rdesc + ether->cur_rx);

	}

	if(complete) {
		__napi_complete(napi);
		__raw_writel(__raw_readl(REG_MIEN) | ENRXINTR,  REG_MIEN);
	}

rx_out:

	ETH_TRIGGER_RX;
	return(rx_cnt);
}

static irqreturn_t nuc970_rx_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *)dev_id;
	struct nuc970_ether *ether = netdev_priv(dev);
	unsigned int status;

	nuc970_get_and_clear_int(dev, &status, 0xFFFF);

	if (unlikely(status & MISTA_RXBERR)) {
		struct platform_device *pdev = ether->pdev;

		dev_err(&pdev->dev, "emc rx bus error\n");
		nuc970_reset_mac(dev, 1);

	} else {
		__raw_writel(__raw_readl(REG_MIEN) & ~ENRXINTR,  REG_MIEN);
		napi_schedule(&ether->napi);
	}
	return IRQ_HANDLED;
}


static int nuc970_ether_open(struct net_device *dev)
{
	struct nuc970_ether *ether;
	struct platform_device *pdev;

	ether = netdev_priv(dev);
	pdev = ether->pdev;

	nuc970_reset_mac(dev, 0);
	nuc970_set_fifo_threshold(dev);
	nuc970_set_curdest(dev);
	nuc970_enable_cam(dev);
	nuc970_enable_cam_command(dev);
	nuc970_enable_mac_interrupt(dev);
	nuc970_set_global_maccmd(dev);
	ETH_ENABLE_RX;


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

	phy_start(ether->phy_dev);
	netif_start_queue(dev);
	napi_enable(&ether->napi);

	ETH_TRIGGER_RX;

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
	struct phy_device *phydev = ether->phy_dev;

	if (!netif_running(dev))
		return -EINVAL;

	if (!phydev)
		return -ENODEV;;

	return phy_mii_ioctl(phydev, ifr, cmd);
}

static void nuc970_get_drvinfo(struct net_device *dev,
					struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, DRV_MODULE_NAME, sizeof(info->driver));
	strlcpy(info->version, DRV_MODULE_VERSION, sizeof(info->version));
	strlcpy(info->fw_version, "N/A", sizeof(info->fw_version));
	strlcpy(info->bus_info, "N/A", sizeof(info->bus_info));
}

static int nuc970_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct nuc970_ether *ether = netdev_priv(dev);
	struct phy_device *phydev = ether->phy_dev;

	if (NULL == phydev)
		return -ENODEV;

	return phy_ethtool_gset(phydev, cmd);
}

static int nuc970_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct nuc970_ether *ether = netdev_priv(dev);
	struct phy_device *phydev = ether->phy_dev;

	if (NULL == phydev)
		return -ENODEV;

	return phy_ethtool_sset(phydev, cmd);
}

static u32 nuc970_get_msglevel(struct net_device *dev)
{
	struct nuc970_ether *ether = netdev_priv(dev);

	return ether->msg_enable;
}

static void nuc970_set_msglevel(struct net_device *dev, u32 level)
{
	struct nuc970_ether *ether = netdev_priv(dev);

	ether->msg_enable = level;
}

static const struct ethtool_ops nuc970_ether_ethtool_ops = {
	.get_settings	= nuc970_get_settings,
	.set_settings	= nuc970_set_settings,
	.get_drvinfo	= nuc970_get_drvinfo,
	.get_msglevel	= nuc970_get_msglevel,
	.set_msglevel	= nuc970_set_msglevel,
	.get_link 	= ethtool_op_get_link,
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


static int nuc970_mii_setup(struct net_device *dev)
{
	struct nuc970_ether *ether = netdev_priv(dev);
	struct platform_device *pdev;
	struct phy_device *phydev;
	int i, err = 0;

	pdev = ether->pdev;

	ether->mii_bus = mdiobus_alloc();
	if (!ether->mii_bus) {
		err = -ENOMEM;
		dev_err(&pdev->dev, "mdiobus_alloc() failed\n");
		goto out0;
	}

	ether->mii_bus->name = "nuc970_rmii1";
	ether->mii_bus->read = &nuc970_mdio_read;
	ether->mii_bus->write = &nuc970_mdio_write;
	ether->mii_bus->reset = &nuc970_mdio_reset;
	snprintf(ether->mii_bus->id, MII_BUS_ID_SIZE, "%s-%x",
		 ether->pdev->name, ether->pdev->id);
	ether->mii_bus->priv = ether;
	ether->mii_bus->parent = &ether->pdev->dev;

	ether->mii_bus->irq = kmalloc(sizeof(int) * PHY_MAX_ADDR, GFP_KERNEL);
	if (!ether->mii_bus->irq) {
		err = -ENOMEM;
		dev_err(&pdev->dev, "kmalloc() failed\n");
		goto out1;

	}

	for (i = 0; i < PHY_MAX_ADDR; i++)
		ether->mii_bus->irq[i] = PHY_POLL;
	//ether->mii_bus->irq[1] = ??   write me after the irq number is known

	platform_set_drvdata(ether->pdev, ether->mii_bus);

	if (mdiobus_register(ether->mii_bus)) {
		dev_err(&pdev->dev, "mdiobus_register() failed\n");
		goto out2;
	}

	phydev = phy_find_first(ether->mii_bus);
	if(phydev == NULL) {
		dev_err(&pdev->dev, "phy_find_first() failed\n");
		goto out3;
	}

	phydev = phy_connect(dev, dev_name(&phydev->dev),
			     &adjust_link,
			     PHY_INTERFACE_MODE_RMII);

	if(IS_ERR(phydev)) {
		err = PTR_ERR(phydev);
		dev_err(&pdev->dev, "phy_connect() failed\n");
		goto out3;
	}

	phydev->supported &= PHY_BASIC_FEATURES;
	phydev->advertising = phydev->supported;
	ether->phy_dev = phydev;

	return 0;

out3:
	mdiobus_unregister(ether->mii_bus);
out2:
	kfree(ether->mii_bus->irq);
out1:
	mdiobus_free(ether->mii_bus);
out0:

	return err;
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
		goto err0;
	}

	ether->txirq = platform_get_irq(pdev, 0);
	if (ether->txirq < 0) {
		dev_err(&pdev->dev, "failed to get ether tx irq\n");
		error = -ENXIO;
		goto err0;
	}

	ether->rxirq = platform_get_irq(pdev, 1);
	if (ether->rxirq < 0) {
		dev_err(&pdev->dev, "failed to get ether rx irq\n");
		error = -ENXIO;
		goto err0;
	}

	SET_NETDEV_DEV(dev, &pdev->dev);
	platform_set_drvdata(pdev, dev);
	ether->ndev = dev;

	ether->eclk = clk_get(NULL, "emac1_eclk");
	if (IS_ERR(ether->eclk)) {
		dev_err(&pdev->dev, "failed to get emac1_eclk clock\n");
		error = PTR_ERR(ether->eclk);
		goto err1;
	}

	// Set MDC to 1M
	clk_set_rate(ether->eclk, 1000000);

	clk_prepare(ether->eclk);
	clk_enable(ether->eclk);

	ether->clk = clk_get(NULL, "emac1_hclk");
	if (IS_ERR(ether->clk)) {
		dev_err(&pdev->dev, "failed to get emac1_hclk clock\n");
		error = PTR_ERR(ether->clk);
		goto err1;
	}

	clk_prepare(ether->clk);
	clk_enable(ether->clk);

	ether->pdev = pdev;
	ether->msg_enable = NETIF_MSG_LINK;

	dev->netdev_ops = &nuc970_ether_netdev_ops;
	dev->ethtool_ops = &nuc970_ether_ethtool_ops;

	dev->tx_queue_len = 16;
	dev->dma = 0x0;
	dev->watchdog_timeo = TX_TIMEOUT;

	get_mac_address(dev);

	ether->cur_tx = 0x0;
	ether->cur_rx = 0x0;
	ether->finish_tx = 0x0;
	ether->link = 0;
	ether->speed = 100;
	ether->duplex = DUPLEX_FULL;

	netif_napi_add(dev, &ether->napi, nuc970_poll, 16);

	ether_setup(dev);

	if((error = nuc970_mii_setup(dev)) < 0) {
		dev_err(&pdev->dev, "nuc970_mii_setup err\n");
		goto err2;
	}

	error = register_netdev(dev);
	if (error != 0) {
		dev_err(&pdev->dev, "register_netdev() failed\n");
		error = -ENODEV;
		goto err2;
	}

	return 0;

err2:
	clk_disable(ether->clk);
	clk_put(ether->clk);
err1:
	platform_set_drvdata(pdev, NULL);
err0:
	free_netdev(dev);

	return error;
}

static int nuc970_ether_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct nuc970_ether *ether = netdev_priv(dev);

	unregister_netdev(dev);

	clk_disable(ether->clk);
	clk_put(ether->clk);

	clk_disable(ether->eclk);
	clk_put(ether->eclk);

	free_irq(ether->txirq, dev);
	free_irq(ether->rxirq, dev);
	phy_disconnect(ether->phy_dev);

	mdiobus_unregister(ether->mii_bus);
	kfree(ether->mii_bus->irq);
	mdiobus_free(ether->mii_bus);

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

