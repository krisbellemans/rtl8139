/*
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/pci.h>
#include <linux/stddef.h>
#include <linux/etherdevice.h>

MODULE_AUTHOR("Kris Bellemans");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("RTL8139 pci driver");

// lspci -x pci configuration space table
#define RTL_VENDOR_ID	0x10EC
#define RTL_DEVICE_ID	0x8139

#define NUM_TX_DESC	4
#define TX_BUF_SIZE	1536
#define TOTAL_TX_BUF_SIZE   (TX_BUF_SIZE * NUM_TX_DESC)

#define RX_BUF_LEN_IDX	2	/* 0==8K, 1==16K, 2==32K, 3==64K */
#define RX_BUF_LEN	(8192 << RX_BUF_LEN_IDX)
#define RX_BUF_PAD	16	/* Bit 11-12 of RCR: 0x44 */
#define RX_BUF_WRAP_PAD	2048	/* spare padding to handle packet wrap */
#define TOTAL_RX_BUF_SIZE   (RX_BUF_LEN + RX_BUF_PAD + RX_BUF_WRAP_PAD)

#define TSD0		0x10
#define TSAD0		0x20
#define RBSTART		0x30
#define CR		0x37
#define CAPR		0x38
#define IMR		0x3C
#define ISR		0x3E
#define TCR		0x40
#define RCR		0x44
#define MPC		0x4C
#define MULINT		0x5C

#define TxUnderrun	0x4000
#define TxStatOK	0x8000
#define TxAborted	0x40000000

#define RxBufEmpty	0x01
#define CmdReset	0x10
#define CmdTxEnb	0x04
#define CmdRxEnb	0x08

#define RxOK		0x01
#define RxErr		0x02
#define TxOK		0x04
#define TxErr		0x08
#define RxOverFlow	0x10
#define RxUnderrun	0x20
#define RxFIFOOver	0x40
#define CableLen	0x2000
#define TimeOut		0x4000
#define SysErr		0x8000

#define INT_MASK	(RxOK | RxErr | TxOK | TxErr | RxOverFlow | RxUnderrun \
		| RxFIFOOver | CableLen | TimeOut | SysErr)

struct rtl8139_private {
	/* pci device info */
	struct pci_dev *pdev;
	void *mmio_addr;	/* memmapped I/O address */
	unsigned long regs_len;	/* memmapped I/O length */

	/* rx status info */
	unsigned char *rx_ring;
	dma_addr_t rx_ring_dma;
	unsigned int cur_rx;

	/* tx status info */
	unsigned int tx_flag;
	unsigned int cur_tx;
	unsigned int dirty_tx;
	unsigned char *tx_buf[NUM_TX_DESC];
	unsigned char *tx_bufs;
	dma_addr_t tx_bufs_dma;

	/* device statistics */
	struct net_device_stats stats;
};

static struct net_device *rtl8139_dev;

#define ETH_MIN_LEN	    60
static int rtl8139_start_xmit(struct sk_buff *skbuf, struct net_device *dev)
{
	struct rtl8139_private *priv = (struct rtl8139_private *)
	    netdev_priv(dev);
	void *ioaddr = priv->mmio_addr;
	unsigned int entry = priv->cur_tx;
	unsigned int len = skbuf->len;
	printk(KERN_DEBUG "send packet");

	if (len < TX_BUF_SIZE) {
		if (len < ETH_MIN_LEN)
			memset(priv->tx_buf[entry], 0, ETH_MIN_LEN);
		skb_copy_and_csum_dev(skbuf, priv->tx_buf[entry]);
		dev_kfree_skb(skbuf);
	} else {
		dev_kfree_skb(skbuf);
		return 0;
	}
	writel(priv->tx_flag | max(len, (unsigned int)ETH_MIN_LEN) | (1 << 16),
	       ioaddr + TSD0 + (entry * sizeof(uint32_t)));
	entry++;
	priv->cur_tx = entry % NUM_TX_DESC;
	if (priv->cur_tx == priv->dirty_tx) {
		netif_stop_queue(dev);
		printk(KERN_DEBUG "queue stopped");
	}
	return 0;
}

static struct net_device_stats *rtl8139_get_stats(struct net_device *dev)
{
	struct rtl8139_private *priv = (struct rtl8139_private *)
	    netdev_priv(dev);
	return &(priv->stats);
}

static void rtl8139_init_ring(struct net_device *dev)
{
	struct rtl8139_private *priv = (struct rtl8139_private *)
	    netdev_priv(dev);
	int i;

	priv->cur_tx = 0;
	priv->dirty_tx = 0;
	for (i = 0; i < NUM_TX_DESC; i++)
		priv->tx_buf[i] = &priv->tx_bufs[i * TX_BUF_SIZE];
}

static void rtl8139_chip_reset(void *ioaddr)
{
	int i;
	writeb(CmdReset, ioaddr + CR);

	for (i = 1000; i > 0; i--) {
		barrier();
		if ((readb(ioaddr + CR) & CmdReset) == 0)
			break;
		udelay(10);
	}
}

static void rtl8139_hw_start(struct net_device *dev)
{
	struct rtl8139_private *priv = (struct rtl8139_private *)
	    netdev_priv(dev);
	void *ioaddr = priv->mmio_addr;
	unsigned int i;
	rtl8139_chip_reset(ioaddr);
	writeb(CmdTxEnb | CmdRxEnb, ioaddr + CR);	// enable tx and rx

	/* tx config */
	writel(0x000000600, ioaddr + TCR);	// DMA burst size 1024

	/* rx config */
	/* rx buffer length (size of ring buffer): 32K + 16
	 * max dma burst size: 1024
	 * wrap bit: 1
	 * accept broadcast packets
	 * accept multicast packets
	 * accept physical match packets */
	writel(((1 << 12) | (6 << 8) | (1 << 7) | (1 << 3) | (1 << 2) | (1 <<
									 1)),
	       ioaddr + RCR);

	/* set tx descriptor buffer DMA addresses */
	for (i = 0; i < NUM_TX_DESC; i++)
		writel(priv->tx_bufs_dma + (priv->tx_buf[i] - priv->tx_bufs),
		       ioaddr + TSAD0 + (i * 4));

	/* init RBSTART */
	writel(priv->rx_ring_dma, ioaddr + RBSTART);

	/* init missed packet counter */
	writel(0, ioaddr + MPC);

	/* no early-rx interrupts */
	writew((readw(ioaddr + MULINT) & 0xF000), ioaddr + MULINT);

	/* Enable all known interrupts by setting the interrupt mask */
	writew(INT_MASK, ioaddr + IMR);

	/* Notify kernel that device is ready */
	netif_start_queue(dev);
}

static irqreturn_t rtl8139_isr(int irq, void *dev_instance)
{
	printk(KERN_DEBUG "interrupt handler invoked");
	struct net_device *dev = (struct net_device *)dev_instance;
	struct rtl8139_private *priv =
	    (struct rtl8139_private *)netdev_priv(dev);
	void *ioaddr = priv->mmio_addr;
	unsigned short isr = readw(ioaddr + ISR);
	/* clear all interrupts */
	writew(0xFFFF, ioaddr + ISR);

	if ((isr & TxOK) || (isr & TxErr)) {
		printk(KERN_DEBUG "tx interrupt");
		while ((priv->dirty_tx != priv->cur_tx) ||
		       netif_queue_stopped(dev)) {
			unsigned int txstatus = readl(ioaddr + TSD0 +
						      priv->dirty_tx *
						      sizeof(uint32_t));
			if (!(txstatus & (TxStatOK | TxAborted | TxUnderrun)))
				break;

			if (txstatus & TxStatOK) {
				printk(KERN_DEBUG "TOK interrupt");
				priv->stats.tx_bytes += (txstatus & 0x1FFF);
				priv->stats.tx_packets++;
			} else {
				printk(KERN_DEBUG "TxErr interrupt");
				priv->stats.tx_errors++;
			}

			priv->dirty_tx++;
			priv->dirty_tx = priv->dirty_tx % NUM_TX_DESC;

			if ((priv->dirty_tx == priv->cur_tx) &
			    netif_queue_stopped(dev)) {
				printk(KERN_DEBUG "Waking up queue");
				netif_wake_queue(dev);
			}
		}
	}

	if (isr & RxErr) {
		printk(KERN_DEBUG "Rx error interrupt");
		priv->stats.rx_errors++;
	}

	if (isr & RxOK) {
		printk(KERN_ERR "Rx OK interrupt");
		while ((readb(ioaddr + CR) & RxBufEmpty) == 0) {
			printk(KERN_ERR "Rx looping");
			unsigned int rx_status;
			unsigned short rx_size;
			unsigned short pkt_size;
			struct sk_buff *skb;

			if (priv->cur_rx > RX_BUF_LEN)
				priv->cur_rx = priv->cur_rx % RX_BUF_LEN;

			/*
			rx_status = *(unsigned int *)(priv->rx_ring +
						      priv->cur_rx);
		      */
			rx_status = le32_to_cpu (*(__le32*)(priv->rx_ring + priv->cur_rx));
			/* frame size */
			rx_size = rx_status >> 16;
			pkt_size = rx_size - 4;

			skb = dev_alloc_skb(pkt_size + 2);
			if (skb) {
				skb->dev = dev;
				/* reserve space for protocol headers; after
				 * ethernet header, proper alignment of ip
				 * header on 4 byte boundary */
				skb_reserve(skb, NET_IP_ALIGN);

				/*
				   eth_copy_and_sum(skb, priv->rx_ring +
				   priv->cur_rx + 4, pkt_size, 0);
				   memcpy(skb_put(skb, pkt_size), priv->rx_ring +
				   priv->cur_rx + 4, pkt_size);
				 */
				/* updates end of data pointer in skbuf */
				skb_copy_to_linear_data(skb, priv->rx_ring +
						priv->cur_rx + 4, pkt_size);
				skb_put(skb, pkt_size);
				/*
				memcpy(skb->data, priv->rx_ring + priv->cur_rx +
				       4, pkt_size);
			       */
				skb->ip_summed = CHECKSUM_UNNECESSARY;
				skb->protocol = eth_type_trans(skb, dev);
				/* socket buffer handoff to upper layer */
				netif_rx(skb);

				dev->last_rx = jiffies;
				priv->stats.rx_bytes += pkt_size;
				priv->stats.rx_packets++;
			} else {
				printk(KERN_DEBUG "No memory");
				priv->stats.rx_dropped++;
			}

			priv->cur_rx = (priv->cur_rx + rx_size + 4 + 3) & ~3;
			/* Update Current Address of Packet Read Register */
			//writew(priv->cur_rx, ioaddr + CAPR);
			writew(priv->cur_rx - 16, ioaddr + CAPR);
		}
	}

	if (isr & CableLen)
		printk(KERN_DEBUG "cable length change interrupt");
	if (isr & TimeOut)
		printk(KERN_DEBUG "time interrupt");
	if (isr & SysErr)
		printk(KERN_DEBUG "system error interrupt");
	return IRQ_HANDLED;
}

static int rtl8139_open(struct net_device *dev)
{
	int retval;
	struct rtl8139_private *priv =
	    (struct rtl8139_private *)netdev_priv(dev);

	/* register interrupt handler */
	retval =
	    request_irq(dev->irq, rtl8139_isr, IRQF_SHARED, dev->name, dev);
	//retval = request_irq(dev->irq, rtl8139_isr, 0, dev->name, dev);
	if (retval)
		return retval;

	priv->tx_bufs = pci_alloc_consistent(priv->pdev, TOTAL_TX_BUF_SIZE,
					     &priv->tx_bufs_dma);

	priv->rx_ring = pci_alloc_consistent(priv->pdev, TOTAL_RX_BUF_SIZE,
					     &priv->rx_ring_dma);

	if (!priv->tx_bufs || !priv->rx_ring) {
		free_irq(dev->irq, dev);
		if (priv->tx_bufs) {
			pci_free_consistent(priv->pdev, TOTAL_TX_BUF_SIZE,
					    priv->tx_bufs, priv->tx_bufs_dma);
			priv->tx_bufs = NULL;
		}

		if (priv->rx_ring) {
			pci_free_consistent(priv->pdev, TOTAL_RX_BUF_SIZE,
					    priv->rx_ring, priv->rx_ring_dma);
			priv->rx_ring = NULL;
		}
		return -ENOMEM;
	}

	priv->tx_flag = 0;
	rtl8139_init_ring(dev);
	rtl8139_hw_start(dev);

	printk(KERN_DEBUG "ifconfig up");
	return 0;
}

static int rtl8139_stop(struct net_device *dev)
{
	printk(KERN_DEBUG "ifconfig down");
	return 0;
}

static const struct net_device_ops rtl8139_ops = {
	.ndo_open = rtl8139_open,
	.ndo_stop = rtl8139_stop,
	.ndo_start_xmit = rtl8139_start_xmit,
	.ndo_get_stats = rtl8139_get_stats,
};

static struct pci_dev *probe_rtl8139_dev(void)
{
	struct pci_dev *pdev = NULL;
	pdev = pci_get_device(RTL_VENDOR_ID, RTL_DEVICE_ID, NULL);
	if (pdev) {
		if (pci_enable_device(pdev)) {
			printk("Could not enable device\n");
			return NULL;
		} else {
			printk("Device enabled\n");
		}
	} else {
		printk("Device not found\n");
		return pdev;
	}
	return pdev;
}

static int rtl8139_init(struct pci_dev *pdev, struct net_device **net_dev)
{
	struct net_device *dev;
	struct rtl8139_private *priv;

	dev = alloc_etherdev(sizeof(struct rtl8139_private));
	if (!dev) {
		printk(KERN_INFO "Could not allocate etherdev\n");
		return -1;
	}

	priv = (struct rtl8139_private *)netdev_priv(dev);
	priv->pdev = pdev;
	*net_dev = dev;
	return 0;
}

static int __init netdev_init(void)
{
	struct pci_dev *pdev;
	long iobase, ioend, iolen, ioflags;
	void *ioaddr;
	struct rtl8139_private *priv;
	int i;

	pdev = probe_rtl8139_dev();
	if (!pdev)
		return 0;

	if (rtl8139_init(pdev, &rtl8139_dev)) {
		printk(KERN_DEBUG "Could not init dev\n");
		return 0;
	}

	priv = (struct rtl8139_private *)netdev_priv(rtl8139_dev);
	iobase = pci_resource_start(pdev, 1);
	ioend = pci_resource_end(pdev, 1);
	iolen = pci_resource_len(pdev, 1);
	ioflags = pci_resource_flags(pdev, 1);

	if (request_region(iobase, iolen, "RTL8139")) {
		printk(KERN_DEBUG "Request pci region failed\n");
		pci_disable_device(pdev);
		return 0;
	}

	pci_set_master(pdev);
	ioaddr = ioremap(iobase, iolen);
	if (!ioaddr) {
		printk(KERN_DEBUG "Could not memmap pci device\n");
		pci_release_regions(pdev);
		pci_disable_device(pdev);
		return 0;
	}

	rtl8139_dev->base_addr = (long)ioaddr;
	priv->mmio_addr = ioaddr;
	priv->regs_len = iolen;

	for (i = 0; i < 6; i++) {
		rtl8139_dev->dev_addr[i] =
		    readb((const volatile void *)rtl8139_dev->base_addr + i);
		rtl8139_dev->broadcast[i] = 0xFF;
	}
	rtl8139_dev->hard_header_len = 14;

	memcpy(rtl8139_dev->name, "RTL8139", sizeof("RTL8139"));
	rtl8139_dev->irq = pdev->irq;
	rtl8139_dev->netdev_ops = &rtl8139_ops;

	if (register_netdev(rtl8139_dev)) {
		printk(KERN_DEBUG "Error registering netdev\n");
		iounmap(priv->mmio_addr);
		pci_release_regions(pdev);
		pci_disable_device(pdev);
		return 0;
	}

	printk(KERN_DEBUG "Module loaded");
	return 0;
}

static void __exit netdev_exit(void)
{
	struct rtl8139_private *priv;
	priv = (struct rtl8139_private *)netdev_priv(rtl8139_dev);

	iounmap(priv->mmio_addr);
	pci_release_regions(priv->pdev);
	unregister_netdev(rtl8139_dev);
	pci_disable_device(priv->pdev);
	free_netdev(rtl8139_dev);
	printk(KERN_DEBUG "Netdev unloaded\n");
}

module_init(netdev_init)
    module_exit(netdev_exit)
