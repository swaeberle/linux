// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Microchip ENCX24J600 ethernet driver
 *
 * Copyright (C) 2023 A. Eberle GmbH & Co. KG
 *
 * Author: Stephan Wurm <stephan.wurm@a-eberle.de>
 *
 * Based on encx24j600 driver by Jon Ringle <jringle@gridpoint.com>
 */

#include <linux/device.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>

#include <linux/platform_device.h>
#include <linux/of_net.h>

#include "enc624j600_psp.h"

#define DRV_NAME	"enc624j600-psp"
#define DRV_VERSION	"1.0"

#define DEFAULT_MSG_ENABLE (NETIF_MSG_DRV | NETIF_MSG_PROBE | NETIF_MSG_LINK)
static int debug = -1;
module_param(debug, int, 0000);
MODULE_PARM_DESC(debug, "Debug level (0=none,...,16=all)");

/* SRAM memory layout:
 *
 * 0x0000-0x05ff TX buffers  1.5KB  (1*1536) reside in the GP area in SRAM
 * 0x0600-0x5fff RX buffers 22.5KB (15*1536) reside in the RX area in SRAM
 */
#define ENC_TX_BUF_START SRAM_GP_START
#define ENC_RX_BUF_START RXSTART_INIT
#define ENC_RX_BUF_END	 RXEND_INIT
#define ENC_SRAM_SIZE	 SRAM_SIZE

enum {
	RXFILTER_NORMAL,
	RXFILTER_MULTI,
	RXFILTER_PROMISC
};

struct enc624j600_priv {
	struct net_device	 *ndev;
	struct mutex		  lock; /* device access lock */
	struct sk_buff_head	  txq;
	struct sk_buff		 *tx_skb;
	struct task_struct	 *kworker_task;
	struct kthread_worker	  kworker;
	struct kthread_work	  tx_work;
	struct kthread_work	  setrx_work;
	void __iomem		 *base;
	u16			  next_packet;
	bool			  hw_enabled;
	bool			  full_duplex;
	bool			  autoneg;
	u16			  speed;
	int			  rxfilter;
	u32			  msg_enable;
};

/* Forward declarations */
static void enc624j600_hw_tx(struct enc624j600_priv *priv);

static void dump_packet(const char *msg, int len, const char *data)
{
	pr_debug(DRV_NAME ": %s - packet len:%d\n", msg, len);
	print_hex_dump_bytes("pk data: ", DUMP_PREFIX_OFFSET, data, len);
}

static void enc624j600_dump_rsv(struct enc624j600_priv *priv, const char *msg,
				struct rsv *rsv)
{
	struct net_device *dev = priv->ndev;

	netdev_info(dev, "RX packet Len:%d\n", rsv->len);
	netdev_dbg(dev, "%s - NextPk: 0x%04x\n", msg,
		   rsv->next_packet);
	netdev_dbg(dev, "RxOK: %d, DribbleNibble: %d\n",
		   RSV_GETBIT(rsv->rxstat, RSV_RXOK),
		   RSV_GETBIT(rsv->rxstat, RSV_DRIBBLENIBBLE));
	netdev_dbg(dev, "CRCErr:%d, LenChkErr: %d, LenOutOfRange: %d\n",
		   RSV_GETBIT(rsv->rxstat, RSV_CRCERROR),
		   RSV_GETBIT(rsv->rxstat, RSV_LENCHECKERR),
		   RSV_GETBIT(rsv->rxstat, RSV_LENOUTOFRANGE));
	netdev_dbg(dev, "Multicast: %d, Broadcast: %d, LongDropEvent: %d, CarrierEvent: %d\n",
		   RSV_GETBIT(rsv->rxstat, RSV_RXMULTICAST),
		   RSV_GETBIT(rsv->rxstat, RSV_RXBROADCAST),
		   RSV_GETBIT(rsv->rxstat, RSV_RXLONGEVDROPEV),
		   RSV_GETBIT(rsv->rxstat, RSV_CARRIEREV));
	netdev_dbg(dev, "ControlFrame: %d, PauseFrame: %d, UnknownOp: %d, VLanTagFrame: %d\n",
		   RSV_GETBIT(rsv->rxstat, RSV_RXCONTROLFRAME),
		   RSV_GETBIT(rsv->rxstat, RSV_RXPAUSEFRAME),
		   RSV_GETBIT(rsv->rxstat, RSV_RXUNKNOWNOPCODE),
		   RSV_GETBIT(rsv->rxstat, RSV_RXTYPEVLAN));
}

static u16 enc624j600_read_reg(struct enc624j600_priv *priv, u8 reg)
{
	u16 offset = SFR_REG_BASE | reg;
	void __iomem *src = priv->base + offset;
	struct net_device *dev = priv->ndev;
	unsigned int val = 0;

	if (likely(reg < SFR_REG_COUNT))
		val = ioread16(src);
	else
		netif_err(priv, drv, dev, "%s: error %d reading SFR reg %02x\n",
			  __func__, -EINVAL, reg);

	return val;
}

static void enc624j600_write_reg(struct enc624j600_priv *priv, u8 reg, u16 val)
{
	u16 offset = SFR_REG_BASE | reg;
	void __iomem *dst = priv->base + offset;
	struct net_device *dev = priv->ndev;

	if (likely(reg < SFR_REG_COUNT))
		iowrite16(val, dst);
	else
		netif_err(priv, drv, dev, "%s: error %d writing SFR reg %02x=%04x\n",
			  __func__, -EINVAL, reg, val);
}

static void enc624j600_clr_bits(struct enc624j600_priv *priv, u8 reg, u16 mask)
{
	u16 offset = SFR_REG_BASE | reg;
	void __iomem *dst = priv->base + offset;

	if ((reg >= 0x40 && reg < 0x6c) || reg >= 0x80) {
		u16 val = enc624j600_read_reg(priv, reg);

		iowrite16(val & ~mask, dst);
	} else {
		iowrite16(mask, dst + SFR_CLR_OFFSET);
	}
}

static void enc624j600_set_bits(struct enc624j600_priv *priv, u8 reg, u16 mask)
{
	u16 offset = SFR_REG_BASE | reg;
	void __iomem *dst = priv->base + offset;

	if ((reg >= 0x40 && reg < 0x6c) || reg >= 0x80) {
		u16 val = enc624j600_read_reg(priv, reg);

		iowrite16(val | mask, dst);
	} else {
		iowrite16(mask, dst + SFR_SET_OFFSET);
	}
}

static void enc624j600_update_reg(struct enc624j600_priv *priv, u8 reg,
				  u16 mask, u16 val)
{
	u16 offset = SFR_REG_BASE | reg;
	void __iomem *dst = priv->base + offset;
	struct net_device *dev = priv->ndev;
	int ret = 0;
	unsigned int set_mask = mask & val;
	unsigned int clr_mask = mask & ~val;

	if (unlikely(reg >= 0x40 && reg < 0x6c) || reg >= 0x80) {
		ret = -EINVAL;
		goto err_out;
	}

	if (set_mask & 0xffff)
		iowrite16(set_mask, dst + SFR_SET_OFFSET);

	if (clr_mask & 0xffff)
		iowrite16(clr_mask, dst + SFR_CLR_OFFSET);

err_out:
	if (unlikely(ret))
		netif_err(priv, drv, dev, "%s: error %d updating SFR reg %02x=%04x~%04x\n",
			  __func__, ret, reg, val, mask);
}

static bool enc624j600_phy_readable(u8 reg)
{
	switch (reg) {
	case PHCON1:
	case PHSTAT1:
	case PHANA:
	case PHANLPA:
	case PHANE:
	case PHCON2:
	case PHSTAT2:
	case PHSTAT3:
		return true;
	default:
		return false;
	}
}

static bool enc624j600_phy_writeable(u8 reg)
{
	switch (reg) {
	case PHCON1:
	case PHCON2:
	case PHANA:
		return true;
	case PHSTAT1:
	case PHSTAT2:
	case PHSTAT3:
	case PHANLPA:
	case PHANE:
	default:
		return false;
	}
}

static u16 enc624j600_read_phy(struct enc624j600_priv *priv, u8 reg)
{
	u16 val = 0;
	u16 phy_reg = MIREGADR_VAL | (reg & PHREG_MASK);
	struct net_device *dev = priv->ndev;

	if (!enc624j600_phy_readable(reg)) {
		netif_err(priv, drv, dev, "%s: error %d reading PHY reg %02x\n",
			  __func__, -EINVAL, reg);
		goto err_out;
	}

	enc624j600_write_reg(priv, MIREGADR, phy_reg);

	enc624j600_write_reg(priv, MICMD, MIIRD);

	usleep_range(26, 100);
	while (enc624j600_read_reg(priv, MISTAT) & BUSY)
		cpu_relax();

	enc624j600_write_reg(priv, MICMD, 0);

	val = enc624j600_read_reg(priv, MIRD);

err_out:
	return val;
}

static void enc624j600_write_phy(struct enc624j600_priv *priv, u8 reg, u16 val)
{
	u16 phy_reg = MIREGADR_VAL | (reg & PHREG_MASK);
	struct net_device *dev = priv->ndev;

	if (!enc624j600_phy_writeable(reg)) {
		netif_err(priv, drv, dev, "%s: error %d writing PHY reg %02x\n",
			  __func__, -EINVAL, reg);
		return;
	}

	enc624j600_write_reg(priv, MIREGADR, phy_reg);

	enc624j600_write_reg(priv, MIWR, val);

	usleep_range(26, 100);
	while (enc624j600_read_reg(priv, MISTAT) & BUSY)
		cpu_relax();
}

/* Emulate SPI commands */
static void enc624j600_cmd(struct enc624j600_priv *priv, u8 cmd)
{
	int ret = 0;
	struct net_device *dev = priv->ndev;

	switch (cmd) {
	case SETETHRST: /* System Reset */
		enc624j600_set_bits(priv, ECON2, ETHRST);
		break;
	case FCDISABLE: /* Flow Control Disable */
		enc624j600_clr_bits(priv, ECON1, FCOP0 | FCOP1);
		break;
	case FCCLEAR: /* Flow Control Clear */
		enc624j600_set_bits(priv, ECON1, FCOP0 | FCOP1);
		break;
	case SETPKTDEC: /* Decrement Packet Counter */
		enc624j600_set_bits(priv, ECON1, PKTDEC);
		break;
	case SETTXRTS: /* Request Packet Transmission */
		enc624j600_set_bits(priv, ECON1, TXRTS);
		break;
	case ENABLERX: /* Enable RX */
		enc624j600_set_bits(priv, ECON1, RXEN);
		break;
	case DISABLERX: /* Disable RX */
		enc624j600_clr_bits(priv, ECON1, RXEN);
		break;
	case SETEIE: /* Enable Interrupts */
		enc624j600_set_bits(priv, EIE, INTIE);
		break;
	case CLREIE: /* Disable Interrupts */
		enc624j600_clr_bits(priv, EIE, INTIE);
		break;
	case FCSINGLE: /* Flow Control Single */
	case FCMULTIPLE: /* Flow Control Multiple */
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	if (unlikely(ret))
		netif_err(priv, drv, dev, "%s: error %d with cmd %02x\n",
			  __func__, ret, cmd);
}

static int enc624j600_raw_read(struct enc624j600_priv *priv, u16 offset, u8 *val,
			       size_t count)
{
	void __iomem *src = priv->base + offset;
	size_t end = offset + count;

	/* Check wrap around */
	if (end <= SRAM_SIZE) {
		memcpy_fromio(val, src, count);
	} else {
		size_t remain = end - SRAM_SIZE;

		memcpy_fromio(val, src, count - remain);
		memcpy_fromio(&val[count - remain], priv->base + RXSTART_INIT, remain);
	}

	return 0;
}

static int enc624j600_raw_write(struct enc624j600_priv *priv, u16 offset,
				const u8 *val, size_t count)
{
	void __iomem *dst = priv->base + offset;

	memcpy_toio(dst, val, count);

	return 0;
}

static void enc624j600_update_phcon1(struct enc624j600_priv *priv)
{
	u16 phcon1 = enc624j600_read_phy(priv, PHCON1);

	if (priv->autoneg == AUTONEG_ENABLE) {
		phcon1 |= ANEN | RENEG;
	} else {
		phcon1 &= ~ANEN;
		if (priv->speed == SPEED_100)
			phcon1 |= SPD100;
		else
			phcon1 &= ~SPD100;

		if (priv->full_duplex)
			phcon1 |= PFULDPX;
		else
			phcon1 &= ~PFULDPX;
	}
	enc624j600_write_phy(priv, PHCON1, phcon1);
}

/* Waits for autonegotiation to complete. */
static int enc624j600_wait_for_autoneg(struct enc624j600_priv *priv)
{
	struct net_device *dev = priv->ndev;
	unsigned long timeout = jiffies + msecs_to_jiffies(2000);
	u16 phstat1;
	u16 estat;

	phstat1 = enc624j600_read_phy(priv, PHSTAT1);
	while ((phstat1 & ANDONE) == 0) {
		if (time_after(jiffies, timeout)) {
			u16 phstat3;

			netif_notice(priv, drv, dev, "timeout waiting for autoneg done\n");

			priv->autoneg = AUTONEG_DISABLE;
			phstat3 = enc624j600_read_phy(priv, PHSTAT3);
			priv->speed = (phstat3 & PHY3SPD100) ? SPEED_100 : SPEED_10;
			priv->full_duplex = (phstat3 & PHY3DPX) ? 1 : 0;
			enc624j600_update_phcon1(priv);
			netif_notice(priv, drv, dev, "Using parallel detection: %s/%s",
				     priv->speed == SPEED_100 ? "100" : "10",
				     priv->full_duplex ? "Full" : "Half");

			return -ETIMEDOUT;
		}
		cpu_relax();
		phstat1 = enc624j600_read_phy(priv, PHSTAT1);
	}

	estat = enc624j600_read_reg(priv, ESTAT);
	if (estat & PHYDPX) {
		enc624j600_set_bits(priv, MACON2, FULDPX);
		enc624j600_write_reg(priv, MABBIPG, 0x15);
	} else {
		enc624j600_clr_bits(priv, MACON2, FULDPX);
		enc624j600_write_reg(priv, MABBIPG, 0x12);
		/* Max retransmittions attempt */
		enc624j600_write_reg(priv, MACLCON, 0x370f);
	}

	return 0;
}

/* Access the PHY to determine link status */
static void enc624j600_check_link_status(struct enc624j600_priv *priv)
{
	struct net_device *dev = priv->ndev;
	u16 estat;

	estat = enc624j600_read_reg(priv, ESTAT);

	if (estat & PHYLNK) {
		if (priv->autoneg == AUTONEG_ENABLE)
			enc624j600_wait_for_autoneg(priv);

		netif_carrier_on(dev);
		netif_info(priv, ifup, dev, "link up\n");
	} else {
		netif_info(priv, ifdown, dev, "link down\n");

		/* Re-enable autoneg since we won't know what we might be
		 * connected to when the link is brought back up again.
		 */
		priv->autoneg  = AUTONEG_ENABLE;
		priv->full_duplex = true;
		priv->speed = SPEED_100;
		netif_carrier_off(dev);
	}
}

static void enc624j600_int_link_handler(struct enc624j600_priv *priv)
{
	enc624j600_check_link_status(priv);
	enc624j600_clr_bits(priv, EIR, LINKIF);
}

static void enc624j600_tx_complete(struct enc624j600_priv *priv, bool err)
{
	struct sk_buff *skb;
	struct net_device *dev = priv->ndev;

	WARN_ON(!priv->tx_skb);

	mutex_lock(&priv->lock);

	netif_stop_queue(dev);

	if (err)
		dev->stats.tx_errors++;
	else
		dev->stats.tx_packets++;

	dev->stats.tx_bytes += priv->tx_skb->len;

	enc624j600_clr_bits(priv, EIR, TXIF | TXABTIF);

	netif_dbg(priv, tx_done, dev, "TX Done%s\n", err ? ": Err" : "");

	dev_kfree_skb(priv->tx_skb);

	skb = skb_dequeue(&priv->txq);
	if (skb) {
		priv->tx_skb = skb;
		enc624j600_hw_tx(priv);
	} else {
		priv->tx_skb = NULL;
	}

	netif_wake_queue(dev);

	mutex_unlock(&priv->lock);
}

static int enc624j600_receive_packet(struct enc624j600_priv *priv,
				     struct rsv *rsv)
{
	struct net_device *dev = priv->ndev;
	struct sk_buff *skb = netdev_alloc_skb(dev, rsv->len + NET_IP_ALIGN);

	if (!skb) {
		dev->stats.rx_dropped++;
		return -ENOMEM;
	}
	skb_reserve(skb, NET_IP_ALIGN);
	enc624j600_raw_read(priv, priv->next_packet + sizeof(struct rsv),
			    skb_put(skb, rsv->len), rsv->len);

	if (netif_msg_pktdata(priv))
		dump_packet("RX", skb->len, skb->data);

	skb->dev = dev;
	skb->protocol = eth_type_trans(skb, dev);
	skb->ip_summed = CHECKSUM_UNNECESSARY;

	/* Maintain stats */
	dev->stats.rx_packets++;
	dev->stats.rx_bytes += rsv->len;

	netif_rx(skb);

	return 0;
}

static void enc624j600_rx_packets(struct enc624j600_priv *priv, u8 packet_count)
{
	struct net_device *dev = priv->ndev;

	while (packet_count--) {
		struct rsv rsv;
		u16 newrxtail;

		enc624j600_raw_read(priv, priv->next_packet, (u8 *)&rsv, sizeof(rsv));

		if (netif_msg_rx_status(priv))
			enc624j600_dump_rsv(priv, __func__, &rsv);

		if (!RSV_GETBIT(rsv.rxstat, RSV_RXOK) ||
		    rsv.len > MAX_FRAMELEN) {
			netif_err(priv, rx_err, dev, "RX Error %04x\n", rsv.rxstat);
			dev->stats.rx_errors++;

			if (RSV_GETBIT(rsv.rxstat, RSV_CRCERROR))
				dev->stats.rx_crc_errors++;
			if (RSV_GETBIT(rsv.rxstat, RSV_LENCHECKERR))
				dev->stats.rx_frame_errors++;
			if (rsv.len > MAX_FRAMELEN)
				dev->stats.rx_over_errors++;
		} else {
			enc624j600_receive_packet(priv, &rsv);
		}

		priv->next_packet = rsv.next_packet;

		newrxtail = priv->next_packet - 2;
		if (newrxtail == ENC_RX_BUF_START)
			newrxtail = SRAM_SIZE - 2;

		enc624j600_cmd(priv, SETPKTDEC);
		enc624j600_write_reg(priv, ERXTAIL, newrxtail);
	}
}

static irqreturn_t enc624j600_isr(int irq, void *dev_id)
{
	struct enc624j600_priv *priv = dev_id;
	struct net_device *dev = priv->ndev;
	int eir;

	/* Clear interrupts */
	enc624j600_cmd(priv, CLREIE);

	eir = enc624j600_read_reg(priv, EIR);

	if (eir & LINKIF)
		enc624j600_int_link_handler(priv);

	if (eir & TXIF)
		enc624j600_tx_complete(priv, false);

	if (eir & TXABTIF)
		enc624j600_tx_complete(priv, true);

	if (eir & RXABTIF) {
		if (eir & PCFULIF) {
			/* Packet counter is full */
			netif_err(priv, rx_err, dev, "Packet counter full\n");
		}
		dev->stats.rx_dropped++;
		enc624j600_clr_bits(priv, EIR, RXABTIF);
	}

	if (eir & PKTIF) {
		u8 packet_count;

		mutex_lock(&priv->lock);

		packet_count = enc624j600_read_reg(priv, ESTAT) & 0xff;
		while (packet_count) {
			enc624j600_rx_packets(priv, packet_count);
			packet_count = enc624j600_read_reg(priv, ESTAT) & 0xff;
		}

		mutex_unlock(&priv->lock);
	}

	/* Enable interrupts */
	enc624j600_cmd(priv, SETEIE);

	return IRQ_HANDLED;
}

static int enc624j600_soft_reset(struct enc624j600_priv *priv)
{
	int ret = 0;
	int timeout;
	u16 eudast;

	/* Write and verify a test value to EUDAST */
	timeout = 10;
	do {
		enc624j600_write_reg(priv, EUDAST, EUDAST_TEST_VAL);
		eudast = enc624j600_read_reg(priv, EUDAST);
		usleep_range(25, 100);
	} while ((eudast != EUDAST_TEST_VAL) && --timeout);

	if (timeout == 0) {
		ret = -ETIMEDOUT;
		goto err_out;
	}

	/* Wait for CLKRDY to become set */
	timeout = 10;
	while (!(enc624j600_read_reg(priv, ESTAT) & CLKRDY) && --timeout)
		usleep_range(25, 100);

	if (timeout == 0) {
		ret = -ETIMEDOUT;
		goto err_out;
	}

	/* Issue a System Reset command */
	enc624j600_cmd(priv, SETETHRST);
	usleep_range(25, 100);

	/* Confirm that EUDAST has 0000h after system reset */
	if (enc624j600_read_reg(priv, EUDAST) != 0) {
		ret = -EINVAL;
		goto err_out;
	}

	/* Wait for PHY register and status bits to become available */
	usleep_range(256, 1000);

err_out:
	return ret;
}

static int enc624j600_hw_reset(struct enc624j600_priv *priv)
{
	int ret;

	mutex_lock(&priv->lock);
	ret = enc624j600_soft_reset(priv);
	mutex_unlock(&priv->lock);

	return ret;
}

static void enc624j600_reset_hw_tx(struct enc624j600_priv *priv)
{
	enc624j600_set_bits(priv, ECON2, TXRST);
	enc624j600_clr_bits(priv, ECON2, TXRST);
}

static void enc624j600_hw_init_tx(struct enc624j600_priv *priv)
{
	/* Reset TX */
	enc624j600_reset_hw_tx(priv);

	/* Clear the TXIF flag if were previously set */
	enc624j600_clr_bits(priv, EIR, TXIF | TXABTIF);

	/* Write the Tx Buffer pointer */
	enc624j600_write_reg(priv, EGPWRPT, ENC_TX_BUF_START);
}

static void enc624j600_hw_init_rx(struct enc624j600_priv *priv)
{
	enc624j600_cmd(priv, DISABLERX);

	/* Set up RX packet start address in the SRAM */
	enc624j600_write_reg(priv, ERXST, ENC_RX_BUF_START);

	/* Preload the RX Data pointer to the beginning of the RX area */
	enc624j600_write_reg(priv, ERXRDPT, ENC_RX_BUF_START);

	priv->next_packet = ENC_RX_BUF_START;

	/* Set up RX end address in the SRAM */
	enc624j600_write_reg(priv, ERXTAIL, ENC_SRAM_SIZE - 2);

	/* Reset the  user data pointers */
	enc624j600_write_reg(priv, EUDAST, ENC_SRAM_SIZE);
	enc624j600_write_reg(priv, EUDAND, ENC_SRAM_SIZE + 1);

	/* Set Max Frame length */
	enc624j600_write_reg(priv, MAMXFL, MAX_FRAMELEN);
}

static void enc624j600_dump_config(struct enc624j600_priv *priv,
				   const char *msg)
{
	pr_info(DRV_NAME ": %s\n", msg);

	/* CHIP configuration */
	pr_info(DRV_NAME " ECON1:   %04X\n", enc624j600_read_reg(priv, ECON1));
	pr_info(DRV_NAME " ECON2:   %04X\n", enc624j600_read_reg(priv, ECON2));
	pr_info(DRV_NAME " ERXFCON: %04X\n", enc624j600_read_reg(priv, ERXFCON));
	pr_info(DRV_NAME " ESTAT:   %04X\n", enc624j600_read_reg(priv, ESTAT));
	pr_info(DRV_NAME " EIR:     %04X\n", enc624j600_read_reg(priv, EIR));
	pr_info(DRV_NAME " EIDLED:  %04X\n", enc624j600_read_reg(priv, EIDLED));

	/* MAC layer configuration */
	pr_info(DRV_NAME " MACON1:  %04X\n", enc624j600_read_reg(priv, MACON1));
	pr_info(DRV_NAME " MACON2:  %04X\n", enc624j600_read_reg(priv, MACON2));
	pr_info(DRV_NAME " MAIPG:   %04X\n", enc624j600_read_reg(priv, MAIPG));
	pr_info(DRV_NAME " MACLCON: %04X\n", enc624j600_read_reg(priv, MACLCON));
	pr_info(DRV_NAME " MABBIPG: %04X\n", enc624j600_read_reg(priv, MABBIPG));

	/* PHY configuration */
	pr_info(DRV_NAME " PHCON1:  %04X\n", enc624j600_read_phy(priv, PHCON1));
	pr_info(DRV_NAME " PHCON2:  %04X\n", enc624j600_read_phy(priv, PHCON2));
	pr_info(DRV_NAME " PHANA:   %04X\n", enc624j600_read_phy(priv, PHANA));
	pr_info(DRV_NAME " PHANLPA: %04X\n", enc624j600_read_phy(priv, PHANLPA));
	pr_info(DRV_NAME " PHANE:   %04X\n", enc624j600_read_phy(priv, PHANE));
	pr_info(DRV_NAME " PHSTAT1: %04X\n", enc624j600_read_phy(priv, PHSTAT1));
	pr_info(DRV_NAME " PHSTAT2: %04X\n", enc624j600_read_phy(priv, PHSTAT2));
	pr_info(DRV_NAME " PHSTAT3: %04X\n", enc624j600_read_phy(priv, PHSTAT3));
}

static void enc624j600_set_rxfilter_mode(struct enc624j600_priv *priv)
{
	switch (priv->rxfilter) {
	case RXFILTER_PROMISC:
		enc624j600_set_bits(priv, MACON1, PASSALL);
		enc624j600_write_reg(priv, ERXFCON, UCEN | MCEN | NOTMEEN);
		break;
	case RXFILTER_MULTI:
		enc624j600_clr_bits(priv, MACON1, PASSALL);
		enc624j600_write_reg(priv, ERXFCON, UCEN | CRCEN | BCEN | MCEN);
		break;
	case RXFILTER_NORMAL:
	default:
		enc624j600_clr_bits(priv, MACON1, PASSALL);
		enc624j600_write_reg(priv, ERXFCON, UCEN | CRCEN | BCEN);
		break;
	}
}

static void enc624j600_hw_init(struct enc624j600_priv *priv)
{
	u16 macon2;

	priv->hw_enabled = false;

	/* PHY Leds: link status,
	 * LEDA: Link State + collision events
	 * LEDB: Link State + transmit/receive events
	 */
	enc624j600_update_reg(priv, EIDLED, 0xff00, 0xcb00);

	/* Loopback disabled */
	enc624j600_write_reg(priv, MACON1, 0x9);

	/* interpacket gap value */
	enc624j600_write_reg(priv, MAIPG, 0x0c12);

	/* Write the auto negotiation pattern */
	enc624j600_write_phy(priv, PHANA, PHANA_DEFAULT);

	enc624j600_update_phcon1(priv);
	enc624j600_check_link_status(priv);

	macon2 = MACON2_RSV1 | TXCRCEN | PADCFG0 | PADCFG2 | MACON2_DEFER;
	if (priv->autoneg == AUTONEG_DISABLE && priv->full_duplex)
		macon2 |= FULDPX;

	enc624j600_set_bits(priv, MACON2, macon2);

	priv->rxfilter = RXFILTER_NORMAL;
	enc624j600_set_rxfilter_mode(priv);

	/* Program the Maximum frame length */
	enc624j600_write_reg(priv, MAMXFL, MAX_FRAMELEN);

	/* Init Tx pointers */
	enc624j600_hw_init_tx(priv);

	/* Init Rx pointers */
	enc624j600_hw_init_rx(priv);

	if (netif_msg_hw(priv))
		enc624j600_dump_config(priv, "Hw is initialized");
}

static void enc624j600_hw_enable(struct enc624j600_priv *priv)
{
	/* Clear the interrupt flags in case was set */
	enc624j600_clr_bits(priv, EIR, (PCFULIF | RXABTIF | TXABTIF | TXIF |
					PKTIF | LINKIF));

	/* Enable the interrupts */
	enc624j600_write_reg(priv, EIE, (PCFULIE | RXABTIE | TXABTIE | TXIE |
					 PKTIE | LINKIE | INTIE));

	/* Enable RX */
	enc624j600_cmd(priv, ENABLERX);

	priv->hw_enabled = true;
}

static void enc624j600_hw_disable(struct enc624j600_priv *priv)
{
	/* Disable all interrupts */
	enc624j600_write_reg(priv, EIE, 0);

	/* Disable RX */
	enc624j600_cmd(priv, DISABLERX);

	priv->hw_enabled = false;
}

static int enc624j600_setlink(struct net_device *dev, u8 autoneg, u16 speed,
			      u8 duplex)
{
	struct enc624j600_priv *priv = netdev_priv(dev);
	int ret = 0;

	if (!priv->hw_enabled) {
		/* link is in low power mode now; duplex setting
		 * will take effect on next enc624j600_hw_init()
		 */
		if (speed == SPEED_10 || speed == SPEED_100) {
			priv->autoneg = (autoneg == AUTONEG_ENABLE);
			priv->full_duplex = (duplex == DUPLEX_FULL);
			priv->speed = (speed == SPEED_100);
		} else {
			netif_warn(priv, link, dev, "unsupported link speed setting\n");
			/*speeds other than SPEED_10 and SPEED_100 */
			/*are not supported by chip */
			ret = -EOPNOTSUPP;
		}
	} else {
		netif_warn(priv, link, dev, "Warning: hw must be disabled to set link mode\n");
		ret = -EBUSY;
	}
	return ret;
}

static void enc624j600_hw_get_macaddr(struct enc624j600_priv *priv,
				      unsigned char *ethaddr)
{
	unsigned short val;

	val = enc624j600_read_reg(priv, MAADR1);

	ethaddr[0] = val & 0x00ff;
	ethaddr[1] = (val & 0xff00) >> 8;

	val = enc624j600_read_reg(priv, MAADR2);

	ethaddr[2] = val & 0x00ffU;
	ethaddr[3] = (val & 0xff00U) >> 8;

	val = enc624j600_read_reg(priv, MAADR3);

	ethaddr[4] = val & 0x00ffU;
	ethaddr[5] = (val & 0xff00U) >> 8;
}

/* Program the hardware MAC address from dev->dev_addr.*/
static int enc624j600_set_hw_macaddr(struct net_device *dev)
{
	struct enc624j600_priv *priv = netdev_priv(dev);

	if (priv->hw_enabled) {
		netif_info(priv, drv, dev, "Hardware must be disabled to set Mac address\n");
		return -EBUSY;
	}

	mutex_lock(&priv->lock);

	netif_info(priv, drv, dev, "%s: Setting MAC address to %pM\n",
		   dev->name, dev->dev_addr);

	enc624j600_write_reg(priv, MAADR3, (dev->dev_addr[4] |
				dev->dev_addr[5] << 8));
	enc624j600_write_reg(priv, MAADR2, (dev->dev_addr[2] |
				dev->dev_addr[3] << 8));
	enc624j600_write_reg(priv, MAADR1, (dev->dev_addr[0] |
				dev->dev_addr[1] << 8));

	mutex_unlock(&priv->lock);

	return 0;
}

/* Store the new hardware address in dev->dev_addr, and update the MAC.*/
static int enc624j600_set_mac_address(struct net_device *dev, void *addr)
{
	struct sockaddr *address = addr;

	if (netif_running(dev))
		return -EBUSY;
	if (!is_valid_ether_addr(address->sa_data))
		return -EADDRNOTAVAIL;

	eth_hw_addr_set(dev, address->sa_data);
	return enc624j600_set_hw_macaddr(dev);
}

static int enc624j600_open(struct net_device *dev)
{
	struct enc624j600_priv *priv = netdev_priv(dev);

	int ret = request_threaded_irq(dev->irq, NULL, enc624j600_isr,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					DRV_NAME, priv);
	if (unlikely(ret < 0)) {
		netdev_err(dev, "request irq %d failed (ret = %d)\n",
			   dev->irq, ret);
		return ret;
	}

	enc624j600_hw_disable(priv);
	enc624j600_hw_init(priv);
	enc624j600_hw_enable(priv);
	netif_start_queue(dev);

	return 0;
}

static int enc624j600_stop(struct net_device *dev)
{
	struct enc624j600_priv *priv = netdev_priv(dev);

	netif_stop_queue(dev);
	enc624j600_hw_disable(priv);
	free_irq(dev->irq, priv);
	return 0;
}

static void enc624j600_setrx_proc(struct kthread_work *ws)
{
	struct enc624j600_priv *priv =
		container_of(ws, struct enc624j600_priv, setrx_work);

	mutex_lock(&priv->lock);
	enc624j600_set_rxfilter_mode(priv);
	mutex_unlock(&priv->lock);
}

static void enc624j600_set_multicast_list(struct net_device *dev)
{
	struct enc624j600_priv *priv = netdev_priv(dev);
	int oldfilter = priv->rxfilter;

	if (dev->flags & IFF_PROMISC) {
		netif_dbg(priv, link, dev, "promiscuous mode\n");
		priv->rxfilter = RXFILTER_PROMISC;
	} else if ((dev->flags & IFF_ALLMULTI) || !netdev_mc_empty(dev)) {
		netif_dbg(priv, link, dev, "%smulticast mode\n",
			  (dev->flags & IFF_ALLMULTI) ? "all-" : "");
		priv->rxfilter = RXFILTER_MULTI;
	} else {
		netif_dbg(priv, link, dev, "normal mode\n");
		priv->rxfilter = RXFILTER_NORMAL;
	}

	if (oldfilter != priv->rxfilter)
		kthread_queue_work(&priv->kworker, &priv->setrx_work);
}

static void enc624j600_hw_tx(struct enc624j600_priv *priv)
{
	struct net_device *dev = priv->ndev;

	netif_info(priv, tx_queued, dev, "TX Packet Len:%d\n",
		   priv->tx_skb->len);

	if (netif_msg_pktdata(priv))
		dump_packet("TX", priv->tx_skb->len, priv->tx_skb->data);

	if (enc624j600_read_reg(priv, EIR) & TXABTIF)
		/* Last transmition aborted due to error. Reset TX interface */
		enc624j600_reset_hw_tx(priv);

	/* Clear the TXIF flag if were previously set */
	enc624j600_clr_bits(priv, EIR, TXIF);

	/* Copy the packet to the TX buffer address in SRAM */
	enc624j600_raw_write(priv, ENC_TX_BUF_START, (u8 *)priv->tx_skb->data,
			     priv->tx_skb->len);

	/* Program the Tx buffer start pointer */
	enc624j600_write_reg(priv, ETXST, ENC_TX_BUF_START);

	/* Program the packet length */
	enc624j600_write_reg(priv, ETXLEN, priv->tx_skb->len);

	/* Start the transmission */
	enc624j600_cmd(priv, SETTXRTS);
}

static void enc624j600_tx_proc(struct kthread_work *ws)
{
	struct enc624j600_priv *priv =
		container_of(ws, struct enc624j600_priv, tx_work);

	mutex_lock(&priv->lock);
	enc624j600_hw_tx(priv);
	mutex_unlock(&priv->lock);
}

static netdev_tx_t enc624j600_tx(struct sk_buff *skb, struct net_device *dev)
{
	struct enc624j600_priv *priv = netdev_priv(dev);

	/* save the timestamp */
	netif_trans_update(dev);

	/* Remember the skb for deferred processing */
	if (priv->tx_skb) {
		skb_queue_tail(&priv->txq, skb);
	} else {
		priv->tx_skb = skb;
		kthread_queue_work(&priv->kworker, &priv->tx_work);
	}

	return NETDEV_TX_OK;
}

/* Deal with a transmit timeout */
static void enc624j600_tx_timeout(struct net_device *dev, unsigned int txqueue)
{
	struct enc624j600_priv *priv = netdev_priv(dev);

	netif_err(priv, tx_err, dev, "TX timeout at %ld, latency %ld\n",
		  jiffies, jiffies - dev_trans_start(dev));

	dev->stats.tx_errors++;
	netif_wake_queue(dev);
}

static int enc624j600_get_regs_len(struct net_device *dev)
{
	return SFR_REG_COUNT;
}

static void enc624j600_get_regs(struct net_device *dev,
				struct ethtool_regs *regs, void *p)
{
	struct enc624j600_priv *priv = netdev_priv(dev);

	regs->version = 1;
	enc624j600_raw_read(priv, SFR_REG_BASE, p, SFR_REG_COUNT);
}

static void enc624j600_get_drvinfo(struct net_device *dev,
				   struct ethtool_drvinfo *info)
{
	strscpy(info->driver, DRV_NAME, sizeof(info->driver));
	strscpy(info->version, DRV_VERSION, sizeof(info->version));
	strscpy(info->bus_info, dev_name(dev->dev.parent),
		sizeof(info->bus_info));
}

static int enc624j600_get_link_ksettings(struct net_device *dev,
					 struct ethtool_link_ksettings *cmd)
{
	struct enc624j600_priv *priv = netdev_priv(dev);
	u32 supported;

	supported = SUPPORTED_10baseT_Half | SUPPORTED_10baseT_Full |
			 SUPPORTED_100baseT_Half | SUPPORTED_100baseT_Full |
			 SUPPORTED_Autoneg | SUPPORTED_TP;

	ethtool_convert_legacy_u32_to_link_mode(cmd->link_modes.supported,
						supported);

	cmd->base.speed = priv->speed;
	cmd->base.duplex = priv->full_duplex ? DUPLEX_FULL : DUPLEX_HALF;
	cmd->base.port = PORT_TP;
	cmd->base.autoneg = priv->autoneg ? AUTONEG_ENABLE : AUTONEG_DISABLE;

	return 0;
}

static int
enc624j600_set_link_ksettings(struct net_device *dev,
			      const struct ethtool_link_ksettings *cmd)
{
	return enc624j600_setlink(dev, cmd->base.autoneg,
				  cmd->base.speed, cmd->base.duplex);
}

static u32 enc624j600_get_msglevel(struct net_device *dev)
{
	struct enc624j600_priv *priv = netdev_priv(dev);

	return priv->msg_enable;
}

static void enc624j600_set_msglevel(struct net_device *dev, u32 val)
{
	struct enc624j600_priv *priv = netdev_priv(dev);

	priv->msg_enable = val;
}

static const struct ethtool_ops enc624j600_ethtool_ops = {
	.get_drvinfo = enc624j600_get_drvinfo,
	.get_msglevel = enc624j600_get_msglevel,
	.set_msglevel = enc624j600_set_msglevel,
	.get_regs_len = enc624j600_get_regs_len,
	.get_regs = enc624j600_get_regs,
	.get_link_ksettings = enc624j600_get_link_ksettings,
	.set_link_ksettings = enc624j600_set_link_ksettings,
};

static const struct net_device_ops enc624j600_netdev_ops = {
	.ndo_open = enc624j600_open,
	.ndo_stop = enc624j600_stop,
	.ndo_start_xmit = enc624j600_tx,
	.ndo_set_rx_mode = enc624j600_set_multicast_list,
	.ndo_set_mac_address = enc624j600_set_mac_address,
	.ndo_tx_timeout = enc624j600_tx_timeout,
	.ndo_validate_addr = eth_validate_addr,
};

static int enc624j600_probe(struct platform_device *pdev)
{
	int ret;

	struct net_device *ndev;
	struct enc624j600_priv *priv;
	u16 eidled;
	u8 addr[ETH_ALEN];

	ndev = alloc_etherdev(sizeof(struct enc624j600_priv));

	if (!ndev) {
		ret = -ENOMEM;
		goto error_out;
	}

	priv = netdev_priv(ndev);
	platform_set_drvdata(pdev, priv);
	dev_set_drvdata(&pdev->dev, priv);
	SET_NETDEV_DEV(ndev, &pdev->dev);

	priv->msg_enable = netif_msg_init(debug, DEFAULT_MSG_ENABLE);
	priv->ndev = ndev;

	/* Map ENC624J600 internal memory */
	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	/* Default configuration PHY configuration */
	priv->full_duplex = true;
	priv->autoneg = AUTONEG_ENABLE;
	priv->speed = SPEED_100;

	ndev->irq = platform_get_irq(pdev, 0);
	ndev->netdev_ops = &enc624j600_netdev_ops;

	mutex_init(&priv->lock);

	/* Reset device and check if it is connected */
	if (enc624j600_hw_reset(priv)) {
		netif_err(priv, probe, ndev,
			  DRV_NAME ": Chip is not detected\n");
		ret = -EIO;
		goto out_free;
	}

	/* Initialize the device HW to the consistent state */
	enc624j600_hw_init(priv);

	skb_queue_head_init(&priv->txq);
	kthread_init_worker(&priv->kworker);
	kthread_init_work(&priv->tx_work, enc624j600_tx_proc);
	kthread_init_work(&priv->setrx_work, enc624j600_setrx_proc);

	priv->kworker_task = kthread_run(kthread_worker_fn, &priv->kworker,
					 DRV_NAME);

	if (IS_ERR(priv->kworker_task)) {
		ret = PTR_ERR(priv->kworker_task);
		goto out_free;
	}

	/* Get the MAC address from the chip */
	enc624j600_hw_get_macaddr(priv, addr);
	eth_hw_addr_set(ndev, addr);

	ndev->ethtool_ops = &enc624j600_ethtool_ops;

	ret = register_netdev(ndev);
	if (unlikely(ret)) {
		netif_err(priv, probe, ndev, "Error %d initializing card enc624j600 card\n",
			  ret);
		goto out_stop;
	}

	eidled = enc624j600_read_reg(priv, EIDLED);
	if (((eidled & DEVID_MASK) >> DEVID_SHIFT) != ENCX24J600_DEV_ID) {
		ret = -EINVAL;
		goto out_unregister;
	}

	netif_info(priv, probe, ndev, "Silicon rev ID: 0x%02x\n",
		   (eidled & REVID_MASK) >> REVID_SHIFT);

	netif_info(priv, drv, priv->ndev, "MAC address %pM\n", ndev->dev_addr);

	return ret;

out_unregister:
	unregister_netdev(priv->ndev);
out_stop:
	kthread_stop(priv->kworker_task);
out_free:
	free_netdev(ndev);

error_out:
	return ret;
}

static int enc624j600_remove(struct platform_device *pdev)
{
	struct enc624j600_priv *priv = dev_get_drvdata(&pdev->dev);

	unregister_netdev(priv->ndev);
	skb_queue_purge(&priv->txq);
	kthread_stop(priv->kworker_task);

	free_netdev(priv->ndev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int enc624j600_suspend(struct device *dev, pm_message_t state)
{
	struct enc624j600_priv *priv = dev_get_drvdata(dev);
	struct net_device *netdev = priv->ndev;

	if (netif_running(netdev)) {
		pr_info("%s: suspending", __func__);
		netif_device_detach(netdev);
		enc624j600_stop(netdev);
	}

	return 0;
}

static int enc624j600_resume(struct device *dev)
{
	struct enc624j600_priv *priv = dev_get_drvdata(dev);
	struct net_device *netdev = priv->ndev;

	if (netif_running(netdev)) {
		pr_info("%s: resuming", __func__);
		enc624j600_open(netdev);
		netif_device_attach(netdev);
	}

	return 0;
}
#endif

static const struct of_device_id enc624j600_match_table[] = {
	{ .compatible = "microchip,enc624j600-psp" },
	{ }
};
MODULE_DEVICE_TABLE(of, enc624j600_match_table);

static struct platform_driver enc624j600_driver = {
	.driver = {
		.name = "enc624j600-psp",
		.of_match_table = enc624j600_match_table,
#ifdef CONFIG_PM_SLEEP
		.suspend = enc624j600_suspend,
		.resume = enc624j600_resume,
#endif
	},
	.probe = enc624j600_probe,
	.remove = enc624j600_remove,
};
module_platform_driver(enc624j600_driver);

MODULE_DESCRIPTION(DRV_NAME " ethernet driver");
MODULE_AUTHOR("Stephan Wurm <stephan.wurm@a-eberle.de>");
MODULE_LICENSE("GPL");
