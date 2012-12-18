/* drivers/net/can/mcp251x.c
 *
 *	Driver for Microchip MCP251x CAN Controller with SPI Interface
 *
 * Integrated into Linux 2.6.27 under contract by: 
 *	Claudio Scordino <claudio@evidence.eu.com>
 *
 * Copyright 2007 Raymarine UK, Ltd. All Rights Reserved.
 * Written under contract by: 
 *	Chris Elston, Katalix Systems, Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Based on Microchip MCP251x CAN controller driver written by
 * David Vrabel, Copyright 2006 Arcom Control Systems Ltd.
 *
 * Based on CAN bus driver for the CCAN controller written by
 * - Sascha Hauer, Marc Kleine-Budde, Pengutronix
 * - Simon Kallweit, intefo AG
 * Copyright 2007
 */

/*
 * Notes:
 * This driver interacts with 2 subsystems.
 * - To the SPI subsystem it is a 'protocol driver'
 * - To the CAN subsystem it is a 'network driver'
 *
 * Because it is an SPI device, it's probing is handled via the SPI device
 * mechanisms (which are very similar to platform devices).
 *
 */

/*
 * Platform file must specify something like:
 *
 *  	static struct mcp251x_platform_data mcp251x_info = {
 *		.oscillator_frequency = 19000000,
 *		.board_specific_setup = myboard_mcp251x_initfunc,
 *		.device_reset = myboard_mcp251x_reset,
 *		.transceiver_enable = NULL,
 *	};
 *
 *	static struct spi_board_info spi_board_info[] __initdata = {
 *	{
 *		.modalias	= "mcp251x",
 *		.platform_data	= &mcp251x_info,
 *		.irq		= 10,
 *		.max_speed_hz	= 8000000,
 *		.bus_num	= 1,
 *		.chip_select	= 0
 *	},
 *	};
 *
 * (See Documentation/spi/spi-summary for more info)
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/can.h>
#include <linux/spi/spi.h>
#include <linux/can/dev.h>
#include <linux/can/core.h>
#include <linux/if_arp.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/completion.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include "mcp251x.h"

/* SPI interface instruction set */
#define INSTRUCTION_WRITE		0x02
#define INSTRUCTION_READ		0x03
#define INSTRUCTION_BIT_MODIFY	0x05
#define INSTRUCTION_LOAD_TXB(n)	(0x40 + 2 * (n))
#define INSTRUCTION_READ_RXB(n)	(((n) == 0) ? 0x90 : 0x94)
#define INSTRUCTION_RESET		0xC0

/* MPC251x registers */
#define CANSTAT       0x0e
#define CANCTRL       0x0f
#  define CANCTRL_REQOP_MASK        0xe0
#  define CANCTRL_REQOP_CONF        0x80
#  define CANCTRL_REQOP_LISTEN_ONLY 0x60
#  define CANCTRL_REQOP_LOOPBACK    0x40
#  define CANCTRL_REQOP_SLEEP       0x20
#  define CANCTRL_REQOP_NORMAL      0x00
#  define CANCTRL_OSM               0x08
#  define CANCTRL_ABAT              0x10
#define TEC           0x1c
#define REC           0x1d
#define CNF1          0x2a
#define CNF2          0x29
#  define CNF2_BTLMODE  0x80
#define CNF3          0x28
#  define CNF3_SOF      0x08
#  define CNF3_WAKFIL   0x04
#  define CNF3_PHSEG2_MASK 0x07
#define CANINTE       0x2b
#  define CANINTE_MERRE 0x80
#  define CANINTE_WAKIE 0x40
#  define CANINTE_ERRIE 0x20
#  define CANINTE_TX2IE 0x10
#  define CANINTE_TX1IE 0x08
#  define CANINTE_TX0IE 0x04
#  define CANINTE_RX1IE 0x02
#  define CANINTE_RX0IE 0x01
#define CANINTF       0x2c
#  define CANINTF_MERRF 0x80
#  define CANINTF_WAKIF 0x40
#  define CANINTF_ERRIF 0x20
#  define CANINTF_TX2IF 0x10
#  define CANINTF_TX1IF 0x08
#  define CANINTF_TX0IF 0x04
#  define CANINTF_RX1IF 0x02
#  define CANINTF_RX0IF 0x01
#define EFLG          0x2d
#  define EFLG_EWARN    0x01
#  define EFLG_RXWAR    0x02
#  define EFLG_TXWAR    0x04
#  define EFLG_RXEP     0x08
#  define EFLG_TXEP     0x10
#  define EFLG_TXBO     0x20
#  define EFLG_RX0OVR   0x40
#  define EFLG_RX1OVR   0x80
#define TXBCTRL(n)  ((n * 0x10) + 0x30)
#  define TXBCTRL_TXREQ  0x08
#define RXBCTRL(n)  ((n * 0x10) + 0x60)
#  define RXBCTRL_BUKT   0x04

/* Buffer size required for the largest SPI transfer (i.e., reading a
 * frame). */
#define CAN_FRAME_MAX_DATA_LEN	8
#define SPI_TRANSFER_BUF_LEN	(2*(6 + CAN_FRAME_MAX_DATA_LEN))
#define CAN_FRAME_MAX_BITS      128

#define DEVICE_NAME "mcp251x"

static int enable_dma; /* Enable SPI DMA. Default: 0 (Off) */
module_param(enable_dma, int, S_IRUGO);
MODULE_PARM_DESC(enable_dma, "Enable SPI DMA. Default: 0 (Off)");

static int loopback; /* Loopback testing. Default: 0 (Off) */
module_param(loopback, int, S_IRUGO);
MODULE_PARM_DESC(loopback, "Loop back frames (for testing). Default: 0 (Off)");

struct mcp251x_priv {
	struct can_priv    can;
	struct net_device *net;
	struct spi_device *spi;

	struct semaphore spi_lock; /* SPI buffer lock */
	u8 *spi_tx_buf;
	u8 *spi_rx_buf;
	dma_addr_t spi_tx_dma;
	dma_addr_t spi_rx_dma;

	struct sk_buff *tx_skb;
	struct workqueue_struct *wq;
	struct work_struct tx_work;
	struct work_struct irq_work;
	struct completion awake;
	int wake;
};

static u8 mcp251x_read_reg(struct spi_device *spi, uint8_t reg)
{
	struct mcp251x_priv *priv = dev_get_drvdata(&spi->dev);
	struct spi_transfer t = {
		.tx_buf = priv->spi_tx_buf,
		.rx_buf = priv->spi_rx_buf,
		.len = 3,
		.cs_change = 0,
	};
	struct spi_message m;
	u8 val = 0;
	int ret;

	down(&priv->spi_lock);

	priv->spi_tx_buf[0] = INSTRUCTION_READ;
	priv->spi_tx_buf[1] = reg;

	spi_message_init(&m);

	if (enable_dma) {
		t.tx_dma = priv->spi_tx_dma;
		t.rx_dma = priv->spi_rx_dma;
		m.is_dma_mapped = 1;
	}

	spi_message_add_tail(&t, &m);

	ret = spi_sync(spi, &m);
	if (ret < 0) {
		dev_dbg(&spi->dev, "%s: failed: ret = %d\n", __FUNCTION__, ret);
	} else
		val = priv->spi_rx_buf[2];

	up(&priv->spi_lock);

	return val;
}


static void mcp251x_write_reg(struct spi_device *spi, u8 reg, uint8_t val)
{
	struct mcp251x_priv *priv = dev_get_drvdata(&spi->dev);
	struct spi_transfer t = {
		.tx_buf = priv->spi_tx_buf,
		.rx_buf = priv->spi_rx_buf,
		.len = 3,
		.cs_change = 0,
	};
	struct spi_message m;
	int ret;

	down(&priv->spi_lock);

	priv->spi_tx_buf[0] = INSTRUCTION_WRITE;
	priv->spi_tx_buf[1] = reg;
	priv->spi_tx_buf[2] = val;

	spi_message_init(&m);

	if (enable_dma) {
		t.tx_dma = priv->spi_tx_dma;
		t.rx_dma = priv->spi_rx_dma;
		m.is_dma_mapped = 1;
	}

	spi_message_add_tail(&t, &m);

	ret = spi_sync(spi, &m);

	up(&priv->spi_lock);

	if (ret < 0)
		dev_dbg(&spi->dev, "%s: failed\n", __FUNCTION__);
}


static void mcp251x_write_bits(struct spi_device *spi, u8 reg,
                               u8 mask, uint8_t val)
{
	struct mcp251x_priv *priv = dev_get_drvdata(&spi->dev);
	struct spi_transfer t = {
		.tx_buf = priv->spi_tx_buf,
		.rx_buf = priv->spi_rx_buf,
		.len = 4,
		.cs_change = 0,
	};
	struct spi_message m;
	int ret;

	down(&priv->spi_lock);

	priv->spi_tx_buf[0] = INSTRUCTION_BIT_MODIFY;
	priv->spi_tx_buf[1] = reg;
	priv->spi_tx_buf[2] = mask;
	priv->spi_tx_buf[3] = val;

	spi_message_init(&m);

	if (enable_dma) {
		t.tx_dma = priv->spi_tx_dma;
		t.rx_dma = priv->spi_rx_dma;
		m.is_dma_mapped = 1;
	}

	spi_message_add_tail(&t, &m);

	ret = spi_sync(spi, &m);

	up(&priv->spi_lock);

	if (ret < 0)
		dev_dbg(&spi->dev, "%s: failed\n", __FUNCTION__);
}


static void mcp251x_hw_tx(struct spi_device *spi, struct can_frame *frame,
int tx_buf_idx)
{
	struct mcp251x_priv *priv = dev_get_drvdata(&spi->dev);
	struct spi_transfer t = {
		.tx_buf = priv->spi_tx_buf,
		.rx_buf = priv->spi_rx_buf,
		.cs_change = 0,
		.len = 6 + CAN_FRAME_MAX_DATA_LEN,
	};
	u8 *tx_buf = priv->spi_tx_buf;
	struct spi_message m;
	int ret;
	u32 sid, eid, exide, rtr;

	dev_dbg(&spi->dev, "%s\n", __FUNCTION__);

	sid = frame->can_id & CAN_SFF_MASK; /* Standard ID */
	eid = frame->can_id & CAN_EFF_MASK; /* Extended ID */
	exide = (frame->can_id & CAN_EFF_FLAG) ? 1 : 0; /* Extended ID Enable */
	rtr = (frame->can_id & CAN_RTR_FLAG) ? 1 : 0; /* Remote transmission */

	down(&priv->spi_lock);

	tx_buf[0] = INSTRUCTION_LOAD_TXB(tx_buf_idx);
	tx_buf[1] = sid >> 3;
	tx_buf[2] = (sid << 5) | (exide << 3) | (eid >> 16);
	tx_buf[3] = eid >> 8;
	tx_buf[4] = eid;
	tx_buf[5] = (rtr << 6) | frame->can_dlc;

	memcpy(tx_buf + 6, frame->data, frame->can_dlc);

	spi_message_init(&m);

	if (enable_dma) {
		t.tx_dma = priv->spi_tx_dma;
		t.rx_dma = priv->spi_rx_dma;
		m.is_dma_mapped = 1;
	}

	spi_message_add_tail(&t, &m);

	ret = spi_sync(spi, &m);

	up(&priv->spi_lock);

	if (ret < 0)
		dev_dbg(&spi->dev, "%s: failed: ret = %d\n", __FUNCTION__, ret);

	/* FIXME: Should we exit with an error here? */

	mcp251x_write_reg(spi, TXBCTRL(tx_buf_idx), TXBCTRL_TXREQ);
}


static void mcp251x_hw_rx(struct spi_device *spi, int buf_idx)
{
	struct mcp251x_priv *priv = dev_get_drvdata(&spi->dev);
	struct spi_transfer t = {
		.tx_buf = priv->spi_tx_buf,
		.rx_buf = priv->spi_rx_buf,
		.cs_change = 0,
		.len = 14, /* RX buffer: RXBnCTRL to RXBnD7 */
	};
	u8 *tx_buf = priv->spi_tx_buf;
	u8 *rx_buf = priv->spi_rx_buf;
	struct spi_message m;
	struct sk_buff *skb;
	struct can_frame *frame;
	int ret;

	dev_dbg(&spi->dev, "%s\n", __FUNCTION__);

	skb = dev_alloc_skb(sizeof(struct can_frame));
	if (!skb) {
		dev_dbg(&spi->dev, "%s: out of memory for Rx'd frame\n", __FUNCTION__);
		priv->net->stats.rx_dropped++;
		return;
	}
	skb->dev = priv->net;
	frame = (struct can_frame *)skb_put(skb, sizeof(struct can_frame));

	down(&priv->spi_lock);

	tx_buf[0] = INSTRUCTION_READ_RXB(buf_idx);

	spi_message_init(&m);

	if (enable_dma) {
		t.tx_dma = priv->spi_tx_dma;
		t.rx_dma = priv->spi_rx_dma;
		m.is_dma_mapped = 1;
	}

	spi_message_add_tail(&t, &m);

	ret = spi_sync(spi, &m);
	if (ret < 0)
		dev_dbg(&spi->dev, "%s: failed: ret = %d\n", __FUNCTION__, ret);
	/* FIXME: Should we exit with an error here? */

	if ((rx_buf[2] >> 3) & 0x1) {
		/* Extended ID format */
		frame->can_id = CAN_EFF_FLAG;
		frame->can_id |= (rx_buf[2] << 16) | (rx_buf[3] << 8) | rx_buf[4];
	} else {
		/* Standard ID format */
		frame->can_id = (rx_buf[1] << 3) | (rx_buf[2] >> 5);
	}

	if ((rx_buf[5] >> 6) & 0x1) {
		/* Remote transmission request */
		frame->can_id |= CAN_RTR_FLAG;
	}

	/* Data length */
	frame->can_dlc = rx_buf[5] & 0x0f;
	memcpy(frame->data, rx_buf + 6, CAN_FRAME_MAX_DATA_LEN);

	up(&priv->spi_lock);

	priv->net->stats.rx_packets++;
	priv->net->stats.rx_bytes += frame->can_dlc;

	skb->protocol = __constant_htons(ETH_P_CAN);
	skb->pkt_type = PACKET_BROADCAST;
	skb->ip_summed = CHECKSUM_UNNECESSARY;
	netif_rx(skb);
}


static void mcp251x_hw_sleep(struct spi_device *spi)
{
	mcp251x_write_reg(spi, CANCTRL, CANCTRL_REQOP_SLEEP);
}

static void mcp251x_hw_wakeup(struct spi_device *spi)
{
	struct mcp251x_priv *priv = dev_get_drvdata(&spi->dev);

	priv->wake = 1;

	/* Can only wake up by generating a wake-up interrupt. */
	mcp251x_write_bits(spi, CANINTE, CANINTE_WAKIE, CANINTE_WAKIE);
	mcp251x_write_bits(spi, CANINTF, CANINTF_WAKIF, CANINTF_WAKIF);

	/* Wait until the device is awake */
	wait_for_completion(&priv->awake);
}


static int mcp251x_hard_start_xmit(struct sk_buff *skb, struct net_device *net)
{
	struct mcp251x_priv *priv = netdev_priv(net);
	struct spi_device *spi = priv->spi;

	dev_dbg(&spi->dev, "%s\n", __FUNCTION__);

	if (skb->len != sizeof(struct can_frame)) {
		dev_dbg(&spi->dev, "dropping packet - bad length\n");
		dev_kfree_skb(skb);
		net->stats.tx_dropped++;
		return 0;
	}

	netif_stop_queue(net);
	priv->tx_skb = skb;
	net->trans_start = jiffies;
	queue_work(priv->wq, &priv->tx_work);

	return NETDEV_TX_OK;
}


static int mcp251x_do_set_mode(struct net_device *net, can_mode_t mode)
{
	struct mcp251x_priv *priv = netdev_priv(net);
	struct spi_device *spi = priv->spi;

	dev_dbg(&spi->dev, "%s (unimplemented)\n", __FUNCTION__);

	switch (mode) {
		default:
			return -EOPNOTSUPP;
	}

	return 0;
}


static int mcp251x_open(struct net_device *net)
{
	struct mcp251x_priv *priv = netdev_priv(net);
	struct spi_device *spi = priv->spi;
	struct mcp251x_platform_data *pdata = spi->dev.platform_data;

	dev_dbg(&spi->dev, "%s\n", __FUNCTION__);

	if (pdata->transceiver_enable)
		pdata->transceiver_enable(1);

	mcp251x_hw_wakeup(spi);

	/* Enable interrupts */
	mcp251x_write_reg(spi, CANINTE,
		CANINTE_ERRIE | CANINTE_TX2IE | CANINTE_TX1IE |
		CANINTE_TX0IE | CANINTE_RX1IE | CANINTE_RX0IE );

	if (loopback) {
		/* Put device into loopback mode */
		mcp251x_write_reg(spi, CANCTRL, CANCTRL_REQOP_LOOPBACK);
	} else {
		/* Put device into normal mode */
		mcp251x_write_reg(spi, CANCTRL, CANCTRL_REQOP_NORMAL);

		/* Wait for the device to enter normal mode */
		while (mcp251x_read_reg(spi, CANSTAT) & 0xE0)
			udelay(10);
	}

	return 0;
}

static int mcp251x_stop(struct net_device *net)
{
	struct mcp251x_priv *priv = netdev_priv(net);
	struct spi_device *spi = priv->spi;
	struct mcp251x_platform_data *pdata = spi->dev.platform_data;

	dev_dbg(&spi->dev, "%s\n", __FUNCTION__);

	/* Disable and clear pending interrupts */
	mcp251x_write_reg(spi, CANINTE, 0x00);
	mcp251x_write_reg(spi, CANINTF, 0x00);

	mcp251x_hw_sleep(spi);

	if (pdata->transceiver_enable)
		pdata->transceiver_enable(0);

	return 0;
}

static int mcp251x_do_set_bit_time(struct net_device *net, struct can_bittime *bt)
{
	struct mcp251x_priv *priv = netdev_priv(net);
	struct spi_device *spi = priv->spi;
	u8 state;

	dev_dbg(&spi->dev, "%s: BRP = %d, PropSeg = %d, PS1 = %d,"
	        " PS2 = %d, SJW = %d\n", __FUNCTION__, bt->std.brp,
		bt->std.prop_seg, bt->std.phase_seg1, bt->std.phase_seg2,
		bt->std.sjw);

	/* Store original mode and set mode to config */
	state = mcp251x_read_reg(spi, CANSTAT) & CANCTRL_REQOP_MASK;
	mcp251x_write_bits(spi, CANCTRL, CANCTRL_REQOP_MASK, CANCTRL_REQOP_CONF);

	mcp251x_write_reg(spi, CNF1, ((bt->std.sjw - 1) << 6) | (bt->std.brp-1));
	mcp251x_write_reg(spi, CNF2, CNF2_BTLMODE | ((bt->std.phase_seg1 - 1) << 3) | (bt->std.prop_seg - 1));
	mcp251x_write_bits(spi, CNF3, CNF3_PHSEG2_MASK, (bt->std.phase_seg2 - 1));

	/* Restore original state */
	mcp251x_write_bits(spi, CANCTRL, CANCTRL_REQOP_MASK, state);

	return 0;
}


static int mcp251x_do_get_state(struct net_device *net, can_state_t *state)
{
	struct mcp251x_priv *priv = netdev_priv(net);
	struct spi_device *spi = priv->spi;
	u8 eflag;

	eflag = mcp251x_read_reg(spi, EFLG);

	if (eflag & EFLG_TXBO) {
		*state = CAN_STATE_BUS_OFF;
	} else if (eflag & (EFLG_RXEP | EFLG_TXEP)) {
		*state = CAN_STATE_BUS_PASSIVE;
	} else if (eflag & EFLG_EWARN) {
		*state = CAN_STATE_BUS_WARNING;
	} else {
		*state = CAN_STATE_ACTIVE;
	}

	return 0;
}


static void mcp251x_tx_work_handler(struct work_struct *ws)
{
	struct mcp251x_priv *priv = container_of(ws, struct mcp251x_priv, tx_work);
	struct spi_device *spi = priv->spi;
	struct can_frame *frame = (struct can_frame *)priv->tx_skb->data;

	dev_dbg(&spi->dev, "%s\n", __FUNCTION__);

	if (frame->can_dlc > CAN_FRAME_MAX_DATA_LEN)
		frame->can_dlc = CAN_FRAME_MAX_DATA_LEN;

	/* FIXME: use all 3 Tx buffers */
	mcp251x_hw_tx(spi, frame, 0);
}


static void mcp251x_irq_work_handler(struct work_struct *ws)
{
	struct mcp251x_priv *priv = container_of(ws, struct mcp251x_priv, irq_work);
	struct spi_device *spi = priv->spi;
	struct net_device *net = priv->net;
	struct mcp251x_platform_data *pdata = spi->dev.platform_data;
	u8 intf;

	for (;;) {
		if (priv->wake) {
			/* Wait whilst the device wakes up */
            int i = 200 * (128 * USEC_PER_SEC / pdata->oscillator_frequency);
            if( i> 1000 ) /*large delay, use mdelay*/
               mdelay(10);
            else
    			udelay(i);
			priv->wake = 0;
		}

		intf = mcp251x_read_reg(spi, CANINTF);
		if (intf == 0x00)
			break;

		dev_dbg(&spi->dev, "interrupt:%s%s%s%s%s%s%s%s\n",
			(intf & CANINTF_MERRF) ? " MERR":"",
			(intf & CANINTF_WAKIF) ? " WAK":"",
			(intf & CANINTF_ERRIF) ? " ERR":"",
			(intf & CANINTF_TX2IF) ? " TX2":"",
			(intf & CANINTF_TX1IF) ? " TX1":"",
			(intf & CANINTF_TX0IF) ? " TX0":"",
			(intf & CANINTF_RX1IF) ? " RX1":"",
			(intf & CANINTF_RX0IF) ? " RX0":"");

		if (intf & CANINTF_WAKIF) {
			complete(&priv->awake);
		}

		if (intf & CANINTF_MERRF) {
			u8 txbnctrl;
			/* if there are no pending Tx buffers, restart queue */
			txbnctrl = mcp251x_read_reg(spi, TXBCTRL(0));
			if (!(txbnctrl & TXBCTRL_TXREQ))
				netif_wake_queue(net);
		}

		if (intf & CANINTF_ERRIF) {
			struct sk_buff *skb;
			struct can_frame *frame = 0;
			u8 eflag = mcp251x_read_reg(spi, EFLG);

			dev_dbg(&spi->dev, "EFLG = 0x%02x\n", eflag);

			/* Create error frame */
			skb = dev_alloc_skb(sizeof(struct can_frame));
			if (skb) {
				frame = (struct can_frame *)skb_put(skb, sizeof(struct can_frame));
				frame->can_id = CAN_ERR_FLAG;
				frame->can_dlc = CAN_ERR_DLC;

				skb->dev = net;
				skb->protocol = __constant_htons(ETH_P_CAN);
				skb->pkt_type = PACKET_BROADCAST;
				skb->ip_summed = CHECKSUM_UNNECESSARY;

				/* Set error frame flags according to bus state */
				if (eflag & EFLG_TXBO) {
					frame->can_id |= CAN_ERR_BUSOFF;
				} else if (eflag & EFLG_TXEP) {
					frame->can_id |= CAN_ERR_CRTL;
					frame->data[1] |= CAN_ERR_CRTL_TX_PASSIVE;
				} else if (eflag & EFLG_RXEP) {
					frame->can_id |= CAN_ERR_CRTL;
					frame->data[1] |= CAN_ERR_CRTL_RX_PASSIVE;
				} else if (eflag & EFLG_TXWAR) {
					frame->can_id |= CAN_ERR_CRTL;
					frame->data[1] |= CAN_ERR_CRTL_TX_WARNING;
				} else if (eflag & EFLG_RXWAR) {
					frame->can_id |= CAN_ERR_CRTL;
					frame->data[1] |= CAN_ERR_CRTL_RX_WARNING;
				}
			}

			if (eflag & (EFLG_RX0OVR | EFLG_RX1OVR)) {
				if (eflag & EFLG_RX0OVR) {
					net->stats.rx_over_errors++;
				} if (eflag & EFLG_RX1OVR) {
					net->stats.rx_over_errors++;
				} if (frame) {
					frame->can_id |= CAN_ERR_CRTL;
					frame->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
				}
				mcp251x_write_reg(spi, EFLG, 0x00);
			}

			if (skb)
				netif_rx(skb);
		}

		if (intf & (CANINTF_TX2IF | CANINTF_TX1IF | CANINTF_TX0IF)) {
			net->stats.tx_packets++;
			net->stats.tx_bytes +=
				((struct can_frame *)(priv->tx_skb->data))->can_dlc;
			dev_kfree_skb(priv->tx_skb);
			netif_wake_queue(net);
		}

		if (intf & CANINTF_RX0IF)
			mcp251x_hw_rx(spi, 0);

		if (intf & CANINTF_RX1IF)
			mcp251x_hw_rx(spi, 1);

		/* Clear everything except RX bits, don't want to miss any */
		mcp251x_write_bits(spi, CANINTF, intf & ~(CANINTF_RX0IF | CANINTF_RX1IF), 0x00);
	}

	mcp251x_read_reg(spi, CANSTAT);

	dev_dbg(&spi->dev, "interrupt ended\n");
}


static irqreturn_t mcp251x_can_isr(int irq, void *dev_id)
{
	struct net_device *net = (struct net_device *)dev_id;
	struct mcp251x_priv *priv = netdev_priv(net);

	/* Schedule bottom half */
	queue_work(priv->wq, &priv->irq_work);

	return IRQ_HANDLED;
}


static void mcp251x_hw_reset(struct spi_device *spi)
{
	struct mcp251x_priv *priv = dev_get_drvdata(&spi->dev);
	int ret;

	down(&priv->spi_lock);

	priv->spi_tx_buf[0] = INSTRUCTION_RESET;

	ret = spi_write(spi, priv->spi_tx_buf, 1);

	up(&priv->spi_lock);

	if (ret < 0)
		dev_dbg(&spi->dev, "%s: failed: ret = %d\n", __FUNCTION__, ret);
}

static struct net_device *alloc_mcp251x_netdev(int sizeof_priv)
{
	struct net_device *net;
	struct mcp251x_priv *priv;

	net = alloc_candev(sizeof_priv);
	if (!net) {
		return NULL;
	}

	priv = netdev_priv(net);

	net->open		= mcp251x_open;
	net->stop		= mcp251x_stop;
	net->hard_start_xmit	= mcp251x_hard_start_xmit;

	priv->can.baudrate	  = 250000;
	priv->can.do_set_bit_time = mcp251x_do_set_bit_time;
	priv->can.do_get_state    = mcp251x_do_get_state;
	priv->can.do_set_mode     = mcp251x_do_set_mode;

	priv->net = net;

	return net;
}


static int __devinit mcp251x_can_probe(struct spi_device *spi)
{
	struct net_device *net;
	struct mcp251x_priv *priv;
	struct mcp251x_platform_data *pdata = spi->dev.platform_data;
	struct can_bittime bit_time;
	int ret = -ENODEV;

	if (!pdata) {
		/* Platform data is required for osc freq */
		goto error_out;
	}

	/* Allocate can/net device */
	net = alloc_mcp251x_netdev(sizeof(struct mcp251x_priv));
	if (!net) {
		ret = -ENOMEM;
		goto error_alloc;
	}

	priv = netdev_priv(net);
	dev_set_drvdata(&spi->dev, priv);

	priv->spi = spi;
	init_MUTEX(&priv->spi_lock);

	/* Not sure why / 4... mcp251x pre-divides by 2 */
	priv->can.can_sys_clock = pdata->oscillator_frequency / 2;

	/* If requested, allocate DMA buffers */
	if (enable_dma) {
		spi->dev.coherent_dma_mask = DMA_32BIT_MASK;

		/* Minimum coherent DMA allocation is PAGE_SIZE, so allocate
                   that much and share it between Tx and Rx DMA buffers. */
		priv->spi_tx_buf = dma_alloc_coherent(&spi->dev,
			PAGE_SIZE, &priv->spi_tx_dma, GFP_DMA);

		if (priv->spi_tx_buf) {
			priv->spi_rx_buf = (u8 *)(priv->spi_tx_buf +
				(PAGE_SIZE / 2));
			priv->spi_rx_dma = (dma_addr_t)(priv->spi_tx_dma +
				(PAGE_SIZE / 2));
		} else {
			/* Fall back to non-DMA */
			enable_dma = 0;
		}
	}

	/* Allocate non-DMA buffers */
	if (!enable_dma) {
		priv->spi_tx_buf = kmalloc(SPI_TRANSFER_BUF_LEN, GFP_KERNEL);
		if (!priv->spi_tx_buf) {
			ret = -ENOMEM;
			goto error_tx_buf;
		}
		priv->spi_rx_buf = kmalloc(SPI_TRANSFER_BUF_LEN, GFP_KERNEL);
		if (!priv->spi_tx_buf) {
			ret = -ENOMEM;
			goto error_rx_buf;
		}
	}

	/* Call out to platform specific setup */
	if (pdata->board_specific_setup)
		pdata->board_specific_setup(spi);

	/* Call out to platform specific hardware reset */
	if (pdata->device_reset)
		pdata->device_reset(spi);

	/* Register IRQ */
	if (spi->irq >= 0) {
		if (request_irq(spi->irq, mcp251x_can_isr, IRQF_TRIGGER_FALLING, DEVICE_NAME, net) < 0) {
			dev_err(&spi->dev, "failed to acquire irq %d\n", spi->irq);
			goto error_irq;
		}
	}

	SET_NETDEV_DEV(net, &spi->dev);

	priv->wq = create_singlethread_workqueue("mcp251x_wq");

	INIT_WORK(&priv->tx_work, mcp251x_tx_work_handler);
	INIT_WORK(&priv->irq_work, mcp251x_irq_work_handler);

	init_completion(&priv->awake);

	/* Configure the SPI bus */
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	spi_setup(spi);

	mcp251x_hw_reset(spi);

	/* Set initial baudrate */
	ret = can_calc_bit_time(&priv->can, priv->can.baudrate, &bit_time.std);
	if (ret != 0)
		dev_err(&spi->dev, "unable to calculate initial baudrate!\n");
	else {
		bit_time.type = CAN_BITTIME_STD;
		ret = priv->can.do_set_bit_time(net, &bit_time);
		if (ret)
			dev_err(&spi->dev, "unable to set initial baudrate!\n");
	}

	/* Enable RX0->RX1 buffer roll over */
	mcp251x_write_bits(spi, RXBCTRL(0), RXBCTRL_BUKT, RXBCTRL_BUKT);

	mcp251x_hw_sleep(spi);

	ret = register_netdev(net);
	if (ret >= 0) {
		dev_info(&spi->dev, "probed%s\n",
			(loopback) ? " (loopback)" : "");
		return ret;
	}

	free_irq(spi->irq, net);
error_irq:
	if (!enable_dma)
		kfree(priv->spi_rx_buf);
error_rx_buf:
	if (!enable_dma)
		kfree(priv->spi_tx_buf);
error_tx_buf:
	free_candev(net);
	if (enable_dma) {
		dma_free_coherent(&spi->dev, PAGE_SIZE,
			priv->spi_tx_buf, priv->spi_tx_dma);
	}
error_alloc:
	dev_err(&spi->dev, "probe failed\n");
error_out:
	return ret;
}

static int __devexit mcp251x_can_remove(struct spi_device *spi)
{
	struct mcp251x_priv *priv = dev_get_drvdata(&spi->dev);
	struct net_device *net = priv->net;

	free_irq(spi->irq, net);

	flush_scheduled_work();

	if (enable_dma) {
		dma_free_coherent(&spi->dev, PAGE_SIZE,
			priv->spi_tx_buf, priv->spi_tx_dma);
	} else {
		kfree(priv->spi_tx_buf);
		kfree(priv->spi_rx_buf);
	}

	unregister_netdev(net);
	free_candev(net);

	return 0;
}

#ifdef CONFIG_PM
static int mcp251x_can_suspend(struct spi_device *spi, pm_message_t state)
{
	struct mcp251x_platform_data *pdata = spi->dev.platform_data;
	struct mcp251x_priv *priv = dev_get_drvdata(&spi->dev);
	struct net_device *net = priv->net;

        if (!netif_running(net))
                return 0;

	netif_device_detach(net);

	mcp251x_hw_sleep(spi);
	if (pdata->transceiver_enable)
		pdata->transceiver_enable(0);

	return 0;
}

static int mcp251x_can_resume(struct spi_device *spi)
{
	struct mcp251x_platform_data *pdata = spi->dev.platform_data;
	struct mcp251x_priv *priv = dev_get_drvdata(&spi->dev);
	struct net_device *net = priv->net;

        if (!netif_running(net))
                return 0;

	if (pdata->transceiver_enable)
		pdata->transceiver_enable(1);
	mcp251x_hw_wakeup(spi);

	netif_device_attach(net);

	return 0;
}
#else
#define mcp251x_can_suspend NULL
#define mcp251x_can_resume NULL
#endif

static struct spi_driver mcp251x_can_driver = {
	.driver = {
		.name		= DEVICE_NAME,
		.bus		= &spi_bus_type,
		.owner		= THIS_MODULE,
	},

	.probe		= mcp251x_can_probe,
	.remove		= __devexit_p(mcp251x_can_remove),
	.suspend	= mcp251x_can_suspend,
	.resume		= mcp251x_can_resume,
};

static int __init mcp251x_can_init(void)
{
	return spi_register_driver(&mcp251x_can_driver);
}

static void __exit mcp251x_can_exit(void)
{
	spi_unregister_driver(&mcp251x_can_driver);
}

module_init(mcp251x_can_init);
module_exit(mcp251x_can_exit);

MODULE_AUTHOR("Claudio Scordino <claudio@evidence.eu.com");
MODULE_DESCRIPTION("Microchip 251x CAN driver");
MODULE_LICENSE("GPL v2");
