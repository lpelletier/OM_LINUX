/*
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Juergen Beisert, kernel@pengutronix.de
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * Note: This implementation was done very close the external PMIC MC13783.
 * The MC13783 seems like a regular I2S device, but it isn't. It does various
 * things in a very special way. But also this SSI unit does some things in
 * very special way. And the datasheet keeps silent.
 */

#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <sound/pcm.h>
#include <mach/imx_ssi.h>
#include "imx2-ssi.h"
#include "imx2-dam.h"


#define DRV_NAME "mxc-ssi"

struct ssi_private {
	void __iomem *io;
	struct platform_device *pdev;
	struct clk *clk;
};

/* global data for all SSI units */
static struct ssi_private unit[2];

static void set_reg_bits(unsigned int mask, unsigned int data,
		unsigned int offset, unsigned int id)
{
	u32 reg;

	reg = readl(unit[id].io + offset);
	reg = (reg & ~mask) | data;
	writel(reg, unit[id].io + offset);
}


static unsigned long get_reg(unsigned int offset, unsigned int id)
{
	return readl(unit[id].io + offset);
}

static void set_reg(unsigned int data, int offset, unsigned int id)
{
	writel(data, unit[id].io + offset);
}

/*
 * Setup an SSI channel to I2S slave mode.
 *****************
 * Note: This setup is for an external MC13783! I guess it will not work
 * with any other I2S device!
 *****************
 *
 * What we want:
 * FS:  __################________________##
 * TX:  ..FEDCBA9876543210FEDCBA9876543210..
 *             rigth          left
 * This seems to work only with i2s master mode. Not slave.
 *
 * Do not use the i2s slave mode! Its useless, as it only transfers one word
 * per sync. All other bits to define more than one word will be ignored.
 * Same with "normal mode" without network. No chance to get it work with an
 * external standard i2s device.
 *
 * Only "normal mode" plus "network mode" will work like an i2s bus.
 *
 * In normal + network mode DC determines "Words Per Frame":
 *             WS=16          DC=2
 * FS:  __#_______________________________#_
 * TX:  ..FEDCBA9876543210FEDCBA9876543210..
 *        |    rigth     ||   left       |
 *
 * From the datasheet:
 * If STFS is configured as input, the external device should drive STFS
 * during rising edge of STCK or SRCK.
 *
 * The external device must provide:
 * - high active FS for one bit
 * - new data in falling edge, stable at rising edge
 *
 * The receive side. PMIC sends (depends on its setup!):
 *             WS=16          DC=4
 * FS:  __#______________________________________________________________#
 * TX:  ..FEDCBA9876543210FEDCBA9876543210FEDCBA9876543210FEDCBA9876543210..
 *      --|    rigth     ||   left       |--------------------------------
 *
 */
int imx_ssi_setup_unit_to_iis_slave(int id)
{
	u32 scr;

	BUG_ON(unit[id].io == NULL);

	pr_debug("%s called for unit %d\n", __func__, id);

	/* set config + disable this unit */
	scr = SSI_SCR_SYN | SSI_SCR_I2S_MODE_SLAVE; /* SSI_SCR_NET; */
	set_reg(scr, SSI_SCR, id);

	set_reg(SSI_STCR_TFEN0 |	/* FIFO 0 enabled */
		SSI_STCR_TXBIT0 |	/* MSB first, data bits 0...15 are valid */
		SSI_STCR_TSCKP |	/* send @falling edge */
		SSI_STCR_TFSI |
// 		SSI_STCR_TFSL |		/* one-bit-long-frame */
		SSI_STCR_TEFS,		/* shift */
		SSI_STCR, id);

	set_reg(SSI_SRCR_RFEN0 |	/* FIFO 0 enabled */
		SSI_SRCR_RXBIT0 |
		SSI_SRCR_RSCKP |	/* capture @rising edge */
		SSI_STCR_TFSI |
// 		SSI_SRCR_RFSL |		/* one-bit-long-frame */
		SSI_SRCR_REFS,
		SSI_SRCR, id);

	/* Used timeslot definitions:        __####____####____####____
	 * bit 0: first timeslot after sync    ____    ____    ____
	 * bit 1: second timeslot after sync       ____    ____    ____
	 */
// 	set_reg(0xFFFFFFFC, SSI_STMSK, id);
	/*
	 * these RX settings must correspond with the settings in PMIC's
	 * reg 39 codec time slot settings
	 */
// 	set_reg(0xFFFFFFFC, SSI_SRMSK, id);

	/*
	 * Watermark settings:
	 * TX FIFO: Set to 6. This means the request is active if more than
	 * five FIFO slots are empty. For this case the DMA burst length
	 * can be up to six words (three samples = six slots)
	 * RX FIFO: Set to 4. This means the request is active when 4 or more
	 * slots are in the FIFO.
	 *
	 * Note: A word is 8, 16 or 24 bit, a sample is of two
	 * words (left/right) there
	 */
	set_reg(SSI_SFCSR_RFWM0(DMA_RXFIFO_BURST) |
		SSI_SFCSR_TFWM0(DMA_TXFIFO_BURST), SSI_SFCSR, id);

	/*
	 * We want to provide I2S. This means we always need two words per
	 * sample signal. In this case 16 bit per word, 32 bit per sample.
	 * PMIC sends always 4 words (=2 samples) per sync!
	 */
	set_reg(SSI_STCCR_WL(16) | SSI_STCCR_DC(0)/*1*/, SSI_STCCR, id);
	set_reg(SSI_SRCCR_WL(16) | SSI_SRCCR_DC(0)/*3*/, SSI_SRCCR, id);

	/* enable the unit */
	set_reg(scr | SSI_SCR_SSIEN, SSI_SCR, id);

	return 0;
}
EXPORT_SYMBOL(imx_ssi_setup_unit_to_iis_slave);

/* id: SSI unit, fifo: FIFO 0 or 1 of this unit */
int imx_ssi_get_dma_tx_channel(int id, int fifo)
{
	struct resource *res;

	BUG_ON(unit[id].io == NULL);

	if (fifo < 0 || fifo > 1)	/* FIXME maybe i.MX27 specific */
		return -ENODEV;

	res = platform_get_resource_byname(unit[id].pdev,
					IORESOURCE_DMA, fifo ? "tx1" : "tx0");

	if (!res)
		return -1;

	return res->start;
}
EXPORT_SYMBOL(imx_ssi_get_dma_tx_channel);

/* return physical address of the FIFO register */
void __iomem *imx_ssi_get_dma_tx_address(int id, int fifo)
{
	struct resource *res;
	void __iomem *adr;

	BUG_ON(unit[id].io == NULL);

	if (fifo < 0 || fifo > 1)	/* FIXME imx27 specific */
		return NULL;

	res = platform_get_resource(unit[id].pdev, IORESOURCE_MEM, 0);

	if (!res)
		return NULL;

	adr = (void __iomem *)res->start + (fifo ? SSI_STX1 : SSI_STX0);

	return adr;
}
EXPORT_SYMBOL(imx_ssi_get_dma_tx_address);

/* id: SSI unit, fifo: FIFO 0 or 1 of this unit */
int imx_ssi_get_dma_rx_channel(int id, int fifo)
{
	struct resource *res;

	BUG_ON(unit[id].io == NULL);

	if (fifo < 0 || fifo > 1)	/* FIXME imx27 specific */
		return -ENODEV;

	res = platform_get_resource_byname(unit[id].pdev,
					IORESOURCE_DMA, fifo ? "rx1" : "rx0");

	if (!res)
		return -1;

	return res->start;
}
EXPORT_SYMBOL(imx_ssi_get_dma_rx_channel);

/* return physical address of the FIFO register */
void __iomem *imx_ssi_get_dma_rx_address(int id, int fifo)
{
	struct resource *res;
	void __iomem *adr;

	BUG_ON(unit[id].io == NULL);

	if (fifo < 0 || fifo > 1)	/* FIXME imx27 specific */
		return NULL;

	res = platform_get_resource(unit[id].pdev, IORESOURCE_MEM, 0);

	if (!res)
		return NULL;

	adr = (void __iomem *)res->start + (fifo ? SSI_SRX1 : SSI_SRX0);

	return adr;
}
EXPORT_SYMBOL(imx_ssi_get_dma_rx_address);

int imx_ssi_prepare(struct snd_pcm_substream *substream,
                            int ssi_id)
{
	set_reg_bits(SSI_SCR_SSIEN, SSI_SCR_SSIEN, SSI_SCR, ssi_id);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		set_reg_bits(SSI_SIER_TDMAE, SSI_SIER_TDMAE, SSI_SIER, ssi_id);
	} else {
		set_reg_bits(SSI_SIER_RDMAE | SSI_SIER_RFF0_EN, SSI_SIER_RDMAE | SSI_SIER_RFF0_EN, SSI_SIER, ssi_id);
	}

	return 0;
}
EXPORT_SYMBOL(imx_ssi_prepare);

void imx_ssi_shutdown(struct snd_pcm_substream *substream,
                             int ssi_id)
{
	set_reg_bits(SSI_SCR_SSIEN, 0, SSI_SCR, ssi_id);

}

int imx_ssi_trigger(struct snd_pcm_substream *substream, int cmd, int ssi_id)
{
	u32 scr = get_reg(SSI_SCR, ssi_id);

	pr_debug("%s called for unit %d\n", __func__, ssi_id);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			scr |= SSI_SCR_TE;
		else
			scr |= SSI_SCR_RE;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			scr &= ~SSI_SCR_TE;
		else
			scr &= ~SSI_SCR_RE;
		break;
	default:
		return -EINVAL;
	}

	set_reg(scr, SSI_SCR, ssi_id);

	return 0;
}
EXPORT_SYMBOL(imx_ssi_trigger);

static int __devinit mx2_ssi_probe(struct platform_device *pdev)
{
	struct imx_ssi_platform_data *pdata;
	struct resource *res;
	int ret, id;

	pdata = pdev->dev.platform_data;

	id = pdev->id;
	pr_info("Probing i.MX2 SSI unit %d\n", id);

	if (unit[id].io != NULL)
		return -EBUSY;

	unit[id].pdev = pdev;

	/* FIXME do it later, when someone wants to play a sound */
	if (pdata->init)
		pdata->init(pdev);

	unit[id].clk = clk_get(&pdev->dev, "ssi_clk");
	if (unit[id].clk == NULL) {
		dev_err(&pdev->dev, "Cannot get the clock for IMX SSI unit %d\n",
			pdev->id);
		ret = -ENODEV;
		goto err1;
	}
	/* FIXME do it later, when someone wants to play a sound */
	ret = clk_enable(unit[id].clk);
	if (ret != 0) {
		dev_err(&pdev->dev, "Cannot enable the clock for IMX SSI unit %d\n",
			pdev->id);
		ret = -ENODEV;
		goto err2;
	}

	/* external clock (if used) should never be greater than 1/4 ipg_clk! */
	pr_debug("Main clock for SSI is: %luHz\n", clk_get_rate(unit[id].clk));

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -ENODEV;
		goto err2;
	}

	if (!request_mem_region(res->start, res->end - res->start + 1, DRV_NAME)) {
		dev_err(&pdev->dev, "request_mem_region failed for IMX SSI %d\n",
			pdev->id);
		ret = -EBUSY;
		goto err2;
	}
	unit[id].io = ioremap(res->start, res->end - res->start + 1);
	if (unit[id].io == NULL) {
		dev_err(&pdev->dev, "Mapping region failed for IMX SSI %d\n",
			pdev->id);
		ret = -ENODEV;
		goto err3;
	}

	return 0;

err3:
	release_mem_region(res->start, res->end - res->start + 1);
	clk_disable(unit[id].clk);
err2:
	clk_put(unit[id].clk);
err1:
	if (pdata->exit)
		(pdata->exit)(pdev);
	return ret;
}

static int __devexit mx2_ssi_remove(struct platform_device *pdev)
{
	struct imx_ssi_platform_data *pdata = pdev->dev.platform_data;
	struct resource *res;
	int id;

	id = pdev->id;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	clk_disable(unit[id].clk);
	clk_put(unit[id].clk);
	if (pdata->exit)
		pdata->exit(pdev);
	release_mem_region(res->start, res->end - res->start + 1);
	iounmap(unit[id].io);
	unit[id].io = NULL;

	return 0;
}

static struct platform_driver mx2_ssi_driver = {
	.driver = {
		.name		= DRV_NAME,
		.owner		= THIS_MODULE,
	},
	.probe		= mx2_ssi_probe,
	.remove		= __devexit_p(mx2_ssi_remove),
#ifdef CONFIG_PM
	.suspend	= NULL, /* TODO mx2_ssi_suspend */
	.resume		= NULL	/* TODO mx2_ssi_resume */
#endif
};

int __init mx2_ssi_init(void)
{
	int err;

	err = mx2_dam_init();
	if (err != 0)
		return err;
	else
		return platform_driver_register(&mx2_ssi_driver);
}

void __exit mx2_ssi_exit(void)
{
	mx2_dam_exit();
	platform_driver_unregister(&mx2_ssi_driver);
}

module_init(mx2_ssi_init);
module_exit(mx2_ssi_exit);

MODULE_AUTHOR("Pengutronix");
MODULE_DESCRIPTION("i.MX2 SSI Driver");
MODULE_LICENSE("GPL");

