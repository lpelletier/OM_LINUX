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
#define DEBUG

#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <sound/pcm.h>
#include <mach/imx_ssi.h>
#include "imx-ssi.h"

#define DRV_NAME "imx-ssi"

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

#ifdef DEBUG
static void dump_ssi_registers(void)
{
	printk("SSI_STX:   0x%x\n", SSI_STX);
	printk("SSI_SRX:   0x%x\n", SSI_SRX);
	printk("SSI_SCSR:  0x%x\n", SSI_SCSR);
	printk("SSI_STCR:  0x%x\n", SSI_STCR);
	printk("SSI_SRCR:  0x%x\n", SSI_SRCR);
	printk("SSI_STCCR: 0x%x\n", SSI_STCCR);
	printk("SSI_SRCCR: 0x%x\n", SSI_SRCCR);
	printk("SSI_STSR:  0x%x\n", SSI_STSR);
	printk("SSI_SFCSR: 0x%x\n", SSI_SFCSR);
// 	printk("SSI_STR:   0x%x\n", SSI_STR);
	printk("SSI_SOR:   0x%x\n", SSI_SOR);
}
#endif

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
#define TRANSMIT_WM_LEVEL 8		/* (in free places) */

int imx_ssi_setup_unit_to_iis_slave(int id)
{
	u32 scsr;

	pr_debug("%s called for unit %d\n", __func__, id);
	BUG_ON(unit[id].io == NULL); /* (only one on i.MXL/1) */

	/* choose mode + disable this unit */
	scsr = SSI_SCSR_I2S_MODE_SLAVE | SSI_SCSR_SYN;
	set_reg(scsr, SSI_SCSR, id);

	/* Transmit config */
	set_reg(SSI_MAE |	/* Activate DMA */
		SSI_FEN |	/* FIFO enabled */
		SSI_SCKP |	/* send @falling edge */
		SSI_FSI |	/* Frame sync active low */
		SSI_EFS,	/* one bit shift */
		SSI_STCR, id);

// 	set_reg(SSI_STCR_TFEN0 |	/* FIFO 0 enabled */
// 		SSI_STCR_TXBIT0 |	/* MSB first, data bits 0...15 are valid */
// 		SSI_STCR_TSCKP |	/* send @falling edge */
// 		SSI_STCR_TFSL |		/* one-bit-long-frame */
// 		SSI_STCR_TEFS,		/* shift */
// 		SSI_STCR, id);

	/* Receive config */
	set_reg(SSI_MAE |	/* Activate DMA */
		SSI_FEN |	/* FIFO enabled */
		SSI_SCKP |	/* receive @falling edge ?? */
		SSI_FSI |	/* Frame sync active low */
		SSI_EFS,	/* one bit shift */
		SSI_SRCR, id);

//
// 	set_reg(SSI_SRCR_RFEN0 |	/* FIFO 0 enabled */
// 		SSI_SRCR_RXBIT0 |
// 		SSI_SRCR_RSCKP |	/* capture @rising edge */
// 		SSI_SRCR_RFSL |		/* one-bit-long-frame */
// 		SSI_SRCR_REFS,
// 		SSI_SRCR, id);

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
	/* SSI FIFO watermark */
// 	SSI_SFCSR = SSI_RFWM(8) | SSI_TFWM(TRANSMIT_WM_LEVEL);
	set_reg(SSI_SFCSR_RFWM(8) | SSI_SFCSR_TFWM(TRANSMIT_WM_LEVEL), SSI_SFCSR, id);

	/*
	 * We want to provide I2S. This means we always need two words per
	 * sample signal. In this case 16 bit per word, 32 bit per sample.
	 * PMIC sends always 4 words (=2 samples) per sync!
	 */
// 	SSI_STCCR = SSI_WL_16 | SSI_DC(0);
	set_reg(SSI_WL_16 | SSI_DC(0), SSI_STCCR, id);
// 	SSI_SRCCR = SSI_WL_16 | SSI_DC(0);
	set_reg(SSI_WL_16 | SSI_DC(0), SSI_SRCCR, id);

	/* enable the unit */
// 	SSI_SCSR = SSI_SCSR_I2S_MODE_SLAVE | SSI_SCSR_SYN | SSI_SCSR_EN;
	set_reg(scsr | SSI_SCSR_EN, SSI_SCSR, id);

	return 0;
}
EXPORT_SYMBOL(imx_ssi_setup_unit_to_iis_slave);

int imx_ssi_setup_unit_to_iis_master(int id)
{
	u32 scsr;

	pr_debug("%s called for unit %d\n", __func__, id);
	BUG_ON(unit[id].io == NULL); /* (only one on i.MXL/1) */

	/* choose mode */
	scsr = SSI_SCSR_I2S_MODE_MSTR | SSI_SCSR_SYN;

	/* disable this unit */
	set_reg(scsr, SSI_SCSR, id);

	/* Transmit config */
	set_reg(SSI_MAE |	/* Activate DMA */
		SSI_FEN |	/* FIFO enabled */
		SSI_FDIR |	/* Frame sync internally generated */
		SSI_DIR |	/* Clock internally generated */
		SSI_SCKP |	/* send @falling edge */
		SSI_FSI |	/* Frame sync active low */
		SSI_EFS,	/* one bit shift */
		SSI_STCR, id);

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
	/* SSI FIFO watermark */
	set_reg(SSI_SFCSR_RFWM(8) | SSI_SFCSR_TFWM(TRANSMIT_WM_LEVEL), SSI_SFCSR, id);

	/*
	 * We want to provide I2S. This means we always need two words per
	 * sample signal. In this case 16 bit per word, 32 bit per sample.
	 * PMIC sends always 4 words (=2 samples) per sync!
	 */
// 	SSI_STCCR = 0x00006103;
// 	SSI_SRCCR = 0x00006103;

	/* enable the unit */
// 	SSI_SCSR = /*SSI_SYS_CLK_EN |*/ SSI_SCSR_I2S_MODE_MSTR | SSI_SCSR_SYN | /*SSI_TE |*/ SSI_SCSR_EN;

	return 0;
}

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

	pr_debug("%s\n", __func__);
	BUG_ON(unit[id].io == NULL);

	if (fifo < 0 || fifo > 1)	/* FIXME imx27 specific */
		return NULL;

	res = platform_get_resource(unit[id].pdev, IORESOURCE_MEM, 0);

	if (!res)
		return NULL;

	adr = (void __iomem *)res->start /*+ (fifo ? SSI_STX1 : SSI_STX0)*/;

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

	pr_debug("%s\n", __func__);
	BUG_ON(unit[id].io == NULL);

	if (fifo < 0 || fifo > 1)	/* FIXME imx27 specific */
		return NULL;

	res = platform_get_resource(unit[id].pdev, IORESOURCE_MEM, 0);

	if (!res)
		return NULL;

	adr = (void __iomem *)res->start /*+ (fifo ? SSI_SRX1 : SSI_SRX0)*/;

	return adr;
}
EXPORT_SYMBOL(imx_ssi_get_dma_rx_address);

int imx_ssi_prepare(struct snd_pcm_substream *substream,
                            int ssi_id)
{
	set_reg_bits(SSI_SCSR_EN, SSI_SCSR_EN, SSI_SCSR, ssi_id);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		set_reg_bits(SSI_MAE, SSI_MAE, SSI_STCR, ssi_id);
	else
		set_reg_bits(SSI_MAE, SSI_MAE, SSI_SRCR, ssi_id);

	return 0;
}
EXPORT_SYMBOL(imx_ssi_prepare);

void imx_ssi_shutdown(struct snd_pcm_substream *substream,
                             int ssi_id)
{
	set_reg_bits(SSI_SCSR_EN, 0, SSI_SCSR, ssi_id);

}

int imx_ssi_trigger(struct snd_pcm_substream *substream, int cmd, int ssi_id)
{
	u32 scsr = get_reg(SSI_SCSR, ssi_id);
	u32 temp;

	pr_debug("%s called\n", __func__);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		/* clear SSI underrun (needed ??) */
		temp = get_reg(SSI_SCSR, ssi_id);
		set_reg(0x0000, SSI_STX, ssi_id);
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			scsr |= SSI_SCSR_TE;
		else
			scsr |= SSI_SCSR_RE;
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			scsr &= ~SSI_SCSR_TE;
		else
			scsr &= ~SSI_SCSR_RE;
		break;

	default:
		return -EINVAL;
	}

	set_reg(scsr, SSI_SCSR, ssi_id);

	return 0;
}
EXPORT_SYMBOL(imx_ssi_trigger);

static int __devinit imx_ssi_probe(struct platform_device *pdev)
{
	struct imx_ssi_platform_data *pdata;
	struct resource *res;
	int ret, id;

	pdata = pdev->dev.platform_data;

	id = pdev->id;
	pr_info("Probing i.MX SSI unit %d\n", id);

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

static int __devexit imx_ssi_remove(struct platform_device *pdev)
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

static struct platform_driver imx_ssi_driver = {
	.driver = {
		.name		= DRV_NAME,
		.owner		= THIS_MODULE,
	},
	.probe		= imx_ssi_probe,
	.remove		= __devexit_p(imx_ssi_remove),
#ifdef CONFIG_PM
	.suspend	= NULL, /* TODO imx_ssi_suspend */
	.resume		= NULL	/* TODO imx_ssi_resume */
#endif
};

int __init imx_ssi_init(void)
{
	return platform_driver_register(&imx_ssi_driver);
}

void __exit imx_ssi_exit(void)
{
	platform_driver_unregister(&imx_ssi_driver);
}

module_init(imx_ssi_init);
module_exit(imx_ssi_exit);

MODULE_AUTHOR("Julien Boibessot, <julien.boibessot@armadeus.com>");
MODULE_DESCRIPTION("i.MX SSI Driver");
MODULE_LICENSE("GPL");
