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
 * Note:
 * This driver works very close to the MC13783 (aka PMIC). There are so many
 * dependencies between the i.MX27 SSI and the PMIC and other strange behaviors
 * on both side, that it is very hard to bring them into a layerd design, where
 * one layer do not know the requirements of the other layer.
 *
 * Currently only filesizes up to 2^32 are supported to play and record at
 * once. This would change, if the DMA API would change.
 */

#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/dma-mapping.h>
#include <linux/soundcard.h>
#ifdef CONFIG_PM
#include <linux/pm.h>
#endif /* CONFIG_PM */

#include <mach/dma.h>
#include <mach/dma-mx1-mx2.h>
#include <mach/imx_sound.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/control.h>

#include "devdma.h"
#include "mc13783.h"
#include "imx2-dam.h"
#include "imx2-ssi.h"

#define DRV_NAME "imx-alsa"

/*
 * PMIC driver buffer policy.
 * Customize here if the sound is not correct
 */
#define MAX_BUFFER_SIZE  			(64 * 1024)
#define DMA_BUF_SIZE				(16 * 1024)
#define MIN_PERIOD_SIZE				128
#define MIN_PERIOD				2
#define MAX_PERIOD				255

#define SOUND_CARD_NAME				"imx sound"

#define PLAYBACK 0
#define RECORD 1

/* runtime info on a per stream base */
struct imx_sound_channel {
	int dmac;	/* DMA channel handle */
	struct device *dev;
	int ssi;	/* iss channel to be used for this stream (0 or 1) */
	int tgt_bus;	/* target's bus number (0 or 1) at PMIC side */

	/* DMA buffer description */
	void *base;		/* physical address of the DMA buffer in memory */
	void *adr;		/* virtual address of the DMA buffer */
	int size;		/* buffer size */
	int sg_count;
	struct scatterlist *sg_list;

	/* stream information */
	int period;		/* bytes per period */
	int periods;		/* count of periods in this buffer */

	/* consumed data information */
	unsigned int offset;	/* current pointer (=period) where the DMA unit reads from or writes to */

	struct snd_pcm_substream *substream;
};

struct imx_snd_card {
	struct snd_card *card;
	struct snd_pcm *pcm;
	int ssi_unit;	/* internal SSI unit (0 or 1) */
	int tgt_bus;	/* PMIC's bus number (0 or 1) */
};

/* ################### internals ####################### */

/* Called by the DMA framework when a period has elapsed */
static void snd_imx_dma_progression(int channel, void *data,
					struct scatterlist *sg)
{
	struct imx_sound_channel *channel_info = data;

	if (sg) {
		channel_info->offset = (unsigned long)sg->dma_address -
					(unsigned long)channel_info->base;
		snd_pcm_period_elapsed(channel_info->substream);
	}
}

/* called when the DMA unit has finished the buffer.
 * it should not happen, but whith a playback or capturing
 * size of 2^32 bytes it will happen!
 */
static void snd_imx_dma_callback(int channel, void *data)
{
	struct imx_sound_channel *channel_info = data;

	pr_err("%s shouldn't be called\n", __func__);

	/* report period's end */
	snd_pcm_period_elapsed(channel_info->substream);
}

static void snd_imx_dma_err_callback(int channel, void *data, int err)
{
	struct imx_sound_channel *channel_info = data;

	pr_debug("DMA error callback called\n");

	pr_debug("DMA timeout on channel %d -%s%s%s%s\n",
		 channel_info->dmac,
		 err & IMX_DMA_ERR_BURST ?    " burst":"",
		 err & IMX_DMA_ERR_REQUEST ?  " request":"",
		 err & IMX_DMA_ERR_TRANSFER ? " transfer":"",
		 err & IMX_DMA_ERR_BUFFER ?   " buffer":"");
}

/* ################### ALSA generic ####################### */

static int snd_imx_pcm_mmap(struct snd_pcm_substream *substream, struct vm_area_struct *vma)
{
	return devdma_mmap(NULL, substream, vma);
}

static snd_pcm_uframes_t snd_imx_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct imx_sound_channel *channel_info = runtime->private_data;

	return bytes_to_frames(substream->runtime,
		channel_info->offset == channel_info->size ? 0 : channel_info->offset);
}

/*
 * This is called when the hardware parameter (hw_params) are set up by the
 * application, that is, once when the buffer size, the period size, the format,
 * etc. are defined for the pcm substream.
 */
static int snd_imx_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *hw_params)
{
	struct imx_snd_card *imx_card = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct imx_sound_channel *channel_info = runtime->private_data;
	unsigned char *dma_adr;
	int err = -EINVAL, i;

	pr_debug("%s called\n", __func__);

	err = mc13783_pcm_hw_params(substream, hw_params);
	if (err)
		return err;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		err = devdma_hw_alloc(imx_card[PLAYBACK].card->dev, substream,
					params_buffer_bytes(hw_params));
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		err = devdma_hw_alloc(imx_card[RECORD].card->dev, substream,
					params_buffer_bytes(hw_params));
	if (err < 0)
		return err;

	pr_debug("DMA area: 0x%p, addr: 0x%p, size: %u\n",
		runtime->dma_area, (void*)runtime->dma_addr, runtime->dma_bytes);

	channel_info->base = (void*)runtime->dma_addr;
	channel_info->adr = runtime->dma_area ;
	channel_info->offset = 0;
	channel_info->size = params_buffer_bytes(hw_params);
	channel_info->periods = params_periods(hw_params);
	channel_info->period = params_period_bytes(hw_params);

	dma_adr = channel_info->base;

	if (channel_info->sg_count != channel_info->periods) {
		if (channel_info->sg_list)
			kfree(channel_info->sg_list);
		/* using sg_alloc_table() instead? */
		channel_info->sg_list = kcalloc(channel_info->periods + 1,
				sizeof(struct scatterlist), GFP_KERNEL);
		pr_debug("Allocating scatter/gather list with %d+1 entries\n",
				channel_info->periods);
		if (!channel_info->sg_list)
			return -ENOMEM;
		channel_info->sg_count = channel_info->periods + 1;
	} else
		pr_debug("Reuse previous scatter/gather memory\n");

	sg_init_table(channel_info->sg_list, channel_info->sg_count);

	for (i = 0; i < channel_info->periods; i++) {
		channel_info->sg_list[i].page_link = 0;
		channel_info->sg_list[i].offset = 0;	/* FIXME */
		channel_info->sg_list[i].dma_address = (dma_addr_t)dma_adr;
		channel_info->sg_list[i].length = channel_info->period;

		dma_adr += channel_info->period;
	}

	/* close the loop */
	sg_chain(channel_info->sg_list, channel_info->sg_count, channel_info->sg_list);

	pr_debug("Activating %d periods with %d bytes each\n",
			channel_info->periods, channel_info->period);

	return 0;
}

static int snd_imx_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct imx_snd_card *imx_card = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct imx_sound_channel *channel_info = runtime->private_data;

	pr_debug("%s called\n", __func__);

	if (channel_info->sg_list != NULL) {
		kfree(channel_info->sg_list);
		channel_info->sg_list = NULL;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		devdma_hw_free(imx_card[PLAYBACK].card->dev, substream);
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		devdma_hw_free(imx_card[RECORD].card->dev, substream);

	return 0;
}

static int snd_imx_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct imx_sound_channel *channel_info = runtime->private_data;
	int err = -EINVAL;
	void __iomem *fifo_io;

	pr_debug("%s called\n", __func__);

	setup_channel_to_iis_slave(channel_info->ssi);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		fifo_io = imx_get_dma_tx_address(channel_info->ssi, 0);
		err = imx_dma_setup_sg(channel_info->dmac, channel_info->sg_list,
				       channel_info->sg_count,
				       -1, /* TODO largest possible size */
				       (unsigned int)fifo_io,
				       DMA_MODE_WRITE);
	}
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		fifo_io = imx_get_dma_rx_address(channel_info->ssi, 0);
		err = imx_dma_setup_sg(channel_info->dmac, channel_info->sg_list,
				       channel_info->sg_count,
				       -1, /* TODO largest possible size */
				       (unsigned int)fifo_io,
				       DMA_MODE_READ);
	}
	if (err < 0) {
		pr_err("Can't setup scatter/gather DMA\n");
		return err;
	}

	imx_ssi_prepare(substream, channel_info->ssi);

	return 0;
}

static int snd_imx_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct imx_sound_channel *channel_info = runtime->private_data;

	pr_debug("%s called\n", __func__);

	imx_ssi_shutdown(substream, channel_info->ssi);

	imx_dma_disable(channel_info->dmac);
	imx_dma_free(channel_info->dmac);

	if (channel_info->sg_list != NULL)
		kfree(channel_info->sg_list);

	kfree(channel_info);
	runtime->private_data = NULL;

	return 0;
}

static int snd_imx_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct imx_sound_channel *channel_info = runtime->private_data;

	pr_debug("%s called\n", __func__);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		pr_debug("IMX: SNDRV_PCM_TRIGGER_START\n");

		imx_dma_enable(channel_info->dmac);

		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		pr_debug("IMX: SNDRV_PCM_TRIGGER_STOP\n");
		imx_dma_disable(channel_info->dmac);
	default:
		return -EINVAL;
	}

	return imx_ssi_trigger(substream, cmd, channel_info->ssi);
}

/* ########### ALSA API ##################### */

#define IMX_PLAYBACK_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | \
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | \
		SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
		SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_64000 | \
		SNDRV_PCM_RATE_96000)

#define IMX_CAPTURE_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000)

/* capabilities of the PMIC */
static struct snd_pcm_hardware snd_imx_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_PAUSE |
		SNDRV_PCM_INFO_RESUME,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rate_min = 8000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = MAX_BUFFER_SIZE,
	.period_bytes_min = MIN_PERIOD_SIZE,
	.period_bytes_max = DMA_BUF_SIZE,
	.periods_min = MIN_PERIOD,
	.periods_max = MAX_PERIOD,
	.fifo_size = 0,
};

static int snd_imx_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct imx_snd_card *imx_card = snd_pcm_substream_chip(substream);
	struct imx_sound_channel *channel_info;
	int channel, burst_length;
	int err, dma_req;

	pr_debug("%s called\n", __func__);

	channel_info = kzalloc(sizeof(struct imx_sound_channel), GFP_KERNEL);
	if (channel_info == NULL)
		return -ENOMEM;

	runtime->private_data = channel_info;
	channel_info->substream = substream;

	runtime->hw = snd_imx_hardware;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		channel = PLAYBACK;
		runtime->hw.rates = IMX_PLAYBACK_RATES;
		runtime->hw.rate_max = 96000;
		burst_length = DMA_TXFIFO_BURST * 2;
		dma_req = imx_get_dma_tx_channel(imx_card[channel].ssi_unit, 0);	/* for FIFO TX0 */
		if (dma_req < 0)
			return dma_req;
	} else {
		channel = RECORD;
		runtime->hw.rates = IMX_CAPTURE_RATES;
		runtime->hw.rate_max = 16000;
		burst_length = DMA_RXFIFO_BURST * 2;
		dma_req = imx_get_dma_rx_channel(imx_card[channel].ssi_unit, 0);	/* for FIFO TX0 */
		if (dma_req < 0)
			return dma_req;
	}

	channel_info->dev = imx_card[channel].card->dev;

	/* FIXME Approx interrupts per buffer */
	err = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	if (err < 0)
		goto on_error_1;

	/* setup DMA controller for playback */
	channel_info->dmac = imx_dma_request_by_prio(DRV_NAME, DMA_PRIO_HIGH);
	if (channel_info->dmac < 0) {
		pr_err("Failed to claim the audio DMA\n");
		goto on_error_1;
	}

	pr_debug("Got DMA channel %u for usage\n", channel_info->dmac);

	err = imx_dma_setup_handlers(channel_info->dmac,
				snd_imx_dma_callback,
				snd_imx_dma_err_callback, channel_info);
	if (err != 0) {
		pr_err("Failed to setup the DMA handler\n");
		err = -EIO;
		goto on_error_2;
	}

	err = imx_dma_setup_progression_handler(channel_info->dmac, snd_imx_dma_progression);
	if (err != 0) {
		pr_err("Failed to setup the DMA handler\n");
		err = -EIO;
		goto on_error_2;
	}

	channel_info->ssi = imx_card[channel].ssi_unit;
	channel_info->tgt_bus = imx_card[channel].tgt_bus;

	/*
	 * we are working with 16 bit words. So we must write
	 * each sample as two 16 bit words.
	 */
	err = imx_dma_config_channel(channel_info->dmac,
			IMX_DMA_MEMSIZE_16 | IMX_DMA_TYPE_FIFO,
			IMX_DMA_MEMSIZE_32 | IMX_DMA_TYPE_LINEAR,
			dma_req, 1);
	if (err < 0) {
		pr_debug("Cannot configure DMA channel\n");
		goto on_error_2;
	}

	imx_dma_config_burstlen(channel_info->dmac, burst_length);

	pr_debug("Burstlen for DMA channel %u is now: %u\n", channel_info->dmac, burst_length);

	return 0;

on_error_2:
	imx_dma_free(channel_info->dmac);
on_error_1:
	kfree(channel_info);
	return err;
}

static struct snd_pcm_ops snd_card_imx_ops = {
	.open = snd_imx_open,
	.close = snd_imx_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = snd_imx_pcm_hw_params,
	.hw_free = snd_imx_pcm_hw_free,
	.prepare = snd_imx_pcm_prepare,
	.trigger = snd_imx_pcm_trigger,
	.pointer = snd_imx_pcm_pointer,
	.mmap    = snd_imx_pcm_mmap
};

/* ##################################################################### */

static int __devinit snd_imx_pcm_streams(struct imx_snd_card *imx_sound, int device)
{
	struct snd_pcm *pcm;
	int err;

	err = snd_pcm_new(imx_sound->card,
			  SOUND_CARD_NAME,
			  device,
			  1,	/* one playback stream possible */
			  1,	/* one capture stream possible */
			  &pcm);

	if (err < 0)
		return err;

	/* one substream instance is to playback some noise */
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
			&snd_card_imx_ops);

	/* one substream instance is to capture some noise */
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
			&snd_card_imx_ops);

	pcm->private_data = imx_sound;
	imx_sound->pcm = pcm;
	pcm->info_flags = 0;
	strncpy(pcm->name, SOUND_CARD_NAME, sizeof(pcm->name));

	return 0;
}

static int __devinit snd_imx_probe(struct platform_device *pdev)
{
	int err, i;
	struct snd_card *card;
	struct imx_snd_card *imx_card;
	struct imx_sound_platform_data *pdata;

	/* register the soundcard */
	card = snd_card_new(-1, "PMICaudio", THIS_MODULE, 2 * sizeof(struct imx_snd_card));
	if (!card)
		return -ENOMEM;

	pdata = pdev->dev.platform_data;
	if (pdata == NULL) {
		pr_err("No platformdata available. Giving up\n");
		err = -ENODEV;
		goto nodev;
	}

	imx_card = (struct imx_snd_card*)&card[1];

	card->private_data = imx_card;

	card->dev = &pdev->dev;

	/* FIXME: Where to know how many SSI units we should use here? */

	imx_card[PLAYBACK].ssi_unit = 0;
	imx_card[PLAYBACK].tgt_bus = pdata->connection[0].dev_port - 1;
	imx_card[PLAYBACK].card = card;

	imx_card[RECORD].ssi_unit = 1;
	imx_card[RECORD].tgt_bus = pdata->connection[1].dev_port - 1;
	imx_card[RECORD].card = card;

	err = snd_imx_pcm_streams(imx_card, 0);	/* TODO ...the ID */
	if (err < 0)
		goto nodev;

	strcpy(card->driver, "MXC");
	strcpy(card->shortname, "PMIC-audio");
	sprintf(card->longname, "MXC Freescale with PMIC");

	err = snd_card_register(card);
	if (err < 0) {
		pr_err("Cannot register sound card. Giving up\n");
		goto nodev;
	}

	platform_set_drvdata(pdev, card);

	err = mc13783_add_ctl(card, NULL);
	if (err < 0 ) {
		pr_err("mc13783_add_ctl() failed with %d!\n", err);
		goto nodev;
	}

	/* route the SSI unit signals and activate the external SSI pins */
	if (pdata->init)
		pdata->init(pdev);

	for (i = 0; i < IMX_SUPPORTED_SSI_UNITS; i++) {
		if (pdata->connection[i].cpu_port &&
				pdata->connection[i].dev_port)
			dam_configure_sync_slave(imx_card[i].ssi_unit + 1,
				pdata->connection[i].cpu_port);
	}

	return 0;

nodev:
	snd_card_free(card);
	return err;
}

static int __devexit snd_imx_remove(struct platform_device *pdev)
{
	struct snd_card *card = platform_get_drvdata(pdev);

	snd_card_free(card);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver imx_sound_driver = {
	.driver = {
		.name = DRV_NAME,
	},
	.probe = snd_imx_probe,
	.remove = __devexit_p(snd_imx_remove),
#ifdef CONFIG_PM
	.suspend = NULL, /* TODO snd_mxc_audio_suspend */
	.resume = NULL, /* TODO snd_mxc_audio_resume */
#endif
};


static int __init imx_sound_init(void)
{
	int rc;

	rc = mx2_dam_init();	/* activate DAM related driver part */
	if (rc != 0)
		return rc;
	rc = mx2_ssi_init();	/* activate SSI related driver part */
	if (rc != 0)
		return rc;
	return platform_driver_register(&imx_sound_driver);
}

static void __exit imx_sound_exit(void)
{
	platform_driver_unregister(&imx_sound_driver);
	mx2_ssi_exit();
	mx2_dam_exit();
}

module_init(imx_sound_init);
module_exit(imx_sound_exit);

MODULE_AUTHOR("FREESCALE SEMICONDUCTOR");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MXC driver for ALSA");
