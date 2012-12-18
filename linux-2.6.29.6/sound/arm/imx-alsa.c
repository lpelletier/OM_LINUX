/*
 * sound/arm/imx-alsa.c
 *
 * Alsa Driver for i.MX1/i.MX2 SSI
 *
 * Copyright (C) 2008 Armadeus Systems <nicolas.colombain@armadeus.com>
 *                                     <julien.boibessot@armadeus.com>
 * Based on omap-alsa.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#define DEBUG

#include <linux/platform_device.h>
#ifdef CONFIG_PM
#include <linux/pm.h>
#endif
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#include <mach/imx-alsa.h>
#include <linux/dma-mapping.h>

#ifdef CONFIG_ARCH_MX2
#include <mach/dma-mx1-mx2.h>
#include <mach/imx_sound.h>
#include "imx2-dam.h"
#else
#include <mach/imx-dma.h>
#endif
#include "imx-ssi.h"

#include "devdma.h"

#define DRIVER_NAME	"imx-pcm"
#define DRIVER_VERSION	"0.4"
#define TSC_MASTER


MODULE_AUTHOR("Nicolas Colombain / Julien Boibessot - Armadeus Systems");
MODULE_DESCRIPTION("i.MX1/L/2 PCM driver for ALSA");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imx-pcm");

static struct snd_card_imx_codec 	*alsa_codec		= NULL;
static struct imx_alsa_codec_config	*alsa_codec_config	= NULL;


/* Called by the DMA framework when a period has elapsed */
static void snd_imx_dma_progression(int channel, void *data,
					struct scatterlist *sg)
{
	struct audio_stream *s = data;
	struct snd_pcm_substream *substream = s->stream;
	struct snd_pcm_runtime *runtime;

	runtime = substream->runtime;

	pr_debug("DMA ");
	if (sg) {
/*		channel_info->offset = (unsigned long)sg->dma_address -
					(unsigned long)channel_info->base;*/
		s->periods++;
		s->periods %= runtime->periods;
		snd_pcm_period_elapsed(substream);
	}
}

/* Called when the DMA unit has finished the buffer.
 * It should not happen, but whith a playback or capturing
 * size of 2^32 bytes it will happen!
 */
static void snd_imx_dma_callback(int channel, void *data)
{
	struct audio_stream *s = data;
	struct snd_pcm_substream *substream = s->stream;

	pr_err("%s shouldn't be called\n", __func__);

	/* report period's end */
	snd_pcm_period_elapsed(substream);
}

/* Called by the DMA framework when an error has occured */
static void snd_imx_dma_err_handler(int channel, void *data, int err)
{
	pr_debug("%s %d %d\n", __func__, channel, err);

	printk("DMA timeout on channel %d -%s%s%s%s%s\n",
		 channel,
		 err & IMX_DMA_ERR_BURST ?    " burst":"",
		 err & IMX_DMA_ERR_REQUEST ?  " request":"",
		 err & IMX_DMA_ERR_TRANSFER ? " transfer":"",
		 err & IMX_DMA_ERR_BUFFER ?   " buffer":"",
		 err & IMX_DMA_ERR_TIMEOUT ?  " hw_chain_watchdog":"");

	imx_dma_disable(channel);
}

/* configure DMA channel of a given substream */
static int snd_imx_dma_request(struct audio_stream *s)
{
	int err=0, chan=0;

	chan = imx_dma_request_by_prio(DRIVER_NAME, DMA_PRIO_HIGH);
	if (chan < 0) {
		printk(KERN_ERR "Unable to grab a DMA channel\n");
		err = chan;
		goto on_error_1;
	}
	s->dma_dev = chan;

	err = imx_dma_setup_handlers(s->dma_dev,
					snd_imx_dma_callback,
					snd_imx_dma_err_handler, s);
	if (err < 0) {
		printk(KERN_ERR "Unable to setup DMA handler for channel %d\n", s->dma_dev);
		err = -EIO;
		goto on_error_2;
	}

	err = imx_dma_setup_progression_handler(s->dma_dev,
						snd_imx_dma_progression);
	if (err != 0) {
		pr_err("Failed to setup the DMA handler\n");
		err = -EIO;
		goto on_error_2;
	}

	pr_debug("snd_imx_dma_request done (%d)\n", s->dma_dev);
	imx_dma_disable(s->dma_dev);

	return 0;

on_error_2:
	imx_dma_free(s->dma_dev);
on_error_1:
	return err;
}

/*
 *  This function should calculate the current position of the dma in the
 *  buffer. It will help alsa middle layer to continue update the buffer.
 *  Its correctness is crucial for good functioning.
 */
static u_int audio_get_dma_pos(struct audio_stream *s)
{
	struct snd_pcm_substream *substream = s->stream;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned int offset;
	unsigned long flags;

	/* this must be called w/ interrupts locked as requested in dma.c */
	spin_lock_irqsave(&s->dma_lock, flags);

	/* For the current period let's see where we are */
	spin_unlock_irqrestore(&s->dma_lock, flags);

	/* Now, the position related to the end of that period */
// 	offset = bytes_to_frames(runtime, s->offset) + (runtime->period_size >> 1); //bytes_to_frames(runtime, count);
	if (s->dma_in_progress) {
		offset = (runtime->period_size * (s->periods)) + (runtime->period_size >> 1);
		if (offset >= runtime->buffer_size)
			offset = runtime->period_size >> 1;
	} else {
		offset = (runtime->period_size * (s->periods));
		if (offset >= runtime->buffer_size)
			offset = 0;
	}
/*	pr_debug("%s: Period=%d/%d Off=%d psize=%d bsize=%d (%d)\n", __func__,
		s->periods, runtime->periods, offset, runtime->period_size,
		runtime->buffer_size, bytes_to_frames(runtime, s->offset)); */

	return offset;
}

/*
 * Alsa section
 * PCM settings and callbacks
 */
static int snd_imx_alsa_trigger(struct snd_pcm_substream * substream, int cmd)
{
	struct snd_card_imx_codec *chip = snd_pcm_substream_chip(substream);
	int stream_id = substream->pstr->stream;
	struct audio_stream *s = &chip->s[stream_id];

	pr_debug("%s called: ", __func__);

	/* note: local interrupts are already disabled in the midlevel code */
	spin_lock(&s->dma_lock);
	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
			pr_debug("START\n");
			imx_dma_enable(s->dma_dev);
			break;

		case SNDRV_PCM_TRIGGER_STOP:
			pr_debug("STOP\n");
			imx_dma_disable(s->dma_dev);
			break;

		default:
			printk("ALSA trigger 0x%x not supported (yet?)\n", cmd);
			return -EINVAL;
			break;
	}
	spin_unlock(&s->dma_lock);

	/* SSI activation done here */
	return imx_ssi_trigger(substream, cmd, /*channel_info->ssi*/0);
}

static int snd_imx_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_card_imx_codec *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct audio_stream *channel_info = &chip->s[substream->pstr->stream];
// 	int stream_id = substream->pstr->stream;
	unsigned int dma_chan = channel_info->dma_dev;
	int err = -EINVAL;
	void __iomem *fifo_io;

	pr_debug("%s called\n", __func__);

	/* set requested samplerate */
	alsa_codec_config->codec_set_samplerate(runtime->rate);
	chip->samplerate = runtime->rate;

	channel_info->period = 0;
	channel_info->periods = 0;

#ifdef TSC_MASTER
	imx_ssi_setup_unit_to_iis_slave(0);
#else
	imx_ssi_setup_unit_to_iis_master(0);
#endif

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		fifo_io = imx_ssi_get_dma_tx_address(/*channel_info->ssi*/0, 0);
		err = imx_dma_setup_sg(dma_chan, channel_info->sg_list,
				       channel_info->sg_count,
				       -1, /* TODO largest possible size */
				       (unsigned int)fifo_io,
				       DMA_MODE_WRITE);
	}
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		fifo_io = imx_ssi_get_dma_rx_address(/*channel_info->ssi*/0, 0);
		err = imx_dma_setup_sg(dma_chan, channel_info->sg_list,
				       channel_info->sg_count,
				       -1, /* TODO largest possible size */
				       (unsigned int)fifo_io,
				       DMA_MODE_READ);
	}
	if (err < 0) {
		pr_err("Can't setup scatter/gather DMA\n");
		return err;
	}

	imx_ssi_prepare(substream, /*channel_info->ssi*/0);

	return 0;
}

static snd_pcm_uframes_t snd_imx_alsa_pointer(struct snd_pcm_substream *substream)
{
	struct snd_card_imx_codec *chip = snd_pcm_substream_chip(substream);

	return audio_get_dma_pos(&chip->s[substream->pstr->stream]);
}

static int snd_imx_pcm_open(struct snd_pcm_substream * substream)
{
	struct snd_card_imx_codec *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int stream_id = substream->pstr->stream;
	int err, dma_req;
	char *type;
	unsigned int burstlen;

	type = (stream_id == SNDRV_PCM_STREAM_PLAYBACK ? "Playback" : "Capture");
	pr_debug("%s %s stream @: 0x%x\n", __func__, type, (unsigned int)substream);

	chip->s[stream_id].stream = substream;
	alsa_codec_config->codec_clock_on();
	if (stream_id == SNDRV_PCM_STREAM_PLAYBACK) {
		runtime->hw = *(alsa_codec_config->snd_imx_alsa_playback);
		burstlen = SSI_TXFIFO_WM * 2; /* TODO: what if samples != 16 bits ? */
		dma_req = imx_ssi_get_dma_tx_channel(0, 0);
		pr_debug("imx_ssi_get_dma_tx_channel = %d\n", dma_req);
		if (dma_req < 0)
			return dma_req;
	} else {
		runtime->hw = *(alsa_codec_config->snd_imx_alsa_capture);
		burstlen = SSI_RXFIFO_WM * 2; /* TODO: what if samples != 16 bits ? */
		dma_req = imx_ssi_get_dma_rx_channel(0, 0);
		pr_debug("imx_ssi_get_dma_rx_channel = %d\n", dma_req);
		if (dma_req < 0)
			return dma_req;
	}
	if ((err = snd_pcm_hw_constraint_integer(runtime,
					   SNDRV_PCM_HW_PARAM_PERIODS)) < 0)
		return err;

	if ((err = snd_pcm_hw_constraint_list(runtime,
					0,
					SNDRV_PCM_HW_PARAM_RATE,
					alsa_codec_config->hw_constraints_rates)) < 0)
		return err;

	/* request DMA channel */
	err = snd_imx_dma_request(&chip->s[stream_id]);
	if (err < 0)
		return err;

	/* configure i.MX DMA control register for given channel */
	err = imx_dma_config_channel(chip->s[stream_id].dma_dev,
				IMX_DMA_MEMSIZE_16 | IMX_DMA_TYPE_FIFO,
				IMX_DMA_MEMSIZE_32 | IMX_DMA_TYPE_LINEAR,
				dma_req, 1);
	if (err < 0) {
		printk("Cannot configure DMA for %s channel\n", type);
	}

	/* configure DMA burst length for channel */
	imx_dma_config_burstlen(chip->s[stream_id].dma_dev, burstlen);
	pr_debug("Burstlen for DMA channel %u is now: %u\n", chip->s[stream_id].dma_dev, burstlen);

#ifdef CONFIG_ARCH_MX2
	/* route sound paths */
	mx2_dam_configure_sync_slave(1, 1);
#endif

	return 0;
}

static int snd_imx_pcm_close(struct snd_pcm_substream * substream)
{
	struct snd_card_imx_codec *chip = snd_pcm_substream_chip(substream);
	int stream_id = substream->pstr->stream;
	struct audio_stream *channel_info = &chip->s[substream->pstr->stream];

	ADEBUG();

	imx_dma_disable(chip->s[stream_id].dma_dev);
	imx_dma_free(chip->s[stream_id].dma_dev);

	alsa_codec_config->codec_clock_off();

	if (channel_info->sg_list != NULL)
		kfree(channel_info->sg_list);

	chip->s[substream->pstr->stream].stream = NULL;

	return 0;
}

/* HW params & free */
static int snd_imx_pcm_hw_params(struct snd_pcm_substream * substream,
				   struct snd_pcm_hw_params * hw_params)
{
	struct snd_card_imx_codec *chip = snd_pcm_substream_chip(substream);
	struct audio_stream *channel_info = &chip->s[substream->pstr->stream];
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned char *dma_adr;
	int err = -EINVAL, i;

	pr_debug("%s called\n", __func__);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		err = devdma_hw_alloc(chip->card->dev, substream,
					params_buffer_bytes(hw_params));
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		err = devdma_hw_alloc(chip->card->dev, substream,
					params_buffer_bytes(hw_params));
	if (err < 0)
		return err;

	pr_debug("DMA area: 0x%p, addr: 0x%p, size: %u\n",
		runtime->dma_area, (void*)runtime->dma_addr, runtime->dma_bytes);

	/* channel_info->base = (void*)runtime->dma_addr; ?? why isn't it already done ?? */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		channel_info->base = (void*)dma_map_single(NULL,
						(void*)runtime->dma_area,
						runtime->dma_bytes,
						DMA_TO_DEVICE);
	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		channel_info->base = (void*)dma_map_single(NULL,
						(void*)runtime->dma_area,
						runtime->dma_bytes,
						DMA_FROM_DEVICE);
	}
	pr_debug("DMA phys: 0x%p\n", channel_info->base);
	channel_info->adr = runtime->dma_area ;
	channel_info->offset = 0;
	channel_info->size = params_buffer_bytes(hw_params);
	channel_info->periods = params_periods(hw_params);
	channel_info->period = params_period_bytes(hw_params);

	pr_debug("Size: %d periods: %d period_size: %d\n", channel_info->size, channel_info->periods, channel_info->period);
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

	return snd_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(hw_params));
}

static int snd_imx_pcm_hw_free(struct snd_pcm_substream * substream)
{
	struct snd_card_imx_codec *chip = snd_pcm_substream_chip(substream);
	struct audio_stream *channel_info = &chip->s[substream->pstr->stream];
// 	struct snd_pcm_runtime *runtime = substream->runtime;

	pr_debug("%s called\n", __func__);

	if (channel_info->sg_list != NULL) {
		kfree(channel_info->sg_list);
		channel_info->sg_list = NULL;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		devdma_hw_free(chip->card->dev, substream);
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		devdma_hw_free(chip->card->dev, substream);

	return snd_pcm_lib_free_pages(substream);
}

/* pcm operations */
static struct snd_pcm_ops snd_card_imx_ops = {
	.open =		snd_imx_pcm_open,
	.close =	snd_imx_pcm_close,
	.ioctl =	snd_pcm_lib_ioctl,
	.hw_params =	snd_imx_pcm_hw_params,
	.hw_free =	snd_imx_pcm_hw_free,
	.prepare =	snd_imx_pcm_prepare,
	.trigger =	snd_imx_alsa_trigger,
	.pointer =	snd_imx_alsa_pointer,
};

/*
 *  Alsa init and exit section
 *
 *  Inits pcm alsa structures, allocate the alsa buffer, suspend, resume
 */
#define CHIP_NAME "i.MX SSI"
static int __init snd_imx_new_pcm(struct snd_card_imx_codec *imx_alsa,
					int device)
{
	struct snd_pcm *pcm;
	int err;

	err = snd_pcm_new(imx_alsa->card,
			  CHIP_NAME,
			  device,
			  1,	/* one playback stream */
			  1,	/* one capture stream */
			  &pcm);

	if (err < 0)
		return err;

	/* sets up initial buffer with continuous allocation */
	snd_pcm_lib_preallocate_pages_for_all(pcm,
				SNDRV_DMA_TYPE_CONTINUOUS,
				snd_dma_continuous_data(GFP_KERNEL),
				32 * 1024,
				32 * 1024);

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &snd_card_imx_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &snd_card_imx_ops);
	pcm->private_data = imx_alsa;
	pcm->info_flags = 0;
	strcpy(pcm->name, CHIP_NAME);

	imx_alsa->pcm = pcm;

	return 0;
}


#ifdef CONFIG_PM
int snd_imx_alsa_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_card_imx_codec *chip;
	struct snd_card *card = platform_get_drvdata(pdev);

	if (card->power_state != SNDRV_CTL_POWER_D3hot) {
		chip = card->private_data;
		if (chip->card->power_state != SNDRV_CTL_POWER_D3hot) {
			snd_power_change_state(chip->card, SNDRV_CTL_POWER_D3hot);
			snd_pcm_suspend_all(chip->pcm);
			/* Mutes and turn clock off */
			alsa_codec_config->codec_clock_off();
			snd_tsc210x_suspend_mixer();
		}
	}

	return 0;
}

int snd_imx_alsa_resume(struct platform_device *pdev)
{
	struct snd_card_imx_codec *chip;
	struct snd_card *card = platform_get_drvdata(pdev);

	if (card->power_state != SNDRV_CTL_POWER_D0) {
		chip = card->private_data;
		if (chip->card->power_state != SNDRV_CTL_POWER_D0) {
			snd_power_change_state(chip->card, SNDRV_CTL_POWER_D0);
			alsa_codec_config->codec_clock_on();
			snd_tsc210x_resume_mixer();
		}
	}

	return 0;
}
#endif	/* CONFIG_PM */


/*
 * Inits alsa soundcard structure.
 * Called by the probe method in codec after function pointers has been set.
 */
int snd_imx_alsa_post_probe(struct platform_device *pdev, struct imx_alsa_codec_config *config)
{
	int err = 0;
	struct snd_card *card;
	struct imx_sound_platform_data *pdata;

	pr_debug("%s\n", __func__);

	pdata = pdev->dev.platform_data;
	if (pdata == NULL) {
		pr_err("No platform data available. Giving up\n");
		err = -ENODEV;
		goto nodev1;
	}

	alsa_codec_config = config;
	if (alsa_codec_config == NULL) {
		pr_err("No config data available. Giving up\n");
		err = -ENODEV;
		goto nodev1;
	}
	alsa_codec_config->codec_clock_setup();
	alsa_codec_config->codec_clock_on();

	if (alsa_codec_config->codec_configure_dev)
		alsa_codec_config->codec_configure_dev();

	alsa_codec_config->codec_clock_off();

	/* instantiate a new soundcard */
	card = snd_card_new(-1, "IMX-ALSA", THIS_MODULE, sizeof(*alsa_codec));
	if (card == NULL)
		goto nodev1;
	alsa_codec = card->private_data;
	alsa_codec->card = card;

	alsa_codec->samplerate = alsa_codec_config->get_default_samplerate();
	spin_lock_init(&alsa_codec->s[0].dma_lock);
	spin_lock_init(&alsa_codec->s[1].dma_lock);

	/* add it a mixer */
	if ((err = snd_imx_mixer(alsa_codec)) < 0)
		goto nodev2;

	/* add it a PCM interface */
	if ((err = snd_imx_new_pcm(alsa_codec, 0)) < 0)
		goto nodev2;

	strcpy(card->driver, "i.MX");
	sprintf(card->shortname, "i.MX+%s audio", alsa_codec_config->name);
	sprintf(card->longname, "Freescale i.MX with %s codec", alsa_codec_config->name);

	snd_card_set_dev(card, &pdev->dev);

	/* register the created soundcard */
	err = snd_card_register(card);
	if (err < 0) {
		pr_err("Cannot register sound card. Giving up\n");
		goto nodev2;
	}

	pr_info("i.MX1/L/2 audio support v" DRIVER_VERSION " initialized\n");
	platform_set_drvdata(pdev, card);

	/* activate the external SSI pins (should be done in imx-ssi !?) */
	if (pdata->init)
		pdata->init(pdev);

	return 0;

nodev2:
	snd_card_free(card);
nodev1:
	return err;
}

int snd_imx_alsa_remove(struct platform_device *pdev)
{
	struct snd_card *card = platform_get_drvdata(pdev);
	struct snd_card_imx_codec *chip = card->private_data;

	snd_card_free(card);

	alsa_codec = NULL;
	card->private_data = NULL;
	kfree(chip);

	platform_set_drvdata(pdev, NULL);

	return 0;
}
