/*
 * sound/arm/imx-alsa-tsc2102.c
 *
 * Alsa codec driver for TSC210x chip on i.MX platforms.
 *
 * Copyright (c) 2008 Jorasse  <jorasse@armadeus.com>
 * Code based on the TSC2101 ALSA driver for omap platforms.
 * Copyright (c) 2006 Andrzej Zaborowski  <balrog@zabor.org>
 * Code based on the TSC2101 ALSA driver.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/delay.h>
#include <linux/soundcard.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/slab.h>
#ifdef CONFIG_PM
# include <linux/pm.h>
#endif
#include <linux/spi/tsc2102.h>
#ifdef CONFIG_DEBUG_FS
# include <linux/debugfs.h>
# include <linux/seq_file.h>
#endif

#include <mach/hardware.h>
#include <mach/imx-alsa.h>

#include "imx-alsa-tsc2102.h"

static struct clk *tsc2102_bclk = 0;

/*
 * Hardware capabilities
 */

/* DAC sampling rates (BCLK = 12 MHz) */
static unsigned int rates[] = {
	7350, 8000, 8820, 9600, 11025, 12000, 14700,
	16000, 22050, 24000, 29400, 32000, 44100, 48000,
};

static struct snd_pcm_hw_constraint_list tsc2102_hw_constraints_rates = {
	.count = ARRAY_SIZE(rates),
	.list = rates,
	.mask = 0,
};

#define IMX_SSI_RATES \
		(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | \
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | \
		SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
		SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_KNOT)

static struct snd_pcm_hardware tsc210x_snd_imx_alsa_playback = {
	.info			= SNDRV_PCM_INFO_INTERLEAVED |
					SNDRV_PCM_INFO_BLOCK_TRANSFER | SNDRV_PCM_INFO_MMAP |
					SNDRV_PCM_INFO_MMAP_VALID,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.rates			= IMX_SSI_RATES,
	.rate_min		= 8000,
	.rate_max		= 48000,
	.channels_min		= 2,
	.channels_max		= 2,
	.buffer_bytes_max	= 32 * 1024,
	.period_bytes_min	= 64,
	.period_bytes_max	= 8 * 1024,
	.periods_min		= 2,
	.periods_max		= 255,
	.fifo_size		= 0,
};

/* Only TSC2101 has (mono) audio in capability */
static struct snd_pcm_hardware tsc2101_snd_imx_alsa_capture = {
	.info			= SNDRV_PCM_INFO_INTERLEAVED |
					SNDRV_PCM_INFO_BLOCK_TRANSFER | SNDRV_PCM_INFO_MMAP |
					SNDRV_PCM_INFO_MMAP_VALID,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.rates			= IMX_SSI_RATES,
	.rate_min		= 8000,
	.rate_max		= 48000,
	.channels_min		= 2,	/* even if TSC is sampling mono input, it uses the 2 I2S channels to transmit data */
	.channels_max		= 2,
	.buffer_bytes_max	= 32 * 1024,
	.period_bytes_min	= 64,
	.period_bytes_max	= 8 * 1024,
	.periods_min		= 2,
	.periods_max		= 255,
	.fifo_size		= 0,
};

/*
 * ALSA operations according to board file
 */

static long current_rate = 0;

/*
 * Sample rate changing
 */
static void tsc2102_set_samplerate(long sample_rate)
{
	pr_debug("%s %li\n", __func__, sample_rate);

#ifndef TSC_MASTER
	int clkgdv = 0;
	u16 srgr1, srgr2;
#endif

	if (sample_rate == current_rate)
		return;
	current_rate = 0;

#if 0
	/* Wait for any frames to complete */
	udelay(125);
#endif
	/* Set the sample rate on the TSC */
	if (tsc210x_set_rate(sample_rate)) {
		printk("error setting TSC at %u Hz sample rate\n", (unsigned int)sample_rate);
		return;
	}

#ifdef TSC_MASTER

#else
	/* Set the sample rate on the i.MX SSI */
	printk("erreur 1");
	imx_get_perclk3();
	clkgdv = CODEC_CLOCK / (sample_rate * (DEFAULT_BITPERSAMPLE * 2 - 1));
	if (clkgdv)
		srgr1 = (FWID(DEFAULT_BITPERSAMPLE - 1) | CLKGDV(clkgdv));
	else
		return;
	xxxxx
	/* Stereo Mode */
	srgr2 = (CLKSM | FSGM | FPER(DEFAULT_BITPERSAMPLE * 2 - 1));
#endif

	current_rate = sample_rate;
}

static void tsc2102_configure(void)
{
	pr_debug("%s\n", __func__);

	tsc210x_dac_power(1);

#ifdef TSC_MASTER
	tsc210x_set_i2s_master(1);
#else
#error "to be implemented"
	tsc210x_set_i2s_master(0);
#endif
}

/*
 * Do clock framework bclk search
 */
static void tsc2102_clock_setup(void)
{
	/*tsc2102_bclk = clk_get(0, "bclk"); no clock management*/
}

/*
 * Do some sanity checks, set clock rate, start it.
 */
static int tsc2102_clock_on(void)
{
/*	int err;
no clock management*/

#ifdef TSC_MASTER

#endif

/*	if (clk_get_usecount(tsc2102_bclk) > 0 &&
			clk_get_rate(tsc2102_bclk) != CODEC_CLOCK) {
		printk(KERN_WARNING
			"BCLK already in use at %d Hz. We change it to %d Hz\n",
			(uint) clk_get_rate(tsc2102_bclk), CODEC_CLOCK);

		err = clk_set_rate(tsc2102_bclk, CODEC_CLOCK);
		if (err) {
			printk(KERN_WARNING "Cannot set BCLK clock rate "
				"for TSC2102 codec, error code = %d\n", err);
		}
	}

	clk_enable(tsc2102_bclk);
no clock management*/

	/* Clock disabled when SSI disabled */
	//?? SSI_SOR |= 0x40;

	return 0;
}

/*
 * Turn off the audio codec and then stop the clock.
 */
static int tsc2102_clock_off(void)
{
	DPRINTK("clock use count = %d\n", clk_get_usecount(tsc2102_bclk));

/*	clk_disable(tsc2102_bclk);
no clock management */

	/* Clock enabled when SSI disabled */
	//?? SSI_SOR &= ~0x40;

	return 0;
}

static int tsc2102_get_default_samplerate(void)
{
	return DEFAULT_SAMPLE_RATE;
}


#ifdef CONFIG_PM
static int snd_imx_alsa_tsc2102_suspend(struct platform_device *pdev,
			pm_message_t state)
{
	tsc210x_dac_power(0);
	current_rate = 0;

	return snd_imx_alsa_suspend(pdev, state);
}

static int snd_imx_alsa_tsc2102_resume(struct platform_device *pdev)
{
	tsc210x_dac_power(1);

#ifdef TSC_MASTER
	tsc210x_set_i2s_master(1);
#else
	tsc210x_set_i2s_master(0);
#endif

	return snd_imx_alsa_resume(pdev);
}
#endif /* CONFIG_PM */


#ifdef CONFIG_DEBUG_FS
static int tsc210x_debug_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "TSC210x registers content:\n");

	seq_printf(s, "TSC2102_AUDIO1_CTRL = 0x%04x\n",
			tsc210x_read_reg(TSC2102_AUDIO1_CTRL));
	seq_printf(s, "TSC210X_DAC_GAIN_CTRL = 0x%04x\n",
			tsc210x_read_reg(TSC210X_DAC_GAIN_CTRL));

	if (tsc210x_is_tsc2101()) {
		seq_printf(s, "TSC2101_HEADSET_AUX_PGA_CTRL = 0x%04x\n",
			tsc210x_read_reg(TSC2101_HEADSET_AUX_PGA_CTRL));
		seq_printf(s, "TSC2101_MIXER_PGA_CTRL = 0x%04x\n",
			tsc210x_read_reg(TSC2101_MIXER_PGA_CTRL));
	}
	seq_printf(s, "TSC2102_AUDIO2_CTRL = 0x%04x\n",
			tsc210x_read_reg(TSC2102_AUDIO2_CTRL));
	seq_printf(s, "TSC210X_CODEC_POWER_CTRL = 0x%04x\n",
			tsc210x_read_reg(TSC210X_CODEC_POWER_CTRL));
	seq_printf(s, "TSC2102_AUDIO3_CTRL = 0x%04x\n",
			tsc210x_read_reg(TSC2102_AUDIO3_CTRL));

	seq_printf(s, "TSC2102_LCH_BASS_BOOST_N0 = 0x%04x\n",
			tsc210x_read_reg(TSC2102_LCH_BASS_BOOST_N0));
	seq_printf(s, "TSC2102_LCH_BASS_BOOST_N1 = 0x%04x\n",
			tsc210x_read_reg(TSC2102_LCH_BASS_BOOST_N1));
	seq_printf(s, "TSC2102_LCH_BASS_BOOST_N2 = 0x%04x\n",
			tsc210x_read_reg(TSC2102_LCH_BASS_BOOST_N2));
	seq_printf(s, "TSC2102_LCH_BASS_BOOST_N3 = 0x%04x\n",
			tsc210x_read_reg(TSC2102_LCH_BASS_BOOST_N3));
	seq_printf(s, "TSC2102_LCH_BASS_BOOST_N4 = 0x%04x\n",
			tsc210x_read_reg(TSC2102_LCH_BASS_BOOST_N4));
	seq_printf(s, "TSC2102_LCH_BASS_BOOST_N5 = 0x%04x\n",
			tsc210x_read_reg(TSC2102_LCH_BASS_BOOST_N5));
	seq_printf(s, "TSC2102_LCH_BASS_BOOST_D1 = 0x%04x\n",
			tsc210x_read_reg(TSC2102_LCH_BASS_BOOST_D1));
	seq_printf(s, "TSC2102_LCH_BASS_BOOST_D2 = 0x%04x\n",
			tsc210x_read_reg(TSC2102_LCH_BASS_BOOST_D2));
	seq_printf(s, "TSC2102_LCH_BASS_BOOST_D4 = 0x%04x\n",
			tsc210x_read_reg(TSC2102_LCH_BASS_BOOST_D4));
	seq_printf(s, "TSC2102_LCH_BASS_BOOST_D5 = 0x%04x\n",
			tsc210x_read_reg(TSC2102_LCH_BASS_BOOST_D5));
	seq_printf(s, "TSC2102_RCH_BASS_BOOST_N0 = 0x%04x\n",
			tsc210x_read_reg(TSC2102_RCH_BASS_BOOST_N0));
	seq_printf(s, "TSC2102_RCH_BASS_BOOST_N1 = 0x%04x\n",
			tsc210x_read_reg(TSC2102_RCH_BASS_BOOST_N1));
	seq_printf(s, "TSC2102_RCH_BASS_BOOST_N2 = 0x%04x\n",
			tsc210x_read_reg(TSC2102_RCH_BASS_BOOST_N2));
	seq_printf(s, "TSC2102_RCH_BASS_BOOST_N3 = 0x%04x\n",
			tsc210x_read_reg(TSC2102_RCH_BASS_BOOST_N3));
	seq_printf(s, "TSC2102_RCH_BASS_BOOST_N4 = 0x%04x\n",
			tsc210x_read_reg(TSC2102_RCH_BASS_BOOST_N4));
	seq_printf(s, "TSC2102_RCH_BASS_BOOST_N5 = 0x%04x\n",
			tsc210x_read_reg(TSC2102_RCH_BASS_BOOST_N5));
	seq_printf(s, "TSC2102_RCH_BASS_BOOST_D1 = 0x%04x\n",
			tsc210x_read_reg(TSC2102_RCH_BASS_BOOST_D1));
	seq_printf(s, "TSC2102_RCH_BASS_BOOST_D2 = 0x%04x\n",
			tsc210x_read_reg(TSC2102_RCH_BASS_BOOST_D2));
	seq_printf(s, "TSC2102_RCH_BASS_BOOST_D4 = 0x%04x\n",
			tsc210x_read_reg(TSC2102_RCH_BASS_BOOST_D4));
	seq_printf(s, "TSC2102_RCH_BASS_BOOST_D5 = 0x%04x\n",
			tsc210x_read_reg(TSC2102_RCH_BASS_BOOST_D5));

	seq_printf(s, "TSC210X_PLL1_CTRL = 0x%04x\n",
			tsc210x_read_reg(TSC210X_PLL1_CTRL));
	seq_printf(s, "TSC210X_PLL2_CTRL = 0x%04x\n",
			tsc210x_read_reg(TSC210X_PLL2_CTRL));
	seq_printf(s, "TSC2102_AUDIO4_CTRL = 0x%04x\n",
			tsc210x_read_reg(TSC2102_AUDIO4_CTRL));
	if (tsc210x_is_tsc2101()) {
		seq_printf(s, "TSC2101_HANDSET_PGA_CTRL = 0x%04x\n",
			tsc210x_read_reg(TSC2101_HANDSET_PGA_CTRL));
		seq_printf(s, "TSC2101_CELL_BUZZER_PGA_CTRL = 0x%04x\n",
			tsc210x_read_reg(TSC2101_CELL_BUZZER_PGA_CTRL));
		seq_printf(s, "TSC2101_AUDIO5_CTRL = 0x%04x\n",
			tsc210x_read_reg(TSC2101_AUDIO5_CTRL));
		seq_printf(s, "TSC2101_AUDIO6_CTRL = 0x%04x\n",
			tsc210x_read_reg(TSC2101_AUDIO6_CTRL));
		seq_printf(s, "TSC2101_AUDIO7_CTRL = 0x%04x\n",
			tsc210x_read_reg(TSC2101_AUDIO7_CTRL));
		seq_printf(s, "TSC2101_GPIO_CTRL = 0x%04x\n",
			tsc210x_read_reg(TSC2101_GPIO_CTRL));
		seq_printf(s, "TSC2101_AGP_CP_IN_CTRL = 0x%04x\n",
			tsc210x_read_reg(TSC2101_AGP_CPIN_CTRL));
		seq_printf(s, "TSC2101_POWERDOWN_STATUS = 0x%04x\n",
			tsc210x_read_reg(TSC2101_POWERDOWN_STATUS));
		seq_printf(s, "TSC2101_MIC_AGC_CTRL = 0x%04x\n",
			tsc210x_read_reg(TSC2101_MIC_AGC_CTRL));
		seq_printf(s, "TSC2101_CELL_PHONE_AGC_CTRL = 0x%04x\n",
			tsc210x_read_reg(TSC2101_CELL_PHONE_AGC_CTRL));
	}

	return 0;
}

static int tsc210x_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, tsc210x_debug_show, NULL);
}

static const struct file_operations tsc210x_debug_operations = {
	.open		= tsc210x_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int tsc210x_debugfs_init(void)
{
	/* /sys/kernel/debug/tsc210x */
	(void) debugfs_create_file("tsc210x", S_IFREG | S_IRUGO, NULL, NULL,
			&tsc210x_debug_operations);

	return 0;
}
#endif /* CONFIG_DEBUG_FS */


static int __init snd_imx_alsa_tsc210x_probe(struct platform_device *pdev)
{
	int ret;
	struct imx_alsa_codec_config *codec_cfg;

	printk("*** %s\n", __func__);

	codec_cfg = kmalloc(sizeof(*codec_cfg), GFP_KERNEL);
	if (codec_cfg) {
		codec_cfg->name = "TSC210x";
		codec_cfg->hw_constraints_rates   =
				&tsc2102_hw_constraints_rates;
		codec_cfg->snd_imx_alsa_playback  =
				&tsc210x_snd_imx_alsa_playback;
		codec_cfg->snd_imx_alsa_capture =
				&tsc2101_snd_imx_alsa_capture;
		codec_cfg->codec_configure_dev    = tsc2102_configure;
		codec_cfg->codec_set_samplerate   = tsc2102_set_samplerate;
		codec_cfg->codec_clock_setup      = tsc2102_clock_setup;
		codec_cfg->codec_clock_on         = tsc2102_clock_on;
		codec_cfg->codec_clock_off        = tsc2102_clock_off;
		codec_cfg->get_default_samplerate =
				tsc2102_get_default_samplerate;
		/* forward codec config to PCM layer */
		ret = snd_imx_alsa_post_probe(pdev, codec_cfg);
	} else {
		ret = -ENODEV;
	}

#ifdef CONFIG_DEBUG_FS
	tsc210x_debugfs_init();
#endif

	return ret;
}

static int snd_imx_alsa_tsc210x_remove(struct platform_device *pdev)
{
	tsc210x_dac_power(0);

	return snd_imx_alsa_remove(pdev);
}

static struct platform_driver imx_alsa_driver = {
	.probe		= snd_imx_alsa_tsc210x_probe,
	.remove 	= snd_imx_alsa_tsc210x_remove,
#ifdef CONFIG_PM
	.suspend	= snd_imx_alsa_tsc2102_suspend,
	.resume		= snd_imx_alsa_tsc2102_resume,
#endif
	.driver		= {
		.name	= "tsc210x-alsa",
	},
};

static int __init imx_alsa_tsc210x_init(void)
{
	int err;

	err = platform_driver_register(&imx_alsa_driver);

	return err;
}

static void __exit imx_alsa_tsc210x_exit(void)
{
	platform_driver_unregister(&imx_alsa_driver);
}

module_init(imx_alsa_tsc210x_init);
module_exit(imx_alsa_tsc210x_exit);

