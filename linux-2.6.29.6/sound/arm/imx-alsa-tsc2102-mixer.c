/*
 * sound/arm/imx-alsa-tsc2102-mixer.c
 *
 * Alsa mixer driver for TSC210x chips.
 *
 * Copyright (c) 2008 Armadeus systems - Jorasse <jorasse@armadeus.com>
 * Code based on the TSC2101 ALSA driver for omap platforms.
 * Copyright (c) 2006 Andrzej Zaborowski  <balrog@zabor.org>
 * Code based on the TSC2101 ALSA driver.
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
 */

#include <linux/types.h>
#include <linux/spi/tsc2102.h>

#include <mach/imx-alsa.h>

#include <sound/initval.h>
#include <sound/control.h>

#include "imx-alsa-tsc2102.h"


/* Audio Out: */
static int vol[2], mute[2], filter[2];
/* Audio In: */
static int hnd_vol, hnd_mute, head_vol, head_mute;

/*
 * Converts the Alsa mixer volume (0 - 100) to actual Digital
 * Gain Control (DGC) value that can be written or read from the
 * TSC2102 registers.
 *
 * Note that the number "OUTPUT_VOLUME_MAX" is smaller than
 * OUTPUT_VOLUME_MIN because DGC works as a volume decreaser.  (The
 * higher the value sent to DAC, the more the volume of controlled
 * channel is decreased)
 */
static void set_dac_gain_stereo(int left_ch, int right_ch)
{
	int lch, rch;

	if (left_ch > 100) {
		vol[0] = 100;
	} else if (left_ch < 0) {
		vol[0] = 0;
	} else {
		vol[0] = left_ch;
	}
	lch = OUTPUT_VOLUME_MIN - vol[0] *
		(OUTPUT_VOLUME_MIN - OUTPUT_VOLUME_MAX) / 100;

	if (right_ch > 100) {
		vol[1] = 100;
	} else if (right_ch < 0) {
		vol[1] = 0;
	} else {
		vol[1] = right_ch;
	}
	rch = OUTPUT_VOLUME_MIN - vol[1] *
		(OUTPUT_VOLUME_MIN - OUTPUT_VOLUME_MAX) / 100;

	tsc210x_set_dac_volume(lch, rch);
}

/* Audio In is amplified, not attenuated like Audio Out */
static void set_handset_volume(int volume)
{
	uint8_t gain;

	if (volume > 100) {
		hnd_vol = 100;
	} else if (volume < 0) {
		hnd_vol = 0;
	} else {
		hnd_vol = volume;
	}
	gain = (hnd_vol * INPUT_HND_VOLUME_RANGE) / 100;

	tsc2101_set_handset_gain(gain);
}

static void set_headset_volume(int volume)
{
	uint8_t gain;

	if (volume > 100) {
		head_vol = 100;
	} else if (volume < 0) {
		head_vol = 0;
	} else {
		head_vol = volume;
	}
	gain = (head_vol * INPUT_HEAD_VOLUME_RANGE) / 100;

	tsc2101_set_headset_gain(gain);
}

void init_playback_targets(void)
{
	set_dac_gain_stereo(DEFAULT_OUTPUT_VOLUME, DEFAULT_OUTPUT_VOLUME);
	if (tsc210x_is_tsc2101())
		set_handset_volume(DEFAULT_INPUT_HND_VOLUME);

	/* unmute Audio Out */
	tsc210x_set_dac_mute(0, 0);
	mute[0] = mute[1] = 0;
	/* mute Audio In */
	if (tsc210x_is_tsc2101()) {
		tsc2101_handset_mute(1);
		hnd_mute = 1;
	}

	filter[0] = filter[1] = 0;
}

/*
 * Initializes TSC210x and playback target.
 */
static void snd_tsc210x_init_mixer(void)
{
	FN_IN;

	init_playback_targets();

	FN_OUT(0);
}

static int __pcm_playback_volume_info(
		struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type			= SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count			= 2;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= 100;

	return 0;
}

static int __pcm_playback_volume_get(
		struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = vol[0];	/* L */
	ucontrol->value.integer.value[1] = vol[1];	/* R */

	return 0;
}

static int __pcm_playback_volume_put(
		struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	set_dac_gain_stereo(
			ucontrol->value.integer.value[0],	/* L */
			ucontrol->value.integer.value[1]);	/* R */

	return 1;
}

static int __pcm_playback_switch_info(
		struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type			= SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count			= 2;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= 1;

	return 0;
}

static int __pcm_playback_switch_get(
		struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = !mute[0];		/* L */
	ucontrol->value.integer.value[1] = !mute[1];		/* R */

	return 0;
}

static int __pcm_playback_switch_put(
		struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	mute[0] = (ucontrol->value.integer.value[0] == 0);	/* L */
	mute[1] = (ucontrol->value.integer.value[1] == 0);	/* R */

	tsc210x_set_dac_mute(mute[0], mute[1]);

	return 1;
}

static int __pcm_playback_deemphasis_info(
		struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type			= SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count			= 1;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= 1;

	return 0;
}

static int __pcm_playback_deemphasis_get(
		struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = filter[0];

	return 0;
}

static int __pcm_playback_deemphasis_put(
		struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	filter[0] = (ucontrol->value.integer.value[0] > 0);
	tsc210x_set_deemphasis(filter[0]);

	return 1;
}

static int __pcm_playback_bassboost_info(
		struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type			= SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count			= 1;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= 1;

	return 0;
}

static int __pcm_playback_bassboost_get(
		struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = filter[1];

	return 0;
}

static int __pcm_playback_bassboost_put(
		struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	filter[1] = (ucontrol->value.integer.value[0] > 0);
	tsc210x_set_bassboost(filter[1]);

	return 1;
}

/* Capture controls (only available on TSC2101) */

static int __capture_mux_enum_info(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_info *uinfo)
{
          static char *texts[3] = {
                  "Headset", "Handset", "None"
          };
          uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
          uinfo->count = 1;
          uinfo->value.enumerated.items = 3;
          if (uinfo->value.enumerated.item > 2)
                  uinfo->value.enumerated.item = 2;
          strcpy(uinfo->value.enumerated.name,
                 texts[uinfo->value.enumerated.item]);
          return 0;
}

static int __capture_mux_enum_get(
		struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = tsc2101_get_micsel();

	return 0;
}

static int __capture_mux_enum_put(
		struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	tsc2101_set_micsel((uint8_t)(ucontrol->value.integer.value[0]));

	return 1;
}

static int __handset_capture_volume_info(
		struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type			= SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count			= 1;	/* Mono */
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= 100;

	return 0;
}

static int __handset_capture_volume_get(
		struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = hnd_vol;

	return 0;
}

static int __handset_capture_volume_put(
		struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	set_handset_volume(ucontrol->value.integer.value[0]); /* hnd_vol mod. here */

	return 1;
}

static int __handset_capture_switch_info(
		struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type			= SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count			= 1;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= 1;

	return 0;
}

static int __handset_capture_switch_get(
		struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = !hnd_mute;

	return 0;
}

static int __handset_capture_switch_put(
		struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	hnd_mute = (ucontrol->value.integer.value[0] == 0);
	tsc2101_handset_mute(hnd_mute);

	return 1;
}

static int __headset_capture_volume_info(
		struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type			= SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count			= 1;	/* Mono */
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= 100;

	return 0;
}

static int __headset_capture_volume_get(
		struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = head_vol;

	return 0;
}

static int __headset_capture_volume_put(
		struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	set_headset_volume(ucontrol->value.integer.value[0]); /* head_vol mod. here */

	return 1;
}

static int __headset_capture_switch_info(
		struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type			= SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count			= 1;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= 1;

	return 0;
}

static int __headset_capture_switch_get(
		struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = !head_mute;

	return 0;
}

static int __headset_capture_switch_put(
		struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	head_mute = (ucontrol->value.integer.value[0] == 0);
	tsc2101_headset_mute(head_mute);

	return 1;
}


static struct snd_kcontrol_new tsc210x_control[] __devinitdata = {
	{
		.name	= "Master Playback Volume",
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.index	= 0,
		.access	= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info	= __pcm_playback_volume_info,
		.get	= __pcm_playback_volume_get,
		.put	= __pcm_playback_volume_put,
	},
	{
		.name	= "Master Playback Switch",
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.index	= 0,
		.access	= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info	= __pcm_playback_switch_info,
		.get	= __pcm_playback_switch_get,
		.put	= __pcm_playback_switch_put,
	},
	{
		.name	= "De-emphasis Filter Switch",
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.index	= 0,
		.access	= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info	= __pcm_playback_deemphasis_info,
		.get	= __pcm_playback_deemphasis_get,
		.put	= __pcm_playback_deemphasis_put,
	},
	{
		.name	= "Bass-boost Filter Switch",
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.index	= 0,
		.access	= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info	= __pcm_playback_bassboost_info,
		.get	= __pcm_playback_bassboost_get,
		.put	= __pcm_playback_bassboost_put,
	},
};

static struct snd_kcontrol_new tsc2101_control[] __devinitdata = {
	{
		.name	= "Mic Capture Route",
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.info	= __capture_mux_enum_info,
		.get	= __capture_mux_enum_get,
		.put	= __capture_mux_enum_put,
	},
	{
		.name	= "Handset Capture Volume",
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.index	= 0,
		.access	= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info	= __handset_capture_volume_info,
		.get	= __handset_capture_volume_get,
		.put	= __handset_capture_volume_put,
	},
	{
		.name	= "Handset Capture Switch",
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.index	= 0,
		.access	= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info	= __handset_capture_switch_info,
		.get	= __handset_capture_switch_get,
		.put	= __handset_capture_switch_put,
	},
	{
		.name	= "Headset Capture Volume",
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.index	= 0,
		.access	= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info	= __headset_capture_volume_info,
		.get	= __headset_capture_volume_get,
		.put	= __headset_capture_volume_put,
	},
	{
		.name	= "Headset Capture Switch",
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.index	= 0,
		.access	= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info	= __headset_capture_switch_info,
		.get	= __headset_capture_switch_get,
		.put	= __headset_capture_switch_put,
	},
};


#ifdef CONFIG_PM
void snd_tsc210x_suspend_mixer(void)
{
	/* Nothing to do */
}

void snd_tsc210x_resume_mixer(void)
{
	/* The chip was reset, restore the last used values */
	set_dac_gain_stereo(vol[0], vol[1]);
	if (tsc210x_is_tsc2101())
		set_handset_volume(hnd_vol);

	tsc210x_set_dac_mute(mute[0], mute[1]);
	if (tsc210x_is_tsc2101())
		tsc2101_handset_mute(hnd_mute);

	tsc210x_set_deemphasis(filter[0]);
	tsc210x_set_bassboost(filter[1]);
}
#endif


/* To be called by upper layer to add a tsc210x mixer to a given soundcard */
int snd_imx_mixer(struct snd_card_imx_codec *tsc210x)
{
	int i, err;

	if (!tsc210x)
		return -EINVAL;

	/* TSC2102/TSC2101 common part */
	for (i = 0; i < ARRAY_SIZE(tsc210x_control); i ++) {
		err = snd_ctl_add(tsc210x->card,
				  snd_ctl_new1(&tsc210x_control[i],
				  tsc210x->card));
		if (err < 0)
			return err;
	}

	/* TSC2101 specific part */
	if (tsc210x_is_tsc2101()) {
		for (i = 0; i < ARRAY_SIZE(tsc2101_control); i ++) {
			err = snd_ctl_add(tsc210x->card,
					  snd_ctl_new1(&tsc2101_control[i],
					  tsc210x->card));
			if (err < 0)
				return err;
		}
	}
	snd_tsc210x_init_mixer();

	return 0;
}

MODULE_AUTHOR("Andrzej Zaborowski, Eric Jarrige, Julien Boibessot");
MODULE_DESCRIPTION("Mixer interface driver for TI TSC210x chips.");
MODULE_LICENSE("GPL");
