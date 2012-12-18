/*
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
 */
#include <linux/device.h>
#include <linux/mfd/mc13783-private.h>
#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>

#include "soc-wrapper.c"
#include "mc13783.h"

#define PMIC_REG_MODE_0 32
#define PMIC_AUDIO_RX_0 36
#define PMIC_AUDIO_RX_1 37
#define PMIC_AUDIO_TX 38
#define PMIC_SSI_NETOWRK 39
#define PMIC_AUDIO_CODEC 40
#define PMIC_AUDIO_DAC 41

struct mc13783_cache {
	u32 reg_36;
	u32 reg_37;
	u32 reg_38;
	u32 reg_39;
	u32 reg_40;
	u32 reg_41;
};

/* register cache */
static struct mc13783_cache regs;

/* counts if the PMIC is in use */
static int mx13783_active;

static struct mc13783 *mc13783;

static unsigned int snd_soc_read(int reg)
{
	unsigned int val;

	mc13783_reg_read(mc13783, reg, &val);

	return val;
}

static void snd_soc_write(int reg, int val)
{
	mc13783_reg_write(mc13783, reg, val);
}

/* power up */
static int mc13783_enable(void)
{
	pr_info("%s called\n", __func__);

	if (!mx13783_active) {
		pr_debug("Audio enabled\n");
		regs.reg_36 |= 0x1;	/* VAUDIOON -> supply audio part */
		regs.reg_36 |= 0x2;	/* BIAS enable */
#if 0
	/* This crashes the system! */
		mc13783_reg_read(mc13783, PMIC_REG_MODE_0, &t);
		t &= 0x07;
		t |= 0x01;
		mc13783_reg_write(mc13783, PMIC_REG_MODE_0, t);
#endif
		mc13783_reg_write(mc13783, PMIC_AUDIO_RX_0, regs.reg_36);
		mc13783_reg_write(mc13783, PMIC_AUDIO_RX_1, regs.reg_37);
		mc13783_reg_write(mc13783, PMIC_AUDIO_TX, regs.reg_38);
		mc13783_reg_write(mc13783, PMIC_SSI_NETOWRK, regs.reg_39);
		mc13783_reg_write(mc13783, PMIC_AUDIO_CODEC, regs.reg_40);
		mc13783_reg_write(mc13783, PMIC_AUDIO_DAC, regs.reg_41);
	}

	mx13783_active++;	/* one more user */

	return 0;
}

/* save power */
static int mc13783_disable(void)
{
	pr_info("%s called\n", __func__);

	if (mx13783_active)
		mx13783_active--;	/* one less user */
	else
		return 0;

	if (!mx13783_active) {
		pr_debug("Audio disabled\n");
#if 0
	/* This crashes the system! */
		mc13783_reg_read(mc13783, PMIC_REG_MODE_0, &t);
		t &= 0x07;
		mc13783_reg_write(mc13783, PMIC_REG_MODE_0, t);
#endif
		regs.reg_36 &= ~0x1;	/* VAUDIOON -> switch off audio part */
		regs.reg_36 &= ~0x2;	/* BIAS disable */
		mc13783_reg_write(mc13783, PMIC_AUDIO_RX_0, regs.reg_36);
	}

	return 0;
}

/* channel: SSI1 (=0) or SSI2 (=1) to be used */
static int mc13783_setup_i2s_master_DAC(int channel, unsigned int rate)
{
	pr_debug("%s: Setup channel %d with rate no %u\n", __func__,
			channel + 1, rate);

	regs.reg_39 |= (3 << 12);	/* two timeslots, TS0 and TS1 */
	if (channel == 1)
		/* connect the DAC to SSI pin group 2 */
		regs.reg_41 |= 1;
	else
		/* connect the DAC to SSI pin group 1 */
		regs.reg_41 &= ~1;

	/* FIXME CLIA is board specific: Here 26MHz */
	regs.reg_41 &= ~(1 << 1);
	regs.reg_41 &= ~(1 << 2);	/* DAC is master */

	/* FIXME clock and fs inversion? */
	regs.reg_41 &= ~(3 << 3);

	/* CLK: 1 = valid@rising, 0 = valid@falling edge -> verified */
	regs.reg_41 |= (1 << 3);
	/* FS: 1 = #__######, 0 = _###_____ -> verified */
	regs.reg_41 |= (0 << 4);

	regs.reg_41 &= ~(0x3 << 5);
	/* short fs with offset -1 (for network mode) -> verified */
	regs.reg_41 |= (0x2 << 5);

	regs.reg_41 &= ~(0x7 << 7);
	regs.reg_41 |= (0x4 << 7);	/* FIXME fixed to 26MHz input */
	regs.reg_41 |= (1 << 11);	/* DAC power up */
	regs.reg_41 |= (1 << 12);	/* enable FS and CLK out */
	regs.reg_41 |= (1 << 15);	/* reset filter */

	/* setup the clock speed */
	regs.reg_41 &= ~(0xf << 17);
	regs.reg_41 |= rate << 17;

	mc13783_reg_write(mc13783, PMIC_SSI_NETOWRK, regs.reg_39);
	mc13783_reg_write(mc13783, PMIC_AUDIO_DAC, regs.reg_41);

	return 0;
}

/* channel: SSI1 (=0) or SSI2 (=1) to be used */
static int mc13783_setup_i2s_master_CODEC(int channel, unsigned int rate)
{
	pr_debug("%s: Setup channel %d with rate no %u\n", __func__,
			channel + 1, rate);

	regs.reg_39 &= ~(0xFFF << 0);
	regs.reg_39 |= (0x00 << 2);	/* primary timeslot RX/TX(?) is 0 */
	regs.reg_39 |= (0x01 << 4);	/* secondary timeslot TX is 1 */
	regs.reg_39 |= (0x01 << 6);	/* secondary timeslot RX is 1 */

	if (channel == 1)
		/* connect the CODEC to SSI pin group 2 */
		regs.reg_40 |= 1;
	else
		/* connect the CODEC to SSI pin group 1 */
		regs.reg_40 &= ~1;

	regs.reg_40 &= ~(1 << 1);	/* CLIA is board specific: Here 26MHz */
	regs.reg_40 &= ~(1 << 2);	/* CODEC is master */

	/* FIXME clock and fs inversion? */
	regs.reg_40 &= ~(3 << 3);

	/* CLK: 1 = valid@rising, 0 = valid@falling edge -> verified */
	regs.reg_40 |= (1 << 3);
	/* FS: 1 = #__######, 0 = _###_____ -> verified */
	regs.reg_40 |= (0 << 4);

	regs.reg_40 &= ~(0x3 << 5);
	/*
	 * PMIC always generates 4 time slots per sync.
	 * So 8kHz sample rate uses a 512kHz clock,
	 * and 16kHz uses a 10.24kHz clock.
	 * With a long fs only the first timeslot contains data.
	 * With a short fs it uses the first two timeslots.
	 */
	/* short fs with offset -1 (for network mode) -> verified */
	regs.reg_40 |= (0x01 << 5);

	regs.reg_40 &= ~(0x7 << 7);
	/* fixed to 26MHz input FIXME: platform_info */
	regs.reg_40 |= (0x4 << 7);

	regs.reg_40 |= (1 << 11);	/* CODEC power up */
	regs.reg_40 |= (1 << 12);	/* enable FS and CLK out */
	regs.reg_40 &= ~(1 << 13);	/* activate FS and CLK out */
	regs.reg_40 |= (1 << 15);	/* reset filter */
	regs.reg_40 &= ~(1 << 16);	/* CODEC not bypassed */
	regs.reg_40 &= ~(1 << 17);	/* disable CODEC's analog loopback */
	regs.reg_40 &= ~(1 << 18);	/* disable CODEC's digital loopback */

	/* setup the clock speed */

	switch (rate) {
	case 0:
		regs.reg_40 &= ~(1 << 10);
		break;
	case 1:
		regs.reg_40 |= (1 << 10);
		break;
	default:
		return -EINVAL;
	}

	mc13783_reg_write(mc13783, PMIC_SSI_NETOWRK, regs.reg_39);
	mc13783_reg_write(mc13783, PMIC_AUDIO_CODEC, regs.reg_40);

	return 0;
}

/* sample rates supported by PMIC for stereo playback operations on StDac. */
static unsigned int mc13783_rates[] = {
	8000, 11025, 12000, 16000,
	22050, 24000, 32000,44100,
	48000, 64000, 96000
};

int mc13783_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	unsigned int rate = params_rate(params);
	int i;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		for (i = 0; i < ARRAY_SIZE(mc13783_rates); i++) {
			if (rate == mc13783_rates[i]) {
				mc13783_setup_i2s_master_DAC(0, i);
				return 0;
			}
		}
	} else {
		mc13783_setup_i2s_master_CODEC(1, rate == 8000 ? 0 : 1);
		return 0;
	}

	return -EINVAL;
}

/* setup reset behaviour */
static int mc13783_reg_init(void)
{
	pr_debug("%s called\n", __func__);

	/* these are the reset values */
	regs.reg_36 = 0x001000;
	regs.reg_37 = 0x00D35A;
	regs.reg_38 = 0x420000;
	regs.reg_39 = 0x013060;
	regs.reg_40 = 0x180027;
	regs.reg_41 = 0x0E0004;

	mc13783_reg_write(mc13783, PMIC_AUDIO_RX_0, regs.reg_36);
	mc13783_reg_write(mc13783, PMIC_AUDIO_RX_1, regs.reg_37);
	mc13783_reg_write(mc13783, PMIC_AUDIO_TX, regs.reg_38);
	mc13783_reg_write(mc13783, PMIC_SSI_NETOWRK, regs.reg_39);
	mc13783_reg_write(mc13783, PMIC_AUDIO_CODEC, regs.reg_40);
	mc13783_reg_write(mc13783, PMIC_AUDIO_DAC, regs.reg_41);

	return 0;
}

static int mc13783_asp_val;

static int mc13783_get_asp(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = mc13783_asp_val;
	return 0;
}

static int mc13783_put_asp(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
	printk("%s %ld\n", __func__, ucontrol->value.integer.value[0]);

	mc13783_asp_val = ucontrol->value.integer.value[0];
//	mc13783_put(3, 4, mc13783_asp_val);

	return 0;
}

static int mc13783_alsp_val;

static int mc13783_get_alsp(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = mc13783_alsp_val;
	return 0;
}

static int mc13783_put_alsp(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
	printk("%s %ld\n", __func__, ucontrol->value.integer.value[0]);

	mc13783_alsp_val = ucontrol->value.integer.value[0];
//	mc13783_put(5, 7, mc13783_alsp_val);

	return 0;
}

static int mc13783_pcm_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int val;

	mc13783_reg_read(mc13783, 36, &val);
	ucontrol->value.enumerated.item[0] = (val >> 22) & 1;

        return 0;
}

static int mc13783_pcm_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	unsigned int r36, r37;

	mc13783_reg_read(mc13783, 36, &r36);
	mc13783_reg_read(mc13783, 37, &r37);

	r36 &= ~(1 << 22);
	r37 &= ~(1 << 5);

	if (ucontrol->value.enumerated.item[0]) {
		r36 |= (1 << 22);
		r37 |= (1 << 5);
	} else {
		r36 &= ~(1 << 22);
		r37 &= ~(1 << 5);
	}

	mc13783_reg_write(mc13783, 36, r36);
	mc13783_reg_write(mc13783, 37, r37);

        return 0;
}

static int mc13783_linein_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int val;

	mc13783_reg_read(mc13783, 36, &val);
	ucontrol->value.enumerated.item[0] = (val >> 23) & 1;

        return 0;
}

static int mc13783_linein_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	unsigned int r36, r37;

	mc13783_reg_read(mc13783, 36, &r36);
	mc13783_reg_read(mc13783, 37, &r37);

	r36 &= ~(1 << 23);
	r37 &= ~(1 << 10);

	if (ucontrol->value.enumerated.item[0]) {
		r36 |= (1 << 23);
		r37 |= (1 << 10);
	} else {
		r36 &= ~(1 << 23);
		r37 &= ~(1 << 10);
	}

	mc13783_reg_write(mc13783, 36, r36);
	mc13783_reg_write(mc13783, 37, r37);

        return 0;
}

static int mc13783_capure_cache;

static int mc13783_get_capture(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	printk("%s: %d\n", __func__, mc13783_capure_cache);
	ucontrol->value.enumerated.item[0] = mc13783_capure_cache;
        return 0;
}

#define AMC1REN (1 << 5)
#define AMC2EN  (1 << 9)
#define ATXINEN (1 << 11)
#define RXINREC (1 << 13)
#define AMC1LEN (1 << 7)

static int mc13783_put_capture(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	unsigned int r38, change;

	mc13783_reg_read(mc13783, 38, &r38);

	change = (mc13783_capure_cache != ucontrol->value.enumerated.item[0]);
	mc13783_capure_cache = ucontrol->value.enumerated.item[0];
	r38 &= ~(AMC1REN | AMC2EN | ATXINEN | RXINREC | AMC1LEN);
	printk("%s: %d\n", __func__, mc13783_capure_cache);
	switch (mc13783_capure_cache) {
	case 0:
		break;
	case 1:
		r38 |= RXINREC;
		break;
	case 2:
		r38 |= AMC1REN | AMC1LEN;
		break;
	case 3:
		r38 |= AMC1REN;
		break;
	case 4:
		r38 |= AMC2EN;
		break;
	case 5:
		r38 |= AMC1LEN | AMC2EN;
		break;
	case 6:
		r38 |= ATXINEN;
		break;
	case 7:
		r38 |= AMC1LEN | ATXINEN;
		break;
	case 8:
		r38 |= AMC1LEN | RXINREC;
		break;
	case 9:
		r38 |= AMC1LEN;
		break;
	default:
		break;
	}

	mc13783_reg_write(mc13783, 38, r38);

	return change;
}

static const char *mc13783_asp[] = {"Off", "Codec", "Right"};
static const char *mc13783_alsp[] = {"Off", "Codec", "Right"};

static const char *mc13783_ahs[] = {"Codec", "Mixer"};

static const char *mc13783_arxout[] = {"Codec", "Mixer"};

static const char *mc13783_capture[] = {"off/off", "rxinl/rxinr",
	"mc1lin/mc1rin", "off/mc1rin", "off/mc2in", "mc1lin/mc2in",
	"off/txin", "mc1lin/txin", "mc1lin/rxinr", "mc1lin/off"};

static const char *mc13783_3d_mixer[] = {"Stereo", "Phase Mix", "Mono", "Mono Mix"};

static const struct soc_enum mc13783_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(mc13783_asp), mc13783_asp),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(mc13783_alsp), mc13783_alsp),

	SOC_ENUM_SINGLE(36, 11, ARRAY_SIZE(mc13783_ahs), mc13783_ahs),
	SOC_ENUM_SINGLE(36, 17, ARRAY_SIZE(mc13783_arxout), mc13783_arxout),

	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(mc13783_capture), mc13783_capture),
	SOC_ENUM_SINGLE(37, 16, ARRAY_SIZE(mc13783_3d_mixer), mc13783_3d_mixer),
};

static struct snd_kcontrol_new mc13783_control_list[] = {
	/* Output Routing */
	SOC_ENUM_EXT("Asp Source", mc13783_enum[0], mc13783_get_asp, mc13783_put_asp),
	SOC_ENUM_EXT("Alsp Source", mc13783_enum[1], mc13783_get_alsp, mc13783_put_alsp),
	SOC_ENUM("Ahs Source", mc13783_enum[2]),
	SOC_SINGLE("Ahsr enable", 36, 9, 1, 0),
	SOC_SINGLE("Ahsl enable", 36, 10, 1, 0),
	SOC_ENUM("Arxout Source", mc13783_enum[3]),
	SOC_SINGLE("ArxoutR enable", 36, 16, 1, 0),
	SOC_SINGLE("ArxoutL enable", 36, 15, 1, 0),
	SOC_SINGLE_EXT("PCM Playback Switch", 0, 0, 1, 0, mc13783_pcm_get, mc13783_pcm_put),
	SOC_SINGLE("PCM Playback Volume", 37, 6, 15, 0),
	SOC_SINGLE_EXT("Line in Switch", 0, 0, 1, 0, mc13783_linein_get, mc13783_linein_put),
	SOC_SINGLE("Line in Volume", 37, 12, 15, 0),
	SOC_ENUM_EXT("Capture Source", mc13783_enum[4], mc13783_get_capture, mc13783_put_capture),
	SOC_DOUBLE("PCM Capture Volume", 38, 19, 14, 31, 0),
	SOC_ENUM("3D Control - Switch", mc13783_enum[5]),
};

int mc13783_add_ctl(struct snd_card *card, void *p_value)
{
	int err, i;

	pr_debug("%s called\n", __func__);
	if (!mc13783)
		return -ENODEV;

	mc13783_reg_init();
	mc13783_enable();

	for (i = 0; i < ARRAY_SIZE(mc13783_control_list); i++) {
		err = snd_ctl_add(card,
			snd_ctl_new1(&mc13783_control_list[i], p_value));
		if (err < 0) {
			pr_err("Cannot register %s\n",
				mc13783_control_list[i].name);
			return err;
		}
		pr_debug("%s registered\n", mc13783_control_list[i].name);
	}
	return 0;
}

/*
 * OK, this stinks. We currently only can support one MC13783.
 * Lets take it as an intermediate to turn this stuff into SoC
 * Audio.
 */
static int mc13783_codec_probe(struct platform_device *pdev)
{
	printk(KERN_INFO "MC13783 Audio Codec\n");

	if (mc13783)
		return -EBUSY;

	mc13783 = platform_get_drvdata(pdev);

	return 0;
}

static int mc13783_codec_remove(struct platform_device *pdev)
{
	mc13783 = NULL;

	return 0;
}

static struct platform_driver mc13783_codec_driver = {
	.driver = {
		   .name = "mc13783-codec",
		   .owner = THIS_MODULE,
		   },
	.probe = mc13783_codec_probe,
	.remove = __devexit_p(mc13783_codec_remove),
//	.suspend = mc13783_suspend,
//	.resume = mc13783_resume,
};

static __init int mc13783_init(void)
{
	return platform_driver_register(&mc13783_codec_driver);
}

static __exit void mc13783_exit(void)
{
	platform_driver_unregister(&mc13783_codec_driver);
}

module_init(mc13783_init);
module_exit(mc13783_exit);

MODULE_DESCRIPTION("ASoC MC13783 driver");
MODULE_AUTHOR("Sascha Hauer, Pengutronix <s.hauer@pengutronix.de>");
MODULE_LICENSE("GPL");

