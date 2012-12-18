/*
 * include/linux/spi/tsc2102.h
 *
 * TI TSC210(1/2) Touchscreen, Audio and Battery control register definitions
 *
 * Copyright (c) 2005 Andrzej Zaborowski  <balrog@zabor.org>
 * Copyright (c) 2008 Armadeus Systems  <julien.boibessot@armadeus.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef __LINUX_SPI_TSC2102_H
#define __LINUX_SPI_TSC2102_H

struct apm_power_info;

struct tsc210x_config {
	unsigned int mclk;	/* Main clock, fixed if TSC is master */
	int use_internal;	/* Use internal reference voltage */
	uint32_t monitor;	/* What inputs are relevant */
	void (*apm_report)(struct apm_power_info *info, int *battery);
				/* Report status to APM based on battery[] */
	void *alsa_config;	/* .platform_data for the ALSA device */
	int (*init)(void);	/* allocate GPIOs */
	void (*exit)(void);
};

#define TSC_BAT1	(1 << 0)
#define TSC_BAT2	(1 << 1)
#define TSC_AUX		(1 << 2)
#define TSC_AUX1	(1 << 2) /* TSC2101 has 2 Aux inputs */
#define TSC_AUX2	(1 << 3)
#define TSC_TEMP	(1 << 4)

extern u16 tsc210x_read_reg(int page, u8 address);

typedef void (*tsc210x_touch_t)(int touching);
typedef void (*tsc210x_coords_t)(int x, int y, int z1, int z2);
typedef void (*tsc210x_ports_t)(int bat1, int bat2, int aux);
typedef void (*tsc210x_temp_t)(int temp);
extern int tsc210x_touch_cb(tsc210x_touch_t handler);
extern int tsc210x_coords_cb(tsc210x_coords_t handler);
extern int tsc210x_ports_cb(tsc210x_ports_t handler);
extern int tsc210x_temp1_cb(tsc210x_temp_t handler);

extern int tsc210x_is_tsc2101(void);
extern int tsc210x_is_tsc2102(void);

#if defined(CONFIG_SOUND) || defined(CONFIG_SOUND_MODULE)
extern void tsc210x_set_dac_volume(uint8_t left_ch, uint8_t right_ch);
extern void tsc210x_set_dac_mute(int left_ch, int right_ch);
extern void tsc210x_get_dac_mute(int *left_ch, int *right_ch);
extern void tsc210x_dac_power(int state);
extern int  tsc210x_set_rate(int rate);
extern void tsc210x_set_i2s_master(int state);
extern void tsc210x_set_deemphasis(int enable);
extern void tsc210x_set_bassboost(int enable);
extern void tsc2101_set_micsel(uint8_t micsel);
extern uint8_t tsc2101_get_micsel(void);
extern void tsc2101_handset_mute(int enable);
extern void tsc2101_set_handset_gain(uint8_t gain);
extern void tsc2101_headset_mute(int enable);
extern void tsc2101_set_headset_gain(uint8_t gain);
#endif

extern void tsc210x_keyclick(int amplitude, int freq, int length);

#define TSC210X_REG(pg, addr)		pg, addr

/* Page 0, Touch Screen & Keypad Data registers */
#define TSC2102_TS_X			TSC210X_REG(0, 0x00)
#define TSC2102_TS_Y			TSC210X_REG(0, 0x01)
#define TSC2102_TS_Z1			TSC210X_REG(0, 0x02)
#define TSC2102_TS_Z2			TSC210X_REG(0, 0x03)
#define TSC2102_TS_BAT1			TSC210X_REG(0, 0x05)
#define TSC2102_TS_BAT2			TSC210X_REG(0, 0x06)
#define TSC2102_TS_AUX			TSC210X_REG(0, 0x07)
#define TSC2102_TS_TEMP1		TSC210X_REG(0, 0x09)
#define TSC2102_TS_TEMP2		TSC210X_REG(0, 0x0a)

/* Page 1, Touch Screen & Keypad Control registers */
#define TSC2102_TS_ADC_CTRL		TSC210X_REG(1, 0x00)
#define TSC2102_TS_STATUS_CTRL		TSC210X_REG(1, 0x01)
#define TSC2102_TS_REF_CTRL		TSC210X_REG(1, 0x03)
#define TSC2102_TS_RESET_CTRL		TSC210X_REG(1, 0x04)
#define TSC2102_TS_CONFIG_CTRL		TSC210X_REG(1, 0x05)

/* Page 2, Audio Control registers */
#define TSC2102_AUDIO1_CTRL		TSC210X_REG(2, 0x00)
#define TSC2101_HEADSET_AUX_PGA_CTRL	TSC210X_REG(2, 0x01)	/* TSC2101 only */
#define TSC210X_DAC_GAIN_CTRL		TSC210X_REG(2, 0x02)
#define TSC2101_MIXER_PGA_CTRL		TSC210X_REG(2, 0x03)	/* TSC2101 only */
#define TSC2102_AUDIO2_CTRL		TSC210X_REG(2, 0x04)
#define TSC210X_CODEC_POWER_CTRL	TSC210X_REG(2, 0x05)
#define TSC2102_AUDIO3_CTRL		TSC210X_REG(2, 0x06)
#define TSC2102_LCH_BASS_BOOST_N0	TSC210X_REG(2, 0x07)
#define TSC2102_LCH_BASS_BOOST_N1	TSC210X_REG(2, 0x08)
#define TSC2102_LCH_BASS_BOOST_N2	TSC210X_REG(2, 0x09)
#define TSC2102_LCH_BASS_BOOST_N3	TSC210X_REG(2, 0x0a)
#define TSC2102_LCH_BASS_BOOST_N4	TSC210X_REG(2, 0x0b)
#define TSC2102_LCH_BASS_BOOST_N5	TSC210X_REG(2, 0x0c)
#define TSC2102_LCH_BASS_BOOST_D1	TSC210X_REG(2, 0x0d)
#define TSC2102_LCH_BASS_BOOST_D2	TSC210X_REG(2, 0x0e)
#define TSC2102_LCH_BASS_BOOST_D4	TSC210X_REG(2, 0x0f)
#define TSC2102_LCH_BASS_BOOST_D5	TSC210X_REG(2, 0x10)
#define TSC2102_RCH_BASS_BOOST_N0	TSC210X_REG(2, 0x11)
#define TSC2102_RCH_BASS_BOOST_N1	TSC210X_REG(2, 0x12)
#define TSC2102_RCH_BASS_BOOST_N2	TSC210X_REG(2, 0x13)
#define TSC2102_RCH_BASS_BOOST_N3	TSC210X_REG(2, 0x14)
#define TSC2102_RCH_BASS_BOOST_N4	TSC210X_REG(2, 0x15)
#define TSC2102_RCH_BASS_BOOST_N5	TSC210X_REG(2, 0x16)
#define TSC2102_RCH_BASS_BOOST_D1	TSC210X_REG(2, 0x17)
#define TSC2102_RCH_BASS_BOOST_D2	TSC210X_REG(2, 0x18)
#define TSC2102_RCH_BASS_BOOST_D4	TSC210X_REG(2, 0x19)
#define TSC2102_RCH_BASS_BOOST_D5	TSC210X_REG(2, 0x1a)
#define TSC210X_PLL1_CTRL		TSC210X_REG(2, 0x1b)
#define TSC210X_PLL2_CTRL		TSC210X_REG(2, 0x1c)
#define TSC2102_AUDIO4_CTRL		TSC210X_REG(2, 0x1d)
#define TSC2101_HANDSET_PGA_CTRL	TSC210X_REG(2, 0x1e)
#define TSC2101_CELL_BUZZER_PGA_CTRL	TSC210X_REG(2, 0x1f)
#define TSC2101_AUDIO5_CTRL		TSC210X_REG(2, 0x20)
#define TSC2101_AUDIO6_CTRL		TSC210X_REG(2, 0x21)
#define TSC2101_AUDIO7_CTRL		TSC210X_REG(2, 0x22)
#define TSC2101_GPIO_CTRL		TSC210X_REG(2, 0x23)
#define TSC2101_AGP_CPIN_CTRL		TSC210X_REG(2, 0x24)
#define TSC2101_POWERDOWN_STATUS	TSC210X_REG(2, 0x25)
#define TSC2101_MIC_AGC_CTRL		TSC210X_REG(2, 0x26)
#define TSC2101_CELL_PHONE_AGC_CTRL	TSC210X_REG(2, 0x27)

/* Field masks for Audio Control 1 */
#define AC1_ADCHPF(ARG)			(((ARG) & 0x03) << 14)	/* TSC2101 only */
#define AC1_WLEN(ARG)			(((ARG) & 0x03) << 10)
#define AC1_DATFM(ARG)			(((ARG) & 0x03) << 8)
#define AC1_DACFS(ARG)			(((ARG) & 0x07) << 3)
#define AC1_ADCFS(ARG)			(((ARG) & 0x07) << 0)
/* on TSC2102 always set ADCFS = DACFS */

/* Field masks for TSC2101_HEADSET_AUX_PGA_CTRL */
/* TSC2101 only */
#define ADMUT_HED			(1 << 15)
#define ADPGA_HED(ARG)			(((ARG) & 0x7f) << 8)
#define AGCTG_HED(ARG)			(((ARG) & 0x07) << 5)
#define AGCTC_HED(ARG)			(((ARG) & 0x0f) << 1)
#define AGCEN_HED(ARG)			(1 << 0)

/* Field masks for TSC2102_DAC_GAIN_CTRL */
#define DGC_DALMU			(1 << 15)
#define DGC_DALVL(ARG)			(((ARG) & 0x7f) << 8)
#define DGC_DARMU			(1 << 7)
#define DGC_DARVL(ARG)			(((ARG) & 0x7f))

/* Field masks for Mixer PGA Control */
/* TSC2101 only: */
#define MPC_ASTMU			(1 << 15)
#define MPC_ASTG(ARG)			(((ARG) & 0x7f) << 8)
#define MPC_MICSEL(ARG)			(((ARG) & 0x07) << 5)
#define MPC_MICADC			(1 << 4)
#define MPC_CPADC			(1 << 3)
#define MPC_ASTGF			(1 << 0)

/* Field formats for TSC2102_AUDIO2_CTRL */
#define AC2_KCLEN			(1 << 15)
#define AC2_KCLAC(ARG)			(((ARG) & 0x07) << 12)
#define AC2_APGASS			(1 << 11)	/* TSC2101 only */
#define AC2_KCLFRQ(ARG)			(((ARG) & 0x07) << 8)
#define AC2_KCLLN(ARG)			(((ARG) & 0x0f) << 4)
#define AC2_DLGAF			(1 << 3)
#define AC2_DRGAF			(1 << 2)
#define AC2_DASTC			(1 << 1)
#define AC2_ADGAF			(1 << 0)	/* TSC2101 only */

/* Field masks for TSC2102_CODEC_POWER_CTRL */
#define CPC_PWDNC			(1 << 15)
#define CPC_DAODRC			(1 << 12)
#define CPC_DAPWDN			(1 << 10)
#define CPC_VGPWDN			(1 << 8)
#define CPC_DAPWDF			(1 << 6)
#define CPC_BASSBC			(1 << 1)
#define CPC_DEEMPF			(1 << 0)
/* TSC2101 specific bits */
#define CPC_MBIAS_HND			(1 << 15)
#define CPC_MBIAS_HED			(1 << 14)
#define CPC_ASTPWD			(1 << 13)
#define CPC_SP1PWDN			(1 << 12)
#define CPC_SP2PWDN			(1 << 11)
#define CPC_ADPWDN			(1 << 9)
#define CPC_COPWDN			(1 << 7)
#define CPC_LSPWDN			(1 << 6)
#define CPC_ADPWDF			(1 << 5)
#define CPC_LDAPWDF			(1 << 4)
#define CPC_RDAPWDF			(1 << 3)
#define CPC_ASTPWF			(1 << 2)
#define CPC_EFFCTL			(1 << 1)

/* Field masks for TSC210x_AUDIO_CTRL_3 */
#define AC3_DMSVOL(ARG)			(((ARG) & 0x03) << 14)
#define AC3_REFFS			(1 << 13)
#define AC3_DAXFM			(1 << 12)
#define AC3_SLVMS			(1 << 11)
#define AC3_ADCOVF			(1 << 8)	/* TSC2101 only */
#define AC3_DALOVF			(1 << 7)
#define AC3_DAROVF			(1 << 6)
#define AC3_CLPST			(1 << 3)	/* TSC2101 only */
#define AC3_REVID(ARG)			(((ARG) & 0x07))

/* Field masks for TSC2102_PLL1_CTRL */
#define PLL1_PLLEN			(1 << 15)
#define PLL1_Q_VAL(ARG)			(((ARG) & 0x0f) << 11)
#define PLL1_P_VAL(ARG)			(((ARG) & 0x07) << 8)
#define PLL1_J_VAL(ARG)			(((ARG) & 0x3f) << 2)

/* Field masks for TSC2102_PLL2_CTRL */
#define PLL2_D_VAL(ARG)			(((ARG) & 0x3fff) << 2)

/* Field masks for TSC210x_AUDIO_CTRL_4 */
#define AC4_DASTPD			(1 << 14)
/* TSC2101 only: */
#define AC4_ADSTPD			(1 << 15)
#define AC4_ASSTPD			(1 << 13)
#define AC4_CISTPD			(1 << 12)
#define AC4_BISTPD			(1 << 11)
#define AC4_AGCHYS(ARG)			(((ARG) & 0x03) << 9)
#define AC4_MB_HED(ARG)			(((ARG) & 0x03) << 7)
#define AC4_MB_HND			(1 << 6)
#define AC4_SCPFL			(1 << 1)

/* !! FOLLOWING REGISTERS ARE ONLY AVAILABLE ON TSC2101 !! */

/* Field masks for TSC2101_HANDSET_PGA_CTRL */
#define ADMUT_HND			(1 << 15)
#define ADPGA_HND(ARG)			(((ARG) & 0x7f) << 8)
#define AGCTG_HND(ARG)			(((ARG) & 0x07) << 5)
#define AGCTC_HND(ARG)			(((ARG) & 0x0f) << 1)
#define AGCEN_HND(ARG)			(1 << 0)

/* Field masks for TSC2101_CELL_BUZZER_PGA_CTRL */
#define CPBGC_MUT_CP			(1 << 15)
#define CPBGC_CPGA(ARG)			(((ARG) & 0x7f) << 8)
#define CPBGC_CPGF			(1 << 7)
#define CPBGC_MUT_BU			(1 << 6)
#define CPBGC_BPGA(ARG)			(((ARG) & 0x0f) << 2)
#define CPBGC_BUGF			(1 << 1)

/* Field masks for TSC2101_AUDIO_CTRL_5 */
#define AC5_DIFFIN			(1 << 15)
#define AC5_DAC2SPK1(ARG)		(((ARG) & 0x03) << 13)
#define AC5_AST2SPK1			(1 << 12)
#define AC5_BUZ2SPK1			(1 << 11)
#define AC5_KCL2SPK1			(1 << 10)
#define AC5_CPI2SPK1			(1 << 9)
#define AC5_DAC2SPK2(ARG)		(((ARG) & 0x03) << 7)
#define AC5_AST2SPK2			(1 << 6)
#define AC5_BUZ2SPK2			(1 << 5)
#define AC5_KCL2SPK2			(1 << 4)
#define AC5_CPI2SPK2			(1 << 3)
#define AC5_MUTSPK1			(1 << 2)
#define AC5_MUTSPK2			(1 << 1)
#define AC5_HDSCPTC			(1 << 0)

/* Field masks for TSC2101_AUDIO_CTRL_6 */
#define AC6_SPL2LSK			(1 << 15)
#define AC6_AST2LSK			(1 << 14)
#define AC6_BUZ2LSK			(1 << 13)
#define AC6_KCL2LSK			(1 << 12)
#define AC6_CPI2LSK			(1 << 11)
#define AC6_MIC2CPO			(1 << 10)
#define AC6_SPL2CPO			(1 << 9)
#define AC6_SPR2CPO			(1 << 8)
#define AC6_MUTLSPK			(1 << 7)
#define AC6_MUTSPK2			(1 << 6)
#define AC6_LDSCPTC			(1 << 5)
#define AC6_VGNDSCPTC			(1 << 4)
#define AC6_CAPINTF			(1 << 3)

/* Field masks for TSC2101_AUDIO_CTRL_7 */
#define AC7_DETECT			(1 << 15)
#define AC7_HESTYPE(ARG)		(((ARG) & 0x03) << 13)
#define AC7_HDDETFL			(1 << 12)
#define AC7_BDETFL			(1 << 11)
#define AC7_HDDEBNPG(ARG)		(((ARG) & 0x03) << 9)
#define AC7_BDEBNPG(ARG)		(((ARG) & 0x03) << 6)
#define AC7_DGPIO2			(1 << 4)
#define AC7_DGPIO1			(1 << 3)
#define AC7_CLKGPIO2			(1 << 2)
#define AC7_ADWSF(ARG)			((ARG) & 0x03)


struct tsc210x_rate_info_s {
	u16 sample_rate;
	u8 divisor;
	u8 fs_44k;	/* 44.1 kHz Fsref if 1, 48 kHz if 0 */
};

#endif	/* __LINUX_SPI_TSC2102_H */
