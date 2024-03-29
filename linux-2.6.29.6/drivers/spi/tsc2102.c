/*
 * drivers/spi/tsc2102.c
 *
 * TSC2101/2 interface driver.
 * Copyright (C) 2008 Armadeus systems (nc)
 * 			-> spi communication reworked to avoid deadlock
 * Copyright (c) 2005 Andrzej Zaborowski  <balrog@zabor.org>
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/suspend.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/spi/tsc2102.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/workqueue.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <asm/io.h>

#ifdef CONFIG_APM
#include <asm/apm.h>
#endif

/* #define DEBUG */
#define DEBUG_AUDIO

/* Bit field definitions for chip registers */
#define TSC2102_ADC_TS_CONTROL		0x8bf5
#define TSC2102_ADC_SCAN_CONTROL	0x2ff4
#define TSC2102_ADC_T1_CONTROL		0x2bf4
#define TSC2102_ADC_T2_CONTROL		0x33f4
#define TSC2102_ADC_DAV			0x4000
#define TSC2102_ADC_PEN			0x0000
#define TSC2102_ADC_PINTDAV		0x8000
#define TSC2102_ADC_INT_REF_125		0x0016
#define TSC2102_ADC_INT_REF_250		0x0017
#define TSC2102_ADC_REF			TSC2102_ADC_INT_REF_250 /*2.5V*/
#define TSC2102_ADC_EXT_REF		0x0002
#define TSC2102_CONFIG_TIMES		0x0012
#define TSC2102_RESET			0xbb00
#define TSC2102_ADC_PSTCM		(1 << 15)
#define TSC2102_ADC_ADST		(1 << 14)
#define TSC2102_PWRDN			(1 << 13)
#define TSC2102_TS_DAV			0x0780
#define TSC2102_PS_DAV			0x0078
#define TSC2102_T1_DAV			0x0004
#define TSC2102_T2_DAV			0x0002
#define TSC2101_DAC_ON                  0x0000
#define TSC2101_DAC_OFF                 0xe7fc
#define TSC2102_DAC_ON			0x3ba0
#define TSC2102_DAC_OFF			0xafa0
#define TSC210X_PLL1_OFF		0x0000
#define TSC210x_AUDIO1_INIT_VAL		(AC1_WLEN(0) | AC1_DATFM(0) | AC1_DACFS(0) | AC1_ADCFS(0))
#define TSC210X_KEYCLICK_OFF		0x0000

#define CS_CHANGE(val)			0

#define SM_BAT_AUX 0
#define SM_TEMP1   1
#define SM_SUSPENDED 2

struct tsc210x_spi_req {
	struct spi_device *dev;
	uint16_t command;
	uint16_t data;
	struct spi_transfer *transfer;
	struct spi_message message;
};

struct tsc210x_dev {
	enum tsc_type {
		tsc2101,
		tsc2102,
	} kind;
	struct tsc210x_config *pdata;
	spinlock_t lock;
	struct mutex lock_sync;
	struct clk *mclk_ck;
	unsigned int mclk;

	int state;			/* Scan modes */
	struct timer_list ts_timer;	/* Busy-wait for PEN UP */
	struct timer_list mode_timer;	/* Change .state every some time */

	uint16_t status, adc_status, adc_data[4];
	tsc210x_touch_t  touch_cb;
	tsc210x_coords_t coords_cb;
	tsc210x_ports_t  ports_cb;
	tsc210x_temp_t   temp1_cb;
	unsigned int ts_msecs;		/* Interval for .ts_timer */
	unsigned int mode_msecs;	/* Interval for .mode_timer */

	struct spi_device *spi;

	int bat[2], aux[2], temp;

	struct device *hwmondev;
	struct work_struct	work;
	unsigned long		todo;
#define WORK_CHANGE_MODE	0	/* change mode */
};

static struct tsc210x_dev tsc;

module_param_named(touch_check_msecs, tsc.ts_msecs, uint, 0);
MODULE_PARM_DESC(touch_check_msecs, "Pen-up polling interval in msecs");

module_param_named(sensor_scan_msecs, tsc.mode_msecs, uint, 0);
MODULE_PARM_DESC(sensor_scan_msecs, "Temperature & battery scan interval");


void tsc210x_write_sync(int page, u8 address, u16 data)
{
	static struct tsc210x_spi_req req;
	static struct spi_transfer transfer[2];
	int ret;

	spi_message_init(&req.message);
	req.transfer = transfer;

	/* Address */
	req.command = (page << 11) | (address << 5);
	req.transfer[0].tx_buf = &req.command;
	req.transfer[0].rx_buf = 0;
	req.transfer[0].len = 2;
	spi_message_add_tail(&req.transfer[0], &req.message);

	/* Data */
	req.transfer[1].tx_buf = &data;
	req.transfer[1].rx_buf = 0;
	req.transfer[1].len = 2;
	req.transfer[1].cs_change = CS_CHANGE(1);
	spi_message_add_tail(&req.transfer[1], &req.message);

	ret = spi_sync(tsc.spi, &req.message);
	if (!ret && req.message.status)
		ret = req.message.status;

	if (ret)
		printk(KERN_ERR "%s: error %i in SPI request\n",
				__FUNCTION__, ret);
}

static void tsc210x_reads_sync(int page, u8 startaddress, u16 *data, int numregs)
{
	static struct tsc210x_spi_req req;
	static struct spi_transfer transfer[6];
	int ret, i, j;

	BUG_ON(numregs + 1 > ARRAY_SIZE(transfer));

	spi_message_init(&req.message);
	req.transfer = transfer;
	i = 0;
	j = 0;

	/* Address */
	req.command = 0x8000 | (page << 11) | (startaddress << 5);
	req.transfer[i].tx_buf = &req.command;
	req.transfer[i].rx_buf = 0;
	req.transfer[i].len = 2;
	spi_message_add_tail(&req.transfer[i ++], &req.message);

	/* Data */
	while (j < numregs) {
		req.transfer[i].tx_buf = 0;
		req.transfer[i].rx_buf = &data[j ++];
		req.transfer[i].len = 2;
		req.transfer[i].cs_change = CS_CHANGE(j == numregs);
		spi_message_add_tail(&req.transfer[i ++], &req.message);
	}

	ret = spi_sync(tsc.spi, &req.message);
	if (!ret && req.message.status)
		ret = req.message.status;

	if (ret)
		printk(KERN_ERR "%s: error %i in SPI request\n",
				__FUNCTION__, ret);
}

static u16 tsc210x_read_sync(int page, u8 address)
{
	u16 ret;

	tsc210x_reads_sync(page, address, &ret, 1);

	return ret;
}

u16 tsc210x_read_reg(int page, u8 address)
{
	u16 ret;

	mutex_lock(&tsc.lock_sync);
	ret = tsc210x_read_sync(page, address);
	mutex_unlock(&tsc.lock_sync);

	return ret;
}
EXPORT_SYMBOL_GPL(tsc210x_read_reg);

/* Workqueue for TSC internal state machine */
static void tsc_work(struct work_struct *work)
{
#define Z2STAT (1<<7)
#define AXSTAT (3<<3)
#define T1STAT (1<<2)
	int status;

 	mutex_lock(&tsc.lock_sync);

	/* state of SM has to be changed ? */
	if (test_and_clear_bit(WORK_CHANGE_MODE, &tsc.todo)) {
		tsc210x_write_sync(TSC2102_TS_ADC_CTRL, TSC2102_ADC_ADST);
		/* change state of SM */
		if (tsc.state == SM_TEMP1) {
			tsc.state = SM_BAT_AUX;
			tsc210x_write_sync(TSC2102_TS_STATUS_CTRL,
				TSC2102_ADC_DAV);
			tsc210x_write_sync(TSC2102_TS_ADC_CTRL,
				TSC2102_ADC_SCAN_CONTROL);
		} else {
			tsc.state = SM_TEMP1;
			tsc210x_write_sync(TSC2102_TS_STATUS_CTRL,
				TSC2102_ADC_DAV);
			tsc210x_write_sync(TSC2102_TS_ADC_CTRL,
				TSC2102_ADC_T1_CONTROL);
		}
		/* restart mode timer */
		mod_timer(&tsc.mode_timer, jiffies +
				msecs_to_jiffies(tsc.mode_msecs));
	}
	status = tsc210x_read_sync(TSC2102_TS_STATUS_CTRL);

	if (status & Z2STAT) {
		/* get pendown status */
		int pendown = tsc210x_read_sync(TSC2102_TS_ADC_CTRL) & TSC2102_ADC_PSTCM;
		/* postponed next interrupt to avoid processor overload */
		msleep(3);
		tsc210x_reads_sync(TSC2102_TS_X, tsc.adc_data, 4);
		/* avoid sending data if pen is up*/
		if (tsc.coords_cb && pendown) {
			tsc.coords_cb(tsc.adc_data[0], tsc.adc_data[1],
					tsc.adc_data[2], tsc.adc_data[3]);
		}
	}

	if (status & AXSTAT) {
		tsc210x_reads_sync(TSC2102_TS_BAT1, tsc.adc_data, 4);
		if (tsc.ports_cb)
			tsc.ports_cb(tsc.adc_data[0], tsc.adc_data[1],
					tsc.adc_data[2]);
		tsc.bat[0] = tsc.adc_data[0];
		tsc.bat[1] = tsc.adc_data[1];
		tsc.aux[0] = tsc.adc_data[2];
		tsc.aux[1] = tsc.adc_data[3];
		/* switch to pintdav and TS control */
		tsc210x_write_sync(TSC2102_TS_ADC_CTRL,
				TSC2102_ADC_TS_CONTROL);
		tsc210x_write_sync(TSC2102_TS_STATUS_CTRL, TSC2102_ADC_PINTDAV);
	}
	else if (status & T1STAT) {
		tsc.temp = tsc210x_read_sync(TSC2102_TS_TEMP1);
		if (tsc.temp1_cb)
			tsc.temp1_cb(tsc.temp);
		/* switch to pintdav and TS control */
		tsc210x_write_sync(TSC2102_TS_ADC_CTRL,
				TSC2102_ADC_TS_CONTROL);
		tsc210x_write_sync(TSC2102_TS_STATUS_CTRL, TSC2102_ADC_PINTDAV);
	}

	mutex_unlock(&tsc.lock_sync);
}

#define tsc210x_cb_register_func(cb, cb_t)	\
int tsc210x_ ## cb(cb_t handler)	\
{	\
	spin_lock(&tsc.lock);	\
	\
	/* Lock the module */	\
	if (handler && !tsc.cb)	\
		if (!try_module_get(THIS_MODULE)) {	\
			printk(KERN_INFO "Failed to get TSC module\n");	\
		}	\
	if (!handler && tsc.cb)	\
		module_put(THIS_MODULE);	\
	\
	tsc.cb = handler;	\
	\
	spin_unlock(&tsc.lock);	\
	return 0;	\
}


tsc210x_cb_register_func(touch_cb, tsc210x_touch_t)
EXPORT_SYMBOL(tsc210x_touch_cb);
tsc210x_cb_register_func(coords_cb, tsc210x_coords_t)
EXPORT_SYMBOL(tsc210x_coords_cb);
tsc210x_cb_register_func(ports_cb, tsc210x_ports_t)
tsc210x_cb_register_func(temp1_cb, tsc210x_temp_t)

#ifdef DEBUG
static void tsc2102_print_dav(void)
{
	u16 status = tsc210x_read_sync(TSC2102_TS_STATUS_CTRL);
	if (status & 0x0fff)
		printk("TSC2102: data in");
	if (status & 0x0400)
		printk(" X");
	if (status & 0x0200)
		printk(" Y");
	if (status & 0x0100)
		printk(" Z1");
	if (status & 0x0080)
		printk(" Z2");
	if (status & 0x0040)
		printk(" BAT1");
	if (status & 0x0020)
		printk(" BAT2");
	if (status & 0x0010)
		printk(" AUX1");
	if (status & 0x0008)
		printk(" AUX2");
	if (status & 0x0004)
		printk(" TEMP1");
	if (status & 0x0002)
		printk(" TEMP2");
	if (status & 0x0001)
		printk(" KP");
	if (status & 0x0fff)
		printk(".\n");
}
#endif

static void tsc2102_mode(unsigned long data)
{
	struct tsc210x_dev *dev = (struct tsc210x_dev *) data;

	if (dev->state != SM_SUSPENDED) {
		/* request change of SM state */
		set_bit(WORK_CHANGE_MODE, &dev->todo);
		schedule_work(&dev->work);
	}
}

/* TSC has new data for us available.  */
static irqreturn_t tsc2102_handler(int irq, void *dev_id)
{
	struct tsc210x_dev *dev = (struct tsc210x_dev *) dev_id;

	if (dev->state != SM_SUSPENDED) {
		/* restart pen up timer */
		mod_timer(&tsc.ts_timer, jiffies +
				msecs_to_jiffies(tsc.ts_msecs));

		schedule_work(&dev->work);
	}

	return IRQ_HANDLED;
}

static void tsc2102_pressure(unsigned long data)
{
	struct tsc210x_dev *dev = (struct tsc210x_dev *) data;

	if (dev->touch_cb)
		dev->touch_cb(0);
}

#if defined(CONFIG_SOUND) || defined(CONFIG_SOUND_MODULE)

/*
 * Volume level values should be in the range [0, 127].
 * Higher values mean lower volume:
 *     0 -> 0dB attenuation, 127 -> -63,5 dB attenuation (steps of -0,5dB)
 */
void tsc210x_set_dac_volume(uint8_t left_ch, uint8_t right_ch)
{
	u16 val;

	if (tsc.kind == tsc2102) {
		/* On TSC2102, 127 means 0dB and 0 means -63,5 dB */
		if (left_ch == 0x00 || left_ch == 0x7f)
			left_ch ^= 0x7f;
		if (right_ch == 0x00 || right_ch == 0x7f)
			right_ch ^= 0x7f;
	}
	mutex_lock(&tsc.lock_sync);
	val = tsc210x_read_sync(TSC210X_DAC_GAIN_CTRL);
	/* ?? No check ?? */

	val &= (DGC_DALMU | DGC_DARMU);	/* Preserve mute-bits */
	val |= DGC_DALVL(left_ch) | DGC_DARVL(right_ch);

	tsc210x_write_sync(TSC210X_DAC_GAIN_CTRL, val);
	mutex_unlock(&tsc.lock_sync);
}
EXPORT_SYMBOL(tsc210x_set_dac_volume);

void tsc210x_set_dac_mute(int left_ch, int right_ch)
{
	u16 val;

	mutex_lock(&tsc.lock_sync);

	val = tsc210x_read_sync(TSC210X_DAC_GAIN_CTRL);
	/* ?? No check ?? */

	val &= DGC_DALVL(0x7f) | DGC_DARVL(0x7f); /* Preserve volume settings */
	if (left_ch)
		val |= DGC_DALMU;
	if (right_ch)
		val |= DGC_DARMU;

	tsc210x_write_sync(TSC210X_DAC_GAIN_CTRL, val);

	mutex_unlock(&tsc.lock_sync);
}
EXPORT_SYMBOL(tsc210x_set_dac_mute);

void tsc210x_get_dac_mute(int *left_ch, int *right_ch)
{
	u16 val;

	mutex_lock(&tsc.lock_sync);
	val = tsc210x_read_sync(TSC210X_DAC_GAIN_CTRL);
	mutex_unlock(&tsc.lock_sync);

	*left_ch  = !!(val & DGC_DALMU);
	*right_ch = !!(val & DGC_DARMU);
}
EXPORT_SYMBOL(tsc210x_get_dac_mute);

void tsc210x_set_deemphasis(int enable)
{
	u16 val;

	mutex_lock(&tsc.lock_sync);
	val = tsc210x_read_sync(TSC210X_CODEC_POWER_CTRL);

	if (enable)
		val &= ~CPC_DEEMPF;
	else
		val |= CPC_DEEMPF;

	tsc210x_write_sync(TSC210X_CODEC_POWER_CTRL, val);
	mutex_unlock(&tsc.lock_sync);
}
EXPORT_SYMBOL(tsc210x_set_deemphasis);

void tsc210x_set_bassboost(int enable)
{
	u16 val;

	mutex_lock(&tsc.lock_sync);
	val = tsc210x_read_sync(TSC210X_CODEC_POWER_CTRL);

	if (enable)
		val &= ~CPC_BASSBC;
	else
		val |= CPC_BASSBC;

	tsc210x_write_sync(TSC210X_CODEC_POWER_CTRL, val);
	mutex_unlock(&tsc.lock_sync);
}
EXPORT_SYMBOL(tsc210x_set_bassboost);

void tsc2101_set_micsel(uint8_t micsel)
{
	u16 sel;

	mutex_lock(&tsc.lock_sync);
	sel = tsc210x_read_sync(TSC2101_MIXER_PGA_CTRL);
	sel &= ~MPC_MICSEL(7); /* clear MICSEL */
	sel |= MPC_MICSEL(micsel);
	tsc210x_write_sync(TSC2101_MIXER_PGA_CTRL, sel);
	mutex_unlock(&tsc.lock_sync);
}
EXPORT_SYMBOL_GPL(tsc2101_set_micsel);

uint8_t tsc2101_get_micsel(void)
{
	return (uint8_t)((tsc210x_read_sync(TSC2101_MIXER_PGA_CTRL) >> 5) & 0x07);
}
EXPORT_SYMBOL_GPL(tsc2101_get_micsel);

void tsc2101_handset_mute(int muted)
{
	u16 val;

	mutex_lock(&tsc.lock_sync);
	val = tsc210x_read_sync(TSC2101_HANDSET_PGA_CTRL);
	if (muted)
		val |= ADMUT_HND;
	else
		val &= ~ADMUT_HND;
	tsc210x_write_sync(TSC2101_HANDSET_PGA_CTRL, val);
	mutex_unlock(&tsc.lock_sync);
}
EXPORT_SYMBOL_GPL(tsc2101_handset_mute);

/* 0 -> 0dB, 127 -> 59,5dB (0.5dB stepping) */
void tsc2101_set_handset_gain(uint8_t gain)
{
	u16 val;

	mutex_lock(&tsc.lock_sync);
	val = tsc210x_read_sync(TSC2101_HANDSET_PGA_CTRL);
	val &= ~ADPGA_HND(0xff); /* clear current gain */
	val |= ADPGA_HND(gain);
	tsc210x_write_sync(TSC2101_HANDSET_PGA_CTRL, val);
	mutex_unlock(&tsc.lock_sync);
}
EXPORT_SYMBOL_GPL(tsc2101_set_handset_gain);

void tsc2101_headset_mute(int muted)
{
	u16 val;

	mutex_lock(&tsc.lock_sync);
	val = tsc210x_read_sync(TSC2101_HEADSET_AUX_PGA_CTRL);
	if (muted)
		val |= ADMUT_HED;
	else
		val &= ~ADMUT_HED;
	tsc210x_write_sync(TSC2101_HEADSET_AUX_PGA_CTRL, val);
	mutex_unlock(&tsc.lock_sync);
}
EXPORT_SYMBOL_GPL(tsc2101_headset_mute);

/* 0 -> 0dB, 127 -> 59,5dB (0.5dB stepping) */
void tsc2101_set_headset_gain(uint8_t gain)
{
	u16 val;

	mutex_lock(&tsc.lock_sync);
	val = tsc210x_read_sync(TSC2101_HEADSET_AUX_PGA_CTRL);
	val &= ~ADPGA_HED(0xff); /* clear current gain */
	val |= ADPGA_HED(gain);
	tsc210x_write_sync(TSC2101_HEADSET_AUX_PGA_CTRL, val);
	mutex_unlock(&tsc.lock_sync);
}
EXPORT_SYMBOL_GPL(tsc2101_set_headset_gain);

/* {rate, divisor (for AC1), fsref} */
static const struct tsc210x_rate_info_s tsc210x_rates[] = {
	/* Fsref / 6.0 */
	{7350,	7,	1},
	{8000,	7,	0},
	/* Fsref / 5.5 */
	{7350,	6,	1},
	{8000,	6,	0},
	/* Fsref / 5.0 */
	{8820,	5,	1},
	{9600,	5,	0},
	/* Fsref / 4.0 */
	{11025,	4,	1},
	{12000,	4,	0},
	/* Fsref / 3.0 */
	{14700,	3,	1},
	{16000,	3,	0},
	/* Fsref / 2.0 */
	{22050,	2,	1},
	{24000,	2,	0},
	/* Fsref / 1.5 */
	{29400,	1,	1},
	{32000,	1,	0},
	/* Fsref */
	{44100,	0,	1},
	{48000,	0,	0},
	/* end of struct */
	{0,	0, 	0},
};

struct pll_values {
	u16 pll1;
	u16 pll2;
};

static void tsc210x_calculate_pll(struct pll_values* pll, unsigned int mclk, unsigned int fsref)
{
	if (fsref == 44100) {
	switch(mclk)
	{
		case 12288000:
			pll->pll1 = (PLL1_PLLEN | PLL1_Q_VAL(0) | PLL1_P_VAL(1) | PLL1_J_VAL(7));
			pll->pll2 = (PLL2_D_VAL(3500));
		break;

		case 16000000:
			pll->pll1 = (PLL1_PLLEN | PLL1_Q_VAL(0) | PLL1_P_VAL(1) | PLL1_J_VAL(5));
			pll->pll2 = (PLL2_D_VAL(6448));
		break;

		case 24000000:
			pll->pll1 = (PLL1_PLLEN | PLL1_Q_VAL(0) | PLL1_P_VAL(2) | PLL1_J_VAL(7));
			pll->pll2 = (PLL2_D_VAL(5264));
		break;

		default:
			pll->pll1 = 0x811c;
			pll->pll2 = (PLL2_D_VAL(5462));
		break;
	}
	} else { /* 48000 */
	switch(mclk)
	{
		case 12288000:
			/*pll->pll1 = (PLL1_Q_VAL(2)) -> doesn't work without enabling PLL */
			pll->pll1 = (PLL1_PLLEN | PLL1_Q_VAL(0) | PLL1_P_VAL(1) | PLL1_J_VAL(8));
			pll->pll2 = (PLL2_D_VAL(0));
		break;

		case 16000000:
			pll->pll1 = (PLL1_PLLEN | PLL1_Q_VAL(0) | PLL1_P_VAL(1) | PLL1_J_VAL(6));
			pll->pll2 = (PLL2_D_VAL(1440));
		break;

		case 24000000:
			pll->pll1 = (PLL1_PLLEN | PLL1_Q_VAL(0) | PLL1_P_VAL(2) | PLL1_J_VAL(8));
			pll->pll2 = (PLL2_D_VAL(1920));
		break;

		default:
			pll->pll1 = 0x8120;
			pll->pll2 = (PLL2_D_VAL(1920));
		break;
	}
	}
}

static void tsc210x_set_fsref(unsigned int fsref)
{
	struct pll_values pll;

	tsc210x_calculate_pll(&pll, tsc.mclk, fsref);
	/* Enable Phase-locked-loop & set up clock dividers */
	tsc210x_write_sync(TSC210X_PLL1_CTRL, pll.pll1);
	tsc210x_write_sync(TSC210X_PLL2_CTRL, pll.pll2);
}

int tsc210x_set_rate(int rate)
{
	int i;
	int val;
	const struct tsc210x_rate_info_s *rates;

	rates = tsc210x_rates;

	for (i = 0; rates[i].sample_rate; i ++)
		if (rates[i].sample_rate == rate)
			break;
	if (rates[i].sample_rate == 0) {
		printk(KERN_ERR "Unknown sampling rate %i.0 Hz\n", rate);
		return -EINVAL;
	}

	mutex_lock(&tsc.lock_sync);

	val = tsc210x_read_sync(TSC2102_AUDIO1_CTRL);
	if (val < 0) {
		printk(KERN_ERR "%s, err %d\n", __FUNCTION__, val);
		return val;
	}

	val &= ~AC1_DACFS(0xff) & ~AC1_ADCFS(0xff);
	val |= AC1_DACFS(rates[i].divisor);
	val |= AC1_ADCFS(rates[i].divisor);
	tsc210x_write_sync(TSC2102_AUDIO1_CTRL, val);

	val = tsc210x_read_sync(TSC2102_AUDIO3_CTRL);

	if (rates[i].fs_44k) {
		tsc210x_write_sync(TSC2102_AUDIO3_CTRL, val | AC3_REFFS);
		tsc210x_set_fsref(44100);
	} else {
		tsc210x_write_sync(TSC2102_AUDIO3_CTRL, val & ~AC3_REFFS);
		tsc210x_set_fsref(48000);
	}

	mutex_unlock(&tsc.lock_sync);

	return 0;
}
EXPORT_SYMBOL(tsc210x_set_rate);

/*
 * Perform basic set-up with default values and power the DAC on.
 */
void tsc210x_dac_power(int state)
{
	mutex_lock(&tsc.lock_sync);

	if (state) {
		/* 16-bit words, I2S mode, sample at Fsref */
		tsc210x_write_sync(TSC2102_AUDIO1_CTRL,
				AC1_WLEN(0) | AC1_DATFM(0) | AC1_DACFS(0));
		/* Keyclicks off, soft-stepping at normal rate */
		tsc210x_write_sync(TSC2102_AUDIO2_CTRL, TSC210X_KEYCLICK_OFF);
		/* 44.1 kHz Fsref, continuous transfer mode, master DAC */
		tsc210x_write_sync(TSC2102_AUDIO3_CTRL, 0x2800);
		if (tsc.kind == tsc2101) {
			/* Soft-stepping enabled + MICBIAS_xx = 2.5V */
			tsc210x_write_sync(TSC2102_AUDIO4_CTRL,
					0x0000 | AC4_MB_HED(1));
		} else {
			/* Soft-stepping enabled */
			tsc210x_write_sync(TSC2102_AUDIO4_CTRL, 0x0000);
		}
		/* PLL generates 44.1 kHz */
		tsc210x_set_fsref(44100);
		/* Codec & DAC power up, virtual ground disabled */
		tsc210x_write_sync(TSC210X_CODEC_POWER_CTRL,
			(tsc.kind == tsc2101) ? TSC2101_DAC_ON : TSC2102_DAC_ON);
	} else {
		/* All off */
		tsc210x_write_sync(TSC2102_AUDIO2_CTRL, TSC210X_KEYCLICK_OFF);
		tsc210x_write_sync(TSC210X_PLL1_CTRL, TSC210X_PLL1_OFF);
	}

	mutex_unlock(&tsc.lock_sync);
}
EXPORT_SYMBOL(tsc210x_dac_power);

void tsc210x_set_i2s_master(int state)
{
	uint16_t val;
	mutex_lock(&tsc.lock_sync);

	val = tsc210x_read_sync(TSC2102_AUDIO3_CTRL);
	/* if (val < 0) {    ?? no check ??
		printk(KERN_ERR "%s, err %d\n", __FUNCTION__, val);
		return;
	}*/

	if (state)
		tsc210x_write_sync(TSC2102_AUDIO3_CTRL, val | AC3_SLVMS);
	else
		tsc210x_write_sync(TSC2102_AUDIO3_CTRL, val & ~AC3_SLVMS);

	mutex_unlock(&tsc.lock_sync);
}
EXPORT_SYMBOL(tsc210x_set_i2s_master);
#endif	/* CONFIG_SOUND || CONFIG_SOUND_MODULE */

static void tsc210x_reset(void)
{
	tsc210x_write_sync(TSC2102_TS_RESET_CTRL, TSC2102_RESET);
}

static int tsc2102_configure(struct tsc210x_dev *dev)
{
	/* Reset the chip */
	tsc210x_reset();

	/* Reference mode, 100 usec delay, 2.5V reference */
	if (dev->pdata->use_internal)
		tsc210x_write_sync(TSC2102_TS_REF_CTRL, TSC2102_ADC_REF);
	else
		tsc210x_write_sync(TSC2102_TS_REF_CTRL, TSC2102_ADC_EXT_REF);

	/* 84 usec precharge time, 32 usec sense time */
	tsc210x_write_sync(TSC2102_TS_CONFIG_CTRL, TSC2102_CONFIG_TIMES);

	/* PINT/DAV acts as PINT */
	tsc210x_write_sync(TSC2102_TS_STATUS_CTRL, TSC2102_ADC_DAV);

#if defined(CONFIG_SOUND) || defined(CONFIG_SOUND_MODULE)
	/* 16-bit words, I2S mode, sample at Fsref */
	tsc210x_write_sync(TSC2102_AUDIO1_CTRL, TSC210x_AUDIO1_INIT_VAL);
	/* Keyclicks off, soft-stepping at normal rate */
	tsc210x_write_sync(TSC2102_AUDIO2_CTRL, TSC210X_KEYCLICK_OFF);
	/* 48 kHz Fsref, continuous transfer mode, master DAC */
	tsc210x_write_sync(TSC2102_AUDIO3_CTRL, 0x0800);
	/* Soft-stepping enabled */
	tsc210x_write_sync(TSC2102_AUDIO4_CTRL, 0x0000);
	/* FSRef = 44,1k */
	tsc210x_set_fsref(44100);

	if (tsc.kind == tsc2101) {
		tsc210x_write_sync(TSC2101_AUDIO5_CTRL, /*AC5_AST2SPK1 |*/ AC5_DAC2SPK1(1) | AC5_DAC2SPK2(2) | AC5_HDSCPTC);
		tsc210x_write_sync(TSC2101_MIXER_PGA_CTRL, 0xc500 /* reset value */ | MPC_MICSEL(1) | MPC_MICADC);
		tsc210x_write_sync(TSC2101_HEADSET_AUX_PGA_CTRL, ADPGA_HED(0));
		tsc210x_write_sync(TSC2101_HANDSET_PGA_CTRL, 0x0000);
		tsc210x_write_sync(TSC210X_CODEC_POWER_CTRL, 0xe2fc);
	} else {
		tsc210x_write_sync(TSC210X_CODEC_POWER_CTRL, TSC2102_DAC_ON);
	}
	/* Un-mute + volume max */
	if (tsc.kind == tsc2101)
		tsc210x_write_sync(TSC210X_DAC_GAIN_CTRL, 0x0000);
	else
		tsc210x_write_sync(TSC210X_DAC_GAIN_CTRL, 0x7F7F);
#endif /* CONFIG_SOUND || CONFIG_SOUND_MODULE */

	/* Init mode state machine */
	dev->state = SM_BAT_AUX;

	/* Start mode timer */
	mod_timer(&dev->mode_timer, jiffies +
			msecs_to_jiffies(dev->mode_msecs));

	return 0;
}

/*
 * Retrieves chip revision.  Should be 1 for TSC2102 and 4 for TSC2101
 */
static int tsc210x_get_revision(void)
{
	return AC3_REVID(tsc210x_read_sync(TSC2102_AUDIO3_CTRL));
}

int tsc210x_is_tsc2101(void)
{
	return tsc.kind == tsc2101;
}
EXPORT_SYMBOL(tsc210x_is_tsc2101);

int tsc210x_is_tsc2102(void)
{
	return tsc.kind == tsc2102;
}
EXPORT_SYMBOL(tsc210x_is_tsc2102);

/*
 * Emit a short keyclick typically in order to give feedback to
 * user on specific events.
 *
 * amplitude must be between 0 (lowest) and 2 (highest).
 * freq must be between 0 (corresponds to 62.5 Hz) and 7 (8 kHz).
 * length should be between 2 and 32 periods.
 *
 * This function sleeps but doesn't sleep until the sound has
 * finished.
 */
void tsc210x_keyclick(int amplitude, int freq, int length)
{
	u16 val;

	mutex_lock(&tsc.lock_sync);
	val = tsc210x_read_sync(TSC2102_AUDIO2_CTRL);
	/* Keeps current control values: */
	val &= AC2_KCLEN | AC2_APGASS | AC2_DLGAF | AC2_DRGAF | AC2_DASTC | AC2_ADGAF;

	/* Set amplitude */
	switch (amplitude) {
		case 1:
			val |= AC2_KCLAC(4);
		break;

		case 2:
			val |= AC2_KCLAC(7);
		break;

		default:
		break;
	}

	/* Frequency */
	val |= AC2_KCLFRQ(0x7);

	/* Round to nearest supported length */
	if (length > 8)
		val |= AC2_KCLLN(4);
	else if (length > 6)
		val |= AC2_KCLLN(3);
	else if (length > 4)
		val |= AC2_KCLLN(2);
	else if (length > 2)
		val |= AC2_KCLLN(1);

	/* Enable keyclick */
	val |= AC2_KCLEN;

	tsc210x_write_sync(TSC2102_AUDIO2_CTRL, val);
	mutex_unlock(&tsc.lock_sync);
}

#if defined(CONFIG_HWMON) || defined(CONFIG_HWMON_MODULE)
#define TSC2102_INPUT(devname, field)	\
static ssize_t show_ ## devname(struct device *dev,	\
		struct device_attribute *devattr, char *buf)	\
{	\
	struct tsc210x_dev *devhwmon = dev_get_drvdata(dev);	\
	int value = devhwmon->field;	\
	return sprintf(buf, "%i\n", value); \
}	\
static DEVICE_ATTR(devname ## _input, S_IRUGO, show_ ## devname, NULL);

TSC2102_INPUT(in0, bat[0])
TSC2102_INPUT(in1, bat[1])
TSC2102_INPUT(in2, aux[0])
TSC2102_INPUT(in3, aux[1])
TSC2102_INPUT(in4, temp)

static ssize_t show_temp1(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	struct tsc210x_dev *devhwmon = dev_get_drvdata(dev);
	int value;

	value = (-devhwmon->temp*7 + 8400)*50 ;	/* Celcius millidegree */

	return sprintf(buf, "%i\n", value);
}
static DEVICE_ATTR(temp1_input, S_IRUGO, show_temp1, NULL);
#endif	/* CONFIG_HWMON */

#ifdef CONFIG_APM
static void tsc2102_get_power_status(struct apm_power_info *info)
{
	tsc.pdata->apm_report(info, tsc.bat);
}
#endif


#ifdef CONFIG_PM
/*
 * Suspend the chip.
 */
static int
tsc210x_suspend(struct spi_device *spi, pm_message_t state)
{
	int reg;
	unsigned long timeout;
	struct tsc210x_dev *dev = dev_get_drvdata(&spi->dev);
	if (!dev)
		return 0;

	mutex_lock(&dev->lock_sync);

	dev->state = SM_SUSPENDED;
	del_timer(&dev->mode_timer);
	del_timer(&dev->ts_timer);
	flush_scheduled_work();

	tsc210x_write_sync(TSC2102_TS_STATUS_CTRL, TSC2102_ADC_PINTDAV);

	if (dev->touch_cb)
		dev->touch_cb(0);

	/* shut down audio parts (start with DACs) */
	/* mute DACs */
	reg = tsc210x_read_sync(TSC210X_DAC_GAIN_CTRL);
	tsc210x_write_sync(TSC210X_DAC_GAIN_CTRL, reg | DGC_DALMU | DGC_DARMU);
	/* speaker power down */
	reg = tsc210x_read_sync(TSC210X_CODEC_POWER_CTRL);
	tsc210x_write_sync(TSC210X_CODEC_POWER_CTRL, reg | CPC_SP1PWDN | CPC_SP2PWDN);
	/* wait until soft stepping is complete */
	timeout = jiffies + HZ;
	while ((tsc210x_read_sync(TSC210X_CODEC_POWER_CTRL) & (CPC_RDAPWDF | CPC_LDAPWDF)) !=
			(CPC_RDAPWDF | CPC_LDAPWDF)) {
		if (time_after(jiffies, timeout)) {
			dev_err(&spi->dev, "unable to shut down speaker's DACs\n");
			break;
		}
	}
	/* finally power down DAC */
	reg = tsc210x_read_sync(TSC210X_CODEC_POWER_CTRL);
	tsc210x_write_sync(TSC210X_CODEC_POWER_CTRL, reg | CPC_DAPWDN);

	/* ADC*/
	if (tsc.kind == tsc2101) {
		/* Headset mute */
		reg = tsc210x_read_sync(TSC2101_HEADSET_AUX_PGA_CTRL);
		tsc210x_write_sync(TSC2101_HEADSET_AUX_PGA_CTRL, reg | ADMUT_HED);
		/* Handdset mute */
		reg = tsc210x_read_sync(TSC2101_HANDSET_PGA_CTRL);
		tsc210x_write_sync(TSC2101_HANDSET_PGA_CTRL, reg | ADMUT_HND);
		/* cell mute */
		reg = tsc210x_read_sync(TSC2101_CELL_BUZZER_PGA_CTRL);
		tsc210x_write_sync(TSC2101_CELL_BUZZER_PGA_CTRL, reg | CPBGC_MUT_CP |
								CPBGC_MUT_BU);
		/* shut down mic bias, sidetone and vgnd */
		reg = tsc210x_read_sync(TSC210X_CODEC_POWER_CTRL);
		tsc210x_write_sync(TSC210X_CODEC_POWER_CTRL, reg |
						CPC_MBIAS_HND | CPC_MBIAS_HED |
						CPC_ASTPWD | CPC_VGPWDN |
						CPC_ADPWDN);
		/* wait until power down is complete */
		timeout = jiffies + HZ;
		while ((tsc210x_read_sync(TSC210X_CODEC_POWER_CTRL) & (CPC_ASTPWF|CPC_ADPWDF)) !=
				(CPC_ASTPWF | CPC_ADPWDF)) {
			if (time_after(jiffies, timeout)) {
				dev_err(&spi->dev, "unable to shut down ADC %x\n", tsc210x_read_sync(TSC210X_CODEC_POWER_CTRL));
				break;
			}
		}
	}

	dev->spi->dev.power.power_state = state;

	mutex_unlock(&dev->lock_sync);

	return 0;
}

/*
 * Resume chip operation.
 */
static int tsc210x_resume(struct spi_device *spi)
{
	struct tsc210x_dev *dev = dev_get_drvdata(&spi->dev);

	if (!dev)
		return 0;

	mutex_lock(&dev->lock_sync);

	dev->spi->dev.power.power_state = PMSG_ON;
	/* Init mode state machine */
	dev->state = SM_BAT_AUX;
	tsc210x_write_sync(TSC2102_TS_STATUS_CTRL, TSC2102_ADC_DAV);

	mutex_unlock(&dev->lock_sync);

	/* Start mode timer */
	mod_timer(&dev->mode_timer, jiffies +
			msecs_to_jiffies(dev->mode_msecs));

	return 0;
}
#else
#define tsc210x_suspend	NULL
#define tsc210x_resume	NULL
#endif

/* REVISIT don't make these static */
static struct platform_device tsc210x_ts_device = {
	.name 		= "tsc210x-ts",
	.id 		= -1,
};

/* TODO
static struct platform_device tsc210x_hwmon_device = {
	.name           = "tsc210x-hwmon",
	.id             = -1,
};
*/

static struct platform_device tsc210x_alsa_device = {
	.name 		= "tsc210x-alsa",
	.id 		= -1,
};

static int tsc210x_probe(struct spi_device *spi)
{
	struct tsc210x_config *pdata = spi->dev.platform_data;
	struct spi_transfer *spi_buffer;
	int err = -EINVAL;
	u16 rev_id = 0;

	if (!pdata) {
		printk(KERN_ERR "TSC210x: Platform data not supplied\n");
		return -ENOENT;
	}

	if (!spi->irq) {
		printk(KERN_ERR "TSC210x: Invalid irq value\n");
		return -ENOENT;
	}

	tsc.pdata = pdata;
	tsc.ts_msecs = 30;
	tsc.mode_msecs = 500;
	tsc.todo = 0;
	tsc.spi = spi;
	INIT_WORK(&tsc.work, tsc_work);

	/* Allocate enough struct spi_transfer's for all requests */
	spi_buffer = kzalloc(sizeof(struct spi_transfer) * 16, GFP_KERNEL);
	if (!spi_buffer) {
		printk(KERN_ERR "TSC2102: No memory for SPI buffers\n");
		return -ENOMEM;
	}

	spin_lock_init(&tsc.lock);
	mutex_init(&tsc.lock_sync);
	mutex_lock(&tsc.lock_sync);

	/* allocate GPIO / IRQ if necessary */
	if (pdata->init)
		pdata->init();

#ifdef CONFIG_TSC_SLAVE
	tsc.bclk_ck = clk_get(0, "bclk"); TODO add clock mechanism to i.MX ???
	if (!tsc.bclk_ck) {
		printk(KERN_ERR "Unable to get the clock BCLK\n");
		err = -EPERM;
		goto done;
	}

	clk_enable(tsc.bclk_ck);
#else
	/* Retrieve Master Clock from platform data */
	if (pdata->mclk)
		tsc.mclk = pdata->mclk;
	else
		tsc.mclk = 16000000; /* 16 MHz by default */
#endif /* CONFIG_TSC_SLAVE */

	if (request_irq(spi->irq, tsc2102_handler, IRQF_DISABLED /*| IRQF_SHARED*/ | IRQF_TRIGGER_FALLING, "tsc2102", &tsc)) {
		printk(KERN_ERR "Couldn't allocate touchscreen IRQ (%d)\n", spi->irq);
		err = -EINVAL;
		goto err_clk;
	}

	setup_timer(&tsc.ts_timer,
			tsc2102_pressure, (unsigned long) &tsc);
	setup_timer(&tsc.mode_timer,
			tsc2102_mode, (unsigned long) &tsc);

	/* Set up the communication bus */
	dev_set_drvdata(&spi->dev, &tsc);
	spi->dev.power.power_state = PMSG_ON;
	spi->mode = SPI_MODE_1;
	spi->bits_per_word = 16;
	err = spi_setup(spi);
	if (err) {
		dev_err(&spi->dev, "unable to setup SPI\n");
		goto err_timer;
	}

	/* Now try to detect the chip, make first contact */
	tsc210x_reset(); /* Seems like TSC can be messed up and needs a reset even to get revision */
	rev_id = tsc210x_get_revision();
	if (rev_id == 0x01) {
		printk("TSC2102 detected\n");
		tsc.kind = tsc2102;
	} else if (rev_id == 0x04) {
		printk("TSC2101 detected\n");
		tsc.kind = tsc2101;
	} else {
		printk(KERN_ERR "No TI TSC210x chip found! Bad revision: %x\n", rev_id);
		goto err_timer;
	}

	err = tsc2102_configure(&tsc);
	if (err)
		goto err_timer;

	/* Register devices controlled by TSC2101/TSC2102 */
	tsc210x_ts_device.dev.platform_data = pdata;
	tsc210x_ts_device.dev.parent = &spi->dev;
	err = platform_device_register(&tsc210x_ts_device);
	if (err)
		goto err_timer;

	tsc210x_alsa_device.dev.platform_data = pdata->alsa_config;
	tsc210x_alsa_device.dev.parent = &spi->dev;
	err = platform_device_register(&tsc210x_alsa_device);
	if (err)
		goto err_ts;

#if defined(CONFIG_HWMON) || defined(CONFIG_HWMON_MODULE)
	tsc.hwmondev = hwmon_device_register(&spi->dev);
	if (IS_ERR(tsc.hwmondev)) {
		printk(KERN_ERR "tsc2102_hwmon: Device registration failed\n");
		err = PTR_ERR(tsc.hwmondev);
		goto err_alsa;
	}

	if (pdata->monitor & TSC_BAT1)
		err |= device_create_file(&spi->dev, &dev_attr_in0_input);
	if (pdata->monitor & TSC_BAT2)
		err |= device_create_file(&spi->dev, &dev_attr_in1_input);
	if (pdata->monitor & TSC_AUX1)
		err |= device_create_file(&spi->dev, &dev_attr_in2_input);
	if (pdata->monitor & TSC_AUX2)
		err |= device_create_file(&spi->dev, &dev_attr_in3_input);
	if (pdata->monitor & TSC_TEMP) {
		err |= device_create_file(&spi->dev, &dev_attr_temp1_input);
		err |= device_create_file(&spi->dev, &dev_attr_in4_input);
	}

	if (err)
		printk(KERN_ERR "tsc2102_hwmon: Creating one or more "
				"attribute files failed\n");
	err = 0;	/* Not fatal */
#endif

#ifdef CONFIG_APM
	if (pdata->apm_report)
		apm_get_power_status = tsc2102_get_power_status;
#endif
	if (!err)
		goto done;

err_alsa:
	platform_device_unregister(&tsc210x_alsa_device);
err_ts:
	platform_device_unregister(&tsc210x_ts_device);
err_timer:
	del_timer(&tsc.ts_timer);
	del_timer(&tsc.mode_timer);
	flush_scheduled_work();
	dev_set_drvdata(&spi->dev, NULL);
	free_irq(spi->irq, &tsc);
err_clk:
/*	clk_disable(tsc.bclk_ck); TODO add clock mechanism to i.MX ???
	clk_put(tsc.bclk_ck);*/
done:
	mutex_unlock(&tsc.lock_sync);

	return err;
}

static int tsc210x_remove(struct spi_device *spi)
{
	struct tsc210x_dev *dev = dev_get_drvdata(&spi->dev);
	mutex_lock(&dev->lock_sync);

	platform_device_unregister(&tsc210x_ts_device);
	platform_device_unregister(&tsc210x_alsa_device);

	dev_set_drvdata(&spi->dev, NULL);

	/* Release the GPIO/IRQ */
	if (dev->pdata->exit)
		dev->pdata->exit();

#ifdef CONFIG_TSC_SLAVE
	/* Release the BCLK */
	clk_disable(dev->bclk_ck);
	clk_put(dev->bclk_ck);
#endif

	del_timer(&tsc.mode_timer);
	del_timer(&tsc.ts_timer);
	flush_scheduled_work();

#if defined(CONFIG_HWMON) || defined(CONFIG_HWMON_MODULE)
	hwmon_device_unregister(dev->hwmondev);
#endif

#ifdef CONFIG_APM
	apm_get_power_status = 0;
#endif
	free_irq(spi->irq, &tsc);

	mutex_unlock(&dev->lock_sync);

	return 0;
}

static struct spi_driver tsc210x_driver = {
	.probe		= tsc210x_probe,
	.remove		= tsc210x_remove,
	.suspend	= tsc210x_suspend,
	.resume		= tsc210x_resume,
	.driver		= {
		.name	= "tsc210x",
		.owner	= THIS_MODULE,
		.bus	= &spi_bus_type,
	},
};

static char __initdata banner[] = KERN_INFO "TI TSC210x driver initializing\n";

static int __init tsc210x_init(void)
{
	int err;

	printk(banner);
	err = spi_register_driver(&tsc210x_driver);

	return err;
}

static void __exit tsc210x_exit(void)
{
	spi_unregister_driver(&tsc210x_driver);
}

module_init(tsc210x_init);
module_exit(tsc210x_exit);

EXPORT_SYMBOL(tsc210x_read_sync);
EXPORT_SYMBOL(tsc210x_reads_sync);
EXPORT_SYMBOL(tsc210x_write_sync);
EXPORT_SYMBOL(tsc210x_keyclick);

MODULE_AUTHOR("Andrzej Zaborowski, Nicolas Colombain, Julien Boibessot");
MODULE_DESCRIPTION("Interface driver for TI TSC210x chips.");
MODULE_LICENSE("GPL");
