 /*
 * apf27-dev-ext-apw.c
 * Support for AFP27Dev's wireless extension board (APW).
 *
 * Copyright (C) 2009-2010 Armadeus Systems
 * Authors: Julien Boibessot <julien.boibessot@armadeus.com>
 *          Nicolas Colombain <nicolas.colombain@armadeus.com>
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

#include <linux/platform_device.h>
#include <linux/delay.h>
#include <mach/common.h>
#include <mach/iomux-mx1-mx2.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include "devices.h"


/* An optional GSM module can be added to wireless extension board */
#ifdef CONFIG_MACH_APF27_DEV_EXT_WIRELESS_GSM

#define GSM_POK_IN	(GPIO_PORTE | 11)	/* UART3_RTS */

static int apw_gsm_pins[] = {
	(GSM_POK_IN | GPIO_OUT | GPIO_GPIO) /* GSM_PWR_ON_not */
};

static int apw_gsm_init(void)
{
	int res = mxc_gpio_setup_multiple_pins(apw_gsm_pins,
			ARRAY_SIZE(apw_gsm_pins), "GSM");
	gpio_set_value(GSM_POK_IN, 0); /* start GSM */
	mdelay(5000);
	gpio_set_value(GSM_POK_IN, 1); /* start GSM */

	return res;
}
#if 0
static int apw_gsm_exit(void)
{
	mxc_gpio_release_multiple_pins(apw_gsm_pins, ARRAY_SIZE(apw_gsm_pins));

	return 0;
}
#endif
#endif /* CONFIG_MACH_APF27_DEV_EXT_WIRELESS_GSM */


/* An optionnal GPS module can be mounted on wireless extension board */
#ifdef CONFIG_MACH_APF27_DEV_EXT_WIRELESS_GPS

#define GPS_WAKEUP	(GPIO_PORTB | 20)	/* CSI_VSYNC_UART5_CTS */
#define GPS_TIMEPULSE	(GPIO_PORTB | 21)	/* CSI_HSYNC_UART5_RTS */

static int apw_gps_pins[] = {
	(GPS_WAKEUP | GPIO_OUT | GPIO_GPIO),
	(GPS_TIMEPULSE | GPIO_IN | GPIO_GPIO),
};

static int apw_gps_init(void)
{
	int res = mxc_gpio_setup_multiple_pins(apw_gps_pins,
			ARRAY_SIZE(apw_gps_pins), "GPS");
	gpio_set_value(GPS_WAKEUP, 1); /* start GPS */

	return res;
}
#if 0
static int apw_gps_exit(void)
{
	mxc_gpio_release_multiple_pins(apw_gps_pins, ARRAY_SIZE(apw_gps_pins));

	return 0;
}
#endif
#endif /* CONFIG_MACH_APF27_DEV_EXT_WIRELESS_GPS */


/* An optional Bluetooth + WiFi module (Wi2Wi) can be mounted on wireless
   extension board */
#ifdef CONFIG_MACH_APF27_DEV_EXT_WIRELESS_BT_WIFI

#define BT_RESETn	(GPIO_PORTB | 14)	/* shared with CSI_D4;
						   driven by userspace */
#define WIFI_PWRDNn	(GPIO_PORTE | 10)	/* UART3_CTS */

static int apw_bt_wifi_pins[] = {
	(WIFI_PWRDNn | GPIO_OUT | GPIO_GPIO),
};

/* SDHC1 is used for WiFi */
static int mxc_sdhc1_pins[] = {
	PE18_PF_SDHC1_D0,
	PE19_PF_SDHC1_D1,
	PE20_PF_SDHC1_D2,
	PE21_PF_SDHC1_D3,
	PE22_PF_SDHC1_CMD,
	PE22_PF_SDHC1_CLK,
};

static int apw_sdhc1_init(struct device *dev, irq_handler_t handler, void *data)
{
	return mxc_gpio_setup_multiple_pins(mxc_sdhc1_pins,
				ARRAY_SIZE(mxc_sdhc1_pins),
				"SDHC1");
}

static void apw_sdhc1_exit(struct device *pdev, void *data)
{
	mxc_gpio_release_multiple_pins(mxc_sdhc1_pins, ARRAY_SIZE(mxc_sdhc1_pins));
}

static struct imxmmc_platform_data apw_sdhc1_pdata = {
	.init = apw_sdhc1_init,
	.exit = apw_sdhc1_exit,
};

/* UART6 is used for Bluetooth (init done in apf27-dev.c depending on
   Linux config) */

static int apw_bt_wifi_init(void)
{
	int res;

	gpio_set_value(WIFI_PWRDNn, 1);	/* WiFi on */
	res = mxc_gpio_setup_multiple_pins(apw_bt_wifi_pins,
			ARRAY_SIZE(apw_bt_wifi_pins), "BT_WIFI");
	if (!res)
		mxc_register_device(&mxc_sdhc_device0, &apw_sdhc1_pdata);

	return res;
}
#endif /* CONFIG_MACH_APF27_DEV_EXT_WIRELESS_BT_WIFI */


void apf27dev_extension_resume(void)
{
#ifdef CONFIG_PM
# ifdef CONFIG_MACH_APF27_DEV_EXT_WIRELESS_WIFI
	gpio_set_value(WIFI_PWRDNn, 1);
# endif
# ifdef CONFIG_MACH_APF27_DEV_EXT_WIRELESS_GPS
	gpio_set_value(GPS_WAKEUP, 1);
# endif
#endif
}

void apf27dev_extension_suspend(void)
{
#ifdef CONFIG_PM
# ifdef CONFIG_MACH_APF27_DEV_EXT_WIRELESS_WIFI
	gpio_set_value(WIFI_PWRDNn, 0);
# endif
# ifdef CONFIG_MACH_APF27_DEV_EXT_WIRELESS_GPS
	gpio_set_value(GPS_WAKEUP, 0);
# endif
#endif
}


void __init apf27dev_extension_init(void)
{
	printk("    Registering APW ressources:");

#ifdef CONFIG_MACH_APF27_DEV_EXT_WIRELESS_GSM
	apw_gsm_init();
#endif
#ifdef CONFIG_MACH_APF27_DEV_EXT_WIRELESS_GPS
	apw_gps_init();
#endif
#ifdef CONFIG_MACH_APF27_DEV_EXT_WIRELESS_BT_WIFI
	apw_bt_wifi_init();
#endif

	printk("done\n");
}

