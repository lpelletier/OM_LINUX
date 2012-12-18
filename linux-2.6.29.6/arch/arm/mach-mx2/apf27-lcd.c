/*
 * apf27-lcd.c
 *
 * Handle support of multiple LCDs on the APF27Dev
 *
 * Copyright (C) 2009 Armadeus Systems
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


#include <mach/iomux-mx1-mx2.h>
#include <mach/gpio.h>
#ifdef CONFIG_FB_MXC /* Freescale Framebuffer */
#include <mach/imx_fb.h>
#include <mach/imxfb.h>
#endif
#include <linux/fb.h>
#include <linux/delay.h>
#include <mach/common.h>
#include "devices.h"


#ifdef CONFIG_IMX_BACKLIGHT_MODULE
#define CONFIG_IMX_BACKLIGHT
#endif

#ifdef CONFIG_FB_IMX_OPTREX_T51638D084_TFT
# define LCD_NAME	"Optrex-T51638D084"
/* PS is used as LCD Display ON/OFF */
# define LCD_USES_PS_AS_GPIO
/* activates LCD at startup */
# define LCD_POWER_GPIO	(GPIO_PORTA | 26)
# define LCD_POWER_ON	(1)
#endif

#ifdef CONFIG_FB_IMX_SHARP_LQ057_TFT
# define LCD_NAME	"Sharp-LQ057"
/* CONTRAST is used as backlight's ON/OFF */
# define LCD_USES_CONTRAST_AS_GPIO
/* activates backlight at startup */
# define LCD_BACKLIGHT_GPIO	(GPIO_PORTA | 30)
# define LCD_BACKLIGHT_ON	(0)
#endif

#ifdef CONFIG_FB_IMX_SHARP_LQ043_TFT
# define LCD_NAME	"Sharp-LQ043"
/* OE_ACD is used as LCD Display ON/OFF */
# define LCD_USES_OE_ACD_AS_GPIO
/* activates LCD at startup */
# define LCD_POWER_GPIO (GPIO_PORTA | 31)
# define LCD_POWER_ON	(1)
#endif

#ifdef CONFIG_FB_IMX_CHIMEI_LW700AT9003_TFT
# define LCD_NAME	"Chimei-LW700AT9003"
#endif

#ifdef CONFIG_FB_IMX_HITACHI_TX12D17VM1BDP_TFT
# define LCD_NAME	"Hitachi-TX12D17VM1BDP"
#endif

/* Please always let the custom LCD in this (last) position: */
#ifdef CONFIG_FB_IMX_CUSTOM_LCD
# define LCD_NAME	"Custom" /* Clone of LQ043 by default */
/* OE_ACD is used as LCD Display ON/OFF */
# define LCD_USES_OE_ACD_AS_GPIO
/* activates LCD at startup */
# define LCD_POWER_GPIO (GPIO_PORTA | 31)
# define LCD_POWER_ON	(1)
#endif
/* Please let these 2 lines here
*/

#if defined(CONFIG_FB_MXC) || defined(CONFIG_FB_IMX)
static int mxc_lcd_pins[] = {
	PA5_PF_LSCLK,
	PA6_PF_LD0,
	PA7_PF_LD1,
	PA8_PF_LD2,
	PA9_PF_LD3,
	PA10_PF_LD4,
	PA11_PF_LD5,
	PA12_PF_LD6,
	PA13_PF_LD7,
	PA14_PF_LD8,
	PA15_PF_LD9,
	PA16_PF_LD10,
	PA17_PF_LD11,
	PA18_PF_LD12,
	PA19_PF_LD13,
	PA20_PF_LD14,
	PA21_PF_LD15,
	PA22_PF_LD16,
	PA23_PF_LD17,
	PA24_PF_REV,
	PA25_PF_CLS,
#ifdef LCD_USES_PS_AS_GPIO
	(GPIO_PORTA | 26 | GPIO_OUT| GPIO_GPIO),
#else
	PA26_PF_PS,
#endif
	PA27_PF_SPL_SPR,
	PA28_PF_HSYNC,
	PA29_PF_VSYNC,
#ifdef LCD_USES_CONTRAST_AS_GPIO
	(GPIO_PORTA | 30 | GPIO_OUT| GPIO_GPIO),
#else
	PA30_PF_CONTRAST,
#endif
#ifdef LCD_USES_OE_ACD_AS_GPIO
	(GPIO_PORTA | 31 | GPIO_OUT| GPIO_GPIO)
#else
	PA31_PF_OE_ACD
#endif
};

static int apf27_fb_init(struct platform_device *pdev)
{
#ifdef CONFIG_FB_MXC
# ifdef LCD_POWER_GPIO
	/* activates LCD power at startup */
	gpio_set_value(LCD_POWER_GPIO, LCD_POWER_ON);
# endif
# ifdef LCD_BACKLIGHT_GPIO
	/* activates backlight at startup */
	gpio_set_value(LCD_BACKLIGHT_GPIO, LCD_BACKLIGHT_ON);
# endif
#endif

	return mxc_gpio_setup_multiple_pins(mxc_lcd_pins, ARRAY_SIZE(mxc_lcd_pins), "LCD");
}

static int apf27_fb_exit(struct platform_device *pdev)
{
	mxc_gpio_release_multiple_pins(mxc_lcd_pins, ARRAY_SIZE(mxc_lcd_pins));

	return 0;
}

/* Freescale driver */
#ifdef CONFIG_FB_MXC
static struct mxc_fb_platform_data apf27_fb_data = {
	.mode = LCD_NAME,
	.init = apf27_fb_init,
	.exit = apf27_fb_exit,
};
#endif

/* Mainline driver */
#ifdef CONFIG_FB_IMX
#include "../mach-imx/apf9328_lcd_config.h"
#endif

# ifdef CONFIG_IMX_BACKLIGHT
static struct imxbl_machinfo imx_bl_machinfo = {
	.max_intensity      = 0xff,
	.default_intensity  = 0x90,
	.limit_mask         = 0x7f, /* when battery is low */
	/* set_bl_intensity = put a function here if you want to overload default one, */
};

static struct platform_device imxbl_device = {
	.name       = "imx-bl",
	.dev        = {
		/* .parent = &imxfb_device.dev, crash kernel even if EXPORT_SYMBOL() is done in generic.c */
		.platform_data  = &imx_bl_machinfo,
	},
	.id        = 0,
};
# endif /* CONFIG_IMX_BACKLIGHT */

#endif /* CONFIG_FB_MXC || CONFIG_FB_IMX */


#ifdef CONFIG_IMX_BACKLIGHT
static struct platform_device *platform_devices[] __initdata = {
	&imxbl_device,
};
#endif

void apf27_lcd_startup(void)
{
	int i;

	/* force LCD data/control lines to 0 before powering the LCD
	otherwise start up conditions may not be respected */
	for (i = 5; i <=  31; i++) {
		if (i != 30) {
			gpio_set_value(GPIO_PORTA | i, 0);
			mxc_gpio_mode(GPIO_PORTA | i | GPIO_OUT| GPIO_GPIO);
		}
	}
	/* wait a few milli */
	mdelay(100);
}

void apf27_lcd_init(void)
{
#ifdef CONFIG_IMX_BACKLIGHT
	platform_add_devices(platform_devices, ARRAY_SIZE(platform_devices));
#endif
#if defined(CONFIG_FB_MXC) || defined(CONFIG_FB_IMX)
	mxc_register_device(&mxc_fb_device, &apf27_fb_data);
#endif
}

