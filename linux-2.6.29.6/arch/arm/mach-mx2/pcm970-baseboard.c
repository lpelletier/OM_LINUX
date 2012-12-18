/*
 * Copyright (C) 2008 Juergen Beisert (kernel@pengutronix.de)
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */

#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/delay.h>

#include <media/soc_camera.h>

#include <asm/mach/arch.h>

#include <mach/hardware.h>
#include <mach/common.h>
#include <mach/imx_i2c.h>
#include <mach/imxfb.h>
#include <mach/pmic/power.h>
#include <mach/iomux-mx1-mx2.h>
#include <mach/imx_cam.h>
#include <mach/mxc_ehci.h>
#include <mach/ulpi.h>

#include "devices.h"

/* count of GPIOs that are occupied by the CPU itself */
#define MAX_INTERNAL_GPIO 192

static struct soc_camera_link iclink[] = {
	{
		.bus_id	= 0, /* Must match with the camera ID above */
		.gpio = MAX_INTERNAL_GPIO,
	}, {
		.bus_id	= 0, /* Must match with the camera ID above */
		.gpio   = MAX_INTERNAL_GPIO,
	}
};

static struct pca953x_platform_data pca9536_data = {
	.gpio_base      = MAX_INTERNAL_GPIO,
};

/* Board I2C devices. */
static struct i2c_board_info __initdata pcm970_i2c_devices[] = {
	{
		/* Must initialize before the camera(s) */
		I2C_BOARD_INFO("pca953x", 0x41),
		.type = "pca9536",
		.platform_data = &pca9536_data,
	}, {
		I2C_BOARD_INFO("mt9v022", 0x48),
		.platform_data = &iclink[0],
		.type = "mt9v022",
	}, {
		I2C_BOARD_INFO("mt9m001", 0x5d),
		.platform_data = &iclink[1],
		.type = "mt9m001",
	},
};

static int mxc_i2c0_pins[] = {
	/*
	 * it seems the data line misses a pullup, so we must enable
	 * the internal pullup as a local workaround
	 */
	PD17_PF_I2C_DATA | GPIO_PUEN,
	PD18_PF_I2C_CLK,
};

static int pcm038_i2c_0_init(struct platform_device *pdev)
{
	return mxc_gpio_setup_multiple_pins(mxc_i2c0_pins,
			ARRAY_SIZE(mxc_i2c0_pins),
			MXC_GPIO_ALLOC_MODE_NORMAL, "I2C0");
}

static int pcm038_i2c_0_exit(struct platform_device *pdev)
{
	return mxc_gpio_setup_multiple_pins(mxc_i2c0_pins,
			ARRAY_SIZE(mxc_i2c0_pins),
			MXC_GPIO_ALLOC_MODE_RELEASE, "I2C0");
}

static struct imx_i2c_platform_data pcm038_i2c_0_data = {
	.max_clk = 100000,
	.init = pcm038_i2c_0_init,
	.exit = pcm038_i2c_0_exit,
};

static int mxc_fb_pins[] = {
	PA5_PF_LSCLK,	PA6_PF_LD0,	PA7_PF_LD1,	PA8_PF_LD2,
	PA9_PF_LD3,	PA10_PF_LD4,	PA11_PF_LD5,	PA12_PF_LD6,
	PA13_PF_LD7,	PA14_PF_LD8,	PA15_PF_LD9,	PA16_PF_LD10,
	PA17_PF_LD11,	PA18_PF_LD12,	PA19_PF_LD13,	PA20_PF_LD14,
	PA21_PF_LD15,	PA22_PF_LD16,	PA23_PF_LD17,	PA24_PF_REV,
	PA25_PF_CLS,	PA26_PF_PS,	PA27_PF_SPL_SPR, PA28_PF_HSYNC,
	PA29_PF_VSYNC,	PA30_PF_CONTRAST, PA31_PF_OE_ACD
};

static int pcm038_fb_init(struct platform_device *pdev)
{
	return mxc_gpio_setup_multiple_pins(mxc_fb_pins,
			ARRAY_SIZE(mxc_fb_pins),
			MXC_GPIO_ALLOC_MODE_NORMAL, "FB");
}

static int pcm038_fb_exit(struct platform_device *pdev)
{
	return mxc_gpio_setup_multiple_pins(mxc_fb_pins,
			ARRAY_SIZE(mxc_fb_pins),
			MXC_GPIO_ALLOC_MODE_RELEASE, "FB");
}

/*
 * Connected is a portrait Sharp-QVGA display
 * of type: LQ035Q7DH06
 */
static struct imx_fb_platform_data pcm038_fb_data = {
	.pixclock	= 5300,	/* PCLK = 5.3MHz */
	.xres		= 240,
	.yres		= 320,

	.bpp		= 16,
	.hsync_len	= 7,
	.left_margin	= 5,
	.right_margin	= 16,

	.vsync_len	= 1,
	.upper_margin	= 7,
	.lower_margin	= 9,
	.fixed_screen_cpu = 0,

	/*
	 * - HSYNC active high
	 * - VSYNC active high
	 * - clk notenabled while idle
	 * - clock not inverted
	 * - data not inverted
	 * - data enable low active
	 * - enable sharp mode
	 */
	.pcr		= 0xFA0080C0,
	.pwmr		= 0x00A903FF,
	.lscr1		= 0x00120300,
	.dmacr		= 0x00020010,

	.init = pcm038_fb_init,
	.exit = pcm038_fb_exit,
};

static int mxc_csi_pins[] = {
	PB10_PF_CSI_D0,
	PB11_PF_CSI_D1,
	PB12_PF_CSI_D2,
	PB13_PF_CSI_D3,
	PB14_PF_CSI_D4,
	PB15_PF_CSI_MCLK,
	PB16_PF_CSI_PIXCLK,
	PB17_PF_CSI_D5,
	PB18_PF_CSI_D6,
	PB19_PF_CSI_D7,
	PB20_PF_CSI_VSYNC,
	PB21_PF_CSI_HSYNC
};

static int pcm970_camera_init(struct platform_device *pdev)
{
#ifdef CONFIG_MXC_PMIC
	pmic_power_regulator_on(REGU_VCAM);
#endif
	return mxc_gpio_setup_multiple_pins(mxc_csi_pins,
			ARRAY_SIZE(mxc_csi_pins),
			MXC_GPIO_ALLOC_MODE_NORMAL, "CSI");
}

static int pcm970_camera_exit(struct platform_device *pdev)
{
	return mxc_gpio_setup_multiple_pins(mxc_csi_pins,
			ARRAY_SIZE(mxc_csi_pins),
			MXC_GPIO_ALLOC_MODE_RELEASE, "CSI");
}

struct mx27_camera_platform_data pcm970_camera = {
	.init = pcm970_camera_init,
	.exit = pcm970_camera_exit,
	.clk  = 48000000,
	.flags = MX27_CAMERA_HSYNC_HIGH | MX27_CAMERA_GATED_CLOCK,
};


static int isp1504_set_vbus_power(void __iomem *view, int on)
{
	int vid, pid, ret = 0;

	vid = (ulpi_read(ISP1504_VID_HIGH, view) << 8) |
		 ulpi_read(ISP1504_VID_LOW, view);
	pid = (ulpi_read(ISP1504_PID_HIGH, view) << 8) |
		 ulpi_read(ISP1504_PID_LOW, view);

	pr_info("ULPI Vendor ID 0x%x    Product ID 0x%x\n", vid, pid);
	if (vid != 0x4cc || pid != 0x1504) {
		pr_err("No ISP1504 found\n");
		return -1;
	}

	if (on) {
		ret = ulpi_set(DRV_VBUS_EXT |	/* enable external Vbus */
			    DRV_VBUS |		/* enable internal Vbus */
			    USE_EXT_VBUS_IND |	/* use external indicator */
			    CHRG_VBUS,		/* charge Vbus */
			    ISP1504_OTGCTL, view);
	} else {
		ret = ulpi_clear(DRV_VBUS_EXT |	/* disable external Vbus */
			      DRV_VBUS,		/* disable internal Vbus */
			      ISP1504_OTGCTL, view);

		ret |= ulpi_set(USE_EXT_VBUS_IND | /* use external indicator */
			    DISCHRG_VBUS,	   /* discharge Vbus */
			    ISP1504_OTGCTL, view);
	}

	return ret;
}

static int mxc_usbh2_pins[] = {
	PA0_PF_USBH2_CLK,
	PA1_PF_USBH2_DIR,
	PA2_PF_USBH2_DATA7,
	PA3_PF_USBH2_NXT,
	PA4_PF_USBH2_STP,
	PD19_AF_USBH2_DATA4,
	PD20_AF_USBH2_DATA3,
	PD21_AF_USBH2_DATA6,
	PD22_AF_USBH2_DATA0,
	PD23_AF_USBH2_DATA2,
	PD24_AF_USBH2_DATA1,
	PD26_AF_USBH2_DATA5,
};

static int pcm970_usbh2_init(struct platform_device *pdev)
{
	int ret;
	uint32_t temp;

	ret = mxc_gpio_setup_multiple(mxc_usbh2_pins,
			ARRAY_SIZE(mxc_usbh2_pins), "usbh2");
	if (ret)
		return ret;

	temp = readl(IO_ADDRESS(OTG_BASE_ADDR) + 0x600);
	pr_debug("USB_CTRL before: 0x%08x\n", temp);
	temp &= ~((3 << 21) | 1);
	temp |= (1 << 5) | (1 << 16) | (1 << 19) | (1 << 11) | (1 << 20);
	writel(temp, IO_ADDRESS(OTG_BASE_ADDR) + 0x600);
	pr_debug("USB_CTRL after: 0x%08x\n", temp);

	temp = readl(IO_ADDRESS(OTG_BASE_ADDR) + 0x584);
	pr_debug("PORTSC1 before: 0x%08x\n", temp);
	temp &= ~(3 << 30);
	temp |= 2 << 30;
	writel(temp, IO_ADDRESS(OTG_BASE_ADDR) + 0x584);
	pr_debug("PORTSC1 after: 0x%08x\n", temp);

	mdelay(10);

	ret = isp1504_set_vbus_power(IO_ADDRESS(OTG_BASE_ADDR + 0x570), 1);
	if (ret)
		mxc_gpio_release_multiple(mxc_usbh2_pins,
			ARRAY_SIZE(mxc_usbh2_pins));
	return ret;
}

struct mxc_usb2_platform_data ehci2_pdata = {
	.init = pcm970_usbh2_init,
};

/*
 * system init for baseboard usage. Will be called by pcm038 init.
 *
 * Add platform devices present on this baseboard and init
 * them from CPU side as far as required to use them later on
 */
void __init pcm970_baseboard_init(void)
{
	/* the first i2c master is used for devices on the baseboard */
	mxc_register_device(&imx_i2c_device0, &pcm038_i2c_0_data);

	i2c_register_board_info(0, pcm970_i2c_devices,
		ARRAY_SIZE(pcm970_i2c_devices));

	mxc_register_device(&mxc_fb_device, &pcm038_fb_data);
	mxc_register_device(&mx27_camera_device, &pcm970_camera);
	mxc_register_device(&mxc_ehci2, &ehci2_pdata);
}
