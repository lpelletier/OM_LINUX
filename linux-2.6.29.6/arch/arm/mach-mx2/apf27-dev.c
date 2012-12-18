 /*
 * apf27-dev.c
 * Support for AFP27 module's development baseboard
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

#include <linux/platform_device.h>
#include <linux/mtd/physmap.h>
#include <asm/mach/flash.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/i2c.h>
#include <linux/serial.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/cpufreq.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>

#include <asm/io.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/mach/time.h>
// #include <asm/arch/clock.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/iomux-mx1-mx2.h>
#include <mach/gpio.h>
#include <mach/imx_i2c.h>
//#include <mach/imx_spi.h>
#include <../mach-imx/include/mach/spi_imx.h>
#include <mach/imx-uart.h>
#include <mach/mmc.h>
#ifdef CONFIG_FB_MXC /* Freescale Framebuffer */
#include <mach/imx_fb.h>
#include <mach/imxfb.h>
#endif
#include <linux/fb.h>
#include <mach/mxc_nand.h>
#include <mach/mxc_ehci.h>
#include <mach/ulpi.h>
#include <linux/spi/max1027.h>
#include <linux/spi/tsc2102.h>
#include "../../../drivers/net/can/mcp251x.h"
#include "../../../drivers/armadeus/pwm/pwm.h"
#include <media/ad9889.h>
#include <mach/imx_dam.h>
#include <mach/imx_sound.h>
#include <mach/imx_ssi.h>
#include <mach/imx-alsa.h>
#include <mach/imx_cam.h>
#include <media/soc_camera.h>
#if defined(CONFIG_CPU_FREQ_IMX27) || defined(CONFIG_CPU_FREQ_IMX27_MODULE)
#include "cpufreq_imx.h"
#endif
#include <mach/board-apf27.h>
#include <asm/mach/map.h>

#include "devices.h"
#include "crm_regs.h"

#ifdef CONFIG_FB_MXC
extern void apf27_lcd_init(void);
#endif
#ifdef CONFIG_MACH_APF27_DEV_EXT
extern void apf27dev_extension_init(void);
extern void apf27dev_extension_resume(void);
extern void apf27dev_extension_suspend(void);
#endif

#ifdef CONFIG_ARMADEUS_MAX1027_MODULE
#define CONFIG_ARMADEUS_MAX1027 1
#endif
#ifdef CONFIG_SPI_TSC2102_MODULE
#define CONFIG_SPI_TSC2102 1
#endif
#ifdef CONFIG_CAN_MCP251X_MODULE
#define CONFIG_CAN_MCP251X 1
#endif
#ifdef CONFIG_SPI_SPIDEV_MODULE
#define CONFIG_SPI_SPIDEV 1
#endif
#ifdef CONFIG_IMX_BACKLIGHT_MODULE
#define CONFIG_IMX_BACKLIGHT
#endif
#ifdef CONFIG_VIDEO_AD9889_MODULE
#define CONFIG_VIDEO_AD9889 1
#endif
#ifdef CONFIG_VIDEO_MX27_MODULE
#define CONFIG_VIDEO_MX27 1
#endif
#ifdef CONFIG_SOC_CAMERA_OV96XX_MODULE
#define CONFIG_SOC_CAMERA_OV96XX 1
#endif
#ifdef CONFIG_ARMADEUS_PWM_DRIVER_MODULE
#define CONFIG_PWM
#endif


#ifdef CONFIG_I2C

/* APF27Dev makes I2C1 bus available */
static int mxc_i2c0_pins[] = {
	PD17_PF_I2C_DATA,
	PD18_PF_I2C_CLK
};

static int apf27_i2c_0_init(struct platform_device *pdev)
{
	return mxc_gpio_setup_multiple_pins(mxc_i2c0_pins, ARRAY_SIZE(mxc_i2c0_pins),
						"I2C1");
}

static int apf27_i2c_0_exit(struct platform_device *pdev)
{
	mxc_gpio_release_multiple_pins(mxc_i2c0_pins, ARRAY_SIZE(mxc_i2c0_pins));

	return 0;
}

static struct imx_i2c_platform_data apf27_i2c_0_data = {
	.max_clk = 400000,
	.init = apf27_i2c_0_init,
	.exit = apf27_i2c_0_exit,
};

/* I2C1 devices: RTC & Optionnal Camera */
# ifdef CONFIG_SOC_CAMERA_OV96XX
static struct soc_camera_link ov96xx_iclink; /* declared later */
#endif
static struct i2c_board_info apf27dev_i2c_devices_bus0[] = {
	[0] = {
		/* APF27Dev has an RTC */
		.type = "ds1374",
		.flags = 0,  /* FIXME */
		.addr = 0x68,	/* E0=0, E1=1, E2=0 */
		.platform_data = NULL,
		.irq = 0
	},
# ifdef CONFIG_SOC_CAMERA_OV96XX
	[1] = {
		.type = "ov96xx",
		.flags = 0,  /* FIXME */
		.addr = 0x30,
		.platform_data = &ov96xx_iclink,
		.irq = 0
	},
# endif
};


/* APF27Dev has an optionnal HDMI video output (controlled with I2C2) */
# ifdef CONFIG_VIDEO_AD9889
#define AD9889_INT 15

static int ad9889_pins[] = {
	(GPIO_PORTC | GPIO_IN | GPIO_GPIO | AD9889_INT),
};

static int ad9889_init(void)
{
	return mxc_gpio_setup_multiple_pins(ad9889_pins, ARRAY_SIZE(ad9889_pins),
						"AD9889");
}

static int ad9889_exit(void)
{
	mxc_gpio_release_multiple_pins(ad9889_pins, ARRAY_SIZE(ad9889_pins));

	return 0;
}

struct ad9889_fb_data {
	struct fb_var_screeninfo var;
	int PA31_GIUS_old_state;
};

static struct ad9889_fb_data apf27_ad9889_fb_data;
extern void acquire_console_sem(void);
extern void release_console_sem(void);

static void ad9889_display_connected(void)
{
	struct fb_var_screeninfo temp_var;
	struct fb_info *info = registered_fb[0];

	apf27_ad9889_fb_data.PA31_GIUS_old_state = 0;
	/* make sure the LCD OE_ACD pin is correctly configured */
	if (__raw_readl(VA_GPIO_BASE + MXC_GIUS(0)) & 0x80000000) {
		apf27_ad9889_fb_data.PA31_GIUS_old_state = 1;
		mxc_gpio_mode(PA31_PF_OE_ACD);
	}

	/* save old FB params */
	memcpy(&apf27_ad9889_fb_data.var, &info->var, sizeof(struct fb_var_screeninfo));
	memcpy(&temp_var, &info->var, sizeof(struct fb_var_screeninfo));

	/* set new FB params */
	temp_var.xres = 640;
	temp_var.yres = 480;
	temp_var.xres_virtual = 640;
	temp_var.yres_virtual = 480;
	temp_var.pixclock = 37538;
	temp_var.left_margin = 47;
	temp_var.right_margin = 47; /* at least 3 & 1 */
	temp_var.upper_margin = 33;
	temp_var.lower_margin = 10;
	temp_var.hsync_len = 63;
	temp_var.vsync_len = 2;
# ifdef CONFIG_FB_MXC
	temp_var.sync = FB_SYNC_OE_ACT_HIGH; /* -> Will change PCR */
# endif
	temp_var.vmode = FB_VMODE_NONINTERLACED,
	temp_var.nonstd = 0;

	acquire_console_sem();
	/* tel the FB client that params have been changed */
	info->flags |= FBINFO_MISC_USEREVENT;
	/* set new params */
	fb_set_var(info, &temp_var);
	info->flags &= ~FBINFO_MISC_USEREVENT;
	release_console_sem();
}

static void ad9889_display_disconnected(void)
{
	struct fb_info *info = registered_fb[0];

	/* restore LCD OE_ACD pin */
	if (apf27_ad9889_fb_data.PA31_GIUS_old_state)
		mxc_gpio_mode(GPIO_PORTA | 31 | GPIO_OUT| GPIO_GPIO);

	acquire_console_sem();
	/* inform the console that the FB params have been changed */
	info->flags |= FBINFO_MISC_USEREVENT;
	/* restore old FB params */
	fb_set_var(info, &apf27_ad9889_fb_data.var);
	info->flags &= ~FBINFO_MISC_USEREVENT;
	release_console_sem();
}

static struct ad9889_config apf27_ad9889_config = {
	.init = ad9889_init,
	.exit = ad9889_exit,
	.display_connected = ad9889_display_connected,
	.display_disconnected = ad9889_display_disconnected,
	.data = &apf27_ad9889_fb_data,
	.EDID_I2C_addr = 0x3f
};
# endif /* CONFIG_VIDEO_AD9889 */

/* I2C2 devices */
static struct i2c_board_info apf27dev_i2c_devices_bus1[] = {
# ifdef CONFIG_VIDEO_AD9889
	[0] = {
		.type = "ad9889",
		.flags = 0,
		.addr = 0x39,
		.platform_data = &apf27_ad9889_config,
		.irq = IRQ_GPIOC(AD9889_INT)
	},
# endif /* CONFIG_VIDEO_AD9889 */
};
#endif /* CONFIG_I2C */


/* UART2 signals are available on J8 connector (multiplexed with Keypad) */
#ifdef CONFIG_SERIAL_IMX_UART2

static int mxc_uart1_pins[] = {
#ifdef CONFIG_SERIAL_IMX_UART2_USE_RTSCTS
	PE3_PF_UART2_CTS,
	PE4_PF_UART2_RTS,
#endif
	PE6_PF_UART2_TXD,
	PE7_PF_UART2_RXD,
};

static int uart_mxc_port1_init(struct platform_device *pdev)
{
	int res = mxc_gpio_setup_multiple_pins(mxc_uart1_pins,
			ARRAY_SIZE(mxc_uart1_pins), "UART2");

	return res;
}

static int uart_mxc_port1_exit(struct platform_device *pdev)
{
	mxc_gpio_release_multiple_pins(mxc_uart1_pins, ARRAY_SIZE(mxc_uart1_pins));

	return 0;
}

static struct imxuart_platform_data uart1_pdata = {
	.init = uart_mxc_port1_init,
	.exit = uart_mxc_port1_exit,
#ifdef CONFIG_SERIAL_IMX_UART2_USE_RTSCTS
	.flags = IMXUART_HAVE_RTSCTS,
#else
	.flags = 0,
#endif
};
#endif /* CONFIG_SERIAL_IMX_UART2 */


/* UART5 signals are available on J9 connector (multiplexed with CSI) */
#ifdef CONFIG_SERIAL_IMX_UART5

static int mxc_uart4_pins[] = {
	PB19_AF_UART5_RXD,
	PB18_AF_UART5_TXD,
#ifdef CONFIG_SERIAL_IMX_UART5_USE_RTSCTS
	PB20_AF_UART5_CTS,
	PB21_AF_UART5_RTS,
#endif
};

static int uart_mxc_port4_init(struct platform_device *pdev)
{
	int res = mxc_gpio_setup_multiple_pins(mxc_uart4_pins,
			ARRAY_SIZE(mxc_uart4_pins), "UART5");

	return res;
}

static int uart_mxc_port4_exit(struct platform_device *pdev)
{
	mxc_gpio_release_multiple_pins(mxc_uart4_pins,
			ARRAY_SIZE(mxc_uart4_pins));

	return 0;
}

static struct imxuart_platform_data uart4_pdata = {
	.init = uart_mxc_port4_init,
	.exit = uart_mxc_port4_exit,
#ifdef CONFIG_SERIAL_IMX_UART5_USE_RTSCTS
	.flags = IMXUART_HAVE_RTSCTS,
#else
	.flags = 0,
#endif
};
#endif /* CONFIG_SERIAL_IMX_UART5 */

/* UART6 signals are available on J9 connector (multiplexed with CSI) */
#ifdef CONFIG_SERIAL_IMX_UART6

static int mxc_uart5_pins[] = {
	PB10_AF_UART6_TXD,
	PB11_AF_UART6_RXD,
#ifdef CONFIG_SERIAL_IMX_UART6_USE_RTSCTS
	PB12_AF_UART6_CTS,
	PB13_AF_UART6_RTS,
#endif
};

static int uart_mxc_port5_init(struct platform_device *pdev)
{
	int res = mxc_gpio_setup_multiple_pins(mxc_uart5_pins,
			ARRAY_SIZE(mxc_uart5_pins), "UART6");

	return res;
}

static int uart_mxc_port5_exit(struct platform_device *pdev)
{
	mxc_gpio_release_multiple_pins(mxc_uart5_pins, ARRAY_SIZE(mxc_uart5_pins));

	return 0;
}

static struct imxuart_platform_data uart5_pdata = {
	.init = uart_mxc_port5_init,
	.exit = uart_mxc_port5_exit,
#ifdef CONFIG_SERIAL_IMX_UART6_USE_RTSCTS
	.flags = IMXUART_HAVE_RTSCTS,
#else
	.flags = 0,
#endif
};
#endif /* CONFIG_SERIAL_IMX_UART6 */


/* SPI ports declarations */
#ifdef CONFIG_SPI_MXC_SELECT1
static int mxc_cspi0_pins[] = {
	/* PD28_PF_CSPI1_SS0, Preferably use CS pin as GPIO */
	PD29_PF_CSPI1_SCLK,
	PD30_PF_CSPI1_MISO,
	PD31_PF_CSPI1_MOSI
};
#endif /* CONFIG_SPI_MXC_SELECT1 */

#ifdef CONFIG_SPI_MXC_SELECT2
static int mxc_cspi1_pins[] = {
	/* PD19_PF_CSPI2_SS2,
	PD20_PF_CSPI2_SS1, Preferably use CS pin as GPIO
	PD21_PF_CSPI2_SS0, */
	PD22_PF_CSPI2_SCLK,
	PD23_PF_CSPI2_MISO,
	PD24_PF_CSPI2_MOSI
};
#endif /* CONFIG_SPI_MXC_SELECT2 */

#ifdef CONFIG_SPI_MXC_SELECT3
static int mxc_cspi2_pins[] = {
	/*
	PE21_AF_CSPI3_SS,
	Preferably use CS pin as GPIO
	*/
	PE23_AF_CSPI3_SCLK,
	PE18_AF_CSPI3_MISO,
	PE22_AF_CSPI3_MOSI
};
#endif /* CONFIG_SPI_MXC_SELECT3 */

#ifdef CONFIG_SPI_MXC_SELECT1
static int gpio_spi0_active(struct platform_device *pdev)
{
	return mxc_gpio_setup_multiple_pins(mxc_cspi0_pins,
			ARRAY_SIZE(mxc_cspi0_pins), "CSPI1");
}

static int gpio_spi0_inactive(struct platform_device *pdev)
{
	mxc_gpio_release_multiple_pins(mxc_cspi0_pins, ARRAY_SIZE(mxc_cspi0_pins));
	return 0;
}
#endif /* CONFIG_SPI_MXC_SELECT1 */

#ifdef CONFIG_SPI_MXC_SELECT2
static int gpio_spi1_active(struct platform_device *pdev)
{
	return mxc_gpio_setup_multiple_pins(mxc_cspi1_pins,
			ARRAY_SIZE(mxc_cspi1_pins), "CSPI2");
}

static int gpio_spi1_inactive(struct platform_device *pdev)
{
	mxc_gpio_release_multiple_pins(mxc_cspi1_pins, ARRAY_SIZE(mxc_cspi1_pins));
	return 0;
}
#endif /* CONFIG_SPI_MXC_SELECT2 */

#ifdef CONFIG_SPI_MXC_SELECT3
static int gpio_spi2_active(struct platform_device *pdev)
{
	return mxc_gpio_setup_multiple_pins(mxc_cspi2_pins, ARRAY_SIZE(mxc_cspi2_pins), "CSPI3");
}

static int gpio_spi2_inactive(struct platform_device *pdev)
{
	mxc_gpio_release_multiple_pins(mxc_cspi2_pins, ARRAY_SIZE(mxc_cspi2_pins));
	return 0;
}
#endif /* CONFIG_SPI_MXC_SELECT3 */

#ifdef CONFIG_SPI_MXC_SELECT1
static struct spi_imx_master imx_spi0_master_info = {
	.num_chipselect	= 1,
	.enable_dma     = 0,
	.init = gpio_spi0_active,
	.exit = gpio_spi0_inactive,
};
#endif /* CONFIG_SPI_MXC_SELECT1 */

#ifdef CONFIG_SPI_MXC_SELECT2
static struct spi_imx_master imx_spi1_master_info = {
	.num_chipselect	= 3,
	.enable_dma     = 0,
	.init = gpio_spi1_active,
	.exit = gpio_spi1_inactive,
};
#endif /* CONFIG_SPI_MXC_SELECT2 */

#ifdef CONFIG_SPI_MXC_SELECT3
static struct spi_imx_master imx_spi2_master_info = {
	.num_chipselect	= 1,
	.enable_dma     = 0,
	.init = gpio_spi2_active,
	.exit = gpio_spi2_inactive,
};
#endif /* CONFIG_SPI_MXC_SELECT3 */



/* APF27Dev has an optionnal 7 channels 10 bits ADC (SPI1) */
#ifdef CONFIG_ARMADEUS_MAX1027
#define MAX1027_EOC_INT 15
#define MAX1027_CNVST 14
#define MAX1027_CS (GPIO_PORTD | 28)

static int max1027_pins[] = {
	(MAX1027_CS | GPIO_OUT | GPIO_GPIO),
	(GPIO_PORTE | GPIO_IN | GPIO_GPIO | MAX1027_EOC_INT),
	(GPIO_PORTE | GPIO_OUT | GPIO_GPIO | MAX1027_CNVST),
};

static int max1027_init(struct spi_device *spi)
{
	gpio_set_value(MAX1027_CS, 1);
	return mxc_gpio_setup_multiple_pins(max1027_pins, ARRAY_SIZE(max1027_pins), "MAX1027");
}

static int max1027_exit(struct spi_device *spi)
{
	mxc_gpio_release_multiple_pins(max1027_pins, ARRAY_SIZE(max1027_pins));
	return 0;
}

static void max1027_cs(u32 command)
{
	if (command == SPI_CS_DEASSERT)
		gpio_set_value(MAX1027_CS, 1);
	else
		gpio_set_value(MAX1027_CS, 0);
}

static struct spi_imx_chip max1027_hw = {
	.cs_control = max1027_cs,
};

static struct max1027_config apf27_max1027_config = {
	.conv = MAX1027_CONV_DEFAULT,
	.setup = MAX1027_SETUP_DEFAULT,
	.avg = MAX1027_AVG_DEFAULT,
	.cnvst_pin = (int)(MAX1027_CNVST|GPIO_PORTE),
	.init = max1027_init,
	.exit = max1027_exit
};
#endif /* CONFIG_ARMADEUS_MAX1027 */

/* APF27Dev has a SPI Touchscreen controller */
#define TSC2101_INT 17
#define TSC2101_CS (GPIO_PORTD | 21)	/* SPI2_SS0 used as GPIO */

/* CS can be used as GPIO too, if communication with TSC is deactivated (thanks
   to TXB0108 chip (U23). INT pin can't and so must be reserved in any case */
static int tsc2101_pins[] = {
        (TSC2101_CS | GPIO_OUT | GPIO_GPIO),
};
static int tsc2101_fixed_pins[] = {
	(GPIO_PORTF | TSC2101_INT | GPIO_IN | GPIO_GPIO), /* configure INT pin as GPIO */
};

static int tsc2101_init_gpio(void)
{
	/* Initialize CS high */
	gpio_set_value(TSC2101_CS, 1);

	return mxc_gpio_setup_multiple_pins(tsc2101_pins, ARRAY_SIZE(tsc2101_pins),
						"TSC2101");
}

static int tsc2101_init_fixed_gpio(void)
{
        return mxc_gpio_setup_multiple_pins(tsc2101_fixed_pins,
					ARRAY_SIZE(tsc2101_fixed_pins), "TSC2101");
}

static void tsc2101_release_gpio(void)
{
        mxc_gpio_release_multiple_pins(tsc2101_pins, ARRAY_SIZE(tsc2101_pins));
}

static void tsc2101_cs(u32 command)
{
	if (command == SPI_CS_DEASSERT)
		gpio_set_value(TSC2101_CS, 1);
	else
		gpio_set_value(TSC2101_CS, 0);
}

static struct spi_imx_chip tsc2101_hw = {
	.cs_control = tsc2101_cs,
};

static struct tsc210x_config apf27_tsc2101_config = {
	.mclk =	12288000, /* MCLK value in Master mode */
	.use_internal = 1, /* -> use internal voltage reference */
	.monitor = TSC_BAT1 | TSC_AUX | TSC_TEMP,
	.init = tsc2101_init_gpio,
	.exit = tsc2101_release_gpio,
};


/* APF27Dev has an optional CAN Bus controller */
#ifdef CONFIG_CAN_MCP251X

#define CAN_MCP251X_INT 19
#define MCP251X_CS (GPIO_PORTD | 20)

static int mcp251x_pins[] = {
	(GPIO_PORTD | CAN_MCP251X_INT | GPIO_IN | GPIO_GPIO), /* IRQ pin as GPIO */
	(MCP251X_CS | GPIO_OUT | GPIO_GPIO),
};

static void mcp251x_init_irq(void)
{
	gpio_set_value(MCP251X_CS, 1);
	mxc_gpio_setup_multiple_pins(mcp251x_pins, ARRAY_SIZE(mcp251x_pins), "MCP251x");
}

static void mcp251x_cs(u32 command)
{
	if (command == SPI_CS_DEASSERT)
		gpio_set_value(MCP251X_CS, 1);
	else
		gpio_set_value(MCP251X_CS, 0);
}

static struct spi_imx_chip mcp251X_hw = {
	.cs_control		= mcp251x_cs,
};

static struct mcp251x_platform_data apf27_mcp251x_config = {
	.oscillator_frequency  = 16000000,
	.board_specific_setup  = mcp251x_init_irq,
	.device_reset          = NULL,
	.transceiver_enable    = NULL,
};
#endif /* CONFIG_CAN_MCP251X */


/* SPI2 can be used as userspace general SPI bus (CS 2) */
#ifdef CONFIG_SPI_SPIDEV
#define SPIDEV_CS (GPIO_PORTB | 17)

static int spidev_pins[] = {
	(SPIDEV_CS | GPIO_OUT | GPIO_GPIO),
};

static int spidev_init_gpio(void)
{
	gpio_set_value(SPIDEV_CS, 1);
	return mxc_gpio_setup_multiple_pins(spidev_pins, ARRAY_SIZE(spidev_pins), "spidev");
}

/* Chip select command for spidev */
static void spidev_cs(u32 command)
{
	if (command == SPI_CS_DEASSERT)
		gpio_set_value(SPIDEV_CS, 1);
	else
		gpio_set_value(SPIDEV_CS, 0);
}

static struct spi_imx_chip spidev_hw = {
	.cs_control     = spidev_cs,
};

static struct spidev_platform_data apf27_spidev_config = {
	.init = spidev_init_gpio,
};
#endif /* CONFIG_SPI_SPIDEV */

static struct spi_board_info spi_board_info[] __initdata = {
#ifdef CONFIG_ARMADEUS_MAX1027
	{
		.modalias		= "max1027",
		.controller_data 	= &max1027_hw,
		.max_speed_hz		= 8000000,
		.bus_num		= 0, /* SPI1 */
		.irq			= IRQ_GPIOE(MAX1027_EOC_INT),
		.chip_select		= 0, /* SS0 */
		.mode 			= 0,
		.platform_data		= &apf27_max1027_config,
	},
#endif /* CONFIG_ARMADEUS_MAX1027 */
#ifdef CONFIG_SPI_TSC2102
	{
		.modalias		= "tsc210x",
		.controller_data 	= &tsc2101_hw,
		.max_speed_hz		= 8000000,
		.bus_num		= 1, /* SPI2 */
		.irq			= IRQ_GPIOF(TSC2101_INT),
		.chip_select		= 0,
		.mode 			= 0,
		.platform_data		= &apf27_tsc2101_config,
	},
#endif /* CONFIG_SPI_TSC2102 */
#ifdef CONFIG_CAN_MCP251X
	{
		.modalias		= "mcp251x",
		.controller_data	= &mcp251X_hw,
		.max_speed_hz		= 8000000, /* 8MHz */
		.bus_num		= 1, /* SPI2 */
		.mode			= 0,
		.chip_select		= 1,
		.irq			= IRQ_GPIOD(CAN_MCP251X_INT),
		.platform_data		= &apf27_mcp251x_config,
	},
#endif /* CONFIG_CAN_MCP251X */
#ifdef CONFIG_SPI_SPIDEV
	{
		.modalias		= "spidev",
		.controller_data	= &spidev_hw,
		.max_speed_hz		= 8000000, /* 8MHz */
		.bus_num		= 1, /* SPI2 */
		.mode			= SPI_MODE_1,
		.chip_select		= 2,
		.platform_data		= &apf27_spidev_config,
	},
#endif /* CONFIG_SPI_SPIDEV */
};


/* APF27Dev adds an USB PHY on i.MX27 USB_HOST port 2 */
#ifdef CONFIG_USB_EHCI_MXC
static int mxc_usbh1_pins[] = {
	PB22_PF_USBH1_SUSP,
	PB23_PF_USB_PWR,
	PB24_PF_USB_OC_B,
	PB25_PF_USBH1_RCV,
	PB26_PF_USBH1_FS,
	PB27_PF_USBH1_OE_B,
	PB28_PF_USBH1_TXDM,
	PB29_PF_USBH1_TXDP,
	PB30_PF_USBH1_RXDM,
	PB31_PF_USBH1_RXDP
};

static int apf27_usbh1_init(struct platform_device *pdev)
{
	int ret;
	uint32_t temp;

	ret = mxc_gpio_setup_multiple_pins(mxc_usbh1_pins,
			ARRAY_SIZE(mxc_usbh1_pins), "usbh1");
	if (ret)
		return ret;

	temp = readl(IO_ADDRESS(OTG_BASE_ADDR) + 0x600);
	pr_debug("USB_CTRL before: 0x%08x\n", temp);
	temp &= ~( (3 << 13) | (1 << 8) |  1 );
	temp |=  (1 << 4) | (1 << 11) ;
	writel(temp, IO_ADDRESS(OTG_BASE_ADDR) + 0x600);
	pr_debug("USB_CTRL after: 0x%08x\n", temp);

	temp = readl(IO_ADDRESS(OTG_BASE_ADDR) + 0x384);
	pr_debug("PORTSC1 before: 0x%08x\n", temp);
	temp &= ~(3 << 30);
	temp |= 3 << 30;
	writel(temp, IO_ADDRESS(OTG_BASE_ADDR) + 0x384);
	pr_debug("PORTSC1 after: 0x%08x\n", temp);
	mdelay(10);

	temp = readl(IO_ADDRESS(OTG_BASE_ADDR) + 0x340);
	temp |= 0x02; /* reset controller */
	writel(temp, IO_ADDRESS(OTG_BASE_ADDR) + 0x340);
	pr_debug("reset controller: 0x%08x\n", temp);
	mdelay(10);

	return ret;
}

struct mxc_usb2_platform_data ehci1_pdata = {
	.init = apf27_usbh1_init,
};
#endif /* CONFIG_USB_EHCI_MXC */


#if defined(CONFIG_SND_IMX_TSC2102) || defined(CONFIG_SND_IMX_TSC2102_MODULE)
static struct imx_alsa_codec_config tsc2101_alsa_pdata;

static struct resource ssi_resources[] = {
	[0] = {
		.start	= SSI1_BASE_ADDR,
		.end	= SSI1_BASE_ADDR + 0x6F,
		.flags	= IORESOURCE_MEM
	},
};

static struct platform_device tsc2101_alsa_device = {
	.name           = "tsc210x-alsa",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(ssi_resources),
	.resource       = ssi_resources,
	.dev            = {
				.platform_data = &tsc2101_alsa_pdata,
			}
};
# define ALSA_SOUND &tsc2101_alsa_device,
#else
# define ALSA_SOUND
#endif /* CONFIG_SND_IMX_TSC2102 || CONFIG_SND_IMX_TSC2102_MODULE */

static struct platform_device *platform_devices[] __initdata = {
	ALSA_SOUND
};


/* SSI1 port is directly connected to TSC2101 and can't be used for
   anything else */
static int mxc_ssi1_pins[] = {
	PC20_PF_SSI1_FS,
	PC21_PF_SSI1_RXD,
	PC22_PF_SSI1_TXD,
	PC23_PF_SSI1_CLK,
};

static int gpio_ssi1_active(struct platform_device *pdev)
{
	int err;

	err = mxc_gpio_setup_multiple_pins(mxc_ssi1_pins,
					ARRAY_SIZE(mxc_ssi1_pins), "mx2-sound");
	if (err < 0)
		pr_err("Failed to register SSI pins\n");

	return err;
}

static int gpio_ssi1_inactive(struct platform_device *pdev)
{
	mxc_gpio_release_multiple_pins(mxc_ssi1_pins, ARRAY_SIZE(mxc_ssi1_pins));
	return 0;
}

static struct imx_ssi_platform_data apf27_ssi0_pdata = {
	.init = gpio_ssi1_active,
	.exit = gpio_ssi1_inactive
};

static int apf27_late_init(void)
{
	mxc_register_device(&mxc_dam_device, NULL);
	mxc_register_device(&imx_ssi_device0, &apf27_ssi0_pdata);

	return 0;
}

late_initcall(apf27_late_init);


/* APF27Dev has a microSD connector */
#ifdef CONFIG_MMC_MXC
static int mxc_sdhc2_pins[] = {
	PB4_PF_SDHC2_D0,
	PB5_PF_SDHC2_D1,
	PB6_PF_SDHC2_D2,
	PB7_PF_SDHC2_D3,
	PB8_PF_SDHC2_CMD,
	PB9_PF_SDHC2_CLK,
};

static int apf27_sdhc2_init(struct device *dev, irq_handler_t detect_irq, void *data)
{
	int ret;

	mxc_gpio_setup_multiple_pins(mxc_sdhc2_pins, ARRAY_SIZE(mxc_sdhc2_pins), "SDHC2");

	ret = request_irq(IRQ_GPIOC(14), detect_irq, 0, "imx-mmc-detect", data);
	if (ret)
		goto out_release_gpio;
	set_irq_type(IRQ_GPIOC(14), IRQ_TYPE_EDGE_BOTH);

	return 0;

out_release_gpio:
	mxc_gpio_release_multiple_pins(mxc_sdhc2_pins, ARRAY_SIZE(mxc_sdhc2_pins));

	return ret;
}

static void apf27_sdhc2_exit(struct device *dev, void *data)
{
	free_irq(IRQ_GPIOC(14), data);
	mxc_gpio_release_multiple_pins(mxc_sdhc2_pins, ARRAY_SIZE(mxc_sdhc2_pins));
}

static struct imxmmc_platform_data apf27_sdhc2_pdata = {
	.init = apf27_sdhc2_init,
	.exit = apf27_sdhc2_exit,
};

static inline void apf27dev_init_mmc(void)
{
	mxc_register_device(&mxc_sdhc_device1, &apf27_sdhc2_pdata);
}
#endif /* CONFIG_MMC_MXC */


/* APF27Dev can have an optionnal Camera module extension using i.MX27 CSI */
#ifdef CONFIG_VIDEO_MX27
# ifdef CONFIG_SOC_CAMERA_OV96XX
#define OV9653_PWRDN	(GPIO_PORTC | 30) /* SSI3_TX as POWERDOWN*/
#define OV9653_RESET	(GPIO_PORTC | 31) /* SSI3_CLK as RESET */

static int apf27dev_ov9653_ctl_pins[] = {
	(OV9653_PWRDN | GPIO_OUT | GPIO_GPIO),
	(OV9653_RESET | GPIO_OUT | GPIO_GPIO),
};
# endif

static int apf27dev_csi_pins[] = {
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

static int apf27dev_camera_init(struct platform_device *pdev)
{
# ifdef CONFIG_SOC_CAMERA_OV96XX
	int err;

	/* Setup PWRDN/RESET pins default value */
	gpio_set_value(OV9653_PWRDN, 0);
	gpio_set_value(OV9653_RESET, 0);
	err = mxc_gpio_setup_multiple_pins(apf27dev_ov9653_ctl_pins,
			ARRAY_SIZE(apf27dev_ov9653_ctl_pins), "OV9653");
	if (err)
		printk("Error %s", __func__);
# endif

	return mxc_gpio_setup_multiple_pins(apf27dev_csi_pins,
				ARRAY_SIZE(apf27dev_csi_pins), "CSI");
}

static int apf27dev_camera_exit(struct platform_device *pdev)
{
# ifdef CONFIG_SOC_CAMERA_OV96XX
	mxc_gpio_release_multiple_pins(apf27dev_ov9653_ctl_pins,
				ARRAY_SIZE(apf27dev_ov9653_ctl_pins));
# endif

	mxc_gpio_release_multiple_pins(apf27dev_csi_pins, ARRAY_SIZE(apf27dev_csi_pins));

	return 0;
}

struct mx27_camera_platform_data apf27_camera = {
	.init = apf27dev_camera_init,
	.exit = apf27dev_camera_exit,
	.clk  = 26600000,
	.flags = MX27_CAMERA_HSYNC_HIGH | MX27_CAMERA_GATED_CLOCK |
			MX27_CAMERA_SWAP16,
};

# ifdef CONFIG_SOC_CAMERA_OV96XX
static int ov96xx_power(struct device *dev, int state)
{
        printk(KERN_DEBUG "*** %s %d\n", __func__, state);

        if (state) /* UP */
                gpio_set_value(OV9653_PWRDN, 0);
        else /* DOWN */
                gpio_set_value(OV9653_PWRDN, 1);

        return 0;
}

static int ov96xx_reset(struct device *dev)
{
        printk(KERN_DEBUG "*** %s\n", __func__);

        gpio_set_value(OV9653_RESET, 1);
        msleep(10);
        gpio_set_value(OV9653_RESET, 0);
        msleep(10);

        return 0;
}

static struct soc_camera_link ov96xx_iclink = {
        .bus_id = 0,
        .power = ov96xx_power,
        .reset = ov96xx_reset,
};
# endif

#endif /* CONFIG_VIDEO_MX27 */


/* APF27Dev power management stuff */

#define APF27DEV_USER_SWITCH		(GPIO_PORTF | 13)
#define APF27DEV_USER_SWITCH_IRQ	(IRQ_GPIOF(13))
#define APF27DEV_USER_LED		(GPIO_PORTF | 14)
#define APF27DEV_POWER_DOWN_NOT		(GPIO_PORTF | 16)

static int apf27dev_pwr_ctl_pins[] = {
	(APF27DEV_POWER_DOWN_NOT | GPIO_OUT | GPIO_GPIO),
#ifdef CONFIG_USE_APF27DEV_IMX_SWITCH_FOR_PM
	(APF27DEV_USER_SWITCH | GPIO_IN | GPIO_GPIO),
#endif
#ifdef CONFIG_USE_APF27DEV_IMX_LED_FOR_PM
	(APF27DEV_USER_LED | GPIO_OUT | GPIO_GPIO),
#endif
};

void apf27_baseboard_resume(void)
{
#ifdef CONFIG_MACH_APF27_DEV_EXT
	apf27dev_extension_resume();
#endif
	gpio_set_value(APF27DEV_POWER_DOWN_NOT, 1);
#ifdef CONFIG_USE_APF27DEV_IMX_LED_FOR_PM
	gpio_set_value(GPIO_PORTF | 14, 0);
#endif
}

void apf27_baseboard_suspend(void)
{
#ifdef CONFIG_MACH_APF27_DEV_EXT
	apf27dev_extension_suspend();
#endif
	gpio_set_value(APF27DEV_POWER_DOWN_NOT, 0);
#ifdef CONFIG_USE_APF27DEV_IMX_LED_FOR_PM
	gpio_set_value(GPIO_PORTF | 14, 1);
#endif
}

#ifdef CONFIG_USE_APF27DEV_IMX_SWITCH_FOR_PM
static irqreturn_t apf27dev_user_switch_irq_handler(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}
#endif

static int apf27dev_pwr_ctrl_gpio_active(void)
{
	int err;

	err = mxc_gpio_setup_multiple_pins(apf27dev_pwr_ctl_pins,
					ARRAY_SIZE(apf27dev_pwr_ctl_pins), "pwr-ctl");
	if (err < 0)
		pr_err("Failed to register power control pins !\n");

	gpio_set_value(APF27DEV_POWER_DOWN_NOT, 1);
#ifdef CONFIG_USE_APF27DEV_IMX_LED_FOR_PM
	gpio_set_value(GPIO_PORTF | 14, 0);
#endif
#ifdef CONFIG_USE_APF27DEV_IMX_SWITCH_FOR_PM
	err = request_irq(APF27DEV_USER_SWITCH_IRQ, apf27dev_user_switch_irq_handler,
				IRQF_TRIGGER_FALLING,
				"PM irq", NULL);
	if (err < 0)
		pr_err("Failed to register PM irq !\n");
#endif

	return err;
}

void __init apf27_baseboard_init(void)
{
	printk("    Registering APF27_Dev ressources:");

#if defined(CONFIG_FB_MXC) || defined(CONFIG_FB_IMX)
	/* APF27Dev has an LCD connector */
	apf27_lcd_init();
#endif

	apf27dev_pwr_ctrl_gpio_active();

#ifdef CONFIG_SERIAL_IMX_UART2
	mxc_register_device(&mxc_uart_device1, &uart1_pdata);
#endif
#ifdef CONFIG_SERIAL_IMX_UART5
	mxc_register_device(&mxc_uart_device4, &uart4_pdata);
#endif
#ifdef CONFIG_SERIAL_IMX_UART6
	mxc_register_device(&mxc_uart_device5, &uart5_pdata);
#endif
#ifdef CONFIG_USB_EHCI_MXC
	mxc_register_device(&mxc_ehci1, &ehci1_pdata);
#endif
#ifdef CONFIG_I2C
	mxc_register_device(&imx_i2c_device0, &apf27_i2c_0_data);
	i2c_register_board_info(0, apf27dev_i2c_devices_bus0,
				ARRAY_SIZE(apf27dev_i2c_devices_bus0));
	i2c_register_board_info(1, apf27dev_i2c_devices_bus1,
				ARRAY_SIZE(apf27dev_i2c_devices_bus1));
#endif
#ifdef CONFIG_SPI_MXC_SELECT1
	mxc_register_device(&mxc_spi_device0, &imx_spi0_master_info);
#endif
#ifdef CONFIG_SPI_MXC_SELECT2
	mxc_register_device(&mxc_spi_device1, &imx_spi1_master_info);
#endif
#ifdef CONFIG_SPI_MXC_SELECT3
	mxc_register_device(&mxc_spi_device2, &imx_spi2_master_info);
#endif


	platform_add_devices(platform_devices, ARRAY_SIZE(platform_devices));

#ifdef CONFIG_SPI_TSC2102
	/* Due to an hardware "bug", force TSC2101 ChipSelect at startup
	   (on some APF27Dev boards CS stays low, due to TXB0108 chip (U23))
	   and too weak i.MX27 internal pullup resistors */
	tsc2101_init_gpio();
	tsc2101_release_gpio();
	/* Reserve "fixed" GPIOs */
	tsc2101_init_fixed_gpio();
#endif
#if defined (CONFIG_ARMADEUS_MAX1027) || defined (CONFIG_SPI_TSC2102) || defined (CONFIG_CAN_MCP251X) || defined(CONFIG_SPI_SPIDEV)
	spi_register_board_info(spi_board_info,
				ARRAY_SIZE(spi_board_info));
#endif
#ifdef CONFIG_MMC_MXC
	apf27dev_init_mmc();
#endif
#ifdef CONFIG_VIDEO_MX27
	mxc_register_device(&mx27_camera_device, &apf27_camera);
#endif
#ifdef CONFIG_MACH_APF27_DEV_EXT
	apf27dev_extension_init();
#endif

	printk("done\n");
}

