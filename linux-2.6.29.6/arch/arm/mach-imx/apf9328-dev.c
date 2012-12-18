/*
 * linux/arch/arm/mach-imx/apf9328-dev.c
 * Declares hardware present on APF9328_DevLight or DevFull boards
 *
 * Copyright (c) 2009 Armadeus systems
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mtd/physmap.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/spi/tsc2102.h>
#include "../../../drivers/net/can/mcp251x.h"

#include <asm/system.h>
#include <mach/hardware.h>
#include <linux/irq.h>
#include <asm/pgtable.h>
#include <asm/page.h>

#include <asm/mach/map.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/imx-uart.h>
#include <mach/mmc.h>
#include <mach/usb.h>
#include <mach/spi_imx.h>
#include <mach/imx-regs.h> /* imx_gpio_mode() */
#include <mach/gpio.h>
#include <mach/imx-alsa.h>
#include <linux/usb/isp116x.h>
#include <mach/imx_ssi.h>

#include "generic.h"


#ifdef CONFIG_SPI_TSC2102_MODULE
#define CONFIG_SPI_TSC2102 1
#endif
#ifdef CONFIG_CAN_MCP251X_MODULE
#define CONFIG_CAN_MCP251X 1
#endif
#ifdef CONFIG_ARMADEUS_ISP1761_MODULE
#define CONFIG_ARMADEUS_ISP1761 1
#endif
#ifdef CONFIG_SPI_SPIDEV_MODULE
#define CONFIG_SPI_SPIDEV 1
#endif



/*
 * APF9328_DevFull board has an ISP1761 USB Host controller
 */
#ifdef CONFIG_ARMADEUS_ISP1761
#define ISP1761_BASE		IMX_CS3_PHYS
#define ISP1761_GPIO_IRQ	(GPIO_PORTC | 10)
#define ISP1761_IRQ		(IRQ_GPIOC(10))

static struct resource devfull_isp1761_resources[] = {
	[0] = {
		.name	= "isp1761-regs",
		.start  = ISP1761_BASE + 0x00000000,
		.end    = ISP1761_BASE + 0x000fffff,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = ISP1761_IRQ,
		.end    = ISP1761_IRQ,
		.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL,
	},
};

static struct platform_device devfull_isp1761_device = {
	.name           = "isp1761",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(devfull_isp1761_resources),
	.resource       = devfull_isp1761_resources,
};
#endif /* CONFIG_ARMADEUS_ISP1761 */


/*
 * APF9328_DevFull board has a TSC2102 for touchscreens control
 * and sound playing
 */
#ifdef CONFIG_SPI_TSC2102
#define TSC2102_CS_GPIOB 17
#define TSC2102_INT_GPIOD 8

static int tsc2102_init_gpio(void)
{
	/* Activate SPI2 port ---- */
	/* PortB 17 is used as chip select (in GPIO mode) */
	DR(1) |= 1 << TSC2102_CS_GPIOB; /* Initializes it High */
	imx_gpio_mode(GPIO_PORTB | TSC2102_CS_GPIOB | GPIO_OUT | GPIO_GIUS | GPIO_DR);
	/* Configure SPI2 outputs */
	imx_gpio_mode(PD7_AIN_SPI2_SCLK);
	imx_gpio_mode(PD10_AIN_SPI2_TXD);
	imx_gpio_mode(PD9_AOUT_SPI2_RXD);
	FMCR |= SPI2_RXD_SEL; /* RX is AOUT on PORTD */

	/* PortD 8 is used as PINTDAV interrupt */
	set_irq_type(IRQ_GPIOD(TSC2102_INT_GPIOD), IRQF_TRIGGER_FALLING);
	imx_gpio_mode(GPIO_PORTD | TSC2102_INT_GPIOD | GPIO_IN | GPIO_GIUS);

	printk("TSC2102 GPIOs initialized\n");
	return 0;
}

/* Chip select command for TSC2102 */
static void tsc2102_cs(u32 command)
{
	/* PortB 19 is used as chip select */
	if (command == SPI_CS_DEASSERT)
		DR(1) |= 1<< TSC2102_CS_GPIOB;
	else
		DR(1) &= ~(1<< TSC2102_CS_GPIOB);
}

static struct spi_imx_chip tsc2102_hw = {
	.cs_control		= tsc2102_cs,
};

static struct tsc210x_config apf9328_tsc2102_config = {
	.use_internal = 1, /* -> use internal voltage reference */
	.monitor = TSC_BAT1 | TSC_AUX | TSC_TEMP,
	.init = tsc2102_init_gpio,
/*    .apm_report = palmte_get_power_status,*/
/*#if defined(CONFIG_SND_IMX_TSC2102) || defined(CONFIG_SND_IMX_TSC2102_MODULE)
	.alsa_config = &apf9328_alsa_pdata,
#endif*/
};

#if defined(CONFIG_SND_IMX_TSC2102) || defined(CONFIG_SND_IMX_TSC2102_MODULE)
static struct imx_sound_platform_data apf9328_alsa_pdata;

static struct platform_device tsc2102_alsa_device = {
	.name           = "tsc210x-alsa",
	.id             = 0,
	.dev            = {
		.platform_data = &apf9328_alsa_pdata,
	}
};
#endif /* defined(CONFIG_SND_IMX_TSC2102) || defined(CONFIG_SND_IMX_TSC2102_MODULE) */

#endif /* CONFIG_SPI_TSC2102 */

/*
 * APF9328_DevFull board has a MCP251X CAN bus Host controller
 */
#ifdef CONFIG_CAN_MCP251X
#define CAN_MPC251X_CS_GPIOB 19
#define CAN_MPC251X_INT_GPIOC 13

/* Chip select command for MCP251X */
static void mcp251X_cs(u32 command)
{
	/* PortB 19 is used as chip select */
	if (command == SPI_CS_DEASSERT)
		DR(1) |= 1<< CAN_MPC251X_CS_GPIOB;
	else
		DR(1) &= ~(1<< CAN_MPC251X_CS_GPIOB);
}

static struct spi_imx_chip mcp251X_hw = {
	.cs_control		= mcp251X_cs,
};

static void mcp251X_init_gpio(void)
{
	/* PortB 19 is used as chip select (in GPIO mode) */
	DR(1) |= 1 << CAN_MPC251X_CS_GPIOB; /* Initializes it High */
	imx_gpio_mode(GPIO_PORTB | CAN_MPC251X_CS_GPIOB | GPIO_OUT | GPIO_GIUS | GPIO_DR);

	/* PortC 13 is used as CAN interrupt */
	set_irq_type(IRQ_GPIOC(CAN_MPC251X_INT_GPIOC), IRQF_TRIGGER_FALLING);
	imx_gpio_mode(GPIO_PORTC | CAN_MPC251X_INT_GPIOC | GPIO_IN | GPIO_GIUS);

	/* Configure SPI2 outputs */
	imx_gpio_mode(PD7_AIN_SPI2_SCLK);
	imx_gpio_mode(PD10_AIN_SPI2_TXD);
	imx_gpio_mode(PD9_AOUT_SPI2_RXD);
	FMCR |= SPI2_RXD_SEL; /* RX is AOUT on PORTD */
}

static struct mcp251x_platform_data apf9328_mcp251x_config = {
	.oscillator_frequency  = 16000000,
	.board_specific_setup  = NULL,
	.device_reset          = NULL,
	.transceiver_enable    = NULL,
};

#endif /* CONFIG_CAN_MCP251X */

/*
 * APF9328 CONFIG SPIDEV
 */
#ifdef CONFIG_SPI_SPIDEV
#define SPIDEV_CS_GPIOB 18

static int spidev_init_gpio(void)
{
	/* SPI1 GPIOs */
	imx_gpio_mode(PC14_PF_SPI1_SCLK);
	imx_gpio_mode(PC16_PF_SPI1_MISO);
	imx_gpio_mode(PC17_PF_SPI1_MOSI);

	/* PortB 18 is used as chip select (in GPIO mode) */
	DR(1) |= 1 << SPIDEV_CS_GPIOB; /* Initializes it High */
	imx_gpio_mode(GPIO_PORTB | SPIDEV_CS_GPIOB | GPIO_OUT | GPIO_GIUS | GPIO_DR);

	return 0;
}

/* Chip select command for spidev */
static void spidev_cs(u32 command)
{
	/* PortB 18 is used as chip select */
	if (command == SPI_CS_DEASSERT)
		DR(1) |= 1<< SPIDEV_CS_GPIOB;
	else
		DR(1) &= ~(1<< SPIDEV_CS_GPIOB);
}

static struct spi_imx_chip spidev_hw = {
	.cs_control     = spidev_cs,
};

static struct spidev_platform_data apf9328_spidev_config = {
	.init = spidev_init_gpio,
};

#endif /* CONFIG_SPI_SPIDEV */


static struct spi_board_info spi_dev_board_info[] __initdata = {
#ifdef CONFIG_SPI_TSC2102
{
	.modalias	= "tsc210x",
	.controller_data= &tsc2102_hw,
	.max_speed_hz	= 8000000,
	.bus_num	= 2, /* SPI2 */
	.irq		= IRQ_GPIOD(TSC2102_INT_GPIOD),
	.chip_select	= 0,
	.mode 		= 0,
	.platform_data	= &apf9328_tsc2102_config,
},
#endif
#ifdef CONFIG_CAN_MCP251X
{
	.modalias	= "mcp251x",
	.max_speed_hz	= 8000000, /* 8MHz */
	.controller_data= &mcp251X_hw,
	.bus_num	= 2,
	.mode		= SPI_MODE_0,
	.chip_select	= 1,
	.irq		= IRQ_GPIOC( CAN_MPC251X_INT_GPIOC ),
	.platform_data	= &apf9328_mcp251x_config,
},
#endif
#ifdef CONFIG_SPI_SPIDEV
{
	.modalias          = "spidev",
	.controller_data    = &spidev_hw,
	.max_speed_hz      = 8000000, /* 8MHz */
	.bus_num           = 1, /* SPI1 */
	.mode              = SPI_MODE_0,
	.chip_select       = 1, 
	.platform_data     = &apf9328_spidev_config,
},
#endif /* CONFIG_SPI_SPIDEV */
};

/*
 * You may connect several types of LCD on these boards
 */
#ifdef CONFIG_FB_IMX
/* all custom LCD configuration is deported to this file for clarity purpose: */
#include "apf9328_lcd_config.h"
#endif


static struct platform_device *devices[] __initdata = {
#if defined(CONFIG_SND_IMX_TSC2102) || defined(CONFIG_SND_IMX_TSC2102_MODULE)
	&tsc2102_alsa_device,
#endif
#ifdef CONFIG_ARMADEUS_ISP1761
	&devfull_isp1761_device,
#endif
};

void __init apf9328_baseboard_init(void)
{
	printk("    Registering APF9328_Dev ressources:");

	/* Initializes serial port 2 GPIOs */
	imx_gpio_mode(PB30_PF_UART2_TXD);
	imx_gpio_mode(PB31_PF_UART2_RXD);
	imx_gpio_mode(PB28_PF_UART2_CTS);
	imx_gpio_mode(PB29_PF_UART2_RTS);

#ifdef CONFIG_FB_IMX
	set_imx_fb_info(&apf9328_fb_info);
#endif
#ifdef CONFIG_CAN_MCP251X
	mcp251X_init_gpio();
#endif
#ifdef CONFIG_ARMADEUS_ISP1761
	set_irq_type(ISP1761_IRQ, IRQF_TRIGGER_FALLING);
	imx_gpio_mode(ISP1761_GPIO_IRQ | GPIO_IN | GPIO_GIUS);
#endif
	platform_add_devices(devices, ARRAY_SIZE(devices));

#if defined (CONFIG_SPI_TSC2102) || defined (CONFIG_CAN_MCP251X) || defined(CONFIG_SPI_SPIDEV)
	spi_register_board_info(spi_dev_board_info, ARRAY_SIZE(spi_dev_board_info));
#endif

	printk(" done\n");
}

