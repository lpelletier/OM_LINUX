/*
 * linux/arch/arm/mach-imx/apf9328.c
 *
 * Copyright (c) 2005-2009 Armadeus systems
 * This work is based on scb9328.c from Sascha Hauer
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
#include <linux/serial_8250.h>
#include <linux/spi/spi.h>

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
#include <linux/spi/max1027.h>
#include <mach/imx_ssi.h>
#include <mach/dma.h>
#include "../../../drivers/armadeus/pwm/pwm.h"
#include <linux/dm9000.h>
#include <mach/imxfb.h>

#include "generic.h"

#ifdef CONFIG_ARMADEUS_MAX1027_MODULE
#define CONFIG_ARMADEUS_MAX1027 1
#endif
#ifdef CONFIG_SPI_IMX_MODULE
#define CONFIG_SPI_IMX 1
#endif
#ifdef CONFIG_SND_IMX_PCM_MODULE
#define CONFIG_SND_IMX_PCM 1
#endif
#ifdef CONFIG_IMX_BACKLIGHT_MODULE
#define CONFIG_IMX_BACKLIGHT 1
#endif
#ifdef CONFIG_ARMADEUS_PWM_DRIVER_MODULE
#define CONFIG_PWM
#endif


/*
 * APF9328 has a DM9000 Ethernet controller
 */
static struct resource dm9000_resources[] = {
	[0] = {
		.start  = (APF9328_ETH_PHYS + 0),
		.end    = (APF9328_ETH_PHYS + 1),
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = (APF9328_ETH_PHYS + 2),
		.end    = (APF9328_ETH_PHYS + 3),
		.flags  = IORESOURCE_MEM,
	},
	[2] = {
		.start  = (APF9328_ETH_IRQ),
		.end    = (APF9328_ETH_IRQ),
		.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE,
	},
};

static struct dm9000_plat_data dm9000_setup = {
	.flags          = DM9000_PLATF_16BITONLY
};

static struct platform_device dm9000x_device = {
	.name           = "dm9000",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(dm9000_resources),
	.resource       = dm9000_resources,
	.dev            = { .platform_data = &dm9000_setup, },
};


#ifdef CONFIG_PWM
static struct resource pwm_resources[] = {
	[0] = {
		.start  = 0x00208000,
		.end    = 0x0020800c,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = (PWM_INT),
		.end    = (PWM_INT),
		.flags  = IORESOURCE_IRQ,
	},
};

static int apf9328_pwm_0_init(void)
{
	/* Init Port PA[2] : PWMO*/
	DDIR(0) |=  0x00000002;
	GIUS(0) &= ~0x00000002;
	GPR(0)  &= ~0x00000002;
	return 0;
}

static int apf9328_pwm_0_exit(void)
{
	return 0;
}

static struct imx_pwm_platform_data apf9328_pwm_0_data = {
	.init = apf9328_pwm_0_init,
	.exit = apf9328_pwm_0_exit,
};

static struct platform_device imx_pwm_device = {
	.name           = "imx-pwm",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(pwm_resources),
	.resource       = pwm_resources,
	.dev            = {
		.platform_data = &apf9328_pwm_0_data,
	}
};
#endif /* CONFIG_PWM */


#ifdef CONFIG_USB_GADGET_IMX
static int apf9328_udc_0_init(struct device * dev)
{
	imx_gpio_mode(PC3_PF_SSI_RXFS);
	imx_gpio_mode(PB20_PF_USBD_AFE);
	imx_gpio_mode(PB21_PF_USBD_OE);
	imx_gpio_mode(PB22_PF_USBD_RCV);
	imx_gpio_mode(PB23_PF_USBD_SUSPND);
	imx_gpio_mode(PB24_PF_USBD_VP);
	imx_gpio_mode(PB25_PF_USBD_VM);
	imx_gpio_mode(PB26_PF_USBD_VPO);
	imx_gpio_mode(PB27_PF_USBD_VMO);

	return 0;
}

static int apf9328_udc_0_exit(struct device * dev)
{
	return 0;
}

static struct imxusb_platform_data apf9328_udc_0_data = {
	.init = apf9328_udc_0_init,
	.exit = apf9328_udc_0_exit,
};
#endif /* CONFIG_USB_GADGET_IMX */


#if defined(CONFIG_SND_IMX_PCM) || defined(CONFIG_SND_IMX_PCM_MODULE)
int apf9328_ssi_gpio_init(struct platform_device *pdev)
{
	/* Activate SSI on PortC */
	imx_gpio_mode(PC3_PF_SSI_RXFS);
	FMCR &= ~(SSI_RXFS_SEL);
	imx_gpio_mode(PC4_PF_SSI_RXCLK);
	FMCR &= ~(SSI_RXCLK_SEL);
	imx_gpio_mode(PC5_PF_SSI_RXDAT);
	FMCR &= ~(SSI_RXDAT_SEL);
	imx_gpio_mode(PC6_PF_SSI_TXDAT);
	imx_gpio_mode(PC7_PF_SSI_TXFS);
	FMCR &= ~(SSI_TXFS_SEL);
	imx_gpio_mode(PC8_PF_SSI_TXCLK);
	FMCR &= ~(SSI_TXCLK_SEL);

	printk("SSI pins configured\n");
	return 0;
}

static struct imx_ssi_platform_data apf9328_ssi_pdata = {
	.init = apf9328_ssi_gpio_init,
};

static struct resource ssi_resources[] = {
	[0] = {
		.start  = (0x00218000),
		.end    = (0x00218000 + 0x28),
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start	= SSI_TX_INT,
		.end	= SSI_TX_INT,
		.flags	= IORESOURCE_IRQ
	},
	[2] = {
		.start	= SSI_TX_ERR_INT,
		.end	= SSI_TX_ERR_INT,
		.flags	= IORESOURCE_IRQ
	},
	[3] = {
		.start	= SSI_RX_INT,
		.end	= SSI_RX_INT,
		.flags	= IORESOURCE_IRQ
	},
	[4] = {
		.start	= SSI_RX_ERR_INT,
		.end	= SSI_RX_ERR_INT,
		.flags	= IORESOURCE_IRQ
	},
	[5] = {
		.name	= "tx0",
		.start	= DMA_REQ_SSI_T,
		.end	= DMA_REQ_SSI_T,
		.flags	= IORESOURCE_DMA
	},
	[6] = {
		.name	= "rx0",
		.start	= DMA_REQ_SSI_R,
		.end	= DMA_REQ_SSI_R,
		.flags	= IORESOURCE_DMA
	},
};

static struct platform_device apf9328_ssi_device = {
	.name           = "imx-ssi",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(ssi_resources),
	.resource       = ssi_resources,
	.dev            = {
		.platform_data = &apf9328_ssi_pdata,
	}
};

#endif /* defined(CONFIG_SND_IMX_PCM) || defined(CONFIG_SND_IMX_PCM_MODULE) */


/* --- SERIAL RESSOURCE --- */
static struct imxuart_platform_data uart1_pdata = {
	.flags = 0,
};

static struct resource imx_uart1_resources[] = {
	[0] = {
		.start  = 0x00206000,
		.end    = 0x002060FF,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = (UART1_MINT_RX),
		.end    = (UART1_MINT_RX),
		.flags  = IORESOURCE_IRQ,
	},
	[2] = {
		.start  = (UART1_MINT_TX),
		.end    = (UART1_MINT_TX),
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device imx_uart1_device = {
	.name           = "imx-uart",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(imx_uart1_resources),
	.resource       = imx_uart1_resources,
	.dev            = {
		.platform_data = &uart1_pdata,
	}
};

static struct imxuart_platform_data uart2_pdata = {
        .flags = IMXUART_HAVE_RTSCTS,
};

static struct resource imx_uart2_resources[] = {
	[0] = {
		.start  = 0x00207000,
		.end    = 0x002070FF,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = (UART2_MINT_RX),
		.end    = (UART2_MINT_RX),
		.flags  = IORESOURCE_IRQ,
	},
	[2] = {
		.start  = (UART2_MINT_TX),
		.end    = (UART2_MINT_TX),
		.flags  = IORESOURCE_IRQ,
	},
	[3] = {
		.start	= UART2_MINT_RTS,
		.end	= UART2_MINT_RTS,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device imx_uart2_device = {
	.name           = "imx-uart",
	.id             = 1,
	.num_resources  = ARRAY_SIZE(imx_uart2_resources),
	.resource       = imx_uart2_resources,
	.dev            = {
		.platform_data = &uart2_pdata,
	}
};

#ifdef CONFIG_SPI_IMX
static struct resource imx_spi1_resource[] = {
	[0] = {
		.start = 0x00213000,
		.end   = 0x00213fff,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = CSPI_INT,
		.end   = CSPI_INT,
		.flags = IORESOURCE_IRQ,
	}
};

static struct spi_imx_master imx_spi1_master_info = {
	.num_chipselect	= 2,
	.enable_dma     = 0,
};

static struct platform_device imx_spi1 = {
	.name		= "spi_imx",
	.id		= 1,
	.resource	= imx_spi1_resource,
	.num_resources	= ARRAY_SIZE(imx_spi1_resource),
	.dev = {
		.platform_data = &imx_spi1_master_info,
	},
};

static struct resource imx_spi2_resource[] = {
	[0] = {
		.start = 0x00219000,
		.end   = 0x00219fff,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = SPI2_INT,
		.end   = SPI2_INT,
		.flags = IORESOURCE_IRQ,
	}
};

static struct spi_imx_master imx_spi2_master_info = {
	.num_chipselect	= 2,
	.enable_dma     = 0,
};

static struct platform_device imx_spi2 = {
	.name		= "spi_imx",
	.id		= 2,
	.resource	= imx_spi2_resource,
	.num_resources	= ARRAY_SIZE(imx_spi2_resource),
	.dev = {
		.platform_data = &imx_spi2_master_info,
	},
};
#endif /* CONFIG_SPI_IMX */

/*
 * APF9328 can have an optionnal MAX1027 ADC
 */
#ifdef CONFIG_ARMADEUS_MAX1027
#define IMX_SPI1_SS_PIN 15
#define IMX_SPI1_SS (1<<15)
#define MAX1027_EOC_INT 13
#define MAX1027_CNVST 14

static int max1027_init(struct spi_device *spi)
{
	/* SPI1 GPIOs */
	imx_gpio_mode(PC14_PF_SPI1_SCLK);
	imx_gpio_mode(PC16_PF_SPI1_MISO);
	imx_gpio_mode(PC17_PF_SPI1_MOSI);

	/* configure CNVST */
	DR(0) |= 1 << MAX1027_CNVST; /* Initializes it High */
	imx_gpio_mode(GPIO_PORTA | MAX1027_CNVST | GPIO_OUT | GPIO_GIUS | GPIO_DR);

	/* configure EOC */
	imx_gpio_mode(GPIO_PORTA | MAX1027_EOC_INT | GPIO_IN | GPIO_GIUS);

	/* SPI CSn is used as chip select but in GPIO mode */
	DR(2) |= IMX_SPI1_SS;
	imx_gpio_mode(GPIO_PORTC | IMX_SPI1_SS_PIN | GPIO_OUT | GPIO_GIUS | GPIO_DR);

	return 0;
}

static int max1027_exit(struct spi_device *spi)
{
	return 0;
}

static void max1027_cs(u32 command)
{
	if (command == SPI_CS_DEASSERT)
		DR(2) |= IMX_SPI1_SS;
	else
		DR(2) &= ~IMX_SPI1_SS;
}

static struct spi_imx_chip max1027_hw = {
	.cs_control		= max1027_cs,
};

static struct max1027_config apf9328_max1027_config = {
	.conv = MAX1027_CONV_DEFAULT,
	.setup = MAX1027_SETUP_DEFAULT,
	.avg = MAX1027_AVG_DEFAULT,
	.cnvst_pin = (int)(MAX1027_CNVST|GPIO_PORTA),
	.init = max1027_init,
	.exit = max1027_exit
};
#endif /* CONFIG_ARMADEUS_MAX1027 */


#ifdef CONFIG_ARMADEUS_MAX1027
static struct spi_board_info spi_board_info[] __initdata = {
{
	.modalias	= "max1027",
	.controller_data= &max1027_hw,
	.max_speed_hz	= 8000000,
	.bus_num	= 1, /* SPI1 */
	.chip_select	= 0,
	.mode 		= 0,
	.irq		= IRQ_GPIOA(MAX1027_EOC_INT),
	.platform_data	= &apf9328_max1027_config,
},
};
#endif

#ifdef CONFIG_IMX_BACKLIGHT
static struct imxbl_machinfo imx_bl_machinfo = {
	.max_intensity      = 0xff,
	.default_intensity  = 0x90,
	.limit_mask         = 0x7f, /* When battery is low */
/*    set_bl_intensity = put a function here if you want to overload default one, */
};

static struct platform_device imxbl_device = {
	.name       = "imx-bl",
	.dev        = {
	//        .parent = &imxfb_device.dev, crash kernel even if EXPORT_SYMBOL() is done in generic.c
		.platform_data	= &imx_bl_machinfo,
	},
	.id        = 0,
};
#endif /* CONFIG_IMX_BACKLIGHT */

/*
 * The APF9328 can have up to 32MB NOR Flash
 */
static struct resource flash_resource = {
	.start  = IMX_CS0_PHYS,
	.end    = IMX_CS0_PHYS + (32 * 1024 * 1024) - 1,
	.flags  = IORESOURCE_MEM,
};

static struct physmap_flash_data apf9328_flash_data = {
	.width  = 2,
};

static struct platform_device apf9328_flash_device = {
	.name           = "physmap-flash",
	.id             = 0,
	.dev = {
		.platform_data = &apf9328_flash_data,
	},
	.resource = &flash_resource,
	.num_resources = 1,
};

static struct platform_device *devices[] __initdata = {
	&imx_uart1_device,
	&imx_uart2_device,
	&dm9000x_device,
#ifdef CONFIG_SPI_IMX
	&imx_spi1,
	&imx_spi2,
#endif
#ifdef CONFIG_PWM
	&imx_pwm_device,
#endif
#ifdef CONFIG_IMX_BACKLIGHT
	&imxbl_device,
#endif
#if defined(CONFIG_SND_IMX_PCM) || defined(CONFIG_SND_IMX_PCM_MODULE)
	&apf9328_ssi_device,
#endif
	&apf9328_flash_device,
};

static void __init apf9328_init(void)
{
	printk("--- Registering APF9328 ressources\n");

	/* Initializes serial port 1/console GPIOs */
	imx_gpio_mode(PC9_PF_UART1_CTS);
	imx_gpio_mode(PC10_PF_UART1_RTS);
	imx_gpio_mode(PC11_PF_UART1_TXD);
	imx_gpio_mode(PC12_PF_UART1_RXD);

#ifdef CONFIG_USB_GADGET_IMX
	set_imx_usb_info(&apf9328_udc_0_data);
#endif

	platform_add_devices(devices, ARRAY_SIZE(devices));

#if defined (CONFIG_ARMADEUS_MAX1027)
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
#endif

	apf9328_baseboard_init();

	printk("--- APF9328 ressources registered\n");
}

/*
 * Register here the memory addresses we want to access from our drivers and
 * which are not already registered in generic.c
 */
#ifdef CONFIG_FB_IMX
#define IMX_FB_PHYS     (0x0C000000 - 0x40000)
#endif

static struct map_desc apf9328_io_desc[] __initdata = {
#ifdef CONFIG_FB_IMX
	{
		.virtual    = IMX_FB_VIRT,
		.pfn        = __phys_to_pfn(IMX_FB_PHYS),
		.length     = IMX_FB_SIZE,
		.type       = MT_DEVICE
	},
#endif
};

void __init apf9328_map_io(void)
{
	imx_map_io();
	iotable_init(apf9328_io_desc, ARRAY_SIZE(apf9328_io_desc));
}

static void __init apf9328_init_irq(void)
{
	/* Init generic IRQs */
	imx_init_irq();

	/* Init our custom IRQs (DM9000) */
	set_irq_type(APF9328_ETH_IRQ, IRQF_TRIGGER_FALLING);
}


MACHINE_START(APF9328, "Armadeus APF9328")
	/* Maintainer: Julien Boibessot, Armadeus */
	.phys_io      = 0x00200000,
	.io_pg_offst  = ((0xe0200000) >> 18) & 0xfffc,
	.boot_params  = 0x08000100,
	.map_io       = apf9328_map_io,
	.init_irq     = apf9328_init_irq,
	.timer        = &imx_timer,
	.init_machine = apf9328_init,
MACHINE_END
