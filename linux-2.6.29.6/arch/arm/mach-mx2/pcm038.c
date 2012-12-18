/*
 * Copyright 2007 Robert Schwebel <r.schwebel@pengutronix.de>, Pengutronix
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
#include <linux/mtd/physmap.h>
#include <linux/mtd/plat-ram.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/mach/time.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/iomux-mx1-mx2.h>
#include <mach/gpio.h>
#include <mach/imx_i2c.h>
#include <mach/imx_spi.h>
#include <mach/imx-uart.h>
#include <mach/imx_ssi.h>
#include <mach/pmic/platform.h>
#include <mach/pmic/power.h>
#include <mach/imx_dam.h>
#include <mach/imx_sound.h>
#include "cpufreq_imx.h"
#include <mach/board-pcm038.h>
#include <mach/mxc_nand.h>

#include "devices.h"
#include "crm_regs.h"

/*
 * Phytec's PCM038 comes with 2MiB battery buffered SRAM,
 * 16 bit width
 */

static struct platdata_mtd_ram pcm038_sram_data = {
	.bankwidth = 2,
};

static struct resource pcm038_sram_resource = {
	.start = CS1_BASE_ADDR,
	.end   = CS1_BASE_ADDR + 512 * 1024 - 1,
	.flags = IORESOURCE_MEM,
};

static struct platform_device pcm038_sram_mtd_device = {
	.name = "mtd-ram",
	.id = 0,
	.dev = {
		.platform_data = &pcm038_sram_data,
	},
	.num_resources = 1,
	.resource = &pcm038_sram_resource,
};

/*
 * Phytec's phyCORE-i.MX27 comes with 32MiB flash,
 * 16 bit width
 */
static struct physmap_flash_data pcm038_flash_data = {
	.width = 2,
};

static struct resource pcm038_flash_resource = {
	.start = 0xc0000000,
	.end   = 0xc1ffffff,
	.flags = IORESOURCE_MEM,
};

static struct platform_device pcm038_nor_mtd_device = {
	.name = "physmap-flash",
	.id = 0,
	.dev = {
		.platform_data = &pcm038_flash_data,
	},
	.num_resources = 1,
	.resource = &pcm038_flash_resource,
};

#ifdef CONFIG_I2C
static int mxc_i2c1_pins[] = {
	PC5_PF_I2C2_SDA,
	PC6_PF_I2C2_SCL
};

static int pcm038_i2c_1_init(struct platform_device *pdev)
{
	return mxc_gpio_setup_multiple_pins(mxc_i2c1_pins, ARRAY_SIZE(mxc_i2c1_pins),
			MXC_GPIO_ALLOC_MODE_NORMAL, "I2C1");
}

static int pcm038_i2c_1_exit(struct platform_device *pdev)
{
	return mxc_gpio_setup_multiple_pins(mxc_i2c1_pins, ARRAY_SIZE(mxc_i2c1_pins),
			MXC_GPIO_ALLOC_MODE_RELEASE, "I2C1");
}

static struct imx_i2c_platform_data pcm038_i2c_1_data = {
	.max_clk = 100000,
	.init = pcm038_i2c_1_init,
	.exit = pcm038_i2c_1_exit,
};

static struct i2c_board_info pcm038_i2c_devices[] = {
	[0] = {
		I2C_BOARD_INFO("at24", 0x52), /* E0=0, E1=1, E2=0 */
		.type = "24c32"
	},
	[1] = {
		I2C_BOARD_INFO("rtc-pcf8563", 0x51),
		.type = "pcf8563"
	},
	[2] = {
		I2C_BOARD_INFO("lm75", 0x4a),
		.type = "lm75"
	}
};
#endif

#ifdef CONFIG_SPI
static int mxc_cspi0_pins[] = {
	PD25_PF_CSPI1_RDY,
	PD27_PF_CSPI1_SS1,
	PD28_PF_CSPI1_SS0,
	PD29_PF_CSPI1_SCLK,
	PD30_PF_CSPI1_MISO,
	PD31_PF_CSPI1_MOSI
};

static int gpio_spi_active_0(struct platform_device *pdev)
{
	return mxc_gpio_setup_multiple_pins(mxc_cspi0_pins, ARRAY_SIZE(mxc_cspi0_pins),
			MXC_GPIO_ALLOC_MODE_NORMAL, "CSPI0");
}

static int gpio_spi_inactive_0(struct platform_device *pdev)
{
	return mxc_gpio_setup_multiple_pins(mxc_cspi0_pins, ARRAY_SIZE(mxc_cspi0_pins),
			MXC_GPIO_ALLOC_MODE_RELEASE, "CSPI0");
}

static struct mxc_spi_master pcm038_spi_0_data = {
	.maxchipselect = 4,	/* FIXME */
	.init = gpio_spi_active_0,
	.exit = gpio_spi_inactive_0,
};

static int gpio_pmic_active(void)
{
	mxc_gpio_mode(GPIO_PORTB | 23 | GPIO_GPIO | GPIO_IN); /* FIXME: Request? */

	return 0;
}

static struct pmic_platform_data pcm038_pmic_irq = {
	.init = gpio_pmic_active,
	.exit = NULL /* TODO required? */
};

/*
 * spi bus 1 description
 *  - serviced by SPI master unit 1 (be seen in kernel as #0)
 *  - SS0 selects the external MC13783
 *    - MC13783's interrupt is connected to PB23
 *  - SS1: external available
 *  - SS2: external available
 *  - SS3: external available
 * FIXME: does this use the spi init functions correctly?
 */
static struct spi_board_info pcm038_spi_board_info[] __initdata = {
	{
		.modalias = "pmic_spi",
		.irq = IRQ_GPIOB(23),
		.max_speed_hz = 3000000, /* FIXME: Depends on internal PLL? */
		.bus_num = 0,
		.chip_select = 0,
		.platform_data = &pcm038_pmic_irq
	}
};
#endif

static int mxc_uart0_pins[] = {
	PE12_PF_UART1_TXD,
	PE13_PF_UART1_RXD,
	PE14_PF_UART1_CTS,
	PE15_PF_UART1_RTS
};

static int uart_mxc_port0_init(struct platform_device *pdev)
{
	return mxc_gpio_setup_multiple_pins(mxc_uart0_pins,
			ARRAY_SIZE(mxc_uart0_pins), "UART0");
}

static int uart_mxc_port0_exit(struct platform_device *pdev)
{
	mxc_gpio_release_multiple_pins(mxc_uart0_pins,
			ARRAY_SIZE(mxc_uart0_pins));
	return 0;
}

static int mxc_uart1_pins[] = {
	PE3_PF_UART2_CTS,
	PE4_PF_UART2_RTS,
	PE6_PF_UART2_TXD,
	PE7_PF_UART2_RXD
};

static int uart_mxc_port1_init(struct platform_device *pdev)
{
	return mxc_gpio_setup_multiple_pins(mxc_uart1_pins,
			ARRAY_SIZE(mxc_uart1_pins), "UART1");
}

static int uart_mxc_port1_exit(struct platform_device *pdev)
{
	mxc_gpio_release_multiple_pins(mxc_uart1_pins,
			ARRAY_SIZE(mxc_uart1_pins));
	return 0;
}

static int mxc_uart2_pins[] = {
	PE8_PF_UART3_TXD,
	PE9_PF_UART3_RXD,
	PE10_PF_UART3_CTS,
	PE11_PF_UART3_RTS
};

static int uart_mxc_port2_init(struct platform_device *pdev)
{
	return mxc_gpio_setup_multiple_pins(mxc_uart2_pins,
			ARRAY_SIZE(mxc_uart2_pins), "UART2");
}

static int uart_mxc_port2_exit(struct platform_device *pdev)
{
	mxc_gpio_release_multiple_pins(mxc_uart2_pins,
			ARRAY_SIZE(mxc_uart2_pins));
	return 0;
}

static struct imxuart_platform_data uart_pdata[] = {
	{
		.init = uart_mxc_port0_init,
		.exit = uart_mxc_port0_exit,
		.flags = IMXUART_HAVE_RTSCTS,
	}, {
		.init = uart_mxc_port1_init,
		.exit = uart_mxc_port1_exit,
		.flags = IMXUART_HAVE_RTSCTS,
	}, {
		.init = uart_mxc_port2_init,
		.exit = uart_mxc_port2_exit,
		.flags = IMXUART_HAVE_RTSCTS,
	},
};

static int mxc_fec_pins[] = {
	PD0_AIN_FEC_TXD0,
	PD1_AIN_FEC_TXD1,
	PD2_AIN_FEC_TXD2,
	PD3_AIN_FEC_TXD3,
	PD4_AOUT_FEC_RX_ER,
	PD5_AOUT_FEC_RXD1,
	PD6_AOUT_FEC_RXD2,
	PD7_AOUT_FEC_RXD3,
	PD8_AF_FEC_MDIO,
	PD9_AIN_FEC_MDC,
	PD10_AOUT_FEC_CRS,
	PD11_AOUT_FEC_TX_CLK,
	PD12_AOUT_FEC_RXD0,
	PD13_AOUT_FEC_RX_DV,
	PD14_AOUT_FEC_CLR,
	PD15_AOUT_FEC_COL,
	PD16_AIN_FEC_TX_ER,
	PF23_AIN_FEC_TX_EN
};

static void gpio_fec_active(void)
{
	mxc_gpio_setup_multiple_pins(mxc_fec_pins,
			ARRAY_SIZE(mxc_fec_pins), "FEC");
}

static void gpio_fec_inactive(void)
{
	mxc_gpio_release_multiple_pins(mxc_fec_pins,
			ARRAY_SIZE(mxc_fec_pins));
}

static struct mxc_nand_platform_data pcm038_nand_board_info = {
	.width = 1,
	.hw_ecc = 1,
};

/* to switch the CPU speed we need various support of other components */
#if ((defined(CONFIG_SPI_MXC) || (defined CONFIG_SPI_MXC_MODULE))  && \
    (defined(CONFIG_CPU_FREQ_IMX27) || defined(CONFIG_CPU_FREQ_IMX27_MODULE)))

static struct clk *mpll_main_0;
static struct clk *mpll_main_1;
static struct clk *cpu;

#define double_byte(x) ((((x)+1)<<1)-1)
#define double_all_bytes(x) ( \
        (double_byte(((x)>>24)&0xFF)<<24) | \
        (double_byte(((x)>>16)&0xFF)<<16) | \
        (double_byte(((x)>> 8)&0xFF)<< 8) | \
        (double_byte((x)&0xFF)) )
/*
 * Switching the CPU frequency makes currently more sense when the
 * MPLL runs a ~800MHz to run the CPU core at ~400MHz. Running the
 * CPU at ~400MHz requires a 1.45V core power supply. As long the
 * boot loader does not provide this voltage we must setup it here
 */
static int pcm038_cpufreq_init(struct platform_device *pd)
{
	t_regulator_config config;
	uint32_t cscr, reg;

	mpll_main_0 = clk_get(NULL, "mpll_main.0");
	mpll_main_1 = clk_get(NULL, "mpll_main.1");
	cpu = clk_get(NULL, "cpu_clk");

	if (IS_ERR(mpll_main_0) || IS_ERR(mpll_main_1) || IS_ERR(cpu)) {
		printk("Cant get one of the required clocks\n");
		/* FIXME error handling */
	}
	/* switch Qvdd (=CPU core supply) to 1.45V */
	config.mode = NO_PULSE_SKIP;
	config.stby_mode = NO_PULSE_SKIP;
	config.voltage.sw1a = SW1A_1_45V;
	config.voltage_lvs.sw1a = SW1A_1_45V;
	config.voltage_stby.sw1a = SW1A_1_45V;
	config.lp_mode = LOW_POWER_DISABLED;
	config.factor = FACTOR_28;

	if (pmic_power_regulator_set_config(SW_SW1A,&config) != PMIC_SUCCESS)
		printk("Regulator error? Ignore this message ;-)\n");

	udelay(1000);	/* TODO tune */

	if (clk_get_rate(mpll_main_0) < 399000000) {
		printk(KERN_INFO "Found MPLL running below ~400Mhz. Tuning "
				"right now.\n");
		/* 0x00331C23 is the setting for 399MHz from the data sheet */
		__raw_writel(0x00331C23, CCM_MPCTL0);

		cscr = __raw_readl(CCM_CSCR);

		/* AHB *must* run below or equal 133MHz */
		cscr &= ~CCM_CSCR_AHB_MASK;
		cscr |= 0x01 << CCM_CSCR_AHB_OFFSET;

		/* start with a 133MHz core frequency. Its more stable than
		 * 400MHz at this point of time.
		 */
		cscr &= ~CCM_CSCR_ARM_MASK;
		cscr |= 0x01 << CCM_CSCR_ARM_OFFSET;

		cscr &= ~CCM_CSCR_ARM_SRC;

		cscr |= CCM_CSCR_MPLLRES;

		/* internal frequency should now be 266MHz */
		reg = __raw_readl(CCM_PCDR1);
		__raw_writel(double_all_bytes(reg), CCM_PCDR1);
		__raw_writel(cscr, CCM_CSCR);	/* switch right now */
		clk_set_parent(cpu, mpll_main_1); /* always the last call! */
		/*
		 * this is a citical moment. It seems the CPU dies if we do
		 * some other things than waiting for the MPLL to lock again
		 */
		udelay(1000);
	}

	return 0;
}

static int pcm038_cpufreq_exit(struct platform_device *pd)
{
	clk_put(mpll_main_0);
	clk_put(mpll_main_1);
	clk_put(cpu);

	return 0;
}

static int pcm038_cpufreq_transit(unsigned long freq)
{
	uint32_t cscr;
	struct clk *new_clk;

	cscr = __raw_readl(CCM_CSCR);

	switch(freq) {
	case 133000:
		/* AHB *can* run at 133MHz */
		cscr &= ~CCM_CSCR_AHB_MASK;
		cscr |= 0x01 << CCM_CSCR_AHB_OFFSET;

		cscr &= ~CCM_CSCR_ARM_MASK;
		cscr |= 0x01 << CCM_CSCR_ARM_OFFSET;	/* 133MHz */

		cscr &= ~CCM_CSCR_ARM_SRC;
		new_clk = mpll_main_1;
		break;

	case 266000:
		/* AHB *can* run at 133MHz */
		cscr &= ~CCM_CSCR_AHB_MASK;
		cscr |= 0x01 << CCM_CSCR_AHB_OFFSET;

		cscr &= ~CCM_CSCR_ARM_MASK;
		cscr |= 0x00 << CCM_CSCR_ARM_OFFSET;	/* 266MHz speed */

		cscr &= ~CCM_CSCR_ARM_SRC;
		new_clk = mpll_main_1;
		break;

	case 400000:
		/* AHB *must* run at 133MHz */
		cscr &= ~CCM_CSCR_AHB_MASK;
		cscr |= 0x01 << CCM_CSCR_AHB_OFFSET;

		cscr &= ~CCM_CSCR_ARM_MASK;
		cscr |= 0x00 << CCM_CSCR_ARM_OFFSET;	/* full speed */

		cscr |= CCM_CSCR_ARM_SRC;
		new_clk = mpll_main_0;
		break;

	default:
		return -EINVAL;
	}
	__raw_writel(cscr, CCM_CSCR);
	clk_set_parent(cpu, new_clk);

	return 0;
}

static struct cpufreq_frequency_table pcm038_freq_table[] = {
	{0x01, .frequency = 133000 }, /* with 400MHz MPLL / 3 @ 1.45V Qvdd */
	{0x02, .frequency = 266000 }, /* with 400MHz MPLL / 2 @ 1.45V Qvdd */
	{0x03, .frequency = 400000 }, /* with 400MHz MPLL / 1 @ 1.45V Qvdd */
	{0, CPUFREQ_TABLE_END}
};

static struct cpufreq_platform_data pcm038_cpufreq_platform_data = {
	.freq_table = pcm038_freq_table,
	.freq_entries = ARRAY_SIZE(pcm038_freq_table),
	.init = pcm038_cpufreq_init,
	.exit = pcm038_cpufreq_exit,
	.transit = pcm038_cpufreq_transit
};

static struct platform_device pcm038_cpu_frequency_device = {
	.name = "imx_cpufreq",
	.id = 0,
	.dev = {
		.platform_data = &pcm038_cpufreq_platform_data,
	}
};
# define CPU_FREQUENCY &pcm038_cpu_frequency_device,
#else
# define CPU_FREQUENCY
#endif

#ifdef CONFIG_SND

/* ------------ sound is complicated ----------------- */

static int mxc_ssi_pins[] = {
	PC20_PF_SSI1_FS,
	PC21_PF_SSI1_RXD,
	PC22_PF_SSI1_TXD,
	PC23_PF_SSI1_CLK,

	PC16_PF_SSI4_FS,
	PC17_PF_SSI4_RXD,
	PC18_PF_SSI4_TXD,
	PC19_PF_SSI4_CLK
};

static int gpio_ssi_activate(struct platform_device *pdev)
{
	int err;

	err = mxc_gpio_setup_multiple_pins(mxc_ssi_pins, ARRAY_SIZE(mxc_ssi_pins),
				MXC_GPIO_ALLOC_MODE_NORMAL, "imx-sound");
	if (err < 0)
		pr_err("Cannot register SSI pins for sound usage\n");

	return err;
}

static struct imx_sound_platform_data pcm038_alsa_sound = {
	.connection = {
		[0] = {
			.cpu_port = 1,	/* SSI1_* pin group */
			.dev_port = 1	/* connected to port 1 of the PMIC */
		},
		[1] = {
			.cpu_port = 4,	/* SSI4_* pin group */
			.dev_port = 2	/* connected to port 2 of the PMIC */
		}
	},
	.init = gpio_ssi_activate,
	.exit = NULL	/* FIXME */
};

static struct platform_device pcm038_alsa_sound_device = {
	.name = "imx-alsa",
	.id = 0,
	.dev = {
		.platform_data = &pcm038_alsa_sound,
		.coherent_dma_mask = DMA_BIT_MASK(32)
	}
};

static struct imx_ssi_platform_data pcm038_ssi0 = {
	.init = NULL,	/* FIXME currently done in the audio device */
	.exit = NULL	/* FIXME */
};

static struct imx_ssi_platform_data pcm038_ssi1 = {
	.init = NULL,	/* FIXME currently done in the audio device */
	.exit = NULL	/* FIXME */
};
# define ALSA_SOUND &pcm038_alsa_sound_device,
#else
# define ALSA_SOUND
#endif

static struct platform_device *platform_devices[] __initdata = {
	&pcm038_nor_mtd_device,
	CPU_FREQUENCY
	ALSA_SOUND
	&mxc_w1_master_device,
	&pcm038_sram_mtd_device,
};

/* On pcm038 there's a sram attached to CS1, we enable the chipselect here and
 * setup other stuffs to access the sram. */
static void __init pcm038_init_sram(void)
{
	__raw_writel(0x0000d843, CSCR_U(1));
	__raw_writel(0x22252521, CSCR_L(1));
	__raw_writel(0x22220a00, CSCR_A(1));
}

static void __init pcm038_init(void)
{
	gpio_fec_active();
	pcm038_init_sram();

	mxc_register_device(&mxc_uart_device0, &uart_pdata[0]);
	mxc_register_device(&mxc_uart_device1, &uart_pdata[1]);
	mxc_register_device(&mxc_uart_device2, &uart_pdata[2]);

	mxc_gpio_mode(PE16_AF_RTCK); /* OWIRE */
	mxc_register_device(&mxc_nand_device, &pcm038_nand_board_info);

#ifdef CONFIG_I2C
	/* only the i2c master 1 is used on this CPU card */
	i2c_register_board_info(1, pcm038_i2c_devices,
				ARRAY_SIZE(pcm038_i2c_devices));

	mxc_register_device(&imx_i2c_device1, &pcm038_i2c_1_data);
#endif

#ifdef CONFIG_SPI
	mxc_register_device(&mxc_spi_device1, &pcm038_spi_0_data);

	spi_register_board_info(pcm038_spi_board_info,
				ARRAY_SIZE(pcm038_spi_board_info));
#endif

#ifdef CONFIG_SND
	mxc_register_device(&mxc_dam_device, NULL);
	mxc_register_device(&imx_ssi_device0, &pcm038_ssi0);
	mxc_register_device(&imx_ssi_device1, &pcm038_ssi1);
#endif

	platform_add_devices(platform_devices, ARRAY_SIZE(platform_devices));

#ifdef CONFIG_MACH_PCM970_BASEBOARD
	pcm970_baseboard_init();
#endif
}

static void __init pcm038_timer_init(void)
{
	mxc_clocks_init(26000000);
	mxc_timer_init("gpt_clk.0");
}

struct sys_timer pcm038_timer = {
	.init = pcm038_timer_init,
};

MACHINE_START(PCM038, "phyCORE-i.MX27")
	.phys_io        = AIPI_BASE_ADDR,
	.io_pg_offst    = ((AIPI_BASE_ADDR_VIRT) >> 18) & 0xfffc,
	.boot_params    = PHYS_OFFSET + 0x100,
	.map_io         = mxc_map_io,
	.init_irq       = mxc_init_irq,
	.init_machine   = pcm038_init,
	.timer          = &pcm038_timer,
MACHINE_END
