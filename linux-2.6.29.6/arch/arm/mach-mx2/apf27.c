 /*
 * apf27.c
 *
 * Support for the APF27 System On Module
 *
 * Copyright (C) 2008-2011 ARMadeus Systems
 * Authors: Julien Boibessot <julien.boibessot@armadeus.com>
 *          Eric Jarrige <eric.jarrige@armadeus.com>
 *
 * Inspired a lot by pcm038.c which is:
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/platform_device.h>
#include <linux/mtd/physmap.h>
#include <linux/i2c.h>
#include <linux/serial.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/cpufreq.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/fsl_devices.h> /* USB device */

#include <asm/mach/flash.h>
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
#include <mach/imx-uart.h>
#include <mach/mxc_nand.h>
#include <mach/mxc_ehci.h>
#include <mach/ulpi.h>
#include "../../../drivers/armadeus/pwm/pwm.h"
#if defined(CONFIG_CPU_FREQ_IMX27) || defined(CONFIG_CPU_FREQ_IMX27_MODULE)
#include "cpufreq_imx.h"
#endif
#include <mach/board-apf27.h>
#include <asm/mach/map.h>

#include "devices.h"
#include "crm_regs.h"

extern void apf27_baseboard_init(void);
#ifdef CONFIG_PM
extern void apf27_baseboard_resume(void);
extern void apf27_baseboard_suspend(void);
#endif
#ifdef CONFIG_FB_MXC
extern void apf27_lcd_startup(void);
#endif

#ifdef CONFIG_ARMADEUS_PWM_DRIVER_MODULE
#define CONFIG_PWM
#endif


#ifdef CONFIG_PWM
static int mxc_pwm0_pins[] = {
	PE5_PF_PWM0
};

static int apf27_pwm_0_init(void)
{
	return mxc_gpio_setup_multiple_pins(mxc_pwm0_pins, ARRAY_SIZE(mxc_pwm0_pins), "PWM0");
}

static int apf27_pwm_0_exit(void)
{
	mxc_gpio_release_multiple_pins(mxc_pwm0_pins, ARRAY_SIZE(mxc_pwm0_pins));
	return 0;
}

static struct imx_pwm_platform_data apf27_pwm_0_data = {
	.init = apf27_pwm_0_init,
	.exit = apf27_pwm_0_exit,
};
#endif


/* APF27 has a Micron 128MiB 1,8V NAND flash, 16 bits width */
static struct mxc_nand_platform_data apf27_nand_board_info = {
	.width = 2,
	.hw_ecc = 1,
};


/* APF27 has an I2C EEPROM on I2C2 Bus */
#ifdef CONFIG_I2C
static int mxc_i2c1_pins[] = {
	PC5_PF_I2C2_SDA,
	PC6_PF_I2C2_SCL
};

static int apf27_i2c_1_init(struct platform_device *pdev)
{
	return mxc_gpio_setup_multiple_pins(mxc_i2c1_pins, ARRAY_SIZE(mxc_i2c1_pins),
						"I2C2");
}

static int apf27_i2c_1_exit(struct platform_device *pdev)
{
	mxc_gpio_release_multiple_pins(mxc_i2c1_pins, ARRAY_SIZE(mxc_i2c1_pins));
	return 0;
}

static struct imx_i2c_platform_data apf27_i2c_1_data = {
	.max_clk = 100000,
	.init = apf27_i2c_1_init,
	.exit = apf27_i2c_1_exit,
};

static struct i2c_board_info apf27_i2c_devices_bus1[] = {
	[0] = {
		.type = "24c02",
		.flags = 0,
		.addr = 0x50,
		.platform_data = NULL,
		.irq = 0
	},
};
#endif /* CONFIG_I2C */


/* APF27 has an RS232 debug port/console on UART1 with "on module" transceiver */
static int mxc_uart0_pins[] = {
	PE12_PF_UART1_TXD,
	PE13_PF_UART1_RXD
};

static int uart_mxc_port0_init(struct platform_device *pdev)
{
	return mxc_gpio_setup_multiple_pins(mxc_uart0_pins, ARRAY_SIZE(mxc_uart0_pins), "UART1");
}

static int uart_mxc_port0_exit(struct platform_device *pdev)
{
	mxc_gpio_release_multiple_pins(mxc_uart0_pins, ARRAY_SIZE(mxc_uart0_pins));

	return 0;
}

static struct imxuart_platform_data uart0_pdata = {
	.init = uart_mxc_port0_init,
	.exit = uart_mxc_port0_exit,
	.flags = 0,
};

/* APF27 has an "on module" transceiver for UART3 */
static int mxc_uart2_pins[] = {
	PE8_PF_UART3_TXD,
	PE9_PF_UART3_RXD,
#ifdef CONFIG_SERIAL_IMX_UART3_USE_RTSCTS
	PE10_PF_UART3_CTS,
	PE11_PF_UART3_RTS,
#endif
};

static int uart_mxc_port2_init(struct platform_device *pdev)
{
	return mxc_gpio_setup_multiple_pins(mxc_uart2_pins, ARRAY_SIZE(mxc_uart2_pins), "UART3");
}

static int uart_mxc_port2_exit(struct platform_device *pdev)
{
	mxc_gpio_release_multiple_pins(mxc_uart2_pins, ARRAY_SIZE(mxc_uart2_pins));

	return 0;
}

static struct imxuart_platform_data uart2_pdata = {
	.init = uart_mxc_port2_init,
	.exit = uart_mxc_port2_exit,
#ifdef CONFIG_SERIAL_IMX_UART3_USE_RTSCTS
	.flags = IMXUART_HAVE_RTSCTS,
#else
	.flags = 0,
#endif
};


/* APF27 has an onboard Ethernet PHY */
#ifdef CONFIG_FEC_OLD
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
	mxc_gpio_setup_multiple_pins(mxc_fec_pins, ARRAY_SIZE(mxc_fec_pins), "FEC");
}
#endif /* CONFIG_FEC_OLD */


/* APF27 has an onboard PHY (ISP1504) on OTG USB port */
static int mxc_usbotg_pins[] = {
	PE0_PF_USBOTG_NXT,
	PE1_PF_USBOTG_STP,
	PE2_PF_USBOTG_DIR,
	PE24_PF_USBOTG_CLK,
	PE25_PF_USBOTG_DATA7,
	PC7_PF_USBOTG_DATA5,
	PC8_PF_USBOTG_DATA6,
	PC9_PF_USBOTG_DATA0,
	PC10_PF_USBOTG_DATA2,
	PC11_PF_USBOTG_DATA1,
	PC12_PF_USBOTG_DATA4,
	PC13_PF_USBOTG_DATA3,
};

static int isp1504_check_presence(void __iomem *view)
{
	int vid, pid;

	vid = (ulpi_read(ISP1504_VID_HIGH, view) << 8) |
		 ulpi_read(ISP1504_VID_LOW, view);
	pid = (ulpi_read(ISP1504_PID_HIGH, view) << 8) |
		 ulpi_read(ISP1504_PID_LOW, view);

	pr_info("ULPI OTG Vendor ID 0x%x    Product ID 0x%x\n", vid, pid);
	if (vid != 0x4cc || pid != 0x1504) {
		pr_err("No ISP1504 found\n");
		return -1;
	}

	return 0;
}

static int isp1504_set_vbus_power(void __iomem *view, int on)
{
	int ret = 0;

	ret = isp1504_check_presence(view);
	if (ret)
		return ret;

	if (on) {
		/* force disconnection of the DM/DP signals. This
			avoid powering the connected device through these lines */
		ulpi_set(RESET, ISP1504_FCNCTL, view);
		mdelay(10); /*wait until the supply is really down*/
		ulpi_clear(RESET, ISP1504_FCNCTL, view);
		mdelay(1);

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

static int isp1504_set_device(void __iomem *view)
{
	int ret = 0;

	ret = isp1504_check_presence(view);
	if (ret)
		return ret;

	ret = ulpi_clear(0x06, ISP1504_OTGCTL, view);

	return ret;
}

static void isp1504_suspend(void __iomem *view)
{
	/* isp1504_set_vbus_power(view, 0); needed ? */
	ulpi_clear(SUSPENDM, ISP1504_FCNCTL, view);
}

static void isp1504_resume(void __iomem *view)
{
	/* uggly but the STP line must be set to 1 before the first PHY access
	   in order to wake up the ULPI clock of the PHY. */
	gpio_set_value(GPIO_PORTE | 1, 1);
	/* deassert STP (gpio mode) */
	mxc_gpio_mode(GPIO_PORTE | 1  | GPIO_OUT | GPIO_GPIO);
	mdelay(1);
	/* STP now managed by the USB host ctrl */
	mxc_gpio_mode(PE1_PF_USBOTG_STP);
}

#define UOG_USBCMD	0x140
#define UOG_ULPIVIEW	0x170
#define UOG_PORTSC1	0x184
#define USB_CTRL	0x600
#define PORTSC_PTS_MASK	(3 << 30)
#define PORTSC_PTS_ULPI (2 << 30)

static void apf27_usbotg_reset(void)
{
	uint32_t temp;

	temp = readl(IO_ADDRESS(OTG_BASE_ADDR) + UOG_USBCMD);
	temp |= 0x02;
	writel(temp, IO_ADDRESS(OTG_BASE_ADDR) + UOG_USBCMD);
	pr_debug("reset controller: 0x%08x\n", temp);
	mdelay(10);
}

static int otg_mode_host = 1;

static int __init apf27_otg_mode(char *options)
{
	if (!strcmp(options, "host"))
		otg_mode_host = 1;
	else if (!strcmp(options, "device"))
		otg_mode_host = 0;
	else {
		pr_info("otg_mode neither \"host\" nor \"device\". "
			"Defaulting to host\n");
	}

	return 0;
}
__setup("otg_mode=", apf27_otg_mode);

static int apf27_usbotg_init(struct platform_device *pdev)
{
	int ret;
	uint32_t temp;

	ret = mxc_gpio_setup_multiple_pins(mxc_usbotg_pins,
			ARRAY_SIZE(mxc_usbotg_pins), "USB OTG");
	if (ret)
		return ret;

	/* Tweak power/wake up handling */
	temp = readl(IO_ADDRESS(OTG_BASE_ADDR) + USB_CTRL);
	temp &= ~( (3 << 29) | 1 );
	temp |= (1 << 24) | (1 << 27) | (1 << 28);
	writel(temp, IO_ADDRESS(OTG_BASE_ADDR) + USB_CTRL);

	/* Select ULPI PHY */
	temp = readl(IO_ADDRESS(OTG_BASE_ADDR) + UOG_PORTSC1);
	temp = (temp & ~PORTSC_PTS_MASK) | PORTSC_PTS_ULPI;
	writel(temp, IO_ADDRESS(OTG_BASE_ADDR) + UOG_PORTSC1);
	mdelay(10);

	apf27_usbotg_reset();

	if (otg_mode_host) {
		ret = isp1504_set_vbus_power((void __iomem *)(IO_ADDRESS(OTG_BASE_ADDR + UOG_ULPIVIEW)), 1);
	} else {
		ret = isp1504_set_device((void __iomem *)(IO_ADDRESS(OTG_BASE_ADDR + UOG_ULPIVIEW)));
	}

	if (ret)
		mxc_gpio_release_multiple_pins(mxc_usbotg_pins,
			ARRAY_SIZE(mxc_usbotg_pins));

	return ret;
}

struct mxc_usb2_platform_data ehci0_pdata = {
	.init = apf27_usbotg_init,
};

static struct fsl_usb2_platform_data otg_device_pdata = {
	.init = apf27_usbotg_init,
	.operating_mode = FSL_USB2_DR_DEVICE,
	.phy_mode = FSL_USB2_PHY_ULPI,
};


/* APF27 frequency scaling stuff */
#if defined(CONFIG_CPU_FREQ_IMX27) || defined(CONFIG_CPU_FREQ_IMX27_MODULE)

static struct clk *mpll_main_0;
static struct clk *mpll_main_1;
static struct clk *cpu;

static int apf27_cpufreq_init(struct platform_device *pd)
{
	mpll_main_0 = clk_get(NULL, "mpll_main.0");
	mpll_main_1 = clk_get(NULL, "mpll_main.1");
	cpu = clk_get(NULL, "cpu_clk");

	if (IS_ERR(mpll_main_0) || IS_ERR(mpll_main_1) || IS_ERR(cpu)) {
		printk("Cant get one of the required clocks\n");
		/* FIXME error handling */
	}

	/* Switch Qvdd (=CPU core supply) to 1.45V (voltage for max core speed) */
	/* The APF27 don't have a PMIC, so 1.45V is already here */

	if (clk_get_rate(mpll_main_0) < 399000000) {
		printk(KERN_INFO "Found MPLL running below ~400Mhz. Tuning "
				"right now.\n");
		/* here we should setup max frequencies, if not already setup by
		   the bootloader, but in our case we fully trust our U-Boot ;-) */
	}

	return 0;
}

static int apf27_cpufreq_exit(struct platform_device *pd)
{
	clk_put(mpll_main_0);
	clk_put(mpll_main_1);
	clk_put(cpu);

	return 0;
}

static int apf27_cpufreq_transit(unsigned long freq)
{
	uint32_t cscr;
	struct clk *new_clk;

	cscr = __raw_readl(CCM_CSCR);

	switch(freq) {
	case 133000:
		/* AHB *can* run at 133MHz */
		cscr &= ~CCM_CSCR_AHB_MASK;
		cscr |= 0x01 << CCM_CSCR_AHB_OFFSET;	/* AHBDIV = 2 */

		cscr &= ~CCM_CSCR_ARM_MASK;
		cscr |= 0x01 << CCM_CSCR_ARM_OFFSET;	/* 133MHz */
		cscr &= ~CCM_CSCR_ARM_SRC;
		new_clk = mpll_main_1;
		break;

	case 266000:
		/* AHB *can* run at 133MHz */
		cscr &= ~CCM_CSCR_AHB_MASK;
		cscr |= 0x01 << CCM_CSCR_AHB_OFFSET;	/* AHBDIV = 2 */

		cscr &= ~CCM_CSCR_ARM_MASK;
		cscr |= 0x00 << CCM_CSCR_ARM_OFFSET;	/* 266MHz speed */
		cscr &= ~CCM_CSCR_ARM_SRC;
		new_clk = mpll_main_1;
		break;

	case 400000:
		/* AHB *must* run at 133MHz */
		cscr &= ~CCM_CSCR_AHB_MASK;
		cscr |= 0x01 << CCM_CSCR_AHB_OFFSET;	/* AHBDIV = 2 */

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

static struct cpufreq_frequency_table apf27_freq_table[] = {
	{0x01, .frequency = 133000 }, /* with 400MHz MPLL * 2 / (3*2) @ 1.45V Qvdd */
	{0x02, .frequency = 266000 }, /* with 400MHz MPLL * 2 / (3*1) @ 1.45V Qvdd */
	{0x03, .frequency = 400000 }, /* with 400MHz MPLL * 2 / (2*1) @ 1.45V Qvdd */
	{0, CPUFREQ_TABLE_END}
};

static struct cpufreq_platform_data apf27_cpufreq_platform_data = {
	.freq_table = apf27_freq_table,
	.freq_entries = ARRAY_SIZE(apf27_freq_table),
	.init = apf27_cpufreq_init,
	.exit = apf27_cpufreq_exit,
	.transit = apf27_cpufreq_transit
};

static struct platform_device apf27_cpu_frequency_device = {
	.name = "imx_cpufreq",
	.id = 0,
	.dev = {
		.platform_data = &apf27_cpufreq_platform_data,
	}
};
# define CPU_FREQUENCY &apf27_cpu_frequency_device,
#else
# define CPU_FREQUENCY
#endif /* CONFIG_CPU_FREQ_IMX27 */


static struct platform_device *platform_devices[] __initdata = {
	CPU_FREQUENCY
#ifdef CONFIG_MXC_VPU
	&mxc_vpu_device,
#endif
};


/* APF27 power management stuff */

static void apf27_pwr_ctl_active(void)
{
#ifdef CONFIG_FB_MXC
	apf27_lcd_startup();
#endif
}

#ifdef CONFIG_PM

#ifdef APF27_PM_DEBUG
#define CCM_PCCR0               (IO_ADDRESS(CCM_BASE_ADDR) + 0x20)
#define CCM_PCCR1               (IO_ADDRESS(CCM_BASE_ADDR) + 0x24)

static void apf27_dump_clocks(void)
{
	printk("CSCR:   0x%08x\n", __raw_readl(CCM_CSCR));
	printk("MPCTL0: 0x%08x\n", __raw_readl(CCM_MPCTL0));
	printk("MPCTL1: 0x%08x\n", __raw_readl(CCM_MPCTL1));
	printk("SPCTL0: 0x%08x\n", __raw_readl(CCM_SPCTL0));
	printk("SPCTL1: 0x%08x\n", __raw_readl(CCM_SPCTL1));
	printk("OSC26:  0x%08x\n", __raw_readl(CCM_OSC26MCTL));
	printk("PCDR0:  0x%08x\n", __raw_readl(CCM_PCDR0));
	printk("PCDR1:  0x%08x\n", __raw_readl(CCM_PCDR1));
	printk("PCCR0:  0x%08x\n", __raw_readl(CCM_PCCR0));
	printk("PCCR1:  0x%08x\n", __raw_readl(CCM_PCCR1));
	printk("CCSR:   0x%08x\n", __raw_readl(CCM_CCSR));
}

char port[6] = {'A', 'B', 'C', 'D', 'E', 'F'};

static void dump_gpio_port(int id)
{
	u32 reg;

	reg =  __raw_readl(VA_GPIO_BASE + MXC_DDIR(id));
	printk("DDIR_%c: 0x%08x\n", port[id], reg);
	reg =  __raw_readl(VA_GPIO_BASE + MXC_DR(id));
	printk("DR_%c: 0x%08x\n", port[id], reg);
	reg =  __raw_readl(VA_GPIO_BASE + MXC_GIUS(id));
	printk("GIUS_%c: 0x%08x\n", port[id], reg);
	reg =  __raw_readl(VA_GPIO_BASE + MXC_OCR1(id));
	printk("OCR1_%c: 0x%08x\n", port[id], reg);
	reg =  __raw_readl(VA_GPIO_BASE + MXC_OCR2(id));
	printk("OCR2_%c: 0x%08x\n", port[id], reg);
	reg =  __raw_readl(VA_GPIO_BASE + MXC_PUEN(id));
	printk("PUEN_%c: 0x%08x\n", port[id], reg);
	printk("---\n");
}

static void dump_gpios(void)
{
	int i;

	printk("*************************\n");
	for (i=0; i<6; i++) {
		dump_gpio_port(i);
	}
}

/*	    DDIR         OCR1        OCR2        DR         GIUS        PUEN */
u32 gpio_val[6][6] = {
	{0x00000000, 0x00000000, 0xF000FFFF, 0x00000000, 0xFFFFFFFF, 0xFFFFFFFF}, /* A */
	{0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xFFFFFFF3, 0xFFFFFFFF}, /* B */
	{0x000003F8, 0x3FFFC000, 0x00000000, 0x00000000, 0xFFFFFFFF, 0xFFFFFC07}, /* C */
	{0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xFFFFFFFF, 0xFFFFFFFF}, /* D */
	{0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xFFFFCCFF, 0xFFFFFFFF}, /* E */
	{0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xFFFFFFFF, 0xFFFFFFFF}, /* F */
};

static void force_gpio_port(int id)
{
	__raw_writel(gpio_val[id][0], VA_GPIO_BASE + MXC_DDIR(id));
	__raw_writel(gpio_val[id][1], VA_GPIO_BASE + MXC_OCR1(id));
	__raw_writel(gpio_val[id][2], VA_GPIO_BASE + MXC_OCR2(id));
	__raw_writel(gpio_val[id][3], VA_GPIO_BASE + MXC_DR(id));
	__raw_writel(gpio_val[id][4], VA_GPIO_BASE + MXC_GIUS(id));
	__raw_writel(gpio_val[id][5], VA_GPIO_BASE + MXC_PUEN(id));
}

static void apf27_force_gpios(void)
{
	force_gpio_port(0);
	force_gpio_port(1);
	force_gpio_port(3);
}
#endif /* APF27_PM_DEBUG */

static int apf27_pm_suspend(struct sys_device *sd, pm_message_t state)
{
	printk("___ %s\n", __func__);

#ifdef CONFIG_USB_EHCI_MXC
	isp1504_suspend((void __iomem *)(IO_ADDRESS(OTG_BASE_ADDR + 0x170)));
#endif
	apf27_baseboard_suspend();
#ifdef APF27_PM_DEBUG
	apf27_force_gpios();
	apf27_dump_clocks();
	dump_gpios();
#endif
	__raw_writel(0x0c090c0A, IO_ADDRESS(SYSCTRL_BASE_ADDR + 0x1c)); /* WBCR */

	return 0;
}

static int apf27_pm_resume(struct sys_device *sd)
{
	printk("___ %s\n", __func__);

#ifdef CONFIG_USB_EHCI_MXC
	isp1504_resume((void __iomem *)(IO_ADDRESS(OTG_BASE_ADDR + 0x170)));
#endif
	apf27_baseboard_resume();

	return 0;
}

#else
# define apf27_pm_suspend NULL
# define apf27_pm_resume NULL
#endif /* CONFIG_PM */

static struct sysdev_class apf27_pm_sysclass = {
	.name		= "mach-apf27",
	.suspend	= apf27_pm_suspend,
	.resume		= apf27_pm_resume,
};

static struct sys_device apf27_pm_sysdev = {
	.cls		= &apf27_pm_sysclass,
};


static void __init apf27_init(void)
{
	sysdev_class_register(&apf27_pm_sysclass);
	sysdev_register(&apf27_pm_sysdev);

	/* Init apf27 power management related pins */
	apf27_pwr_ctl_active();

#ifdef CONFIG_FEC_OLD
	gpio_fec_active();
#endif
	/* UART1 and UART3 have "on module" transceivers */
	mxc_register_device(&mxc_uart_device0, &uart0_pdata); /* console */
	mxc_register_device(&mxc_uart_device2, &uart2_pdata);

	mxc_register_device(&mxc_nand_device, &apf27_nand_board_info);

#ifdef CONFIG_PWM
	mxc_register_device(&mxc_pwm_device0, &apf27_pwm_0_data);
#endif
#ifdef CONFIG_WATCHDOG
	mxc_register_device(&mxc_wdt, NULL);
#endif
	if (otg_mode_host) {
		mxc_register_device(&mxc_otg_host, &ehci0_pdata);
	} else {
		mxc_register_device(&mxc_otg_udc_device, &otg_device_pdata);
	}
#ifdef CONFIG_I2C
	mxc_register_device(&imx_i2c_device1, &apf27_i2c_1_data);
	i2c_register_board_info(1, apf27_i2c_devices_bus1,
				ARRAY_SIZE(apf27_i2c_devices_bus1));
#endif
	platform_add_devices(platform_devices, ARRAY_SIZE(platform_devices));

	printk("i.MX27 chip revision: %d\n", mx27_revision() );

	apf27_baseboard_init();
}

static void __init apf27_timer_init(void)
{
	mxc_clocks_init(26000000); /* ext ref even if not used */
	mxc_timer_init("gpt_clk.0");
}

struct sys_timer apf27_timer = {
	.init = apf27_timer_init,
};

void __init apf27_map_io(void)
{
	mxc_map_io();
}

MACHINE_START(APF27, "Armadeus APF27")
	/* Maintainer: Julien Boibessot <julien.boibessot@armadeus.com> */
	.phys_io        = AIPI_BASE_ADDR,
	.io_pg_offst    = ((AIPI_BASE_ADDR_VIRT) >> 18) & 0xfffc,
	.boot_params    = PHYS_OFFSET + 0x100,
	.map_io         = apf27_map_io,
	.init_irq       = mxc_init_irq,
	.init_machine   = apf27_init,
	.timer          = &apf27_timer,
MACHINE_END
