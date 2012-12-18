/*
 *
 * Armadeus LCD configuration file
 *
 * Put here all that is needed to configure the Hardware
 * interface to your LCD
 *
 *
 */

#ifndef APF_LCD_CONFIG_H
#define APF_LCD_CONFIG_H


#ifdef CONFIG_FB_IMX

#include <mach/imxfb.h>
#include <linux/delay.h>
#include <mach/gpio.h>

#ifdef CONFIG_ARCH_IMX
#define LCDC_BASE_ADDR	IMX_LCDC_BASE
#endif
#define LCDISR_EOF	(1 << 1)

#ifdef CONFIG_MACH_APF9328
#define OPTREX_F51900_POWER_DOWN	(GPIO_PORTD | 7)	/* PD7_REV */
#define SHARP_LQ043_POWER_DOWN		(GPIO_PORTD | 12)	/* PD12_ACD_OE */
#define CONTRAST_LINE			(GPIO_PORTD | 11)	/* PD11_CONTRAST */
#endif

#ifdef CONFIG_MACH_APF27
#define OPTREX_F51900_POWER_DOWN	(GPIO_PORTA | 24)	/* PA24_REV */
#define SHARP_LQ043_POWER_DOWN		(GPIO_PORTA | 31)	/* PA31_OE_ACD */
#define CONTRAST_LINE			(GPIO_PORTA | 30)	/* PA30_CONTRAST */
#endif

#define DEFAULT_DMA_SETTINGS        (DMACR_BURST | DMACR_HM(8) | DMACR_TM(4))

#ifdef CONFIG_MACH_APF9328
static int apf9328_fb_init(struct platform_device *pdev);
static int apf9328_fb_exit(struct platform_device *pdev);
#endif

/*
 * Power on/off LCD's internal logic
 */
static void apf_lcd_power(int on)
{
	u32 isr;

	pr_debug("%s: %s\n", __func__, on ? "on":"off");
	isr = 0;
#ifdef CONFIG_FB_IMX_OPTREX_F51900_CSTN
	if (on)
		gpio_set_value(OPTREX_F51900_POWER_DOWN, 1);
	else
		gpio_set_value(OPTREX_F51900_POWER_DOWN, 0);
#elif defined(CONFIG_FB_IMX_MICROTIPS_MTF_T035_TFT)
	/* No LCD powerdown yet */
#elif defined(CONFIG_FB_IMX_SHARP_LQ043_TFT)
	if (on) {
		mdelay(200); /* at least ten frames have to be processed before
				enabling the display */
		/* Waits end of current frame */
		isr = readl(IO_ADDRESS(LCDC_BASE_ADDR) + 0x40);
		do {
			isr = readl(IO_ADDRESS(LCDC_BASE_ADDR) + 0x40);
			udelay(1000);
			pr_debug("%08x\n", isr);
		} while ((isr & LCDISR_EOF) == 0);

		gpio_set_value(SHARP_LQ043_POWER_DOWN, 1);
		mdelay(200);
	} else {
		gpio_set_value(SHARP_LQ043_POWER_DOWN, 0);
		mdelay(200); /* needs to wait 10 frames after DISP goes down
				before shutting down LCDC (done in imxfb) */
	}
#else
	if (on) {
		gpio_set_value(CONTRAST_LINE, 0);
	} else {
		gpio_set_value(CONTRAST_LINE, 1);
	}
#endif /* CONFIG_FB_IMX_OPTREX_F51900_CSTN */
}

/*
 * Power on/off LCD's backlight
 */
static void apf_lcd_backlight_power(int on)
{
	u32 pwmr;

	pr_debug("%s: %s\n", __func__, on ? "on":"off");

	pwmr = readl(IO_ADDRESS(LCDC_BASE_ADDR) + 0x2c);
#ifdef CONFIG_MACH_APF9328
	if (pwmr)
		imx_gpio_mode(PD11_PF_CONTRAST);
#endif
	if (on) {
		pwmr |= PWMR_CC_EN;
	} else {
		pwmr &= ~PWMR_CC_EN;
	}
	writel(pwmr, IO_ADDRESS(LCDC_BASE_ADDR) + 0x2c);
}

#ifdef CONFIG_MACH_APF9328
static struct imx_fb_platform_data apf9328_fb_info /*__initdata*/ = {
#elif CONFIG_MACH_APF27
static struct imx_fb_platform_data apf27_fb_data = {
#endif
#ifdef CONFIG_FB_IMX_SHARP_LQ043_TFT
	.pixclock	= 125000, /* picoS -> ~8MHz */
	.bpp		= 16,
	.xres		= 480,
	.yres		= 272,

	.hsync_len	= 41,
	.vsync_len	= 10,
	.left_margin	= 3, /* should be 2 but i.MX doesn't support it */
	.upper_margin	= 3,
	.right_margin	= 2,
	.lower_margin	= 2,

	.pcr		= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 | PCR_BPIX_16 | PCR_FLMPOL | PCR_LPPOL |
				PCR_CLKPOL | PCR_SCLKIDLE | PCR_SCLK_SEL | PCR_PCD(5),
	.pwmr		= 0x000003ff,    /* Contrast with PWM @ Pixel clock / 256, max width by default */
	.dmacr		= DEFAULT_DMA_SETTINGS,
	.backlight_power= apf_lcd_backlight_power,
	.lcd_power	= apf_lcd_power,
#elif CONFIG_FB_IMX_MICROTIPS_MTF_T035_TFT
	.pixclock	= 156250, /* picoS -> ~6.4MHz */
	.bpp		= 16,
	.xres		= 320,
	.yres		= 240,

	.hsync_len	= 31,
	.vsync_len	= 3,
	.left_margin	= 4,
	.upper_margin	= 20,
	.right_margin	= 69,
	.lower_margin	= 20,

	.pcr		= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 | PCR_BPIX_16 | PCR_FLMPOL | PCR_LPPOL |
				PCR_SCLKIDLE | PCR_SCLK_SEL | PCR_PCD(7),
	.pwmr		= 0x00000150,    /* Contrast with PWM @ Line Pulse / 256, medium width by default */
	.dmacr		= DEFAULT_DMA_SETTINGS,
	.backlight_power= apf_lcd_backlight_power,
	.lcd_power	= apf_lcd_power,
#elif CONFIG_FB_IMX_SHARP_LQ057_TFT
	.pixclock	= 158730, /* picoS -> ~6.3MHz */
	.bpp		= 16,
	.xres		= 320,
	.yres		= 240,

	.hsync_len	= 9,
	.vsync_len	= 2,
	.left_margin	= 9,
	.upper_margin	= 0,
	.right_margin	= 27,
	.lower_margin	= 7,

	.pcr		= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 | PCR_BPIX_16 | /*PCR_CLKPOL |*/
				PCR_SCLKIDLE | PCR_SCLK_SEL | PCR_PCD(9),
	.pwmr		= 0,    /* No contrast management */
	.dmacr		= DEFAULT_DMA_SETTINGS,
	.lcd_power	= apf_lcd_power,
#elif CONFIG_FB_IMX_OPTREX_F51900_CSTN
	.pixclock	= 62500,
	.bpp		= 8,
	.xres		= 320,
	.yres		= 240,

	.hsync_len	= 2,
	.vsync_len	= 2,
	.left_margin	= 2,
	.upper_margin	= 2,
	.right_margin	= 2,
	.lower_margin	= 2,

	.pcr		= PCR_COLOR | PCR_PBSIZ_8 | PCR_BPIX_8 | PCR_ACD(5) |
				PCR_END_BYTE_SWAP | PCR_PCD(3),
	.dmacr		= DEFAULT_DMA_SETTINGS,
	.pwmr		= 0,    /* No contrast management */
	.lcd_power	= apf_lcd_power,
#elif CONFIG_FB_IMX_MOTOROLA_A910_TFT
	.pixclock	= 158730, /* picoS -> ~6.3MHz */
	.bpp		= 16,
	.xres		= 240,
	.yres		= 320,

	.hsync_len	= 9,
	.vsync_len	= 2,
	.left_margin	= 9,
	.upper_margin	= 2,
	.right_margin	= 27,
	.lower_margin	= 4,

	.pcr		= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 | PCR_BPIX_16 | PCR_CLKPOL |
				PCR_SCLKIDLE | PCR_SCLK_SEL | PCR_PCD(9),
	.dmacr		= DEFAULT_DMA_SETTINGS,
	.pwmr		= 0,    /* No contrast management */
	.lcd_power	= apf_lcd_power,
#else
#error Please define a imx_fb_platform_data structure with your LCD parameters
#endif
#ifdef CONFIG_MACH_APF9328
	.init		= apf9328_fb_init,
	.exit		= apf9328_fb_exit,
#elif CONFIG_MACH_APF27
	.init		= apf27_fb_init,
	.exit		= apf27_fb_exit,
#endif
};

/*
 * Configure all GPIOs needed by LCDs
 */
#ifdef CONFIG_MACH_APF9328
static int apf9328_fb_init(struct platform_device *pdev)
{
	struct imx_fb_platform_data *fb_inf = &apf9328_fb_info;
	int width;

	pr_debug("%s\n", __func__);

#ifdef CONFIG_FB_IMX_OPTREX_F51900_CSTN
	gpio_set_value(OPTREX_F51900_POWER_DOWN, 1); /* Initializes it High */
	imx_gpio_mode(OPTREX_F51900_POWER_DOWN | GPIO_OUT | GPIO_DR);
#endif
#ifdef CONFIG_FB_IMX_SHARP_LQ043_TFT
	/* ACD_OE (SHARP_LQ043_POWER_DOWN) used as power down signal */
	gpio_set_value(SHARP_LQ043_POWER_DOWN, 0); /* Initializes it Low */
	imx_gpio_mode(SHARP_LQ043_POWER_DOWN | GPIO_OUT | GPIO_GIUS | GPIO_DR);
#else
	/* otherwise use ACD_OE as standard LCD controller signal */
	imx_gpio_mode(PD12_PF_ACD_OE);
#endif

	if (fb_inf->pcr & PCR_TFT)
		width = 16;
	else
		width = 1 << ((fb_inf->pcr >> 28) & 0x3);

	switch (width) {
	case 16:
		imx_gpio_mode(PD30_PF_LD15);
		imx_gpio_mode(PD29_PF_LD14);
		imx_gpio_mode(PD28_PF_LD13);
		imx_gpio_mode(PD27_PF_LD12);
		imx_gpio_mode(PD26_PF_LD11);
		imx_gpio_mode(PD25_PF_LD10);
		imx_gpio_mode(PD24_PF_LD9);
		imx_gpio_mode(PD23_PF_LD8);
	case 8:
		imx_gpio_mode(PD22_PF_LD7);
		imx_gpio_mode(PD21_PF_LD6);
		imx_gpio_mode(PD20_PF_LD5);
		imx_gpio_mode(PD19_PF_LD4);
	case 4:
		imx_gpio_mode(PD18_PF_LD3);
		imx_gpio_mode(PD17_PF_LD2);
	case 2:
		imx_gpio_mode(PD16_PF_LD1);
	case 1:
		imx_gpio_mode(PD15_PF_LD0);
	}

	imx_gpio_mode(PD6_PF_LSCLK);
	imx_gpio_mode(PD14_PF_FLM_VSYNC);
	imx_gpio_mode(PD13_PF_LP_HSYNC);

	/* Sharp's HR TFT displays specific */
	if (fb_inf->pcr & PCR_SHARP) {
		imx_gpio_mode(PD7_PF_REV);
		imx_gpio_mode(PD8_PF_CLS);
		imx_gpio_mode(PD9_PF_PS);
		imx_gpio_mode(PD10_PF_SPL_SPR);
	}

	gpio_set_value(CONTRAST_LINE, 0); /* Initializes it Low */
	/* GPIO Function for CONTRAST pin */
	imx_gpio_mode(CONTRAST_LINE | GPIO_OUT | GPIO_GIUS | GPIO_DR);

	return 0;
}

static int apf9328_fb_exit(struct platform_device *pdev)
{
	/* TO BE DONE */
	return 0;
}
#endif /* CONFIG_MACH_APF9328 */

#endif /* CONFIG_FB_IMX */

#endif /* APF_LCD_CONFIG_H */
