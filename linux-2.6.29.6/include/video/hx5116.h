/*
 *
 * HX5116 160CH Single Chip Driver for LPTS AMOLED
 *
 * Copyright 2009 ARMadeus Systems
 *  Fabien Marteau <fabien.marteau@armadeus.com>
 *
 *  http://www.armadeus.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define HX5116_REG(x)	(x)

/***********************/
/* Standards registers */
/***********************/
#define HX5116_INDEX			HX5116_REG(0x00)

#define HX5116_CHIP_ID0x		HX5116_REG(0x01)
#define HX5116_OTP_ENABLE		HX5116_REG(0x02)

#define HX5116_DISPLAY_MODE1		HX5116_REG(0x03)
#define HX5116_DISPLAY_MODE2		HX5116_REG(0x04)
#define HX5116_DISPLAY_MODE3		HX5116_REG(0x05)

#define HX5116_DRIVER_CAPABILITY	HX5116_REG(0x07)

#define HX5116_POWER_CTRL1		HX5116_REG(0x06)
#define HX5116_POWER_CTRL2		HX5116_REG(0x08)
#define HX5116_POWER_CTRL3		HX5116_REG(0x09)
#define HX5116_POWER_CTRL4		HX5116_REG(0x0a)
#define HX5116_POWER_CTRL5		HX5116_REG(0x0b)
#define HX5116_POWER_CTRL6		HX5116_REG(0x0c)

#define HX5116_POWER_SEQ1		HX5116_REG(0x0d)
#define HX5116_POWER_SEQ2		HX5116_REG(0x0e)

#define HX5116_R_SLOPE			HX5116_REG(0x10)
#define HX5116_G_SLOPE			HX5116_REG(0x11)
#define HX5116_B_SLOPE			HX5116_REG(0x12)

#define HX5116_R_GAMMA0			HX5116_REG(0x13)
#define HX5116_R_GAMMA10		HX5116_REG(0x14)
#define HX5116_R_GAMMA36		HX5116_REG(0x15)
#define HX5116_R_GAMMA80		HX5116_REG(0x16)
#define HX5116_R_GAMMA124		HX5116_REG(0x17)
#define HX5116_R_GAMMA168		HX5116_REG(0x18)
#define HX5116_R_GAMMA212		HX5116_REG(0x19)
#define HX5116_R_GAMMA255		HX5116_REG(0x1a)
#define HX5116_G_GAMMA0			HX5116_REG(0x1b)
#define HX5116_G_GAMMA10		HX5116_REG(0x1c)
#define HX5116_G_GAMMA36		HX5116_REG(0x1d)
#define HX5116_G_GAMMA80		HX5116_REG(0x1e)
#define HX5116_G_GAMMA124		HX5116_REG(0x1f)
#define HX5116_G_GAMMA168		HX5116_REG(0x20)
#define HX5116_G_GAMMA212		HX5116_REG(0x21)
#define HX5116_G_GAMMA255		HX5116_REG(0x22)
#define HX5116_B_GAMMA0			HX5116_REG(0x23)
#define HX5116_B_GAMMA10		HX5116_REG(0x24)
#define HX5116_B_GAMMA36		HX5116_REG(0x25)
#define HX5116_B_GAMMA80		HX5116_REG(0x26)
#define HX5116_B_GAMMA124		HX5116_REG(0x27)
#define HX5116_B_GAMMA168		HX5116_REG(0x28)
#define HX5116_B_GAMMA212		HX5116_REG(0x29)
#define HX5116_B_GAMMA255		HX5116_REG(0x2a)

#define HX5116_T3			HX5116_REG(0x34)
#define HX5116_T4			HX5116_REG(0x35)
#define HX5116_TF			HX5116_REG(0x36)
#define HX5116_TB			HX5116_REG(0x37)

#define HX5116_VSTS			HX5116_REG(0x38)
#define HX5116_HSTS			HX5116_REG(0x39)

#define HX5116_RGB_CONTRAST		HX5116_REG(0x3a)
#define HX5116_R_CONTRAST		HX5116_REG(0x3b)
#define HX5116_G_CONTRAST		HX5116_REG(0x3c)
#define HX5116_B_CONTRAST		HX5116_REG(0x3d)

#define HX5116_BRIGHT_OFFSET		HX5116_REG(0x3e)

/*****************/
/* OTP registers */
/*****************/
#define HX5116_OTP_R_SLOPE		HX5116_REG(0x00)
#define HX5116_OTP_G_SLOPE		HX5116_REG(0x01)
#define HX5116_OTP_B_SLOPE		HX5116_REG(0x02)

#define HX5116_OTP_VGAM1_LEVEL		HX5116_REG(0x03)

/*****************************/
/* SPI interface definitions */
/*****************************/
#define HX5116_SPI_IDCODE		(0x70)
#define HX5116_SPI_ID(x)		((x) << 2)
#define HX5116_SPI_READ			(0x01)
#define HX5116_SPI_WRITE		(0x00)
#define HX5116_SPI_DATA			(0x02)
#define HX5116_SPI_INDEX		(0x00)

/* platform data to pass configuration from lcd */
struct hx5116_display {
	struct  display_device *display_dev;
	struct  spi_device *spi;  /* spi bus */
	void	(*reset_on)(int); /* function that let hx5116 on reset */
	int	gcontrast_value;  /* current contrast value */
};


