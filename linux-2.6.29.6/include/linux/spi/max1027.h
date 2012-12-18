/*
 * include/linux/spi/max1027.h
 *
 * 8x10bits ADC
 *
 * Copyright (c) 2008 Armadeus Systems
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef __LINUX_SPI_MAX1027_H
#define __LINUX_SPI_MAX1027_H

/* 00 : Scans channels 0 through N.
 * 01 : Scans channels N through the highest numbered channel.
 * 10 : Scans channel N repeatedly. The averaging register sets the number of results
 * 11 : No scan. Converts channel N once only. */
#define SCAN_MODE_00				(0)
#define SCAN_MODE_01				(1)
#define SCAN_MODE_10				(2)
#define SCAN_MODE_11				(3)

#define MAX1027_CONV                (0x80)
#define MAX1027_CONV_CHSEL(x) 		((x&0x0f)<<3)
#define MAX1027_CONV_SCAN(x)  		((x&0x03)<<1)
#define MAX1027_CONV_TEMP  			0x01
#define GET_SCAN_MODE(conv)			( (conv & 0x06) >> 1 )

#define MAX1027_SETUP               (0x40)
#define MAX1027_SETUP_CLKSEL(x) 	((x&0x03)<<4)
#define MAX1027_SETUP_REFSEL(x) 	((x&0x03)<<2)
#define MAX1027_SETUP_DIFFSEL(x)  	(x&0x03)

#define MAX1027_AVG                 (0x20)
#define MAX1027_AVG_AVGON(x)  		((x&0x01)<<4)
#define MAX1027_AVG_NAVG(x)   		((x&0x03)<<2)
#define MAX1027_AVG_NSCAN(x)  		(x&0x03)

/* All channels and temperature are scanned per conversion */
#define MAX1027_CONV_DEFAULT 		(MAX1027_CONV_CHSEL(6) | \
									MAX1027_CONV_SCAN(0) | MAX1027_CONV_TEMP)
/* Internal clock and reference stays on */
#define MAX1027_SETUP_DEFAULT 		(MAX1027_SETUP_CLKSEL(0) | \
										MAX1027_SETUP_REFSEL(2))
/* Averaging Off */
#define MAX1027_AVG_DEFAULT 		( MAX1027_AVG_AVGON(0) )

struct spi_device;
struct max1027_config {
	u8 conv;	/* initial conversion register value*/
	u8 setup;	/* initial setup register value */
	u8 avg;	    /* initial average register value */
    int cnvst_pin; /*gpio to start conversion. -1 for software start */
	int (*init)(struct spi_device *spi);
	int (*exit)(struct spi_device *spi);
};


#endif	/* __LINUX_SPI_MAX1027_H */
