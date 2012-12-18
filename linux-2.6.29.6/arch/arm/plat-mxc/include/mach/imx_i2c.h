/*
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Juergen Beisert, kernel@pengutronix.de
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

#ifndef __ASM_ARCH_MXC_I2C_H__
#define __ASM_ARCH_MXC_I2C_H__

struct imx_i2c_platform_data {
	u32 max_clk;
	int (*init)(struct platform_device *pdev);
	int (*exit)(struct platform_device *pdev);
};

/*
 * This file contains the I2C chip level configuration details.
 *
 * It also contains the API function that other drivers can use to read/write
 * to the I2C device registers.
 */

/*
 * This defines the string used to identify MXC I2C Bus drivers
 */
#define MXC_ADAPTER_NAME        "MXC I2C Adapter"

#define MXC_I2C_FLAG_READ	0x01	/* if set, is read; else is write */
#define MXC_I2C_FLAG_POLLING	0x02	/* if set, is polling mode; else is interrupt mode */

int mxc_i2c_read(int bus_id, unsigned int addr, char *reg, int reg_len,
		 char *buf, int num);

int mxc_i2c_write(int bus_id, unsigned int addr, char *reg, int reg_len,
		  char *buf, int num);

int mxc_i2c_polling_read(int bus_id, unsigned int addr, char *reg, int reg_len,
			 char *buf, int num);

int mxc_i2c_polling_write(int bus_id, unsigned int addr, char *reg, int reg_len,
			  char *buf, int num);

/* FIXME: This should be in a generic register file */

/* Address offsets of the I2C registers */
#define MXC_IADR                0x00	/* Address Register */
#define MXC_IFDR                0x04	/* Freq div register */
#define MXC_I2CR                0x08	/* Control regsiter */
#define MXC_I2SR                0x0C	/* Status register */
#define MXC_I2DR                0x10	/* Data I/O register */

/* Bit definitions of I2CR */
#define MXC_I2CR_IEN            0x0080
#define MXC_I2CR_IIEN           0x0040
#define MXC_I2CR_MSTA           0x0020
#define MXC_I2CR_MTX            0x0010
#define MXC_I2CR_TXAK           0x0008
#define MXC_I2CR_RSTA           0x0004

/* Bit definitions of I2SR */
#define MXC_I2SR_ICF            0x0080
#define MXC_I2SR_IAAS           0x0040
#define MXC_I2SR_IBB            0x0020
#define MXC_I2SR_IAL            0x0010
#define MXC_I2SR_SRW            0x0004
#define MXC_I2SR_IIF            0x0002
#define MXC_I2SR_RXAK           0x0001

#endif /* __ASM_ARCH_MXC_I2C_H__ */
