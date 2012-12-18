/*
 * Eric Jarrige <jorasse@users.sourceforge.net>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 *
 */
#ifndef _i2c_h_
#define _i2c_h_

#include <asm/arch/imx-regs.h>

/*
 * I2C module
 */
extern unsigned long i2c_bases[];

#define IADR(x)   __REG(i2c_bases[x] + 0x000) /* I2C Address Register */
#define IFDR(x)   __REG(i2c_bases[x] + 0x004) /* I2C Frequency Divider Register*/
#define I2CR(x)   __REG(i2c_bases[x] + 0x008) /* I2C Control Register */
#define I2SR(x)   __REG(i2c_bases[x] + 0x00C) /* I2C Status Register */
#define I2DR(x)   __REG(i2c_bases[x] + 0x010) /* I2C Data I/O Register */
/* I2C Control Register Bit Fields */
#define I2CR_IEN 	(1<<7)		/* I2C Enable */
#define I2CR_IIEN 	(1<<6)		/* I2C Interrupt Enable */
#define I2CR_MSTA 	(1<<5)		/* I2C Master/Slave Mode Select */
#define I2CR_MTX 	(1<<4)		/* I2C Transmit/Receive Mode Select */
#define I2CR_TXAK 	(1<<3)		/* I2C Transmit Acknowledge Enable */
#define I2CR_RSTA 	(1<<2)		/* I2C Repeated START */
#define I2SR_ICF 	(1<<7)		/* I2C Data Transfer */
#define I2SR_IAAS 	(1<<6)		/* I2C Addressed As a Slave */
#define I2SR_IBB 	(1<<5)		/* I2C Bus Busy */
#define I2SR_IAL 	(1<<4)		/* I2C Arbitration Lost */
#define I2SR_SRW 	(1<<2)		/* I2C Slave Read/Write	*/
#define I2SR_IIF 	(1<<1)		/* I2C interrupt */
#define I2SR_RXAK 	(1<<0)		/* I2C Received Acknowledge */


#endif /* _i2c_h_ */
