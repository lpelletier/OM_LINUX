/*
 *	Copyright (C) 2002 Motorola GSG-China
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version 2
 *	of the License, or (at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program; if not, write to the Free Software
 *	Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307,
 *      USA.
 *
 * File Name: i2c-imx.h
 *
 * Author:  Torsten Koschorrek, synertronixx GmbH
 *
 * Desc.:   Implementation of I2C Adapter/Algorithm Driver
 *
 *          Derived from Motorola GSG China I2C example driver
 *
 *          Copyright (C) 2002 Motorola GSG-China
 *          Copyright (C) 2005 Torsten Koschorrek <koschorrek at synertronixx.de>
 *          Portions:
 *          Copyright (C) 2005 Matthias Blaschke <blaschke at synertronixx.de>
 *
 * History: 2002/2/07 use msgs[]
 *                    Motorola GSG-China
 *          2004/3/03 port to linux kernel v2.6.x for i.MX
 *                    adding ioctl (change bus freq)
 *                    adding cpu- and bus-usage optimization (..slp)
 *                    T. Koschorrek <koschorrek at synertronixx.de>
 *          2005/2/28 changes in timings
 *                    M. Blaschke <blaschke at synertronixx.de>
 *          2005/6/01 added module parameters, cleaning up, cpu- and bus-usage
 *                    modularized (..slave)
 *                    T. Koschorrek <koschorrek at synertronixx.de>
 *          2007/1/14 fix i2c_imx_read/write to return number of messages
 *                      processed.
 *                    Change form for compatibility with kernel > 2.6.18.
 *                    Change default CLKDIV to 0x0F (400kHz i2c / 96MHz HCLK). 
 *                    Jorasse <jorasse@users.sourceforge.net>
 */

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/spinlock.h>

#include <mach/irqs.h>
#include <mach/hardware.h>

#define I2C_IMX_DRIVER_VERSION "14 January 2007"
#define I2C_IMX_DRIVER_AUTHOR  "T. Koschorrek, synertronixx GmbH"
#define I2C_IMX_DRIVER_DESC    "I2C Adapter/Algorithm driver"

#define I2C_IMX_DEFAULT_DEBUG   0
#define I2C_IMX_DEFAULT_CLKDIV  16 // 0x0f

#define I2C_IMX_TIME_GRAB       2000
#define I2C_IMX_TIME_BUSY       2000
#define I2C_IMX_TIME_RELEASE    2000
#define I2C_IMX_TIME_TXCOMPLETE 2000
#define I2C_IMX_MAX_GRAB        3

#define I2C_IMX_IO_SET_CLKDIV   0xaa
#define I2C_IMX_IO_GET_CLKDIV   0xab

#define I2C_IMX_ERR_INIT        0x01
#define I2C_IMX_ERR_SMBUS_XFER  0x02
#define I2C_IMX_ERR_RXACK       0x03
#define I2C_IMX_ERR_TXCOMPLETE  0x04
#define I2C_IMX_ERR_BUSY        0x05
#define I2C_IMX_ERR_GRAB        0x06



#ifdef I2C_IMX_SLAVESUPPORT
#include "i2c-imx-slave.h"
#endif



/* function declarations */
static int i2c_imx_xfer(struct i2c_adapter *i2c_adap, struct i2c_msg msgs[ ],
			int num);
static int i2c_imx_smbus(struct i2c_adapter * adapter, u16 addr,
			 unsigned short flags, char read_write, u8 command,
			 int size, union i2c_smbus_data * data);
static int i2c_imx_ioctl(struct i2c_adapter * adapter, unsigned int cmd,
			 unsigned long arg);
static u32 i2c_imx_func(struct i2c_adapter *adap);

/* structs */
static struct i2c_algorithm i2c_imx_algorithm = {
	.master_xfer	= 	i2c_imx_xfer,
	.smbus_xfer	=	i2c_imx_smbus,
	.functionality	= 	i2c_imx_func
};

static struct i2c_adapter i2c_imx_adapter = {
	.owner		= 	THIS_MODULE,
	.name 		= 	"IMX I2C adapter",
	.class 		= 	I2C_CLASS_HWMON,
	.algo 		= 	&i2c_imx_algorithm
};

struct i2c_regs {
  	/* address reg */     volatile u32 iadr;
	/* freq div. reg */   volatile u32 ifdr;
	/* control reg */     volatile u32 i2cr;
	/* status reg */      volatile u32 i2sr;
	/* data io reg */     volatile u32 i2dr;
};




/* global variables */
static struct i2c_regs *i2c_imx_reg = (struct i2c_regs *)IMX_I2C_BASE;
static spinlock_t i2c_imx_irqlock = SPIN_LOCK_UNLOCKED;
static int i2c_imx_irq_ok;
static int i2c_imx_i2sr;

#ifdef I2C_IMX_SLAVESUPPORT
static struct i2c_slave i2c_imx_slave[128];
#endif

int i2c_imx_errcnt_rxack = 0;
int i2c_imx_errcnt_txcomplete = 0;
int i2c_imx_errcnt_busy = 0;
int i2c_imx_errcnt_grab = 0;

int dbg = I2C_IMX_DEFAULT_DEBUG;
int i2c_imx_dbg = I2C_IMX_DEFAULT_DEBUG;
int clkdiv = I2C_IMX_DEFAULT_CLKDIV;
int i2c_imx_clkdiv = I2C_IMX_DEFAULT_CLKDIV;




/* debug messages */
#ifdef DEBUG
#define I2C_DBG(stuff ...) {                                        \
	if (i2c_imx_dbg) {                                          \
		printk(KERN_NOTICE stuff);                          \
	}                                                           \
	}
#define I2C_ERR(stuff ...) {                                        \
	if (i2c_imx_dbg) {                                          \
		printk(KERN_ERR "i2c_adap.%d ", i2c_imx_adapter.nr);\
		printk(stuff);                                      \
	}                                                           \
	}
#else
#define I2C_DBG(stuff ...) while(0){}
#define I2C_ERR(stuff ...) while(0){}
#endif

#ifdef DEBUG
#define I2C_ERRCNT {                                                       \
	I2C_ERR("  RX ACK     : %d Error(s)\n", i2c_imx_errcnt_rxack);     \
	I2C_ERR("  TX COMPLETE: %d Error(s)\n", i2c_imx_errcnt_txcomplete);\
	I2C_ERR("  BUS BUSY   : %d Error(s)\n", i2c_imx_errcnt_busy);      \
	I2C_ERR("  BUS GRAB   : %d Error(s)\n", i2c_imx_errcnt_grab);      \
}
#else
#define I2C_ERRCNT
#endif
