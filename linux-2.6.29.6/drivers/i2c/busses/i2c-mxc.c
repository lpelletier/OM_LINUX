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

/*
 * Driver for the Freescale Semiconductor MXC I2C buses.
 * Based on i2c driver algorithm for PCF8584 adapters
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/clock.h>
#include <mach/imx_i2c.h>

#define DRV_NAME "imx_i2c"

typedef struct {
	struct i2c_adapter adap;
	wait_queue_head_t wq;
	unsigned long membase;	/* FIXME iomem? */
	int irq;
	unsigned int clkdiv;
	struct clk *clk;
	bool low_power;
	struct imx_i2c_platform_data *pdata;
	int trans_flag;
} imx_i2c_device;

/*
 * HACK: Why is it not channel specific?
 * Boolean to indicate if data was transferred
 */
static bool transfer_done = false;

/*
 * HACK: Why is it not channel specific?
 * Boolean to indicate if we received an ACK for the data transmitted
 */
static bool tx_success = false;

struct clk_div_table {
	int reg_value;
	int div;
};

static const struct clk_div_table i2c_clk_table[] = {
	{0x20, 22}, {0x21, 24}, {0x22, 26}, {0x23, 28},
	{0, 30}, {1, 32}, {0x24, 32}, {2, 36},
	{0x25, 36}, {0x26, 40}, {3, 42}, {0x27, 44},
	{4, 48}, {0x28, 48}, {5, 52}, {0x29, 56},
	{6, 60}, {0x2A, 64}, {7, 72}, {0x2B, 72},
	{8, 80}, {0x2C, 80}, {9, 88}, {0x2D, 96},
	{0xA, 104}, {0x2E, 112}, {0xB, 128}, {0x2F, 128},
	{0xC, 144}, {0xD, 160}, {0x30, 160}, {0xE, 192},
	{0x31, 192}, {0x32, 224}, {0xF, 240}, {0x33, 256},
	{0x10, 288}, {0x11, 320}, {0x34, 320}, {0x12, 384},
	{0x35, 384}, {0x36, 448}, {0x13, 480}, {0x37, 512},
	{0x14, 576}, {0x15, 640}, {0x38, 640}, {0x16, 768},
	{0x39, 768}, {0x3A, 896}, {0x17, 960}, {0x3B, 1024},
	{0x18, 1152}, {0x19, 1280}, {0x3C, 1280}, {0x1A, 1536},
	{0x3D, 1536}, {0x3E, 1792}, {0x1B, 1920}, {0x3F, 2048},
	{0x1C, 2304}, {0x1D, 2560}, {0x1E, 3072}, {0x1F, 3840},
	{0, 0}
};

/* Transmit a STOP signal to the slave device */
static void imx_i2c_stop(imx_i2c_device * dev)
{
	unsigned int cr;
	int retry = 16;

	cr = readw(dev->membase + MXC_I2CR);
	cr &= ~(MXC_I2CR_MSTA | MXC_I2CR_MTX);
	writew(cr, dev->membase + MXC_I2CR);

	/*
	 * Make sure STOP meets setup requirement.
	 */
	for (;;) {
		unsigned int sr = readw(dev->membase + MXC_I2SR);
		if ((sr & MXC_I2SR_IBB) == 0) break;
		if (retry-- <= 0) {
#ifdef I2C_DEBUG
			printk(KERN_DEBUG "Bus busy\n");
#endif
			break;
		}
		udelay(3);
	}
}

/*
 * Wait for the transmission of the data byte to complete. This function waits
 * till we get a signal from the interrupt service routine indicating completion
 * of the address cycle or we time out.
 * The function returns 0 on success or -1 if an ack was not received
 */
static int imx_i2c_wait_for_tc(imx_i2c_device * dev, int trans_flag)
{
	int retry;

	if (trans_flag & I2C_M_IGNORE_NAK)
		retry = 1;
	else
		retry = 4;

	while (retry-- && !transfer_done) {
		wait_event_interruptible_timeout(dev->wq,
						 transfer_done,
						 dev->adap.timeout);
	}
	transfer_done = false;

	if (retry <= 0) {
		if (!tx_success) {
			/* An ACK was not received */
#ifdef I2C_DEBUG
			printk(KERN_DEBUG "ACK not received \n");
#endif
		}
		/* Unable to send data */
#ifdef I2C_DEBUG
		printk(KERN_DEBUG "Data not transmitted\n");
#endif
		if (trans_flag & I2C_M_IGNORE_NAK)
			return 0;
		else
			return -1;
	}

	return 0;
}

/* Transmit a START signal to the slave device */
static void imx_i2c_start(imx_i2c_device * dev, struct i2c_msg *msg)
{
	unsigned int cr, sr;
	unsigned int addr_trans;
	int retry = 16;

	/*
	 * Set the slave address and the requested transfer mode
	 * in the data register
	 */
	addr_trans = msg->addr << 1;
	if ((msg->flags & I2C_M_RD) && ((msg->flags & I2C_M_REV_DIR_ADDR)==0)) {
		addr_trans |= 0x01;
	} else if ((msg->flags & I2C_M_REV_DIR_ADDR) != 0) {
		addr_trans |= 0x01;
	}

	/* Set the Master bit */
	cr = readw(dev->membase + MXC_I2CR);
	cr |= MXC_I2CR_MSTA;
	writew(cr, dev->membase + MXC_I2CR);

	/* Wait till the Bus Busy bit is set */
	sr = readw(dev->membase + MXC_I2SR);
	while (retry-- && (!(sr & MXC_I2SR_IBB))) {
		udelay(3);
		sr = readw(dev->membase + MXC_I2SR);
	}
	if (retry <= 0) {
#ifdef I2C_DEBUG
		printk(KERN_DEBUG "Could not grab Bus ownership\n");
#endif
	}

	/* Set the Transmit bit */
	cr = readw(dev->membase + MXC_I2CR);
	cr |= MXC_I2CR_MTX;
	writew(cr, dev->membase + MXC_I2CR);

	writew(addr_trans, dev->membase + MXC_I2DR);
}

/* Transmit a REPEAT START to the slave device */
static void imx_i2c_repstart(imx_i2c_device * dev, struct i2c_msg *msg)
{
	unsigned int cr;
	unsigned int addr_trans;

	/*
	 * Set the slave address and the requested transfer mode
	 * in the data register
	 */
	addr_trans = msg->addr << 1;
	if ((msg->flags & I2C_M_RD) && ((msg->flags & I2C_M_REV_DIR_ADDR)==0)) {
		addr_trans |= 0x01;
	} else if ((msg->flags & I2C_M_REV_DIR_ADDR) != 0) {
		addr_trans |= 0x01;
	}
	cr = readw(dev->membase + MXC_I2CR);
	cr |= MXC_I2CR_RSTA;
	writew(cr, dev->membase + MXC_I2CR);
	udelay(3);
	writew(addr_trans, dev->membase + MXC_I2DR);
}

/*
 * Read the received data. The function waits till data is available or times
 * out. Generates a stop signal if this is the last message to be received.
 * Sends an ack for all the bytes received except the last byte.
 * The function returns the number of bytes read or -1 on time out.
 */
static int imx_i2c_readbytes(imx_i2c_device * dev, struct i2c_msg *msg,
			     int last, int addr_comp)
{
	int i;
	char *buf = msg->buf;
	int len = msg->len;
	unsigned int cr;

	cr = readw(dev->membase + MXC_I2CR);

	/* switch to receive mode */
	cr &= ~MXC_I2CR_MTX;
	/*
	 * Clear the TXAK bit to gen an ack when receiving only one byte.
	 */
	if (len == 1)
		cr |= MXC_I2CR_TXAK;
	else
		cr &= ~MXC_I2CR_TXAK;

	writew(cr, dev->membase + MXC_I2CR);
	/*
	 * Dummy read only at the end of an address cycle
	 */
	if (addr_comp > 0)
		readw(dev->membase + MXC_I2DR);

	for (i = 0; i < len; i++) {
		/* Wait for data transmission to complete */
		if (imx_i2c_wait_for_tc(dev, msg->flags)) {
			imx_i2c_stop(dev);
			return -1;
		}
		/* Do not generate an ACK for the last byte
		 * or if I2C_M_NO_RD_ACK flag is set
		 */
		if (i == (len - 2)) {
			cr = readw(dev->membase + MXC_I2CR);
			if ((msg->flags & I2C_M_NO_RD_ACK) == 0)
				cr |= MXC_I2CR_TXAK;
			else
				cr &= ~MXC_I2CR_TXAK;
			writew(cr, dev->membase + MXC_I2CR);
		} else if (i == (len - 1)) {
			if (last)
				imx_i2c_stop(dev);
		}
		/* Read the data */
		*buf++ = readw(dev->membase + MXC_I2DR);
	}

	return i;
}

/*
 * Write the data to the data register. Generates a stop signal if this is
 * the last message to be sent or if no ack was received for the data sent.
 * The function returns the number of bytes written or -1 on time out
 * or if no ack was received for the data that was sent.
 */
static int imx_i2c_writebytes(imx_i2c_device * dev, struct i2c_msg *msg,
				int last)
{
	int i;
	char *buf = msg->buf;
	int len = msg->len;
	unsigned int cr;

	cr = readw(dev->membase + MXC_I2CR);
	/* switch to transmit mode */
	cr |= MXC_I2CR_MTX;
	writew(cr, dev->membase + MXC_I2CR);

	for (i = 0; i < len; i++) {
		/* Write the data */
		writew(*buf++, dev->membase + MXC_I2DR);
		if (imx_i2c_wait_for_tc(dev, msg->flags)) {
			imx_i2c_stop(dev);
			return -1;
		}
	}
	if (last > 0) {
		imx_i2c_stop(dev);
	}

	return i;
}

/* Function enables the I2C module and initializes the registers */
static void imx_i2c_module_en(imx_i2c_device * dev, int trans_flag)
{
	clk_enable(dev->clk);
	/* Set the frequency divider */
	writew(dev->clkdiv, dev->membase + MXC_IFDR);
	/* Clear the status register */
	writew(0x0, dev->membase + MXC_I2SR);
	/* Enable I2C and its interrupts */
	writew(MXC_I2CR_IEN, dev->membase + MXC_I2CR);
	writew(MXC_I2CR_IEN | MXC_I2CR_IIEN, dev->membase + MXC_I2CR);
}

/* Disables the I2C module */
static void imx_i2c_module_dis(imx_i2c_device * dev)
{
	writew(0x0, dev->membase + MXC_I2CR);
	clk_disable(dev->clk);
}

/*
 * The function is registered in the adapter structure. It is called when an MXC
 * driver wishes to transfer data to a device connected to the I2C device.
 * The function returns the number of messages transferred, -EREMOTEIO on I2C
 * failure and a 0 if the num argument is less than 0.
 */
static int imx_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[],
			int num)
{
	imx_i2c_device *dev = (imx_i2c_device *) (i2c_get_adapdata(adap));
	int i, ret = 0, addr_comp = 0;
	unsigned int sr;

	if (dev->low_power) {
		printk(KERN_ERR "I2C Device in low power mode\n");
		return -EREMOTEIO;
	}

	if (num < 1)
		return 0;

	imx_i2c_module_en(dev, msgs[0].flags);
	sr = readw(dev->membase + MXC_I2SR);

	/* Check bus state */
	if (sr & MXC_I2SR_IBB) {
		imx_i2c_module_dis(dev);
#ifdef I2C_DEBUG
		printk(KERN_DEBUG "Bus busy\n");
#endif
		return -EREMOTEIO;
	}

	transfer_done = false;
	tx_success = false;
	for (i = 0; i < num && ret >= 0; i++) {
		addr_comp = 0;
		dev->trans_flag = msgs[i].flags;
		/*
		 * Send the slave address and transfer direction in the
		 * address cycle
		 */
		if (i == 0) {
			/* Send a start or repeat start signal */
			imx_i2c_start(dev, &msgs[0]);
			/* Wait for the address cycle to complete */
			if (imx_i2c_wait_for_tc(dev, msgs[0].flags)) {
				imx_i2c_stop(dev);
				imx_i2c_module_dis(dev);
				return -EREMOTEIO;
			}
			addr_comp = 1;
		} else {
			/*
			 * Generate repeat start only if required i.e the address
			 * changed or the transfer direction changed
			 */
			if ((msgs[i].addr != msgs[i - 1].addr) ||
			    ((msgs[i].flags & I2C_M_RD) !=
			     (msgs[i - 1].flags & I2C_M_RD))) {
				if ((msgs[i].flags & I2C_M_NOSTART) == 0) {
					imx_i2c_repstart(dev, &msgs[i]);
					/* Wait for the address cycle to complete */
					if (imx_i2c_wait_for_tc(dev, msgs[i].flags)) {
						imx_i2c_stop(dev);
						imx_i2c_module_dis(dev);
						return -EREMOTEIO;
					}
				}
				addr_comp = 1;
			}
		}

		/* Transfer the data */
		if (msgs[i].flags & I2C_M_RD) {
			/* Read the data */
			ret = imx_i2c_readbytes(dev, &msgs[i], (i + 1 == num),
						addr_comp);
			if (ret < 0) {
				printk(KERN_ERR "imx_i2c_readbytes: fail.\n");
				break;
			}
		} else {
			/* Write the data */
			ret = imx_i2c_writebytes(dev, &msgs[i], (i + 1 == num));
			if (ret < 0) {
				printk(KERN_ERR "imx_i2c_writebytes: fail.\n");
				break;
			}
		}
	}

	imx_i2c_module_dis(dev);
	return i;
}

/*
 * Returns the i2c functionality supported by this driver.
 * Returns the functionality that is supported.
 */
static u32 imx_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_PROTOCOL_MANGLING ;
}

/*
 * Stores the pointers for the i2c algorithm functions. The algorithm functions
 * is used by the i2c bus driver to talk to the i2c bus
 */
static struct i2c_algorithm imx_i2c_algorithm = {
	.master_xfer = imx_i2c_xfer,
	.functionality = imx_i2c_func
};

/*
 * Interrupt Service Routine. It signals to the process about the data transfer
 * completion. Also sets a flag if bus arbitration is lost.
 * The function returns IRQ_HANDLED.
 */
static irqreturn_t imx_i2c_handler(int irq, void *dev_id)
{
	imx_i2c_device *dev = dev_id;
	unsigned int sr, cr;

	sr = readw(dev->membase + MXC_I2SR);
	cr = readw(dev->membase + MXC_I2CR);

	/*
	 * Clear the interrupt bit
	 */
	writew(0x0, dev->membase + MXC_I2SR);

	if (sr & MXC_I2SR_IAL) {
#ifdef I2C_DEBUG
		printk(KERN_DEBUG "Bus Arbitration lost\n");
#endif
	} else {
		/* Interrupt due byte transfer completion */
		tx_success = false;
		/* Check if RXAK is received in Transmit mode */
		if (cr & MXC_I2CR_MTX) {
			if( !(sr & MXC_I2SR_RXAK)) {
				tx_success = true;
				transfer_done = true;
			}
		} else {
			tx_success = true;
			transfer_done = true;
		}
		wake_up_interruptible(&dev->wq);
	}

	return IRQ_HANDLED;
}

/*
 * This function is called to put the I2C adapter in a low power state. Refer to the
 * document driver-model/driver.txt in the kernel source tree for more
 * information.
 * The function returns 0 on success and -1 on failure.
 */
static int mxci2c_suspend(struct platform_device *pdev, pm_message_t state)
{
	imx_i2c_device *mxcdev = platform_get_drvdata(pdev);
	unsigned int sr;

	if (mxcdev == NULL)
		return -1;

	/* Prevent further calls to be processed */
	mxcdev->low_power = true;
	/* Wait till we finish the current transfer */
	sr = readw(mxcdev->membase + MXC_I2SR);
	while (sr & MXC_I2SR_IBB) {
		msleep(10);
		sr = readw(mxcdev->membase + MXC_I2SR);
	}

	mxcdev->pdata->exit(pdev);

	return 0;
}

/*
 * This function is called to bring the I2C adapter back from a low power state. Refer
 * to the document driver-model/driver.txt in the kernel source tree for more
 * information.
 * The function returns 0 on success and -1 on failure
 */
static int mxci2c_resume(struct platform_device *pdev)
{
	imx_i2c_device *mxcdev = platform_get_drvdata(pdev);

	if (mxcdev == NULL)
		return -1;

	mxcdev->low_power = false;
	mxcdev->pdata->init(pdev);

	return 0;
}

/*
 * This function is called during the driver binding process.
 * The function always returns 0.
 */
static int mxci2c_probe(struct platform_device *pdev)
{
	imx_i2c_device *imx_i2c;
	struct imx_i2c_platform_data *i2c_plat_data = pdev->dev.platform_data;
	struct resource *res;
	int id = pdev->id;
	u32 clk_freq;
	int ret = 0;
	int i=0;

	imx_i2c = kzalloc(sizeof(imx_i2c_device), GFP_KERNEL);
	if (!imx_i2c) {
		return -ENOMEM;
	}

	imx_i2c->pdata = i2c_plat_data;
	/* claim the region we will work on */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		ret = -ENODEV;
		goto err1;
	}
	if (!request_mem_region(res->start, res->end - res->start + 1, DRV_NAME)) {
		dev_err(&pdev->dev, "request_mem_region failed for IMX I2C %d\n", id);
		     ret = -EBUSY;
		goto err1;
	}

	/* TODO: use iomap() */
	imx_i2c->membase = IO_ADDRESS(res->start);

	/* Claim the I2C irq line */
	imx_i2c->irq = platform_get_irq(pdev, 0);
	if (imx_i2c->irq < 0) {
		dev_err(&pdev->dev, "No interrupt defined for IMX I2C %d\n", id);
		ret = imx_i2c->irq;
		goto err2;
	}
	ret = request_irq(imx_i2c->irq, imx_i2c_handler,
			  0, DRV_NAME, imx_i2c);
	if (ret < 0) {
		dev_err(&pdev->dev, "Cannot claim interrupt %d for IMX I2C %d\n",
			imx_i2c->irq, id);
		goto err2;
	}

	init_waitqueue_head(&imx_i2c->wq);

	imx_i2c->low_power = false;

	imx_i2c->pdata->init(pdev);

	imx_i2c->clk = clk_get(&pdev->dev, "i2c_clk");
	if (imx_i2c == NULL) {
		dev_err(&pdev->dev, "Cannot get clock for for IMX I2C %d\n", id);
		goto err3;
	}

	clk_freq = clk_get_rate(imx_i2c->clk);
	imx_i2c->clkdiv = -1;
	if (i2c_plat_data->max_clk) {
		/* Calculate divider and round up any fractional part */
		int div = (clk_freq + i2c_plat_data->max_clk - 1) /
		    i2c_plat_data->max_clk;
		for (i = 0; i2c_clk_table[i].div != 0; i++) {
			if (i2c_clk_table[i].div >= div) {
				imx_i2c->clkdiv = i2c_clk_table[i].reg_value;
				break;
			}
		}
	}
	if (imx_i2c->clkdiv == -1) {
		i--;
		imx_i2c->clkdiv = 0x1F;	/* Use max divider */
	}
	dev_dbg(&pdev->dev, "i2c speed is %d/%d = %d bps, reg val = 0x%02X\n",
		clk_freq, i2c_clk_table[i].div,
		clk_freq / i2c_clk_table[i].div, imx_i2c->clkdiv);

	/*
	 * Set the adapter information
	 */
	strcpy(imx_i2c->adap.name, MXC_ADAPTER_NAME);
	imx_i2c->adap.id = id;
	imx_i2c->adap.nr = id;
	imx_i2c->adap.algo = &imx_i2c_algorithm;
	imx_i2c->adap.timeout = 1;
	platform_set_drvdata(pdev, imx_i2c);
	i2c_set_adapdata(&imx_i2c->adap, imx_i2c);
	if ((ret = i2c_add_numbered_adapter(&imx_i2c->adap)) < 0) {
		dev_err(&pdev->dev, "Cannot register the IMX I2C %d master\n", id);
		goto err3;
	}

	return 0;

err3:
	imx_i2c->pdata->exit(pdev);
	free_irq(imx_i2c->irq, imx_i2c);
err2:
	release_mem_region(res->start, res->end - res->start + 1);
err1:
	dev_err(&pdev->dev, "failed to probe i2c adapter\n");
	kfree(imx_i2c);
	return ret;
}

/*
 * Dissociates the driver from the I2C device.
 */
static int mxci2c_remove(struct platform_device *pdev)
{
	imx_i2c_device *imx_i2c = platform_get_drvdata(pdev);
	struct resource *res;

	free_irq(imx_i2c->irq, imx_i2c);
	i2c_del_adapter(&imx_i2c->adap);
	imx_i2c->pdata->exit(pdev);
	clk_put(imx_i2c->clk);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, res->end - res->start + 1);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

/*
 * This structure contains pointers to the power management callback functions.
 */
static struct platform_driver mxci2c_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
	},
	.probe = mxci2c_probe,
	.remove = mxci2c_remove,
	.suspend = mxci2c_suspend,
	.resume = mxci2c_resume,
};

static int __init imx_i2c_init(void)
{
	return platform_driver_register(&mxci2c_driver);
}

static void __exit imx_i2c_exit(void)
{
	platform_driver_unregister(&mxci2c_driver);
}

subsys_initcall(imx_i2c_init);
module_exit(imx_i2c_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC I2C driver");
MODULE_LICENSE("GPL");
