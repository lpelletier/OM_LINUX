/*
 * An rtc/alarm/i2c driver for the Dallas ds1374
 * based on rts-ds1672 driver from Alessandro Zummo
 * Copyright 2007 Eric Jarrige
 *
 * Author: Jorasse <jorasse@users.sourceforge.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/rtc.h>

#define DRV_VERSION "1.0"

/* Addresses to scan: 68. This is the defailt chip address. */
static unsigned short normal_i2c[] = { 0x68, I2C_CLIENT_END };
/* Insmod parameters */
I2C_CLIENT_INSMOD;

/* Registers */

#define ds1374_REG_CNT_BASE	0
#define ds1374_REG_ALARM_BASE	4
#define ds1374_REG_CONTROL	7
#define ds1374_REG_STATUS	8
#define ds1374_REG_TRICKLE	9

#define ds1374_REG_CONTROL_EOSC	0x80

/* Prototypes */
static int ds1374_probe(struct i2c_adapter *adapter, int address, int kind);

/*
 * In the routines that deal directly with the ds1374 hardware, we use
 * rtc_time -- month 0-11, hour 0-23, yr = calendar year-epoch
 * Epoch is initialized as 2000. Time is set to UTC.
 */
static int ds1374_get_datetime(struct i2c_client *client, struct rtc_time *tm)
{
	unsigned long time;
	unsigned char addr = ds1374_REG_CNT_BASE;
	unsigned char buf[4];

	struct i2c_msg msgs[] = {
		{ client->addr, 0, 1, &addr },		/* setup read ptr */
		{ client->addr, I2C_M_RD, 4, buf },	/* read date */
	};

	/* read date registers */
	if ((i2c_transfer(client->adapter, &msgs[0], 2)) != 2) {
		dev_err(&client->dev, "%s: read error\n", __FUNCTION__);
		return -EIO;
	}

	/*dev_dbg(&client->dev,
		"%s: raw read data - counters=%02x,%02x,%02x,%02x\n"
		__FUNCTION__, buf[0], buf[1], buf[2], buf[3]);*/

	time = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];

	rtc_time_to_tm(time, tm);

	dev_dbg(&client->dev, "%s: tm is secs=%d, mins=%d, hours=%d, "
		"mday=%d, mon=%d, year=%d, wday=%d\n",
		__FUNCTION__, tm->tm_sec, tm->tm_min, tm->tm_hour,
		tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);

	return 0;
}

static int ds1374_set_mmss(struct i2c_client *client, unsigned long secs)
{
	int xfer;
	unsigned char buf[6];

	buf[0] = ds1374_REG_CNT_BASE;
	buf[1] = secs & 0x000000FF;
	buf[2] = (secs & 0x0000FF00) >> 8;
	buf[3] = (secs & 0x00FF0000) >> 16;
	buf[4] = (secs & 0xFF000000) >> 24;
	buf[5] = 0;	/* set control reg to enable counting */

	xfer = i2c_master_send(client, buf, 6);
	if (xfer != 6) {
		dev_err(&client->dev, "%s: send: %d\n", __FUNCTION__, xfer);
		return -EIO;
	}

	return 0;
}

static int ds1374_set_datetime(struct i2c_client *client, struct rtc_time *tm)
{
	unsigned long secs;

	dev_dbg(&client->dev,
		"%s: secs=%d, mins=%d, hours=%d,  \
        mday=%d, mon=%d, year=%d, wday=%d \n",
		__FUNCTION__,
		tm->tm_sec, tm->tm_min, tm->tm_hour,
		tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);

	rtc_tm_to_time(tm, &secs);

	return ds1374_set_mmss(client, secs);
}

static int ds1374_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	return ds1374_get_datetime(to_i2c_client(dev), tm);
}

static int ds1374_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	return ds1374_set_datetime(to_i2c_client(dev), tm);
}

static int ds1374_rtc_set_mmss(struct device *dev, unsigned long secs)
{
	return ds1374_set_mmss(to_i2c_client(dev), secs);
}

static int ds1374_get_control(struct i2c_client *client, u8 *status)
{
	unsigned char addr = ds1374_REG_CONTROL;

	struct i2c_msg msgs[] = {
		{ client->addr, 0, 1, &addr },		/* setup read ptr */
		{ client->addr, I2C_M_RD, 1, status },	/* read control */
	};

	/* read control register */
	if ((i2c_transfer(client->adapter, &msgs[0], 2)) != 2) {
		dev_err(&client->dev, "%s: read error\n", __FUNCTION__);
		return -EIO;
	}

	return 0;
}

/* following are the sysfs callback functions */
static ssize_t show_control(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 control;
	int err;

	err = ds1374_get_control(client, &control);
	if (err)
		return err;

	return sprintf(buf, "%s\n", (control & ds1374_REG_CONTROL_EOSC)
					? "disabled" : "enabled");
}
static DEVICE_ATTR(control, S_IRUGO, show_control, NULL);

static struct rtc_class_ops ds1374_rtc_ops = {
	.read_time	= ds1374_rtc_read_time,
	.set_time	= ds1374_rtc_set_time,
	.set_mmss	= ds1374_rtc_set_mmss,
};

static int ds1374_attach(struct i2c_adapter *adapter)
{
	return i2c_probe(adapter, &addr_data, ds1374_probe);
}

static int ds1374_detach(struct i2c_client *client)
{
	int err;
	struct rtc_device *rtc = i2c_get_clientdata(client);

 	if (rtc)
		rtc_device_unregister(rtc);

	if ((err = i2c_detach_client(client)))
		return err;

	kfree(client);

	return 0;
}

static struct i2c_driver ds1374_driver = {
	.driver		= {
		.name	= "ds1374-legacy",
	},
	.id		= I2C_DRIVERID_DS1374,
	.attach_adapter = &ds1374_attach,
	.detach_client	= &ds1374_detach,
};

static int ds1374_probe(struct i2c_adapter *adapter, int address, int kind)
{
	int err = 0;
	u8 control;
	struct i2c_client *client;
	struct rtc_device *rtc;

	dev_dbg(&adapter->dev, "%s\n", __FUNCTION__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit;
	}

	if (!(client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit;
	}

	/* I2C client */
	client->addr = address;
	client->driver = &ds1374_driver;
	client->adapter	= adapter;

	strlcpy(client->name, ds1374_driver.driver.name, I2C_NAME_SIZE);

	/* Inform the i2c layer */
	if ((err = i2c_attach_client(client)))
		goto exit_kfree;

	dev_info(&client->dev, "chip found, driver version " DRV_VERSION "\n");

	rtc = rtc_device_register(ds1374_driver.driver.name, &client->dev,
				&ds1374_rtc_ops, THIS_MODULE);

	if (IS_ERR(rtc)) {
		err = PTR_ERR(rtc);
		goto exit_detach;
	}

	i2c_set_clientdata(client, rtc);

	/* read control register */
	err = ds1374_get_control(client, &control);
	if (err)
		goto exit_detach;

	if (control & ds1374_REG_CONTROL_EOSC)
		dev_warn(&client->dev, "Oscillator not enabled. "
					"Set time to enable.\n");

	/* Register sysfs hooks */
	err = device_create_file(&client->dev, &dev_attr_control);
    if (err)
        printk(KERN_WARNING "Unable to create sysfs attribute file for DS1374\n");
    else
        return 0;

exit_detach:
	if (!IS_ERR(rtc))
		rtc_device_unregister(rtc);

	if (!i2c_detach_client(client))

exit_kfree:
	kfree(client);

exit:
	return err;
}

static int __init ds1374_init(void)
{
	return i2c_add_driver(&ds1374_driver);
}

static void __exit ds1374_exit(void)
{
	i2c_del_driver(&ds1374_driver);
}

MODULE_AUTHOR("Jorasse <jorasse@users.sourceforge.net>");
MODULE_DESCRIPTION("Dallas/Maxim ds1374 timekeeper with alarm driver (legacy)");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

module_init(ds1374_init);
module_exit(ds1374_exit);
