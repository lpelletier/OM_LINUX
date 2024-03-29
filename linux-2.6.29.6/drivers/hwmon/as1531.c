/*
 * as1531.c
 *
 * Driver for Austria-Microsystem Analog to Digital Converter.
 *
 * Copyright (c) 2010 Fabien Marteau <fabien.marteau@armadeus.com>
 * sponsored by ARMadeus Systems.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>

#define AS1531_SPI_SPEED 64941
#define AS1531_MAX_VALUE 2500

#define AS1531_START_BIT					0x80
#define AS1531_CHAN0						(0<<4)
#define AS1531_CHAN1						(4<<4)
#define AS1531_CHAN2						(1<<4)
#define AS1531_CHAN3						(5<<4)
#define AS1531_CHAN4						(2<<4)
#define AS1531_CHAN5						(6<<4)
#define AS1531_CHAN6						(3<<4)
#define AS1531_CHAN7						(7<<4)

#define AS1531_RANGE_0_TO_VREF			  (1<<3)
#define AS1531_RANGE_HALFVREF_TO_HALFVREF   (0<<3)

#define AS1531_MODE_COM					 (1<<2)
#define AS1531_MODE_DIFF					(0<<2)

#define AS1531_POWER_DOWN				   0x0
#define AS1531_POWER_REDUCED				0x1
#define AS1531_POWER_REDUCED_BIS			0x2
#define AS1531_POWER_NORMAL				 0x3

struct as1531 {
	struct device *hwmon_dev;
	struct mutex lock;
};

static int as1531_message(struct spi_device *spi, int cmd, int *ret_value)
{
	struct spi_message	message;
	struct spi_transfer	x[1];
	int status, i;
	u8	cmd_send;
	unsigned char buf[64];
	unsigned char buf_read[64];

	cmd_send = cmd;

	spi_message_init(&message);
	memset(x, 0, sizeof x);
	memset(buf, 0, sizeof(buf));
	memset(buf_read, 0, sizeof(buf_read));

	for (i = 0; i < 8; i++) {
		buf[i] = ((cmd_send & 0x80)>>7);
		cmd_send = cmd_send << 1;
	}

	x[0].tx_buf = buf;
	x[0].len = 24;
	x[0].rx_buf = buf_read;
	x[0].speed_hz = AS1531_SPI_SPEED;
	x[0].bits_per_word = 1;
	spi_message_add_tail(&x[0], &message);

	status = spi_sync(spi, &message);
	if (status < 0)
		return status;

	*ret_value = buf_read[11] & 0x01;
	for (i = 12; i < 23 ; i++) {
		*ret_value = *ret_value << 1;
		*ret_value = *ret_value | (buf_read[i]&0x01);
	}

	return 0;
}

static ssize_t as1531_read(struct device *dev,
			   struct device_attribute *devattr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct as1531 *adc = dev_get_drvdata(&spi->dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	int status = 0;
	int ret_value;
	int32_t cmd;


	if (mutex_lock_interruptible(&adc->lock))
		return -ERESTARTSYS;

	switch (attr->index) {
	case 0:
		cmd = AS1531_START_BIT | AS1531_CHAN0 |
		AS1531_RANGE_0_TO_VREF | AS1531_MODE_COM |
		AS1531_POWER_NORMAL;
		break;
	case 1:
		cmd = AS1531_START_BIT | AS1531_CHAN1 |
		AS1531_RANGE_0_TO_VREF | AS1531_MODE_COM |
		AS1531_POWER_NORMAL;
		break;
	case 2:
		cmd = AS1531_START_BIT | AS1531_CHAN2 |
		AS1531_RANGE_0_TO_VREF | AS1531_MODE_COM |
		AS1531_POWER_NORMAL;
		break;
	case 3:
		cmd = AS1531_START_BIT | AS1531_CHAN3 |
		AS1531_RANGE_0_TO_VREF | AS1531_MODE_COM |
		AS1531_POWER_NORMAL;
		break;
	case 4:
		cmd = AS1531_START_BIT | AS1531_CHAN4 |
		AS1531_RANGE_0_TO_VREF | AS1531_MODE_COM |
		AS1531_POWER_NORMAL;
		break;
	case 5:
		cmd = AS1531_START_BIT | AS1531_CHAN5 |
		AS1531_RANGE_0_TO_VREF | AS1531_MODE_COM |
		AS1531_POWER_NORMAL;
		break;
	case 6:
		cmd = AS1531_START_BIT | AS1531_CHAN6 |
		AS1531_RANGE_0_TO_VREF | AS1531_MODE_COM |
		AS1531_POWER_NORMAL;
		break;
	case 7:
		cmd = AS1531_START_BIT | AS1531_CHAN7 |
		AS1531_RANGE_0_TO_VREF | AS1531_MODE_COM |
		AS1531_POWER_NORMAL;
		break;
	default:
		status = -EINVAL;
		goto out;
	}
	status = as1531_message(spi, cmd, &ret_value);
	if (status < 0)
		goto out;

	status = sprintf(buf, "%d\n", ret_value*2500/4096);
out:
	mutex_unlock(&adc->lock);
	return status;
}


static ssize_t as1531_show_min(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	/* The minimum reference is 0 for this chip family */
	return sprintf(buf, "0\n");
}

static ssize_t as1531_show_max(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%d\n", AS1531_MAX_VALUE);
}

static ssize_t as1531_show_name(struct device *dev, struct device_attribute
				*devattr, char *buf)
{
	return sprintf(buf, "as1531\n");
}

static struct sensor_device_attribute as1531_input[] = {
	SENSOR_ATTR(name, S_IRUGO, as1531_show_name, NULL, 0),
	SENSOR_ATTR(in_min, S_IRUGO, as1531_show_min, NULL, 0),
	SENSOR_ATTR(in_max, S_IRUGO, as1531_show_max, NULL, 0),
	SENSOR_ATTR(in0_input, S_IRUGO, as1531_read, NULL, 0),
	SENSOR_ATTR(in1_input, S_IRUGO, as1531_read, NULL, 1),
	SENSOR_ATTR(in2_input, S_IRUGO, as1531_read, NULL, 2),
	SENSOR_ATTR(in3_input, S_IRUGO, as1531_read, NULL, 3),
	SENSOR_ATTR(in4_input, S_IRUGO, as1531_read, NULL, 4),
	SENSOR_ATTR(in5_input, S_IRUGO, as1531_read, NULL, 5),
	SENSOR_ATTR(in6_input, S_IRUGO, as1531_read, NULL, 6),
	SENSOR_ATTR(in7_input, S_IRUGO, as1531_read, NULL, 7),
};

/*----------------------------------------------------------------------*/

static int __devinit as1531_probe(struct spi_device *spi)
{
	struct as1531 *adc;
	int status;
	int i;

	adc = kzalloc(sizeof(struct as1531), GFP_KERNEL);
	if (adc == NULL)
		return -ENOMEM;

	mutex_init(&adc->lock);
	mutex_lock(&adc->lock);

	dev_set_drvdata(&spi->dev, adc);

	for (i = 0; i < 11; i++) {
		status = device_create_file(&spi->dev,
						&as1531_input[i].dev_attr);
		if (status < 0) {
			dev_err(&spi->dev, "device_create_file failed.\n");
			goto out_err;
		}
	}

	adc->hwmon_dev = hwmon_device_register(&spi->dev);
	if (IS_ERR(adc->hwmon_dev)) {
		dev_err(&spi->dev, "hwmon_device_register failed.\n");
		status = PTR_ERR(adc->hwmon_dev);
		goto out_err;
	}

	mutex_unlock(&adc->lock);
	return 0;

out_err:
	for (i--; i >= 0; i--)
		device_remove_file(&spi->dev, &as1531_input[i].dev_attr);
	dev_set_drvdata(&spi->dev, NULL);
	mutex_unlock(&adc->lock);
	kfree(adc);
	return status;
}

static int __devexit as1531_remove(struct spi_device *spi)
{
	struct as1531 *adc = dev_get_drvdata(&spi->dev);
	int i;

	mutex_lock(&adc->lock);
	hwmon_device_unregister(adc->hwmon_dev);
	for (i = 0; i < 8; i++)
		device_remove_file(&spi->dev, &as1531_input[i].dev_attr);

	dev_set_drvdata(&spi->dev, NULL);
	mutex_unlock(&adc->lock);
	kfree(adc);

	return 0;
}

/* SPI structures */

static struct spi_driver as1531_driver = {
	.driver = {
		.name	= "as1531",
		.owner	= THIS_MODULE,
	},
	.probe	= as1531_probe,
	.remove	= __devexit_p(as1531_remove),
};

/* Init module */

static int __init init_as1531(void)
{
	return spi_register_driver(&as1531_driver);
}

static void __exit exit_as1531(void)
{
	spi_unregister_driver(&as1531_driver);
}

module_init(init_as1531);
module_exit(exit_as1531);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Fabien Marteau <fabien.marteau@armadeus.com>");
MODULE_DESCRIPTION("Driver for AS1531 ADC");
