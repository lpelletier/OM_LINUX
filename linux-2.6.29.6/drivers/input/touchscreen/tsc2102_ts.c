/*
 * input/touchscreen/tsc2102_ts.c
 *
 * Touchscreen input device driver for boards using the TSC 2102 chip.
 *
 * Copyright (c) 2006 Andrzej Zaborowski  <balrog@zabor.org>
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

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/kbd_kern.h>

#include <linux/spi/tsc2102.h>

#define DRIVER_NAME "TSC210x Touchscreen"

struct input_dev *dev;

static void tsc210x_touch(int touching)
{
	if (!touching) {
		input_report_abs(dev, ABS_X, 0);
		input_report_abs(dev, ABS_Y, 0);
		input_report_abs(dev, ABS_PRESSURE, 0);
		input_sync(dev);
	}

	input_report_key(dev, BTN_TOUCH, touching);

	//do_poke_blanked_console = 1;
}

static void tsc210x_coords(int x, int y, int z1, int z2)
{
	int p;

	/* Calculate the touch resistance a la equation #1 */
	if (z1 != 0)
		p = x * (z2 - z1) / (z1 << 4);
	else
		p = 1;

	input_report_abs(dev, ABS_X, x);
	input_report_abs(dev, ABS_Y, y);
	input_report_abs(dev, ABS_PRESSURE, p);
	input_sync(dev);
}

static int tsc210x_ts_probe(struct platform_device *pdev)
{
	int status;

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;

	status = tsc210x_touch_cb(tsc210x_touch);
	if (status) {
		goto error;
	}

	status = tsc210x_coords_cb(tsc210x_coords);
	if (status) {
		goto error;
	}

	dev->name = DRIVER_NAME;
	dev->dev.parent = &pdev->dev;
	dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	dev->keybit[BIT_WORD(BTN_TOUCH)] |= BIT_MASK(BTN_TOUCH);
	dev->absbit[0] = BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) | BIT_MASK(ABS_PRESSURE);
	status = input_register_device(dev);
	if (status) {
		printk(KERN_INFO "Unable to register TSC210x as input device !\n");
		goto error;
	}

	printk(DRIVER_NAME " driver initialized\n");
	return 0;

error:
	input_free_device(dev);
	return status;
}

static int tsc210x_ts_remove(struct platform_device *pdev)
{
	tsc210x_touch_cb(0);
	tsc210x_coords_cb(0);
	input_unregister_device(dev);
	return 0;
}

static int
tsc210x_ts_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* Nothing yet */
	return 0;
}

static int tsc210x_ts_resume(struct platform_device *pdev)
{
	/* Nothing either */
	return 0;
}

static struct platform_driver tsc210x_ts_driver = {
	.probe 		= tsc210x_ts_probe,
	.remove 	= tsc210x_ts_remove,
	.suspend 	= tsc210x_ts_suspend,
	.resume 	= tsc210x_ts_resume,
	.driver		= {
		.name	= "tsc210x-ts",
	},
};

static int __init tsc210x_ts_init(void)
{
	int ret;

	ret = platform_driver_register(&tsc210x_ts_driver);
	if (ret)
		return -ENODEV;

	return 0;
}

static void __exit tsc210x_ts_exit(void)
{
	platform_driver_unregister(&tsc210x_ts_driver);
}

module_init(tsc210x_ts_init);
module_exit(tsc210x_ts_exit);

MODULE_AUTHOR("Andrzej Zaborowski");
MODULE_DESCRIPTION("Touchscreen input driver for TI TSC210x.");
MODULE_LICENSE("GPL");
