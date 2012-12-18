/*
 * Copyright (c) 2010 Fabien Marteau <fabien.marteau@armadeus.com>
 *
 * Sponsored by ARMadeus Systems
 */

/*
 * Driver for Austria Microsystems joysticks AS5011
 */

/*
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * TODO:
 *	- Use threaded IRQ instead of Hard IRQ
 *	- Manage power mode
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/sysfs.h>
#include <linux/ctype.h>
#include <linux/hwmon-sysfs.h>

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>

#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>

#include <linux/as5011.h>
#define DRIVER_DESC "Driver for Austria Microsystems AS5011 joystick"

MODULE_AUTHOR("Fabien Marteau <fabien.marteau@armadeus.com>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

#define SIZE_NAME 30
#define SIZE_EVENT_PATH 40
#define AS5011_MAX_AXIS	80
#define AS5011_MIN_AXIS	(-80)
#define AS5011_FUZZ	8
#define AS5011_FLAT	40

static int g_num = 1; /* device counter */

static signed char as5011_i2c_read(struct i2c_client *client,
			   uint8_t aRegAddr);
static int as5011_i2c_write(struct i2c_client *client,
			    uint8_t aRegAddr,
			    uint8_t aValue);

/*
 * interrupts operations
 */

static irqreturn_t button_interrupt(int irq, void *dev_id)
{
	struct as5011_platform_data *plat_dat =
		(struct as5011_platform_data *)dev_id;
	int ret;

	ret = gpio_get_value(plat_dat->button_gpio);
	if (ret)
		input_report_key(plat_dat->input_dev, BTN_JOYSTICK, 0);
	else
		input_report_key(plat_dat->input_dev, BTN_JOYSTICK, 1);
	input_sync(plat_dat->input_dev);
	return IRQ_HANDLED;
}

static void as5011_update_axes(struct work_struct *work)
{
	struct as5011_platform_data *plat_dat =
		container_of(work,
			     struct as5011_platform_data,
			     update_axes_work);
	signed char x, y;

	x = as5011_i2c_read(plat_dat->i2c_client, AS5011_X_RES_INT);
	y = as5011_i2c_read(plat_dat->i2c_client, AS5011_Y_RES_INT);
	input_report_abs(plat_dat->input_dev, ABS_X, x);
	input_report_abs(plat_dat->input_dev, ABS_Y, y);
	input_sync(plat_dat->input_dev);
}

static irqreturn_t as5011_int_interrupt(int irq, void *dev_id)
{
	struct as5011_platform_data *plat_dat =
		(struct as5011_platform_data *)dev_id;

	queue_work(plat_dat->workqueue, &plat_dat->update_axes_work);

	return IRQ_HANDLED;
}

/*
 * I2C bus operation
 */

static int as5011_i2c_write(struct i2c_client *client,
			    uint8_t aRegAddr,
			    uint8_t aValue)
{
	int ret;
	uint8_t data[2] = { aRegAddr, aValue };
	struct i2c_msg msg = {	client->addr,
				I2C_M_IGNORE_NAK,
				2,
				(uint8_t *)data };

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		return ret;
	return 1;
}

static signed char as5011_i2c_read(struct i2c_client *client,
			   uint8_t aRegAddr)
{
	int ret;
	uint8_t data[2];
	struct i2c_msg msg_set[2];
	struct i2c_msg msg1 = {	client->addr,
				I2C_M_REV_DIR_ADDR,
				1,
				(uint8_t *)data};
	struct i2c_msg msg2 = {	client->addr,
				I2C_M_RD|I2C_M_NOSTART,
				1,
				(uint8_t *)data};

	data[0] = aRegAddr;
	msg_set[0] = msg1;
	msg_set[1] = msg2;

	ret = i2c_transfer(client->adapter, msg_set, 2);
	if (ret < 0)
		return ret;

	if (data[0] & 0x80)
		return -1*(1+(~data[0]));
	else
		return data[0];
}

static int __devinit as5011_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct as5011_platform_data *plat_dat = client->dev.platform_data;
	int retval = 0;

	plat_dat->num = g_num;
	g_num++;

	scnprintf(plat_dat->workqueue_name,
			  SIZE_NAME,
			  "as5011_%d_workqueue", plat_dat->num);
	plat_dat->workqueue =
		create_singlethread_workqueue(plat_dat->workqueue_name);
	if (plat_dat->workqueue == NULL) {
		dev_err(&client->dev, "Failed to create workqueue\n");
		retval = -EINVAL;
		goto err_out;
	}

	INIT_WORK(&plat_dat->update_axes_work, as5011_update_axes);

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_PROTOCOL_MANGLING)) {
		dev_err(&client->dev,
		"i2c bus does not support protocol mangling, as5011 can't work\n");
		retval = -ENODEV;
		goto err_destroy_worqueue;
	}
	plat_dat->i2c_client = client;

	plat_dat->input_dev = input_allocate_device();
	if (plat_dat->input_dev == NULL) {
		dev_err(&client->dev,
		"not enough memory for input devices structure\n");
		retval = -ENOMEM;
		goto err_destroy_worqueue;
	}

	plat_dat->input_dev->name = "Austria Microsystem as5011 joystick";
	plat_dat->input_dev->uniq = "Austria0";
	plat_dat->input_dev->id.bustype = BUS_I2C;
	plat_dat->input_dev->phys = NULL;
	plat_dat->input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	plat_dat->input_dev->keybit[BIT_WORD(BTN_JOYSTICK)] =
					BIT_MASK(BTN_JOYSTICK);

	input_set_abs_params(plat_dat->input_dev,
			     ABS_X,
			     AS5011_MIN_AXIS,
			     AS5011_MAX_AXIS,
			     AS5011_FUZZ,
			     AS5011_FLAT);
	input_set_abs_params(plat_dat->input_dev,
			     ABS_Y,
			     AS5011_MIN_AXIS,
			     AS5011_MAX_AXIS,
			     AS5011_FUZZ,
			     AS5011_FLAT);

	plat_dat->button_irq_name = kmalloc(SIZE_NAME, GFP_KERNEL);
	if (plat_dat->button_irq_name == NULL) {
		dev_err(&client->dev,
		"not enough memory for input devices irq name\n");
		retval = -ENOMEM;
		goto err_input_free_device;
	}

	scnprintf(plat_dat->button_irq_name,
		  SIZE_NAME,
		  "as5011_%d_button",
		  plat_dat->num);

	retval = request_irq(	plat_dat->button_irq,
				button_interrupt,
				0,
				plat_dat->button_irq_name,
				(void *)plat_dat);
	if (retval < 0) {
		dev_err(&client->dev, "Can't allocate irq %d\n",
			plat_dat->button_irq);
		retval = -EBUSY;
		goto err_free_button_irq_name;
	}

	plat_dat->int_irq_name = kmalloc(SIZE_NAME, GFP_KERNEL);
	if (plat_dat->int_irq_name == NULL) {
		dev_err(&client->dev,
		"not enough memory for input devices int irq name\n");
		retval = -ENOMEM;
		goto err_free_irq_button_interrupt;
	}

	scnprintf(plat_dat->int_irq_name,
		  SIZE_NAME,
		  "as5011_%d_int",
		  plat_dat->num);

	if (request_irq(plat_dat->int_irq, as5011_int_interrupt,
			0, plat_dat->int_irq_name, (void *)plat_dat)) {
		dev_err(&client->dev, "Can't allocate int irq %d\n",
			plat_dat->int_irq);
		retval = -EBUSY;
		goto err_free_int_irq_name;
	}


	if (plat_dat->init_gpio != NULL) {
		retval = plat_dat->init_gpio();
		if (retval < 0) {
			dev_err(&client->dev, "Failed to init gpios\n");
			goto err_free_irq_int;
		}
	}

	/* chip soft reset */
	retval = as5011_i2c_write(plat_dat->i2c_client,
				  AS5011_CTRL1,
				  AS5011_CTRL1_SOFT_RST);
	if (retval < 0) {
		dev_err(&plat_dat->i2c_client->dev,
			"Soft reset failed\n");
		goto err_exit_gpio;
	}

	retval = as5011_i2c_write(plat_dat->i2c_client,
				  AS5011_CTRL1,
				  AS5011_CTRL1_LP_PULSED |
				  AS5011_CTRL1_LP_ACTIVE |
				  AS5011_CTRL1_INT_ACT_EN
				  );
	if (retval < 0) {
		dev_err(&plat_dat->i2c_client->dev,
			"Power config failed\n");
		goto err_exit_gpio;
	}

	retval = as5011_i2c_write(plat_dat->i2c_client,
				  AS5011_CTRL2,
				  AS5011_CTRL2_INV_SPINNING);
	if (retval < 0) {
		dev_err(&plat_dat->i2c_client->dev,
			"Can't invert spinning\n");
		goto err_exit_gpio;
	}

	/* write threshold */
	retval = as5011_i2c_write(plat_dat->i2c_client,
				  AS5011_XP,
				  plat_dat->Xp);
	if (retval < 0) {
		dev_err(&plat_dat->i2c_client->dev,
			"Can't write threshold\n");
		goto err_exit_gpio;
	}

	retval = as5011_i2c_write(plat_dat->i2c_client,
				  AS5011_XN,
				  plat_dat->Xn);
	if (retval < 0) {
		dev_err(&plat_dat->i2c_client->dev,
			"Can't write threshold\n");
		goto err_exit_gpio;
	}

	retval = as5011_i2c_write(plat_dat->i2c_client,
				  AS5011_YP,
				  plat_dat->Yp);
	if (retval < 0) {
		dev_err(&plat_dat->i2c_client->dev,
			"Can't write threshold\n");
		goto err_exit_gpio;
	}

	retval = as5011_i2c_write(plat_dat->i2c_client,
				  AS5011_YN,
				  plat_dat->Yn);
	if (retval < 0) {
		dev_err(&plat_dat->i2c_client->dev,
			"Can't write threshold\n");
		goto err_exit_gpio;
	}


	/* to free irq gpio in chip*/
	as5011_i2c_read(plat_dat->i2c_client, AS5011_X_RES_INT);

	retval = input_register_device(plat_dat->input_dev);
	if (retval) {
		dev_err(&client->dev, "Failed to register device\n");
		goto err_exit_gpio;
	}

	return 0;

	/* Error management */
err_unregister_input_dev:
	input_unregister_device(plat_dat->input_dev);
err_exit_gpio:
	if (plat_dat->exit_gpio != NULL)
		plat_dat->exit_gpio();
err_free_irq_int:
	free_irq(plat_dat->int_irq, as5011_int_interrupt);
err_free_int_irq_name:
	kfree(plat_dat->int_irq_name);
err_free_irq_button_interrupt:
	free_irq(plat_dat->button_irq, button_interrupt);
err_free_button_irq_name:
	kfree(plat_dat->button_irq_name);
err_input_free_device:
	input_free_device(plat_dat->input_dev);
err_destroy_worqueue:
	destroy_workqueue(plat_dat->workqueue);
err_out:
	return retval;
}
static int as5011_remove(struct i2c_client *client)
{
	struct as5011_platform_data *plat_dat = client->dev.platform_data;

	destroy_workqueue(plat_dat->workqueue);
	input_unregister_device(plat_dat->input_dev);
	free_irq(plat_dat->button_irq, plat_dat);
	free_irq(plat_dat->int_irq, plat_dat);
	kfree(plat_dat->button_irq_name);
	if (plat_dat->exit_gpio != NULL)
		plat_dat->exit_gpio();

	return 0;
}
static const struct i2c_device_id as5011_id[] = {
	{ "as5011", 0 },
	{ }
};

static struct i2c_driver as5011_driver = {
	.driver = {
		.name = "as5011",
	},
	.probe		= as5011_probe,
	.remove		= __devexit_p(as5011_remove),
	.id_table	= as5011_id,
};

/*
 * Module initialization
 */

static int __init as5011_init(void)
{
	return i2c_add_driver(&as5011_driver);
}

static void __exit as5011_exit(void)
{
	i2c_del_driver(&as5011_driver);
}

module_init(as5011_init);
module_exit(as5011_exit);
