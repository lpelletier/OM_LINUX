#ifndef _AS5011_H
#define _AS5011_H

/*
 * Copyright (c) 2010 Fabien Marteau
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#define AS5011_MAX_NAME_LENGTH	64
#define AS5011_MAX_CNAME_LENGTH	16
#define AS5011_MAX_PHYS_LENGTH	64
#define AS5011_MAX_LENGTH	64

/* registers */
#define AS5011_CTRL1		0x76
#define AS5011_CTRL2		0x75
#define AS5011_XP		0x43
#define AS5011_XN		0x44
#define AS5011_YP		0x53
#define AS5011_YN		0x54
#define AS5011_X_REG		0x41
#define AS5011_Y_REG		0x42
#define AS5011_X_RES_INT	0x51
#define AS5011_Y_RES_INT	0x52

/* CTRL1 bits */
#define AS5011_CTRL1_LP_PULSED		0x80
#define AS5011_CTRL1_LP_ACTIVE		0x40
#define AS5011_CTRL1_LP_CONTINUE	0x20
#define AS5011_CTRL1_INT_WUP_EN		0x10
#define AS5011_CTRL1_INT_ACT_EN		0x08
#define AS5011_CTRL1_EXT_CLK_EN		0x04
#define AS5011_CTRL1_SOFT_RST		0x02
#define AS5011_CTRL1_DATA_VALID		0x01

/* CTRL2 bits */
#define AS5011_CTRL2_EXT_SAMPLE_EN	0x08
#define AS5011_CTRL2_RC_BIAS_ON		0x04
#define AS5011_CTRL2_INV_SPINNING	0x02


struct as5011_platform_data {
	/* public */
	int button_gpio;
	int button_irq;
	int int_gpio;
	int int_irq;
	char Xp, Xn; /* threshold for x axis */
	char Yp, Yn; /* threshold for y axis */

	int (*init_gpio)(void); /* init interrupts gpios */
	void (*exit_gpio)(void);/* exit gpios */

	/* private */
	int num;
	struct input_dev *input_dev;
	struct i2c_client *i2c_client;
	unsigned char *button_irq_name;
	unsigned char *int_irq_name;
	char name[AS5011_MAX_NAME_LENGTH];
	char cname[AS5011_MAX_CNAME_LENGTH];
	char phys[AS5011_MAX_PHYS_LENGTH];
	unsigned char data[AS5011_MAX_LENGTH];
	char workqueue_name[AS5011_MAX_NAME_LENGTH];
	struct workqueue_struct *workqueue;
	struct work_struct update_axes_work;
};

#endif /* _AS5011_H */
