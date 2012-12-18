/*
 * Driver for the i.MX/MXC matrix keyboard controller.
 *
 * Copyright (c) 2009 by Holger Schurig <hs4233@mail.mn-solutions.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*

This driver is just a "generic" framework, it cannot produce input events on
it's own. That's because diverse board/devices have very different
keyboards (numeric, alphanumeric, cell phone like etc).

Therefore, you'll do the actual scancode -> input event code conversion
in a function that you'll provide in your platform data:

static struct mxc_keypad_platform_data tt8000_keypad_data = {
	.output_pins = 0x0f00,
	.input_pins  = 0x001f,
	.init        = tt8000_keypad_init,
	.exit        = tt8000_keypad_exit,
	.handle_key  = tt8000_keypad_key,
};

static void tt8000_keypad_init(struct input_dev *input_dev,
	struct platform_device *pdev);

This function is called before the input device registers, it can setup
primary/alternate GPIO functions and modify it, e.g. by setting
the propriate bits in input_dev->keybit.

static void tt8000_keypad_exit(struct platform_device *pdev);

The opposite function.

static void tt8000_keypad_key(struct input_dev *input_dev,
	int col, int row, int down);

Called for each key press or release event. Should determine
the input event code and sent it via input_report_key().

*/


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/hardware.h>
#include <mach/mxc_keypad.h>

#define KPCR		0x00

#define KPSR		0x02
#define KPSR_KPP_EN	(1 << 10)
#define KPSR_KRIE	(1 << 9)
#define KPSR_KDIE	(1 << 8)
#define KPSR_KRSS	(1 << 3)
#define KPSR_KDSC	(1 << 2)
#define KPSR_KPKR	(1 << 1)
#define KPSR_KPKD	(1 << 0)
#define KPSR_INT_MASK	(KPSR_KRIE|KPSR_KDIE)

#define KDDR		0x04

#define KPDR		0x06


#define MAX_INPUTS	8
#define MAX_OUTPUTS	8


//static void mxc_keypad_tasklet(unsigned long);

struct mxc_keypad {
	struct mxc_keypad_platform_data *pdata;
	struct resource *res;
	int irq;
	void __iomem *base;

	struct clk *clk;
	struct input_dev *input_dev;
	struct timer_list timer;
	struct tasklet_struct tasklet;

	u16 keystate_prev[MAX_INPUTS];
};


static void mxc_keypad_scan(struct mxc_keypad *kp)
{
	u16 col_bit, reg;
	int i, j;
	int snum = 0;
	u16 keystate_cur[MAX_INPUTS];
	u16 keystate_xor[MAX_INPUTS];
	int modifierkey_held = 0;

	/* write ones to KPDR8:15, setting columns data */
	__raw_writew(kp->pdata->output_pins, kp->base + KPDR);

	/* Quick-discharge by setting to totem-pole, */
	/* then turn back to open-drain */
	__raw_writew(kp->pdata->input_pins, kp->base + KPCR);
	wmb();
	udelay(2);
	__raw_writew(kp->pdata->output_pins |
		kp->pdata->input_pins, kp->base + KPCR);

	col_bit = 1;
	while (col_bit) {
		if (col_bit & kp->pdata->output_pins) {
			u16 last_reg = 0;
			int debounce = 0;

			__raw_writew(~col_bit, kp->base + KPDR);
			wmb();

			while (debounce < 5) {
				udelay(1);
				reg = ~__raw_readw(kp->base + KPDR);
				if (reg == last_reg)
					debounce++;
				else
					debounce = 0;
				last_reg = reg;
			}
			keystate_cur[snum] = reg & kp->pdata->input_pins;

			if (snum++ >= MAX_INPUTS)
				break;
		}
		col_bit <<= 1;
	}
	for (i = 0; i < snum; i++) {
		keystate_xor[i] = kp->keystate_prev[i] ^ keystate_cur[i];
		for (j = 0; j < MAX_OUTPUTS; j++) {
			int down = !!(keystate_cur[i] & (1 << j));

			if (down && kp->pdata->is_modifier_key &&
					kp->pdata->is_modifier_key(i, j))
				modifierkey_held = 1;

			if (keystate_xor[i] & (1 << j)) {
				pr_debug("i,j: %d,%d, down %d\n", i, j, down);
				kp->pdata->handle_key(kp->input_dev,
					i, j, down);
			}
		}
		kp->keystate_prev[i] = keystate_cur[i];
	}

	/* set columns back to 0 */
	__raw_writew(0, kp->base + KPDR);

	/*
	 * Clear KPKD and KPKR status bit(s) by writing to a "1",
	 * set the KPKR synchronizer chain by writing "1" to KRSS register,
	 * clear the KPKD synchronizer chain by writing "1" to KDSC register
	 */
	reg = __raw_readw(kp->base + KPSR);
	reg |= KPSR_KPKD | KPSR_KPKR | KPSR_KRSS | KPSR_KDSC;

	if (modifierkey_held) {
		/* poll by timer to avoid too many interrupts while key held */
		mod_timer(&kp->timer, jiffies + HZ / 20);
		//reg |= KPSR_INT_MASK; /* detect presses and releases */
	}
	__raw_writew(reg, kp->base + KPSR);
}

static void mxc_keypad_tasklet(unsigned long data)
{
	struct mxc_keypad *kp = (struct mxc_keypad *) data;

	mxc_keypad_scan(kp);
}

static irqreturn_t mxc_keypad_irq_handler(int irq, void *data)
{
	struct mxc_keypad *kp = data;
	u16 stat;

	stat = __raw_readw(kp->base + KPSR);
	stat &= ~KPSR_INT_MASK;

	if (stat & KPSR_KPKD)
		stat |= KPSR_KRIE; /* pressed - interrupt on next release */

	if (stat & KPSR_KPKR)
		stat |= KPSR_KDIE; /* all released - interrupt on next press */

	/* clear interrupt status */
	stat |= KPSR_KPKD;
	stat |= KPSR_KPKR;
	__raw_writew(stat | KPSR_KPKD | KPSR_KPKR, kp->base + KPSR);

	tasklet_schedule(&kp->tasklet);

	pr_debug("KPCR: %04x (0f1f)\n", readw(kp->base + KPCR));
	pr_debug("KPSR: %04x (0502)\n", readw(kp->base + KPSR));
	pr_debug("KDDR: %04x (0f00)\n", readw(kp->base + KDDR));
	pr_debug("KPDR: %04x (705f)\n", readw(kp->base + KPDR));

	return IRQ_HANDLED;
}

static void mxc_keypad_timer(unsigned long data)
{
	struct mxc_keypad *kp = (struct mxc_keypad *) data;

	tasklet_schedule(&kp->tasklet);
}


static int __devinit mxc_keypad_probe(struct platform_device *pdev)
{
	struct mxc_keypad *kp;
	struct input_dev *input_dev;
	struct mxc_keypad_platform_data *pdata;
	struct resource *res;
	struct clk *clk;
	int irq, error;

	pdata = pdev->dev.platform_data;
	if (!pdata || !pdata->handle_key)
		return -EINVAL;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (!res || !irq)
		return -ENXIO;

	res = request_mem_region(res->start, resource_size(res), pdev->name);
	if (!res)
		return -EBUSY;

	kp = kzalloc(sizeof(struct mxc_keypad), GFP_KERNEL);
	if (!kp) {
		error = -ENOMEM;
		goto failed_release;
	}

	kp->res = res;
	kp->pdata = pdata;

	/* Create and register the input driver. */
	input_dev = kp->input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&pdev->dev, "input_allocate_device\n");
		error = -ENOMEM;
		goto failed_put_clk;
	}

	kp->base = ioremap(res->start, resource_size(res));
	if (!kp->base) {
		dev_err(&pdev->dev, "ioremap\n");
		error = -ENOMEM;
		goto failed_free_dev;
	}

	clk = clk_get(&pdev->dev, "kpp_clk");
	if (IS_ERR(clk)) {
		error = PTR_ERR(clk);
		goto failed_unmap;
	}

	input_dev->name = pdev->name;
	input_dev->dev.parent = &pdev->dev;
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;
	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REP);
	input_set_drvdata(input_dev, kp);

	platform_set_drvdata(pdev, kp);

	/* should call input_set_capability(input_dev, EV_KEY, code); */
	if (kp->pdata->init)
		kp->pdata->init(input_dev, pdev);

	setup_timer(&kp->timer, mxc_keypad_timer, (unsigned long)kp);
	tasklet_init(&kp->tasklet, mxc_keypad_tasklet, (unsigned long)kp);

	error = request_irq(irq, mxc_keypad_irq_handler,
		IRQF_DISABLED, pdev->name, kp);
	if (error) {
		dev_err(&pdev->dev, "request_irq\n");
		goto failed_clk_put;
	}

	kp->irq = irq;

	/* Register the input device */
	error = input_register_device(input_dev);
	if (error) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto failed_free_irq;
	}

	device_init_wakeup(&pdev->dev, 1);

	/* configure keypad */
	clk_enable(clk);
	__raw_writew(kp->pdata->output_pins |
		kp->pdata->input_pins, kp->base + KPCR);
	__raw_writew(0, kp->base + KPDR);
	__raw_writew(kp->pdata->output_pins, kp->base + KDDR);
	__raw_writew(KPSR_KPP_EN | KPSR_KDIE |
		KPSR_KPKD | KPSR_KRSS | KPSR_KDSC, kp->base + KPSR);

	return 0;

failed_free_irq:
	free_irq(irq, kp);
	platform_set_drvdata(pdev, NULL);
failed_clk_put:
	clk_disable(clk);
	clk_put(clk);
failed_unmap:
	iounmap(kp->base);
failed_free_dev:
	input_free_device(input_dev);
failed_put_clk:
	kfree(kp);
failed_release:
	release_mem_region(res->start, resource_size(res));
	return error;
}

static int __devexit mxc_keypad_remove(struct platform_device *pdev)
{
	struct mxc_keypad *kp = platform_get_drvdata(pdev);

	device_init_wakeup(&pdev->dev, 0);

	if (kp) {
		/* Mask interrupts */
		u16 stat = __raw_readw(kp->base + KPSR);
		stat &= ~KPSR_INT_MASK;
		__raw_writew(stat | KPSR_KPKD | KPSR_KPKR, kp->base + KPSR);

		free_irq(kp->irq, kp);
		tasklet_disable(&kp->tasklet);
		del_timer_sync(&kp->timer);

		if (kp->pdata->exit)
			kp->pdata->exit(pdev);

		if (kp->base)
			iounmap(kp->base);

		clk_disable(kp->clk);
		clk_put(kp->clk);
		tasklet_kill(&kp->tasklet);

		input_unregister_device(kp->input_dev);
		release_mem_region(kp->res->start, resource_size(kp->res));
	}

	platform_set_drvdata(pdev, NULL);
	kfree(kp);
	return 0;
}

/* work with hotplug and coldplug */
MODULE_ALIAS("platform:mxc-keypad");

static struct platform_driver mxc_keypad_driver = {
	.probe		= mxc_keypad_probe,
	.remove		= __devexit_p(mxc_keypad_remove),
	.driver		= {
		.name	= "mxc-keypad",
		.owner	= THIS_MODULE,
	},
};

static int __init mxc_keypad_init(void)
{
	return platform_driver_register(&mxc_keypad_driver);
}

static void __exit mxc_keypad_exit(void)
{
	platform_driver_unregister(&mxc_keypad_driver);
}

module_init(mxc_keypad_init);
module_exit(mxc_keypad_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("i.MX Keypad Driver");
MODULE_AUTHOR("Holger Schurig <hs4233@@mail.mn-solutions.de>");
