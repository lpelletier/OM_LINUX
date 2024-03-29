/*
 * linux/arch/arm/mach-mx2/pm.c
 *
 * i.MX2x Power Management Routines
 *
 * Original code for the SA11x0:
 * Copyright (c) 2001 Cliff Brake <cbrake@accelent.com>
 *
 * Modified for the PXA250 by Nicolas Pitre:
 * Copyright (c) 2002 Monta Vista Software, Inc.
 *
 * Modified for the OMAP1510 by David Singleton:
 * Copyright (c) 2002 Monta Vista Software, Inc.
 *
 * Cleanup 2004 for OMAP1510/1610 by Dirk Behme <dirk.behme@de.bosch.com>
 *
 * Modified for the i.MX27
 * Copyright 2007 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * Modified for i.MX2x by Julien Boibessot <julien.boibessot@armadeus.com>
 * Copyright (c) 2009 Armadeus systems
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 */

#define DEBUG

#include <linux/pm.h>
#include <linux/suspend.h>
#include <asm/io.h>
// #include <asm/arch/mxc_pm.h>
#include <mach/system.h> /* arch_idle() */
#include <mach/hardware.h>


#define DOZE_MODE 0
#define WAIT_MODE 1
#define STOP_MODE 2
#define DSM_MODE  3

#define CCM_CSCR                (IO_ADDRESS(CCM_BASE_ADDR) + 0x0)

static void mx2_pm_lowpower(u32 mode)
{
	u32 cscr;

	local_irq_disable();

	/* WAIT and DOZE execute WFI only */
	switch (mode) {
	case STOP_MODE:
	case DSM_MODE:
		/* Clear MPEN and SPEN to disable MPLL/SPLL */
		cscr = __raw_readl(CCM_CSCR);
		cscr &= 0xFFFFFFFC;
		__raw_writel(cscr, CCM_CSCR);
		break;
	}

	/* Executes WFI */
	arch_idle();

	local_irq_enable();
}


/* All the power modes we support */
static int mx2_pm_valid(suspend_state_t state)
{
	switch (state) {
		case PM_SUSPEND_ON:
		case PM_SUSPEND_STANDBY:
		case PM_SUSPEND_MEM:
			return 1;

		default:
			return 0;
	}
}

static int mx2_pm_enter(suspend_state_t state)
{
	pr_debug("%s\n", __func__);

	switch (state) {
	case PM_SUSPEND_MEM:
 		mx2_pm_lowpower(STOP_MODE);
		break;

	case PM_SUSPEND_STANDBY:
 		mx2_pm_lowpower(WAIT_MODE);
		break;

	case PM_SUSPEND_ON:
		printk("PM_SUSPEND_ON\n");
		break;

	case PM_SUSPEND_MAX:
 		mx2_pm_lowpower(DSM_MODE);
		break;

	default:
		printk("unsupported suspend state: %d\n", state);
		return -1;
	}

	return 0;
}

/* Called after processes are frozen, but before we shut down devices. */
static int mx2_pm_prepare(void)
{
	pr_debug("%s\n", __func__);
	return 0;
}

/* Called after devices are re-setup, but before processes are thawed. */
static void mx2_pm_finish(void)
{
	pr_debug("%s: proc out of sleep\n", __func__);
}

/* Called right prior to thawing processes. */
static void mx2_pm_end(void)
{
	pr_debug("%s: all devices should be running now\n", __func__);
}

struct platform_suspend_ops mx2_pm_ops = {
	.valid	 = mx2_pm_valid,
	.prepare = mx2_pm_prepare,
	.enter   = mx2_pm_enter,
	.finish  = mx2_pm_finish,
	.end     = mx2_pm_end,
};

static int __init mx2_pm_init(void)
{
	printk("Power Management for Freescale i.MX2x\n");

	suspend_set_ops(&mx2_pm_ops);

	return 0;
}

device_initcall(mx2_pm_init);
