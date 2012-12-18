/*
 * cpu.c: clock scaling for the iMX
 *
 * Copyright (C) 2000 2001, The Delft University of Technology
 * Copyright (c) 2004 Sascha Hauer <sascha@saschahauer.de>
 * Copyright (C) 2006 Inky Lung <ilung@cwlinux.com>
 * Copyright (C) 2006 Pavel Pisa, PiKRON <ppisa@pikron.com>
 * Copyright (C) 2008 Juergen Beisert, <kernel@pengutronix.de>
 *
 * Based on SA1100 version written by:
 * - Johan Pouwelse (J.A.Pouwelse@its.tudelft.nl): initial version
 * - Erik Mouw (J.A.K.Mouw@its.tudelft.nl):
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/cpufreq.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <asm/system.h>
#include <mach/clock.h>

#include "cpufreq_imx.h"

static struct clk *cpu_clk;
static struct cpufreq_platform_data *platform;

static int imx_verify_speed(struct cpufreq_policy *policy)
{

	if (policy->cpu != 0)
		return -EINVAL;

	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
					policy->cpuinfo.max_freq);

	return 0;
}

static unsigned int imx_get_speed(unsigned int cpu)
{
	if (cpu)
		return 0;

	return clk_get_rate(cpu_clk) / 1000;
}

static unsigned int calc_frequency(int target, unsigned int relation)
{
	int i;

	if (relation == CPUFREQ_RELATION_H) {
		for (i = platform->freq_entries - 1; i > 0; i--) {
			if (platform->freq_table[i].frequency <= target)
				return platform->freq_table[i].frequency;
		}
	} else if (relation == CPUFREQ_RELATION_L) {
		for (i = 0; i < platform->freq_entries - 1; i++) {
			if (platform->freq_table[i].frequency >= target)
				return platform->freq_table[i].frequency;
		}
	}

	return 400000;
}

static int imx_set_target(struct cpufreq_policy *policy,
			  unsigned int target_freq,
			  unsigned int relation)
{
	struct cpufreq_freqs freqs;
	int ret;

	/*
	 * Some governors do not respects CPU and policy lower limits
	 * which leads to bad things (division by zero etc), ensure
	 * that such things do not happen.
	 */
	if (target_freq < policy->cpuinfo.min_freq)
		target_freq = policy->cpuinfo.min_freq;

	if (target_freq < policy->min)
		target_freq = policy->min;

	freqs.old = imx_get_speed(0);
	freqs.new = calc_frequency(target_freq, relation);;
	freqs.cpu = 0;
	freqs.flags = 0;

	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
	ret = (platform->transit)(freqs.new);
	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	return ret;
}

static int __init imx_cpufreq_driver_init(struct cpufreq_policy *policy)
{
	int ret;

	if (policy->cpu != 0)
		return -EINVAL;

	policy->cur = policy->min = policy->max = clk_get_rate(cpu_clk) / 1000;
	policy->governor = CPUFREQ_DEFAULT_GOVERNOR;
	policy->cpuinfo.min_freq = policy->cpuinfo.max_freq =
			platform->freq_table[0].frequency;
	if (platform->freq_entries > 2)
		policy->cpuinfo.max_freq =
		platform->freq_table[platform->freq_entries-2].frequency;

	/* Manual states, that PLL stabilizes in two CLK32 periods */
	/* TODO: This time depends on the mechanic the BSP uses */
	policy->cpuinfo.transition_latency = 4 * 1000000000LL / 32768;

	ret = cpufreq_frequency_table_cpuinfo(policy, platform->freq_table);
	if (ret < 0) {
		clk_put(cpu_clk);
		return ret;
	}
	cpufreq_frequency_table_get_attr(platform->freq_table, policy->cpu);

	return 0;
}

static int __devexit imx_cpufreq_driver_exit(struct cpufreq_policy *policy)
{
	cpufreq_frequency_table_put_attr(policy->cpu);
	return 0;
}

static struct cpufreq_driver imx_driver = {
	.flags	= CPUFREQ_STICKY,
	.verify	= imx_verify_speed,
	.target	= imx_set_target,
	.get	= imx_get_speed,
	.init	= imx_cpufreq_driver_init,
	.exit	= __devexit_p(imx_cpufreq_driver_exit),
	.name	= "imx2x",
};

static int __init imx_cpufreq_driver_probe(struct platform_device *pdev)
{
	struct cpufreq_platform_data *pdata = pdev->dev.platform_data;
	int ret;

	printk(KERN_INFO "i.MX cpu freq change driver v0.1\n");

	if (pdata == NULL || pdata->transit == NULL) {
		printk(KERN_ERR"Cannot run CPU frequency scaling without boundary data\n");
		return -ENODEV;
	}

	cpu_clk = clk_get(NULL, "cpu_clk");
	if (IS_ERR(cpu_clk))
		return PTR_ERR(cpu_clk);

	if (pdata->init) {
		ret = (pdata->init)(pdev);
		if (ret != 0)
			return ret;
	}
	platform = pdata;

	return cpufreq_register_driver(&imx_driver);
}

static int __devexit imx_cpufreq_driver_remove(struct platform_device *pdev)
{
	struct cpufreq_platform_data *pdata = pdev->dev.platform_data;

	cpufreq_unregister_driver(&imx_driver);

	if (pdata->exit)
		(pdata->exit)(pdev);

	clk_put(cpu_clk);
	return 0;
}

static struct platform_driver imx_cpufreq_driver = {
	.driver = {
		.name = "imx_cpufreq",
		.owner = THIS_MODULE,
	},
	.probe = imx_cpufreq_driver_probe,
	.remove = __devexit_p(imx_cpufreq_driver_remove),
	.suspend = NULL,	/* TODO */
	.resume = NULL,		/* TODO */
};

static int __init imx_cpufreq_init(void)
{
	return platform_driver_register(&imx_cpufreq_driver);
}

static void __exit imx_cpufreq_exit(void)
{
	platform_driver_unregister(&imx_cpufreq_driver);
}

module_init(imx_cpufreq_init);
module_exit(imx_cpufreq_exit);

MODULE_AUTHOR("Juergen Beisert <kernel@pengutronix.de>");
MODULE_DESCRIPTION("CPUfreq driver for i.mx2x");
MODULE_LICENSE("GPL");
