/*
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

#ifndef __SOUND_ARM_IMX_SOUND_H
#define __SOUND_ARM_IMX_SOUND_H

#define IMX_SUPPORTED_SSI_UNITS 2

struct imx_sound_connection {
	int cpu_port;	/* from 1 to 4 on i.MX27, 0 unused */
	int dev_port;	/* I2S device specific. 1 to 2 for PMIC MC13783, 0 unused */
};

struct imx_sound_platform_data {
	struct imx_sound_connection connection[IMX_SUPPORTED_SSI_UNITS];
	int (*init)(struct platform_device *pdev);
	void (*exit)(struct platform_device *pdev);
};

#endif
