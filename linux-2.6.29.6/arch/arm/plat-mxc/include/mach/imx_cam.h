/*
    imx-cam.h - i.MX27 camera driver header file

    Copyright (C) 2003, Intel Corporation
    Copyright (C) 2008, Sascha Hauer <s.hauer@pengutronix.de>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#ifndef __ASM_ARCH_CAMERA_H_
#define __ASM_ARCH_CAMERA_H_

#define MX27_CAMERA_SWAP16		(1 << 0)
#define MX27_CAMERA_EXT_VSYNC		(1 << 1)
#define MX27_CAMERA_CCIR		(1 << 2)
#define MX27_CAMERA_CCIR_INTERLACE	(1 << 3)
#define MX27_CAMERA_HSYNC_HIGH		(1 << 4)
#define MX27_CAMERA_GATED_CLOCK		(1 << 5)
#define MX27_CAMERA_INV_DATA		(1 << 6)
#define MX27_CAMERA_PCLK_SAMPLE_RISING	(1 << 7)
#define MX27_CAMERA_PACK_DIR_MSB	(1 << 8)

struct mx27_camera_platform_data {
	int (*init)(struct platform_device *);
	int (*exit)(struct platform_device *);

	unsigned long flags;

	unsigned long clk;
};

extern int mx27_init_camera(struct mx27_camera_platform_data *);

#endif /* __ASM_ARCH_CAMERA_H_ */

