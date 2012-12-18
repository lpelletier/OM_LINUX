/*
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
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

#ifndef __ASM_ARCH_MXCFB_H__
#define __ASM_ARCH_MXCFB_H__

#include <linux/fb.h>
#include <linux/platform_device.h>

struct mxc_fb_platform_data {
	char *mode;

	int (*init)(struct platform_device *pdev);
	int (*exit)(struct platform_device *pdev);

	void (*power_backlight)(struct platform_device *pdev, int on);
	int (*power_lcd)(struct platform_device *pdev, int on);
};


int mxc_init_fb(struct mxc_fb_platform_data *pdata);

#define FB_SYNC_OE_ACT_HIGH	0x80000000
#define FB_SYNC_CLK_INVERT	0x40000000
#define FB_SYNC_DATA_INVERT	0x20000000
#define FB_SYNC_CLK_IDLE_EN	0x10000000
#define FB_SYNC_SHARP_MODE	0x08000000
#define FB_SYNC_SWAP_RGB	0x04000000

struct mxcfb_gbl_alpha {
	int enable;
	int alpha;
};

struct mxcfb_color_key {
	int enable;
	__u32 color_key;
};

#define MXCFB_WAIT_FOR_VSYNC	_IOW('F', 0x20, u_int32_t)
#define MXCFB_SET_GBL_ALPHA     _IOW('F', 0x21, struct mxcfb_gbl_alpha)
#define MXCFB_SET_CLR_KEY       _IOW('F', 0x22, struct mxcfb_color_key)
#define MXCFB_SET_BRIGHTNESS    _IOW('F', 0x23, __u8)

#ifdef __KERNEL__

extern const struct fb_videomode mxcfb_modedb[];
extern int mxcfb_modedb_sz;

enum {
	MXCFB_REFRESH_OFF,
	MXCFB_REFRESH_AUTO,
	MXCFB_REFRESH_PARTIAL,
};

struct mxcfb_rect {
	u32 top;
	u32 left;
	u32 width;
	u32 height;
};

int mxcfb_set_refresh_mode(struct fb_info *fbi, int mode,
			   struct mxcfb_rect *update_region);

#endif				/* __KERNEL__ */
#endif
