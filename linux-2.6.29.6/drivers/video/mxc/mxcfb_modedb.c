/*
 * Copyright 2006-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2008 Juergen Beisert
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

#include <linux/kernel.h>
#include <mach/imx_fb.h>

const struct fb_videomode mxcfb_modedb[] = {
	[0] = {
	       /*
		* 240x320 @ 60 Hz
		* Colour bits: 0 dark, 0x3f full
		* Clock: rising edge latches
		* DE: high active?
		* Vsync (FLM): high active
		* Hsync (LP): high active
		*/
		.name = "Sharp-QVGA",
		.refresh = 60,
		.xres = 240,
		.yres =320,
		.pixclock = 185925,
		.left_margin = 9, .right_margin = 16,
		.upper_margin = 7, .lower_margin = 9,
		.hsync_len = 1, .vsync_len = 1,
		.sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT |
			FB_SYNC_SHARP_MODE | FB_SYNC_CLK_IDLE_EN,
		.vmode = FB_VMODE_NONINTERLACED,
		.flag = 0,
	       },
	[1] = {
	       /* 640x480 @ 60 Hz */
	       "NEC-VGA", 60, 640, 480, 38255, 144, 0, 34, 40, 1, 1,
	       FB_SYNC_VERT_HIGH_ACT | FB_SYNC_OE_ACT_HIGH,
	       FB_VMODE_NONINTERLACED,
	       0,
	       },
	[2] = {
	       /* NTSC TV output */
	       "TV-NTSC", 60, 640, 480, 37538,
	       38, 858 - 640 - 38 - 3,
	       36, 518 - 480 - 36 - 1,
	       3, 1,
	       0,
	       FB_VMODE_NONINTERLACED,
	       0,
	       },
	[3] = {
	       /* PAL TV output */
	       "TV-PAL", 50, 640, 480, 37538,
	       38, 960 - 640 - 38 - 32,
	       32, 555 - 480 - 32 - 3,
	       32, 3,
	       0,
	       FB_VMODE_NONINTERLACED,
	       0,
	       },
	[4] = {
	       /* TV output VGA mode, 640x480 @ 65 Hz */
	       "TV-VGA", 60, 640, 480, 40574, 35, 45, 9, 1, 46, 5,
	       0, FB_VMODE_NONINTERLACED, 0,
	       },
	{
		/* PSP TFT */
		.name = "Sharp-LQ043",
		.refresh = 60,
		.xres = 480,
		.yres = 272,
		.pixclock = 111111, /* picoS */
		.left_margin = 3, .right_margin = 2, /* at least 3 & 1 */
		.upper_margin = 3, .lower_margin = 2,
		.hsync_len = 41, .vsync_len = 10,
		.sync = FB_SYNC_CLK_IDLE_EN | FB_SYNC_CLK_INVERT,
		.vmode = FB_VMODE_NONINTERLACED,
		.flag = 0,
		},
        {
		.name = "Sharp-LQ057",
		.refresh = 60,
		.xres = 320,
		.yres = 240,
		.pixclock = 158730, /* picoS -> 6,3MHz */
		.left_margin = 52, .right_margin = 28, /* at least 3 & 1 */ /* 320 + 52 + 28 = 400 */
		.upper_margin = 3, .lower_margin = 3, /* 240 + 5 + 3 + 3 = 251 */
		.hsync_len = 96, .vsync_len = 5,
		.sync = FB_SYNC_CLK_IDLE_EN | FB_SYNC_OE_ACT_HIGH | FB_SYNC_CLK_INVERT,
		.vmode = FB_VMODE_NONINTERLACED,
		.flag = 0,
	},
        {
		.name = "Chimei-LW700AT9003",
		.refresh = 60,
		.xres = 800,
		.yres = 480,
		.pixclock = 30303, /* picoS */
		.left_margin = 96, .right_margin = 96, /* at least 3 & 1 */
		.upper_margin = 0x14, .lower_margin = 0x15,
		.hsync_len = 64, .vsync_len = 4,
		.sync = FB_SYNC_CLK_IDLE_EN | FB_SYNC_OE_ACT_HIGH | FB_SYNC_CLK_INVERT,
		.vmode = FB_VMODE_NONINTERLACED,
		.flag = 0,
	},
        {
		.name = "Optrex-T51638D084",
		.refresh = 60,
		.xres = 640,
		.yres = 480,
		.pixclock = 40000, /* picoS */
		.left_margin = 48, .right_margin = 144, /* at least 3 & 1 */
		.upper_margin = 10, .lower_margin = 35,
		.hsync_len = 64, .vsync_len = 2,
		.sync = FB_SYNC_CLK_IDLE_EN | FB_SYNC_OE_ACT_HIGH | FB_SYNC_CLK_INVERT,
		.vmode = FB_VMODE_NONINTERLACED,
		.flag = 0,
	},
        {
		.name = "Hitachi-TX12D17VM1BDP",
		.refresh = 60,
		.xres = 640,
		.yres = 480,
		.pixclock = 40000, /* picoS */
		.left_margin = 80, .right_margin = 48, /* at least 3 & 1 */
		.upper_margin = 11, .lower_margin = 34,
		.hsync_len = 64, .vsync_len = 2,
		.sync = FB_SYNC_CLK_IDLE_EN | FB_SYNC_OE_ACT_HIGH | FB_SYNC_CLK_INVERT,
		.vmode = FB_VMODE_NONINTERLACED,
		.flag = 0,
	},
	/* Please let this LCD always in last position: */
	{
		.name = "Custom", /* Custom LCD, freely usable (clone of LQ043 by default */
		.refresh = 60,
		.xres = 480,
		.yres = 272,
		.pixclock = 111111, /* picoS */
		.left_margin = 3, .right_margin = 2, /* at least 3 & 1 */
		.upper_margin = 3, .lower_margin = 2,
		.hsync_len = 41, .vsync_len = 10,
		.sync = FB_SYNC_CLK_IDLE_EN | FB_SYNC_CLK_INVERT,
		.vmode = FB_VMODE_NONINTERLACED,
		.flag = 0,
	},
	/* Please let these 2 lines here
	*/
};

int mxcfb_modedb_sz = ARRAY_SIZE(mxcfb_modedb);
EXPORT_SYMBOL(mxcfb_modedb);
EXPORT_SYMBOL(mxcfb_modedb_sz);
