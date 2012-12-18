/*
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
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

#undef DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <asm/uaccess.h>
#include <mach/imx_fb.h>
#include <mach/hardware.h>

#include "mx2fb.h"

#define MX2FB_TYPE_BG          0
#define MX2FB_TYPE_GW          1

static char *fb_mode = 0;
static int fb_enabled = 0;
static unsigned long default_bpp = 16;
static unsigned char brightness = 255;
static ATOMIC_NOTIFIER_HEAD(mx2fb_notifier_list);
static struct clk *lcdc_clk;

/* Structure containing the MX2 specific framebuffer information */
struct mx2fb_info {
	int type;
	char *id;
	int registered;
	int blank;
	unsigned long pseudo_palette[16];

	struct mxc_fb_platform_data *pdata;
	struct platform_device *pdev;
};

/* Framebuffer APIs */
static int mx2fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info);
static int mx2fb_set_par(struct fb_info *info);
static int mx2fb_setcolreg(unsigned regno, unsigned red, unsigned green,
			   unsigned blue, unsigned transp,
			   struct fb_info *info);
static int mx2fb_pan_display(struct fb_var_screeninfo *var,
			     struct fb_info *info);
static int mx2fb_blank(int blank_mode, struct fb_info *info);
static int mx2fb_ioctl(struct fb_info *info, unsigned int cmd,
		       unsigned long arg);

/* Driver entries */
int __init mx2fb_init(void);
void __exit mx2fb_exit(void);
#ifndef MODULE
static int __init mx2fb_setup(char *);
#endif

/* Internal functions */
static int __init _init_fbinfo(struct fb_info *info,
			       struct platform_device *pdev);
static int __init _install_fb(struct fb_info *info,
			      struct platform_device *pdev);
static void __exit _uninstall_fb(struct fb_info *info);
static int _map_video_memory(struct fb_info *info);
static void _unmap_video_memory(struct fb_info *info);
static void _set_fix(struct fb_info *info);
static void _enable_lcdc(struct fb_info *info);
static void _disable_lcdc(struct fb_info *info);
static void _enable_graphic_window(struct fb_info *info);
static void _disable_graphic_window(struct fb_info *info);
static void _update_lcdc(struct fb_info *info);
static void _set_brightness(unsigned char level);
static void _request_irq(void);
static void _free_irq(void);

#ifdef CONFIG_PM
static int mx2fb_suspend(struct platform_device *pdev, pm_message_t state);
static int mx2fb_resume(struct platform_device *pdev);
#else
#define mx2fb_suspend	0
#define mx2fb_resume	0
#endif

static int mx2fb_probe(struct platform_device *pdev);

#ifdef CONFIG_FB_MXC_TVOUT
#error "not supported yet"
#include <linux/video_encoder.h>
/*
 * FIXME: VGA mode is not defined by video_encoder.h
 * while FS453 supports VGA output.
 */
#ifndef VIDEO_ENCODER_VGA
#define VIDEO_ENCODER_VGA	32
#endif

#define MODE_PAL		"TV-PAL"
#define MODE_NTSC		"TV-NTSC"
#define MODE_VGA		"TV-VGA"

extern int fs453_ioctl(unsigned int cmd, void *arg);
#endif

struct mx2fb_info mx2fbi_bg = {
	.type = MX2FB_TYPE_BG,
	.id = "DISP0 BG",
	.registered = 0,
};

#ifdef CONFIG_FB_MXC_OVERLAY
static struct mx2fb_info mx2fbi_gw = {
	.type = MX2FB_TYPE_GW,
	.id = "DISP0 FG",
	.registered = 0,
};
#endif

/* Current graphic window information */
static struct fb_gwinfo g_gwinfo = {
	.enabled = 0,
	.alpha_value = 255,
	.ck_enabled = 0,
	.ck_red = 0,
	.ck_green = 0,
	.ck_blue = 0,
	.xpos = 0,
	.ypos = 0,
};

/*
 * Framebuffer information structures.
 * There are up to 3 framebuffers: background, TVout, and graphic window.
 * If graphic window is configured, it must be the last framebuffer.
 */
static struct fb_info mx2fb_info[] = {
	{.par = &mx2fbi_bg},
#ifdef CONFIG_FB_MXC_OVERLAY
	{.par = &mx2fbi_gw},
#endif
};

/* callback functions for power management */
static struct platform_driver mx2fb_driver = {
	.driver = {
		   .name = "mxc_sdc_fb",
		   .owner = THIS_MODULE,
		   .bus = &platform_bus_type,
		   },
	.probe = mx2fb_probe,
	.suspend = mx2fb_suspend,
	.resume = mx2fb_resume,
};

/* Our exported framebuffer file operations */
static struct fb_ops mx2fb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = mx2fb_check_var,
	.fb_set_par = mx2fb_set_par,
	.fb_setcolreg = mx2fb_setcolreg,
	.fb_blank = mx2fb_blank,
	//.fb_pan_display = mx2fb_pan_display, double buffering not supported yet
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	//.fb_cursor = soft_cursor,
	.fb_ioctl = mx2fb_ioctl,
};

static int mx2fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	unsigned long htotal, vtotal;

	if (var->xres_virtual < var->xres)
		var->xres_virtual = var->xres;
	if (var->yres_virtual < var->yres)
		var->yres_virtual = var->yres;

	if (var->xoffset < 0)
		var->xoffset = 0;

	if (var->yoffset < 0)
		var->yoffset = 0;

	if (var->xoffset + info->var.xres > info->var.xres_virtual)
		var->xoffset = info->var.xres_virtual - info->var.xres;

	if (var->yoffset + info->var.yres > info->var.yres_virtual)
		var->yoffset = info->var.yres_virtual - info->var.yres;


	switch (var->bits_per_pixel) {
	case 16:
		var->red.length = 5;
		var->red.offset = 11;
		var->red.msb_right = 0;

		var->green.length = 6;
		var->green.offset = 5;
		var->green.msb_right = 0;

		var->blue.length = 5;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 0;
		var->transp.msb_right = 0;
		break;
	case 24:
		var->red.length = 8;
		var->red.offset = 16;
		var->red.msb_right = 0;

		var->green.length = 8;
		var->green.offset = 8;
		var->green.msb_right = 0;

		var->blue.length = 8;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 0;
		var->transp.msb_right = 0;
		break;
	case 32:
		var->red.length = 8;
		var->red.offset = 16;
		var->red.msb_right = 0;

		var->green.length = 8;
		var->green.offset = 8;
		var->green.msb_right = 0;

		var->blue.length = 8;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 8;
		var->transp.offset = 24;
		var->transp.msb_right = 0;
		break;
	default: /* 8bits */
		var->red.length = 8;
		var->red.offset = 0;
		var->red.msb_right = 0;

		var->green.length = 8;
		var->green.offset = 0;
		var->green.msb_right = 0;

		var->blue.length = 8;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 0;
		var->transp.msb_right = 0;
	break;
	}

	if (var->pixclock < 1000) {
		htotal = var->xres + var->right_margin + var->hsync_len +
		    var->left_margin;
		vtotal = var->yres + var->lower_margin + var->vsync_len +
		    var->upper_margin;
		var->pixclock = (vtotal * htotal * 6UL) / 100UL;
		var->pixclock = KHZ2PICOS(var->pixclock);
		dev_dbg(info->device,
			"pixclock set for 60Hz refresh = %u ps\n",
			var->pixclock);
	}

	var->height = -1;
	var->width = -1;
	var->grayscale = 0;

	/* Copy nonstd field to/from sync for fbset usage */
	var->sync |= var->nonstd;
	var->nonstd |= var->sync;

	return 0;
}

static int mx2fb_set_par(struct fb_info *info)
{
	unsigned long len;
	struct mx2fb_info *mx2fbi = (struct mx2fb_info *)info->par;

	_set_fix(info);

	len = info->var.yres_virtual * info->fix.line_length;
	if (len > info->fix.smem_len) {
		if (info->fix.smem_start)
			_unmap_video_memory(info);

		/* Memory allocation for framebuffer */
		if (_map_video_memory(info)) {
			dev_err(info->device, "Unable to allocate fb memory\n");
			return -ENOMEM;
		}
	}

	_update_lcdc(info);
	if (info->fbops->fb_blank)
		info->fbops->fb_blank(mx2fbi->blank, info);

	return 0;
}

#define CNVT_TOHW(val,width) ((((val)<<(width))+0x7FFF-(val))>>16)

static int mx2fb_setpalettereg(u_int regno, u_int red, u_int green, u_int blue,
		u_int trans, struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;
	u_int palette_size = var->bits_per_pixel == 8 ? 256 : 16;
	u_int val, ret = 1;
	unsigned int reg_size = 4;

	if (regno < palette_size) {
		reg_size = 6; /* MX2 and TFT */
		val = (CNVT_TOHW(red, reg_size) << (reg_size * 2)) |
		      (CNVT_TOHW(green, reg_size) << reg_size) |
		      CNVT_TOHW(blue, reg_size);

		__raw_writel(val, LCDC_REG(0x800 + (regno << 2)));
		ret = 0;
	}
	return ret;
}

/*
 * Set a single color register. The values supplied have a 16 bit magnitude
 * which needs to be scaled in this function for the hardware. Things to take
 * into consideration are how many color registers, if any, are supported with
 * the current color visual. With truecolor mode no color palettes are
 * supported. Here a psuedo palette is created which we store the value in
 * pseudo_palette in struct fb_info. For pseudocolor mode we have a limited
 * color palette.
 */
static int mx2fb_setcolreg(u_int regno, u_int red, u_int green,
			   u_int blue, u_int transp, struct fb_info *info)
{
	int ret = 1;

	/*
	 * If greyscale is true, then we convert the RGB value
	 * to greyscale no matter what visual we are using.
	 */
	if (info->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
				      7471 * blue) >> 16;
	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/*
		 * 16-bit True Colour.  We encode the RGB value
		 * according to the RGB bitfield information.
		 */
		if (regno < 16) {
			u32 *pal = info->pseudo_palette;
			u32 v;

			red = CNVT_TOHW(red, info->var.red.length);
			green = CNVT_TOHW(green, info->var.green.length);
			blue = CNVT_TOHW(blue, info->var.blue.length);
			transp = CNVT_TOHW(transp, info->var.transp.length);

			v = (red << info->var.red.offset) |
			    (green << info->var.green.offset) |
			    (blue << info->var.blue.offset) |
			    (transp << info->var.transp.offset);

			pal[regno] = v;
			ret = 0;
		}
		break;
	case FB_VISUAL_STATIC_PSEUDOCOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		ret = mx2fb_setpalettereg(regno, red, green, blue, transp, info);
		break;
	}

	return ret;
}

static int mx2fb_pan_display(struct fb_var_screeninfo *var,
			     struct fb_info *info)
{
	if ((info->var.xoffset == var->xoffset) &&
	    (info->var.yoffset == var->yoffset)) {
		return 0;	/* No change, do nothing */
	}

	if (var->xoffset < 0 || var->yoffset < 0
	    || var->xoffset + info->var.xres > info->var.xres_virtual
	    || var->yoffset + info->var.yres > info->var.yres_virtual)
		return -EINVAL;

	info->var.xoffset = var->xoffset;
	info->var.yoffset = var->yoffset;

	_update_lcdc(info);	/* ????????????? */

	if (var->vmode & FB_VMODE_YWRAP)
		info->var.vmode |= FB_VMODE_YWRAP;
	else
		info->var.vmode &= ~FB_VMODE_YWRAP;

	return 0;
}

static int mx2fb_blank(int blank_mode, struct fb_info *info)
{
	struct mx2fb_info *mx2fbi = (struct mx2fb_info *)info->par;

	dev_dbg(info->device, "blank mode = %d\n", blank_mode);

	mx2fbi->blank = blank_mode;

	switch (blank_mode) {
	case FB_BLANK_POWERDOWN:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
		_disable_lcdc(info);
		break;
	case FB_BLANK_UNBLANK:
		_enable_lcdc(info);
		break;
	}

	return 0;
}

/* support customized ioctl operations */
static int mx2fb_ioctl(struct fb_info *info, unsigned int cmd,
		       unsigned long arg)
{
	struct mx2fb_info *mx2fbi = (struct mx2fb_info *)info->par;
	struct mx2fb_gbl_alpha ga;
	struct mx2fb_color_key ck;
	unsigned char level;

	switch (cmd) {
	case MX2FB_SET_GBL_ALPHA:
		if (mx2fbi->type != MX2FB_TYPE_GW)
			return -ENODEV;

		if (!arg)
			return -EINVAL;

		/* set graphic window information */
		if (copy_from_user((void *)&ga, (void *)arg, sizeof(ga)))
			return -EFAULT;

		g_gwinfo.alpha_value = ga.alpha;

		if (g_gwinfo.enabled)
			_enable_graphic_window(info);
		else
			_disable_graphic_window(info);
		break;
	case MX2FB_SET_CLR_KEY:
		if (mx2fbi->type != MX2FB_TYPE_GW)
			return -ENODEV;

		if (!arg)
			return -EINVAL;

		/* set graphic window information */
		if (copy_from_user((void *)&ck, (void *)arg, sizeof(ck)))
			return -EFAULT;

		g_gwinfo.ck_enabled = ck.enable;
		g_gwinfo.ck_red = (ck.color_key & 0x003F0000) >> 16;
		g_gwinfo.ck_green = (ck.color_key & 0x00003F00) >> 8;
		g_gwinfo.ck_blue = ck.color_key & 0x0000003F;

		if (g_gwinfo.enabled)
			_enable_graphic_window(info);
		else
			_disable_graphic_window(info);
		break;
	case MX2FB_SET_BRIGHTNESS:
		if (copy_from_user((void *)&level, (void *)arg, sizeof(level)))
			return -EFAULT;
		brightness = level;
		_set_brightness(level);
		break;
	case FBIOGET_GWINFO:
		if (mx2fbi->type != MX2FB_TYPE_GW)
			return -ENODEV;

		if (!arg)
			return -EINVAL;

		/* get graphic window information */
		if (copy_to_user((void *)arg, (void *)&g_gwinfo,
				 sizeof(g_gwinfo)))
			return -EFAULT;
		break;
	case FBIOPUT_GWINFO:
		if (mx2fbi->type != MX2FB_TYPE_GW)
			return -ENODEV;

		if (!arg)
			return -EINVAL;

		/* set graphic window information */
		if (copy_from_user((void *)&g_gwinfo, (void *)arg,
				   sizeof(g_gwinfo)))
			return -EFAULT;

		if (g_gwinfo.enabled)
			_enable_graphic_window(info);
		else
			_disable_graphic_window(info);
		break;
#ifdef CONFIG_FB_MXC_TVOUT
	case ENCODER_GET_CAPABILITIES:{
			int ret;
			struct video_encoder_capability cap;

			if (mx2fbi->type != MX2FB_TYPE_BG)
				return -ENODEV;

			ret = fs453_ioctl(cmd, &cap);
			if (ret)
				return ret;

			if (copy_to_user((void *)arg, &cap, sizeof(cap)))
				return -EFAULT;
			break;
		}
	case ENCODER_SET_NORM:{
			int ret;
			unsigned long mode;
			char *smode;
			struct fb_var_screeninfo var;

			if (mx2fbi->type != MX2FB_TYPE_BG)
				return -ENODEV;

			if (copy_from_user(&mode, (void *)arg, sizeof(mode)))
				return -EFAULT;
			if ((ret = fs453_ioctl(cmd, &mode)))
				return ret;

			if (mode == VIDEO_ENCODER_PAL)
				smode = MODE_PAL;
			else if (mode == VIDEO_ENCODER_NTSC)
				smode = MODE_NTSC;
			else
				smode = MODE_VGA;

			var = info->var;
			var.nonstd = 0;
			ret = fb_find_mode(&var, info, smode, mxcfb_modedb,
					   mxcfb_modedb_sz, NULL, default_bpp);
			if ((ret != 1) && (ret != 2))	/* specified mode not found */
				return -ENODEV;

			info->var = var;
			fb_mode = smode;
			return mx2fb_set_par(info);
		}
	case ENCODER_SET_INPUT:
	case ENCODER_SET_OUTPUT:
	case ENCODER_ENABLE_OUTPUT:{
			unsigned long varg;

			if (mx2fbi->type != MX2FB_TYPE_BG)
				return -ENODEV;

			if (copy_from_user(&varg, (void *)arg, sizeof(varg)))
				return -EFAULT;
			return fs453_ioctl(cmd, &varg);
		}
#endif
	default:
		dev_dbg(info->device, "Unknown ioctl command (0x%08X)\n", cmd);
		return -EINVAL;
	}

	return 0;
}

static void _set_fix(struct fb_info *info)
{
	struct fb_fix_screeninfo *fix = &info->fix;
	struct fb_var_screeninfo *var = &info->var;
	struct mx2fb_info *mx2fbi = (struct mx2fb_info *)info->par;

	strncpy(fix->id, mx2fbi->id, strlen(mx2fbi->id));
	fix->line_length = var->xres_virtual * var->bits_per_pixel / 8;
	fix->type = FB_TYPE_PACKED_PIXELS;
	fix->accel = FB_ACCEL_NONE;

	if ((var->bits_per_pixel == 16)) {
		fix->visual = FB_VISUAL_TRUECOLOR;
	} else {
		/*
		 * Some people have weird ideas about wanting static
		 * pseudocolor maps.  I suspect their user space
		 * applications are broken.
		 */
		fix->visual = FB_VISUAL_PSEUDOCOLOR;
	}
	fix->xpanstep = 0;
	fix->ypanstep = 0;
}

static int __init _init_fbinfo(struct fb_info *info,
			       struct platform_device *pdev)
{
	struct mx2fb_info *mx2fbi = (struct mx2fb_info *)info->par;

	info->device = &pdev->dev;
	info->var.activate = FB_ACTIVATE_NOW;
	info->fbops = &mx2fb_ops;
	info->flags = FBINFO_FLAG_DEFAULT;
	info->pseudo_palette = &mx2fbi->pseudo_palette;

	/* Allocate colormap */
	fb_alloc_cmap(&info->cmap, 1 << info->var.bits_per_pixel, 0);

	return 0;
}

static int __init _install_fb(struct fb_info *info,
			      struct platform_device *pdev)
{
	struct mx2fb_info *mx2fbi = (struct mx2fb_info *)info->par;
	struct mxc_fb_platform_data *pdata = pdev->dev.platform_data;
	if (_init_fbinfo(info, pdev))
		return -EINVAL;

	mx2fbi->pdev = pdev;
	mx2fbi->pdata = pdata;

	if (fb_mode == 0)
		fb_mode = pdata->mode;

	if (!fb_find_mode(&info->var, info, fb_mode, mxcfb_modedb,
			  mxcfb_modedb_sz, NULL, default_bpp)) {
		fb_dealloc_cmap(&info->cmap);
		return -EBUSY;
	}

	/* Default Y virtual size is 2x panel size */
	/* info->var.yres_virtual = info->var.yres << 1; */

	if (mx2fbi->type == MX2FB_TYPE_GW)
		mx2fbi->blank = FB_BLANK_NORMAL;
	else
		mx2fbi->blank = FB_BLANK_UNBLANK;

	if (mx2fb_set_par(info)) {
		fb_dealloc_cmap(&info->cmap);
		return -EINVAL;
	}

	if (register_framebuffer(info) < 0) {
		_unmap_video_memory(info);
		fb_dealloc_cmap(&info->cmap);
		return -EINVAL;
	}

	mx2fbi->registered = 1;
	dev_info(info->device, "fb%d: %s fb device registered successfully.\n",
		 info->node, info->fix.id);

	return 0;
}

static void __exit _uninstall_fb(struct fb_info *info)
{
	struct mx2fb_info *mx2fbi = (struct mx2fb_info *)info->par;

	if (!mx2fbi->registered)
		return;

	unregister_framebuffer(info);
	_unmap_video_memory(info);
	if (&info->cmap)
		fb_dealloc_cmap(&info->cmap);

	mx2fbi->registered = 0;
}

/* grab a piece of memory to act as framebuffer */
static int _map_video_memory(struct fb_info *info)
{
	info->fix.smem_len = info->fix.line_length * info->var.yres_virtual;
	info->screen_base = dma_alloc_coherent(0,
					       info->fix.smem_len,
					       (dma_addr_t *) & info->fix.
					       smem_start,
					       GFP_DMA | GFP_KERNEL);

	if (info->screen_base == 0) {
		dev_err(info->device, "Unable to allocate fb memory\n");
		return -EBUSY;
	}
	dev_dbg(info->device, "Allocated fb @ paddr=0x%08lX, size=%d.\n",
		info->fix.smem_start, info->fix.smem_len);

	info->screen_size = info->fix.smem_len;

	/* Clear the screen */
	memset((char *)info->screen_base, 0, info->fix.smem_len);

	return 0;
}

static void _unmap_video_memory(struct fb_info *info)
{
	dma_free_coherent(0, info->fix.smem_len, info->screen_base,
			  (dma_addr_t) info->fix.smem_start);

	info->screen_base = 0;
	info->fix.smem_start = 0;
	info->fix.smem_len = 0;
}

static void _enable_lcdc(struct fb_info *info)
{
	static int first_enable = 1;
	struct mx2fb_info *mx2fbi = (struct mx2fb_info *)info->par;

	/*
	 * Graphic window can only be enabled while the HCLK to the LCDC
	 * is disabled. Once enabled it can subsequently be disabled and
	 * enabled without turning off the HCLK.
	 * The graphic window is enabled and then disabled here. So next
	 * time to enable graphic window the HCLK to LCDC does not need
	 * to be disabled, and the flicker (due to disabling of HCLK to
	 * LCDC) is avoided.
	 */
	if (first_enable) {
		_enable_graphic_window(info);
		_disable_graphic_window(info);
		first_enable = 0;
	}

	if (mx2fbi->type == MX2FB_TYPE_GW)
		_enable_graphic_window(info);
	else if (!fb_enabled) {
		clk_enable(lcdc_clk);
		mx2fbi->pdata->init(mx2fbi->pdev);
		if (mx2fbi->pdata->power_lcd)
			mx2fbi->pdata->power_lcd(mx2fbi->pdev, 1);
		if (mx2fbi->pdata->power_backlight)
			mx2fbi->pdata->power_backlight(mx2fbi->pdev, 1);

		_set_brightness(brightness);
		fb_enabled++;
#ifdef CONFIG_FB_MXC_TVOUT
		if (fb_mode) {
			unsigned long mode = 0;

			if (strcmp(fb_mode, MODE_VGA) == 0)
				mode = VIDEO_ENCODER_VGA;
			else if (strcmp(fb_mode, MODE_NTSC) == 0)
				mode = VIDEO_ENCODER_NTSC;
			else if (strcmp(fb_mode, MODE_PAL) == 0)
				mode = VIDEO_ENCODER_PAL;
			if (mode)
				fs453_ioctl(ENCODER_SET_NORM, &mode);
		}
#endif
	}
}

static void _disable_lcdc(struct fb_info *info)
{
	struct mx2fb_info *mx2fbi = (struct mx2fb_info *)info->par;

	if (mx2fbi->type == MX2FB_TYPE_GW)
		_disable_graphic_window(info);
	else {
		if (fb_enabled) {
			mx2fbi->pdata->exit(mx2fbi->pdev);
			if (mx2fbi->pdata->power_backlight)
				mx2fbi->pdata->power_backlight(mx2fbi->pdev, 0);
			if (mx2fbi->pdata->power_lcd)
				mx2fbi->pdata->power_lcd(mx2fbi->pdev, 0);
			_set_brightness(0);
			clk_disable(lcdc_clk);
			fb_enabled = 0;
		}
#ifdef CONFIG_FB_MXC_TVOUT
		if (fb_mode) {
			int enable = 0;

			if ((strcmp(fb_mode, MODE_VGA) == 0)
			    || (strcmp(fb_mode, MODE_NTSC) == 0)
			    || (strcmp(fb_mode, MODE_PAL) == 0))
				fs453_ioctl(ENCODER_ENABLE_OUTPUT, &enable);
		}
#endif
	}
}

static void _enable_graphic_window(struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;

	g_gwinfo.enabled = 1;

	g_gwinfo.base = (var->yoffset * var->xres_virtual + var->xoffset);
	g_gwinfo.base *= (var->bits_per_pixel) / 8;
	g_gwinfo.base += info->fix.smem_start;

	g_gwinfo.xres = var->xres;
	g_gwinfo.yres = var->yres;
	g_gwinfo.xres_virtual = var->xres_virtual;

	mx2_gw_set(&g_gwinfo);
}

static void _disable_graphic_window(struct fb_info *info)
{
	unsigned long i = 0;

	g_gwinfo.enabled = 0;

	/*
	 * Set alpha value to zero and reduce gw size, otherwise the graphic
	 * window will not be able to be enabled again.
	 */
	__raw_writel(__raw_readl(LCDC_REG(LCDC_LGWCR)) & 0x00FFFFFF,
		     LCDC_REG(LCDC_LGWCR));
	__raw_writel(((16 >> 4) << 20) + 16, LCDC_REG(LCDC_LGWSR));
	while (i < 1000)
		i++;

	/* Now disable graphic window */
	__raw_writel(__raw_readl(LCDC_REG(LCDC_LGWCR)) & ~0x00400000,
		     LCDC_REG(LCDC_LGWCR));

	dev_dbg(info->device, "Graphic window disabled.\n");
}

void mx2_gw_set(struct fb_gwinfo *gwinfo)
{
	int width, height, xpos, ypos;
	int width_bg, height_bg;
	unsigned long lgwcr = 0x00400000;	/* Graphic window control register */

	if (!gwinfo->enabled) {
		_disable_graphic_window(0);
		return;
	}

	/* Graphic window start address register */
	__raw_writel(gwinfo->base, LCDC_REG(LCDC_LGWSAR));

	/*
	 * The graphic window width, height, x position and y position
	 * must be synced up width the background window, otherwise there
	 * may be flickering.
	 */
	width_bg = (__raw_readl(LCDC_REG(LCDC_LSR)) & 0x03F00000) >> 16;
	height_bg = __raw_readl(LCDC_REG(LCDC_LSR)) & 0x000003FF;

	width = (gwinfo->xres > width_bg) ? width_bg : gwinfo->xres;
	height = (gwinfo->yres > height_bg) ? height_bg : gwinfo->yres;

	xpos = gwinfo->xpos;
	ypos = gwinfo->ypos;

	if (xpos + width > width_bg)
		xpos = width_bg - width;
	if (ypos + height > height_bg)
		ypos = height_bg - height;

	/* Graphic window size register */
	__raw_writel(((width >> 4) << 20) + height, LCDC_REG(LCDC_LGWSR));

	/* Graphic window virtual page width register */
	__raw_writel(gwinfo->xres_virtual >> 1, LCDC_REG(LCDC_LGWVPWR));

	/* Graphic window position register */
	__raw_writel(((xpos & 0x000003FF) << 16) | (ypos & 0x000003FF),
		     LCDC_REG(LCDC_LGWPR));

	/* Graphic window panning offset register */
	__raw_writel(0, LCDC_REG(LCDC_LGWPOR));

	/* Graphic window DMA control register */
	if (mx27_revision() >= CHIP_REV_2_0)
		__raw_writel(0x00040060, LCDC_REG(LCDC_LGWDCR));
	else
		__raw_writel(0x00020010, LCDC_REG(LCDC_LGWDCR));

	/* Graphic window control register */
	lgwcr |= (gwinfo->alpha_value & 0x000000FF) << 24;
	lgwcr |= gwinfo->ck_enabled ? 0x00800000 : 0;
	lgwcr |= gwinfo->vs_reversed ? 0x00200000 : 0;

	/*
	 * Color keying value
	 * Todo: assume always use RGB565
	 */
	lgwcr |= (gwinfo->ck_red & 0x0000003F) << 12;
	lgwcr |= (gwinfo->ck_green & 0x0000003F) << 6;
	lgwcr |= gwinfo->ck_blue & 0x0000003F;

	__raw_writel(lgwcr, LCDC_REG(LCDC_LGWCR));

	pr_debug("Graphic window enabled.\n");
}

static void _update_lcdc(struct fb_info *info)
{
	unsigned long base;
	unsigned long perclk3, pcd, pcr;
	struct fb_var_screeninfo *var = &info->var;
	struct mx2fb_info *mx2fbi = (struct mx2fb_info *)info->par;

	if (mx2fbi->type == MX2FB_TYPE_GW) {
		_enable_graphic_window(info);
		return;
	}

	base = (var->yoffset * var->xres_virtual + var->xoffset);
	base *= (var->bits_per_pixel) / 8;
	base += info->fix.smem_start;

	/* Screen start address register */
	__raw_writel(base, LCDC_REG(LCDC_LSSAR));

	/* Size register */
	dev_dbg(info->device, "xres = %d, yres = %d\n",
		info->var.xres, info->var.yres);
	__raw_writel(((info->var.xres >> 4) << 20) + info->var.yres,
		     LCDC_REG(LCDC_LSR));

	/* Virtual page width register */
	__raw_writel((var->xres * var->bits_per_pixel / 8 / 4), LCDC_REG(LCDC_LVPWR));

	/* To setup LCDC pixel clock */
	perclk3 = clk_round_rate(lcdc_clk, 134000000);
	if (clk_set_rate(lcdc_clk, perclk3)) {
		printk(KERN_INFO "mx2fb: Unable to set clock to %lu\n",
		       perclk3);
		perclk3 = clk_get_rate(lcdc_clk);
	}

	/* Calculate pixel clock divider, and round to the nearest integer */
	pcd = (perclk3 * 8 / (PICOS2KHZ(var->pixclock) * 1000UL) + 4) / 8;
	if (--pcd > 0x3F)
		pcd = 0x3F;

	/* Panel configuration register */
	pcr = 0xFA000B80 | pcd;
	pcr |= (var->sync & FB_SYNC_CLK_INVERT) ? LPCR_INV_CLOCK : 0;
	pcr |= (var->sync & FB_SYNC_DATA_INVERT) ? LPCR_INV_PIXEL : 0;
	pcr |= (var->sync & FB_SYNC_HOR_HIGH_ACT) ? 0 : LPCR_INV_LINE_PULSE;
	pcr |= (var->sync & FB_SYNC_VERT_HIGH_ACT) ? 0 : LPCR_INV_FIRST_LINE_MARKER;
	pcr |= (var->sync & FB_SYNC_SHARP_MODE) ? 0x00000040 : 0;
	pcr |= (var->sync & FB_SYNC_OE_ACT_HIGH) ? 0 : LPCR_INV_OE;
	pcr |= (var->sync & FB_SYNC_CLK_IDLE_EN) ? 0x00080000 : 0;

	/* i.MX LCD controller can support 8bpp even if configured to output 16 bits (TFT) */
	if ((var->bits_per_pixel == 8)) {
#define PCR_BPIX_MASK	(7 << 25)
#define PCR_BPIX_8	(3 << 25)
#define PCR_END_BYTE_SWAP ( 1<< 17)
		pr_debug("Switching imxfb to 8bpp\n");
		__raw_writel((pcr & (~PCR_BPIX_MASK)) |
			PCR_BPIX_8 |
			PCR_END_BYTE_SWAP, LCDC_REG(LCDC_LPCR));
	} else {
		__raw_writel(pcr, LCDC_REG(LCDC_LPCR));
	}

	/* Horizontal and vertical configuration register */
	__raw_writel(((var->hsync_len - 1) << 26)
		     + ((var->right_margin - 1) << 8)
		     + (var->left_margin - 3), LCDC_REG(LCDC_LHCR));
	__raw_writel((var->vsync_len << 26)
		     + (var->lower_margin << 8)
		     + var->upper_margin, LCDC_REG(LCDC_LVCR));

	/*
	 * Sharp configuration register
	 */
	__raw_writel(0x00120300, LCDC_REG(LCDC_LSCR));

	/* Refresh mode control reigster */
	__raw_writel(0x00000000, LCDC_REG(LCDC_LRMCR));

	/* DMA control register */
	if (mx27_revision() >= CHIP_REV_2_0)
		__raw_writel(0x00040060, LCDC_REG(LCDC_LDCR));
	else
		__raw_writel(0x00020010, LCDC_REG(LCDC_LDCR));

	/* PWM contrast control register */
	_set_brightness(brightness);
}

static void _set_brightness(unsigned char level)
{
	/* Set LCDC PWM contract control register */
	__raw_writel(0x00A90300 | level, LCDC_REG(LCDC_LPCCR));
}

static irqreturn_t mx2fb_isr(int irq, void *dev_id)
{
	struct fb_event event;
	unsigned long status = __raw_readl(LCDC_REG(LCDC_LISR));

	if (status & MX2FB_INT_EOF) {
		event.info = &mx2fb_info[0];
		atomic_notifier_call_chain(&mx2fb_notifier_list,
					   FB_EVENT_MXC_EOF, &event);
	}
#ifdef CONFIG_FB_MXC_OVERLAY
	if (status & MX2FB_INT_GW_EOF) {
		event.info = &mx2fb_info[1];
		atomic_notifier_call_chain(&mx2fb_notifier_list,
					   FB_EVENT_MXC_EOF, &event);
	}
#endif

	return IRQ_HANDLED;
}

static void _request_irq(void)
{
	unsigned long status;
	unsigned long flags;

	/* Read to clear the status */
	status = __raw_readl(LCDC_REG(LCDC_LISR));

	if (request_irq(MXC_INT_LCDC, mx2fb_isr, 0, "LCDC", 0))	/* FIXME source of interrupt number */
		pr_info("Request LCDC IRQ failed.\n");
	else {
		spin_lock_irqsave(&mx2fb_notifier_list.lock, flags);

		/* Enable interrupt in case client has registered */
		if (mx2fb_notifier_list.head != NULL) {
			unsigned long status;
			unsigned long ints = MX2FB_INT_EOF;
#ifdef CONFIG_FB_MXC_OVERLAY
			ints |= MX2FB_INT_GW_EOF;
#endif
			/* Read to clear the status */
			status = __raw_readl(LCDC_REG(LCDC_LISR));

			/* Configure interrupt condition for EOF */
			__raw_writel(0x0, LCDC_REG(LCDC_LICR));

			/* Enable EOF and graphic window EOF interrupt */
			__raw_writel(ints, LCDC_REG(LCDC_LIER));
		}

		spin_unlock_irqrestore(&mx2fb_notifier_list.lock, flags);
	}
}

static void _free_irq(void)
{
	/* Disable all LCDC interrupt */
	__raw_writel(0x0, LCDC_REG(LCDC_LIER));

	free_irq(MXC_INT_LCDC, 0);
}

int mx2fb_register_client(struct notifier_block *nb)
{
	unsigned long flags;
	int ret;

	ret = atomic_notifier_chain_register(&mx2fb_notifier_list, nb);

	spin_lock_irqsave(&mx2fb_notifier_list.lock, flags);

	/* Enable interrupt in case client has registered */
	if (mx2fb_notifier_list.head != NULL) {
		unsigned long status;
		unsigned long ints = MX2FB_INT_EOF;
#ifdef CONFIG_FB_MXC_OVERLAY
		ints |= MX2FB_INT_GW_EOF;
#endif
		/* Read to clear the status */
		status = __raw_readl(LCDC_REG(LCDC_LISR));

		/* Configure interrupt condition for EOF */
		__raw_writel(0x0, LCDC_REG(LCDC_LICR));

		/* Enable EOF and graphic window EOF interrupt */
		__raw_writel(ints, LCDC_REG(LCDC_LIER));
	}

	spin_unlock_irqrestore(&mx2fb_notifier_list.lock, flags);

	return ret;
}

int mx2fb_unregister_client(struct notifier_block *nb)
{
	unsigned long flags;
	int ret;

	ret = atomic_notifier_chain_unregister(&mx2fb_notifier_list, nb);

	spin_lock_irqsave(&mx2fb_notifier_list.lock, flags);

	/* Mask interrupt in case no client registered */
	if (mx2fb_notifier_list.head == NULL)
		__raw_writel(0x0, LCDC_REG(LCDC_LIER));

	spin_unlock_irqrestore(&mx2fb_notifier_list.lock, flags);

	return ret;
}

#ifdef CONFIG_PM
/*
 * Power management hooks. Note that we won't be called from IRQ context,
 * unlike the blank functions above, so we may sleep.
 */

/* Suspends the framebuffer and blanks the screen */
static int mx2fb_suspend(struct platform_device *pdev, pm_message_t state)
{
	_disable_lcdc(&mx2fb_info[0]);

	return 0;
}

/* Resumes the framebuffer and unblanks the screen */
static int mx2fb_resume(struct platform_device *pdev)
{
	_enable_lcdc(&mx2fb_info[0]);

	return 0;
}

#endif /* CONFIG_PM */

static int mx2fb_probe(struct platform_device *pdev)
{
	int ret, i;

	lcdc_clk = clk_get(&pdev->dev, "lcdc_clk");

	for (i = 0; i < sizeof(mx2fb_info) / sizeof(struct fb_info); i++) {
		if ((ret = _install_fb(&mx2fb_info[i], pdev))) {
			dev_err(&pdev->dev,
				"Failed to register framebuffer %d\n", i);
			return ret;
		}
	}
	_request_irq();

	return 0;
}

int __init mx2fb_init(void)
{
	/*
	 * For kernel boot options (in 'video=xxxfb:<options>' format)
	 */
#ifndef MODULE
	{
		char *option;

		if (fb_get_options("mxcfb", &option))
			return -ENODEV;
		mx2fb_setup(option);
	}
#endif
	return platform_driver_register(&mx2fb_driver);
}

void __exit mx2fb_exit(void)
{
	int i;

	_free_irq();
	for (i = sizeof(mx2fb_info) / sizeof(struct fb_info); i > 0; i--)
		_uninstall_fb(&mx2fb_info[i - 1]);

	platform_driver_unregister(&mx2fb_driver);
}

#ifndef MODULE
/* Parse user specified options
 * Example: video=mxcfb:240x320,bpp=16,Sharp-QVGA
 */
static int __init mx2fb_setup(char *options)
{
	char *opt;

	if (!options || !*options)
		return 0;

	while ((opt = strsep(&options, ",")) != NULL) {
		if (!*opt)
			continue;

		if (!strncmp(opt, "bpp=", 4))
			default_bpp = simple_strtoul(opt + 4, NULL, 0);
		else
			fb_mode = opt;
	}

	return 0;
}
#endif

/* Modularization */
module_init(mx2fb_init);
module_exit(mx2fb_exit);

EXPORT_SYMBOL(mx2_gw_set);
EXPORT_SYMBOL(mx2fb_register_client);
EXPORT_SYMBOL(mx2fb_unregister_client);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MX2 framebuffer driver");
MODULE_LICENSE("GPL");
