/*
 * A V4L2 driver for OmniVision OV7670 cameras.
 *
 * Copyright 2006 One Laptop Per Child Association, Inc.  Written
 * by Jonathan Corbet with substantial inspiration from Mark
 * McClelland's ovcamchip code.
 *
 * Copyright 2006-7 Jonathan Corbet <corbet@lwn.net>
 *
 * Copyright 2010 Armadeus Systems
 * Modified to be compatible with soc_camera infrastructure
 *
 * This file may be distributed under the terms of the GNU General
 * Public License, version 2.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/videodev.h>
#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <linux/i2c.h>
#include <media/soc_camera.h>
#ifdef DEBUG
#include <linux/proc_fs.h>
#endif

MODULE_AUTHOR("Jonathan Corbet <corbet@lwn.net>, Julien Boibessot <julien.boibessot@armadeus.com>");
MODULE_DESCRIPTION("SoC camera driver for OmniVision ov7670 sensors");
MODULE_LICENSE("GPL");
#define DRIVER_NAME  "ov7670"

/*
 * Basic window sizes.  These probably belong somewhere more globally
 * useful.
 */
#define VGA_WIDTH	640
#define VGA_HEIGHT	480
#define QVGA_WIDTH	320
#define QVGA_HEIGHT	240
#define CIF_WIDTH	352
#define CIF_HEIGHT	288
#define QCIF_WIDTH	176
#define	QCIF_HEIGHT	144

/*
 * Array of image sizes supported by OV9640.  These must be ordered from
 * smallest image size to largest.
 */
struct capture_size {
	unsigned long width;
	unsigned long height;
};

enum image_size { QQCIF, QQVGA, QCIF, QVGA, CIF, VGA, SXGA };
const static struct capture_size ov7670_sizes[] = {
	{   88,  72 },	/* QQCIF */
	{  160, 120 },	/* QQVGA */
	{  176, 144 },	/* QCIF */
	{  320, 240 },	/* QVGA */
	{  352, 288 },	/* CIF */
	{  640, 480 },	/* VGA */
	{ 1280, 960 },	/* SXGA */
};

/*
 * Our nominal (default) frame rate.
 */
#define OV7670_FRAME_RATE 30

/* Registers */
#define REG_GAIN	0x00	/* Gain lower 8 bits (rest in vref) */
#define REG_BLUE	0x01	/* blue gain */
#define REG_RED		0x02	/* red gain */
#define REG_VREF	0x03	/* Pieces of GAIN, VSTART, VSTOP */
#define REG_COM1	0x04	/* Control 1 */
#define  COM1_CCIR656	  0x40  /* CCIR656 enable */
#define REG_BAVE	0x05	/* U/B Average level */
#define REG_GbAVE	0x06	/* Y/Gb Average level */
#define REG_AECHH	0x07	/* AEC MS 5 bits */
#define REG_RAVE	0x08	/* V/R Average level */
#define REG_COM2	0x09	/* Control 2 */
#define  COM2_SSLEEP	  0x10	/* Soft sleep mode */
#define REG_PID		0x0a	/* Product ID MSB */
#define REG_VER		0x0b	/* Product ID LSB */
#define REG_COM3	0x0c	/* Control 3 */
#define  COM3_SWAP	  0x40	  /* Byte swap */
#define  COM3_SCALEEN	  0x08	  /* Enable scaling */
#define  COM3_DCWEN	  0x04	  /* Enable downsamp/crop/window */
#define REG_COM4	0x0d	/* Control 4 */
#define REG_COM5	0x0e	/* All "reserved" */
#define REG_COM6	0x0f	/* Control 6 */
#define REG_AECH	0x10	/* More bits of AEC value */
#define REG_CLKRC	0x11	/* Clocl control */
#define   CLK_EXT	  0x40	  /* Use external clock directly */
#define   CLK_SCALE	  0x3f	  /* Mask for internal clock scale */
#define REG_COM7	0x12	/* Control 7 */
#define   COM7_RESET	  0x80	  /* Register reset */
#define   COM7_FMT_MASK	  0x38
#define   COM7_FMT_VGA	  0x00
#define	  COM7_FMT_CIF	  0x20	  /* CIF format */
#define   COM7_FMT_QVGA	  0x10	  /* QVGA format */
#define   COM7_FMT_QCIF	  0x08	  /* QCIF format */
#define	  COM7_RGB	  0x04	  /* bits 0 and 2 - RGB format */
#define	  COM7_YUV	  0x00	  /* YUV */
#define	  COM7_BAYER	  0x01	  /* Bayer format */
#define	  COM7_PBAYER	  0x05	  /* "Processed bayer" */
#define REG_COM8	0x13	/* Control 8 */
#define   COM8_FASTAEC	  0x80	  /* Enable fast AGC/AEC */
#define   COM8_AECSTEP	  0x40	  /* Unlimited AEC step size */
#define   COM8_BFILT	  0x20	  /* Band filter enable */
#define   COM8_AGC	  0x04	  /* Auto gain enable */
#define   COM8_AWB	  0x02	  /* White balance enable */
#define   COM8_AEC	  0x01	  /* Auto exposure enable */
#define REG_COM9	0x14	/* Control 9  - gain ceiling */
#define REG_COM10	0x15	/* Control 10 */
#define   COM10_HSYNC	  0x40	  /* HSYNC instead of HREF */
#define   COM10_PCLK_HB	  0x20	  /* Suppress PCLK on horiz blank */
#define   COM10_HREF_REV  0x08	  /* Reverse HREF */
#define   COM10_VS_LEAD	  0x04	  /* VSYNC on clock leading edge */
#define   COM10_VS_NEG	  0x02	  /* VSYNC negative */
#define   COM10_HS_NEG	  0x01	  /* HSYNC negative */
#define REG_HSTART	0x17	/* Horiz start high bits */
#define REG_HSTOP	0x18	/* Horiz stop high bits */
#define REG_VSTART	0x19	/* Vert start high bits */
#define REG_VSTOP	0x1a	/* Vert stop high bits */
#define REG_PSHFT	0x1b	/* Pixel delay after HREF */
#define REG_MIDH	0x1c	/* Manuf. ID high */
#define REG_MIDL	0x1d	/* Manuf. ID low */
#define REG_MVFP	0x1e	/* Mirror / vflip */
#define   MVFP_MIRROR	  0x20	  /* Mirror image */
#define   MVFP_FLIP	  0x10	  /* Vertical flip */

#define REG_AEW		0x24	/* AGC upper limit */
#define REG_AEB		0x25	/* AGC lower limit */
#define REG_VPT		0x26	/* AGC/AEC fast mode op region */
#define REG_HSYST	0x30	/* HSYNC rising edge delay */
#define REG_HSYEN	0x31	/* HSYNC falling edge delay */
#define REG_HREF	0x32	/* HREF pieces */
#define REG_TSLB	0x3a	/* lots of stuff */
#define   TSLB_YLAST	  0x04	  /* UYVY or VYUY - see com13 */
#define REG_COM11	0x3b	/* Control 11 */
#define   COM11_NIGHT	  0x80	  /* NIght mode enable */
#define   COM11_NMFR	  0x60	  /* Two bit NM frame rate */
#define   COM11_HZAUTO	  0x10	  /* Auto detect 50/60 Hz */
#define	  COM11_50HZ	  0x08	  /* Manual 50Hz select */
#define   COM11_EXP	  0x02
#define REG_COM12	0x3c	/* Control 12 */
#define   COM12_HREF	  0x80	  /* HREF always */
#define REG_COM13	0x3d	/* Control 13 */
#define   COM13_GAMMA	  0x80	  /* Gamma enable */
#define	  COM13_UVSAT	  0x40	  /* UV saturation auto adjustment */
#define   COM13_UVSWAP	  0x01	  /* V before U - w/TSLB */
#define REG_COM14	0x3e	/* Control 14 */
#define   COM14_DCWEN	  0x10	  /* DCW/PCLK-scale enable */
#define REG_EDGE	0x3f	/* Edge enhancement factor */
#define REG_COM15	0x40	/* Control 15 */
#define   COM15_R10F0	  0x00	  /* Data range 10 to F0 */
#define	  COM15_R01FE	  0x80	  /*            01 to FE */
#define   COM15_R00FF	  0xc0	  /*            00 to FF */
#define   COM15_RGB565	  0x10	  /* RGB565 output */
#define   COM15_RGB555	  0x30	  /* RGB555 output */
#define REG_COM16	0x41	/* Control 16 */
#define   COM16_AWBGAIN   0x08	  /* AWB gain enable */
#define REG_COM17	0x42	/* Control 17 */
#define   COM17_AECWIN	  0xc0	  /* AEC window - must match COM4 */
#define   COM17_CBAR	  0x08	  /* DSP Color bar */

/*
 * This matrix defines how the colors are generated, must be
 * tweaked to adjust hue and saturation.
 *
 * Order: v-red, v-green, v-blue, u-red, u-green, u-blue
 *
 * They are nine-bit signed quantities, with the sign bit
 * stored in 0x58.  Sign for v-red is bit 0, and up from there.
 */
#define	REG_CMATRIX_BASE 0x4f
#define   CMATRIX_LEN 6
#define REG_CMATRIX_SIGN 0x58


#define REG_BRIGHT	0x55	/* Brightness */
#define REG_CONTRAS	0x56	/* Contrast control */

#define REG_GFIX	0x69	/* Fix gain control */

#define REG_REG76	0x76	/* OV's name */
#define   R76_BLKPCOR	  0x80	  /* Black pixel correction enable */
#define   R76_WHTPCOR	  0x40	  /* White pixel correction enable */

#define REG_RGB444	0x8c	/* RGB 444 control */
#define   R444_ENABLE	  0x02	  /* Turn on RGB444, overrides 5x5 */
#define   R444_RGBX	  0x01	  /* Empty nibble at end */

#define REG_HAECC1	0x9f	/* Hist AEC/AGC control 1 */
#define REG_HAECC2	0xa0	/* Hist AEC/AGC control 2 */

#define REG_BD50MAX	0xa5	/* 50hz banding step limit */
#define REG_HAECC3	0xa6	/* Hist AEC/AGC control 3 */
#define REG_HAECC4	0xa7	/* Hist AEC/AGC control 4 */
#define REG_HAECC5	0xa8	/* Hist AEC/AGC control 5 */
#define REG_HAECC6	0xa9	/* Hist AEC/AGC control 6 */
#define REG_HAECC7	0xaa	/* Hist AEC/AGC control 7 */
#define REG_BD60MAX	0xab	/* 60hz banding step limit */


/*
 * Information we maintain about a known sensor.
 */
struct ov7670_format_struct;  /* coming later */

struct ov7670 {
	struct i2c_client *i2c_client;
	struct soc_camera_device icd;
	int model;	/* V4L2_IDENT_* codes from v4l2-chip-ident.h */
// 	int ver;				/*ov96xx chip version*/

	struct v4l2_int_device *v4l2_int_device;
	struct v4l2_pix_format pix;
	struct v4l2_fract timeperframe;
	struct ov7670_format_struct *fmt;  /* Current format */
	unsigned char sat;		/* Saturation value */
	int hue;			/* Hue value */
};



/*
 * The default register settings, as obtained from OmniVision.  There
 * is really no making sense of most of these - lots of "reserved" values
 * and such.
 *
 * These settings give VGA YUYV.
 */

struct regval_list {
	unsigned char reg_num;
	unsigned char value;
};

static struct regval_list ov7670_default_regs[] = {
	{ REG_COM7, COM7_RESET },
/*
 * Clock scale: 3 = 15fps
 *              2 = 20fps
 *              1 = 30fps
 */
	{ REG_CLKRC, 0x1 },	/* OV: clock scale (30 fps) */
	{ REG_TSLB,  0x04 },	/* OV */
	{ REG_COM7, 0 },	/* VGA */
	/*
	 * Set the hardware window.  These values from OV don't entirely
	 * make sense - hstop is less than hstart.  But they work...
	 */
	{ REG_HSTART, 0x13 },	{ REG_HSTOP, 0x01 },
	{ REG_HREF, 0xb6 },	{ REG_VSTART, 0x02 },
	{ REG_VSTOP, 0x7a },	{ REG_VREF, 0x0a },

	{ REG_COM3, 0 },	{ REG_COM14, 0 },
	/* Mystery scaling numbers */
	{ 0x70, 0x3a },		{ 0x71, 0x35 },
	{ 0x72, 0x11 },		{ 0x73, 0xf0 },
	{ 0xa2, 0x02 },		{ REG_COM10, 0x0 },

	/* Gamma curve values */
	{ 0x7a, 0x20 },		{ 0x7b, 0x10 },
	{ 0x7c, 0x1e },		{ 0x7d, 0x35 },
	{ 0x7e, 0x5a },		{ 0x7f, 0x69 },
	{ 0x80, 0x76 },		{ 0x81, 0x80 },
	{ 0x82, 0x88 },		{ 0x83, 0x8f },
	{ 0x84, 0x96 },		{ 0x85, 0xa3 },
	{ 0x86, 0xaf },		{ 0x87, 0xc4 },
	{ 0x88, 0xd7 },		{ 0x89, 0xe8 },

	/* AGC and AEC parameters.  Note we start by disabling those features,
	   then turn them only after tweaking the values. */
	{ REG_COM8, COM8_FASTAEC | COM8_AECSTEP | COM8_BFILT },
	{ REG_GAIN, 0 },	{ REG_AECH, 0 },
	{ REG_COM4, 0x40 }, /* magic reserved bit */
	{ REG_COM9, 0x18 }, /* 4x gain + magic rsvd bit */
	{ REG_BD50MAX, 0x05 },	{ REG_BD60MAX, 0x07 },
	{ REG_AEW, 0x95 },	{ REG_AEB, 0x33 },
	{ REG_VPT, 0xe3 },	{ REG_HAECC1, 0x78 },
	{ REG_HAECC2, 0x68 },	{ 0xa1, 0x03 }, /* magic */
	{ REG_HAECC3, 0xd8 },	{ REG_HAECC4, 0xd8 },
	{ REG_HAECC5, 0xf0 },	{ REG_HAECC6, 0x90 },
	{ REG_HAECC7, 0x94 },
	{ REG_COM8, COM8_FASTAEC|COM8_AECSTEP|COM8_BFILT|COM8_AGC|COM8_AEC },

	/* Almost all of these are magic "reserved" values.  */
	{ REG_COM5, 0x61 },	{ REG_COM6, 0x4b },
	{ 0x16, 0x02 },		{ REG_MVFP, 0x07 },
	{ 0x21, 0x02 },		{ 0x22, 0x91 },
	{ 0x29, 0x07 },		{ 0x33, 0x0b },
	{ 0x35, 0x0b },		{ 0x37, 0x1d },
	{ 0x38, 0x71 },		{ 0x39, 0x2a },
	{ REG_COM12, 0x78 },	{ 0x4d, 0x40 },
	{ 0x4e, 0x20 },		{ REG_GFIX, 0 },
	{ 0x6b, 0x4a },		{ 0x74, 0x10 },
	{ 0x8d, 0x4f },		{ 0x8e, 0 },
	{ 0x8f, 0 },		{ 0x90, 0 },
	{ 0x91, 0 },		{ 0x96, 0 },
	{ 0x9a, 0 },		{ 0xb0, 0x84 },
	{ 0xb1, 0x0c },		{ 0xb2, 0x0e },
	{ 0xb3, 0x82 },		{ 0xb8, 0x0a },

	/* More reserved magic, some of which tweaks white balance */
	{ 0x43, 0x0a },		{ 0x44, 0xf0 },
	{ 0x45, 0x34 },		{ 0x46, 0x58 },
	{ 0x47, 0x28 },		{ 0x48, 0x3a },
	{ 0x59, 0x88 },		{ 0x5a, 0x88 },
	{ 0x5b, 0x44 },		{ 0x5c, 0x67 },
	{ 0x5d, 0x49 },		{ 0x5e, 0x0e },
	{ 0x6c, 0x0a },		{ 0x6d, 0x55 },
	{ 0x6e, 0x11 },		{ 0x6f, 0x9f }, /* "9e for advance AWB" */
	{ 0x6a, 0x40 },		{ REG_BLUE, 0x40 },
	{ REG_RED, 0x60 },
	{ REG_COM8, COM8_FASTAEC|COM8_AECSTEP|COM8_BFILT|COM8_AGC|COM8_AEC|COM8_AWB },

	/* Matrix coefficients */
	{ 0x4f, 0x80 },		{ 0x50, 0x80 },
	{ 0x51, 0 },		{ 0x52, 0x22 },
	{ 0x53, 0x5e },		{ 0x54, 0x80 },
	{ 0x58, 0x9e },

	{ REG_COM16, COM16_AWBGAIN },	{ REG_EDGE, 0 },
	{ 0x75, 0x05 },		{ 0x76, 0xe1 },
	{ 0x4c, 0 },		{ 0x77, 0x01 },
	{ REG_COM13, 0xc3 },	{ 0x4b, 0x09 },
	{ 0xc9, 0x60 },		{ REG_COM16, 0x38 },
	{ 0x56, 0x40 },

	{ 0x34, 0x11 },		{ REG_COM11, COM11_EXP|COM11_HZAUTO },
	{ 0xa4, 0x88 },		{ 0x96, 0 },
	{ 0x97, 0x30 },		{ 0x98, 0x20 },
	{ 0x99, 0x30 },		{ 0x9a, 0x84 },
	{ 0x9b, 0x29 },		{ 0x9c, 0x03 },
	{ 0x9d, 0x4c },		{ 0x9e, 0x3f },
	{ 0x78, 0x04 },

	/* Extra-weird stuff.  Some sort of multiplexor register */
	{ 0x79, 0x01 },		{ 0xc8, 0xf0 },
	{ 0x79, 0x0f },		{ 0xc8, 0x00 },
	{ 0x79, 0x10 },		{ 0xc8, 0x7e },
	{ 0x79, 0x0a },		{ 0xc8, 0x80 },
	{ 0x79, 0x0b },		{ 0xc8, 0x01 },
	{ 0x79, 0x0c },		{ 0xc8, 0x0f },
	{ 0x79, 0x0d },		{ 0xc8, 0x20 },
	{ 0x79, 0x09 },		{ 0xc8, 0x80 },
	{ 0x79, 0x02 },		{ 0xc8, 0xc0 },
	{ 0x79, 0x03 },		{ 0xc8, 0x40 },
	{ 0x79, 0x05 },		{ 0xc8, 0x30 },
	{ 0x79, 0x26 },

	{ 0xff, 0xff },	/* END MARKER */
};


/*
 * Here we'll try to encapsulate the changes for just the output
 * video format.
 *
 * RGB656 and YUV422 come from OV; RGB444 is homebrewed.
 *
 * IMPORTANT RULE: the first entry must be for COM7, see ov7670_s_fmt for why.
 */


static struct regval_list ov7670_fmt_yuv422[] = {
	{ REG_COM7, 0x0 },  /* Selects YUV mode */
	{ REG_RGB444, 0 },	/* No RGB444 please */
	{ REG_COM1, 0 },
	{ REG_COM15, COM15_R00FF },
	{ REG_COM9, 0x18 }, /* 4x gain ceiling; 0x8 is reserved bit */
	{ 0x4f, 0x80 }, 	/* "matrix coefficient 1" */
	{ 0x50, 0x80 }, 	/* "matrix coefficient 2" */
	{ 0x51, 0    },		/* vb */
	{ 0x52, 0x22 }, 	/* "matrix coefficient 4" */
	{ 0x53, 0x5e }, 	/* "matrix coefficient 5" */
	{ 0x54, 0x80 }, 	/* "matrix coefficient 6" */
	{ REG_COM13, COM13_GAMMA|COM13_UVSAT },
	{ 0xff, 0xff },
};

static struct regval_list ov7670_fmt_rgb565[] = {
	{ REG_COM7, COM7_RGB },	/* Selects RGB mode */
	{ REG_RGB444, 0 },	/* No RGB444 please */
	{ REG_COM1, 0x0 },
	{ REG_COM15, COM15_RGB565 },
	{ REG_COM9, 0x38 }, 	/* 16x gain ceiling; 0x8 is reserved bit */
	{ 0x4f, 0xb3 }, 	/* "matrix coefficient 1" */
	{ 0x50, 0xb3 }, 	/* "matrix coefficient 2" */
	{ 0x51, 0    },		/* vb */
	{ 0x52, 0x3d }, 	/* "matrix coefficient 4" */
	{ 0x53, 0xa7 }, 	/* "matrix coefficient 5" */
	{ 0x54, 0xe4 }, 	/* "matrix coefficient 6" */
	{ REG_COM13, COM13_GAMMA|COM13_UVSAT },
	{ 0xff, 0xff },
};

static struct regval_list ov7670_fmt_rgb444[] = {
	{ REG_COM7, COM7_RGB },	/* Selects RGB mode */
	{ REG_RGB444, R444_ENABLE },	/* Enable xxxxrrrr ggggbbbb */
	{ REG_COM1, 0x40 },	/* Magic reserved bit */
	{ REG_COM15, COM15_R01FE|COM15_RGB565 }, /* Data range needed? */
	{ REG_COM9, 0x38 }, 	/* 16x gain ceiling; 0x8 is reserved bit */
	{ 0x4f, 0xb3 }, 	/* "matrix coefficient 1" */
	{ 0x50, 0xb3 }, 	/* "matrix coefficient 2" */
	{ 0x51, 0    },		/* vb */
	{ 0x52, 0x3d }, 	/* "matrix coefficient 4" */
	{ 0x53, 0xa7 }, 	/* "matrix coefficient 5" */
	{ 0x54, 0xe4 }, 	/* "matrix coefficient 6" */
	{ REG_COM13, COM13_GAMMA|COM13_UVSAT|0x2 },  /* Magic rsvd bit */
	{ 0xff, 0xff },
};

static struct regval_list ov7670_fmt_raw[] = {
	{ REG_COM7, COM7_BAYER },
	{ REG_COM13, 0x08 }, /* No gamma, magic rsvd bit */
	{ REG_COM16, 0x3d }, /* Edge enhancement, denoise */
	{ REG_REG76, 0xe1 }, /* Pix correction, magic rsvd */
	{ 0xff, 0xff },
};



/*
 * Low-level register I/O.
 */
#if 0
static int ov7670_read(struct i2c_client *c, unsigned char reg,
		unsigned char *value)
{
	int ret;

	ret = i2c_smbus_read_byte_data(c, reg);
	if (ret >= 0) {
		*value = (unsigned char) ret;
		ret = 0;
	}
	return ret;
}
#endif

static int ov7670_read(struct i2c_client *client, u8 reg, u8 *val)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[1];

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;
	*data = reg;
	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0) {
		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
	}
	if (err >= 0) {
		*val = *data;
		return 0;
	}
	return err;
}

static int ov7670_write(struct i2c_client *c, unsigned char reg,
		unsigned char value)
{
	int ret = i2c_smbus_write_byte_data(c, reg, value);
	if (reg == REG_COM7 && (value & COM7_RESET))
		msleep(2);  /* Wait for reset to run */
	return ret;
}


/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int ov7670_write_array(struct i2c_client *c, struct regval_list *vals)
{
	while (vals->reg_num != 0xff || vals->value != 0xff) {
		int ret = ov7670_write(c, vals->reg_num, vals->value);
		if (ret < 0)
			return ret;
		vals++;
	}
	return 0;
}


/*
 * Stuff that knows about the sensor.
 */
static void ov7670_reset(struct i2c_client *client)
{
	ov7670_write(client, REG_COM7, COM7_RESET);
	msleep(1);
}

static int ov7670_initialize(struct i2c_client *client)
{
	dev_info(&client->dev, "initializing\n");

	return ov7670_write_array(client, ov7670_default_regs);
}

static int ov7670_init(struct soc_camera_device *icd)
{
	struct ov7670 *ov7670 = container_of(icd, struct ov7670, icd);
/*	struct i2c_client *client = ov7670->i2c_client; */
	struct soc_camera_link *icl = ov7670->i2c_client->dev.platform_data;
	int ret;

	printk(KERN_DEBUG "- %s\n", __func__);

	if (icl->power) {
		ret = icl->power(&ov7670->i2c_client->dev, 1);
		if (ret < 0) {
			dev_err(icd->vdev->parent,
				"Platform failed to power-on the camera.\n");
			return ret;
		}
	} else {
		/* soft powerup ? */
	}

	return 0; /*ov7670_initialize(client);*/
}

static int ov7670_release(struct soc_camera_device *icd)
{
	struct ov7670 *ov7670 = container_of(icd, struct ov7670, icd);
	struct soc_camera_link *icl = ov7670->i2c_client->dev.platform_data;

	printk(KERN_DEBUG "- %s\n", __func__);

	/* powerdown the chip */
	if (icl->power) {
		icl->power(&ov7670->i2c_client->dev, 0);
	} else {
		/* soft powerdown ? */
	}

	return 0;
}

static int ov7670_start_capture(struct soc_camera_device *icd)
{
	/* TBDJB */
	printk(KERN_DEBUG "- %s\n", __func__);
	return 0;
}

static int ov7670_stop_capture(struct soc_camera_device *icd)
{
	/* TBDJB */
	printk(KERN_DEBUG "- %s\n", __func__);
	return 0;
}

static int ov7670_set_bus_param(struct soc_camera_device *icd,
				 unsigned long flags)
{
	/* TBDJB */
	printk(KERN_DEBUG "- %s\n", __func__);
	return 0;
}

static unsigned long ov7670_query_bus_param(struct soc_camera_device *icd)
{
	unsigned int width_flag = SOCAM_DATAWIDTH_10 | SOCAM_DATAWIDTH_8;

	printk(KERN_DEBUG "- %s\n", __func__);
	/* OV7670 has all capture_format parameters fixed ?? TBDJB*/
	return SOCAM_PCLK_SAMPLE_RISING |
		SOCAM_HSYNC_ACTIVE_HIGH |
		SOCAM_VSYNC_ACTIVE_HIGH |
		SOCAM_MASTER |
		width_flag;
}

static int ov7670_detect(struct i2c_client *client)
{
	unsigned char v;
	int ret;

	printk(KERN_DEBUG "- %s\n", __func__);
#if 0
	ret = ov7670_initialize(client);
	if (ret < 0)
		return ret;
#endif
	ret = ov7670_read(client, REG_MIDH, &v);
	if (ret < 0)
		return ret;
	if (v != 0x7f) /* OV manuf. id. */
		return -ENODEV;
	ret = ov7670_read(client, REG_MIDL, &v);
	if (ret < 0)
		return ret;
	if (v != 0xa2)
		return -ENODEV;
	/*
	 * OK, we know we have an OmniVision chip...but which one?
	 */
	ret = ov7670_read(client, REG_PID, &v);
	if (ret < 0)
		return ret;
	if (v != 0x76)  /* PID + VER = 0x76 / 0x73 */
		return -ENODEV;
	ret = ov7670_read(client, REG_VER, &v);
	if (ret < 0)
		return ret;
	if (v != 0x73)  /* PID + VER = 0x76 / 0x73 */
		return -ENODEV;

	ret = ov7670_initialize(client);
	if (ret < 0)
		return ret;

	return 0;
}


/*
 * Store information about the video data format.  The color matrix
 * is deeply tied into the format, so keep the relevant values here.
 * The magic matrix nubmers come from OmniVision.
 */
static struct ov7670_format_struct {
	__u8 *desc;
	__u32 pixelformat;
	struct regval_list *regs;
	int cmatrix[CMATRIX_LEN];
	int bpp;   /* Bytes per pixel */
} ov7670_formats[] = {
	{
		.desc		= "YUYV 4:2:2",
		.pixelformat	= V4L2_PIX_FMT_YUYV,
		.regs 		= ov7670_fmt_yuv422,
		.cmatrix	= { 128, -128, 0, -34, -94, 128 },
		.bpp		= 2,
	},
	{
		.desc		= "RGB 444",
		.pixelformat	= V4L2_PIX_FMT_RGB444,
		.regs		= ov7670_fmt_rgb444,
		.cmatrix	= { 179, -179, 0, -61, -176, 228 },
		.bpp		= 2,
	},
	{
		.desc		= "RGB 565",
		.pixelformat	= V4L2_PIX_FMT_RGB565,
		.regs		= ov7670_fmt_rgb565,
		.cmatrix	= { 179, -179, 0, -61, -176, 228 },
		.bpp		= 2,
	},
	{
		.desc		= "Raw RGB Bayer",
		.pixelformat	= V4L2_PIX_FMT_SBGGR8,
		.regs 		= ov7670_fmt_raw,
		.cmatrix	= { 0, 0, 0, 0, 0, 0 },
		.bpp		= 1
	},
};
#define N_OV7670_FMTS ARRAY_SIZE(ov7670_formats)

static const struct soc_camera_data_format ov7670_soc_formats[] = {
	{
		.name		= "YUYV 4:2:2",
		.fourcc	= V4L2_PIX_FMT_YUYV,
		.depth = 16,
	},
	{
		.name		= "RGB 444",
		.fourcc	= V4L2_PIX_FMT_RGB444,
		.depth = 16,
	},
	{
		.name		= "RGB 565",
		.fourcc	= V4L2_PIX_FMT_RGB565,
		.depth = 16,
	},
	{
		.name		= "Raw RGB Bayer",
		.fourcc	= V4L2_PIX_FMT_SBGGR8,
		.depth = 8,
	},

};

/*
 * Then there is the issue of window sizes.  Try to capture the info here.
 */

/*
 * QCIF mode is done (by OV) in a very strange way - it actually looks like
 * VGA with weird scaling options - they do *not* use the canned QCIF mode
 * which is allegedly provided by the sensor.  So here's the weird register
 * settings.
 */
static struct regval_list ov7670_qcif_regs[] = {
	{ REG_COM3, COM3_SCALEEN|COM3_DCWEN },
	{ REG_COM3, COM3_DCWEN },
	{ REG_COM14, COM14_DCWEN | 0x01},
	{ 0x73, 0xf1 },
	{ 0xa2, 0x52 },
	{ 0x7b, 0x1c },
	{ 0x7c, 0x28 },
	{ 0x7d, 0x3c },
	{ 0x7f, 0x69 },
	{ REG_COM9, 0x38 },
	{ 0xa1, 0x0b },
	{ 0x74, 0x19 },
	{ 0x9a, 0x80 },
	{ 0x43, 0x14 },
	{ REG_COM13, 0xc0 },
	{ 0xff, 0xff },
};

static struct ov7670_win_size {
	int	width;
	int	height;
	unsigned char com7_bit;
	int	hstart;		/* Start/stop values for the camera.  Note */
	int	hstop;		/* that they do not always make complete */
	int	vstart;		/* sense to humans, but evidently the sensor */
	int	vstop;		/* will do the right thing... */
	struct regval_list *regs; /* Regs to tweak */
/* h/vref stuff */
} ov7670_win_sizes[] = {
	/* VGA */
	{
		.width		= VGA_WIDTH,
		.height		= VGA_HEIGHT,
		.com7_bit	= COM7_FMT_VGA,
		.hstart		= 158,		/* These values from */
		.hstop		=  14,		/* Omnivision */
		.vstart		=  10,
		.vstop		= 490,
		.regs 		= NULL,
	},
	/* CIF */
	{
		.width		= CIF_WIDTH,
		.height		= CIF_HEIGHT,
		.com7_bit	= COM7_FMT_CIF,
		.hstart		= 170,		/* Empirically determined */
		.hstop		=  90,
		.vstart		=  14,
		.vstop		= 494,
		.regs 		= NULL,
	},
	/* QVGA */
	{
		.width		= QVGA_WIDTH,
		.height		= QVGA_HEIGHT,
		.com7_bit	= COM7_FMT_QVGA,
		.hstart		= 164,		/* Empirically determined */
		.hstop		=  20,
		.vstart		=  14,
		.vstop		= 494,
		.regs 		= NULL,
	},
	/* QCIF */
	{
		.width		= QCIF_WIDTH,
		.height		= QCIF_HEIGHT,
		.com7_bit	= COM7_FMT_VGA, /* see comment above */
		.hstart		= 456,		/* Empirically determined */
		.hstop		=  24,
		.vstart		=  14,
		.vstop		= 494,
		.regs 		= ov7670_qcif_regs,
	},
};

#define N_WIN_SIZES (ARRAY_SIZE(ov7670_win_sizes))


/*
 * Store a set of start/stop values into the camera.
 */
static int ov7670_set_hw(struct i2c_client *client, int hstart, int hstop,
		int vstart, int vstop)
{
	int ret;
	unsigned char v;
/*
 * Horizontal: 11 bits, top 8 live in hstart and hstop.  Bottom 3 of
 * hstart are in href[2:0], bottom 3 of hstop in href[5:3].  There is
 * a mystery "edge offset" value in the top two bits of href.
 */
	ret =  ov7670_write(client, REG_HSTART, (hstart >> 3) & 0xff);
	ret += ov7670_write(client, REG_HSTOP, (hstop >> 3) & 0xff);
	ret += ov7670_read(client, REG_HREF, &v);
	v = (v & 0xc0) | ((hstop & 0x7) << 3) | (hstart & 0x7);
	msleep(10);
	ret += ov7670_write(client, REG_HREF, v);
/*
 * Vertical: similar arrangement, but only 10 bits.
 */
	ret += ov7670_write(client, REG_VSTART, (vstart >> 2) & 0xff);
	ret += ov7670_write(client, REG_VSTOP, (vstop >> 2) & 0xff);
	ret += ov7670_read(client, REG_VREF, &v);
	v = (v & 0xf0) | ((vstop & 0x3) << 2) | (vstart & 0x3);
	msleep(10);
	ret += ov7670_write(client, REG_VREF, v);
	return ret;
}

static int ov7670_try_format(struct i2c_client *client , struct v4l2_format *fmt,
		struct ov7670_format_struct **ret_fmt,
		struct ov7670_win_size **ret_wsize)
{
	int index;
	struct ov7670_win_size *wsize;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;

	if (!fmt) {
		printk("Invalid format\n");
		return -EINVAL;
	}
	for (index = 0; index < N_OV7670_FMTS; index++) {
		if (ov7670_formats[index].pixelformat == pix->pixelformat)
			break;
	}
	if (index >= N_OV7670_FMTS) {
		/* default to first format */
		index = 0;
		pix->pixelformat = ov7670_formats[0].pixelformat;
	}
	/*
	 * Fields: the OV devices claim to be progressive.
	 */
	pix->field = V4L2_FIELD_NONE;
	/*
	 * Round requested image size down to the nearest
	 * we support, but not below the smallest.
	 */
	for (wsize = ov7670_win_sizes; wsize < ov7670_win_sizes + N_WIN_SIZES;
	     wsize++)
		if (pix->width >= wsize->width && pix->height >= wsize->height)
			break;
	if (wsize >= ov7670_win_sizes + N_WIN_SIZES)
		wsize--;   /* Take the smallest one */
	/*
	 * Note the size we'll actually handle.
	 */
	pix->width = wsize->width;
	pix->height = wsize->height;
	pix->bytesperline = pix->width*ov7670_formats[index].bpp;
	pix->sizeimage = pix->height*pix->bytesperline;

	return 0;
}

static int ov7670_try_fmt(struct soc_camera_device *icd, struct v4l2_format *fmt)
{
	struct ov7670 *ov7670 = container_of(icd, struct ov7670, icd);
	struct i2c_client *client = ov7670->i2c_client;

	return ov7670_try_format(client, fmt, NULL, NULL);
}


/*
 * Set a format.
 */
static int ov7670_set_fmt(struct soc_camera_device *icd,
				__u32 pixfmt,
				struct v4l2_rect *rect)
{
	struct ov7670 *ov7670 = container_of(icd, struct ov7670, icd);
	struct i2c_client *c = ov7670->i2c_client;
	int ret = -EINVAL;
	int i;
	struct ov7670_win_size *wsize;
	unsigned char com7, clkrc;

	for (i = 0; i < N_OV7670_FMTS; i++) {
		if (pixfmt == ov7670_formats[i].pixelformat) {
			ret = 0;
			ov7670->fmt = &ov7670_formats[i];
			icd->current_fmt = &ov7670_soc_formats[i];
			break;
		}
	}

/*	ret = ov7670_try_format(c, fmt, &ovfmt, &wsize);*/
	if (ret)
		return ret;

	/* Duplicated code ? */
	for (wsize = ov7670_win_sizes; wsize < ov7670_win_sizes + N_WIN_SIZES;
	     wsize++)
		if (rect->width >= wsize->width && rect->height >= wsize->height)
			break;
	if (wsize >= ov7670_win_sizes + N_WIN_SIZES)
		wsize--;   /* Take the smallest one */

	/*
	 * HACK: if we're running rgb565 we need to grab then rewrite
	 * CLKRC.  If we're *not*, however, then rewriting clkrc hoses
	 * the colors.
	 */
	if (pixfmt == V4L2_PIX_FMT_RGB565) {
		ret = ov7670_read(c, REG_CLKRC, &clkrc);
		if (ret)
			return ret;
	}
	/*
	 * COM7 is a pain in the ass, it doesn't like to be read then
	 * quickly written afterward.  But we have everything we need
	 * to set it absolutely here, as long as the format-specific
	 * register sets list it first.
	 */
	com7 = ov7670->fmt->regs[0].value;
	com7 |= wsize->com7_bit;
	ov7670_write(c, REG_COM7, com7);
	/*
	 * Now write the rest of the array.  Also store start/stops
	 */
	ov7670_write_array(c, ov7670->fmt->regs + 1);
	ov7670_set_hw(c, wsize->hstart, wsize->hstop, wsize->vstart,
			wsize->vstop);
	ret = 0;
	if (wsize->regs)
		ret = ov7670_write_array(c, wsize->regs);

	if (pixfmt == V4L2_PIX_FMT_RGB565 && ret == 0)
		ret = ov7670_write(c, REG_CLKRC, clkrc);

	return ret;
}

/*
 * Implement G/S_PARM.  There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
#if 0
static int ov7670_g_parm(struct i2c_client *c, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	unsigned char clkrc;
	int ret;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	ret = ov7670_read(c, REG_CLKRC, &clkrc);
	if (ret < 0)
		return ret;
	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe.numerator = 1;
	cp->timeperframe.denominator = OV7670_FRAME_RATE;
	if ((clkrc & CLK_EXT) == 0 && (clkrc & CLK_SCALE) > 1)
		cp->timeperframe.denominator /= (clkrc & CLK_SCALE);
	return 0;
}

static int ov7670_s_parm(struct i2c_client *c, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct v4l2_fract *tpf = &cp->timeperframe;
	unsigned char clkrc;
	int ret, div;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (cp->extendedmode != 0)
		return -EINVAL;
	/*
	 * CLKRC has a reserved bit, so let's preserve it.
	 */
	ret = ov7670_read(c, REG_CLKRC, &clkrc);
	if (ret < 0)
		return ret;
	if (tpf->numerator == 0 || tpf->denominator == 0)
		div = 1;  /* Reset to full rate */
	else
		div = (tpf->numerator*OV7670_FRAME_RATE)/tpf->denominator;
	if (div == 0)
		div = 1;
	else if (div > CLK_SCALE)
		div = CLK_SCALE;
	clkrc = (clkrc & 0x80) | div;
	tpf->numerator = 1;
	tpf->denominator = OV7670_FRAME_RATE/div;
	return ov7670_write(c, REG_CLKRC, clkrc);
}
#endif


/*
 * Code for dealing with controls.
 */

static int ov7670_store_cmatrix(struct i2c_client *client,
		int matrix[CMATRIX_LEN])
{
	int i, ret;
	unsigned char signbits = 0;

	/*
	 * Weird crap seems to exist in the upper part of
	 * the sign bits register, so let's preserve it.
	 */
	ret = ov7670_read(client, REG_CMATRIX_SIGN, &signbits);
	signbits &= 0xc0;

	for (i = 0; i < CMATRIX_LEN; i++) {
		unsigned char raw;

		if (matrix[i] < 0) {
			signbits |= (1 << i);
			if (matrix[i] < -255)
				raw = 0xff;
			else
				raw = (-1 * matrix[i]) & 0xff;
		}
		else {
			if (matrix[i] > 255)
				raw = 0xff;
			else
				raw = matrix[i] & 0xff;
		}
		ret += ov7670_write(client, REG_CMATRIX_BASE + i, raw);
	}
	ret += ov7670_write(client, REG_CMATRIX_SIGN, signbits);
	return ret;
}


/*
 * Hue also requires messing with the color matrix.  It also requires
 * trig functions, which tend not to be well supported in the kernel.
 * So here is a simple table of sine values, 0-90 degrees, in steps
 * of five degrees.  Values are multiplied by 1000.
 *
 * The following naive approximate trig functions require an argument
 * carefully limited to -180 <= theta <= 180.
 */
#define SIN_STEP 5
static const int ov7670_sin_table[] = {
	   0,	 87,   173,   258,   342,   422,
	 499,	573,   642,   707,   766,   819,
	 866,	906,   939,   965,   984,   996,
	1000
};

static int ov7670_sine(int theta)
{
	int chs = 1;
	int sine;

	if (theta < 0) {
		theta = -theta;
		chs = -1;
	}
	if (theta <= 90)
		sine = ov7670_sin_table[theta/SIN_STEP];
	else {
		theta -= 90;
		sine = 1000 - ov7670_sin_table[theta/SIN_STEP];
	}
	return sine*chs;
}

static int ov7670_cosine(int theta)
{
	theta = 90 - theta;
	if (theta > 180)
		theta -= 360;
	else if (theta < -180)
		theta += 360;
	return ov7670_sine(theta);
}

static void ov7670_calc_cmatrix(struct ov7670 *info,
		int matrix[CMATRIX_LEN])
{
	int i;
	/*
	 * Apply the current saturation setting first.
	 */
	for (i = 0; i < CMATRIX_LEN; i++)
		matrix[i] = (info->fmt->cmatrix[i]*info->sat) >> 7;
	/*
	 * Then, if need be, rotate the hue value.
	 */
	if (info->hue != 0) {
		int sinth, costh, tmpmatrix[CMATRIX_LEN];

		memcpy(tmpmatrix, matrix, CMATRIX_LEN*sizeof(int));
		sinth = ov7670_sine(info->hue);
		costh = ov7670_cosine(info->hue);

		matrix[0] = (matrix[3]*sinth + matrix[0]*costh)/1000;
		matrix[1] = (matrix[4]*sinth + matrix[1]*costh)/1000;
		matrix[2] = (matrix[5]*sinth + matrix[2]*costh)/1000;
		matrix[3] = (matrix[3]*costh - matrix[0]*sinth)/1000;
		matrix[4] = (matrix[4]*costh - matrix[1]*sinth)/1000;
		matrix[5] = (matrix[5]*costh - matrix[2]*sinth)/1000;
	}
}

static int ov7670_tweak_sat(struct i2c_client *client, int value)
{
	struct ov7670 *info = i2c_get_clientdata(client);
	int matrix[CMATRIX_LEN];
	int ret;

	info->sat = value;
	ov7670_calc_cmatrix(info, matrix);
	ret = ov7670_store_cmatrix(client, matrix);
	return ret;
}

static int ov7670_query_sat(struct i2c_client *client, __s32 *value)
{
	struct ov7670 *info = i2c_get_clientdata(client);

	*value = info->sat;
	return 0;
}

static int ov7670_tweak_hue(struct i2c_client *client, int value)
{
	struct ov7670 *info = i2c_get_clientdata(client);
	int matrix[CMATRIX_LEN];
	int ret;

	if (value < -180 || value > 180)
		return -EINVAL;
	info->hue = value;
	ov7670_calc_cmatrix(info, matrix);
	ret = ov7670_store_cmatrix(client, matrix);
	return ret;
}

static int ov7670_query_hue(struct i2c_client *client, __s32 *value)
{
	struct ov7670 *info = i2c_get_clientdata(client);

	*value = info->hue;
	return 0;
}


/*
 * Some weird registers seem to store values in a sign/magnitude format!
 */
static unsigned char ov7670_sm_to_abs(unsigned char v)
{
	if ((v & 0x80) == 0)
		return v + 128;
	else
		return 128 - (v & 0x7f);
}

static unsigned char ov7670_abs_to_sm(unsigned char v)
{
	if (v > 127)
		return v & 0x7f;
	else
		return (128 - v) | 0x80;
}

static int ov7670_tweak_brightness(struct i2c_client *client, int value)
{
	unsigned char com8 = 0, v;
	int ret;

	ov7670_read(client, REG_COM8, &com8);
	com8 &= ~COM8_AEC;
	ov7670_write(client, REG_COM8, com8);
	v = ov7670_abs_to_sm(value);
	ret = ov7670_write(client, REG_BRIGHT, v);
	return ret;
}

static int ov7670_query_brightness(struct i2c_client *client, __s32 *value)
{
	unsigned char v = 0;
	int ret = ov7670_read(client, REG_BRIGHT, &v);

	*value = ov7670_sm_to_abs(v);
	return ret;
}

static int ov7670_tweak_contrast(struct i2c_client *client, int value)
{
	return ov7670_write(client, REG_CONTRAS, (unsigned char) value);
}

static int ov7670_query_contrast(struct i2c_client *client, __s32 *value)
{
	unsigned char v = 0;
	int ret = ov7670_read(client, REG_CONTRAS, &v);

	*value = v;
	return ret;
}

static int ov7670_query_hflip(struct i2c_client *client, __s32 *value)
{
	int ret;
	unsigned char v = 0;

	ret = ov7670_read(client, REG_MVFP, &v);
	*value = (v & MVFP_MIRROR) == MVFP_MIRROR;
	return ret;
}

static int ov7670_tweak_hflip(struct i2c_client *client, int value)
{
	unsigned char v = 0;
	int ret;

	ret = ov7670_read(client, REG_MVFP, &v);
	if (value)
		v |= MVFP_MIRROR;
	else
		v &= ~MVFP_MIRROR;
	msleep(10);  /* FIXME */
	ret += ov7670_write(client, REG_MVFP, v);
	return ret;
}

static int ov7670_query_vflip(struct i2c_client *client, __s32 *value)
{
	int ret;
	unsigned char v = 0;

	ret = ov7670_read(client, REG_MVFP, &v);
	*value = (v & MVFP_FLIP) == MVFP_FLIP;
	return ret;
}

static int ov7670_tweak_vflip(struct i2c_client *client, int value)
{
	unsigned char v = 0;
	int ret;

	ret = ov7670_read(client, REG_MVFP, &v);
	if (value)
		v |= MVFP_FLIP;
	else
		v &= ~MVFP_FLIP;
	msleep(10);  /* FIXME */
	ret += ov7670_write(client, REG_MVFP, v);
	return ret;
}


static const struct v4l2_queryctrl ov7670_controls[] = {
	{
			.id = V4L2_CID_BRIGHTNESS,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Brightness",
			.minimum = 0,
			.maximum = 255,
			.step = 1,
			.default_value = 0x80,
			.flags = V4L2_CTRL_FLAG_SLIDER
	}, {
			.id = V4L2_CID_CONTRAST,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Contrast",
			.minimum = 0,
			.maximum = 127,
			.step = 1,
			.default_value = 0x40,   /* XXX ov7670 spec */
			.flags = V4L2_CTRL_FLAG_SLIDER
	}, {
			.id = V4L2_CID_SATURATION,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Saturation",
			.minimum = 0,
			.maximum = 256,
			.step = 1,
			.default_value = 0x80,
			.flags = V4L2_CTRL_FLAG_SLIDER
	}, {
			.id = V4L2_CID_HUE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "HUE",
			.minimum = -180,
			.maximum = 180,
			.step = 5,
			.default_value = 0,
			.flags = V4L2_CTRL_FLAG_SLIDER
	}, {
			.id = V4L2_CID_VFLIP,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "Vertical flip",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
	}, {
			.id = V4L2_CID_HFLIP,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "Horizontal mirror",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
	},
};
#define N_CONTROLS (ARRAY_SIZE(ov7670_controls))

/* To simplify control get/set methods
   (Should correspond to ov7670_controls)
 */
static struct ov7670_control {
	__u32 id;
	int (*query)(struct i2c_client *c, __s32 *value);
	int (*tweak)(struct i2c_client *c, int value);
} ov7670_priv_controls[] = {
	{
		.id = V4L2_CID_BRIGHTNESS,
		.tweak = ov7670_tweak_brightness,
		.query = ov7670_query_brightness,
	}, {
		.id = V4L2_CID_CONTRAST,
		.tweak = ov7670_tweak_contrast,
		.query = ov7670_query_contrast,
	}, {
		.id = V4L2_CID_SATURATION,
		.tweak = ov7670_tweak_sat,
		.query = ov7670_query_sat,
	}, {
		.id = V4L2_CID_HUE,
		.tweak = ov7670_tweak_hue,
		.query = ov7670_query_hue,
	}, {
		.id = V4L2_CID_VFLIP,
		.tweak = ov7670_tweak_vflip,
		.query = ov7670_query_vflip,
	}, {
		.id = V4L2_CID_HFLIP,
		.tweak = ov7670_tweak_hflip,
		.query = ov7670_query_hflip,
	},
};

static struct ov7670_control *ov7670_find_control(__u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++)
		if (ov7670_priv_controls[i].id == id)
			return ov7670_priv_controls + i;
	return NULL;
}

static int ov7670_get_control(struct soc_camera_device *icd,
				struct v4l2_control *ctrl)
{
	struct ov7670 *ov7670 = container_of(icd, struct ov7670, icd);
	struct ov7670_control *octrl = ov7670_find_control(ctrl->id);
	int ret;

	if (octrl == NULL)
		return -EINVAL;

	ret = octrl->query(ov7670->i2c_client, &ctrl->value);
	if (ret >= 0)
		return 0;
	return ret;
}

static int ov7670_set_control(struct soc_camera_device *icd,
				struct v4l2_control *ctrl)
{
	struct ov7670 *ov7670 = container_of(icd, struct ov7670, icd);
	struct ov7670_control *octrl = ov7670_find_control(ctrl->id);
	int ret;

	if (octrl == NULL)
		return -EINVAL;

	ret =  octrl->tweak(ov7670->i2c_client, ctrl->value);
	if (ret >= 0)
		return 0;
	return ret;
}

static int ov7670_get_chip_id(struct soc_camera_device *icd,
			       struct v4l2_dbg_chip_ident *id)
{
	struct ov7670 *ov7670 = container_of(icd, struct ov7670, icd);

	if (id->match.type != V4L2_CHIP_MATCH_I2C_ADDR)
		return -EINVAL;

	if (id->match.addr != ov7670->i2c_client->addr)
		return -ENODEV;

	id->ident	= ov7670->model;
	id->revision	= 0;

	return 0;
}

#ifdef DEBUG
static int
ov7670_proc_read_register( __attribute__ ((unused)) struct file *file,
				const char *buf, unsigned long count,
				void *data)
{
	int len;
	char given_param[16];
	u8 reg_addr;
	u8 reg_val;
	struct i2c_client *i2c_client = data;

	if (count <= 0) {
		printk("Empty string transmitted !\n");
		return 0;
	}
	if (count > 4) {
		len = 4;
		printk("Only 4x[0-9] decimal values supported !\n");
	} else {
		len = count;
	}

	if (copy_from_user(given_param, buf, len)) {
		return -EFAULT;
	}

	reg_addr = (u8)(simple_strtol(given_param, 0, 16));
	/*current_reg_addr = reg_addr;*/
	ov7670_read(i2c_client, reg_addr, &reg_val);
	printk("Read 0x%02x @ 0x%02x\n", reg_val, reg_addr);

	return len;
}
#endif

/* Interface active, can use i2c. If it fails, it can indeed mean, that
 * this wasn't our capture interface, so, we wait for the right one */
static int ov7670_video_probe(struct soc_camera_device *icd)
{
	struct ov7670 *ov7670 = container_of(icd, struct ov7670, icd);
	struct soc_camera_link *icl = ov7670->i2c_client->dev.platform_data;
#ifdef DEBUG
	struct proc_dir_entry *proc_read/*, *proc_write*/;
#endif
	struct i2c_client *client = ov7670->i2c_client;
	int ret;

	if (icl->reset) {
		ret = icl->reset(&ov7670->i2c_client->dev);
	} else {
		ov7670_reset(client); /* soft reset */
	}

	ret = ov7670_detect(client);
	if (ret < 0) {
		dev_err(&client->dev, "Unable to detect " DRIVER_NAME " sensor\n");
		return ret;
	}
	dev_info(&client->dev, "found an 0V7670 sensor\n");
	ov7670->model = V4L2_IDENT_OV7670;

	icd->formats = ov7670_soc_formats;
	icd->num_formats = ARRAY_SIZE(ov7670_soc_formats);

#ifdef DEBUG
	/* create proc files */
	proc_read = create_proc_entry("driver/ovread", S_IWUSR | S_IRGRP | S_IROTH, NULL);
	/*proc_write = create_proc_entry("driver/ovwrite", S_IWUSR | S_IRGRP | S_IROTH, NULL);*/
	if ((proc_read == NULL) /*|| (proc_write == NULL)*/) {
		printk(KERN_ERR DRIVER_NAME ": Could not register one /proc file. Terminating\n");
		return -ENOMEM;
	} else {
		proc_read->write_proc = ov7670_proc_read_register;
		proc_read->data = client;
		/*proc_write->write_proc = ov7670_proc_write_register;
		proc_write->data = client;*/
	}
#endif

	/* Now that we know the model, we can start video */
	ret = soc_camera_video_start(icd);
	if (ret)
		goto err;

	return 0;

err:
	return ret;
}

static void ov7670_video_remove(struct soc_camera_device *icd)
{
	struct ov7670 *ov7670 = container_of(icd, struct ov7670, icd);

	dev_dbg(&icd->dev, "Video %x removed: %p, %p\n", ov7670->i2c_client->addr,
		icd->dev.parent, icd->vdev);
	soc_camera_video_stop(icd);
}

static int ov7670_suspend(struct soc_camera_device *icd, pm_message_t state)
{
	struct ov7670 *ov7670 = container_of(icd, struct ov7670, icd);
	struct soc_camera_link *icl = ov7670->i2c_client->dev.platform_data;
	int ret;

	printk("--- %s %d\n", __func__, state.event);

	/* supposing camera link power() is a powerdown: */
	if (icl->power) {
		ret = icl->power(&ov7670->i2c_client->dev, 0);
		if (ret < 0) {
			dev_err(icd->vdev->parent,
				"Platform failed to powerdown the camera.\n");
			return ret;
		}
	}

	return 0;
}

static int ov7670_resume(struct soc_camera_device *icd)
{
	struct ov7670 *ov7670 = container_of(icd, struct ov7670, icd);
	struct soc_camera_link *icl = ov7670->i2c_client->dev.platform_data;
	int ret;

	printk("--- %s\n", __func__);

	if (icl->power) {
		ret = icl->power(&ov7670->i2c_client->dev, 1);
		if (ret < 0) {
			dev_err(icd->vdev->parent,
				"Platform failed to resume the camera.\n");
			return ret;
		}
	}

	return 0;
}

static struct soc_camera_ops ov7670_ops = {
	.owner			= THIS_MODULE,
	.probe			= ov7670_video_probe,
	.remove			= ov7670_video_remove,
	.suspend		= ov7670_suspend,
	.resume			= ov7670_resume,
	.init			= ov7670_init,
	.release		= ov7670_release,
	.start_capture		= ov7670_start_capture,
	.stop_capture		= ov7670_stop_capture,
	.set_fmt		= ov7670_set_fmt,
	.try_fmt		= ov7670_try_fmt,
	.set_bus_param		= ov7670_set_bus_param,
	.query_bus_param	= ov7670_query_bus_param,
	.controls		= ov7670_controls,
	.num_controls		= N_CONTROLS,
	.get_control		= ov7670_get_control,
	.set_control		= ov7670_set_control,
	.get_chip_id		= ov7670_get_chip_id,
/*#ifdef CONFIG_VIDEO_ADV_DEBUG
	.get_register		= ov7670_get_register,
	.set_register		= ov7670_set_register,
#endif*/
};

static int __init
ov7670_probe(struct i2c_client *client, const struct i2c_device_id *did)
{
	struct ov7670 *ov7670;
	struct soc_camera_device *icd;
	struct soc_camera_link *icl = client->dev.platform_data;
	int ret;

	if (!icl) {
		dev_err(&client->dev, DRIVER_NAME " driver needs platform data ?\n");
/* 		return -EINVAL; TBDJB ! */
	}

	if (i2c_get_clientdata(client))
		return -EBUSY;

	ov7670 = kzalloc(sizeof(struct ov7670), GFP_KERNEL);
	if (!ov7670)
		return -ENOMEM;

	ov7670->i2c_client = client;
	i2c_set_clientdata(client, ov7670);

	/* Second stage probe - when a capture adapter is there */
	icd = &(ov7670->icd);
	icd->ops	= &ov7670_ops;
	icd->control	= &client->dev;
	icd->x_min	= 20;
	icd->y_min	= 12;
	icd->x_current	= 20;
	icd->y_current	= 12;
	icd->width_min	= 88;
	icd->width_max	= 1280;
	icd->height_min	= 72;
	icd->height_max	= 1024;
	icd->y_skip_top	= 1;
/* 	icd->iface	= icl->bus_id; TBDJB !! */

	ret = soc_camera_device_register(icd);
	if (ret)
		goto err;

	return 0;

err:
	kfree(ov7670);
	return ret;
}

static int __exit
ov7670_remove(struct i2c_client *client)
{
 	struct ov7670 *ov7670 = i2c_get_clientdata(client);

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */
#ifdef DEBUG
	remove_proc_entry("driver/ovread", NULL);
	/*remove_proc_entry("driver/ovwrite", NULL);*/
#endif
	soc_camera_device_unregister(&ov7670->icd);
	i2c_set_clientdata(client, NULL);
	kfree(ov7670);

	return 0;
}


/* auto-detection */
static const struct i2c_device_id ov7670_id[] = {
	{ DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov7670_id);

static struct i2c_driver ov7670_i2c_driver = {
	.driver = {
		.name	= DRIVER_NAME,
	},
	.probe = ov7670_probe,
	.remove = __exit_p(ov7670_remove),
	.id_table = ov7670_id,
};


/*
 * Module initialization
 */
static int __init ov7670_mod_init(void)
{
	printk(KERN_NOTICE "OmniVision ov7670 sensor driver, at your service\n");
	return i2c_add_driver(&ov7670_i2c_driver);
}

static void __exit ov7670_mod_exit(void)
{
	i2c_del_driver(&ov7670_i2c_driver);
}

module_init(ov7670_mod_init);
module_exit(ov7670_mod_exit);
