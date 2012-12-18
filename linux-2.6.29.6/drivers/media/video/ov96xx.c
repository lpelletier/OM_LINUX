/*
 * drivers/media/video/ov96xx.c
 *
 * OV96xx sensor driver
 *
 * Author: Julien Boibessot <julien.boibessot@armadeus.com>
 * Based on work from:
 *     Andy Lowe (source@mvista.com)
 *     Trilok Soni <soni.trilok@gmail.com>
 *
 * Copyright (C) 2004 MontaVista Software, Inc.
 * Copyright (C) 2004 Texas Instruments.
 * Copyright (C) 2009-2010 Armadeus Systems
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <media/v4l2-int-device.h>
#include <linux/proc_fs.h>
#include <media/soc_camera.h>

#define DEBUG
// #include "ov9640.h"
/* to put in include/media/v4l2-chip-ident.h : */
#define V4L2_IDENT_OV9653 253

#define DRIVER_NAME  "ov96xx"
/* ov96xx i2c address 0x30
 * The platform has to define i2c_board_info
 * and call i2c_register_board_info() */
#define OV96XX_I2C_ADDR		0x30	/* = 0x60 with read/write bit */

/*
 * register offsets
 */
#define OV96XX_GAIN		0x00
#define OV96XX_BLUE		0x01
#define OV96XX_RED		0x02
#define OV96XX_VREF		0x03
#define OV96XX_COM1		0x04
#define OV96XX_BAVE		0x05
#define OV96XX_GEAVE		0x06
#define OV96XX_RAVE		0x08
#define OV96XX_COM2		0x09
#define OV96XX_PID		0x0A
#define OV96XX_VER		0x0B
#define OV96XX_COM3		0x0C
#define OV96XX_COM4		0x0D
#define OV96XX_COM5		0x0E
#define OV96XX_COM6		0x0F
#define OV96XX_AECH		0x10
#define OV96XX_CLKRC		0x11
#define REG_COM7	0x12	/* Control 7 */
#define   COM7_RESET	  0x80	  /* Register reset */
#define   COM7_FMT_MASK	  0x78
#define   COM7_FMT_VGA	  0x40
#define	  COM7_FMT_CIF	  0x20	  /* CIF format */
#define   COM7_FMT_QVGA	  0x10	  /* QVGA format */
#define   COM7_FMT_QCIF	  0x08	  /* QCIF format */
#define	  COM7_RGB	  0x04	  /* RGB */
#define	  COM7_YUV	  0x00	  /* YUV */
#define	  COM7_BAYER	  0x01	  /* Bayer format */
#define	  COM7_PBAYER	  0x05	  /* "Processed bayer" */
#define OV96XX_COM8		0x13
#define OV96XX_COM9		0x14
#define OV96XX_COM10		0x15
#define OV96XX_HSTRT		0x17
#define OV96XX_HSTOP		0x18
#define OV96XX_VSTRT		0x19
#define OV96XX_VSTOP		0x1A
#define OV96XX_PSHFT		0x1B
#define OV96XX_MIDH		0x1C
#define OV96XX_MIDL		0x1D
#define OV96XX_MVFP		0x1E
#define OV96XX_LAEC		0x1F
#define OV96XX_BOS		0x20
#define OV96XX_GBOS		0x21
#define OV96XX_GROS		0x22
#define OV96XX_ROS		0x23
#define OV96XX_AEW		0x24
#define OV96XX_AEB		0x25
#define OV96XX_VPT		0x26
#define OV96XX_BBIAS		0x27
#define OV96XX_GBBIAS		0x28
#define OV96XX_EXHCH		0x2A
#define OV96XX_EXHCL		0x2B
#define OV96XX_RBIAS		0x2C
#define OV96XX_ADVFL		0x2D
#define OV96XX_ADVFH		0x2E
#define OV96XX_YAVE		0x2F
#define OV96XX_HSYST		0x30
#define OV96XX_HSYEN		0x31
#define OV96XX_HREF		0x32
#define OV96XX_CHLF		0x33
#define OV96XX_ARBLM		0x34
#define OV96XX_ADC		0x37
#define OV96XX_ACOM		0x38
#define OV96XX_OFON		0x39
#define OV96XX_TSLB		0x3A
#define OV96XX_COM11		0x3B
#define OV96XX_COM12		0x3C
#define OV96XX_COM13		0x3D
#define OV96XX_COM14		0x3E
#define OV96XX_EDGE		0x3F
#define OV96XX_COM15		0x40
#define OV96XX_COM16		0x41
#define OV96XX_COM17		0x42
#define OV96XX_MTX1		0x4F
#define OV96XX_MTX2		0x50
#define OV96XX_MTX3		0x51
#define OV96XX_MTX4		0x52
#define OV96XX_MTX5		0x53
#define OV96XX_MTX6		0x54
#define OV96XX_MTX7		0x55
#define OV96XX_MTX8		0x56
#define OV96XX_MTX9		0x57
#define OV96XX_MTXS		0x58
#define OV96XX_LCC1		0x62
#define OV96XX_LCC2		0x63
#define OV96XX_LCC3		0x64
#define OV96XX_LCC4		0x65
#define OV96XX_LCC5		0x66
#define OV96XX_MANU		0x67
#define OV96XX_MANV		0x68
#define OV96XX_HV		0x69
#define OV96XX_MBD		0x6A
#define OV96XX_DBLV		0x6B
#define OV96XX_GSP1		0x6C
#define OV96XX_GSP2		0x6D
#define OV96XX_GSP3		0x6E
#define OV96XX_GSP4		0x6F
#define OV96XX_GSP5		0x70
#define OV96XX_GSP6		0x71
#define OV96XX_GSP7		0x72
#define OV96XX_GSP8		0x73
#define OV96XX_GSP9		0x74
#define OV96XX_GSP10		0x75
#define OV96XX_GSP11		0x76
#define OV96XX_GSP12		0x77
#define OV96XX_GSP13		0x78
#define OV96XX_GSP14		0x79
#define OV96XX_GSP15		0x7A
#define OV96XX_GSP16		0x7B
#define OV96XX_GST1		0x7C
#define OV96XX_GST2		0x7D
#define OV96XX_GST3		0x7E
#define OV96XX_GST4		0x7F
#define OV96XX_GST5		0x80
#define OV96XX_GST6		0x81
#define OV96XX_GST7		0x82
#define OV96XX_GST8		0x83
#define OV96XX_GST9		0x84
#define OV96XX_GST10		0x85
#define OV96XX_GST11		0x86
#define OV96XX_GST12		0x87
#define OV96XX_GST13		0x88
#define OV96XX_GST14		0x89
#define OV96XX_GST15		0x8A

#define OV96XX_NUM_REGS		(OV96XX_GST15 + 1)

#define OV96XX_PID_MAGIC	0x96	/* high byte of product ID number */
#define OV96XX_VER_REV2		0x48	/* low byte of product ID number */
#define OV96XX_VER_REV3		0x49	/* low byte of product ID number */
#define OV96XX_MIDH_MAGIC	0x7F	/* high byte of mfg ID */
#define OV96XX_MIDL_MAGIC	0xA2	/* low byte of mfg ID */

#define OV96XX_REG_TERM 0xFF	/* terminating list entry for reg */
#define OV96XX_VAL_TERM 0xFF	/* terminating list entry for val */

/* define a structure for ov96xx register initialization values */
struct ov96xx_reg {
	unsigned char reg;
	unsigned char val;
};

enum image_size { QQCIF = 0, QQVGA, QCIF, QVGA, CIF, VGA, SXGA };
enum pixel_format { YUV = 0, RGB565, RGB555 };

#define NUM_IMAGE_SIZES 7
#define NUM_PIXEL_FORMATS 3

struct capture_size {
	unsigned long width;
	unsigned long height;
};

struct ov96xx_platform_data {
	/* Set power state, zero is off, non-zero is on. */
	int (*power_set)(int power);
	/* Default registers written after power-on or reset. */
	const struct ov96xx_reg *default_regs;
	int (*ifparm)(struct v4l2_ifparm *p);
};

/*
 * Array of image sizes supported by OV9640.  These must be ordered from
 * smallest image size to largest.
 */
const static struct capture_size ov96xx_sizes[] = {
	{   88,  72 },	/* QQCIF */
	{  160, 120 },	/* QQVGA */
	{  176, 144 },	/* QCIF */
	{  320, 240 },	/* QVGA */
	{  352, 288 },	/* CIF */
	{  640, 480 },	/* VGA */
	{ 1280, 960 },	/* SXGA */
	{    0,   0 },  /* END */
};



struct ov96xx_sensor {
	struct i2c_client *i2c_client;
	struct soc_camera_device icd;
	int model;
	int ver;				/*ov96xx chip version*/

	const struct ov96xx_platform_data *pdata;
	struct v4l2_int_device *v4l2_int_device;
	struct v4l2_pix_format pix;
	struct v4l2_fract timeperframe;
};

// static struct ov96xx_sensor ov96xx;
// static struct i2c_driver ov96xxsensor_i2c_driver;

/* list of image formats supported by OV96xx sensor */
static const struct soc_camera_data_format ov96xx_formats[] = {
	{
		/* Note:  V4L2 defines RGB565 as:
		 *
		 *	Byte 0			  Byte 1
		 *	g2 g1 g0 r4 r3 r2 r1 r0	  b4 b3 b2 b1 b0 g5 g4 g3
		 *
		 * We interpret RGB565 as:
		 *
		 *	Byte 0			  Byte 1
		 *	g2 g1 g0 b4 b3 b2 b1 b0	  r4 r3 r2 r1 r0 g5 g4 g3
		 */
		.name	= "RGB565, le",
		.fourcc	= V4L2_PIX_FMT_RGB565,
		.depth = 16,
	},
	{
		/* Note:  V4L2 defines RGB565X as:
		 *
		 *	Byte 0			  Byte 1
		 *	b4 b3 b2 b1 b0 g5 g4 g3	  g2 g1 g0 r4 r3 r2 r1 r0
		 *
		 * We interpret RGB565X as:
		 *
		 *	Byte 0			  Byte 1
		 *	r4 r3 r2 r1 r0 g5 g4 g3	  g2 g1 g0 b4 b3 b2 b1 b0
		 */
		.name	= "RGB565, be",
		.fourcc	= V4L2_PIX_FMT_RGB565X,
		.depth = 16,
	},
	{
		.name	= "YUYV (YUV 4:2:2), packed",
		.fourcc	= V4L2_PIX_FMT_YUYV,
		.depth = 16,
	},
	{
		.name	= "UYVY, packed",
		.fourcc	= V4L2_PIX_FMT_UYVY,
		.depth = 16,
	},
	{
		/* Note:  V4L2 defines RGB555 as:
		 *
		 *	Byte 0			  Byte 1
		 *	g2 g1 g0 r4 r3 r2 r1 r0	  x  b4 b3 b2 b1 b0 g4 g3
		 *
		 * We interpret RGB555 as:
		 *
		 *	Byte 0			  Byte 1
		 *	g2 g1 g0 b4 b3 b2 b1 b0	  x  r4 r3 r2 r1 r0 g4 g3
		 */
		.name	= "RGB555, le",
		.fourcc	= V4L2_PIX_FMT_RGB555,
		.depth = 16,
	},
	{
		/* Note:  V4L2 defines RGB555X as:
		 *
		 *	Byte 0			  Byte 1
		 *	x  b4 b3 b2 b1 b0 g4 g3	  g2 g1 g0 r4 r3 r2 r1 r0
		 *
		 * We interpret RGB555X as:
		 *
		 *	Byte 0			  Byte 1
		 *	x  r4 r3 r2 r1 r0 g4 g3	  g2 g1 g0 b4 b3 b2 b1 b0
		 */
		.name	= "RGB555, be",
		.fourcc	= V4L2_PIX_FMT_RGB555X,
		.depth = 16,
	},
};

#define NUM_CAPTURE_FORMATS ARRAY_SIZE(ov96xx_formats)

/*
 * OV9640 register configuration for all combinations of pixel format and
 * image size
 */
	/* YUV (YCbCr) QQCIF */
const static struct ov96xx_reg qqcif_yuv[] = {
	{ 0x12, 0x08 }, { 0x3C, 0x46 }, { 0x40, 0xC0 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x24 }, { 0x0C, 0x00 }, { 0x0D, 0x40 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x50 }, { 0x50, 0x43 }, { 0x51, 0x0D },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x19 }, { 0x53, 0x4C }, { 0x54, 0x65 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x40 }, { 0x56, 0x40 }, { 0x57, 0x40 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x0F },					/* MTXS */
	{ OV96XX_REG_TERM, OV96XX_VAL_TERM }
};
	/* YUV (YCbCr) QQVGA */
const static struct ov96xx_reg qqvga_yuv[] = {
	{ 0x12, 0x10 }, { 0x3C, 0x46 }, { 0x40, 0xC0 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x24 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x50 }, { 0x50, 0x43 }, { 0x51, 0x0D },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x19 }, { 0x53, 0x4C }, { 0x54, 0x65 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x40 }, { 0x56, 0x40 }, { 0x57, 0x40 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x0F },					/* MTXS */
	{ OV96XX_REG_TERM, OV96XX_VAL_TERM }
};
	/* YUV (YCbCr) QCIF */
const static struct ov96xx_reg qcif_yuv[] = {
	{ 0x12, 0x08 }, { 0x3C, 0x46 }, { 0x40, 0xC0 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x50 }, { 0x50, 0x43 }, { 0x51, 0x0D },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x19 }, { 0x53, 0x4C }, { 0x54, 0x65 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x40 }, { 0x56, 0x40 }, { 0x57, 0x40 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x0F },					/* MTXS */
	{ OV96XX_REG_TERM, OV96XX_VAL_TERM }
};
	/* YUV (YCbCr) QVGA */
const static struct ov96xx_reg qvga_yuv[] = {
	{ 0x12, 0x10 }, { 0x3C, 0x46 }, { 0x40, 0xC0 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0x80/*0xC0*/ },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x50 }, { 0x50, 0x43 }, { 0x51, 0x0D },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x19 }, { 0x53, 0x4C }, { 0x54, 0x65 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x40 }, { 0x56, 0x40 }, { 0x57, 0x40 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x0F },					/* MTXS */
	{ 0x1e, 0x10 }, /* vertical flip */
	{ OV96XX_REG_TERM, OV96XX_VAL_TERM }
};
	/* YUV (YCbCr) CIF */
const static struct ov96xx_reg cif_yuv[] = {
	{ 0x12, 0x20 }, { 0x3C, 0x46 }, { 0x40, 0xC0 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x50 }, { 0x50, 0x43 }, { 0x51, 0x0D },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x19 }, { 0x53, 0x4C }, { 0x54, 0x65 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x40 }, { 0x56, 0x40 }, { 0x57, 0x40 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x0F },					/* MTXS */
	{ OV96XX_REG_TERM, OV96XX_VAL_TERM }
};
	/* YUV (YCbCr) VGA */
const static struct ov96xx_reg vga_yuv[] = {
	{ 0x12, 0x40 }, { 0x3C, 0x46 }, { 0x40, 0xC0 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x50 }, { 0x50, 0x43 }, { 0x51, 0x0D },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x19 }, { 0x53, 0x4C }, { 0x54, 0x65 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x40 }, { 0x56, 0x40 }, { 0x57, 0x40 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x0F },					/* MTXS */
	{ OV96XX_REG_TERM, OV96XX_VAL_TERM }
};
	/* YUV (YCbCr) SXGA */
const static struct ov96xx_reg sxga_yuv[] = {
	{ 0x12, 0x00 }, { 0x3C, 0x46 }, { 0x40, 0xC0 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x00 }, { 0x0D, 0x40 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x50 }, { 0x50, 0x43 }, { 0x51, 0x0D },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x19 }, { 0x53, 0x4C }, { 0x54, 0x65 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x40 }, { 0x56, 0x40 }, { 0x57, 0x40 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x0F },					/* MTXS */
	{ OV96XX_REG_TERM, OV96XX_VAL_TERM }
};
	/* RGB565 QQCIF */
const static struct ov96xx_reg qqcif_565[] = {
	{ 0x12, 0x0C }, { 0x3C, 0x40 }, { 0x40, 0x10 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x24 }, { 0x0C, 0x00 }, { 0x0D, 0x40 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 },					/* MTXS */
	{ OV96XX_REG_TERM, OV96XX_VAL_TERM }
};
	/* RGB565 QQVGA */
const static struct ov96xx_reg qqvga_565[] = {
	{ 0x12, 0x14 }, { 0x3C, 0x40 }, { 0x40, 0x10 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x24 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 },					/* MTXS */
	{ OV96XX_REG_TERM, OV96XX_VAL_TERM }
};
	/* RGB565 QCIF */
const static struct ov96xx_reg qcif_565[] = {
	{ 0x12, 0x0C }, { 0x3C, 0x40 }, { 0x40, 0x10 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 },					/* MTXS */
	{ OV96XX_REG_TERM, OV96XX_VAL_TERM }
};
	/* RGB565 QVGA */
const static struct ov96xx_reg qvga_565[] = {
	{ 0x12, 0x14 }, { 0x3C, 0x40 }, { 0x40, 0x10 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 },					/* MTXS */
	{ OV96XX_REG_TERM, OV96XX_VAL_TERM }
};
	/* RGB565 CIF */
const static struct ov96xx_reg cif_565[] = {
	{ 0x12, 0x24 }, { 0x3C, 0x40 }, { 0x40, 0x10 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 },					/* MTXS */
	{ OV96XX_REG_TERM, OV96XX_VAL_TERM }
};
	/* RGB565 VGA */
const static struct ov96xx_reg vga_565[] = {
	{ 0x12, 0x44 }, { 0x3C, 0x40 }, { 0x40, 0x10 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 },					/* MTXS */
	{ OV96XX_REG_TERM, OV96XX_VAL_TERM }
};
	/* RGB565 SXGA */
const static struct ov96xx_reg sxga_565[] = {
	{ 0x12, 0x04 }, { 0x3C, 0x40 }, { 0x40, 0x10 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x00 }, { 0x0D, 0x40 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 },					/* MTXS */
	{ OV96XX_REG_TERM, OV96XX_VAL_TERM }
};
	/* RGB555 QQCIF */
const static struct ov96xx_reg qqcif_555[] = {
	{ 0x12, 0x0C }, { 0x3C, 0x40 }, { 0x40, 0x30 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x24 }, { 0x0C, 0x00 }, { 0x0D, 0x40 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 },					/* MTXS */
	{ OV96XX_REG_TERM, OV96XX_VAL_TERM }
};
	/* RGB555 QQVGA */
const static struct ov96xx_reg qqvga_555[] = {
	{ 0x12, 0x14 }, { 0x3C, 0x40 }, { 0x40, 0x30 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x24 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 },					/* MTXS */
	{ OV96XX_REG_TERM, OV96XX_VAL_TERM }
};
	/* RGB555 QCIF */
const static struct ov96xx_reg qcif_555[] = {
	{ 0x12, 0x0C }, { 0x3C, 0x40 }, { 0x40, 0x30 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 },					/* MTXS */
	{ OV96XX_REG_TERM, OV96XX_VAL_TERM }
};
	/* RGB555 QVGA */
const static struct ov96xx_reg qvga_555[] = {
	{ 0x12, 0x14 }, { 0x3C, 0x40 }, { 0x40, 0x30 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 },					/* MTXS */
	{ OV96XX_REG_TERM, OV96XX_VAL_TERM }
};
	/* RGB555 CIF */
const static struct ov96xx_reg cif_555[] = {
	{ 0x12, 0x24 }, { 0x3C, 0x40 }, { 0x40, 0x30 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 },					/* MTXS */
	{ OV96XX_REG_TERM, OV96XX_VAL_TERM }
};
	/* RGB555 VGA */
const static struct ov96xx_reg vga_555[] = {
	{ 0x12, 0x44 }, { 0x3C, 0x40 }, { 0x40, 0x30 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 },					/* MTXS */
	{ OV96XX_REG_TERM, OV96XX_VAL_TERM }
};
	/* RGB555 SXGA */
const static struct ov96xx_reg sxga_555[] = {
	{ 0x12, 0x04 }, { 0x3C, 0x40 }, { 0x40, 0x30 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x00 }, { 0x0D, 0x40 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 },					/* MTXS */
	{ OV96XX_REG_TERM, OV96XX_VAL_TERM }
};


#define DEF_GAIN         31
#define DEF_AUTOGAIN      1
#define DEF_EXPOSURE    154
#define DEF_AEC           1
#define DEF_FREEZE_AGCAEC 0
#define DEF_BLUE        153
#define DEF_RED         (255 - DEF_BLUE)
#define DEF_AWB           1
#define DEF_HFLIP         0
#define DEF_VFLIP         0

/* Our own specific controls */
#define V4L2_CID_FREEZE_AGCAEC		V4L2_CID_PRIVATE_BASE
#define V4L2_CID_AUTOEXPOSURE		V4L2_CID_PRIVATE_BASE + 1
#define V4L2_CID_LAST_PRIV		V4L2_CID_AUTOEXPOSURE

/*  Video controls  */
struct vcontrol {
// 	struct v4l2_queryctrl qc;
	int current_value;
	u8 reg;
	u8 mask;
	u8 start_bit;
};

/* Should match with ov96xx_controls */
static const struct vcontrol video_control[] = {
	{ /* Gain */
		.current_value	= 0,
		.reg		= OV96XX_GAIN,
		.mask		= 0x3f,
		.start_bit	= 0,
	}, { /* Auto Gain */
		.current_value	= 0,
		.reg		= OV96XX_COM8,
		.mask		= 0x04,
		.start_bit	= 2,
	}, { /* Exposure */
		.current_value	= 0,
		.reg		= OV96XX_AECH,
		.mask		= 0xff,
		.start_bit	= 0,
	}, { /* Auto Exposure */
		.current_value	= 0,
		.reg		= OV96XX_COM8,
		.mask		= 0x01,
		.start_bit	= 0,
	}, { /* Freeze AGC/AEC */
		.current_value	= 0,
		.reg		= OV96XX_COM9,
		.mask		= 0x01,
		.start_bit	= 0,
	}, { /* Red Balance */
		.current_value	= 0,
		.reg		= OV96XX_RED,
		.mask		= 0xff,
		.start_bit	= 0,
	}, { /* Blue Balance */
		.current_value	= 0,
		.reg		= OV96XX_BLUE,
		.mask		= 0xff,
		.start_bit	= 0,
	}, { /* Auto White Balance */
		.current_value	= 0,
		.reg		= OV96XX_COM8,
		.mask		= 0x02,
		.start_bit	= 1,
	}, { /* Mirror Image */
		.current_value	= 0,
		.reg		= OV96XX_MVFP,
		.mask		= 0x20,
		.start_bit	= 5,
	}, { /* Vertical Flip */
		.current_value	= 0,
		.reg		= OV96XX_MVFP,
		.mask		= 0x10,
		.start_bit	= 4,
	},
};

static const struct v4l2_queryctrl ov96xx_controls[] = {
	{
		.id = V4L2_CID_GAIN,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Gain",
		.minimum = 0,
		.maximum = 63,
		.step = 1,
		.default_value = DEF_GAIN,
	}, {
		.id = V4L2_CID_AUTOGAIN,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "Auto Gain",
		.minimum = 0,
		.maximum = 1,
		.step = 0,
		.default_value = DEF_AUTOGAIN,
	}, {
		.id = V4L2_CID_EXPOSURE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Exposure",
		.minimum = 0,
		.maximum = 255,
		.step = 1,
		.default_value = DEF_EXPOSURE,
	}, {
		.id = V4L2_CID_AUTOEXPOSURE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "Auto Exposure",
		.minimum = 0,
		.maximum = 1,
		.step = 0,
		.default_value = DEF_AEC,
	}, {
		.id = V4L2_CID_FREEZE_AGCAEC,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "Freeze AGC/AEC",
		.minimum = 0,
		.maximum = 1,
		.step = 0,
		.default_value = DEF_FREEZE_AGCAEC,
	}, {
		.id = V4L2_CID_RED_BALANCE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Red Balance",
		.minimum = 0,
		.maximum = 255,
		.step = 1,
		.default_value = DEF_RED,
	}, {
		.id = V4L2_CID_BLUE_BALANCE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Blue Balance",
		.minimum = 0,
		.maximum = 255,
		.step = 1,
		.default_value = DEF_BLUE,
	}, {
		.id = V4L2_CID_AUTO_WHITE_BALANCE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "Auto White Balance",
		.minimum = 0,
		.maximum = 1,
		.step = 0,
		.default_value = DEF_AWB,
	}, {
		.id = V4L2_CID_HFLIP,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "Mirror Image",
		.minimum = 0,
		.maximum = 1,
		.step = 0,
		.default_value = DEF_HFLIP,
	}, {
		.id = V4L2_CID_VFLIP,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "Vertical Flip",
		.minimum = 0,
		.maximum = 1,
		.step = 0,
		.default_value = DEF_VFLIP,
	},
};



const static struct ov96xx_reg *
	ov96xx_reg_init[NUM_PIXEL_FORMATS][NUM_IMAGE_SIZES] =
{
 { qqcif_yuv, qqvga_yuv, qcif_yuv, qvga_yuv, cif_yuv, vga_yuv, sxga_yuv },
 { qqcif_565, qqvga_565, qcif_565, qvga_565, cif_565, vga_565, sxga_565 },
 { qqcif_555, qqvga_555, qcif_555, qvga_555, cif_555, vga_555, sxga_555 },
};


/*
 * Read a value from a register in an OV96xx sensor device.
 * The value is returned in 'val'.
 * Returns zero if successful, or non-zero otherwise.
 */
static int
ov96xx_read_reg(struct i2c_client *client, u8 reg, u8 *val)
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

/*
 * Write a value to a register in an OV96xx sensor device.
 * Returns zero if successful, or non-zero otherwise.
 */
static int ov96xx_write(struct i2c_client *client, u8 reg, u8 val)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 2;
	msg->buf = data;
	data[0] = reg;
	data[1] = val;
	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0)
		return 0;
	return err;
}

static int
ov96xx_write_reg_mask(struct i2c_client *client, u8 reg, u8 *val, u8 mask)
{
	u8 oldval, newval;
	int rc;

	if (mask == 0xff)
		newval = *val;
	else {
		/* need to do read - modify - write */
		rc = ov96xx_read_reg(client, reg, &oldval);
		if (rc)
			return rc;
		oldval &= (~mask);              /* Clear the masked bits */
		*val &= mask;                  /* Enforce mask on value */
		newval = oldval | *val;        /* Set the desired bits */
	}

	/* write the new value to the register */
	rc = ov96xx_write(client, reg, newval);
	if (rc)
		return rc;

	rc = ov96xx_read_reg(client, reg, &newval);
	if (rc)
		return rc;

	*val = newval & mask;
	return 0;
}

static int
ov96xx_read_reg_mask(struct i2c_client *client, u8 reg, u8 *val, u8 mask)
{
	int rc;

	rc = ov96xx_read_reg(client, reg, val);
	if (rc)
		return rc;
	(*val) &= mask;

	return 0;
}

/*
 * Initialize a list of OV9640 registers.
 * The list of registers is terminated by the pair of values
 * { OV96XX_REG_TERM, OV96XX_VAL_TERM }.
 * Returns zero if successful, or non-zero otherwise.
 */
static int
ov96xx_write_regs(struct i2c_client *client, const struct ov96xx_reg reglist[])
{
	int err;
	const struct ov96xx_reg *next = reglist;

	while (!((next->reg == OV96XX_REG_TERM)
		&& (next->val == OV96XX_VAL_TERM))) {
		err = ov96xx_write(client, next->reg, next->val);
		udelay(100);
		if (err)
			return err;
		next++;
	}
	return 0;
}

/* Returns the index of the requested ID from the control structure array */
static int
find_vctrl(int id)
{
	int i;

	if (id < V4L2_CID_BASE)
		return -EDOM;

	for (i = (ARRAY_SIZE(video_control) - 1); i >= 0; i--)
		if (ov96xx_controls[i].id == id)
			break;
	if (i < 0)
		i = -EINVAL;
	return i;
}

/* Calculate the internal clock divisor (value of the CLKRC register) of the
 * OV9640 given the image size, the frequency (in Hz) of its XCLK input and a
 * desired frame period (in seconds).  The frame period 'fper' is expressed as
 * a fraction.  The frame period is an input/output parameter.
 * Returns the value of the OV9640 CLKRC register that will yield the frame
 * period returned in 'fper' at the specified xclk frequency.  The
 * returned period will be as close to the requested period as possible.
 */
static unsigned char
ov96xx_clkrc(enum image_size isize, unsigned long xclk, struct v4l2_fract *fper)
{
	unsigned long fpm, fpm_max;	/* frames per minute */
	unsigned long divisor;
	const unsigned long divisor_max = 64;
	/* FIXME
	 * clks_per_frame should come from platform data
	 */
#ifdef CONFIG_ARCH_OMAP24XX
	const static unsigned long clks_per_frame[] =
		{ 200000, 400000, 200000, 400000, 400000, 800000, 3200000 };
      /*         QQCIF   QQVGA    QCIF    QVGA  CIF     VGA	SXGA
       *         199680,400000, 199680, 400000, 399360, 800000, 3200000
       */
#else
	const static unsigned long clks_per_frame[] =
		{ 200000, 200000, 200000, 200000, 400000, 800000, 3200000 };
#endif

	if (fper->numerator > 0)
		fpm = (fper->denominator*60)/fper->numerator;
	else
		fpm = 0xffffffff;
	fpm_max = (xclk*60)/clks_per_frame[isize];
	if (fpm_max == 0)
		fpm_max = 1;
	if (fpm > fpm_max)
		fpm = fpm_max;
	if (fpm == 0)
		fpm = 1;
	divisor = fpm_max/fpm;
	if (divisor > divisor_max)
		divisor = divisor_max;
	fper->numerator = divisor*60;
	fper->denominator = fpm_max;

	/* try to reduce the fraction */
	while (!(fper->denominator % 5) && !(fper->numerator % 5)) {
		fper->numerator /= 5;
		fper->denominator /= 5;
	}
	while (!(fper->denominator % 3) && !(fper->numerator % 3)) {
		fper->numerator /= 3;
		fper->denominator /= 3;
	}
	while (!(fper->denominator % 2) && !(fper->numerator % 2)) {
		fper->numerator /= 2;
		fper->denominator /= 2;
	}
	if (fper->numerator < fper->denominator) {
		if (!(fper->denominator % fper->numerator)) {
			fper->denominator /= fper->numerator;
			fper->numerator = 1;
		}
	} else {
		if (!(fper->numerator % fper->denominator)) {
			fper->numerator /= fper->denominator;
			fper->denominator = 1;
		}
	}

	/* we set bit 7 in CLKRC to enable the digital PLL */
	return (0x80 | (divisor - 1));
}

/*
 * Find the best match for a requested image capture size.  The best match
 * is chosen as the nearest match that has the same number or fewer pixels
 * as the requested size, or the smallest image size if the requested size
 * has fewer pixels than the smallest image.
 */
static enum image_size
ov96xx_find_size(unsigned int width, unsigned int height)
{
	enum image_size isize;
	unsigned long pixels = width*height;

	for (isize = QQCIF; isize < SXGA; isize++) {
		if (ov96xx_sizes[isize + 1].height *
			ov96xx_sizes[isize + 1].width > pixels)
			return isize;
	}
	return SXGA;
}

/*
 * The nominal xclk input frequency of the OV9640 is 24MHz, maximum
 * frequency is 48MHz, and minimum frequency is 10MHz.
 */
#define XCLK_MIN 10000000
#define XCLK_MAX 48000000
#define XCLK_NOM 24000000

/*
 * Given the image capture format in pix, the nominal frame period in
 * timeperframe, calculate the required xclk frequency
 */
static unsigned long
ov96xxsensor_calc_xclk(struct i2c_client *c)
{
	struct ov96xx_sensor *sensor = i2c_get_clientdata(c);
	struct v4l2_fract *timeperframe = &sensor->timeperframe;
	struct v4l2_pix_format *pix = &sensor->pix;

	unsigned long tgt_xclk;			/* target xclk */
	unsigned long tgt_fpm;			/* target frames per minute */
	enum image_size isize;

	/* We use arbitrary rules to select the xclk frequency.  If the
	 * capture size is VGA and the frame rate is greater than 900
	 * frames per minute, or if the capture size is SXGA and the
	 * frame rate is greater than 450 frames per minutes, then the
	 * xclk frequency will be set to 48MHz.  Otherwise, the xclk
	 * frequency will be set to 24MHz.  If the mclk frequency is such that
	 * the target xclk frequency is not achievable, then xclk will be set
	 * as close as to the target as possible.
	 */
	if ((timeperframe->numerator == 0)
		|| (timeperframe->denominator == 0)) {
		/* supply a default nominal_timeperframe of 15 fps */
		timeperframe->numerator = 1;
		timeperframe->denominator = 15;
	}
	tgt_fpm = (timeperframe->denominator*60)
		/ timeperframe->numerator;
	tgt_xclk = XCLK_NOM;
	isize = ov96xx_find_size(pix->width, pix->height);
	switch (isize) {
	case SXGA:
		if (tgt_fpm > 450)
			tgt_xclk = XCLK_MAX;
		break;
	case VGA:
		if (tgt_fpm > 900)
			tgt_xclk = XCLK_MAX;
		break;
	default:
		break;
	}
	return tgt_xclk;
}

/*
 * Configure the OV96xx for a specified image size, pixel format, and frame
 * period.  xclk is the frequency (in Hz) of the xclk input to the OV96xx.
 * fper is the frame period (in seconds) expressed as a fraction.
 * Returns zero if successful, or non-zero otherwise.
 * The actual frame period is returned in fper.
 */
static int ov96xx_configure(struct soc_camera_device *icd)
{
	struct ov96xx_sensor *sensor = container_of(icd, struct ov96xx_sensor, icd);
	struct v4l2_pix_format *pix = &sensor->pix;
	struct v4l2_fract *fper = &sensor->timeperframe;
	struct i2c_client *client = sensor->i2c_client;
	enum image_size isize;
	unsigned long xclk;

	int err;
	unsigned char clkrc;
	enum pixel_format pfmt = YUV;

	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB565X:
		pfmt = RGB565;
		break;
	case V4L2_PIX_FMT_RGB555:
	case V4L2_PIX_FMT_RGB555X:
		pfmt = RGB555;
		break;
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
	default:
		pfmt = YUV;
	}

	xclk = ov96xxsensor_calc_xclk(client);
	printk("%s xclk: %lu\n", __func__, xclk);
	return 0;

	isize = ov96xx_find_size(pix->width, pix->height);

	/* common register initialization */
	err = ov96xx_write_regs(client, sensor->pdata->default_regs);
	if (err)
		return err;

	/* configure image size and pixel format */
	err = ov96xx_write_regs(client, ov96xx_reg_init[pfmt][isize]);
	if (err)
		return err;

	/* configure frame rate */
	clkrc = ov96xx_clkrc(isize, xclk, fper);
	err = ov96xx_write(client, OV96XX_CLKRC, clkrc);
	if (err)
		return err;

	return 0;
}

/*
 * Detect if an OV96xx is present, and if so which revision.
 * A device is considered to be detected if the manufacturer ID (MIDH and MIDL)
 * and the product ID (PID) registers match the expected values.
 * Any value of the version ID (VER) register is accepted.
 * Here are the version numbers we know about:
 *	0x48 --> OV9640 Revision 1 or OV9640 Revision 2
 *	0x49 --> OV9640 Revision 3
 * Returns a negative error number if no device is detected, or the
 * non-negative value of the version ID register if a device is detected.
 */
static int
ov96xx_detect(struct i2c_client *client)
{
	u8 midh, midl, pid, ver;

	if (!client)
		return -ENODEV;

	if (ov96xx_read_reg(client, OV96XX_MIDH, &midh))
		return -ENODEV;
	if (ov96xx_read_reg(client, OV96XX_MIDL, &midl))
		return -ENODEV;
	if (ov96xx_read_reg(client, OV96XX_PID, &pid))
		return -ENODEV;
	if (ov96xx_read_reg(client, OV96XX_VER, &ver))
		return -ENODEV;

	dev_dbg(&client->dev, "MIDH=0x%02x MIDL=0x%02x PID=0x%02x VER=0x%02x\n",
			midh, midl, pid, ver);

	if ((midh != OV96XX_MIDH_MAGIC)
		|| (midl != OV96XX_MIDL_MAGIC)
		|| (pid != OV96XX_PID_MAGIC))
		/*
		 * We didn't read the values we expected, so
		 * this must not be an OV96xx.
		 */
		return -ENODEV;

	return ver;
}

static int ov96xx_get_control(struct soc_camera_device *icd,
				struct v4l2_control *vctrl)
{
	struct ov96xx_sensor *sensor = container_of(icd, struct ov96xx_sensor, icd);
	struct i2c_client *client = sensor->i2c_client;
	int i, val;
	struct vcontrol *lvc;

	i = find_vctrl(vctrl->id);
	if (i < 0)
		return -EINVAL;

	lvc = (struct vcontrol *)&video_control[i];
	if (ov96xx_read_reg_mask(client, lvc->reg, (u8 *)&val, lvc->mask))
		return -EIO;

	val = val >> lvc->start_bit;
	if (val >= 0) {
		vctrl->value = lvc->current_value = val;
		return 0;
	} else
		return val;
}

static int ov96xx_set_control(struct soc_camera_device *icd,
				struct v4l2_control *ctrl)
{
	struct ov96xx_sensor *sensor = container_of(icd, struct ov96xx_sensor, icd);
	struct i2c_client *client = sensor->i2c_client;
	struct vcontrol *lvc;
	int val = ctrl->value;
	int i;

	i = find_vctrl(ctrl->id);
	if (i < 0)
		return -EINVAL;

	lvc = (struct vcontrol *)&video_control[i];
	val = val << lvc->start_bit;
	if (ov96xx_write_reg_mask(client, lvc->reg, (u8 *)&val, (u8)lvc->mask))
		return -EIO;

	val = val >> lvc->start_bit;
	if (val >= 0) {
		lvc->current_value = val;
		return 0;
	} else
		return val;
}

static int ov96xx_get_chip_id(struct soc_camera_device *icd,
			       struct v4l2_dbg_chip_ident *id)
{
	struct ov96xx_sensor *ov96xx = container_of(icd, struct ov96xx_sensor, icd);

	if (id->match.type != V4L2_CHIP_MATCH_I2C_ADDR)
		return -EINVAL;

	if (id->match.addr != ov96xx->i2c_client->addr)
		return -ENODEV;

	id->ident	= ov96xx->model;
	id->revision	= 0;

	return 0;
}

/*
 * Negotiates the image capture size and pixel format without actually making
 * it take effect.
 */
static int ov96xx_try_fmt(struct soc_camera_device *icd, struct v4l2_format *fmt)
{
	enum image_size isize;
	int ifmt;
	struct v4l2_pix_format *pix;

	if ((!fmt) || (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)) {
		printk("Invalid format\n");
		return -EINVAL;
	}
	pix = &(fmt->fmt.pix);

	isize = ov96xx_find_size(pix->width, pix->height);

	pix->width = ov96xx_sizes[isize].width;
	pix->height = ov96xx_sizes[isize].height;

	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (pix->pixelformat == ov96xx_formats[ifmt].fourcc)
			break;
	}
	if (ifmt == NUM_CAPTURE_FORMATS)
		ifmt = 0; /* default to first format */

	pix->pixelformat = ov96xx_formats[ifmt].fourcc;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = pix->width * (ov96xx_formats[ifmt].depth <= 8 ? 1 : 2);
	printk("%s bytesperline = %d\n", __func__, pix->bytesperline);
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->priv = 0;
	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
	default:
		pix->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB565X:
	case V4L2_PIX_FMT_RGB555:
	case V4L2_PIX_FMT_RGB555X:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		break;
	}
	return 0;
}

static int ov96xx_release(struct soc_camera_device *icd)
{
	/* Disable the chip */
// 	reg_write(icd, MT9M001_OUTPUT_CONTROL, 0);
	return 0;
}

static int ov96xx_start_capture(struct soc_camera_device *icd)
{
	/* Switch to master "normal" mode */
/*	if (reg_write(icd, MT9M001_OUTPUT_CONTROL, 2) < 0)
		return -EIO;*/
	return 0;
}

static int ov96xx_stop_capture(struct soc_camera_device *icd)
{
	/* Stop sensor readout */
/*	if (reg_write(icd, MT9M001_OUTPUT_CONTROL, 0) < 0)
		return -EIO;*/
	return 0;
}

static int ov96xx_set_bus_param(struct soc_camera_device *icd,
				 unsigned long flags)
{
// 	struct ov96xx_sensor *ov96xx = container_of(icd, struct ov96xx_sensor, icd);

	printk("- %s\n", __func__);
	return 0;
}

static unsigned long ov96xx_query_bus_param(struct soc_camera_device *icd)
{
	unsigned int width_flag = SOCAM_DATAWIDTH_10 | SOCAM_DATAWIDTH_8;

	/* OV96xx has all capture_format parameters fixed */
	return SOCAM_PCLK_SAMPLE_RISING |
		SOCAM_HSYNC_ACTIVE_HIGH |
		SOCAM_VSYNC_ACTIVE_HIGH |
		SOCAM_MASTER |
		width_flag;
}

static int ov96xx_set_fmt(struct soc_camera_device *icd,
				__u32 pixfmt,
				struct v4l2_rect *rect)
{
	struct ov96xx_sensor *ov96xx = container_of(icd, struct ov96xx_sensor, icd);
//	struct v4l2_pix_format *pix = &fmt->fmt.pix;
	int ret;
	int i;

	printk("%s %d 0x%08x\n", __func__, __LINE__, pixfmt);

	for (i = 0; i < NUM_CAPTURE_FORMATS; i++) {
		if (pixfmt == ov96xx_formats[i].fourcc) {
			ov96xx->pix.pixelformat = ov96xx_formats[i].fourcc;
			icd->current_fmt = &ov96xx_formats[i];
			break;
		}
	}
	if (i == NUM_CAPTURE_FORMATS)
		return -EINVAL;

	printk("%s %d 0x%08x\n", __func__, __LINE__, pixfmt);

	ov96xx->pix.width = rect->width;
	ov96xx->pix.height = rect->height;
/*	rval = ov96xx_try_fmt(icd, fmt);
	if (rval)
		return rval; */

	ret = ov96xx_configure(icd);

/*	if (!ret)
		ov96xx->pix = *pix; */

	return ret;
}

static void ov96xx_reset(struct i2c_client *client)
{
	ov96xx_write(client, REG_COM7, COM7_RESET);
	msleep(1);
}

static int ov96xx_init(struct soc_camera_device *icd)
{
	struct ov96xx_sensor *ov96xx = container_of(icd, struct ov96xx_sensor, icd);
	struct soc_camera_link *icl = ov96xx->i2c_client->dev.platform_data;
	int ret;

	if (icl->power) {
		ret = icl->power(&ov96xx->i2c_client->dev, 1);
		if (ret < 0) {
			dev_err(icd->vdev->parent,
				"Platform failed to power-on the camera.\n");
			return ret;
		}
	}

	return 0; /*ov96xx_configure(icd);*/
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov96xx_get_register(struct soc_camera_device *icd,
				struct v4l2_dbg_register *reg)
{
	struct ov96xx_sensor *ov96xx = container_of(icd, struct ov96xx_sensor, icd);
	int ret;
	u8 val;

	if (reg->match.type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
		return -EINVAL;

	if (reg->match.addr != ov96xx->i2c_client->addr)
		return -ENODEV;

	ret = ov96xx_read_reg(ov96xx->i2c_client, (u8)reg->reg, &val);
	if (ret)
		return ret;
	reg->val = val;

	return 0;
}

static int ov96xx_set_register(struct soc_camera_device *icd,
				struct v4l2_dbg_register *reg)
{
	struct ov96xx_sensor *ov96xx = container_of(icd, struct ov96xx_sensor, icd);
	int ret;

	if (reg->match.type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
		return -EINVAL;

	if (reg->match.addr != ov96xx->i2c_client->addr)
		return -ENODEV;

	ret = ov96xx_write(ov96xx->i2c_client, (u8)reg->reg, (u8)reg->val);
	if (ret)
		return ret;

	return 0;
}
#endif /* CONFIG_VIDEO_ADV_DEBUG */

static u8 current_reg_addr;

#ifdef DEBUG
static int ov96xx_proc_read_register( __attribute__ ((unused)) struct file *file, const char *buf, unsigned long count, void *data)
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
	current_reg_addr = reg_addr;
	ov96xx_read_reg(i2c_client, reg_addr, &reg_val);
	printk("Read 0x%02x @ 0x%02x\n", reg_val, reg_addr);

	return len;
}

static int ov96xx_proc_write_register( __attribute__ ((unused)) struct file *file, const char *buf, unsigned long count, void *data)
{
	int len, ret;
	char given_param[16];
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

	reg_val = (u8)(simple_strtol(given_param, 0, 16));
	printk("Writing 0x%02x @ 0x%02x\n", reg_val, current_reg_addr);
/* 	ret = ov96xx_write(i2c_client, current_reg_addr, reg_val); */
	ret = i2c_smbus_write_byte_data(i2c_client, current_reg_addr, reg_val);
	udelay(100);
	if (ret) {
		printk("Error while writing\n");
	}

	return len;
}
#endif /* DEBUG */


/* Interface active, can use i2c. If it fails, it can indeed mean, that
 * this wasn't our capture interface, so, we wait for the right one */
static int ov96xx_video_probe(struct soc_camera_device *icd)
{
	struct ov96xx_sensor *ov96xx = container_of(icd, struct ov96xx_sensor, icd);
	struct soc_camera_link *icl = ov96xx->i2c_client->dev.platform_data;
#ifdef DEBUG
	struct proc_dir_entry *proc_read, *proc_write;
#endif
	struct i2c_client *client = ov96xx->i2c_client;
	int version, ret;

	/* Make the default capture format QCIF RGB565 */
/*	ov96xx->pix.width = ov96xx_sizes[QCIF].width;
	ov96xx->pix.height = ov96xx_sizes[QCIF].height;
	ov96xx->pix.pixelformat = V4L2_PIX_FMT_RGB565; */

	if (icl->reset) {
		ret = icl->reset(&ov96xx->i2c_client->dev);
	} else {
		dev_info(&client->dev, "No hardware reset available,"
					" using soft one\n");
		ov96xx_reset(client); /* soft reset */
	}

	version = ov96xx_detect(client);
	if (version < 0) {
		dev_err(&client->dev, "Unable to detect " DRIVER_NAME " sensor\n");
		return version;
	}

	dev_info(&icd->dev, "Detected an ov96%02x sensor\n", version);
// 	switch (version) { ??
	ov96xx->model = V4L2_IDENT_OV9653;
	icd->formats = ov96xx_formats;
	icd->num_formats = ARRAY_SIZE(ov96xx_formats);

	/* configure image size and pixel format */
	ret = ov96xx_write_regs(client, ov96xx_reg_init[YUV][QVGA]);
	if (ret)
		goto err;

#ifdef DEBUG
	/* create proc files */
	proc_read = create_proc_entry("driver/ovread", S_IWUSR | S_IRGRP | S_IROTH, NULL);
	proc_write = create_proc_entry("driver/ovwrite", S_IWUSR | S_IRGRP | S_IROTH, NULL);
	if ((proc_read == NULL) || (proc_write == NULL)) {
		printk(KERN_ERR DRIVER_NAME ": Could not register one /proc file. Terminating\n");
		return -ENOMEM;
	} else {
		proc_read->write_proc = ov96xx_proc_read_register;
		proc_read->data = client;
		proc_write->write_proc = ov96xx_proc_write_register;
		proc_write->data = client;
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

static void ov96xx_video_remove(struct soc_camera_device *icd)
{
	struct ov96xx_sensor *ov96xx = container_of(icd, struct ov96xx_sensor, icd);

	dev_dbg(&icd->dev, "Video %x removed: %p, %p\n", ov96xx->i2c_client->addr,
		ov96xx->icd.dev.parent, ov96xx->icd.vdev);
	soc_camera_video_stop(icd);
}

static int ov96xx_suspend(struct soc_camera_device *icd, pm_message_t state)
{
	struct ov96xx_sensor *ov96xx = container_of(icd, struct ov96xx_sensor, icd);
	struct soc_camera_link *icl = ov96xx->i2c_client->dev.platform_data;
	int ret;

	printk("--- %s %d\n", __func__, state.event);

	if (icl->power) {
		ret = icl->power(&ov96xx->i2c_client->dev, 0);
		if (ret < 0) {
			dev_err(icd->vdev->parent,
				"Platform failed to powerdown the camera.\n");
			return ret;
		}
	}

	return 0;
}

static int ov96xx_resume(struct soc_camera_device *icd)
{
	struct ov96xx_sensor *ov96xx = container_of(icd, struct ov96xx_sensor, icd);
	struct soc_camera_link *icl = ov96xx->i2c_client->dev.platform_data;
	int ret;

	printk("--- %s\n", __func__);

	if (icl->power) {
		ret = icl->power(&ov96xx->i2c_client->dev, 1);
		if (ret < 0) {
			dev_err(icd->vdev->parent,
				"Platform failed to resume the camera.\n");
			return ret;
		}
	}

	return 0;
}

static struct soc_camera_ops ov96xx_ops = {
	.owner			= THIS_MODULE,
	.probe			= ov96xx_video_probe,
	.remove			= ov96xx_video_remove,
	.suspend		= ov96xx_suspend,
	.resume			= ov96xx_resume,
	.init			= ov96xx_init,
	.release		= ov96xx_release,
	.start_capture		= ov96xx_start_capture,
	.stop_capture		= ov96xx_stop_capture,
	.set_fmt		= ov96xx_set_fmt,
	.try_fmt		= ov96xx_try_fmt,
	.set_bus_param		= ov96xx_set_bus_param,
	.query_bus_param	= ov96xx_query_bus_param,
	.controls		= ov96xx_controls,
	.num_controls		= ARRAY_SIZE(ov96xx_controls),
	.get_control		= ov96xx_get_control,
	.set_control		= ov96xx_set_control,
	.get_chip_id		= ov96xx_get_chip_id,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.get_register		= ov96xx_get_register,
	.set_register		= ov96xx_set_register,
#endif
};

static int __init ov96xx_probe(struct i2c_client *client,
				const struct i2c_device_id *did)
{
	struct ov96xx_sensor *ov96xx;
	struct soc_camera_device *icd;
	struct soc_camera_link *icl = client->dev.platform_data;
	int ret;

	if (!icl) {
		dev_err(&client->dev, DRIVER_NAME " driver needs platform data\n");
		return -EINVAL;
	}

	if (i2c_get_clientdata(client))
		return -EBUSY;

	ov96xx = kzalloc(sizeof(struct ov96xx_sensor), GFP_KERNEL);
	if (!ov96xx)
		return -ENOMEM;

// 	sensor->v4l2_int_device = &ov96xx_int_device;
	ov96xx->i2c_client = client;
	i2c_set_clientdata(client, ov96xx);

	/* Second stage probe - when a capture adapter is there */
	icd = &(ov96xx->icd);
	icd->ops	= &ov96xx_ops;
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
/*	icd->iface	= icl->bus_id; XXX */

	ret = soc_camera_device_register(icd);
	if (ret)
		goto err;

	return 0;

err:
	i2c_set_clientdata(client, NULL);
	kfree(ov96xx);
	return ret;
}

static int __exit ov96xx_remove(struct i2c_client *client)
{
 	struct ov96xx_sensor *ov96xx = i2c_get_clientdata(client);

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */
#ifdef DEBUG
	remove_proc_entry("driver/ovread", NULL);
	remove_proc_entry("driver/ovwrite", NULL);
#endif
	soc_camera_device_unregister(&ov96xx->icd);
	i2c_set_clientdata(client, NULL);
	kfree(ov96xx);

	return 0;
}

/* auto-detection */
static const struct i2c_device_id ov96xx_id[] = {
	{ DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov96xx_id);

static struct i2c_driver ov96xx_i2c_driver = {
	.driver = {
		.name	= DRIVER_NAME,
	},
	.probe = ov96xx_probe,
	.remove = __exit_p(ov96xx_remove),
	.id_table = ov96xx_id,
};


static int __init ov96xx_mod_init(void)
{
	printk(KERN_NOTICE "OmniVision ov96xx sensor driver, at your service\n");
	return i2c_add_driver(&ov96xx_i2c_driver);
}

static void __exit ov96xx_mod_exit(void)
{
	i2c_del_driver(&ov96xx_i2c_driver);
}

module_init(ov96xx_mod_init);
module_exit(ov96xx_mod_exit);

MODULE_DESCRIPTION("SoC camera driver for OV96xx");
MODULE_AUTHOR("Julien Boibessot <julien.boibessot@armadeus.com>");
MODULE_LICENSE("GPL");
