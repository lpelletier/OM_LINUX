/*
 * OV3640 driver (soc_camera)
 *
 * Copyright 2010 Armadeus Systems
 *
 * Based on drivers which are:
 * Copyright 2005-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2010 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#define DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
/*#include <media/v4l2-int-device.h>
#include "mxc_v4l2_capture.h" */
#include <media/soc_camera.h>
#include <media/v4l2-chip-ident.h>

#include "ov3640_regs.h"

#define DRIVER_NAME "ov3640"

#define OV3640_VOLTAGE_ANALOG               2800000
#define OV3640_VOLTAGE_DIGITAL_CORE         1500000
#define OV3640_VOLTAGE_DIGITAL_IO           1800000


/* Check these values! */
#define MIN_FPS 15
#define MAX_FPS 30
#define DEFAULT_FPS 30

#define OV3640_XCLK_MIN 6000000
#define OV3640_XCLK_MAX 24000000

#define OV3640_MIN_BRIGHT              0
#define OV3640_MAX_BRIGHT              6
#define OV3640_DEF_BRIGHT              0
#define OV3640_BRIGHT_STEP             1

#define OV3640_DEF_CONTRAST            0
#define OV3640_MIN_CONTRAST            0
#define OV3640_MAX_CONTRAST            6
#define OV3640_CONTRAST_STEP           1

#define OV3640_DEF_COLOR               0
#define OV3640_MIN_COLOR               0
#define OV3640_MAX_COLOR               2
#define OV3640_COLOR_STEP              1

/* Check these values : */
#define OV3640_DEF_GAIN			31
#define OV3640_DEF_AUTOGAIN		1
#define OV3640_DEF_EXPOSURE		154
#define OV3640_DEF_AEC			1
#define OV3640_DEF_FREEZE_AGCAEC	0
#define OV3640_DEF_BLUE			153
#define OV3640_DEF_RED			(255 - OV3640_DEF_BLUE)
#define OV3640_DEF_AWB			1
#define OV3640_DEF_HFLIP		0
#define OV3640_DEF_VFLIP		0


enum ov3640_mode {
	ov3640_mode_MIN = 0,
	ov3640_mode_VGA_640_480 = 0,
	ov3640_mode_QVGA_320_240 = 1,
	ov3640_mode_QXGA_2048_1536 = 2,
	ov3640_mode_XGA_1024_768 = 3,
	ov3640_mode_MAX = 3
};

enum ov3640_frame_rate {
	ov3640_15_fps,
	ov3640_30_fps
};

struct reg_value {
	u16 u16RegAddr;
	u8 u8Val;
	u8 u8Mask;
	u32 u32Delay_ms;
};

struct ov3640_mode_info {
	enum ov3640_mode mode;
	u32 width;
	u32 height;
	struct reg_value *init_data_ptr;
	u32 init_data_size;
};

struct capture_size {
	unsigned long width;
	unsigned long height;
};

/*
 * Array of image sizes supported by OV3640.  These must be ordered from
 * smallest image size to largest.
 */
const static struct capture_size ov3640_sizes[] = {
	{  320, 240 },	/* QVGA */
	{  640, 480 },	/* VGA */
	{ 1024, 768 },  /* XGA */
	{ 1280, 960 },	/* SXGA */
	{ 2048, 1536},  /* QXGA */
	{    0,   0 },  /* END */
};

/*!
 * Maintains the information on the current state of the sesor.
 */
struct ov3640_sensor {
	struct soc_camera_device icd;
	int model;
	int ver;

	const struct ov3640_platform_data *platform_data;
	struct v4l2_int_device *v4l2_int_device;
	struct i2c_client *i2c_client;
	struct v4l2_pix_format pix;
	struct v4l2_captureparm streamcap;
	bool on;

	/* control settings */
	int brightness;
	int hue;
	int contrast;
	int saturation;
	int red;
	int green;
	int blue;
	int ae_mode;

	u32 mclk;
	int csi;
};

/*struct ov3640_sensor ov3640_data; */

/* List of image formats supported by OV3640 sensor */
static const struct soc_camera_data_format ov3640_formats[] = {
#ifdef OV3640_RAW_MODE
       {
		.name = "RAW10",
		.fourcc = V4L2_PIX_FMT_SGRBG10,
		.depth = 16,
       },
#else
       {
		.name = "RGB565, le",
		.fourcc = V4L2_PIX_FMT_RGB565,
		.depth = 16,
       },
       {
		.name = "RGB565, be",
		.fourcc = V4L2_PIX_FMT_RGB565X,
		.depth = 16,
       },
       {
		.name = "YUYV (YUV 4:2:2), packed",
		.fourcc = V4L2_PIX_FMT_YUYV,
		.depth = 16,
       },
       {
		.name = "UYVY, packed",
		.fourcc = V4L2_PIX_FMT_UYVY,
		.depth = 16,
       },
       {
		.name = "RGB555, le",
		.fourcc = V4L2_PIX_FMT_RGB555,
		.depth = 16,
       },
       {
		.name = "RGB555, be",
		.fourcc = V4L2_PIX_FMT_RGB555X,
		.depth = 16,
       },
#endif
};

#define NUM_CAPTURE_FORMATS (sizeof(ov3640_formats) / sizeof(ov3640_formats[0]))

static struct reg_value ov3640_setting_15fps_QXGA_2048_1536[] = {
	{OV3640_SYS, 0x80, 0, 0},
	{0x304d, 0x45, 0, 0},
	{0x30a7, 0x5e, 0, 0},
	{OV3640_TMC11, 0x16, 0, 0},
	{0x309c, 0x1a, 0, 0},	/* reserved */
	{0x30a2, 0xe4, 0, 0},	/* reserved */
	{0x30aa, 0x42, 0, 0},	/* reserved */
	{OV3640_IO_CTRL0, 0xff, 0, 0},
	{OV3640_IO_CTRL1, 0xff, 0, 0},
	{OV3640_IO_CTRL2, 0x10, 0, 0},
	{OV3640_PLL_1, 0x32, 0, 0},
	{OV3640_PLL_2, 0x21, 0, 0},
	{OV3640_PLL_3, 0x20, 0, 0},
	{OV3640_CLK, 0x00, 0, 0},
	{0x304c, 0x81, 0, 0},	/* reserved */
	{0x30d7, 0x10, 0, 0},	/* reserved */
	{0x30d9, 0x0d, 0, 0},	/* reserved */
	{0x30db, 0x08, 0, 0},	/* reserved */
	{0x3016, 0x82, 0, 0},	/* reserved */
	{OV3640_WPT_HISH, 0x38, 0, 0},
	{OV3640_BPT_HISL, 0x30, 0, 0},
	{OV3640_VPT, 0x61, 0, 0},
	{OV3640_TMC7, 0x00, 0, 0},
	{OV3640_TMC11, 0x02, 0, 0},
	{0x3082, 0x20, 0, 0},	/* reserved */
	{OV3640_AUTO_3, 0x12, 0, 0},
	{OV3640_AUTO_2, 0x04, 0, 0},
	{OV3640_AUTO_1, 0xf7, 0, 0},
	{OV3640_AHW_H, 0x08, 0, 0},
	{OV3640_AHW_L, 0x18, 0, 0},
	{OV3640_AVH_H, 0x06, 0, 0},
	{OV3640_AVH_L, 0x0c, 0, 0},
	{OV3640_WEIGHT0, 0x62, 0, 0},
	{OV3640_WEIGHT1, 0x26, 0, 0},
	{OV3640_WEIGHT2, 0xe6, 0, 0},
	{OV3640_WEIGHT3, 0x6e, 0, 0},
	{OV3640_WEIGHT4, 0xea, 0, 0},
	{OV3640_WEIGHT5, 0xae, 0, 0},
	{OV3640_WEIGHT6, 0xa6, 0, 0},
	{OV3640_WEIGHT7, 0x6a, 0, 0},
	{OV3640_SC_SYN_CTRL0, 0x02, 0, 0},
	{OV3640_SC_SYN_CTRL1, 0xfd, 0, 0},
	{OV3640_SC_SYN_CTRL2, 0x00, 0, 0},
	{OV3640_SC_SYN_CTRL3, 0xff, 0, 0},
	{OV3640_DSP_CTRL_0, 0x12, 0, 0},
	{OV3640_DSP_CTRL_1, 0xde, 0, 0},
	{OV3640_DSP_CTRL_2, 0xcf, 0, 0},
	{0x3312, 0x26, 0, 0},
	{0x3314, 0x42, 0, 0},
	{0x3313, 0x2b, 0, 0},
	{0x3315, 0x42, 0, 0},
	{0x3310, 0xd0, 0, 0},
	{0x3311, 0xbd, 0, 0},
	{0x330c, 0x18, 0, 0},
	{0x330d, 0x18, 0, 0},
	{0x330e, 0x56, 0, 0},
	{0x330f, 0x5c, 0, 0},
	{0x330b, 0x1c, 0, 0},
	{0x3306, 0x5c, 0, 0},
	{0x3307, 0x11, 0, 0},
	{OV3640_R_A1, 0x52, 0, 0},
	{OV3640_G_A1, 0x46, 0, 0},
	{OV3640_B_A1, 0x38, 0, 0},
	{OV3640_DSPC0, 0x20, 0, 0},
	{OV3640_DSPC1, 0x17, 0, 0},
	{OV3640_DSPC2, 0x04, 0, 0},
	{OV3640_DSPC3, 0x08, 0, 0},
	{0x3507, 0x06, 0, 0},
	{0x350a, 0x4f, 0, 0},
	{OV3640_SC_CTRL0, 0x02, 0, 0},
	{OV3640_DSP_CTRL_1, 0xde, 0, 0},
	{OV3640_DSP_CTRL_4, 0x00, 0, 0},
	{OV3640_FMT_MUX_CTRL0, 0x00, 0, 0},
	{OV3640_FMT_CTRL00, 0x02, 0, 0},
	{OV3640_OUT_CTRL00, 0xc4, 0, 0},
	{OV3640_DSP_CTRL_2, 0xef, 0, 0},
	{OV3640_HS_H, 0x01, 0, 0},
	{OV3640_HS_L, 0x1d, 0, 0},
	{OV3640_VS_H, 0x00, 0, 0},
	{OV3640_VS_L, 0x0a, 0, 0},
	{OV3640_HW_H, 0x08, 0, 0},
	{OV3640_HW_L, 0x00, 0, 0},
	{OV3640_VH_H, 0x06, 0, 0},
	{OV3640_VH_L, 0x00, 0, 0},
	{OV3640_SIZE_IN_MISC, 0x68, 0, 0},
	{OV3640_HSIZE_IN_L, 0x00, 0, 0},
	{OV3640_VSIZE_IN_L, 0x00, 0, 0},
	{OV3640_SIZE_OUT_MISC, 0x68, 0, 0},
	{OV3640_HSIZE_OUT_L, 0x00, 0, 0},
	{OV3640_VSIZE_OUT_L, 0x00, 0, 0},
	{OV3640_ISP_PAD_CTR2, 0x00, 0, 0},
	{OV3640_ISP_XOUT_H, 0x08, 0, 0},
	{OV3640_ISP_XOUT_L, 0x00, 0, 0},
	{OV3640_ISP_YOUT_H, 0x06, 0, 0},
	{OV3640_ISP_YOUT_L, 0x00, 0, 0},
	{OV3640_TMC6, 0x10, 0, 0},
	{0x3090, 0xc0, 0, 0},
	{0x304c, 0x84, 0, 0},	/* reserved */
	{OV3640_TMC13, 0x04, 0, 0},
	{OV3640_TMC10, 0x03, 0, 0},
	{OV3640_TMC10, 0x00, 0, 0},
	{OV3640_SYS, 0x00, 0, 0},
	{OV3640_HS_H, 0x01, 0, 0},
	{OV3640_HS_L, 0x1d, 0, 0},
	{OV3640_VS_H, 0x00, 0, 0},
	{OV3640_VS_L, 0x0a, 0, 0},
	{OV3640_HW_H, 0x08, 0, 0},
	{OV3640_HW_L, 0x18, 0, 0},
	{OV3640_VH_H, 0x06, 0, 0},
	{OV3640_VH_L, 0x0c, 0, 0},
	{OV3640_VTS_H, 0x06, 0, 0},
	{OV3640_VTS_L, 0x20, 0, 0},
	{OV3640_VSYNCOPT, 0x44, 0, 0},
	{OV3640_PCLK, 0x00, 0, 0},
	{0x30d7, 0x00, 0, 0},	/* reserved */
	{OV3640_BLC9, 0x40, 0, 0},
	{OV3640_AVH_H, 0x01, 0, 0},
	{OV3640_AVH_L, 0x80, 0, 0},
	{OV3640_DSP_CTRL_2, 0x20, 0, 0},
	{OV3640_SIZE_IN_MISC, 0x68, 0, 0},
	{OV3640_HSIZE_IN_L, 0x18, 0, 0},
	{OV3640_VSIZE_IN_L, 0x0c, 0, 0},
	{OV3640_SIZE_OUT_MISC, 0x68, 0, 0},
	{OV3640_HSIZE_OUT_L, 0x08, 0, 0},
	{OV3640_VSIZE_OUT_L, 0x04, 0, 0},
	{OV3640_ISP_PAD_CTR2, 0x42, 0, 0},
	{OV3640_ISP_XOUT_H, 0x08, 0, 0},
	{OV3640_ISP_XOUT_L, 0x00, 0, 0},
	{OV3640_ISP_YOUT_H, 0x06, 0, 0},
	{OV3640_ISP_YOUT_L, 0x00, 0, 0},
};

static struct reg_value ov3640_setting_15fps_XGA_1024_768[] = {
	{OV3640_SYS, 0x80, 0, 0}, {0x304d, 0x45, 0, 0}, {0x30a7, 0x5e, 0, 0},
	{OV3640_TMC11, 0x16, 0, 0}, {0x309c, 0x1a, 0, 0},	/* reserved */ {0x30a2, 0xe4, 0, 0},	/* reserved */
	{0x30aa, 0x42, 0, 0},	/* reserved */ {OV3640_IO_CTRL0, 0xff, 0, 0}, {OV3640_IO_CTRL1, 0xff, 0, 0},
	{OV3640_IO_CTRL2, 0x10, 0, 0}, {OV3640_PLL_1, 0x32, 0, 0}, {OV3640_PLL_2, 0x21, 0, 0},
	{OV3640_PLL_3, 0x20, 0, 0}, {OV3640_CLK, 0x00, 0, 0}, {0x304c, 0x81, 0, 0},	/* reserved */
	{0x3016, 0x82, 0, 0},	/* reserved */ {OV3640_WPT_HISH, 0x38, 0, 0}, {OV3640_BPT_HISL, 0x30, 0, 0},
	{OV3640_VPT, 0x61, 0, 0}, {OV3640_TMC7, 0x00, 0, 0}, {OV3640_TMC11, 0x02, 0, 0},
	{0x3082, 0x20, 0, 0},	/* reserved */ {OV3640_AUTO_3, 0x12, 0, 0}, {OV3640_AUTO_2, 0x04, 0, 0},
	{OV3640_AUTO_1, 0xf7, 0, 0}, {OV3640_AHW_H, 0x08, 0, 0}, {OV3640_AHW_L, 0x18, 0, 0},
	{OV3640_AVH_H, 0x06, 0, 0}, {OV3640_AVH_L, 0x0c, 0, 0}, {OV3640_WEIGHT0, 0x62, 0, 0},
	{OV3640_WEIGHT1, 0x26, 0, 0}, {OV3640_WEIGHT2, 0xe6, 0, 0}, {OV3640_WEIGHT3, 0x6e, 0, 0},
	{OV3640_WEIGHT4, 0xea, 0, 0}, {OV3640_WEIGHT5, 0xae, 0, 0}, {OV3640_WEIGHT6, 0xa6, 0, 0},
	{OV3640_WEIGHT7, 0x6a, 0, 0}, {OV3640_SC_SYN_CTRL0, 0x02, 0, 0}, {OV3640_SC_SYN_CTRL1, 0xfd, 0, 0},
	{OV3640_SC_SYN_CTRL2, 0x00, 0, 0}, {OV3640_SC_SYN_CTRL3, 0xff, 0, 0}, {OV3640_DSP_CTRL_0, 0x12, 0, 0},
	{OV3640_DSP_CTRL_1, 0xde, 0, 0}, {OV3640_DSP_CTRL_2, 0xcf, 0, 0}, {0x3312, 0x26, 0, 0},
	{0x3314, 0x42, 0, 0}, {0x3313, 0x2b, 0, 0}, {0x3315, 0x42, 0, 0},
	{0x3310, 0xd0, 0, 0}, {0x3311, 0xbd, 0, 0}, {0x330c, 0x18, 0, 0},
	{0x330d, 0x18, 0, 0}, {0x330e, 0x56, 0, 0}, {0x330f, 0x5c, 0, 0},
	{0x330b, 0x1c, 0, 0}, {0x3306, 0x5c, 0, 0}, {0x3307, 0x11, 0, 0},
	{OV3640_R_A1, 0x52, 0, 0}, {OV3640_G_A1, 0x46, 0, 0}, {OV3640_B_A1, 0x38, 0, 0},
	{OV3640_DSPC0, 0x20, 0, 0}, {OV3640_DSPC1, 0x17, 0, 0}, {OV3640_DSPC2, 0x04, 0, 0},
	{OV3640_DSPC3, 0x08, 0, 0}, {0x3507, 0x06, 0, 0}, {0x350a, 0x4f, 0, 0},
	{OV3640_SC_CTRL0, 0x02, 0, 0}, {OV3640_DSP_CTRL_1, 0xde, 0, 0}, {OV3640_DSP_CTRL_4, 0x00, 0, 0},
	{OV3640_FMT_MUX_CTRL0, 0x01, 0, 0}, {OV3640_FMT_CTRL00, 0x1d, 0, 0}, {OV3640_OUT_CTRL00, 0xc4, 0, 0},
	{OV3640_DSP_CTRL_2, 0xef, 0, 0}, {OV3640_HS_H, 0x01, 0, 0}, {OV3640_HS_L, 0x1d, 0, 0},
	{OV3640_VS_H, 0x00, 0, 0}, {OV3640_VS_L, 0x0a, 0, 0}, {OV3640_HW_H, 0x08, 0, 0},
	{OV3640_HW_L, 0x00, 0, 0}, {OV3640_VH_H, 0x06, 0, 0}, {OV3640_VH_L, 0x00, 0, 0},
	{OV3640_SIZE_IN_MISC, 0x68, 0, 0}, {OV3640_HSIZE_IN_L, 0x00, 0, 0}, {OV3640_VSIZE_IN_L, 0x00, 0, 0},
	{OV3640_SIZE_OUT_MISC, 0x34, 0, 0}, {OV3640_HSIZE_OUT_L, 0x00, 0, 0}, {OV3640_VSIZE_OUT_L, 0x00, 0, 0},
	{OV3640_ISP_PAD_CTR2, 0x00, 0, 0}, {OV3640_ISP_XOUT_H, 0x04, 0, 0}, {OV3640_ISP_XOUT_L, 0x00, 0, 0},
	{OV3640_ISP_YOUT_H, 0x03, 0, 0}, {OV3640_ISP_YOUT_L, 0x00, 0, 0}, {OV3640_TMC6, 0x10, 0, 0},
	{0x3090, 0xc0, 0, 0}, {0x304c, 0x84, 0, 0},	/* reserved */ {OV3640_TMC13, 0x04, 0, 0},
	{OV3640_TMC10, 0x03, 0, 0}, {OV3640_TMC10, 0x00, 0, 0}, {OV3640_CLK, 0x01, 0, 0},
};

static struct reg_value ov3640_setting_30fps_XGA_1024_768[] = {
	{0x0, 0x0, 0}
};

static struct reg_value ov3640_setting_15fps_VGA_640_480_org[] = {
	{OV3640_SYS, OV3640_SYS_SRST, 0, 0},
	{0x304d, 0x45, 0, 0},	/* reserved */
	{0x30a7, 0x5e, 0, 0},	/* reserved */
	{OV3640_TMC11, 0x16, 0, 0},
	{0x309c, 0x1a, 0, 0},	/* reserved */
	{0x30a2, 0xe4, 0, 0},	/* reserved */
	{0x30aa, 0x42, 0, 0},	/* reserved */
	{OV3640_IO_CTRL0, 0xff, 0, 0},
	{OV3640_IO_CTRL1, 0xff, 0, 0},
	{OV3640_IO_CTRL2, 0x10, 0, 0},
	{OV3640_PLL_1, 0x32, 0, 0},
	{OV3640_PLL_2, 0x21, 0, 0},
	{OV3640_PLL_3, 0x20, 0, 0},
	{OV3640_CLK, 0x00, 0, 0},
	{0x304c, 0x81, 0, 0},	/* reserved */
	{0x30d7, 0x10, 0, 0},	/* reserved */
	{0x30d9, 0x0d, 0, 0},	/* reserved */
	{0x30db, 0x08, 0, 0},	/* reserved */
	{0x3016, 0x82, 0, 0},	/* reserved */
	{OV3640_WPT_HISH, 0x38, 0, 0},
	{OV3640_BPT_HISL, 0x30, 0, 0},
	{OV3640_VPT, 0x61, 0, 0},
	{OV3640_TMC7, 0x00, 0, 0},
	{OV3640_TMC11, 0x02, 0, 0},
	{0x3082, 0x20, 0, 0},	/* reserved */
	{OV3640_AUTO_3, 0x12, 0, 0},
	{OV3640_AUTO_2, 0x04, 0, 0},
	{OV3640_AUTO_1, 0xf7, 0, 0},
	{OV3640_AHW_H, 0x08, 0, 0},
	{OV3640_AHW_L, 0x18, 0, 0},
	{OV3640_AVH_H, 0x06, 0, 0},
	{OV3640_AVH_L, 0x0c, 0, 0},
	{OV3640_WEIGHT0, 0x62, 0, 0},
	{OV3640_WEIGHT1, 0x26, 0, 0},
	{OV3640_WEIGHT2, 0xe6, 0, 0},
	{OV3640_WEIGHT3, 0x6e, 0, 0},
	{OV3640_WEIGHT4, 0xea, 0, 0},
	{OV3640_WEIGHT5, 0xae, 0, 0},
	{OV3640_WEIGHT6, 0xa6, 0, 0},
	{OV3640_WEIGHT7, 0x6a, 0, 0},
	{OV3640_SC_SYN_CTRL0, 0x02, 0, 0},
	{OV3640_SC_SYN_CTRL1, 0xfd, 0, 0},
	{OV3640_SC_SYN_CTRL2, 0x00, 0, 0},
	{OV3640_SC_SYN_CTRL3, 0xff, 0, 0},
	{OV3640_DSP_CTRL_0, 0x12, 0, 0},
	{OV3640_DSP_CTRL_1, 0xde, 0, 0},
	{OV3640_DSP_CTRL_2, 0xcf, 0, 0},
	{0x3312, 0x26, 0, 0},	/* reserved */
	{0x3314, 0x42, 0, 0},	/* reserved */
	{0x3313, 0x2b, 0, 0},	/* reserved */
	{0x3315, 0x42, 0, 0},	/* reserved */
	{0x3310, 0xd0, 0, 0},	/* reserved */
	{0x3311, 0xbd, 0, 0},	/* reserved */
	{0x330c, 0x18, 0, 0},	/* reserved */
	{0x330d, 0x18, 0, 0},	/* reserved */
	{0x330e, 0x56, 0, 0},	/* reserved */
	{0x330f, 0x5c, 0, 0},	/* reserved */
	{0x330b, 0x1c, 0, 0},	/* reserved */
	{0x3306, 0x5c, 0, 0},	/* reserved */
	{0x3307, 0x11, 0, 0},	/* reserved */
	{OV3640_R_A1, 0x52, 0, 0},
	{OV3640_G_A1, 0x46, 0, 0},
	{OV3640_B_A1, 0x38, 0, 0},
	{OV3640_DSPC0, 0x20, 0, 0},
	{OV3640_DSPC1, 0x17, 0, 0},
	{OV3640_DSPC2, 0x04, 0, 0},
	{OV3640_DSPC3, 0x08, 0, 0},
	{0x3507, 0x06, 0, 0},	/* reserved */
	{0x350a, 0x4f, 0, 0},	/* reserved */
	{OV3640_SC_CTRL0, 0x02, 0, 0},
	{OV3640_DSP_CTRL_1, 0xde, 0, 0},
	{OV3640_DSP_CTRL_4, 0x00, 0, 0},
	{OV3640_FMT_MUX_CTRL0, 0x00, 0, 0},
	{OV3640_FMT_CTRL00, 0x42, 0, 0},
	{OV3640_OUT_CTRL00, 0xc4, 0, 0},
	{OV3640_DSP_CTRL_2, 0xef, 0, 0},
	{OV3640_HS_H, 0x01, 0, 0},
	{OV3640_HS_L, 0x1d, 0, 0},
	{OV3640_VS_H, 0x00, 0, 0},
	{OV3640_VS_L, 0x0a, 0, 0},
	{OV3640_HW_H, 0x08, 0, 0},
	{OV3640_HW_L, 0x00, 0, 0},
	{OV3640_VH_H, 0x06, 0, 0},
	{OV3640_VH_L, 0x00, 0, 0},
	{OV3640_SIZE_IN_MISC, 0x68, 0, 0},
	{OV3640_HSIZE_IN_L, 0x00, 0, 0},
	{OV3640_VSIZE_IN_L, 0x00, 0, 0},
	{OV3640_SIZE_OUT_MISC, 0x12, 0, 0},
	{OV3640_HSIZE_OUT_L, 0x80, 0, 0},
	{OV3640_VSIZE_OUT_L, 0xe0, 0, 0},
	{OV3640_ISP_PAD_CTR2, 0x00, 0, 0},
	{OV3640_ISP_XOUT_H, 0x02, 0, 0},
	{OV3640_ISP_XOUT_L, 0x80, 0, 0},
	{OV3640_ISP_YOUT_H, 0x01, 0, 0},
	{OV3640_ISP_YOUT_L, 0xe0, 0, 0},
	{OV3640_TMC6, 0x10, 0, 0},
	{0x3090, 0xc0, 0, 0},	/* reserved */
	{0x304c, 0x84, 0, 0},	/* reserved */
	{OV3640_TMC13, 0x04, 0, 0},
	{OV3640_TMC10, 0x03, 0, 0},
	{OV3640_TMC10, 0x00, 0, 0},
	{OV3640_CLK, 0x00, 0, 0},
};

static struct reg_value ov3640_setting_15fps_VGA_640_480mine[] = {
	{OV3640_SYS, OV3640_SYS_SRST, 0, 0},	/* reset */
// 	{0x304d, 0x45, 0, 0},	/* reserved */
// 	{0x30a7, 0x5e, 0, 0},	/* reserved */
	{OV3640_TMC11, 0x16, 0, 0},
// 	{0x309c, 0x1a, 0, 0},	/* reserved */
// 	{0x30a2, 0xe4, 0, 0},	/* reserved */
// 	{0x30aa, 0x42, 0, 0},	/* reserved */
	{OV3640_IO_CTRL0, 0xff, 0, 0},	/* DATA = OUT */
	{OV3640_IO_CTRL1, 0xff, 0, 0},	/* CTRL = OUT */
	{OV3640_IO_CTRL2, 0x10, 0, 0},
	{OV3640_PLL_1, 0x32, 0, 0},
	{OV3640_PLL_2, 0x21, 0, 0},
	{OV3640_PLL_3, 0x20, 0, 0},
	{OV3640_CLK, 0x00, 0, 0},
// 	{0x304c, 0x81, 0, 0},	/* reserved */
// 	{0x30d7, 0x10, 0, 0},	/* reserved */
// 	{0x30d9, 0x0d, 0, 0},	/* reserved */
// 	{0x30db, 0x08, 0, 0},	/* reserved */
// 	{0x3016, 0x82, 0, 0},	/* reserved */
	{OV3640_WPT_HISH, 0x38, 0, 0},
	{OV3640_BPT_HISL, 0x30, 0, 0},
	{OV3640_VPT, 0x61, 0, 0},
	{OV3640_TMC7, 0x00, 0, 0},
	{OV3640_TMC11, 0x02, 0, 0},
// 	{0x3082, 0x20, 0, 0},	/* reserved */
	{OV3640_AUTO_3, 0x12, 0, 0},
	{OV3640_AUTO_2, 0x04, 0, 0},
	{OV3640_AUTO_1, 0xf7, 0, 0},
	{OV3640_AHW_H, 0x08, 0, 0},
	{OV3640_AHW_L, 0x18, 0, 0},
	{OV3640_AVH_H, 0x06, 0, 0},
	{OV3640_AVH_L, 0x0c, 0, 0},
	{OV3640_WEIGHT0, 0x62, 0, 0},
	{OV3640_WEIGHT1, 0x26, 0, 0},
	{OV3640_WEIGHT2, 0xe6, 0, 0},
	{OV3640_WEIGHT3, 0x6e, 0, 0},
	{OV3640_WEIGHT4, 0xea, 0, 0},
	{OV3640_WEIGHT5, 0xae, 0, 0},
	{OV3640_WEIGHT6, 0xa6, 0, 0},
	{OV3640_WEIGHT7, 0x6a, 0, 0},
	/* Bring used modules out of reset */
	{OV3640_SC_SYN_CTRL0, 0x02, 0, 0},
	{OV3640_SC_SYN_CTRL1, 0xfd, 0, 0},
	{OV3640_SC_SYN_CTRL2, 0x00, 0, 0},
	{OV3640_SC_SYN_CTRL3, 0xff, 0, 0},
	{OV3640_DSP_CTRL_0, 0x12, 0, 0},
	{OV3640_DSP_CTRL_1, 0xde, 0, 0},
	{OV3640_DSP_CTRL_2, 0xcf, 0, 0},
// 	{0x3312, 0x26, 0, 0},	/* reserved */
// 	{0x3314, 0x42, 0, 0},	/* reserved */
// 	{0x3313, 0x2b, 0, 0},	/* reserved */
// 	{0x3315, 0x42, 0, 0},	/* reserved */
// 	{0x3310, 0xd0, 0, 0},	/* reserved */
// 	{0x3311, 0xbd, 0, 0},	/* reserved */
// 	{0x330c, 0x18, 0, 0},	/* reserved */
// 	{0x330d, 0x18, 0, 0},	/* reserved */
// 	{0x330e, 0x56, 0, 0},	/* reserved */
// 	{0x330f, 0x5c, 0, 0},	/* reserved */
// 	{0x330b, 0x1c, 0, 0},	/* reserved */
// 	{0x3306, 0x5c, 0, 0},	/* reserved */
// 	{0x3307, 0x11, 0, 0},	/* reserved */
	{OV3640_R_A1, 0x52, 0, 0},
	{OV3640_G_A1, 0x46, 0, 0},
	{OV3640_B_A1, 0x38, 0, 0},
	{OV3640_DSPC0, 0x20, 0, 0},
	{OV3640_DSPC1, 0x17, 0, 0},
	{OV3640_DSPC2, 0x04, 0, 0},
	{OV3640_DSPC3, 0x08, 0, 0},
// 	{0x3507, 0x06, 0, 0},	/* reserved */
// 	{0x350a, 0x4f, 0, 0},	/* reserved */
	{OV3640_SC_CTRL0, 0x02, 0, 0},	/* disable compression */
//	{OV3640_DSP_CTRL_1, 0xde, 0, 0},
	{OV3640_DSP_CTRL_4, 0x00, 0, 0},

	{OV3640_FMT_MUX_CTRL0, 0x01, 0, 0},	/* ISP RGB */
	{OV3640_FMT_CTRL00, 0x11, 0, 0},	/* RGB565 */
	{OV3640_OUT_CTRL00, 0xc0, 0, 0},
	{OV3640_OUT_CTRL01, 0x81, 0, 0},	/* PCLK gated */
	{OV3640_DSP_CTRL_2, 0xef, 0, 0},
	{OV3640_OUT_CTRL46, 0x4d, 0, 0},	/* HSYNC_EN + HSYNC_DVP_EN */

	{OV3640_HS_H, 0x01, 0, 0},
	{OV3640_HS_L, 0x1d, 0, 0},
	{OV3640_VS_H, 0x00, 0, 0},
	{OV3640_VS_L, 0x0a, 0, 0},
	{OV3640_HW_H, 0x08, 0, 0},
	{OV3640_HW_L, 0x00, 0, 0},
	{OV3640_VH_H, 0x06, 0, 0},
	{OV3640_VH_L, 0x00, 0, 0},
	{OV3640_SIZE_IN_MISC, 0x68, 0, 0},
	{OV3640_HSIZE_IN_L, 0x00, 0, 0},
	{OV3640_VSIZE_IN_L, 0x00, 0, 0},
	{OV3640_SIZE_OUT_MISC, 0x12, 0, 0},
	{OV3640_HSIZE_OUT_L, 0x80, 0, 0},
	{OV3640_VSIZE_OUT_L, 0xe0, 0, 0},
	{OV3640_ISP_PAD_CTR2, 0x00, 0, 0},
	/* 0x280 x 0x1e0 -> 640 x 480 */
	{OV3640_ISP_XOUT_H, 0x02, 0, 0},
	{OV3640_ISP_XOUT_L, 0x80, 0, 0},
	{OV3640_ISP_YOUT_H, 0x01, 0, 0},
	{OV3640_ISP_YOUT_L, 0xe0, 0, 0},
//	{OV3640_TMC6, 0x10, 0, 0},
// 	{0x3090, 0xc0, 0, 0},	/* reserved */
// 	{0x304c, 0x84, 0, 0},	/* reserved */
	{OV3640_TMC13, 0x04, 0, 0},
	{OV3640_TMC10, 0x03, 0, 0},
	{OV3640_TMC10, 0x00, 0, 0},
	{OV3640_CLK, 0x00, 0, 0},
};

/* Generated by Omnivision tool : */
static struct reg_value ov3640_setting_15fps_VGA_640_480[] = {
	{OV3640_SYS, 0x00, 0, 0},

	/* added by me */
	{OV3640_IO_CTRL0, 0xff, 0, 0},	/* DATA = OUT */
	{OV3640_IO_CTRL1, 0xff, 0, 0},	/* CTRL = OUT */
	{OV3640_IO_CTRL2, 0x10, 0, 0},
	{OV3640_OUT_CTRL01, 0x81, 0, 0},	/* PCLK gated */
//	{OV3640_OUT_CTRL46, 0x4d, 0, 0},	/* HSYNC_EN + HSYNC_DVP_EN */

	{OV3640_HS_H, 0x01, 0, 0},
	{OV3640_HS_L, 0x1d, 0, 0},
	{OV3640_VS_H, 0x00, 0, 0},
	{OV3640_VS_L, 0x0a, 0, 0},
	{OV3640_HW_H, 0x08, 0, 0},
	{OV3640_HW_L, 0x18, 0, 0},
	{OV3640_VH_H, 0x06, 0, 0},
	{OV3640_VH_L, 0x0c, 0, 0},

	{OV3640_VTS_H, 0x06, 0, 0},
	{OV3640_VTS_L, 0x20, 0, 0},
	{OV3640_VSYNCOPT, 0x44, 0, 0},
	{OV3640_PCLK, 0x00, 0, 0},
	{0x30d7, 0x00, 0x80, 0},
	{OV3640_BLC9, 0x40, 0x40, 0},
	{OV3640_AVH_H, 0x01, 0, 0},
	{OV3640_AVH_L, 0x80, 0, 0},

	{OV3640_DSP_CTRL_2, 0x20, 0x20, 0},
	{OV3640_SIZE_IN_MISC, 0x68, 0, 0},
	{OV3640_HSIZE_IN_L, 0x18, 0, 0},
	{OV3640_VSIZE_IN_L, 0x0c, 0, 0},
	{OV3640_SIZE_OUT_MISC, 0x12, 0, 0},// ;Zoom_out output size
	{OV3640_HSIZE_OUT_L, 0x88, 0, 0},
	{OV3640_VSIZE_OUT_L, 0xe4, 0, 0},
	{OV3640_ISP_PAD_CTR2, 0x42, 0, 0},
	/* 0x280 x 0x1e0 -> 640 x 480 */
	{OV3640_ISP_XOUT_H, 0x02, 0, 0},// ;x_output_size
	{OV3640_ISP_XOUT_L, 0x80, 0, 0},
	{OV3640_ISP_YOUT_H, 0x01, 0, 0},// ;y_output_size
	{OV3640_ISP_YOUT_L, 0xe0, 0, 0},

	{OV3640_SC_CTRL0, 0x02, 0, 0},
	{OV3640_DSP_CTRL_1, 0x10, 0x30, 0},
	{OV3640_DSP_CTRL_4, 0x00, 0x03, 0},
//	{OV3640_FMT_MUX_CTRL0, 0x01, 0, 0}, /* ISP RGB */
//	{OV3640_FMT_CTRL00, 0x11, 0, 0},    /* RGB565 */
	{OV3640_FMT_MUX_CTRL0, 0x10, 0, 0}, /* ISP YUV */
	{OV3640_FMT_CTRL00, 0x00, 0, 0},    /* YUV422 */

	{0x304c, 0x84, 0, 0},
	{OV3640_CLK, 00, 0, 0},// ;15fps
};

static struct reg_value ov3640_setting_30fps_VGA_640_480_org[] = {
	{OV3640_SYS, 0x80, 0, 0},
	{0x304d, 0x45, 0, 0},	/* reserved */
	{0x30a7, 0x5e, 0, 0},	/* reserved */
	{OV3640_TMC11, 0x16, 0, 0},
	{0x309c, 0x1a, 0, 0},	/* reserved */
	{0x30a2, 0xe4, 0, 0},	/* reserved */
	{0x30aa, 0x42, 0, 0},	/* reserved */
	{OV3640_IO_CTRL0, 0xff, 0, 0},
	{OV3640_IO_CTRL1, 0xff, 0, 0},
	{OV3640_IO_CTRL2, 0x10, 0, 0},
	{OV3640_PLL_1, 0x32, 0, 0},
	{OV3640_PLL_2, 0x21, 0, 0},
	{OV3640_PLL_3, 0x20, 0, 0},
	{OV3640_CLK, 0x01, 0, 0},
	{0x304c, 0x82, 0, 0},	/* reserved */
	{0x30d7, 0x10, 0, 0},	/* reserved */
	{0x30d9, 0x0d, 0, 0},	/* reserved */
	{0x30db, 0x08, 0, 0},	/* reserved */
	{0x3016, 0x82, 0, 0},	/* reserved */
	{OV3640_WPT_HISH, 0x38, 0, 0},
	{OV3640_BPT_HISL, 0x30, 0, 0},
	{OV3640_VPT, 0x61, 0, 0},
	{OV3640_TMC7, 0x00, 0, 0},
	{OV3640_TMC11, 0x02, 0, 0},
	{0x3082, 0x20, 0, 0},	/* reserved */
	{OV3640_AUTO_3, 0x12, 0, 0},
	{OV3640_AUTO_2, 0x0c, 0, 0},
	{OV3640_AUTO_1, 0xf7, 0, 0},
	{OV3640_AHW_H, 0x08, 0, 0},
	{OV3640_AHW_L, 0x18, 0, 0},
	{OV3640_AVH_H, 0x06, 0, 0},
	{OV3640_AVH_L, 0x0c, 0, 0},
	{OV3640_WEIGHT0, 0x62, 0, 0},
	{OV3640_WEIGHT1, 0x26, 0, 0},
	{OV3640_WEIGHT2, 0xe6, 0, 0},
	{OV3640_WEIGHT3, 0x6e, 0, 0},
	{OV3640_WEIGHT4, 0xea, 0, 0},
	{OV3640_WEIGHT5, 0xae, 0, 0},
	{OV3640_WEIGHT6, 0xa6, 0, 0},
	{OV3640_WEIGHT7, 0x6a, 0, 0},
	{OV3640_SC_SYN_CTRL0, 0x02, 0, 0},
	{OV3640_SC_SYN_CTRL1, 0xfd, 0, 0},
	{OV3640_SC_SYN_CTRL2, 0x00, 0, 0},
	{OV3640_SC_SYN_CTRL3, 0xff, 0, 0},
	{OV3640_DSP_CTRL_0, 0x12, 0, 0},
	{OV3640_DSP_CTRL_1, 0xde, 0, 0},
	{OV3640_DSP_CTRL_2, 0xcf, 0, 0},
	{0x3312, 0x26, 0, 0},	/* reserved */
	{0x3314, 0x42, 0, 0},	/* reserved */
	{0x3313, 0x2b, 0, 0},	/* reserved */
	{0x3315, 0x42, 0, 0},	/* reserved */
	{0x3310, 0xd0, 0, 0},	/* reserved */
	{0x3311, 0xbd, 0, 0},	/* reserved */
	{0x330c, 0x18, 0, 0},	/* reserved */
	{0x330d, 0x18, 0, 0},	/* reserved */
	{0x330e, 0x56, 0, 0},	/* reserved */
	{0x330f, 0x5c, 0, 0},	/* reserved */
	{0x330b, 0x1c, 0, 0},	/* reserved */
	{0x3306, 0x5c, 0, 0},	/* reserved */
	{0x3307, 0x11, 0, 0},	/* reserved */
	{OV3640_R_A1, 0x52, 0, 0},
	{OV3640_G_A1, 0x46, 0, 0},
	{OV3640_B_A1, 0x38, 0, 0},
	{OV3640_DSP_CTRL_0, 0x13, 0, 0},
	{OV3640_DSPC0, 0x20, 0, 0},
	{OV3640_DSPC1, 0x17, 0, 0},
	{OV3640_DSPC2, 0x04, 0, 0},
	{OV3640_DSPC3, 0x08, 0, 0},
	{OV3640_SC_CTRL0, 0x02, 0, 0},
	{OV3640_DSP_CTRL_1, 0x10, 0x30, 0},
	{OV3640_DSP_CTRL_4, 0x00, 0x03, 0},
	{OV3640_FMT_MUX_CTRL0, 0x00, 0, 0},
	{OV3640_FMT_CTRL00, 0x02, 0, 0},
	{OV3640_OUT_CTRL00, 0xc0, 0, 0},
	{OV3640_TMC13, 0x04, 0, 0},
	{OV3640_TMC10, 0x03, 0, 0},
	{OV3640_TMC10, 0x00, 0, 0},
	{OV3640_SYS, 0x10, 0, 0},
	{OV3640_VS_L, 0x06, 0, 0},
	{OV3640_VH_H, 0x03, 0, 0},
	{OV3640_VH_L, 0x04, 0, 0},
	{OV3640_VTS_H, 0x03, 0, 0},
	{OV3640_VTS_L, 0x10, 0, 0},
	{OV3640_VSYNCOPT, 0x24, 0, 0},
	{OV3640_PCLK, 0x01, 0, 0},
	{0x30d7, 0x80, 0x80, 0},	/* reserved */
	{OV3640_BLC9, 0x00, 0x40, 0},
	{OV3640_AVH_H, 0x00, 0, 0},
	{OV3640_AVH_L, 0xc0, 0, 0},
	{OV3640_DSP_CTRL_2, 0x20, 0x20, 0},
	{OV3640_SIZE_IN_MISC, 0x34, 0, 0},
	{OV3640_HSIZE_IN_L, 0x0c, 0, 0},
	{OV3640_VSIZE_IN_L, 0x04, 0, 0},
	{OV3640_SIZE_OUT_MISC, 0x12, 0, 0},
	{OV3640_HSIZE_OUT_L, 0x88, 0, 0},
	{OV3640_VSIZE_OUT_L, 0xe4, 0, 0},
	{OV3640_ISP_PAD_CTR2, 0x42, 0, 0},
	{OV3640_ISP_XOUT_H, 0x02, 0, 0},
	{OV3640_ISP_XOUT_L, 0x80, 0, 0},
	{OV3640_ISP_YOUT_H, 0x01, 0, 0},
	{OV3640_ISP_YOUT_L, 0xe0, 0, 0},
	{OV3640_SIZE_OUT_MISC, 0x12, 0, 0},
	{OV3640_HSIZE_OUT_L, 0x88, 0, 0},
	{OV3640_VSIZE_OUT_L, 0xe4, 0, 0},
	{OV3640_ISP_PAD_CTR2, 0x42, 0, 0},
	{OV3640_ISP_XOUT_H, 0x02, 0, 0},
	{OV3640_ISP_XOUT_L, 0x80, 0, 0},
	{OV3640_ISP_YOUT_H, 0x01, 0, 0},
	{OV3640_ISP_YOUT_L, 0xe0, 0, 0},
	{OV3640_PLL_1, 0x37, 0, 0},
	{OV3640_PLL_2, 0xe1, 0, 0},
	{OV3640_PLL_3, 0x22, 0, 0},
	{OV3640_CLK, 0x01, 0, 0},
	{0x304c, 0x84, 0, 0},	/* reserved */
	{OV3640_AUTO_2, 0x04, 0, 0},
	{OV3640_AUTO_3, 0x02, 0, 0},
	{OV3640_EXVTS_L, 0x00, 0, 0},
	{OV3640_EXVTS_H, 0x00, 0, 0},
};

static struct reg_value ov3640_setting_30fps_VGA_640_480[] = {
	{OV3640_SYS, OV3640_SYS_SRST, 0, 0},	/* reset */
// 	{0x304d, 0x45, 0, 0},	/* reserved */
// 	{0x30a7, 0x5e, 0, 0},	/* reserved */
	{OV3640_TMC11, 0x16, 0, 0},
// 	{0x309c, 0x1a, 0, 0},	/* reserved */
// 	{0x30a2, 0xe4, 0, 0},	/* reserved */
// 	{0x30aa, 0x42, 0, 0},	/* reserved */
	{OV3640_IO_CTRL0, 0xff, 0, 0},	/* DATA = OUT */
	{OV3640_IO_CTRL1, 0xff, 0, 0},	/* CTRL = OUT */
	{OV3640_IO_CTRL2, 0x10, 0, 0},
/*	{OV3640_PLL_1, 0x32, 0, 0},
	{OV3640_PLL_2, 0x21, 0, 0},
	{OV3640_PLL_3, 0x20, 0, 0},
	{OV3640_CLK, 0x01, 0, 0}, */
// 	{0x304c, 0x82, 0, 0},	/* reserved */
// 	{0x30d7, 0x10, 0, 0},	/* reserved */
// 	{0x30d9, 0x0d, 0, 0},	/* reserved */
// 	{0x30db, 0x08, 0, 0},	/* reserved */
// 	{0x3016, 0x82, 0, 0},	/* reserved */
/*	{OV3640_WPT_HISH, 0x38, 0, 0},
	{OV3640_BPT_HISL, 0x30, 0, 0},
	{OV3640_VPT, 0x61, 0, 0}, */
	{OV3640_TMC7, 0x00, 0, 0},
	{OV3640_TMC11, 0x02, 0, 0},
// 	{0x3082, 0x20, 0, 0},	/* reserved */
/*	{OV3640_AUTO_3, 0x12, 0, 0},
	{OV3640_AUTO_2, 0x0c, 0, 0},
	{OV3640_AUTO_1, 0xf7, 0, 0},
	{OV3640_AHW_H, 0x08, 0, 0},
	{OV3640_AHW_L, 0x18, 0, 0},
	{OV3640_AVH_H, 0x06, 0, 0},
	{OV3640_AVH_L, 0x0c, 0, 0},
	{OV3640_WEIGHT0, 0x62, 0, 0},
	{OV3640_WEIGHT1, 0x26, 0, 0},
	{OV3640_WEIGHT2, 0xe6, 0, 0},
	{OV3640_WEIGHT3, 0x6e, 0, 0},
	{OV3640_WEIGHT4, 0xea, 0, 0},
	{OV3640_WEIGHT5, 0xae, 0, 0},
	{OV3640_WEIGHT6, 0xa6, 0, 0},
	{OV3640_WEIGHT7, 0x6a, 0, 0}, */
	/* Bring used modules out of reset */
	{OV3640_SC_SYN_CTRL0, 0x02, 0, 0},
	{OV3640_SC_SYN_CTRL1, 0xfd, 0, 0},
	{OV3640_SC_SYN_CTRL2, 0x00, 0, 0},
	{OV3640_SC_SYN_CTRL3, 0xff, 0, 0},
	{OV3640_DSP_CTRL_0, 0x12, 0, 0},
	{OV3640_DSP_CTRL_1, 0xde, 0, 0},
	{OV3640_DSP_CTRL_2, 0xcf, 0, 0},
// 	{0x3312, 0x26, 0, 0},	/* reserved */
// 	{0x3314, 0x42, 0, 0},	/* reserved */
// 	{0x3313, 0x2b, 0, 0},	/* reserved */
// 	{0x3315, 0x42, 0, 0},	/* reserved */
// 	{0x3310, 0xd0, 0, 0},	/* reserved */
// 	{0x3311, 0xbd, 0, 0},	/* reserved */
// 	{0x330c, 0x18, 0, 0},	/* reserved */
// 	{0x330d, 0x18, 0, 0},	/* reserved */
// 	{0x330e, 0x56, 0, 0},	/* reserved */
// 	{0x330f, 0x5c, 0, 0},	/* reserved */
// 	{0x330b, 0x1c, 0, 0},	/* reserved */
// 	{0x3306, 0x5c, 0, 0},	/* reserved */
// 	{0x3307, 0x11, 0, 0},	/* reserved */
	{OV3640_R_A1, 0x52, 0, 0},
	{OV3640_G_A1, 0x46, 0, 0},
	{OV3640_B_A1, 0x38, 0, 0},
	{OV3640_DSPC0, 0x20, 0, 0},
	{OV3640_DSPC1, 0x17, 0, 0},
	{OV3640_DSPC2, 0x04, 0, 0},
	{OV3640_DSPC3, 0x08, 0, 0},
	{OV3640_SC_CTRL0, 0x02, 0, 0},	/* disable compression */
	{OV3640_DSP_CTRL_0, 0x13, 0, 0},
	{OV3640_DSP_CTRL_1, 0x10, 0x30, 0},
	{OV3640_DSP_CTRL_4, 0x00, 0x03, 0},

	{OV3640_FMT_MUX_CTRL0, 0x01, 0, 0},	/* ISP RGB */
	{OV3640_FMT_CTRL00, 0x11, 0, 0},	/* RGB565 */
	{OV3640_OUT_CTRL00, 0xc0, 0, 0},
	{OV3640_OUT_CTRL01, 0x81, 0, 0},	/* PCLK gated */
	{OV3640_DSP_CTRL_2, 0xef, 0, 0},
	{OV3640_OUT_CTRL46, 0x4d, 0, 0},	/* HSYNC_EN + HSYNC_DVP_EN */

	{OV3640_SYS, 0x10, 0, 0},	/* XGA @ 30 fps */
	{OV3640_VS_L, 0x06, 0, 0},
	{OV3640_VH_H, 0x03, 0, 0},
	{OV3640_VH_L, 0x04, 0, 0},
	{OV3640_VTS_H, 0x03, 0, 0},
	{OV3640_VTS_L, 0x10, 0, 0},
	{OV3640_VSYNCOPT, 0x24, 0, 0},
	{OV3640_PCLK, 0x01, 0, 0},
// 	{0x30d7, 0x80, 0x80, 0},	/* reserved */
	{OV3640_BLC9, 0x00, 0x40, 0},
	{OV3640_AVH_H, 0x00, 0, 0},
	{OV3640_AVH_L, 0xc0, 0, 0},
	{OV3640_DSP_CTRL_2, 0x20, 0x20, 0},
	{OV3640_SIZE_IN_MISC, 0x34, 0, 0},
	{OV3640_HSIZE_IN_L, 0x0c, 0, 0},
	{OV3640_VSIZE_IN_L, 0x04, 0, 0},
	{OV3640_SIZE_OUT_MISC, 0x12, 0, 0},
	{OV3640_HSIZE_OUT_L, 0x88, 0, 0},
	{OV3640_VSIZE_OUT_L, 0xe4, 0, 0},
	{OV3640_ISP_PAD_CTR2, 0x42, 0, 0},
	/* 0x280 x 0x1e0 -> 640 x 480 */
	{OV3640_ISP_XOUT_H, 0x02, 0, 0},
	{OV3640_ISP_XOUT_L, 0x80, 0, 0},
	{OV3640_ISP_YOUT_H, 0x01, 0, 0},
	{OV3640_ISP_YOUT_L, 0xe0, 0, 0},
	{OV3640_PLL_1, 0x37, 0, 0},
	{OV3640_PLL_2, 0xe1, 0, 0},
	{OV3640_PLL_3, 0x22, 0, 0},
// 	{0x304c, 0x84, 0, 0},	/* reserved */
	{OV3640_AUTO_2, 0x04, 0, 0},
	{OV3640_AUTO_3, 0x02, 0, 0},
/*	{OV3640_EXVTS_L, 0x00, 0, 0},
	{OV3640_EXVTS_H, 0x00, 0, 0}, */
	{OV3640_TMC13, 0x04, 0, 0},
	{OV3640_TMC10, 0x03, 0, 0},
	{OV3640_TMC10, 0x00, 0, 0},
	{OV3640_CLK, 0x01, 0, 0},
};

static struct reg_value ov3640_setting_15fps_QVGA_320_240[] = {
	{OV3640_SYS, 0x80, 0, 0},
	{0x304d, 0x45, 0, 0},
	{0x30a7, 0x5e, 0, 0},
	{OV3640_TMC11, 0x16, 0, 0},
	{0x309c, 0x1a, 0, 0},	/* reserved */
	{0x30a2, 0xe4, 0, 0},	/* reserved */
	{0x30aa, 0x42, 0, 0},	/* reserved */
	{OV3640_IO_CTRL0, 0xff, 0, 0},
	{OV3640_IO_CTRL1, 0xff, 0, 0},
	{OV3640_IO_CTRL2, 0x10, 0, 0},
	{OV3640_PLL_1, 0x32, 0, 0},
	{OV3640_PLL_2, 0x21, 0, 0},
	{OV3640_PLL_3, 0x20, 0, 0},
	{OV3640_CLK, 0x00, 0, 0},
	{0x304c, 0x81, 0, 0},	/* reserved */
	{0x30d7, 0x10, 0, 0},	/* reserved */
	{0x30d9, 0x0d, 0, 0},	/* reserved */
	{0x30db, 0x08, 0, 0},	/* reserved */
	{0x3016, 0x82, 0, 0},	/* reserved */
	{OV3640_WPT_HISH, 0x38, 0, 0},
	{OV3640_BPT_HISL, 0x30, 0, 0},
	{OV3640_VPT, 0x61, 0, 0},
	{OV3640_TMC7, 0x00, 0, 0},
	{OV3640_TMC11, 0x02, 0, 0},
	{0x3082, 0x20, 0, 0},	/* reserved */
	{OV3640_AUTO_3, 0x12, 0, 0},
	{OV3640_AUTO_2, 0x04, 0, 0},
	{OV3640_AUTO_1, 0xf7, 0, 0},
	{OV3640_AHW_H, 0x08, 0, 0},
	{OV3640_AHW_L, 0x18, 0, 0},
	{OV3640_AVH_H, 0x06, 0, 0},
	{OV3640_AVH_L, 0x0c, 0, 0},
	{OV3640_WEIGHT0, 0x62, 0, 0},
	{OV3640_WEIGHT1, 0x26, 0, 0},
	{OV3640_WEIGHT2, 0xe6, 0, 0},
	{OV3640_WEIGHT3, 0x6e, 0, 0},
	{OV3640_WEIGHT4, 0xea, 0, 0},
	{OV3640_WEIGHT5, 0xae, 0, 0},
	{OV3640_WEIGHT6, 0xa6, 0, 0},
	{OV3640_WEIGHT7, 0x6a, 0, 0},
	{OV3640_SC_SYN_CTRL0, 0x02, 0, 0},
	{OV3640_SC_SYN_CTRL1, 0xfd, 0, 0},
	{OV3640_SC_SYN_CTRL2, 0x00, 0, 0},
	{OV3640_SC_SYN_CTRL3, 0xff, 0, 0},
	{OV3640_DSP_CTRL_0, 0x12, 0, 0},
	{OV3640_DSP_CTRL_1, 0xde, 0, 0},
	{OV3640_DSP_CTRL_2, 0xcf, 0, 0},
	{0x3312, 0x26, 0, 0},
	{0x3314, 0x42, 0, 0},
	{0x3313, 0x2b, 0, 0},
	{0x3315, 0x42, 0, 0},
	{0x3310, 0xd0, 0, 0},
	{0x3311, 0xbd, 0, 0},
	{0x330c, 0x18, 0, 0},
	{0x330d, 0x18, 0, 0},
	{0x330e, 0x56, 0, 0},
	{0x330f, 0x5c, 0, 0},
	{0x330b, 0x1c, 0, 0},
	{0x3306, 0x5c, 0, 0},
	{0x3307, 0x11, 0, 0},
	{OV3640_R_A1, 0x52, 0, 0},
	{OV3640_G_A1, 0x46, 0, 0},
	{OV3640_B_A1, 0x38, 0, 0},
	{OV3640_DSPC0, 0x20, 0, 0},
	{OV3640_DSPC1, 0x17, 0, 0},
	{OV3640_DSPC2, 0x04, 0, 0},
	{OV3640_DSPC3, 0x08, 0, 0},
	{0x3507, 0x06, 0, 0},
	{0x350a, 0x4f, 0, 0},
	{OV3640_SC_CTRL0, 0x02, 0, 0},
	{OV3640_DSP_CTRL_1, 0xde, 0, 0},
	{OV3640_DSP_CTRL_4, 0x00, 0, 0},
	{OV3640_FMT_MUX_CTRL0, 0x00, 0, 0},
	{OV3640_FMT_CTRL00, 0x42, 0, 0},
	{OV3640_OUT_CTRL00, 0xc4, 0, 0},
	{OV3640_DSP_CTRL_2, 0xef, 0, 0},
	{OV3640_HS_H, 0x01, 0, 0},
	{OV3640_HS_L, 0x1d, 0, 0},
	{OV3640_VS_H, 0x00, 0, 0},
	{OV3640_VS_L, 0x0a, 0, 0},
	{OV3640_HW_H, 0x08, 0, 0},
	{OV3640_HW_L, 0x00, 0, 0},
	{OV3640_VH_H, 0x06, 0, 0},
	{OV3640_VH_L, 0x00, 0, 0},
	{OV3640_SIZE_IN_MISC, 0x68, 0, 0},
	{OV3640_HSIZE_IN_L, 0x00, 0, 0},
	{OV3640_VSIZE_IN_L, 0x00, 0, 0},
	{OV3640_SIZE_OUT_MISC, 0x01, 0, 0},
	{OV3640_HSIZE_OUT_L, 0x40, 0, 0},
	{OV3640_VSIZE_OUT_L, 0xf0, 0, 0},
	{OV3640_ISP_PAD_CTR2, 0x00, 0, 0},
	{OV3640_ISP_XOUT_H, 0x01, 0, 0},
	{OV3640_ISP_XOUT_L, 0x40, 0, 0},
	{OV3640_ISP_YOUT_H, 0x00, 0, 0},
	{OV3640_ISP_YOUT_L, 0xf0, 0, 0},
	{OV3640_TMC6, 0x10, 0, 0},
	{0x3090, 0xc0, 0, 0},
	{0x304c, 0x84, 0, 0},	/* reserved */
	{OV3640_TMC13, 0x04, 0, 0},
	{OV3640_TMC10, 0x03, 0, 0},
	{OV3640_TMC10, 0x00, 0, 0},
	{OV3640_CLK, 0x01, 0, 0},
};

static struct reg_value ov3640_setting_30fps_QVGA_320_240[] = {
	{OV3640_SYS, 0x80, 0, 0},
	{0x304d, 0x45, 0, 0},
	{0x30a7, 0x5e, 0, 0},
	{OV3640_TMC11, 0x16, 0, 0},
	{0x309c, 0x1a, 0, 0},	/* reserved */
	{0x30a2, 0xe4, 0, 0},	/* reserved */
	{0x30aa, 0x42, 0, 0},	/* reserved */
	{OV3640_IO_CTRL0, 0xff, 0, 0},
	{OV3640_IO_CTRL1, 0xff, 0, 0},
	{OV3640_IO_CTRL2, 0x10, 0, 0},
	{OV3640_PLL_1, 0x32, 0, 0},
	{OV3640_PLL_2, 0x21, 0, 0},
	{OV3640_PLL_3, 0x20, 0, 0},
	{OV3640_CLK, 0x01, 0, 0},
	{0x304c, 0x82, 0, 0},	/* reserved */
	{0x30d7, 0x10, 0, 0},	/* reserved */
	{0x30d9, 0x0d, 0, 0},	/* reserved */
	{0x30db, 0x08, 0, 0},	/* reserved */
	{0x3016, 0x82, 0, 0},	/* reserved */
	{OV3640_WPT_HISH, 0x38, 0, 0},
	{OV3640_BPT_HISL, 0x30, 0, 0},
	{OV3640_VPT, 0x61, 0, 0},
	{OV3640_TMC7, 0x00, 0, 0},
	{OV3640_TMC11, 0x02, 0, 0},
	{0x3082, 0x20, 0, 0},	/* reserved */
	{OV3640_AUTO_3, 0x12, 0, 0},
	{OV3640_AUTO_2, 0x0c, 0, 0},
	{OV3640_AUTO_1, 0xf7, 0, 0},
	{OV3640_AHW_H, 0x08, 0, 0},
	{OV3640_AHW_L, 0x18, 0, 0},
	{OV3640_AVH_H, 0x06, 0, 0},
	{OV3640_AVH_L, 0x0c, 0, 0},
	{OV3640_WEIGHT0, 0x62, 0, 0},
	{OV3640_WEIGHT1, 0x26, 0, 0},
	{OV3640_WEIGHT2, 0xe6, 0, 0},
	{OV3640_WEIGHT3, 0x6e, 0, 0},
	{OV3640_WEIGHT4, 0xea, 0, 0},
	{OV3640_WEIGHT5, 0xae, 0, 0},
	{OV3640_WEIGHT6, 0xa6, 0, 0},
	{OV3640_WEIGHT7, 0x6a, 0, 0},
	{OV3640_SC_SYN_CTRL0, 0x02, 0, 0},
	{OV3640_SC_SYN_CTRL1, 0xfd, 0, 0},
	{OV3640_SC_SYN_CTRL2, 0x00, 0, 0},
	{OV3640_SC_SYN_CTRL3, 0xff, 0, 0},
	{OV3640_DSP_CTRL_0, 0x12, 0, 0},
	{OV3640_DSP_CTRL_1, 0xde, 0, 0},
	{OV3640_DSP_CTRL_2, 0xcf, 0, 0},
	{0x3312, 0x26, 0, 0},
	{0x3314, 0x42, 0, 0},
	{0x3313, 0x2b, 0, 0},
	{0x3315, 0x42, 0, 0},
	{0x3310, 0xd0, 0, 0},
	{0x3311, 0xbd, 0, 0},
	{0x330c, 0x18, 0, 0},
	{0x330d, 0x18, 0, 0},
	{0x330e, 0x56, 0, 0},
	{0x330f, 0x5c, 0, 0},
	{0x330b, 0x1c, 0, 0},
	{0x3306, 0x5c, 0, 0},
	{0x3307, 0x11, 0, 0},
	{OV3640_R_A1, 0x52, 0, 0},
	{OV3640_G_A1, 0x46, 0, 0},
	{OV3640_B_A1, 0x38, 0, 0},
	{OV3640_DSP_CTRL_0, 0x13, 0, 0},
	{OV3640_DSPC0, 0x20, 0, 0},
	{OV3640_DSPC1, 0x17, 0, 0},
	{OV3640_DSPC2, 0x04, 0, 0},
	{OV3640_DSPC3, 0x08, 0, 0},
	{OV3640_SC_CTRL0, 0x02, 0, 0},
	{OV3640_DSP_CTRL_1, 0x10, 0x30, 0},
	{OV3640_DSP_CTRL_4, 0x00, 0x03, 0},
	{OV3640_FMT_MUX_CTRL0, 0x00, 0, 0},
	{OV3640_FMT_CTRL00, 0x02, 0, 0},
	{OV3640_OUT_CTRL00, 0xc0, 0, 0},
	{OV3640_TMC13, 0x04, 0, 0},
	{OV3640_TMC10, 0x03, 0, 0},
	{OV3640_TMC10, 0x00, 0, 0},
	{OV3640_SYS, 0x10, 0, 0},
	{OV3640_VS_L, 0x06, 0, 0},
	{OV3640_VH_H, 0x03, 0, 0},
	{OV3640_VH_L, 0x04, 0, 0},
	{OV3640_VTS_H, 0x03, 0, 0},
	{OV3640_VTS_L, 0x10, 0, 0},
	{OV3640_VSYNCOPT, 0x24, 0, 0},
	{OV3640_PCLK, 0x01, 0, 0},
	{0x30d7, 0x80, 0x80, 0},	/* reserved */
	{OV3640_BLC9, 0x00, 0x40, 0},
	{OV3640_AVH_H, 0x00, 0, 0},
	{OV3640_AVH_L, 0xc0, 0, 0},
	{OV3640_DSP_CTRL_2, 0x20, 0x20, 0},
	{OV3640_SIZE_IN_MISC, 0x34, 0, 0},
	{OV3640_HSIZE_IN_L, 0x0c, 0, 0},
	{OV3640_VSIZE_IN_L, 0x04, 0, 0},
	{OV3640_SIZE_OUT_MISC, 0x34, 0, 0},
	{OV3640_HSIZE_OUT_L, 0x08, 0, 0},
	{OV3640_VSIZE_OUT_L, 0x04, 0, 0},
	{OV3640_ISP_PAD_CTR2, 0x42, 0, 0},
	{OV3640_ISP_XOUT_H, 0x04, 0, 0},
	{OV3640_ISP_XOUT_L, 0x00, 0, 0},
	{OV3640_ISP_YOUT_H, 0x03, 0, 0},
	{OV3640_ISP_YOUT_L, 0x00, 0, 0},
	{OV3640_SIZE_OUT_MISC, 0x12, 0, 0},
	{OV3640_HSIZE_OUT_L, 0x88, 0, 0},
	{OV3640_VSIZE_OUT_L, 0xe4, 0, 0},
	{OV3640_ISP_PAD_CTR2, 0x42, 0, 0},
	{OV3640_ISP_XOUT_H, 0x02, 0, 0},
	{OV3640_ISP_XOUT_L, 0x80, 0, 0},
	{OV3640_ISP_YOUT_H, 0x01, 0, 0},
	{OV3640_ISP_YOUT_L, 0xe0, 0, 0},
	{OV3640_PLL_1, 0x37, 0, 0},
	{OV3640_PLL_2, 0xe1, 0, 0},
	{OV3640_PLL_3, 0x22, 0, 0},
	{OV3640_CLK, 0x01, 0, 0},
	{0x304c, 0x84, 0, 0},	/* reserved */
};

static struct ov3640_mode_info ov3640_mode_info_data[2][ov3640_mode_MAX + 1] = {
	{
		{ov3640_mode_VGA_640_480,    640,  480,
		ov3640_setting_15fps_VGA_640_480,
		ARRAY_SIZE(ov3640_setting_15fps_VGA_640_480)},
		{ov3640_mode_QVGA_320_240,   320,  240,
		ov3640_setting_15fps_QVGA_320_240,
		ARRAY_SIZE(ov3640_setting_15fps_QVGA_320_240)},
		{ov3640_mode_XGA_1024_768,   1024, 768,
		ov3640_setting_15fps_XGA_1024_768,
		ARRAY_SIZE(ov3640_setting_15fps_XGA_1024_768)},
		{ov3640_mode_QXGA_2048_1536, 2048, 1536,
		ov3640_setting_15fps_QXGA_2048_1536,
		ARRAY_SIZE(ov3640_setting_15fps_QXGA_2048_1536)},
	},
	{
		{ov3640_mode_VGA_640_480,    640,  480,
		ov3640_setting_30fps_VGA_640_480,
		ARRAY_SIZE(ov3640_setting_30fps_VGA_640_480)},
		{ov3640_mode_QVGA_320_240,   320,  240,
		ov3640_setting_30fps_QVGA_320_240,
		ARRAY_SIZE(ov3640_setting_30fps_QVGA_320_240)},
		{ov3640_mode_XGA_1024_768,   1024, 768,
		ov3640_setting_30fps_XGA_1024_768,
		ARRAY_SIZE(ov3640_setting_30fps_XGA_1024_768)},
		{ov3640_mode_QXGA_2048_1536, 0, 0, NULL, 0},
	},
};
/*
static struct regulator *io_regulator;
static struct regulator *core_regulator;
static struct regulator *analog_regulator;
static struct regulator *gpo_regulator;

static int ov3640_probe(struct i2c_client *adapter,
				const struct i2c_device_id *device_id);
static int ov3640_remove(struct i2c_client *client);

extern void gpio_sensor_active(unsigned int csi_index);
extern void gpio_sensor_inactive(unsigned int csi);
*/

/*
	XXX Controls to check....

	Miss that one:
	case V4L2_CID_SATURATION:
	case V4L2_CID_HUE:
	case V4L2_CID_DO_WHITE_BALANCE:
	case V4L2_CID_GAMMA:
*/

static struct vcontrol {
	struct v4l2_queryctrl qc;
	int current_value;
} video_control[] = {
#ifndef OV3640_RAW_MODE
	{
		{
			.id = V4L2_CID_BRIGHTNESS,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Brightness",
			.minimum = OV3640_MIN_BRIGHT,
			.maximum = OV3640_MAX_BRIGHT,
			.step = OV3640_BRIGHT_STEP,
			.default_value = OV3640_DEF_BRIGHT,
		},
		.current_value = OV3640_DEF_BRIGHT,
	}, {
		{
			.id = V4L2_CID_CONTRAST,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Contrast",
			.minimum = OV3640_MIN_CONTRAST,
			.maximum = OV3640_MAX_CONTRAST,
			.step = OV3640_CONTRAST_STEP,
			.default_value = OV3640_DEF_CONTRAST,
		},
		.current_value = OV3640_DEF_CONTRAST,
	}, {
		{
			.id = V4L2_CID_PRIVATE_BASE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Color Effects",
			.minimum = OV3640_MIN_COLOR,
			.maximum = OV3640_MAX_COLOR,
			.step = OV3640_COLOR_STEP,
			.default_value = OV3640_DEF_COLOR,
		},
		.current_value = OV3640_DEF_COLOR,
	},{
		{
			.id = V4L2_CID_GAIN,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Gain",
			.minimum = 0,
			.maximum = 63,
			.step = 1,
			.default_value = OV3640_DEF_GAIN,
		},
		.current_value = OV3640_DEF_GAIN,
	}, {
		{
			.id = V4L2_CID_AUTOGAIN,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "Auto Gain",
			.minimum = 0,
			.maximum = 1,
			.step = 0,
			.default_value = OV3640_DEF_AUTOGAIN,
		},
		.current_value = OV3640_DEF_AUTOGAIN,
	}, {
		{
			.id = V4L2_CID_EXPOSURE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Exposure",
			.minimum = 0,
			.maximum = 255,
			.step = 1,
			.default_value = OV3640_DEF_EXPOSURE,
		},
		.current_value = OV3640_DEF_EXPOSURE,
	}, /* {
		.id = V4L2_CID_AUTOEXPOSURE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "Auto Exposure",
		.minimum = 0,
		.maximum = 1,
		.step = 0,
		.default_value = OV3640_DEF_AEC,
	}, {
		.id = V4L2_CID_FREEZE_AGCAEC,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "Freeze AGC/AEC",
		.minimum = 0,
		.maximum = 1,
		.step = 0,
		.default_value = OV3640_DEF_FREEZE_AGCAEC,
	},*/ {
		{
			.id = V4L2_CID_RED_BALANCE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Red Balance",
			.minimum = 0,
			.maximum = 255,
			.step = 1,
			.default_value = OV3640_DEF_RED,
		},
		.current_value = OV3640_DEF_RED,
	}, {
		{
			.id = V4L2_CID_BLUE_BALANCE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Blue Balance",
			.minimum = 0,
			.maximum = 255,
			.step = 1,
			.default_value = OV3640_DEF_BLUE,
		},
		.current_value = OV3640_DEF_BLUE,
	}, {
		{
			.id = V4L2_CID_AUTO_WHITE_BALANCE,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "Auto White Balance",
			.minimum = 0,
			.maximum = 1,
			.step = 0,
			.default_value = OV3640_DEF_AWB,
		},
		.current_value = OV3640_DEF_AWB,
	}, {
		{
			.id = V4L2_CID_HFLIP,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "Mirror Image",
			.minimum = 0,
			.maximum = 1,
			.step = 0,
			.default_value = OV3640_DEF_HFLIP,
		},
		.current_value = OV3640_DEF_HFLIP,
	}, {
		{
			.id = V4L2_CID_VFLIP,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "Vertical Flip",
			.minimum = 0,
			.maximum = 1,
			.step = 0,
			.default_value = OV3640_DEF_VFLIP,
		},
		.current_value = OV3640_DEF_VFLIP,
	},
#endif
};

/* Returns the index of the requested ID from the control structure array */
static int find_vctrl(int id)
{
	int i;

	if (id < V4L2_CID_BASE)
		return -EDOM;

	for (i = (ARRAY_SIZE(video_control) - 1); i >= 0; i--)
		if (video_control[i].qc.id == id)
			break;
	if (i < 0)
		i = -EINVAL;
	return i;
}

static struct v4l2_queryctrl ov3640_controls[ARRAY_SIZE(video_control)];

/* extract V4L2 only controls from video_control[] (required by SoC) */
static void ov3640_allocate_video_controls(void)
{
	int i;

	for (i = 0; i < (ARRAY_SIZE(video_control) - 1); i++) {
		memcpy(&ov3640_controls[i], &video_control[i], sizeof(struct v4l2_queryctrl));
	}
}

static int ov3640_write(struct i2c_client *client, u16 reg, u8 val)
{
	u8 au8Buf[3] = {0};

	if (!client->adapter)
		return -ENODEV;

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val;

	if (i2c_master_send(client, au8Buf, 3) < 0) {
		dev_err(&client->dev, "error while writing reg 0x%04x with val=0x%x\n",
			reg, val);
		return -ENODEV;
	}

	return 0;
}

static int ov3640_read(struct i2c_client *client, u16 reg, u8 *val)
{
	int ret = 0;
	u8 au8RegBuf[2] = {0};
	u8 u8RdVal = 0;

	if (!client->adapter)
		return -ENODEV;

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;

	if (2 != i2c_master_send(client, au8RegBuf, 2)) {
		pr_err("%s:write reg error:reg=%x\n",
				__func__, reg);
		return -ENODEV;
	}

	if (1 != i2c_master_recv(client, &u8RdVal, 1)) {
		dev_err(&client->dev, "%s:read reg error:reg=%x,val=%x\n",
				__func__, reg, u8RdVal);
		return -ENODEV;
	}

	*val = u8RdVal;

	return ret;
}

/*static int ov3640_init_mode(struct i2c_client *client,
				enum ov3640_frame_rate frame_rate,
				enum ov3640_mode mode)*/

static int ov3640_configure(struct soc_camera_device *icd)
{
	struct ov3640_sensor *ov3640 = container_of(icd, struct ov3640_sensor, icd);
	struct v4l2_pix_format *pix = &ov3640->pix;
//	struct v4l2_fract *fper = &ov3640->timeperframe;
	struct i2c_client *client = ov3640->i2c_client;

	struct reg_value *pModeSetting = NULL;
	s32 i = 0;
	s32 iModeSettingArySize = 0;
	register u32 Delay_ms = 0;
	register u16 RegAddr = 0;
	register u8 Mask = 0;
	register u8 Val = 0;
	u8 RegVal = 0;
	int retval = 0;

	/* XXX */
	enum ov3640_mode mode = ov3640_mode_VGA_640_480;
	enum ov3640_frame_rate frame_rate = ov3640_15_fps;/*ov3640_30_fps;*/

	if (mode > ov3640_mode_MAX || mode < ov3640_mode_MIN) {
		pr_err("Wrong ov3640 mode detected!\n");
		return -1;
	}

	pModeSetting = ov3640_mode_info_data[frame_rate][mode].init_data_ptr;
	iModeSettingArySize =
		ov3640_mode_info_data[frame_rate][mode].init_data_size;

	pix->width = ov3640_mode_info_data[frame_rate][mode].width;
	pix->height = ov3640_mode_info_data[frame_rate][mode].height;

	for (i = 0; i < iModeSettingArySize; ++i, ++pModeSetting) {
		Delay_ms = pModeSetting->u32Delay_ms;
		RegAddr = pModeSetting->u16RegAddr;
		Val = pModeSetting->u8Val;
		Mask = pModeSetting->u8Mask;

		if (Mask) {
			retval = ov3640_read(client, RegAddr, &RegVal);
			if (retval < 0)
				goto err;

			RegVal &= ~(u8)Mask;
			Val &= Mask;
			Val |= RegVal;
		}

		retval = ov3640_write(client, RegAddr, Val);
		if (retval < 0)
			goto err;

		if (Delay_ms)
			msleep(Delay_ms);
	}
err:

	return retval;
}

/* --------------- IOCTL functions from v4l2_int_ioctl_desc --------------- */
#if 0
static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	CAMERA_TRACE(("In ov3640:ioctl_g_ifparm\n"));
	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	memset(p, 0, sizeof(*p));
	p->u.bt656.clock_curr = ov3640_data.mclk;
	pr_debug("   clock_curr=mclk=%d\n", ov3640_data.mclk);
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.clock_min = OV3640_XCLK_MIN;
	p->u.bt656.clock_max = OV3640_XCLK_MAX;
	p->u.bt656.bt_sync_correct = 1;  /* Indicate external vsync */

	return 0;
}

static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	CAMERA_TRACE(("In ov3640:ioctl_g_parm\n"));
	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		CAMERA_TRACE(("   type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n"));
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		CAMERA_TRACE(("   type is not " \
			"V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			a->type));
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}
#endif

#if 0
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor *sensor = s->priv;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	u32 tgt_fps;	/* target frames per secound */
	enum ov3640_frame_rate frame_rate;
	int ret = 0;

	CAMERA_TRACE(("In ov3640:ioctl_s_parm\n"));
	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		CAMERA_TRACE(("   type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n"));

		/* Check that the new frame rate is allowed. */
		if ((timeperframe->numerator == 0) ||
		    (timeperframe->denominator == 0)) {
			timeperframe->denominator = DEFAULT_FPS;
			timeperframe->numerator = 1;
		}

		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps > MAX_FPS) {
			timeperframe->denominator = MAX_FPS;
			timeperframe->numerator = 1;
		} else if (tgt_fps < MIN_FPS) {
			timeperframe->denominator = MIN_FPS;
			timeperframe->numerator = 1;
		}

		/* Actual frame rate we use */
		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps == 15)
			frame_rate = ov3640_15_fps;
		else if (tgt_fps == 30)
			frame_rate = ov3640_30_fps;
		else {
			pr_err(" The camera frame rate is not supported!\n");
			return -EINVAL;
		}

		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode =
				(u32)a->parm.capture.capturemode;

		ret = ov3640_init_mode(frame_rate,
				       sensor->streamcap.capturemode);
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_debug("   type is not " \
			"V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			a->type);
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*
	case V4L2_CID_BRIGHTNESS:
		vc->value = ov3640_data.brightness;
	case V4L2_CID_HUE:
		vc->value = ov3640_data.hue;
	case V4L2_CID_CONTRAST:
		vc->value = ov3640_data.contrast;
	case V4L2_CID_SATURATION:
		vc->value = ov3640_data.saturation;
	case V4L2_CID_RED_BALANCE:
		vc->value = ov3640_data.red;
	case V4L2_CID_BLUE_BALANCE:
		vc->value = ov3640_data.blue;
	case V4L2_CID_EXPOSURE:
		vc->value = ov3640_data.ae_mode;
*/
/*
	case V4L2_CID_BRIGHTNESS:
	case V4L2_CID_CONTRAST:
	case V4L2_CID_SATURATION:
	case V4L2_CID_HUE:
	case V4L2_CID_AUTO_WHITE_BALANCE:
	case V4L2_CID_DO_WHITE_BALANCE:
	case V4L2_CID_RED_BALANCE:
	case V4L2_CID_BLUE_BALANCE:
	case V4L2_CID_GAMMA:
	case V4L2_CID_EXPOSURE:
	case V4L2_CID_AUTOGAIN:
	case V4L2_CID_GAIN:
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
*/
#endif

#if 0
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct sensor *sensor = s->priv;
	u32 tgt_xclk;	/* target xclk */
	u32 tgt_fps;	/* target frames per secound */
	enum ov3640_frame_rate frame_rate;

	CAMERA_TRACE(("In ov3640:ioctl_dev_init\n"));

	gpio_sensor_active(ov3640_data.csi);
	ov3640_data.on = true;

	/* mclk */
	tgt_xclk = ov3640_data.mclk;
	tgt_xclk = min(tgt_xclk, (u32)OV3640_XCLK_MAX);
	tgt_xclk = max(tgt_xclk, (u32)OV3640_XCLK_MIN);
	ov3640_data.mclk = tgt_xclk;

	pr_debug("   Setting mclk to %d MHz\n", tgt_xclk / 1000000);
	set_mclk_rate(&ov3640_data.mclk, ov3640_data.csi);

	/* Default camera frame rate is set in probe */
	tgt_fps = sensor->streamcap.timeperframe.denominator /
		  sensor->streamcap.timeperframe.numerator;

	if (tgt_fps == 15)
		frame_rate = ov3640_15_fps;
	else if (tgt_fps == 30)
		frame_rate = ov3640_30_fps;
	else
		return -EINVAL; /* Only support 15fps or 30fps now. */

	return ov3640_init_mode(frame_rate,
				sensor->streamcap.capturemode);
}
#endif

static int ov3640_detect(struct i2c_client *client)
{
	u8 pidh, pidl, sccb_id;
	int ret;

	if (!client)
		return -ENODEV;

	if ((ret = ov3640_read(client, OV3640_PIDH, &pidh)))
		return ret;
	if ((ret = ov3640_read(client, OV3640_PIDL, &pidl)))
		return ret;
	if ((ret = ov3640_read(client, OV3640_SCCB_ID, &sccb_id)))
		return ret;

	dev_dbg(&client->dev, "PIDH=0x%02x PIDL=0x%02x SCCB_ID=0x%02x\n",
			pidh, pidl, sccb_id);

	if ((pidh != OV3640_PIDH_MAGIC) || ((pidl != OV3640_PIDL_MAGIC1A) &&
						(pidl != OV3640_PIDL_MAGIC2A) &&
						(pidl != OV3640_PIDL_MAGIC2C)))
		/*
		 * We didn't read the values we expected, so
		 * this must not be an OV3640.
		 */
		return -ENODEV;

	return pidl;
}


static int ov3640_get_control(struct soc_camera_device *icd,
				struct v4l2_control *vctrl)
{
/*	struct ov3640_sensor *sensor = container_of(icd, struct ov3640_sensor, icd);
	struct i2c_client *client = sensor->i2c_client; */
	int i/*, val*/;
	struct vcontrol *lvc;

	i = find_vctrl(vctrl->id);
	if (i < 0)
		return -EINVAL;

	lvc = (struct vcontrol *)&video_control[i];
/*	XXX if (ov3640_read_reg_mask(client, lvc->reg, (u8 *)&val, lvc->mask))
		return -EIO;

	val = val >> lvc->start_bit;
	if (val >= 0) {
		vctrl->value = lvc->current_value = val;
		return 0;
	} else
		return val; */

	vctrl->value = lvc->current_value;
	return 0;
}

static int ov3640_set_control(struct soc_camera_device *icd,
				struct v4l2_control *ctrl)
{
/*	struct ov3640_sensor *sensor = container_of(icd, struct ov3640_sensor, icd);
	struct i2c_client *client = sensor->i2c_client; */
	struct vcontrol *lvc;
	int val = ctrl->value;
	int i;

	i = find_vctrl(ctrl->id);
	if (i < 0)
		return -EINVAL;

	lvc = (struct vcontrol *)&video_control[i];
/* XXX	val = val << lvc->start_bit;
	if (ov3640_write_reg_mask(client, lvc->reg, (u8 *)&val, (u8)lvc->mask))
		return -EIO;

	val = val >> lvc->start_bit;
	if (val >= 0) {
		lvc->current_value = val;
		return 0;
	} else
		return val; */

	lvc->current_value = val;
	return 0;
}

static int ov3640_get_chip_id(struct soc_camera_device *icd,
			       struct v4l2_dbg_chip_ident *id)
{
	struct ov3640_sensor *ov3640 = container_of(icd, struct ov3640_sensor, icd);

	if (id->match.type != V4L2_CHIP_MATCH_I2C_ADDR)
		return -EINVAL;

	if (id->match.addr != ov3640->i2c_client->addr)
		return -ENODEV;

	id->ident	= ov3640->model;
	id->revision	= 0;

	return 0;
}


static int ov3640_try_fmt(struct soc_camera_device *icd, struct v4l2_format *fmt)
{
	printk("- %s\n", __func__);
	return 0;
}

static int ov3640_release(struct soc_camera_device *icd)
{
	printk("- %s\n", __func__);
	return 0;
}

static int ov3640_start_capture(struct soc_camera_device *icd)
{
	printk("- %s\n", __func__);
	return 0;
}

static int ov3640_stop_capture(struct soc_camera_device *icd)
{
	printk("- %s\n", __func__);
	return 0;
}

static int ov3640_set_bus_param(struct soc_camera_device *icd,
				 unsigned long flags)
{
	printk("- %s\n", __func__);
	return 0;
}

static unsigned long ov3640_query_bus_param(struct soc_camera_device *icd)
{
	unsigned int width_flag = SOCAM_DATAWIDTH_10 | SOCAM_DATAWIDTH_8;

	/* OV3640 has all capture_format parameters fixed */
	return SOCAM_PCLK_SAMPLE_RISING |
		SOCAM_HSYNC_ACTIVE_HIGH |
		SOCAM_VSYNC_ACTIVE_HIGH |
		SOCAM_MASTER |
		width_flag;
}

static int ov3640_set_fmt(struct soc_camera_device *icd,
				__u32 pixfmt,
				struct v4l2_rect *rect)
{
	struct ov3640_sensor *ov3640 = container_of(icd, struct ov3640_sensor, icd);
	int ret;
	int i;

	printk("%s %d 0x%08x\n", __func__, __LINE__, pixfmt);

	for (i = 0; i < NUM_CAPTURE_FORMATS; i++) {
		if (pixfmt == ov3640_formats[i].fourcc) {
			ov3640->pix.pixelformat = ov3640_formats[i].fourcc;
			icd->current_fmt = &ov3640_formats[i];
			break;
		}
	}
	if (i == NUM_CAPTURE_FORMATS)
		return -EINVAL;

	printk("%s %d 0x%08x\n", __func__, __LINE__, pixfmt);

	ov3640->pix.width = rect->width;
	ov3640->pix.height = rect->height;

	ret = ov3640_configure(icd);

	return ret;
}

static void ov3640_reset(struct i2c_client *client)
{
	/* XXX: tweak register OV3640_SC_SYN_CTRL0 */
	msleep(1);
}

static int ov3640_init(struct soc_camera_device *icd)
{
	struct ov3640_sensor *ov3640 = container_of(icd, struct ov3640_sensor, icd);
	struct soc_camera_link *icl = ov3640->i2c_client->dev.platform_data;
	int ret;

	if (icl->power) {
		ret = icl->power(&ov3640->i2c_client->dev, 1);
		if (ret < 0) {
			dev_err(&ov3640->i2c_client->dev,
				"Platform failed to power-on the camera.\n");
			return ret;
		}
	}

	return 0;
}


#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov3640_get_register(struct soc_camera_device *icd,
				struct v4l2_dbg_register *reg)
{
	struct ov3640_sensor *ov3640 = container_of(icd, struct ov3640_sensor, icd);
	int ret;
	u8 val;

	if (reg->match.type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xffff)
		return -EINVAL;

	if (reg->match.addr != ov3640->i2c_client->addr)
		return -ENODEV;

	ret = ov3640_read(ov3640->i2c_client, (u16)reg->reg, &val);
	if (ret)
		return ret;
	reg->val = val;

	return 0;
}

static int ov3640_set_register(struct soc_camera_device *icd,
				struct v4l2_dbg_register *reg)
{
	struct ov3640_sensor *ov3640 = container_of(icd, struct ov3640_sensor, icd);
	int ret;

	if (reg->match.type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xffff)
		return -EINVAL;

	if (reg->match.addr != ov3640->i2c_client->addr)
		return -ENODEV;

	ret = ov3640_write(ov3640->i2c_client, (u16)reg->reg, (u8)reg->val);
	if (ret)
		return ret;

	return 0;
}
#endif /* CONFIG_VIDEO_ADV_DEBUG */

static int ov3640_video_probe(struct soc_camera_device *icd)
{
	struct ov3640_sensor *ov3640 = container_of(icd, struct ov3640_sensor, icd);
	struct soc_camera_link *icl = ov3640->i2c_client->dev.platform_data;
	struct i2c_client *client = ov3640->i2c_client;
	int version, ret;

	if (icl->reset) {
		ret = icl->reset(&ov3640->i2c_client->dev);
	} else {
		dev_info(&client->dev, "No hardware reset available,"
					" using soft one\n");
		ov3640_reset(client); /* soft reset */
	}

	version = ov3640_detect(client);
	if (version < 0) {
		dev_err(&client->dev, "Unable to detect " DRIVER_NAME " sensor\n");
		return version;
	}

	dev_info(&icd->dev, "Detected an ov3640 sensor (rev %x)\n", version);
	ov3640->model = V4L2_IDENT_OV3640;
	icd->formats = ov3640_formats;
	icd->num_formats = NUM_CAPTURE_FORMATS;

	/* configure default image size and pixel format */
	ret = ov3640_configure(icd);
	if (ret)
		goto err;

	/* Now that we know the model, we can start video */
	ret = soc_camera_video_start(icd);
	if (ret)
		goto err;

	return 0;

err:
	return ret;
}

static void ov3640_video_remove(struct soc_camera_device *icd)
{
	struct ov3640_sensor *ov3640 = container_of(icd, struct ov3640_sensor, icd);

	dev_dbg(&icd->dev, "Video %x removed: %p, %p\n", ov3640->i2c_client->addr,
		ov3640->icd.dev.parent, ov3640->icd.vdev);
	soc_camera_video_stop(icd);
}

static int ov3640_suspend(struct soc_camera_device *icd, pm_message_t state)
{
	struct ov3640_sensor *ov3640 = container_of(icd, struct ov3640_sensor, icd);
	struct soc_camera_link *icl = ov3640->i2c_client->dev.platform_data;
	int ret;

	printk("--- %s %d\n", __func__, state.event);

	if (icl->power) {
		ret = icl->power(&ov3640->i2c_client->dev, 0);
		if (ret < 0) {
			dev_err(&icd->dev,
				"Platform failed to powerdown the camera.\n");
			return ret;
		}
	}

	return 0;
}

static int ov3640_resume(struct soc_camera_device *icd)
{
	struct ov3640_sensor *ov3640 = container_of(icd, struct ov3640_sensor, icd);
	struct soc_camera_link *icl = ov3640->i2c_client->dev.platform_data;
	int ret;

	printk("--- %s\n", __func__);

	if (icl->power) {
		ret = icl->power(&ov3640->i2c_client->dev, 1);
		if (ret < 0) {
			dev_err(&icd->dev,
				"Platform failed to resume the camera.\n");
			return ret;
		}
	}

	return 0;
}

static struct soc_camera_ops ov3640_ops = {
	.owner			= THIS_MODULE,
	.probe			= ov3640_video_probe,
	.remove			= ov3640_video_remove,
	.suspend		= ov3640_suspend,
	.resume			= ov3640_resume,
	.init			= ov3640_init,
	.release		= ov3640_release,
	.start_capture		= ov3640_start_capture,
	.stop_capture		= ov3640_stop_capture,
	.set_fmt		= ov3640_set_fmt,
	.try_fmt		= ov3640_try_fmt,
	.set_bus_param		= ov3640_set_bus_param,
	.query_bus_param	= ov3640_query_bus_param,
	.controls		= ov3640_controls,
	.num_controls		= ARRAY_SIZE(ov3640_controls),
	.get_control		= ov3640_get_control,
	.set_control		= ov3640_set_control,
	.get_chip_id		= ov3640_get_chip_id,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.get_register		= ov3640_get_register,
	.set_register		= ov3640_set_register,
#endif
};

static int __init ov3640_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct ov3640_sensor *ov3640;
	struct soc_camera_device *icd;
	struct soc_camera_link *icl = client->dev.platform_data;
	int ret;

	if (!icl) {
		dev_err(&client->dev, DRIVER_NAME " driver needs platform data\n");
		return -EINVAL;
	}

	if (i2c_get_clientdata(client))
		return -EBUSY;

	ov3640 = kzalloc(sizeof(struct ov3640_sensor), GFP_KERNEL);
	if (!ov3640)
		return -ENOMEM;

	ov3640->i2c_client = client;
	i2c_set_clientdata(client, ov3640);

	/* Second stage probe - when a capture adapter is there */
	icd = &(ov3640->icd);
	ov3640_allocate_video_controls();
	icd->ops	= &ov3640_ops;
	icd->control	= &client->dev;
	icd->x_min	= 20; /* to check */
	icd->y_min	= 12; /* to check */
	icd->x_current	= 20; /* to check */
	icd->y_current	= 12; /* to check */
	icd->width_min	= 88; /* to check */
	icd->width_max	= 2048;
	icd->height_min	= 72; /* to check */
	icd->height_max	= 1536;
	icd->y_skip_top	= 1; /* to check */
/*	icd->iface	= icl->bus_id; XXX */

	ret = soc_camera_device_register(icd);
	if (ret)
		goto err;

	return 0;

err:
	i2c_set_clientdata(client, NULL);
	kfree(ov3640);
	return ret;
}

static int ov3640_remove(struct i2c_client *client)
{
 	struct ov3640_sensor *ov3640 = i2c_get_clientdata(client);

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	soc_camera_device_unregister(&ov3640->icd);
//	ov3640_free_video_controls();
	i2c_set_clientdata(client, NULL);
	kfree(ov3640);

	return 0;
}

static const struct i2c_device_id ov3640_id[] = {
	{DRIVER_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ov3640_id);

static struct i2c_driver ov3640_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = DRIVER_NAME,
		  },
	.probe  = ov3640_probe,
	.remove = __exit_p(ov3640_remove),
	.id_table = ov3640_id,
};

static __init int ov3640_mod_init(void)
{
	printk(KERN_NOTICE "OmniVision ov3640 sensor driver, at your service\n");
	return i2c_add_driver(&ov3640_i2c_driver);
}

static void __exit ov3640_mod_exit(void)
{
	i2c_del_driver(&ov3640_i2c_driver);
}

module_init(ov3640_mod_init);
module_exit(ov3640_mod_exit);

MODULE_DESCRIPTION("SoC camera driver for OV3640");
MODULE_AUTHOR("Julien Boibessot <julien.boibessot@armadeus.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
