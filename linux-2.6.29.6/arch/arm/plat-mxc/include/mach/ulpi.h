#ifndef __MACH_ULPI_H
#define __MACH_ULPI_H

int ulpi_set(u8 bits, int reg, void __iomem *view);
int ulpi_clear(u8 bits, int reg, void __iomem *view);
int ulpi_read(int reg, void __iomem *view);

/* ISP 1504 register addresses */
#define ISP1504_VID_LOW		0x00	/* Vendor ID low */
#define ISP1504_VID_HIGH	0x01	/* Vendor ID high */
#define ISP1504_PID_LOW		0x02	/* Product ID low */
#define ISP1504_PID_HIGH	0x03	/* Product ID high */
#define ISP1504_FCNCTL		0x04	/* Function Control */
#define ISP1504_ITFCTL		0x07	/* Interface Control */
#define ISP1504_OTGCTL		0x0A	/* OTG Control */

/* add to above register address to access Set/Clear functions */
#define ISP1504_REG_SET		0x01
#define ISP1504_REG_CLEAR	0x02

/* USB3315 register addresses */
#define USB3315_VID_LOW		0x00	/* Vendor ID low */
#define USB3315_VID_HIGH	0x01	/* Vendor ID high */
#define USB3315_PID_LOW		0x02	/* Product ID low */
#define USB3315_PID_HIGH	0x03	/* Product ID high */
#define USB3315_FCNCTL		0x04	/* Function Control */
#define USB3315_ITFCTL		0x07	/* Interface Control */
#define USB3315_OTGCTL		0x0A	/* OTG Control */

/* Add to above register address to access Set/Clear functions */
#define USB3315_REG_SET		0x01
#define USB3315_REG_CLEAR	0x02

/* 1504 Function Control Register bits */
#define SUSPENDM		(1 << 6)		/* Suspend LOW */
#define RESET			(1 << 5)		/* Reset */
#define OPMODE(x)		(((x) & 0x03) << 3)	/* Operation mode */
#define TERMSELECT		(1 << 2)		/* Terminat° select */
#define XCVRSELECT(x)		(((x) & 0x03))		/* Tranceiver select */

/* 1504 OTG Control Register bits */
#define USE_EXT_VBUS_IND	(1 << 7)	/* Use ext. Vbus indicator */
#define DRV_VBUS_EXT		(1 << 6)	/* Drive Vbus external */
#define DRV_VBUS		(1 << 5)	/* Drive Vbus */
#define CHRG_VBUS		(1 << 4)	/* Charge Vbus */
#define DISCHRG_VBUS		(1 << 3)	/* Discharge Vbus */
#define DM_PULL_DOWN		(1 << 2)	/* enable DM Pull Down */
#define DP_PULL_DOWN		(1 << 1)	/* enable DP Pull Down */
#define ID_PULL_UP		(1 << 0)	/* enable ID Pull Up */

#endif /* __MACH_ULPI_H */

