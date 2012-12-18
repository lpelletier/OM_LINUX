/*
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Juergen Beisert, kernel@pengutronix.de
 * Copyright 2009 Armadeus systems, <julien.boibessot@armadeus.com>
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

#ifndef __ASM_ARCH_IMX_SSI_H
#define __ASM_ARCH_IMX_SSI_H

#include <linux/platform_device.h>

struct imx_ssi_platform_data {
	int (*init)(struct platform_device *pdev);
	int (*exit)(struct platform_device *pdev);
};

/* Transmit Data Register */
#define SSI_STX			0x00

/* Receive Data Register */
#define SSI_SRX			0x04

/* Control/Status Register */
#define SSI_SCSR		0x08
# define SSI_SCSR_SYS_CLK_EN	(1<<15)		/* System Clock Enable */
# define SSI_SCSR_I2S_MODE_NORM	(0x00<<13)	/* Normal Mode Select */
# define SSI_SCSR_I2S_MODE_MSTR	(0x01<<13)	/* I2S Master Select */
# define SSI_SCSR_I2S_MODE_SLAVE	(0x02<<13)	/* I2S Slave Select */
# define SSI_SCSR_SYN		(1<<12)		/* Synchronous Mode */
# define SSI_SCSR_NET		(1<<11)		/* Network Mode */
# define SSI_SCSR_RE		(1<<10)		/* Receive Enable */
# define SSI_SCSR_TE		(1<<9)		/* Transmit Enable */
# define SSI_SCSR_EN		(1<<8)		/* SSI Enable*/
# define SSI_SCSR_RDR		(1<<7)		/* Receive Data Ready */
# define SSI_SCSR_TDE		(1<<6)		/* Transmit Data Reg. Empty */
# define SSI_SCSR_ROE		(1<<5)		/* Receive Overrun Error */
# define SSI_SCSR_TUE		(1<<4)		/* Transmitter Underrun Error */
# define SSI_SCSR_TFS		(1<<3)		/* Transmit Frame Sync */
# define SSI_SCSR_RFS		(1<<2)		/* Receive Frame Sync */
# define SSI_SCSR_RFF		(1<<1)		/* Receive FIFO Full */
# define SSI_SCSR_TFE		(1<<0)		/* Transmit FIFO Empty */
# define SSI_SCSR_I2S_MODE_SHIFT	  13

/* Transmit Configuration Register */
#define SSI_STCR	0x0c
/* Receive Configuration Register */
#define SSI_SRCR	0x10
/* STCR & SRCR have the same bit fields */
# define SSI_BIT0    (1<<10)    /* Transmit/Receive Bit0 */
# define SSI_MAE     (1<<9)     /* Transmit/Receive DMA Enable */
# define SSI_IE      (1<<8)     /* Transmit/Receive Interrupt Enable */
# define SSI_FEN     (1<<7)     /* Transmit/Receive FIFO Enable */
# define SSI_FDIR    (1<<6)     /* Transmit/Receive Frame Direction */
# define SSI_DIR     (1<<5)     /* Transmit/Receive Direction */
# define SSI_SHFD    (1<<4)     /* Transmit/Receive Shift Direction */
# define SSI_SCKP    (1<<3)     /* Transmit/Receive Clock Polarity */
# define SSI_FSI     (1<<2)     /* Transmit/Receive Frame Sync Invert */
# define SSI_FSL     (1<<1)     /* Transmit/Receive Frame Sync Length */
# define SSI_EFS     (1<<0)     /* Transmit/Receive Early Frame Sync */

/* Transmit Clock Control Register */
#define SSI_STCCR	0x14
/* Receive Clock Control Register */
#define SSI_SRCCR	0x18
/* STCCR & SRCCR have the same bit fields */
# define SSI_PSR	(1 << 15)		/* Prescaler Range */
# define SSI_WL_8	(0x00 << 13)		/* Word Length = 8 bits */
# define SSI_WL_10	(0x01 << 13)		/* Word Length = 10 bits */
# define SSI_WL_12	(0x02 << 13)		/* Word Length = 12 bits */
# define SSI_WL_16	(0x03 << 13)		/* Word Length = 16 bits */
# define SSI_DC(x)	(((x) & 0x1f) << 8)	/* Frame Rate Divider Control */
# define SSI_PM(x)	(((x) & 0xff) << 0)	/* Prescaler Modulus Select */

/* Time Slot Register */
#define SSI_STSR	0x1c

/* FIFO Control/Status Register */
#define SSI_SFCSR		0x20
# define SSI_SFCSR_RFCNT(x)	(((x) & 0x0f) << 12)	/* Receive FIFO Counter */
# define SSI_SFCSR_TFCNT(x)	(((x) & 0x0f) << 8)	/* Transmit FIFO Counter */
# define SSI_SFCSR_RFWM(x)	(((x) & 0x0f) << 4)	/* RX FIFO Full Watermark */
# define SSI_SFCSR_TFWM(x)	(((x) & 0x0f) << 0)	/* TX FIFO Empty Watermark */

/* Option Register */
#define SSI_SOR		0x28
# define SSI_SOR_CLKOFF	(1<<6)		/* Clock Off */
# define SSI_SOR_RX_CLR	(1<<5)		/* Receiver Clear */
# define SSI_SOR_TX_CLR	(1<<4)		/* Transmitter Clear */
# define SSI_SOR_SYNRST	(1<<0)		/* Frame Sync Reset */

#endif /* __ASM_ARCH_IMX_SSI_H */
