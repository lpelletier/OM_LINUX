/*
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Juergen Beisert, kernel@pengutronix.de
 * Copyright 2008 Luotao Fu, kernel@pengutronix.de
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

#ifndef __MXC_SPI_MX27_H__
#define __MXC_SPI_MX27_H__

#include <mach/hardware.h>
#include <asm/mach-types.h>

/*
 * This structure is used to define the SPI master controller's platform
 * data. It includes the SPI  bus number and the maximum number of
 * slaves/chips it supports.
 */
struct mxc_spi_master {
	unsigned int bus_num;		/* bus number. */
	unsigned int maxchipselect;	/* number of chip selects. */
	unsigned int spi_version;	/* CSPI Hardware Version. */

	int (*init)(struct platform_device *pdev);
	int (*exit)(struct platform_device *pdev);
};

/* Register definitions.
 * XXX: The ifdef CONFIG_ARCH_MX2 segments are merged from the
 * freescale mxc_spi_mx27.h. This is actually highly strange, since according
 * to the processor reference manuals the spi controllers on mx27 and mx31 are
 * identical. We might want to clearify this after chatting with freescale on
 * this issue */

#define MXC_CSPIRXDATA		0x00
#define MXC_CSPITXDATA		0x04
#define MXC_CSPICTRL		0x08
#define MXC_CSPIINT		0x0C

#ifdef CONFIG_ARCH_MX2
#define MXC_CSPIDMA		0x18
#define MXC_CSPISTAT		0x0C
#define MXC_CSPIPERIOD		0x14
#define MXC_CSPITEST		0x10
#define MXC_CSPIRESET		0x1C
#define MXC_CSPICTRL_ENABLE	(1 << 10)
#else
#define MXC_CSPIDMA		0x10
#define MXC_CSPISTAT		0x14
#define MXC_CSPIPERIOD		0x18
#define MXC_CSPITEST		0x1C
#define MXC_CSPIRESET		0x00
#define MXC_CSPICTRL_ENABLE	0x1
#endif

#define MXC_CSPICTRL_DISABLE	0x0

#ifdef CONFIG_ARCH_MX2
#define MXC_CSPICTRL_MASTER	(1 << 11)
#else
#define MXC_CSPICTRL_MASTER	(1 << 1)
#endif

#define MXC_CSPICTRL_SLAVE	0x0

#ifdef CONFIG_ARCH_MX2
#define MXC_CSPICTRL_XCH	(1 << 9)
#define MXC_CSPICTRL_LOWPOL	(1 << 5)
#else
#define MXC_CSPICTRL_XCH	(1 << 2)
#define MXC_CSPICTRL_SMC	(1 << 3)
#define MXC_CSPICTRL_LOWPOL	(1 << 4)
#endif

#define MXC_CSPICTRL_HIGHPOL	0x0

#ifdef CONFIG_ARCH_MX2
#define MXC_CSPICTRL_PHA	(1 << 6)
#else
#define MXC_CSPICTRL_PHA	(1 << 5)
#endif

#define MXC_CSPICTRL_NOPHA	0x0

#ifdef CONFIG_ARCH_MX2
#define MXC_CSPICTRL_SSCTL	(1 << 7)
#define MXC_CSPICTRL_HIGHSSPOL 	(1 << 8)
#else
#define MXC_CSPICTRL_SSCTL	(1 << 6)
#define MXC_CSPICTRL_HIGHSSPOL 	(1 << 7)
#endif

#define MXC_CSPICTRL_LOWSSPOL	0x0
#define MXC_CSPICTRL_CSMASK	0x3

#ifdef CONFIG_ARCH_MX2
#define MXC_CSPICTRL_MAXDATRATE	0x10
#define MXC_CSPICTRL_DATAMASK	0x1F
#define MXC_CSPICTRL_DATASHIFT 	14
/* This adjustment in the shift is valid only for even states only(i.e. divide
   ratio of 2). SDHC_SPIEN is not set by default. If SDHC_SPIEN bit is set in
   MXC_CSPICTRL, then divide ratio is 3, this shift adjustment is invalid. */
#define MXC_CSPICTRL_ADJUST_SHIFT(x) ((x) = ((x) - 1) * 2)
#else
#define MXC_CSPICTRL_MAXDATRATE	0x7
#define MXC_CSPICTRL_DATAMASK	0x7
#define MXC_CSPICTRL_DATASHIFT 	16
#define MXC_CSPICTRL_ADJUST_SHIFT(x) ((x) -= 2)
#endif

#define MXC_CSPICTRL_CSSHIFT_0_7	12
#define MXC_CSPICTRL_BCSHIFT_0_7	20
#define MXC_CSPICTRL_BCMASK_0_7		0xFFF
#define MXC_CSPICTRL_DRCTRLSHIFT_0_7	8

#define MXC_CSPICTRL_CSSHIFT_0_5	12
#define MXC_CSPICTRL_BCSHIFT_0_5	20
#define MXC_CSPICTRL_BCMASK_0_5		0xFFF
#define MXC_CSPICTRL_DRCTRLSHIFT_0_5	8

#define MXC_CSPICTRL_CSSHIFT_0_4	24
#define MXC_CSPICTRL_BCSHIFT_0_4	8
#define MXC_CSPICTRL_BCMASK_0_4		0x1F
#define MXC_CSPICTRL_DRCTRLSHIFT_0_4	20

#define MXC_CSPICTRL_CSSHIFT_0_0	19
#define MXC_CSPICTRL_BCSHIFT_0_0	0
#define MXC_CSPICTRL_BCMASK_0_0		0x1F
#define MXC_CSPICTRL_DRCTRLSHIFT_0_0	12

#define MXC_CSPIINT_IRQSHIFT_0_7	8
#define MXC_CSPIINT_IRQSHIFT_0_5	9
#define MXC_CSPIINT_IRQSHIFT_0_4	9
#define MXC_CSPIINT_IRQSHIFT_0_0	18

#ifdef CONFIG_ARCH_MX2
#define MXC_CSPIINT_TEEN	(1 << 9)
#define MXC_CSPIINT_THEN	(1 << 10)
#define MXC_CSPIINT_TFEN	(1 << 11)
#define MXC_CSPIINT_RREN	(1 << 13)
#define MXC_CSPIINT_RHEN        (1 << 14)
#define MXC_CSPIINT_RFEN        (1 << 15)
#define MXC_CSPIINT_ROEN        (1 << 16)
#else
#define MXC_CSPIINT_TEEN	(1 << 0)
#define MXC_CSPIINT_THEN	(1 << 1)
#define MXC_CSPIINT_TFEN	(1 << 2)
#define MXC_CSPIINT_RREN	(1 << 3)
#define MXC_CSPIINT_RHEN        (1 << 4)
#define MXC_CSPIINT_RFEN        (1 << 5)
#define MXC_CSPIINT_ROEN        (1 << 6)
#endif

#define MXC_CSPIINT_TCEN_0_7	(1 << 7)
#define MXC_CSPIINT_TCEN_0_5	(1 << 8)
#define MXC_CSPIINT_TCEN_0_4	(1 << 8)

#ifdef CONFIG_ARCH_MX2
#define MXC_CSPIINT_TCEN_0_0	(1 << 12)
#else
#define MXC_CSPIINT_TCEN_0_0	(1 << 3)
#endif

#define MXC_CSPIINT_BOEN_0_7	0
#define MXC_CSPIINT_BOEN_0_5	(1 << 7)
#define MXC_CSPIINT_BOEN_0_4	(1 << 7)

#ifdef CONFIG_ARCH_MX2
#define MXC_CSPIINT_BOEN_0_0	(1 << 17)
#else
#define MXC_CSPIINT_BOEN_0_0	(1 << 8)
#endif

#define MXC_CSPISTAT_TE		(1 << 0)
#define MXC_CSPISTAT_TH		(1 << 1)
#define MXC_CSPISTAT_TF		(1 << 2)
#define MXC_CSPISTAT_RR		(1 << 3)
#define MXC_CSPISTAT_RH         (1 << 4)
#define MXC_CSPISTAT_RF         (1 << 5)
#define MXC_CSPISTAT_RO         (1 << 6)
#define MXC_CSPISTAT_TC_0_7	(1 << 7)
#define MXC_CSPISTAT_TC_0_5	(1 << 8)
#define MXC_CSPISTAT_TC_0_4	(1 << 8)
#define MXC_CSPISTAT_TC_0_0	(1 << 3)
#define MXC_CSPISTAT_BO_0_7	0
#define MXC_CSPISTAT_BO_0_5	(1 << 7)
#define MXC_CSPISTAT_BO_0_4	(1 << 7)
#define MXC_CSPISTAT_BO_0_0	(1 << 8)

#define MXC_CSPIPERIOD_32KHZ	(1 << 15)

#define MXC_CSPITEST_LBC	(1 << 14)

/*
 * This structure contains information that differs with
 * SPI master controller hardware version
 */
struct mxc_spi_unique_def {
	unsigned int intr_bit_shift;	/* Width of valid bits in MXC_CSPIINT. */
	unsigned int cs_shift;	/* Chip Select shift. */
	unsigned int bc_shift;	/* Bit count shift. */
	unsigned int bc_mask;	/* Bit count mask. */
	unsigned int drctrl_shift;	/* Data Control shift. */
	unsigned int xfer_complete;	/* Transfer Complete shift. */
	unsigned int bc_overflow;	/* Bit counnter overflow shift. */
};

#endif /*__MXC_SPI_MX27_H__ */
