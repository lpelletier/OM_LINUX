/*
 * imx_watchdog.h
 *
 * Copyright (C) 2008 Juergen Beisert (kernel@pengutronix.de)
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

/*
 * Watchdog register definition for:
 * - i.MX21
 * - i.MX27
 */

/* for direct access, note: All registers are 16 bit wide */
#define IMX_WCR_REG(x)	(*(volatile unsigned short __force*)(x))
#define IMX_WSR_REG(x)	(*(volatile unsigned short __force*)((x) + 2))
#define IMX_WRSR_REG(x)	(*(volatile unsigned short __force*)((x) + 4))

/* for drivers usage */
#define IMX_WCR 0
#define IMX_WSR 2
#define IMX_WRSR 4

/* Bit definitions */
#define IMX_WCR_WT_SHIFT	8
#define IMX_WCR_WT_MASK		0xFF00
#define IMX_WCR_WDA		(1<<5)
#define IMX_WCR_SRS		(1<<4)
#define IMX_WCR_WRE		(1<<3)
#define IMX_WCR_WDE		(1<<2)
#define IMX_WCR_WDBG		(1<<1)
#define IMX_WCR_WDZST		(1<<0)
