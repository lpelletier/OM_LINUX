/*
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

#ifndef __ASM_ARCH_MXC_DAM_H
#define __ASM_ARCH_MXC_DAM_H

#define DAM_HPCR1	0x00
#define DAM_HPCR2	0x04
/*
 * manual says this port is a host port. But its connected like
 * the other peripheral ports to the external SSI_4* pin group
 */
#define DAM_HPCR3	0x08

/* all host ports share these bits */
# define DAM_HPCR_INMMASK_MASK (0xff << 0)
# define DAM_HPCR_INMMASK(x) ((x) & 0xff)
# define DAM_HPCR_INMEN (1 << 8)
# define DAM_HPCR_SYN (1 << 12)
# define DAM_HPCR_RXDSEL_MASK (0x7 << 13)
# define DAM_HPCR_RXDSEL(x) (((x) & 0x7) << 13)
# define DAM_HPCR_RFCSEL_MASK (0xf << 20)
# define DAM_HPCR_RFCSEL(x) (((x) & 0xf) << 20)
# define DAM_HPCR_RCLKDIR (1 << 24)
# define DAM_HPCR_RFSDIR (1 << 25)
# define DAM_HPCR_TFCSEL_MASK (0xf << 26)
# define DAM_HPCR_TFCSEL(x) (((x) & 0xf) << 26)
# define DAM_HPCR_TCLKDIR (1 << 30)
# define DAM_HPCR_TFSDIR (1 << 31)

#define DAM_PPCR1	0x10
#define DAM_PPCR2	0x14
#define DAM_PPCR3	0x1C

/* all peripheral ports share these bits */
# define DAM_PPCR_TXRXEN (1 << 10)
# define DAM_PPCR_SYN (1 << 12)
# define DAM_PPCR_RXDSEL_MASK (0x7 << 13)
# define DAM_PPCR_RXDSEL(x) (((x) & 0x7) << 13)
# define DAM_PPCR_RFCSEL_MASK (0xf << 20)
# define DAM_PPCR_RFCSEL(x) (((x) & 0xf) << 20)
# define DAM_PPCR_RCLKDIR (1 << 24)
# define DAM_PPCR_RFSDIR (1 << 25)
# define DAM_PPCR_TFCSEL_MASK (0xf << 26)
# define DAM_PPCR_TFCSEL(x) (((x) & 0xf) << 26)
# define DAM_PPCR_TCLKDIR (1 << 30)
# define DAM_PPCR_TFSDIR (1 << 31)

#endif /* __ASM_ARCH_MXC_DAM_H */
