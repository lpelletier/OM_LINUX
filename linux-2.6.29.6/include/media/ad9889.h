/*
 * AD9889 driver
 *
 * Copyright (c) 2009 Armadeus Systems
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef __LINUX_VIDE0_AD9889_H
#define __LINUX_VIDE0_AD9889_H

struct ad9889_config {
    int (*init)(void);
    int (*exit)(void);
	void (*display_connected)(void);
	void (*display_disconnected)(void);
	void* data;
    unsigned char EDID_I2C_addr; /*base address of the EDID ram*/
};


#endif	/* __LINUX_VIDE0_AD9889_H */
