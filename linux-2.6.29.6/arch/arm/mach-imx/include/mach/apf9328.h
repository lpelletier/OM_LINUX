/*
 * linux/include/asm-arm/arch-imx/apf9328.h
 *
 * Copyright (c) 2005 Armadeus
 * This work is based on scb9328.h from Sascha Hauer
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef __ASM_ARCH_APF9328_H
#define __ASM_ARCH_APF9328_H

/* ------------------------------------------------------------------------ */
/* Memory Map for the APF9328 Board                                         */
/* ------------------------------------------------------------------------ */

/* Ethernet */
#define APF9328_ETH_VIRT ( IMX_CS4_VIRT + 0x00C00000 )
#define APF9328_ETH_PHYS ( IMX_CS4_PHYS + 0x00C00000 )
#define APF9328_ETH_IRQ  ( IRQ_GPIOB(14) )

/* FLASH */
#define APF9328_FLASH_PHYS      IMX_CS0_PHYS
#ifdef CONFIG_IMX_APF9328_FLASH_8MB
# define APF9328_FLASH_SIZE     0x00800000
#endif
#ifdef CONFIG_IMX_APF9328_FLASH_16MB
# define APF9328_FLASH_SIZE     0x01000000
#endif
#define APF9328_FLASH_BASE      IMX_CS0_VIRT

#define CLK32 32768

#ifndef __ASSEMBLY__
/*
 * The APF9328 module needs a baseboard to work. After basic initializing
 * its own devices, it calls baseboard's init function.
 * TODO: Add your own baseboard init function and call it from
 * inside apf9328_init().
 *
 * This example here is for the DevLight/Full development boards. Refer apf9328-dev.c.
 */

extern void apf9328_baseboard_init(void);
#endif

#endif /* __ASM_ARCH_APF9328_H */
