/*
 * Copyright (C) 2009-2011 ARMadeus Systems
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */

#ifndef __ASM_ARCH_ARMADEUS_FPGA_H__
#define __ASM_ARCH_ARMADEUS_FPGA_H__

#ifndef CONFIG_MACH_APF9328
# include <mach/irqs.h>
# include <mach/gpio.h>
# include <mach/iomux-mx1-mx2.h>
#endif

#if defined(CONFIG_MACH_APF27) /* */
# define APF27_FPGA_BASE_ADDR		0xd6000000
# define ARMADEUS_FPGA_BASE_ADDR	APF27_FPGA_BASE_ADDR
# define APF27_FPGA_INT_PIN		(GPIO_PORTF | 12)
# define APF27_FPGA_IRQ			(IRQ_GPIOF(12))
# define ARMADEUS_FPGA_IRQ		APF27_FPGA_IRQ
# define IRQ_FPGA_START			MXC_BOARD_IRQ_START
#elif defined(CONFIG_MACH_APF9328) /* */
# define APF9328_FPGA_BASE_ADDR		0x12000000
# define ARMADEUS_FPGA_BASE_ADDR	APF9328_FPGA_BASE_ADDR
# define APF9328_FPGA_IRQ		(IRQ_GPIOA(1))
# define ARMADEUS_FPGA_IRQ		APF9328_FPGA_IRQ
#else
# error "Unsupported platform"
#endif

#define IRQ_FPGA(x)     (IRQ_FPGA_START + x)
#endif /* __ASM_ARCH_ARMADEUS_FPGA_H__ */
