/*
 * Copyright (C) 2004 Sascha Hauer, Synertronixx GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 */

#include <common.h>
#include <asm/arch/imx-regs.h>
#include "apf9328fpga.h"

extern void imx_gpio_mode (int gpio_mode);
#if (CFG_FLASH_CFI) && (CFG_FLASH_CFI_DRIVER)
extern flash_info_t flash_info[];
#endif
int
board_init (void)
{
	DECLARE_GLOBAL_DATA_PTR;

	gd->bd->bi_arch_number = CONFIG_MACH_TYPE;
	gd->bd->bi_boot_params = CONFIG_BOOT_PARAMS_ADDR;

	/* On apf9328 U-Boot is always in the relocatated stated */
	gd->flags |= GD_FLG_RELOC;

	return 0;
}

int
dram_init (void)
{
	DECLARE_GLOBAL_DATA_PTR;

#if ( CONFIG_NR_DRAM_BANKS > 0 )
	gd->bd->bi_dram[0].start = CFG_SDRAM_1_BASE;
	gd->bd->bi_dram[0].size = CFG_SDRAM_1_SIZE;
#endif
#if ( CONFIG_NR_DRAM_BANKS > 1 )
	gd->bd->bi_dram[1].start = CFG_SDRAM_2_BASE;
	gd->bd->bi_dram[1].size = CFG_SDRAM_2_SIZE;
#endif

	return 0;
}

/*
 * Miscellaneous intialization
 */
int
misc_init_r (void)
{
	char *s;
	
#if (CONFIG_FPGA)
	apf9328_init_fpga ();
#endif

#if (CONFIG_DRIVER_DM9000)
	imx_gpio_mode (GPIO_PORTB | GPIO_DR | GPIO_IN | 14);
#endif

	/* adjust size of rootfs if needed */ 
#if (CFG_FLASH_CFI) && (CFG_FLASH_CFI_DRIVER)
	s = getenv ("rootfs_len");
	if (( NULL == s ) || (0 == strcmp(s, ""))) {
		char rootfslen[16];
		sprintf (rootfslen, "0x%lX", flash_info[0].size -
				(CONFIG_ROOTFS_ADDR - CFG_FLASH_BASE));
		setenv ("rootfs_len", rootfslen);
	}
#endif

	/* detect compatibility issue of environment version */ 
	s = getenv ("env_version");
	if (( NULL == s ) || (0 != strcmp(s, CONFIG_ENV_VERSION))) {
		printf("*** Warning - Environment version change suggests: "
			"run flash_reset_env; reset\n");
	}

	return 0;
}

void
show_boot_progress (int status)
{
	return;
}

void raise() {}
void abort() {}
