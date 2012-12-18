/*
 * (C) Copyright 2005
 * Nicolas Colombin <thom25@users.sourceforge.net>
 * Eric Jarrige <jorasse@users.sourceforge.net>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
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

/*
 * Spartan 3 FPGA configuration support for the APF9328 daughter board
 */

#include <common.h>
#include <spartan3.h>
#include <command.h>
#include <asm/arch/imx-regs.h>
#include <asm/io.h>
#include "apf9328fpga.h"

#if (CONFIG_FPGA)
#if 0
#define FPGA_DEBUG
#endif

#ifdef FPGA_DEBUG
#define	PRINTF(fmt,args...)	printf (fmt ,##args)
#else
#define	PRINTF(fmt,args...)
#endif

/* Note that these are pointers to code that is in Flash.  They will be
 * relocated at runtime.
 * Spartan3 code is used to download our Spartan 3 :) code is compatible. 
 * Just take care about the file size  
*/
Xilinx_Spartan3_Slave_Serial_fns fpga_fns = {
	fpga_pre_fn,
	fpga_pgm_fn,
	fpga_clk_fn,
	fpga_init_fn,
	fpga_done_fn,
	fpga_wr_fn,
};

Xilinx_desc fpga[CONFIG_FPGA_COUNT] = {
	{Xilinx_Spartan3,
	 slave_serial,
	 XILINX_XC3S400_SIZE,
	 (void *) &fpga_fns,
	 0}
};

/*
 * Initialize the fpga.  Return 1 on success, 0 on failure.
 */
int
apf9328_init_fpga (void)
{
	char *autoload = getenv ("firmware_autoload");
	DECLARE_GLOBAL_DATA_PTR;

	int i,lout=1;

	PRINTF ("%s:%d: Initialize FPGA interface (relocation offset = 0x%.8lx)\n",
		__FUNCTION__, __LINE__, gd->reloc_off);
	fpga_init (gd->reloc_off);

	for (i = 0; i < CONFIG_FPGA_COUNT; i++) {
		PRINTF ("%s:%d: Adding fpga %d\n", __FUNCTION__, __LINE__, i);
		fpga_add (fpga_xilinx, &fpga[i]);
	}

	if (( autoload ) && (0 == strcmp(autoload, "1"))) {
		if (FPGA_SUCCESS != fpga_load( 0, (void *)CONFIG_FIRMWARE_ADDR, 
				(size_t) CONFIG_FIRMWARE_LEN )) {
			lout = 0;
			printf("Firmware not loaded!\n");
		}
	}
	return 1;
}

#endif /* CONFIG_FPGA */
