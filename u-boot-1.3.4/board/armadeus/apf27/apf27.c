/*
 * Copyright (C) 2007 Sascha Hauer, Pengutronix
 * Copyright (C) 2008,2009 Eric Jarrige <jorasse@users.sourceforge.net>
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
#include "crc.h"
#include "fpga.h"
#include <nand.h>
#include <asm/arch/imx-regs.h>

DECLARE_GLOBAL_DATA_PTR;

static int read_firmware (size_t offset, size_t end, u_char * buf)
{
	size_t amount_loaded = 0;
	size_t blocksize;

	u_char *char_ptr;

	blocksize = nand_info[0].erasesize;

	while (offset < end) {
		if (nand_block_isbad(&nand_info[0], offset)) {
			offset += blocksize;
		} else {
			char_ptr = &buf[amount_loaded];
			if (nand_read(&nand_info[0], offset, &blocksize, char_ptr))
				return 1;
			offset += blocksize;
			amount_loaded += blocksize;
		}
	}

	return amount_loaded;
}


static int apf27_devices_init(void)
{
	int i;
	unsigned int mode[] = {
		PD0_AIN_FEC_TXD0,
		PD1_AIN_FEC_TXD1,
		PD2_AIN_FEC_TXD2,
		PD3_AIN_FEC_TXD3,
		PD4_AOUT_FEC_RX_ER,
		PD5_AOUT_FEC_RXD1,
		PD6_AOUT_FEC_RXD2,
		PD7_AOUT_FEC_RXD3,
		PD8_AF_FEC_MDIO,
		PD9_AIN_FEC_MDC | GPIO_PUEN,
		PD10_AOUT_FEC_CRS,
		PD11_AOUT_FEC_TX_CLK,
		PD12_AOUT_FEC_RXD0,
		PD13_AOUT_FEC_RX_DV,
		PD14_AOUT_FEC_CLR,
		PD15_AOUT_FEC_COL,
		PD16_AIN_FEC_TX_ER,
		PF23_AIN_FEC_TX_EN,
		PE12_PF_UART1_TXD,
		PE13_PF_UART1_RXD,
	};

	for (i = 0; i < ARRAY_SIZE(mode); i++)
		imx_gpio_mode(mode[i]);

	return 0;
}

int
board_init (void)
{

	gd->bd->bi_arch_number = CONFIG_MACH_TYPE;
	gd->bd->bi_boot_params = CONFIG_BOOT_PARAMS_ADDR;

	/* On apf27 U-Boot is always in the relocatated stated */
	gd->flags |= GD_FLG_RELOC;

	apf27_devices_init();

	return 0;
}

int
dram_init (void)
{

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
	u_char * firmware_buffer = (u_char *)(CFG_LOAD_ADDR + CFG_MONITOR_LEN);
	size_t end	= 0;
	size_t size	= 0;
	size_t offset	= -1;
 	char *autoload = getenv ("firmware_autoload");

#if defined(CONFIG_FPGA)
	/* init and download fpga */
	if (( autoload ) && (0 == strcmp(autoload, "1"))) {
		if ((s = getenv("firmware_offset")) != NULL) {
			offset = simple_strtoul(s, NULL, 16);
		}
		if ((s = getenv("firmware_len")) != NULL) {
			end = offset + simple_strtoul(s, NULL, 16);
		}
		if ((-1 != offset) && (offset != end)) {
		    size = read_firmware (offset, end, firmware_buffer);
		}
	}
	APF27_init_fpga (firmware_buffer, size);
#endif

	/* detect compatibility issue of environment version */
	s = getenv ("env_version");
	if (( NULL == s ) || (0 != strcmp(s, CONFIG_ENV_VERSION))) {
		printf("*** Warning - Environment version change suggests: "
			"run flash_reset_env; reset\n");
	}

	/* Unlock whole flash but U-Boot */
	s = getenv ("env_offset");
	offset = CFG_ENV_OFFSET;
	if ((s != NULL) && (0 != strcmp(s, "0"))) {
		offset = simple_strtoul(s, NULL, 16);
	}

	if (nand_unlock(&nand_info[0], offset, nand_info[0].size - offset)) {
		printf("NAND flash lock/unlocked failed\n");
	}


	return 0;
}

void
show_boot_progress (int status)
{
#ifdef CONFIG_SHOW_BOOT_PROGRESS
#endif
	return;
}

int checkboard(void)
{
	printf("Armadeus APF2\n");
	return 0;
}

void raise() {}
void abort() {}

