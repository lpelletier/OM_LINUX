/*
 * (C) Copyright 2005
 * Eric JARRIGE, <jorasse@users.sourceforge.net>
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
 */


#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <memory.h>
#include <unistd.h>

#include "breclib.h"

#ifndef __ASSEMBLY__
#define	__ASSEMBLY__			/* Dirty trick to get only #defines	*/
#endif
#define	__ASM_STUB_PROCESSOR_H__	/* don't include asm/processor.		*/
#include <asm/arch/imx-regs.h>
#include <config.h>
#undef	__ASSEMBLY__

/* if not defined generate default freescale MX1ADS eval board */
#ifndef CFG_GPCR_VAL
#warning "CFG_GPCR_VAL is not defined in your config file!! assume 0x03AB"
#define CFG_GPCR_VAL 0x03AB
#endif

#ifndef CFG_PRECHARGE_CMD
#warning "CFG_PRECHARGE_CMD is not defined in your config file!! assume 0x92120200"
#define CFG_PRECHARGE_CMD 0x92120200
#endif

#ifndef CFG_SDRAM_1_BASE
#warning "CFG_SDRAM_1_BASE is not defined in your config file!! assume 0x08000000"
#define CFG_SDRAM_1_BASE 0x08000000
#endif
#ifndef CFG_SDRAM_PRECHARGE_ALL_VAL
#warning "CFG_SDRAM_PRECHARGE_ALL_VAL is not defined in your config file!! assume 0x00200000"
#define CFG_SDRAM_PRECHARGE_ALL_VAL 0x00200000
#endif
#ifndef CFG_AUTOREFRESH_CMD
#warning "CFG_AUTOREFRESH_CMD is not defined in your config file!! assume 0xA2120200"
#define CFG_AUTOREFRESH_CMD 0xA2120200
#endif
#ifndef CFG_SET_MODE_REG_CMD
#warning "CFG_SET_MODE_REG_CMD is not defined in your config file!! assume 0xB2120200"
#define CFG_SET_MODE_REG_CMD 0xB2120200
#endif
#ifndef CFG_SDRAM_MODE_REGISTER_VAL
#warning "CFG_SDRAM_MODE_REGISTER_VAL is not defined in your config file!! assume 0x00111800"
#define CFG_SDRAM_MODE_REGISTER_VAL 0x00111800
#endif
#ifndef CFG_NORMAL_RW_CMD
#warning "CFG_NORMAL_RW_CMD is not defined in your config file!! assume 0x82124200"
#define CFG_NORMAL_RW_CMD 0x82124200
#endif


char *the_prog_name;


void
usage ()
{
	fprintf (stderr, "Usage: %s u-boot.bin [u-boot.brec]\n"
			 "\n"
			 "    %s converts binary files (u-boot.bin) into so called \n"
			 "    bootstrap records file (u-boot.brec) that are accepted by\n"
			 "    Motorola's MC9328MX1/L (a.k.a. DragaonBall i.MX) in \"Bootstrap Mode\" \n"
			 "    add memory init commands and run command\n"
			 "    at end of file\n"
			 "    NOTE: \n"
			 " \n",
 		the_prog_name, the_prog_name);
}

int generate_init_brec(FILE *a_ifd) {
	char a_buf[80];


	fprintf (a_ifd, "********************************************\n");
	fprintf (a_ifd, "* Initialize I/O Pad Driving Strength      *\n");
	fprintf (a_ifd, "********************************************\n");
	build_brec_long_write_req(a_buf, GPCR, CFG_GPCR_VAL);
	fprintf (a_ifd, "%s	; pre-charge command\n",a_buf);
	fprintf (a_ifd, "********************************************\n");
	fprintf (a_ifd, "* Initialize SDRAM                         *\n");
	fprintf (a_ifd, "********************************************\n");
	build_brec_long_write_req(a_buf, SDCTL0, CFG_PRECHARGE_CMD);
	fprintf (a_ifd, "%s	; pre-charge command\n",a_buf);
	build_brec_long_read_req(a_buf, CFG_SDRAM_1_BASE+CFG_SDRAM_PRECHARGE_ALL_VAL);
	fprintf (a_ifd, "%s		; special read\n",a_buf);
	build_brec_long_write_req(a_buf, SDCTL0, CFG_AUTOREFRESH_CMD);
	fprintf (a_ifd, "%s	; auto-refresh command\n",a_buf);
	build_brec_long_read_req(a_buf, CFG_SDRAM_1_BASE);
	fprintf (a_ifd, "%s		; 8 special reads\n",a_buf);
	fprintf (a_ifd, "%s		; Issue AutoRefresh Command\n",a_buf);
	fprintf (a_ifd, "%s		;\n",a_buf);
	fprintf (a_ifd, "%s		;\n",a_buf);
	fprintf (a_ifd, "%s		;\n",a_buf);
	fprintf (a_ifd, "%s		;\n",a_buf);
	fprintf (a_ifd, "%s		;\n",a_buf);
	fprintf (a_ifd, "%s		;\n",a_buf);
	build_brec_long_write_req(a_buf, SDCTL0, CFG_SET_MODE_REG_CMD);
	fprintf (a_ifd, "%s	; set mode register\n",a_buf);
	build_brec_long_read_req(a_buf, CFG_SDRAM_1_BASE+CFG_SDRAM_MODE_REGISTER_VAL);
	fprintf (a_ifd, "%s		; special read\n",a_buf);
	build_brec_long_write_req(a_buf, SDCTL0, CFG_NORMAL_RW_CMD);
	fprintf (a_ifd, "%s	; set normal mode\n",a_buf);

	return 0;
}

int generate_start_uboot_brec(FILE *a_ifd) {
	char a_buf[80];

	fprintf (a_ifd, "********************************************\n");
	fprintf (a_ifd, "* send execute command	                    *\n");
	fprintf (a_ifd, "********************************************\n");
	build_brec_exe_req(a_buf, CFG_SDRAM_1_BASE);
	fprintf (a_ifd, "%s		; execute\n", a_buf);

	return 0;
}

int generate_uboot_code_brec(FILE *a_uboot_fd, FILE *a_ifd) {
	char a_in_buf[80];
	char a_out_buf[80];
	size_t byte_read;
	long dest_addr = CFG_SDRAM_1_BASE;

	fprintf (a_ifd, "********************************************\n");
	fprintf (a_ifd, "* bin code                                 *\n");
	fprintf (a_ifd, "********************************************\n");

	byte_read =fread(a_in_buf, 1, 16, a_uboot_fd);

	while ( byte_read != 0)
	{
		build_brec_mem_write_req(a_out_buf, dest_addr, a_in_buf, byte_read);
		fprintf (a_ifd, "%s\n",a_out_buf);
		dest_addr += byte_read;
		byte_read =fread(a_in_buf, 1, 16, a_uboot_fd);

	} 

	return 0;
}

int main (int argc, char *argv[])
{
	FILE *in_fd, *out_fd;

	the_prog_name = *argv;

	if ((argc < 2)||(argc > 3)||(*argv[1] == '-')) {
		usage();
		return (0);
	}
	else {

		in_fd = fopen(argv[1], "r");

		if (in_fd == NULL) {
			fprintf (stderr, "%s: Can't open input file %s: %s\n",
			the_prog_name, argv[1], strerror(errno));
			exit (EXIT_FAILURE);
		}
		
		out_fd = fopen(argv[2], "w");
		if (out_fd == NULL) {
			fprintf (stderr, "%s: Can't open output file %s: %s\n",
			the_prog_name, argv[2], strerror(errno));
			exit (EXIT_FAILURE);
		}

		generate_init_brec(out_fd);
		generate_uboot_code_brec(in_fd, out_fd);
		generate_start_uboot_brec(out_fd);

		if (fclose(out_fd)) {
			fprintf (stderr, "%s: write error on %s: %s\n",
				the_prog_name, argv[2], strerror(errno));
			exit (EXIT_FAILURE);
		}

		if (fclose(in_fd)) {
			fprintf (stderr, "%s: read error on %s: %s\n",
				the_prog_name, argv[1], strerror(errno));
			exit (EXIT_FAILURE);
		}
	}

	return (0);
}
