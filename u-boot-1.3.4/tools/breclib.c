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


#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#include "breclib.h"

#define BS_WRITE	0x00<<5
#define BS_READ		0x01<<5
#define BS_BYTE		0x00<<6
#define BS_HALF_WORD	0x01<<6
#define BS_WORD		0x03<<6

#define BS_WRITE_BYTES( X ) 	(BS_WRITE | BS_BYTE | X)
#define BS_WRITE_HALF_WORD( X ) (BS_WRITE | BS_HALF_WORD | X)
#define BS_WRITE_WORD( X ) 	(BS_WRITE | BS_WORD | X)
#define BS_READ_BYTES( X ) 	(BS_READ | BS_BYTE | X)
#define BS_READ_HALF_WORD( X ) 	(BS_READ | BS_HALF_WORD | X)
#define BS_READ_WORD( X ) 	(BS_READ | BS_WORD | X)
#define BS_EXEC( ) 		0x00



int build_brec_long_read_req(char *a_brecord, long a_addr) {
	sprintf(a_brecord, "%08lX%02X", a_addr, BS_READ_WORD( 4 ));
	return 0;
}

int build_brec_long_write_req(char *a_brecord, long a_addr, long a_value) {
	sprintf(a_brecord, "%08lX%02X%08lX", a_addr, BS_WRITE_WORD( 4 ), a_value);
	return 0;
}

int build_brec_mem_write_req(char *a_brecord, long a_addr, void *a_mem, int a_size) {
	int  i = a_size;
	char buff[63];
	char *dst_ptr = buff;
	unsigned char *src_ptr = a_mem;

	

	if (i > 31) {
		fprintf(stderr, "cannot write more than 31 bytes at once in a bootstrap record\n");
		exit (EXIT_FAILURE);
	}

	for(;i>0;i--, dst_ptr+=2, src_ptr++)
		sprintf(dst_ptr,"%02X", *src_ptr);
	sprintf(a_brecord, "%08lX%02X%s", a_addr, BS_WRITE_BYTES(a_size), buff);
	return 0;
}

int build_brec_exe_req(char *a_brecord, long a_addr) {
	sprintf(a_brecord, "%08lX%02X", a_addr, BS_EXEC());
	return 0;
}
