/************************************************************************
 *
 * fpgaregs.c
 * a program to write/read 16/32 bits values on FPGA address map using mmap()
 *
 * (c) Copyright 2008-2011 The Armadeus Project - ARMadeus Systems
 * Fabien Marteau <fabien.marteau@armadeus.com>
 * Modified by Sebastien Van Cauwenberghe <svancau@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 ***********************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>	/* file management */
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>	/* sleep, write(), read() */
#include <string.h>	/* converting string */
#include <sys/mman.h>	/* memory management */

#ifdef IMX27
# define PLATFORM "APF27"
# define FPGA_ADDRESS 0xD6000000
#elif IMX51
# define PLATFORM "APF51"
# define FPGA_ADDRESS 0xB8000000
#else
# define PLATFORM "APF9328"
# define FPGA_ADDRESS 0x12000000
#endif

#define WORD_ACCESS (2)
#define LONG_ACCESS (4)


void displayResult(char* text, unsigned int accesstype, unsigned int value, unsigned int address)
{
	if (accesstype == WORD_ACCESS)
		printf("%s 0x%04x at 0x%08x\n", text, (unsigned short)value, address);
	else
		printf("%s 0x%08x at 0x%08x\n", text, value, address);
}


int main(int argc, char *argv[])
{
	unsigned short address;
	unsigned int value;
	int ffpga, accesstype = LONG_ACCESS;
	void* ptr_fpga;

	if ((argc < 3) || (argc > 4)) {
		printf("invalid arguments number\n");
		printf("fpgaregs for %s platform\n\n", PLATFORM);
		printf("Usage: fpgaregs [w,l] address [value]\n\n");
		printf("Reads or writes a value at given FPGA's relative address.\n\n");
		printf("Options:\n");
		printf("        w        word (16 bits) access\n");
		printf("        l        long (32 bits) access\n");
		printf("        value    value to write\n\n");
		printf("Examples:\n");
		printf("        fpgaregs w 0x10 --> read @ 0x10\n");
		printf("        fpgaregs w 0x10 0x1234 --> writes 0x1234 @ 0x10\n");
		printf("        fpgaregs l 0x10 0x12345678 --> writes 0x12345678 @ 0x10\n");
		return -1;
	}

	ffpga = open("/dev/mem", O_RDWR|O_SYNC);
	if (ffpga < 0) {
		printf("can't open file /dev/mem\n");
		return -1;
	}

	ptr_fpga = mmap(0, 8192, PROT_READ|PROT_WRITE, MAP_SHARED, ffpga, FPGA_ADDRESS);
	if (ptr_fpga == MAP_FAILED) {
		printf("mmap failed\n");
		return -1;
	}

	address = (unsigned int)strtol(argv[2], (char **)NULL, 16);

	if (argc < 3) {
		printf("invalid command line");
	} else {
		if (*(argv[1]) == 'w')
			accesstype = WORD_ACCESS;
		if (accesstype == WORD_ACCESS) {
			if (address%2 != 0) {
				printf("Can't do word access at odd address (%d). Use a 16bits aligned one.\n", address);
				return -1;
			}
		} else { /* LONG_ACCESS */
			if (address%4 != 0) {
				printf("Can't do long access at non-32bits-aligned address (%d)\n", address);
				return -1;
			}
		}

		/* write value at given address */
		if (argc == 4) {
			value = strtoul(argv[3], (char **)NULL, 16);
			if (accesstype == WORD_ACCESS)
				*(unsigned short*)(ptr_fpga+(address)) = (unsigned short)value;
			else
				*(unsigned int*)(ptr_fpga+(address)) = (unsigned int)value;

			displayResult("Write", accesstype, value, address);
		/* read at given address */
		} else if (argc == 3) {
			if (accesstype == WORD_ACCESS)
				value = *(unsigned short*)(ptr_fpga+(address));
			else
				value = *(unsigned int*)(ptr_fpga+(address));

			displayResult("Read", accesstype, value, address);
		}
	}
	close(ffpga);
	return 0;
}

