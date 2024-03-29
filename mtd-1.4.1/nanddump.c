/*
 *  nanddump.c
 *
 *  Copyright (C) 2000 David Woodhouse (dwmw2@infradead.org)
 *                     Steven J. Hill (sjhill@realitydiluted.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Overview:
 *   This utility dumps the contents of raw NAND chips or NAND
 *   chips contained in DoC devices.
 */

#define _GNU_SOURCE
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <asm/types.h>
#include <mtd/mtd-user.h>

#define PROGRAM "nanddump"
#define VERSION "$Revision: 1.29 $"

static struct nand_oobinfo none_oobinfo = {
	.useecc = MTD_NANDECC_OFF,
};

static void display_help (void)
{
	printf(
"Usage: nanddump [OPTIONS] MTD-device\n"
"Dumps the contents of a nand mtd partition.\n"
"\n"
"           --help               Display this help and exit\n"
"           --version            Output version information and exit\n"
"-a         --forcebinary        Force printing of binary data to tty\n"
"-c         --canonicalprint     Print canonical Hex+ASCII dump\n"
"-f file    --file=file          Dump to file\n"
"-l length  --length=length      Length\n"
"-n         --noecc              Read without error correction\n"
"-N         --noskipbad          Read without bad block skipping\n"
"-o         --omitoob            Omit oob data\n"
"-b         --omitbad            Omit bad blocks from the dump\n"
"-p         --prettyprint        Print nice (hexdump)\n"
"-q         --quiet              Don't display progress and status messages\n"
"-s addr    --startaddress=addr  Start address\n"
	);
	exit(EXIT_SUCCESS);
}

static void display_version (void)
{
	printf(PROGRAM " " VERSION "\n"
			"\n"
			PROGRAM " comes with NO WARRANTY\n"
			"to the extent permitted by law.\n"
			"\n"
			"You may redistribute copies of " PROGRAM "\n"
			"under the terms of the GNU General Public Licence.\n"
			"See the file `COPYING' for more information.\n");
	exit(EXIT_SUCCESS);
}

// Option variables

static bool		pretty_print = false;	// print nice
static bool		noecc = false;		// don't error correct
static bool		noskipbad = false;	// don't skip bad blocks
static bool		omitoob = false;	// omit oob data
static unsigned long	start_addr;		// start address
static unsigned long	length;			// dump length
static const char	*mtddev;		// mtd device name
static const char	*dumpfile;		// dump file name
static bool		omitbad = false;
static bool		quiet = false;		// suppress diagnostic output
static bool		canonical = false;	// print nice + ascii
static bool		forcebinary = false;	// force printing binary to tty

static void process_options (int argc, char * const argv[])
{
	int error = 0;

	for (;;) {
		int option_index = 0;
		static const char *short_options = "bs:f:l:opqnNca";
		static const struct option long_options[] = {
			{"help", no_argument, 0, 0},
			{"version", no_argument, 0, 0},
			{"forcebinary", no_argument, 0, 'a'},
			{"canonicalprint", no_argument, 0, 'c'},
			{"file", required_argument, 0, 'f'},
			{"prettyprint", no_argument, 0, 'p'},
			{"omitoob", no_argument, 0, 'o'},
			{"omitbad", no_argument, 0, 'b'},
			{"startaddress", required_argument, 0, 's'},
			{"length", required_argument, 0, 'l'},
			{"noecc", no_argument, 0, 'n'},
			{"noskipbad", no_argument, 0, 'N'},
			{"quiet", no_argument, 0, 'q'},
			{0, 0, 0, 0},
		};

		int c = getopt_long(argc, argv, short_options,
				long_options, &option_index);
		if (c == EOF) {
			break;
		}

		switch (c) {
			case 0:
				switch (option_index) {
					case 0:
						display_help();
						break;
					case 1:
						display_version();
						break;
				}
				break;
			case 'b':
				omitbad = true;
				break;
			case 's':
				start_addr = strtol(optarg, NULL, 0);
				break;
			case 'f':
				if (!(dumpfile = strdup(optarg))) {
					perror("stddup");
					exit(EXIT_FAILURE);
				}
				break;
			case 'l':
				length = strtol(optarg, NULL, 0);
				break;
			case 'o':
				omitoob = true;
				break;
			case 'a':
				forcebinary = true;
				break;
			case 'c':
				canonical = true;
			case 'p':
				pretty_print = true;
				break;
			case 'q':
				quiet = true;
				break;
			case 'n':
				noecc = true;
				break;
			case 'N':
				noskipbad = true;
				break;
			case '?':
				error++;
				break;
		}
	}

	if (quiet && pretty_print) {
		fprintf(stderr, "The quiet and pretty print options are mutually-\n"
				"exclusive. Choose one or the other.\n");
		exit(EXIT_FAILURE);
	}

	if (forcebinary && pretty_print) {
		fprintf(stderr, "The forcebinary and pretty print options are\n"
				"mutually-exclusive. Choose one or the "
				"other.\n");
		exit(EXIT_FAILURE);
	}

	if ((argc - optind) != 1 || error)
		display_help ();

	mtddev = argv[optind];
}

#define PRETTY_ROW_SIZE 16
#define PRETTY_BUF_LEN 80

/**
 * pretty_dump_to_buffer - formats a blob of data to "hex ASCII" in memory
 * @buf: data blob to dump
 * @len: number of bytes in the @buf
 * @linebuf: where to put the converted data
 * @linebuflen: total size of @linebuf, including space for terminating NULL
 * @pagedump: true - dumping as page format; false - dumping as OOB format
 * @ascii: dump ascii formatted data next to hexdump
 * @prefix: address to print before line in a page dump, ignored if !pagedump
 *
 * pretty_dump_to_buffer() works on one "line" of output at a time, i.e.,
 * PRETTY_ROW_SIZE bytes of input data converted to hex + ASCII output.
 *
 * Given a buffer of unsigned char data, pretty_dump_to_buffer() converts the
 * input data to a hex/ASCII dump at the supplied memory location. A prefix
 * is included based on whether we are dumping page or OOB data. The converted
 * output is always NULL-terminated.
 *
 * e.g.
 *   pretty_dump_to_buffer(data, data_len, prettybuf, linelen, true,
 *                         false, 256);
 * produces:
 *   0x00000100: 40 41 42 43 44 45 46 47 48 49 4a 4b 4c 4d 4e 4f
 * NOTE: This function was adapted from linux kernel, "lib/hexdump.c"
 */
static void pretty_dump_to_buffer(const unsigned char *buf, size_t len,
		char *linebuf, size_t linebuflen, bool pagedump, bool ascii,
		unsigned int prefix)
{
	static const char hex_asc[] = "0123456789abcdef";
	unsigned char ch;
	int j, lx = 0;
	int ascii_column;

	if (pagedump)
		sprintf(linebuf, "0x%.8x: ", prefix);
	else
		sprintf(linebuf, "  OOB Data: ");
	lx += 12;

	if (!len)
		goto nil;
	if (len > PRETTY_ROW_SIZE)	/* limit to one line at a time */
		len = PRETTY_ROW_SIZE;

	for (j = 0; (j < len) && (lx + 3) <= linebuflen; j++) {
		ch = buf[j];
		linebuf[lx++] = hex_asc[(ch & 0xf0) >> 4];
		linebuf[lx++] = hex_asc[ch & 0x0f];
		linebuf[lx++] = ' ';
	}
	if (j)
		lx--;

	ascii_column = 3 * PRETTY_ROW_SIZE + 14;

	if (!ascii)
		goto nil;

	while (lx < (linebuflen - 1) && lx < (ascii_column - 1))
		linebuf[lx++] = ' ';
	linebuf[lx++] = '|';
	for (j = 0; (j < len) && (lx + 2) < linebuflen; j++)
		linebuf[lx++] = (isascii(buf[j]) && isprint(buf[j])) ? buf[j]
			: '.';
	linebuf[lx++] = '|';
nil:
	linebuf[lx++] = '\n';
	linebuf[lx++] = '\0';
}

/*
 * Buffers for reading data from flash
 */
#define NAND_MAX_PAGESIZE 4096
#define NAND_MAX_OOBSIZE 256
static unsigned char readbuf[NAND_MAX_PAGESIZE];
static unsigned char oobbuf[NAND_MAX_OOBSIZE];

/*
 * Main program
 */
int main(int argc, char * const argv[])
{
	unsigned long ofs, end_addr = 0;
	unsigned long long blockstart = 1;
	int ret, i, fd, ofd, bs, badblock = 0;
	struct mtd_oob_buf oob = {0, 16, oobbuf};
	mtd_info_t meminfo;
	char pretty_buf[PRETTY_BUF_LEN];
	int oobinfochanged = 0 ;
	struct nand_oobinfo old_oobinfo;
	struct mtd_ecc_stats stat1, stat2;
	bool eccstats = false;

	process_options(argc, argv);

	/* Open MTD device */
	if ((fd = open(mtddev, O_RDONLY)) == -1) {
		perror(mtddev);
		exit (EXIT_FAILURE);
	}

	/* Fill in MTD device capability structure */
	if (ioctl(fd, MEMGETINFO, &meminfo) != 0) {
		perror("MEMGETINFO");
		close(fd);
		exit (EXIT_FAILURE);
	}

	/* Make sure device page sizes are valid */
	if (!(meminfo.oobsize == 224 && meminfo.writesize == 4096) &&
			!(meminfo.oobsize == 218 && meminfo.writesize == 4096) &&
			!(meminfo.oobsize == 128 && meminfo.writesize == 4096) &&
			!(meminfo.oobsize == 64 && meminfo.writesize == 4096) &&
			!(meminfo.oobsize == 64 && meminfo.writesize == 2048) &&
			!(meminfo.oobsize == 32 && meminfo.writesize == 1024) &&
			!(meminfo.oobsize == 16 && meminfo.writesize == 512) &&
			!(meminfo.oobsize == 8 && meminfo.writesize == 256)) {
		fprintf(stderr, "Unknown flash (not normal NAND)\n");
		close(fd);
		exit(EXIT_FAILURE);
	}
	/* Read the real oob length */
	oob.length = meminfo.oobsize;

	if (noecc)  {
		ret = ioctl(fd, MTDFILEMODE, (void *) MTD_MODE_RAW);
		if (ret == 0) {
			oobinfochanged = 2;
		} else {
			switch (errno) {
			case ENOTTY:
				if (ioctl (fd, MEMGETOOBSEL, &old_oobinfo) != 0) {
					perror ("MEMGETOOBSEL");
					close (fd);
					exit (EXIT_FAILURE);
				}
				if (ioctl (fd, MEMSETOOBSEL, &none_oobinfo) != 0) {
					perror ("MEMSETOOBSEL");
					close (fd);
					exit (EXIT_FAILURE);
				}
				oobinfochanged = 1;
				break;
			default:
				perror ("MTDFILEMODE");
				close (fd);
				exit (EXIT_FAILURE);
			}
		}
	} else {

		/* check if we can read ecc stats */
		if (!ioctl(fd, ECCGETSTATS, &stat1)) {
			eccstats = true;
			if (!quiet) {
				fprintf(stderr, "ECC failed: %d\n", stat1.failed);
				fprintf(stderr, "ECC corrected: %d\n", stat1.corrected);    
				fprintf(stderr, "Number of bad blocks: %d\n", stat1.badblocks);    
				fprintf(stderr, "Number of bbt blocks: %d\n", stat1.bbtblocks);    
			}
		} else
			perror("No ECC status information available");
	}

	/* Open output file for writing. If file name is "-", write to standard
	 * output. */
	if (!dumpfile) {
		ofd = STDOUT_FILENO;
	} else if ((ofd = open(dumpfile, O_WRONLY | O_TRUNC | O_CREAT, 0644))== -1) {
		perror (dumpfile);
		close(fd);
		exit(EXIT_FAILURE);
	}

	if (!pretty_print && !forcebinary && isatty(ofd)) {
		fprintf(stderr, "Not printing binary garbage to tty. Use '-a'\n"
				"or '--forcebinary' to override.\n");
		close(fd);
		exit(EXIT_FAILURE);
	}

	/* Initialize start/end addresses and block size */
	if (length)
		end_addr = start_addr + length;
	if (!length || end_addr > meminfo.size)
		end_addr = meminfo.size;

	bs = meminfo.writesize;

	/* Print informative message */

	if (!quiet) {
		fprintf(stderr, "Block size %u, page size %u, OOB size %u\n",
				meminfo.erasesize, meminfo.writesize, meminfo.oobsize);
		fprintf(stderr,
				"Dumping data starting at 0x%08x and ending at 0x%08x...\n",
				(unsigned int) start_addr, (unsigned int) end_addr);
	}
	/* Dump the flash contents */
	for (ofs = start_addr; ofs < end_addr ; ofs+=bs) {

		// new eraseblock , check for bad block
		if (noskipbad) {
			badblock = 0;
		} else if (blockstart != (ofs & (~meminfo.erasesize + 1))) {
			blockstart = ofs & (~meminfo.erasesize + 1);
			if ((badblock = ioctl(fd, MEMGETBADBLOCK, &blockstart)) < 0) {
				perror("ioctl(MEMGETBADBLOCK)");
				goto closeall;
			}
		}

		if (badblock) {
			if (omitbad)
				continue;
			memset (readbuf, 0xff, bs);
		} else {
			/* Read page data and exit on failure */
			if (pread(fd, readbuf, bs, ofs) != bs) {
				perror("pread");
				goto closeall;
			}
		}

		/* ECC stats available ? */
		if (eccstats) {
			if (ioctl(fd, ECCGETSTATS, &stat2)) {
				perror("ioctl(ECCGETSTATS)");
				goto closeall;
			}
			if (stat1.failed != stat2.failed)
				fprintf(stderr, "ECC: %d uncorrectable bitflip(s)"
						" at offset 0x%08lx\n",
						stat2.failed - stat1.failed, ofs);
			if (stat1.corrected != stat2.corrected)
				fprintf(stderr, "ECC: %d corrected bitflip(s) at"
						" offset 0x%08lx\n",
						stat2.corrected - stat1.corrected, ofs);
			stat1 = stat2;
		}

		/* Write out page data */
		if (pretty_print) {
			for (i = 0; i < bs; i += PRETTY_ROW_SIZE) {
				pretty_dump_to_buffer(readbuf+i, PRETTY_ROW_SIZE,
						pretty_buf, PRETTY_BUF_LEN, true, canonical, ofs+i);
				write(ofd, pretty_buf, strlen(pretty_buf));
			}
		} else
			write(ofd, readbuf, bs);



		if (omitoob)
			continue;

		if (badblock) {
			memset (readbuf, 0xff, meminfo.oobsize);
		} else {
			/* Read OOB data and exit on failure */
			oob.start = ofs;
			if (ioctl(fd, MEMREADOOB, &oob) != 0) {
				perror("ioctl(MEMREADOOB)");
				goto closeall;
			}
		}

		/* Write out OOB data */
		if (pretty_print) {
			for (i = 0; i < meminfo.oobsize; i += 16) {
				pretty_dump_to_buffer(oobbuf+i, meminfo.oobsize-i,
						pretty_buf, PRETTY_BUF_LEN, false, canonical, 0);
				write(ofd, pretty_buf, strlen(pretty_buf));
			}
		} else
			write(ofd, oobbuf, meminfo.oobsize);
	}

	/* reset oobinfo */
	if (oobinfochanged == 1) {
		if (ioctl (fd, MEMSETOOBSEL, &old_oobinfo) != 0) {
			perror ("MEMSETOOBSEL");
			close(fd);
			close(ofd);
			return EXIT_FAILURE;
		}
	}
	/* Close the output file and MTD device */
	close(fd);
	close(ofd);

	/* Exit happy */
	return EXIT_SUCCESS;

closeall:
	/* The new mode change is per file descriptor ! */
	if (oobinfochanged == 1) {
		if (ioctl (fd, MEMSETOOBSEL, &old_oobinfo) != 0)  {
			perror ("MEMSETOOBSEL");
		}
	}
	close(fd);
	close(ofd);
	exit(EXIT_FAILURE);
}
