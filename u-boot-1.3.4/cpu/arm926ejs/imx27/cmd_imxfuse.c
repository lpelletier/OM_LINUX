/*
 * cmd_imxfuse-c Interface to iMX IC Identification Module
 * 	Based on Freescale iMX27 Board Support Package
 *
 * (C) Copyright 2008,2009 Eric Jarrige <eric.jarrige@armadeus.org>
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

#include <config.h>
#include <common.h>
#include <command.h>

#ifdef CONFIG_CMD_IMX_FUSE

#include <asm/arch/imx-regs.h>
#include <asm/io.h>

#define IIM_ERR_SHIFT	   8
#define POLL_FUSE_PRGD	  (IIM_STAT_PRGD | (IIM_ERR_PRGE << IIM_ERR_SHIFT))
#define POLL_FUSE_SNSD	  (IIM_STAT_SNSD | (IIM_ERR_SNSE << IIM_ERR_SHIFT))

static void imx_fuse_op_start(void)
{
	/* Do not generate interrupt */
	writel(0x00, &IIM_STATM);
	// clear the status bits and error bits
	writel(0x03, &IIM_STAT);
	writel(0xFE, &IIM_ERR);
}

/*
 * The action should be either:
 *		  POLL_FUSE_PRGD 
 * or:
 *		  POLL_FUSE_SNSD
 */
static int imx_poll_fuse_op_done(int action)
{

	u32 status, error;

	if (action != POLL_FUSE_PRGD && action != POLL_FUSE_SNSD) {
		printf("%s(%d) invalid operation\n", __FUNCTION__, action);
		return -1;
	}

	/* Poll busy bit till it is NOT set */
	while ((readl(&IIM_STAT) & IIM_STAT_BUSY) != 0 ) {
	}

	/* Test for successful write */
	status = readl(&IIM_STAT);
	error = readl(&IIM_ERR);

	if ((status & action) != 0 && (error & (action >> IIM_ERR_SHIFT)) == 0) {
		if (error) {
			printf("Even though the operation seems successful...\n");
			printf("There are some error(s) at addr=0x%x: 0x%x\n",
						readl(&IIM_ERR), error);
		}
		return 0;
	}
	printf("%s(0x%x) failed\n", __FUNCTION__, action);
	printf("status address=0x%x, value=0x%x\n",
				readl(&IIM_STAT), status);
	printf("There are some error(s) at addr=0x%x: 0x%x\n",
				readl(&IIM_ERR), error);
	return -1;
}

static int imx_read_shadow_fuse(int bank, int row, int bit)
{
	printf("Shadow fuses at (bank:%d, row:%d) = 0x%x\n",
		bank, row, readl(&IIM_BANK_REG(bank,row)));
	return 0;
}

static int imx_sense_fuse(int bank, int row, int bit)
{
	int addr, addr_l, addr_h;

	imx_fuse_op_start();
	
	/* Enable IIM Program Protect */
	writel(0x0, &IIM_PROG_P);

	addr = ((bank << 11) | (row << 3) | (bit & 0x7));
	/* Set IIM Program Upper Address */
	addr_h = (addr >> 8) & 0x000000FF;
	/* Set IIM Program Lower Address */
	addr_l = (addr & 0x000000FF);

#ifdef IIM_FUSE_DEBUG
	printf("%s: addr_h=0x%x, addr_l=0x%x\n",
		__FUNCTION__, addr_h, addr_l);
#endif
	writel(addr_h, &IIM_UA);
	writel(addr_l, &IIM_LA);

	/* Start sensing */
	writel(0x08, &IIM_FCTL);
	if (imx_poll_fuse_op_done(POLL_FUSE_SNSD) != 0) {
		printf("%s(bank: %d, row: %d, bit: %d failed\n",
			__FUNCTION__, bank, row, bit);
	}
	
	printf("fuses at (bank:%d, row:%d) = 0x%x\n",
		bank, row, readl(&IIM_SDAT));
	return 0;
}

/* Blow fuses based on the bank, row and bit positions (all 0-based)
*/
static int imx_fuse_blow(int bank,int row,int bit)
{
	int addr, addr_l, addr_h, ret = 1;

	imx_fuse_op_start();

	/* Disable IIM Program Protect */
	writel(0xAA, &IIM_PROG_P);

	addr = ((bank << 11) | (row << 3) | (bit & 0x7));
	/* Set IIM Program Upper Address */
	addr_h = (addr >> 8) & 0x000000FF;
	/* Set IIM Program Lower Address */
	addr_l = (addr & 0x000000FF);

#ifdef IIM_FUSE_DEBUG
	printf("blowing addr_h=0x%x, addr_l=0x%x\n", addr_h, addr_l);
#endif

	writel(addr_h, &IIM_UA);
	writel(addr_l, &IIM_LA);
	/* Start Programming */
	writel(0x31, &IIM_FCTL);
	if (imx_poll_fuse_op_done(POLL_FUSE_PRGD) == 0) {
		ret = 0;
	}

	/* Enable IIM Program Protect */
	writel(0x0, &IIM_PROG_P);
	return ret;
}

/* Blow byte fuses based on the bank and row positions (all 0-based)
*/
static int imx_fuse_blow_byte(int bank,int row,unsigned char value)
{
	int i, ret = 0;

		for (i = 0; i < 8; i++) {
			if (((value >> i) & 0x1) == 0) {
				continue;
			}
			ret |= imx_fuse_blow(bank, row, i);
		}

	return ret;
}

static int imx_mac_read(unsigned char pmac[6])
{
	int i;
	int uninitialized = 0;

	for (i=0;i<6;i++) {
		pmac[6-1-i] = readl(&IIM_BANK_REG(0,(IIM0_MAC+i)));
	}

	/* uninitialized if all 00 */
	if ((pmac[0] == 0) && (pmac[1] == 0) && (pmac[2] == 0) &&
            (pmac[3] == 0) && (pmac[4] == 0) && (pmac[5] == 0))
                uninitialized = 1;

	/* uninitialized if all FF (could be safe) */
        if ((pmac[0] == 0xff) && (pmac[1] == 0xff) && (pmac[2] == 0xff) &&
	    (pmac[3] == 0xff) && (pmac[4] == 0xff) && (pmac[5] == 0xff))
	        uninitialized = 1;

	return uninitialized;
}

static int imx_mac_blow(unsigned char pmac[6])
{
	int i, ret = 1;
	unsigned char mac[6];
	int uninitialized = 0;

	uninitialized = imx_mac_read(mac);

	if (uninitialized) {
		ret = 0;
		for(i=0;i<6;i++) { 
			ret |= imx_fuse_blow_byte(0, (IIM0_MAC+i), pmac[6 - 1 -i]);
		}
	}

	return ret;
}


int do_imx_fuse(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	uint32_t bank, row, value, i;
	int ret = 1;

	if (argc < 2) {
		printf("It is too dangeous for you to use this command.\n");
		printf("Usage:\n%s\n", cmdtp->usage);
		return ret;
	}

	if ((!strcmp(argv[1], "sense"))&&((argc == 4))) {
		bank = simple_strtoul(argv[2], NULL, 16);
		row = simple_strtoul(argv[3], NULL, 16);

		printf("Sense read fuse at bank:%d row:%d\n", bank, row);
		ret = imx_sense_fuse(bank, row, 0);		
	}else if ((!strcmp(argv[1], "read"))&&((argc == 4))) {
		bank = simple_strtoul(argv[2], NULL, 16);
		row = simple_strtoul(argv[3], NULL, 16);

		printf("Shadow read fuse at bank:%d row:%d\n", bank, row);
		ret = imx_read_shadow_fuse(bank, row, 0);		
	}else if ((!strcmp(argv[1], "blow"))&&(argc == 5)) {
		bank = simple_strtoul(argv[2], NULL, 16);
		row = simple_strtoul(argv[3], NULL, 16);
		value = simple_strtoul(argv[4], NULL, 16);
		
		printf("Blowing fuse at bank:%d row:%d value:%d\n",
				bank, row, value);
		for (i = 0; i < 8; i++) {
			if (((value >> i) & 0x1) == 0) {
				continue;
			}
			if (imx_fuse_blow(bank, row, i) != 0) {
				printf("fuse_blow(bank: %d, row: %d, bit:"
					" %d failed\n",	bank, row, i);
			} else {
				printf("fuse_blow(bank: %d, row: %d, bit:"
					" %d successful\n", bank, row, i);
				ret = 0;
			}
		}

		/* read back fuse by shadow register if applicable */
		ret |= imx_read_shadow_fuse(bank, row, 0);

	}else if ((!strcmp(argv[1], "mac"))&&(argc == 2)) {
		unsigned char mac[6];
		ret = imx_mac_read(mac);

		printf("%siMX mac_addr in fuse: %02X:%02X:%02X:%02X:%02X:%02X\n",
		ret?"No ":"",
		mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	}else if ((!strcmp(argv[1], "mac"))&&(argc == 3)) {
		unsigned char mac[6];
		char *tmp, *end;

		tmp = argv[2];
		/* convert MAC from string to int */
		for (i=0; i<6; i++) {
		mac[i] = tmp ? simple_strtoul(tmp, &end, 16) : 0;
		if (tmp)
			tmp = (*end) ? end+1 : end;
		}

		ret = imx_mac_blow(mac);
		if (ret) {
			printf("Failed to blow mac_addr in fuse: "
				"%02X:%02X:%02X:%02X:%02X:%02X\n",
				mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
		} else {
			imx_mac_read(mac);
			printf("Mac_addr blowed in fuse: "
				"%02X:%02X:%02X:%02X:%02X:%02X\n",
				mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
		}
	}else if ((!strcmp(argv[1], "suid"))&&(argc == 2)) {

		printf("iMX SUID: ");
		for (i=0;i<6;i++) {
			printf("%02x", readl(&IIM_BANK_REG(1,(IIM1_SUID+i))));
		}
		printf("\n");
		ret = 0;
	}else if ((!strcmp(argv[1], "scc_key"))&&(argc == 2)) {

		printf("iMX SCC_KEY: ");
		for (i=0;i<21;i++) {
			printf("%02x", readl(&IIM_BANK_REG(0,(IIM0_SCC_KEY+i))));
		}
		printf("\n");
		ret = 0;
	} else { printf("arc:%d\n", argc);
		printf("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}

	return ret;
}

U_BOOT_CMD(imxfuse, 5, 0, do_imx_fuse,
	"imxfuse - Read/Blow some iMX fuses\n",\
	"sense <bank> <row> - sense read iMX fuses at <bank>/<row>\n" \
	"imxfuse read <bank> <row> - shadow read iMX fuses at <bank>/<row>\n" \
	"imxfuse blow <bank> <row> <value> - blow <value> at <bank>/<row>\n"\
	"	- Read/Blow 8 bits <Value> for some iMX fuses at <bank>/<row>\n"\
	"imxfuse mac [<mac_addr>]  - read/blow <mac_addr> in iMX fuses\n"\
	"imxfuse suid - read iMX SUID\n"\
	"imxfuse scc_key - read iMX SCC_KEY\n");

#endif /* CONFIG_CMD_IMX_FUSE */
