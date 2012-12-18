/*
 * (C) Copyright 2002-2008
 * Eric Jarrige <eric.jarrige@armadeus.org>
 * Rich Ireland, Enterasys Networks, rireland@enterasys.com.
 * Keith Outwater, keith_outwater@mvis.com.
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
#include <common.h>

#if defined(CONFIG_FPGA)

#include <asm/arch/imx-regs.h>
#include <asm/io.h>
#include <command.h>
#include <config.h>
#include "fpga.h"
#include <spartan3.h>

#define GPIO_PORT(x)  ((x & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT)
#define GPIO_SET(x)   (DR(GPIO_PORT(x)) |= (1<<(x & GPIO_PIN_MASK)))
#define GPIO_CLEAR(x) (DR(GPIO_PORT(x)) &= ~(1<<(x & GPIO_PIN_MASK)))
#define GPIO_WRITE(x,y) ( y ? GPIO_SET(x) : GPIO_CLEAR(x) )
#define GPIO_READ(x)  ((SSR (GPIO_PORT(x)) & (1<<(x & GPIO_PIN_MASK)))>> \
                        (x & GPIO_PIN_MASK))
#ifdef FPGA_DEBUG
#define	PRINTF(fmt,args...)	printf (fmt ,##args)
#else
#define	PRINTF(fmt,args...)
#endif

/* Note that these are pointers to code that is in Flash.  They will be
 * relocated at runtime.
 * Spartan2 code is used to download our Spartan 3 :) code is compatible.
 * Just take care about the file size
*/
Xilinx_Spartan3_Slave_Parallel_fns fpga_fns = {
	fpga_pre_fn,
	fpga_pgm_fn,
	fpga_init_fn,
	NULL,
	fpga_done_fn,
	fpga_clk_fn,
	fpga_cs_fn,
	fpga_wr_fn,
	fpga_rdata_fn,
	fpga_wdata_fn,
   fpga_busy_fn,
	fpga_abort_fn,
	fpga_post_fn,
};

Xilinx_desc fpga[CONFIG_FPGA_COUNT] = {
	{Xilinx_Spartan3,
	 slave_parallel,
	 1196128l/8,
	 (void *) &fpga_fns,
	 0}
};

/*
 * nitialize GPIO port B before download
 */
int
fpga_pre_fn (int cookie)
{
	// Initialize GPIO pins
	GPIO_SET(CFG_FPGA_PWR);
	imx_gpio_mode (CFG_FPGA_INIT | GPIO_IN | GPIO_PUEN | GPIO_GPIO);
	imx_gpio_mode (CFG_FPGA_DONE | GPIO_IN | GPIO_PUEN | GPIO_GPIO);
	imx_gpio_mode (CFG_FPGA_PRG | GPIO_OUT | GPIO_PUEN | GPIO_GPIO);
	imx_gpio_mode (CFG_FPGA_CLK | GPIO_OUT | GPIO_PUEN | GPIO_GPIO);
	imx_gpio_mode (CFG_FPGA_RW | GPIO_OUT | GPIO_PUEN | GPIO_GPIO);
	imx_gpio_mode (CFG_FPGA_CS | GPIO_OUT | GPIO_PUEN | GPIO_GPIO);
	imx_gpio_mode (CFG_FPGA_SUSPEND | GPIO_OUT | GPIO_PUEN | GPIO_GPIO);
	GPIO_SET(CFG_FPGA_RESET);
	imx_gpio_mode (CFG_FPGA_RESET | GPIO_OUT | GPIO_PUEN | GPIO_GPIO);
	imx_gpio_mode (CFG_FPGA_PWR | GPIO_OUT | GPIO_PUEN | GPIO_GPIO);
	GPIO_SET(CFG_FPGA_PRG);
	GPIO_SET(CFG_FPGA_CLK);
	GPIO_SET(CFG_FPGA_RW);
	GPIO_SET(CFG_FPGA_CS);
	GPIO_CLEAR(CFG_FPGA_SUSPEND);
// 	GPIO_CLEAR(CFG_FPGA_RESET);
	GPIO_CLEAR(CFG_FPGA_PWR);
	udelay(30000); /*wait until supply started*/

	return cookie;
}

/*
 * Set the FPGA's active-low program line to the specified level
 */
int
fpga_pgm_fn (int assert, int flush, int cookie)
{
	PRINTF ("%s:%d: FPGA PROGRAM %s", __FUNCTION__, __LINE__,
					assert?"high":"low");
	GPIO_WRITE( CFG_FPGA_PRG, !assert);
	return assert;
}

/*
 * Set the FPGA's active-high clock line to the specified level
 */
int
fpga_clk_fn (int assert_clk, int flush, int cookie)
{
	PRINTF ("%s:%d: FPGA CLOCK %s", __FUNCTION__, __LINE__,
					assert_clk?"high":"low");
	GPIO_WRITE( CFG_FPGA_CLK, !assert_clk);
	return assert_clk;
}

/*
 * Test the state of the active-low FPGA INIT line.  Return 1 on INIT
 * asserted (low).
 */
int
fpga_init_fn (int cookie)
{
	int value;
	PRINTF ("%s:%d: INIT check... ", __FUNCTION__, __LINE__);
	value = GPIO_READ(CFG_FPGA_INIT);
	//printf("init value read %x",value);
#ifdef CONFIG_SYS_FPGA_IS_PROTO
	return value;
#else
	return !value;
#endif
}

/*
 * Test the state of the active-high FPGA DONE pin
 */
int
fpga_done_fn (int cookie)
{
	PRINTF ("%s:%d: DONE check... %s", __FUNCTION__, __LINE__,
					GPIO_READ(CFG_FPGA_DONE)?"high":"low");
	return(GPIO_READ(CFG_FPGA_DONE)?FPGA_SUCCESS:FPGA_FAIL);
}

/*
 * Set the FPGA's wr line to the specified level
 */
int
fpga_wr_fn (int assert_write, int flush, int cookie)
{
	PRINTF ("%s:%d: FPGA RW... %s ", __FUNCTION__, __LINE__,
					assert_write?"high":"low");
	GPIO_WRITE( CFG_FPGA_RW, !assert_write);
	return assert_write;
}

int
fpga_cs_fn (int assert_cs, int flush, int cookie)
{
	PRINTF ("%s:%d: FPGA CS %s ", __FUNCTION__, __LINE__,
					assert_cs?"high":"low");
	GPIO_WRITE( CFG_FPGA_CS, !assert_cs);
	return assert_cs;
}

int
fpga_rdata_fn ( unsigned char *data, int cookie )
{
	PRINTF ("%s:%d: FPGA READ DATA %02X ", __FUNCTION__, __LINE__,
					*((char*)CFG_FPGA_RDATA));
	*data = (unsigned char)((*((unsigned short*)CFG_FPGA_RDATA))&0x00FF);
	return *data;
}

int
fpga_wdata_fn ( unsigned char data, int cookie )
{
	PRINTF ("%s:%d: FPGA WRITE DATA %02X ", __FUNCTION__, __LINE__,
					data);
	*((unsigned short*)CFG_FPGA_WDATA) = data;
	return data;
}

int
fpga_abort_fn ( int cookie )
{
	return cookie;
}


int
fpga_busy_fn ( int cookie )
{
	return 1;
}

int
fpga_post_fn (int cookie)
{
	PRINTF ("%s:%d: FPGA POST ", __FUNCTION__, __LINE__);

	imx_gpio_mode (CFG_FPGA_RW | GPIO_PF | GPIO_PUEN);
	imx_gpio_mode (CFG_FPGA_CS | GPIO_PF | GPIO_PUEN);
	imx_gpio_mode (CFG_FPGA_CLK | GPIO_PF | GPIO_PUEN);
	GPIO_SET(CFG_FPGA_PRG);
	GPIO_CLEAR(CFG_FPGA_RESET);
	imx_gpio_mode (CFG_FPGA_RESET | GPIO_OUT | GPIO_PUEN | GPIO_GPIO);
	return cookie;
}

/*
 * Initialize the fpga.  Return 1 on success, 0 on failure.
 */
int
APF27_init_fpga (u_char * buffer, size_t size)
{
	char *autoload = getenv ("firmware_autoload");
	DECLARE_GLOBAL_DATA_PTR;

	int i,lout=0;

	PRINTF ("%s:%d: Initialize FPGA interface (relocation offset = 0x%.8lx)\n",
		__FUNCTION__, __LINE__, gd->reloc_off);
	fpga_init (gd->reloc_off);

	for (i = 0; i < CONFIG_FPGA_COUNT; i++) {
		PRINTF ("%s:%d: Adding fpga %d\n", __FUNCTION__, __LINE__, i);
		fpga_add (fpga_xilinx, &fpga[i]);
	}

	if ((size >= fpga[0].size) && ( autoload ) && (0 == strcmp(autoload, "1"))) {
		if (FPGA_SUCCESS != fpga_load( 0, (void *)buffer, size )) {
			lout = 1;
			printf("Firmware download failed!\n");
		}
      else
          printf("Firmware successfully programmed\n");
	}
	return lout;
}

#endif /* CONFIG_FPGA */
