/*
 * (C) Copyright 2002
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

#if (CONFIG_FPGA)

#include <asm/arch/imx-regs.h>
#include <asm/io.h>
#include <config.h>

extern void imx_gpio_mode (int gpio_mode);


#define GPIO_PORT(x)  ((x >> 5) & 3)
#define GPIO_SET(x)   (DR(GPIO_PORT(x)) |= (1<<(x & GPIO_PIN_MASK)))
#define GPIO_CLEAR(x) (DR(GPIO_PORT(x)) &= ~(1<<(x & GPIO_PIN_MASK)))
#define GPIO_WRITE(x,y) ( y ? GPIO_SET(x) : GPIO_CLEAR(x) )
#define GPIO_READ(x)  ((SSR (GPIO_PORT(x)) & (1<<(x & GPIO_PIN_MASK))))

#if 0
#define FPGA_DEBUG
#endif

#ifdef FPGA_DEBUG
#define	PRINTF(fmt,args...)	printf (fmt ,##args)
#else
#define	PRINTF(fmt,args...)
#endif

/*
 * Port bit numbers for the serial slave controls
 */
#define FPGA_INIT	CFG_FPGA_INIT
#define FPGA_DONE	CFG_FPGA_DONE
#define FPGA_DIN	CFG_FPGA_DATA
#define FPGA_PROGRAM	CFG_FPGA_PRG
#define FPGA_CLOCK	CFG_FPGA_CLK

/* Note that these are pointers to code that is in Flash.  They will be
 * relocated at runtime.
 * Spartan2 code is used to download our Spartan 3 :) code is compatible. 
 * Just take care about the file size  
*/

/*
 * nitialize GPIO port B before download
 */
int
fpga_pre_fn (int cookie)
{
	PRINTF ("%s:%d: FPGA PRE ", __FUNCTION__, __LINE__);

	// Initialize GPIO pins
	imx_gpio_mode (FPGA_INIT | GPIO_DR | GPIO_IN );
	imx_gpio_mode (FPGA_DONE | GPIO_DR | GPIO_IN );
	imx_gpio_mode (FPGA_DIN  | GPIO_DR | GPIO_OUT );
	imx_gpio_mode (FPGA_PROGRAM | GPIO_DR | GPIO_OUT );
	imx_gpio_mode (FPGA_CLOCK | GPIO_DR | GPIO_OUT );
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
	GPIO_WRITE( FPGA_PROGRAM, !assert);
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
	GPIO_WRITE( FPGA_CLOCK, assert_clk);
	return assert_clk;
}

/*
 * Test the state of the active-low FPGA INIT line.  Return 1 on INIT
 * asserted (low).
 */
int
fpga_init_fn (int cookie)
{
	PRINTF ("%s:%d: INIT check... ", __FUNCTION__, __LINE__);
	return(!GPIO_READ(FPGA_INIT));
}

/*
 * Test the state of the active-high FPGA DONE pin
 */
int
fpga_done_fn (int cookie)
{
	PRINTF ("%s:%d: DONE check... ", __FUNCTION__, __LINE__);
	return(GPIO_READ(FPGA_DONE));
}

/*
 * Set the FPGA's data line to the specified level
 */
int
fpga_wr_fn (int assert_write, int flush, int cookie)
{
	PRINTF ("%s:%d: DATA write... ", __FUNCTION__, __LINE__);
	GPIO_WRITE( FPGA_DIN, assert_write);
	return assert_write;
}

#endif /* CONFIG_FPGA */
