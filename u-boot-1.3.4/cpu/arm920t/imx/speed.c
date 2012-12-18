/*
 *
 * (c) 2004 Sascha Hauer <sascha@saschahauer.de>
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


#include <common.h>
#if defined (CONFIG_IMX)

#include <asm/arch/imx-regs.h>

/* ------------------------------------------------------------------------- */
/* NOTE: This describes the proper use of this file.
 *
 * CONFIG_SYS_CLK_FREQ should be defined as the input frequency of the PLL.
 * SH FIXME: 16780000 in our case
 * get_FCLK(), get_HCLK(), get_PCLK() and get_UCLK() return the clock of
 * the specified bus in HZ.
 */
/* ------------------------------------------------------------------------- */
static ulong get_PLLCLK(u32 sys_clk_freq, u32 pllctl0)
{
	/* FIXME: We assume System_SEL = 0 here */
	u32 mfi = (pllctl0 >> 10) & 0xf;
	u32 mfn = pllctl0 & 0x3ff;
	u32 mfd = (pllctl0 >> 16) & 0x3ff;
	u32 pd =  (pllctl0 >> 26) & 0xf;

	mfi = mfi<=5 ? 5 : mfi;
/*	return (2*(CONFIG_SYSPLL_CLK_FREQ>>10)*( (mfi<<10) + (mfn<<10)/(mfd+1)))/(pd+1);*/
	return (2*(u64)sys_clk_freq* (mfi*(mfd+1) + mfn))/((mfd+1)*(pd+1));
}

ulong get_systemPLLCLK(void)
{
	return (get_PLLCLK(CONFIG_SYSPLL_CLK_FREQ, SPCTL0));
}

ulong get_mcuPLLCLK(void)
{
	return (get_PLLCLK(CONFIG_SYS_CLK_FREQ, MPCTL0));
}

ulong get_FCLK(void)
{
	return (( CSCR>>15)&1) ? get_mcuPLLCLK()>>1 : get_mcuPLLCLK();
}

/* return HCLK frequency */
ulong get_HCLK(void)
{
	u32 bclkdiv = (( CSCR >> 10 ) & 0xf) + 1;
	return get_systemPLLCLK() / bclkdiv;
}

/* return BCLK frequency */
ulong get_BCLK(void)
{
	return get_HCLK();
}

ulong get_PERCLK1(void)
{
	return get_systemPLLCLK() / (((PCDR) & 0xf)+1);
}

ulong get_PERCLK2(void)
{
	return get_systemPLLCLK() / (((PCDR>>4) & 0xf)+1);
}

ulong get_PERCLK3(void)
{
	return get_systemPLLCLK() / (((PCDR>>16) & 0x7f)+1);
}

#endif /* defined (CONFIG_IMX) */
