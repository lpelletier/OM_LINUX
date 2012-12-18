/*
 * Copyright (C) 2004 Sascha Hauer, Synertronixx GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 *
 */

#include <config.h>
#include <version.h>
#include <asm/arch/imx-regs.h>

.globl lowlevel_init
lowlevel_init:

	mov	r10, lr

/* Change PERCLK1DIV to 14 ie 14+1 */
	ldr		r0,	=PCDR
	ldr		r1,	=CFG_PCDR_VAL
	str		r1,	[r0]

/* set MCU PLL Control Register 0 */

	ldr		r0,	=MPCTL0
	ldr		r1,	=CFG_MPCTL0_VAL
	str		r1,	[r0]

/* set MCU PLL Control Register 1 */

	ldr		r0,	=MPCTL1
	ldr		r1,	=CFG_MPCTL1_VAL
	str		r1,	[r0]

/* set mpll restart bit */
	ldr		r0, =CSCR
	ldr		r1, [r0]
	orr		r1,r1,#(1<<21)
	str		r1, [r0]

	mov		r2,#0x10
1:
	mov		r3,#0x2000
2:
	subs	r3,r3,#1
	bne		2b

	subs	r2,r2,#1
	bne		1b

/* set System PLL Control Register 0 */

	ldr		r0,	=SPCTL0
	ldr		r1,	=CFG_SPCTL0_VAL
	str		r1,	[r0]

/* set System PLL Control Register 1 */

	ldr		r0,	=SPCTL1
	ldr		r1,	=CFG_SPCTL1_VAL
	str		r1,	[r0]

/* set spll restart bit */
	ldr		r0, =CSCR
	ldr		r1, [r0]
	orr		r1,r1,#(1<<22)
	str		r1, [r0]

	mov		r2,#0x10
1:
	mov		r3,#0x2000
2:
	subs	r3,r3,#1
	bne		2b

	subs	r2,r2,#1
	bne		1b

	ldr		r0,	=CSCR
	ldr		r1,	=CFG_CSCR_VAL
	str		r1,	[r0]

	ldr		r0,	=GPCR
	ldr		r1,	=CFG_GPCR_VAL
	str		r1,	[r0]

/* I have now read the ARM920 DataSheet back-to-Back, and have stumbled upon
 *this.....
 *
 * It would appear that from a Cold-Boot the ARM920T enters "FastBus" mode CP15
 * register 1, this stops it using the output of the PLL and thus runs at the
 * slow rate. Unless you place the Core into "Asynch" mode, the CPU will never
 * use the value set in the CM_OSC registers...regardless of what you set it
 * too!  Thus, although i thought i was running at 140MHz, i'm actually running
 * at 40!..

 * Slapping this into my bootloader does the trick...

 * MRC p15,0,r0,c1,c0,0	 ; read core configuration register
 * ORR r0,r0,#0xC0000000	; set asynchronous clocks and not fastbus mode
 * MCR p15,0,r0,c1,c0,0	 ; write modified value to core configuration
 * register
 */
	MRC p15,0,r0,c1,c0,0
	ORR r0,r0,#0xC0000000
	MCR p15,0,r0,c1,c0,0

/*	ldr		r0,	=GPR(0) 
	ldr		r1,	=CFG_GPR_A_VAL
	str		r1,	[r0]
*/

	ldr		r0,	=DR(0)
	ldr		r1,	=CFG_DR_A_VAL
	str		r1,	[r0]

	ldr		r0,	=OCR1(0)
	ldr		r1,	=CFG_OCR1_A_VAL
	str		r1,	[r0]

	ldr		r0,	=OCR2(0)
	ldr		r1,	=CFG_OCR2_A_VAL
	str		r1,	[r0]

	ldr		r0,	=ICONFA1(0)
	ldr		r1,	=CFG_ICFA1_A_VAL
	str		r1,	[r0]

	ldr		r0,	=ICONFA2(0)
	ldr		r1,	=CFG_ICFA2_A_VAL
	str		r1,	[r0]

	ldr		r0,	=ICONFB1(0)
	ldr		r1,	=CFG_ICFB1_A_VAL
	str		r1,	[r0]

	ldr		r0,	=ICONFB2(0)
	ldr		r1,	=CFG_ICFB2_A_VAL
	str		r1,	[r0]

	ldr		r0,	=ICR1(0)
	ldr		r1,	=CFG_ICR1_A_VAL
	str		r1,	[r0]

	ldr		r0,	=ICR2(0)
	ldr		r1,	=CFG_ICR2_A_VAL
	str		r1,	[r0]

	ldr		r0,	=IMR(0)
	ldr		r1,	=CFG_IMR_A_VAL
	str		r1,	[r0]

	ldr		r0,	=DDIR(0)
	ldr		r1,	=CFG_DDIR_A_VAL
	str		r1,	[r0]

	ldr		r0,	=GPR(0)
	ldr		r1,	=CFG_GPR_A_VAL
	str		r1,	[r0]

	ldr		r0,	=PUEN(0)
	ldr		r1,	=CFG_PUEN_A_VAL
	str		r1,	[r0]

	ldr		r0,	=GIUS(0)
	ldr		r1,	=CFG_GIUS_A_VAL
	str		r1,	[r0]

	ldr		r0,	=DR(1)
	ldr		r1,	=CFG_DR_B_VAL
	str		r1,	[r0]

	ldr		r0,	=OCR1(1)
	ldr		r1,	=CFG_OCR1_B_VAL
	str		r1,	[r0]

	ldr		r0,	=OCR2(1)
	ldr		r1,	=CFG_OCR2_B_VAL
	str		r1,	[r0]

	ldr		r0,	=ICONFA1(1)
	ldr		r1,	=CFG_ICFA1_B_VAL
	str		r1,	[r0]

	ldr		r0,	=ICONFA2(1)
	ldr		r1,	=CFG_ICFA2_B_VAL
	str		r1,	[r0]

	ldr		r0,	=ICONFB1(1)
	ldr		r1,	=CFG_ICFB1_B_VAL
	str		r1,	[r0]

	ldr		r0,	=ICONFB2(1)
	ldr		r1,	=CFG_ICFB2_B_VAL
	str		r1,	[r0]

	ldr		r0,	=ICR1(1)
	ldr		r1,	=CFG_ICR1_B_VAL
	str		r1,	[r0]

	ldr		r0,	=ICR2(1)
	ldr		r1,	=CFG_ICR2_B_VAL
	str		r1,	[r0]

	ldr		r0,	=IMR(1)
	ldr		r1,	=CFG_IMR_B_VAL
	str		r1,	[r0]

	ldr		r0,	=DDIR(1)
	ldr		r1,	=CFG_DDIR_B_VAL
	str		r1,	[r0]

	ldr		r0,	=GPR(1)
	ldr		r1,	=CFG_GPR_B_VAL
	str		r1,	[r0]

	ldr		r0,	=PUEN(1)
	ldr		r1,	=CFG_PUEN_B_VAL
	str		r1,	[r0]

	ldr		r0,	=GIUS(1)
	ldr		r1,	=CFG_GIUS_B_VAL
	str		r1,	[r0]

	ldr		r0,	=DR(2)
	ldr		r1,	=CFG_DR_C_VAL
	str		r1,	[r0]

	ldr		r0,	=OCR1(2)
	ldr		r1,	=CFG_OCR1_C_VAL
	str		r1,	[r0]

	ldr		r0,	=OCR2(2)
	ldr		r1,	=CFG_OCR2_C_VAL
	str		r1,	[r0]

	ldr		r0,	=ICONFA1(2)
	ldr		r1,	=CFG_ICFA1_C_VAL
	str		r1,	[r0]

	ldr		r0,	=ICONFA2(2)
	ldr		r1,	=CFG_ICFA2_C_VAL
	str		r1,	[r0]

	ldr		r0,	=ICONFB1(2)
	ldr		r1,	=CFG_ICFB1_C_VAL
	str		r1,	[r0]

	ldr		r0,	=ICONFB2(2)
	ldr		r1,	=CFG_ICFB2_C_VAL
	str		r1,	[r0]

	ldr		r0,	=ICR1(2)
	ldr		r1,	=CFG_ICR1_C_VAL
	str		r1,	[r0]

	ldr		r0,	=ICR2(2)
	ldr		r1,	=CFG_ICR2_C_VAL
	str		r1,	[r0]

	ldr		r0,	=IMR(2)
	ldr		r1,	=CFG_IMR_C_VAL
	str		r1,	[r0]

	ldr		r0,	=DDIR(2)
	ldr		r1,	=CFG_DDIR_C_VAL
	str		r1,	[r0]

	ldr		r0,	=GPR(2)
	ldr		r1,	=CFG_GPR_C_VAL
	str		r1,	[r0]

	ldr		r0,	=PUEN(2)
	ldr		r1,	=CFG_PUEN_C_VAL
	str		r1,	[r0]

	ldr		r0,	=GIUS(2)
	ldr		r1,	=CFG_GIUS_C_VAL
	str		r1,	[r0]

	ldr		r0,	=DR(3)
	ldr		r1,	=CFG_DR_D_VAL
	str		r1,	[r0]

	ldr		r0,	=OCR1(3)
	ldr		r1,	=CFG_OCR1_D_VAL
	str		r1,	[r0]

	ldr		r0,	=OCR2(3)
	ldr		r1,	=CFG_OCR2_D_VAL
	str		r1,	[r0]

	ldr		r0,	=ICONFA1(3)
	ldr		r1,	=CFG_ICFA1_D_VAL
	str		r1,	[r0]

	ldr		r0,	=ICONFA2(3)
	ldr		r1,	=CFG_ICFA2_D_VAL
	str		r1,	[r0]

	ldr		r0,	=ICONFB1(3)
	ldr		r1,	=CFG_ICFB1_D_VAL
	str		r1,	[r0]

	ldr		r0,	=ICONFB2(3)
	ldr		r1,	=CFG_ICFB2_D_VAL
	str		r1,	[r0]

	ldr		r0,	=ICR1(3)
	ldr		r1,	=CFG_ICR1_D_VAL
	str		r1,	[r0]

	ldr		r0,	=ICR2(3)
	ldr		r1,	=CFG_ICR2_D_VAL
	str		r1,	[r0]

	ldr		r0,	=IMR(3)
	ldr		r1,	=CFG_IMR_D_VAL
	str		r1,	[r0]

	ldr		r0,	=DDIR(3)
	ldr		r1,	=CFG_DDIR_D_VAL
	str		r1,	[r0]

	ldr		r0,	=GPR(3)
	ldr		r1,	=CFG_GPR_D_VAL
	str		r1,	[r0]

	ldr		r0,	=PUEN(3)
	ldr		r1,	=CFG_PUEN_D_VAL
	str		r1,	[r0]

	ldr		r0,	=GIUS(3)
	ldr		r1,	=CFG_GIUS_D_VAL
	str		r1,	[r0]

/* CS3 becomes CS3 by clearing reset default bit 1 in FMCR */

	ldr		r0,	=FMCR
	ldr		r1,	=CFG_FMCR_VAL
	str		r1,	[r0]

	ldr		r0,	=CS0U
	ldr		r1,	=CFG_CS0U_VAL
	str		r1,	[r0]

	ldr		r0,	=CS0L
	ldr		r1,	=CFG_CS0L_VAL
	str		r1,	[r0]

	ldr		r0,	=CS1U
	ldr		r1,	=CFG_CS1U_VAL
	str		r1,	[r0]

	ldr		r0,	=CS1L
	ldr		r1,	=CFG_CS1L_VAL
	str		r1,	[r0]

	ldr		r0,	=CS2U
	ldr		r1,	=CFG_CS2U_VAL
	str		r1,	[r0]

	ldr		r0,	=CS2L
	ldr		r1,	=CFG_CS2L_VAL
	str		r1,	[r0]

	ldr		r0,	=CS3U
	ldr		r1,	=CFG_CS3U_VAL
	str		r1,	[r0]

	ldr		r0,	=CS3L
	ldr		r1,	=CFG_CS3L_VAL
	str		r1,	[r0]

	ldr		r0,	=CS4U
	ldr		r1,	=CFG_CS4U_VAL
	str		r1,	[r0]

	ldr		r0,	=CS4L
	ldr		r1,	=CFG_CS4L_VAL
	str		r1,	[r0]

	ldr		r0,	=CS5U
	ldr		r1,	=CFG_CS5U_VAL
	str		r1,	[r0]

	ldr		r0,	=CS5L
	ldr		r1,	=CFG_CS5L_VAL
	str		r1,	[r0]

	adr	r0, sdramsetup	/* r0 <- current position of code */
	ldr	r1, =CFG_SDRAM_1_BASE	/* test if we run from flash or RAM */
	cmp	r1, r0		/* don't reloc during debug*/
	bhi		sdramsetup
	ldr	r1, =CFG_FLASH_BASE	/* test if we run from flash or RAM */
	cmp	r0, r1		/* don't reloc during debug */
	bmi		endofmemsetup

/* SDRAM Setup */
sdramsetup:
	ldr		r0, =SDCTL0
	ldr		r1, =CFG_PRECHARGE_CMD
	str		r1,	[r0]

	ldr		r1, =CFG_SDRAM_1_BASE+CFG_SDRAM_PRECHARGE_ALL_VAL
	ldr		r2,	[r1]

	ldr		r1, =CFG_AUTOREFRESH_CMD
	str		r1,	[r0]

	ldr		r1, =CFG_SDRAM_1_BASE
	ldr		r2,	[r1] /* Issue AutoRefresh Command */
	ldr		r2,	[r1]
	ldr		r2,	[r1]
	ldr		r2,	[r1]
	ldr		r2,	[r1]
	ldr		r2,	[r1]
	ldr		r2,	[r1]
	ldr		r2,	[r1]

	ldr		r1, =CFG_SET_MODE_REG_CMD
	str		r1,	[r0]

	ldr		r1, =CFG_SDRAM_1_BASE+CFG_SDRAM_MODE_REGISTER_VAL
	str		r2,	[r1]

	ldr		r1, =CFG_NORMAL_RW_CMD 
	str		r1,	[r0]

endofmemsetup:
	mov	pc,r10
