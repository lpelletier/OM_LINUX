/*
 * For clock initialization, see chapter 3 of the "MCIMX27 Multimedia
 * Applications Processor Reference Manual, Rev. 0.2".
 *
 * (C) Copyright 2008 Eric Jarrige <eric.jarrige@armadeus.org>
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
#include <version.h>
#include <asm/arch/imx-regs.h>



#define writel(reg, val) \
	ldr		r0,	=reg;	\
	ldr		r1,	=val;	\
	str		r1,   [r0];

.macro port_init
    writel( FMCR, CFG_FMCR_VAL)
/* PORTA */
    writel( DR(PORTA), CFG_DR_A_VAL)
    writel( OCR1(PORTA), CFG_OCR1_A_VAL)
    writel( OCR2(PORTA), CFG_OCR2_A_VAL)
    writel( ICONFA1(PORTA), CFG_ICFA1_A_VAL)
    writel( ICONFA2(PORTA), CFG_ICFA2_A_VAL)
    writel( ICONFB1(PORTA), CFG_ICFB1_A_VAL)
    writel( ICONFB2(PORTA), CFG_ICFB2_A_VAL)
    writel( ICR1(PORTA), CFG_ICR1_A_VAL)
    writel( ICR2(PORTA), CFG_ICR2_A_VAL)
    writel( IMR(PORTA), CFG_IMR_A_VAL)
    writel( DDIR(PORTA), CFG_DDIR_A_VAL)
    writel( GPR(PORTA), CFG_GPR_A_VAL)
    writel( PUEN(PORTA), CFG_PUEN_A_VAL)
    writel( GIUS(PORTA), CFG_GIUS_A_VAL)
/* PORTB */
    writel( DR(PORTB), CFG_DR_B_VAL)
    writel( OCR1(PORTB), CFG_OCR1_B_VAL)
    writel( OCR2(PORTB), CFG_OCR2_B_VAL)
    writel( ICONFA1(PORTB), CFG_ICFA1_B_VAL)
    writel( ICONFA2(PORTB), CFG_ICFA2_B_VAL)
    writel( ICONFB1(PORTB), CFG_ICFB1_B_VAL)
    writel( ICONFB2(PORTB), CFG_ICFB2_B_VAL)
    writel( ICR1(PORTB), CFG_ICR1_B_VAL)
    writel( ICR2(PORTB), CFG_ICR2_B_VAL)
    writel( IMR(PORTB), CFG_IMR_B_VAL)
    writel( DDIR(PORTB), CFG_DDIR_B_VAL)
    writel( GPR(PORTB), CFG_GPR_B_VAL)
    writel( PUEN(PORTB), CFG_PUEN_B_VAL)
    writel( GIUS(PORTB), CFG_GIUS_B_VAL)
/* PORTC */
    writel( DR(PORTC), CFG_DR_C_VAL)
    writel( OCR1(PORTC), CFG_OCR1_C_VAL)
    writel( OCR2(PORTC), CFG_OCR2_C_VAL)
    writel( ICONFA1(PORTC), CFG_ICFA1_C_VAL)
    writel( ICONFA2(PORTC), CFG_ICFA2_C_VAL)
    writel( ICONFB1(PORTC), CFG_ICFB1_C_VAL)
    writel( ICONFB2(PORTC), CFG_ICFB2_C_VAL)
    writel( ICR1(PORTC), CFG_ICR1_C_VAL)
    writel( ICR2(PORTC), CFG_ICR2_C_VAL)
    writel( IMR(PORTC), CFG_IMR_C_VAL)
    writel( DDIR(PORTC), CFG_DDIR_C_VAL)
    writel( GPR(PORTC), CFG_GPR_C_VAL)
    writel( PUEN(PORTC), CFG_PUEN_C_VAL)
    writel( GIUS(PORTC), CFG_GIUS_C_VAL)
/* PORTD */
    writel( DR(PORTD), CFG_DR_D_VAL)
    writel( OCR1(PORTD), CFG_OCR1_D_VAL)
    writel( OCR2(PORTD), CFG_OCR2_D_VAL)
    writel( ICONFA1(PORTD), CFG_ICFA1_D_VAL)
    writel( ICONFA2(PORTD), CFG_ICFA2_D_VAL)
    writel( ICONFB1(PORTD), CFG_ICFB1_D_VAL)
    writel( ICONFB2(PORTD), CFG_ICFB2_D_VAL)
    writel( ICR1(PORTD), CFG_ICR1_D_VAL)
    writel( ICR2(PORTD), CFG_ICR2_D_VAL)
    writel( IMR(PORTD), CFG_IMR_D_VAL)
    writel( DDIR(PORTD), CFG_DDIR_D_VAL)
    writel( GPR(PORTD), CFG_GPR_D_VAL)
    writel( PUEN(PORTD), CFG_PUEN_D_VAL)
    writel( GIUS(PORTD), CFG_GIUS_D_VAL)
/* PORTE */
    writel( DR(PORTE), CFG_DR_E_VAL)
    writel( OCR1(PORTE), CFG_OCR1_E_VAL)
    writel( OCR2(PORTE), CFG_OCR2_E_VAL)
    writel( ICONFA1(PORTE), CFG_ICFA1_E_VAL)
    writel( ICONFA2(PORTE), CFG_ICFA2_E_VAL)
    writel( ICONFB1(PORTE), CFG_ICFB1_E_VAL)
    writel( ICONFB2(PORTE), CFG_ICFB2_E_VAL)
    writel( ICR1(PORTE), CFG_ICR1_E_VAL)
    writel( ICR2(PORTE), CFG_ICR2_E_VAL)
    writel( IMR(PORTE), CFG_IMR_E_VAL)
    writel( DDIR(PORTE), CFG_DDIR_E_VAL)
    writel( GPR(PORTE), CFG_GPR_E_VAL)
    writel( PUEN(PORTE), CFG_PUEN_E_VAL)
    writel( GIUS(PORTE), CFG_GIUS_E_VAL)
/* PORTF */
    writel( DR(PORTF), CFG_DR_F_VAL)
    writel( OCR1(PORTF), CFG_OCR1_F_VAL)
    writel( OCR2(PORTF), CFG_OCR2_F_VAL)
    writel( ICONFA1(PORTF), CFG_ICFA1_F_VAL)
    writel( ICONFA2(PORTF), CFG_ICFA2_F_VAL)
    writel( ICONFB1(PORTF), CFG_ICFB1_F_VAL)
    writel( ICONFB2(PORTF), CFG_ICFB2_F_VAL)
    writel( ICR1(PORTF), CFG_ICR1_F_VAL)
    writel( ICR2(PORTF), CFG_ICR2_F_VAL)
    writel( IMR(PORTF), CFG_IMR_F_VAL)
    writel( DDIR(PORTF), CFG_DDIR_F_VAL)
    writel( GPR(PORTF), CFG_GPR_F_VAL)
    writel( PUEN(PORTF), CFG_PUEN_F_VAL)
    writel( GIUS(PORTF), CFG_GIUS_F_VAL)
.endm /* port_init */

.macro init_aipi
	/*
	 * setup AIPI1 and AIPI2
	 */
	writel(AIPI1_PSR0, CFG_AIPI1_PSR0_VAL)
	writel(AIPI1_PSR1, CFG_AIPI1_PSR1_VAL)
	writel(AIPI2_PSR0, CFG_AIPI2_PSR0_VAL)
	writel(AIPI2_PSR1, CFG_AIPI2_PSR1_VAL)

	/* Change SDRAM signal strengh */
	ldr r0, =GPCR
	ldr r1, =CFG_GPCR_VAL
	ldr r5, [r0]
	orr r5, r5, r1
	str r5, [r0]

.endm /* init_aipi */

.macro init_clock
	ldr r0, =CSCR
	/* disable MPLL/SPLL first */
	ldr r1, [r0]
	bic r1, r1, #(CSCR_MPEN|CSCR_SPEN)
	str r1, [r0]

 	/*
	 * pll clock initialization predefined in apf27.h
	 */
	writel(MPCTL0, CFG_MPCTL0_VAL)
	writel(SPCTL0, CFG_SPCTL0_VAL)

	writel(CSCR, CFG_CSCR_VAL | CSCR_MPLL_RESTART | CSCR_SPLL_RESTART)

	/*
	 * add some delay here
	 */
	mov r1, #0x1000
	1:  subs r1, r1, #0x1
	bne 1b

	/* peripheral clock divider */
	writel(PCDR0, CFG_PCDR0_VAL)
	writel(PCDR1, CFG_PCDR1_VAL)

	/* Configure PCCR0 and PCCR1 */
	writel(PCCR0, CFG_PCCR0_VAL)
	writel(PCCR1, CFG_PCCR1_VAL)


.endm /* init_clock */

.macro sdram_init
	/* wait for SDRAM/LPDDR ready (SDRAMRDY) */
	ldr  r0, =ESDMISC
	ldr  r4, =ESDMISC_SDRAM_RDY
2:	ldr  r1, [r0]
	ands r1, r1, r4
	bpl 2b

	/* LPDDR Soft Reset Mobile/Low Power DDR SDRAM. */
	ldr		r0, =ESDMISC
	ldr		r4, =CFG_ESDMISC_VAL
	orr		r1, r4, #ESDMISC_MDDR_DL_RST
	str		r1, [r0]

	/* Hold for more than 200ns */
	ldr r1, =0x10000
	1:	subs r1, r1, #0x1
	bne 1b

	str		r4, [r0]

	writel(ESDCFG0, CFG_SDRAM_ESDCFG_REGISTER_VAL)

	/* writel(ESDCTL0, CFG_PRECHARGE_CMD) */
	ldr		r0, =ESDCTL0
	ldr		r1, =CFG_PRECHARGE_CMD
	str		r1, [r0]

	/* writeb(0xA0001000, any value) */
	ldr		r1, =CFG_SDRAM_1_BASE+CFG_SDRAM_PRECHARGE_ALL_VAL
	ldrb		r2, [r1]

	/* writel(ESDCTL0, CFG_AUTOREFRESH_CMD) */
	ldr		r1, =CFG_AUTOREFRESH_CMD
	str		r1, [r0]

	ldr 		r4, =CFG_SDRAM_1_BASE	/* CSD0 base address	*/

	ldr 		r6,=0x7			/* load loop counter	*/
1:	ldr 		r5,[r4]			/* run auto-refresh cycle to array 0 */
	subs 		r6,r6,#1		/* decrease counter value */
	bne 1b

	/* writel(CFG_PRECHARGE_CMD, CFG_SET_MODE_REG_CMD) */
	ldr		r1, =CFG_SET_MODE_REG_CMD
	str		r1, [r0]

	/* set standard mode register */
	ldr		r4, = CFG_SDRAM_1_BASE+CFG_SDRAM_MODE_REGISTER_VAL
	ldrb		r2, [r4]

	/* set extended mode register */
	ldr		r4, =CFG_SDRAM_1_BASE+CFG_SDRAM_EXT_MODE_REGISTER_VAL
	ldrb		r5, [r4]

	/* writel(CFG_PRECHARGE_CMD, CFG_NORMAL_RW_CMD) */
	ldr		r1, =CFG_NORMAL_RW_CMD
	str		r1, [r0]

#if (CONFIG_NR_DRAM_BANKS > 1)
	/* 2nd sdram */
	writel(ESDCFG1, CFG_SDRAM_ESDCFG_REGISTER_VAL )

	/* writel(ESDCTL1, CFG_PRECHARGE_CMD) */
	ldr		r0, =ESDCTL1
	ldr		r1, =CFG_PRECHARGE_CMD
	str		r1, [r0]

	/* writeb(0xB0001000, any value) */
	ldr		r1, =CFG_SDRAM_2_BASE+CFG_SDRAM_PRECHARGE_ALL_VAL
	ldrb		r2, [r1]

	/* writel(ESDCTL1, CFG_AUTOREFRESH_CMD) */
	ldr		r1, =CFG_AUTOREFRESH_CMD
	str		r1, [r0]

	ldr 		r4, =CFG_SDRAM_2_BASE	/* CSD1 base address */

	ldr 		r6,=0x7			/* load loop counter */
1:	ldr 		r5,[r4]			/* run auto-refresh cycle to array 0 */
	subs 		r6,r6,#1		/* decrease counter value */
	bne 1b

	/* writel(ESDCTL1, CFG_SET_MODE_REG_CMD) */
	ldr		r1, =CFG_SET_MODE_REG_CMD
	str		r1, [r0]

	/* set standard mode register */
	ldr		r4, =CFG_SDRAM_2_BASE+CFG_SDRAM_MODE_REGISTER_VAL
	ldrb		r2, [r4]

	/* set extended mode register */
	ldr		r4, =CFG_SDRAM_2_BASE+CFG_SDRAM_EXT_MODE_REGISTER_VAL
	ldrb		r2, [r4]

	/* writel(ESDCTL1, CFG_NORMAL_RW_CMD) */
	ldr		r1, =CFG_NORMAL_RW_CMD
	str		r1, [r0]
#endif  /* CONFIG_NR_DRAM_BANKS > 1 */
.endm /* sdram_init */

.globl board_init_lowlevel
board_init_lowlevel:
.globl	lowlevel_init
lowlevel_init:

	mov	r10, lr

	init_aipi

	/* configure csx */
	writel(CS0U, CFG_CS0U_VAL)
	writel(CS0L, CFG_CS0L_VAL)
	writel(CS0A, CFG_CS0A_VAL)

	writel(CS1U, CFG_CS1U_VAL)
	writel(CS1L, CFG_CS1L_VAL)
	writel(CS1A, CFG_CS1A_VAL)

	writel(CS2U, CFG_CS2U_VAL)
	writel(CS2L, CFG_CS2L_VAL)
	writel(CS2A, CFG_CS2A_VAL)

	writel(CS3U, CFG_CS3U_VAL)
	writel(CS3L, CFG_CS3L_VAL)
	writel(CS3A, CFG_CS3A_VAL)

	writel(CS4U, CFG_CS4U_VAL)
	writel(CS4L, CFG_CS4L_VAL)
	writel(CS4A, CFG_CS4A_VAL)

	writel(CS5U, CFG_CS5U_VAL)
	writel(CS5L, CFG_CS5L_VAL)
	writel(CS5A, CFG_CS5A_VAL)

	writel(EIM,  CFG_EIM_VAL)

	port_init

    /* Configure FPGA CLKO */
	writel(CCSR, CFG_CCSR_VAL)

    /* Configure strentgh for FPGA */
	writel(DSCR10, CFG_DSCR10_VAL)
	writel(DSCR3, CFG_DSCR3_VAL)
	writel(DSCR7, CFG_DSCR7_VAL)
	writel(DSCR2, CFG_DSCR2_VAL)



	/* skip clock and sdram initialization if we run from ram */
	cmp	pc, #0xa0000000
	bls	1f
	cmp	pc, #0xc0000000
	bhi	1f

	mov	pc,r10
1:
	init_clock

	sdram_init

	mov	pc,r10

