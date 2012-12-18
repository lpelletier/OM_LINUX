/*
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 *
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Alex Zuepke <azu@sysgo.de>
 *
 * (C) Copyright 2002
 * Gary Jennejohn, DENX Software Engineering, <gj@denx.de>
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
#include <asm/arch/imx-regs.h>
#include <asm/arch/clock.h>

#define GPT(x) __REG(IMX_TIM1_BASE + (x))

/*uint64_t imx_clocksource_read(void)
{
	return GPT(GPT_TCN);
}
*/
/*static struct clocksource cs = {
	.read	= imx_clocksource_read,
	.mask	= 0xffffffff,
	.shift	= 10,
};*/

int timer_init (void)
{
	int i;
	/* setup GP Timer 1 */
	GPT(GPT_TCTL) = TCTL_SWR;

#ifdef CONFIG_IMX27
	PCCR0 |= PCCR0_GPT1_EN;
	PCCR1 |= PCCR1_PERCLK1_EN;
#endif

	for (i = 0; i < 100; i++)
		GPT(GPT_TCTL) = 0; /* We have no udelay by now */

	GPT(GPT_TPRER) = imx_get_perclk1() / 1000000; /* 1 MHz */
	GPT(GPT_TCTL) |= TCTL_FRR | (1<<TCTL_CLKSOURCE); /* Freerun Mode, PERCLK1 input */
	GPT(GPT_TCTL) &= ~TCTL_TEN;
	GPT(GPT_TCTL) |= TCTL_TEN; /* Enable timer */

/*	cs.mult = clocksource_hz2mult(imx_get_perclk1(), cs.shift);

	init_clock(&cs);*/

	return 0;
}

void reset_timer_masked (void)
{
       GPT(GPT_TCTL) = 0;
/*       GPT(GPT_TCTL) = GPTCR_CLKSOURCE_32 | GPTCR_TEN;*/ /* Freerun Mode, PERCLK1 input 
*/
	GPT(GPT_TCTL) |= TCTL_FRR | (1<<TCTL_CLKSOURCE); /* Freerun Mode, PERCLK1 input */
	GPT(GPT_TCTL) &= ~TCTL_TEN;
	GPT(GPT_TCTL) |= TCTL_TEN; /* Enable timer */

}

ulong get_timer_masked (void)
{
       ulong val = GPT(GPT_TCN);
       return val;
}

ulong get_timer (ulong base)
{
       return get_timer_masked () - base;
}

void set_timer (ulong t)
{
}

void reset_timer(void)
{
	reset_timer_masked();
}

/*
 * This function is derived from PowerPC code (read timebase as long long).
 * On ARM it just returns the timer value.
 */
unsigned long long get_ticks(void)
{
	return get_timer(0);
}

/*
 * This function is derived from PowerPC code (timebase clock frequency).
 * On ARM it returns the number of timer ticks per second.
 */
ulong get_tbclk (void)
{
	ulong tbclk;

	tbclk = CFG_HZ;

	return tbclk;
}


/* delay x useconds AND perserve advance timstamp value */
void udelay (unsigned long usec)
{
       ulong tmo, tmp;

 	ulong endtime = get_timer_masked() + usec;
	signed long diff;

	do {
		ulong now = get_timer_masked ();
		diff = endtime - now;
	} while (diff >= 0);

	return;
      if (usec >= 1000) {                     /* if "big" number, spread normalization to seconds */
               tmo = usec / 1000;              /* start to normalize for usec to ticks per sec */
               tmo *= CFG_HZ;                  /* find number of "ticks" to wait to achieve target */
               tmo /= 1000;                    /* finish normalize. */
       } else {                                        /* else small number, don't kill it prior to HZ multiply */
               tmo = usec * CFG_HZ;
               tmo /= (1000*1000);
       }

       tmp = get_timer (0);            /* get current timestamp */
       if ( (tmo + tmp + 1) < tmp )/* if setting this forward will roll time stamp */
               reset_timer_masked ();  /* reset "advancing" timestamp to 0, set lastinc value */
       else
               tmo     += tmp;                         /* else, set advancing stamp wake up time */
       while (get_timer_masked () < tmo)/* loop till event */
               /*NOP*/;
}

/*
 * Reset the cpu by setting up the watchdog timer and let it time out
 */
void reset_cpu (ulong ignored)
{
	/* Disable watchdog and set Time-Out field to 0 */
	WCR = 0x00000000;

	/* Write Service Sequence */
	WSR = 0x00005555;
	WSR = 0x0000AAAA;

	/* Enable watchdog */
	WCR = WCR_WDE;

	while (1);
	/*NOTREACHED*/
}
