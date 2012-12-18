/*
 * Eric Jarrige <jorasse@users.sourceforge.net>
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 *
 */

#include <common.h>

#ifdef CONFIG_HARD_I2C

#include <asm/io.h>
#include <asm/arch/imx-regs.h>
#include <i2c.h>

/*#define       DEBUG_I2C*/

extern void imx_gpio_mode (int gpio_mode);

/*-----------------------------------------------------------------------
 * Definitions
 */

#define I2C_ACK		0	/* level to ack a byte */
#define I2C_NOACK	1	/* level to noack a byte */


#ifdef DEBUG_I2C
#define PRINTD(fmt,args...)	do {	\
	DECLARE_GLOBAL_DATA_PTR;	\
	if (gd->have_console)		\
		printf (fmt ,##args);	\
	} while (0)
#else
#define PRINTD(fmt,args...)
#endif

/*-----------------------------------------------------------------------
 * Local functions
 */

/*-----------------------------------------------------------------------
 * START: High -> Low on SDA while SCL is High
 * after check for a bus free
 */
static void
imxi2c_send_start (void)
{
	while ((I2SR & I2SR_IBB)) ;
	I2CR |= I2CR_MSTA;
	I2SR &= ~I2SR_IIF;
}

/*-----------------------------------------------------------------------
 * STOP: Low -> High on SDA while SCL is High
 * after the end of previous transfer
 */
static void
imxi2c_send_stop (void)
{
	while (!(I2SR & I2SR_ICF)) ;
	I2CR &= ~I2CR_MSTA;
}

/*-----------------------------------------------------------------------
 * Send 8 bits and look for an acknowledgement.
 */
static int
imxi2c_write_byte (uchar data)
{
	while (!(I2SR & I2SR_ICF)) ;	/* Wait end of transfer */

	I2CR |= I2CR_MTX;
	I2SR &= ~I2SR_IIF;
	I2DR = data;

	while (!(I2SR & I2SR_IIF)) ;	/* checking IIF before ICF seems required */

	I2SR &= ~I2SR_IIF;

	while (!(I2SR & I2SR_ICF)) ;	/* Wait end of transfer */

	return (I2SR & I2SR_RXAK);	/* not a nack is an ack */
}

/*-----------------------------------------------------------------------
 * if ack == I2C_ACK, ACK the byte so can continue reading, else
 * send I2C_NOACK to end the read.
 */
static uchar
imxi2c_read_byte (int ack)
{
	int data;

	while (!(I2SR & I2SR_ICF)) ;
	I2CR &= ~I2CR_MTX;

	if (ack)
	{
		I2CR |= I2CR_TXAK;
	}
	else
	{
		I2CR &= ~I2CR_TXAK;
	}

	data = I2DR;
	return (data);
}

/* ------------------------------------------------------------------------
 * API Functions
 * ------------------------------------------------------------------------
 */

/*-----------------------------------------------------------------------
 * i2c_init compute the i2c divider to reach the requested speed
 * see mxl reference manual
 */
void
i2c_init (int speed, int slaveaddr)
{
	int hclk_dividers[] = {
		30, 32, 36, 42, 48, 52, 60, 72,
		80, 88, 104, 128, 144, 160, 192, 240,
		288, 320, 384, 480, 576, 640, 768, 960,
		1152, 1280, 1536, 1920, 2304, 2560, 3072, 3840,
		22, 24, 26, 26, 32, 36, 40, 44,
		48, 56, 64, 72, 80, 96, 112, 128,
		160, 192, 224, 256, 320, 384, 448, 512,
		640, 768, 896, 1024, 1280, 1536, 1792, 2048
	};
	int refDiv = get_HCLK () / speed;
	int i, tmpIC;

	imx_gpio_mode (PA15_PF_I2C_SDA);
	imx_gpio_mode (PA16_PF_I2C_SCL);

	tmpIC = (sizeof (hclk_dividers) / sizeof (int)) - 1;
	for (i = tmpIC; i >= 0; i--)
	{
		if ((hclk_dividers[i] >= refDiv)
		    && (hclk_dividers[i] < hclk_dividers[tmpIC]))
		{
			tmpIC = i;
		}
	}
	
	IFDR = tmpIC;
	IADR = slaveaddr << 1;

	if (I2SR & I2SR_IBB)
		imxi2c_send_stop ();

	I2CR |= I2CR_IEN;
}

/*-----------------------------------------------------------------------
 * Probe to see if a chip is present. Also good for checking for the
 * completion of EEPROM writes since the chip stops responding until
 * the write completes (typically 10mSec).
 * probe sends a read command to probe a an address
 */
int
i2c_probe (uchar addr)
{
	int rc;

	imxi2c_send_start ();
	rc = imxi2c_write_byte ((addr << 1) | 0);
	imxi2c_send_stop ();

	return (rc ? 1 : 0);
}

/*-----------------------------------------------------------------------
 * Read bytes
 */
int
i2c_read (uchar chip, uint addr, int alen, uchar * buffer, int len)
{
	int shift;
	PRINTD ("i2c_read: chip %02X addr %02X alen %d buffer %p len %d\n",
		chip, addr, alen, buffer, len);

#ifdef CFG_I2C_EEPROM_ADDR_OVERFLOW
	/*
	 * EEPROM chips that implement "address overflow" are ones
	 * like Catalyst 24WC04/08/16 which has 9/10/11 bits of
	 * address and the extra bits end up in the "chip address"
	 * bit slots. This makes a 24WC08 (1Kbyte) chip look like
	 * four 256 byte chips.
	 *
	 * Note that we consider the length of the address field to
	 * still be one byte because the extra address bits are
	 * hidden in the chip address.
	 */
	chip |= ((addr >> (alen * 8)) & CFG_I2C_EEPROM_ADDR_OVERFLOW);

	PRINTD ("i2c_read: fix addr_overflow: chip %02X addr %02X\n",
		chip, addr);
#endif

	/*
	 * Do the addressing portion of a write cycle to set the
	 * chip's address pointer. If the address length is zero,
	 * don't do the normal write cycle to set the address pointer,
	 * there is no address pointer in this chip.
	 */
	imxi2c_send_start ();
	if (alen > 0)
	{
		if (imxi2c_write_byte (chip << 1))
		{		/* write cycle */
			imxi2c_send_stop ();
			PRINTD ("i2c_read, no chip responded %02X\n", chip);
			return (1);
		}
		shift = (alen - 1) * 8;
		while (alen-- > 0)
		{
			if (imxi2c_write_byte (addr >> shift))
			{
				PRINTD ("i2c_read, address not <ACK>ed\n");
				return (1);
			}
			shift -= 8;
		}
		imxi2c_send_stop ();	/* reportedly some chips need a full stop */
		imxi2c_send_start ();
	}
	/*
	 * Send the chip address again, this time for a read cycle.
	 * Then read the data. On the last byte, we do a NACK instead
	 * of an ACK(len == 0) to terminate the read.
	 */
	imxi2c_write_byte ((chip << 1) | 1);	/* read cycle */
	imxi2c_read_byte (len <= 2);
	while (len-- > 1)
	{
		*buffer++ = imxi2c_read_byte (len == 1);
	}
	imxi2c_send_stop ();
	*buffer++ = imxi2c_read_byte (0);
	return (0);
}

/*-----------------------------------------------------------------------
 * Write bytes
 */
int
i2c_write (uchar chip, uint addr, int alen, uchar * buffer, int len)
{
	int shift, failures = 0;

	PRINTD ("i2c_write: chip %02X addr %02X alen %d buffer %p len %d\n",
		chip, addr, alen, buffer, len);

	imxi2c_send_start ();
	if (imxi2c_write_byte (chip << 1))
	{			/* write cycle */
		imxi2c_send_stop ();
		PRINTD ("i2c_write, no chip responded %02X\n", chip);
		return (1);
	}
	shift = (alen - 1) * 8;
	while (alen-- > 0)
	{
		if (imxi2c_write_byte (addr >> shift))
		{
			PRINTD ("i2c_write, address not <ACK>ed\n");
			return (1);
		}
		shift -= 8;
	}

	while (len-- > 0)
	{
		if (imxi2c_write_byte (*buffer++))
		{
			failures++;
		}
	}
	imxi2c_send_stop ();
	return (failures);
}

/*-----------------------------------------------------------------------
 * Read a register
 */
uchar
i2c_reg_read (uchar i2c_addr, uchar reg)
{
	char buf;

	i2c_read (i2c_addr, reg, 1, &buf, 1);

	return (buf);
}

/*-----------------------------------------------------------------------
 * Write a register
 */
void
i2c_reg_write (uchar i2c_addr, uchar reg, uchar val)
{
	i2c_write (i2c_addr, reg, 1, &val, 1);
}

#endif /* CONFIG_HARD_I2C */
