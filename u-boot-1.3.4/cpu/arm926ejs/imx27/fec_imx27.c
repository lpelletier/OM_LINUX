/*
 * (C) Copyright 2008,2009 Eric Jarrige <eric.jarrige@armadeus.org>
 * (C) Copyright 2008 Armadeus Systems nc
 * (C) Copyright 2007 Pengutronix, Sascha Hauer <s.hauer@pengutronix.de>
 * (C) Copyright 2007 Pengutronix, Juergen Beisert <j.beisert@pengutronix.de>
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

/************************** TODO eth_register + cleanup gfec !! *****/


#include <common.h>
#include <malloc.h>
#include <net.h>
#include "miiphy.h"
#include "fec_imx27.h"

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/io.h>

DECLARE_GLOBAL_DATA_PTR;

#ifdef CONFIG_DRIVER_FEC_IMX27

#if !(defined(CONFIG_MII) || defined(CONFIG_CMD_MII))
#error "CONFIG_MII has to be defined!"
#endif

//#undef CONFIG_FEC_IMX27_DEBUG
#ifdef CONFIG_FEC_IMX27_DEBUG
#define	PRINTF(fmt,args...)	printf (fmt ,##args)
#else
#define PRINTF(fmt,args...)
#endif

static int fec_miiphy_read(struct miiphy_device *mdev, uint8_t phyAddr,
	uint8_t regAddr, uint16_t * retVal);
static int fec_miiphy_write(struct miiphy_device *mdev, uint8_t phyAddr,
	uint8_t regAddr, uint16_t data);

typedef struct {
	uint8_t data[1500];	/**< actual data */
	int length;		/**< actual length */
	int used;		/**< buffer in use or not */
	uint8_t head[16];	/**< MAC header(6 + 6 + 2) + 2(aligned) */
} NBUF;

fec_priv gfec=
{
	.eth       = (ethernet_regs *)IMX_FEC_BASE,
	.xcv_type  = MII100,
	.rbd_base  = NULL,
	.rbd_index = 0,
	.tbd_base  = NULL,
	.tbd_index = 0,
	.miiphy =
		{
			CONFIG_PHY_ADDR,
			fec_miiphy_read,
			fec_miiphy_write,
			0,
			NULL
		},
	.bd        = NULL,
};

/*
 * MII-interface related functions
 */
static int fec_miiphy_read(struct miiphy_device *mdev, uint8_t phyAddr,
	uint8_t regAddr, uint16_t * retVal)
{
	struct eth_device *edev = mdev->edev;
	fec_priv *fec = (fec_priv *)edev->priv;

	uint32_t reg;		/* convenient holder for the PHY register */
	uint32_t phy;		/* convenient holder for the PHY */
	uint32_t start;

	/*
	 * reading from any PHY's register is done by properly
	 * programming the FEC's MII data register.
	 */
	writel(FEC_IEVENT_MII, &fec->eth->ievent);
	reg = regAddr << FEC_MII_DATA_RA_SHIFT;
	phy = phyAddr << FEC_MII_DATA_PA_SHIFT;

	writel(FEC_MII_DATA_ST | FEC_MII_DATA_OP_RD | FEC_MII_DATA_TA | phy | reg, &fec->eth->mii_data);

	/*
	 * wait for the related interrupt
	 */
	start = get_timer_masked(); /* get_time_ns(); */
	while (!(readl(&fec->eth->ievent) & FEC_IEVENT_MII)) {
		if (get_timer (start) > (CFG_HZ /1000)  /* is_timeout(start, MSECOND)*/) {
			printf("Read MDIO failed...\n");
			return -1;
		}
	}

	/*
	 * clear mii interrupt bit
	 */
	writel(FEC_IEVENT_MII, &fec->eth->ievent);

	/*
	 * it's now safe to read the PHY's register
	 */
	*retVal = readl(&fec->eth->mii_data);
	PRINTF("fec_miiphy_read: phy: %02x reg:%02x val:%#x\n", phyAddr, regAddr, *retVal);
	return 0;
}

static int fec_miiphy_write(struct miiphy_device *mdev, uint8_t phyAddr,
	uint8_t regAddr, uint16_t data)
{
	struct eth_device *edev = mdev->edev;
	fec_priv *fec = (fec_priv *)edev->priv;

	uint32_t reg;		/* convenient holder for the PHY register */
	uint32_t phy;		/* convenient holder for the PHY */
	uint32_t start;

	reg = regAddr << FEC_MII_DATA_RA_SHIFT;
	phy = phyAddr << FEC_MII_DATA_PA_SHIFT;

	writel(FEC_MII_DATA_ST | FEC_MII_DATA_OP_WR |
		FEC_MII_DATA_TA | phy | reg | data, &fec->eth->mii_data);

	/*
	 * wait for the MII interrupt
	 */
	start = get_timer_masked(); /* get_time_ns(); */
	while (!(readl(&fec->eth->ievent) & FEC_IEVENT_MII)) {
		if (get_timer (start) > (CFG_HZ /1000)  /* is_timeout(start, MSECOND)*/) {
			printf("Write MDIO failed...\n");
			return -1;
		}
	}

	/*
	 * clear MII interrupt bit
	 */
	writel(FEC_IEVENT_MII, &fec->eth->ievent);
	PRINTF("fec_miiphy_write: phy: %02x reg:%02x val:%#x\n", phyAddr, regAddr, data);

	return 0;
}

static int fec_rx_task_enable(fec_priv *fec)
{
	writel(1 << 24, &fec->eth->r_des_active);
	return 0;
}

static int fec_rx_task_disable(fec_priv *fec)
{
	return 0;
}

static int fec_tx_task_enable(fec_priv *fec)
{
	writel(1 << 24, &fec->eth->x_des_active);
	return 0;
}

static int fec_tx_task_disable(fec_priv *fec)
{
	return 0;
}

/**
 * Initialize receive task's buffer descriptors
 * @param[in] fec all we know about the device yet
 * @param[in] count receive buffer count to be allocated
 * @param[in] size size of each receive buffer
 * @return 0 on success
 *
 * For this task we need additional memory for the data buffers. And each
 * data buffer requires some alignment. Thy must be aligned to a specific
 * boundary each (DB_DATA_ALIGNMENT).
 */
static int fec_rbd_init(fec_priv *fec, int count, int size, int once)
{
	int ix;
	uint32_t p=0;

	if (!once) {
		/* reserve data memory and consider alignment */
		p = (uint32_t)malloc(size * count + DB_DATA_ALIGNMENT);
		memset((void *)p, 0, size * count + DB_DATA_ALIGNMENT);
		p += DB_DATA_ALIGNMENT-1;
		p &= ~(DB_DATA_ALIGNMENT-1);
	}

	for (ix = 0; ix < count; ix++) {
		if (!once) {
			writel(p, &fec->rbd_base[ix].data_pointer);
			p += size;
		}
		writew(FEC_RBD_EMPTY, &fec->rbd_base[ix].status);
		writew(0, &fec->rbd_base[ix].data_length);
	}
	/*
	 * mark the last RBD to close the ring
	 */
	writew(FEC_RBD_WRAP | FEC_RBD_EMPTY, &fec->rbd_base[ix - 1].status);
	fec->rbd_index = 0;

	return 0;
}

/**
 * Initialize transmit task's buffer descriptors
 * @param[in] fec all we know about the device yet
 *
 * Transmit buffers are created externally. We only have to init the BDs here.\n
 * Note: There is a race condition in the hardware. When only one BD is in
 * use it must be marked with the WRAP bit to use it for every transmitt.
 * This bit in combination with the READY bit results into double transmit
 * of each data buffer. It seems the state machine checks READY earlier then
 * resetting it after the first transfer.
 * Using two BDs solves this issue.
 */
static void fec_tbd_init(fec_priv *fec)
{
	writew(0x0000, &fec->tbd_base[0].status);
	writew(FEC_TBD_WRAP, &fec->tbd_base[1].status);
	fec->tbd_index = 0;
}

/**
 * Mark the given read buffer descriptor as free
 * @param[in] last 1 if this is the last buffer descriptor in the chain, else 0
 * @param[in] pRbd buffer descriptor to mark free again
 */
static void fec_rbd_clean(int last, FEC_BD *pRbd)
{
	/*
	 * Reset buffer descriptor as empty
	 */
	if (last)
		writew(FEC_RBD_WRAP | FEC_RBD_EMPTY, &pRbd->status);
	else
		writew(FEC_RBD_EMPTY, &pRbd->status);
	/*
	 * no data in it
	 */
	writew(0, &pRbd->data_length);
}

static int fec_get_hwaddr(struct eth_device *dev, unsigned char *mac)
{
	int i;
	int uninitialized = 0;

	for (i=0;i<6;i++) {
		mac[6-1-i] = readl(&IIM_BANK_REG(0,(IIM0_MAC+i)));
	}

	/* uninitialized if all 00 */
	if ((mac[0] == 0) && (mac[1] == 0) && (mac[2] == 0) &&
            (mac[3] == 0) && (mac[4] == 0) && (mac[5] == 0))
                uninitialized = 1;

	/* uninitialized if all FF (could be safe) */
        if ((mac[0] == 0xff) && (mac[1] == 0xff) && (mac[2] == 0xff) &&
	    (mac[3] == 0xff) && (mac[4] == 0xff) && (mac[5] == 0xff))
	        uninitialized = 1;

	return uninitialized;
}

static int fec_set_hwaddr(struct eth_device *dev, unsigned char *mac)
{
	fec_priv *fec = (fec_priv *)dev->priv;
//#define WTF_IS_THIS
#ifdef WTF_IS_THIS
	uint32_t crc = 0xffffffff;	/* initial value */
	uint8_t currByte;			/* byte for which to compute the CRC */
	int byte;			/* loop - counter */
	int bit;			/* loop - counter */

	/*
	 * The algorithm used is the following:
	 * we loop on each of the six bytes of the provided address,
	 * and we compute the CRC by left-shifting the previous
	 * value by one position, so that each bit in the current
	 * byte of the address may contribute the calculation. If
	 * the latter and the MSB in the CRC are different, then
	 * the CRC value so computed is also ex-ored with the
	 * "polynomium generator". The current byte of the address
	 * is also shifted right by one bit at each iteration.
	 * This is because the CRC generatore in hardware is implemented
	 * as a shift-register with as many ex-ores as the radixes
	 * in the polynomium. This suggests that we represent the
	 * polynomiumm itself as a 32-bit constant.
	 */
	for (byte = 0; byte < 6; byte++) {
		currByte = mac[byte];
		for (bit = 0; bit < 8; bit++) {
			if ((currByte & 0x01) ^ (crc & 0x01)) {
				crc >>= 1;
				crc = crc ^ 0xedb88320;
			} else {
				crc >>= 1;
			}
			currByte >>= 1;
		}
	}

	crc = crc >> 26;

	/*
	 * Set individual hash table register
	 */
	if (crc >= 32) {
		fec->eth->iaddr1 = (1 << (crc - 32));
		fec->eth->iaddr2 = 0;
	} else {
		fec->eth->iaddr1 = 0;
		fec->eth->iaddr2 = (1 << crc);
	}
#else
	writel(0, &fec->eth->iaddr1);
	writel(0, &fec->eth->iaddr2);
	writel(0, &fec->eth->gaddr1);
	writel(0, &fec->eth->gaddr2);
#endif
	/*
	 * Set physical address
	 */
 	writel((mac[0] << 24) + (mac[1] << 16) + (mac[2] << 8) + mac[3], &fec->eth->paddr1);
	writel((mac[4] << 24) + (mac[5] << 16) + 0x8808, &fec->eth->paddr2);

        return 0;
}

/**
 * Start the FEC engine
 * @param[in] dev Our device to handle
 */
static int fec_open(struct eth_device *edev)
{
	fec_priv *fec = (fec_priv *)edev->priv;

	PRINTF("fec_open: fec_open(dev)\n");
	writel(1 << 2, &fec->eth->x_cntrl);	/* full-duplex, heartbeat disabled */
	fec->rbd_index = 0;

	/*
	 * Enable FEC-Lite controller
	 */
	writel(FEC_ECNTRL_ETHER_EN, &fec->eth->ecntrl);

	if (fec->xcv_type != SEVENWIRE) {
		miiphy_wait_aneg(&fec->miiphy);
		miiphy_print_status(&fec->miiphy);
	}

	/*
	 * Enable SmartDMA receive task
	 */
	fec_rx_task_enable(fec);

	udelay(100000);
	return 0;
}

static int fec_init(struct eth_device *dev, bd_t* bd)
{
	static int once = 0;
	uint32_t base;
	fec_priv *fec = (fec_priv *)dev->priv;

	if( !once )
	{
		/*
		 * reserve memory for both buffer descriptor chains at once
		 * Datasheet forces the startaddress of each chain is 16 byte aligned
		 */
		base = (uint32_t)malloc( (2 + FEC_RBD_NUM) * sizeof(FEC_BD) + DB_ALIGNMENT );
		memset((void *)base, 0, (2 + FEC_RBD_NUM) * sizeof(FEC_BD) + DB_ALIGNMENT);
		base += (DB_ALIGNMENT-1);
		base &= ~(DB_ALIGNMENT-1);

		fec->rbd_base = (FEC_BD*)base;

		base += FEC_RBD_NUM * sizeof(FEC_BD);

		fec->tbd_base = (FEC_BD*)base;
	}

	/*
	 * Set interrupt mask register
	 */
	writel(0x00000000, &fec->eth->imask);

	/*
	 * Clear FEC-Lite interrupt event register(IEVENT)
	 */
	writel(0xffffffff, &fec->eth->ievent);


	/*
	 * Set FEC-Lite receive control register(R_CNTRL):
	 */
	if (fec->xcv_type == SEVENWIRE) {
		/*
		 * Frame length=1518; 7-wire mode
		 */
		writel(0x05ee0020, &fec->eth->r_cntrl);	/* FIXME 0x05ee0000 */
	} else {
		/*
		 * Frame length=1518; MII mode;
		 */
		writel(0x05ee0024, &fec->eth->r_cntrl);	/* FIXME 0x05ee0004 */
		/*
		 * Set MII_SPEED = (1/(mii_speed * 2)) * System Clock
		 * and do not drop the Preamble.
		 */
		writel((((imx_get_ahbclk() /1000000)+2) / 5) << 1, &fec->eth->mii_speed);	/* No MII for 7-wire mode */
		PRINTF("fec_init: mii_speed %#lx\n", (((imx_get_ahbclk() /1000000)+2) / 5) << 1);
	}
	/*
	 * Set Opcode/Pause Duration Register
	 */
	writel(0x00010020, &fec->eth->op_pause);	/* FIXME 0xffff0020; */
	writel(0x2, &fec->eth->x_wmrk);
	/*
	 * Set multicast address filter
	 */
	writel(0x00000000, &fec->eth->gaddr1);
	writel(0x00000000, &fec->eth->gaddr2);


	/* clear MIB RAM */
	long* mib_ptr = (long*)(IMX_FEC_BASE + 0x200);
	while (mib_ptr <= (long*)(IMX_FEC_BASE + 0x2FC)) {
		*mib_ptr++ = 0;
	}

	/* FIFO receive start register */	
	writel(0x520, &fec->eth->r_fstart);

	/* size and address of each buffer */
	writel(FEC_MAX_PKT_SIZE, &fec->eth->emrbr);
    	writel((uint32_t)fec->tbd_base, &fec->eth->etdsr);
    	writel((uint32_t)fec->rbd_base, &fec->eth->erdsr);

	/*
	 * Initialize RxBD/TxBD rings
	 */
	fec_rbd_init(fec, FEC_RBD_NUM, FEC_MAX_PKT_SIZE, once);
	fec_tbd_init(fec);


	if (fec->xcv_type != SEVENWIRE)
		miiphy_restart_aneg(&fec->miiphy);

	once = 1;	/* malloc done now (and once) */

	fec_open(dev);
	return 0;
}

/**
 * Halt the FEC engine
 * @param[in] dev Our device to handle
 */
static void fec_halt(struct eth_device *dev)
{
	fec_priv *fec = &gfec;
	int counter = 0xffff;

	/*
	 * issue graceful stop command to the FEC transmitter if necessary
	 */
	writel(FEC_ECNTRL_RESET | readl(&fec->eth->x_cntrl), &fec->eth->x_cntrl);

	PRINTF("eth_halt: wait for stop regs\n");
	/*
	 * wait for graceful stop to register
	 */
	while ((counter--) && (!(readl(&fec->eth->ievent) & FEC_IEVENT_GRA)))
		;	/* FIXME ensure time */

	/*
	 * Disable SmartDMA tasks
	 */
	fec_tx_task_disable(fec);
	fec_rx_task_disable(fec);

	/*
	 * Disable the Ethernet Controller
	 * Note: this will also reset the BD index counter!
	 */
	writel(0, &fec->eth->ecntrl);
	fec->rbd_index = 0;
	fec->tbd_index = 0;
	PRINTF("eth_halt: done\n");
}

/**
 * Transmit one frame
 * @param[in] dev Our ethernet device to handle
 * @param[in] packet Pointer to the data to be transmitted
 * @param[in] length Data count in bytes
 * @return 0 on success
 */
static int fec_send(struct eth_device *dev, volatile void* packet, int length)
{
	unsigned int status;

	/*
	 * This routine transmits one frame.  This routine only accepts
	 * 6-byte Ethernet addresses.
	 */
	fec_priv *fec = (fec_priv *)dev->priv;

	/*
	 * Check for valid length of data.
	 */
	if ((length > 1500) || (length <= 0)) {
		printf("Payload (%d) to large!\n", length);
		return -1;
	}

	/*
	 * Setup the transmitt buffer
	 * Note: We are always using the first buffer for transmission,
	 * the second will be empty and only used to stop the DMA engine
	 */
/*	{
		int i;
		PRINTF("fec_send %d bytes:", length);
			for (i=0;i<length;i++)
				PRINTF(" %02x", ((char*)packet)[i]);
			PRINTF("\n");
	}
*/	writew(length, &fec->tbd_base[fec->tbd_index].data_length);
	writel((uint32_t)packet, &fec->tbd_base[fec->tbd_index].data_pointer);
	/*
	 * update BD's status now
	 * This block:
	 * - is always the last in a chain (means no chain)
	 * - should transmitt the CRC
	 * - might be the last BD in the list, so the address counter should
	 *   wrap (-> keep the WRAP flag)
	 */
	status = readw(&fec->tbd_base[fec->tbd_index].status) & FEC_TBD_WRAP;
	status |= FEC_TBD_LAST | FEC_TBD_TC | FEC_TBD_READY;
	writew(status, &fec->tbd_base[fec->tbd_index].status);

	/*
	 * Enable SmartDMA transmit task
	 */
	fec_tx_task_enable(fec);

	/*
	 * wait until frame is sent .
	 */
	while (readw(&fec->tbd_base[fec->tbd_index].status) & FEC_TBD_READY) {
		/* FIXME: Timeout */
	}
	PRINTF("fec_send: status 0x%x index %d\n", readw(&fec->tbd_base[fec->tbd_index].status), fec->tbd_index);
	/* for next transmission use the other buffer */
	if (fec->tbd_index)
		fec->tbd_index = 0;
	else
		fec->tbd_index = 1;

	return 0;
}

/**
 * Pull one frame from the card
 * @param[in] dev Our ethernet device to handle
 * @return Length of packet read
 */
static int fec_recv(struct eth_device *dev)
{
	fec_priv *fec = (fec_priv *)dev->priv;
	FEC_BD *rbd = &fec->rbd_base[fec->rbd_index];
	unsigned long ievent;
	int frame_length, len = 0;
	NBUF *frame;
	uint16_t bd_status;
	uchar buff[FEC_MAX_PKT_SIZE];

	/*
	 * Check if any critical events have happened
	 */
	ievent = readl(&fec->eth->ievent);
	writel(ievent, &fec->eth->ievent);
	PRINTF("fec_recv: ievent 0x%x\n", ievent );
	if (ievent & FEC_IEVENT_BABR) {
		fec_halt(dev);
		fec_init(dev, fec->bd);
		printf("some error: 0x%08lx\n", ievent);
		return 0;
	}
	if (ievent & FEC_IEVENT_HBERR) {
		/* Heartbeat error */
		writel(0x00000001 | readl(&fec->eth->x_cntrl), &fec->eth->x_cntrl);
	}
	if (ievent & FEC_IEVENT_GRA) {
		/* Graceful stop complete */
		if (readl(&fec->eth->x_cntrl) & 0x00000001) {
			fec_halt(dev);
			writel(~0x00000001 & readl(&fec->eth->x_cntrl), &fec->eth->x_cntrl);
			fec_init(dev, fec->bd);
		}
	}

	/*
	 * ensure reading the right buffer status
	 */
	bd_status = readw(&rbd->status);
	PRINTF("fec_recv: status 0x%x\n", bd_status );

	if (!(bd_status & FEC_RBD_EMPTY)) {
		if ((bd_status & FEC_RBD_LAST) && !(bd_status & FEC_RBD_ERR) &&
			((readw(&rbd->data_length) - 4) > 14)) {
			/*
			 * Get buffer address and size
			 */
			frame = (NBUF *)readl(&rbd->data_pointer);
			frame_length = readw(&rbd->data_length) - 4;
			/*
			 *  Fill the buffer and pass it to upper layers
			 */
			memcpy(buff, frame->data, frame_length);
/*			PRINTF("fec_recv %d bytes:", frame_length);
			for (len=0;len<frame_length;len++)
				PRINTF(" %02x", buff[len]);
			PRINTF("\n");
*/			NetReceive(buff, frame_length);
			len = frame_length;
		} else {
			if (bd_status & FEC_RBD_ERR) {
				printf("error frame: 0x%08lx 0x%08x\n", (ulong)rbd->data_pointer, bd_status);
			}
		}
		/*
		 * free the current buffer, restart the engine
		 * and move forward to the next buffer
		 */
		fec_rbd_clean(fec->rbd_index == (FEC_RBD_NUM - 1) ? 1 : 0, rbd);
		fec_rx_task_enable(fec);
		fec->rbd_index = (fec->rbd_index + 1) % FEC_RBD_NUM;
	}
	PRINTF("fec_recv: stop\n");

	return len;
}

static int fec_probe(bd_t * bd)
{
	/*struct fec_platform_data *pdata = (struct fec_platform_data *)dev->platform_data;*/
	struct eth_device *edev;
	fec_priv *fec = &gfec;
	unsigned char ethaddr_str[20];
	unsigned char ethaddr[6];
	char *tmp = getenv ("ethaddr");
	char *end;

	/* enable FEC clock */
	PCCR1 |= PCCR1_HCLK_FEC;
	PCCR0 |= PCCR0_FEC_EN;

	/*create and fill edev struct*/
	edev = (struct eth_device *)malloc(sizeof(struct eth_device));
	edev->priv = fec;
	edev->init = fec_init;
	edev->send = fec_send;
	edev->recv = fec_recv;
	edev->halt = fec_halt;

	fec->eth = (ethernet_regs *)IMX_FEC_BASE;
	fec->bd = bd;

	/* Reset chip. */
	writel(FEC_ECNTRL_RESET, &fec->eth->ecntrl);
	while(readl(&fec->eth->ecntrl) & 1) {
		udelay(10);
	}

	fec->xcv_type = MII100; /*pdata->xcv_type;*/

	sprintf(edev->name, "FEC ETHERNET");

	if (fec->xcv_type != SEVENWIRE) {
		fec->miiphy.read = fec_miiphy_read;
		fec->miiphy.write = fec_miiphy_write;
		fec->miiphy.address = CONFIG_PHY_ADDR;
		fec->miiphy.flags = fec->xcv_type == MII10 ? MIIPHY_FORCE_10 : 0;
		fec->miiphy.edev = edev;

		/* if multiple PHY have to be supported */
		/*miiphy_register (edev_>name, fec_miiphy_read, fec_miiphy_write);*/
	}

	//eth_register(edev);

	if (( NULL != tmp ) && (12 <= strlen(tmp))) {
		int i;
		/* convert MAC from string to int */
		for (i=0; i<6; i++) {
			ethaddr[i] = tmp ? simple_strtoul(tmp, &end, 16) : 0;
			if (tmp)
				tmp = (*end) ? end+1 : end;
		}
	}
	else if (fec_get_hwaddr(edev, ethaddr) == 0) {
		sprintf ((char*)ethaddr_str, "%02X:%02X:%02X:%02X:%02X:%02X",
			ethaddr[0], ethaddr[1], ethaddr[2], ethaddr[3],
			ethaddr[4], ethaddr[5]);
		printf("got MAC address from EEPROM: %s\n",ethaddr_str);
		setenv ("ethaddr", (char*)ethaddr_str);
	}
	memcpy(edev->enetaddr, ethaddr, 6);
	fec_set_hwaddr(edev, ethaddr);

	return 0;
}

static int once = 0;

int eth_init(bd_t * bd)
{

	if (!once)
	{
		PRINTF("eth_init: fec_probe(bd)\n");
		fec_probe(bd);	
		once = 1;
	}	
	PRINTF("eth_init: fec_init(gfec.miiphy.edev, bd)\n");
	return fec_init(gfec.miiphy.edev, bd);
};

int fec_eth_initialize(bd_t *bd)
{
int lout=1;

	if (!once)
	{
		PRINTF("eth_init: fec_probe(bd)\n");
		lout = fec_probe(bd);	
		once = 1;
	}
	return lout;	
}

int eth_send(volatile void *packet, int length)
{
	PRINTF("eth_send: fec_send(gfec.miiphy.edev, packet 0x%08lx, length)\n", packet);
	return fec_send(gfec.miiphy.edev, packet, length);
};

int eth_rx(void){
	PRINTF("eth_rcv: fec_rcv(gfec.miiphy.edev)\n");
	return fec_recv(gfec.miiphy.edev);
};


void eth_halt(void)
{
	PRINTF("eth_halt: fec_halt(gfec)\n");
	fec_halt(NULL);
	return;
};

/**
 * @file
 * @brief Network driver for FreeScale's FEC implementation.
 * This type of hardware can be found on i.MX27 CPUs
 */

#endif /* CONFIG_DRIVER_FEC_IMX27 */

