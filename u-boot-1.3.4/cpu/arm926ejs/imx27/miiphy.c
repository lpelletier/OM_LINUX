/*
 * miiphy.c - generic phy abstraction
 *
 * (C) Copyright 2008 Eric Jarrige <eric.jarrige@armadeus.org>
 * Copyright (c) 2007 Sascha Hauer <s.hauer@pengutronix.de>, Pengutronix
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <common.h>
#include "miiphy.h" 

int miiphy_restart_aneg(struct miiphy_device *mdev)
{
	uint16_t status;
	int timeout;

	mdev->read(mdev, mdev->address, MII_SPECIAL, &status);
	if( status != 0xff){
		/* 
		 * Wake up from sleep if necessary
		 * Reset PHY, then delay 300ns
		 */
		mdev->write(mdev, mdev->address, MII_SPECIAL, 0x00FF);
		mdev->write(mdev, mdev->address, MII_BMCR, BMCR_RESET);
		udelay(1000);

		if (mdev->flags & MIIPHY_FORCE_10) {
			printf("Forcing 10 Mbps ethernet link... ");
			mdev->read(mdev, mdev->address, MII_BMSR, &status);
			mdev->write(mdev, mdev->address, MII_BMCR, BMCR_FULLDPLX | BMCR_CTST);

			timeout = 20;
			do {	/* wait for link status to go down */
				udelay(10000);
				if ((timeout--) == 0) {
	#if (DEBUG & 0x2)
					printf("hmmm, should not have waited...");
	#endif
					break;
				}
				mdev->read(mdev, mdev->address, MII_BMSR, &status);
			} while (status & BMSR_LSTATUS);
	
		} else {	/* MII100 */
			/*
			 * Set the auto-negotiation advertisement register bits
			 */
			mdev->write(mdev, mdev->address, MII_ADVERTISE, ADVERTISE_ALL);
			mdev->write(mdev, mdev->address, MII_BMCR, BMCR_ANENABLE | BMCR_ANRESTART);
		}
	}
	return 0;
}

int miiphy_wait_aneg(struct miiphy_device *mdev)
{
	uint32_t start;
	uint16_t status;

	/*
	 * Wait for AN completion
	 */
	start = get_timer_masked(); /* get_time_ns(); */
	do {
		if (get_timer (start) > (CFG_HZ * 5)) {
			printf("%s: Autonegotiation timeout\n", mdev->edev->name);
			return -1;
		}

		if (mdev->read(mdev, mdev->address, MII_BMSR, &status)) {
			printf("%s: Autonegotiation failed. status: 0x%04x\n", mdev->edev->name, status);
			return -1;
		}
	} while (!(status & BMSR_LSTATUS));

	return 0;
}

int miiphy_print_status(struct miiphy_device *mdev)
{
	uint16_t bmsr, bmcr, lpa;
	char *duplex;
	int speed;

	if (mdev->read(mdev, mdev->address, MII_BMSR, &bmsr) != 0)
		goto err_out;
	if (mdev->read(mdev, mdev->address, MII_BMCR, &bmcr) != 0)
		goto err_out;
	if (mdev->read(mdev, mdev->address, MII_LPA, &lpa) != 0)
		goto err_out;

	printf("%s: Link is %s", mdev->edev->name,
			bmsr & BMSR_LSTATUS ? "up" : "down");

	if (bmcr & BMCR_ANENABLE) {
		duplex = lpa & LPA_DUPLEX ? "Full" : "Half";
		speed = lpa & LPA_100 ? 100 : 10;
	} else {
		duplex = bmcr & BMCR_FULLDPLX ? "Full" : "Half";
		speed = bmcr & BMCR_SPEED100 ? 100 : 10;
	}

	printf(" - %d/%s\n", speed, duplex);

	return 0;
err_out:
	printf("%s: failed to read\n", mdev->edev->name);
	return -1;
}


