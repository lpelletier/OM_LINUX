/*
 * Copyright 2008 Juergen Beisert, kernel@pengutronix.de
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/platform_device.h>
#include <mach/imx_dam.h>
#include <asm/io.h>

#define DRV_NAME "mxc-dam"

struct dam_data {
	void __iomem *io;
	struct platform_device *pdev;
};

/*
 * Table of internal DAM port mappings to their offchip
 * SSI pin groups. This is verified for the i.MX27 CPU.
 */
static const int DAM_routing[] = {
	3,	/* SSI1_* pin group is port 4 at the DAM unit */
	4,	/* SSI2_* pin group is port 5 at the DAM unit */
	5,	/* SSI3_* pin group is port 6 at the DAM unit */
	2	/* SSI4_* pin group is port 3 at the DAM unit */
};

/* there is only one unit in this CPU */
static struct dam_data dam;

/* this unit has hardly predictable register offsets */
static const unsigned int reg_offsets[] = {
	DAM_HPCR1, DAM_HPCR2, DAM_HPCR3, DAM_PPCR1, DAM_PPCR2, DAM_PPCR3
};

#define    _reg_DAM_HPCR(a)	reg_offsets[a]	/* 0..1 */
#define    _reg_DAM_PPCR(a)	reg_offsets[a]	/* 2..5 */

/*
 * Route an internal SSI channel (1 or 2) as a slave to two
 * of the four SSI pin groups (1 to 4).
 * Note: 6-wire mode and i2s data only. This means no network
 * feature on this bus.
 *
 * This means:
 *
 *  SSI unit       DAM@SSI         DAM@per       SSI_pin
 * ---------------------------------------------------------------------------
 *    SRCK(in)    SRCK(out)                      SSI?_CLK (2)
 *    SRFS(in)    SRFS(out)                      SSI?_FS (2)
 *    SRXD(in)                      Da           SSI?_RXD (2)
 *    STCK(in)    TCLK(out)         CLK          SSI?_CLK (1)
 *    STFS(in)    TFS(out)          FS           SSI?_FS (1)
 *    STXD(out)                     Db           SSI?_TXD (1)
 *
 * -> ssi_pin1 means the output channel, ssi_pin2 means the input channel.
 * -> RXD from ssi_pin1 is not used, TXD from ssi_pin2 is not used.
s */

/*
 * Route an internal SSI channel (1 or 2) as a slave to one
 * of the four SSI pin groups (1 to 4).
 * Note: 4-wire mode and i2s data only. This means no network
 * feature on this bus.
 *
 * This means:
 *
 *  SSI unit      DAM@SSI          DAM@per        SSI_pin
 * ---------------------------------------------------------------------------
 *    SRCK (in)
 *    SRFS (in)
 *    SRXD (in)                      Da           SSI?_RXD
 *    STCK (in)    TCLK(out)         CLK          SSI?_CLK
 *    STFS (in)    TFS(out)          FS           SSI?_FS
 *    STXD (out)                     Db           SSI?_TXD
 */
int mx2_dam_configure_sync_slave(int ssi_unit, int ssi_pin)
{
	int host_port, target_port;
	u32 host_val, target_val;

	if (ssi_unit == 0 || ssi_pin == 0) {
		dev_err(&dam.pdev->dev, "DAM: Source or target port not defined\n");
		return -ENODEV;
	}

	host_port = ssi_unit - 1;
	target_port = DAM_routing[ssi_pin - 1];

	pr_debug("DAM: Connecting SSI%d's port %d to peripheral port %d\n",
		ssi_unit, ssi_unit, target_port + 1);

	/* starting to configure the host port */
	host_val = DAM_HPCR_SYN;	/* 4wire mode */
	/* where the RxD signal comes from */
	host_val |= DAM_HPCR_RXDSEL(target_port);
	/*
	 * SSI@STFS(in) <-- DAM@TFS(out)
	 * SSI@STCK(in) <-- DAM@TCLK(out)
	 */
	host_val |= DAM_HPCR_TFSDIR;	/* TFS is output at this port */
	host_val |= DAM_HPCR_TCLKDIR;	/* TCLK is output at this port */
	/*
	 * DAM@TFS(out) select from which other port and route to port's TFS (not RFS)
	 * DAM@TCLK(out) select from which other port and route to port's TCLK (not TCLK)
	 */
	host_val |= DAM_HPCR_TFCSEL(target_port);

	/* and now configure the port to the peripheral */
	target_val = DAM_PPCR_SYN;	/* 4wire mode */
	/* where the RxD signal comes from */
	target_val |= DAM_PPCR_RXDSEL(host_port);

	iowrite32(target_val, dam.io + reg_offsets[target_port]);
	iowrite32(host_val, dam.io + reg_offsets[host_port]);

	pr_debug("H1: %08X, H2: %08X, S1: %08X, S2: %08X, S3: %08X, S4: %08X\n",
		ioread32(dam.io + _reg_DAM_HPCR(0)),
		ioread32(dam.io + _reg_DAM_HPCR(1)),
		ioread32(dam.io + _reg_DAM_PPCR(2)),
		ioread32(dam.io + _reg_DAM_PPCR(3)),
		ioread32(dam.io + _reg_DAM_PPCR(4)),
		ioread32(dam.io + _reg_DAM_PPCR(5)));
	return 0;
}
EXPORT_SYMBOL(mx2_dam_configure_sync_slave);

static int __devinit mx2_dam_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret;

	pr_info("Probing i.MX DAM unit\n");

	dam.pdev = pdev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "No resource data for IMX DAM\n");
		return -ENODEV;
	}

	if (!request_mem_region(res->start, res->end - res->start + 1, DRV_NAME)) {
		dev_err(&pdev->dev, "request_mem_region failed for IMX DAM\n");
		return -EBUSY;
	}

	dam.io = ioremap(res->start, res->end - res->start + 1);
	if (dam.io == NULL) {
		dev_err(&pdev->dev, "Mapping region failed for IMX DAM\n");
		ret = -ENODEV;
		goto err1;
	}

	return 0;	/* return happy */

err1:
	release_mem_region(res->start, res->end - res->start + 1);
	return ret;
}

static int __devexit mx2_dam_remove(struct platform_device *pdev)
{
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, res->end - res->start + 1);
	iounmap(dam.io);
	dam.io = NULL;

	return 0;
}

static struct platform_driver mx2_dam_driver = {
	.driver = {
		.name		= DRV_NAME,
		.owner		= THIS_MODULE,
	},
	.probe		= mx2_dam_probe,
	.remove		= __devexit_p(mx2_dam_remove),
};

int __init mx2_dam_init(void)
{
	return platform_driver_register(&mx2_dam_driver);
}

void __exit mx2_dam_exit(void)
{
	platform_driver_unregister(&mx2_dam_driver);
}
