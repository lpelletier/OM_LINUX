#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/delay.h>

#include <mach/ulpi.h>

/* ULPIVIEW register bits */
#define ULPIVW_WU		(1 << 31)	/* Wakeup */
#define ULPIVW_RUN		(1 << 30)	/* read/write run */
#define ULPIVW_WRITE		(1 << 29)	/* 0=read  1=write */
#define ULPIVW_SS		(1 << 27)	/* SyncState */
#define ULPIVW_PORT_MASK	0x07	/* Port field */
#define ULPIVW_PORT_SHIFT	24
#define ULPIVW_ADDR_MASK	0xFF	/* data address field */
#define ULPIVW_ADDR_SHIFT	16
#define ULPIVW_RDATA_MASK	0xFF	/* read data field */
#define ULPIVW_RDATA_SHIFT	8
#define ULPIVW_WDATA_MASK	0xFF	/* write data field */
#define ULPIVW_WDATA_SHIFT	0

static int ulpi_poll(void __iomem *view, uint32_t bit)
{
	uint32_t data;
	int timeout = 10000;

	data = __raw_readl(view);
	while (data & bit) {
		if (!timeout--) {
			printk("%s: timeout\n", __func__);
			return -1;
		}

		udelay(1);
		data = __raw_readl(view);
	}
	return (data >> ULPIVW_RDATA_SHIFT) & ULPIVW_RDATA_MASK;
}

int ulpi_read(int reg, void __iomem *view)
{
	uint32_t data;

	/* make sure interface is running */
	if (!(__raw_readl(view) && ULPIVW_SS)) {
		__raw_writel(ULPIVW_WU, view);

		/* wait for wakeup */
		if (ulpi_poll(view, ULPIVW_WU) == -1)
			return -1;
	}

	/* read the register */
	__raw_writel((ULPIVW_RUN | (reg << ULPIVW_ADDR_SHIFT)), view);

	/* wait for completion */
	data = ulpi_poll(view, ULPIVW_RUN);
	if (data == -1)
		return -1;

	return data;
}

int ulpi_set(u8 bits, int reg, void __iomem *view)
{
	/* make sure interface is running */
	if (!(__raw_readl(view) && ULPIVW_SS)) {
		__raw_writel(ULPIVW_WU, view);
		/* wait for wakeup */
		if (ulpi_poll(view, ULPIVW_WU) == -1)
			return -1;
	}

	__raw_writel((ULPIVW_RUN | ULPIVW_WRITE |
		      ((reg + ISP1504_REG_SET) << ULPIVW_ADDR_SHIFT) |
		      ((bits & ULPIVW_WDATA_MASK) << ULPIVW_WDATA_SHIFT)),
		     view);

	/* wait for completion */
	return ulpi_poll(view, ULPIVW_RUN) == -1 ? -1 : 0;
}

int ulpi_clear(u8 bits, int reg, void __iomem *view)
{
	__raw_writel((ULPIVW_RUN | ULPIVW_WRITE |
		      ((reg + ISP1504_REG_CLEAR) << ULPIVW_ADDR_SHIFT) |
		      ((bits & ULPIVW_WDATA_MASK) << ULPIVW_WDATA_SHIFT)),
		     view);

	/* wait for completion */
	return ulpi_poll(view, ULPIVW_RUN) == -1 ? -1 : 0;
}

