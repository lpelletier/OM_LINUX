/*
 *  linux/arch/arm/mach-imx/dma.c
 *
 *  imx DMA registration and IRQ dispatching
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  2004-03-03 Sascha Hauer <sascha@saschahauer.de>
 *             initial version heavily inspired by
 *             linux/arch/arm/mach-pxa/dma.c
 *
 *  2005-04-17 Pavel Pisa <pisa@cmp.felk.cvut.cz>
 *             Changed to support scatter gather DMA
 *             by taking Russell's code from RiscPC
 *
 *  2006-05-31 Pavel Pisa <pisa@cmp.felk.cvut.cz>
 *             Corrected error handling code.
 *
 */

#undef DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/scatterlist.h>

#include <asm/scatterlist.h>
#include <asm/system.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <mach/dma.h>
#include <mach/imx-dma.h>

struct imx_dma_channel imx_dma_channels[IMX_DMA_CHANNELS];

static int imx_dma_hw_chain(struct imx_dma_channel *imxdma)
{
#ifdef CONFIG_ARCH_MX2
	if (cpu_is_mx27())
		return imxdma->hw_chaining;
	else
#endif
		return 0;
}

static void dump_channels(void)
{
	int i = 0;

	printk("Channel | Owner\n");
	for (i=0; i<IMX_DMA_CHANNELS; i++) {
		printk("  %2d    | %s\n", i, imx_dma_channels[i].name);
	}
}

/*
 * imx_dma_sg_next - prepare next chunk for scatter-gather DMA emulation
 * @dma_ch: i.MX DMA channel number
 * @lastcount: number of bytes transferred during last transfer
 *
 * Functions prepares DMA controller for next sg data chunk transfer.
 * The @lastcount argument informs function about number of bytes transferred
 * during last block. Zero value can be used for @lastcount to setup DMA
 * for the first chunk.
 */
static inline int imx_dma_sg_next(imx_dmach_t channel, struct scatterlist *sg)
{
	struct imx_dma_channel *imxdma = &imx_dma_channels[channel];
	unsigned long now;

	if (!imxdma->name) {
		printk(KERN_CRIT "%s: called for  not allocated channel %d\n",
		       __func__, channel);
		return 0;
	}

	now = min(imxdma->resbytes, sg->length);
	imxdma->resbytes -= now;

	if ((imxdma->dma_mode & DMA_MODE_MASK) == DMA_MODE_READ)
		DAR(channel) = sg->dma_address;
	else
		SAR(channel) = sg->dma_address;

	CNTR(channel) = now
;
	pr_debug("imxdma%d: next sg chunk dst 0x%08x, src 0x%08x, size 0x%08x\n",
		 channel, DAR(channel), SAR(channel), CNTR(channel));

	return now;
}

/*
 * imx_dma_setup_sg_base - scatter-gather DMA emulation
 * @dma_ch: i.MX DMA channel number
 * @sg: pointer to the scatter-gather list/vector
 * @sgcount: scatter-gather list hungs count
 *
 * Functions sets up i.MX DMA state for emulated scatter-gather transfer
 * and sets up channel registers to be ready for the first chunk
 */
static int
imx_dma_setup_sg_base(imx_dmach_t dma_ch,
		      struct scatterlist *sg, unsigned int sgcount)
{
	struct imx_dma_channel *imxdma = &imx_dma_channels[dma_ch];

	imxdma->sg = sg;
	imxdma->sgcount = sgcount;
	imxdma->sgbc = 0;
	return imx_dma_sg_next(dma_ch, 0);
}

/**
 * imx_dma_setup_single - setup i.MX DMA channel for linear memory to/from device transfer
 * @dma_ch: i.MX DMA channel number
 * @dma_address: the DMA/physical memory address of the linear data block
 *		to transfer
 * @dma_length: length of the data block in bytes
 * @dev_addr: physical device port address
 * @dmamode: DMA transfer mode, %DMA_MODE_READ from the device to the memory
 *           or %DMA_MODE_WRITE from memory to the device
 *
 * The function setups DMA channel source and destination addresses for transfer
 * specified by provided parameters. The scatter-gather emulation is disabled,
 * because linear data block
 * form the physical address range is transferred.
 * Return value: if incorrect parameters are provided -%EINVAL.
 *		Zero indicates success.
 */
int
imx_dma_setup_single(imx_dmach_t dma_ch, dma_addr_t dma_address,
		     unsigned int dma_length, unsigned int dev_addr,
		     unsigned int dmamode)
{
	struct imx_dma_channel *imxdma = &imx_dma_channels[dma_ch];

	imxdma->sg = NULL;
	imxdma->sgcount = 0;
	imxdma->dma_mode = dmamode;

	if (!dma_address) {
		printk(KERN_ERR "imxdma%d: imx_dma_setup_single null address\n",
		       dma_ch);
		return -EINVAL;
	}

	if (!dma_length) {
		printk(KERN_ERR "imxdma%d: imx_dma_setup_single zero length\n",
		       dma_ch);
		return -EINVAL;
	}

	if ((dmamode & DMA_MODE_MASK) == DMA_MODE_READ) {
		pr_debug("imxdma%d: mx_dma_setup_single2dev dma_addressg=0x%08x dma_length=%d dev_addr=0x%08x for read\n",
			dma_ch, (unsigned int)dma_address, dma_length,
			dev_addr);
		SAR(dma_ch) = dev_addr;
		DAR(dma_ch) = (unsigned int)dma_address;
	} else if ((dmamode & DMA_MODE_MASK) == DMA_MODE_WRITE) {
		pr_debug("imxdma%d: mx_dma_setup_single2dev dma_addressg=0x%08x dma_length=%d dev_addr=0x%08x for write\n",
			dma_ch, (unsigned int)dma_address, dma_length,
			dev_addr);
		SAR(dma_ch) = (unsigned int)dma_address;
		DAR(dma_ch) = dev_addr;
	} else {
		printk(KERN_ERR "imxdma%d: imx_dma_setup_single bad dmamode\n",
		       dma_ch);
		return -EINVAL;
	}

	CNTR(dma_ch) = dma_length;

	return 0;
}

/**
 * imx_dma_setup_sg - setup i.MX DMA channel SG list to/from device transfer
 * @dma_ch: i.MX DMA channel number
 * @sg: pointer to the scatter-gather list/vector
 * @sgcount: scatter-gather list hungs count
 * @dma_length: total length of the transfer request in bytes
 * @dev_addr: physical device port address
 * @dmamode: DMA transfer mode, %DMA_MODE_READ from the device to the memory
 *           or %DMA_MODE_WRITE from memory to the device
 *
 * The function sets up DMA channel state and registers to be ready for transfer
 * specified by provided parameters. The scatter-gather emulation is set up
 * according to the parameters.
 *
 * The full preparation of the transfer requires setup of more register
 * by the caller before imx_dma_enable() can be called.
 *
 * %BLR(dma_ch) holds transfer burst length in bytes, 0 means 64 bytes
 *
 * %RSSR(dma_ch) has to be set to the DMA request line source %DMA_REQ_xxx
 *
 * %CCR(dma_ch) has to specify transfer parameters, the next settings is typical
 * for linear or simple scatter-gather transfers if %DMA_MODE_READ is specified
 *
 * %CCR_DMOD_LINEAR | %CCR_DSIZ_32 | %CCR_SMOD_FIFO | %CCR_SSIZ_x
 *
 * The typical setup for %DMA_MODE_WRITE is specified by next options combination
 *
 * %CCR_SMOD_LINEAR | %CCR_SSIZ_32 | %CCR_DMOD_FIFO | %CCR_DSIZ_x
 *
 * Be careful here and do not mistakenly mix source and target device
 * port sizes constants, they are really different:
 * %CCR_SSIZ_8, %CCR_SSIZ_16, %CCR_SSIZ_32,
 * %CCR_DSIZ_8, %CCR_DSIZ_16, %CCR_DSIZ_32
 *
 * Return value: if incorrect parameters are provided -%EINVAL.
 * Zero indicates success.
 */
int
imx_dma_setup_sg(imx_dmach_t dma_ch,
		 struct scatterlist *sg, unsigned int sgcount, unsigned int dma_length,
		 unsigned int dev_addr, unsigned int dmamode)
{
	int res;
	struct imx_dma_channel *imxdma = &imx_dma_channels[dma_ch];

	if (imxdma->in_use)
		return -EBUSY;

	imxdma->sg = sg;
	imxdma->dma_mode = dmamode;
	imxdma->resbytes = dma_length;

	if (!sg || !sgcount) {
		printk(KERN_ERR "imxdma%d: imx_dma_setup_sg epty sg list\n",
		       dma_ch);
		return -EINVAL;
	}

	if (!sg->length) {
		printk(KERN_ERR "imxdma%d: imx_dma_setup_sg zero length\n",
		       dma_ch);
		return -EINVAL;
	}

	if ((dmamode & DMA_MODE_MASK) == DMA_MODE_READ) {
		pr_debug("imxdma%d: mx_dma_setup_sg2dev sg=%p sgcount=%d total length=%d dev_addr=0x%08x for read\n",
			dma_ch, sg, sgcount, dma_length, dev_addr);
		SAR(dma_ch) = dev_addr;
	} else if ((dmamode & DMA_MODE_MASK) == DMA_MODE_WRITE) {
		pr_debug("imxdma%d: mx_dma_setup_sg2dev sg=%p sgcount=%d total length=%d dev_addr=0x%08x for write\n",
			dma_ch, sg, sgcount, dma_length, dev_addr);
		DAR(dma_ch) = dev_addr;
	} else {
		printk(KERN_ERR "imxdma%d: imx_dma_setup_sg bad dmamode\n",
		       dma_ch);
		return -EINVAL;
	}

	imx_dma_sg_next(dma_ch, sg);

	return 0;
}

int
imx_dma_config_channel(int channel, unsigned int config_dest,
	unsigned int config_src, unsigned int dmareq, int hw_chaining)
{
	struct imx_dma_channel *imxdma = &imx_dma_channels[channel];
	u32 dreq = 0;

	imxdma->hw_chaining = 0;

// 	if (hw_chaining) {
// 		imxdma->hw_chaining = 1;
// 		if (!imx_dma_hw_chain(imxdma))
// 			return -EINVAL;
// 	}

	if (dmareq)
		dreq = CCR_REN;

	CCR(channel) = config_src | (config_dest << 2) | dreq;
// 	imxdma->ccr_to_device = config_mem | (config_port << 2) | dreq;

	RSSR(channel) = dmareq;

	return 0;
}
EXPORT_SYMBOL(imx_dma_config_channel);

void imx_dma_config_burstlen(int channel, unsigned int burstlen)
{
	BLR(channel) = burstlen;
}
EXPORT_SYMBOL(imx_dma_config_burstlen);

/**
 * imx_dma_setup_handlers - setup i.MX DMA channel end and error notification handlers
 * @dma_ch: i.MX DMA channel number
 * @irq_handler: the pointer to the function called if the transfer
 *		ends successfully
 * @err_handler: the pointer to the function called if the premature
 *		end caused by error occurs
 * @data: user specified value to be passed to the handlers
 */
int
imx_dma_setup_handlers(imx_dmach_t dma_ch,
		       void (*irq_handler) (int, void *),
		       void (*err_handler) (int, void *, int),
		       void *data)
{
	struct imx_dma_channel *imxdma = &imx_dma_channels[dma_ch];
	unsigned long flags;

	if (!imxdma->name) {
		printk(KERN_CRIT "%s: called for  not allocated channel %d\n",
		       __func__, dma_ch);
		return -ENODEV;
	}

	local_irq_save(flags);
	DISR = (1 << dma_ch);
	imxdma->irq_handler = irq_handler;
	imxdma->err_handler = err_handler;
	imxdma->data = data;
	local_irq_restore(flags);
	return 0;
}

/**
 * imx_dma_setup_progression_handler - setup i.MX DMA channel progression
 * handlers
 * @channel: i.MX DMA channel number
 * @prog_handler: the pointer to the function called if the transfer progresses
 */
int
imx_dma_setup_progression_handler(int channel,
			void (*prog_handler) (int, void*, struct scatterlist*))
{
	struct imx_dma_channel *imxdma = &imx_dma_channels[channel];
	unsigned long flags;

	if (!imxdma->name) {
		printk(KERN_CRIT "%s: called for  not allocated channel %d\n",
		       __func__, channel);
		return -ENODEV;
	}

	local_irq_save(flags);
	imxdma->prog_handler = prog_handler;
	local_irq_restore(flags);
	return 0;
}
EXPORT_SYMBOL(imx_dma_setup_progression_handler);

/**
 * imx_dma_enable - function to start i.MX DMA channel operation
 * @dma_ch: i.MX DMA channel number
 *
 * The channel has to be allocated by driver through imx_dma_request()
 * or imx_dma_request_by_prio() function.
 * The transfer parameters has to be set to the channel registers through
 * call of the imx_dma_setup_single() or imx_dma_setup_sg() function
 * and registers %BLR(dma_ch), %RSSR(dma_ch) and %CCR(dma_ch) has to
 * be set prior this function call by the channel user.
 */
void imx_dma_enable(imx_dmach_t dma_ch)
{
	struct imx_dma_channel *imxdma = &imx_dma_channels[dma_ch];
	unsigned long flags;

	pr_debug("imxdma%d: imx_dma_enable\n", dma_ch);

	if (!imxdma->name) {
		printk(KERN_CRIT "%s: called for  not allocated channel %d\n",
		       __func__, dma_ch);
		return;
	}

	if (imxdma->in_use)
		return;

	local_irq_save(flags);
	DISR = (1 << dma_ch);
	DIMR &= ~(1 << dma_ch);
	CCR(dma_ch) |= CCR_CEN;
	imxdma->in_use = 1;

	local_irq_restore(flags);
}

/**
 * imx_dma_disable - stop, finish i.MX DMA channel operatin
 * @dma_ch: i.MX DMA channel number
 */
void imx_dma_disable(int channel)
{
	struct imx_dma_channel *imxdma = &imx_dma_channels[channel];
	unsigned long flags;

	pr_debug("imxdma%d: imx_dma_disable\n", channel);

	local_irq_save(flags);
	DIMR |= (1 << channel);
	CCR(channel) &= ~CCR_CEN;
	DISR = (1 << channel);
	imxdma->in_use = 0;
	local_irq_restore(flags);
}

/**
 * imx_dma_request - request/allocate specified channel number
 * @dma_ch: i.MX DMA channel number
 * @name: the driver/caller own non-%NULL identification
 */
int imx_dma_request(imx_dmach_t dma_ch, const char *name)
{
	struct imx_dma_channel *imxdma = &imx_dma_channels[dma_ch];
	unsigned long flags;

	/* basic sanity checks */
	if (!name)
		return -EINVAL;

	if (dma_ch >= IMX_DMA_CHANNELS) {
		printk(KERN_CRIT "%s: called for  non-existed channel %d\n",
		       __func__, dma_ch);
		return -EINVAL;
	}

	local_irq_save(flags);
	if (imxdma->name) {
		local_irq_restore(flags);
		return -ENODEV;
	}

	imxdma->name = name;
	imxdma->irq_handler = NULL;
	imxdma->err_handler = NULL;
	imxdma->data = NULL;
	imxdma->sg = NULL;
	local_irq_restore(flags);

	return 0;
}

/**
 * imx_dma_free - release previously acquired channel
 * @dma_ch: i.MX DMA channel number
 */
void imx_dma_free(imx_dmach_t dma_ch)
{
	unsigned long flags;
	struct imx_dma_channel *imxdma = &imx_dma_channels[dma_ch];

	if (!imxdma->name) {
		printk(KERN_CRIT
		       "%s: trying to free channel %d which is already freed\n",
		       __func__, dma_ch);
		return;
	}

	local_irq_save(flags);
	/* Disable interrupts */
	DIMR |= (1 << dma_ch);
	CCR(dma_ch) &= ~CCR_CEN;
	imxdma->name = NULL;
	local_irq_restore(flags);
}

/**
 * imx_dma_request_by_prio - find and request some of free channels best suiting requested priority
 * @name: the driver/caller own non-%NULL identification
 * @prio: one of the hardware distinguished priority level:
 *        %DMA_PRIO_HIGH, %DMA_PRIO_MEDIUM, %DMA_PRIO_LOW
 *
 * This function tries to find free channel in the specified priority group
 * if the priority cannot be achieved it tries to look for free channel
 * in the higher and then even lower priority groups.
 *
 * Return value: If there is no free channel to allocate, -%ENODEV is returned.
 *               On successful allocation channel is returned.
 */
imx_dmach_t imx_dma_request_by_prio(const char *name, imx_dma_prio prio)
{
	int i;
	int best;

	switch (prio) {
	case (DMA_PRIO_HIGH):
		best = 8;
		break;
	case (DMA_PRIO_MEDIUM):
		best = 4;
		break;
	case (DMA_PRIO_LOW):
	default:
		best = 0;
		break;
	}

	for (i = best; i < IMX_DMA_CHANNELS; i++) {
		if (!imx_dma_request(i, name)) {
			return i;
		}
	}

	for (i = best - 1; i >= 0; i--) {
		if (!imx_dma_request(i, name)) {
			return i;
		}
	}

	printk(KERN_ERR "%s: no free DMA channel found\n", __func__);
	dump_channels();

	return -ENODEV;
}

static irqreturn_t dma_err_handler(int irq, void *dev_id)
{
	int i, disr = DISR;
	struct imx_dma_channel *channel;
	unsigned int err_mask = DBTOSR | DRTOSR | DSESR | DBOSR;
	int errcode;

	DISR = disr & err_mask;
	for (i = 0; i < IMX_DMA_CHANNELS; i++) {
		if(!(err_mask & (1 << i)))
			continue;
		channel = &imx_dma_channels[i];
		errcode = 0;

		if (DBTOSR & (1 << i)) {
			DBTOSR = (1 << i);
			errcode |= IMX_DMA_ERR_BURST;
		}
		if (DRTOSR & (1 << i)) {
			DRTOSR = (1 << i);
			errcode |= IMX_DMA_ERR_REQUEST;
		}
		if (DSESR & (1 << i)) {
			DSESR = (1 << i);
			errcode |= IMX_DMA_ERR_TRANSFER;
		}
		if (DBOSR & (1 << i)) {
			DBOSR = (1 << i);
			errcode |= IMX_DMA_ERR_BUFFER;
		}

		/*
		 * The cleaning of @sg field would be questionable
		 * there, because its value can help to compute
		 * remaining/transferred bytes count in the handler
		 */
		/*imx_dma_channels[i].sg = NULL;*/

		if (channel->name && channel->err_handler) {
			channel->err_handler(i, channel->data, errcode);
			continue;
		}

		imx_dma_channels[i].sg = NULL;

		printk(KERN_WARNING
		       "DMA timeout on channel %d (%s) -%s%s%s%s\n",
		       i, channel->name,
		       errcode&IMX_DMA_ERR_BURST?    " burst":"",
		       errcode&IMX_DMA_ERR_REQUEST?  " request":"",
		       errcode&IMX_DMA_ERR_TRANSFER? " transfer":"",
		       errcode&IMX_DMA_ERR_BUFFER?   " buffer":"");
	}
	return IRQ_HANDLED;
}

static void dma_irq_handle_channel(int chno)
{
	struct imx_dma_channel *channel = &imx_dma_channels[chno];

	if (!channel->name) {
		/*
		 * IRQ for an unregistered DMA channel:
		 * let's clear the interrupts and disable it.
		 */
		printk(KERN_WARNING
		       "spurious IRQ for DMA channel %d\n", chno);
		return;
	}

	if (channel->sg) {
		struct scatterlist *current_sg = channel->sg;
		channel->sg = sg_next(channel->sg);

		if (channel->sg) {
			imx_dma_sg_next(chno, channel->sg);

			CCR(chno) &= ~CCR_CEN;
			mb();
			CCR(chno) |= CCR_CEN;

			if (channel->prog_handler)
				channel->prog_handler(chno, channel->data,
						current_sg);

			return;
		}
	}

	CCR(chno) = 0;
	channel->in_use = 0;

	if (channel->irq_handler)
		channel->irq_handler(chno, channel->data);
}

static irqreturn_t dma_irq_handler(int irq, void *dev_id)
{
	int i, disr = DISR;

	pr_debug("imxdma: dma_irq_handler called, disr=0x%08x\n",
		     disr);

	DISR = disr;
	for (i = 0; i < IMX_DMA_CHANNELS; i++) {
		if (disr & (1 << i))
			dma_irq_handle_channel(i);
	}
	return IRQ_HANDLED;
}

static int __init imx_dma_init(void)
{
	int ret;
	int i;

	/* reset DMA module */
	DCR = DCR_DRST;

	ret = request_irq(DMA_INT, dma_irq_handler, 0, "DMA", NULL);
	if (ret) {
		printk(KERN_CRIT "Wow!  Can't register IRQ for DMA\n");
		return ret;
	}

	ret = request_irq(DMA_ERR, dma_err_handler, 0, "DMA", NULL);
	if (ret) {
		printk(KERN_CRIT "Wow!  Can't register ERRIRQ for DMA\n");
		free_irq(DMA_INT, NULL);
	}

	/* enable DMA module */
	DCR = DCR_DEN;

	/* clear all interrupts */
	DISR = (1 << IMX_DMA_CHANNELS) - 1;

	/* enable interrupts */
	DIMR = (1 << IMX_DMA_CHANNELS) - 1;

	for (i = 0; i < IMX_DMA_CHANNELS; i++) {
		imx_dma_channels[i].sg = NULL;
		imx_dma_channels[i].dma_num = i;
	}

	return ret;
}

arch_initcall(imx_dma_init);

EXPORT_SYMBOL(imx_dma_setup_single);
EXPORT_SYMBOL(imx_dma_setup_sg);
EXPORT_SYMBOL(imx_dma_setup_handlers);
EXPORT_SYMBOL(imx_dma_enable);
EXPORT_SYMBOL(imx_dma_disable);
EXPORT_SYMBOL(imx_dma_request);
EXPORT_SYMBOL(imx_dma_free);
EXPORT_SYMBOL(imx_dma_request_by_prio);
EXPORT_SYMBOL(imx_dma_channels);
