/*
 * V4L2 Driver for MX27 camera host
 *
 * Copyright (C) 2008, Sascha Hauer, Pengutronix
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/time.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/clk.h>

#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/videobuf-dma-contig.h>
#include <media/soc_camera.h>

#include <linux/videodev2.h>

#include <mach/imx_cam.h>
#include <asm/dma.h>
#include <mach/dma-mx1-mx2.h>

#include <asm/dma.h>

#define MX27_CAM_DRV_NAME "mx27-camera"
#define MX27_CAM_VERSION_CODE KERNEL_VERSION(0, 0, 5) /* FIXME: Whats this? */

static const char *mx27_cam_driver_description = "i.MX27_Camera";

/* reset values */
#define CSICR1_RESET_VAL	0x40000800
#define CSICR2_RESET_VAL	0x0
#define CSICR3_RESET_VAL	0x0

/* csi control reg 1 */
#define CSICR1_SWAP16_EN	(1 << 31)
#define CSICR1_EXT_VSYNC	(1 << 30)
#define CSICR1_EOF_INTEN	(1 << 29)
#define CSICR1_PRP_IF_EN	(1 << 28)
#define CSICR1_CCIR_MODE	(1 << 27)
#define CSICR1_COF_INTEN	(1 << 26)
#define CSICR1_SF_OR_INTEN	(1 << 25)
#define CSICR1_RF_OR_INTEN	(1 << 24)
#define CSICR1_STATFF_LEVEL	(3 << 22)
#define CSICR1_STATFF_INTEN	(1 << 21)
#define CSICR1_RXFF_LEVEL(l)	(((l) & 3) << 19)
#define CSICR1_RXFF_INTEN	(1 << 18)
#define CSICR1_SOF_POL		(1 << 17)
#define CSICR1_SOF_INTEN	(1 << 16)
#define CSICR1_MCLKDIV(d)	(((d) & 0xF) << 12)
#define CSICR1_HSYNC_POL	(1 << 11)
#define CSICR1_CCIR_EN		(1 << 10)
#define CSICR1_MCLKEN		(1 << 9)
#define CSICR1_FCC		(1 << 8)
#define CSICR1_PACK_DIR		(1 << 7)
#define CSICR1_CLR_STATFIFO	(1 << 6)
#define CSICR1_CLR_RXFIFO	(1 << 5)
#define CSICR1_GCLK_MODE	(1 << 4)
#define CSICR1_INV_DATA		(1 << 3)
#define CSICR1_INV_PCLK		(1 << 2)
#define CSICR1_REDGE		(1 << 1)

#define SHIFT_STATFF_LEVEL	22
#define SHIFT_RXFF_LEVEL	19
#define SHIFT_MCLKDIV		12

/* control reg 3 */
#define CSICR3_FRMCNT		(0xFFFF << 16)
#define CSICR3_FRMCNT_RST	(1 << 15)
#define CSICR3_CSI_SUP		(1 << 3)
#define CSICR3_ZERO_PACK_EN	(1 << 2)
#define CSICR3_ECC_INT_EN	(1 << 1)
#define CSICR3_ECC_AUTO_EN	(1 << 0)

#define SHIFT_FRMCNT		16

/* csi status reg */
#define CSISR_SFF_OR_INT	(1 << 25)
#define CSISR_RFF_OR_INT	(1 << 24)
#define CSISR_STATFF_INT	(1 << 21)
#define CSISR_RXFF_INT		(1 << 18)
#define CSISR_EOF_INT		(1 << 17)
#define CSISR_SOF_INT		(1 << 16)
#define CSISR_F2_INT		(1 << 15)
#define CSISR_F1_INT		(1 << 14)
#define CSISR_COF_INT		(1 << 13)
#define CSISR_ECC_INT		(1 << 1)
#define CSISR_DRDY		(1 << 0)

#define CSICR1		0x00
#define CSICR2		0x04
#define CSISR		0x08
#define CSISTATFIFO	0x0c
#define CSIRFIFO	0x10
#define CSIRXCNT	0x14
#define CSICR3		0x1C

/* EMMA PrP */
#define PRP_CNTL			0x00
#define PRP_INTR_CNTL			0x04
#define PRP_INTRSTATUS			0x08
#define PRP_SOURCE_Y_PTR		0x0c
#define PRP_SOURCE_CB_PTR		0x10
#define PRP_SOURCE_CR_PTR		0x14
#define PRP_DEST_RGB1_PTR		0x18
#define PRP_DEST_RGB2_PTR		0x1c
#define PRP_DEST_Y_PTR			0x20
#define PRP_DEST_CB_PTR			0x24
#define PRP_DEST_CR_PTR			0x28
#define PRP_SRC_FRAME_SIZE		0x2c
#define PRP_DEST_CH1_LINE_STRIDE	0x30
#define PRP_SRC_PIXEL_FORMAT_CNTL	0x34
#define PRP_CH1_PIXEL_FORMAT_CNTL	0x38
#define PRP_CH1_OUT_IMAGE_SIZE		0x3c
#define PRP_CH2_OUT_IMAGE_SIZE		0x40
#define PRP_SRC_LINE_STRIDE		0x44
#define PRP_CSC_COEF_012		0x48
#define PRP_CSC_COEF_345		0x4c
#define PRP_CSC_COEF_678		0x50
#define PRP_CH1_RZ_HORI_COEF1		0x54
#define PRP_CH1_RZ_HORI_COEF2		0x58
#define PRP_CH1_RZ_HORI_VALID		0x5c
#define PRP_CH1_RZ_VERT_COEF1		0x60
#define PRP_CH1_RZ_VERT_COEF2		0x64
#define PRP_CH1_RZ_VERT_VALID		0x68
#define PRP_CH2_RZ_HORI_COEF1		0x6c
#define PRP_CH2_RZ_HORI_COEF2		0x70
#define PRP_CH2_RZ_HORI_VALID		0x74
#define PRP_CH2_RZ_VERT_COEF1		0x78
#define PRP_CH2_RZ_VERT_COEF2		0x7c
#define PRP_CH2_RZ_VERT_VALID		0x80

#define PRP_CNTL_CH1EN		(1 << 0)
#define PRP_CNTL_CH2EN		(1 << 1)
#define PRP_CNTL_CSIEN		(1 << 2)
#define PRP_CNTL_DATA_IN_YUV420	(0 << 3)
#define PRP_CNTL_DATA_IN_YUV422	(1 << 3)
#define PRP_CNTL_DATA_IN_RGB16	(2 << 3)
#define PRP_CNTL_DATA_IN_RGB32	(3 << 3)
#define PRP_CNTL_CH1_OUT_RGB8	(0 << 5)
#define PRP_CNTL_CH1_OUT_RGB16	(1 << 5)
#define PRP_CNTL_CH1_OUT_RGB32	(2 << 5)
#define PRP_CNTL_CH1_OUT_YUV422	(3 << 5)
#define PRP_CNTL_CH2_OUT_YUV420	(0 << 7)
#define PRP_CNTL_CH2_OUT_YUV422 (1 << 7)
#define PRP_CNTL_CH2_OUT_YUV444	(2 << 7)
#define PRP_CNTL_CH1_LEN	(1 << 9)
#define PRP_CNTL_CH2_LEN	(1 << 10)
#define PRP_CNTL_SKIP_FRAME	(1 << 11)
#define PRP_CNTL_SWRST		(1 << 12)
#define PRP_CNTL_CLKEN		(1 << 13)
#define PRP_CNTL_WEN		(1 << 14)
#define PRP_CNTL_CH1BYP		(1 << 15)
#define PRP_CNTL_IN_TSKIP(x)	((x) << 16)
#define PRP_CNTL_CH1_TSKIP(x)	((x) << 19)
#define PRP_CNTL_CH2_TSKIP(x)	((x) << 22)
#define PRP_CNTL_INPUT_FIFO_LEVEL(x)	((x) << 25)
#define PRP_CNTL_RZ_FIFO_LEVEL(x)	((x) << 27)
#define PRP_CNTL_CH2B1EN	(1 << 29)
#define PRP_CNTL_CH2B2EN	(1 << 30)
#define PRP_CNTL_CH2FEN		(1 << 31)

/* IRQ Enable and status register */
#define PRP_INTR_RDERR		(1 << 0)
#define PRP_INTR_CH1WERR	(1 << 1)
#define PRP_INTR_CH2WERR	(1 << 2)
#define PRP_INTR_CH1FC		(1 << 3)
#define PRP_INTR_CH2FC		(1 << 5)
#define PRP_INTR_LBOVF		(1 << 7)
#define PRP_INTR_CH2OVF		(1 << 8)

#define mx27_camera_emma(pcdev)	(pcdev->use_emma)

/* Currently we do not need irqs. All we need is DMA callback
 * Leave it here for reference for some time.
 */
#undef MX27_CAMERA_USE_IRQ

struct mx27_camera_dev {
	struct device		*dev;
	struct soc_camera_device *icd;
	struct clk		*clk_csi, *clk_emma;

	unsigned int		irq_csi, irq_emma;
	void __iomem		*base_csi, *base_emma;

	struct mx27_camera_platform_data *pdata;
	struct resource		*res_csi, *res_emma;
	unsigned long		platform_flags;

	struct list_head	capture;
	struct list_head	active_bufs;

	spinlock_t		lock;

	int			dma;
	struct mx27_buffer	*active;

	int			use_emma;

	unsigned int		csicr1;

	void __iomem		*discard_buffer;
	dma_addr_t		discard_buffer_dma;
	size_t			discard_size;
};

/* buffer for one video frame */
struct mx27_buffer {
	/* common v4l buffer stuff -- must be first */
	struct videobuf_buffer vb;

	const struct soc_camera_data_format        *fmt;

	int bufnum;
};

static DEFINE_MUTEX(camera_lock);

static int mclk_get_divisor(struct mx27_camera_dev *pcdev)
{
	dev_info(pcdev->dev, "%s not implemented. Running at max speed\n",
			__func__);

#if 0
	unsigned int mclk = pcdev->pdata->clk_csi;
	unsigned int pclk = clk_get_rate(pcdev->clk_csi);
	int i;

	dev_dbg(pcdev->dev, "%s: %ld %ld\n", __func__, mclk, pclk);

	for (i = 0; i < 0xf; i++)
		if ((i + 1) * 2 * mclk <= pclk)
			break;
	return i;
#endif
	return 0;
}

static void mx27_camera_deactivate(struct mx27_camera_dev *pcdev)
{
	clk_disable(pcdev->clk_csi);
	writel(0, pcdev->base_csi + CSICR1);
	if (mx27_camera_emma(pcdev))
		writel(0, pcdev->base_emma + PRP_CNTL);
}

/* The following two functions absolutely depend on the fact, that
 * there can be only one camera on mx27 quick capture interface */
static int mx27_camera_add_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mx27_camera_dev *pcdev = ici->priv;
	int ret;
	u32 csicr1;

	mutex_lock(&camera_lock);

	if (pcdev->icd) {
		ret = -EBUSY;
		goto ebusy;
	}

	dev_info(&icd->dev, "Camera driver attached to camera %d\n",
		 icd->devnum);

	clk_enable(pcdev->clk_csi);

	csicr1 = CSICR1_MCLKDIV(mclk_get_divisor(pcdev)) |
			CSICR1_MCLKEN;
	if (mx27_camera_emma(pcdev)) {
		csicr1 |= CSICR1_PRP_IF_EN | CSICR1_FCC |
			CSICR1_RXFF_LEVEL(0);
	} else
		csicr1 |= CSICR1_SOF_INTEN | CSICR1_RXFF_LEVEL(2);

	pcdev->csicr1 = csicr1;
	writel(pcdev->csicr1, pcdev->base_csi + CSICR1);

	ret = icd->ops->init(icd);

	if (!ret)
		pcdev->icd = icd;

ebusy:
	mutex_unlock(&camera_lock);

	return ret;
}

static void mx27_camera_remove_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mx27_camera_dev *pcdev = ici->priv;

	BUG_ON(icd != pcdev->icd);

	dev_info(&icd->dev, "Camera driver detached from camera %d\n",
		 icd->devnum);

	icd->ops->release(icd);

	mx27_camera_deactivate(pcdev);

	if (pcdev->discard_buffer) {
		dma_free_coherent(NULL, pcdev->discard_size,
				pcdev->discard_buffer,
				pcdev->discard_buffer_dma);
	}
	pcdev->discard_buffer = 0;

	pcdev->icd = NULL;
}

static void mx27_camera_dma_enable(struct mx27_camera_dev *pcdev)
{
	u32 tmp;

	imx_dma_enable(pcdev->dma);

	tmp = readl(pcdev->base_csi + CSICR1);
	tmp |= CSICR1_RF_OR_INTEN;
	writel(tmp, pcdev->base_csi + CSICR1);
}

static irqreturn_t mx27_camera_irq(int irq_csi, void *data)
{
	struct mx27_camera_dev *pcdev = data;
	u32 status = readl(pcdev->base_csi + CSISR);

	if (status & CSISR_SOF_INT && pcdev->active) {
		u32 tmp;

		tmp = readl(pcdev->base_csi + CSICR1);
		tmp |= CSICR1_CLR_RXFIFO;
		writel(tmp, pcdev->base_csi + CSICR1);
		mx27_camera_dma_enable(pcdev);
		writel(CSISR_SOF_INT | CSISR_RFF_OR_INT,
				pcdev->base_csi + CSISR);
		status &= ~CSISR_RFF_OR_INT;
	}

	writel(CSISR_SOF_INT | CSISR_RFF_OR_INT, pcdev->base_csi + CSISR);

	return IRQ_HANDLED;
}

static unsigned int vid_limit = 16;	/* Video memory limit, in Mb */

/*
 *  Videobuf operations
 */
static int mx27_videobuf_setup(struct videobuf_queue *vq, unsigned int *count,
			      unsigned int *size)
{
	struct soc_camera_device *icd = vq->priv_data;

	dev_dbg(&icd->dev, "count=%d, size=%d\n", *count, *size);

	*size = icd->width * icd->height *
		((icd->current_fmt->depth + 7) >> 3);

	if (0 == *count)
		*count = 32;
	while (*size * *count > vid_limit * 1024 * 1024)
		(*count)--;

	return 0;
}

static void free_buffer(struct videobuf_queue *vq, struct mx27_buffer *buf)
{
	struct soc_camera_device *icd = vq->priv_data;

	BUG_ON(in_interrupt());

	dev_dbg(&icd->dev, "%s (vb=0x%p) 0x%08lx %d\n", __func__,
		&buf->vb, buf->vb.baddr, buf->vb.bsize);

	/* This waits until this buffer is out of danger, i.e., until it is no
	 * longer in STATE_QUEUED or STATE_ACTIVE */
	videobuf_waiton(&buf->vb, 0, 0);

	videobuf_dma_contig_free(vq, &buf->vb);
	dev_dbg(&icd->dev, "%s freed\n", __func__);

	buf->vb.state = VIDEOBUF_NEEDS_INIT;
}

static int mx27_videobuf_prepare(struct videobuf_queue *vq,
		struct videobuf_buffer *vb, enum v4l2_field field)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct mx27_buffer *buf = container_of(vb, struct mx27_buffer, vb);
	int ret = 0;

#ifdef DEBUG
	/* This can be useful if you want to see if we actually fill
	 * the buffer with something */
	memset((void *)vb->baddr, 0xaa, vb->bsize);
#endif

	dev_dbg(&icd->dev, "%s (vb=0x%p) 0x%08lx %d\n", __func__,
		vb, vb->baddr, vb->bsize);

	if (buf->fmt	!= icd->current_fmt ||
	    vb->width	!= icd->width ||
	    vb->height	!= icd->height ||
	    vb->field	!= field) {
		buf->fmt	= icd->current_fmt;
		vb->width	= icd->width;
		vb->height	= icd->height;
		vb->field	= field;
		vb->state	= VIDEOBUF_NEEDS_INIT;
	}

	vb->size = vb->width * vb->height * ((buf->fmt->depth + 7) >> 3);
	if (vb->baddr && vb->bsize < vb->size) {
		ret = -EINVAL;
		goto out;
	}

	if (vb->state == VIDEOBUF_NEEDS_INIT) {
		ret = videobuf_iolock(vq, vb, NULL);
		if (ret)
			goto fail;

		vb->state = VIDEOBUF_PREPARED;
	}

	return 0;

fail:
	free_buffer(vq, buf);
out:
	return ret;
}

static void mx27_videobuf_queue(struct videobuf_queue *vq,
			       struct videobuf_buffer *vb)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct soc_camera_host *ici =
		to_soc_camera_host(icd->dev.parent);
	struct mx27_camera_dev *pcdev = ici->priv;
	struct mx27_buffer *buf = container_of(vb, struct mx27_buffer, vb);
	unsigned long flags;
	int ret;

	dev_dbg(&icd->dev, "%s (vb=0x%p) 0x%08lx %d\n", __func__,
		vb, vb->baddr, vb->bsize);

	spin_lock_irqsave(&pcdev->lock, flags);

	if (mx27_camera_emma(pcdev)) {
		list_add_tail(&vb->queue, &pcdev->capture);
		vb->state = VIDEOBUF_QUEUED;
	} else {
		list_add_tail(&vb->queue, &pcdev->capture);
		vb->state = VIDEOBUF_ACTIVE;

		if (!pcdev->active) {
			ret = imx_dma_setup_single(pcdev->dma,
					videobuf_to_dma_contig(vb), vb->size,
					CSI_BASE_ADDR + 0x10, DMA_MODE_READ);
			if (ret) {
				vb->state = VIDEOBUF_ERROR;
				wake_up(&vb->done);
				goto out;
			}

			pcdev->active = buf;
		}
	}

out:
	spin_unlock_irqrestore(&pcdev->lock, flags);
}

static void mx27_videobuf_release(struct videobuf_queue *vq,
				 struct videobuf_buffer *vb)
{
	struct mx27_buffer *buf = container_of(vb, struct mx27_buffer, vb);

#ifdef DEBUG
	struct soc_camera_device *icd = vq->priv_data;

	dev_dbg(&icd->dev, "%s (vb=0x%p) 0x%08lx %d\n", __func__,
		vb, vb->baddr, vb->bsize);

	switch (vb->state) {
	case VIDEOBUF_ACTIVE:
		dev_info(&icd->dev, "%s (active)\n", __func__);
		break;
	case VIDEOBUF_QUEUED:
		dev_info(&icd->dev, "%s (queued)\n", __func__);
		break;
	case VIDEOBUF_PREPARED:
		dev_info(&icd->dev, "%s (prepared)\n", __func__);
		break;
	default:
		dev_info(&icd->dev, "%s (unknown) %d\n", __func__,
				vb->state);
		break;
	}
#endif

	free_buffer(vq, buf);
}

static struct videobuf_queue_ops mx27_videobuf_ops = {
	.buf_setup      = mx27_videobuf_setup,
	.buf_prepare    = mx27_videobuf_prepare,
	.buf_queue      = mx27_videobuf_queue,
	.buf_release    = mx27_videobuf_release,
};

static void mx27_camera_init_videobuf(struct videobuf_queue *q,
			      struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mx27_camera_dev *pcdev = ici->priv;

	/* We must pass NULL as dev pointer, then all pci_* dma operations
	 * transform to normal dma_* ones. */
	videobuf_queue_dma_contig_init(q, &mx27_videobuf_ops, NULL,
			&pcdev->lock, V4L2_BUF_TYPE_VIDEO_CAPTURE,
			V4L2_FIELD_NONE, sizeof(struct mx27_buffer), icd);
}

#define MX27_BUS_FLAGS	(SOCAM_DATAWIDTH_8 | \
			SOCAM_MASTER | \
			SOCAM_VSYNC_ACTIVE_HIGH | \
			SOCAM_HSYNC_ACTIVE_HIGH | \
			SOCAM_HSYNC_ACTIVE_LOW | \
			SOCAM_PCLK_SAMPLE_RISING | \
			SOCAM_PCLK_SAMPLE_FALLING)

static int mx27_camera_emma_prp_reset(struct mx27_camera_dev *pcdev)
{
	unsigned int cntl;

	cntl = readl(pcdev->base_emma + PRP_CNTL);
	writel(PRP_CNTL_SWRST, pcdev->base_emma + PRP_CNTL);
	while (readl(pcdev->base_emma + PRP_CNTL) & PRP_CNTL_SWRST)
		barrier();

	return 0;
}

static int mx27_camera_set_bus_param(struct soc_camera_device *icd,
		__u32 pixfmt)
{
	struct soc_camera_host *ici =
		to_soc_camera_host(icd->dev.parent);
	struct mx27_camera_dev *pcdev = ici->priv;
	unsigned long camera_flags, common_flags;
	int ret = 0;
	u32 csicr1 = pcdev->csicr1;

	camera_flags = icd->ops->query_bus_param(icd);

	common_flags = soc_camera_bus_param_compatible(camera_flags,
				MX27_BUS_FLAGS);
	if (!common_flags)
		return -EINVAL;

	if ((common_flags & SOCAM_HSYNC_ACTIVE_HIGH) &&
	    (common_flags & SOCAM_HSYNC_ACTIVE_LOW)) {
		if (pcdev->platform_flags & MX27_CAMERA_HSYNC_HIGH)
			common_flags &= ~SOCAM_HSYNC_ACTIVE_LOW;
		else
			common_flags &= ~SOCAM_HSYNC_ACTIVE_HIGH;
	}

	if ((common_flags & SOCAM_PCLK_SAMPLE_RISING) &&
	    (common_flags & SOCAM_PCLK_SAMPLE_FALLING)) {
		if (pcdev->platform_flags & MX27_CAMERA_PCLK_SAMPLE_RISING)
			common_flags &= ~SOCAM_PCLK_SAMPLE_FALLING;
		else
			common_flags &= ~SOCAM_PCLK_SAMPLE_RISING;
	}

	ret = icd->ops->set_bus_param(icd, common_flags);
	if (ret < 0)
		return ret;

	if (common_flags & SOCAM_PCLK_SAMPLE_FALLING)
		csicr1 |= CSICR1_INV_PCLK;
	if (common_flags & SOCAM_PCLK_SAMPLE_RISING)
		csicr1 |= CSICR1_REDGE;
	if (common_flags & SOCAM_HSYNC_ACTIVE_HIGH)
		csicr1 |= CSICR1_HSYNC_POL;
	if (pcdev->platform_flags & MX27_CAMERA_SWAP16)
		csicr1 |= CSICR1_SWAP16_EN;
	if (pcdev->platform_flags & MX27_CAMERA_EXT_VSYNC)
		csicr1 |= CSICR1_EXT_VSYNC;
	if (pcdev->platform_flags & MX27_CAMERA_CCIR)
		csicr1 |= CSICR1_CCIR_EN;
	if (pcdev->platform_flags & MX27_CAMERA_CCIR_INTERLACE)
		csicr1 |= CSICR1_CCIR_MODE;
	if (pcdev->platform_flags & MX27_CAMERA_GATED_CLOCK)
		csicr1 |= CSICR1_GCLK_MODE;
	if (pcdev->platform_flags & MX27_CAMERA_INV_DATA)
		csicr1 |= CSICR1_INV_DATA;
	if (pcdev->platform_flags & MX27_CAMERA_PACK_DIR_MSB)
		csicr1 |= CSICR1_PACK_DIR;

	if (mx27_camera_emma(pcdev)) {
		int bytesperline = (icd->width * icd->current_fmt->depth) >> 3;

		if (mx27_camera_emma_prp_reset(pcdev))
			return -ENODEV;

		if (pcdev->discard_buffer)
			dma_free_coherent(NULL, pcdev->discard_size,
				pcdev->discard_buffer,
				pcdev->discard_buffer_dma);

		/* I didn't manage to properly enable/disable the prp
		 * on a per frame basis during running transfers,
		 * thus we allocate a buffer here and use it to
		 * discard frames when no buffer is available.
		 * Feel free to work on this ;)
		 */
		pcdev->discard_size = icd->height * bytesperline;
		pcdev->discard_buffer = dma_alloc_coherent(NULL,
				pcdev->discard_size, &pcdev->discard_buffer_dma,
				GFP_KERNEL);
		if (!pcdev->discard_buffer)
			return -ENOMEM;

		writel(pcdev->discard_buffer_dma,
				pcdev->base_emma + PRP_DEST_RGB1_PTR);
		writel(pcdev->discard_buffer_dma,
				pcdev->base_emma + PRP_DEST_RGB2_PTR);

		/* We only use the EMMA engine to get rid of the f**king
		 * DMA Engine. No color space consversion at the moment.
		 * We adjust incoming and outgoing pixelformat to rgb16
		 * and adjust the bytesperline accordingly.
		 */
		writel(PRP_CNTL_CH1EN |
			PRP_CNTL_CSIEN |
			PRP_CNTL_DATA_IN_RGB16 |
			PRP_CNTL_CH1_OUT_RGB16 |
			PRP_CNTL_CH1_LEN |
			PRP_CNTL_CH1BYP |
			PRP_CNTL_CH1_TSKIP(0) |
			PRP_CNTL_IN_TSKIP(0),
			pcdev->base_emma + PRP_CNTL);

		writel(((bytesperline >> 1) << 16) | icd->height,
				pcdev->base_emma + PRP_SRC_FRAME_SIZE);
		writel(((bytesperline >> 1) << 16) | icd->height,
				pcdev->base_emma + PRP_CH1_OUT_IMAGE_SIZE);
		writel(bytesperline,
				pcdev->base_emma + PRP_DEST_CH1_LINE_STRIDE);
		writel(0x2ca00565,
				pcdev->base_emma + PRP_SRC_PIXEL_FORMAT_CNTL);
		writel(0x2ca00565,
				pcdev->base_emma + PRP_CH1_PIXEL_FORMAT_CNTL);

		/* Enable interrupts */
		writel(PRP_INTR_RDERR |
			PRP_INTR_CH1WERR |
			PRP_INTR_CH2WERR |
			PRP_INTR_CH1FC |
			PRP_INTR_CH2FC |
			PRP_INTR_LBOVF |
			PRP_INTR_CH2OVF
			, pcdev->base_emma + PRP_INTR_CNTL);
	}

	pcdev->csicr1 = csicr1;

	writel(csicr1, pcdev->base_csi + CSICR1);

	return 0;
}

/* DEPRECATED
static int mx27_camera_try_bus_param(struct soc_camera_device *icd,
		__u32 pixfmt)
{
	unsigned long bus_flags, camera_flags;

	bus_flags = MX27_BUS_FLAGS;

	camera_flags = icd->ops->query_bus_param(icd);

	return soc_camera_bus_param_compatible(camera_flags, bus_flags) ?
				0 : -EINVAL;
}
*/

static int mx27_camera_set_fmt_cap(struct soc_camera_device *icd,
				  __u32 pixfmt, struct v4l2_rect *rect)
{
	return icd->ops->set_fmt(icd, pixfmt, rect);
}

static int mx27_camera_try_fmt_cap(struct soc_camera_device *icd,
				  struct v4l2_format *f)
{
	return 0;
}

static int mx27_camera_querycap(struct soc_camera_host *ici,
			       struct v4l2_capability *cap)
{
	/* cap->name is set by the friendly caller:-> */
	strlcpy(cap->card, mx27_cam_driver_description, sizeof(cap->card));
	cap->version = MX27_CAM_VERSION_CODE;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

	return 0;
}

static int mx27_camera_reqbufs(struct soc_camera_file *icf,
			      struct v4l2_requestbuffers *p)
{
	int i;

	for (i = 0; i < p->count; i++) {
		struct mx27_buffer *buf = container_of(icf->vb_vidq.bufs[i],
						      struct mx27_buffer, vb);
		INIT_LIST_HEAD(&buf->vb.queue);
	}

	return 0;
}

static void mx27_camera_frame_done(struct mx27_camera_dev *pcdev, int state)
{
	struct videobuf_buffer *vb;
	struct mx27_buffer *buf;
	int ret;

	if (!pcdev->active) {
		dev_err(pcdev->dev, "%s called with no active buffer!\n",
				__func__);
		return;
	}

	vb = &pcdev->active->vb;
	buf = container_of(vb, struct mx27_buffer, vb);
	WARN_ON(list_empty(&vb->queue));
	dev_dbg(pcdev->dev, "%s (vb=0x%p) 0x%08lx %d\n", __func__,
		vb, vb->baddr, vb->bsize);

	/* _init is used to debug races, see comment in pxa_camera_reqbufs() */
	list_del_init(&vb->queue);
	vb->state = state;
	do_gettimeofday(&vb->ts);
	vb->field_count++;

	wake_up(&vb->done);

	if (list_empty(&pcdev->capture)) {
		pcdev->active = NULL;
		return;
	}

	pcdev->active = list_entry(pcdev->capture.next,
			struct mx27_buffer, vb.queue);

	vb = &pcdev->active->vb;
	vb->state = VIDEOBUF_ACTIVE;

	ret = imx_dma_setup_single(pcdev->dma, videobuf_to_dma_contig(vb),
			vb->size, CSI_BASE_ADDR + 0x10, DMA_MODE_READ);
	if (ret) {
		vb->state = VIDEOBUF_ERROR;
		wake_up(&vb->done);
		return;
	}
}

static void mx27_camera_dma_err_callback(int channel, void *data, int err)
{
	struct mx27_camera_dev *pcdev = data;
	unsigned long flags;

	spin_lock_irqsave(&pcdev->lock, flags);

	mx27_camera_frame_done(pcdev, VIDEOBUF_ERROR);

	spin_unlock_irqrestore(&pcdev->lock, flags);
}

static void mx27_camera_dma_callback(int channel, void *data)
{
	struct mx27_camera_dev *pcdev = data;
	unsigned long flags;

	spin_lock_irqsave(&pcdev->lock, flags);

	mx27_camera_frame_done(pcdev, VIDEOBUF_DONE);

	spin_unlock_irqrestore(&pcdev->lock, flags);
}

static unsigned int mx27_camera_poll(struct file *file, poll_table *pt)
{
	struct soc_camera_file *icf = file->private_data;
	struct mx27_buffer *buf;

	buf = list_entry(icf->vb_vidq.stream.next, struct mx27_buffer,
			 vb.stream);

	poll_wait(file, &buf->vb.done, pt);

	if (buf->vb.state == VIDEOBUF_DONE ||
	    buf->vb.state == VIDEOBUF_ERROR)
		return POLLIN | POLLRDNORM;

	return 0;
}

/* Should beallocated dynamically too, but we have only one. */
static struct soc_camera_host_ops mx27_soc_camera_host_ops = {
	.owner		= THIS_MODULE,
	.add		= mx27_camera_add_device,
	.remove		= mx27_camera_remove_device,
	.set_fmt	= mx27_camera_set_fmt_cap,
	.try_fmt	= mx27_camera_try_fmt_cap,
	.init_videobuf	= mx27_camera_init_videobuf,
	.reqbufs	= mx27_camera_reqbufs,
	.poll		= mx27_camera_poll,
	.querycap	= mx27_camera_querycap,
/*	.try_bus_param	= mx27_camera_try_bus_param, */
	.set_bus_param	= mx27_camera_set_bus_param,
};

static void mx27_camera_frame_done_emma(struct mx27_camera_dev *pcdev,
		int bufnum, int state)
{
	struct mx27_buffer *buf;
	struct videobuf_buffer *vb;
	unsigned long phys;

	if (!list_empty(&pcdev->active_bufs)) {
		buf = list_entry(pcdev->active_bufs.next,
			struct mx27_buffer, vb.queue);

		if (buf->bufnum == bufnum) {
			vb = &buf->vb;
#ifdef DEBUG
			phys = videobuf_to_dma_contig(vb);
			if (readl(pcdev->base_emma + PRP_DEST_RGB1_PTR +
						4 * bufnum) != phys) {
				dev_err(pcdev->dev, "%p != %p\n", phys,
						readl(pcdev->base_emma +
						PRP_DEST_RGB1_PTR +
						4 * bufnum));
			}
#endif
			dev_dbg(pcdev->dev, "%s (vb=0x%p) 0x%08lx %d\n",
					__func__, vb, vb->baddr, vb->bsize);

			list_del(&vb->queue);
			vb->state = state;
			do_gettimeofday(&vb->ts);
			vb->field_count++;

			wake_up(&vb->done);
		}
	}

	if (list_empty(&pcdev->capture)) {
		writel(pcdev->discard_buffer_dma, pcdev->base_emma +
				PRP_DEST_RGB1_PTR + 4 * bufnum);
		return;
	}

	buf = list_entry(pcdev->capture.next,
			struct mx27_buffer, vb.queue);

	buf->bufnum = bufnum;

	list_move_tail(pcdev->capture.next, &pcdev->active_bufs);

	vb = &buf->vb;
	vb->state = VIDEOBUF_ACTIVE;

	phys = videobuf_to_dma_contig(vb);
	writel(phys, pcdev->base_emma + PRP_DEST_RGB1_PTR + 4 * bufnum);
}

static irqreturn_t mx27_camera_emma_irq(int irq_emma, void *data)
{
	struct mx27_camera_dev *pcdev = data;
	unsigned int status = readl(pcdev->base_emma + PRP_INTRSTATUS);

	if (status & (1 << 6))
		mx27_camera_frame_done_emma(pcdev, 0, VIDEOBUF_DONE);
	if (status & (1 << 5))
		mx27_camera_frame_done_emma(pcdev, 1, VIDEOBUF_DONE);
	if (status & (1 << 7)) {
		uint32_t cntl;
		cntl = readl(pcdev->base_emma + PRP_CNTL);
		writel(cntl & ~PRP_CNTL_CH1EN, pcdev->base_emma + PRP_CNTL);
		writel(cntl, pcdev->base_emma + PRP_CNTL);
	}

	writel(status, pcdev->base_emma + PRP_INTRSTATUS);

	return IRQ_HANDLED;
}

/* Should be allocated dynamically too, but we have only one. */
static struct soc_camera_host mx27_soc_camera_host = {
	.drv_name		= MX27_CAM_DRV_NAME,
	.ops			= &mx27_soc_camera_host_ops,
};

int mx27_camera_emma_init(struct mx27_camera_dev *pcdev)
{
	struct resource *res_emma = pcdev->res_emma;
	int err = 0;

	if (!request_mem_region(res_emma->start, resource_size(res_emma),
				MX27_CAM_DRV_NAME)) {
		err = -EBUSY;
		goto out;
	}

	pcdev->base_emma = ioremap(res_emma->start, resource_size(res_emma));
	if (!pcdev->base_emma) {
		err = -ENOMEM;
		goto exit_release;
	}

	err = request_irq(pcdev->irq_emma, mx27_camera_emma_irq, 0,
			MX27_CAM_DRV_NAME, pcdev);
	if (err) {
		dev_err(pcdev->dev, "Camera EMMA interrupt register failed \n");
		goto exit_iounmap;
	}

	pcdev->clk_emma = clk_get(pcdev->dev, "emma_clk");
	if (IS_ERR(pcdev->clk_emma)) {
		err = PTR_ERR(pcdev->clk_emma);
		goto exit_free_irq;
	}

	clk_enable(pcdev->clk_emma);

	err = mx27_camera_emma_prp_reset(pcdev);
	if (err)
		goto exit_clk_emma_put;

	return err;

exit_clk_emma_put:
	clk_disable(pcdev->clk_emma);
	clk_put(pcdev->clk_emma);
exit_free_irq:
	free_irq(pcdev->irq_emma, pcdev);
exit_iounmap:
	iounmap(pcdev->base_emma);
exit_release:
	release_mem_region(res_emma->start, resource_size(res_emma));
out:
	return err;
}

static int mx27_camera_probe(struct platform_device *pdev)
{
	struct mx27_camera_dev *pcdev;
	struct resource *res_csi, *res_emma;
	void __iomem *base_csi;
	unsigned int irq_csi, irq_emma;
	int err = 0;

	dev_info(&pdev->dev, "initialising\n");

	res_csi = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq_csi = platform_get_irq(pdev, 0);
	if (!res_csi || !irq_csi) {
		dev_err(&pdev->dev, "No platform irq\n");
		err = -ENODEV;
		goto exit;
	}

	pcdev = kzalloc(sizeof(*pcdev), GFP_KERNEL);
	if (!pcdev) {
		dev_err(&pdev->dev, "Could not allocate pcdev\n");
		err = -ENOMEM;
		goto exit;
	}

	pcdev->clk_csi = clk_get(&pdev->dev, "csi_perclk");
	if (IS_ERR(pcdev->clk_csi)) {
		err = PTR_ERR(pcdev->clk_csi);
		goto exit_kfree;
	}

	dev_info(&pdev->dev, "Camera clock frequency: %ld\n",
			clk_get_rate(pcdev->clk_csi));

	/* Initialize DMA */
	pcdev->dma = imx_dma_request_by_prio("CSI RX DMA", DMA_PRIO_HIGH);
	if (pcdev->dma < 0) {
		dev_err(&pdev->dev,
			"mxc_v4l_read failed to request DMA channel\n");
		err = -EIO;
		goto exit_clk_put;
	}

	err = imx_dma_setup_handlers(pcdev->dma, mx27_camera_dma_callback,
					mx27_camera_dma_err_callback, pcdev);
	if (err != 0) {
		dev_err(&pdev->dev,
				"mxc_v4l_read failed to set DMA callback\n");
		err = -EIO;
		goto exit_dma_free;
	}

	imx_dma_config_channel(pcdev->dma,
			IMX_DMA_MEMSIZE_32 | IMX_DMA_TYPE_FIFO,
			IMX_DMA_MEMSIZE_32 | IMX_DMA_TYPE_LINEAR,
			DMA_REQ_CSI_RX, 1);

	imx_dma_config_burstlen(pcdev->dma, 64);

	dev_set_drvdata(&pdev->dev, pcdev);
	pcdev->res_csi = res_csi;

	pcdev->pdata = pdev->dev.platform_data;
	pcdev->platform_flags = pcdev->pdata->flags;

	clk_set_rate(pcdev->clk_csi, pcdev->pdata->clk * 2);

	INIT_LIST_HEAD(&pcdev->capture);
	INIT_LIST_HEAD(&pcdev->active_bufs);
	spin_lock_init(&pcdev->lock);

	/*
	 * Request the regions.
	 */
	if (!request_mem_region(res_csi->start, resource_size(res_csi),
				MX27_CAM_DRV_NAME)) {
		err = -EBUSY;
		goto exit_dma_free;
	}

	base_csi = ioremap(res_csi->start, resource_size(res_csi));
	if (!base_csi) {
		err = -ENOMEM;
		goto exit_release;
	}
	pcdev->irq_csi = irq_csi;
	pcdev->base_csi = base_csi;
	pcdev->dev = &pdev->dev;

	pcdev->pdata->init(pdev);

	err = request_irq(pcdev->irq_csi, mx27_camera_irq, 0, MX27_CAM_DRV_NAME,
			  pcdev);
	if (err) {
		dev_err(pcdev->dev, "Camera interrupt register failed \n");
		goto exit_iounmap;
	}

	mx27_soc_camera_host.priv	= pcdev;
	mx27_soc_camera_host.dev.parent	= &pdev->dev;
	mx27_soc_camera_host.nr		= pdev->id;

	/* EMMA support */
	res_emma = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	irq_emma = platform_get_irq(pdev, 1);

	if (res_emma && irq_emma) {
		dev_info(&pdev->dev, "Using EMMA\n");
		pcdev->use_emma = 1;
		pcdev->res_emma = res_emma;
		pcdev->irq_emma = irq_emma;
		if (mx27_camera_emma_init(pcdev))
			goto exit_free_irq;
	}

	err = soc_camera_host_register(&mx27_soc_camera_host);
	if (err)
		goto exit_free_emma;

	return 0;

exit_free_emma:
	free_irq(pcdev->irq_emma, pcdev);
	clk_disable(pcdev->clk_emma);
	clk_put(pcdev->clk_emma);
	iounmap(pcdev->base_emma);
	release_mem_region(res_emma->start, resource_size(res_emma));
exit_free_irq:
	free_irq(pcdev->irq_csi, pcdev);
exit_iounmap:
	iounmap(base_csi);
exit_release:
	release_mem_region(res_csi->start, resource_size(res_csi));
exit_dma_free:
	imx_dma_free(pcdev->dma);
exit_clk_put:
	clk_put(pcdev->clk_csi);
exit_kfree:
	kfree(pcdev);
exit:
	return err;
}

static int __devexit mx27_camera_remove(struct platform_device *pdev)
{
	struct mx27_camera_dev *pcdev = platform_get_drvdata(pdev);
	struct resource *res;

	clk_put(pcdev->clk_csi);
	imx_dma_free(pcdev->dma);
	free_irq(pcdev->irq_csi, pcdev);
	if (mx27_camera_emma(pcdev))
		free_irq(pcdev->irq_emma, pcdev);

	soc_camera_host_unregister(&mx27_soc_camera_host);

	iounmap(pcdev->base_csi);

	if (mx27_camera_emma(pcdev)) {
		clk_disable(pcdev->clk_emma);
		clk_put(pcdev->clk_emma);
		iounmap(pcdev->base_emma);
		res = pcdev->res_emma;
		release_mem_region(res->start, resource_size(res));
	}

	pcdev->pdata->exit(pdev);

	res = pcdev->res_csi;
	release_mem_region(res->start, resource_size(res));

	kfree(pcdev);

	dev_info(&pdev->dev, "MX27 Camera driver unloaded\n");

	return 0;
}

static struct platform_driver mx27_camera_driver = {
	.driver 	= {
		.name	= MX27_CAM_DRV_NAME,
	},
	.probe		= mx27_camera_probe,
	.remove		= __exit_p(mx27_camera_remove),
};


static int __devinit mx27_camera_init(void)
{
	return platform_driver_register(&mx27_camera_driver);
}

static void __exit mx27_camera_exit(void)
{
	return platform_driver_unregister(&mx27_camera_driver);
}

module_init(mx27_camera_init);
module_exit(mx27_camera_exit);

MODULE_DESCRIPTION("i.MX27 SoC Camera Host driver");
MODULE_AUTHOR("Sascha Hauer <sha@pengutronix.de>");
MODULE_LICENSE("GPL");
