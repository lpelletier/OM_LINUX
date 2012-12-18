/*
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
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

#ifndef __SOUND_ARM_IMX2_SSI_H
#define __SOUND_ARM_IMX2_SSI_H

int imx_ssi_prepare(struct snd_pcm_substream *substream, int ssi_id);
void imx_ssi_shutdown(struct snd_pcm_substream *substream, int ssi_id);
int imx_ssi_trigger(struct snd_pcm_substream *substream, int cmd, int ssi_id);

extern int imx_ssi_get_dma_tx_channel(int, int);
extern int imx_ssi_get_dma_rx_channel(int, int);
extern void __iomem *imx_ssi_get_dma_tx_address(int, int);
extern void __iomem *imx_ssi_get_dma_rx_address(int, int);
extern int imx_ssi_setup_unit_to_iis_slave(int);

/* to handle it like a module */
extern int mx2_ssi_init(void);
extern void mx2_ssi_exit(void);

/* possible DMA bursts in FIFO entries */
#define DMA_TXFIFO_BURST	0x6
#define DMA_RXFIFO_BURST	0x4

#endif /* __SOUND_ARM_IMX2_SSI_H */
