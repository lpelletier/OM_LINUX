#
# Makefile for ALSA
#

obj-$(CONFIG_SND_SA11XX_UDA1341) += snd-sa11xx-uda1341.o 
snd-sa11xx-uda1341-objs		:= sa11xx-uda1341.o

obj-$(CONFIG_SND_ARMAACI)	+= snd-aaci.o
snd-aaci-objs			:= aaci.o devdma.o

obj-$(CONFIG_SND_PXA2XX_PCM)	+= snd-pxa2xx-pcm.o
snd-pxa2xx-pcm-objs		:= pxa2xx-pcm.o

obj-$(CONFIG_SND_PXA2XX_LIB)	+= snd-pxa2xx-lib.o
snd-pxa2xx-lib-y		:= pxa2xx-pcm-lib.o
snd-pxa2xx-lib-$(CONFIG_SND_PXA2XX_LIB_AC97)	+= pxa2xx-ac97-lib.o

obj-$(CONFIG_SND_PXA2XX_AC97)	+= snd-pxa2xx-ac97.o
snd-pxa2xx-ac97-objs		:= pxa2xx-ac97.o

#snd-mc13783-objs		:= imx2-pcm.o imx2-ssi.o imx2-dam.o mc13783.o devdma.o
#obj-$(CONFIG_SND_IMX2_PCM)	+= snd-mc13783.o

obj-$(CONFIG_SND_IMX2_PCM)	+= snd-mx2.o
snd-mx2-objs			:= imx2-ssi.o imx2-dam.o

obj-$(CONFIG_SND_IMX_PCM)	+= snd-imx.o
snd-imx-objs			:= imx-ssi.o

obj-$(CONFIG_SND_IMX_TSC2102)	+= snd-imx-alsa-tsc2102.o
snd-imx-alsa-tsc2102-objs	:= imx-alsa-tsc2102.o imx-alsa-tsc2102-mixer.o imx-alsa.o devdma.o

