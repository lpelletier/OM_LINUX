#
# Makefile for the linux kernel.
#

# Object file lists.

obj-y	:=  system.o generic.o devices.o serial.o

obj-$(CONFIG_MACH_MX27) += cpu_imx27.o
obj-$(CONFIG_MACH_MX27) += clock_imx27.o
obj-$(CONFIG_CPU_FREQ_IMX27) +=  cpufreq_imx27.o
obj-$(CONFIG_PM) += pm.o

obj-$(CONFIG_MACH_APF27) += apf27.o
obj-$(CONFIG_MACH_APF27_DEV) += apf27-dev.o
obj-$(CONFIG_MACH_APF27_DEV) += apf27-lcd.o
obj-$(CONFIG_MACH_APF27_DEV_EXT_WIRELESS) += apf27-dev-ext-apw.o
obj-$(CONFIG_MACH_PPS) += pps.o
obj-$(CONFIG_MACH_MX27ADS) += mx27ads.o
obj-$(CONFIG_MACH_PCM038) += pcm038.o
obj-$(CONFIG_MACH_PCM970_BASEBOARD) += pcm970-baseboard.o
