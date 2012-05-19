#
# Makefile for the Ingenic JZ4770.
#

# Object file lists.

obj-y += prom.o irq.o time.o reset.o setup.o dma.o \
	platform.o cpm.o fpu.o

obj-$(CONFIG_PROC_FS)		+= proc.o

obj-$(CONFIG_NAND_DEVICE_ID)	+= board_device_id.o


# PM support

obj-$(CONFIG_PM)         += pm.o
obj-$(CONFIG_PM_POWERDOWN_P0) += sleep.o

# CPU Frequency scaling support

obj-$(CONFIG_CPU_FREQ)       += cpufreq.o

obj-$(CONFIG_ANDROID_TIMED_GPIO)	+= gpiolib.o
