#
# Makefile for MMC/SD host controller drivers
#

ifneq ($(CONFIG_MMC_BLOCK_BOUNCE),y)
obj-$(CONFIG_MSC0_JZ4750)	+= jz_mmc_dma.o
obj-$(CONFIG_MSC1_JZ4750)	+= jz_mmc_dma.o
obj-$(CONFIG_MSC0_JZ4760)	+= jz_mmc_dma.o
obj-$(CONFIG_MSC1_JZ4760)	+= jz_mmc_dma.o
obj-$(CONFIG_MSC0_JZ4770)	+= jz_mmc_dma_nonbounce.o
obj-$(CONFIG_MSC1_JZ4770)	+= jz_mmc_dma_nonbounce.o
obj-$(CONFIG_MSC2_JZ4770)	+= jz_mmc_dma_nonbounce.o
else
obj-$(CONFIG_MSC0_JZ4750)	+= jz_mmc_dma.o
obj-$(CONFIG_MSC1_JZ4750)	+= jz_mmc_dma.o
obj-$(CONFIG_MSC0_JZ4760)	+= jz_mmc_dma.o
obj-$(CONFIG_MSC1_JZ4760)	+= jz_mmc_dma.o
obj-$(CONFIG_MSC0_JZ4770)	+= jz_mmc_dma.o
obj-$(CONFIG_MSC1_JZ4770)	+= jz_mmc_dma.o
obj-$(CONFIG_MSC2_JZ4770)	+= jz_mmc_dma.o
endif