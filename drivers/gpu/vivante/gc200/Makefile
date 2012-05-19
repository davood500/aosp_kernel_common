
OS_KERNEL_DIR   := hal/os/linux/kernel
ARCH_KERNEL_DIR := arch/XAQ2/hal/kernel
HAL_KERNEL_DIR  := hal/kernel

VIV_DRIVER_PATH := drivers/gpu/vivante/gc200

# GC200 galcore config
EXTRA_CFLAGS += -Werror -DLINUX -DDRIVER -DENUM_WORKAROUND=0 -DDBG=0 -DNO_DMA_COHERENT -DENABLE_ARM_L2_CACHE=0 -DUSE_PLATFORM_DRIVER=1 -DVIVANTE_PROFILER=0 -DANDROID=1 -DENABLE_GPU_CLOCK_BY_DRIVER=0 -DUSE_NEW_LINUX_SIGNAL=0  -I $(VIV_DRIVER_PATH)/hal/inc -I $(VIV_DRIVER_PATH)/hal/kernel -I $(VIV_DRIVER_PATH)/arch/XAQ2/hal/kernel -I $(VIV_DRIVER_PATH)/hal/user/

obj-y += $(OS_KERNEL_DIR)/Driver.o \
        $(OS_KERNEL_DIR)/Device.o \
        $(OS_KERNEL_DIR)/Debug.o \
        $(OS_KERNEL_DIR)/Kernel.o \
        $(OS_KERNEL_DIR)/Os.o


obj-y += $(HAL_KERNEL_DIR)/Command.o \
        $(HAL_KERNEL_DIR)/Event.o \
        $(HAL_KERNEL_DIR)/Heap.o \
        $(HAL_KERNEL_DIR)/Kernel.o \
        $(HAL_KERNEL_DIR)/Mmu.o \
        $(HAL_KERNEL_DIR)/VidMem.o

obj-y += $(ARCH_KERNEL_DIR)/Hardware.o

