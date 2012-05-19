/*
 * linux/arch/mips/jz4760/irq.c
 *
 * JZ4760 interrupt routines.
 *
 * Copyright (c) 2006-2007  Ingenic Semiconductor Inc.
 * Author: <lhhuang@ingenic.cn>
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 */
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/kernel_stat.h>
#include <linux/module.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/timex.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/sysdev.h>

#include <asm/bootinfo.h>
#include <asm/io.h>
#include <asm/mipsregs.h>
#include <asm/system.h>
#include <asm/jzsoc.h>

static unsigned long jz_intc_wakeup[2];
static unsigned long jz_intc_saved[2];
static unsigned long jz_gpio_wakeup[6];
static unsigned long cpuflags;

/*
 * INTC irq type
 */

#define enable_vpu_irq() do{						\
  unsigned int val;							\
  unsigned long cpuflags;						\
  local_irq_save(cpuflags);						\
  val = read_c0_status();						\
  val |= 0x800;								\
  write_c0_status(val);							\
  local_irq_restore(cpuflags);						\
}while(0)
   
#define disable_vpu_irq() do{						\
     unsigned int val;							\
     unsigned long cpuflags;						\
     local_irq_save(cpuflags);						\
     val = read_c0_status();						\
     val &= ~0x800;							\
     write_c0_status(val);						\
     local_irq_restore(cpuflags);					\
   }while(0)

static inline void __set_wake_intc(int irq,int on)
{
	if(on) {
		if(irq > 31) {
			jz_intc_wakeup[1] |= 1 << (irq - 32);
			return;
		}
		jz_intc_wakeup[0] |= 1 << irq;
	} else {
		if(irq > 31) {
			jz_intc_wakeup[1] &= ~(1 << (irq - 32));
			return;	
		}
		jz_intc_wakeup[0] &= ~(1 << irq);
	}
}

static inline void __enable_intc_irq(unsigned int irq)
{
	if(irq < IRQ_INTC_MAX - 1)
		__intc_unmask_irq(irq);
	else if(irq == IRQ_INTC_MAX - 1) {
      		enable_vpu_irq();
	}
}

static inline void __disable_intc_irq(unsigned int irq)
{
	if(irq < IRQ_INTC_MAX - 1)
		__intc_mask_irq(irq);
	else if(irq == IRQ_INTC_MAX - 1) {
		disable_vpu_irq();
	}
}

static void disable_intc_irq(struct irq_data *data)
{
	__disable_intc_irq(data->irq);
}

static void enable_intc_irq(struct irq_data *data)
{
	__enable_intc_irq(data->irq);
}

static void mask_and_ack_intc_irq(struct irq_data *data)
{
	if(data->irq < IRQ_INTC_MAX - 1) {
		__intc_mask_irq(data->irq);
		__intc_ack_irq(data->irq);
	} else if(data->irq == IRQ_INTC_MAX - 1) {
		disable_vpu_irq();
	}
}

static unsigned int startup_intc_irq(struct irq_data *data)
{
	__enable_intc_irq(data->irq);
	return 0;
}

static void shutdown_intc_irq(struct irq_data *data)
{
	__disable_intc_irq(data->irq);
}

static int set_wake_intc(struct irq_data *data, unsigned int on)
{
	__set_wake_intc(data->irq,on);
	return 0;
}

static struct irq_chip intc_irq_type = {
	.name = "INTC",
	.irq_startup = startup_intc_irq,
	.irq_shutdown = shutdown_intc_irq,
	.irq_enable = enable_intc_irq,
	.irq_disable = disable_intc_irq,
	.irq_unmask = enable_intc_irq,
	.irq_mask = disable_intc_irq,
	.irq_mask_ack = mask_and_ack_intc_irq,
	.irq_set_wake = set_wake_intc,
};

/*
 * GPIO irq type
 */

static void enable_gpio_irq(struct irq_data *data)
{
	unsigned int intc_irq;
	unsigned int irq = data->irq;

	if (irq < (IRQ_GPIO_0 + 32)) {
		intc_irq = IRQ_GPIO0;
	}
	else if (irq < (IRQ_GPIO_0 + 64)) {
		intc_irq = IRQ_GPIO1;
	}
	else if (irq < (IRQ_GPIO_0 + 96)) {
		intc_irq = IRQ_GPIO2;
	}
	else if (irq < (IRQ_GPIO_0 + 128)) {
		intc_irq = IRQ_GPIO3;
	}
	else if (irq < (IRQ_GPIO_0 + 160)) {
		intc_irq = IRQ_GPIO4;
	}
	else {
		intc_irq = IRQ_GPIO5;
	}

	__enable_intc_irq(intc_irq);
	__gpio_unmask_irq(irq - IRQ_GPIO_0);
}

static void disable_gpio_irq(struct irq_data *data)
{
	__gpio_mask_irq(data->irq - IRQ_GPIO_0);
}

static void mask_and_ack_gpio_irq(struct irq_data *data)
{
	__gpio_mask_irq(data->irq - IRQ_GPIO_0);
	__gpio_ack_irq(data->irq - IRQ_GPIO_0);
}

static unsigned int startup_gpio_irq(struct irq_data *data)
{
	enable_gpio_irq(data);
	return 0;
}

static void shutdown_gpio_irq(struct irq_data *data)
{
	disable_gpio_irq(data);
}

static int set_wake_gpio(struct irq_data *data, unsigned int on)
{
	unsigned int irq = data->irq;

	__set_wake_intc(IRQ_GPIO0 - (irq - IRQ_GPIO_0)/32,on);

	if(on)
		jz_gpio_wakeup[irq/32] |= 1 << (irq%32);
	else
		jz_gpio_wakeup[irq/32] &= ~(1 << (irq%32));

	return 0;
}

static struct irq_chip gpio_irq_type = {
	.name = "GPIO",
	.irq_startup = startup_gpio_irq,
	.irq_shutdown = shutdown_gpio_irq,
	.irq_enable = enable_gpio_irq,
	.irq_disable = disable_gpio_irq,
	.irq_unmask = enable_gpio_irq,
	.irq_mask = disable_gpio_irq,
	.irq_mask_ack = mask_and_ack_gpio_irq,
	.irq_set_wake = set_wake_gpio,
};

/*
 * DMA irq type
 */

static void enable_dma_irq(struct irq_data *data)
{
	unsigned int intc_irq;
	unsigned int irq = data->irq;

	if ( irq < (IRQ_DMA_0 + HALF_DMA_NUM) ) 	/* DMAC Group 0 irq */
		intc_irq = IRQ_DMAC0;
	else if ( irq < (IRQ_DMA_0 + MAX_DMA_NUM) ) 	/* DMAC Group 1 irq */
		intc_irq = IRQ_DMAC1;
	else {
		printk("%s, unexpected dma irq #%d\n", __FILE__, irq);
		return;
	}
	__intc_unmask_irq(intc_irq);
	__dmac_channel_enable_irq(data->irq - IRQ_DMA_0);
}

static void disable_dma_irq(struct irq_data *data)
{
	__dmac_channel_disable_irq(data->irq - IRQ_DMA_0);
}

static void mask_and_ack_dma_irq(struct irq_data *data)
{
	unsigned int intc_irq;
	unsigned int irq = data->irq;

	if ( irq < (IRQ_DMA_0 + HALF_DMA_NUM) ) 	/* DMAC Group 0 irq */
		intc_irq = IRQ_DMAC0;
	else if ( irq < (IRQ_DMA_0 + MAX_DMA_NUM) ) 	/* DMAC Group 1 irq */
		intc_irq = IRQ_DMAC1;
	else {
		printk("%s, unexpected dma irq #%d\n", __FILE__, irq);
		return ;
	}
	__intc_ack_irq(intc_irq);
	__dmac_channel_ack_irq(irq-IRQ_DMA_0); /* needed?? add 20080506, Wolfgang */
	__dmac_channel_disable_irq(irq - IRQ_DMA_0);
}

static unsigned int startup_dma_irq(struct irq_data *data)
{
	enable_dma_irq(data);
	return 0;
}

static void shutdown_dma_irq(struct irq_data *data)
{
	disable_dma_irq(data);
}

static struct irq_chip dma_irq_type = {
	.name = "DMA",
	.irq_startup = startup_dma_irq,
	.irq_shutdown = shutdown_dma_irq,
	.irq_enable = enable_dma_irq,
	.irq_disable = disable_dma_irq,
	.irq_unmask = enable_dma_irq,
	.irq_mask = disable_dma_irq,
	.irq_mask_ack = mask_and_ack_dma_irq,
};

/*
 * BDMA irq type
 */

static void enable_bdma_irq(struct irq_data *data)
{
	unsigned int intc_irq;
	unsigned int irq = data->irq;

	if (irq < IRQ_BDMA_0 + MAX_BDMA_NUM)
		intc_irq = IRQ_BDMA;
	else {
		printk("%s, unexpected bdma irq #%d\n", __FILE__, irq);
		return;
	}
	__intc_unmask_irq(intc_irq);
	__bdmac_channel_enable_irq(irq - IRQ_BDMA_0);
}

static void disable_bdma_irq(struct irq_data *data)
{
	__bdmac_channel_disable_irq(data->irq - IRQ_BDMA_0);
}

static void mask_and_ack_bdma_irq(struct irq_data *data)
{
	unsigned int intc_irq;
	unsigned int irq = data->irq;

	if ( irq < IRQ_BDMA_0 + MAX_BDMA_NUM ) 	/* DMAC Group 0 irq */
		intc_irq = IRQ_BDMA;
	else {
		printk("%s, unexpected bdma irq #%d\n", __FILE__, irq);
		return ;
	}
	__intc_ack_irq(intc_irq);
	__bdmac_channel_ack_irq(irq - IRQ_BDMA_0);
	__bdmac_channel_disable_irq(irq - IRQ_BDMA_0);
}

static unsigned int startup_bdma_irq(struct irq_data *data)
{
	enable_bdma_irq(data);
	return 0;
}

static void shutdown_bdma_irq(struct irq_data *data)
{
	disable_bdma_irq(data);
}

static struct irq_chip bdma_irq_type = {
	.name = "BDMA",
	.irq_startup = startup_bdma_irq,
	.irq_shutdown = shutdown_bdma_irq,
	.irq_enable = enable_bdma_irq,
	.irq_disable = disable_bdma_irq,
	.irq_unmask = enable_bdma_irq,
	.irq_mask = disable_bdma_irq,
	.irq_mask_ack = mask_and_ack_bdma_irq,
};

void __init arch_init_irq(void)
{
	int i;

	clear_c0_status(0xff04); /* clear ERL */
	set_c0_status(0x0400);   /* set IP2 */

	/* Set up INTC irq
	 */
	for (i = 0; i < NUM_INTC; i++) {
		__disable_intc_irq(i);
		irq_set_chip_and_handler(i, &intc_irq_type, handle_level_irq);
	}

	/* Set up DMAC irq
	 */
	for (i = 0; i < NUM_DMA; i++) {
		__dmac_channel_disable_irq(i);
		irq_set_chip_and_handler(IRQ_DMA_0 + i, &dma_irq_type, handle_level_irq);
	}

	/* Set up BDMA irq
	 */
	for (i = 0; i < MAX_BDMA_NUM; i++) {
		__bdmac_channel_disable_irq(i);
		irq_set_chip_and_handler(IRQ_BDMA_0 + i, &bdma_irq_type, handle_level_irq);
	}

	/* Set up GPIO irq
	 */
	for (i = 0; i < NUM_GPIO; i++) {
		if (__gpio_is_irq(i))
			__gpio_mask_irq(i);
		irq_set_chip_and_handler(IRQ_GPIO_0 + i, &gpio_irq_type, handle_level_irq);
	}
}

static int plat_real_irq(int irq)
{
	switch (irq) {
		case IRQ_DMAC0:
		case IRQ_DMAC1:
			irq = __dmac_get_irq() + IRQ_DMA_0;
			break;
		case IRQ_BDMA:
			irq = __bdmac_get_irq() + IRQ_BDMA_0;
			break;
	}

	return irq;
}

asmlinkage void plat_irq_dispatch(void)
{
	int irq = 0, group;

	unsigned long intc_ipr0 = 0, intc_ipr1 = 0, vpu_pending = 0;

	intc_ipr0 = REG_INTC_ICPR(0);
	intc_ipr1 = REG_INTC_ICPR(1) & 0xffff;

	if (intc_ipr0) {
		irq = fls(intc_ipr0) - 1;
		intc_ipr0 &= ~(1<<irq);
	} else if(intc_ipr1){
		irq = fls(intc_ipr1) - 1;
		intc_ipr1 &= ~(1<<irq);
		irq += 32;
	} else {
		__asm__ __volatile__ (
				"mfc0  %0, $13,  0   \n\t"
				"nop                  \n\t"
				:"=r"(vpu_pending)
				:);

		if(vpu_pending & 0x800)
			irq = IRQ_VPU;
		else
			return;
	}

	if ((irq >= IRQ_GPIO5) && (irq <= IRQ_GPIO0)) {
		group = IRQ_GPIO0 - irq;
		irq = __gpio_group_irq(group);
		if (irq < 0) {
			return;
		}
		irq += IRQ_GPIO_0 + 32 * group;
	} else {
		irq = plat_real_irq(irq);
	}

	do_IRQ(irq);
}

void jz4770_irq_suspend(void)
{
	jz_intc_saved[0] = INREG32(INTC_ICMSR(0));
	jz_intc_saved[1] = INREG32(INTC_ICMSR(1));

	OUTREG32(INTC_ICMSR(0), 0xffffffff & ~jz_intc_wakeup[0]);
	OUTREG32(INTC_ICMSR(1), 0xffff & ~jz_intc_wakeup[1]);

	REG_GPIO_PXMSKC(0) = jz_gpio_wakeup[0];
	REG_GPIO_PXMSKC(1) = jz_gpio_wakeup[1];
	REG_GPIO_PXMSKC(2) = jz_gpio_wakeup[2];
	REG_GPIO_PXMSKC(3) = jz_gpio_wakeup[3];
	REG_GPIO_PXMSKC(4) = jz_gpio_wakeup[4];
	REG_GPIO_PXMSKC(5) = jz_gpio_wakeup[5];

	/* Save CPU irqs */
	local_irq_save(cpuflags);
}

void jz4770_irq_resume(void)
{
	OUTREG32(INTC_ICMR(0), jz_intc_saved[0]);
	OUTREG32(INTC_ICMR(1), jz_intc_saved[1]);
	/* Restore CPU interrupt flags */
	local_irq_restore(cpuflags);
}


