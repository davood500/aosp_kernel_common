/*
 * chip-intc.h
 * JZ4760 INTC register definition
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 *
 * Author: cjwang@ingenic.cn
 */

#ifndef __CHIP_INTC_H__
#define __CHIP_INTC_H__

/*
 * Interrupt controller module(INTC) address definition
 */
#define	INTC_BASE	0xB0001000


/*
 * INTC registers offset address definition
 */
#define INTC_ICSR_OFFSET	(0x00)	/* 32, r,  0x00000000 */
#define INTC_ICMR_OFFSET	(0x04)	/* 32, rw, 0xffffffff */
#define INTC_ICMSR_OFFSET	(0x08)	/* 32, w,  0x???????? */
#define INTC_ICMCR_OFFSET	(0x0c)	/* 32, w,  0x???????? */
#define INTC_ICPR_OFFSET	(0x10)	/* 32, r,  0x00000000 */

/* INTC groups offset */
#define INTC_GOS		0x20


/*
 * INTC registers address definition
 */
#define INTC_ICSR(n)	(INTC_BASE + (n) * INTC_GOS + INTC_ICSR_OFFSET)
#define INTC_ICMR(n)	(INTC_BASE + (n) * INTC_GOS + INTC_ICMR_OFFSET)
#define INTC_ICMSR(n)	(INTC_BASE + (n) * INTC_GOS + INTC_ICMSR_OFFSET)
#define INTC_ICMCR(n)	(INTC_BASE + (n) * INTC_GOS + INTC_ICMCR_OFFSET)
#define INTC_ICPR(n)	(INTC_BASE + (n) * INTC_GOS + INTC_ICPR_OFFSET)


/*
 * INTC registers common define
 */

/* 1st-level interrupts */
#define IRQ_I2C1	0
#define IRQ_I2C0	1
#define IRQ_UART3	2
#define IRQ_UART2	3
#define IRQ_UART1	4
#define IRQ_UART0	5
#define IRQ_GPU		6
#define IRQ_SSI1	7
#define IRQ_SSI0	8
#define IRQ_TSSI	9
#define IRQ_BDMA	10
#define IRQ_KBC		11
#define IRQ_GPIO5	12
#define IRQ_GPIO4	13
#define IRQ_GPIO3	14
#define IRQ_GPIO2	15
#define IRQ_GPIO1	16
#define IRQ_GPIO0	17
#define IRQ_SADC	18
#define IRQ_ETH		19
#define IRQ_UHC		20
#define IRQ_OTG		21
#define IRQ_I2C2	22
#define IRQ_DMAC1	23
#define IRQ_DMAC0	24
#define IRQ_TCU2	25
#define IRQ_TCU1	26
#define IRQ_TCU0	27
#define IRQ_GPS		28
#define IRQ_IPU		29
#define IRQ_CIM		30
#define IRQ_LCD		31

#define IRQ_RTC		32	/* 32 + 0 */
#define IRQ_OWI		33	/* 32 + 1 */
#define IRQ_AIC 	34	/* 32 + 2 */
#define IRQ_MSC2	35	/* 32 + 3 */
#define IRQ_MSC1	36	/* 32 + 4 */
#define IRQ_MSC0	37	/* 32 + 5 */
#define IRQ_SCC		38	/* 32 + 6 */
#define IRQ_BCH		39	/* 32 + 7 */
#define IRQ_PCM0	40	/* 32 + 8 */
#define IRQ_PCM1	41      /* 32 + 9 */
#define IRQ_UART4	42	/* 32 + 10 */
#define IRQ_AOSD	43	/* 32 + 11 */
#define IRQ_ARB2	44	/* 32 + 12 */
#define IRQ_I2S2CH	45      /* 32 + 13 */
#define IRQ_DONOTUSE    46      /* RESERVED */   
#define IRQ_CPM		47	/* 32 + 15 */
#define IRQ_VPU		48
#define IRQ_INTC_MAX	49

/* 2nd-level interrupts */
#define NUM_INTC		49
#define NUM_DMA         	MAX_DMA_NUM
#define NUM_BDMA         	MAX_BDMA_NUM
#define NUM_GPIO        	MAX_GPIO_NUM

#define IRQ_DMA_BASE	(IRQ_INTC_MAX)
#define IRQ_DMA_MAX	(IRQ_DMA_BASE + NUM_DMA)

#define IRQ_BDMA_BASE	(IRQ_DMA_MAX)
#define IRQ_BDMA_MAX	(IRQ_BDMA_BASE + NUM_BDMA)

/* To be cleanup begin */
#define IRQ_DMA_0	50
#define IRQ_DMA_1	(IRQ_DMA_0 + HALF_DMA_NUM)	/* 50 +  6 = 56 */
#define IRQ_BDMA_0	(IRQ_DMA_0 + MAX_DMA_NUM)	/* 50 + 12 = 62 */

#define IRQ_GPIO_0		64  /* 64 to (64+MAX_GPIO_NUM-1) for GPIO pin 0 to MAX_GPIO_NUM-1 */

/* To be cleanup end */

#ifndef __MIPS_ASSEMBLER

#define REG_INTC_ICMR(n)	REG32(INTC_ICMR(n))
#define REG_INTC_ICMSR(n)	REG32(INTC_ICMSR(n))
#define REG_INTC_ICMCR(n)	REG32(INTC_ICMCR(n))
#define REG_INTC_ICPR(n)	REG32(INTC_ICPR(n))

#define __intc_unmask_irq(n)	(REG_INTC_ICMCR((n)/32) = (1 << ((n)%32)))
#define __intc_mask_irq(n)	(REG_INTC_ICMSR((n)/32) = (1 << ((n)%32)))
#define __intc_ack_irq(n)


#endif /* __MIPS_ASSEMBLER */

#endif /* __CHIP_INTC_H__ */
