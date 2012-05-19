/*
 * chip-bch.h
 * JZ4760 BCH register definition
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 *
 * Author: xqjia@ingenic.cn
 */

#ifndef __CHIP_BCH_H__
#define __CHIP_BCH_H__


/*
 * Bose-Chaudhuri-Hocquenghem controller module(BCH) address definition
 */
#define BCH_BASE	0xb34d0000


/*
 * BCH registers offset addresses definition
 */
#define	BCH_BHCR_OFFSET		(0x00)	/*  r, 32, 0x00000000 */
#define	BCH_BHCSR_OFFSET	(0x04)	/*  w, 32, 0x???????? */
#define	BCH_BHCCR_OFFSET	(0x08)	/*  w, 32, 0x???????? */
#define	BCH_BHCNT_OFFSET	(0x0c)	/* rw, 32, 0x00000000 */
#define	BCH_BHDR_OFFSET		(0x10)	/*  w,  8, 0x???????? */
#define	BCH_BHPAR0_OFFSET	(0x14)	/* rw, 32, 0x00000000 */
#define	BCH_BHPAR1_OFFSET	(0x18)	/* rw, 32, 0x00000000 */
#define	BCH_BHPAR2_OFFSET	(0x1c)	/* rw, 32, 0x00000000 */
#define	BCH_BHPAR3_OFFSET	(0x20)	/* rw, 32, 0x00000000 */
#define	BCH_BHPAR4_OFFSET	(0x24)	/* rw, 32, 0x00000000 */
#define	BCH_BHPAR5_OFFSET	(0x28)	/* rw, 32, 0x00000000 */
#define	BCH_BHPAR6_OFFSET	(0x2c)	/* rw, 32, 0x00000000 */
#define	BCH_BHPAR7_OFFSET	(0x30)	/* rw, 32, 0x00000000 */
#define	BCH_BHPAR8_OFFSET	(0x34)	/* rw, 32, 0x00000000 */
#define	BCH_BHPAR9_OFFSET	(0x38)	/* rw, 32, 0x00000000 */
#define	BCH_BHERR0_OFFSET	(0x3c)	/*  r, 32, 0x00000000 */
#define	BCH_BHERR1_OFFSET	(0x40)	/*  r, 32, 0x00000000 */
#define	BCH_BHERR2_OFFSET	(0x44)	/*  r, 32, 0x00000000 */
#define	BCH_BHERR3_OFFSET	(0x48)	/*  r, 32, 0x00000000 */
#define	BCH_BHERR4_OFFSET	(0x4c)	/*  r, 32, 0x00000000 */
#define	BCH_BHERR5_OFFSET	(0x50)	/*  r, 32, 0x00000000 */
#define	BCH_BHERR6_OFFSET	(0x54)	/*  r, 32, 0x00000000 */
#define	BCH_BHERR7_OFFSET	(0x58)	/*  r, 32, 0x00000000 */
#define	BCH_BHERR8_OFFSET	(0x5c)	/*  r, 32, 0x00000000 */
#define	BCH_BHERR9_OFFSET	(0x60)	/*  r, 32, 0x00000000 */
#define	BCH_BHERR10_OFFSET	(0x64)	/*  r, 32, 0x00000000 */
#define	BCH_BHERR11_OFFSET	(0x68)	/*  r, 32, 0x00000000 */
#define	BCH_BHINT_OFFSET	(0x6c)	/*  r, 32, 0x00000000 */
#define	BCH_BHINTE_OFFSET	(0x70)	/* rw, 32, 0x00000000 */
#define	BCH_BHINTES_OFFSET	(0x74)	/*  w, 32, 0x???????? */
#define	BCH_BHINTEC_OFFSET	(0x78)	/*  w, 32, 0x???????? */


/*
 * BCH registers addresses definition
 */
#define	BCH_BHCR        (BCH_BASE + BCH_BHCR_OFFSET)
#define	BCH_BHCSR       (BCH_BASE + BCH_BHCSR_OFFSET)
#define	BCH_BHCCR       (BCH_BASE + BCH_BHCCR_OFFSET)
#define	BCH_BHCNT    	(BCH_BASE + BCH_BHCNT_OFFSET)
#define	BCH_BHDR     	(BCH_BASE + BCH_BHDR_OFFSET)
#define	BCH_BHPAR0    	(BCH_BASE + BCH_BHPAR0_OFFSET)
#define	BCH_BHPAR1    	(BCH_BASE + BCH_BHPAR1_OFFSET)
#define	BCH_BHPAR2    	(BCH_BASE + BCH_BHPAR2_OFFSET)
#define	BCH_BHPAR3    	(BCH_BASE + BCH_BHPAR3_OFFSET)
#define	BCH_BHPAR4    	(BCH_BASE + BCH_BHPAR4_OFFSET)
#define	BCH_BHPAR5    	(BCH_BASE + BCH_BHPAR5_OFFSET)
#define	BCH_BHPAR6    	(BCH_BASE + BCH_BHPAR6_OFFSET)
#define	BCH_BHPAR7    	(BCH_BASE + BCH_BHPAR7_OFFSET)
#define	BCH_BHPAR8    	(BCH_BASE + BCH_BHPAR8_OFFSET)
#define	BCH_BHPAR9    	(BCH_BASE + BCH_BHPAR9_OFFSET)
#define	BCH_BHERR0      (BCH_BASE + BCH_BHERR0_OFFSET)
#define	BCH_BHERR1      (BCH_BASE + BCH_BHERR1_OFFSET)
#define	BCH_BHERR2      (BCH_BASE + BCH_BHERR2_OFFSET)
#define	BCH_BHERR3      (BCH_BASE + BCH_BHERR3_OFFSET)
#define	BCH_BHERR4      (BCH_BASE + BCH_BHERR4_OFFSET)
#define	BCH_BHERR5      (BCH_BASE + BCH_BHERR5_OFFSET)
#define	BCH_BHERR6      (BCH_BASE + BCH_BHERR6_OFFSET)
#define	BCH_BHERR7      (BCH_BASE + BCH_BHERR7_OFFSET)
#define	BCH_BHERR8      (BCH_BASE + BCH_BHERR8_OFFSET)
#define	BCH_BHERR9      (BCH_BASE + BCH_BHERR9_OFFSET)
#define	BCH_BHERR10     (BCH_BASE + BCH_BHERR10_OFFSET)
#define	BCH_BHERR11     (BCH_BASE + BCH_BHERR11_OFFSET)
#define	BCH_BHINT    	(BCH_BASE + BCH_BHINT_OFFSET)
#define	BCH_BHINTES     (BCH_BASE + BCH_BHINTES_OFFSET)
#define	BCH_BHINTEC     (BCH_BASE + BCH_BHINTEC_OFFSET)
#define	BCH_BHINTE	(BCH_BASE + BCH_BHINTE_OFFSET)


/*
 * BCH registers common define
 */

/* BCH control register (BHCR) */
#define	BHCR_DMAE		BIT7  /* BCH DMA enable */
#define BHCR_ENCE		BIT2
#define	BHCR_BRST		BIT1  /* BCH reset */
#define	BHCR_BCHE		BIT0  /* BCH enable */

#define	BHCR_BSEL_LSB		3
#define	BHCR_BSEL_MASK		BITS_H2L(5, BHCR_BSEL_LSB)
 #define BHCR_BSEL(n)		(((n)/4 - 1) << BHCR_BSEL_LSB)	/* n = 4, 8, 12, 16, 20, 24 */

/* BCH interrupt status register (BHINT) */
#define BHINT_ALL_F		BIT4
#define	BHINT_DECF		BIT3
#define	BHINT_ENCF		BIT2
#define	BHINT_UNCOR		BIT1
#define	BHINT_ERR		BIT0

#define	BHINT_ERRC_LSB		27
#define	BHINT_ERRC_MASK		BITS_H2L(31, BHINT_ERRC_LSB)

/* BCH ENC/DEC count register (BHCNT) */
#define BHCNT_DEC_LSB		16
#define BHCNT_DEC_MASK		BITS_H2L(26, BHCNT_DEC_LSB)

#define BHCNT_ENC_LSB		0
#define BHCNT_ENC_MASK		BITS_H2L(10, BHCNT_ENC_LSB)

/* BCH error report register (BCHERR)*/
#define BCH_ERR_INDEX_LSB	0
#define BCH_ERR_INDEX_MASK	BITS_H2L(12, BCH_ERR_INDEX_LSB)


/* BCH common macro define */
#define BCH_ENCODE		1
#define BCH_DECODE		0


#ifndef __MIPS_ASSEMBLER

#define	REG_BCH_BHCR		REG32(BCH_BHCR)
#define	REG_BCH_BHCSR		REG32(BCH_BHCSR)
#define	REG_BCH_BHCCR		REG32(BCH_BHCCR)
#define	REG_BCH_BHCNT		REG32(BCH_BHCNT)
#define	REG_BCH_BHDR		REG8(BCH_BHDR)
#define	REG_BCH_BHPAR0		REG32(BCH_BHPAR0)
#define	REG_BCH_BHPAR1		REG32(BCH_BHPAR1)
#define	REG_BCH_BHPAR2		REG32(BCH_BHPAR2)
#define	REG_BCH_BHPAR3		REG32(BCH_BHPAR3)
#define	REG_BCH_BHPAR4		REG32(BCH_BHPAR4)
#define	REG_BCH_BHPAR5		REG32(BCH_BHPAR5)
#define	REG_BCH_BHPAR6		REG32(BCH_BHPAR6)
#define	REG_BCH_BHPAR7		REG32(BCH_BHPAR7)
#define	REG_BCH_BHPAR8		REG32(BCH_BHPAR8)
#define	REG_BCH_BHPAR9		REG32(BCH_BHPAR9)
#define	REG_BCH_BHERR0		REG32(BCH_BHERR0)
#define	REG_BCH_BHERR1		REG32(BCH_BHERR1)
#define	REG_BCH_BHERR2		REG32(BCH_BHERR2)
#define	REG_BCH_BHERR3		REG32(BCH_BHERR3)
#define	REG_BCH_BHERR4		REG32(BCH_BHERR4)
#define	REG_BCH_BHERR5		REG32(BCH_BHERR5)
#define	REG_BCH_BHERR6		REG32(BCH_BHERR6)
#define	REG_BCH_BHERR7		REG32(BCH_BHERR7)
#define	REG_BCH_BHERR8		REG32(BCH_BHERR8)
#define	REG_BCH_BHERR9		REG32(BCH_BHERR9)
#define	REG_BCH_BHERR10		REG32(BCH_BHERR10)
#define	REG_BCH_BHERR11		REG32(BCH_BHERR11)
#define	REG_BCH_BHINT		REG32(BCH_BHINT)
#define	REG_BCH_BHINTE		REG32(BCH_BHINTE)
#define	REG_BCH_BHINTEC		REG32(BCH_BHINTEC)
#define	REG_BCH_BHINTES		REG32(BCH_BHINTES)

#define __ecc_enable(encode, bit)			\
do {							\
	unsigned int tmp = BHCR_BRST | BHCR_BCHE;	\
	if (encode)					\
		tmp |= BHCR_ENCE;			\
	tmp |= BHCR_BSEL(bit);				\
	REG_BCH_BHCSR = tmp;				\
	REG_BCH_BHCCR = ~tmp;				\
} while (0)
#define __ecc_disable()		(REG_BCH_BHCCR = BHCR_BCHE)

#define __ecc_dma_enable()	(REG_BCH_BHCSR = BHCR_DMAE)
#define __ecc_dma_disable()	(REG_BCH_BHCCR = BHCR_DMAE)

#define __ecc_cnt_enc(n)	CMSREG32(BCH_BHCNT, (n) << BHCNT_ENC_LSB, BHCNT_ENC_MASK)
#define __ecc_cnt_dec(n)	CMSREG32(BCH_BHCNT, (n) << BHCNT_DEC_LSB, BHCNT_DEC_MASK)

#define __ecc_encode_sync()				\
do {							\
	unsigned int i = 1;				\
	while (!(REG_BCH_BHINT & BHINT_ENCF) && i++);	\
} while (0)

#define __ecc_decode_sync()				\
do {							\
	unsigned int i = 1;				\
	while (!(REG_BCH_BHINT & BHINT_DECF) && i++);	\
} while (0)


#endif /* __MIPS_ASSEMBLER */

#endif /* __CHIP_BCH_H__ */
