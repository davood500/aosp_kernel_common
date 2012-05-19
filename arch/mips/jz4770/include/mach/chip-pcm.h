/*
 * chip-pcm.h
 * JZ4760 PCM register definition
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 *
 * Author: whxu@ingenic.cn
 */

#ifndef __CHIP_PCM_H__
#define __CHIP_PCM_H__


/*
 * Pulse-code modulation module(PCM) address definition
 */
#define	PCM_BASE        0xb0071000

/*
 * pcm number
 */

#define PCM0		0
#define PCM1		1

/* PCM groups offset */
#define PCM_GOS		0x3000

/*
 * PCM registers offset address definition
 */
#define PCM_PCTL_OFFSET		(0x00)	/* rw, 32, 0x00000000 */
#define PCM_PCFG_OFFSET		(0x04)  /* rw, 32, 0x00000110 */
#define PCM_PDP_OFFSET		(0x08)  /* rw, 32, 0x00000000 */
#define PCM_PINTC_OFFSET	(0x0c)  /* rw, 32, 0x00000000 */
#define PCM_PINTS_OFFSET	(0x10)  /* rw, 32, 0x00000100 */
#define PCM_PDIV_OFFSET		(0x14)  /* rw, 32, 0x00000001 */

/*
 * PCM registers address definition
 */
#define PCM_PCTL(n)	(PCM_BASE + (n) * PCM_GOS + PCM_PCTL_OFFSET)
#define PCM_PCFG(n)	(PCM_BASE + (n) * PCM_GOS + PCM_PCFG_OFFSET)
#define PCM_PDP(n)	(PCM_BASE + (n) * PCM_GOS + PCM_PDP_OFFSET)
#define PCM_PINTC(n)	(PCM_BASE + (n) * PCM_GOS + PCM_PINTC_OFFSET)
#define PCM_PINTS(n)	(PCM_BASE + (n) * PCM_GOS + PCM_PINTS_OFFSET)
#define PCM_PDIV(n)	(PCM_BASE + (n) * PCM_GOS + PCM_PDIV_OFFSET)


/*
 * CPM registers common define
 */

/* PCM controller control register (PCTL) */
#define PCTL_ERDMA	BIT9
#define PCTL_ETDMA	BIT8
#define PCTL_LSMP	BIT7
#define PCTL_ERPL	BIT6
#define PCTL_EREC	BIT5
#define PCTL_FLUSH	BIT4
#define PCTL_RST	BIT3
#define PCTL_CLKEN	BIT1
#define PCTL_PCMEN	BIT0

/* PCM controller configure register (PCFG) */
#define PCFG_ISS_16BIT		BIT12
#define PCFG_OSS_16BIT		BIT11
#define PCFG_IMSBPOS		BIT10
#define PCFG_OMSBPOS		BIT9
#define PCFG_MODE_SLAVE		BIT0

#define PCFG_SLOT_LSB		13
#define PCFG_SLOT_MASK		BITS_H2L(14, PCFG_SLOT_LSB)
#define PCFG_SLOT(val)		((val) << PCFG_SLOT_LSB)

#define	PCFG_RFTH_LSB		5
#define	PCFG_RFTH_MASK		BITS_H2L(8, PCFG_RFTH_LSB)

#define	PCFG_TFTH_LSB		1
#define	PCFG_TFTH_MASK		BITS_H2L(4, PCFG_TFTH_LSB)

/* PCM controller interrupt control register(PINTC) */
#define PINTC_ETFS	BIT3
#define PINTC_ETUR	BIT2
#define PINTC_ERFS	BIT1
#define PINTC_EROR	BIT0

/* PCM controller interrupt status register(PINTS) */
#define PINTS_RSTS	BIT14
#define PINTS_TFS	BIT8
#define PINTS_TUR	BIT7
#define PINTS_RFS	BIT1
#define PINTS_ROR	BIT0

#define PINTS_TFL_LSB		9
#define PINTS_TFL_MASK		BITS_H2L(13, PINTS_TFL_LSB)

#define PINTS_RFL_LSB		2
#define PINTS_RFL_MASK		BITS_H2L(6, PINTS_RFL_LSB)

/* PCM controller clock division register(PDIV) */
#define PDIV_SYNL_LSB		11
#define PDIV_SYNL_MASK		BITS_H2L(16, PDIV_SYNL_LSB)

#define PDIV_SYNDIV_LSB		6
#define PDIV_SYNDIV_MASK	BITS_H2L(10, PDIV_SYNDIV_LSB)

#define PDIV_CLKDIV_LSB		0
#define PDIV_CLKDIV_MASK	BITS_H2L(5, PDIV_CLKDIV_LSB)


#ifndef __MIPS_ASSEMBLER


#define REG_PCM_PCTL(n)		REG32(PCM_PCTL(n))
#define REG_PCM_PCFG(n)		REG32(PCM_PCFG(n))
#define REG_PCM_PDP(n)		REG32(PCM_PDP(n))
#define REG_PCM_PINTC(n)	REG32(PCM_PINTC(n))
#define REG_PCM_PINTS(n)	REG32(PCM_PINTS(n))
#define REG_PCM_PDIV(n)		REG32(PCM_PDIV(n))

/*
 * PCM_DIN, PCM_DOUT, PCM_CLK, PCM_SYN
 */
#define __gpio_as_pcm(n) 						\
do {									\
	switch(n) {							\
	case PCM0:		__gpio_as_pcm0();break;			\
	case PCM1:		__gpio_as_pcm1();break;			\
	}								\
									\
} while (0)

#define __pcm_enable(n)			(REG_PCM_PCTL(n) |= PCTL_PCMEN)
#define __pcm_disable(n)		(REG_PCM_PCTL(n) &= ~PCTL_PCMEN)

#define __pcm_clk_enable(n)		(REG_PCM_PCTL(n) |= PCTL_CLKEN)
#define __pcm_clk_disable(n)		(REG_PCM_PCTL(n) &= ~PCTL_CLKEN)

#define __pcm_reset(n)			(REG_PCM_PCTL(n) |= PCTL_RST)
#define __pcm_flush_fifo(n)		(REG_PCM_PCTL(n) |= PCTL_FLUSH)

#define __pcm_enable_record(n)		(REG_PCM_PCTL(n) |= PCTL_EREC)
#define __pcm_disable_record(n)		(REG_PCM_PCTL(n) &= ~PCTL_EREC)
#define __pcm_enable_playback(n)	(REG_PCM_PCTL(n) |= PCTL_ERPL)
#define __pcm_disable_playback(n)	(REG_PCM_PCTL(n) &= ~PCTL_ERPL)

#define __pcm_enable_rxfifo(n)		__pcm_enable_record(n)
#define __pcm_disable_rxfifo(n)		__pcm_disable_record(n)
#define __pcm_enable_txfifo(n)		__pcm_enable_playback(n)
#define __pcm_disable_txfifo(n)		__pcm_disable_playback(n)

#define __pcm_last_sample(n)		(REG_PCM_PCTL(n) |= PCTL_LSMP)
#define __pcm_zero_sample(n)		(REG_PCM_PCTL(n) &= ~PCTL_LSMP)

#define __pcm_enable_transmit_dma(n)	(REG_PCM_PCTL(n) |= PCTL_ETDMA)
#define __pcm_disable_transmit_dma(n)	(REG_PCM_PCTL(n) &= ~PCTL_ETDMA)
#define __pcm_enable_receive_dma(n)	(REG_PCM_PCTL(n) |= PCTL_ERDMA)
#define __pcm_disable_receive_dma(n)	(REG_PCM_PCTL(n) &= ~PCTL_ERDMA)

#define __pcm_as_master(n)		(REG_PCM_PCFG(n) &= ~PCFG_MODE_SLAVE)
#define __pcm_as_slave(n)		(REG_PCM_PCFG(n) |= PCFG_MODE_SLAVE)

#define __pcm_set_transmit_trigger(n, val)		\
do {							\
	REG_PCM_PCFG(n) &= ~PCFG_TFTH_MASK;		\
	REG_PCM_PCFG(n) |= ((val) << PCFG_TFTH_LSB);	\
							\
} while(0)

#define __pcm_set_receive_trigger(n, val)		\
do {							\
	REG_PCM_PCFG(n) &= ~PCFG_RFTH_MASK;		\
	REG_PCM_PCFG(n) |= ((val) << PCFG_RFTH_LSB);	\
							\
} while(0)

#define __pcm_omsb_same_sync(n)   	(REG_PCM_PCFG(n) &= ~PCFG_OMSBPOS)
#define __pcm_omsb_next_sync(n)   	(REG_PCM_PCFG(n) |= PCFG_OMSBPOS)

#define __pcm_imsb_same_sync(n)   	(REG_PCM_PCFG(n) &= ~PCFG_IMSBPOS)
#define __pcm_imsb_next_sync(n)   	(REG_PCM_PCFG(n) |= PCFG_IMSBPOS)

#define __pcm_set_iss(n, val)				\
do {							\
	if ((val) == 16)				\
		REG_PCM_PCFG(n) |= PCFG_ISS_16BIT;	\
	else						\
		REG_PCM_PCFG(n) &= ~PCFG_ISS_16BIT;	\
							\
} while (0)

#define __pcm_set_oss(n, val)				\
do {							\
	if ((val) == 16)				\
		REG_PCM_PCFG(n) |= PCFG_OSS_16BIT;	\
	else						\
		REG_PCM_PCFG(n) &= ~PCFG_OSS_16BIT;	\
							\
} while (0)						\

#define __pcm_set_valid_slot(n, val)					\
	(REG_PCM_PCFG(n) = (REG_PCM_PCFG(n) & ~PCFG_SLOT_MASK) | PCFG_SLOT(val))

#define __pcm_write_data(n, val)	(REG_PCM_PDP(n) = (val))
#define __pcm_read_data(n)		(REG_PCM_PDP(n))

#define __pcm_enable_tfs_intr(n)	(REG_PCM_PINTC(n) |= PINTC_ETFS)
#define __pcm_disable_tfs_intr(n)	(REG_PCM_PINTC(n) &= ~PINTC_ETFS)

#define __pcm_enable_tur_intr(n)	(REG_PCM_PINTC(n) |= PINTC_ETUR)
#define __pcm_disable_tur_intr(n)	(REG_PCM_PINTC(n) &= ~PINTC_ETUR)

#define __pcm_enable_rfs_intr(n)	(REG_PCM_PINTC(n) |= PINTC_ERFS)
#define __pcm_disable_rfs_intr(n)	(REG_PCM_PINTC(n) &= ~PINTC_ERFS)

#define __pcm_enable_ror_intr(n)	(REG_PCM_PINTC(n) |= PINTC_EROR)
#define __pcm_disable_ror_intr(n)	(REG_PCM_PINTC(n) &= ~PINTC_EROR)

#define __pcm_ints_valid_tx(n)		(((REG_PCM_PINTS(n) & PINTS_TFL_MASK) >> PINTS_TFL_LSB))
#define __pcm_ints_valid_rx(n)		(((REG_PCM_PINTS(n) & PINTS_RFL_MASK) >> PINTS_RFL_LSB))

#define __pcm_set_clk_div(n, val)					\
	(REG_PCM_PDIV(n) = (REG_PCM_PDIV(n) & ~PDIV_CLKDIV_MASK) | ((val) << PDIV_CLKDIV_LSB))

#define __pcm_set_clk_rate(n, sysclk, pcmclk)				\
	__pcm_set_clk_div((n), ((sysclk) / (pcmclk) - 1))

#define __pcm_set_sync_div(n, val)					\
	(REG_PCM_PDIV(n) = (REG_PCM_PDIV(n) & ~PDIV_SYNDIV_MASK) | ((val) << PDIV_SYNDIV_LSB))

#define __pcm_set_sync_rate(n, pcmclk, sync)				\
	__pcm_set_sync_div((n), ((pcmclk) / (8 * (sync)) - 1))

#define __pcm_set_sync_len(n, val)					\
	(REG_PCM_PDIV(n) = (REG_PCM_PDIV(n) & ~PDIV_SYNL_MASK) | ((val) << PDIV_SYNL_LSB))

#endif /* __MIPS_ASSEMBLER */

#endif /* __CHIP_PCM_H__ */
