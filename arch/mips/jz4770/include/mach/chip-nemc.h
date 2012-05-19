/*
 * arch/mips/mach-jz4760/include/mach/chip-nemc.h
 *
 * JZ4760 NEMC register definition.
 *
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 */

#ifndef __CHIP_NEMC_H__
#define __CHIP_NEMC_H__


#define NEMC_BASE	0xB3410000

/*************************************************************************
 * NEMC (External Memory Controller for NAND)
 *************************************************************************/

#define NEMC_NFCSR	(NEMC_BASE + 0x50) /* NAND Flash Control/Status Register */
#define NEMC_SMCR	(NEMC_BASE + 0x14)  /* Static Memory Control Register 1 */
#define NEMC_PNCR 	(NEMC_BASE + 0x100)
#define NEMC_PNDR 	(NEMC_BASE + 0x104)
#define NEMC_BITCNT	(NEMC_BASE + 0x108)

#define REG_NEMC_NFCSR	REG32(NEMC_NFCSR)
#define REG_NEMC_SMCR1	REG32(NEMC_SMCR)
#define REG_NEMC_PNCR 	REG32(NEMC_PNCR)
#define REG_NEMC_PNDR 	REG32(NEMC_PNDR)
#define REG_NEMC_BITCNT	REG32(NEMC_BITCNT)

/* NAND Flash Control/Status Register */
#define NEMC_NFCSR_NFCE4	(1 << 7) /* NAND Flash Enable */
#define NEMC_NFCSR_NFE4		(1 << 6) /* NAND Flash FCE# Assertion Enable */
#define NEMC_NFCSR_NFCE3	(1 << 5)
#define NEMC_NFCSR_NFE3		(1 << 4)
#define NEMC_NFCSR_NFCE2	(1 << 3)
#define NEMC_NFCSR_NFE2		(1 << 2)
#define NEMC_NFCSR_NFCE1	(1 << 1)
#define NEMC_NFCSR_NFE1		(1 << 0)


#ifndef __MIPS_ASSEMBLER

#endif /* __MIPS_ASSEMBLER */

#endif /* __CHIP_NEMC_H__ */
