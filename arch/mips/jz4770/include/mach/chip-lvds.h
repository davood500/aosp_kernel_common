/*
 * arch/mips/mach-jz4770/include/mach/chip-lvds.h
 *
 * JZ4770 LVDS register definition.
 *
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 */

#ifndef __CHIP_LVDS_H__
#define __CHIP_LVDS_H__

#define LVDS_BASE     0xB3050300  

#define LVDS_TXCTRL_OFFSET 0xc0
#define LVDS_TXPLL0_OFFSET 0xc4		 
#define LVDS_TXPLL1_OFFSET 0xc8

#define LVDS_TXCTRL      (LVDS_BASE + LVDS_TXCTRL_OFFSET )
#define LVDS_TXPLL0      (LVDS_BASE + LVDS_TXPLL0_OFFSET )		 
#define LVDS_TXPLL1      (LVDS_BASE + LVDS_TXPLL1_OFFSET )


#ifndef __MIPS_ASSEMBLER

#define REG_LVDS_TXCTRL   REG32(LVDS_TXCTRL)
#define REG_LVDS_TXPLL0   REG32(LVDS_TXPLL0)
#define REG_LVDS_TXPLL1   REG32(LVDS_TXPLL1)

#endif /* __MIPS_ASSEMBLER__ */

/* TXCTRL (LVDS Transmitter Control Register) */
#define LVDS_MODEL_SEL         (1 << 31) /* 1:JEIDA 0:VESA */
#define LVDS_TX_PDB            (1 << 30) /* 0:power down   */
#define LVDS_TX_PDB_CK         (1 << 29) /* 0:power down   */
#define LVDS_RESERVE(n)        (1 << (20 + (n)) /* n = 0,1,2,3,4,5,6,7 */
#define LVDS_RSTB              (1 << 18) /* 0:RESET        */
#define LVDS_CKBIT_PHA_SEL     (1 << 17) /* 0:Rising edge 1:Falling edge */
#define LVDS_CKBYTE_PHA_SEL    (1 << 16) /* 0:Rising edge 1:Falling edge */

#define LVDS_CKOUT_PHA_S_LSB   13
#define LVDS_CKOUT_PHA_S_MASK  (0x07 << LVDS_CKOUT_PHA_S_LSB)

#define LVDS_CKOUT_SET         (1 << 12) /* 0:1x colock output 1:7x colock output */
#define LVDS_TX_OUT_SEL        (1 << 11) /* 0:LVDS output 1:CMOS RGB output */ 
#define LVDS_TX_DLY_SEL_LSB    8
#define LVDS_TX_DLY_SEL_MASK   (0x07 << LVDS_TX_DLY_SEL_LSB)
#define LVDS_TX_AMP_ADJ        (1 << 7)  
#define LVDS_TX_LVDS           (1 << 6) /* 0:200mv 1:350mv */
#define LVDS_TX_CR_LSB         3
#define LVDS_TX_CR_MASK        (0x07 << LVDS_TX_CR_LSB)
#define LVDS_TX_CR_CK          (1 << 2) 
#define LVDS_TX_OD_S	       (1 << 1) /* output level selectable pin */
#define LVDS_TX_OD_EN          (1 << 0) /* 0:disable 1:enable */

/* TXPLL0 (LVDS Transmitter's PLL Control Register 0) */

#define LVDS_PLL_LOCK           (1 << 31) /* 1:Lock derection output */
#define LVDS_PLL_DIS            (1 << 30) /* 1:Power down PLL */      
#define LVDS_BG_PWD             (1 << 29) /* 1:Band-gap power down */
#define LVDS_PLL_SSC_EN         (1 << 27)
#define LVDS_PLL_SSC_MODE       (1 << 26) /* 0:Down spread 1:Center spread */
#define LVDS_PLL_TEST           (1 << 25)
#define LVDS_PLL_POST_DIVA_LSB  21
#define LVDS_PLL_POST_DIVA_MASK (0x03 << LVDS_PLL_POST_DIVA_LSB)
#define LVDS_PLL_POST_DIVB_LSB  16
#define LVDS_PLL_POST_DIVB_MASK (0x1f << LVDS_PLL_POST_DIVB_LSB)
#define LVDS_PLL_PLLN_LSB       8
#define LVDS_PLL_PLLN_MASK      (0x7ff << LVDS_PLL_PLLN_LSB)
#define LVDS_PLL_TEST_DIV_LSB   6
#define LVDS_PLL_TEST_DIV_MASK  (0x03 << LVDS_PLL_TEST_DIV_LSB)
#define LVDS_PLL_TEST_DIV_2     (0 << LVDS_PLL_TEST_DIV_LSB)
#define LVDS_PLL_TEST_DIV_4     (1 << LVDS_PLL_TEST_DIV_LSB)
#define LVDS_PLL_TEST_DIV_8     (2 << LVDS_PLL_TEST_DIV_LSB)
#define LVDS_PLL_TEST_DIV_16    (3 << LVDS_PLL_TEST_DIV_LSB)
#define LVDS_PLL_IN_BYPASS      (1 << 5)
#define LVDS_PLL_INDIV_LSB      0
#define LVDS_PLL_INDIV_MASK     (0x1f << LVDS_PLL_INDIV_LSB)

/* TXPLL1 (LVDS Transmitter's PLL Control Register 1) */

#define LVDS_PLL_ICP_SEL_LSB    29
#define LVDS_PLL_ICP_SEL_MASK   (0x07 << LVDS_PLL_ICP_SEL_LSB)
#define LVDS_PLL_KVCO_LSB       26
#define LVDS_PLL_KVCO_MASK       (0x03 << LVDS_PLL_KVCO_LSB)
#define LVDS_PLL_IVCO_SEL_LSB   24
#define LVDS_PLL_IVCO_SEL_MASK  (0x03 << LVDS_PLL_IVCO_SEL_LSB)
#define LVDS_PLL_SSCN_LSB       17
#define LVDS_PLL_SSCN_MASK      (0x7f << LVDS_PLL_SSCN_LSB)
#define LVDS_PLL_COUNT_LSB      4
#define LVDS_PLL_COUNT_MASK     (0x1fff << LVDS_PLL_COUNT_LSB)
#define LVDS_PLL_GAIN_LSB       0
#define LVDS_PLL_GAIN_MASK      (0x0f << LVDS_PLL_GAIN_LSB)



#endif /* __CHIP_LVDS_H__ */
