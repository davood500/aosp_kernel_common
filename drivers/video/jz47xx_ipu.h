#ifndef _JZ47XX_IPU_H_
#define _JZ47XX_IPU_H_

#ifdef __KERNEL__

//#define REG32(val)  (*((volatile unsigned int *)(val)))

// IPU_REG_BASE
#ifndef IPU_V_BASE
#define IPU_V_BASE  0xB3080000
#endif
#ifndef IPU_P_BASE
#define IPU_P_BASE  0x13080000
#endif


// CLKGR
//#define CPM_CLKGR         0x10000020
#define CPM_CLKGR_VADDR   0xB0000020

// Module for CLKGR
#define IDCT_CLOCK      (1 << 27)
#define DBLK_CLOCK      (1 << 26)
#define ME_CLOCK        (1 << 25)
#define MC_CLOCK        (1 << 24)
#define IPU_CLOCK       (1 << 13)


// Register offset
#define  REG_CTRL               0x0
#define  REG_STATUS             0x4
#define  REG_D_FMT              0x8
#define  REG_Y_ADDR             0xc
#define  REG_U_ADDR             0x10
#define  REG_V_ADDR             0x14
#define  REG_IN_FM_GS           0x18
#define  REG_Y_STRIDE           0x1c
#define  REG_UV_STRIDE          0x20
#define  REG_OUT_ADDR           0x24
#define  REG_OUT_GS             0x28
#define  REG_OUT_STRIDE         0x2c
#define  REG_RSZ_COEF_INDEX     0x30
#define  REG_CSC_C0_COEF        0x34
#define  REG_CSC_C1_COEF        0x38
#define  REG_CSC_C2_COEF        0x3c
#define  REG_CSC_C3_COEF        0x40
#define  REG_CSC_C4_COEF        0x44
#define  REG_HRSZ_LUT_BASE      0x48
#define  REG_VRSZ_LUT_BASE      0x4c
#define  HRSZ_LUT_BASE      0x48
#define  VRSZ_LUT_BASE      0x4c
#define  REG_CSC_OFFSET_PARA    0x50
#define  REG_SRC_TLB_ADDR       0x54
#define  REG_DEST_TLB_ADDR      0x58
#define  REG_TLB_MONITOR        0x60
#define  REG_ADDR_CTRL          0x64
#define  REG_Y_ADDR_N           0x84
#define  REG_U_ADDR_N           0x88
#define  REG_V_ADDR_N           0x8c
#define  REG_OUT_ADDR_N         0x90
#define  REG_SRC_TLB_ADDR_N     0x94
#define  REG_DEST_TLB_ADDR_N    0x98
#define  REG_TLB_CTRL           0x68





// REG_CTRL field define
#define IPU_EN          (1 << 0)
#define IPU_RUN         (1 << 1)
#define HRSZ_EN         (1 << 2)
#define VRSZ_EN         (1 << 3)
#define CSC_EN          (1 << 4)
#define FM_IRQ_EN       (1 << 5)
#define IPU_RESET       (1 << 6)
#define IPU_STOP        (1 << 7)
#define H_UP_SCALE      (1 << 8) /* removed in jz47xx */
#define V_UP_SCALE      (1 << 9) /* removed in jz47xx */
#define SPKG_SEL        (1 << 10)
#define LCDC_SEL        (1 << 11)
#define SPAGE_MAP       (1 << 12)
#define DPAGE_MAP       (1 << 13)
#define DISP_SEL        (1 << 14)
#define FIELD_CONF_EN   (1 << 15)
#define FIELD_SEL       (1 << 16)
#define DFIX_SEL        (1 << 17)
#define ZOOM_SEL        (1 << 18)
#define BURST_SEL       (1 << 19)
#define ADDR_SEL        (1 << 20) /* jz4750 not support? */
#define OPMZ_SEL        (1 << 22) /* bus optimize select */


//#define YUV_READY       (1<<)

// REG_STATUS field define
#define OUT_END         (1 << 0)
#define FMT_ERR         (1 << 1)
#define SIZE_ERR        (1 << 2)
#define MSTATUS_SFT     (4)
#define MSTATUS_MSK     (3)
#define MSTATUS_IPU_FREE        (0 << MSTATUS_SFT)
#define MSTATUS_IPU_RUNNING     (1 << MSTATUS_SFT)
#define MSTATUS                 (2 << MSTATUS_SFT)


// REG_IPU_ADDR_CTRL
#define Y_RY            (1<<0)
#define U_RY            (1<<1)
#define V_RY            (1<<2)
#define D_RY            (1<<3)
#define PTS_RY          (1<<4)
#define PTD_RY          (1<<5)


// REG_TLB_MONITOR
#define MIS_CNT_SFT     (1)
#define MIS_CNT_MSK     (0x3FF)

// REG_TLB_CTRL
#define SRC_PAGE_SFT        (0)
#define SRC_PAGE_MSK        (0xF)
#define SRC_BURST_SFT       (4)
#define SRC_BURST_MSK       (0xF)
#define DEST_PAGE_SFT       (16)
#define DEST_PAGE_MSK       (0xF)
#define DEST_BURST_SFT      (20)
#define DEST_BURST_MSK      (0xF)

// REG_IN_GS field define
#define IN_FM_W(val)    ((val) << 16)
#define IN_FM_H(val)    ((val) << 0)

// REG_OUT_GS field define
#define OUT_FM_W(val)    ((val) << 16)
#define OUT_FM_H(val)    ((val) << 0)

// REG_UV_STRIDE field define
#define U_STRIDE(val)     ((val) << 16)
#define V_STRIDE(val)     ((val) << 0)



// REG_RSZ_COEF_INDEX    // defined in asm/mach-jz47xx/jz47xxipu.h
//#define VE_IDX_SFT        0
//#define VE_IDX_MSK        (0x1F)
//#define HE_IDX_SFT        16
//#define HE_IDX_MSK        (0x1F)



// RSZ_LUT_FIELD
#define START_N_SFT     (0)
// REG_HRSZ_COEF_LUT group
// BI-CUBE
#define H_CONF_SFT      (1)
#define H_CONF_MSK      (1)
#define H_CONF          (1<<H_CONF_SFT)
#define HRSZ_OFT_SFT    (1)
#define HRSZ_OFT_MSK    (0x1F)
#define HRSZ_OFT_MASK   (HRSZ_OFT_MSK<<HRSZ_OFT_SFT)
#define W_COEF_20_SFT   (6)
#define W_COEF_20_MSK   (0x7FF)
#define W_COEF_20_MASK  (W_COEF_20_MSK<<W_COEF_20_SFT)
#define W_COEF_31_SFT   (17)
#define W_COEF_31_MSK   (0x7FF)
#define W_COEF_31_MASK  (W_COEF_31_MSK<<W_COEF_31_SFT)

// BI-LINEAR
#define H_OFT_SFT       (1)
#define H_OFT_MSK       (0x1F)
#define H_OFT_MASK      (H_OFT_MSK<<H_OFT_SFT)
#define W_COEF0_SFT     (6)
#define W_COEF0_MSK     (0x7FF)
#define W_COEF0_MASK    (W_COEF0_MSK<<W_COEF0_SFT)


// REG_VRSZ_COEF_LUT group
// BI-CUBE
#define V_CONF_SFT      H_CONF_SFT
#define V_CONF_MSK      H_CONF_MSK
#define V_CONF          H_CONF
#define VRSZ_OFT_SFT    HRSZ_OFT_SFT
#define VRSZ_OFT_MSK    HRSZ_OFT_MSK
#define VRSZ_OFT_MASK   HRSZ_OFT_MASK

// BI-LINEAR
#define V_OFT_SFT       H_OFT_SFT       
#define V_OFT_MSK       H_OFT_MSK       
#define V_OFT_MASK      H_OFT_MASK      



// REG_CSC_OFFSET_PARA // defined in asm/mach-jz47xx/jz47xxipu.h
//#define LUMA_OF_SFT     (0)
//#define LUMA_OF_MSK     (0xFF)
//#define LUMA_OF_MASK    (LUMA_OF_MSK<<LUMA_OF_SFT)
//#define CHROM_OF_SFT    (16)
//#define CHROM_OF_MSK    (0xFF)
//#define CHROM_OF_MASK   (CHROM_OF_MSK<<CHROM_OF_SFT)




// parameter
// R = 1.164 * (Y - 16) + 1.596 * (cr - 128)    {C0, C1}
// G = 1.164 * (Y - 16) - 0.392 * (cb -128) - 0.813 * (cr - 128)  {C0, C2, C3}
// B = 1.164 * (Y - 16) + 2.017 * (cb - 128)    {C0, C4}

#if 1
#define YUV_CSC_C0 0x4A8        /* 1.164 * 1024 */
#define YUV_CSC_C1 0x662        /* 1.596 * 1024 */
#define YUV_CSC_C2 0x191        /* 0.392 * 1024 */
#define YUV_CSC_C3 0x341        /* 0.813 * 1024 */
#define YUV_CSC_C4 0x811        /* 2.017 * 1024 */

//#define YUV_CSC_CHROM				128
//#define YUV_CSC_LUMA				0

#define YUV_CSC_OFFSET_PARA         0x800010  /* chroma,luma */
  //#define YUV_CSC_OFFSET_PARA         0x800080  /* chroma,luma */
#else
#define YUV_CSC_C0 0x400
#define YUV_CSC_C1 0x59C
#define YUV_CSC_C2 0x161
#define YUV_CSC_C3 0x2DC
#define YUV_CSC_C4 0x718
#endif


#endif	/* #ifdef __KERNEL__ */

///////////////////////////////////////////

// Data Format Register, export to libipu
#define RGB_888_OUT_FMT				( 1 << 25 )

#define RGB_OUT_OFT_BIT				( 22 )
#define RGB_OUT_OFT_MASK			( 7 << RGB_OUT_OFT_BIT )
#define RGB_OUT_OFT_RGB				( 0 << RGB_OUT_OFT_BIT )
#define RGB_OUT_OFT_RBG				( 1 << RGB_OUT_OFT_BIT )
#define RGB_OUT_OFT_GBR				( 2 << RGB_OUT_OFT_BIT )
#define RGB_OUT_OFT_GRB				( 3 << RGB_OUT_OFT_BIT )
#define RGB_OUT_OFT_BRG				( 4 << RGB_OUT_OFT_BIT )
#define RGB_OUT_OFT_BGR				( 5 << RGB_OUT_OFT_BIT )

#define OUT_FMT_BIT					( 19 )
#define OUT_FMT_MASK				( 3 <<  OUT_FMT_BIT )
#define OUT_FMT_RGB555				( 0 <<  OUT_FMT_BIT )
#define OUT_FMT_RGB565				( 1 <<  OUT_FMT_BIT )
#define OUT_FMT_RGB888				( 2 <<  OUT_FMT_BIT )
#define OUT_FMT_YUV422				( 3 <<  OUT_FMT_BIT )
#define OUT_FMT_RGBAAA				( 3 <<  OUT_FMT_BIT )

#define YUV_PKG_OUT_OFT_BIT			( 16 )
#define YUV_PKG_OUT_OFT_MASK		( 7 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_Y1UY0V		( 0 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_Y1VY0U		( 1 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_UY1VY0		( 2 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_VY1UY0		( 3 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_Y0UY1V		( 4 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_Y0VY1U		( 5 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_UY0VY1		( 6 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_VY0UY1		( 7 << YUV_PKG_OUT_OFT_BIT )

#define IN_OFT_BIT					( 3 )
#define IN_OFT_MASK					( 3 << IN_OFT_BIT )
#define IN_OFT_Y1UY0V				( 0 << IN_OFT_BIT )
#define IN_OFT_Y1VY0U				( 1 << IN_OFT_BIT )
#define IN_OFT_UY1VY0				( 2 << IN_OFT_BIT )
#define IN_OFT_VY1UY0				( 3 << IN_OFT_BIT )

#define IN_FMT_BIT					( 0 )
#define IN_FMT_MASK					( 7 << IN_FMT_BIT )
#define IN_FMT_YUV420				( 0 << IN_FMT_BIT )
#define IN_FMT_YUV420_B				( 4 << IN_FMT_BIT )
#define IN_FMT_YUV422				( 1 << IN_FMT_BIT )
#define IN_FMT_YUV444				( 2 << IN_FMT_BIT )
#define IN_FMT_RGB				    (IN_FMT_YUV444)
#define IN_FMT_YUV411				( 3 << IN_FMT_BIT )


#define enable_ipu_irq(IPU_V_BASE) \
REG32(IPU_V_BASE + REG_CTRL) |= FM_IRQ_EN;

#define disable_ipu_irq(IPU_V_BASE) \
REG32(IPU_V_BASE + REG_CTRL) &= ~FM_IRQ_EN;




#endif // _JZ47XX_IPU_H_
