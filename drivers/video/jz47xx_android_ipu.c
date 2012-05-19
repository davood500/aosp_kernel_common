/*
 * linux/drivers/video/jz4750_android_ipu.c 
 *
 * Copyright (C) 2005-2010, Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.

 *  Author:   <chlfeng@ingenic.cn>
 *
 *  Create:   2009-10-21, by Emily
 *  
 *  http://www.ingenic.cn
 */

#include <linux/module.h>
#include <linux/kernel.h>

#include <asm/irq.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/delay.h>


#include "jz_ipu.h"
#include "jz47xx_android_lcd.h"
#include <mach/jz4770.h>
#include <asm/uaccess.h>


//#define IPU_DBG
#undef  IPU_DBG

#define IPU_TAG "JZ47XX-IPU: "
#define MY_TAG IPU_TAG

#ifdef IPU_DBG
#define ENTER()								\
	do {								\
		printk(MY_TAG"%03d ENTER %s\n", __LINE__, __FUNCTION__);	\
	} while (0)
#define LEAVE()								\
	do {								\
		printk(MY_TAG"%03d LEAVE %s\n", __LINE__, __FUNCTION__);	\
	} while (0)
#define MY_DBG(sss, aaa...)						\
	do {								\
		printk(MY_TAG"%03d %s, " sss "\n", __LINE__, __FUNCTION__, ##aaa); \
		udelay(100000);						\
	} while (0)
#else
#define ENTER()					\
	do {					\
	} while (0)
#define LEAVE()					\
	do {					\
	} while (0)
#define MY_DBG(sss, aaa...)			\
	do {					\
	} while (0)

#endif	// 



#define IPU_OPEN							(1 << 0)
#define IPU_INIT							(1 << 1)
#define IPU_SET_STATE_BIT					(25)	// 7 bit
#define IPU_SET_STATE_MASK					(0x7f << IPU_SET_STATE_BIT)
#define SET_BIT(x)							(1 << (IPU_SET_STATE_BIT+x))
#define IPU_CHANGE_BUF						(SET_BIT(0))
#define IPU_SET_CTRL						(SET_BIT(1))
#define IPU_SET_DAT_FMT						(SET_BIT(2))
#define IPU_SET_CSC							(SET_BIT(3))
#define IPU_RATIO_MUL						(100000)



#ifdef PHYS
#undef PHYS
#endif

#define PHYS(x) (x)

struct ipu_driver_priv g_ipu_native_data = 
{
	.img = {
		.version = sizeof(struct ipu_img_param_t),
	},
	.frame_requested = 0,
	.frame_done = 0,
};
struct ipu_driver_priv *ipu_priv = &g_ipu_native_data;

extern struct jz47xxlcd_info jz47xx_lcd_panel;
extern struct jz47xxlcd_info *jz47xx_lcd_info;

extern unsigned char *lcd_frame;

/*----------------------------------------------------------------------------------*/
struct ipu_reg_struct {
	char * name;
	unsigned int addr;
};


static int ipu_is_suspended(void)
{
	struct ipu_driver_priv *ipu = &g_ipu_native_data;

	if ( ipu->is_suspended ) {
		return 1;
	}

	return 0;
}

static int isOsd2LayerMode(void)
{
	struct ipu_img_param_t *img = &g_ipu_native_data.img;
	int mode = img->output_mode ;
	if ( (mode & IPU_OUTPUT_TO_LCD_FG1)
		 && !(mode & IPU_OUTPUT_MODE_FG0_OFF) 
		)
		return 1;

	return 0;
}


static int isIPUtoLcdcFG1Mode(void)
{
	struct ipu_img_param_t *img = &g_ipu_native_data.img;
	int mode = img->output_mode ;
	if ( (mode & IPU_OUTPUT_TO_LCD_FG1) )
		return 1;
	
	return 0;
}

static int get_ipu_restart_trigger(struct jz47xxlcd_panel_t *panel_info) {
	int frond_porch;
	int third_of_vblank;
	int trigger_value;
	int ht, vt, vds, vde, hde;
	/* LCDC spec: ipu_restart_trigger = frond_porch + ((HT-0)x(VPE-VPS))/3 */

	vt = (REG_LCD_VAT&LCD_VAT_VT_MASK)>>LCD_VAT_VT_BIT;
	ht = (REG_LCD_VAT&LCD_VAT_HT_MASK)>>LCD_VAT_HT_BIT;
	vds = (REG_LCD_DAV&LCD_DAV_VDS_MASK)>>LCD_DAV_VDS_BIT; /* Vsync start at 0. */
	vde = (REG_LCD_DAV&LCD_DAV_VDE_MASK)>>LCD_DAV_VDE_BIT;
	hde = (REG_LCD_DAH&LCD_DAH_HDE_MASK)>>LCD_DAV_VDE_BIT;

	if (hde>ht) {
		printk("LCDC config error hde>ht\n");
		hde = ht;
	}
	if (vde>vt) {
		printk("LCDC config error vde>vt\n");
		vde = vt;
	}

	third_of_vblank = (ht*vds)/3;
	frond_porch = (ht-hde) + (vt-vde)*ht;
	trigger_value = (frond_porch + third_of_vblank);
#if 0
	printk("\n");
	printk("\n REG_LCD_VAT=0x%08X", REG_LCD_VAT);
	printk("\n REG_LCD_DAV=0x%08X", REG_LCD_DAV);
	printk("\n REG_LCD_DAH=0x%08X", REG_LCD_DAH);
	printk("\n");
	printk("\t ht = %d", ht);
	printk("\t vt = %d", vt);
	printk("\t vds = %d",vds );
	printk("\t vde = %d", vde);
	printk("\t hde = %d", hde);
	printk("\n third_of_vblank = %d", third_of_vblank);
	printk("\n frond_porch = %d", frond_porch);
	printk("\n trigger_value = %d", trigger_value);
	printk("\n");

#endif

	return trigger_value;
}


/* ---------------------lcd_ops -----------------------------*/
int resize_fg1_for_ipu_to_lcd(int w, int h)
{
	int size;
	int j, count = 100000;
	size = h << LCD_DESSIZE_HEIGHT_BIT | w << LCD_DESSIZE_WIDTH_BIT;
	if (REG_LCD_SIZE1 == size) {
		printk("FG1: same size\n");
		return 0;// -EFAULT;
	}
	else {
		if (!(REG_LCD_CTRL & LCD_CTRL_DIS)
		    && (REG_LCD_CTRL & LCD_CTRL_ENA)
		    && (REG_LCD_OSDC & LCD_OSDC_F1EN)) {
			REG_LCD_OSDCTRL |= LCD_OSDCTRL_CHANGES;
			REG_LCD_SIZE1 = size;
			while(!(REG_LCD_OSDS & LCD_OSDS_READY));
			j = count;
			msleep(40);
			while((REG_LCD_OSDCTRL & LCD_OSDCTRL_CHANGES) && j--);
			if(j == 0) {
				printk("Error FG1 Position: Wait change fail.\n");
				return -EFAULT;

			}
		}
		else 
			REG_LCD_SIZE1 = size;
	}
	return 0;
}

int move_fg1_for_ipu_to_lcd(int x, int y)
{
	int pos;
	int j, count = 100000;
	pos = y << LCD_XYP_YPOS_BIT | x << LCD_XYP_XPOS_BIT;
	if (REG_LCD_XYP1 == pos) {
		printk("FG1: same size\n");
		return 0;// -EFAULT;
	}
	else {
		if (!(REG_LCD_CTRL & LCD_CTRL_DIS)
		    && (REG_LCD_CTRL & LCD_CTRL_ENA)
		    && (REG_LCD_OSDC & LCD_OSDC_F1EN)) {
			REG_LCD_XYP1 = pos;
			REG_LCD_OSDCTRL |= LCD_OSDCTRL_CHANGES;
			while(!(REG_LCD_OSDS & LCD_OSDS_READY));
			j = count;
			msleep(40);
			while((REG_LCD_OSDCTRL & LCD_OSDCTRL_CHANGES) && j--);
			if(j == 0) {
				printk("Error FG1 Position: Wait change fail.\n");
				return -EFAULT;
				
			}
		}
		else 
			REG_LCD_XYP1 = pos;
	}
	return 0;
}

int lcd_enable_ipu_to_fg1(void)
{

	return 0;
}
int lcd_disableipu_to_fg1(void)
{
	return 0;
}

/* ---------------------ipu_ops -----------------------------*/
struct ipu_reg_struct ipu_regs_name[] = {
	{"REG_CTRL",	        REG_CTRL},
	{"REG_STATUS",	        REG_STATUS},
	{"REG_D_FMT",	        REG_D_FMT},
	{"REG_Y_ADDR",	        REG_Y_ADDR},
	{"REG_U_ADDR",	        REG_U_ADDR},
	{"REG_V_ADDR",	        REG_V_ADDR},
	{"REG_IN_FM_GS",        REG_IN_FM_GS},
	{"REG_Y_STRIDE",        REG_Y_STRIDE},
	{"REG_UV_STRIDE",       REG_UV_STRIDE},
	{"REG_OUT_ADDR",        REG_OUT_ADDR},
	{"REG_OUT_GS",	        REG_OUT_GS},
	{"REG_OUT_STRIDE",      REG_OUT_STRIDE},
	{"RSZ_COEF_INDEX",      REG_RSZ_COEF_INDEX},
	{"REG_CSC_C0_COEF",     REG_CSC_C0_COEF},
	{"REG_CSC_C1_COEF",     REG_CSC_C1_COEF},
	{"REG_CSC_C2_COEF",     REG_CSC_C2_COEF},
	{"REG_CSC_C3_COEF",     REG_CSC_C3_COEF},
	{"REG_HRSZ_LUT_BASE",   REG_HRSZ_LUT_BASE}, /* write only */
	{"REG_VRSZ_LUT_BASE",   REG_VRSZ_LUT_BASE}, /* write only */
	{"REG_CSC_OFFSET_PARA", REG_CSC_OFFSET_PARA},
	{"REG_SRC_TLB_ADDR",    REG_SRC_TLB_ADDR},
	{"REG_DEST_TLB_ADDR",   REG_DEST_TLB_ADDR},
	{"REG_TLB_MONITOR",     REG_TLB_MONITOR},
	{"REG_ADDR_CTRL",       REG_ADDR_CTRL},
	{"REG_Y_ADDR_N",        REG_Y_ADDR_N},
	{"REG_U_ADDR_N",        REG_U_ADDR_N},
	{"REG_V_ADDR_N",        REG_V_ADDR_N},
	{"REG_OUT_ADDR_N",      REG_OUT_ADDR_N},
	{"REG_SRC_TLB_ADDR_N",  REG_SRC_TLB_ADDR_N},
	{"REG_DEST_TLB_ADDR_N", REG_DEST_TLB_ADDR_N},
	{"REG_TLB_CTRL",        REG_TLB_CTRL},
};

int jz47_dump_ipu_regs(struct ipu_driver_priv *ipu, int num)
{
	int i, total;

	int *hoft_table; /*  */
	int *hcoef_table;
	int hcoef_real_heiht;
	/* jz47xx's ipu HRSZ_COEF_LUT*/
	int *voft_table;
	int *vcoef_table;
	int vcoef_real_heiht;
	struct ipu_img_param_t *img;

	ENTER();
	if (ipu == NULL) {
		return -1;
	}
	img = &ipu->img;
	hoft_table = ipu->hoft_table; /*  */
	hcoef_table= ipu->hcoef_table;
	hcoef_real_heiht = img->hcoef_real_heiht;
	voft_table = ipu->voft_table; /*  */
	vcoef_table= ipu->vcoef_table;
	vcoef_real_heiht = img->vcoef_real_heiht;


	if (num >= 0) {
		return (1);
	}

	if (num == -1) {
		total = sizeof(ipu_regs_name) / sizeof(struct ipu_reg_struct);
		for (i = 0; i < total; i++) {
			printk("ipu_reg: %s: \t0x%08x\r\n", ipu_regs_name[i].name,
			       REG32((unsigned int) IPU_V_BASE + ipu_regs_name[i].addr));
		}
	}
	if (num == -2) {
		printk(" //bi-cube resize\nint cube_hcoef_table[H_OFT_LUT][4] = {");
		for ( i = 0 ; i < hcoef_real_heiht ; i++) 
			printk("\t\t\t{0x%02x,0x%02x, 0x%02x, 0x%02x}, \n", ipu->cube_hcoef_table[i][0], ipu->cube_hcoef_table[i][1], 
			       ipu->cube_hcoef_table[i][2], ipu->cube_hcoef_table[i][3] ); 
		printk("};");
		
		printk(" int cube_vcoef_table[V_OFT_LUT][4] = {");
		for ( i = 0 ; i < vcoef_real_heiht ; i++) 
			printk("\t\t\t{0x%02x,0x%02x, 0x%02x, 0x%02x}, \n", ipu->cube_vcoef_table[i][0], ipu->cube_vcoef_table[i][1], 
			       ipu->cube_vcoef_table[i][2], ipu->cube_vcoef_table[i][3] ); 
		printk("};");
	}
	else if (num == -3) {
		printk("hcoef_real_heiht=%d\n", hcoef_real_heiht);
		for (i = 0; i < IPU_LUT_LEN; i++) {
			printk("ipu_H_LUT(%02d): hcoef, hoft: %05d %02d\n",
			       i, hcoef_table[i], hoft_table[i]);
		}
		printk("vcoef_real_veiht=%d\n", vcoef_real_heiht);
		for (i = 0; i < IPU_LUT_LEN; i++) {
			printk("ipu_V_LUT(%02d): vcoef, voft: %05d %02d\n",
			       i, vcoef_table[i], voft_table[i]);
		}
	}

	return 1;
}


void print_img(struct ipu_driver_priv *ipu)
{
	struct ipu_img_param_t *img;
	ENTER();
	if (ipu == NULL) {
		return;
	}

	img = &ipu->img;

	printk("ipu_cmd[%#x]\r\n", (unsigned int) img->ipu_cmd);
	printk("output_mode[%#x]\r\n", (unsigned int) img->output_mode);
	printk("in_width[%#x]\r\n", (unsigned int) img->in_width);
	printk("in_height[%#x]\r\n", (unsigned int) img->in_height);
	printk("in_bpp[%#x]\r\n", (unsigned int) img->in_bpp);
	printk("in_fmt[%#x]\n", (unsigned int) img->in_fmt);
	printk("out_fmt[%#x]\n",(unsigned int) img->out_fmt);
	printk("out_x[%#x]\n",(unsigned int) img->out_x);
	printk("out_y[%#x]\n",(unsigned int) img->out_y);
	printk("out_width[%#x]\r\n", (unsigned int) img->out_width);
	printk("out_height[%#x]\r\n", (unsigned int) img->out_height);
	printk("y_buf_v[%#x]\r\n", (unsigned int) img->y_buf_v);
	printk("u_buf_v[%#x]\r\n", (unsigned int) img->u_buf_v);
	printk("v_buf_v[%#x]\r\n", (unsigned int) img->v_buf_v);
	printk("y_buf_p[%#x]\r\n", (unsigned int) img->y_buf_p);
	printk("u_buf_p[%#x]\r\n", (unsigned int) img->u_buf_p);
	printk("v_buf_p[%#x]\r\n", (unsigned int) img->v_buf_p);
	printk("out_buf_v[%#x]\r\n", (unsigned int) img->out_buf_v);
	printk("out_buf_p[%#x]\r\n", (unsigned int) img->out_buf_p);
	printk("src_page_mapping[%#x]\r\n", (unsigned int)img->src_page_mapping);
	printk("dst_page_mapping[%#x]\r\n", (unsigned int)img->dst_page_mapping);
	printk("y_t_addr[%#x]\r\n", (unsigned int) img->y_t_addr);
	printk("u_t_addr[%#x]\r\n", (unsigned int) img->u_t_addr);
	printk("v_t_addr[%#x]\r\n", (unsigned int) img->v_t_addr);
	printk("out_t_addr[%#x]\r\n", (unsigned int) img->out_t_addr);
	printk("stride.y[%#x]\r\n", (unsigned int) img->stride.y);
	printk("stride.u[%#x]\r\n", (unsigned int) img->stride.u);
	printk("stride.v[%#x]\r\n", (unsigned int) img->stride.v);
	printk("Wdiff[%#x]\r\n", (unsigned int) img->Wdiff);
	printk("Hdiff[%#x]\r\n", (unsigned int) img->Hdiff);
	printk("zoom_mode[%#x]\r\n", (unsigned int) img->zoom_mode);
	printk("hcoef_real_heiht[%#x]\r\n", (unsigned int) img->hcoef_real_heiht);
	printk("vcoef_real_heiht[%#x]\r\n", (unsigned int) img->vcoef_real_heiht);
	printk("hoft_table[%#x]\r\n", (unsigned int) img->hoft_table);
	printk("voft_table[%#x]\r\n", (unsigned int) img->voft_table);
	printk("hcoef_table[%#x]\r\n", (unsigned int) img->hcoef_table);
	printk("voft_table[%#x]\r\n", (unsigned int) img->voft_table);
	printk("hcoef_table[%#x]\r\n", (unsigned int) img->hcoef_table);
	printk("vcoef_table[%#x]\r\n", (unsigned int) img->vcoef_table);
	printk("cube_hcoef_table[%#x]\r\n", (unsigned int) img->cube_hcoef_table);
	printk("cube_vcoef_table[%#x]\r\n", (unsigned int) img->cube_vcoef_table);
	return ;
}


/* pixel format definitions to ipu pixel format */
static unsigned int hal_to_ipu_infmt( int hal_fmt)
{
	unsigned int ipu_fmt = IN_FMT_YUV420;

	/* hardware/libhardware/include/hardware/hardware.h */
	switch ( hal_fmt ) {
/*	case HAL_PIXEL_FORMAT_RGBA_8888:
	ipu_fmt = ;
	break;
	case HAL_PIXEL_FORMAT_RGB_565:
	ipu_fmt = ;
	break;
	case HAL_PIXEL_FORMAT_BGRA_8888:
	ipu_fmt = ;
	break;
	case HAL_PIXEL_FORMAT_RGBA_5551:
	ipu_fmt = ;
	break;
	case HAL_PIXEL_FORMAT_RGBA_4444:
	ipu_fmt = ;
	break; */
	case HAL_PIXEL_FORMAT_YCbCr_422_SP:
		ipu_fmt = IN_FMT_YUV422;
		break;
	case HAL_PIXEL_FORMAT_YCbCr_420_SP:
		ipu_fmt = IN_FMT_YUV420;
		break;
	case HAL_PIXEL_FORMAT_YCbCr_422_P:
		ipu_fmt = IN_FMT_YUV422;
		break;
	case HAL_PIXEL_FORMAT_YCbCr_422_I:
		ipu_fmt = IN_FMT_YUV422;
		break;
	case HAL_PIXEL_FORMAT_YCbCr_420_I:
		ipu_fmt = IN_FMT_YUV420;
		break;
	case HAL_PIXEL_FORMAT_YCbCr_420_B:
	case HAL_PIXEL_FORMAT_JZ_YUV_420_B:
		ipu_fmt = IN_FMT_YUV420_B;
		break;		
	case HAL_PIXEL_FORMAT_YCbCr_420_P:
	case HAL_PIXEL_FORMAT_JZ_YUV_420_P:
	default:
		ipu_fmt = IN_FMT_YUV420;
		break;
	}

	return ipu_fmt;
}


/* pixel format definitions to ipu pixel format */
static unsigned int hal_to_ipu_outfmt( int hal_fmt)
{
	unsigned int ipu_fmt = OUT_FMT_RGB888;
	/* hardware/libhardware/include/hardware/hardware.h */
	switch ( hal_fmt ) {
	case HAL_PIXEL_FORMAT_RGBA_8888:
	case HAL_PIXEL_FORMAT_RGBX_8888:
	case HAL_PIXEL_FORMAT_RGB_888:
	case HAL_PIXEL_FORMAT_BGRA_8888:
	case HAL_PIXEL_FORMAT_BGRX_8888:
		ipu_fmt = OUT_FMT_RGB888;
		break;
	case HAL_PIXEL_FORMAT_RGB_565:
		ipu_fmt = OUT_FMT_RGB565;
		break;
	case HAL_PIXEL_FORMAT_RGBA_5551:
		ipu_fmt = OUT_FMT_RGB555;
		break;
/*	case HAL_PIXEL_FORMAT_RGBA_4444:
	ipu_fmt = ;
	break;
	case HAL_PIXEL_FORMAT_YCbCr_422_SP:
	ipu_fmt = ;
	break;
	case HAL_PIXEL_FORMAT_YCbCr_420_SP:
	ipu_fmt = ;
	break;
	case HAL_PIXEL_FORMAT_YCbCr_422_P:
	ipu_fmt = ;
	break;
	case HAL_PIXEL_FORMAT_YCbCr_422_I:
	ipu_fmt = ;
	break;
	case HAL_PIXEL_FORMAT_YCbCr_420_I:
	ipu_fmt = ;
	break;
	case HAL_PIXEL_FORMAT_YCbCr_420_P:
	default:
	ipu_fmt = ;
	break; */
	}
	return ipu_fmt;
}

/* get RGB order */
static unsigned int get_out_fmt_rgb_order(int hal_out_fmt) {
	unsigned int order = RGB_OUT_OFT_RGB;

	switch (hal_out_fmt) {
	case HAL_PIXEL_FORMAT_RGBA_8888:
	case HAL_PIXEL_FORMAT_RGBX_8888:
		order = RGB_OUT_OFT_BGR;
		break;
	case HAL_PIXEL_FORMAT_RGB_888:
	case HAL_PIXEL_FORMAT_RGB_565:
	case HAL_PIXEL_FORMAT_BGRA_8888:
	case HAL_PIXEL_FORMAT_BGRX_8888:
	case HAL_PIXEL_FORMAT_RGBA_5551:
	default :
		order = RGB_OUT_OFT_RGB;
		break;
	}

	return order;
}

void dump_ipu_regs(void)
{
	int i, total;
	total = sizeof(ipu_regs_name) / sizeof(struct ipu_reg_struct);
	for (i = 0; i < total; i++) {
		printk("ipu_reg: %s: \t0x%08x\r\n", ipu_regs_name[i].name,
		       REG32((unsigned int) IPU_V_BASE + ipu_regs_name[i].addr));
	}
}


/* max timeout 100ms */
static inline int jz47_ipu_wait_frame_end_flag(void)
{
		unsigned long long clock_start;
		unsigned long long clock_now;
		clock_start = sched_clock();
		while ( 1 ) {
			unsigned int ipu_frame_end = REG32(IPU_V_BASE + REG_STATUS) & OUT_END;
			if ( ipu_frame_end ) 
				break;	// wait the end flag
			clock_now = sched_clock();
			if ( (clock_now - clock_start) > (30 * 1000000)) { /* timeout: 30ms */
				printk("jz47_ipu_wait_frame_end_flag() timeout....\n");
				break;	// wait the end flag
			}
		}

	return 0;
}

static inline int jz47_wait_lcd_frame_end_flag(void)
{
	unsigned int OSDC;
	unsigned int F0EN;
	unsigned int F1EN;

	OSDC = REG_LCD_OSDC;
	F0EN = OSDC & (1<<3);
	F1EN = OSDC & (1<<4);

	if ( F1EN && (!F0EN)  ) {
		if((REG32(IPU_V_BASE + REG_STATUS) & MSTATUS_IPU_RUNNING)){
			jz47_ipu_wait_frame_end_flag();
		}
	}
	else {
		unsigned long long clock_start;
		unsigned long long clock_now;
		clock_start = sched_clock();
		REG_LCD_STATE &= ~LCD_STATE_EOF; // Clear previous EOF flag
		while ( 1 ) {
			if (__lcd_end_of_frame()) 
				break;	// wait the end flag
			clock_now = sched_clock();
			if ( (clock_now - clock_start) > (30 * 1000000)) { /* timeout: 30ms */
				printk("jz47_wait_lcd_frame_end_flag() timeout....\n");
				break;	// wait the end flag
			}
		}
	}

	return 0;
}

static inline int disable_lcd_controller_with_wait_lcd_frame_end(void)
{
	jz47_wait_lcd_frame_end_flag();
	__lcd_clr_ena();
	return 0;
}


// set ipu data format.
static int jz47_set_ipu_csc_cfg(struct ipu_driver_priv *ipu, int outW, int outH,
				int Wdiff, int Hdiff)
{
	struct ipu_img_param_t *img;
	unsigned int in_fmt;
	unsigned int out_fmt;
	unsigned int out_rgb_order;

	ENTER();
	if (ipu == NULL) {
		return -1;
	}

	img = &ipu->img;

	in_fmt = hal_to_ipu_infmt(img->in_fmt);
	out_fmt = hal_to_ipu_outfmt(img->out_fmt);
	/* set RGB order */
	out_rgb_order = get_out_fmt_rgb_order(img->out_fmt);

	switch (in_fmt) {
	case IN_FMT_YUV420:
	case IN_FMT_YUV420_B:
		Hdiff = (Hdiff + 1) & ~1;
		Wdiff = (Wdiff + 1) & ~1;
		break;
	case IN_FMT_YUV422:
		Wdiff = (Wdiff + 1) & ~1;
		break;
	case IN_FMT_YUV444:
	case IN_FMT_YUV411:
		break;
	default:
		printk("Error: 111 Input data format isn't support\n");
		return (-1);
	}

	switch (out_fmt) {
	case OUT_FMT_RGB888:
		outW = outW << 2;
		break;
	case OUT_FMT_RGB555:
		outW = outW << 1;
		break;
	case OUT_FMT_RGB565:
		outW = outW << 1;
		break;
	}

	// Set GS register
	REG32(IPU_V_BASE + REG_IN_FM_GS) =
	    IN_FM_W(img->in_width - Wdiff) | IN_FM_H((img->in_height - Hdiff) & ~0x1);
	REG32(IPU_V_BASE + REG_OUT_GS) = OUT_FM_W(outW) | OUT_FM_H(outH);

	// Set out stride
	if (img->stride.out != 0) {
		REG32(IPU_V_BASE + REG_OUT_STRIDE) = img->stride.out;
	} else {
		switch ( img->output_mode & IPU_OUTPUT_MODE_MASK) {
		case IPU_OUTPUT_TO_LCD_FG1:
			break;
		case IPU_OUTPUT_TO_LCD_FB0:
		case IPU_OUTPUT_TO_LCD_FB1:
			REG32(IPU_V_BASE + REG_OUT_STRIDE) = ipu->fb_w * img->in_bpp >> 3;
			break;
		case IPU_OUTPUT_TO_FRAMEBUFFER:
		default:
			outW = img->out_width;
			switch (out_fmt) {
			default:
			case OUT_FMT_RGB888:
				outW = outW << 2;
				break;
			case OUT_FMT_RGB555:
			case OUT_FMT_RGB565:
				outW = outW << 1;
				break;
			}
			REG32(IPU_V_BASE + REG_OUT_STRIDE) = outW;
			break;
		}

	}

	if ( in_fmt == IN_FMT_YUV422 ) {
		printk("*** jz47xx ipu driver: IN_FMT_YUV422 use IN_OFT_Y1VY0U\n");
		in_fmt |= IN_OFT_Y1VY0U;
	}

	// set Format
	REG32(IPU_V_BASE + REG_D_FMT) = in_fmt | out_fmt | out_rgb_order;
	// set CSC parameter
	if ((in_fmt != IN_FMT_YUV444) && (out_fmt != OUT_FMT_YUV422)) {
		REG32(IPU_V_BASE + REG_CTRL) |= CSC_EN;
		REG32(IPU_V_BASE + REG_CSC_C0_COEF) = YUV_CSC_C0;
		if (in_fmt == IN_FMT_YUV420_B) {
			// interchange C1 with C4, C2 with C3 for IPU Block format
			REG32(IPU_V_BASE + REG_CSC_C1_COEF) = YUV_CSC_C4;
			REG32(IPU_V_BASE + REG_CSC_C2_COEF) = YUV_CSC_C3;
			REG32(IPU_V_BASE + REG_CSC_C3_COEF) = YUV_CSC_C2;
			REG32(IPU_V_BASE + REG_CSC_C4_COEF) = YUV_CSC_C1;
		}
		else {
			REG32(IPU_V_BASE + REG_CSC_C1_COEF) = YUV_CSC_C1;
			REG32(IPU_V_BASE + REG_CSC_C2_COEF) = YUV_CSC_C2;
			REG32(IPU_V_BASE + REG_CSC_C3_COEF) = YUV_CSC_C3;
			REG32(IPU_V_BASE + REG_CSC_C4_COEF) = YUV_CSC_C4;
		}
		REG32(IPU_V_BASE + REG_CSC_OFFSET_PARA) = YUV_CSC_OFFSET_PARA;
	}
	return 0;
}

static int jz47_set_ipu_resize(struct ipu_driver_priv *ipu)
{
	int i;
	int *oft_table;
	int *coef_table;
	struct ipu_img_param_t *img;

	ENTER();
	if (ipu == NULL) {
		return -1;
	}

	img = &ipu->img;

	REG32(IPU_V_BASE + REG_CTRL) &= ~(VRSZ_EN | HRSZ_EN);

	if (img->out_width != img->in_width)
		REG32(IPU_V_BASE + REG_CTRL) |= HRSZ_EN;
	if (img->out_height != img->in_height)
		REG32(IPU_V_BASE + REG_CTRL) |= VRSZ_EN;
	/* no use in jz47xx
	  REG32(IPU_V_BASE + REG_CTRL) |= (img->out_width >= img->in_width) ? H_UP_SCALE : 0;
	  REG32(IPU_V_BASE + REG_CTRL) |= (img->out_height >= img->in_height) ? V_UP_SCALE : 0;
	*/
	REG32(IPU_V_BASE + REG_RSZ_COEF_INDEX) =
	    ((img->vcoef_real_heiht - 1) << VE_IDX_SFT)
	    | ((img->hcoef_real_heiht - 1) << HE_IDX_SFT);

	if (img->zoom_mode != ZOOM_MODE_BILINEAR){
		REG32(IPU_V_BASE + REG_CTRL) |= ZOOM_SEL;

		/* set_hrsz_lut_weigth_cube */
		oft_table = &ipu->hoft_table[1];
		REG32(IPU_V_BASE + HRSZ_LUT_BASE) = (1 << START_N_SFT);
		for (i=0;i<img->hcoef_real_heiht;i++) {
			REG32(IPU_V_BASE + HRSZ_LUT_BASE) = 
				((ipu->cube_hcoef_table[i][0] & W_COEF_20_MSK)<<W_COEF_20_SFT)
				| ((ipu->cube_hcoef_table[i][1] & W_COEF_31_MSK)<<W_COEF_31_SFT);
			REG32(IPU_V_BASE + HRSZ_LUT_BASE) =
				((ipu->cube_hcoef_table[i][2] & W_COEF_20_MSK)<<W_COEF_20_SFT)
				| (((ipu->cube_hcoef_table[i][3]) & W_COEF_31_MSK)<<W_COEF_31_SFT)
				| ((oft_table[i] & HRSZ_OFT_MSK) << HRSZ_OFT_SFT);
		}

		/* set_vrsz_lut_weigth_cube */
		oft_table = &ipu->voft_table[1];
		REG32(IPU_V_BASE + VRSZ_LUT_BASE) = (1 << START_N_SFT);
		for (i=0;i<img->vcoef_real_heiht;i++) {
			REG32(IPU_V_BASE + VRSZ_LUT_BASE) = 
				((ipu->cube_vcoef_table[i][0] & W_COEF_20_MSK)<<W_COEF_20_SFT)
				| ((ipu->cube_vcoef_table[i][1] & W_COEF_31_MSK)<<W_COEF_31_SFT);
			REG32(IPU_V_BASE + VRSZ_LUT_BASE) = 
				((ipu->cube_vcoef_table[i][2] & W_COEF_20_MSK)<<W_COEF_20_SFT)
				| ((ipu->cube_vcoef_table[i][3] & W_COEF_31_MSK)<<W_COEF_31_SFT)
				| ((oft_table[i] & VRSZ_OFT_MSK) << VRSZ_OFT_SFT);
		}
	}
	else {
		REG32(IPU_V_BASE + REG_CTRL) &= ~ZOOM_SEL;

		/* set_vrsz_lut_coef_line */
		oft_table = &ipu->voft_table[1];
		coef_table = &ipu->vcoef_table[1];
		/* set IPU LUT register */
		REG32(IPU_V_BASE + VRSZ_LUT_BASE) = (1 << START_N_SFT);
		for (i = 0; i < img->vcoef_real_heiht; i++) {
			REG32(IPU_V_BASE + VRSZ_LUT_BASE) = 
				((coef_table[i]&W_COEF0_MSK)<< W_COEF0_SFT)
				| ((oft_table[i]&V_OFT_MSK) << V_CONF_SFT);
		}

		/* set_hrsz_lut_coef_line */
		oft_table = &ipu->hoft_table[1];
		coef_table = &ipu->hcoef_table[1];
		
		REG32(IPU_V_BASE + HRSZ_LUT_BASE) = (1 << START_N_SFT);
		for (i = 0; i < img->hcoef_real_heiht; i++) {
			REG32(IPU_V_BASE + HRSZ_LUT_BASE) = 
				((coef_table[i]&W_COEF0_MSK)<< W_COEF0_SFT)
				| ((oft_table[i]&H_OFT_MSK) << H_CONF_SFT);
		}
	}

	return (0);
}

////////////////////////////////////////////////////////////////////////////
static int jz47_set_ipu_buf(struct ipu_driver_priv *ipu, struct ipu_img_param_t *new_img)
{
	//      int ret;
	unsigned int py_buf;
	unsigned int pu_buf;
	unsigned int pv_buf;
	unsigned int py_t_buf;
	unsigned int pu_t_buf;
	unsigned int pv_t_buf;
	unsigned int out_buf;
	unsigned int spage_map;
	unsigned int dpage_map;
	unsigned int lcdc_sel;
	unsigned int in_fmt;

	struct ipu_img_param_t *img;

	ENTER();
	if (ipu == NULL) {
		return -1;
	}

	img = &ipu->img;
	if ( new_img ) {
		unsigned int old_bpp = ipu->img.in_bpp;
		*img = *new_img;
		img->in_bpp = old_bpp;
	}

	spage_map = img->src_page_mapping;
	dpage_map = img->dst_page_mapping;
	lcdc_sel = REG32(IPU_V_BASE + REG_CTRL) & LCDC_SEL;

	py_buf = ((unsigned int) img->y_buf_p);
	pu_buf = ((unsigned int) img->u_buf_p);
	pv_buf = ((unsigned int) img->v_buf_p);

	py_t_buf = ((unsigned int) img->y_t_addr);
	pu_t_buf = ((unsigned int) img->u_t_addr);
	pv_t_buf = ((unsigned int) img->v_t_addr);

	in_fmt = hal_to_ipu_infmt(img->in_fmt);

	MY_DBG("py_buf=0x%08x, pu_buf=0x%08x, pv_buf=0x%08x, py_t_buf=0x%08x, pu_t_buf=0x%08x, pv_t_buf=0x%08x", 
	       py_buf, pu_buf, pv_buf, py_t_buf, pu_t_buf, pv_t_buf);

	MY_DBG("REG32(IPU_V_BASE + REG_CTRL)=%x, spage_map=%x, dpage_map=%x, lcdc_sel=%x",
	       REG32(IPU_V_BASE + REG_CTRL), spage_map, dpage_map, lcdc_sel);

	if (spage_map != 0) {
		MY_DBG();
		if ((py_t_buf == 0) || (pu_t_buf == 0) || (pv_t_buf == 0)) {
			printk
			    (" Can not found source map table, use no map now!\r\n");
			spage_map = 0;
			REG32(IPU_V_BASE + REG_CTRL) &= ~SPAGE_MAP;
		} else {
		MY_DBG();
			py_t_buf = PHYS((unsigned int) img->y_t_addr);
			pu_t_buf = PHYS((unsigned int) img->u_t_addr);
			pv_t_buf = PHYS((unsigned int) img->v_t_addr);

			py_buf = py_t_buf & 0xfff;
			pu_buf = pu_t_buf & 0xfff;
			pv_buf = pv_t_buf & 0xfff;

			REG32(IPU_V_BASE + REG_CTRL) |= SPAGE_MAP;
			// set phy table addr
			REG32(IPU_V_BASE + REG_Y_PHY_T_ADDR) = py_t_buf;
			REG32(IPU_V_BASE + REG_U_PHY_T_ADDR) = pu_t_buf;
			REG32(IPU_V_BASE + REG_V_PHY_T_ADDR) = pv_t_buf;
		}
	} else {
#if 0
		MY_DBG();
		if (py_buf == 0) {
			printk
			    ("++ py_buf Can not found buffer(0x%x,0x%x,0x%x) physical addr since addr errors +++\n",
			     (unsigned int) img->y_buf_p,
			     (unsigned int) img->u_buf_p,
			     (unsigned int) img->v_buf_p);
			//return (-1);
		}
		if ( in_fmt == IN_FMT_YUV420) {
			if ((pu_buf == 0) || (pv_buf == 0)) {
				printk
					("++ Can not found buffer(0x%x,0x%x,0x%x) physical addr since addr errors +++\n",
					 (unsigned int) img->y_buf_p,
					 (unsigned int) img->u_buf_p,
					 (unsigned int) img->v_buf_p);
				//return (-1);
			}
		}
//		REG32(IPU_V_BASE + REG_CTRL) &= ~SPAGE_MAP;
#endif
	}

	MY_DBG("py_buf=0x%08x, pu_buf=0x%08x, pv_buf=0x%08x, py_t_buf=0x%08x, pu_t_buf=0x%08x, pv_t_buf=0x%08x", 
	       py_buf, pu_buf, pv_buf, py_t_buf, pu_t_buf, pv_t_buf);
	//set Y,U,V addr register
	REG32(IPU_V_BASE + REG_Y_ADDR) = py_buf;
	REG32(IPU_V_BASE + REG_U_ADDR) = pu_buf;
	REG32(IPU_V_BASE + REG_V_ADDR) = pv_buf;

	REG32(IPU_V_BASE + REG_Y_STRIDE) = img->stride.y;
	REG32(IPU_V_BASE + REG_UV_STRIDE) = U_STRIDE(img->stride.u) | V_STRIDE(img->stride.v);

	// set out put
	if ((dpage_map != 0) && (lcdc_sel == 0)) {
		MY_DBG();
		if (PHYS((unsigned int) img->out_t_addr) == 0) {
			printk
			    (" Can not found destination map table, use no map now!\r\n");
			//      pipu_ctrl->dpage_map = 0;
			dpage_map = 0;
			REG32(IPU_V_BASE + REG_CTRL) &= ~DPAGE_MAP;

			if (PHYS((unsigned int) img->out_buf_p) == 0) {
				printk
				    ("Can not found the destination buf[%#x]\r\n",
				     (unsigned int) img->out_buf_p);
				return (-1);
			} else {
				REG32(IPU_V_BASE + REG_OUT_ADDR) =
				    PHYS((unsigned int) img->out_buf_p);
			}
		} else {
			REG32(IPU_V_BASE + REG_CTRL) |= DPAGE_MAP;
			REG32(IPU_V_BASE + REG_OUT_ADDR) =
			    PHYS((unsigned int) img->out_t_addr) & 0xfff;
			REG32(IPU_V_BASE + REG_OUT_PHY_T_ADDR) =
			    PHYS((unsigned int) img->out_t_addr);
		}
	} else {
		MY_DBG();
		dpage_map = 0;
		REG32(IPU_V_BASE + REG_CTRL) &= ~DPAGE_MAP;
		if (lcdc_sel == 0) {
			if (PHYS((unsigned int) img->out_buf_p) == 0) {
				printk
				    ("Can not found the destination buf[%#x]\r\n",
				     (unsigned int) img->out_buf_p);
				return (-1);
			} else {
				MY_DBG("img->out_buf_p=0x%x",img->out_buf_p);
				out_buf = img->out_buf_p;
				REG32(REG_OUT_ADDR + IPU_V_BASE) =
				    PHYS(out_buf);
			}
		}
	}
	REG32(IPU_V_BASE + REG_ADDR_CTRL) = 0xF; /* enable address reset */

	// flush the dcache
	// __dcache_writeback_all();
	LEAVE();
	return (0);
}


static int ipu_irq_cnt = 0;
static irqreturn_t ipu_interrupt_handler(int irq, void *dev_id)
{
	struct ipu_img_param_t *img;
	struct ipu_driver_priv *ipu = (struct ipu_driver_priv *) dev_id;
	unsigned long irq_flags;
	unsigned int dummy_read;

	dummy_read = REG32(IPU_V_BASE + REG_STATUS); /* avoid irq looping or disable_irq*/
	disable_ipu_irq(IPU_V_BASE); // failed
	img = &ipu->img;

	if (img->output_mode & IPU_OUTPUT_BLOCK_MODE) {
		spin_lock_irqsave(&ipu->update_lock, irq_flags);
	}

#if 0
	unsigned int ctrl;
	//      if (ipu_change_buf != 0)
	{
		if (((REG32(IPU_V_BASE + REG_STATUS)) & OUT_END) == 0) {
			printk("outend\n");
			return -1;
		}

		REG32(IPU_V_BASE + REG_STATUS) &= ~OUT_END;
		ctrl = REG32(IPU_V_BASE + REG_CTRL);
		if (!(ctrl & FIELD_SEL)) {
			// jz47_set_ipu_buf();
			REG32(IPU_V_BASE + REG_Y_ADDR) =
			    PHYS((unsigned int) img->y_buf);
			REG32(IPU_V_BASE + REG_U_ADDR) =
			    PHYS((unsigned int) img->u_buf);
			REG32(IPU_V_BASE + REG_V_ADDR) =
			    PHYS((unsigned int) img->v_buf);
			//REG32(IPU_V_BASE + REG_ADDR_CTRL) = YUV_READY;
		} else {
			REG32(IPU_V_BASE + REG_Y_ADDR) =
			    PHYS((unsigned int) img->y_buf + img->stride->y);
			REG32(IPU_V_BASE + REG_U_ADDR) =
			    PHYS((unsigned int) img->u_buf + img->stride->u);
			REG32(IPU_V_BASE + REG_V_ADDR) =
			    PHYS((unsigned int) img->v_buf + img->stride->v);
			//REG32(IPU_V_BASE + REG_ADDR_CTRL) = YUV_READY;

		}
		// ipu_change_buf = 0;
		//OUTREG32(A_IPU_Y_STRIDE, *py_stride);
		//OUTREG32(A_IPU_UV_STRIDE, U_STRIDE(*pu_stride) | V_STRIDE(*pv_stride));

		REG32(IPU_V_BASE + REG_CTRL) |= IPU_RUN;
		//printk(" %s\n", __FUNCTION__);

		//IPU_INTC_DISABLE();
	}
#else
//	printk("*** ipu_interrupt_handler() not implemented\n");
//	printk("REG32(IPU_V_BASE + REG_STATUS)=0x%08x\n", REG32(IPU_V_BASE + REG_STATUS));
#endif

	if (img->output_mode & IPU_OUTPUT_BLOCK_MODE) {
		ipu->frame_done = ipu->frame_requested;
		spin_unlock_irqrestore(&ipu->update_lock, irq_flags);
		wake_up(&ipu->frame_wq);
	}

	ipu_irq_cnt++;

//	printk("ipu_irq_cnt=%d, ipu->frame_requested=%d\n", ipu_irq_cnt, ipu->frame_requested);
//	printk("*** ipu_interrupt_handler() not implemented\n");
//	printk("REG32(IPU_V_BASE + REG_STATUS)=0x%08x, ipu->frame_done=%d\n", 
//	       REG32(IPU_V_BASE + REG_STATUS), ipu->frame_done);

	return IRQ_HANDLED;
}

void ipu_pic_enhance_table_set(int *tab)
{
	int i;
	int *ptr = tab;
	int reg_offset = REG_PIC_ENHANCE_T_ADDR;

	REG32(IPU_V_BASE + REG_CTRL) |= (PIC_ENHANCE_EN << PIC_ENHANCE_EN_SFT);

	for ( i = 0 ; i < 256; i ++)
	{
//		printk("penh: %x-%x\n", reg_offset - REG_PIC_ENHANCE_T_ADDR, ptr[i]);
		REG32(IPU_V_BASE + reg_offset) = ptr[i];
		reg_offset += 4;
	}
}

int jz47_ipu_init(struct ipu_driver_priv *ipu, struct ipu_img_param_t *img)
{
	int ret = -1;
	int in_fmt;
	int out_fmt;
	int outW, outH, Wdiff, Hdiff;

	ENTER();
	if (!img)
	{
		printk("ipu_init: parameter error\r\n");
		return (ret);
	}
	switch ( img->output_mode & IPU_OUTPUT_MODE_MASK) {
	case IPU_OUTPUT_TO_LCD_FG1://ipu_driver_priv
		if(ipu->lcd_info->panel.cfg & LCD_CFG_TVEN){			
			REG32(IPU_V_BASE + REG_CTRL) |= (LCDC_SEL | ADDR_SEL | DISP_SEL | FIELD_CONF_EN | FIELD_SEL);
		}
		else{
		   	REG32(IPU_V_BASE + REG_CTRL) |= (LCDC_SEL | ADDR_SEL);
		}
		break;
	default:
		REG32(IPU_V_BASE + REG_CTRL) |= ADDR_SEL; /*  */
	}

	outW = img->out_width;
	outH = img->out_height;
	Wdiff = img->Wdiff;
	Hdiff = img->Hdiff;

	MY_DBG("outW=%d, outH=%d, Wdiff=%d, Hdiff=%d", outW, outH, Wdiff, Hdiff);

	/* set src and dst format */
	in_fmt = hal_to_ipu_infmt(img->in_fmt);
	out_fmt = hal_to_ipu_outfmt(img->out_fmt);

	if ((in_fmt == IN_FMT_YUV444) && (out_fmt != OUT_FMT_YUV422)) {
		REG32(IPU_V_BASE + REG_CTRL) &= ~SPKG_SEL;
	}

	if ((in_fmt == IN_FMT_YUV422)) { /* YUV422 force to package format, Wolfgang, 2010-04-01 */
		REG32(IPU_V_BASE + REG_CTRL) |= SPKG_SEL;
	}

	MY_DBG("outW=%d, outH=%d, Wdiff=%d, Hdiff=%d", outW, outH, Wdiff, Hdiff);
	// set IPU resize field
	jz47_set_ipu_resize(ipu);
	MY_DBG("outW=%d, outH=%d, Wdiff=%d, Hdiff=%d", outW, outH, Wdiff, Hdiff);
	// set CSC parameter and format
	ret = jz47_set_ipu_csc_cfg(ipu, outW, outH, Wdiff, Hdiff);
	if (ret != 0) {
		printk("jz47_set_ipu_csc_cfg error : out!\n");
		return (ret);
	}
	// set  stride
	if (img->stride.y == 0) {	/* set default stride */

		if ( in_fmt == IN_FMT_YUV420_B) {
			REG32(IPU_V_BASE + REG_Y_STRIDE) = img->in_width*16;
		}
		else if (REG32(IPU_V_BASE + REG_CTRL) & SPKG_SEL) {
			REG32(IPU_V_BASE + REG_Y_STRIDE) = img->in_width * 2;
		}
		else {
			REG32(IPU_V_BASE + REG_Y_STRIDE) = img->in_width;
		}

		switch (in_fmt) {	///EMILY
		case IN_FMT_YUV420:
		case IN_FMT_YUV422:
			REG32(IPU_V_BASE + REG_UV_STRIDE) =
			    U_STRIDE(img->in_width /
				     2) | V_STRIDE(img->in_width / 2);
			break;
		case IN_FMT_YUV420_B:
			REG32(IPU_V_BASE + REG_UV_STRIDE) =
			    U_STRIDE(8*img->in_width) | V_STRIDE(8*img->in_width);
			break;
		case IN_FMT_YUV444:
			REG32(IPU_V_BASE + REG_UV_STRIDE) =
			    U_STRIDE(img->in_width) | V_STRIDE(img->in_width);
			break;
		case IN_FMT_YUV411:
			REG32(IPU_V_BASE + REG_UV_STRIDE) =
			    U_STRIDE(img->in_width /
				     4) | V_STRIDE(img->in_width / 4);
			break;
		default:
			printk("Error: 222 Input data format isn't support\n");
			return (-1);
		}
	} else {
		REG32(IPU_V_BASE + REG_Y_STRIDE) = img->stride.y;
		REG32(IPU_V_BASE + REG_UV_STRIDE) =
			U_STRIDE(img->stride.u) | V_STRIDE(img->stride.v);
	}

	// set the ctrl
//	REG32(IPU_V_BASE + REG_CTRL) |= BURST_SEL | OPMZ_SEL;
	REG32(IPU_V_BASE + REG_CTRL) |= BURST_SEL;
	REG32(IPU_V_BASE + REG_CTRL) = (REG32(IPU_V_BASE + REG_CTRL)&~(0x300)) | IPU_EN;

	if (in_fmt == IN_FMT_YUV420_B) {
		REG32(IPU_V_BASE + REG_D_FMT) |= 1 << 4; //BLK_SEL
		//printk("jz47xx_ipu: set BLK_SEL\n");
	}

//	jz47_dump_ipu_regs(ipu, -1);
//      jz47_dump_ipu_regs(ipu, -2); /* delay 1s */
	LEAVE();
	return (0);
}

int ipu_driver_open(struct ipu_driver_priv *ipu)
{
	ENTER();

	if (ipu == NULL) {
		return -1;
	}

	ipu = &g_ipu_native_data;
	ipu->inited = 0;

#if defined(CONFIG_FB_JZ4750_ANDROID_LCD)
	ipu->lcd_info = &jz4750_lcd_panel;
#elif defined(CONFIG_FB_JZ47XX_ANDROID_LCD)
	ipu->lcd_info = jz47xx_lcd_info;//&jz47xx_lcd_panel;
#endif

	ipu->restart_trigger = get_ipu_restart_trigger(&ipu->lcd_info->panel);

	return 0;
}


int ipu_driver_close(struct ipu_driver_priv *ipu)
{
	struct ipu_img_param_t *img;

	ENTER();
	if (ipu == NULL) {
		return -1;
	}

	ipu_driver_stop(ipu);

	img = &ipu->img;

	/* OSD operation, disable alpha, colorkey */
	if ( isOsd2LayerMode() ) {
		if (img->output_mode & IPU_DST_USE_COLOR_KEY) {
			__lcd_disable_colorkey0();
		}
		if (img->output_mode & IPU_DST_USE_ALPHA) {
			__lcd_disable_alpha();
		}
		if (img->output_mode & IPU_DST_USE_PERPIXEL_ALPHA) {
			__lcd_disable_alpha();
			__lcd_disable_alphamd();
		}
	}
		/* If JZ47XX's OSD MODE, enable LCDC's Compress-DeCompress */
#if defined(CONFIG_SOC_JZ4770)
		if ( isOsd2LayerMode() ) {
			lcd_disable_compress_decompress_mode();
		}
#endif

	ipu->inited = 0;

#ifdef IPU_DBG
	ipu_driver_dump_regs(ipu);
#endif

	return 0;
}

/*
 * ipu_init(), init ipu, not start ipu
 */

int ipu_driver_init(struct ipu_driver_priv *ipu, struct ipu_img_param_t *new_img)
{
	int ret;
	struct ipu_img_param_t *img;

	ENTER();
	if (ipu == NULL) {
		return -1;
	}


	spin_lock_init(&ipu->update_lock);
	init_waitqueue_head(&ipu->frame_wq);
	ipu->frame_requested = ipu->frame_done = 0;
	ipu_irq_cnt = 0;

	img = &ipu->img;
	*img = *new_img;	/* use the new parameter */

	MY_DBG("ipu->inited=%d", ipu->inited);
	if (ipu->inited == 0) {
		cpm_start_clock(CGM_IPU);
		REG32(IPU_V_BASE + REG_CTRL) |= IPU_EN;
		//jz47_ipu_wait_frame_end_flag(); /* clear OUT_END, start */
		MY_DBG("clear REG_STATUS");
		REG32(IPU_V_BASE + REG_STATUS) = 0;

	} //else 


	switch ( img->output_mode & IPU_OUTPUT_MODE_MASK) {
	case IPU_OUTPUT_TO_LCD_FG1:
		/* disable LCD_FG1 and set size*/
		if (REG_LCD_OSDC & LCD_OSDC_F1EN) 
			disable_lcd_controller_with_wait_lcd_frame_end();

		//REG32(IPU_V_BASE + REG_CTRL) |= IPU_STOP;
		//jz47_ipu_wait_frame_end_flag();

		/* If JZ47XX's OSD MODE, enable LCDC's Compress-DeCompress */
#if defined(CONFIG_SOC_JZ4770)
		if ( isOsd2LayerMode() ) {
			lcd_enable_compress_decompress_mode();
		}
#endif
		/* set ipu to fg1 */
		if (REG_LCD_OSDC & LCD_OSDC_F1EN)
			__lcd_disable_f1();
		__lcd_fg1_use_ipu();
		__lcd_enable_ipu_restart();
		__lcd_set_ipu_restart_triger(ipu->restart_trigger);


		if(img->output_mode & IPU_OUTPUT_MODE_FG1_TVOUT){
			ipu->lcd_info = jz47xx_lcd_info;
			ipu->fb_w = ipu->lcd_info->osd.fg1.w; /* fg0, fg1 ??? */
			ipu->fb_h = ipu->lcd_info->osd.fg1.h;
			img->out_x = ipu->lcd_info->osd.fg1.x;
			img->out_y = ipu->lcd_info->osd.fg1.y;
			img->out_width = ipu->lcd_info->osd.fg1.w;
			img->out_height = ipu->lcd_info->osd.fg1.h;
		}
		else
		{
			ipu->fb_w = ipu->lcd_info->osd.fg1.w; /* fg0, fg1 ??? */
			ipu->fb_h = ipu->lcd_info->osd.fg1.h;
			/* lcd controller configure */
			/* check dest rect? */
			if ( img->out_x > ipu->fb_w || img->out_y > ipu->fb_h) {
				printk("*** jz47xx ipu dest rect out range error.\n");
				//mdelay(20);			/* Test, Wolfgang, 2010-07-30 */
				__lcd_set_ena();
				return -1;
			}

			if ( img->out_x + img->out_width > ipu->fb_w )
				img->out_width = ipu->fb_w - img->out_x;
			if ( img->out_y + img->out_height > ipu->fb_h )
				img->out_height = ipu->fb_h - img->out_y;

		}
		resize_fg1_for_ipu_to_lcd(img->out_width, img->out_height);
		move_fg1_for_ipu_to_lcd(img->out_x, img->out_y);

		//mdelay(20);			/* Test, Wolfgang, 2010-07-30 */
		__lcd_set_ena();

		REG_LCD_STATE = 0;
		break;
	case IPU_OUTPUT_TO_LCD_FB0:
		break;
	case IPU_OUTPUT_TO_LCD_FB1:
		break;
	case IPU_OUTPUT_TO_FRAMEBUFFER:
		break;
	default:
		break;
	}

	//if ((REG32(IPU_V_BASE + REG_CTRL)) & IPU_RUN)
	//jz47_ipu_wait_frame_end_flag();
			
	REG32(IPU_V_BASE + REG_CTRL) |= IPU_RESET;	// reset controller
	REG32(IPU_V_BASE + REG_CTRL) &= ~IPU_RESET;

	ret = jz47_ipu_init(ipu, img);	// old init
	if (ret < 0) {
		return ret;
	}

	ipu->inited = 1;

#ifdef IPU_DBG
	ipu_driver_dump_regs(ipu);
#endif
	return 0;
}

int ipu_driver_reinit(struct ipu_driver_priv *ipu, struct ipu_img_param_t *new_img)
{
	int ret;
	struct ipu_img_param_t *img;

	ENTER();
	//printk("ENTER ipu_driver_deinit\n");

	if (ipu == NULL) {
		return -1;
	}
/*
	spin_lock_init(&ipu->update_lock);
	init_waitqueue_head(&ipu->frame_wq);
	ipu->frame_requested = ipu->frame_done = 0;
	ipu_irq_cnt = 0;
*/
	img = &ipu->img;
	*img = *new_img;	/* use the new parameter */

/**********************************************************************************/
	if (img->output_mode & IPU_OUTPUT_TO_LCD_FG1) {
		/* Disable lcd fg1 */
		if (REG_LCD_OSDC & LCD_OSDC_F1EN) {
			disable_lcd_controller_with_wait_lcd_frame_end();
			__lcd_disable_f1();
		   	__lcd_enable_f0();	/* Test */
			//udelay(50);
			//mdelay(1);			/* Test, Wolfgang, 2010-07-30 */
		}

		REG32(IPU_V_BASE + REG_CTRL) |= IPU_STOP;
		//jz47_ipu_wait_frame_end_flag();
	}
	else {
		jz47_ipu_wait_frame_end_flag();
	}

	REG32(IPU_V_BASE + REG_CTRL) &= ~IPU_RUN;

/**********************************************************************************/

	switch ( img->output_mode & IPU_OUTPUT_MODE_MASK) {
	case IPU_OUTPUT_TO_LCD_FG1:
		/* disable LCD_FG1 and set size*/
		if (REG_LCD_OSDC & LCD_OSDC_F1EN){ 
			disable_lcd_controller_with_wait_lcd_frame_end();
		}

		//REG32(IPU_V_BASE + REG_CTRL) |= IPU_STOP;
		//jz47_ipu_wait_frame_end_flag();

		/* If JZ47XX's OSD MODE, enable LCDC's Compress-DeCompress */
#if defined(CONFIG_SOC_JZ4770)
		if ( isOsd2LayerMode() ) {
			lcd_enable_compress_decompress_mode();
		}
#endif
		/* set ipu to fg1 */
		//printk("set ipu to fg1\n");
		if (REG_LCD_OSDC & LCD_OSDC_F1EN)
			__lcd_disable_f1();
		__lcd_fg1_use_ipu();
		__lcd_enable_ipu_restart();
		__lcd_set_ipu_restart_triger(ipu->restart_trigger);


		if(img->output_mode & IPU_OUTPUT_MODE_FG1_TVOUT){
			ipu->lcd_info = jz47xx_lcd_info;
			ipu->fb_w = ipu->lcd_info->osd.fg1.w; /* fg0, fg1 ??? */
			ipu->fb_h = ipu->lcd_info->osd.fg1.h;
			img->out_x = ipu->lcd_info->osd.fg1.x;
			img->out_y = ipu->lcd_info->osd.fg1.y;
			img->out_width = ipu->lcd_info->osd.fg1.w;
			img->out_height = ipu->lcd_info->osd.fg1.h;
		}
		else
		{
			ipu->fb_w = ipu->lcd_info->osd.fg1.w; /* fg0, fg1 ??? */
			ipu->fb_h = ipu->lcd_info->osd.fg1.h;
			/* lcd controller configure */
			/* check dest rect? */
			if ( img->out_x > ipu->fb_w || img->out_y > ipu->fb_h) {
				printk("*** jz47xx ipu dest rect out range error.\n");
				//mdelay(20);			/* Test, Wolfgang, 2010-07-30 */
				__lcd_set_ena();
				return -1;
			}

			if ( img->out_x + img->out_width > ipu->fb_w )
				img->out_width = ipu->fb_w - img->out_x;
			if ( img->out_y + img->out_height > ipu->fb_h )
				img->out_height = ipu->fb_h - img->out_y;

		}
		resize_fg1_for_ipu_to_lcd(img->out_width, img->out_height);
		move_fg1_for_ipu_to_lcd(img->out_x, img->out_y);

		//mdelay(20);			/* Test, Wolfgang, 2010-07-30 */

		REG_LCD_STATE = 0;
		break;
	case IPU_OUTPUT_TO_LCD_FB0:
		break;
	case IPU_OUTPUT_TO_LCD_FB1:
		break;
	case IPU_OUTPUT_TO_FRAMEBUFFER:
		break;
	default:
		break;
	}

	//if ((REG32(IPU_V_BASE + REG_CTRL)) & IPU_RUN)
	//jz47_ipu_wait_frame_end_flag();
			
	REG32(IPU_V_BASE + REG_CTRL) |= IPU_RESET;	// reset controller
	REG32(IPU_V_BASE + REG_CTRL) &= ~IPU_RESET;



	ret = jz47_ipu_init(ipu, img);	// old init
	if (ret < 0) {
		printk("jz47_ipu_reinit\n");
		return ret;
	}

	ret = jz47_set_ipu_buf(ipu, img);
	if (ret < 0) {
		printk("jz47_set_ipu_buf\n");
		return ret;
	}

/**********************************************************************************/
#if 0
	if (img->output_mode & IPU_OUTPUT_BLOCK_MODE) {
		/* Wait for current frame to finished */
		spin_lock_irqsave(&ipu->update_lock, irq_flags);
		ipu->frame_requested++;
		spin_unlock_irqrestore(&ipu->update_lock, irq_flags);
	}
#endif 

	/* LCDC_OSD 2 lyaer mode operation */
	if ( isOsd2LayerMode() ) {
		/* set alpha, colorkey */
		if (img->output_mode & IPU_DST_USE_COLOR_KEY ) {
			__lcd_set_colorkey0(img->colorkey);
			__lcd_enable_colorkey0();
		}
		if (img->output_mode & IPU_DST_USE_ALPHA) {
			if (img->alpha > 0xFF)
				img->alpha = 0xFF;
			REG_LCD_ALPHA = img->alpha;	/* Max: 0xFF(255) */
			__lcd_enable_alpha();
		}
		if (img->output_mode & IPU_DST_USE_PERPIXEL_ALPHA) {
			//REG_LCD_OSDC |= 1<<1; /* PerPixel Alpha */
			__lcd_enable_alphamd();
			__lcd_enable_alpha();
		}
		else {
			__lcd_disable_alphamd();
		}
	}

	if ( isIPUtoLcdcFG1Mode() ) {
		__lcd_enable_f1();
		if ( ! isOsd2LayerMode() ) {
			__lcd_disable_f0();		/* For test, Wolfgang */
		}
	}

	/* start ipu */
	REG32(IPU_V_BASE + REG_STATUS) &= ~OUT_END;

	__lcd_set_ena();

	if (img->output_mode & IPU_OUTPUT_BLOCK_MODE) {
		enable_ipu_irq(IPU_V_BASE);
	}

	MY_DBG("img->output_mode=0x%x", img->output_mode);

#ifdef IPU_DBG
	ipu_driver_dump_regs(ipu);
	print_lcdc_registers();	/* debug */
#endif

	REG32(IPU_V_BASE + REG_CTRL) |= IPU_RUN;
	

//	if (img->output_mode & IPU_OUTPUT_TO_LCD_FG1) {
//			mdelay(20);			/* Test, Wolfgang, 2010-07-30 */
//		__lcd_set_ena();
//	}

	MY_DBG();

	if (img->output_mode & IPU_OUTPUT_BLOCK_MODE) {
	MY_DBG();
		/* Wait for current frame to finished */
		if (ipu->frame_requested != ipu->frame_done)
			wait_event_interruptible_timeout(
				ipu->frame_wq, ipu->frame_done == ipu->frame_requested, HZ/10 + 1); /* HZ = 100 */
	}
/**********************************************************************************/   

#ifdef IPU_DBG
	ipu_driver_dump_regs(ipu);
#endif
	return 0;
}

int ipu_driver_deinit(struct ipu_driver_priv *ipu, struct ipu_img_param_t *new_img)
{
	ENTER();

	if (ipu == NULL) {
		return -1;
	}

	return 0;
}

int ipu_driver_start(struct ipu_driver_priv *ipu)
{
	struct ipu_img_param_t *img;
	unsigned long irq_flags;
	ENTER();

	if (ipu == NULL) {
		return -1;
	}

	img = &ipu->img;

	if (img->output_mode & IPU_OUTPUT_BLOCK_MODE) {
		/* Wait for current frame to finished */
		spin_lock_irqsave(&ipu->update_lock, irq_flags);
		ipu->frame_requested++;
		spin_unlock_irqrestore(&ipu->update_lock, irq_flags);
	}

	/* LCDC_OSD 2 lyaer mode operation */
	if ( isOsd2LayerMode() ) {
		/* set alpha, colorkey */
		if (img->output_mode & IPU_DST_USE_COLOR_KEY ) {
			__lcd_set_colorkey0(img->colorkey);
			__lcd_enable_colorkey0();
		}
		if (img->output_mode & IPU_DST_USE_ALPHA) {
			if (img->alpha > 0xFF)
				img->alpha = 0xFF;
			REG_LCD_ALPHA = img->alpha;	/* Max: 0xFF(255) */
			__lcd_enable_alpha();
		}
		if (img->output_mode & IPU_DST_USE_PERPIXEL_ALPHA) {
			//REG_LCD_OSDC |= 1<<1; /* PerPixel Alpha */
			__lcd_enable_alphamd();
			__lcd_enable_alpha();
		}
		else {
			__lcd_disable_alphamd();
		}
	}

	//if (img->output_mode & IPU_OUTPUT_TO_LCD_FG1) {
	if ( isIPUtoLcdcFG1Mode() ) {
		//__lcd_close_backlight();
		//REG_LCD_STATE &= ~LCD_STATE_EOF; // Clear previous EOF flag
		//while(!(__lcd_end_of_frame())) { }
		disable_lcd_controller_with_wait_lcd_frame_end();

		if (!(ipu->is_suspended)) {
			__lcd_enable_f1();/* when IPU is suspended do not enable f1 */
		}

		//if (img->output_mode & IPU_OUTPUT_MODE_FG0_OFF) {
		if ( ! isOsd2LayerMode() ) {
			__lcd_disable_f0();		/* For test, Wolfgang */
		}
	}


	/* start ipu */
	REG32(IPU_V_BASE + REG_STATUS) &= ~OUT_END;

	if (img->output_mode & IPU_OUTPUT_BLOCK_MODE) {
		enable_ipu_irq(IPU_V_BASE);
	}

	MY_DBG("img->output_mode=0x%x", img->output_mode);

#ifdef IPU_DBG
	ipu_driver_dump_regs(ipu);
	print_lcdc_registers();	/* debug */
#endif

	REG32(IPU_V_BASE + REG_CTRL) |= IPU_RUN;
	if (img->output_mode & IPU_OUTPUT_TO_LCD_FG1) {
//			mdelay(20);			/* Test, Wolfgang, 2010-07-30 */
		__lcd_set_ena();
	}

	MY_DBG();

	if (img->output_mode & IPU_OUTPUT_BLOCK_MODE) {
	MY_DBG();
		/* Wait for current frame to finished */
		if (ipu->frame_requested != ipu->frame_done)
			wait_event_interruptible_timeout(
				ipu->frame_wq, ipu->frame_done == ipu->frame_requested, HZ/10 + 1); /* HZ = 100 */
	}
	LEAVE();
	return 0;
}

int ipu_driver_stop(struct ipu_driver_priv *ipu)
{
	struct ipu_img_param_t *img;
        int timeout = 20;
	ENTER();

	if (ipu == NULL) {
		return -1;
	}

	img = &ipu->img;

	if (img->output_mode & IPU_OUTPUT_TO_LCD_FG1) {
		/* Disable lcd fg1 */
		if (!ipu_is_suspended() && (REG_LCD_OSDC & LCD_OSDC_F1EN)) {
			if (!(REG_LCD_STATE & LCD_STATE_QD) && !(REG_LCD_STATE & LCD_STATE_LDD)) { /* LCD is not diabled */
				REG_LCD_STATE &= ~LCD_STATE_EOF; // Clear previous EOF flag
				while(!(__lcd_end_of_frame()) && (--timeout )) {
					msleep(1);
				}
			}
			
			disable_lcd_controller_with_wait_lcd_frame_end();
			__lcd_disable_f1();
			__lcd_enable_f0();	/* Test */
			udelay(50);
			//mdelay(1);			/* Test, Wolfgang, 2010-07-30 */
			__lcd_set_ena();
		}

		REG32(IPU_V_BASE + REG_CTRL) |= IPU_STOP;
		//jz47_ipu_wait_frame_end_flag();
	}
	else {
		jz47_ipu_wait_frame_end_flag();
	}

	REG32(IPU_V_BASE + REG_CTRL) &= ~IPU_RUN;

	return 0;
}

int ipu_driver_swapBuffer(struct ipu_driver_priv *ipu, struct ipu_img_param_t *new_img)
{
	ENTER();

	if (ipu == NULL) {
		return -1;
	}

	return jz47_set_ipu_buf(ipu, new_img);
}

int ipu_driver_setBuffer(struct ipu_driver_priv *ipu, struct ipu_img_param_t *new_img)
{
	ENTER();

	if (ipu == NULL) {
		return -1;
	}


	return jz47_set_ipu_buf(ipu, new_img);
}

int ipu_driver_setBufferSize(struct ipu_driver_priv *ipu, struct ipu_img_param_t *new_img)
{
	struct ipu_img_param_t *img;
	int outW;
	int out_fmt;

	ENTER();
	if (ipu == NULL || new_img == NULL) {
		return -1;
	}

	img = new_img;

	outW = img->out_width;
	out_fmt = hal_to_ipu_outfmt(img->out_fmt);
	switch (out_fmt) {
	case OUT_FMT_RGB888:
		outW = outW << 2;
		break;
	case OUT_FMT_RGB555:
		outW = outW << 1;
		break;
	case OUT_FMT_RGB565:
		outW = outW << 1;
		break;
	}

	/* width and height Maybe change */
	// Set GS register
	REG32(IPU_V_BASE + REG_IN_FM_GS) =
	    IN_FM_W(img->in_width) | IN_FM_H((img->in_height) & ~0x1);
	REG32(IPU_V_BASE + REG_OUT_GS) = OUT_FM_W(outW) | OUT_FM_H(img->out_height);




	return 0;
}

int ipu_driver_resize(struct ipu_driver_priv *ipu, struct ipu_img_param_t *new_img)
{
	int ret;
	ENTER();

	if (ipu == NULL) {
		return -1;
	}

	ret = ipu_driver_init(ipu, new_img);
	LEAVE();
	return ret;
}


int ipu_driver_register_irq(struct ipu_driver_priv *ipu)
{
	int ret;
	ret = request_irq(IRQ_IPU, ipu_interrupt_handler, IRQF_DISABLED, "ipu", (void*)ipu);
	if ( ret ) {
		printk("Error: request_irq IRQ_IPU=%d\n", IRQ_IPU);
	}

	return ret;
}


int ipu_driver_dump_regs(struct ipu_driver_priv *ipu)
{
	ENTER();
	print_lcdc_registers();
	if (ipu == NULL) {
		return -1;
	}

	print_img(ipu);
	jz47_dump_ipu_regs(ipu, -1);
//	jz47_dump_ipu_regs(ipu, -2);
//	jz47_dump_ipu_regs(ipu, -3);

	return 0;
}
/* If change size, call this function */
static void copy_ipu_tabel_from_user(struct ipu_driver_priv *ipu, struct ipu_img_param_t *img)
{
	copy_from_user((void *)ipu->hoft_table,
		       (void *)img->hoft_table,
		       sizeof(ipu->hoft_table));
	copy_from_user((void *)ipu->voft_table,
		       (void *)img->voft_table,
		       sizeof(ipu->voft_table));
	copy_from_user((void *)ipu->pic_enhance_table,
		       (void *)img->pic_enhance_table,
		       sizeof(ipu->pic_enhance_table));

	if (img->zoom_mode != ZOOM_MODE_BILINEAR) {
		copy_from_user((void *)ipu->cube_hcoef_table,
			       (void *)img->cube_hcoef_table,
			       sizeof(ipu->cube_hcoef_table));
		copy_from_user((void *)ipu->cube_vcoef_table,
			       (void *)img->cube_vcoef_table,
			       sizeof(ipu->cube_vcoef_table));
	}
	else {
		copy_from_user((void *)ipu->hcoef_table,
			       (void *)img->hcoef_table,
			       sizeof(ipu->hcoef_table));
		copy_from_user((void *)ipu->vcoef_table,
			       (void *)img->vcoef_table,
			       sizeof(ipu->vcoef_table));
	}
}


int ipu_driver_suspend(struct ipu_driver_priv *ipu)
{
//	printk("ENTER ipu_driver_suspend()\n");
//	struct ipu_driver_priv *ipu = &g_ipu_native_data;

	ipu->is_suspended = 1;

	/* Stop IPU if IPU running*/
	if ((REG_LCD_OSDC & LCD_OSDC_F1EN)
		&& (REG32(IPU_V_BASE + REG_CTRL) & (LCDC_SEL))
		) {
		printk("*** ipu_driver_suspend() Run IPU.\n");
		printk("ipu REG_CTRL = 0x%08x REG_STATUS= 0x%08x\n", 
			   REG32(IPU_V_BASE + REG_CTRL), 
			   REG32(IPU_V_BASE + REG_STATUS)
			);

		REG32(IPU_V_BASE + REG_CTRL) |= IPU_STOP;
		//jz47_ipu_wait_frame_end_flag();
	}

	cpm_stop_clock(CGM_IPU);

	return 0;
}

int ipu_driver_resume(struct ipu_driver_priv *ipu)
{
//	printk("ENTER ipu_driver_resume()\n");
	cpm_start_clock(CGM_IPU);

	/* run IPU if IPU running */
	if ((REG_LCD_OSDC & LCD_OSDC_F1EN)
		&& (REG32(IPU_V_BASE + REG_CTRL) & (LCDC_SEL))
		) {
		printk("*** ipu_driver_resume() Run IPU.\n");
		printk("ipu REG_CTRL = 0x%08x REG_STATUS= 0x%08x\n", 
			   REG32(IPU_V_BASE + REG_CTRL),
			   REG32(IPU_V_BASE + REG_STATUS)
			);

		REG32(IPU_V_BASE + REG_STATUS) &= ~OUT_END;

		REG32(IPU_V_BASE + REG_CTRL) |= IPU_RUN;
		//jz47_ipu_wait_frame_end_flag();
	}

	ipu->is_suspended = 0;

	return 0;
}


int ipu_driver_ioctl(struct ipu_driver_priv *ipu, void *uimg)
{
	unsigned int cmd;
	int ret = 0;
	struct ipu_img_param_t *m_img;
	struct ipu_img_param_t ipu_img;

	if (uimg == NULL) {
		return -1;
	}

	copy_from_user((void *) &ipu_img, (void *)uimg,
		       sizeof(struct ipu_img_param_t));

	m_img = &ipu->img;
	if (ipu_img.version != m_img->version) {
		printk("*** Warning, ipu_img_param_t.version wrong ***\n");
		return -1;
	}

	cmd = ipu_img.ipu_cmd;
	MY_DBG("ipu_ioctl cmd= %d\n", cmd);

/* suspended operation

	switch (cmd) {
	case IOCTL_IPU_START:
	case IOCTL_IPU_SET_BUFF:
		if ( ipu_is_suspended() ) {
			printk("ipu_suspended, no respond to operate= 0x%x\n", cmd);
			return -1;
		}
		break;
		//default:
	}
*/
	switch (cmd) {
	case IOCTL_IPU_OPEN:
		ret = ipu_driver_open(ipu);
		break;
	case IOCTL_IPU_CLOSE:
		ret = ipu_driver_close(ipu);
		break;
	case IOCTL_IPU_INIT:
		copy_ipu_tabel_from_user(ipu, &ipu_img);
		ret = ipu_driver_init(ipu, &ipu_img);
		break;
	case IOCTL_IPU_DEINIT:
		break;
	case IOCTL_IPU_REINIT:
		copy_ipu_tabel_from_user(ipu, &ipu_img);
		ret = ipu_driver_reinit(ipu, &ipu_img);
	case IOCTL_IPU_SET_BUFF:
		ret = ipu_driver_setBuffer(ipu, &ipu_img);
		break;
	case IOCTL_IPU_SET_BUFF_SIZE:
		ret = ipu_driver_setBufferSize(ipu, &ipu_img);
		break;
	case IOCTL_IPU_START:
		ret = ipu_driver_start(ipu);
		break;
	case IOCTL_IPU_STOP:
		ret = ipu_driver_stop(ipu);
		break;
	case IOCTL_IPU_RESIZE:
		copy_ipu_tabel_from_user(ipu, &ipu_img);
		ipu_driver_resize(ipu, &ipu_img);
		break;
	case IOCTL_IPU_SET_CTRL_REG:
		break;
	case IOCTL_IPU_SET_FMT_REG:
		break;
	case IOCTL_IPU_SET_CSC:
		break;
	case IOCTL_IPU_SET_STRIDE:
		break;
	case IOCTL_IPU_FB_SIZE:
		break;
	case IOCTL_IPU_PENDING_OUTEND:
		jz47_ipu_wait_frame_end_flag();
		REG32(IPU_V_BASE + REG_STATUS) &= ~OUT_END;
		break;
	case IOCTL_IPU_GET_ADDR:
		printk(" IOCTL_IPU_GET_ADDR:%s[%d] function droped.\n",
		       __FILE__, __LINE__);
		//get_ipu_addr((unsigned int*)buff);
		break;
	case IOCTL_IPU_DUMP_REGS:
		ret = ipu_driver_dump_regs(ipu);
		break;
	default:
		printk("ipu not support ioctl cmd[%#x]\r\n", cmd);
		return (-1);
		break;
	}

	return ret;
}
