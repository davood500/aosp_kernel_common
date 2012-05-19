
/*
 * kernel/drivers/video/jz47xx_android_lcd.h -- Ingenic Jz4760 On-Chip LCD frame buffer device
 *
 * Copyright (C) 2005-2010, Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __JZ47XX_ANDROID_LCD_H__
#define __JZ47XX_ANDROID_LCD_H__

#include <asm/jzsoc.h>

#define NR_PALETTE	256
#define PALETTE_SIZE	(NR_PALETTE*2)

//extern struct lcd_panel_ops;

/* use new descriptor(8 words) */
struct jz47xx_lcd_dma_desc {
	unsigned int next_desc; 	/* LCDDAx */
	unsigned int databuf;   	/* LCDSAx */
	unsigned int frame_id;  	/* LCDFIDx */ 
	unsigned int cmd; 		/* LCDCMDx */
	unsigned int offsize;       	/* Stride Offsize(in word) */
	unsigned int page_width; 	/* Stride Pagewidth(in word) */
	unsigned int cmd_num; 		/* Command Number(for SLCD) */
	unsigned int desc_size; 	/* Foreground Size */
};

struct jz47xxlcd_panel_t {
	unsigned int cfg;	/* panel mode and pin usage etc. */
	unsigned int slcd_cfg;	/* Smart lcd configurations */
	unsigned int ctrl;	/* lcd controll register */
	unsigned int phys_width;/* width of screen physical size, in mm */
	unsigned int phys_height;/* height of screen physical size, in mm */
	unsigned int w;		/* Panel Width(in pixel) */
	unsigned int h;		/* Panel Height(in line) */
	unsigned int fclk;	/* frame clk */
	unsigned int hsw;	/* hsync width, in pclk */
	unsigned int vsw;	/* vsync width, in line count */
	unsigned int elw;	/* end of line, in pclk */
	unsigned int blw;	/* begin of line, in pclk */
	unsigned int efw;	/* end of frame, in line count */
	unsigned int bfw;	/* begin of frame, in line count */
};


struct jz47xxlcd_fg_t {
	int bpp;	/* foreground bpp */
	int x;		/* foreground start position x */
	int y;		/* foreground start position y */
	int w;		/* foreground width */
	int h;		/* foreground height */
};

struct jz47xxlcd_osd_t {
	unsigned int osd_cfg;	/* OSDEN, ALHPAEN, F0EN, F1EN, etc */
	unsigned int osd_ctrl;	/* IPUEN, OSDBPP, etc */
	unsigned int rgb_ctrl;	/* RGB Dummy, RGB sequence, RGB to YUV */
	unsigned int bgcolor;	/* background color(RGB888) */
	unsigned int colorkey0;	/* foreground0's Colorkey enable, Colorkey value */
	unsigned int colorkey1; /* foreground1's Colorkey enable, Colorkey value */
	unsigned int alpha;	/* ALPHAEN, alpha value */
	unsigned int ipu_restart; /* IPU Restart enable, ipu restart interval time */

#define FG_NOCHANGE 		0x0000
#define FG0_CHANGE_SIZE 	0x0001
#define FG0_CHANGE_POSITION 	0x0002
#define FG1_CHANGE_SIZE 	0x0010
#define FG1_CHANGE_POSITION 	0x0020
#define FG_CHANGE_ALL 		( FG0_CHANGE_SIZE | FG0_CHANGE_POSITION | \
				  FG1_CHANGE_SIZE | FG1_CHANGE_POSITION )
	int fg_change;
	struct jz47xxlcd_fg_t fg0;	/* foreground 0 */
	struct jz47xxlcd_fg_t fg1;	/* foreground 1 */
};

struct jz47xxlcd_info {
	struct jz47xxlcd_panel_t panel;
	struct jz47xxlcd_osd_t osd;
};


/***********************Emily****************************/
#define MAX_SIZE		(1024 * 720)

/* JZ_ANDROID_PANELNUM remove ??? */
#ifdef	CONFIG_JZ47XX_HDMI_DISPLAY
#define JZ_ANDROID_PANELNUM 4
#else
#ifdef CONFIG_FB_JZ47XX_TVE
#define JZ_ANDROID_PANELNUM 3
#else
#define JZ_ANDROID_PANELNUM 1
#endif
#endif


#define DISPLAY_TYPE_MASK	0xffff0000
#define DISPLAY_INDEX_MASK	0x0000ffff


#define PANEL_MODE_LCD_BASE     0x0001
#define PANEL_MODE_TVE_BASE     0x0002
#define PANEL_MODE_HDMI_BASE    0x0004

#define PANEL_MODE_LCD_PANEL    0x00010001

#define BUILD_HDMI_TAG(index)   ((PANEL_MODE_HDMI_BASE << 16) | index)
#define BUILD_TVE_TAG(index)    ((PANEL_MODE_TVE_BASE <<16) | index)

#define GET_PANEL_INDEX(index)  (index & 0x0000ffff)
#define GET_PANEL_TYPE(index)   ((index & 0xffff0000) >> 16)

#define INDEX_IS_HDMI(index) ((GET_PANEL_TYPE(index) == PANEL_MODE_HDMI_BASE)?true:false) 
#define INDEX_IS_TVE(index) ((GET_PANEL_TYPE(index) == PANEL_MODE_TVE_BASE)?true:false) 
#define INDEX_IS_LCD(index) ((GET_PANEL_TYPE(index) == PANEL_MODE_LCD_BASE)?true:false) 

struct jz_android_din_t{
	unsigned int x;
	unsigned int y;
	unsigned int w;
	unsigned int h;
	unsigned int index;
};

struct android_display_info_t {
	unsigned int flag;
	unsigned int fg0_number; /**/
	unsigned int fg0_index;
	unsigned int fg0_alpha; /* */
	unsigned int fg0_colorkey;/**/
	unsigned int fg0_enable;/**/
	unsigned int fg0_x;        /*fg0 start position x*/
	unsigned int fg0_y;        /*fg0 start position y*/
	unsigned int fg0_w;        /*the weight of fg0*/
	unsigned int fg0_h;        /*the height of fg0*/
	unsigned int fg1_x;        /*fg1 start position x*/
	unsigned int fg1_y;        /*fg1 start position y*/
	unsigned int fg1_w;        /*the weight of fg1*/
	unsigned int fg1_h;        /*the height of fg1*/
	unsigned int fg1_enable;   /*start or stop fg1*/
    unsigned int cmd;          /*for ipu_ioctl command */
    void *       data_buf;     /*for ipu_ioctl*/
};

#define FBIO_ANDROID_CTL                        0xad10

#define ANDROID_GET_DISPLAY_NUM 		0x00000001 
#define ANDROID_GET_DISPLAY_INFO 		0x00000002 
#define ANDROID_SET_DISPLAY_INDEX		0x00000004 
#define ANDROID_SET_FG0_ALPHA   		0x00000008
#define ANDROID_SET_FG0_COLORKEY  		0x00000010
#define ANDROID_SET_FG0_ENABLE    		0x00000020
#define ANDROID_SET_FG1_POS       		0x00000040
#define ANDROID_SET_FG1_SIZE       		0x00000080
#define ANDROID_SET_FG1_ENABLE      		0x00000100
#define ANDROID_SET_FG1_IPU_DIRECT      	0x00000200
#define ANDROID_GET_PANEL_SIZE                  0x00000400
#define ANDROID_IPU_IOCTL                       0x00000800
#define ANDROID_FG1_RESIZE                      0x00001000
//#define ANDROID_SET_IPU                         0x00002000
//#define ANDROID_SET_IPU_VBASE                   0x00004000
#define ANDROID_SET_LCD_PANEL                   0x00008000
#define BARRIER __asm__ __volatile__(".set noreorder\n\t" \
				     "nop; nop; nop; nop; nop; nop;\n\t" \
				     ".set reorder\n\t")
extern void local_flush_tlb_all(void);
unsigned int jz_lcd_get_width(void);

unsigned int jz_lcd_get_height(void);
/******************************Emily**************************************/

/* Jz LCDFB supported I/O controls. */
#define FBIOSETBACKLIGHT	0x4688 /* set back light level */
#define FBIODISPON		0x4689 /* display on */
#define FBIODISPOFF		0x468a /* display off */
#define FBIORESET		0x468b /* lcd reset */
#define FBIOPRINT_REG		0x468c /* print lcd registers(debug) */
#define FBIOROTATE		0x46a0 /* rotated fb */
#define FBIOGETBUFADDRS		0x46a1 /* get buffers addresses */
#define FBIO_GET_MODE		0x46a2 /* get lcd info */
#define FBIO_SET_MODE		0x46a3 /* set osd mode */
#define FBIO_DEEP_SET_MODE	0x46a4 /* set panel and osd mode */
#define FBIO_MODE_SWITCH	0x46a5 /* switch mode between LCD and TVE */
#define FBIO_GET_TVE_MODE	0x46a6 /* get tve info */
#define FBIO_SET_TVE_MODE	0x46a7 /* set tve mode */
#define FBIODISON_FG		0x46a8 /* FG display on */
#define FBIODISOFF_FG		0x46a9 /* FG display on */
#define FBIO_SET_LCD_TO_TVE	0x46b0 /* set lcd to tve mode */
#define FBIO_SET_FRM_TO_LCD	0x46b1 /* set framebuffer to lcd */
#define FBIO_SET_IPU_TO_LCD	0x46b2 /* set ipu to lcd directly */
#define FBIO_CHANGE_SIZE	0x46b3 /* change FG size */
#define FBIO_CHANGE_POSITION	0x46b4 /* change FG starts position */
#define FBIO_SET_BG_COLOR	0x46b5 /* set background color */
#define FBIO_SET_IPU_RESTART_VAL	0x46b6 /* set ipu restart value */
#define FBIO_SET_IPU_RESTART_ON	0x46b7 /* set ipu restart on */
#define FBIO_SET_IPU_RESTART_OFF	0x46b8 /* set ipu restart off */
#define FBIO_ALPHA_ON		0x46b9 /* enable alpha */
#define FBIO_ALPHA_OFF		0x46c0 /* disable alpha */
#define FBIO_SET_ALPHA_VAL	0x46c1 /* set alpha value */
#define FBIO_WAIT_LCD_SWAP_SIGNAL 0x46c2 /* wait until end of frambuffer interrupt signal,Rill add for surfaceFlinger 100719*/
#define FBIO_CAPTION_ON    0x4700 /* change to hdmi caption FB */
#define FBIO_CAPTION_OFF   0x4701 /* change to UI FB */
#define FBIO_GET_POSITION  0x4702 /* get FG position */
#define FBIO_GET_BUFF_SIZE 0x4703 /* get buffer size */
#define FBIO_DISABLE_LCDC  0x4704 /* disable lcdc */
#define FBIO_ENABLE_LCDC   0x4705 /* enable lcdc */
#define FBIO_UPDATE_CAPTION 0x4706 /* update hdmi caption */

#define FBIO_GET_FPS       0x5000 /* get fps */

/* /\** */
/*  * Soc specific definition */
/*  *  */
/*  * In order to achieve soc-independent lcd panel drivers,  */
/*  *  re-define the soc specific operations here, so the lcd panels  */
/*  *  can use them instead of using the soc operations directly. */
/*  *\/ */
/* void gpio_as_output(unsigned int pin) */
/* { */
/* 	__gpio_as_output(pin); */
/* } */

/* void gpio_as_input(unsigned int pin) */
/* { */
/* 	__gpio_as_input(pin); */
/* } */

/* void gpio_set_pin(unsigned int pin) */
/* { */
/* 	__gpio_as_output(pin); */
/* 	__gpio_set_pin(pin); */
/* } */

/* int gpio_get_pin(unsigned int pin) */
/* { */
/* 	return __gpio_get_pin(pin); */
/* } */

/* void gpio_clear_pin(unsigned int pin) */
/* { */
/* 	__gpio_as_output(pin); */
/* 	__gpio_clear_pin(pin); */
/* } */

/* void gpio_as_lcd_16bit(void) */
/* { */
/* 	__gpio_as_lcd_16bit(); */
/* } */

/* void gpio_as_lcd_18bit(void) */
/* { */
/* 	__gpio_as_lcd_18bit(); */
/* } */

/* void gpio_as_lcd_24bit(void) */
/* { */
/* 	__gpio_as_lcd_24bit(); */
/* } */

/*****************************************************************************
 * LCD display pin dummy macros
 *****************************************************************************/
#ifndef __lcd_slcd_special_on
#define __lcd_slcd_special_on()
#endif

extern void print_lcdc_registers(void);	/* debug */

void lcd_enable_compress_decompress_mode(void);
void lcd_disable_compress_decompress_mode(void);
int lcd_should_use_compress_decompress_mode(void);


#endif /* __JZ47XX_ANDROID_LCD_H__ */
