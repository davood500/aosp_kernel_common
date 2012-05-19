
/*
 * kernel/drivers/video/jz47xx_android_lcd.c -- Ingenic Jz4770 LCD frame buffer device
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
 */
/*
 * --------------------------------
 * NOTE:
 * This LCD driver support TFT16 TFT32 LCD, not support STN and Special TFT LCD
 * now.
 * 	It seems not necessory to support STN and Special TFT.
 * 	If it's necessary, update this driver in the future.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/gfp.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <linux/earlysuspend.h>
#include <linux/pm.h>
#include <linux/leds.h>
#include <linux/wait.h>
#include <linux/timer.h>
#include <linux/list.h>

#include <linux/jz_lcd.h>
#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/processor.h>
#include <asm/cacheflush.h>
#include <asm/jzsoc.h>

#include "console/fbcon.h"
#include "jz47xx_android_lcd.h"
#include "jz47xx_tve.h"
#include "jz47xx_hdmi.h"
#include "jz_ipu.h"

#if defined(CONFIG_SOC_JZ4770)
#include <linux/time.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <asm/time.h>
#include "jz47xx_aosd.h"
#endif

#include "jz47xx_lcd_swapbuffer_counter.h"

MODULE_DESCRIPTION("Jz4760 LCD Controller driver");
MODULE_AUTHOR("Wolfgang Wang <lgwang@ingenic.cn>, Lemon Liu <zyliu@ingenic.cn>, Emily Feng <clfeng@ingenic.cn>");
MODULE_LICENSE("GPL");

//#define LCD_DEBUG
#undef LCD_DEBUG
//#define CALC_COMP_RATIO
#undef CALC_COMP_RATIO


/* 32bpp RGB ORDER */
#define FORMAT_X8R8G8B8             1
#define FORMAT_X8B8G8R8             2
//#define BPP32_FORMAT_ORDER  FORMAT_X8R8G8B8
#define BPP32_FORMAT_ORDER  FORMAT_X8B8G8R8


#define AOSD_ALIGN (64 * 4)		/* AOSD 64 words aligned */
#define FB_ALIGN (64 * 4)			/* FrameBuffer 64 words aligned */


#ifndef LCD_PANEL_POWER_ON_STABLE_TIME
#define LCD_PANEL_POWER_ON_STABLE_TIME 200 /* default 200ms */
#endif

#ifdef CONFIG_JZSOC_BOOT_LOGO
extern int load_565_image(char *filename,unsigned char*);

//#define BOOT_ANIMATION_TEST	 /* xyang, 2010-07-30*/

#ifdef BOOT_ANIMATION_TEST
#define ANIMATION_BUF_NUM 4 /*at lease one */
char *logofilename[4]={"/logo0.rle","/logo1.rle","/logo2.rle","/logo3.rle"};
#endif /* #ifdef BOOT_ANIMATION_TEST	 */

#ifdef CONFIG_FB_565RLE_LOGO
#define INIT_IMAGE_FILE "/logo.rle"
#endif
#ifdef CONFIG_FB_565RGB_LOGO
#define INIT_IMAGE_FILE "/logo.rgb565"
#endif

#endif /* CONFIG_JZSOC_BOOT_LOGO */

#ifdef LCD_DEBUG
#define dprintk(x...)	printk(x)
#define print_dbg(f, arg...) printk("dbg::" __FILE__ ",LINE(%d): " f "\n", __LINE__, ## arg)
#define ENTER() printk("%d %s ENTER\n", __LINE__, __FUNCTION__)
#define LEAVE() printk("%d %s LEAVE\n", __LINE__, __FUNCTION__)
#else
#define dprintk(x...)
#define print_dbg(f, arg...) do {} while (0)
#define ENTER()
#define LEAVE()
#endif

#define print_err(f, arg...) printk(KERN_ERR DRIVER_NAME ": " f "\n", ## arg)
#define print_warn(f, arg...) printk(KERN_WARNING DRIVER_NAME ": " f "\n", ## arg)
#define print_info(f, arg...) printk(KERN_INFO DRIVER_NAME ": " f "\n", ## arg)


#define JZ_LCD_ID "jz-lcd"
#define ANDROID_NUMBER_OF_BUFFERS 2

#define USE_CHANGE_FOR_SURFACEFLINGER 1	/* Rill add for surfaceFlinger 100719 */

#define CMD_LCD_PANEL_INFO_CHANGE   0x1 /* add for hdmi test */
#define CMD_LCD_HDMI_SYNC_OUTPUT   0x2


/* flush dcache with prefetch allocate */
#define CFG_DCACHE_SIZE  16384
void flush_dcache_with_prefetch_allocate(void)
{
	int addr;

	for (addr = KSEG0; addr < (KSEG0 + CFG_DCACHE_SIZE); addr += 256) { /* 256 = 32byte * 8 */
		asm ( ".set\tmips32\n\t"
		      "pref %0,	0(%1)\n\t"
		      "pref %0,	32(%1)\n\t"
		      "pref %0,	64(%1)\n\t"
		      "pref %0,	96(%1)\n\t"
		      "pref %0, 128(%1)\n\t"
		      "pref %0, 160(%1)\n\t"
		      "pref %0, 192(%1)\n\t"
		      "pref %0, 224(%1)\n\t"
		      :
		      : "I" (30), "r"(addr));
	}
}

/* ============================================= */

#ifdef CONFIG_JZ47XX_AOSDC
struct jz47xx_aosd_info *aosd_info;
wait_queue_head_t compress_wq;
unsigned char *buf_comp, *buf_comp1; /* buffer for compress output */
static unsigned int buf_comp_needroom = 0, buf_comp1_needroom = 0;
#endif

#define DRIVER_NAME	"jz-lcd"
volatile static int lcd_backlight_is_suspend = 0;
//static unsigned int first_frame = 1;

struct jz_lcd_module_state{
	unsigned int backlight;
	unsigned int panel;
	unsigned int controller;
};
static struct jz_lcd_module_state jz_lcd_stat;

/* used to notice lcd_set_backlight_level() */
static wait_queue_head_t wait_backlight_resume;
/* ============================================= */

#if (defined(CONFIG_JZ47XX_Z800))
static unsigned int is_backlight_closed = 0;
static int current_backlight_level = 88;
#endif

struct lcd_cfb_info {
	struct fb_info		fb0;	/* foreground 0 */
	struct display_switch	*dispsw;
	signed int		currcon;
	int			func_use_count;
	spinlock_t	update_lock;
	unsigned 	frame_requested;
	unsigned 	frame_done;
	wait_queue_head_t frame_wq;
	struct early_suspend earlier_suspend;
	struct early_suspend early_suspend;
	struct {
		u16 red, green, blue;
	} palette[NR_PALETTE];
};

static struct lcd_cfb_info *jz47xxfb_info;

struct lcd_board_info *lcd_board;
struct lcd_panel_info *lcd_panel;

static unsigned int SPEN;
static unsigned int SPRS;
static unsigned int SPCK;
static unsigned int SPDT;

static int current_dma0_id, current_dma1_id;
static struct jz47xx_lcd_dma_desc *dma_desc_base;
static struct jz47xx_lcd_dma_desc *dma0_desc_palette, *dma0_desc0, *dma0_desc1, *dma1_desc0, *dma1_desc1;
static struct jz47xx_lcd_dma_desc *dma0_desc0_change, *dma1_desc0_change, *dma0_desc1_change, *dma1_desc1_change;

static struct list_head _panels = LIST_HEAD_INIT(_panels);

#define DMA_DESC_NUM 		9
//#define IMEM_MAX_ORDER         12 /*16M*/

static unsigned char *lcd_palette;
static unsigned char *lcd_frame0;
unsigned char *lcd_frame;//Emily
static unsigned int g_next_frameID = 0;
static unsigned int fb_needroom = 0;

#ifdef BOOT_ANIMATION_TEST
struct timer_list animation_timer; 
unsigned char *animation_frame_buffer;
static unsigned char *animation_buf[ANIMATION_BUF_NUM-1];
static unsigned int animation_page_shift;
#endif

//static int lcd_controller_is_enabled = 0;
//static int should_set_backlight = 0;
static spinlock_t lcd_controller_startup_update_lock;

#define DEFAULT_BACKLIGHT_LEVEL 80
static int lcd_backlight_level = DEFAULT_BACKLIGHT_LEVEL;
static int lcd_backlight_is_on = 1;

static int backlight_off_flag = 0;

static int ipu_flag = 0;
/* APP */
static void jz47xxfb_set_mode(struct jz47xxlcd_osd_t * lcd_osd_info );
void jz47xxfb_deep_set_mode(struct jz47xxlcd_info * lcd_info );
//void print_lcdc_registers(void);
static void jz47xxlcd_info_switch_to_TVE(int mode);
static int jz47xxfb0_foreground_resize(struct jz47xxlcd_osd_t *lcd_osd_info);
static int jz47xxfb0_foreground_move(struct jz47xxlcd_osd_t *lcd_osd_info);

static void jz47xxfb_descriptor_init( struct jz47xxlcd_info * lcd_info );
static void jz47xxfb0_wait_flip_end(void); //Rill 100719
static unsigned int get_current_frameID(void); //Rill 100721

struct jz47xxlcd_info jz47xx_lcd_panel;

unsigned int g_disp_type = PANEL_MODE_LCD_BASE;


static int jz_panel_num = JZ_ANDROID_PANELNUM;
static int jz_panel_index = PANEL_MODE_LCD_PANEL;
//static int jz_panel_type= PANEL_MODE_LCD_PANEL;
static int jz_panel_type= PANEL_MODE_LCD_BASE;
extern int ipu_inited;

static struct android_display_info_t adi = {0};

struct jz47xxlcd_info *jz47xx_lcd_info = &jz47xx_lcd_panel; /* default output to lcd panel */

struct jz47xxlcd_info jz47xx_info_tve_pal = {
	.panel = {
		.cfg = LCD_CFG_TVEN | /* output to tve */
		LCD_CFG_NEWDES | /* 8words descriptor */
		LCD_CFG_RECOVER | /* underrun protect */
		LCD_CFG_MODE_INTER_CCIR656, /* Interlace CCIR656 mode */
		.ctrl =  LCD_CTRL_BST_32 ,	/* 16words burst */
		TVE_WIDTH_PAL, TVE_HEIGHT_PAL, TVE_FREQ_PAL, 0, 0, 0, 0, 0, 0,
	},
	.osd = {
		 .osd_cfg = LCD_OSDC_OSDEN | /* Use OSD mode */
		 LCD_OSDC_ALPHAEN | /* enable alpha */
		 LCD_OSDC_F1EN,	/* enable Foreground0,Foreground1 */
		 .osd_ctrl = LCD_OSDCTRL_IPU,		/* disable ipu,  */
		 .rgb_ctrl = LCD_RGBC_YCC, /* enable RGB => YUV */
		 .bgcolor = 0x00000000, /* set background color Black */
		 .colorkey0 = 0x80000000, /* enable colorkey */
		 //.colorkey0 = 0, /* disable colorkey */
		 .colorkey1 = 0, /* disable colorkey */
		 .alpha = 0xA0,	/* alpha value */
		 .ipu_restart = 0x80001000, /* ipu restart */
		 .fg_change = FG_CHANGE_ALL, /* change all initially */
//		.fg0 = {32,28,48,670,480}, /*  */
//		.fg1 = {32,28,48,670,480},
			 .fg0 = {32,(TVE_WIDTH_PAL - TVE_WIDTH_PAL16) / 2,(TVE_HEIGHT_PAL - TVE_HEIGHT_PAL16) / 2,TVE_WIDTH_PAL16,TVE_HEIGHT_PAL16}, /*	*/
			 .fg1 = {32,(TVE_WIDTH_PAL - TVE_WIDTH_PAL16) / 2,(TVE_HEIGHT_PAL - TVE_HEIGHT_PAL16) / 2,TVE_WIDTH_PAL16,TVE_HEIGHT_PAL16},
	},
};
struct jz47xxlcd_info jz47xx_info_tve_ntsc = {
	.panel = {
		.cfg = LCD_CFG_TVEN | /* output to tve */
		LCD_CFG_NEWDES | /* 8words descriptor */
		LCD_CFG_RECOVER | /* underrun protect */
		LCD_CFG_MODE_INTER_CCIR656, /* Interlace CCIR656 mode */
		.ctrl =  LCD_CTRL_BST_32, //| LCD_CTRL_OFUM ,	/* 16words burst */
		TVE_WIDTH_NTSC, TVE_HEIGHT_NTSC, TVE_FREQ_NTSC, 0, 0, 0, 0, 0, 0,
	},
	.osd = {
		 .osd_cfg = LCD_OSDC_OSDEN | /* Use OSD mode */
		 LCD_OSDC_ALPHAEN | /* enable alpha */
		 LCD_OSDC_F1EN,	/* enable Foreground1 */
		 .osd_ctrl = LCD_OSDCTRL_IPU,		/* disable ipu,  */
		 .rgb_ctrl = LCD_RGBC_YCC, /* enable RGB => YUV */
		 .bgcolor = 0x00000000, /* set background color Black */
		 .colorkey0 = 0x80000000, /* enable colorkey */
		 //.colorkey0 = 0, /* disable colorkey */
		 .colorkey1 = 0, /* disable colorkey */
		 .alpha = 0xA0,	/* alpha value */
		 .ipu_restart = 0x80001000, /* ipu restart */
		 .fg_change = FG_CHANGE_ALL, /* change all initially */
//			 .fg0 = {32,38,20,640,445}, /*	*/
//			 .fg1 = {32,38,20,640,445},
			 .fg0 = {32,(TVE_WIDTH_NTSC - TVE_WIDTH_NTSC16) / 2,(TVE_HEIGHT_NTSC - TVE_HEIGHT_NTSC16) / 2,TVE_WIDTH_NTSC16,TVE_HEIGHT_NTSC16}, /*	*/
			 .fg1 = {32,(TVE_WIDTH_NTSC - TVE_WIDTH_NTSC16) / 2,(TVE_HEIGHT_NTSC - TVE_HEIGHT_NTSC16) / 2,TVE_WIDTH_NTSC16,TVE_HEIGHT_NTSC16},
	},
};


static int use_compress_decompress_mode = 0;

void lcd_enable_compress_decompress_mode(void)
{
	ENTER();

	if (use_compress_decompress_mode == 0) {
		use_compress_decompress_mode = 1;
	}
}

void lcd_disable_compress_decompress_mode(void)
{
	ENTER();

	if (use_compress_decompress_mode) {
		use_compress_decompress_mode = 0;
	}
}


int lcd_should_use_compress_decompress_mode(void)
{
#ifdef CONFIG_JZ47XX_AOSDC
	return use_compress_decompress_mode;
#else
	return 0;
#endif
}


static void lcd_init_backlight(void);
static void lcd_backlight_resume(void);
static void lcd_panel_display_on(void);
static void lcd_panel_display_off(void);

static void lcd_close_backlight(void)
{
	if (lcd_backlight_is_on == 0)
		return;

	lcd_backlight_is_on = 0;
	__lcd_close_backlight();
	jz_lcd_stat.backlight = 0;
}

static void lcd_set_backlight_level(int value)
{
	int val = (int)value;

	wait_event_interruptible(wait_backlight_resume, !lcd_backlight_is_suspend);

	if (val < 1) {
		lcd_backlight_level = 0;
		lcd_close_backlight();
	} else {
		lcd_backlight_is_on = 1;
		__lcd_set_backlight_level(val);
		lcd_backlight_level = val;
	}
	jz_lcd_stat.backlight = value;
}

static void lcd_init_backlight(void)
{
	if (lcd_backlight_is_on)
		return;

	lcd_backlight_is_on=1;
	__lcd_init_backlight(DEFAULT_BACKLIGHT_LEVEL);
	lcd_set_backlight_level(DEFAULT_BACKLIGHT_LEVEL);
}

static void lcd_backlight_resume(void)
{
	if (lcd_backlight_is_on)
		return;

	lcd_backlight_is_on=1;

	if ( lcd_backlight_level < 1)
	  lcd_backlight_level = 1;
	__lcd_init_backlight(lcd_backlight_level);
	lcd_set_backlight_level(lcd_backlight_level);
}

static void led_set_backlight_level(struct led_classdev *led_cdev, enum led_brightness value)
{
//	if (!backlight_off_flag)
		lcd_set_backlight_level(value);
}

#ifdef CONFIG_LEDS_CLASS

static enum led_brightness led_get_backlight_level(struct led_classdev *led_cdev)
{
	return lcd_backlight_level;
}
static struct led_classdev lcd_backlight_led = {
	.name			= "lcd-backlight",
	.brightness_set		= led_set_backlight_level,
	.brightness_get		= led_get_backlight_level,
};
#endif

static void lcd_set_ena(void)   // add by sbhuang 20110719
{
	if(REG_LCD_OSDC & LCD_OSDC_F1EN){
		if(!(REG_LCD_IPUR & LCD_IPUR_IPUREN)){
			printk("ERROR:LCDC DMA1 didn't have proper initialized for FG1!!!\n");
			__lcd_disable_f1();
		}
	}

	__lcd_set_ena();	                /* Enable LCD Controller */
}
static void lcd_controller_enable(void)
{
	lcd_set_ena();
	jz_lcd_stat.controller = 1;
}
static void lcd_controller_disable(void)
{
	__lcd_clr_ena();	/* Quick Disable */	
	jz_lcd_stat.controller = 0;
}

static void lcd_clock_start(void)
{
//	cpm_start_clock(CGM_LCD);
}

static void lcd_clock_stop(void)
{
	/* cpm_stop_clock(CGM_LCD), cause system hang if suspend when playing video, fix it in the future */
//	cpm_stop_clock(CGM_LCD);
}

static void lcd_panel_display_on(void)
{
	if(!jz_lcd_stat.controller) {
		lcd_controller_enable();
	}
	if(!jz_lcd_stat.panel) {
		lcd_panel->panel_ops->panel_display_on();                     /* Turn on panel */
		jz_lcd_stat.panel = 1;
	}
}
static void lcd_panel_display_off(void)
{
	lcd_panel->panel_ops->panel_display_off();                     /* Turn off panel */
	jz_lcd_stat.panel = 0;
}
static void lcd_display_on(void)
{

	lcd_clock_start();

	lcd_controller_enable();		/* Enable LCD Controller */
	lcd_panel_display_on();
}

static void lcd_display_off(void)
{

	lcd_close_backlight();


	lcd_panel_display_off();
	//__lcd_clr_ena(); /* Quick Disable */
	lcd_controller_disable();

//	schedule_timeout_uninterruptible(10); /* Quick disable 0 ms */

	lcd_clock_stop();
}

void jzfb_get_panel_size(unsigned int *w, unsigned *h)
{
	*w = jz47xx_lcd_info->panel.w;
	*h = jz47xx_lcd_info->panel.h;
}

unsigned int jz_lcd_get_width(void)
{
	return (jz47xx_lcd_info->panel.w);
}

unsigned int jz_lcd_get_height(void)
{
	return (jz47xx_lcd_info->panel.h);
}

#ifdef BOOT_ANIMATION_TEST	

static void mydisplay(int num)
{
	switch(num)
	{
	case -1:
		dma0_desc0->databuf = (unsigned int)virt_to_phys((void *)lcd_frame0);
		dma_cache_wback((unsigned int)(dma0_desc0), sizeof(struct jz47xx_lcd_dma_desc));
		break;
	default:
		dma0_desc0->databuf = (unsigned int)virt_to_phys((void *)animation_buf[num]);
		dma_cache_wback((unsigned int)(dma0_desc0), sizeof(struct jz47xx_lcd_dma_desc));	
		break;
	}
	return;
}

static void animation_do_timer(unsigned long arg)
{
	static int animation_flag=0;
	int num=ANIMATION_BUF_NUM;
	mod_timer(&animation_timer,jiffies+HZ);
	if(num==1)
		mydisplay(-1);
	else
	{
		if(animation_flag==0)
			mydisplay(-1);
		else	
			if(animation_flag==1)
				mydisplay(0);
			else
				if(num>2 && (animation_flag==2) )
					mydisplay(1);
				else
					if(num==4 && (animation_flag==3))
						mydisplay(2);			
	}
	animation_flag=(animation_flag+1)%num;
	return;
}

static int alloc_boot_animation_buffer()
{	
    unsigned int animation_frame_buffer_size=0;
	int width,height, bpp;
	int num;
	num=ANIMATION_BUF_NUM;
	width=jz47xx_lcd_info->osd.fg0.w;
	height=jz47xx_lcd_info->osd.fg0.h;
	bpp=jz47xx_lcd_info->osd.fg0.bpp;

	animation_frame_buffer_size=width*height*bpp>>3;
	for (animation_page_shift = 0; animation_page_shift < 12; animation_page_shift++)
		if ((PAGE_SIZE << animation_page_shift) >= (animation_frame_buffer_size*ANIMATION_BUF_NUM))
			break;
	if(num<2)
		printk("one buffer\n");
	else {
		animation_frame_buffer = (unsigned char *)__get_free_pages(GFP_KERNEL,animation_page_shift);
		memset((void *)animation_frame_buffer, 0, PAGE_SIZE << (animation_page_shift));
	}
	if(ANIMATION_BUF_NUM>1)
	{
		int i;
		for(i=0;i<ANIMATION_BUF_NUM-1;i++)
		{
			animation_buf[i]= animation_frame_buffer+animation_frame_buffer_size*i;
		}
	}
}
static void boot_animation_init(void)
{
//	__gpio_as_output(GPIO_LCD_PWM);
//   __gpio_set_pin(GPIO_LCD_PWM);
   
	alloc_boot_animation_buffer();
	init_timer(&animation_timer);
	animation_timer.function=&animation_do_timer;
	animation_timer.expires=jiffies+HZ;
	add_timer(&animation_timer);
}

static void boot_animation_finish(void)
{
	del_timer(&animation_timer);
	free_pages((int)animation_frame_buffer,animation_page_shift+1);
}
#endif	/* #ifdef BOOT_ANIMATION_TEST	 */

/********************************************************************************************/

/************************************
 *      Jz4760 Framebuffer ops
 ************************************/

static int jz47xxfb0_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			  u_int transp, struct fb_info *info)
{
	struct lcd_cfb_info *cfb = (struct lcd_cfb_info *)info;
	unsigned short *ptr, ctmp;

	if (regno >= NR_PALETTE)
		return 1;

	cfb->palette[regno].red		= red ;
	cfb->palette[regno].green	= green;
	cfb->palette[regno].blue	= blue;
	if (cfb->fb0.var.bits_per_pixel <= 16) {
		red	>>= 8;
		green	>>= 8;
		blue	>>= 8;

		red	&= 0xff;
		green	&= 0xff;
		blue	&= 0xff;
	}
	switch (cfb->fb0.var.bits_per_pixel) {
	case 1:
	case 2:
	case 4:
	case 8:
		if (((jz47xx_lcd_info->panel.cfg & LCD_CFG_MODE_MASK) == LCD_CFG_MODE_SINGLE_MSTN ) ||
		    ((jz47xx_lcd_info->panel.cfg & LCD_CFG_MODE_MASK) == LCD_CFG_MODE_DUAL_MSTN )) {
			ctmp = (77L * red + 150L * green + 29L * blue) >> 8;
			ctmp = ((ctmp >> 3) << 11) | ((ctmp >> 2) << 5) |
				(ctmp >> 3);
		} else {
			/* RGB 565 */
			if (((red >> 3) == 0) && ((red >> 2) != 0))
			red = 1 << 3;
			if (((blue >> 3) == 0) && ((blue >> 2) != 0))
				blue = 1 << 3;
			ctmp = ((red >> 3) << 11)
				| ((green >> 2) << 5) | (blue >> 3);
		}

		ptr = (unsigned short *)lcd_palette;
		ptr = (unsigned short *)(((u32)ptr)|0xa0000000);
		ptr[regno] = ctmp;

		break;

	case 15:
		if (regno < 16)
			((u32 *)cfb->fb0.pseudo_palette)[regno] =
				((red >> 3) << 10) |
				((green >> 3) << 5) |
				(blue >> 3);
		break;
	case 16:
		if (regno < 16) {
			((u32 *)cfb->fb0.pseudo_palette)[regno] =
				((red >> 3) << 11) |
				((green >> 2) << 5) |
				(blue >> 3);
		}
		break;
	case 17 ... 32:
		if (regno < 16)
			((u32 *)cfb->fb0.pseudo_palette)[regno] =
				(red << 16) |
				(green << 8) |
				(blue << 0);

		break;
	}
	return 0;
}

static int jz47xxfb0_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	int ret = 0, nret = -1;
	int disp_type,disp_index;
        void __user *argp = (void __user *)arg;
	struct jz47xxlcd_fg_t fg0;
    struct jz47xxlcd_info *phi;
    struct jz47xxlcd_panel_t *ppanel,panel;
//    struct android_display_info_t *padi;

//	printk("\nlcd_ioctl:cmd=0x%X\n",cmd);
	switch (cmd) {
	case FBIOSETBACKLIGHT:
		lcd_set_backlight_level(arg);	/* We support 8 levels here. */
		break;
	case FBIODISPON:
		REG_LCD_STATE = 0; /* clear lcdc status */
		__lcd_slcd_special_on();
		__lcd_clr_dis();
		lcd_controller_enable();		/* Enable LCD Controller */
//		__lcd_display_on();
		lcd_panel_display_on();
		break;
	case FBIODISPOFF:
//		__lcd_display_off();
		lcd_panel_display_off();
		if ( jz47xx_lcd_info->panel.cfg & LCD_CFG_LCDPIN_SLCD)
			lcd_controller_disable();/* Smart lcd and TVE mode only support quick disable */
		else
			__lcd_set_dis(); /* regular disable */
		break;
	case FBIOPRINT_REG:
		print_lcdc_registers();
		break;
	case FBIO_GET_MODE:
		print_dbg("fbio get mode\n");
		if (copy_to_user(argp, jz47xx_lcd_info, sizeof(struct jz47xxlcd_info)))
			return -EFAULT;
		break;
	case FBIO_SET_MODE:
		print_dbg("fbio set mode\n");
		if (copy_from_user(jz47xx_lcd_info, argp, sizeof(struct jz47xxlcd_info)))
			return -EFAULT;
		/* set mode */
		jz47xxfb_set_mode(&jz47xx_lcd_info->osd);
//		jz47xxfb_set_mode(jz47xx_lcd_info);
		break;
	case FBIO_DEEP_SET_MODE:
		print_dbg("fbio deep set mode\n");
		if (copy_from_user(jz47xx_lcd_info, argp, sizeof(struct jz47xxlcd_info)))
			return -EFAULT;
		jz47xxfb_deep_set_mode(jz47xx_lcd_info);
		break;

	case FBIO_GET_TVE_MODE:
		//moveout
		break;
	case FBIO_SET_TVE_MODE:
		//moveout
		break;

	case FBIODISON_FG: //pass
		/*lcdc_enable_fg0();*/
		jz47xx_lcd_info->osd.osd_cfg |= LCD_OSDC_F0EN;
		__lcd_enable_f0();
		break;
	case FBIODISOFF_FG://pass
		/*lcdc_disable_fg0();*/
		print_dbg("lcdc_disable_fg0()\n");
		jz47xx_lcd_info->osd.osd_cfg &= ~LCD_OSDC_F0EN;
		__lcd_disable_f0();
		break;
	case FBIO_CHANGE_SIZE:
		/*fg0_change_size();*/
		if(!(REG_LCD_OSDC & LCD_OSDC_F0EN))
			return -EFAULT;
		if (copy_from_user(&fg0, argp, sizeof(struct jz47xxlcd_fg_t)))
			return -EFAULT;
		jz47xx_lcd_info->osd.fg0.w = fg0.w;
		jz47xx_lcd_info->osd.fg0.h = fg0.h;
		jz47xxfb0_foreground_resize(&jz47xx_lcd_info->osd);
		break;
	case FBIO_CHANGE_POSITION:
		if (copy_from_user(&fg0, argp, sizeof(struct jz47xxlcd_fg_t)))
			return -EFAULT;
		jz47xx_lcd_info->osd.fg0.x = fg0.x;
		jz47xx_lcd_info->osd.fg0.y = fg0.y;
		jz47xxfb0_foreground_move(&jz47xx_lcd_info->osd);
		break;
	case FBIO_SET_BG_COLOR://pass
		jz47xx_lcd_info->osd.bgcolor = arg;
		/*lcdc_set_bgcolor(arg);*/
		REG_LCD_BGC = jz47xx_lcd_info->osd.bgcolor;
		break;
	case FBIO_ALPHA_ON://pass
		/*lcdc_enable_alpha();*/
		jz47xx_lcd_info->osd.osd_cfg |= LCD_OSDC_ALPHAEN;
		__lcd_enable_alpha();
		break;
	case FBIO_ALPHA_OFF://pass
		/*lcdc_disable_alpha();*/
		jz47xx_lcd_info->osd.osd_cfg &= ~LCD_OSDC_ALPHAEN;
		__lcd_disable_alpha();
		break;
	case FBIO_SET_ALPHA_VAL://pass
		jz47xx_lcd_info->osd.alpha = arg;
		/*lcdc_set_alpha(arg);*/
		REG_LCD_ALPHA = jz47xx_lcd_info->osd.alpha;
		break;
	case FBIO_ANDROID_CTL:
//        printk("FBIO_ANDROID_CTL!\n");
		if (copy_from_user(&adi, argp, sizeof(struct android_display_info_t))){	
            printk("copy_from_user error!!!\n");
//            kfree(padi);
			return -EFAULT;
        }
   //     printk("adi.flag = 0x%X\n",adi.flag);
		switch (adi.flag) 
		{
			case ANDROID_GET_DISPLAY_NUM://pass
				adi.fg0_number = jz_panel_num;
				if (copy_to_user(argp, &adi, sizeof(struct android_display_info_t)))
					return -EFAULT;
				break;
			case ANDROID_GET_DISPLAY_INFO://pass
				adi.fg0_x = jz47xx_lcd_info->osd.fg1.x;
				adi.fg0_y = jz47xx_lcd_info->osd.fg1.y;
				adi.fg0_w = jz47xx_lcd_info->osd.fg1.w;
				adi.fg0_h = jz47xx_lcd_info->osd.fg1.h;

				if (copy_to_user(argp, &adi, sizeof(struct android_display_info_t)))
					return -EFAULT;
				break;
			case ANDROID_SET_DISPLAY_INDEX: //pass
			{
				ipu_flag = 0;
				jz_panel_type = GET_PANEL_TYPE(jz_panel_index);
				disp_type = GET_PANEL_TYPE(adi.fg0_index);
				disp_index = GET_PANEL_INDEX(adi.fg0_index);
				printk("fg0_index=0x%08x,disp_type=0x%x\n",adi.fg0_index,disp_type);
				switch (disp_type) 
				{
					case PANEL_MODE_HDMI_BASE:
					{
						g_disp_type = PANEL_MODE_HDMI_BASE; 
						if (jz47xx_lcd_info->panel.cfg & LCD_CFG_TVEN )
						jz47xxtve_disable_tve();
						if(REG_LCD_CTRL & LCD_CTRL_ENA)
							__lcd_clr_ena();	/* Quick Disable */
						__lcd_disable_f1();//1118
						printk("HDMI disp_index:0x%x\n",disp_index);
						jz47xx_lcd_info = jz47xx_set_hdmi_info(disp_index);						
						jz_panel_index = adi.fg0_index;
            
                        phi = jz47xx_lcd_info;
                        if(adi.cmd == CMD_LCD_PANEL_INFO_CHANGE){
                            ppanel= &panel;
                        //    ppanel=(struct jz47xxlcd_panel_t *) adi.data_buf;
		                    if (copy_from_user(ppanel, adi.data_buf, sizeof(struct jz47xxlcd_panel_t))){
                                printk("copy_from_user error  adi.cmd = 1\n");
					          //  return -EFAULT;
                            }else{	
                                phi->panel.w = ppanel->w;
                                phi->panel.h = ppanel->h;
                                phi->panel.fclk= ppanel->fclk;
                                phi->panel.hsw= ppanel->hsw;
                                phi->panel.vsw= ppanel->vsw;
                                phi->panel.elw= ppanel->elw;
                                phi->panel.blw=ppanel->blw;
                                phi->panel.efw=ppanel->efw;
                                phi->panel.bfw=ppanel->bfw;

                                phi->osd.fg0.w= phi->panel.w;
                                phi->osd.fg0.h= phi->panel.h;
                                phi->osd.fg1.w= phi->panel.w;
                                phi->osd.fg1.h= phi->panel.h;
                                printk("HMDI test (struct panel changed)!\n");
                            }

                        }

                        printk("INFO: %d %d %d %d %d %d %d %d %d\n",phi->panel.w,phi->panel.h,phi->panel.fclk,
                          phi->panel.hsw,phi->panel.vsw,phi->panel.elw,phi->panel.blw,phi->panel.efw,phi->panel.bfw);

						jz47xxfb_deep_set_mode(jz47xx_lcd_info);
						
					//	printk("adi.cmd = %d\n",adi.cmd);

						if(lcd_backlight_is_on){
							if(adi.cmd != CMD_LCD_PANEL_INFO_CHANGE &&
							 	adi.cmd != CMD_LCD_HDMI_SYNC_OUTPUT){
								lcd_close_backlight();
								msleep(100);
								lcd_panel_display_off();
								backlight_off_flag = 1;
							}
						}else{											
							if(!lcd_backlight_is_suspend && adi.cmd == CMD_LCD_HDMI_SYNC_OUTPUT)
							{
								//lcd_panel->panel_ops->panel_display_on();			  /* Turn on panel */
								lcd_panel_display_on();
								msleep(400);			
								lcd_backlight_resume();
								backlight_off_flag = 0;
							}
						}
						
                        if(adi.cmd != 0)
                            adi.cmd = 0;

					break;
					}
					case PANEL_MODE_TVE_BASE:	
					{			
							g_disp_type = PANEL_MODE_TVE_BASE;
							printk("++PANEL_MODE_TVE+++\n");
							if (jz47xx_lcd_info->panel.cfg & LCD_CFG_TVEN )
								jz47xxtve_disable_tve();
							if(REG_LCD_CTRL & LCD_CTRL_ENA)
								lcd_controller_disable();

							__lcd_disable_f1();//1118
							//ipu_close();//1118
							//ipu_deinit();
	
							jz47xxlcd_info_switch_to_TVE(adi.fg0_index);
							jz47xx_lcd_info->osd.osd_cfg &= ~LCD_OSDC_F1EN;				
							jz47xxtve_outfmt_init(PANEL_OUT_FMT_CVBS);
							jz47xxtve_init(adi.fg0_index); /* tve controller init */
							jz47xxtve_enable_tve();
							jz_panel_index = adi.fg0_index;
							jz47xxfb_deep_set_mode(jz47xx_lcd_info);
	
							/* turn off lcd backlight */
							if(lcd_backlight_is_on){
								lcd_close_backlight();
								/* Do not close the panel for some tablets ( npm702 ...)when TVE*/
								//lcd_panel_display_off();
							}
							break;
					}			
					case PANEL_MODE_LCD_BASE: 	/* switch to LCD mode */
					default:
					{
						g_disp_type = PANEL_MODE_LCD_BASE;	
							/* turn off TVE, turn off DACn... */
							//ipu_deinit();
							if (jz47xx_lcd_info->panel.cfg & LCD_CFG_TVEN )
								jz47xxtve_disable_tve();
							if(REG_LCD_CTRL & LCD_CTRL_ENA)
								lcd_controller_disable();
						
							__lcd_disable_f1();//1118
							//ipu_deinit();
							//ipu_close();//1118
							jz47xx_lcd_info = &jz47xx_lcd_panel;
							jz_panel_index = adi.fg0_index;
							jz47xxfb_deep_set_mode(jz47xx_lcd_info);

							/* turn on lcd backlight */
							if(!lcd_backlight_is_on && !lcd_backlight_is_suspend){
								lcd_panel_display_on();
								msleep(400);			
								lcd_backlight_resume();
							}
	
							break;
					}
				}
			break;
			}
			case ANDROID_IPU_IOCTL:
			//printk(" ANDROID_IPU_IOCTL:%s[%d]\n",__FILE__,__LINE__);
		        ret = ipu_driver_ioctl(ipu_priv, (void*)adi.data_buf);
			break;

		default:
			printk("FG0:%s, unknown android command(0x%x)", __FILE__, adi.flag);
			return nret;
		}
		break;
		#if USE_CHANGE_FOR_SURFACEFLINGER/*Rill add for serfaceFlinger 100719*/
		case FBIO_WAIT_LCD_SWAP_SIGNAL:
		{
			jz47xxfb0_wait_flip_end();
		}
		break;
		#endif
#ifdef JZ47XX_LCD_SWAPBUFFER_GET_LATEST_FPS
	case FBIO_GET_FPS:
	{
		int fps;
		fps = jz47xx_lcd_swapbuffer_get_latest_fps(0);
		if ( argp ) {
				if (copy_to_user(argp, &fps, sizeof(int)))
					return nret;
		}
		ret = fps;
	}
	break;
#endif
	default:
		printk("FG0:%s, unknown command(0x%x)", __FILE__, cmd);
		return nret;
	}

	return ret;
}

/* jz47xxfb0_open() 
 * 
 * If FG1 enable, switch to FG0.
 *
 */
static int jzfb0_open_cnt = 0;
static int jz47xxfb0_open(struct fb_info *info, int user)
{
	++jzfb0_open_cnt;
	printk("jz47xxfb0_open() jzfb0_open_cnt=%d\n", jzfb0_open_cnt);

	/* reset lcdc after SurfaceFlinger crach when playing video */
	if (REG_LCD_OSDC & LCD_OSDC_F1EN) {
		printk("jz47xxfb0_open() LCD_OSDC_F1EN, disable FG1.\n");

		lcd_controller_disable();

		/* Disable lcd fg1 */
		REG32(IPU_V_BASE + REG_CTRL) |= IPU_STOP; /* stop ipu */

		__lcd_fg1_unuse_ipu();
		__lcd_disable_f1();
		__lcd_enable_f0();

		lcd_disable_compress_decompress_mode();

		__lcd_disable_colorkey0();
		__lcd_disable_alpha();

//			mdelay(20);			/* Test, Wolfgang, 2010-07-30 */
		lcd_controller_enable();		/* Enable LCD Controller */
	}


	return 0;

}



static int jz47xxfb0_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	struct lcd_cfb_info *cfb = (struct lcd_cfb_info *)info;
	unsigned long start;
	unsigned long off;
	u32 len;

	off = vma->vm_pgoff << PAGE_SHIFT;
	//fb->fb_get_fix(&fix, PROC_CONSOLE(info), info);

	/* frame buffer memory */
	start = cfb->fb0.fix.smem_start;
	len = PAGE_ALIGN((start & ~PAGE_MASK) + cfb->fb0.fix.smem_len);
	start &= PAGE_MASK;

	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;
	off += start;

	vma->vm_pgoff = off >> PAGE_SHIFT;
	vma->vm_flags |= VM_IO;
//	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);	/* Uncacheable */

 	pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
// 	pgprot_val(vma->vm_page_prot) |= _CACHE_UNCACHED;		/* Uncacheable */
	pgprot_val(vma->vm_page_prot) |= _CACHE_CACHABLE_NONCOHERENT;	/* Write-Back */

	if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
			       vma->vm_end - vma->vm_start,
			       vma->vm_page_prot)) {
		return -EAGAIN;
	}
	return 0;
}
  /* end of Foreground 0 ops*/


/* checks var and eventually tweaks it to something supported,
 * DO NOT MODIFY PAR */
static int jz47xxfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
/*	if((var->rotate & 1) != (info->var.rotate & 1)) {
		if((var->xres != info->var.yres) ||
		   (var->yres != info->var.xres) ||
		   (var->xres_virtual != info->var.yres) ||
		   (var->yres_virtual >
		    info->var.xres * ANDROID_NUMBER_OF_BUFFERS) ||
		   (var->yres_virtual < info->var.xres )) {
			return -EINVAL;
		}
	}
	else {
		if((var->xres != info->var.xres) ||
		   (var->yres != info->var.yres) ||
		   (var->xres_virtual != info->var.xres) ||
		   (var->yres_virtual >
		    info->var.yres * ANDROID_NUMBER_OF_BUFFERS) ||
		   (var->yres_virtual < info->var.yres )) {
			return -EINVAL;
		}
	}
	if((var->xoffset != info->var.xoffset) ||
	   (var->bits_per_pixel != info->var.bits_per_pixel)) {// ||
//	   (var->grayscale != info->var.grayscale)) {
		return -EINVAL;
		}
*/
	return 0;
}


/*
 * set the video mode according to info->var
 */
static int jz47xxfb_set_par(struct fb_info *info)
{
	//dprintk("jz47xxfb_set_par, not implemented\n");
	return 0;
}


/*
 * (Un)Blank the display.
 * Fix me: should we use VESA value?
 */
static int jz47xxfb_blank(int blank_mode, struct fb_info *info)
{
	switch (blank_mode) {
	case FB_BLANK_UNBLANK:
		//case FB_BLANK_NORMAL:
			/* Turn on panel */

		lcd_controller_enable();		/* Enable LCD Controller */
//		__lcd_display_on();
		lcd_panel_display_on();
		break;

	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		/* Turn off panel */
#if 0
//		__lcd_display_off();
		lcd_panel->panel_ops->panel_display_off();
		__lcd_set_dis();
#endif
		break;
	default:
		break;

	}
	return 0;
}

static unsigned int get_current_frameID(void)
{
	return REG_LCD_FID0;
}

static void jz47xxfb0_wait_flip_end(void)
{
	unsigned long irq_flags;
	struct lcd_cfb_info *cfb = jz47xxfb_info;

	if (!(REG_LCD_CTRL & LCD_CTRL_ENA)) {
		return;
	}

	//printk("wait_flip_end g_next_frameID=%x\t", g_next_frameID);

	if(g_next_frameID == get_current_frameID()) {
		;//printk("lcd pass: %#x\n",g_next_frameID);
	}
	else {
		//struct timeval tv1, tv2;

		/* Wait for current frame to finished */
		spin_lock_irqsave(&cfb->update_lock, irq_flags);

		REG_LCD_STATE &= ~LCD_STATE_EOF; // Clear previous EOF flag
		__lcd_enable_eof_intr();

		cfb->frame_requested++;

		spin_unlock_irqrestore(&cfb->update_lock, irq_flags);

		//do_gettimeofday(&tv1);
		wait_event_interruptible_timeout(cfb->frame_wq, cfb->frame_done == cfb->frame_requested, HZ/50 + 1);  /* HZ = 100 */ 
		//do_gettimeofday(&tv2);
		__lcd_disable_eof_intr();
		
		//printk("lcd delaytime== %d us=====\n",((tv2.tv_sec-tv1.tv_sec)*1000000 + tv2.tv_usec-tv1.tv_usec));
	}
}

static int jz47xxfb0_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	#if (!USE_CHANGE_FOR_SURFACEFLINGER)//Rill add  for surfaceFlinger 100719
	unsigned long irq_flags;
	struct lcd_cfb_info *cfb = jz47xxfb_info;
	#endif

	struct fb_info *fb = info;
	struct jz47xxlcd_info *lcd_info = jz47xx_lcd_info;
	int fg0_line_size, fg0_frm_size;
	int dy;

#ifdef JZ47XX_LCD_SWAPBUFFER_GET_LATEST_FPS
	jz47xx_lcd_swapbuffer_count_increase(1);
#endif

#ifdef BOOT_ANIMATION_TEST
	boot_animation_finish();
#endif
	if (!(REG_LCD_CTRL & LCD_CTRL_ENA)) {
		return 0;
	}

	if (!(REG_LCD_OSDC & LCD_OSDC_F0EN)) {
		return 0;
	}

	if (!var || !fb) {
		return -EINVAL;
	}
	if (var->xoffset - fb->var.xoffset) {
		/* No support for X panning for now! */
		return -EINVAL;
	}

	dy = var->yoffset;// - fb->var.yoffset;
//	fb->fix.line_length = ( jz47xx_lcd_panel.osd.fg0.w * (lcd_info->osd.fg0.bpp) / 8);
//	printk("fb->fix.line_length=%d\n", fb->fix.line_length);

	fg0_line_size = ( jz47xx_lcd_info->osd.fg0.w * (lcd_info->osd.fg0.bpp) / 8);
	fg0_line_size = ((fg0_line_size + 3) >> 2) << 2;
	fg0_frm_size = fg0_line_size * lcd_info->osd.fg0.h;

#ifdef CONFIG_JZ47XX_AOSDC
	if (lcd_should_use_compress_decompress_mode()) {
		flush_dcache_with_prefetch_allocate();
//		__flush_cache_vmap();		/* blast_dcache(), flush ui-fb. */

		aosd_info->bpp = jz47xx_lcd_info->osd.fg0.bpp;
		aosd_info->width = jz47xx_lcd_info->osd.fg0.w;
		aosd_info->height = jz47xx_lcd_info->osd.fg0.h;	// 
		aosd_info->aligned_64 = 0;						/* meanless */
		aosd_info->src_stride = fb->fix.line_length; // in bytes.
		aosd_info->dst_stride = (aosd_info->src_stride + (AOSD_ALIGN))&(~(AOSD_ALIGN-1)); // in bytes. 32 or 64 words aligned

		if (dy) {		
			aosd_info->addr0 = (unsigned int)virt_to_phys((void *)lcd_frame0 + (fb->fix.line_length * dy));
			aosd_info->waddr = (unsigned int)virt_to_phys((void *)buf_comp);
		}
		else {
			aosd_info->addr0 = (unsigned int)virt_to_phys((void *)lcd_frame0);
			aosd_info->waddr = (unsigned int)virt_to_phys((void *)buf_comp1);
		}

		jz47xx_compress_set_mode(aosd_info);
		jz47xx_start_compress();
		wait_event_interruptible(
			compress_wq, aosd_info->compress_done == 1);

		dma0_desc0->cmd = LCD_CMD_EOFINT | LCD_CMD_UNCOMP_EN | (aosd_info->height & LCD_CMD_LEN_MASK);
		//dma0_desc0->cmd |= LCD_CMD_UNCOMPRESS_WITHOUT_ALPHA;
		// don't need to set dma page width in decompress mode.
		//dma0_desc0->page_width = 0;
		// dma offsize in decompress mode indicates how many words of a line in source buffer
		dma0_desc0->offsize = (aosd_info->dst_stride)>>2; /* in words */

		if (dy) {
			dma0_desc0->databuf = (unsigned int)virt_to_phys((void *)buf_comp);
			dma0_desc0->frame_id = 0x82826661;
			g_next_frameID = 0x82826661;
		}
		else {
			dma0_desc0->databuf = (unsigned int)virt_to_phys((void *)buf_comp1);
			dma0_desc0->frame_id = 0x19491001;
			g_next_frameID = 0x19491001;
		}
		dma_cache_wback((unsigned int)(dma0_desc0), sizeof(struct jz47xx_lcd_dma_desc));
#ifdef CALC_COMP_RATIO
		calc_comp_ratio(dy, aosd_info->height, aosd_info->width, dma0_desc0->offsize);
#endif
	}
	else {
		flush_dcache_with_prefetch_allocate();
//		__flush_cache_vmap();		/* blast_dcache(); */

		dma0_desc0->cmd = LCD_CMD_EOFINT | fg0_frm_size/4;
		dma0_desc0->offsize = 0;
		if (dy) {
			dma0_desc0->databuf = (unsigned int)virt_to_phys((void *)lcd_frame0 + (fb->fix.line_length * dy));
			dma0_desc0->frame_id = 0x82826661;
			g_next_frameID = 0x82826661;
		}
		else {
			dma0_desc0->databuf = (unsigned int)virt_to_phys((void *)lcd_frame0);
			dma0_desc0->frame_id = 0x19491001;
			g_next_frameID = 0x19491001;
		}
		dma_cache_wback((unsigned int)(dma0_desc0), sizeof(struct jz47xx_lcd_dma_desc));
	}
#else
	if (dy) {
		dma0_desc0->databuf = (unsigned int)virt_to_phys((void *)lcd_frame0 + (fb->fix.line_length * dy));
		dma0_desc0->frame_id = 0x82826661;
		g_next_frameID = 0x82826661;
	}
	else {
		dma0_desc0->databuf = (unsigned int)virt_to_phys((void *)lcd_frame0);
		dma0_desc0->frame_id = 0x19491001;
		g_next_frameID = 0x19491001;
	}
	flush_dcache_with_prefetch_allocate();
//	__flush_cache_vmap();		/* blast_dcache(); */

#endif

	//printk("pan_display yoffset=%d, g_next_frameID=%x\n", dy, g_next_frameID);

	#if (!USE_CHANGE_FOR_SURFACEFLINGER)//Rill add  for surfaceFlinger 100719
	/* Wait for current frame to finished */
	spin_lock_irqsave(&cfb->update_lock, irq_flags);

	REG_LCD_STATE &= ~LCD_STATE_EOF; // Clear previous EOF flag
	__lcd_enable_eof_intr();

	cfb->frame_requested++;

	spin_unlock_irqrestore(&cfb->update_lock, irq_flags);

	if (cfb->frame_requested != cfb->frame_done)
		/* Fix android bug: 176. timeout+1 avoid wait_event return early than expacted. */
		wait_event_interruptible_timeout(
			cfb->frame_wq, cfb->frame_done == cfb->frame_requested, HZ/50 + 1); /* HZ = 100 */

	__lcd_disable_eof_intr();
	#endif	/* USE_CHANGE_FOR_SURFACEFLINGER */

	return 0;
}

/* use default function cfb_fillrect, cfb_copyarea, cfb_imageblit */
/* static struct fb_ops jz47xxfb_ops = { */
/* 	.fb_blank		= jz47xxfb_blank, */
/* }; */

/* use default function cfb_fillrect, cfb_copyarea, cfb_imageblit */
static struct fb_ops jz47xxfb0_ops = {
	.owner			= THIS_MODULE,
	.fb_open		= jz47xxfb0_open,
	.fb_setcolreg		= jz47xxfb0_setcolreg,
	.fb_check_var 		= jz47xxfb_check_var,
	.fb_set_par 		= jz47xxfb_set_par,
	.fb_blank		= jz47xxfb_blank,
	.fb_pan_display		= jz47xxfb0_pan_display,
	.fb_fillrect		= cfb_fillrect,
	.fb_copyarea		= cfb_copyarea,
	.fb_imageblit		= cfb_imageblit,
	.fb_mmap		= jz47xxfb0_mmap,
	.fb_ioctl		= jz47xxfb0_ioctl,
};

static int jz47xxfb_set_var(struct fb_var_screeninfo *var, int con,
			struct fb_info *info)
{
	struct fb_info *fb = info;
	struct jz47xxlcd_info *lcd_info = jz47xx_lcd_info;
	int chgvar = 0;

	if (con == 0) {
		var->xres = lcd_info->osd.fg0.w;
		var->yres = lcd_info->osd.fg0.h;
		var->bits_per_pixel = lcd_info->osd.fg0.bpp;
	}
	else {
		var->xres = lcd_info->osd.fg1.w;
		var->yres = lcd_info->osd.fg1.h;
		var->bits_per_pixel = lcd_info->osd.fg1.bpp;
	}

	var->vmode                  = FB_VMODE_NONINTERLACED;
//	var->vmode                  = FB_VMODE_DOUBLE
	var->activate               = fb->var.activate;
	var->width                  = lcd_info->panel.phys_width; /* example: 170mm */
	var->height                 = lcd_info->panel.phys_height;/* example: 100mm */
	var->xres_virtual           = var->xres;
	var->yres_virtual           = var->yres * ANDROID_NUMBER_OF_BUFFERS;
	var->xoffset                = 0;
	var->yoffset                = 0;
	var->pixclock               = KHZ2PICOS(cpm_get_clock(CGU_LPCLK)/1000);
#if 0
	var->left_margin            = lcd_info->panel.elw;
	var->right_margin           = lcd_info->panel.blw;
	var->upper_margin           = lcd_info->panel.efw;
	var->lower_margin           = lcd_info->panel.bfw;
#else
	var->left_margin            = lcd_info->panel.blw;
	var->right_margin           = lcd_info->panel.elw;
	var->upper_margin           = lcd_info->panel.bfw;
	var->lower_margin           = lcd_info->panel.efw;
#endif
	var->hsync_len              = lcd_info->panel.hsw;
	var->vsync_len              = lcd_info->panel.vsw;
	var->sync                   = 0;
	var->activate               = FB_ACTIVATE_NOW;

	/*
	 * CONUPDATE and SMOOTH_XPAN are equal.  However,
	 * SMOOTH_XPAN is only used internally by fbcon.
	 */
	if (var->vmode & FB_VMODE_CONUPDATE) {
		var->vmode |= FB_VMODE_YWRAP;
		var->xoffset = fb->var.xoffset;
		var->yoffset = fb->var.yoffset;
	}

	if (var->activate & FB_ACTIVATE_TEST)
		return 0;

	if ((var->activate & FB_ACTIVATE_MASK) != FB_ACTIVATE_NOW)
		return -EINVAL;

	if (fb->var.xres != var->xres)
		chgvar = 1;
	if (fb->var.yres != var->yres)
		chgvar = 1;
	if (fb->var.xres_virtual != var->xres_virtual)
		chgvar = 1;
	if (fb->var.yres_virtual != var->yres_virtual)
		chgvar = 1;
	if (fb->var.bits_per_pixel != var->bits_per_pixel)
		chgvar = 1;

	//display = fb_display + con;

	var->red.msb_right	= 0;
	var->green.msb_right	= 0;
	var->blue.msb_right	= 0;

	switch(var->bits_per_pixel){
	case 1:	/* Mono */
		fb->fix.visual	= FB_VISUAL_MONO01;
		fb->fix.line_length	= (var->xres * var->bits_per_pixel) / 8;
		break;
	case 2:	/* Mono */
		var->red.offset		= 0;
		var->red.length		= 2;
		var->green.offset	= 0;
		var->green.length	= 2;
		var->blue.offset	= 0;
		var->blue.length	= 2;

		fb->fix.visual	= FB_VISUAL_PSEUDOCOLOR;
		fb->fix.line_length	= (var->xres * var->bits_per_pixel) / 8;
		break;
	case 4:	/* PSEUDOCOLOUR*/
		var->red.offset		= 0;
		var->red.length		= 4;
		var->green.offset	= 0;
		var->green.length	= 4;
		var->blue.offset	= 0;
		var->blue.length	= 4;

		fb->fix.visual	= FB_VISUAL_PSEUDOCOLOR;
		fb->fix.line_length	= var->xres / 2;
		break;
	case 8:	/* PSEUDOCOLOUR, 256 */
		var->red.offset		= 0;
		var->red.length		= 8;
		var->green.offset	= 0;
		var->green.length	= 8;
		var->blue.offset	= 0;
		var->blue.length	= 8;

		fb->fix.visual	= FB_VISUAL_PSEUDOCOLOR;
		fb->fix.line_length	= var->xres ;
		break;
	case 15: /* DIRECTCOLOUR, 32k */
		var->bits_per_pixel	= 15;
		var->red.offset		= 10;
		var->red.length		= 5;
		var->green.offset	= 5;
		var->green.length	= 5;
		var->blue.offset	= 0;
		var->blue.length	= 5;

		fb->fix.visual	= FB_VISUAL_DIRECTCOLOR;
		fb->fix.line_length	= var->xres_virtual * 2;
		break;
	case 16: /* DIRECTCOLOUR, 64k */
		var->bits_per_pixel	= 16;
		var->red.offset		= 11;
		var->red.length		= 5;
		var->green.offset	= 5;
		var->green.length	= 6;
		var->blue.offset	= 0;
		var->blue.length	= 5;

		fb->fix.visual	= FB_VISUAL_TRUECOLOR;
		fb->fix.line_length	= var->xres_virtual * 2;
		break;
	case 17 ... 32:
		/* DIRECTCOLOUR, 256 */
		var->bits_per_pixel	= 32;
#if BPP32_FORMAT_ORDER == FORMAT_X8B8G8R8
		var->red.offset		= 0;
		var->green.offset	= 8;
		var->blue.offset	= 16;
#else  /* default: FORMAT_X8R8G8B8*/
		var->red.offset		= 16;
		var->green.offset	= 8;
		var->blue.offset	= 0;
#endif
		var->red.length		= 8;
		var->green.length	= 8;
		var->blue.length	= 8;
		var->transp.offset  	= 24;
		var->transp.length 	= 0; //8;

		fb->fix.visual	= FB_VISUAL_TRUECOLOR;
		fb->fix.line_length	= var->xres_virtual * 4;
		break;

	default: /* in theory this should never happen */
		printk(KERN_WARNING "%s: don't support for %dbpp\n",
		       fb->fix.id, var->bits_per_pixel);
		break;
	}

	fb->var = *var;
	fb->var.activate &= ~FB_ACTIVATE_ALL;

	/*
	 * Update the old var.  The fbcon drivers still use this.
	 * Once they are using cfb->fb.var, this can be dropped.
	 *					--rmk
	 */
	//display->var = cfb->fb.var;
	/*
	 * If we are setting all the virtual consoles, also set the
	 * defaults used to create new consoles.
	 */
	fb_set_cmap(&fb->cmap, fb);
	return 0;
}

static struct lcd_cfb_info * jz47xxfb_alloc_fb_info(void)
{
 	struct lcd_cfb_info *cfb;
	cfb = kmalloc(sizeof(struct lcd_cfb_info) + sizeof(u32) * 16, GFP_KERNEL);

	if (!cfb)
		return NULL;

	jz47xxfb_info = cfb;

	memset(cfb, 0, sizeof(struct lcd_cfb_info) );

	cfb->currcon		= -1;


	/* Foreground 0 -- fb0 */
	strcpy(cfb->fb0.fix.id, "jzlcd-fg0");
	cfb->fb0.fix.type	= FB_TYPE_PACKED_PIXELS;
	cfb->fb0.fix.type_aux	= 0;
	cfb->fb0.fix.xpanstep	= 1;
	cfb->fb0.fix.ypanstep	= 1;
	cfb->fb0.fix.ywrapstep	= 0;
	cfb->fb0.fix.accel	= FB_ACCEL_NONE;

	cfb->fb0.var.nonstd	= 0;
	cfb->fb0.var.activate	= FB_ACTIVATE_NOW;
	cfb->fb0.var.height	= -1;
	cfb->fb0.var.width	= -1;
	cfb->fb0.var.accel_flags	= FB_ACCELF_TEXT;

	cfb->fb0.fbops		= &jz47xxfb0_ops;
	cfb->fb0.flags		= FBINFO_FLAG_DEFAULT;

	cfb->fb0.pseudo_palette	= (void *)(cfb + 1);

	switch (jz47xx_lcd_info->osd.fg0.bpp) {
	case 1:
		fb_alloc_cmap(&cfb->fb0.cmap, 4, 0);
		break;
	case 2:
		fb_alloc_cmap(&cfb->fb0.cmap, 8, 0);
		break;
	case 4:
		fb_alloc_cmap(&cfb->fb0.cmap, 32, 0);
		break;
	case 8:

	default:
		fb_alloc_cmap(&cfb->fb0.cmap, 256, 0);
		break;
	}
	dprintk("fb0.cmap.len:%d....\n", cfb->fb0.cmap.len);
	return cfb;
}

/*
 * Map screen memory
 */
static int jz47xxfb_map_smem(struct lcd_cfb_info *cfb)
{
	unsigned long page;
	unsigned int bpp, w, h;
	unsigned char *fb_palette, *fb_frame;

	/* caculate the mem size of Foreground 0 */
	bpp = jz47xx_lcd_info->osd.fg0.bpp;
	if (bpp == 18 || bpp == 24)
		bpp = 32;
	if (bpp == 15)
		bpp = 16;

/* 	w = (jz47xx_lcd_info->osd.fg0.w > TVE_WIDTH_PAL) ? jz47xx_lcd_info->osd.fg0.w : TVE_WIDTH_PAL; */
/* 	h = (jz47xx_lcd_info->osd.fg0.h > TVE_HEIGHT_PAL) ? jz47xx_lcd_info->osd.fg0.h : TVE_HEIGHT_PAL; */

	w = jz47xx_lcd_info->osd.fg0.w;
	h = jz47xx_lcd_info->osd.fg0.h;

	bpp >>= 3;					/* in bytes */
	/* FrameBuffer 16words aligned at least */
#ifdef CONFIG_JZ47XX_AOSDC
	{
		int line_length;
		/* FrameBuffer Size */
		line_length = w * bpp;
		line_length = (line_length + (FB_ALIGN-1)) & (~(FB_ALIGN-1));
		/* double framebuffer and reserve a page size for start address align */
		fb_needroom = line_length * h * ANDROID_NUMBER_OF_BUFFERS + PAGE_SIZE;

		/* AOSD buffer Size */
		line_length = w * bpp;
		if ( !(line_length & (AOSD_ALIGN-1)) ) {
			line_length += 16*4;	/* 16words */
		}
		line_length = (line_length + (AOSD_ALIGN-1)) & (~(AOSD_ALIGN-1));
		/* reserve a page size for start address align */
		buf_comp_needroom = buf_comp1_needroom = line_length * h + PAGE_SIZE;
		printk("jz47xxfb_map_smem FB_Size = %d bytes\n", fb_needroom);
	}
#else
	/* double framebuffer and reserve a page size for start address align */
	fb_needroom = (w * bpp ) * h * ANDROID_NUMBER_OF_BUFFERS + PAGE_SIZE;
	printk("jz47xxfb_map_smem FB_Size = %d bytes\n", fb_needroom);
#endif

	/* Alloc memory */
	fb_palette = (unsigned char *)__get_free_pages(GFP_KERNEL, 0);
	fb_frame = (unsigned char *)kmalloc(fb_needroom, GFP_KERNEL | __GFP_DMA);

	if ((!fb_palette) || (!fb_frame))
		return -ENOMEM;

	memset((void *)fb_palette, 0, PAGE_SIZE);

	lcd_palette = fb_palette;
	dma_desc_base = (struct jz47xx_lcd_dma_desc *)((void*)lcd_palette
												   + ((PALETTE_SIZE + 3) / 4) * 4);

	/*
	 * Set page reserved so that mmap will work. This is necessary
	 * since we'll be remapping normal memory.
	 */
	page = (unsigned long)lcd_palette;
	SetPageReserved(virt_to_page((void*)page));

	/* FrameBuffer start address PAGE_SIZE align */
	if ((unsigned int)fb_frame & (PAGE_SIZE / 4 - 1)) {
		printk("framebuffer start address need to set %d words align\n", (int)PAGE_SIZE / 4);
		fb_frame = (unsigned char *)(((unsigned int)fb_frame
									  + (PAGE_SIZE / 4 - 1)) & (~(PAGE_SIZE / 4 - 1)));
		fb_needroom -= PAGE_SIZE;
	}
	memset((void *)fb_frame, 0, fb_needroom);
	for (page = (unsigned long)fb_frame;
		 page < PAGE_ALIGN((unsigned long)fb_frame + fb_needroom);
		 page += PAGE_SIZE) {
		SetPageReserved(virt_to_page((void*)page));
	}

	lcd_frame0 = fb_frame;
	cfb->fb0.fix.smem_start = virt_to_phys((void *)lcd_frame0);
	cfb->fb0.fix.smem_len = fb_needroom;
	cfb->fb0.screen_base =
		(unsigned char *)(((unsigned int)lcd_frame0 & 0x1fffffff) | 0xa0000000);
	if (!cfb->fb0.screen_base) {
		printk("jz47xxfb0, %s: unable to map screen memory\n", cfb->fb0.fix.id);
		return -ENOMEM;
	}

#ifdef CONFIG_JZ47XX_AOSDC
	/* Alloc memory for compress function*/
	printk("<< alloc memeory for compress %d bytes======== >>\n", buf_comp_needroom * 2);

	buf_comp = (unsigned char *)kmalloc(buf_comp_needroom, GFP_KERNEL | __GFP_DMA);
	buf_comp1 = (unsigned char *)kmalloc(buf_comp1_needroom, GFP_KERNEL | __GFP_DMA);

	if (!buf_comp || !buf_comp1) {
		printk("<< no memory for buf_comp >>\n");
		return -ENOMEM;
	}

	/* AOSD start address PAGE_SIZE align */
	if ((unsigned int)buf_comp & (PAGE_SIZE / 4 - 1)) {
		printk("buf_comp start address need to set %d words align\n", (int)PAGE_SIZE / 4);
		buf_comp = (unsigned char *)(((unsigned int)buf_comp
									  + (PAGE_SIZE / 4 - 1)) & (~(PAGE_SIZE / 4 - 1)));
		buf_comp_needroom -= PAGE_SIZE;
	}
	memset((void *)buf_comp, 0, buf_comp_needroom);
	for (page = (unsigned long)buf_comp;
		 page < PAGE_ALIGN((unsigned long)buf_comp + buf_comp_needroom);
		 page += PAGE_SIZE) {
		SetPageReserved(virt_to_page((void*)page));
	}

	if ((unsigned int)buf_comp1 & (PAGE_SIZE / 4 - 1)) {
		printk("buf_comp1 start address need to set %d words align\n", (int)PAGE_SIZE / 4);
		buf_comp1 = (unsigned char *)(((unsigned int)buf_comp1
									   + (PAGE_SIZE / 4 - 1)) & (~(PAGE_SIZE / 4 - 1)));
		buf_comp1_needroom -= PAGE_SIZE;
	}
	memset((void *)buf_comp1, 0, buf_comp1_needroom);
	for (page = (unsigned long)buf_comp1;
		 page < PAGE_ALIGN((unsigned long)buf_comp1 + buf_comp1_needroom);
		 page += PAGE_SIZE) {
		SetPageReserved(virt_to_page((void*)page));
	}

	printk("<< alloc memeory for compress done!!! buf_comp: %p, buf_comp1: %p======== >>\n",
		   buf_comp, buf_comp1);
#endif

	return 0;
}

static void jz47xxfb_free_fb_info(struct lcd_cfb_info *cfb)
{
	if (cfb) {
//		fb_alloc_cmap(&cfb->fb.cmap, 0, 0);
		fb_alloc_cmap(&cfb->fb0.cmap, 0, 0);
		kfree(cfb);
	}
}

static void jz47xxfb_unmap_smem(struct lcd_cfb_info *cfb)
{
	struct page * map = NULL;
	unsigned long tmp;

/* 	bpp = jz47xx_lcd_info->osd.fg1.bpp; */
/* 	if ( bpp == 18 || bpp == 24) */
/* 		bpp = 32; */
/* 	if ( bpp == 15 ) */
/* 		bpp = 16; */
/* 	w = jz47xx_lcd_info->osd.fg1.w; */
/* 	h = jz47xx_lcd_info->osd.fg1.h; */
/* 	needroom += ((w * bpp + 7) >> 3) * h; */

/* 	if (cfb && cfb->fb.screen_base) { */
/* 		iounmap(cfb->fb.screen_base); */
/* 		cfb->fb.screen_base = NULL; */
/* 		release_mem_region(cfb->fb.fix.smem_start, */
/* 				   cfb->fb.fix.smem_len); */
/* 	} */

	if (lcd_palette) {
		map = virt_to_page(lcd_palette);
		clear_bit(PG_reserved, &map->flags);
		free_pages((int)lcd_palette, 0);
	}

	if (lcd_frame0) {
		if ((unsigned int)lcd_frame0 & (PAGE_SIZE / 4 - 1)) {
			fb_needroom += PAGE_SIZE;/* map_smem: start address need to be page aligned */
		}
		for (tmp = (unsigned long)lcd_frame0;
		     tmp < PAGE_ALIGN((unsigned long)lcd_frame0 + (unsigned long)fb_needroom);
		     tmp += PAGE_SIZE) {
			map = virt_to_page(phys_to_virt(tmp));
			clear_bit(PG_reserved, &map->flags);
			free_page((int)map);
		}
	}

#ifdef CONFIG_JZ47XX_AOSDC
	if (buf_comp) {
		if ((unsigned int)buf_comp & (PAGE_SIZE / 4 - 1)) {
			buf_comp_needroom += PAGE_SIZE;
		}
		for (tmp = (unsigned long)buf_comp;
		     tmp < PAGE_ALIGN((unsigned long)buf_comp + (unsigned long)buf_comp_needroom);
		     tmp += PAGE_SIZE) {
			map = virt_to_page(phys_to_virt(tmp));
			clear_bit(PG_reserved, &map->flags);
			free_page((int)map);
		}
	}

	if (buf_comp1) {
		if ((unsigned int)buf_comp1 & (PAGE_SIZE / 4 - 1)) {
			buf_comp1_needroom += PAGE_SIZE;
		}
		for (tmp = (unsigned long)buf_comp1;
		     tmp < PAGE_ALIGN((unsigned long)buf_comp1 + (unsigned long)buf_comp1_needroom);
		     tmp += PAGE_SIZE) {
			map = virt_to_page(phys_to_virt(tmp));
			clear_bit(PG_reserved, &map->flags);
			free_page((int)map);
		}
	}
#endif
}

/************************************
 *      Jz4760 Chipset OPS
 ************************************/
static void dump_lcdc_registers(void)
{
	/* LCD Controller Resgisters */
	printk("REG_LCD_CFG:\t0x%08x\n", REG_LCD_CFG);
	printk("REG_LCD_CTRL:\t0x%08x\n", REG_LCD_CTRL);
	printk("REG_LCD_STATE:\t0x%08x\n", REG_LCD_STATE);
	printk("REG_LCD_OSDC:\t0x%08x\n", REG_LCD_OSDC);
	printk("REG_LCD_OSDCTRL:\t0x%08x\n", REG_LCD_OSDCTRL);
	printk("REG_LCD_OSDS:\t0x%08x\n", REG_LCD_OSDS);
	printk("REG_LCD_BGC:\t0x%08x\n", REG_LCD_BGC);
	printk("REG_LCD_KEY0:\t0x%08x\n", REG_LCD_KEY0);
	printk("REG_LCD_KEY1:\t0x%08x\n", REG_LCD_KEY1);
	printk("REG_LCD_ALPHA:\t0x%08x\n", REG_LCD_ALPHA);
	printk("REG_LCD_IPUR:\t0x%08x\n", REG_LCD_IPUR);
	printk("REG_LCD_VAT:\t0x%08x\n", REG_LCD_VAT);
	printk("REG_LCD_DAH:\t0x%08x\n", REG_LCD_DAH);
	printk("REG_LCD_DAV:\t0x%08x\n", REG_LCD_DAV);
	printk("REG_LCD_XYP0:\t0x%08x\n", REG_LCD_XYP0);
	printk("REG_LCD_XYP1:\t0x%08x\n", REG_LCD_XYP1);
	printk("REG_LCD_SIZE0:\t0x%08x\n", REG_LCD_SIZE0);
	printk("REG_LCD_SIZE1:\t0x%08x\n", REG_LCD_SIZE1);
	printk("REG_LCD_RGBC\t0x%08x\n", REG_LCD_RGBC);
	printk("REG_LCD_VSYNC:\t0x%08x\n", REG_LCD_VSYNC);
	printk("REG_LCD_HSYNC:\t0x%08x\n", REG_LCD_HSYNC);
	printk("REG_LCD_PS:\t0x%08x\n", REG_LCD_PS);
	printk("REG_LCD_CLS:\t0x%08x\n", REG_LCD_CLS);
	printk("REG_LCD_SPL:\t0x%08x\n", REG_LCD_SPL);
	printk("REG_LCD_REV:\t0x%08x\n", REG_LCD_REV);
	printk("REG_LCD_IID:\t0x%08x\n", REG_LCD_IID);
	printk("REG_LCD_DA0:\t0x%08x\n", REG_LCD_DA0);
	printk("REG_LCD_SA0:\t0x%08x\n", REG_LCD_SA0);
	printk("REG_LCD_FID0:\t0x%08x\n", REG_LCD_FID0);
	printk("REG_LCD_CMD0:\t0x%08x\n", REG_LCD_CMD0);
	printk("REG_LCD_OFFS0:\t0x%08x\n", REG_LCD_OFFS0);
	printk("REG_LCD_PW0:\t0x%08x\n", REG_LCD_PW0);
	printk("REG_LCD_CNUM0:\t0x%08x\n", REG_LCD_CNUM0);
	printk("REG_LCD_DESSIZE0:\t0x%08x\n", REG_LCD_DESSIZE0);
	printk("REG_LCD_DA1:\t0x%08x\n", REG_LCD_DA1);
	printk("REG_LCD_SA1:\t0x%08x\n", REG_LCD_SA1);
	printk("REG_LCD_FID1:\t0x%08x\n", REG_LCD_FID1);
	printk("REG_LCD_CMD1:\t0x%08x\n", REG_LCD_CMD1);
	printk("REG_LCD_OFFS1:\t0x%08x\n", REG_LCD_OFFS1);
	printk("REG_LCD_PW1:\t0x%08x\n", REG_LCD_PW1);
	printk("REG_LCD_CNUM1:\t0x%08x\n", REG_LCD_CNUM1);
	printk("REG_LCD_DESSIZE1:\t0x%08x\n", REG_LCD_DESSIZE1);
	printk("==================================\n");
	printk("REG_LCD_VSYNC:\t%d:%d\n", REG_LCD_VSYNC>>16, REG_LCD_VSYNC&0xfff);
	printk("REG_LCD_HSYNC:\t%d:%d\n", REG_LCD_HSYNC>>16, REG_LCD_HSYNC&0xfff);
	printk("REG_LCD_VAT:\t%d:%d\n", REG_LCD_VAT>>16, REG_LCD_VAT&0xfff);
	printk("REG_LCD_DAH:\t%d:%d\n", REG_LCD_DAH>>16, REG_LCD_DAH&0xfff);
	printk("REG_LCD_DAV:\t%d:%d\n", REG_LCD_DAV>>16, REG_LCD_DAV&0xfff);
	printk("==================================\n");
#if defined(CONFIG_SOC_JZ4770)
	printk("REG_LCD_PCFG:\t0x%08x\n", REG_LCD_PCFG);
#endif
	/* Smart LCD Controller Resgisters */
	printk("REG_SLCD_CFG:\t0x%08x\n", REG_SLCD_CFG);
	printk("REG_SLCD_CTRL:\t0x%08x\n", REG_SLCD_CTRL);
	printk("REG_SLCD_STATE:\t0x%08x\n", REG_SLCD_STATE);
	printk("==================================\n");

	if ( 1 ) {
		unsigned int * pii = (unsigned int *)dma_desc_base;
		int i, j;
		for (j=0;j< DMA_DESC_NUM ; j++) {
			printk("dma_desc%d(0x%08x):\n", j, (unsigned int)pii);
			for (i =0; i<8; i++ ) {
				printk("\t\t0x%08x\n", *pii++);
			}
		}
	}
}
void print_lcdc_registers(void)	/* debug */
{
//#ifdef  LCD_DEBUG
#if 0
	dump_lcdc_registers();
#endif
}

#ifdef CONFIG_FB_JZ47XX_TVE
static void jz47xxlcd_info_switch_to_TVE(int mode)
{
	struct jz47xxlcd_info *info;

	switch ( mode ) {
	case PANEL_MODE_TVE_PAL:
		info = jz47xx_lcd_info = &jz47xx_info_tve_pal;	
		info->panel.cfg |= LCD_CFG_TVEPEH; /* TVE PAL enable extra halfline signal */
		info->panel.w = TVE_WIDTH_PAL;
		info->panel.h = TVE_HEIGHT_PAL;
		info->panel.fclk = TVE_FREQ_PAL;
		break;
	case PANEL_MODE_TVE_NTSC:
		info = jz47xx_lcd_info = &jz47xx_info_tve_ntsc;
		info->panel.cfg &= ~LCD_CFG_TVEPEH; /* TVE NTSC disable extra halfline signal */
		info->panel.w = TVE_WIDTH_NTSC;
		info->panel.h = TVE_HEIGHT_NTSC;
		info->panel.fclk = TVE_FREQ_NTSC;
		break;
	default:
		printk("%s, %s: Unknown tve mode\n", __FILE__, __FUNCTION__);
		break;
	}
}
#endif
/* initial dma descriptors */
static void jz47xxfb_descriptor_init( struct jz47xxlcd_info * lcd_info )
{
	unsigned int pal_size;
	int fg0_line_size, fg0_frm_size;
	int buffer_line_size, buffer_frm_size;
	int panel_line_size, panel_frm_size;
	int size0, size1;


	switch ( lcd_info->osd.fg0.bpp ) {
	case 1:
		pal_size = 4;
		break;
	case 2:
		pal_size = 8;
		break;
	case 4:
		pal_size = 32;
		break;
	case 8:
	default:
		pal_size = 512;
	}

	pal_size /= 4;

	/*
	 * Normal TFT panel's DMA Chan0:
	 *	TO LCD Panel:
	 * 		no palette:	dma0_desc0 <<==>> dma0_desc0
	 * 		palette :	dma0_desc_palette <<==>> dma0_desc0
	 *	TO TV Encoder:
	 * 		no palette:	dma0_desc0 <<==>> dma0_desc1
	 * 		palette:	dma0_desc_palette --> dma0_desc0
	 * 				--> dma0_desc1 --> dma0_desc_palette --> ...
	 * DMA Chan1:
	 *	TO LCD Panel:
	 * 		dma1_desc0 <<==>> dma1_desc0
	 *	TO TV Encoder:
	 * 		dma1_desc0 <<==>> dma1_desc1
	 */

	dma0_desc_palette 	= dma_desc_base + 0;
	dma0_desc0 		= dma_desc_base + 1;
	dma0_desc1 		= dma_desc_base + 2;
	dma0_desc0_change	= dma_desc_base + 3;
	dma0_desc1_change	= dma_desc_base + 4;

	/* Foreground 0, caculate size */
	if ( lcd_info->osd.fg0.x >= lcd_info->panel.w )
		lcd_info->osd.fg0.x = lcd_info->panel.w - 1;
	if ( lcd_info->osd.fg0.y >= lcd_info->panel.h )
		lcd_info->osd.fg0.y = lcd_info->panel.h - 1;

	size0 = lcd_info->osd.fg0.h << 16 | lcd_info->osd.fg0.w;

	/* lcd display area */
	panel_line_size = (lcd_info->panel.w - 2 * lcd_info->osd.fg0.x) * lcd_info->osd.fg0.bpp / 8;
	panel_line_size = ((panel_line_size + 3) >> 2) << 2; /* word aligned */
	panel_frm_size= panel_line_size * (lcd_info->panel.h - 2 * lcd_info->osd.fg0.y);

	/* total fg0 buffer area */
	fg0_line_size = (lcd_info->osd.fg0.w * (lcd_info->osd.fg0.bpp) / 8);
	fg0_line_size = ((fg0_line_size + 3) >> 2) << 2; /* word aligned */
	fg0_frm_size = fg0_line_size * lcd_info->osd.fg0.h;

	/*total buffer size*/
	buffer_line_size = jz47xx_lcd_panel.osd.fg0.w * lcd_info->osd.fg0.bpp / 8;
	buffer_line_size = ((buffer_line_size + 3) >> 2) << 2; /* word aligned */
	buffer_frm_size= buffer_line_size * jz47xx_lcd_panel.panel.h;

	/* Palette Descriptor */
	dma0_desc_palette->next_desc 	= (unsigned int)virt_to_phys(dma0_desc0);
	dma0_desc_palette->databuf 	= (unsigned int)virt_to_phys((void *)lcd_palette);
	dma0_desc_palette->frame_id 	= (unsigned int)0xaaaaaaaa;
	dma0_desc_palette->cmd 		= LCD_CMD_PAL | pal_size; /* Palette Descriptor */
	/* DMA0 Descriptor */
	/* Normal TFT LCD */
	/* next */
	dma0_desc0->next_desc = (unsigned int)virt_to_phys(dma0_desc0);
	dma0_desc0_change->next_desc = (unsigned int)virt_to_phys(dma0_desc0_change);

	/* frame phys addr */
	dma0_desc0->databuf = dma0_desc0_change->databuf = virt_to_phys((void *)lcd_frame0);

	/* frame id */
	dma0_desc0->frame_id = (unsigned int)0x0000da00; /* DMA0'0 */
	dma0_desc0_change->frame_id = (unsigned int)0x0da000c0; /* DMA0'2 */

	/* others */
	dma0_desc0->cmd = LCD_CMD_EOFINT | panel_frm_size/4;
	dma0_desc0->offsize = (fg0_line_size - panel_line_size)/4;
	dma0_desc0->page_width = panel_line_size/4;

	dma0_desc0->desc_size = size0;

	if (lcd_info->osd.fg0.bpp <= 8) /* load palette only once at setup */
		REG_LCD_DA0 = virt_to_phys(dma0_desc_palette);
	else
		REG_LCD_DA0 = virt_to_phys(dma0_desc0); //tft
	REG_LCD_SIZE0 = size0;
	current_dma0_id = 0;//dma0_desc0;

	dma_cache_wback((unsigned int)(dma_desc_base), (DMA_DESC_NUM)*sizeof(struct jz47xx_lcd_dma_desc));

	/* DMA1 Dummy Descriptor,just for initialization */
	dma1_desc0 = dma0_desc0;
	dma1_desc1 = dma0_desc1;
	dma1_desc0_change = dma0_desc0_change;
	dma1_desc1_change = dma0_desc1_change;

	size1 = size0;
	REG_LCD_DA1 = virt_to_phys(dma1_desc0);	/* set Dma-chan1's Descripter Addrress */
	REG_LCD_SIZE1 = size1;
	current_dma1_id = 1;//dma1_desc0;
}

static void jz47xxfb_set_panel_mode(struct jz47xxlcd_info * lcd_info)
{
	struct jz47xxlcd_panel_t *panel = &lcd_info->panel;

	/* set bpp */
	lcd_info->panel.ctrl &= ~LCD_CTRL_BPP_MASK;
	if ( lcd_info->osd.fg0.bpp == 1 )
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_1;
	else if ( lcd_info->osd.fg0.bpp == 2 )
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_2;
	else if ( lcd_info->osd.fg0.bpp == 4 )
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_4;
	else if ( lcd_info->osd.fg0.bpp == 8 )
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_8;
	else if ( lcd_info->osd.fg0.bpp == 15 )
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_16 | LCD_CTRL_RGB555;
	else if ( lcd_info->osd.fg0.bpp == 16 )
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_16 | LCD_CTRL_RGB565;
	else if ( lcd_info->osd.fg0.bpp > 16 && lcd_info->osd.fg0.bpp < 32+1 ) {
		lcd_info->osd.fg0.bpp = 32;
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_18_24;
	}
	else {
		printk("The BPP %d is not supported\n", lcd_info->osd.fg0.bpp);
		lcd_info->osd.fg0.bpp = 32;
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_18_24;
	}
	lcd_info->panel.ctrl = (lcd_info->panel.ctrl & ~(LCD_CTRL_BST_MASK)) | LCD_CTRL_BST_64; /* Jz4760 support 64bst, should set Conti-16 in the future */
	lcd_info->panel.cfg |= LCD_CFG_NEWDES; /* use 8words descriptor always */
	REG_LCD_CTRL = lcd_info->panel.ctrl; /* LCDC Controll Register */
	REG_LCD_CFG = lcd_info->panel.cfg; /* LCDC Configure Register */
	switch ( lcd_info->panel.cfg & LCD_CFG_MODE_MASK ) {
	case LCD_CFG_MODE_GENERIC_TFT:
	case LCD_CFG_MODE_INTER_CCIR656:
	case LCD_CFG_MODE_NONINTER_CCIR656:
	case LCD_CFG_MODE_SLCD:
	default:		/* only support TFT16 TFT32, not support STN and Special TFT by now(10-06-2008)*/
		REG_LCD_VAT = (((panel->blw + panel->w + panel->elw + panel->hsw)) << 16) | (panel->vsw + panel->bfw + panel->h + panel->efw);
		REG_LCD_DAH = ((panel->hsw + panel->blw) << 16) | (panel->hsw + panel->blw + panel->w);
		REG_LCD_DAV = ((panel->vsw + panel->bfw) << 16) | (panel->vsw + panel->bfw + panel->h);
		REG_LCD_HSYNC = (0 << 16) | panel->hsw;
		REG_LCD_VSYNC = (0 << 16) | panel->vsw;
		break;
	}
}

static void jz47xxfb_set_osd_mode(struct jz47xxlcd_osd_t *lcd_osd_info)
{
	unsigned int rgbc;

	lcd_osd_info->osd_ctrl &= ~(LCD_OSDCTRL_OSDBPP_MASK);
	if ( lcd_osd_info->fg1.bpp == 15 )
		lcd_osd_info->osd_ctrl |= LCD_OSDCTRL_OSDBPP_15_16|LCD_OSDCTRL_RGB555;
	else if ( lcd_osd_info->fg1.bpp == 16 )
		lcd_osd_info->osd_ctrl |= LCD_OSDCTRL_OSDBPP_15_16|LCD_OSDCTRL_RGB565;
	else {
		lcd_osd_info->fg1.bpp = 32;
		lcd_osd_info->osd_ctrl |= LCD_OSDCTRL_OSDBPP_18_24;
	}

	REG_LCD_OSDC 	= lcd_osd_info->osd_cfg; /* F0, F1, alpha, */

	REG_LCD_OSDCTRL = lcd_osd_info->osd_ctrl; /* IPUEN, bpp */

	rgbc = lcd_osd_info->rgb_ctrl;
#if defined(CONFIG_SOC_JZ4770)
#if BPP32_FORMAT_ORDER == FORMAT_X8B8G8R8
	printk("BPP32_FORMAT_ORDER == FORMAT_X8B8G8R8\n");
	/* RGBC = 0xD5, (XBGR) */
	if ( 1 ) { // if gralloc use HAL_PIXEL_FORMAT_RGBX_8888, enable this.
//		unsigned int rgbc;
#define RGB_ORDER_BIT (1<<7)
		//rgbc = REG_LCD_RGBC;
		rgbc = (RGB_ORDER_BIT) | (LCD_RGBC_ODD_BGR<<LCD_RGBC_ODDRGB_BIT) | (LCD_RGBC_EVEN_BGR<<LCD_RGBC_EVENRGB_BIT);
		//REG_LCD_RGBC = rgbc;
//		printk("============ REG_LCD_RGBC= 0x%08X\n", REG_LCD_RGBC);
	}
#endif
#endif

	if(g_disp_type == PANEL_MODE_TVE_BASE)
		rgbc  = lcd_osd_info->rgb_ctrl;
		
	REG_LCD_RGBC 	= rgbc;
	REG_LCD_BGC  	= lcd_osd_info->bgcolor;
	REG_LCD_KEY0 	= lcd_osd_info->colorkey0;
	REG_LCD_KEY1 	= lcd_osd_info->colorkey1;
	REG_LCD_ALPHA 	= lcd_osd_info->alpha;
	REG_LCD_IPUR 	= lcd_osd_info->ipu_restart;
	REG_LCD_XYP0 	= lcd_osd_info->fg0.y << 16 | lcd_osd_info->fg0.x ;
	REG_LCD_XYP1 	= lcd_osd_info->fg1.y << 16 | lcd_osd_info->fg1.x;
}


/* Change Position of Foreground 0 */
static int jz47xxfb0_foreground_move(struct jz47xxlcd_osd_t *lcd_osd_info)
{
	int pos;
	/*
	 * Foreground, only one of the following can be change at one time:
	 * 	1. F0 size, 2. F0 position, 3. F1 size, 4. F1 position
	 *
	 * The rules of fg0 position:
	 * 	fg0.x + fg0.w <= panel.w;
	 * 	fg0.y + fg0.h <= panel.h;
	 *
	 * When output is LCD panel, fg.y can be odd number or even number.
	 * When output is TVE, as the TVE has odd frame and even frame,
	 * to simplified operation, fg.y should be even number always.
	 *
	 */

	/* Foreground 0  */
	if (lcd_osd_info->fg0.x + lcd_osd_info->fg0.w > jz47xx_lcd_info->panel.w)
		lcd_osd_info->fg0.x = jz47xx_lcd_info->panel.w - lcd_osd_info->fg0.w;
	if (lcd_osd_info->fg0.y + lcd_osd_info->fg0.h > jz47xx_lcd_info->panel.h)
		lcd_osd_info->fg0.y = jz47xx_lcd_info->panel.h - lcd_osd_info->fg0.h;

	if (lcd_osd_info->fg0.x >= jz47xx_lcd_info->panel.w)
		lcd_osd_info->fg0.x = jz47xx_lcd_info->panel.w - 1;
	if (lcd_osd_info->fg0.y >= jz47xx_lcd_info->panel.h)
		lcd_osd_info->fg0.y = jz47xx_lcd_info->panel.h - 1;

	pos = lcd_osd_info->fg0.y << 16 | lcd_osd_info->fg0.x;
	if (REG_LCD_XYP0 == pos){
		printk("FG0: same position\n");
		return 0;
	}

	/****jz47xx****/
//	REG_LCD_OSDC &= ~LCD_OSDC_F0EN;
	REG_LCD_OSDCTRL |= LCD_OSDCTRL_CHANGES;
	REG_LCD_XYP0 = pos;
//	REG_LCD_OSDC |= LCD_OSDC_F0EN;
	/*********************************************/
	return 0;
}
/* Change Window size of Foreground 0 */
static int jz47xxfb0_foreground_resize(struct jz47xxlcd_osd_t *lcd_osd_info)
{
	struct lcd_cfb_info *cfb = jz47xxfb_info;
	int size, fg0_line_size, fg0_frm_size;
//	int desc_len = sizeof(struct jz47xx_lcd_dma_desc);
	/*
	 * NOTE:
	 * Foreground change sequence:
	 * 	1. Change Position Registers -> LCD_OSDCTL.Change;
	 * 	2. LCD_OSDCTRL.Change -> descripter->Size
	 * Foreground, only one of the following can be change at one time:
	 * 	1. F0 size;
	 *	2. F0 position
	 * 	3. F1 size
	 *	4. F1 position
	 */

	/*
	 * The rules of f0, f1's position:
	 * 	f0.x + f0.w <= panel.w;
	 * 	f0.y + f0.h <= panel.h;
	 *
	 * When output is LCD panel, fg.y and fg.h can be odd number or even number.
	 * When output is TVE, as the TVE has odd frame and even frame,
	 * to simplified operation, fg.y and fg.h should be even number always.
	 *
	 */
	/* Foreground 0  */
	if (lcd_osd_info->fg0.x + lcd_osd_info->fg0.w > jz47xx_lcd_info->panel.w)
		lcd_osd_info->fg0.w = jz47xx_lcd_info->panel.w - lcd_osd_info->fg0.x;
	if (lcd_osd_info->fg0.y + lcd_osd_info->fg0.h > jz47xx_lcd_info->panel.h)
		lcd_osd_info->fg0.h = jz47xx_lcd_info->panel.h - lcd_osd_info->fg0.y;

	size = lcd_osd_info->fg0.h << 16 | lcd_osd_info->fg0.w;

	if (REG_LCD_SIZE0 == size) {
		printk("FG0: same size\n");
		return 0;
	}

	fg0_line_size = lcd_osd_info->fg0.w * lcd_osd_info->fg0.bpp / 8;
	fg0_line_size = ((fg0_line_size + 3) >> 2) << 2; /* word aligned */
	fg0_frm_size = fg0_line_size * lcd_osd_info->fg0.h;

	REG_LCD_OSDCTRL |= LCD_OSDCTRL_CHANGES;
	/* set change bit */
	REG_LCD_OSDCTRL |= LCD_OSDCTRL_CHANGES;

	dma0_desc0->cmd = dma0_desc1->cmd = LCD_CMD_EOFINT | fg0_frm_size/4;
	dma0_desc0->offsize = dma0_desc1->offsize =0;
	dma0_desc0->page_width = dma0_desc1->page_width = 0;
	dma0_desc0->desc_size = dma0_desc1->desc_size = size;
	REG_LCD_SIZE0 = size;

	dma_cache_wback((unsigned int)(dma_desc_base), (DMA_DESC_NUM)*sizeof(struct jz47xx_lcd_dma_desc));

	jz47xxfb_set_var(&cfb->fb0.var, 0, &cfb->fb0);
	return 0;
}

/*
 * Set lcd pixel clock
 */
static void jz47xxfb_change_clock( struct jz47xxlcd_info * lcd_info )
{
	unsigned int val = 0;
	unsigned int pclk;

	/* Timing setting */
	cpm_stop_clock(CGM_LCD);

	val = lcd_info->panel.fclk; /* frame clk */

	if ( (lcd_info->panel.cfg & LCD_CFG_MODE_MASK) != LCD_CFG_MODE_SERIAL_TFT) {
		pclk = val * (lcd_info->panel.w + lcd_info->panel.hsw + lcd_info->panel.elw + lcd_info->panel.blw) * (lcd_info->panel.h + lcd_info->panel.vsw + lcd_info->panel.efw + lcd_info->panel.bfw); /* Pixclk */
	}
	else {
		/* serial mode: Hsync period = 3*Width_Pixel */
		pclk = val * (lcd_info->panel.w*3 + lcd_info->panel.hsw + lcd_info->panel.elw + lcd_info->panel.blw) * (lcd_info->panel.h + lcd_info->panel.vsw + lcd_info->panel.efw + lcd_info->panel.bfw); /* Pixclk */
	}

	/********* In TVE mode PCLK = 27MHz ***********/
	if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) { 		/* LCDC output to TVE */
		cpm_stop_clock(CGM_TVE);
		pclk = 27000000;
		cpm_set_clock(CGU_TVECLK, pclk);
		cpm_start_clock(CGM_TVE);
	} else {
		cpm_set_clock(CGU_LPCLK, pclk);
	}
	printk("LCDC:Request PixCLK:%d\n",pclk);
	printk("LCDC:Real    PixCLK:%d\n",cpm_get_clock(CGU_LPCLK));
	cpm_start_clock(CGM_LCD);
	udelay(1000);
}


/*
 * jz47xxfb_set_mode(), set osd configure, resize foreground
 *
 */
static void jz47xxfb_set_mode(struct jz47xxlcd_osd_t * lcd_osd_info)
{
	jz47xxfb_set_osd_mode(lcd_osd_info);
	jz47xxfb0_foreground_resize(lcd_osd_info);
}

/*
 * jz47xxfb_deep_set_mode,
 *
 */
void jz47xxfb_deep_set_mode( struct jz47xxlcd_info * lcd_info )
{
	/* configurate sequence:
	 * 1. disable lcdc.
	 * 2. init frame descriptor.
	 * 3. set panel mode
	 * 4. set osd mode
	 * 5. start lcd clock in CPM
	 * 6. enable lcdc.
	 */
	struct lcd_cfb_info *cfb = jz47xxfb_info;
	if(REG_LCD_CTRL & LCD_CTRL_ENA)
		lcd_controller_disable();
	lcd_info->osd.fg_change = FG_CHANGE_ALL; /* change FG0, FG1 size, postion??? */
	jz47xxfb_set_osd_mode(&lcd_info->osd);
	jz47xxfb_set_panel_mode(lcd_info);
	jz47xxfb_descriptor_init(lcd_info);
	jz47xxfb_change_clock(lcd_info);

	jz47xxfb_set_var(&cfb->fb0.var, 0, &cfb->fb0);
//	jz47xxfb_set_var(&cfb->fb.var, 1, &cfb->fb);
	//	if (ipu_flag)  ipu_open();
	lcd_controller_enable();		/* Enable LCD Controller */
}


static irqreturn_t jz47xxfb_interrupt_handler(int irq, void *dev_id)
{
	unsigned long irq_flags;
	unsigned int state;//, osdstate;
	struct lcd_cfb_info *cfb = jz47xxfb_info;
	static int irqcnt = 0;

	spin_lock_irqsave(&cfb->update_lock, irq_flags);
//	dprintk("Lcd irq, state=0x%08x, osdstate=0x%08x\n", state, osdstate);
	state = REG_LCD_STATE;
//	osdstate = REG_LCD_OSDS;
	if (state & LCD_STATE_EOF) {/* End of frame */
		REG_LCD_STATE = state & ~LCD_STATE_EOF;
//		dprintk("lcd dma eof interrupt\n");
	}
/****************************************************************/
#if 0
	if (state & LCD_STATE_IFU0) {
		printk("%s, InFiFo0 underrun\n", __FUNCTION__);
		REG_LCD_STATE = state & ~LCD_STATE_IFU0;
	}

	if (state & LCD_STATE_IFU1) {
		printk("%s, InFiFo1 underrun\n", __FUNCTION__);
		REG_LCD_STATE = state & ~LCD_STATE_IFU1;
	}
#endif
	if (state & LCD_STATE_OFU) {
		REG_LCD_STATE = state & ~LCD_STATE_OFU;
		if ( irqcnt++ > 100 ) {
			__lcd_disable_ofu_intr();
			printk("disable Out FiFo underrun irq.\n");
		}
		printk("%s, Out FiFo underrun.\n", __FUNCTION__);
	}
/****************************************************************/
	cfb->frame_done = cfb->frame_requested;
	spin_unlock_irqrestore(&cfb->update_lock, irq_flags);
	wake_up(&cfb->frame_wq);

	return IRQ_HANDLED;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
volatile int screen_state_on = 2;
/*
 * screen_state_on is used as flag of screen state;
 * value 2 lcd controler are resumed.
 * 	 1 lcd controler will resume.;
 *	 0 lcd controler will suspend;
 */
static void jz47xxfb_backlight_suspend(struct early_suspend *h)
{
	ENTER();
	lcd_backlight_is_suspend = 1;
	//lcd_close_backlight();
	screen_state_on = 0;
	LEAVE();
}

static void jz47xxfb_backlight_resume(struct early_suspend *h)
{
	ENTER();
	//printk("backlight_resume Wait lcd panel stable %d ms\n", LCD_PANEL_POWER_ON_STABLE_TIME);
	/* Wait lcd panel stable */
	//schedule_timeout_uninterruptible((LCD_PANEL_POWER_ON_STABLE_TIME)/10); /* Wait 200ms */

	lcd_backlight_is_suspend = 0;
	wake_up(&wait_backlight_resume);
	//lcd_backlight_resume();

	LEAVE();
}

static void jz47xxfb_lcd_controller_suspend(struct early_suspend *h)
{
	ENTER();

	lcd_panel_display_off();
	lcd_controller_disable();

	/* 
	 * Disable FG1 to avoid screen jumpiness.
	 * situation: playing video then sleep,and resume again.
	 * When resume the IPU need not to open FG1,video player will open it.
	*/
	if ((REG_LCD_OSDC & LCD_OSDC_F1EN) && (REG_LCD_OSDCTRL & LCD_OSDCTRL_IPU)) {
		__lcd_disable_f1();/* Make sure LCDC is disabled */
		printk("LCD FG1 disable!\n");
   	}
	
	ipu_driver_suspend(ipu_priv);

	/*
	 * It doesn't need to do __cpm_stop_lcd() here, because CPM
	 * will stop lcd clock automatically when entering sleep.
	 */
	LEAVE();
}

static void jz47xxfb_lcd_controller_resume(struct early_suspend *h)
{
	ENTER();
	/*
	 * It doesn't need to do __cpm_start_lcd() here, because CPM
	 * will stop lcd clock automatically when entering sleep.
	 */

	if (jz_lcd_stat.controller) {
		lcd_controller_disable();
	}

	ipu_driver_resume(ipu_priv);

	if(REG_LCD_OSDC & LCD_OSDC_F0EN){
		printk("LCD FG0 enable!\n");
	}else{
		printk("LCD FG0 disable!\n");
	}

	/* 
	 * Disable FG1 to avoid screen jumpiness.
	 * situation: playing video then sleep,and resume again.
	 * When resume the IPU need not to open FG1,video player will open it.
	*/
	if (REG_LCD_OSDC & LCD_OSDC_F1EN) {
		__lcd_disable_f1();/* Make sure LCDC is disabled */
		printk("LCD FG1 disable!\n");
		if (REG_LCD_OSDC & LCD_OSDC_F1EN) {
			printk("failed to disable FG1.\n");
		}
   	}

	lcd_controller_enable();
	lcd_panel_display_on();

	screen_state_on = 2;

	LEAVE();
}

#endif /* CONFIG_HAS_EARLYSUSPEND */

/* The following routine is only for test */

static void jz47xx_lcd_gpio_init(void)
{
	/* gpio init __gpio_as_lcd */
	if (jz47xx_lcd_info->panel.cfg & LCD_CFG_MODE_TFT_16BIT)
		__gpio_as_lcd_16bit();
//		lcd_pdata->ops->gpio_as_lcd_16bit();
	else if (jz47xx_lcd_info->panel.cfg & LCD_CFG_MODE_TFT_24BIT)
		__gpio_as_lcd_24bit();
//		lcd_pdata->ops->gpio_as_lcd_24bit();
	else
 		__gpio_as_lcd_18bit();
//		lcd_pdata->ops->gpio_as_lcd_18bit();

	/* Configure SLCD module for setting smart lcd control registers */
#if defined(CONFIG_FB_JZ47XX_SLCD)
	__lcd_as_smart_lcd();
	__slcd_disable_dma();
	__init_slcd_bus();	/* Note: modify this depend on you lcd */
#endif
//	__lcd_display_pin_init();
	lcd_panel->panel_ops->panel_init();
}

static void jz47xx_lcd_init_cfg(void)
{

	jz47xx_lcd_info->osd.osd_cfg |= LCD_OSDC_F0EN; /* only open fg0 */
	//jz47xx_lcd_info->osd.osd_cfg |= LCD_OSDC_F1EN; /* only open fg1 */

	/* In special mode, we only need init special pin,
	 * as general lcd pin has init in uboot */

	/* Foreground 0 support bpp = 1, 2, 4, 8, 15, 16, 18, 24 */
	switch ( jz47xx_lcd_info->osd.fg0.bpp ) {
	case 17 ... 32:
		jz47xx_lcd_info->osd.fg0.bpp = 32;
		break;
	default:
		break;
	}

	/* Foreground 1 support bpp = 15, 16, 18, 24 */
	switch ( jz47xx_lcd_info->osd.fg1.bpp ) {
	case 15:
	case 16:
		break;
	case 17 ... 32:
		jz47xx_lcd_info->osd.fg1.bpp = 32;
		break;
	default:
		printk("jz47xxfb fg1 not support bpp(%d), force to 32bpp\n",
		       jz47xx_lcd_info->osd.fg1.bpp);
		jz47xx_lcd_info->osd.fg1.bpp = 32;
	}
}

#ifdef LCD_DEBUG
static void display_v_color_bar(void)
{
	int i,j;
	int w, h;
	unsigned short * p16;
	unsigned int * p32;
	int bpp;

	p16 = (unsigned short *)lcd_frame0;
	p32 = (unsigned int *)lcd_frame0;
	w = jz47xx_lcd_info->osd.fg0.w;
	h = jz47xx_lcd_info->osd.fg0.h;
	bpp = jz47xx_lcd_info->osd.fg0.bpp;

	printk("======================= LCD COLOR BAR w,h,bpp(%d,%d,%d)============================\n", w,h,bpp);

	for (i=0;i<h;i++) {
		for (j=0;j<w;j++) {
			short c16;
			int c32;
			switch ((j/10)%4) {
			case 0: 
				c16 = 0xF800;
				c32 = 0x00FF0000;
				break;
			case 1: 
				c16 = 0x07C0;
				c32 = 0x0000FF00;
				break;
			case 2: 
				c16 = 0x001F;
				c32 = 0x000000FF;
				break;
			default: 
				c16 = 0xFFFF;
				c32 = 0xFFFFFFFF;
				break;
			}
			switch (bpp) {
			case 18:
			case 24:
			case 32:
				*p32++ = c32;
				break;
			default:
				*p16 ++ = c16;
			}
		}
	}
}
#endif	/* #ifdef LCD_DEBUG */


/**
 * Soc specific definition
 * 
 * In order to achieve soc-independent lcd panel drivers, 
 *  re-define the soc specific operations here, so the lcd panels 
 *  can use them instead of using the soc operations directly.
 */
#undef GPIO_GPA0_SPEC_USED

static void gpio_as_output(unsigned int pin)
{
#ifndef GPIO_GPA0_SPEC_USED
	if(pin == 0)	return ;
#endif
	__gpio_as_output(pin);
}

static void gpio_as_input(unsigned int pin)
{
#ifndef GPIO_GPA0_SPEC_USED
	if(pin == 0)	return ;
#endif
	__gpio_as_input(pin);
}

static void gpio_set_pin(unsigned int pin)
{
#ifndef GPIO_GPA0_SPEC_USED
	if(pin == 0)	return ;
#endif
	__gpio_as_output(pin);
	__gpio_set_pin(pin);
}

static int gpio_get_pin(unsigned int pin)
{
#ifndef GPIO_GPA0_SPEC_USED
	if(pin == 0)
		printk("%s:%d  GPA0 is true ???",__FILE__,__LINE__);
#endif
	return __gpio_get_pin(pin);
}

static void gpio_clear_pin(unsigned int pin)
{
#ifndef GPIO_GPA0_SPEC_USED
	if(pin == 0)	return ;
#endif
	__gpio_as_output(pin);
	__gpio_clear_pin(pin);
}

static void gpio_enable_pull(unsigned int pin)
{
#ifndef GPIO_GPA0_SPEC_USED
	if(pin == 0)	return ;
#endif
	__gpio_enable_pull(pin);
}

static void gpio_disable_pull(unsigned int pin)
{
#ifndef GPIO_GPA0_SPEC_USED
	if(pin == 0)	return ;
#endif
	__gpio_disable_pull(pin);
}

static void spi_init(struct lcd_board_info *board, unsigned int timming)
{
	if (board) {
		SPEN = board->board_pin->SPEN;
		SPCK = board->board_pin->SPCK;
		SPDT = board->board_pin->SPDT;
		SPRS = board->board_pin->SPRS;
	}
}

static void spi_read(unsigned char reg, unsigned char *val, 
		     unsigned char readByte)
{
	unsigned char i,j;
	unsigned char value = reg;

	if((val == 0)||(readByte == 0))
		return;
		
//	printk("reg=%x, readByte=%d\n",reg,readByte);		

	gpio_clear_pin(SPEN);
	udelay(50);
	value = reg;
	
	for(i=0; i<8; i++)
	{
		if((value & 0x80) == 0x80)
			gpio_set_pin(SPDT);
		else
			gpio_clear_pin(SPDT);

		udelay(50);
		gpio_clear_pin(SPCK);
		udelay(50);
		gpio_set_pin(SPCK);
		value = (value << 1);
		udelay(50);
	} 

	gpio_as_input(SPDT);
	
	//dummy clock input
	udelay(50);
	gpio_clear_pin(SPCK);
	udelay(50);
	gpio_set_pin(SPCK);
		
	gpio_set_pin(SPRS);
	gpio_disable_pull(SPDT);
	
	for (i = 0; i < readByte; i++)
	{
		val[i] = 0x0;

		for(j = 0; j < 8; j++)
		{
			val[i] = (val[i] << 1);
			gpio_clear_pin(SPCK);
			udelay(50);
			gpio_set_pin(SPCK);
			udelay(50);

			val[i] |= gpio_get_pin(SPDT);
		}
	}
    
	gpio_set_pin(SPEN);
	udelay(400);
}

static void spi_read_nodummy(unsigned char reg, unsigned char *val, 
		     unsigned char readByte)
{
	unsigned char i,j;
	unsigned char value = reg;

	if((val == 0)||(readByte == 0))
		return;
		
//	printk("reg=%x, readByte=%d\n",reg,readByte);		

	gpio_clear_pin(SPEN);
	udelay(50);
	value = reg;
	
	for(i=0; i<8; i++)
	{
		if((value & 0x80) == 0x80)
			gpio_set_pin(SPDT);
		else
			gpio_clear_pin(SPDT);

		udelay(50);
		gpio_clear_pin(SPCK);
		udelay(50);
		gpio_set_pin(SPCK);
		value = (value << 1);
	} 


	udelay(10);
	gpio_as_input(SPDT);
	
	gpio_set_pin(SPRS);
	gpio_disable_pull(SPDT);
	
	for (i = 0; i < readByte; i++)
	{
		val[i] = 0x0;

		for(j = 0; j < 8; j++)
		{
			val[i] = (val[i] << 1);
			gpio_clear_pin(SPCK);
			udelay(50);
			gpio_set_pin(SPCK);
			udelay(50);

			val[i] |= gpio_get_pin(SPDT);
		}
	}
    
	gpio_set_pin(SPEN);
	udelay(400);
}

static void spi_write(unsigned char val, unsigned char data)
{
	unsigned char no;
	unsigned char value;

	if (data)
		gpio_set_pin(SPRS);
	
	udelay(50);
	value = val;
	
	for(no=0; no<8; no++)
	{
		gpio_clear_pin(SPCK);
		udelay(2);
	
		if((value & 0x80) == 0x80)
			gpio_set_pin(SPDT);
		else
			gpio_clear_pin(SPDT);
	
		value = (value << 1);
		udelay(50);
		gpio_set_pin(SPCK);
		udelay(50);
	} 
}

struct lcd_soc_ops_info soc_ops = {
	.gpio_as_output		= gpio_as_output,
	.gpio_as_input		= gpio_as_input,
	.gpio_set_pin		= gpio_set_pin,
	.gpio_get_pin		= gpio_get_pin,
	.gpio_clear_pin		= gpio_clear_pin,
	.gpio_enable_pull	= gpio_enable_pull,
	.gpio_disable_pull	= gpio_disable_pull,

	.spi_init		= spi_init,
	.spi_read		= spi_read,
	.spi_write		= spi_write,
	.spi_read_nodummy	= spi_read_nodummy,
};

struct lcd_soc_info soc_info = {
	.soc_ops	=	&soc_ops,
};

struct lcd_soc_info *lcd_soc = &soc_info;
/**
 * lcd_register_panel() - register lcd panel with lcd driver
 * @panel: the panel to be registered
 *
 * @lcd_register_panel() is called during lcd panel init.
 */
void lcd_register_panel(struct lcd_panel_info *panel)
{
	list_add_tail(&panel->list, &_panels);
}
/*
 * check_invalid_pin() - check invalid pins in board
 * @check_invalid_pin() is called during lcd panel probe.
*/
int check_invalid_pins(void)
{
	struct lcd_board_pin_info *pboard_pin;
	unsigned int *ppin;
	int i,all_pins,invalid_pins = 0, undef_pins = 0;
	
#ifndef  INVALID_PIN
//#error "Please defined INVALID_PIN in Board Head file first!!!"
#define INVALID_PIN   0
#endif
	dprintk("Check invaild pin !\n");
	pboard_pin = lcd_board->board_pin;
	ppin = &pboard_pin->LCD_DITHB_PIN;
	all_pins = sizeof(struct lcd_board_pin_info)/sizeof(unsigned int);
	for(i=0;i< all_pins;i++){
		if(*ppin == INVALID_PIN)
			invalid_pins++;
		
		if(!(*ppin)){
			*ppin = INVALID_PIN;
			invalid_pins++; 
			undef_pins++;
		}
		
		ppin++;
	}
	if(undef_pins)
		printk(KERN_WARNING "WARNNING:LCD Panel: %d invalid_pins in all,and %d undef_pins in it!\n",invalid_pins,undef_pins);

	return invalid_pins;
}

/**
 * panel_detect() - determine which lcd panel is in use.
 * @pdata: the platform_data from platform_device
 */
int panel_detect(struct lcd_board_info *board_info)
{
	struct lcd_panel_info *p;

	char *named_panel = board_info->named_panel;
	if (named_panel) {
		list_for_each_entry(p, &_panels, list) {
			if(!strcmp(named_panel, p->name)) {
				if(p->panel_ops->panel_probe) 
					p->panel_ops->panel_probe(lcd_soc, board_info);
				printk(KERN_INFO "select named_panel %s.\n", p->name);
				memcpy(lcd_panel, p, sizeof(struct lcd_panel_info));
				return 0;
			}
		}
	}

	list_for_each_entry(p, &_panels, list) {
		if (p && !p->panel_probable) {
			if(p->panel_ops->panel_probe) 
				p->panel_ops->panel_probe(lcd_soc, board_info);
			memcpy(lcd_panel, p, sizeof(struct lcd_panel_info));
			continue;
		}

		if (p && p->panel_ops->panel_probe) {
			if(!p->panel_ops->panel_probe(lcd_soc, board_info)) {
				memcpy(lcd_panel, p, sizeof(struct lcd_panel_info));
			}
		} else {
			printk(KERN_ERR "panel %s has no probe method\n", p->name);
		}
	}
	/* nothing in the list_head */
	if (lcd_panel) {
		printk(KERN_INFO "panel %s selected\n", lcd_panel->name);
		return 0;
	}
	else
		return -1;
}

/**
 * jz47xx_lcd_config() - pad struct jz47xxlcd_info with struct jz_lcd_panel_t
 */
static void jz47xx_lcd_config(struct jz47xxlcd_info *info, struct lcd_panel_info *panel)
{
	info->panel.cfg |= LCD_CFG_RECOVER | 	/* Underrun recover */
		LCD_CFG_NEWDES; 		/* 8words descriptor */
	info->panel.ctrl = LCD_CTRL_BST_32 | panel->panel_attr->ctrl;
	info->osd.osd_cfg = LCD_OSDC_OSDEN | LCD_OSDC_F0EN;
	info->osd.osd_ctrl = 0;			/* disable ipu,  */
	info->osd.rgb_ctrl = 0;
	info->osd.bgcolor = 0x000000;		/* set background color Black */
	info->osd.colorkey0 = 0;		/* disable colorkey */
	info->osd.colorkey1 = 0;		/* disable colorkey */
	info->osd.alpha = 0xff;			/* alpha value */
	info->osd.ipu_restart = 0x80001000;	/* ipu restart */
	info->osd.fg_change = FG_CHANGE_ALL;	/* change all initially */

	info->panel.slcd_cfg	= panel->panel_attr->slcd_cfg;
	/* support libgralloc/framebuffer.cpp xdpi and ydpi */
	info->panel.phys_width  = panel->panel_attr->phys_width;
	info->panel.phys_height = panel->panel_attr->phys_height;
	info->panel.w		= panel->panel_attr->w;
	info->panel.h		= panel->panel_attr->h;
	info->panel.fclk	= panel->panel_attr->fclk;
	info->panel.hsw	= panel->panel_attr->hsw;
	info->panel.vsw	= panel->panel_attr->vsw;
	info->panel.elw	= panel->panel_attr->elw;
	info->panel.blw	= panel->panel_attr->blw;
	info->panel.efw	= panel->panel_attr->efw;
	info->panel.bfw	= panel->panel_attr->bfw;

	if (panel->panel_attr->bpp == 24) {
		info->panel.cfg |= LCD_CFG_MODE_TFT_24BIT;
		info->panel.ctrl |= LCD_CTRL_BPP_18_24;
	}
	else if (panel->panel_attr->bpp == 18)
		info->panel.cfg |= LCD_CFG_MODE_TFT_18BIT;
	else
		info->panel.cfg |= LCD_CFG_MODE_TFT_16BIT;

	if (panel->panel_attr->vsp == 1)
		info->panel.cfg |= LCD_CFG_VSP;
	if (panel->panel_attr->hsp == 1)
		info->panel.cfg |= LCD_CFG_HSP;
	if (panel->panel_attr->pcp == 1)
		info->panel.cfg |= LCD_CFG_PCP;
	if (panel->panel_attr->dep == 1)
		info->panel.cfg |= LCD_CFG_DEP;
	if (panel->panel_attr->slcd_cfg == 0)
		info->panel.cfg |= LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_LCDPIN_LCD;
	else 
		info->panel.cfg |= LCD_CFG_MODE_SLCD;

	info->osd.fg0.bpp = panel->panel_attr->fg0_bpp;
	info->osd.fg0.x = 0;
	info->osd.fg0.y = 0;
	info->osd.fg0.w = info->panel.w;
	info->osd.fg0.h = info->panel.h;

	info->osd.fg1.bpp = panel->panel_attr->fg1_bpp;
	info->osd.fg1.x = 0;
	info->osd.fg1.y = 0;
	info->osd.fg1.w = info->panel.w;
	info->osd.fg1.h = info->panel.h;

}

static ssize_t dump_lcd(struct device *dev, struct device_attribute *attr, char *buf)
{
	dump_lcdc_registers();
	return 0;
}

extern void dump_ipu_regs(void);
static ssize_t dump_ipu(struct device *dev, struct device_attribute *attr, char *buf)
{
	dump_ipu_regs();
	return 0;
}

static ssize_t dump_aosd(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef CONFIG_JZ47XX_AOSDC
	print_aosd_registers();
#endif
	return 0;
}

static struct device_attribute lcd_sysfs_attrs[] = {
	__ATTR(dump_lcd, S_IRUGO|S_IWUSR, dump_lcd, NULL),
    __ATTR(dump_ipu, S_IRUGO|S_IWUSR, dump_ipu, NULL),
	__ATTR(dump_aosd, S_IRUGO|S_IWUSR, dump_aosd, NULL),
};

static int jz47xxfb_probe(struct platform_device *pdev)
{
	struct lcd_cfb_info *cfb={0,};
	int i, err = 0;

	if (!pdev)
		return -EINVAL;
	lcd_board = pdev->dev.platform_data;
	if(!lcd_board) {
		printk("platform_data is null!\n");
		return -EINVAL;
	}

	// init the spi using the pins from pdev
	spi_init(lcd_board, 0);

	lcd_panel = kzalloc(sizeof(*lcd_panel), GFP_KERNEL);

	// detect which panel we are in use. 
	// if succeed, copy the panel struct to lcd_panel.
	if(panel_detect(lcd_board)) {
		printk("panel detect error, have you chosen the panel yet?\n");
		return -EINVAL;
	}

	if(!lcd_panel)
		printk("lcd_panel is null\n");

	// filled the struct jz47xx_lcd_info with lcd_panel from panel driver.
	jz47xx_lcd_config(jz47xx_lcd_info, lcd_panel);
	if(!jz47xx_lcd_info)
		printk("jz47xx_lcd_info is null\n");

	jz_lcd_stat.controller = 0;
	jz_lcd_stat.panel = 0;
	jz_lcd_stat.backlight = 0;

	lcd_close_backlight();
	jz47xx_lcd_gpio_init();		/* gpio init */
	jz47xx_lcd_init_cfg();		/* first config of lcd */

	__lcd_clr_dis();
	lcd_controller_disable();

	/* init clock */
	__lcd_slcd_special_on();

#ifdef CONFIG_JZ47XX_AOSDC
	aosd_info = kmalloc(sizeof(struct jz47xx_aosd_info) , GFP_KERNEL);
	if (!aosd_info) {
		printk("malloc aosd_info failed\n");
		return -ENOMEM;
	}
	memset(aosd_info, 0, sizeof(struct jz47xx_aosd_info));

	jz47xx_compress_init();
#endif	

	for (i = 0; i < ARRAY_SIZE(lcd_sysfs_attrs); i++) {
		err = device_create_file(&pdev->dev, &lcd_sysfs_attrs[i]);
		if (err)
			break;
	}
	if(err)
		goto failed;

	cfb = jz47xxfb_alloc_fb_info();
	if (!cfb)
		goto failed;

	err = jz47xxfb_map_smem(cfb);
	if (err)
		goto failed;
	spin_lock_init(&lcd_controller_startup_update_lock);
	spin_lock_init(&cfb->update_lock);
	init_waitqueue_head(&cfb->frame_wq);
	init_waitqueue_head(&wait_backlight_resume);
	cfb->frame_requested = cfb->frame_done = 0;

#ifdef CONFIG_HAS_EARLYSUSPEND
	cfb->early_suspend.suspend = jz47xxfb_lcd_controller_suspend;
	cfb->early_suspend.resume = jz47xxfb_lcd_controller_resume;
	cfb->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&cfb->early_suspend);

	cfb->earlier_suspend.suspend = jz47xxfb_backlight_suspend;
	cfb->earlier_suspend.resume = jz47xxfb_backlight_resume;
	cfb->earlier_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	register_early_suspend(&cfb->earlier_suspend);
#endif

	jz47xxfb_deep_set_mode( jz47xx_lcd_info );

	/* registers frame buffer devices */
	/* register fg0 */
	err = register_framebuffer(&cfb->fb0);
	if (err < 0) {
		dprintk("jz47xxfb_init(): register framebuffer err.\n");
		goto failed;
	}
	printk("fb%d: %s frame buffer device, using %dK of video memory\n",
	       cfb->fb0.node, cfb->fb0.fix.id, cfb->fb0.fix.smem_len>>10);

#ifdef CONFIG_JZSOC_BOOT_LOGO
	{
#ifndef BOOT_ANIMATION_TEST
		load_565_image(INIT_IMAGE_FILE,lcd_frame0);
#if ANDROID_NUMBER_OF_BUFFERS > 1
		{
			int yres, yres_virtual;
			int line_length;
			yres = cfb->fb0.var.yres;
			yres_virtual = cfb->fb0.var.yres_virtual;
			line_length = cfb->fb0.fix.line_length;
			//printk("cfb->fb0.var.yres=%d, yres_virtual=%d, line_length=%d\n", yres, yres_virtual, line_length);
			if ( yres_virtual >= 2*yres) {
				load_565_image(INIT_IMAGE_FILE, (unsigned char *)lcd_frame0 + yres*line_length);
			}
		}
#endif	/* ANDROID_NUMBER_OF_BUFFERSxsx */
#else /*BOOT_ANIMATION_TEST*/
		int index;
		boot_animation_init();
		load_565_image(logofilename[0],lcd_frame0);
		for(index=0;index<ANIMATION_BUF_NUM-1;index++)
			load_565_image(logofilename[index+1],animation_buf[index]);
#endif	/* #ifdef BOOT_ANIMATION_TEST	 */
		__flush_cache_vmap();
	}
#endif
	if (request_irq(IRQ_LCD, jz47xxfb_interrupt_handler, IRQF_DISABLED,
			"lcd", 0)) {
		err = -EBUSY;
		goto failed;
	}

//#if 1							/* REG_LCD_PCFG */
#ifdef CONFIG_SOC_JZ4760B
	//REG_LCD_PCFG = 0xC0000888;
	if (REG_LCD_PCFG != (0x70000000))
		REG_LCD_PCFG = (0x70000000);
#elif defined(CONFIG_SOC_JZ4770)

	if ( 1 ) {
		unsigned int pcfg;

//		pcfg = 0xC0000000 | (256<<18) | (256<<9) | (256<<0) ;
		pcfg = 0xC0000000 | (511<<18) | (400<<9) | (256<<0) ;

		REG_LCD_PCFG = pcfg;
		printk("============ REG_LCD_PCFG= 0x%08X\n", pcfg);
	}
#endif

#if 0 //defined(CONFIG_SOC_JZ4770)
#if BPP32_FORMAT_ORDER == FORMAT_X8B8G8R8
	printk("BPP32_FORMAT_ORDER == FORMAT_X8B8G8R8\n");
	/* RGBC = 0xD5, (XBGR) */
	if ( 1 ) { // if gralloc use HAL_PIXEL_FORMAT_RGBX_8888, enable this.
		unsigned int rgbc;
#define RGB_ORDER_BIT (1<<7)
		//rgbc = REG_LCD_RGBC;
		rgbc = (RGB_ORDER_BIT) | (LCD_RGBC_ODD_BGR<<LCD_RGBC_ODDRGB_BIT) | (LCD_RGBC_EVEN_BGR<<LCD_RGBC_EVENRGB_BIT);
		REG_LCD_RGBC = rgbc;
//		printk("============ REG_LCD_RGBC= 0x%08X\n", REG_LCD_RGBC);
	}
#endif
#endif

#ifdef CONFIG_LEDS_CLASS
	err = led_classdev_register(&pdev->dev, &lcd_backlight_led);
	if (err < 0)
		goto failed;
#endif

	lcd_display_on();

#ifdef CONFIG_LEDS_CLASS
	/* Wait lcd panel stable */
	schedule_timeout_uninterruptible((LCD_PANEL_POWER_ON_STABLE_TIME)/10); /* Wait 200ms */
	lcd_init_backlight();
#endif
	print_lcdc_registers();

	/* register IPU IRQ */
	ipu_driver_register_irq(ipu_priv);

#ifdef LCD_DEBUG
	/* turn on PWM early */
	gpio_as_output(GPIO_LCD_PWM);
	gpio_set_pin(GPIO_LCD_PWM);
	display_v_color_bar();
//	while(1);
#endif
	return 0;

failed:
	print_dbg();
	jz47xxfb_unmap_smem(cfb);
	jz47xxfb_free_fb_info(cfb);
	lcd_display_off();

	return err;
}
static int jz47xxfb_suspend(struct platform_device *pdev,pm_message_t state)
{
	if(jz_lcd_stat.backlight)
		lcd_close_backlight();
	if(jz_lcd_stat.panel)
		lcd_panel_display_off();
	if(jz_lcd_stat.controller)
		lcd_controller_disable();
	return 0;
}
static int jz47xxfb_resume(struct platform_device *pdev)
{
	lcd_controller_enable();		/* Enable LCD Controller */
	if(!jz_lcd_stat.panel)
		lcd_panel_display_on();
	return 0;
}
static int jz47xxfb_remove(struct platform_device *pdev)
{
	struct lcd_cfb_info *cfb = platform_get_drvdata(pdev);
#ifdef CONFIG_LEDS_CLASS
	led_classdev_unregister(&lcd_backlight_led);
#endif
	jz47xxfb_unmap_smem(cfb);
	jz47xxfb_free_fb_info(cfb);
	return 0;
}

static struct platform_driver jz_lcd_driver = {
	.probe 	= jz47xxfb_probe,
	.remove = jz47xxfb_remove,
	.suspend= jz47xxfb_suspend,
	.resume = jz47xxfb_resume,
	.driver = {
		   .name = DRIVER_NAME,
	},
};

static int __init jz47xxfb_init(void)
{
	return platform_driver_register(&jz_lcd_driver);
}

static void __exit jz47xxfb_cleanup(void)
{
	platform_driver_unregister(&jz_lcd_driver);
}

//module_init(jz47xxfb_init);
late_initcall(jz47xxfb_init);
module_exit(jz47xxfb_cleanup);
