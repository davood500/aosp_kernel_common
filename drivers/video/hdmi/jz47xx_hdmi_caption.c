/*
 * kernel/drivers/video/jz4770_android_hdmi_caption.c -- Ingenic Jz4770 hdmi caption frame buffer device
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
 * This hdmi caption driver can output caption on the TV.
 * It only has one framebuffer(3M) and shares the compress buffer with
 * LCD driver.
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

#include "../console/fbcon.h"
#include "../jz47xx_android_lcd.h"
#include "../jz_ipu.h"
//#include "jz47xx_hdmi.h"


#if defined(CONFIG_SOC_JZ4770)
#include <linux/time.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <asm/time.h>
#include "../jz47xx_aosd.h"
#endif

#define DMA_DESC_NUM 		9
#define AOSD_ALIGN (64*4)		/* AOSD 64words aligned at least */

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

struct jz47xxlcd_fg_t default_hdmi_caption_position[3] = {
       {32, 0, 280, 800, 200},  /* HDMI_480p */
       {32, 0, 500, 1280, 200}, /* HDMI_720p */
       {32, 0, 800, 1920, 200}, /* HDMI_1080p */
};

extern struct jz47xxlcd_info jz47xx_lcd_panel;
static struct jz47xxlcd_info *jz47xx_lcd_info = &jz47xx_lcd_panel;
extern  void jz47xxfb_deep_set_mode(struct jz47xxlcd_info * lcd_info );

#ifdef CONFIG_JZ47xx_AOSDC
static struct jz47xx_aosd_info *aosd_info;
extern unsigned char *buf_comp, *buf_comp1; /* buffer for compress output */
#endif


static struct lcd_cfb_info *jz47xxfb_hdmi_fb_info;
struct jz47xxlcd_info jz47xx_hdmi_panel;
static struct jz47xxlcd_info *jz47xx_hdmi_caption_info = &jz47xx_hdmi_panel;

static struct jz47xx_lcd_dma_desc *dma_desc_base;
static struct jz47xx_lcd_dma_desc  *dma0_desc0;
static unsigned char *hdmi_fb_palette;
static unsigned char *hdmi_fb_frame;
static int buff_size;
static int dy;

extern void flush_dcache_with_prefetch_allocate(void);

static void init_hdmi_config(struct jz47xxlcd_info *info)
{   
    info->panel.w   = 1280;
    info->panel.h   = 720;
  
    info->osd.fg0.bpp = 32;
    info->osd.fg0.x = 0;
    info->osd.fg0.y = 400;
    info->osd.fg0.w = 800;
    info->osd.fg0.h = 80;
}

static int jz_hdmi_fb_open_cnt = 0;
static int jz47xxfb_hdmi_fb_open(struct fb_info *info, int user)
{
	int w;
	int reg_val;
    ++jz_hdmi_fb_open_cnt;
    printk("jz47xxfb_hdmi_fb_open() jz_hdmi_fb_open_cnt=%d\n", jz_hdmi_fb_open_cnt);

	reg_val = REG_LCD_DAH;
	w = (reg_val&0x00000FFF)-(reg_val >> 16);
	if (w >= 700 && w <= 900) {
		jz47xx_hdmi_caption_info->osd.fg0 = default_hdmi_caption_position[0];
		jz47xx_hdmi_caption_info->panel.w = 800;
		jz47xx_hdmi_caption_info->panel.h = 480;
	} else if (w >= 1180 && w <= 1380) {
		jz47xx_hdmi_caption_info->osd.fg0 = default_hdmi_caption_position[1];
		jz47xx_hdmi_caption_info->panel.w = 1280;
		jz47xx_hdmi_caption_info->panel.h = 720;
	} else if (w >= 1820 && w <= 2020) {
		jz47xx_hdmi_caption_info->osd.fg0 = default_hdmi_caption_position[2];
		jz47xx_hdmi_caption_info->panel.w = 1920;
		jz47xx_hdmi_caption_info->panel.h = 1080;
	} else {
		jz47xx_hdmi_caption_info->osd.fg0 = default_hdmi_caption_position[1];
		jz47xx_hdmi_caption_info->panel.w = 1280;
		jz47xx_hdmi_caption_info->panel.h = 720;
	}
	
    return 0;
}

static int hdmi_fb_change_size(struct jz47xxlcd_info *info)
{
    int size0,fg0_line_size, fg0_frm_size;
	int j, count = 100000;
	
	/* Foreground 0  */
	if (info->osd.fg0.x + info->osd.fg0.w > info->panel.w)
		info->osd.fg0.w = info->panel.w - info->osd.fg0.x;
	if (info->osd.fg0.y + info->osd.fg0.h > info->panel.h)
		info->osd.fg0.h = info->panel.h - info->osd.fg0.y;

	fg0_line_size = info->osd.fg0.w * info->osd.fg0.bpp / 8;
	fg0_line_size = ((fg0_line_size + 3) >> 2) << 2; /* word aligned */
	fg0_frm_size = fg0_line_size * info->osd.fg0.h;

	dma0_desc0 = dma_desc_base + 1;
	size0 = info->osd.fg0.h << 16 | info->osd.fg0.w;
	
	if (REG_LCD_SIZE0 == size0) {
		printk("FG0: same size\n");
		return 0;
	} else {
		if (!(REG_LCD_CTRL & LCD_CTRL_DIS)
		    && (REG_LCD_CTRL & LCD_CTRL_ENA)
		    && (REG_LCD_OSDC & LCD_OSDC_F0EN)) {

			REG_LCD_OSDCTRL |= LCD_OSDCTRL_CHANGES;
			j = count;
			while(!(REG_LCD_OSDS & LCD_OSDS_READY) && j--);	
	
			REG_LCD_SIZE0 = size0;
			dma0_desc0->desc_size = size0;
			dma0_desc0->cmd = LCD_CMD_EOFINT | fg0_frm_size/4;
			dma0_desc0->offsize = 0;
			dma0_desc0->page_width = 0;
            dma_cache_wback((unsigned int)(dma_desc_base), (DMA_DESC_NUM)*sizeof(struct jz47xx_lcd_dma_desc));

			j = count;
			msleep(40);
			while((REG_LCD_OSDCTRL & LCD_OSDCTRL_CHANGES) && j--);
			if(j == 0) {
				printk("Error FG0 size: Wait change fail.\n");
				return -EFAULT;
			}
		} else {
			REG_LCD_SIZE0 = size0;
		    dma0_desc0->desc_size = size0;
			dma0_desc0->cmd = LCD_CMD_EOFINT | fg0_frm_size/4;
			dma0_desc0->offsize = 0;
			dma0_desc0->page_width = 0;
            dma_cache_wback((unsigned int)(dma_desc_base), (DMA_DESC_NUM)*sizeof(struct jz47xx_lcd_dma_desc));
		}
	} 
   return 0;
}

static int hdmi_fb_change_position(struct jz47xxlcd_info *info)
{
	int pos;
	int j, count = 100000;   

	/* Foreground 0  */
	if (info->osd.fg0.x + info->osd.fg0.w > info->panel.w)
		info->osd.fg0.x = info->panel.w - info->osd.fg0.w;
	if (info->osd.fg0.y + info->osd.fg0.h > info->panel.h)
		info->osd.fg0.y = info->panel.h - info->osd.fg0.h;

	if (info->osd.fg0.x >= info->panel.w)
		info->osd.fg0.x = info->panel.w - 1;
	if (info->osd.fg0.y >= info->panel.h)
		info->osd.fg0.y = info->panel.h - 1;

	pos = (info->osd.fg0.y << 16) | (info->osd.fg0.x);
	if (REG_LCD_XYP0 == pos) {
		printk("FG0: same position\n");
		return 0;
	} else {
		if (!(REG_LCD_CTRL & LCD_CTRL_DIS)
		    && (REG_LCD_CTRL & LCD_CTRL_ENA)
		    && (REG_LCD_OSDC & LCD_OSDC_F0EN)) {
	
			REG_LCD_OSDCTRL |= LCD_OSDCTRL_CHANGES;
			j = count;
			while(!(REG_LCD_OSDS & LCD_OSDS_READY) && j--);

			REG_LCD_XYP0 = pos;

			j = count;
			msleep(40);
			while((REG_LCD_OSDCTRL & LCD_OSDCTRL_CHANGES) && j--);
			if(j == 0) {
				printk("Error FG0 Position: Wait change fail.\n");
				return -EFAULT;				
			}
		} else 
			REG_LCD_XYP0 = pos;
	}	

   return 0;
}

/* change to hdmi caption framebuffer */
static void hdmi_fb_descriptor_init(struct jz47xxlcd_info *info)
{
    int size0;
    int fg0_line_size, fg0_frm_size;
 
    fg0_line_size = info->osd.fg0.w * info->osd.fg0.bpp / 8;
    fg0_line_size = ((fg0_line_size + 3) >> 2) << 2;
    fg0_frm_size = fg0_line_size * info->osd.fg0.h;

    size0 = info->osd.fg0.h << 16 | info->osd.fg0.w;

    dma0_desc0 = dma_desc_base + 1;
	
    dma0_desc0->next_desc = (unsigned int)virt_to_phys(dma0_desc0);

    dma0_desc0->databuf = virt_to_phys((void *)hdmi_fb_frame);
    dma0_desc0->frame_id = (unsigned int)0x0000da00;
    dma0_desc0->cmd = LCD_CMD_EOFINT | fg0_frm_size/4;
	
    dma0_desc0->offsize =0;
    dma0_desc0->page_width = 0;
    dma0_desc0->desc_size = size0;
    
    dma_cache_wback((unsigned int)(dma_desc_base), (DMA_DESC_NUM)*sizeof(struct jz47xx_lcd_dma_desc));
	REG_LCD_DA0 = virt_to_phys(dma0_desc0);
	REG_LCD_SIZE0 = size0;
}

static int FG1_WITH_IPU_RUNNING;
static void hdmi_disable_lcdc(void)
{
	/* configurate sequence:
	 * 1. stop F1.
	 * 2. stop IPU. 
	 * 3. disable lcdc. 
    */
    int FG1EN;
    int FG1_USE_IPU;
	int j, count = 100000;

    FG1EN = REG_LCD_OSDC & LCD_OSDC_F1EN;
    FG1_USE_IPU = REG_LCD_OSDCTRL & LCD_OSDCTRL_IPU;
    FG1_WITH_IPU_RUNNING = FG1EN && FG1_USE_IPU;

    if( FG1_WITH_IPU_RUNNING )  {       		
        REG_LCD_OSDCTRL |= LCD_OSDCTRL_CHANGES;
        /* wait change ready */
		j = count;
		while(!(REG_LCD_OSDS & LCD_OSDS_READY) && j--);

		 __lcd_disable_f1();

		/* check change */
		j = count;
		//msleep(40);
		while((REG_LCD_OSDCTRL & LCD_OSDCTRL_CHANGES) && j--);
		if(j == 0) {
			printk("Error FG1 Disable: Wait change fail.\n");
		}

        REG32(IPU_V_BASE + REG_CTRL) |= IPU_STOP;
    }

	if(!(REG_LCD_CTRL & LCD_CTRL_ENA)){
        printk("lcdc already disable\n");
        return;
    }
    __lcd_clr_ena();
}

static void hdmi_enable_lcdc(void)
{
	/* configurate sequence:
	 * 1. resume IPU.
	 * 2. enable F1. 
	 * 3. enable lcdc. 
    */
	int j, count = 100000;
    
    if ( FG1_WITH_IPU_RUNNING )  {
        REG32(IPU_V_BASE + REG_STATUS) &= ~OUT_END;
        REG32(IPU_V_BASE + REG_CTRL) |= IPU_RUN;
        
	    REG_LCD_OSDCTRL |= LCD_OSDCTRL_CHANGES;
        /* wait change ready */
		j = count;
		while(!(REG_LCD_OSDS & LCD_OSDS_READY) && j--);
		__lcd_enable_f1();
		/* check change */
		j = count;
		//msleep(40); 
		while((REG_LCD_OSDCTRL & LCD_OSDCTRL_CHANGES) && j--);
		if(j == 0) {
			printk("Error FG1 Disable: Wait change fail.\n");
		}
    }

	if(REG_LCD_CTRL & LCD_CTRL_ENA){
        printk("lcdc already enable\n");
        return;
    }
    __lcd_set_ena();/* enable lcdc */
}

int hdmi_change_fb(void)
{
    /* 
	 * configurate sequence:
	 * 1. stop ipu and disable lcdc.
	 * 2. change fg0 size. 
	 * 3. change fg0 position.
	 * 4. change frame descriptor.
	 * 5. resume ipu and enable lcdc.
	*/
    REG_LCD_OSDC |= (1<<2);/* enable alpha */
    REG_LCD_OSDC &= ~(1<<1);/* use global alpha */
    REG_LCD_ALPHA = 0x80;
    
    hdmi_disable_lcdc();
    hdmi_fb_change_size(&jz47xx_hdmi_panel);
    hdmi_fb_change_position(&jz47xx_hdmi_panel);
	hdmi_fb_descriptor_init(jz47xx_hdmi_caption_info);
    hdmi_enable_lcdc();

    return 0;
}

/* change AOSD buffer */
static int jz47xxfb_update_caption(void)
{
	struct fb_info *fb = &jz47xxfb_hdmi_fb_info->fb0;
	struct jz47xxlcd_info *lcd_info = jz47xx_hdmi_caption_info;
	int fg0_line_size, fg0_frm_size;

	if (!(REG_LCD_CTRL & LCD_CTRL_ENA)) {
		return 0;
	}

	if (!(REG_LCD_OSDC & LCD_OSDC_F0EN)) {
		return 0;
	}

	if (!fb) {
		return -EINVAL;
	}

	dma0_desc0 = dma_desc_base + 1;

	fg0_line_size = ( lcd_info->osd.fg0.w * (lcd_info->osd.fg0.bpp) / 8);
	fg0_line_size = ((fg0_line_size + 3) >> 2) << 2;
	fg0_frm_size = fg0_line_size * lcd_info->osd.fg0.h;

	if (lcd_info->osd.fg0.bpp == 16) {
		fb->fix.line_length	= lcd_info->osd.fg0.w * 2;
	} else if (lcd_info->osd.fg0.bpp == 24 || lcd_info->osd.fg0.bpp == 32) {
		fb->fix.line_length	= lcd_info->osd.fg0.w * 4;
	}

#ifdef CONFIG_JZ47xx_AOSDC
	if (/*lcd_should_use_compress_decompress_mode()*/ 1 ) {
		flush_dcache_with_prefetch_allocate();
		
		aosd_info->bpp = lcd_info->osd.fg0.bpp;
		aosd_info->width = lcd_info->osd.fg0.w;
		aosd_info->height = lcd_info->osd.fg0.h;
		aosd_info->aligned_64 = 0;						/* meanless */
		aosd_info->src_stride = fb->fix.line_length;    /* in bytes */
		aosd_info->dst_stride = (aosd_info->src_stride + (AOSD_ALIGN))&(~(AOSD_ALIGN-1)); // in bytes. 32 or 64 words aligned		
		aosd_info->addr0 = (unsigned int)virt_to_phys((void *)hdmi_fb_frame);/* just one framebuffer(3M) */

		if (dy) {
			aosd_info->waddr = (unsigned int)virt_to_phys((void *)buf_comp);/* dy is 1 change to buff_comp buffer */
		} else {
			aosd_info->waddr = (unsigned int)virt_to_phys((void *)buf_comp1);/* dy is 0 change to buff_comp1 buffer */
		}

		jz47xx_compress_set_mode(aosd_info);
		jz47xx_start_compress();

	   	dma0_desc0->cmd = LCD_CMD_EOFINT | LCD_CMD_UNCOMP_EN | (aosd_info->height & LCD_CMD_LEN_MASK);
		//dma0_desc0->cmd |= LCD_CMD_UNCOMPRESS_WITHOUT_ALPHA;
		// don't need to set dma page width in decompress mode.
		//dma0_desc0->page_width = 0;
		// dma offsize in decompress mode indicates how many words of a line in source buffer
		dma0_desc0->offsize = (aosd_info->dst_stride)>>2; /* in words */
		
		if (dy) {
			dma0_desc0->databuf = (unsigned int)virt_to_phys((void *)buf_comp);
		} else {
			dma0_desc0->databuf = (unsigned int)virt_to_phys((void *)buf_comp1);
		}
		dma_cache_wback((unsigned int)(dma0_desc0), sizeof(struct jz47xx_lcd_dma_desc));
	} 
	else {
		flush_dcache_with_prefetch_allocate();

		dma0_desc0->cmd = LCD_CMD_EOFINT | fg0_frm_size/4;
		dma0_desc0->offsize = 0;
	   	dma0_desc0->databuf = (unsigned int)virt_to_phys((void *)hdmi_fb_frame);
		dma_cache_wback((unsigned int)(dma0_desc0), sizeof(struct jz47xx_lcd_dma_desc));
	}
#else			
	flush_dcache_with_prefetch_allocate();

	dma0_desc0->databuf = (unsigned int)virt_to_phys((void *)hdmi_fb_frame);
	dma_cache_wback((unsigned int)(dma0_desc0), sizeof(struct jz47xx_lcd_dma_desc));
#endif
	
	return 0;
}

static int jz47xxfb_hdmi_fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
    struct jz47xxlcd_fg_t fg0;
	void __user *argp = (void __user *)arg;

    switch(cmd){
	case FBIO_GET_POSITION:
		fg0.bpp = jz47xx_hdmi_caption_info->osd.fg0.bpp;
		fg0.x = jz47xx_hdmi_caption_info->osd.fg0.x;
		fg0.y = jz47xx_hdmi_caption_info->osd.fg0.y;
		fg0.w = jz47xx_hdmi_caption_info->osd.fg0.w;
		fg0.h = jz47xx_hdmi_caption_info->osd.fg0.h;
		if(copy_to_user(argp, &fg0, sizeof(struct jz47xxlcd_fg_t)))
			return -EFAULT;
		break;	
	case FBIO_CHANGE_POSITION:
		if (copy_from_user(&fg0, argp, sizeof(struct jz47xxlcd_fg_t)))
			return -EFAULT;
		if (fg0.bpp == 24 || fg0.bpp == 32 ){
			jz47xx_hdmi_caption_info->osd.fg0.bpp = 32;
		}
		jz47xx_hdmi_caption_info->osd.fg0.x = fg0.x;
		jz47xx_hdmi_caption_info->osd.fg0.y = fg0.y;
		jz47xx_hdmi_caption_info->osd.fg0.w = fg0.w;
		jz47xx_hdmi_caption_info->osd.fg0.h = fg0.h;
		break;  	            		           	
	case FBIO_CAPTION_ON:
		hdmi_change_fb();
		break;
	case FBIO_CAPTION_OFF:
		jz47xxfb_deep_set_mode( jz47xx_lcd_info );
		break;
	case FBIO_GET_BUFF_SIZE:
		put_user(buff_size, (int *) argp);
		break;
	case FBIO_UPDATE_CAPTION:
		if (copy_from_user(&dy, argp, sizeof(int)))
			return -EFAULT;
		jz47xxfb_update_caption();
		break;
	case FBIO_DISABLE_LCDC:
		hdmi_disable_lcdc();
		break;
	case FBIO_ENABLE_LCDC:
		hdmi_enable_lcdc();
		break;		
	case FBIO_ALPHA_ON:
		/*lcdc_enable_alpha();*/
		jz47xx_hdmi_caption_info->osd.osd_cfg |= LCD_OSDC_ALPHAEN;
		__lcd_enable_alpha();
		break;
	case FBIO_ALPHA_OFF:
		/*lcdc_disable_alpha();*/
		jz47xx_hdmi_caption_info->osd.osd_cfg &= ~LCD_OSDC_ALPHAEN;
		__lcd_disable_alpha();
		break;
	case FBIO_SET_ALPHA_VAL:
		jz47xx_hdmi_caption_info->osd.alpha = arg;
		/*lcdc_set_alpha(arg);*/
		REG_LCD_ALPHA = jz47xx_hdmi_caption_info->osd.alpha;
		break;
	default:
		break;
    }
    return 0;
}

static int jz47xxfb_hdmi_fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
    unsigned long start;
    unsigned long off;
    u32 len;

    off = vma->vm_pgoff << PAGE_SHIFT;

    start = info->fix.smem_start;
    len = PAGE_ALIGN((start & ~PAGE_MASK) + info->fix.smem_len);
    start &= PAGE_MASK;

    if ((vma->vm_end - vma->vm_start + off) > len)
        return -EINVAL;
    off += start;

    vma->vm_pgoff = off >> PAGE_SHIFT;
    vma->vm_flags |= VM_IO;

    pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
    pgprot_val(vma->vm_page_prot) |= _CACHE_CACHABLE_NONCOHERENT;

    if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
                vma->vm_end - vma->vm_start,
                vma->vm_page_prot)) {
        return -EAGAIN;
    }
    return 0;
}

static struct fb_ops jz47xxfb_hdmi_fb_ops = {
    .owner          = THIS_MODULE,
    .fb_open        = jz47xxfb_hdmi_fb_open,
    .fb_mmap        = jz47xxfb_hdmi_fb_mmap,
	.fb_pan_display	= NULL,
    .fb_ioctl       = jz47xxfb_hdmi_fb_ioctl,
};

static struct lcd_cfb_info *alloc_hdmi_fb_info(void)
{
    struct lcd_cfb_info *cfb = NULL;

    cfb = kmalloc(sizeof(struct lcd_cfb_info) + sizeof(u32) * 16, GFP_KERNEL);
    if(cfb == NULL)
        return NULL;

    jz47xxfb_hdmi_fb_info = cfb;
    memset(cfb, 0, sizeof(struct lcd_cfb_info) );

    cfb->currcon = -1;

    strcpy(cfb->fb0.fix.id, "jzlcd-fg0");
    cfb->fb0.fix.type   = FB_TYPE_PACKED_PIXELS;
    cfb->fb0.fix.type_aux   = 0;
    cfb->fb0.fix.xpanstep   = 1;
    cfb->fb0.fix.ypanstep   = 1;
    cfb->fb0.fix.ywrapstep  = 0;
    cfb->fb0.fix.accel  = FB_ACCEL_NONE;

    cfb->fb0.var.nonstd = 0;
    cfb->fb0.var.activate   = FB_ACTIVATE_NOW;
    cfb->fb0.var.height = -1;
    cfb->fb0.var.width  = -1;
    cfb->fb0.var.accel_flags    = FB_ACCELF_TEXT;

    cfb->fb0.fbops      = &jz47xxfb_hdmi_fb_ops;
    cfb->fb0.flags      = FBINFO_FLAG_DEFAULT;

    cfb->fb0.pseudo_palette = (void *)(cfb + 1);
    
    switch (jz47xx_hdmi_caption_info->osd.fg0.bpp) {
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

static void free_hdmi_fb_info(void)
{
    if (jz47xxfb_hdmi_fb_info) {
        fb_alloc_cmap(&jz47xxfb_hdmi_fb_info->fb0.cmap, 0, 0);
        kfree(jz47xxfb_hdmi_fb_info);
    }
}

static unsigned int page_shift;
static int alloc_hdmi_fb_map_smem(struct lcd_cfb_info *cfb){
    unsigned long page;   
    unsigned char *fb_palette, *fb_frame;	
	unsigned int needroom;
    
	needroom = 3145728;/* 3M */
	buff_size = needroom;
    printk("jz47xxfb_hdmi_fb_map_smem FB_Size=%d\n",needroom);

    for (page_shift = 0; page_shift < 12; page_shift++)
        if ((PAGE_SIZE << page_shift) >= needroom)
            break;
    fb_palette = (unsigned char *)__get_free_pages(GFP_KERNEL, 0);
    fb_frame = (unsigned char *)__get_free_pages(GFP_KERNEL, page_shift);
    printk("FrameBuffer page_shift= %d \n",page_shift);

    if ((!fb_palette) || (!fb_frame))
        return -ENOMEM;
    memset((void *)fb_palette, 0, PAGE_SIZE);

    memset((void *)fb_frame, 0xFF, PAGE_SIZE << page_shift);   
    hdmi_fb_palette = fb_palette;
    dma_desc_base = (struct jz47xx_lcd_dma_desc *)((void*)hdmi_fb_palette + ((PALETTE_SIZE+3)/4)*4);

    page = (unsigned long)hdmi_fb_palette;
    SetPageReserved(virt_to_page((void*)page));

    for(page = (unsigned long)fb_frame;
        page < PAGE_ALIGN((unsigned long)fb_frame + (PAGE_SIZE<<page_shift));
        page += PAGE_SIZE) {
        SetPageReserved(virt_to_page((void*)page));
    }

    hdmi_fb_frame = fb_frame;
    cfb->fb0.fix.smem_start = virt_to_phys((void *)hdmi_fb_frame);
    cfb->fb0.fix.smem_len = needroom;
    cfb->fb0.screen_base =
        (unsigned char *)(((unsigned int)hdmi_fb_frame&0x1fffffff) | 0xa0000000);
    if (!cfb->fb0.screen_base) {
        printk("jz47xxfb0, %s: unable to map screen memory\n", cfb->fb0.fix.id);
        return -ENOMEM;
    }
    return 0;
}

static int free_hdmi_fb_map_smem(void)
{
    struct page * map = NULL;
    unsigned char *tmp;

    if (hdmi_fb_palette) {
        map = virt_to_page(hdmi_fb_palette);
        clear_bit(PG_reserved, &map->flags);
        free_pages((int)hdmi_fb_palette, 0);
    }

    if (hdmi_fb_frame) {
        for(tmp=(unsigned char *)hdmi_fb_frame;
            tmp < hdmi_fb_frame + (PAGE_SIZE << page_shift);
            tmp += PAGE_SIZE) {
            map = virt_to_page(tmp);
            clear_bit(PG_reserved, &map->flags);
        }
        free_pages((int)hdmi_fb_frame, page_shift);
    }

    return 0;
}

static int __init init_hdmi_caption_fb(void)
{
    int err = -1;
    struct lcd_cfb_info *cfb = NULL;

    init_hdmi_config(jz47xx_hdmi_caption_info);

    cfb = alloc_hdmi_fb_info();
    if(cfb == NULL){
        printk("alloc jz47xxfb_hdmi_caption_info failed.\n");
        goto failed;
    }

    err = alloc_hdmi_fb_map_smem(cfb);
    if(err < 0){
        printk("alloc_hdmi_fb_map_smem failed.\n");
        goto failed;
    }
    
    spin_lock_init(&cfb->update_lock);
    init_waitqueue_head(&cfb->frame_wq);
    cfb->frame_requested = cfb->frame_done = 0;
   
    err = register_framebuffer(&cfb->fb0);
    if(err < 0){
        dprintk("init_hdmi_caption_fb(): register framebuffer err.\n");
        goto failed;
    }
    printk("fb%d: %s frame buffer device, using %dK of video memory\n",
            cfb->fb0.node, cfb->fb0.fix.id, cfb->fb0.fix.smem_len>>10);

#ifdef CONFIG_JZ47xx_AOSDC
	aosd_info = kmalloc(sizeof(struct jz47xx_aosd_info) , GFP_KERNEL);
	if (!aosd_info) {
		printk("malloc aosd_info failed\n");
		return -ENOMEM;
	}
	memset(aosd_info, 0, sizeof(struct jz47xx_aosd_info));

	jz47xx_compress_init();
#endif

    return 0; 
failed:
    print_dbg();
    free_hdmi_fb_map_smem();
    free_hdmi_fb_info();
    return err;
}

static void __exit remove_hdmi_caption_fb(void)
{
	struct lcd_cfb_info *cfb = jz47xxfb_hdmi_fb_info;
	if (cfb){
		unregister_framebuffer(&cfb->fb0);
		free_hdmi_fb_map_smem();
		free_hdmi_fb_info();
	}
}

late_initcall(init_hdmi_caption_fb);
module_exit(remove_hdmi_caption_fb);
