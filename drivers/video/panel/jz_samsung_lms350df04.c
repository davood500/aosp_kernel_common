/*
 * kernel/drivers/video/panel/jz_lms350df04.c -- Ingenic LCD panel device
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

#include <linux/init.h>
#include <linux/platform_device.h>
#include <asm/jzsoc.h>

#include <linux/jz_lcd.h>

extern void lcd_register_panel(struct lcd_panel_info *p);

static struct lcd_board_info *lcd_board;
static struct lcd_soc_info *lcd_soc;

#define __spi_write_reg(reg, val)		\
	do {					\
		unsigned char no;	    	\
		unsigned int  value;		\
		unsigned char a=0;		\
		unsigned short b=0;		\
		a=reg & 0xff;			\
		b=val;                      \
		lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->SPEN);		\
		lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->SPCK);		\
		lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->SPDT);		\
		udelay(50);					\
		lcd_soc->soc_ops->gpio_clear_pin(lcd_board->board_pin->SPEN);		\
		udelay(50);                 \
		value=((a<<16)|(b&0xFFFF));	\
		for(no=0;no<24;no++)		\
		{						\
			lcd_soc->soc_ops->gpio_clear_pin(lcd_board->board_pin->SPCK);	\
			if((value&0x800000)==0x800000){		\
				lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->SPDT);}	\
			else{				      \
				lcd_soc->soc_ops->gpio_clear_pin(lcd_board->board_pin->SPDT); } \
			udelay(50);					\
			lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->SPCK);		\
			value=(value<<1);				\
			udelay(50);					\
		}							\
		lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->SPEN);			\
		udelay(50);						\
	} while (0)
#define __spi_read_reg(reg,val)			\
	do{					\
		unsigned char no;		\
		unsigned short value;			\
		value = reg;	                        \
		val = 0;				\
		lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->SPEN);			\
		lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->SPCK);           \
		lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->SPDT);			\
		udelay(50);				        \
		lcd_soc->soc_ops->gpio_clear_pin(SPEN);			\
		udelay(50);                     \
		for (no = 0; no < 24; no++ ) {				\
			lcd_soc->soc_ops->gpio_clear_pin(lcd_board->board_pin->lcd_board->board_pin->SPCK);		\
			if(no < 8)					\
			{						\
				if (value & 0x80) /* send data */	\
					lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->lcd_board->board_pin->SPDT);	\
				else					\
					lcd_soc->soc_ops->gpio_clear_pin(lcd_board->board_pin->lcd_board->board_pin->SPDT);		\
				udelay(50);				\
				lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->SPCK);			\
				value = (value << 1);			\
				udelay(50);				\
            }                           \
			else						\
			{						\
				udelay(100);				\
				lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->SPCK);			\
				udelay(50);				\
				val = (val << 1);			\
				val |= lcd_soc->soc_ops->gpio_get_pin(lcd_board->board_pin->lcd_board->board_pin->SPRS);		\
            }                                       \
		}							\
        lcd_soc->soc_ops->gpio_set_pin(SPEN);               \
		udelay(400);						\
	} while(0)

static void spi_write_reg(unsigned int reg, unsigned int val)
{
    __spi_write_reg(0x74, reg);
    __spi_write_reg(0x76, val);    
}

static void panel_reg_init(void)
{
    spi_write_reg(0x10, 0x0000);

    mdelay(2);
   
    /* reset */
    spi_write_reg(0x07, 0x0000);

    mdelay(15);
    
    spi_write_reg(0x11, 0x222f);
    spi_write_reg(0x12, 0x0c00); 
    spi_write_reg(0x13, 0x7fd9);
    spi_write_reg(0x76, 0x2213);
    spi_write_reg(0x74, 0x0001);
    spi_write_reg(0x76, 0x0000);
    spi_write_reg(0x10, 0x5604);

    mdelay(110);
    
    spi_write_reg(0x12, 0x0c63);

    mdelay(90);
    
#ifdef PANEL_REVERSE
    spi_write_reg(0x01, 0x0b3B);  // rev, line480
#else
    spi_write_reg(0x01, 0x083B);  // rev, line480
#endif

    spi_write_reg(0x02, 0x0300);  // frame invertion
    //spi_write_reg(0x03, 0xf040);  // vsync, hsync, pixel clck && de polarity
    spi_write_reg(0x03, 0xd040);  // vsync, hsync, pixel clck && de polarity
    spi_write_reg(0x08, 0x0002);  // bfw 4
    spi_write_reg(0x09, 0x000B);  // blw 11
    spi_write_reg(0x76, 0x2213);
    spi_write_reg(0x0B, 0x3340);
    spi_write_reg(0x0C, 0x0020);  // 24bit 0x0020->0x0000
    spi_write_reg(0x1C, 0x7770);
    spi_write_reg(0x76, 0x0000);
    spi_write_reg(0x0D, 0x0000);
    spi_write_reg(0x0E, 0x0500);
    spi_write_reg(0x14, 0x0000);
    spi_write_reg(0x15, 0x0803);
    spi_write_reg(0x16, 0x0000);

    spi_write_reg(0x30, 0x0003);
    spi_write_reg(0x31, 0x070f);
    spi_write_reg(0x32, 0x0D05);
    spi_write_reg(0x33, 0x0405);
    spi_write_reg(0x34, 0x090D);
    spi_write_reg(0x35, 0x0501);
    spi_write_reg(0x36, 0x0400);
    spi_write_reg(0x37, 0x0504);
    spi_write_reg(0x38, 0x0C09);
    spi_write_reg(0x39, 0x010C);

    mdelay(20);
    spi_write_reg(0x07, 0x0001);
    mdelay(40);
    spi_write_reg(0x07, 0x0101);                     
    mdelay(40);
    spi_write_reg(0x07, 0x0103);
}

static void panel_display_off(void)
{
#if defined CONFIG_JZ4750_AQUILA   
    __gpio_set_pin(LCD_SWITCH_PIN1);        
#endif

    spi_write_reg(0x0b, 0x3000);
    spi_write_reg(0x07, 0x0102);
    mdelay(40);
    spi_write_reg(0x07, 0x0000);
    mdelay(40);
    spi_write_reg(0x12, 0x0000);
    spi_write_reg(0x10, 0x0600);
    spi_write_reg(0x10, 0x0601);    
}

static void panel_display_on(void)
{
    lcd_soc->soc_ops->gpio_clear_pin(lcd_board->board_pin->LCD_RESET_PIN);
    udelay(30);
    lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->LCD_RESET_PIN);
    mdelay(20);
    panel_reg_init();
}

static void lms350df04_panel_display_init(void)
{
	lcd_soc->soc_ops->gpio_as_output(lcd_board->board_pin->SPEN); /* use SPEN */
	lcd_soc->soc_ops->gpio_as_output(lcd_board->board_pin->SPCK); /* use SPCK */
	lcd_soc->soc_ops->gpio_as_output(lcd_board->board_pin->SPDT); /* use SPDT */
	lcd_soc->soc_ops->gpio_as_input(lcd_board->board_pin->SPRS);  /* use SPRS */
	lcd_soc->soc_ops->gpio_as_output(lcd_board->board_pin->LCD_RESET_PIN);
}

static void lms350df04_panel_display_on(void)
{
	if(!lcd_board)
		printk("%s: lcd_board is null\n", __func__);
	lcd_board->board_ops->lcd_board_power_on();
//	pdata->ops->lcd_close_backlight();
//	printk("%s : \n", __func__);
	panel_display_on();
}

static void lms350df04_panel_display_off(void)
{
	panel_display_off();
	lcd_board->board_ops->lcd_board_power_off();
	lcd_soc->soc_ops->gpio_clear_pin(lcd_board->board_pin->LCD_POWERON);
}

static int lms350df04_panel_probe(struct lcd_soc_info *soc , struct lcd_board_info *board)
{
	/* check the parameters from lcd_driver */
	if (soc) {
		lcd_soc = soc;
	} else {
		printk("%s: struct lcd_soc_info is null\n", __func__);
		return -EINVAL;
	}
	if (board) {
		lcd_board = board;
	} else {
		printk("%s: struct lcd_board_info is null\n", __func__);
		return -EINVAL;
	}

	/* check if the pin definations in use are defined*/
	if(!(lcd_board->board_pin->LCD_RESET_PIN)) {
		printk(KERN_ERR JZ_LCD_PFX": lcd board pin defination error\n");
		return -1;
	}

	D("board_pin: LCD_POWERON=%d, LCD_RESET_PIN=%d.\n", lcd_board->board_pin->LCD_POWERON, lcd_board->board_pin->LCD_RESET_PIN);
//	__gpio_set_lcd_data_driving_strength(1);
//	__gpio_set_lcd_sync_driving_strength(1);
//	__gpio_set_lcd_clk_driving_strength(2);
	return 0;
}

static struct lcd_panel_ops_info lms350df04_panel_ops = {
	.panel_init		=	lms350df04_panel_display_init,
	.panel_display_on	=	lms350df04_panel_display_on,
	.panel_display_off	=	lms350df04_panel_display_off,
	.panel_probe		=	lms350df04_panel_probe,
};

static struct lcd_panel_attr_info lms350df04_panel_attr = {
	.bpp = 24,	/* */
	.vsp = 1,	/* VSYNC polarity: 0-rising,1-falling */
	.hsp = 1,	/* HSYNC polarity: 0-active high,1-active low */
	.dep = 0,	/* DE polarity:0-active high,1-active low */
	.pcp = 0,	/* PCP Pixel Clock polarity: 0-translation at rising edge, 1-translation at falling edge  */
	.slcd_cfg = 0,	/* Smart lcd configurations */
	.phys_width = 0,/* width of screen physical size, in mm */
	.phys_height = 0,/* height of screen physical size, in mm */
	.w = 320,	/* Panel Width(in pixel) */
	.h = 480,	/* Panel Height(in line) */
	.fclk = 60,	/* frame clk */
	.hsw = 4,	/* hsync width, in pclk */
	.vsw = 2,	/* vsync width, in line count */
	.elw = 5,	/* end of line, in pclk */
	.blw = 7,	/* begin of line, in pclk */
	.efw = 5,	/* end of frame, in line count */
	.bfw = 2,	/* begin of frame, in line count */
	.fg0_bpp = 32,
	.fg1_bpp = 32,
};

static struct lcd_panel_info lms350df04_lcd_panel = {
	.name = "lms350df04",
	.panel_probable = 0,	// not probable
	.panel_attr = &lms350df04_panel_attr,
	.panel_ops = &lms350df04_panel_ops,
};

static int __init lms350df04_init(void)
{
	// register the panel with lcd drivers
	lcd_register_panel(&lms350df04_lcd_panel);
	return 0;
}

module_init(lms350df04_init);
