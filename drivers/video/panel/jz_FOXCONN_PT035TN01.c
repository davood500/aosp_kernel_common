/*
 * kernel/drivers/video/panel/jz_FOXCONN_PT035TN01.c -- Ingenic LCD panel device
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

#if defined(CONFIG_ANDROID_LCD_FOXCONN_PT035TN01)
#define MODE 0xcd 		/* 24bit parellel RGB */
#else /*CONFIG_ANDROID_LCD_INNOLUX_PT035TN01_SERIAL */
#define MODE 0xc9		/* 8bit serial RGB */
#endif

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
		lcd_soc->soc_ops->gpio_clear_pin(lcd_board->board_pin->SPDT);		\
		udelay(25);					\
		lcd_soc->soc_ops->gpio_clear_pin(lcd_board->board_pin->SPEN);		\
		udelay(25);                 \
		value=((a<<8)|(b&0xFF));	\
		for(no=0;no<16;no++)		\
		{						\
			lcd_soc->soc_ops->gpio_clear_pin(lcd_board->board_pin->SPCK);	\
			if((value&0x8000)==0x8000){		\
				lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->SPDT);}	\
			else{				      \
				lcd_soc->soc_ops->gpio_clear_pin(lcd_board->board_pin->SPDT); } \
			udelay(25);					\
			lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->SPCK);		\
			value=(value<<1);				\
			udelay(25);					\
		}							\
		lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->SPEN);			\
		udelay(100);						\
	} while (0)

static void spi_write_reg(reg, val)
{

	__spi_write_reg((reg<<2|2), val);
	udelay(100);
}

static void FOXCONN_PT035TN01_panel_display_init(void)
{

	lcd_soc->soc_ops->gpio_clear_pin(lcd_board->board_pin->LCD_RESET_PIN);
	mdelay(150);
	lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->LCD_RESET_PIN);
}
static void panel_display_on (void)
{

	udelay(50);				
	lcd_soc->soc_ops->gpio_clear_pin(lcd_board->board_pin->LCD_RESET_PIN);
	mdelay(150);
	lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->LCD_RESET_PIN);
	mdelay(10);
	spi_write_reg(0x00, 0x03);
	spi_write_reg(0x01, 0x40); 
	spi_write_reg(0x02, 0x11); 
	spi_write_reg(0x03, MODE); /* mode */ 
	spi_write_reg(0x04, 0x32); 
	spi_write_reg(0x05, 0x0e); 
	spi_write_reg(0x07, 0x03); 
	spi_write_reg(0x08, 0x08); 
	spi_write_reg(0x09, 0x32); 
	spi_write_reg(0x0A, 0x88); 
	spi_write_reg(0x0B, 0xc6); 
	spi_write_reg(0x0C, 0x20); 
	spi_write_reg(0x0D, 0x20); 
	//reg 0x0a is control the display direction:DB0->horizontal level DB1->vertical level
}

static void panel_display_off(void)
{

	spi_write_reg(0x00, 0x03);
}

static void FOXCONN_PT035TN01_panel_display_on(void)
{

	if(!lcd_board)
		printk("%s: lcd_board is null\n", __func__);
	lcd_board->board_ops->lcd_board_power_on();
	panel_display_on();
}

static void FOXCONN_PT035TN01_panel_display_off(void)
{

	panel_display_off();
	lcd_board->board_ops->lcd_board_power_off();
	lcd_soc->soc_ops->gpio_clear_pin(lcd_board->board_pin->LCD_STBY_PIN);
	lcd_soc->soc_ops->gpio_clear_pin(lcd_board->board_pin->LCD_POWERON);
}

static int FOXCONN_PT035TN01_panel_probe(struct lcd_soc_info *soc , struct lcd_board_info *board)
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
	if(!(lcd_board->board_pin->LCD_RESET_PIN && lcd_board->board_pin->SPCK
	     && lcd_board->board_pin->SPEN && lcd_board->board_pin->SPDT)) {
		printk(KERN_ERR JZ_LCD_PFX": lcd board pin defination error\n");
		return -1;
	}

	   D("board_pin: SPCK=%d, SPEN=%d, SPDT=%d, LCD_RESET_PIN=%d.\n", 
	     lcd_board->board_pin->SPCK, lcd_board->board_pin->SPEN, 
	     lcd_board->board_pin->SPDT, lcd_board->board_pin->LCD_RESET_PIN);

	return 0;
}

static struct lcd_panel_ops_info FOXCONN_PT035TN01_panel_ops = {
	.panel_init		=	FOXCONN_PT035TN01_panel_display_init,
	.panel_display_on	=	FOXCONN_PT035TN01_panel_display_on,
	.panel_display_off	=	FOXCONN_PT035TN01_panel_display_off,
	.panel_probe		=	FOXCONN_PT035TN01_panel_probe,
};

static struct lcd_panel_attr_info FOXCONN_PT035TN01_panel_attr = {
	.bpp = 24,	/* */
	.vsp = 1,	/* VSYNC polarity: 0-rising,1-falling */
	.hsp = 1,	/* HSYNC polarity: 0-active high,1-active low */
	.dep = 0,	/* DE polarity:0-active high,1-active low */
	.pcp = 0,	/* PCP Pixel Clock polarity: 0-translation at rising edge, 1-translation at falling edge  */
	.slcd_cfg = 0,	/* Smart lcd configurations */
	.phys_width = 70,/* width of screen physical size, in mm */
	.phys_height = 53,/* height of screen physical size, in mm */
	.ctrl = (1<<11),/* Output FIFO underrun interrupt mask*/
	.w = 320,	/* Panel Width(in pixel) */
	.h = 240,	/* Panel Height(in line) */
	.fclk = 56,	/* frame clk */
	.hsw = 1,	/* hsync width, in pclk */
	.vsw = 1,	/* vsync width, in line count */
	.elw = 10,	/* end of line, in pclk */
	.blw = 50,	/* begin of line, in pclk */
	.efw = 10,	/* end of frame, in line count */
	.bfw = 13,	/* begin of frame, in line count */
	.fg0_bpp = 32,
	.fg1_bpp = 32,
};

static struct lcd_panel_info FOXCONN_PT035TN01_lcd_panel = {
	.name = "FOXCONN_PT035TN01",
	.panel_probable = 0,	// not probable
	.panel_attr = &FOXCONN_PT035TN01_panel_attr,
	.panel_ops = &FOXCONN_PT035TN01_panel_ops,
};

static int __init FOXCONN_PT035TN01_init(void)
{

	// register the panel with lcd drivers
	lcd_register_panel(&FOXCONN_PT035TN01_lcd_panel);
	return 0;
}

module_init(FOXCONN_PT035TN01_init);
