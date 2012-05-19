/*
 * kernel/drivers/video/panel/jz_LS032T8HX01.c -- Ingenic LCD panel device
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
/*
static unsigned int SPEN = 0;
static unsigned int SPCK = 0;
static unsigned int SPDT = 0;
static unsigned int SPRS = 0;
static unsigned int LCD_RESET_PIN = 0;
static unsigned int LCD_POWERON = 0;
*/
static void LS032T8HX01_panel_display_init(void)
{
	//lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->LCD_VCC_EN);
	//nothing to do
}

static void spi_write_reg(unsigned char reg, unsigned char val)
{
	unsigned char no;
	unsigned int  value;
	
	__gpio_as_output(lcd_board->board_pin->SPEN); /* use SPDT */	
	__gpio_as_output(lcd_board->board_pin->SPCK); /* use SPCK */	
	__gpio_as_output(lcd_board->board_pin->SPDT); /* use SPDT */	

	__gpio_set_pin(lcd_board->board_pin->SPEN);		
	__gpio_set_pin(lcd_board->board_pin->SPCK);		
	udelay(500);		           
	__gpio_clear_pin(lcd_board->board_pin->SPEN);		
	udelay(50);                 
	value=(((reg&0x7f)<<16)|(val|0x100));	
	
	
	for(no=0;no<32;no++)		
	{
		__gpio_clear_pin(lcd_board->board_pin->SPCK);		
		
		if((value&0x80000000)==0x80000000)
		{   
			__gpio_set_pin(lcd_board->board_pin->SPDT);
		}	      
		else
		{				      
			__gpio_clear_pin(lcd_board->board_pin->SPDT); 
		}
		
		udelay(50);			
		__gpio_set_pin(lcd_board->board_pin->SPCK);		
		value=(value<<1);
		udelay(50);			
	}                               
	__gpio_set_pin(lcd_board->board_pin->SPEN);			
	__gpio_set_pin(lcd_board->board_pin->SPDT);	
	udelay(50);

}

#if 0
static void panel_reg_init()
{
	spi_write_reg(0x00,0x00);
	spi_write_reg(0x7e,0x00);
	spi_write_reg(0x02,0x00);
	spi_write_reg(0x04,0x00);
	spi_write_reg(0x06,0x);
	spi_write_reg(007x,0x);
	spi_write_reg(0x08,0x);
	spi_write_reg(0x09,0x);
	spi_write_reg(0x0a,0x);
	spi_write_reg(0xb,0x);
	spi_write_reg(0xc,0x);
	spi_write_reg(0xd,0x);
	spi_write_reg(0xe,0x);
	spi_write_reg(0xf,0x);
	spi_write_reg(0x10,0x);
	spi_write_reg(0x2a,0x);
	spi_write_reg(0x2c,0x);
	spi_write_reg(0x2d,0x);
	spi_write_reg(0x2e,0x);
	spi_write_reg(0x2f,0x);
	spi_write_reg(0x31,0x);
	spi_write_reg(0x32,0x);
	spi_write_reg(0x33,0x);
	spi_write_reg(0x34,0x);
	spi_write_reg(0x36,0x);
	spi_write_reg(0x38,0x);
	spi_write_reg(0x39,0x);
	spi_write_reg(0x3c,0x);
	spi_write_reg(0x3d,0x);
	spi_write_reg(0x3e,0x);
	spi_write_reg(0x3f,0x);
	spi_write_reg(0x40,0x);
	spi_write_reg(0x41,0x);
	spi_write_reg(0x42,0x);
	spi_write_reg(0x43,0x);
	spi_write_reg(0x44,0x);
	spi_write_reg(0x45,0x);
	spi_write_reg(0x47,0x);
	spi_write_reg(0x48,0x);
	spi_write_reg(0x49,0x);
	spi_write_reg(0x4a,0x);
	spi_write_reg(0x4c,0x);
	spi_write_reg(0x4e,0x);
	spi_write_reg(0x4f,0x);
	spi_write_reg(0x52,0x);
	spi_write_reg(0x53, 0x);
	spi_write_reg(0x54, 0x);
	spi_write_reg(0x55, 0x);
	spi_write_reg(0x56, 0x);
	spi_write_reg(0x57, 0x);
	spi_write_reg(0x6e, 0x);
	spi_write_reg(0x70, 0x);
	spi_write_reg(0x71, 0x);
	spi_write_reg(0x72, 0x);
	spi_write_reg(0x70, 0x);
	mdelay(5);
	spi_write_reg(0x72, 0x);
	mdelay(5);
	spi_write_reg(0x72, 0x);
	mdelay(30);
	spi_write_reg(0x03, 0x);
	spi_write_reg(0x72, 0x);
	spi_write_reg(0x71, 0x);
	spi_write_reg(0x03, 0x);
	mdelay(60);
	spi_write_reg(0x, 0x);
	spi_write_reg(0x, 0x);
	spi_write_reg(0x, 0x);
	spi_write_reg(0x, 0x);
	spi_write_reg(0x, 0x);
	spi_write_reg(0x, 0x);
	spi_write_reg(0x, 0x);
	spi_write_reg(0x, 0x);
	spi_write_reg(0x, 0x);
	spi_write_reg(0x, 0x);
}
#else

static void panel_reg_init(void)
{
//	do
//	{
//		mdelay(5);
//		spi_write_reg(0x5a, 0xa5);
//		mdelay(5);
//		spi_write_reg(0xaa, 0x5a);
//	}
//	while(1);

	spi_write_reg(0x00, 0x00);
	spi_write_reg(0x7e, 0x00);
	spi_write_reg(0x04, 0x00);
	spi_write_reg(0x06, 0x03);
	spi_write_reg(0x07, 0x00);
	
	spi_write_reg(0x08, 0xC1);
	spi_write_reg(0x09, 0x00);
	spi_write_reg(0x0a, 0x30);
	spi_write_reg(0x0b, 0x50);
	spi_write_reg(0x0c, 0x57);
	
	spi_write_reg(0x0d, 0x00);
	spi_write_reg(0x0e, 0x00);
	spi_write_reg(0x0f, 0x10);
	spi_write_reg(0x10, 0x55);
	spi_write_reg(0x2a, 0xd6);
	
	spi_write_reg(0x2c, 0x01);
	spi_write_reg(0x2d, 0x00);
	spi_write_reg(0x2e, 0x01);
	spi_write_reg(0x2f, 0x01);
	spi_write_reg(0x31, 0x04);
	spi_write_reg(0x32, 0x3c);
	spi_write_reg(0x33, 0x73);
	spi_write_reg(0x34, 0xd1);
	spi_write_reg(0x36, 0x62);
	spi_write_reg(0x38, 0xc0);
	spi_write_reg(0x39, 0x01);
	spi_write_reg(0x3c, 0x78);
	spi_write_reg(0x3d, 0x64);
	spi_write_reg(0x3e, 0x16);
	spi_write_reg(0x3f, 0x00);
	spi_write_reg(0x40, 0x00);
	spi_write_reg(0x41, 0x00);

	mdelay(50);
	spi_write_reg(0x6E, 0x01);

}
#endif

static void LS032T8HX01_panel_display_on(void)
{
	if(!lcd_board)
		printk("%s: lcd_board is null\n", __func__);
	lcd_board->board_ops->lcd_board_power_on();
	
	lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->LCD_POWERON);
	//lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->LCD_VSYNC_PIN);
	//lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->LCD_HSYNC_PIN);
	mdelay(10);
	lcd_soc->soc_ops->gpio_as_output(lcd_board->board_pin->LCD_RESET_PIN);
	lcd_soc->soc_ops->gpio_clear_pin(lcd_board->board_pin->LCD_RESET_PIN);
	mdelay(50);
	lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->LCD_RESET_PIN);
	mdelay(10);
	
	panel_reg_init();
}

static void LS032T8HX01_panel_display_off(void)
{
	lcd_board->board_ops->lcd_board_power_off();
	lcd_soc->soc_ops->gpio_clear_pin(lcd_board->board_pin->LCD_POWERON);
}

static int LS032T8HX01_panel_probe(struct lcd_soc_info *soc , struct lcd_board_info *board)
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
	if(!(lcd_board->board_pin->LCD_POWERON && lcd_board->board_pin->LCD_RESET_PIN)) {
		printk(KERN_ERR JZ_LCD_PFX": lcd board pin defination error\n");
		return -1;
	}

	D("board_pin: LCD_POWERON=%d, LCD_RESET_PIN=%d.\n", lcd_board->board_pin->LCD_POWERON, lcd_board->board_pin->LCD_RESET_PIN);

	return 0;
}

static struct lcd_panel_ops_info LS032T8HX01_panel_ops = {
	.panel_init		=	LS032T8HX01_panel_display_init,
	.panel_display_on	=	LS032T8HX01_panel_display_on,
	.panel_display_off	=	LS032T8HX01_panel_display_off,
	.panel_probe		=	LS032T8HX01_panel_probe,
};

static struct lcd_panel_attr_info LS032T8HX01_panel_attr = {
	.bpp = 24,	/* */
	.vsp = 0,	/* VSYNC polarity: 0-rising,1-falling */
	.hsp = 0,	/* HSYNC polarity: 0-active high,1-active low */
	.dep = 0,	/* DE polarity:0-active high,1-active low */
	.slcd_cfg = 0,	/* Smart lcd configurations */
	.phys_width = 0,/* width of screen physical size, in mm */
	.phys_height = 0,/* height of screen physical size, in mm */
	.w = 480,	/* Panel Width(in pixel) */
	.h = 854,	/* Panel Height(in line) */
	.fclk = 55,	/* frame clk */
	.hsw = 12,	/* hsync width, in pclk */
	.vsw = 2,	/* vsync width, in line count */
	.elw = 4,	/* end of line, in pclk */
	.blw = 2,	/* begin of line, in pclk */
	.efw = 1,	/* end of frame, in line count */
	.bfw = 2,	/* begin of frame, in line count */
	.fg0_bpp = 32,
	.fg1_bpp = 32,
};
//128, 2, 40, 88, 10, 33,
static struct lcd_panel_info LS032T8HX01_lcd_panel = {
	.name = "LS032T8HX01",
	.panel_probable = 0,	// not probable
	.panel_attr = &LS032T8HX01_panel_attr,
	.panel_ops = &LS032T8HX01_panel_ops,
};

static int __init LS032T8HX01_init(void)
{
	// register the panel with lcd drivers
	lcd_register_panel(&LS032T8HX01_lcd_panel);
	return 0;
}

module_init(LS032T8HX01_init);
