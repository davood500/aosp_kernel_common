/*
 * kernel/drivers/video/panel/jz_BYD_BM1024600.c -- Ingenic LCD panel device
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

static void BYD_BM1024600_panel_display_init(void)
{
	//nothing to do
}

static void BYD_BM1024600_panel_display_on(void)
{
	if(!lcd_board)
		printk("%s: lcd_board is null\n", __func__);
	lcd_board->board_ops->lcd_board_power_on();
//	lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->LCD_VSYNC_PIN);
//	lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->LCD_HSYNC_PIN);
	lcd_soc->soc_ops->gpio_clear_pin(lcd_board->board_pin->LCD_CSB);
	lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->LCD_DIMO);
	lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->LCD_DIMI);
	
	lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->LCD_STBY_PIN);
	lcd_soc->soc_ops->gpio_clear_pin(lcd_board->board_pin->LCD_RESET_PIN);
	mdelay(5);
	lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->LCD_RESET_PIN);
	mdelay(20);
}

static void BYD_BM1024600_panel_display_off(void)
{
	lcd_soc->soc_ops->gpio_clear_pin(lcd_board->board_pin->LCD_STBY_PIN);
	lcd_board->board_ops->lcd_board_power_off();
}

static int BYD_BM1024600_panel_probe(struct lcd_soc_info *soc , struct lcd_board_info *board)
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

	return 0;
}

static struct lcd_panel_ops_info BYD_BM1024600_panel_ops = {
	.panel_init		=	BYD_BM1024600_panel_display_init,
	.panel_display_on	=	BYD_BM1024600_panel_display_on,
	.panel_display_off	=	BYD_BM1024600_panel_display_off,
	.panel_probe		=	BYD_BM1024600_panel_probe,
};

static struct lcd_panel_attr_info BYD_BM1024600_panel_attr = {
	.bpp = 24,	/* */
	.vsp = 1,	/* VSYNC polarity: 0-rising,1-falling */
	.hsp = 1,	/* HSYNC polarity: 0-active high,1-active low */
	.dep = 0,	/* DE polarity:0-active high,1-active low */
	.pcp = 0,	/* PCP Pixel Clock polarity: 0-translation at rising edge, 1-translation at falling edge  */
	.slcd_cfg = 0,	/* Smart lcd configurations */
	.phys_width = 0,/* width of screen physical size, in mm */
	.phys_height = 0,/* height of screen physical size, in mm */
	.w = 1024,	/* Panel Width(in pixel) */
	.h = 600,	/* Panel Height(in line) */
	.fclk = 56,	/* frame clk */
	.hsw = 1,	/* 1-140 hsync width, in pclk */
	.vsw = 1,	/* 1-20 vsync width, in line count */
	.elw = 16,	/* end of line, in pclk: HSYNC front porch*/
	.blw = 160,	/* begin of line, in pclk */
	.efw = 1,	/* end of frame, in line count: VSYNC front porch */
	.bfw = 23,	/* begin of frame, in line count: */
	.fg0_bpp = 32,
	.fg1_bpp = 32,
};

static struct lcd_panel_info BYD_BM1024600_lcd_panel = {
	.name = "BYD_BM1024600",
	.panel_probable = 0,	// not probable
	.panel_attr = &BYD_BM1024600_panel_attr,
	.panel_ops = &BYD_BM1024600_panel_ops,
};

static int __init BYD_BM1024600_init(void)
{
	// register the panel with lcd drivers
	lcd_register_panel(&BYD_BM1024600_lcd_panel);
	return 0;
}

module_init(BYD_BM1024600_init);
