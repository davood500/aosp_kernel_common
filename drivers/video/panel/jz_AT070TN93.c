/*
 * kernel/drivers/video/panel/jz_AT070TN93.c -- Ingenic LCD panel device
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

#include <panel/lcd_base.h>
#include <panel/jz_AT070TN93.h>

extern void lcd_register_panel(struct lcd_panel_info *p);

static struct lcd_board_info *lcd_board;
static struct lcd_soc_info *lcd_soc;

static void AT070TN93_panel_display_init(void)
{
	/* nothing to do */
}

static void AT070TN93_panel_display_on(void)
{
	if(!lcd_board)
		printk("%s: lcd_board is null\n", __func__);
	lcd_board->board_ops->lcd_board_power_on();
	lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->LCD_POWERON);
	msleep(30);

#ifdef CONFIG_LCD_DE_MODE
	lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->LCD_MODE_PIN);

	lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->LCD_VSYNC_PIN);
	lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->LCD_HSYNC_PIN);	
#else
	lcd_soc->soc_ops->gpio_clear_pin(lcd_board->board_pin->LCD_MODE_PIN);
#endif
	lcd_soc->soc_ops->gpio_clear_pin(lcd_board->board_pin->LCD_UD_PIN);
	lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->LCD_LR_PIN);
	lcd_soc->soc_ops->gpio_clear_pin(lcd_board->board_pin->LCD_DITHB_PIN);
	lcd_soc->soc_ops->gpio_clear_pin(lcd_board->board_pin->LCD_RESET_PIN);
	mdelay(2);
	lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->LCD_RESET_PIN);
	msleep(110);
}

static void AT070TN93_panel_display_off(void)
{
	lcd_board->board_ops->lcd_board_power_off();
	lcd_soc->soc_ops->gpio_clear_pin(lcd_board->board_pin->LCD_POWERON);
}


static int AT070TN93_panel_probe(struct lcd_soc_info *soc , struct lcd_board_info *board)
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

static struct lcd_panel_ops_info AT070TN93_panel_ops = {
	.panel_init		=	AT070TN93_panel_display_init,
	.panel_display_on	=	AT070TN93_panel_display_on,
	.panel_display_off	=	AT070TN93_panel_display_off,
	.panel_probe		=	AT070TN93_panel_probe,
};

static struct lcd_panel_attr_info AT070TN93_panel_attr = {
	.bpp = 24,	/* bits per pixel */
	.vsp = 1,	/* VSYNC polarity: 0-rising,1-falling */
	.hsp = 1,	/* HSYNC polarity: 0-active high,1-active low */
	.dep = 0,	/* DE polarity:0-active high,1-active low */
	.pcp = 0,	/* PCP Pixel Clock polarity: 0-translation at rising edge, 1-translation at falling edge  */
	.slcd_cfg = 0,	/* Smart lcd configurations */
	.phys_width = 154,/* width of screen physical size, in mm */
	.phys_height = 86,/* height of screen physical size, in mm */
	.w = 800,	/* Panel Width(in pixel) */
	.h = 480,	/* Panel Height(in line) */
	.fclk = 55,	/* frame clk */
	.hsw = 30,	/* hsync width, in pclk */
	.vsw = 16,	/* vsync width, in line count */
	.elw = 48,	/* end of line, in pclk */
	.blw = 16,	/* begin of line, in pclk */
	.efw = 23,	/* end of frame, in line count */
	.bfw = 7,	/* begin of frame, in line count */
	.fg0_bpp = 32,
	.fg1_bpp = 32,
};



static struct lcd_panel_info AT070TN93_lcd_panel = {
	.name = "AT070TN93",
	.panel_probable = 0,	// not probable
	.panel_attr = &AT070TN93_panel_attr,
	.panel_ops = &AT070TN93_panel_ops,
};

static int __init AT070TN93_init(void)
{
	/* register the panel with lcd drivers */
	lcd_register_panel(&AT070TN93_lcd_panel);
	return 0;
}

module_init(AT070TN93_init);
