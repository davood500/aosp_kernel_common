/*
 * kernel/drivers/video/panel/jz_HDMI_DEFAULT.c -- Ingenic LCD panel device
 *
 * A dummy display resolution for device which only has HDMI output not have LCD Panel
 *
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

#ifndef CONFIG_HDMI_DEFAULT_WIDTH
#define CONFIG_HDMI_DEFAULT_WIDTH 1280
#endif

#ifndef CONFIG_HDMI_DEFAULT_HEIGHT
#define CONFIG_HDMI_DEFAULT_HEIGHT 720
#endif

#ifndef CONFIG_HDMI_DEFAULT_FPS
#define CONFIG_HDMI_DEFAULT_FPS 60
#endif

#define HDMI_DEFAULT_WIDTH  CONFIG_HDMI_DEFAULT_WIDTH
#define HDMI_DEFAULT_HEIGHT CONFIG_HDMI_DEFAULT_HEIGHT
#define HDMI_DEFAULT_FPS    CONFIG_HDMI_DEFAULT_FPS


extern void lcd_register_panel(struct lcd_panel_info *p);

static struct lcd_board_info *lcd_board;
static struct lcd_soc_info *lcd_soc;

static void HDMI_DEFAULT_panel_display_init(void)
{
	//nothing to do
}

static void HDMI_DEFAULT_panel_display_on(void)
{
	return;
}

static void HDMI_DEFAULT_panel_display_off(void)
{
	return;
}

static int HDMI_DEFAULT_panel_probe(struct lcd_soc_info *soc , struct lcd_board_info *board)
{
	/* check the parameters from lcd_driver */
	printk("HDMI_DEFAULT_panel_probe()");
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

	return 0;
}

static struct lcd_panel_ops_info HDMI_DEFAULT_panel_ops = {
	.panel_init		=	HDMI_DEFAULT_panel_display_init,
	.panel_display_on	=	HDMI_DEFAULT_panel_display_on,
	.panel_display_off	=	HDMI_DEFAULT_panel_display_off,
	.panel_probe		=	HDMI_DEFAULT_panel_probe,
};

static struct lcd_panel_attr_info HDMI_DEFAULT_panel_attr = {
	.bpp = 24,	/* */
	.vsp = 1,	/* VSYNC polarity: 0-rising,1-falling */
	.hsp = 1,	/* HSYNC polarity: 0-active high,1-active low */
	.dep = 0,	/* DE polarity:0-active high,1-active low */
	.pcp = 0,	/* PCP Pixel Clock polarity: 0-translation at rising edge, 1-translation at falling edge  */
	.slcd_cfg = 0,	/* Smart lcd configurations */
	.phys_width = 0,/* width of screen physical size, in mm */
	.phys_height = 0,/* height of screen physical size, in mm */
	.w = HDMI_DEFAULT_WIDTH,	/* Panel Width(in pixel) */
	.h = HDMI_DEFAULT_HEIGHT,	/* Panel Height(in line) */
	.fclk = HDMI_DEFAULT_FPS,	/* frame clk */
	.hsw = 1,	/* hsync width, in pclk */
	.vsw = 1,	/* vsync width, in line count */
	.elw = 32,	/* end of line, in pclk */
	.blw = 44,	/* begin of line, in pclk */
	.efw = 14,	/* end of frame, in line count */
	.bfw = 21,	/* begin of frame, in line count */
	.fg0_bpp = 32,
	.fg1_bpp = 32,
};

static struct lcd_panel_info HDMI_DEFAULT_lcd_panel = {
	.name = "HDMI_DEFAULT",
	.panel_probable = 0,	// not probable
	.panel_attr = &HDMI_DEFAULT_panel_attr,
	.panel_ops = &HDMI_DEFAULT_panel_ops,
};

static int __init HDMI_DEFAULT_init(void)
{
	// register the panel with lcd drivers
	lcd_register_panel(&HDMI_DEFAULT_lcd_panel);

	return 0;
}

module_init(HDMI_DEFAULT_init);
