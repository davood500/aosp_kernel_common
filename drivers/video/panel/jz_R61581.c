/*
 * kernel/drivers/video/panel/jz_R61581.c -- Ingenic LCD panel device
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

static unsigned int SPEN = 0;
static unsigned int SPCK = 0;
static unsigned int SPDT = 0;
static unsigned int SPRS = 0;
static unsigned int LCD_RESET_PIN = 0;
static unsigned int LCD_POWERON = 0;

static void spi_write_cmd(unsigned char cmd)
{
	lcd_soc->soc_ops->gpio_set_pin(SPEN);
	lcd_soc->soc_ops->gpio_set_pin(SPCK);
	udelay(50);
	lcd_soc->soc_ops->gpio_as_output(SPDT);
	lcd_soc->soc_ops->gpio_clear_pin(SPDT);
	lcd_soc->soc_ops->gpio_clear_pin(SPRS);
	lcd_soc->soc_ops->gpio_clear_pin(SPEN);
	lcd_soc->soc_ops->spi_write(cmd, 0);
}

static void spi_write_data(unsigned char val)
{
	lcd_soc->soc_ops->spi_write(val, 1);
}

static void spi_read_data(unsigned char reg, unsigned char *val, unsigned char readByte)
{
	lcd_soc->soc_ops->gpio_set_pin(SPEN);
	lcd_soc->soc_ops->gpio_set_pin(SPCK);
	udelay(50);
	lcd_soc->soc_ops->gpio_as_output(SPDT);
	lcd_soc->soc_ops->gpio_clear_pin(SPDT);
	lcd_soc->soc_ops->gpio_clear_pin(SPRS);
	lcd_soc->soc_ops->gpio_clear_pin(SPEN);
	lcd_soc->soc_ops->spi_read(reg, val, readByte);
}

static void panel_reg_init_R61581(void)
{
	spi_write_cmd(0x11);
	udelay(150);

	spi_write_cmd(0xB0);
	spi_write_data(0x00);

	spi_write_cmd(0xB3);
	spi_write_data(0x02);
	spi_write_data(0x00);
	spi_write_data(0x00);
	spi_write_data(0x00);

	spi_write_cmd(0xB4);///DPI
	spi_write_data(0x11);

	spi_write_cmd(0xC0);
	spi_write_data(0x03);
	spi_write_data(0x3B);
	spi_write_data(0x00);
	spi_write_data(0x02);
	spi_write_data(0x00);
	spi_write_data(0x01);
	spi_write_data(0x00);
	spi_write_data(0x43);

	spi_write_cmd(0xC1);
	spi_write_data(0x08);
	spi_write_data(0x17);
	spi_write_data(0x08);
	spi_write_data(0x08);

	spi_write_cmd(0xC4);
	spi_write_data(0x22);
	spi_write_data(0x02);
	spi_write_data(0x00);
	spi_write_data(0x00);

	spi_write_cmd(0xC6);////EPL=1
	spi_write_data(0x03);

	spi_write_cmd(0xC8);
	spi_write_data(0x09);
	spi_write_data(0x08);
	spi_write_data(0x10);
	spi_write_data(0x85);
	spi_write_data(0x07);
	spi_write_data(0x08);
	spi_write_data(0x16);
	spi_write_data(0x05);
	spi_write_data(0x00);
	spi_write_data(0x32);
	spi_write_data(0x05);
	spi_write_data(0x16);
	spi_write_data(0x08);
	spi_write_data(0x88);
	spi_write_data(0x09);
	spi_write_data(0x10);
	spi_write_data(0x09);
	spi_write_data(0x04);
	spi_write_data(0x32);
	spi_write_data(0x00);

	spi_write_cmd(0x2A);
	spi_write_data(0x00);
	spi_write_data(0x00);
	spi_write_data(0x01);
	spi_write_data(0x3F);

	spi_write_cmd(0x2B);
	spi_write_data(0x00);
	spi_write_data(0x00);
	spi_write_data(0x01);
	spi_write_data(0xDF);

	spi_write_cmd(0x35);
	spi_write_data(0x00);

	spi_write_cmd(0x3A);
	spi_write_data(0x61);////R5G6B5

	spi_write_cmd(0x44);
	spi_write_data(0x00);
	spi_write_data(0x01);

	spi_write_cmd(0xD0);
	spi_write_data(0x07);
	spi_write_data(0x07);
	spi_write_data(0x16);
	spi_write_data(0x72);

	spi_write_cmd(0xD1);
	spi_write_data(0x03);
	spi_write_data(0x3A);
	spi_write_data(0x0A);

	spi_write_cmd(0xD2);
	spi_write_data(0x02);
	spi_write_data(0x44);
	spi_write_data(0x04);

	spi_write_cmd(0x29);
	udelay(10);
	spi_write_cmd(0x2C);

	lcd_soc->soc_ops->gpio_set_pin(SPEN);
}

static void R61581_panel_display_on(void)
{
	panel_reg_init_R61581();
}

static void R61581_panel_display_off(void)
{
	lcd_soc->soc_ops->gpio_set_pin(LCD_POWERON);
	lcd_soc->soc_ops->gpio_as_output(LCD_POWERON);

	spi_write_cmd(0x28);
	spi_write_cmd(0x10);  // sleep in
	mdelay(10);
}

static void R61581_panel_display_init(void)
{
	lcd_soc->soc_ops->gpio_as_output(SPEN);
	lcd_soc->soc_ops->gpio_as_output(SPCK);
	lcd_soc->soc_ops->gpio_as_output(SPDT);
	lcd_soc->soc_ops->gpio_as_output(SPRS);
	lcd_soc->soc_ops->gpio_as_output(LCD_RESET_PIN);
	lcd_soc->soc_ops->gpio_disable_pull(LCD_RESET_PIN);
}

static int __init R61581_panel_probe(struct lcd_soc_info *soc , struct lcd_board_info *board)
{
	if (soc) {
		lcd_soc = kzalloc(sizeof(struct lcd_soc_info), GFP_KERNEL);
		lcd_soc = soc;
	}
	if (board) {
		lcd_board = kzalloc(sizeof(struct lcd_board_info), GFP_KERNEL);
		lcd_board = board;
	}

	LCD_POWERON = lcd_board->board_pin->LCD_POWERON;
	LCD_RESET_PIN = lcd_board->board_pin->LCD_RESET_PIN;
	SPEN = lcd_board->board_pin->SPEN;
	SPCK = lcd_board->board_pin->SPCK;
	SPDT = lcd_board->board_pin->SPDT;
	SPRS = lcd_board->board_pin->SPRS;

	if(!(SPEN && SPCK && SPDT && SPRS && LCD_POWERON && LCD_RESET_PIN)) {
		printk(KERN_ERR JZ_LCD_PFX": lcd board pin defination error\n");
		return -1;
	}

	D("board_pin: SPEN=%d, SPCK=%d, SPDT=%d, SPRS=%d, LCD_POWERON=%d, LCD_RESET_PIN=%d.\n", SPEN, SPCK, SPDT, SPRS, LCD_POWERON, LCD_RESET_PIN);

//	R61581_panel_display_init();

	return 0;
}

static struct lcd_panel_ops_info R61581_panel_ops = {
	.panel_init		=	R61581_panel_display_init,
	.panel_display_on	=	R61581_panel_display_on,
	.panel_display_off	=	R61581_panel_display_off,
	.panel_probe		=	R61581_panel_probe,
};

static struct lcd_panel_attr_info R61581_panel_attr = {
	.bpp = 18,	/* */
	.vsp = 1,	/* VSYNC polarity: 0-rising,1-falling */
	.hsp = 1,	/* HSYNC polarity: 0-active high,1-active low */
	.dep = 0,	/* DE polarity:0-active high,1-active low */
	.slcd_cfg = 0,	/* Smart lcd configurations */
	.phys_width = 0,/* width of screen physical size, in mm */
	.phys_height = 0,/* height of screen physical size, in mm */
	.w = 320,	/* Panel Width(in pixel) */
	.h = 480,	/* Panel Height(in line) */
	.fclk = 50,	/* frame clk */
	.hsw = 2,	/* hsync width, in pclk */
	.vsw = 1,	/* vsync width, in line count */
	.elw = 2,	/* end of line, in pclk */
	.blw = 2,	/* begin of line, in pclk */
	.efw = 8,	/* end of frame, in line count */
	.bfw = 8,	/* begin of frame, in line count */
	.fg0_bpp = 32,
	.fg1_bpp = 32,
};

static struct lcd_panel_info R61581_lcd_panel = {
	.name = "R61581",
	.panel_probable = 0,
	.panel_attr = &R61581_panel_attr,
	.panel_ops = &R61581_panel_ops,
};

static int __init R61581_init(void)
{
	lcd_register_panel(&R61581_lcd_panel);
	return 0;
}

module_init(R61581_init);
