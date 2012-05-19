/*
 * kernel/drivers/video/panel/jz_RM68040.c -- Ingenic LCD panel device
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

static unsigned char regAddr = 0xBF;
static unsigned char regCount = 4;

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

static void panel_reg_init_RM68040(void)
{
	spi_write_cmd(0x11); 		
	mdelay(20);

	spi_write_cmd(0xd0); 		
	spi_write_data(0x07);
	spi_write_data(0x41);//42
	spi_write_data(0x1d);//0c

	spi_write_cmd(0xd1); 		
	spi_write_data(0x00);
	spi_write_data(0x02);//15
	spi_write_data(0x12);//13

	spi_write_cmd(0xd2); 		
	spi_write_data(0x01);
	spi_write_data(0x22);//22

	spi_write_cmd(0xc0); 		
	spi_write_data(0x00);//00
	spi_write_data(0x3b);//3b
	spi_write_data(0x00);
	spi_write_data(0x02);
	spi_write_data(0x11);  

	spi_write_cmd(0xc5); 		
	spi_write_data(0x03);  

	spi_write_cmd(0xc6); 		
	spi_write_data(0x02);

	spi_write_cmd(0xb4);
	spi_write_data(0x10);

	//gamma
	spi_write_cmd(0xc8); 		
	spi_write_data(0x02);
	spi_write_data(0x75);
	spi_write_data(0x77);
	spi_write_data(0x05);
	spi_write_data(0x03);
	spi_write_data(0x01);
	spi_write_data(0x00);
	spi_write_data(0x20);
	spi_write_data(0x57);
	spi_write_data(0x50);
	spi_write_data(0x01);
	spi_write_data(0x03);

	spi_write_cmd(0xF8); 		
	spi_write_data(0x01);

	spi_write_cmd(0xfe); 		
	spi_write_data(0x00);
	spi_write_data(0x02);

	spi_write_cmd(0x36); 	//set scan direction	
	spi_write_data(0x0a);	//


	spi_write_cmd(0x3a);    //set pixel format		
	spi_write_data(0x66);	//18bpp

	spi_write_cmd(0x2a); 	//set hor address 	
	spi_write_data(0x00);
	spi_write_data(0x00);	//start
	spi_write_data(0x01);
	spi_write_data(0x3f);	//end

	spi_write_cmd(0x2b); 	//set ver address	
	spi_write_data(0x00);
	spi_write_data(0x00);	//start
	spi_write_data(0x01);
	spi_write_data(0xdf);	//end

	mdelay(120);

	spi_write_cmd(0x29); 	//display on
	lcd_soc->soc_ops->gpio_set_pin(SPEN);
}

static void RM68040_panel_display_on(void)
{
	panel_reg_init_RM68040();
}

static void RM68040_panel_display_off(void)
{
	//spi_write_cmd(0x28);  // display off
	spi_write_cmd(0x10);  // sleep in
	mdelay(10);
}

static void RM68040_panel_display_init(void)
{
	lcd_soc->soc_ops->gpio_as_output(SPEN);
	lcd_soc->soc_ops->gpio_as_output(SPCK);
	lcd_soc->soc_ops->gpio_as_output(SPDT);
	lcd_soc->soc_ops->gpio_as_output(SPRS);
	lcd_soc->soc_ops->gpio_as_output(LCD_RESET_PIN);
	lcd_soc->soc_ops->gpio_disable_pull(LCD_RESET_PIN);
}

static int do_panel_probe(void)
{
	unsigned char code[10] = {0};

	spi_write_cmd(0xf8);
	spi_write_data(0x01);
	spi_write_cmd(0xfa);
	spi_write_data(0x00);
	spi_write_data(0x00);
	spi_write_data(0x00);
	spi_write_data(0x00);
	spi_write_data(0x00);
	spi_write_data(0x00);
	spi_write_data(0x00);
	spi_write_data(0x00);
	spi_write_data(0x00);
	spi_write_data(0x00);
	spi_write_data(0x20);
	spi_write_cmd(0xc6);
	spi_write_data(0x80);
	spi_read_data(regAddr, code, regCount);
	D("spi read from RM68040: %x, %x, %x, %x, %x, %x\n",code[0],code[1],code[2],code[3],code[4],code[5]);
	if((code[0] == 0x1)&&(code[1] == 0xd0)&&(code[2] == 0x68)&&(code[3] == 0x4))//Rm68040
		return 0;
	else
		return -1;
}

static int __init RM68040_panel_probe(struct lcd_soc_info *soc , struct lcd_board_info *board)
{
	int ret = -1;

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

	RM68040_panel_display_init();

	lcd_soc->soc_ops->gpio_disable_pull(LCD_POWERON);
	lcd_soc->soc_ops->gpio_clear_pin(LCD_POWERON);
	lcd_soc->soc_ops->gpio_as_output(LCD_POWERON);
	mdelay(50);

	lcd_soc->soc_ops->gpio_set_pin(LCD_RESET_PIN);
	mdelay(1);
	lcd_soc->soc_ops->gpio_clear_pin(LCD_RESET_PIN);
	mdelay(10);
	lcd_soc->soc_ops->gpio_set_pin(LCD_RESET_PIN);
	mdelay(100);

	ret = do_panel_probe();

	lcd_soc->soc_ops->gpio_clear_pin(LCD_RESET_PIN);
	mdelay(10);
	lcd_soc->soc_ops->gpio_set_pin(LCD_RESET_PIN);
	mdelay(100);

	return ret;
}

static struct lcd_panel_ops_info RM68040_panel_ops = {
	.panel_init		=	RM68040_panel_display_init,
	.panel_display_on	=	RM68040_panel_display_on,
	.panel_display_off	=	RM68040_panel_display_off,
	.panel_probe		=	RM68040_panel_probe,
};

static struct lcd_panel_attr_info RM68040_panel_attr = {
	.bpp = 18,	/* */
	.vsp = 1,	/* VSYNC polarity: 0-rising,1-falling */
	.hsp = 1,	/* HSYNC polarity: 0-active high,1-active low */
	.pcp = 1,
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

static struct lcd_panel_info RM68040_lcd_panel = {
	.name = "RM68040",
	.panel_probable = 1,
	.panel_attr = &RM68040_panel_attr,
	.panel_ops = &RM68040_panel_ops,
};

static int __init RM68040_init(void)
{
	lcd_register_panel(&RM68040_lcd_panel);
	return 0;
}

module_init(RM68040_init);
