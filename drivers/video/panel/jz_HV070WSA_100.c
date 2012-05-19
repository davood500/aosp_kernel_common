/*
 * kernel/drivers/video/panel/jz_HV070WSA_100.c -- Ingenic LCD panel device
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

/* LVDS init function */
static void HV070WSA_100_lvds_config(void)
{	
	REG_LVDS_TXCTRL &= ~(1 << 18);               /*reset*/
	REG_LVDS_TXPLL0 &= ~(1 << 29);               /*bg power on*/
	
	mdelay(5);
	
	REG_LVDS_TXPLL0 &= ~(1 << 30);               /*pll power on*/
	
	udelay(20);
	
	REG_LVDS_TXCTRL |= (1 << 18);                /*disable reset*/
					
	REG_LVDS_TXPLL0 &= ~(0x1f);
	REG_LVDS_TXPLL0 &= ~(0x7f << 8);
	
	/* 50M */
	REG_LVDS_TXPLL0 |= 0xc;

/* 
 * When you use 8 bits(RGB 0:7) or more bits LVDS panel.
 * The highest bit(LVDS_MODEL_SEL) of REG_LVDS_TXCTRL 
 * according to the order of panel's LVDS input signal.
 * if the order is VESA format this bit need to set 1,
 * else(JEIDA format) set 0. 
*/	
	REG_LVDS_TXCTRL = 0xe00524a1;
	REG_LVDS_TXPLL0 = 0x00002108;
	REG_LVDS_TXPLL1 = 0x81000000;

	printk("wait pll0 lock:");
	while(!(REG_LVDS_TXPLL0 & (1 << 31)))       /* wait and check LVDS_PLL_LOCK lock */
	{
		static int count = 0;
		printk(".");
		mdelay(10);
		if (count++ > 100)
		{
			count = 0;
			printk("wait pll0 timeout!\n");
			break;
		}
	}
	printk("...ok\n");
/*	
	printk("REG_LVDS_TXCTRL = %08x\n", REG_LVDS_TXCTRL);	
	printk("REG_LVDS_TXPLL0 = %08x\n", REG_LVDS_TXPLL0);
	printk("REG_LVDS_TXPLL1 = %08x\n", REG_LVDS_TXPLL1);
*/
	/*
        54M ok
		[Read][130503c0]:       0xe00524a1
		[Read][130503c4]:       0x80002108
		[Read][130503c8]:       0x81000000
	*/
}

static void HV070WSA_100_lvds_on(void)
{	
	HV070WSA_100_lvds_config();	
	REG_LVDS_TXCTRL |= 0x1;
	printk("HV070WSA_100_lvds_on\n");
}

static void HV070WSA_100_lvds_off(void)
{
	REG_LVDS_TXCTRL &= ~0x1;
	printk("HV070WSA_100_lvds_off\n");
}

static void HV070WSA_100_panel_display_init(void)
{
	/* nothing to do */
}

/* 
 * You can not lock the pll0 until the pixel clock was output.
 * So move the lvds config to the panel display on function.
*/
static void HV070WSA_100_panel_display_on(void)
{
	if(!lcd_board)
		printk("%s: lcd_board is null\n", __func__);
	
	HV070WSA_100_lvds_on();           	
	lcd_board->board_ops->lcd_board_power_on();
		
	msleep(80);
}

static void HV070WSA_100_panel_display_off(void)
{
	if(!lcd_board)
		printk("%s: lcd_board is null\n", __func__);

	HV070WSA_100_lvds_off();
	lcd_board->board_ops->lcd_board_power_off();	
}

static int HV070WSA_100_panel_probe(struct lcd_soc_info *soc , struct lcd_board_info *board)
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
		
	return 0;
}

static struct lcd_panel_ops_info HV070WSA_100_panel_ops = {
	.panel_init		=	HV070WSA_100_panel_display_init,
	.panel_display_on	=	HV070WSA_100_panel_display_on,
	.panel_display_off	=	HV070WSA_100_panel_display_off,
	.panel_probe		=	HV070WSA_100_panel_probe,
};

static struct lcd_panel_attr_info HV070WSA_100_panel_attr = {
	.bpp = 24,	/* bits per pixel, LVDS D0:D2,bpp = 18, LVDS D0:D3,bpp = 24 */
	.vsp = 1,	/* VSYNC polarity: 0-rising,1-falling */
	.hsp = 1,	/* HSYNC polarity: 0-active high,1-active low */
	.dep = 0,	/* DE polarity:0-active high,1-active low */
	.pcp = 0,	/* PCP Pixel Clock polarity: 0-translation at rising edge, 1-translation at falling edge  */
	.slcd_cfg = 0,	/* Smart lcd configurations */
	.phys_width = 154,/* width of screen physical size, in mm */
	.phys_height = 90,/* height of screen physical size, in mm */
	.w = 1024,	/* Panel Width(in pixel) */
	.h = 600,	/* Panel Height(in line) */
	.fclk = 60,	/* frame clk */
	.hsw = 0,	/* hsync width, in pclk */
	.vsw = 0,	/* vsync width, in line count */
	.elw = 0,	/* end of line, in pclk */
	.blw = 336,	/* begin of line, in pclk */
	.efw = 0,	/* end of frame, in line count */
	.bfw = 62,	/* begin of frame, in line count */
	.fg0_bpp = 32,
	.fg1_bpp = 32,
};

static struct lcd_panel_info HV070WSA_100_lcd_panel = {
	.name = "HV070WSA_100",
	.panel_probable = 0,	/* not probable */
	.panel_attr = &HV070WSA_100_panel_attr,
	.panel_ops = &HV070WSA_100_panel_ops,
};

static int __init HV070WSA_100_init(void)
{
	/* register the panel with lcd drivers */
	lcd_register_panel(&HV070WSA_100_lcd_panel);
	return 0;
}

module_init(HV070WSA_100_init);
