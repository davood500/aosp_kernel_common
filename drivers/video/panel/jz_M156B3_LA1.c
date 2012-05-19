/*
 * kernel/drivers/video/panel/jz_M156B3_LA1.c -- Ingenic LCD panel device
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
extern int check_invalid_pins(void);

static struct lcd_board_info *lcd_board;
static struct lcd_soc_info *lcd_soc;

static void M156B3_LA1_lvds_config(void)
{
	printk("*************************** LVDS init *************************\n");
	
	REG_LVDS_TXCTRL &= ~(1 << 18);               /*reset*/
	REG_LVDS_TXPLL0 &= ~(1 << 29);               /*bg power on*/
	
	mdelay(5);
	
	REG_LVDS_TXPLL0 &= ~(1 << 30);               /*pll power on*/
	
	udelay(20);
	
	REG_LVDS_TXCTRL |= (1 << 18);                /*disable reset*/		
/*	
	REG_LVDS_TXPLL0 &= ~(0x1f);
	REG_LVDS_TXPLL0 &= ~(0x7f << 8);
	
	//>50M
	REG_LVDS_TXPLL0 |= 0xc;
*/
	REG_LVDS_TXCTRL = 0x600525a1;
	REG_LVDS_TXPLL0 = 0x00002108;
	REG_LVDS_TXPLL1 = 0x85000000;

	printk("REG_LVDS_TXCTRL = %08x\n", REG_LVDS_TXCTRL);	
	printk("REG_LVDS_TXPLL0 = %08x\n", REG_LVDS_TXPLL0);
	printk("REG_LVDS_TXPLL1 = %08x\n", REG_LVDS_TXPLL1);
		
	printk("wait pll0 lock:\n");
	while(!(REG_LVDS_TXPLL0 & (1 << 31)))
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
	  48M ok
	  [Read][130503c0]:       0x600525a1
	  [Read][130503c4]:       0x80002108
	  [Read][130503c8]:       0x85000000
	*/	
}

static void M156B3_LA1_lvds_on(void){
	printk("M156B3_LA1_lvds_on\n");
	M156B3_LA1_lvds_config();
	REG_LVDS_TXCTRL = REG_LVDS_TXCTRL | 0x1;
}

static void M156B3_LA1_lvds_off(void){
	printk("M156B3_LA1_lvds_off\n");
	REG_LVDS_TXCTRL = REG_LVDS_TXCTRL & (~0x1);
}

static void M156B3_LA1_panel_display_init(void)
{
	//M156B3_LA1_lvds_config();
}

static void M156B3_LA1_panel_display_on(void)
{
	if(!lcd_board)
		printk("%s: lcd_board is null\n", __func__);

	M156B3_LA1_lvds_on();
	lcd_board->board_ops->lcd_board_power_on();	

	msleep(80);
}

static void M156B3_LA1_panel_display_off(void)
{
	if(!lcd_board)
		printk("%s: lcd_board is null\n", __func__);

	M156B3_LA1_lvds_off();
	lcd_board->board_ops->lcd_board_power_off();
}

static int M156B3_LA1_panel_probe(struct lcd_soc_info *soc , struct lcd_board_info *board)
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

static struct lcd_panel_ops_info M156B3_LA1_panel_ops = {
	.panel_init		=	M156B3_LA1_panel_display_init,
	.panel_display_on	=	M156B3_LA1_panel_display_on,
	.panel_display_off	=	M156B3_LA1_panel_display_off,
	.panel_probe		=	M156B3_LA1_panel_probe,
};

static struct lcd_panel_attr_info M156B3_LA1_panel_attr = {
	.bpp = 18,	/* */
	.vsp = 1,	/* VSYNC polarity: 0-rising,1-falling */
	.hsp = 1,	/* HSYNC polarity: 0-active high,1-active low */
	.dep = 0,	/* DE polarity:0-active high,1-active low */
	.pcp = 0,	/* PCP Pixel Clock polarity: 0-translation at rising edge, 1-translation at falling edge  */
	.slcd_cfg = 0,	/* Smart lcd configurations */
	.phys_width = 344,/* width of screen physical size, in mm */
	.phys_height = 194,/* height of screen physical size, in mm */
	.w = 1366,	/* Panel Width(in pixel) */
	.h = 768,	/* Panel Height(in line) */
	.fclk = 43,	/* frame clk */
	.hsw = 0,	/* hsync width, in pclk */
	.vsw = 0,	/* vsync width, in line count */
	.elw = 0,	/* end of line, in pclk */
	.blw = 120,	/* begin of line, in pclk */
	.efw = 0,	/* end of frame, in line count */
	.bfw = 20,	/* begin of frame, in line count */
	.fg0_bpp = 32,
	.fg1_bpp = 32,
};



static struct lcd_panel_info M156B3_LA1_lcd_panel = {
	.name = "M156B3_LA1",
	.panel_probable = 0,	// not probable
	.panel_attr = &M156B3_LA1_panel_attr,
	.panel_ops = &M156B3_LA1_panel_ops,
};

static int __init M156B3_LA1_init(void)
{
	// register the panel with lcd drivers
	lcd_register_panel(&M156B3_LA1_lcd_panel);
	return 0;
}

module_init(M156B3_LA1_init);
