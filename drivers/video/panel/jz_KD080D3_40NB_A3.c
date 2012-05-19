/*
 * kernel/drivers/video/panel/jz_LTN097XL01_A01.c -- Ingenic LCD panel device
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

static void KD080D3_40NB_A3_lvds_config(void)
{
	printk("*************************** LVDS init ****************************\n");
	
	REG_LVDS_TXCTRL &= ~(1 << 18);               /*reset*/
	REG_LVDS_TXPLL0 &= ~(1 << 29);               /*bg power on*/
	
	mdelay(5);
	
	REG_LVDS_TXPLL0 &= ~(1 << 30);               /*pll power on*/
	
	udelay(20);
	
	REG_LVDS_TXCTRL |= (1 << 18);                /*reset*/			
	
	REG_LVDS_TXPLL0 &= ~(0x1f);
	REG_LVDS_TXPLL0 &= ~(0x7f << 8);
	
	//>50M
	REG_LVDS_TXPLL0 |= 0xc;
	
	//REG_LVDS_TXCTRL = 0x600487a1;
	
	printk("wait pll0 lock:");
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
	
	REG_LVDS_TXCTRL = 0x600523a1;
	REG_LVDS_TXPLL0 = 0x80002108;
	REG_LVDS_TXPLL1 = 0x83000000;
		
	printk("REG_LVDS_TXCTRL = %08x\n", REG_LVDS_TXCTRL);	
	printk("REG_LVDS_TXPLL0 = %08x\n", REG_LVDS_TXPLL0);
	printk("REG_LVDS_TXPLL1 = %08x\n", REG_LVDS_TXPLL1);

	/*
	  76M ok
	  [Read][130503c0]:       0x600522a1
	  [Read][130503c4]:       0x80002108
	  [Read][130503c8]:       0x85000000
	*/
}

static void KD080D3_40NB_A3_lvds_on(void){
	printk("KD080D3_40NB_A3_lvds_on\n");
	//KD080D3_40NB_A3_lvds_config();
	REG_LVDS_TXCTRL = REG_LVDS_TXCTRL | 0x1;
}

static void KD080D3_40NB_A3_lvds_off(void){
	printk("KD080D3_40NB_A3_lvds_off\n");
	REG_LVDS_TXCTRL = REG_LVDS_TXCTRL & (~0x1);
}

static void KD080D3_40NB_A3_panel_display_init(void)
{
	KD080D3_40NB_A3_lvds_config();
}

static void KD080D3_40NB_A3_panel_display_on(void)
{
	if(!lcd_board)
		printk("%s: lcd_board is null\n", __func__);

	KD080D3_40NB_A3_lvds_on();
	lcd_board->board_ops->lcd_board_power_on();

	lcd_soc->soc_ops->gpio_clear_pin(lcd_board->board_pin->LCD_RESET_PIN);
	mdelay(2);
	lcd_soc->soc_ops->gpio_set_pin(lcd_board->board_pin->LCD_RESET_PIN);
	msleep(100);
}

static void KD080D3_40NB_A3_panel_display_off(void)
{
	if(!lcd_board)
		printk("%s: lcd_board is null\n", __func__);

	KD080D3_40NB_A3_lvds_off();
	lcd_board->board_ops->lcd_board_power_off();
}

static int KD080D3_40NB_A3_panel_probe(struct lcd_soc_info *soc , struct lcd_board_info *board)
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

static struct lcd_panel_ops_info KD080D3_40NB_A3_panel_ops = {
	.panel_init		=	KD080D3_40NB_A3_panel_display_init,
	.panel_display_on	=	KD080D3_40NB_A3_panel_display_on,
	.panel_display_off	=	KD080D3_40NB_A3_panel_display_off,
	.panel_probe		=	KD080D3_40NB_A3_panel_probe,
};

static struct lcd_panel_attr_info KD080D3_40NB_A3_panel_attr = {
	.bpp = 18,	/* */
	.vsp = 0,	/* VSYNC polarity: 0-rising,1-falling */
	.hsp = 0,	/* HSYNC polarity: 0-active high,1-active low */
	.dep = 0,	/* DE polarity:0-active high,1-active low */
	.pcp = 0,	/* PCP Pixel Clock polarity: 0-translation at rising edge, 1-translation at falling edge  */
	.slcd_cfg = 0,	/* Smart lcd configurations */
	.phys_width = 0,/* width of screen physical size, in mm */
	.phys_height = 0,/* height of screen physical size, in mm */
	.w = 1024,	/* Panel Width(in pixel) */
	.h = 768,	/* Panel Height(in line) */
	.fclk = 55,	/* frame clk */
	.hsw = 0,	/* hsync width, in pclk */
	.vsw = 0,	/* vsync width, in line count */
	.elw = 0,	/* end of line, in pclk */
	.blw = 256,	/* begin of line, in pclk */
	.efw = 0,	/* end of frame, in line count */
	.bfw = 2,	/* begin of frame, in line count */
	.fg0_bpp = 32,
	.fg1_bpp = 32,
};



static struct lcd_panel_info KD080D3_40NB_A3_lcd_panel = {
	.name = "KD080D3_40NB_A3",
	.panel_probable = 0,	// not probable
	.panel_attr = &KD080D3_40NB_A3_panel_attr,
	.panel_ops = &KD080D3_40NB_A3_panel_ops,
};

static int __init KD080D3_40NB_A3_init(void)
{
	// register the panel with lcd drivers
	lcd_register_panel(&KD080D3_40NB_A3_lcd_panel);
	return 0;
}

module_init(KD080D3_40NB_A3_init);
