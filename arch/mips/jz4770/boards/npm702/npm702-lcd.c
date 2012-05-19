/*
 * kernel/arch/mips/mach-jz4770/boards/npm702/npm702-lcd.c
 *
 * JZ4770 npm702 board lcd setup routines.
 *
 * Copyright (c) 2006-2010  Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/console.h>
#include <linux/delay.h>

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>

#include <linux/wakelock.h>

#include <asm/jzsoc.h>

#include <linux/jz_lcd.h>
#include <linux/act8600_power.h>

extern struct platform_device jz_lcd_device;
static struct lcd_board_pin_info npm702_lcd_board_pin = {
//#ifdef  CONFIG_NPM701_V_1_1
	.LCD_DITHB_PIN 	=	(0),	/*   */
	.LCD_UD_PIN	=	(0),	/*PF17*/
	.LCD_LR_PIN	=	(0),	/*PF16*/
	.LCD_MODE_PIN	=	(0),	/*  */
	.LCD_RESET_PIN	=	(32*4+11),	/*PE11*/
	.LCD_VCC_EN	=	GPIO_LCD_VCC_EN,	/*PF0*/
	.LCD_DE_PIN	=	(32*2+9),	/*PC9*/
	.LCD_VSYNC_PIN	=	(32*2+19),	/*PC19*/
	.LCD_HSYNC_PIN	=	(32*2+18),	/*PC18*/
	.LCD_POWERON	=	(0),	/*PE0*/
	.SPCK		=	(0),
	.SPEN		=	(0),
	.SPDT		=	(0),
	.SPRS		=	(0),
//#else
#if 0
	.LCD_DITHB_PIN 	=	(0),	/*   */
	.LCD_UD_PIN	=	(32*5+17),	/*PF17*/
	.LCD_LR_PIN	=	(32*5+16),	/*PF16*/
	.LCD_MODE_PIN	=	(0),	/*  */
	.LCD_RESET_PIN	=	(32*4+11),	/*PE11*/
	.LCD_VCC_EN	=	GPIO_LCD_VCC_EN,	/*PF0*/
	.LCD_DE_PIN	=	(32*2+9),	/*PC9*/
	.LCD_VSYNC_PIN	=	(32*2+19),	/*PC19*/
	.LCD_HSYNC_PIN	=	(32*2+18),	/*PC18*/
	.LCD_POWERON	=	(0),	/*PE0*/
	.SPCK		=	(0),
	.SPEN		=	(0),
	.SPDT		=	(0),
	.SPRS		=	(0),
//#endif
#endif
};

static void lcd_power_on(void)
{
	__gpio_as_output(npm702_lcd_board_pin.LCD_VCC_EN);
	__gpio_set_pin(npm702_lcd_board_pin.LCD_VCC_EN);
}

static void lcd_power_off(void)
{
	__gpio_as_output(npm702_lcd_board_pin.LCD_VCC_EN);
	__gpio_clear_pin(npm702_lcd_board_pin.LCD_VCC_EN);
}

static struct lcd_board_ops_info npm702_lcd_board_ops = {

	.lcd_board_power_on		= lcd_power_on,
	.lcd_board_power_off		= lcd_power_off,
};

static struct lcd_board_info npm702_lcd_board_info = {
	.board_pin 	=	&npm702_lcd_board_pin, 
	.board_ops	=	&npm702_lcd_board_ops,
};

static int __init jz_lcd_init(void)
{
	platform_device_add_data(&jz_lcd_device, &npm702_lcd_board_info, sizeof(struct lcd_board_info));
	return 0;
}

arch_initcall(jz_lcd_init);
