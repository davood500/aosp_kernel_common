/*
 * kernel/arch/mips/mach-jz4770/boards/linden/linden-lcd.c
 *
 * JZ4770 linden board lcd setup routines.
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
static struct lcd_board_pin_info linden_lcd_board_pin = {
	.LCD_VCC_EN	=	(32*5+0),	/*PF0*/
	.LCD_RESET_PIN	=	(32*4+2),	/*PE2*/
	.LCD_DE_PIN	=	(32*2+9),	/*PC9*/
	.LCD_VSYNC_PIN	=	(32*2+19),	/*PC19*/
	.LCD_HSYNC_PIN	=	(32*2+18),	/*PC18*/
	.LCD_STBY_PIN	=	(0),	/*PE0, 1 normal, 0 off*/
	.LCD_CSB	=	(32*4+16),
	.LCD_DIMO	=	(32*5+16),	/*  */
	.LCD_DIMI	=	(32*5+17),
};

static void lcd_power_on(void)
{
	act8600_output_enable(ACT8600_OUT7, ACT8600_OUT_ON);
	act8600_output_enable(ACT8600_OUT8, ACT8600_OUT_ON);
	__gpio_as_output(linden_lcd_board_pin.LCD_VCC_EN);
	__gpio_set_pin(linden_lcd_board_pin.LCD_VCC_EN);
}

static void lcd_power_off(void)
{
	__gpio_as_output(linden_lcd_board_pin.LCD_VCC_EN);
	__gpio_clear_pin(linden_lcd_board_pin.LCD_VCC_EN);
	act8600_output_enable(ACT8600_OUT7, ACT8600_OUT_OFF);
	act8600_output_enable(ACT8600_OUT8, ACT8600_OUT_OFF);
}

static struct lcd_board_ops_info linden_lcd_board_ops = {

	.lcd_board_power_on		= lcd_power_on,
	.lcd_board_power_off		= lcd_power_off,
};

static struct lcd_board_info linden_lcd_board_info = {
	.board_pin 	=	&linden_lcd_board_pin, 
	.board_ops	=	&linden_lcd_board_ops,
};

static int __init jz_lcd_init(void)
{
	platform_device_add_data(&jz_lcd_device, &linden_lcd_board_info, sizeof(struct lcd_board_info));
	return 0;
}

arch_initcall(jz_lcd_init);
