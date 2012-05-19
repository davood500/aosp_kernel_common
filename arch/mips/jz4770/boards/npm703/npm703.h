/*
 * Copyright (C) 2010 Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4770_NPM703_H__
#define __ASM_JZ4770_NPM703_H__

#define JZ_PMEM_BASE		0x0e000000
#define JZ_PMEM_SIZE		0x02000000
#define JZ_PMEM_ADSP_BASE	0x3c000000
#define JZ_PMEM_ADSP_SIZE	0x04000000
#define JZ_GPU_MEM_BASE		0x38000000
#define JZ_GPU_MEM_SIZE		0x04000000

/*======================================================================
 * Frequencies of on-board oscillators
 */
#define JZ_EXTAL		12000000  /* Main extal freq:	24 MHz */
#define JZ_EXTAL2		32768     /* RTC extal freq:	32.768 KHz */

#define CFG_UART_BASE UART2_BASE

/*======================================================================
 * GT801 TOUCHSCREEN
 */
#define GPIO_TP_DRV_EN			GPE12/*GPE12*/
#define GPIO_GT801_INT			(32 * 1 + 27)	
#define GPIO_GT801_SHUTDOWN		(32 * 1 + 19)	
#define GPIO_GT801_IRQ		(IRQ_GPIO_0 + GPIO_GT801_INT)//* Used for GPIO-based bitbanging I2C 
/*======================================================================
 * JZ_REMOUTE 
 */
#define GPIO_RMC_INT			(32 * 3 + 10)	//GPD10
#define GPIO_RMC_IRQ			(IRQ_GPIO_0 + GPIO_RMC_INT)
/*======================================================================
 * MMA8452 SENSORS
 */

#define GPIO_MMA8452_INT1		(32 * 0 + 9)	//GPA9
#define GPIO_MMA8452_INT2		(32 * 0 + 1)	//GPA1
#define GPIO_MMA8452_IRQ1		(IRQ_GPIO_0 + GPIO_MMA8452_INT1)//* Used for GPIO-based bitbanging I2C 
#define GPIO_MMA8452_IRQ2		(IRQ_GPIO_0 + GPIO_MMA8452_INT2)//* Used for GPIO-based bitbanging I2C 

/*======================================================================
 * SYSTEM GPIO
 */
#define GPIO_DC_DETE		GPF05  //(32 * 5 + 5) /* GPF05 */
#define GPIO_USB_DETE		GPB05  //(32 * 1 + 5) /* GPB05 */
#define GPIO_CHARG_DETE     	GPF14  //(32 * 5 + 14) /* GPF14 */
#define GPIO_CHARG_SET     	GPF13  //(32 * 5 + 13) /* GPF13 */

#define GPIO_LCD_PWM		GPE01  //(32 * 4 + 1) /* LED_EN */
#define GPIO_LCD_VCC_EN        	GPB18 //(32 * 1 + 18)
#define GPIO_LCD_PCLK_DS       	GPC08   //(32 * 2 + 8)    
#define GPIO_LCD_HSYN_DS       	GPC18   //(32 * 2 + 18)   
#define GPIO_LCD_VSYN_DS       	GPC19   //(32 * 2 + 19)
#define GPIO_LCD_DS_DEFAULT     0x0     //2.2ma 
#define GPIO_LCD_DS_HDMI      	0x1     //6.5ma 
////#define GPIO_DISP_OFF_N     GPE00

#define GPIO_BOOT_SEL0		GPD17  //(32 * 3 + 17) /* GPD17 */
#define GPIO_BOOT_SEL1		GPD18  //(32 * 3 + 18) /* GPD18 */
#define GPIO_BOOT_SEL2		GPD19  //(32 * 3 + 19) /* GPD19 */

#define GPIO_PMU_IRQ_PIN	GPE19  //(32 * 4 + 19) /* GPE19 */
#define GPIO_PMU_IRQ		(IRQ_GPIO_0 + GPIO_PMU_IRQ_PIN)

#define INVALID_PIN  		GPC31  //(32*2 + 31)         /* GPC31 UART2_RTS_N */
#define NULL_PIN 		INVALID_PIN
#define GPIO_NULL		NULL_PIN

#define GPIO_CAMERA_RST         	(32*4+3) /*GPE3*/ 
#define GPIO_CAMERA_PDN   		    (32*4+4) /*GPE4*/ 


#define GPIO_OTG_ID_PIN         GPE14
/*======================================================================
  =======
 * HDMI
 */
//#define GPIO_HDMI_RST_N	 		
//#define GPIO_HDMI_INT_N			GPE06
#define GPIO_HDMI_HPD			GPD04
#define GPIO_HDMI_INT_N			GPE06
#define GPIO_HDMI_PS2_KDATA		GPD07
#define GPIO_HDMI_PS2_KCLK		GPD06

/*====================================================================
 * GPIO KEYS and ADKEYS (GPIO_WAKEUP used for end call)
 */
#define GPIO_VOLUMEUP 	        GPE29
#define GPIO_VOLUMEDOWN   	(GPIO_BOOT_SEL1)

#define GPIO_ENDCALL    	(32 * 0 + 30) // WAKEUP-GPA30
#define ACTIVE_LOW_ENDCALL  	1
#define ACTIVE_LOW_VOLUMEDOWN   0
#define ACTIVE_LOW_VOLUMEUP     1

/*======================================================================
 * HA2605
 */
#define GPIO_HA2605_INT		(32 * 3 + 5)	//GPD5
#define GPIO_HA2605_IRQ		(IRQ_GPIO_0 + GPIO_HA2605_INT)//* Used for GPIO-based bitbanging I2C

#define GPIO_CAM1_PWR_ON	(32 * 1 + 4)	/*GPB4*/
#define GPIO_CAM2_PWR_ON	(32 * 5 + 7)	/*GPF7*/
#define GPIO_CAM_PWR_ON	GPIO_CAM2_PWR_ON
#define GPIO_I2C2_SDA	(32 * 5 + 16) 
#define GPIO_I2C2_SCK	(32 * 5 + 17) 
#define GPIO_I2C3_SDA	(32 * 4 + 25)	/* GPE25 */
#define GPIO_I2C3_SCK	(32 * 4 + 28)	/* GPE28 */

#define GPIO_I2C4_SDA	(32 * 0 + 6)	/* GPA3 */
#define GPIO_I2C4_SCK	(32 * 0 + 3)	/* GPA6 */

#define GPIO_I2C5_SDA	(32 * 3 + 7)	/* GPD7 */
#define GPIO_I2C5_SCK	(32 * 3 + 6)	/* GPD6 */

/*======================================================================
 * MMC/SD
 */
#define GPIO_SD2_VCC_EN_N	(32 * 2 + 29) /* GPC29 */
#define GPIO_SD2_CD_N		(32 * 1 + 24) /* GPB24 */

#define MSC2_HOTPLUG_PIN	GPIO_SD2_CD_N
#define MSC2_HOTPLUG_IRQ	(IRQ_GPIO_0 + GPIO_SD2_CD_N)


/*======================================================================
 * WIFI
 */
#define GPIO_WL_RST_N	        GPE18

//=====================================================================

#define GPIO_SPEAKER_EN	        GPF21//(32 * 5 + 20) /* GPF20 */
#define GPIO_HEAD_DET	        INVALID_PIN
#define ACTIVE_LEVEL_HEAD_DET   1
#define GPIO_HP_MUTE            GPF19//(32 * 5 + 19)

/*======================================================================

 * motor gpio
 */
#define GPIO_MOTOR_PIN           (32 * 3 + 11) /* GPD11 */

/*======================================================================
 * LCD backlight
 */
#define LCD_PWM_CHN		1	/* pwm channel */
#define LCD_PWM_FULL		256
#define PWM_BACKLIGHT_CHIP	1	/*0: digital pusle; 1: PWM*/


#if PWM_BACKLIGHT_CHIP

#define __lcd_init_backlight(n)						\
	do {    							\
		__lcd_set_backlight_level(n);				\
	} while (0)

/* 100 level: 0,1,...,100 */
#define __lcd_set_backlight_level(n)					\
	do {								\
		int _val = n;					\
		if (_val>=LCD_PWM_FULL)					\
		_val = LCD_PWM_FULL-1;				\
		if (_val<1)						\
		_val =1;						\
		__gpio_as_pwm(1);					\
		__tcu_disable_pwm_output(LCD_PWM_CHN);			\
		__tcu_stop_counter(LCD_PWM_CHN);			\
		__tcu_init_pwm_output_high(LCD_PWM_CHN);		\
		__tcu_set_pwm_output_shutdown_abrupt(LCD_PWM_CHN);	\
		__tcu_select_clk_div1(LCD_PWM_CHN);			\
		__tcu_mask_full_match_irq(LCD_PWM_CHN);			\
		__tcu_mask_half_match_irq(LCD_PWM_CHN);			\
		__tcu_clear_counter_to_zero(LCD_PWM_CHN);		\
		__tcu_set_full_data(LCD_PWM_CHN, JZ_EXTAL / 30000);	\
		__tcu_set_half_data(LCD_PWM_CHN, JZ_EXTAL / 30000 * _val / (LCD_PWM_FULL)); \
		__tcu_enable_pwm_output(LCD_PWM_CHN);			\
		__tcu_select_extalclk(LCD_PWM_CHN);			\
		__tcu_start_counter(LCD_PWM_CHN);			\
	} while (0)

#define __lcd_close_backlight()					\
	do {							\
		__gpio_as_output(GPIO_LCD_PWM);			\
		__gpio_clear_pin(GPIO_LCD_PWM);			\
	} while (0)

#else	/* PWM_BACKLIGHT_CHIP */

#define __send_low_pulse(n)					\
	do {							\
		unsigned int i;					\
		for (i = n; i > 0; i--)	{			\
			__gpio_clear_pin(GPIO_LCD_PWM);		\
			udelay(1);				\
			__gpio_set_pin(GPIO_LCD_PWM);		\
			udelay(3);				\
		}						\
	} while (0)

#define MAX_BRIGHTNESS_STEP	16				/* RT9365 supports 16 brightness step */
#define CONVERT_FACTOR		(256/MAX_BRIGHTNESS_STEP)	/* System support 256 brightness step */

#define __lcd_init_backlight(n)					\
	do {							\
		unsigned int tmp = (n)/CONVERT_FACTOR + 1;	\
		__gpio_as_output(GPIO_LCD_PWM);			\
		__gpio_set_pin(GPIO_LCD_PWM);			\
		udelay(30);					\
		__send_low_pulse(MAX_BRIGHTNESS_STEP-tmp);	\
	} while (0)

#define __lcd_set_backlight_level(n)					\
	do {								\
		unsigned int last = lcd_backlight_level / CONVERT_FACTOR + 1; \
		unsigned int tmp = (n) / CONVERT_FACTOR + 1;		\
		if (tmp <= last) {					\
			__send_low_pulse(last-tmp);			\
		} else {						\
			__send_low_pulse(last + MAX_BRIGHTNESS_STEP - tmp); \
		}							\
		udelay(30);						\
	} while (0)

#define __lcd_close_backlight()					\
	do {							\
		__gpio_as_output(GPIO_LCD_PWM);			\
		__gpio_clear_pin(GPIO_LCD_PWM);			\
	} while (0)

#endif	/*PWM_BACKLIGHT_CHIP*/

#endif /* __ASM_JZ4770_NP_M701_H__ */
