/*
 * Copyright (C) 2010 Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4770_LINDEN_H__
#define __ASM_JZ4770_LINDEN_H__

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
#define GPIO_TP_DRV_EN			(32 * 1 + 23)
#define GPIO_GT801_INT			(32 * 1 + 19)	
#define GPIO_GT801_SHUTDOWN		(32 * 1 + 18)	
#define GPIO_GT801_IRQ		(IRQ_GPIO_0 + GPIO_GT801_INT)//* Used for GPIO-based bitbanging I2C 


/*======================================================================
 * FT5X0X TS
 */

#define GPIO_CTP_INT		(32 * 1 + 19)	//GPB19
#define GPIO_CTP_WAKE	(32 * 1 + 18)	//GPB18
#define GPIO_CTP_IRQ		(IRQ_GPIO_0 + GPIO_CTP_INT)//* Used for GPIO-based bitbanging I2C 

#define GPIO_FT5X0X_WAKE (32 * 1 + 18)
#define GPIO_FT5X0X_EN (32 * 1 + 23)

#define GPIO_CTP_VCC_EN		(32 * 1 + 23) //GPB23
/*======================================================================
 * SYSTEM GPIO
 */
#define GPIO_DC_DETE		GPF05  //(32 * 5 + 5) /* GPF05 */
#define GPIO_USB_DETE		GPB05  //(32 * 1 + 5) /* GPB05 */

#ifdef CONFIG_LINDEN_V_1_1
#define GPIO_CHARG_DETE		GPE09  //(32 * 2 + 31) /* GPC31 */
#else
#define GPIO_CHARG_DETE		GPF02  //(32 * 5 + 2) /* GPF02 */
#endif

#define GPIO_OTG_ID_PIN         GPF18  //(32 * 5 + 18)	// add by bcjia

#define GPIO_LCD_PWM		GPE01  //(32 * 4 + 1) /* GPE01 */
/*
#define GPIO_LCD_VCC_EN         GPE13  //(32 * 5 + 0)
//#define GPIO_LCD_VCC_EN_N    GPIO_LCD_VCC_EN
#define GPIO_LCD_PWM           GPE01   //(32 * 4 + 1)  //GPE01
#define GPIO_LCD_PW_EN         GPE00   //(32 * 4 + 0)   //GPE0
#define GPIO_LCD_PCLK_DS       GPC08   //(32 * 2 + 8)    
#define GPIO_LCD_HSYN_DS       GPC18   //(32 * 2 + 18)   
#define GPIO_LCD_VSYN_DS       GPC19   //(32 * 2 + 19)
*/
#define GPIO_LCD_DS_DEFAULT       0x0     //2.2ma 
#define GPIO_LCD_DS_HDMI      0x1     //6.5ma 
////#define GPIO_DISP_OFF_N     GPE00

#define GPIO_BOOT_SEL0		GPD17  //(32 * 3 + 17) /* GPD17 */
#define GPIO_BOOT_SEL1		GPD18  //(32 * 3 + 18) /* GPD18 */
#define GPIO_BOOT_SEL2		GPD19  //(32 * 3 + 19) /* GPD19 */

#define GPIO_PMU_IRQ_PIN	GPE19  //(32 * 4 + 19) /* GPE19 */
#define GPIO_PMU_IRQ		(IRQ_GPIO_0 + GPIO_PMU_IRQ_PIN)

#define INVALID_PIN  		GPE15  /* GPE15 pi380010 and pi380011 invalid pin */

#define GPIO_CHARG_LED	    GPE26 //for charge led control

#define NULL_PIN 		INVALID_PIN
#define GPIO_NULL		NULL_PIN

/*====================================================================
 * GPIO KEYS and ADKEYS (GPIO_WAKEUP used for end call)
 */

#define GPIO_VOLUMEUP 		(GPIO_BOOT_SEL0) // SW4-GPD17
//#define GPIO_BACK			(32 * 4 + 25) // SW6-GPE25
//#define GPIO_HOME			(32 * 3 + 19) // SW7-GPD19
#define ACTIVE_LOW_VOLUMEUP     0
//#define ACTIVE_LOW_BACK			1
//#define ACTIVE_LOW_HOME			1


#define GPIO_VOLUMEDOWN   	(GPIO_BOOT_SEL1) // SW3
#define GPIO_ENDCALL    	(32 * 0 + 30) // WAKEUP-GPA30
//#define GPIO_MENU		(32 * 4 + 24) // SW3-GPE24

//#define ACTIVE_LOW_MENU		1
#define ACTIVE_LOW_ENDCALL  	1
#define ACTIVE_LOW_VOLUMEDOWN   0

/*====================================================================
 *  ADKEYS LEVEL
 */

//#define GPIO_ADKEY_INT		(32 * 1 + 3)  // GPE8
//#define DPAD_LEFT_LEVEL		186 	//0.15V, 186=0.15/3.3*4096
//#define DPAD_DOWN_LEVEL		2482    //2.0V
//#define DPAD_UP_LEVEL		1985    //1.6V 
//#define DPAD_CENTER_LEVEL	1489    //1.2V
//#define DPAD_RIGHT_LEVEL	868     //0.7V

/*======================================================================
 * I2C
 */
#define GPIO_I2C3_SDA	(32 * 3 + 5)	/* GPD5 */
#define GPIO_I2C3_SCK	(32 * 3 + 4)	/* GPD4 */

#define GPIO_I2C4_SDA	(32 * 3 + 6)	/* GPD6 */
#define GPIO_I2C4_SCK	(32 * 3 + 7)	/* GPD7 */

/*======================================================================
 * CAMERA
 */
#define GPIO_CAMERA_RST         	(32*4+3) /*GPE3*/ 
#define GPIO_CAMERA_PDN   		(32*4+4) /*GPE4*/
#define GPIO_CAMERA_VCC_EN		(32*5+22)
#define GPIO_PWDN_2M         	(32*3+10) /*GPE3*/ 

/*======================================================================
 * MMC/SD
 */
#define GPIO_SD2_VCC_EN_N	(32 * 2 + 31) /* GPE09 */
#define GPIO_SD2_CD_N		(32 * 1 + 2) /* GPB02 */

#define MSC2_HOTPLUG_PIN	GPIO_SD2_CD_N
#define MSC2_HOTPLUG_IRQ	(IRQ_GPIO_0 + GPIO_SD2_CD_N)

/*======================================================================
 * LSM303D
 */
#define GPIO_LSM303D_INT1	(32 * 5 + 13) /* GPF13 */
#define GPIO_LSM303D_INT2	(32 * 4 + 18) /* GPE18 */

#define GPIO_LSM303D_IRQ1	(IRQ_GPIO_0 +  GPIO_LSM303D_INT1)
#define GPIO_LSM303D_IRQ2	(IRQ_GPIO_0 +  GPIO_LSM303D_INT2)

/*======================================================================
 * MMA8452 SENSORS
 */

#define GPIO_MMA8452_INT1		(32 * 5 + 13)	//GPA9
#define GPIO_MMA8452_INT2		(32 * 4 + 18)	//GPA1
#define GPIO_MMA8452_IRQ1		(IRQ_GPIO_0 + GPIO_MMA8452_INT1)//* Used for GPIO-based bitbanging I2C 
#define GPIO_MMA8452_IRQ2		(IRQ_GPIO_0 + GPIO_MMA8452_INT2)//* Used for GPIO-based bitbanging I2C 


/*======================================================================
 * WIFI
 */
#define GPIO_WL_RST_N	        (32 * 1 + 25) /* GPB25 */
#define GPIO_WL_REG_ON	        (32 * 5 + 4)  /* GPF4 */
#define GPIO_WL_GPIO_1	        (32 * 5 + 9)  /* GPF9 */
#define GPIO_WLAN_PW_EN	        (32 * 5 + 10) /* GPF10 */
#define GPIO_WL_GPIO_0	        (32 * 5 + 11) /* GPF11 */

//=====================================================================

#define GPIO_SPEAKER_EN	        (32 * 5 + 20) /* GPF20 */
#define GPIO_HEAD_DET	        (32 * 5 + 21) /* GPF21 */
#define ACTIVE_LEVEL_HEAD_DET   1
#define GPIO_HP_MUTE            (32 * 5 + 3) /* GPF3 */

/*======================================================================
 * HDMI
 */
#define GPIO_HDMI_HPD	 		GPF19

#define GPIO_HDMI_RST_N	 		GPE06
#define GPIO_HDMI_INT_N			GPF12
#define GPIO_HDMI_I2C3_SCK		GPD07
#define GPIO_HDMI_I2C3_SDA		GPD06

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
	__gpio_set_pin(GPIO_LCD_PWM);			\
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

#endif /* __ASM_JZ4770_LINDEN_H__ */
