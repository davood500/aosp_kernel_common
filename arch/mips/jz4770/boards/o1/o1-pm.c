/*
 * linux/arch/mips/jz4760/board-o1.c
 *
 * Copyright (c) 2006-2010  Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/input.h>

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>
#include <asm/jzsoc.h>
#include <linux/act8600_power.h>

// default gpio state is input pull;
int gpio_sleep_state_table[][2] = {
	/* GPIO Group - A */
	{32 * 0 +  0,	GSS_INPUT_PULL  }, /* NC */
	{32 * 0 +  1,	GSS_INPUT_PULL	}, /* NC */
	{32 * 0 +  2,	GSS_INPUT_PULL  }, /* NC */
	{32 * 0 +  3,	GSS_INPUT_PULL  }, /* NC */
	{32 * 0 +  4,	GSS_INPUT_PULL  }, /* NC */
	{32 * 0 +  5,	GSS_INPUT_PULL  }, /* NC */
	{32 * 0 +  6,	GSS_INPUT_PULL  }, /* NC */
	{32 * 0 +  7,	GSS_INPUT_PULL  }, /* NC */
	{32 * 0 +  8,	GSS_INPUT_PULL  }, /* NC */
	{32 * 0 +  9,	GSS_INPUT_PULL  }, /* NC */
	{32 * 0 + 10,	GSS_INPUT_PULL  }, /* NC */
	{32 * 0 + 11,	GSS_INPUT_PULL  }, /* NC */
	{32 * 0 + 12,	GSS_INPUT_PULL  }, /* NC */
	{32 * 0 + 13,	GSS_INPUT_PULL  }, /* NC */
	{32 * 0 + 14,	GSS_INPUT_PULL  }, /* NC */
	{32 * 0 + 15,	GSS_INPUT_PULL  }, /* NC */
	{32 * 0 + 16,	GSS_INPUT_PULL  }, /* NC */
	{32 * 0 + 17,	GSS_INPUT_PULL  }, /* NC */
	{32 * 0 + 18,	GSS_IGNORE	}, /* MSC0_CLK */
	{32 * 0 + 19,	GSS_IGNORE	}, /* MSC0_CMD */
	{32 * 0 + 20,	GSS_IGNORE	}, /* MSC0_DATA */
	{32 * 0 + 21,	GSS_IGNORE	}, /* MSC0_DATA */
	{32 * 0 + 22,	GSS_IGNORE	}, /* MSC0_DATA */
	{32 * 0 + 23,	GSS_IGNORE	}, /* MSC0_DATA */
	{32 * 0 + 24,	GSS_INPUT_PULL  }, /* NC */
	{32 * 0 + 25,	GSS_INPUT_PULL  }, /* NC */
	{32 * 0 + 26,	GSS_INPUT_PULL  }, /* NC */
	{32 * 0 + 27,	GSS_OUTPUT_HIGH}, /* WAIT_N */
	{32 * 0 + 28,	GSS_INPUT_PULL  }, /* ?? */
	{32 * 0 + 29,	GSS_INPUT_PULL  }, /* ?? */
	{32 * 0 + 30,	GSS_IGNORE	}, /* WAKE_UP*/
	{32 * 0 + 31,	GSS_IGNORE	}, /* ?? */
	/* GPIO Group - B */

	{32 * 1 +  0,	GSS_INPUT_PULL  }, /* NC */
	{32 * 1 +  1,	GSS_INPUT_PULL  }, /* NC */
	{32 * 1 +  2,	GSS_IGNORE  	}, /* SD1_CD_N*/
	{32 * 1 +  3,	GSS_INPUT_PULL  }, /* NC */
	{32 * 1 +  4,	GSS_INPUT_PULL  }, /* NC */
	{32 * 1 +  5,	GSS_INPUT_PULL  }, /* USB_DETE */
	{32 * 1 +  6,	GSS_OUTPUT_LOW	}, /* CIM_PCLK */
	{32 * 1 +  7,	GSS_OUTPUT_LOW	}, /* CIM_HS */ 
	{32 * 1 +  8,	GSS_OUTPUT_LOW	}, /* CIM_VS */ 
	{32 * 1 +  9,	GSS_OUTPUT_LOW	}, /* CIM_MCLK */ 
	{32 * 1 + 10,	GSS_OUTPUT_LOW	}, /* CIM_DATA */ 
	{32 * 1 + 11,	GSS_OUTPUT_LOW	}, /* CIM_DATA */ 
	{32 * 1 + 12,	GSS_OUTPUT_LOW	}, /* CIM_DATA */ 
	{32 * 1 + 13,	GSS_OUTPUT_LOW	}, /* CIM_DATA */ 
	{32 * 1 + 14,	GSS_OUTPUT_LOW	}, /* CIM_DATA */ 
	{32 * 1 + 15,	GSS_OUTPUT_LOW	}, /* CIM_DATA */ 
	{32 * 1 + 16,	GSS_OUTPUT_LOW	}, /* CIM_DATA */ 
	{32 * 1 + 17,	GSS_OUTPUT_LOW	}, /* CIM_DATA */ 
	{32 * 1 + 18,	GSS_INPUT_NOPULL}, /* CTP_WAKE_UP*/ 
	{32 * 1 + 19,	GSS_INPUT_NOPULL}, /* CTP_IRQ */ 
	{32 * 1 + 20,	GSS_INPUT_PULL  }, /* NC */
	{32 * 1 + 21,	GSS_INPUT_PULL  }, /* NC */
	{32 * 1 + 22,	GSS_OUTPUT_LOW}, /* AVDEFUSE_EN_N */
	{32 * 1 + 24,	GSS_INPUT_NOPULL}, /* BT_RST_N */ 
	{32 * 1 + 25,	GSS_INPUT_NOPULL}, /* WL_RST_N */
	{32 * 1 + 26,	GSS_INPUT_PULL  }, /* NC */
	{32 * 1 + 27,	GSS_OUTPUT_LOW	}, /* TSD7 */ 
	{32 * 1 + 28,	GSS_OUTPUT_LOW	}, /* TSCLK */
	{32 * 1 + 29,	GSS_OUTPUT_LOW	}, /* TSSTR */
	{32 * 1 + 30,	GSS_OUTPUT_LOW	}, /* TS_IRQ_N */
	{32 * 1 + 31,	GSS_OUTPUT_LOW	}, /* TSFRM */

	/* GPIO Group - C */
	{32 * 2 +  0,	GSS_OUTPUT_LOW	}, /* LCD_B */ 
	{32 * 2 +  1,	GSS_OUTPUT_LOW	}, /* LCD_B */ 
	{32 * 2 +  2,	GSS_OUTPUT_LOW	}, /* LCD_B */ 
	{32 * 2 +  3,	GSS_OUTPUT_LOW	}, /* LCD_B */ 
	{32 * 2 +  4,	GSS_OUTPUT_LOW	}, /* LCD_B */ 
	{32 * 2 +  5,	GSS_OUTPUT_LOW	}, /* LCD_B */ 
	{32 * 2 +  6,	GSS_OUTPUT_LOW	}, /* LCD_B */ 
	{32 * 2 +  7,	GSS_OUTPUT_LOW	}, /* LCD_B */ 
	{32 * 2 +  8,	GSS_OUTPUT_LOW	}, /* LCD_PCLK */ 
	{32 * 2 +  9,	GSS_OUTPUT_LOW	}, /* LCD_DE */ 
	{32 * 2 + 10,	GSS_OUTPUT_LOW	}, /* LCD_G */ 
	{32 * 2 + 11,	GSS_OUTPUT_LOW	}, /* LCD_G */ 
	{32 * 2 + 12,	GSS_OUTPUT_LOW	}, /* LCD_G */ 
	{32 * 2 + 13,	GSS_OUTPUT_LOW	}, /* LCD_G */ 
	{32 * 2 + 14,	GSS_OUTPUT_LOW	}, /* LCD_G */ 
	{32 * 2 + 15,	GSS_OUTPUT_LOW	}, /* LCD_G */ 
	{32 * 2 + 16,	GSS_OUTPUT_LOW	}, /* LCD_G */ 
	{32 * 2 + 17,	GSS_OUTPUT_LOW	}, /* LCD_G */ 
	{32 * 2 + 18,	GSS_OUTPUT_LOW 	}, /* LCD_HS */ 
	{32 * 2 + 19,	GSS_OUTPUT_LOW 	}, /* LCD_VS */ 
	{32 * 2 + 20,	GSS_OUTPUT_LOW 	}, /* LCD_R */ 
	{32 * 2 + 21,	GSS_OUTPUT_LOW 	}, /* LCD_R */ 
	{32 * 2 + 22,	GSS_OUTPUT_LOW 	}, /* LCD_R */ 
	{32 * 2 + 23,	GSS_OUTPUT_LOW 	}, /* LCD_R */ 
	{32 * 2 + 24,	GSS_OUTPUT_LOW 	}, /* LCD_R */ 
	{32 * 2 + 25,	GSS_OUTPUT_LOW	}, /* LCD_R */ 
	{32 * 2 + 26,	GSS_OUTPUT_LOW	}, /* LCD_R */ 
	{32 * 2 + 27,	GSS_OUTPUT_LOW	}, /* LCD_R */ 
	{32 * 2 + 28,	GSS_OUTPUT_HIGH	}, /* UART2_RXD */
	{32 * 2 + 29,	GSS_OUTPUT_HIGH	}, /* FVDD_EN */
	{32 * 2 + 30,	GSS_INPUT_PULL	}, /* UART2_TXD */ 
	{32 * 2 + 31,	GSS_INPUT_PULL	}, /* NC */
	/* GPIO Group - D */
	{32 * 3 +  0,	GSS_INPUT_NOPULL}, /* PCM_DO WIFI*/ 
	{32 * 3 +  1,	GSS_INPUT_NOPULL}, /* PCM_SYN WIFI*/
	{32 * 3 +  2,	GSS_INPUT_NOPULL}, /* PCM_CLK WIFI*/ 
	{32 * 3 +  3,	GSS_INPUT_NOPULL}, /* PCM_DI WIFI*/
	{32 * 3 +  4,	GSS_INPUT_NOPULL}, /* I2C2_SCK */
	{32 * 3 +  5,	GSS_INPUT_NOPULL}, /* I2C2_SDA */
	{32 * 3 +  6,	GSS_INPUT_NOPULL}, /* I2C3_SDA */
	{32 * 3 +  7,	GSS_INPUT_NOPULL}, /* I2C3_SCK */
	{32 * 3 +  8,	GSS_INPUT_PULL	}, /* NC */  
	{32 * 3 +  9,	GSS_INPUT_PULL	}, /* NC */  
	{32 * 3 + 10,	GSS_INPUT_PULL	}, /* NC */  
	{32 * 3 + 11,	GSS_INPUT_PULL	}, /* NC */  
	{32 * 3 + 12,	GSS_OUTPUT_LOW	}, /* I2S_BCLK */
	{32 * 3 + 13,	GSS_OUTPUT_LOW	}, /* I2S_WS  */
	{32 * 3 + 14,	GSS_IGNORE	}, /* CLK32 */ 
	{32 * 3 + 15,	GSS_INPUT_PULL	}, /* NC */  
	{32 * 3 + 16,	GSS_INPUT_PULL	}, /* NC */  
	{32 * 3 + 17,	GSS_INPUT_NOPULL}, /* BOOT_SEL*/
	{32 * 3 + 18,	GSS_INPUT_NOPULL}, /* BOOT_SEL*/
	{32 * 3 + 19,	GSS_INPUT_NOPULL}, /* BOOT_SEL */
	{32 * 3 + 20,	GSS_OUTPUT_LOW	}, /* MSC1_DATA */ 
	{32 * 3 + 21,	GSS_OUTPUT_LOW	}, /* MSC1_DATA */ 
	{32 * 3 + 22,	GSS_OUTPUT_LOW	}, /* MSC1_DATA */ 
	{32 * 3 + 23,	GSS_OUTPUT_LOW	}, /* MSC1_DATA */ 
	{32 * 3 + 24,	GSS_OUTPUT_LOW	}, /* MSC1_C */
	{32 * 3 + 25,	GSS_OUTPUT_LOW	}, /* MSC1_C */
	{32 * 3 + 26,   GSS_INPUT_NOPULL}, /* UART1_RXD WIFI*/
	{32 * 3 + 27,   GSS_INPUT_NOPULL}, /* UART1_CTS_N WIFI*/
	{32 * 3 + 28,   GSS_INPUT_NOPULL}, /* UART1_TXD WIFI*/
	{32 * 3 + 29,   GSS_INPUT_NOPULL}, /* UART1_RTS_N WIFI*/
	{32 * 3 + 30,	GSS_INPUT_NOPULL}, /* I2C0_SDA */ 
	{32 * 3 + 31,	GSS_INPUT_NOPULL}, /* I2C0_SCK */ 
	
	/* GPIO Group - E */
	{32 * 4 +  0,	GSS_INPUT_PULL	}, /* NC */  
	{32 * 4 +  1,	GSS_OUTPUT_LOW	}, /* LCD_PWM */
	{32 * 4 +  2,	GSS_OUTPUT_LOW	}, /* LCD_RESET */ 
	{32 * 4 +  3,	GSS_OUTPUT_LOW	}, /* CIM_RESET */ 
	{32 * 4 +  4,	GSS_OUTPUT_LOW	}, /* CIM_EN */
	{32 * 4 +  5,	GSS_OUTPUT_LOW	}, /* I2S_MCLK */ 
	{32 * 4 +  6,	GSS_OUTPUT_LOW	}, /* HDMI_RST_N */ 
	{32 * 4 +  7,	GSS_OUTPUT_LOW	}, /* I2S_D0/SPDIF */ 
	{32 * 4 +  8,	GSS_INPUT_PULL	}, /* NC */  
	{32 * 4 +  9,	GSS_OUTPUT_HIGH	}, /* SD1_VCC_EN */ 
	{32 * 4 + 10,	GSS_IGNORE	}, /* DRVVBUS */ 
	{32 * 4 + 11,	GSS_INPUT_NOPULL}, /* GYROSCOPE_INT2 */ 
	{32 * 4 + 12,	GSS_INPUT_NOPULL}, /* GYROSCOPE_INT1 */ 
	{32 * 4 + 15,	GSS_OUTPUT_LOW	}, /* SSI_CLK */ 
	{32 * 4 + 16,	GSS_OUTPUT_LOW	}, /* SSI_CE0 */ 
	{32 * 4 + 17,	GSS_OUTPUT_LOW	}, /* SSI_DT */ 
	{32 * 4 + 18,	GSS_INPUT_NOPULL}, /* SENSOR_INT2 */ 
	{32 * 4 + 19,	GSS_IGNORE	}, /* PMU_IRQ_N */ 
	{32 * 4 + 20,	GSS_INPUT_NOPULL}, /* SD_D0 WIFI */
	{32 * 4 + 21,	GSS_INPUT_NOPULL}, /* SD_D1 WIFI */
	{32 * 4 + 22,	GSS_INPUT_NOPULL}, /* SD_D2 WIFI */
	{32 * 4 + 23,	GSS_INPUT_NOPULL}, /* SD_D3 WIFI */
	{32 * 4 + 24,	GSS_INPUT_NOPULL}, /* KEY0 */ 
	{32 * 4 + 25,	GSS_INPUT_NOPULL}, /* KEY1 */ 
	{32 * 4 + 26,	GSS_INPUT_PULL	}, /* POWER_IND_N ??*/  
	{32 * 4 + 27,	GSS_INPUT_PULL	}, /* NC */  
	{32 * 4 + 28,	GSS_INPUT_NOPULL}, /* SDIO_CLK/WIFI_SD_CLK */
	{32 * 4 + 29,	GSS_INPUT_NOPULL}, /* SD_CMD WIFI */
	{32 * 4 + 30,	GSS_INPUT_NOPULL}, /* I2C2_SDA */ 
	{32 * 4 + 31,	GSS_INPUT_NOPULL}, /* I2C2_SCK */ 
	/* GPIO Group - F */
	{32 * 5 +  0,   GSS_OUTPUT_LOW	}, /* LCD_VCC_EN */
	{32 * 5 +  1,	GSS_INPUT_PULL	}, /* NC */  
	{32 * 5 +  2,	GSS_IGNORE	}, /* CHARGE_DET_N */ 
	{32 * 5 +  3,	GSS_IGNORE	}, /* HP_MUTE */ 
	{32 * 5 +  4,	GSS_IGNORE	}, /* WL_BT_REG_ON */ 
	{32 * 5 +  5,	GSS_IGNORE	}, /* DC_DETE */ 
	{32 * 5 +  6,	GSS_IGNORE	}, /* BT_INT */ 
	{32 * 5 +  7,	GSS_INPUT_NOPULL}, /* LS_GS1 */ 
	{32 * 5 +  8,	GSS_IGNORE	}, /* BT_WAKE */ 
	{32 * 5 +  9,	GSS_IGNORE	}, /* WL_INT */ 
	{32 * 5 + 10,	GSS_IGNORE	}, /* BT/WLAN_PW_EN */ 
	{32 * 5 + 11,	GSS_IGNORE	}, /* WL_WAKE */ 
	{32 * 5 + 12,	GSS_INPUT_NOPULL}, /* HDMI_INT_N */ 
	{32 * 5 + 13,	GSS_INPUT_NOPULL}, /* SENSOR_INT1 */ 
	{32 * 5 + 14,	GSS_INPUT_NOPULL}, /* LS_GS2 */ 
	{32 * 5 + 16,	GSS_OUTPUT_LOW	}, /* LCD_L/R */ 
	{32 * 5 + 17,	GSS_OUTPUT_LOW	}, /* LCD_U/D */ 
	{32 * 5 + 18,	GSS_IGNORE	}, /* ID */ 
	{32 * 5 + 19,	GSS_INPUT_PULL	}, /* NC */  
	{32 * 5 + 20,	GSS_IGNORE	}, /* AMPEN */ 
	{32 * 5 + 21,	GSS_IGNORE	}, /* JD */ 
	{32 * 5 + 22,	GSS_OUTPUT_LOW	}, /* CIM_VCC_EN */ 
	/* GPIO Group Set End */
	{GSS_TABLET_END,GSS_TABLET_END	}
};

struct wakeup_key_s {
	int gpio;       /* gpio pin number */
};

/* add wakeup keys here */
static struct wakeup_key_s wakeup_key[] = {
	{
		.gpio = GPIO_ENDCALL,
	},
	{
		.gpio = GPIO_SD1_CD_N,
	},
	{
		.gpio = GPIO_OTG_ID_PIN,
	},
	{
		.gpio = GPIO_PMU_IRQ_PIN,
	},
};

void wakeup_key_setup(void)
{
	int i;
	int num = sizeof(wakeup_key) / sizeof(wakeup_key[0]);

	for(i = 0; i < num; i++) {
		__gpio_ack_irq(wakeup_key[i].gpio);
		__gpio_unmask_irq(wakeup_key[i].gpio);
		__intc_unmask_irq(IRQ_GPIO0 - (wakeup_key[i].gpio/32));  /* unmask IRQ_GPIOn */
	}
}
/* Power off board voltage */
void board_power_off(void)
{
	act8600_output_enable(ACT8600_OUT4, ACT8600_OUT_OFF);
	act8600_output_enable(ACT8600_OUT5, ACT8600_OUT_OFF);
	act8600_output_enable(ACT8600_OUT6, ACT8600_OUT_OFF);
	act8600_output_enable(ACT8600_OUT7, ACT8600_OUT_OFF);
	act8600_output_enable(ACT8600_OUT8, ACT8600_OUT_OFF);
}
