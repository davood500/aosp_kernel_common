/*
 * drivers/video/panels/include/jz_AT070TN93.h -- Ingenic LCD driver
 */
#ifndef __AT070TN93_H
#define __AT070TN93_H

/* AT070TN93 LCD need special pin */
struct lcd_board_pin_info {
	unsigned int LCD_DITHB_PIN;/*dither function*/
	unsigned int LCD_UD_PIN;
	unsigned int LCD_LR_PIN;
	unsigned int LCD_MODE_PIN;
	unsigned int LCD_RESET_PIN;
	unsigned int LCD_VCC_EN;
	unsigned int LCD_DE_PIN;
	unsigned int LCD_VSYNC_PIN;
	unsigned int LCD_HSYNC_PIN;
	unsigned int LCD_POWERON;
};

#endif /* __AT070TN93_H */
