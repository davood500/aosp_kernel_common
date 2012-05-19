/*
 * chip-gpio.h
 * JZ4760 GPIO register definition
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 *
 * Author: whxu@ingenic.cn
 */

#ifndef __CHIP_GPIO_H__
#define __CHIP_GPIO_H__


/*
 * General purpose I/O port module(GPIO) address definition
 */
#define	GPIO_BASE	0xb0010000  

/* GPIO group offset */
#define GPIO_GOS	0x100

/* Each group address */
#define GPIO_BASEA	(GPIO_BASE + (0) * GPIO_GOS)
#define GPIO_BASEB	(GPIO_BASE + (1) * GPIO_GOS)
#define GPIO_BASEC	(GPIO_BASE + (2) * GPIO_GOS)
#define GPIO_BASED	(GPIO_BASE + (3) * GPIO_GOS)
#define GPIO_BASEE	(GPIO_BASE + (4) * GPIO_GOS)
#define GPIO_BASEF	(GPIO_BASE + (5) * GPIO_GOS)


/*
 * GPIO registers offset address definition
 */
#define GPIO_PXPIN_OFFSET	(0x00)	/*  r, 32, 0x00000000 */
#define GPIO_PXINT_OFFSET	(0x10)	/*  r, 32, 0x00000000 */
#define GPIO_PXINTS_OFFSET	(0x14)  /*  w, 32, 0x???????? */
#define GPIO_PXINTC_OFFSET	(0x18)  /*  w, 32, 0x???????? */
#define GPIO_PXMSK_OFFSET	(0x20)  /*  r, 32, 0xffffffff */
#define GPIO_PXMSKS_OFFSET	(0x24)  /*  w, 32, 0x???????? */
#define GPIO_PXMSKC_OFFSET	(0x28)  /*  w, 32, 0x???????? */
#define GPIO_PXPAT1_OFFSET	(0x30)  /*  r, 32, 0x00000000 */
#define GPIO_PXPAT1S_OFFSET	(0x34)  /*  w, 32, 0x???????? */
#define GPIO_PXPAT1C_OFFSET	(0x38)  /*  w, 32, 0x???????? */
#define GPIO_PXPAT0_OFFSET	(0x40)  /*  r, 32, 0x00000000 */
#define GPIO_PXPAT0S_OFFSET	(0x44)  /*  w, 32, 0x???????? */
#define GPIO_PXPAT0C_OFFSET	(0x48)  /*  w, 32, 0x???????? */
#define GPIO_PXFLG_OFFSET	(0x50)  /*  r, 32, 0x00000000 */
#define GPIO_PXFLGC_OFFSET	(0x58)  /*  w, 32, 0x???????? */
#define GPIO_PXPEN_OFFSET	(0x70)  /*  r, 32, 0x00000000 */
#define GPIO_PXPENS_OFFSET	(0x74)  /*  w, 32, 0x???????? */
#define GPIO_PXPENC_OFFSET	(0x78)  /*  w, 32, 0x???????? */

/*
 * GPIO registers address definition
 */
#define GPIO_PXPIN(n)	(GPIO_BASE + (n)*GPIO_GOS + GPIO_PXPIN_OFFSET)
#define GPIO_PXINT(n)	(GPIO_BASE + (n)*GPIO_GOS + GPIO_PXINT_OFFSET)
#define GPIO_PXINTS(n)	(GPIO_BASE + (n)*GPIO_GOS + GPIO_PXINTS_OFFSET)
#define GPIO_PXINTC(n)	(GPIO_BASE + (n)*GPIO_GOS + GPIO_PXINTC_OFFSET)
#define GPIO_PXMSK(n)	(GPIO_BASE + (n)*GPIO_GOS + GPIO_PXMSK_OFFSET)
#define GPIO_PXMSKS(n)	(GPIO_BASE + (n)*GPIO_GOS + GPIO_PXMSKS_OFFSET)
#define GPIO_PXMSKC(n)	(GPIO_BASE + (n)*GPIO_GOS + GPIO_PXMSKC_OFFSET)
#define GPIO_PXPAT1(n)	(GPIO_BASE + (n)*GPIO_GOS + GPIO_PXPAT1_OFFSET)
#define GPIO_PXPAT1S(n)	(GPIO_BASE + (n)*GPIO_GOS + GPIO_PXPAT1S_OFFSET)
#define GPIO_PXPAT1C(n)	(GPIO_BASE + (n)*GPIO_GOS + GPIO_PXPAT1C_OFFSET)
#define GPIO_PXPAT0(n)	(GPIO_BASE + (n)*GPIO_GOS + GPIO_PXPAT0_OFFSET)
#define GPIO_PXPAT0S(n)	(GPIO_BASE + (n)*GPIO_GOS + GPIO_PXPAT0S_OFFSET)
#define GPIO_PXPAT0C(n)	(GPIO_BASE + (n)*GPIO_GOS + GPIO_PXPAT0C_OFFSET)
#define GPIO_PXFLG(n)	(GPIO_BASE + (n)*GPIO_GOS + GPIO_PXFLG_OFFSET)
#define GPIO_PXFLGC(n)	(GPIO_BASE + (n)*GPIO_GOS + GPIO_PXFLGC_OFFSET)
#define GPIO_PXPEN(n)	(GPIO_BASE + (n)*GPIO_GOS + GPIO_PXPEN_OFFSET)
#define GPIO_PXPENS(n)	(GPIO_BASE + (n)*GPIO_GOS + GPIO_PXPENS_OFFSET)
#define GPIO_PXPENC(n)	(GPIO_BASE + (n)*GPIO_GOS + GPIO_PXPENC_OFFSET)


/*  */
#define GPIO_PORT_NUM   6
#define MAX_GPIO_NUM	192
#define GPIO_WAKEUP     (30)


#ifndef __MIPS_ASSEMBLER

//n = 0,1,2,3,4,5 (PORTA, PORTB, PORTC, PORTD, PORTE, PORTF)
#define REG_GPIO_PXPIN(n)	REG32(GPIO_PXPIN(n))
#define REG_GPIO_PXINT(n)	REG32(GPIO_PXINT(n))
#define REG_GPIO_PXINTS(n)	REG32(GPIO_PXINTS(n))
#define REG_GPIO_PXINTC(n)	REG32(GPIO_PXINTC(n))
#define REG_GPIO_PXMSK(n)	REG32(GPIO_PXMSK(n))
#define REG_GPIO_PXMSKS(n)	REG32(GPIO_PXMSKS(n))
#define REG_GPIO_PXMSKC(n)	REG32(GPIO_PXMSKC(n))
#define REG_GPIO_PXPAT1(n)	REG32(GPIO_PXPAT1(n))
#define REG_GPIO_PXPAT1S(n)	REG32(GPIO_PXPAT1S(n))
#define REG_GPIO_PXPAT1C(n)	REG32(GPIO_PXPAT1C(n))
#define REG_GPIO_PXPAT0(n)	REG32(GPIO_PXPAT0(n))
#define REG_GPIO_PXPAT0S(n)	REG32(GPIO_PXPAT0S(n))
#define REG_GPIO_PXPAT0C(n)	REG32(GPIO_PXPAT0C(n))
#define REG_GPIO_PXFLG(n)	REG32(GPIO_PXFLG(n))
#define REG_GPIO_PXFLGC(n)	REG32(GPIO_PXFLGC(n))
#define REG_GPIO_PXPEN(n)	REG32(GPIO_PXPEN(n))
#define REG_GPIO_PXPENS(n)	REG32(GPIO_PXPENS(n))
#define REG_GPIO_PXPENC(n)	REG32(GPIO_PXPENC(n))

/*----------------------------------------------------------------
 * p is the port number (0,1,2,3,4,5)
 * o is the pin offset (0-31) inside the port
 * n is the absolute number of a pin (0-127), regardless of the port
 */

//----------------------------------------------------------------
// Function Pins Mode

#define __gpio_as_func0(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXINTC(p) = (1 << o);		\
	REG_GPIO_PXMSKC(p) = (1 << o);		\
	REG_GPIO_PXPAT1C(p) = (1 << o);		\
	REG_GPIO_PXPAT0C(p) = (1 << o);		\
} while (0)

#define __gpio_as_func1(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXINTC(p) = (1 << o);		\
	REG_GPIO_PXMSKC(p) = (1 << o);		\
	REG_GPIO_PXPAT1C(p) = (1 << o);		\
	REG_GPIO_PXPAT0S(p) = (1 << o);		\
} while (0)

#define __gpio_as_func2(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXINTC(p) = (1 << o);		\
	REG_GPIO_PXMSKC(p) = (1 << o);		\
	REG_GPIO_PXPAT1S(p) = (1 << o);		\
	REG_GPIO_PXPAT0C(p) = (1 << o);		\
} while (0)

#define __gpio_as_func3(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXINTC(p) = (1 << o);		\
	REG_GPIO_PXMSKC(p) = (1 << o);		\
	REG_GPIO_PXPAT1S(p) = (1 << o);		\
	REG_GPIO_PXPAT0S(p) = (1 << o);		\
} while (0)

#define __gpio_as_eth()				\
do {						\
	REG_GPIO_PXINTC(1) =  0x00000010;	\
	REG_GPIO_PXMSKC(1) = 0x00000010;	\
	REG_GPIO_PXPAT1S(1) = 0x00000010;	\
	REG_GPIO_PXPAT0C(1) = 0x00000010;	\
	REG_GPIO_PXINTC(3) =  0x3c000000;	\
	REG_GPIO_PXMSKC(3) = 0x3c000000;	\
	REG_GPIO_PXPAT1C(3) = 0x3c000000;	\
	REG_GPIO_PXPAT0S(3) = 0x3c000000;	\
	REG_GPIO_PXINTC(5) =  0x0000fff0;	\
	REG_GPIO_PXMSKC(5) = 0x0000fff0;	\
	REG_GPIO_PXPAT1C(5) = 0x0000fff0;	\
	REG_GPIO_PXPAT0C(5) = 0x0000fff0;	\
} while (0)

/*
 * UART0_TxD, UART0_RxD
 */
#define __gpio_as_uart0()			\
do {						\
	unsigned int bits = BIT3 | BIT0;	\
	REG_GPIO_PXINTC(5) = bits;		\
	REG_GPIO_PXMSKC(5) = bits;		\
	REG_GPIO_PXPAT1C(5) = bits;		\
	REG_GPIO_PXPAT0C(5)  = bits;		\
} while (0)

/*
 * UART0_TxD, UART0_RxD, UART0_CTS, UART0_RTS
 */
#define __gpio_as_uart0_ctsrts()		\
do {						\
	unsigned int bits = BITS_H2L(3,0);	\
	REG_GPIO_PXINTC(5) = bits;		\
	REG_GPIO_PXMSKC(5) = bits;		\
	REG_GPIO_PXPAT1C(5) = bits;		\
	REG_GPIO_PXPAT0C(5)  = bits;		\
} while (0)

/*
 * UART1_TxD, UART1_RxD
 */
#define __gpio_as_uart1()			\
do {						\
	unsigned int bits = BIT28 | BIT26;	\
	REG_GPIO_PXINTC(3) = bits;		\
	REG_GPIO_PXMSKC(3) = bits;		\
	REG_GPIO_PXPAT1C(3) = bits;		\
	REG_GPIO_PXPAT0C(3)  = bits;		\
} while (0)

/*
 * UART1_TxD, UART1_RxD, UART1_CTS, UART1_RTS
 */
#define __gpio_as_uart1_ctsrts()		\
do {						\
	unsigned int bits = BITS_H2L(29, 26);	\
	REG_GPIO_PXINTC(3) = bits;		\
	REG_GPIO_PXMSKC(3) = bits;		\
	REG_GPIO_PXPAT1C(3) = bits;		\
	REG_GPIO_PXPAT0C(3)  = bits;		\
} while (0)


/*
 * UART2_TxD, UART2_RxD
 */
#define __gpio_as_uart2()			\
do {						\
	unsigned int bits = BIT30 | BIT28;	\
	REG_GPIO_PXINTC(2) = bits;		\
	REG_GPIO_PXMSKC(2) = bits;		\
	REG_GPIO_PXPAT1C(2) = bits;		\
	REG_GPIO_PXPAT0C(2)  = bits;		\
} while (0)

/*
 * UART2_TxD, UART2_RxD, UART2_CTS, UART2_RTS
 */
#define __gpio_as_uart2_ctsrts()		\
do {						\
	unsigned int bits = BITS_H2L(31, 28);	\
	REG_GPIO_PXINTC(2) = bits;		\
	REG_GPIO_PXMSKC(2) = bits;		\
	REG_GPIO_PXPAT1C(2) = bits;		\
	REG_GPIO_PXPAT0C(2)  = bits;		\
} while (0)

/* WARNING: the folloing macro do NOT check */
/*
 * UART3_TxD, UART3_RxD
 */
#define __gpio_as_uart3()			\
do {						\
	unsigned int bits = BIT12;	        \
	REG_GPIO_PXINTC(3) = bits;		\
	REG_GPIO_PXMSKC(3) = bits;		\
	REG_GPIO_PXPAT1C(3) = bits;		\
	REG_GPIO_PXPAT0C(3)  = bits;	        \
	bits = BIT5;	                        \
	REG_GPIO_PXINTC(4) = bits;		\
	REG_GPIO_PXMSKC(4) = bits;		\
	REG_GPIO_PXPAT1C(4) = bits;		\
	REG_GPIO_PXPAT0S(4)  = bits;	        \
} while (0)
/*
 * UART3_TxD, UART3_RxD, UART3_CTS, UART3_RTS
 */
#define __gpio_as_uart3_ctsrts()		\
do {						\
	REG_GPIO_PXINTC(3) = (1 << 12);		\
	REG_GPIO_PXMSKC(3) = (1 << 12);		\
	REG_GPIO_PXPAT1C(3) = (1 << 12);		\
	REG_GPIO_PXPAT0C(3)  = (1 << 12);		\
	REG_GPIO_PXINTC(4) = 0x00000320;	\
	REG_GPIO_PXMSKC(4) = 0x00000320;	\
	REG_GPIO_PXPAT1C(4) = 0x00000320;	\
	REG_GPIO_PXPAT0S(4) = 0x00000020;      	\
	REG_GPIO_PXPAT0C(4) = 0x00000300;	\
} while (0)

/*
 * SD0 ~ SD7, CS1#, CLE, ALE, FRE#, FWE#, FRB#
 * @n: chip select number(1 ~ 6)
 */
#define __gpio_as_nand_8bit(n)						\
do {		              						\
									\
	REG_GPIO_PXINTC(0) = 0x002c00ff; /* SD0 ~ SD7, CS1#, FRE#, FWE# */ \
	REG_GPIO_PXMSKC(0) = 0x002c00ff;				\
	REG_GPIO_PXPAT1C(0) = 0x002c00ff;				\
	REG_GPIO_PXPAT0C(0) = 0x002c00ff;				\
	REG_GPIO_PXPENS(0) = 0x002c00ff;				\
									\
	REG_GPIO_PXINTC(1) = 0x0000000c; /* CLE(SA2), ALE(SA3) */	\
	REG_GPIO_PXMSKC(1) = 0x0000000c;				\
	REG_GPIO_PXPAT1C(1) = 0x0000000c;				\
	REG_GPIO_PXPAT0C(1) = 0x0000000c;				\
	REG_GPIO_PXPENS(1) = 0x0000000c;				\
									\
	REG_GPIO_PXINTC(0) = 0x00200000 << ((n)-1); /* CSn */		\
	REG_GPIO_PXMSKC(0) = 0x00200000 << ((n)-1);			\
	REG_GPIO_PXPAT1C(0) = 0x00200000 << ((n)-1);			\
	REG_GPIO_PXPAT0C(0) = 0x00200000 << ((n)-1);			\
	REG_GPIO_PXPENS(0) = 0x00200000 << ((n)-1);			\
									\
	REG_GPIO_PXINTC(0) = 0x00100000; /* FRB#(input) */		\
	REG_GPIO_PXMSKS(0) = 0x00100000;				\
	REG_GPIO_PXPAT1S(0) = 0x00100000;				\
	REG_GPIO_PXPENS(0) = 0x00100000;				\
} while (0)

#define __gpio_as_nand_16bit(n)						\
do {		              						\
									\
	REG_GPIO_PXINTC(0) = 0x002cffff; /* SD0 ~ SD15, CS1#, FRE#, FWE# */ \
	REG_GPIO_PXMSKC(0) = 0x002cffff;				\
	REG_GPIO_PXPAT1C(0) = 0x002cffff;				\
	REG_GPIO_PXPAT0C(0) = 0x002cffff;				\
	REG_GPIO_PXPENS(0) = 0x002cffff;				\
									\
	REG_GPIO_PXINTC(1) = 0x0000000c; /* CLE(SA2), ALE(SA3) */	\
	REG_GPIO_PXMSKC(1) = 0x0000000c;				\
	REG_GPIO_PXPAT1C(1) = 0x0000000c;				\
	REG_GPIO_PXPAT0C(1) = 0x0000000c;				\
	REG_GPIO_PXPENS(1) = 0x0000000c;				\
									\
	REG_GPIO_PXINTC(0) = 0x00200000 << ((n)-1); /* CSn */		\
	REG_GPIO_PXMSKC(0) = 0x00200000 << ((n)-1);			\
	REG_GPIO_PXPAT1C(0) = 0x00200000 << ((n)-1);			\
	REG_GPIO_PXPAT0C(0) = 0x00200000 << ((n)-1);			\
	REG_GPIO_PXPENS(0) = 0x00200000 << ((n)-1);			\
									\
	REG_GPIO_PXINTC(0) = 0x00100000; /* FRB#(input) */		\
	REG_GPIO_PXMSKS(0) = 0x00100000;				\
	REG_GPIO_PXPAT1S(0) = 0x00100000;				\
	REG_GPIO_PXPENS(0) = 0x00100000;					\
} while (0)

#if 0
/*
 *  SLCD
 */
#define __gpio_as_slcd_16bit() \
do {						\
	REG_GPIO_PXFUNS(2) = 0x03cff0fc;	\
	REG_GPIO_PXTRGC(2) = 0x03cff0fc;	\
	REG_GPIO_PXSELC(2) = 0x03cff0fc;    \
	REG_GPIO_PXPES(2) = 0x03cff0fc;    \
} while (0)
#endif 

/*
 * LCD_R3~LCD_R7, LCD_G2~LCD_G7, LCD_B3~LCD_B7,
 * LCD_PCLK, LCD_HSYNC, LCD_VSYNC, LCD_DE
 */
#define __gpio_as_lcd_16bit()			\
do {						\
	REG_GPIO_PXINTC(2) = 0x0f8ff3f8;	\
	REG_GPIO_PXMSKC(2) = 0x0f8ff3f8;	\
	REG_GPIO_PXPAT0C(2) = 0x0f8ff3f8;	\
	REG_GPIO_PXPAT1C(2) = 0x0f8ff3f8;		\
} while (0)

/*
 * LCD_R2~LCD_R7, LCD_G2~LCD_G7, LCD_B2~LCD_B7,
 * LCD_PCLK, LCD_HSYNC, LCD_VSYNC, LCD_DE
 */
#define __gpio_as_lcd_18bit()			\
do {						\
	REG_GPIO_PXINTC(2) = 0x0fcff3fc;	\
	REG_GPIO_PXMSKC(2) = 0x0fcff3fc;	\
	REG_GPIO_PXPAT0C(2) = 0x0fcff3fc;	\
	REG_GPIO_PXPAT1C(2) = 0x0fcff3fc;		\
} while (0)

/*
 * LCD_R0~LCD_R7, LCD_G0~LCD_G7, LCD_B0~LCD_B7,
 * LCD_PCLK, LCD_HSYNC, LCD_VSYNC, LCD_DE
 */
#define __gpio_as_lcd_24bit()			\
do {						\
	REG_GPIO_PXINTC(2) = 0x0fffffff;	\
	REG_GPIO_PXMSKC(2)  = 0x0fffffff;	\
	REG_GPIO_PXPAT0C(2) = 0x0fffffff;	\
	REG_GPIO_PXPAT1C(2) = 0x0fffffff;		\
} while (0)

/*
 * LCD_R0~LCD_R7, LCD_G0~LCD_G7, LCD_B0~LCD_B7,
 * LCD_PCLK, LCD_HSYNC, LCD_VSYNC, LCD_DE
 */
#define __gpio_clear_lcd_24bit()	\
do {					\
	REG_GPIO_PXINTC(2) = 0x0fffffff; \
	REG_GPIO_PXMSKS(2)  = 0x0fffffff; \
	REG_GPIO_PXPAT0C(2) = 0x0fffffff; \
	REG_GPIO_PXPAT1C(2) = 0x0fffffff; \
} while (0)

#define __gpio_as_lcd_r6b6_lvds_g6_pins()	\
do {						\
	unsigned int tmp;			\
	REG_GPIO_PXINTC(2) = 0x0fc001fc;	\
	REG_GPIO_PXMSKC(2) = 0x0fc001fc;	\
	REG_GPIO_PXPAT0C(2) = 0x0fc001fc;	\
	REG_GPIO_PXPAT1C(2) = 0x0fc001fc;	\
	tmp = REG_LVDS_TXCTRL;			\
	tmp |= (LVDS_TX_OD_EN | LVDS_TX_OD_S | LVDS_TX_OUT_SEL | LVDS_TX_PDB_CK | LVDS_TX_PDB); \
	REG_LVDS_TXCTRL = tmp;			\
	tmp = REG_LVDS_TXPLL0;			\
	tmp &= ~(LVDS_BG_PWD);			\
	REG_LVDS_TXPLL0 = tmp;			\
}while(0)
#if 0
/* Set data pin driver strength v: 0~7 */
#define __gpio_set_lcd_data_driving_strength(v) \
do {						\
	unsigned int d;			\
	d = v & 0x1;				\
	if(d) REG_GPIO_PXDS0S(2) = 0x0ff3fcff;	\
	else REG_GPIO_PXDS0C(2) = 0x0ff3fcff;	\
	d = v & 0x2;				\
	if(d) REG_GPIO_PXDS1S(2) = 0x0ff3fcff;	\
	else REG_GPIO_PXDS1C(2) = 0x0ff3fcff;	\
	d = v & 0x4;				\
	if(d) REG_GPIO_PXDS2S(2) = 0x0ff3fcff;	\
	else REG_GPIO_PXDS2C(2) = 0x0ff3fcff;	\
} while(0)
#endif

/*
 *  LCD_CLS, LCD_SPL, LCD_PS, LCD_REV
 */
#define __gpio_as_lcd_special()			\
do {						\
	REG_GPIO_PXINTC(2) = 0x0fffffff;	\
	REG_GPIO_PXMSKC(2) = 0x0fffffff;	\
	REG_GPIO_PXPAT0C(2) = 0x0feffbfc;	\
	REG_GPIO_PXPAT0S(2) = 0x00100403;	\
	REG_GPIO_PXPAT1C(2) = 0x0fffffff;		\
} while (0)

#if 0
/* Set HSYNC VSYNC DE driver strength v: 0~7 */
#define __gpio_set_lcd_sync_driving_strength(v) \
do {						\
	unsigned int d;				\
	d = v & 0x1;				\
	if(d) REG_GPIO_PXDS0S(2) = 0x000c0200;	\
	else REG_GPIO_PXDS0C(2) = 0x000c0200;	\
	d = v & 0x2;				\
	if(d) REG_GPIO_PXDS1S(2) = 0x000c0200;	\
	else REG_GPIO_PXDS1C(2) = 0x000c0200;	\
	d = v & 0x4;				\
	if(d) REG_GPIO_PXDS2S(2) = 0x000c0200;	\
	else REG_GPIO_PXDS2C(2) = 0x000c0200;	\
} while(0)
/* Set PCLK driver strength v: 0~7 */
#define __gpio_set_lcd_clk_driving_strength(v)	\
do {						\
	unsigned int d;				\
	d = v & 0x1;				\
	if(d) REG_GPIO_PXDS0S(2) = (1 << 8);	\
	else REG_GPIO_PXDS0C(2) = (1 << 8);	\
	d = v & 0x2;				\
	if(d) REG_GPIO_PXDS1S(2) = (1 << 8);	\
	else REG_GPIO_PXDS1C(2) = (1 << 8);	\
	d = v & 0x4;				\
	if(d) REG_GPIO_PXDS2S(2) = (1 << 8);	\
	else REG_GPIO_PXDS2C(2) = (1 << 8);	\
} while(0)


/* Set fast slew rate */
#define __gpio_set_lcd_data_fslew(n)		\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXSLS(p) = 0x0ff3fcff;		\
} while(0)

/* Set slow slew rate */
#define __gpio_set_lcd_data_sslew(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXSLC(p) = 0x0ff3fcff;		\
} while(0)

/* Set fast slew rate */
#define __gpio_set_lcd_sync_fslew(n)		\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXSLS(p) = 0x000c0200;		\
} while(0)

/* Set slow slew rate */
#define __gpio_set_lcd_sync_sslew(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXSLC(p) = 0x000c0200;		\
} while(0)

/* Set fast slew rate */
#define __gpio_set_lcd_pclk_fslew(n)		\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXSLS(p) = (1 << 8);		\
} while(0)

/* Set slow slew rate */
#define __gpio_set_lcd_pclk_sslew(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXSLC(p) = (1 << 8);		\
} while(0)


#define __gpio_as_epd()				\
do {						\
	REG_GPIO_PXFUNS(1) = 0x00011e00;	\
	REG_GPIO_PXTRGS(1) = 0x00011e00;	\
	REG_GPIO_PXSELS(1) = 0x00011e00;	\
	REG_GPIO_PXPES(1)  = 0x00011e00;	\
} while (0)
#endif 

/*
 * CIM_D0~CIM_D7, CIM_MCLK, CIM_PCLK, CIM_VSYNC, CIM_HSYNC
 */
#define __gpio_as_cim()				\
do {						\
	        REG_GPIO_PXINTC(1) = 0x0003ffc0;	\
		REG_GPIO_PXMSKC(1) = 0x0003ffc0;	\
		REG_GPIO_PXPAT1C(1) = 0x0003ffc0;	\
		REG_GPIO_PXPAT0C(1) = 0x0003ffc0;	\
} while (0)

/*
 * SDATO, SDATI, BCLK, SYNC, SCLK_RSTN(gpio sepc) or
 * SDATA_OUT, SDATA_IN, BIT_CLK, SYNC, SCLK_RESET(aic spec)
 */
#define __gpio_as_aic()		\
do {					\
	REG_GPIO_PXINTC(4) = 0x000000c0;	\
	REG_GPIO_PXMSKC(4) = 0x000000c0;	\
	REG_GPIO_PXPAT1C(4) = 0x000000c0;	\
	REG_GPIO_PXPAT0C(4) = 0x000000c0;	\
	REG_GPIO_PXINTC(5) = 0x000c0000;	\
	REG_GPIO_PXMSKC(5) = 0x000c0000;	\
	REG_GPIO_PXPAT1C(5) = 0x000c0000;	\
	REG_GPIO_PXPAT0C(5) = 0x000c0000;	\
	} while (0)

#if 0
#define __gpio_as_spdif()		\
do {					\
	REG_GPIO_PXFUNS(3) = 0x00003000;	\
	REG_GPIO_PXTRGC(3) = 0x00003000;	\
	REG_GPIO_PXSELS(3) = 0x00001000;	\
	REG_GPIO_PXSELC(3) = 0x00002000;	\
	REG_GPIO_PXPES(3)  = 0x00003000;	\
	REG_GPIO_PXFUNS(4) = 0x000038e0;	\
	REG_GPIO_PXTRGC(4) = 0x000038c0;	\
	REG_GPIO_PXTRGS(4) = 0x00000020;	\
	REG_GPIO_PXSELC(4) = 0x000038e0;	\
	REG_GPIO_PXPES(4)  = 0x000038e0;	\
} while (0)
#endif

/*
 * MSC0_CMD, MSC0_CLK, MSC0_D0 ~ MSC0_D3
 */
#define __gpio_as_msc0_pa_4bit()		\
do {						\
	REG_GPIO_PXINTC(0) = 0x00fc0000;	\
	REG_GPIO_PXMSKC(0) = 0x00fc0000;	\
	REG_GPIO_PXPAT1C(0) = 0x00fc0000;	\
	REG_GPIO_PXPAT0S(0) = 0x00ec0000;	\
	REG_GPIO_PXPAT0C(0)  = 0x00100000;	\
} while (0)

/*
 * MSC0_CMD, MSC0_CLK, MSC0_D0 ~ MSC0_D7
 */
#define __gpio_as_msc0_pe_8bit()			\
do {						\
	REG_GPIO_PXINTC(4) = 0x3ff00000;	\
	REG_GPIO_PXMSKC(4)  = 0x3ff00000;	\
	REG_GPIO_PXPAT0C(4) = 0x3ff00000;	\
	REG_GPIO_PXPAT1C(4) = 0x3ff00000;	\
} while (0)
/*
 * MSC0_CMD, MSC0_CLK, MSC0_D0 ~ MSC0_D3
 */
#define __gpio_as_msc0_pe_4bit()			\
do {						\
	REG_GPIO_PXINTC(4) = 0x30f00000;	\
	REG_GPIO_PXMSKC(4)  = 0x30f00000;	\
	REG_GPIO_PXPAT0C(4) = 0x30f00000;	\
	REG_GPIO_PXPAT1C(4) = 0x30f00000;	\
} while (0)

#if 0
#define __gpio_as_msc0_boot()			\
do {        					\
	REG_GPIO_PXFUNS(0) = 0x00ec0000;	\
	REG_GPIO_PXTRGC(0) = 0x00ec0000;	\
	REG_GPIO_PXSELS(0) = 0x00ec0000;	\
	REG_GPIO_PXPES(0)  = 0x00ec0000;	\
	REG_GPIO_PXFUNS(0) = 0x00100000;	\
	REG_GPIO_PXTRGC(0) = 0x00100000;	\
	REG_GPIO_PXSELC(0) = 0x00100000;	\
	REG_GPIO_PXPES(0)  = 0x00100000;	\
						\
} while (0)

#else

#define __gpio_as_msc0_boot()			\
do {        					\
	REG_GPIO_PXINTC(0)  = 0x00ec0000;	\
	REG_GPIO_PXMSKC(0)  = 0x00ec0000;	\
	REG_GPIO_PXPAT1C(0) = 0x00ec0000;	\
	REG_GPIO_PXPAT0S(0) = 0x00ec0000;       \
	REG_GPIO_PXPENS(0)   = 0x00ec0000;	\				\
	REG_GPIO_PXINTC(0)  = 0x00100000;	\
	REG_GPIO_PXMSKC(0)  = 0x00100000;	\
	REG_GPIO_PXPAT1C(0) = 0x00100000;	\
	REG_GPIO_PXPAT0C(0) = 0x00100000;       \
	REG_GPIO_PXPENS(0)   = 0x00100000;	\				\
						\
} while (0)

#endif




/*
 * MSC1_CMD, MSC1_CLK, MSC1_D0 ~ MSC1_D7
 */
#define __gpio_as_msc1_pe_8bit()			\
do {						\
	REG_GPIO_PXINTC(4) = 0x3ff00000;	\
	REG_GPIO_PXMSKC(4)  = 0x3ff00000;	\
	REG_GPIO_PXPAT0S(4) = 0x3ff00000;	\
	REG_GPIO_PXPAT1C(4) = 0x3ff00000;	\
	REG_GPIO_PXPENS(4) = 0x3ff00000;	\
} while (0)
/*
 * MSC1_CMD, MSC1_CLK, MSC1_D0 ~ MSC1_D3
 */
#define __gpio_as_msc1_pe_4bit()			\
do {						\
	REG_GPIO_PXINTC(4) = 0x30f00000;	\
	REG_GPIO_PXMSKC(4) = 0x30f00000;	\
	REG_GPIO_PXPAT0S(4) = 0x30f00000;	\
	REG_GPIO_PXPAT1C(4)  = 0x30f00000;	\
	REG_GPIO_PXPENS(4)  = 0x30f00000;	\
} while (0)



/*
 * MSC1_CMD, MSC1_CLK, MSC1_D0 ~ MSC1_D3
 */
#define __gpio_as_msc1_pd_4bit()			\
do {						\
	REG_GPIO_PXINTC(3) = 0x3f00000;	\
	REG_GPIO_PXMSKC(3) = 0x3f00000;	\
	REG_GPIO_PXPAT0C(3) = 0x3f00000;	\
	REG_GPIO_PXPAT1C(3)  = 0x3f00000;	\
	REG_GPIO_PXPENS(3)  = 0x3f00000;	\
} while (0)


/* Port B
 * MSC2_CMD, MSC2_CLK, MSC2_D0 ~ MSC2_D3
 */
#define __gpio_as_msc2_pb_4bit()		\
do {						\
	REG_GPIO_PXINTC(1) = 0xf0300000;	\
	REG_GPIO_PXMSKC(1) = 0xf0300000;	\
	REG_GPIO_PXPAT1C(1) = 0xf0300000;	\
	REG_GPIO_PXPAT0C(1)  = 0xf0300000;	\
	REG_GPIO_PXPENS(1)  = 0xf0300000;	\
} while (0)

/* wip : with internal pull*/
#define __gpio_as_msc2_pb_wip_4bit()		\
do {						\
	REG_GPIO_PXINTC(1) = 0xf0300000;	\
	REG_GPIO_PXMSKC(1) = 0xf0300000;	\
	REG_GPIO_PXPAT1C(1) = 0xf0300000;	\
	REG_GPIO_PXPAT0C(1)  = 0xf0300000;	\
	REG_GPIO_PXPENC(1)  = 0xf0300000;	\
} while (0)

/*
 * MSC2_CMD, MSC2_CLK, MSC2_D0 ~ MSC2_D7
 */
#define __gpio_as_msc2_pe_8bit()			\
do {						\
        REG_GPIO_PXINTC(4) = 0x3ff00000;	\
	REG_GPIO_PXMSKC(4)  = 0x3ff00000;	\
	REG_GPIO_PXPAT0C(4) = 0x3ff00000;	\
	REG_GPIO_PXPAT1S(4) = 0x3ff00000;	\
	REG_GPIO_PXPENS(4) = 0x3ff00000;	\
} while (0)
/*
 * MSC2_CMD, MSC2_CLK, MSC2_D0 ~ MSC2_D3
 */
#define __gpio_as_msc2_pe_4bit()			\
do {						\
	REG_GPIO_PXINTC(4) = 0x30f00000;	\
	REG_GPIO_PXMSKC(4) = 0x30f00000;	\
	REG_GPIO_PXPAT0C(4) = 0x30f00000;	\
	REG_GPIO_PXPAT1S(4)  = 0x30f00000;	\
	REG_GPIO_PXPENS(4)  = 0x30f00000;	\
} while (0)
#define __gpio_as_msc0_4bit	__gpio_as_msc0_pe_4bit /* default as msc0 4bit */
#define __gpio_as_msc1_4bit __gpio_as_msc1_pd_4bit /* msc1 only support 4bit */
#define __gpio_as_msc 	__gpio_as_msc0_4bit /* default as msc0 4bit */
#define __gpio_as_msc0 	__gpio_as_msc0_4bit /* msc0 default as 4bit */
#define __gpio_as_msc1 	__gpio_as_msc1_4bit /* msc1 only support 4bit */

#if 0
/*
 * TSCLK, TSSTR, TSFRM, TSFAIL, TSDI0~7
 */
#define __gpio_as_tssi_1()			\
do {						\
	REG_GPIO_PXFUNS(1) = 0x0003ffc0;	\
	REG_GPIO_PXTRGC(1) = 0x0003ffc0;	\
	REG_GPIO_PXSELS(1) = 0x0003ffc0;	\
	REG_GPIO_PXPES(1)  = 0x0003ffc0;	\
} while (0)

/*
 * TSCLK, TSSTR, TSFRM, TSFAIL, TSDI0~7
 */
#define __gpio_as_tssi_2()			\
do {						\
	REG_GPIO_PXFUNS(1) = 0xfff00000;	\
	REG_GPIO_PXTRGC(1) = 0x0fc00000;	\
	REG_GPIO_PXTRGS(1) = 0xf0300000;	\
	REG_GPIO_PXSELC(1) = 0xfff00000;	\
	REG_GPIO_PXPES(1)  = 0xfff00000;	\
} while (0)
#endif 

/*
 * TSCLK, TSSTR, TSFRM, TSFAIL, TSDI0~7
 */
#define __gpio_as_tssi()                        \
do {                                            \
        REG_GPIO_PXINTC(1) = 0xf0300000;        \
        REG_GPIO_PXMSKC(1)  = 0xf0300000;                \
        REG_GPIO_PXPAT0S(1) = 0xf0300000;       \
        REG_GPIO_PXPAT1S(1) = 0xf0300000;               \
                                                        \
        REG_GPIO_PXINTC(1) = 0x0fc00000;        \
        REG_GPIO_PXMSKC(1)  = 0x0fc00000;                \
        REG_GPIO_PXPAT0C(1) = 0x0fc00000;       \
        REG_GPIO_PXPAT1C(1) = 0x0fc00000;               \
} while (0)

/*
 * SSI_CE0, SSI_CE1, SSI_GPC, SSI_CLK, SSI_DT, SSI_DR
 */
#define __gpio_as_ssi()				\
do {						\
	REG_GPIO_PXINTC(0) = 0x002c0000; /* SSI0_CE0, SSI0_CLK, SSI0_DT	*/ \
	REG_GPIO_PXMSKC(0) = 0x002c0000;	\
	REG_GPIO_PXPAT1S(0) = 0x002c0000;	\
	REG_GPIO_PXPAT0C(0)  = 0x002c0000;	\
	REG_GPIO_PXPENS(0)  = 0x002c0000;	\
						\
	REG_GPIO_PXINTC(0) = 0x00100000; /* SSI0_DR */	\
	REG_GPIO_PXMSKC(0) = 0x00100000;	\
	REG_GPIO_PXPAT1C(0) = 0x00100000;	\
	REG_GPIO_PXPAT0S(0)  = 0x00100000;	\
	REG_GPIO_PXPENS(0)  = 0x00100000;	\
} while (0)

/*
 * SSI_CE0, SSI_CE2, SSI_GPC, SSI_CLK, SSI_DT, SSI1_DR
 */
#define __gpio_as_ssi_1()			\
do {						\
	REG_GPIO_PXINTC(1) = 0xf0300000;	\
	REG_GPIO_PXMSKC(1) = 0xf0300000;	\
	REG_GPIO_PXPAT1C(1) = 0xf0300000;	\
	REG_GPIO_PXPAT0S(1)  = 0xf0300000;	\
	REG_GPIO_PXPENS(1)  = 0xf0300000;	\
} while (0)

/* Port B
 * SSI2_CE0, SSI2_CE2, SSI2_GPC, SSI2_CLK, SSI2_DT, SSI12_DR
 */
#if 0
#define __gpio_as_ssi2_1()			\
do {						\
	REG_GPIO_PXFUNS(1) = 0xf0180000;	\
	REG_GPIO_PXTRGC(1) = 0xf0180000;	\
	REG_GPIO_PXSELS(1) = 0xf0180000;	\
	REG_GPIO_PXPES(1)  = 0xf0180000;	\
} while (0)
#endif 

#define __gpio_as_pcm0() \
do {						\
	REG_GPIO_PXINTC(3) = 0xf;	\
	REG_GPIO_PXMSKC(3) = 0xf;	\
	REG_GPIO_PXPAT1C(3) = 0xf;	\
	REG_GPIO_PXPAT0C(3)  = 0xf;	\
	REG_GPIO_PXPENS(3)  = 0xf;	\
} while (0)

#define __gpio_as_pcm1() \
do {						\
	REG_GPIO_PXINTC(5) = 0xf000;	\
	REG_GPIO_PXMSKC(5) = 0xf000;	\
	REG_GPIO_PXPAT1C(5) = 0xf000;	\
	REG_GPIO_PXPAT0S(5)  = 0xf000;	\
	REG_GPIO_PXPENS(5)  = 0xf000;	\
} while (0)
/*
 * I2C_SCK, I2C_SDA
 */

#define __gpio_as_i2c(n)		       		\
do {							\
	if(n == 2) {					\
		__gpio_as_i2c2();			\
	} else {					\
	        REG_GPIO_PXINTC(3 + (n)) = 0xC0000000;	\
		REG_GPIO_PXPAT1C(3 + (n)) = 0xC0000000;	\
		REG_GPIO_PXPAT0C(3 + (n)) = 0xC0000000;	\
		REG_GPIO_PXPENC(3 + (n)) = 0xC0000000;	\
		REG_GPIO_PXMSKC(3 + (n)) = 0xC0000000;	\
	}						\
} while (0)

#define __gpio_as_i2c2()				\
	do {						\
		REG_GPIO_PXINTC(5) = 0x00030000;	\
		REG_GPIO_PXPAT1S(5) = 0x00030000;	\
		REG_GPIO_PXPAT0C(5) = 0x00030000;	\
		REG_GPIO_PXPENC(5) = 0x00030000;	\
		REG_GPIO_PXMSKC(5) = 0x00030000;	\
	} while(0)

#define __gpio_as_i2c_outlf(n,m)		       	\
do {							\
	if(n == 2) {					\
		__gpio_as_i2c2_outlf(m);		\
	} else {					\
	        REG_GPIO_PXINTC(3 + (n)) = 0xC0000000;	\
		REG_GPIO_PXPAT1C(3 + (n)) = 0xC0000000;	\
		REG_GPIO_PXPAT0C(3 + (n)) = 0xC0000000;	\
		REG_GPIO_PXPENS(3 + (n)) = 0xC0000000;	\
		REG_GPIO_PXMSKS(3 + (n)) = 0xC0000000;	\
		udelay(m);				\
		REG_GPIO_PXPENC(3 + (n)) = 0xC0000000;	\
		REG_GPIO_PXMSKC(3 + (n)) = 0xC0000000;	\
	}						\
} while (0)

#define __gpio_as_i2c2_outlf(m)				\
	do {						\
		REG_GPIO_PXINTC(5) = 0x00030000;	\
		REG_GPIO_PXPAT1S(5) = 0x00030000;	\
		REG_GPIO_PXPAT0C(5) = 0x00030000;	\
		REG_GPIO_PXPENS(5) = 0x00030000;	\
		REG_GPIO_PXMSKS(5) = 0x00030000;	\
		udelay(m);				\
		REG_GPIO_PXPENC(5) = 0x00030000;	\
		REG_GPIO_PXMSKC(5) = 0x00030000;	\
	} while(0)
/*
 * PWM0
 */
#define __gpio_as_pwm0()			\
do {						\
	REG_GPIO_PXINTC(4) = 0x1;	\
	REG_GPIO_PXMSKC(4) = 0x1;	\
	REG_GPIO_PXPAT1C(4) = 0x1;	\
	REG_GPIO_PXPAT0C(4) = 0x1;		\
	REG_GPIO_PXPENS(4) = 0x1;		\
} while (0)

/*
 * PWM1
 */
#define __gpio_as_pwm1()			\
do {						\
	REG_GPIO_PXINTC(4) = 0x2;	        \
	REG_GPIO_PXMSKC(4) = 0x2;		\
	REG_GPIO_PXPAT1C(4) = 0x2;		\
	REG_GPIO_PXPAT0C(4) = 0x2;		\
	REG_GPIO_PXPENS(4) = 0x2;		\
} while (0)

/*
 * PWM2
 */
#define __gpio_as_pwm2()		\
do {					\
	REG_GPIO_PXINTC(4) = 0x4;	\
	REG_GPIO_PXMSKC(4) = 0x4;	\
	REG_GPIO_PXPAT1C(4) = 0x4;	\
	REG_GPIO_PXPAT0C(4) = 0x4;	\
	REG_GPIO_PXPENS(4) = 0x4;	\
} while (0)

/*
 * PWM3
 */
#define __gpio_as_pwm3()		\
do {					\
	REG_GPIO_PXINTC(4) = 0x8;	\
	REG_GPIO_PXMSKC(4) = 0x8;	\
	REG_GPIO_PXPAT1C(4) = 0x8;	\
	REG_GPIO_PXPAT0C(4) = 0x8;	\
	REG_GPIO_PXPENS(4) = 0x8;	\
} while (0)

/*
 * PWM4
 */
#define __gpio_as_pwm4()		\
do {					\
	REG_GPIO_PXINTC(4) = 0x10;	\
	REG_GPIO_PXMSKC(4) = 0x10;	\
	REG_GPIO_PXPAT1C(4) = 0x10;	\
	REG_GPIO_PXPAT0C(4) = 0x10;	\
	REG_GPIO_PXPENS(4) = 0x10;	\
} while (0)

/*
 * PWM5
 */
#define __gpio_as_pwm5()		\
do {					\
	REG_GPIO_PXINTC(4) = 0x20;	\
	REG_GPIO_PXMSKC(4) = 0x20;	\
	REG_GPIO_PXPAT1C(4) = 0x20;	\
	REG_GPIO_PXPAT0C(4) = 0x20;	\
	REG_GPIO_PXPENS(4) = 0x20;	\
} while (0)

/*
 * n = 0 ~ 5
 */
#define __gpio_as_pwm(n)	__gpio_as_pwm##n()

/*
 * OWI - PA29 function 1
 */
#define __gpio_as_owi()				\
do {						\
	REG_GPIO_PXINTC(0) = 0x20000000;	\
	REG_GPIO_PXMSKC(0) = 0x20000000;	\
	REG_GPIO_PXPAT1C(0) = 0x20000000;	\
	REG_GPIO_PXPAT0S(0) = 0x20000000;	\
	REG_GPIO_PXPENS(0) = 0x20000000;	\
} while (0)

/*
 * SCC - PD08 function 0
 *       PD09 function 0
 */
#define __gpio_as_scc()				\
do {						\
	REG_GPIO_PXINTC(3) = 0xc0000300;	\
	REG_GPIO_PXMSKC(3) = 0xc0000300;	\
	REG_GPIO_PXPAT1C(3) = 0xc0000300;	\
	REG_GPIO_PXPAT0C(3) = 0xc0000300;	\
	REG_GPIO_PXPENS(3) = 0xc0000300;	\
} while (0)

#define __gpio_as_otg_drvvbus()	\
do {	\
	REG_GPIO_PXINTC(4) = (1 << 10);		\
	REG_GPIO_PXMSKC(4) = (1 << 10);		\
	REG_GPIO_PXPAT1C(4) = (1 << 10);	\
	REG_GPIO_PXPAT0C(4) = (1 << 10);	\
	REG_GPIO_PXPENS(4) = (1 << 10);		\
} while (0)
 
//-------------------------------------------
// GPIO or Interrupt Mode

#define __gpio_get_port(p)	(REG_GPIO_PXPIN(p))

#define __gpio_port_as_output(p, o)		\
do {						\
    REG_GPIO_PXINTC(p) = (1 << (o));		\
    REG_GPIO_PXMSKS(p) = (1 << (o));		\
    REG_GPIO_PXPAT1C(p) = (1 << (o));		\
    REG_GPIO_PXPENS(p) = (1 << o);		\
} while (0)

#define __gpio_port_as_input(p, o)		\
do {						\
    REG_GPIO_PXINTC(p) = (1 << (o));		\
    REG_GPIO_PXMSKS(p) = (1 << (o));		\
    REG_GPIO_PXPAT1S(p) = (1 << (o));		\
    REG_GPIO_PXPENS(p) = (1 << o);		\
} while (0)

#define __gpio_as_output(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	__gpio_port_as_output(p, o);		\
} while (0)

#define __gpio_as_input(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	__gpio_port_as_input(p, o);		\
} while (0)

#define __gpio_set_pin(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXPAT0S(p) = (1 << o);		\
} while (0)

#define __gpio_clear_pin(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXPAT0C(p) = (1 << o);		\
} while (0)

#define __gpio_get_pin(n)			\
({						\
	unsigned int p, o, v;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	if (__gpio_get_port(p) & (1 << o))	\
		v = 1;				\
	else					\
		v = 0;				\
	v;					\
})

#define __gpio_as_irq_high_level(n)		\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXINTS(p) = (1 << o);		\
	REG_GPIO_PXMSKC(p) = (1 << o);		\
	REG_GPIO_PXPAT1C(p) = (1 << o);		\
	REG_GPIO_PXPAT0S(p) = (1 << o);		\
	REG_GPIO_PXPENS(p) = (1 << o);		\
} while (0)

#define __gpio_as_irq_low_level(n)		\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXINTS(p) = (1 << o);	\
	REG_GPIO_PXMSKC(p) = (1 << o);	\
	REG_GPIO_PXPAT1C(p) = (1 << o);	\
	REG_GPIO_PXPAT0C(p) = (1 << o);	\
	REG_GPIO_PXPENS(p) = (1 << o);		\
} while (0)

#define __gpio_as_irq_rise_edge(n)		\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXINTS(p) = (1 << o);		\
	REG_GPIO_PXMSKC(p) = (1 << o);		\
	REG_GPIO_PXPAT1S(p) = (1 << o);		\
	REG_GPIO_PXPAT0S(p) = (1 << o);		\
	REG_GPIO_PXPENS(p) = (1 << o);		\
} while (0)

#define __gpio_as_irq_fall_edge(n)		\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXINTS(p) = (1 << o);  \
        REG_GPIO_PXMSKC(p) = (1 << o);   \
        REG_GPIO_PXPAT1S(p) = (1 << o); \
        REG_GPIO_PXPAT0C(p) = (1 << o); \
	REG_GPIO_PXPENS(p) = (1 << o);		\
} while (0)

#define __gpio_mask_irq(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXMSKS(p) = (1 << o);		\
} while (0)

#define __gpio_unmask_irq(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXMSKC(p) = (1 << o);		\
} while (0)

#define __gpio_ack_irq(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXFLGC(p) = (1 << o);		\
} while (0)

#define __gpio_get_irq()			\
({						\
	unsigned int p, i, tmp, v = 0;		\
	for (p = 5; p >= 0; p--) {		\
		tmp = REG_GPIO_PXFLG(p);	\
		for (i = 0; i < 32; i++)	\
			if (tmp & (1 << i))	\
				v = (32*p + i);	\
	}					\
	v;					\
})

#define __gpio_group_irq(n)			\
({						\
	register int tmp, i;			\
	tmp = REG_GPIO_PXFLG(n);	        \
	for (i=31;i>=0;i--)			\
		if (tmp & (1 << i))		\
			break;			\
	i;					\
})

#define __gpio_enable_pull(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXPENC(p) = (1 << o);		\
} while (0)

#define __gpio_disable_pull(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXPENS(p) = (1 << o);		\
} while (0)

#define __gpio_is_irq(n) \
({ \
	unsigned int p, o, v;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	if (REG_GPIO_PXINT(p) & (1 << o))	\
		v = 1;				\
	else					\
		v = 0;				\
	v;					\
})

#if 0
/* Set pin driver strength v: 0~7 */
#define __gpio_set_driving_strength(n, v)	\
do {						\
	unsigned int p, o, d;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	d = v & 0x1;				\
	if(d) REG_GPIO_PXDS0S(p) = (1 << o);	\
	else REG_GPIO_PXDS0C(p) = (1 << o);	\
	d = v & 0x2;				\
	if(d) REG_GPIO_PXDS1S(p) = (1 << o);	\
	else REG_GPIO_PXDS1C(p) = (1 << o);	\
	d = v & 0x4;				\
	if(d) REG_GPIO_PXDS2S(p) = (1 << o);	\
	else REG_GPIO_PXDS2C(p) = (1 << o);	\
} while(0)

/* Set fast slew rate */
#define __gpio_set_fslew(n)	\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXSLS(p) = (1 << o);	\
} while(0)

/* Set slow slew rate */
#define __gpio_set_sslew(n)	\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXSLC(p) = (1 << o);	\
} while(0)
#endif

//////////////////////////////////////////////////////////////

#define __gpio_group_as_output_low(n,v)		\
do {						\
    REG_GPIO_PXINTC(n) = v;			\
    REG_GPIO_PXMSKS(n) = v;			\
    REG_GPIO_PXPAT1C(n) = v;			\
    REG_GPIO_PXPAT0C(n) = v;			\
    REG_GPIO_PXPENS(n) = v;			\
}while(0)

#define __gpio_group_as_output_high(n,v)	\
do {						\
    REG_GPIO_PXINTC(n) = v;			\
    REG_GPIO_PXMSKS(n) = v;			\
    REG_GPIO_PXPAT1C(n) = v;			\
    REG_GPIO_PXPAT0S(n) = v;			\
    REG_GPIO_PXPENS(n) = v;			\
}while(0)

#define __gpio_group_as_input_pull(n,v)		\
do {						\
    REG_GPIO_PXINTC(n) = v;			\
    REG_GPIO_PXMSKS(n) = v;			\
    REG_GPIO_PXPAT1S(n) = v;			\
    REG_GPIO_PXPENC(n) = v;			\
}while(0)

#define __gpio_group_as_input_nopull(n,v)	\
do {						\
    REG_GPIO_PXINTC(n) = v;			\
    REG_GPIO_PXMSKS(n) = v;			\
    REG_GPIO_PXPAT1S(n) = v;			\
    REG_GPIO_PXPENS(n) = v;			\
}while(0)


//////////////////////////////////////////////////////////////
#endif /* __MIPS_ASSEMBLER */

#endif /* __CHIP_GPIO_H__ */
