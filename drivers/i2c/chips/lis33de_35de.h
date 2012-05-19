/* include/linux/lis35de.h
 *
 *  Copyright (C) 2010 Ingenic Semiconductor, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef _LINUX_LIS33DE_35DE_H
#define _LINUX_LIS33DE_35DE_H


/* lis35de register definitions */
enum lis35de_regs {
	LIS35DE_CTRL_REG1	= 0x20,
	LIS35DE_CTRL_REG2	= 0x21,
	LIS35DE_CTRL_REG3	= 0x22,  //Interrupt register 
	LIS35DE_HP_FILTER_RESET = 0x23,
	LIS35DE_STATUS_REG	= 0x27,
	LIS35DE_OUT_X		= 0x29,
	LIS35DE_OUT_Y		= 0x2B,
	LIS35DE_OUT_Z		= 0x2D,
	LIS35DE_FF_WU_CFG_1 	= 0x30,
	LIS35DE_FF_WU_SRC_1 	= 0x31,
	LIS35DE_FF_WU_THS_1 	= 0x32,
	LIS35DE_FF_WUDURATION_1 = 0x33,
	LIS35DE_FF_WU_CFG_2 	= 0x34,
	LIS35DE_FF_WU_SRC_2 	= 0x35,
	LIS35DE_FF_WU_THS_2 	= 0x36,
	LIS35DE_FF_WUDURATION_2 = 0x37,
	LIS35DE_CLICK_CFG	= 0x38,
	LIS35DE_CLICK_SRC	= 0x39,
	LIS35DE_CLICK_THSY_X	= 0x3B,
	LIS35DE_CLICK_THSZ	= 0x3c,
	LIS35DE_CLICK_TL	= 0x3d,
	LIS35DE_CLICK_LAT	= 0x3e,
	LIS35DE_CLICK_WIN	= 0x3f,
};

/* g-range */
enum lis35de_range {
	LIS_RANGE_8G	= 0x0,	/* sensitivity: 16 LSB/g */
	LIS_RANGE_2G	= 0x1,	/*		64 LSB/g */
};

/* digital filter band width */
enum lis35de_bandwidth {
	LIS_BW_100HZ	= 0,	/* 100Hz */
	LIS_BW_400HZ	= 1,	/* 400Hz */
};

/* working mode */
enum lis35de_mode {
	LIS35DE_MODE_STANDBY		= 0x0,
	LIS35DE_MODE_ACTIVE		= 0x1,
};



#endif /* _LINUX_LIS35DE_H */

