/* include/linux/mma7455l.h
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

#ifndef _LINUX_MMA7455L_H
#define _LINUX_MMA7455L_H

#define MMA7455L_NAME		"mma7455l"
#define CALIBRATION_COUNT	3

/* mma7455l register definitions */
enum mma7455l_regs {
	MMA7455L_REG_XOUTL	= 0x00, /* 10 bits output value X LSB */
	MMA7455L_REG_XOUTH	= 0x01, /* 10 bits output value X MSB */
	MMA7455L_REG_YOUTL	= 0x02,
	MMA7455L_REG_YOUTH	= 0x03,
	MMA7455L_REG_ZOUTL	= 0x04,
	MMA7455L_REG_ZOUTH	= 0x05,
	MMA7455L_REG_XOUT8	= 0x06,
	MMA7455L_REG_YOUT8	= 0x07,
	MMA7455L_REG_ZOUT8	= 0x08,
	MMA7455L_REG_STATUS	= 0x09,
	MMA7455L_REG_DETSRC	= 0x0A,	/* Detection Source Register	*/
	MMA7455L_REG_TOUT	= 0x0B, /* "Temperature output value" (Optional) */
	MMA7455L_REG_RESERVED1	= 0x0C,
	MMA7455L_REG_I2CAD	= 0x0D, /* I2C device address		*/
	MMA7455L_REG_USRINFO	= 0x0E, /* User information (Optional)	*/
	MMA7455L_REG_WHOAMI	= 0x0F, /* Who am I value (Optional)	*/
	MMA7455L_REG_XOFFL	= 0x10, /* Offset drift value(LSB)	*/
	MMA7455L_REG_XOFFH	= 0x11, /* Offset drift value(MSB)	*/
	MMA7455L_REG_YOFFL	= 0x12,
	MMA7455L_REG_YOFFH	= 0x13,
	MMA7455L_REG_ZOFFL	= 0x14,
	MMA7455L_REG_ZOFFH	= 0x15,
	MMA7455L_REG_MCTRL	= 0x16, /* Mode control			*/
	MMA7455L_REG_INTRST	= 0x17, /* Interrupt latch reset	*/
	MMA7455L_REG_CTRL1	= 0x18,
	MMA7455L_REG_CTRL2	= 0x19,
	MMA7455L_REG_LDTH	= 0x1A, /* Level detection threshold limit value */
	MMA7455L_REG_PDTH	= 0x1B, /* Pulse detection threshold limit value */
	MMA7455L_REG_PW		= 0x1C, /* Pulse duration value 	*/
	MMA7455L_REG_LT		= 0x1D, /* Latency time value 		*/
	MMA7455L_REG_TW		= 0x1E, /* Time window for 2nd pulse value */
	MMA7455L_REG_RESERVED2	= 0x0F,
};

/* g-range */
enum mma7455l_range {
	MMA_RANGE_8G	= 0x0,	/* sensitivity: 16 LSB/g */
	MMA_RANGE_2G	= 0x1,	/*		64 LSB/g */
	MMA_RANGE_4G	= 0x2,	/*		32 LSB/g */
};

/* digital filter band width */
enum mma7455l_bandwidth {
	MMA_BW_625HZ	= 0,	/* 62.5Hz */
	MMA_BW_125HZ	= 1,	/* 125 Hz */
};

/* working mode */
enum mma7455l_mode {
	MMA7455L_MODE_STANDBY		= 0x0,
	MMA7455L_MODE_MEASUREMENT	= 0x1,
	MMA7455L_MODE_LEVELDETECT	= 0x2,
	MMA7455L_MODE_PULSEDETECT	= 0x3,
};

struct mma7455l_platform_data {
	int intr;
};

#define MMA7455IO		0xbf

/* IOCTLs*/
#define MMA7455_IOCTL_INIT		_IO(MMA7455IO, 0x31)
#define MMA7455_IOCTL_WRITE		_IOW(MMA7455IO, 0x32, char[5])
#define MMA7455_IOCTL_READ		_IOWR(MMA7455IO, 0x33, char[5])
#define MMA7455_IOCTL_READ_ACCELERATION	_IOWR(MMA7455IO, 0x34, short[7])
#define MMA7455_IOCTL_SET_MODE		_IOW(MMA7455IO, 0x35, short)
#define MMA7455_IOCTL_GET_INT		_IOR(MMA7455IO, 0x36, short)
#define MMA7455_IOCTL_DO_CALIBRATION	_IOR(MMA7455IO, 0x37, short[6])
#define MMA7455_IOCTL_REPORT		_IOR(MMA7455IO, 0x38, short[3])
#define MMA7455_IOCTL_WRITE_OFFSET	_IOW(MMA7455IO, 0x39, short[6])
#define MMA7455_IOCTL_SET_AFLAG		_IOW(MMA7455IO, 0x40, short)
#define MMA7455_IOCTL_GET_AFLAG		_IOW(MMA7455IO, 0x41, short)

#endif /* _LINUX_MMA7455L_H */

