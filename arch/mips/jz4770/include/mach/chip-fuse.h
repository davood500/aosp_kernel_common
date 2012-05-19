/*
 * arch/mips/mach-jz4760b/include/mach/chip-i2c.h
 *
 * JZ4760B I2C register definition.
 *
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 */

#ifndef __CHIP_FUSE_H__
#define __CHIP_FUSE_H__


#define	FUSE_BASE	0xB34100dc


/*************************************************************************
 * I2C
 *************************************************************************/
#define	EFSCTL		(FUSE_BASE + 0x00)
#define	EFUSE0		(FUSE_BASE + 0x04)
#define	EFUSE1		(FUSE_BASE + 0x08)
#define	EFUSE2		(FUSE_BASE + 0x0c)
#define	EFUSE3		(FUSE_BASE + 0x10)
#define	EFUSE4		(FUSE_BASE + 0x14)
#define	EFUSE5		(FUSE_BASE + 0x18)
#define	EFUSE6		(FUSE_BASE + 0x1c)
#define	EFUSE7		(FUSE_BASE + 0x20)

#define	REG_EFSCTL 	REG8(EFSCTL)
#define	REG32_EFUSE0 	REG32(EFUSE0)
#define	REG32_EFUSE1 	REG32(EFUSE1)
#define	REG32_EFUSE2 	REG32(EFUSE2)
#define	REG32_EFUSE3 	REG32(EFUSE3)
#define	REG32_EFUSE4 	REG32(EFUSE4)
#define	REG32_EFUSE5 	REG32(EFUSE5)
#define	REG32_EFUSE6 	REG32(EFUSE6)
#define	REG32_EFUSE7 	REG32(EFUSE7)

#endif 

