/*
 * linux/drivers/video/jz47xx_lcd.h -- Ingenic Jz4760 On-Chip LCD frame buffer device
 *
 * Copyright (C) 2005-2008, Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __JZ47XX_AOSD_H__
#define __JZ47XX_AOSD_H__

#ifndef CONFIG_JZ47XX_AOSDC
#define  CONFIG_JZ47XX_AOSDC 1
#endif

struct jz47xx_aosd_info {
	unsigned long addr0;	/* 64Words aligned at least */
	unsigned long addr1;
	unsigned long addr2;
	unsigned long addr3;
	unsigned long waddr;	/* 64Words aligned at least */

	unsigned long smem_start;
	char __iomem *addr0_base;
	__u32         addr_len;

	__u32         compress_done;
	__u32         alpha_value;
	__u32         frmlv;
	__u32         order;
	__u32         format_mode;
	__u32         alpha_en;		/* 0: 24bpp, 1: 32bpp or 16bpp */
	__u32         bpp;
	__u32         alpha_mode;
	__u32         aligned_64; 	/* meanless */
	__u32         height;		/* in lines */
	__u32         width;		/* in pixels */
	__u32         src_stride;	/* in bytes, 16Words aligned at least */
	__u32         dst_stride;	/* in bytes, 16Words aligned at least  */
	__u32         buf;
}; 

#define ALPHA_OSD_START         0x46a8
#define ALPHA_OSD_GET_INFO      0x46a9
#define ALPHA_OSD_SET_MODE      0x46aa
#define COMPRESS_START          0x46ab
#define COMPRESS_GET_INFO      0x46ac
#define COMPRESS_SET_MODE      0x46ad
#define ALPHA_OSD_PRINT      0x46ae

void print_aosd_registers(void); 
int jz47xx_get_aosd_info(struct jz47xx_aosd_info *info);
void jz47xx_start_compress(void);
void jz47xx_compress_set_mode(struct jz47xx_aosd_info *info);
void jz47xx_aosd_set_mode(struct jz47xx_aosd_info *info);
int jz47xx_compress_init(void);
void calc_comp_ratio(int dy, unsigned int height, unsigned int width, unsigned int frame_size);
#endif /* __JZ47XX_AOSD_H__ */
