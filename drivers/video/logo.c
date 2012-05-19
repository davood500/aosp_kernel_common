/* drivers/video/logo.c
 *
 * Show Logo in RLE 565 or RGB 565 format
 * RLE Logo which created by img2rle
 *
 * Copyright (C) 2008 Google Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fb.h>
#include <linux/vt_kern.h>
#include <linux/unistd.h>
#include <linux/syscalls.h>

#include <linux/irq.h>
#include <asm/system.h>

#define fb_width(fb)	((fb)->var.xres)
#define fb_height(fb)	((fb)->var.yres)
#define fb_size(fb)	((fb)->var.xres * (fb)->var.yres * 2)


void jzfb_get_panel_size(unsigned int *w, unsigned *h);

#ifdef CONFIG_FB_565RLE_LOGO
static void memset16(void *_ptr, unsigned short val, unsigned count)
{
	unsigned short *ptr = _ptr;
	while (count--)
		*ptr++ = val;
}
static void memset32(void *_ptr,unsigned short val ,unsigned count)
{
 	int val_32;
	int rdata,gdata,bdata;
	unsigned int *ptr=_ptr;
	struct fb_info *info;

	info = registered_fb[0];
	if (!info) {
		printk(KERN_WARNING "%s: Can not access framebuffer\n",
			__func__);
	}

	rdata=val>>11;
	gdata=val>>5 &0x003F;
	bdata=val&0x001F;
	if (info->var.red.offset == 0 && info->var.green.offset == 8 && info->var.blue.offset == 16) {
		val_32=bdata<<19 | 0x7<<16 | gdata<<10 | 0x3<<8 | rdata<<3 | 0x7;/* FORMAT_X8B8G8R8 */
	} else {
		val_32=rdata<<19 | 0x7<<16 | gdata<<10 | 0x3<<8 | bdata<<3 | 0x7;/* default: FORMAT_X8R8G8B8*/
	}

	while(count--)
		*ptr++ = val_32;
}
/* 565RLE image format: [count(2 bytes), rle(2 bytes)] */

int display_fb_rle565(unsigned short *buf, unsigned char * dst_buf,int count) {
	struct fb_info *info;
	int vm_width, vm_height;
	unsigned short  *ptr;
	unsigned int *bits_32;
	unsigned short *bits_16;
	int bpp;
	int photo_width,photo_height;
	int width,height,ewidth,eheight,flag;				
	int extra_width=photo_width-vm_width;

	info = registered_fb[0];
	
	if (!info) {
		printk(KERN_WARNING "%s: Can not access framebuffer\n",
			__func__);
		return -ENODEV;
	}

	bpp = info->var.bits_per_pixel;

	jzfb_get_panel_size(&vm_width, &vm_height);

	ptr = (unsigned short *)buf;
	bits_32=(unsigned int *)dst_buf;
	bits_16=(unsigned short *)dst_buf;
		
	photo_width=ptr[0];
	photo_height=ptr[1];
	ptr +=2;
		
	height=photo_height<vm_height ? photo_height : vm_height;
	ewidth=vm_width-photo_width;
	eheight=vm_height-photo_height;
	
	flag=photo_width*photo_height-vm_width*vm_height;
	
	if (flag<=0) {
			while(count > 0)
			{
				if(eheight>0)
				{
					if(bpp==16)
						bits_16 += (eheight/2)*vm_width;
					else
						bits_32 += (eheight/2)*vm_width;
				}
				while(height >0)
				{					
					width=photo_width<vm_width ? photo_width : vm_width;
					if(ewidth > 0)
						{
							if(bpp==16)
								bits_16 +=ewidth/2;
							else
								bits_32 +=ewidth/2;
						}
					while(width > 0)
					{	
						unsigned n = ptr[0];

						if( count < 0)
								break;
						if(bpp==16)
							{
								memset16(bits_16, ptr[1], n);
								bits_16 +=n;
							}
						else
							{
								memset32(bits_32, ptr[1], n);
								bits_32 +=n;
							}
						ptr +=2;
						count -= 2;
						width -=n;
					}
					if(ewidth<0 &&(width <0))
					{
						ptr -=2;
						count +=2;
						if(bpp==16)
							bits_16 +=width;
						else
							bits_32 +=width;
						ptr[0]=-width;
					}
					
					if(ewidth > 0|| (ewidth==0))					
					{
						if(bpp==16)
							bits_16 +=ewidth/2;
						else
							bits_32 +=ewidth/2;
					}
		   			else 
					{
						int xwidth;
						xwidth=-ewidth;
						while(xwidth>0)
						{
							unsigned m = ptr[0];
							ptr +=2;
							count -= 2;
							xwidth -=m;
						}
						if(ewidth < 0)
						{
							ptr -=2;
							count +=2;
							ptr[0] = -xwidth;
						}
					}
					height -=1;
				}
				if(eheight>0)
				{
					if(bpp==16)
						bits_16 += (eheight/2)*vm_width;
					else
						bits_32 += (eheight/2)*vm_width;
				}
				if( height <= 0 )
						return 0;
			}
	}
	else
	{
		while(count > 0)
		{
			while(height > 0)
			{
				width=photo_width<vm_width ? photo_width : vm_width;
				while(width > 0)
				{
					unsigned n = ptr[0];

					if(count < 0)
						break;
					if(bpp==16)
						{
							memset16(bits_16, ptr[1], n);
							bits_16 +=n;
						}
					else
						{
							memset32(bits_32, ptr[1], n);
							bits_32 +=n;
						}
					ptr +=2;
					count -= 2;
					width -=n;						
				}
				if(width <0 )
				{
					ptr -=2;
					count +=2;
					if(bpp==16)
						bits_16 +=width;
					else
						bits_32 +=width;
					ptr[0]=-width;
				}
				while(extra_width > 0)
				{
					unsigned n = ptr[0];
					ptr +=2;
					count -= 2;
					extra_width -=n;
				}
				if(extra_width < 0)
				{
					ptr -=2;
					count +=2;
					ptr[0] = -extra_width;
				}
				height -=1;
			}
			if( height <= 0 )
					break;		
		}
	}
	return 0;
}
#endif


/* 565RGB image format: rgb565 */
#ifdef CONFIG_FB_565RGB_LOGO
int display_fb_rgb565(unsigned short *buf, unsigned count) {
	struct fb_info *info;
	unsigned max;
	int vm_width, vm_height, stride, ppl;
	unsigned short *bits, *ptr;
	info = registered_fb[0];

	if (!info) {
		printk(KERN_WARNING "%s: Can not access framebuffer\n",
			__func__);
		return -ENODEV;
	}

	jzfb_get_panel_size(&vm_width, &vm_height);

	max = vm_width * vm_height;
	stride = fb_width(info) - vm_width;
	ppl = fb_width(info); /*pixel per line*/
	ptr = (unsigned short *)buf;
	bits = (unsigned short *)(info->screen_base);
	if (vm_width < ppl) {
		while (vm_height--) {
			memcpy((void *)bits, (void *)ptr, vm_width<<1);
			bits += ppl;
			ptr += vm_width;
		}
	}
	else {
		memcpy((void *)bits, (void *)ptr, max<<1);
	}
	return 0;
}
#endif

int load_565_image(char *filename, unsigned char * dst_buf)
{
	int fd, err = 0;
	unsigned count;
	unsigned short *data;

	fd = sys_open(filename, O_RDONLY, 0);
	if (fd < 0) {
		printk(KERN_WARNING "%s: Can not open %s\n",
			__func__, filename);
		return -ENOENT;
	}
	count = (unsigned)sys_lseek(fd, (off_t)0, 2);	
	
	if (count == 0) {
		sys_close(fd);
		err = -EIO;
		goto err_logo_close_file;
	}
	sys_lseek(fd, (off_t)0, 0);

	data = kmalloc(count, GFP_KERNEL);
	if (!data) {
		printk(KERN_WARNING "%s:Can not alloc data\n", __func__);
		err = -ENOMEM;
		goto err_logo_close_file;
		}
	if ((unsigned)sys_read(fd, (char *)data, count) != count) {
		err = -EIO;
		goto err_logo_free_data;
		}

#ifdef CONFIG_FB_565RLE_LOGO
	/* 565RLE image format: [count(2 bytes), rle(2 bytes)] */
	display_fb_rle565(data,dst_buf, count);


#endif
#ifdef CONFIG_FB_565RGB_LOGO
	/* 565RGB image format: rgb565 */
	display_fb_rgb565(data, count);
#endif


err_logo_free_data:
	kfree(data);

err_logo_close_file:
	sys_close(fd);
	return err;
}
EXPORT_SYMBOL(load_565_image);
