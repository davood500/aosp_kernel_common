/*
 * Copyright (c) 2006-2010  Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; see the file COPYING. If not, write to the Free Software
 * Foundation, 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * Tablet get device id.
 *
 * Author: Wolfgang Wang <lgwang@ingenic.cn>
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/err.h>
#include <linux/mtd/mtd.h>
#include <linux/sched.h>

#include "mach/board_device_id.h"

//#define DEBUG_DEVICE_ID


#define PRINT_PREF KERN_INFO "board_device_id: "

#ifdef DEBUG_DEVICE_ID
#define dbg_printk(f,a...) \
	do {\
		printk("%s %d " f, __FILE__, __LINE__, ##a);	\
	} while(0)
#else
#define dbg_printk(f,a...)
#endif


void * ingenic_board_id=NULL;

#ifdef CONFIG_MTD

static int device_id_be_readed = 0;

static int pgsize;

#ifdef DEBUG_DEVICE_ID
static int ebcnt;
static int pgcnt;

static void dump_mtd_info(struct mtd_info *mtd)
{
	printk(PRINT_PREF"dump_mtd_info() ENTER\n");

	printk("type=%x\n", mtd->type);
	printk("flags=%x\n", mtd->flags);
	printk("size=%llx\n", mtd->size);
	printk("erasesize=%x\n", mtd->erasesize);
	printk("writesize=%x\n", mtd->writesize);
	printk("oobsize=%x\n", mtd->oobsize);
	printk("oobavail=%x\n", mtd->oobavail);
	printk("erasesize_shift=%x\n", mtd->erasesize_shift);
	printk("writesize_shift=%x\n", mtd->writesize_shift);
	printk("erasesize_mask=%x\n", mtd->erasesize_mask);
	printk("writesize_mask=%x\n", mtd->writesize_mask);
	printk("index=%x\n", mtd->index);
	printk("numeraseregions=%x\n", mtd->numeraseregions);
	printk("name=%s\n", mtd->name);
	printk("subpage_sft=%x\n", mtd->subpage_sft);
	printk("usecount=%x\n", mtd->usecount);

}
#endif	/* DEBUG_DEVICE_ID */

static void dump_buff(unsigned char *buf, int size)
{
	int cnt;
	printk("dump_buff(%p, %d)\n", buf, size);
	if (size<1) 
		return ;
	cnt = 0;
	while (cnt++ < size) {
		printk("%02x ", (unsigned char)*buf++);
		if ( cnt%20 ==0 )
			printk("\n");
	}
	printk("\n");
}



static int read_mtd_page(struct mtd_info *mtd, int page_idx, u_char * buf)
{
	size_mtd_t read = 0;
	int ret, err = 0;
	loff_t addr = page_idx * pgsize;

	if ( buf == NULL ) {
		err = -EINVAL;
		return err;
	}

	dbg_printk("mtd=%p, addr%#llx, read=%d, buf=%p\n", 
		   mtd, addr, pgsize, buf);

	ret = mtd->read(mtd, addr, pgsize, &read, buf);
	if (ret == -EUCLEAN)
		ret = 0;
	if (ret || read != pgsize) {
		printk(PRINT_PREF "error: read failed at %#llx\n",
			   (long long)addr);
		if (!err)
			err = ret;
		if (!err)
			err = -EINVAL;
	}

	return err;
}

#if 0
static int write_mtd_page(struct mtd_info *mtd, int page_idx, u_char * buf)
{
	size_mtd_t read = 0;
	int ret, err = 0;
//	loff_t addr = page_idx * mtd->erasesize;
	loff_t addr = page_idx * pgsize;

	if ( buf == NULL ) {
		err = -EINVAL;
		return err;
	}

	dbg_printk("mtd=%p, addr%#llx, write=%d, buf=%p\n", 
		   mtd, addr, pgsize, buf);

	ret = mtd->write(mtd, addr, pgsize, &read, buf);
	if (ret == -EUCLEAN)
		ret = 0;
	if (ret || read != pgsize) {
		printk(PRINT_PREF "error: write failed at %#llx\n",
			   (long long)addr);
		if (!err)
			err = ret;
		if (!err)
			err = -EINVAL;
	}

	return err;
}
#endif

static int read_one_id(struct mtd_info *mtd, int block_idx, unsigned char *iobuf, unsigned char *device_id)
{
	printk("---block %d bad? %d\n",block_idx, mtd->block_isbad(mtd, mtd->erasesize*block_idx));
        
	while(mtd->block_isbad(mtd, mtd->erasesize*block_idx)) {
		printk("0read_one_id: block_idx=%d mtd->erasesize=%d\n", block_idx, mtd->erasesize);

		block_idx++;
		if (block_idx == DEVICE_ID_BAK_NUM)
                    break;
        }
	if  (block_idx != DEVICE_ID_BAK_NUM + 1)
		read_mtd_page(mtd, mtd->erasesize*block_idx/mtd->writesize, (u_char *)iobuf);
        
	memcpy(device_id, iobuf, DEVICE_ID_MAX_SIZE);
	printk("1read_one_id: block_idx=%d\n", block_idx);
	return block_idx;
}

/* 
 * id_is_same return value:
 * 		0: not same.
 * 		1: same.
 */
static int id_is_same(unsigned char *device_id0, unsigned char *device_id1)
{
	int i;

//#ifdef DEBUG_DEVICE_ID
#if 1
	printk("---compare two id.\n");
	dump_buff(device_id0, DEVICE_ID_MAX_SIZE);
	dump_buff(device_id1, DEVICE_ID_MAX_SIZE);
#endif
	for (i = 0; i < DEVICE_ID_MAX_SIZE; i++) {
		if (device_id0[i] != device_id1[i])
		return 0;
	}
	return 1;
}

/* 
 * check_board_id return value:
 * 		0: failed.
 * 		1: pass.
 */
static int check_board_id(unsigned char *device_id) 
{
	int iii;

	for (iii=0; iii < DEVICE_ID_MAX_SIZE; iii++) {
		if ( *device_id++ != 0xFF ) 
			return 1;
	}

	/* 16 byte: FF */
	return 0;
}

int board_read_device_id(void)
{
	struct mtd_info *mtd;
	unsigned char *iobuf = NULL;
	int err = 0;
	unsigned char *device_id = NULL;
	int block_idx, sn_idx;
        
#ifdef DEBUG_DEVICE_ID
	uint64_t tmp;
	printk(KERN_INFO "\n");
	printk(KERN_INFO "=================================================\n");
	printk(PRINT_PREF "MTD device: %d\n", DEVICE_ID_MTD_BLOCK_IDX);
#endif

	if (device_id_be_readed) {
		return 0;
	}

	device_id_be_readed = 1;

	mtd = get_mtd_device(NULL, DEVICE_ID_MTD_BLOCK_IDX); /* kpanic block */
	if (IS_ERR(mtd)) {
		err = PTR_ERR(mtd);
		printk(PRINT_PREF "error: Cannot get MTD device\n");
		return err;
	}

#ifdef DEBUG_DEVICE_ID
	dump_mtd_info(mtd);
#endif
	if (mtd->writesize == 1) {
		printk(PRINT_PREF "not NAND flash, assume page size is 512 "
		       "bytes.\n");
		pgsize = 512;
	} else
		pgsize = mtd->writesize;

#ifdef DEBUG_DEVICE_ID
	tmp = mtd->size;
	do_div(tmp, mtd->erasesize);
	ebcnt = tmp;
	pgcnt = mtd->erasesize / mtd->writesize;

	printk(PRINT_PREF "MTD device size %llu, eraseblock size %u, "
	       "page size %u, count of eraseblocks %u, pages per "
	       "eraseblock %u, OOB size %u\n",
	       (unsigned long long)mtd->size, mtd->erasesize,
	       pgsize, ebcnt, pgcnt, mtd->oobsize);
#endif

#if 0
	iobuf = kmalloc(pgsize, GFP_KERNEL);
#else
	iobuf = (unsigned char *)__get_free_pages(GFP_KERNEL, get_order(pgsize));
#endif
	if (!iobuf) {
		printk(PRINT_PREF "error: cannot allocate memory\n");
		err = -ENOMEM;
		goto out;
	}

	device_id = kmalloc((DEVICE_ID_MAX_SIZE+1)*2, GFP_KERNEL);
	if (!device_id) {
		printk(PRINT_PREF "error: cannot allocate memory\n");
		err = -ENOMEM;
		goto out;
	}

	memset(device_id, 0x0 , (DEVICE_ID_MAX_SIZE+1)*2);

#if 0
	{
		printk("================ write mtd %d\n", DEVICE_ID_MTD_BLOCK_IDX);
		memset(iobuf, 0x5A, pgsize/2);
		memset(iobuf+pgsize/2, 0xA5, pgsize/2);
		err = write_mtd_page(mtd, DEVICE_ID_MTD_PAGE_IDX, (u_char *)iobuf);
	}
#endif
	memset(iobuf, 0 , pgsize);

	block_idx = sn_idx = 0;
        
	block_idx = read_one_id(mtd, block_idx, (u_char *)iobuf, device_id);
	block_idx++;
	sn_idx++;
        
	while(block_idx < DEVICE_ID_BAK_NUM) {

		block_idx = read_one_id(mtd, block_idx, (u_char *)iobuf,
			device_id+(DEVICE_ID_MAX_SIZE+1)*(sn_idx%2));

		block_idx++;
		sn_idx++;
		if(id_is_same(device_id, device_id+DEVICE_ID_MAX_SIZE+1))
		break;
	}
        
#ifdef DEBUG_DEVICE_ID
	dump_buff((unsigned char *)iobuf, pgsize);
#endif
        
	if(ingenic_board_id != NULL) 
		kfree(ingenic_board_id);

	if (check_board_id(device_id) && (block_idx != DEVICE_ID_BAK_NUM + 1)) {
		printk("device_id available\n");
		dump_buff(device_id, DEVICE_ID_MAX_SIZE);
		ingenic_board_id = device_id;
	} else {
		printk("device_id not available\n");
		dump_buff(device_id, DEVICE_ID_MAX_SIZE);
		ingenic_board_id = NULL;
	}

out:
        if (!iobuf)
            __free_pages((void *)iobuf, get_order(pgsize));
        
	put_mtd_device(mtd);
	if (err) {
		printk(PRINT_PREF "error %d occurred\n", err);
		if (device_id)
			kfree(device_id);
	}

	return err;
}

#else //CONFIG_MTD
int board_read_device_id(void)
{
	return 0;
}
#endif //CONFIG_MTD
