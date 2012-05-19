#ifndef __BOARD_DEVICE_ID__
#define __BOARD_DEVICE_ID__

/*
 * device id stored at:
 * 		mtdblock 8: page 0. Addr: 0x1000000
 * This must keep sync with kernel/drivers/mtd/nand/jz4760_nand.c mtd_partition
 *
 *
 *
 *
 */



#define DEVICE_ID_MTD_BLOCK_IDX 5
#define DEVICE_ID_MTD_PAGE_IDX 0
#define DEVICE_ID_BAK_NUM  16
#define DEVICE_ID_MAX_SIZE 16	/* example: GS10305000001VT8 */

extern void *ingenic_board_id;

int board_read_device_id(void);

#endif //__BOARD_DEVICE_ID__
