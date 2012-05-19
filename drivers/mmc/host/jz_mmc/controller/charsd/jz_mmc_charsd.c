/*
 *  linux/drivers/mmc/host/jz_mmc/charsd/jz_mmc_charsd.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */

#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <asm/io.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <linux/dma-mapping.h>
#include <linux/completion.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>

#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>

#include <asm/jzmmc/jz_mmc_charsd.h>

#include <linux/slab.h>
//#define YKDBG

int jz_charsd_major = JZ_CHARSD_MAJOR;
static struct class *jz_charsd_class;
struct charsd_dev *jz_charsd_devp;

static struct charsd_palu_buffer *jz_charsd_fetch_buffer(struct charsd_dev *devp)
{
	struct charsd_palu_buffer *pb;

	pb = devp->next_palu_buffer;
	if(pb->state == PBUF_STATE_FULL)
		return pb;
	else {
		pb = pb->next;
		if(pb->state == PBUF_STATE_FULL)
			return pb;
		else
			return NULL;
	}
}

static int jz_charsd_status_polling(struct charsd_dev *devp,unsigned int target)
{
	struct jz_mmc_host *host = devp->host;
	unsigned int stat = 0;

	while(!((REG_MSC_STAT(devp->pdev_id) & target) ||
		 (host->eject) || (REG_MSC_STAT(devp->pdev_id) & WAITMASK)));          //polling

	if(host->eject) {
		
		printk("CHARSD-%d:data trans eject while polling\n", devp->pdev_id);
		jz_mmc_stop_dma(host);

		return -1;
		
	} else if(REG_MSC_STAT(devp->pdev_id) & WAITMASK) {
		
		printk("CHARSD-%d: CMD error report(dir = %d)--", devp->pdev_id,devp->data_dir);
 		
		stat = REG_MSC_STAT(devp->pdev_id);		

		if (stat & MSC_STAT_TIME_OUT_READ) {
			REG_MSC_IREG(devp->pdev_id) = MSC_IREG_TIMEOUT_READ;
			printk("Timeout while reading\n");
		} else if (stat & MSC_STAT_TIME_OUT_RES) {
			REG_MSC_IREG(devp->pdev_id) = MSC_IREG_TIMEOUT_RES;
			printk("Timeout while responsing\n");
		} else if (stat & MSC_STAT_CRC_WRITE_ERROR) {
			REG_MSC_IREG(devp->pdev_id) = MSC_IREG_CRC_WRITE_ERR;
			printk("CRC error while writing\n");
		} else if (stat & MSC_STAT_CRC_READ_ERROR) {
			REG_MSC_IREG(devp->pdev_id) = MSC_IREG_CRC_READ_ERR;
			printk("CRC error while reading\n");
		} else if (stat & MSC_STAT_CRC_RES_ERR) {
			REG_MSC_IREG(devp->pdev_id) = MSC_IREG_CRC_RES_ERR;
			printk("CRC error while responsing\n");
		} else {
			printk("stat = %x\n", stat);
		}

		jz_mmc_stop_dma(host);
		return -1;
	}

	return 0;
}

static unsigned int jz_charsd_get_resp(struct charsd_dev *devp)
{
	unsigned int resp;
	unsigned short buf[5];
	unsigned int res;

	res = REG_MSC_RES(devp->pdev_id);
	buf[0] = (res >> 8) & 0xff;
	buf[1] = res & 0xff;
	res = REG_MSC_RES(devp->pdev_id);
	buf[2] = (res >> 8) & 0xff;
	buf[3] = res & 0xff;
	res = REG_MSC_RES(devp->pdev_id);
	buf[4] = res & 0xff;
	
	resp =	buf[1] << 24 | buf[2] << 16 | buf[3] << 8 | buf[4];
	return resp;
}

static int jz_charsd_wait_program(struct charsd_dev *devp)
{
	unsigned int resp;
	unsigned int cmdat = MSC_CMDAT_BUS_WIDTH_4BIT |			\
		MSC_CMDAT_RESPONSE_R1 | MSC_CMDAT_RTRG_EQUALT_16 | MSC_CMDAT_TTRG_LESS_16;
	unsigned char retries = 5;

	do {
		REG_MSC_RDTO(devp->pdev_id) = 0xffffff;
		REG_MSC_IMASK(devp->pdev_id) = 0xffff;      // mask interrupts
		REG_MSC_IREG(devp->pdev_id) = 0xffff;	// clear status
		REG_MSC_CMD(devp->pdev_id) = MMC_SEND_STATUS;
		REG_MSC_CMDAT(devp->pdev_id) = cmdat;
		REG_MSC_ARG(devp->pdev_id) = devp->rca << 16;
		
		REG_MSC_RESTO(devp->pdev_id) = 0xff;
		
		REG_MSC_STRPCL(devp->pdev_id) |= MSC_STRPCL_START_OP;      //start the operation
		
		if(jz_charsd_status_polling(devp,MSC_STAT_END_CMD_RES)) {
			printk("CharSD-%d: Time out while waiting CMD13's response!!\n", devp->pdev_id);
			return -ESERVERFAULT;
		}
		REG_MSC_IREG(devp->pdev_id) = MSC_IREG_END_CMD_RES;	                   //clear irq flag 

		resp = jz_charsd_get_resp(devp);

		retries--;
	} while((!(resp & R1_READY_FOR_DATA) || (R1_CURRENT_STATE(resp) == 7)) && retries);

	if(retries == 0) {
		printk("CharSD-%d: Time out while waiting program done!!\n", devp->pdev_id);
		return -ESERVERFAULT;
	}

	return 0;
}

static int jz_charsd_start_cpu(struct charsd_dev *devp)
{
	unsigned int *cpu_addr_ptr = 0;
	unsigned int cpu_len = 0;
	int i = 0;
	unsigned int cmdat = 0;

	cmdat = MSC_CMDAT_BUS_WIDTH_4BIT | MSC_CMDAT_DATA_EN | \
		MSC_CMDAT_RESPONSE_R1 | MSC_CMDAT_RTRG_EQUALT_16 | MSC_CMDAT_TTRG_LESS_16;

	REG_MSC_RDTO(devp->pdev_id) = 0xffffff;
	REG_MSC_BLKLEN(devp->pdev_id) = 512;
	REG_MSC_IMASK(devp->pdev_id) = 0xffff;      // mask interrupts
	REG_MSC_IREG(devp->pdev_id) = 0xffff;	// clear status
	REG_MSC_NOB(devp->pdev_id) = devp->blks_2wr;

	cpu_len = devp->blks_2wr << 9;

	if(!devp->data_dir) {
		cmdat &= ~MSC_CMDAT_WRITE;
		if(devp->blks_2wr == 1) {
			REG_MSC_CMD(devp->pdev_id) = MMC_READ_SINGLE_BLOCK;
		}
		else {
			REG_MSC_CMD(devp->pdev_id) = MMC_READ_MULTIPLE_BLOCK;
			cmdat |= MSC_CMDAT_SEND_AS_STOP;
		}
	} else {
		cmdat |= MSC_CMDAT_WRITE;
		if(devp->blks_2wr== 1) {
			REG_MSC_CMD(devp->pdev_id) = MMC_WRITE_BLOCK;
		}
		else {
			REG_MSC_CMD(devp->pdev_id) = MMC_WRITE_MULTIPLE_BLOCK;
			cmdat |= MSC_CMDAT_SEND_AS_STOP;
		}
	}
	
	//start the operation
	cpu_addr_ptr = (unsigned int *)devp->buf_2wr;

	REG_MSC_CMDAT(devp->pdev_id) = cmdat;
	
	if(devp->ishc) {
		REG_MSC_ARG(devp->pdev_id) = devp->data_addr >> 9;
	} else {
		REG_MSC_ARG(devp->pdev_id) = devp->data_addr;
	}

	REG_MSC_RESTO(devp->pdev_id) = 0xff;

	REG_MSC_STRPCL(devp->pdev_id) |= MSC_STRPCL_START_OP;      //start the operation

	if(jz_charsd_status_polling(devp,MSC_STAT_END_CMD_RES))
		return -ESERVERFAULT;
	REG_MSC_IREG(devp->pdev_id) = MSC_IREG_END_CMD_RES;	                   //clear irq flag 
	
	if(!devp->data_dir) {
		for(i = 0;i < (cpu_len >> 2);i++) {
			while (REG_MSC_STAT(devp->pdev_id) & MSC_STAT_DATA_FIFO_EMPTY);
			*cpu_addr_ptr++ = REG_MSC_RXFIFO(devp->pdev_id);
		}

		REG_MSC_IREG(devp->pdev_id) = MSC_IREG_DATA_TRAN_DONE;
	} else {
		for(i = 0;i < (cpu_len >> 2);i++) {
			while (REG_MSC_STAT(devp->pdev_id) & MSC_STAT_DATA_FIFO_FULL);
			REG_MSC_TXFIFO(devp->pdev_id) = *cpu_addr_ptr++;
		}

		if(jz_charsd_status_polling(devp,MSC_STAT_PRG_DONE))
			return -ESERVERFAULT;

		REG_MSC_IREG(devp->pdev_id) = MSC_IREG_PRG_DONE;

		if(jz_charsd_wait_program(devp) < 0)
			return -ESERVERFAULT;
	}
	return 0;
}

static void jz_charsd_dma_pre(struct charsd_dev *devp,int chan)
{
	unsigned long flags;
	unsigned int dcmd = 0;
	dma_addr_t buf_phyaddr = CPHYSADDR(devp->buf_2wr);

	if (!(buf_phyaddr & 0xf0000000)) {
		buf_phyaddr += 0x20000000;
	}

	flags = claim_dma_lock();

	REG_DMAC_DCCSR(chan) = 0;
	REG_DMAC_DTCR(chan) = (devp->blks_2wr << 9) / DMA_TS;	
	dcmd = DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_32 | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_TIE;

	switch (DMA_TS) { 
	case 64:
		dcmd |= DMAC_DCMD_DS_64BYTE;
		break;
		
	case 32:
		dcmd |= DMAC_DCMD_DS_32BYTE;
		break;
		
	case 16:
		dcmd |= DMAC_DCMD_DS_16BYTE;
		break;
		
	case 4:
		dcmd |= DMAC_DCMD_DS_32BIT;
		break;
		
	default:
		;
	}


	if(!devp->data_dir) {                 //read
		dma_cache_inv((unsigned int)devp->buf_2wr, (devp->blks_2wr << 9));

		REG_DMAC_DCMD(chan) = (dcmd | DMAC_DCMD_DAI);

		REG_DMAC_DSAR(chan) = CPHYSADDR(MSC_RXFIFO(devp->pdev_id));
		
		REG_DMAC_DTAR(chan) = buf_phyaddr;

		if(devp->pdev_id == 0)
			REG_DMAC_DRSR(chan) = DMAC_DRSR_RS_MSC0IN;
		else if(devp->pdev_id == 1)
			REG_DMAC_DRSR(chan) = DMAC_DRSR_RS_MSC1IN;
		else
			REG_DMAC_DRSR(chan) = DMAC_DRSR_RS_MSC2IN;
		
	} else {                              //write
		dma_cache_wback_inv((unsigned int)devp->buf_2wr, (devp->blks_2wr << 9));		

		REG_DMAC_DCMD(chan) = (dcmd | DMAC_DCMD_SAI);
		
		REG_DMAC_DSAR(chan) = buf_phyaddr;
		
		REG_DMAC_DTAR(chan) = CPHYSADDR(MSC_TXFIFO(devp->pdev_id));
		
		if(devp->pdev_id == 0)
			REG_DMAC_DRSR(chan) = DMAC_DRSR_RS_MSC0OUT;
		else if(devp->pdev_id == 1)
			REG_DMAC_DRSR(chan) = DMAC_DRSR_RS_MSC1OUT;
		else
			REG_DMAC_DRSR(chan) = DMAC_DRSR_RS_MSC2OUT;

	}

	// Enable DMA 
	REG_DMAC_DMACR(chan / HALF_DMA_NUM) |= DMAC_DMACR_DMAE;	
	// Setup DMA channel control/status register 
	REG_DMAC_DCCSR(chan) = DMAC_DCCSR_NDES | DMAC_DCCSR_EN;

	release_dma_lock(flags);
}

static int jz_charsd_start_dma(struct charsd_dev *devp)
{
	unsigned int cmdat = 0;
	int chan = 0;

	cmdat = MSC_CMDAT_BUS_WIDTH_4BIT | MSC_CMDAT_DATA_EN | MSC_CMDAT_DMA_EN |\
		MSC_CMDAT_RESPONSE_R1 | MSC_CMDAT_RTRG_EQUALT_16 | MSC_CMDAT_TTRG_LESS_16;

	REG_MSC_RDTO(devp->pdev_id) = 0xffffff;
	REG_MSC_BLKLEN(devp->pdev_id) = 512;
	REG_MSC_IMASK(devp->pdev_id) = 0xffff;      // mask interrupts
	REG_MSC_IREG(devp->pdev_id) = 0xffff;	// clear status
	REG_MSC_NOB(devp->pdev_id) = devp->blks_2wr;
	chan = devp->host->dma.channel;

	if(!devp->data_dir) {

		cmdat &= ~MSC_CMDAT_WRITE;
		if(devp->blks_2wr == 1) {
			REG_MSC_CMD(devp->pdev_id) = MMC_READ_SINGLE_BLOCK;
		}
		else {
			REG_MSC_CMD(devp->pdev_id) = MMC_READ_MULTIPLE_BLOCK;
			cmdat |= MSC_CMDAT_SEND_AS_STOP;
		}
	} else {

		cmdat |= MSC_CMDAT_WRITE;
		if(devp->blks_2wr== 1) {
			REG_MSC_CMD(devp->pdev_id) = MMC_WRITE_BLOCK;
		}
		else {
			REG_MSC_CMD(devp->pdev_id) = MMC_WRITE_MULTIPLE_BLOCK;
			cmdat |= MSC_CMDAT_SEND_AS_STOP;
		}
	}

	REG_MSC_CMDAT(devp->pdev_id) = cmdat;

	if(!devp->data_dir) 
		jz_charsd_dma_pre(devp,chan);
	if(devp->ishc) {
		REG_MSC_ARG(devp->pdev_id) = devp->data_addr >> 9;
	} else {
		REG_MSC_ARG(devp->pdev_id) = devp->data_addr;
	}

	REG_MSC_RESTO(devp->pdev_id) = 0xff;
	REG_MSC_STRPCL(devp->pdev_id) |= MSC_STRPCL_START_OP;

	if(jz_charsd_status_polling(devp,MSC_STAT_END_CMD_RES))
		return -ESERVERFAULT;
	REG_MSC_IREG(devp->pdev_id) = MSC_IREG_END_CMD_RES;	                   //clear irq flag 

	if(devp->data_dir)
		jz_charsd_dma_pre(devp,chan);

	return 0;
}

static int jz_charsd_dma_done(struct charsd_dev *devp)
{
	if(!devp->data_dir) {
		if(jz_charsd_status_polling(devp, MSC_STAT_DATA_TRAN_DONE)) {
			return -1;
		}
		REG_MSC_IREG(devp->pdev_id) = MSC_IREG_DATA_TRAN_DONE;
	} else {
		if(jz_charsd_status_polling(devp,MSC_STAT_PRG_DONE)) {
			printk("CharSD-%d:Waiting programe done error!\n",devp->pdev_id);
			return -ESERVERFAULT;
		}
		REG_MSC_IREG(devp->pdev_id) = MSC_IREG_PRG_DONE;
		REG_MSC_IREG(devp->pdev_id) = MSC_IREG_DATA_TRAN_DONE;
		if(jz_charsd_wait_program(devp) < 0)
			return -ESERVERFAULT;
	}

	return 0;
}

static loff_t jz_charsd_llseek(struct file *filp,loff_t offset,int orig)
{
	return 0;
}

static ssize_t jz_charsd_read(struct file *filp,char __user *buf,size_t count,loff_t *ppos)
{
	struct charsd_dev *devp = filp->private_data;
	struct charsd_palu_buffer *full_pb = jz_charsd_fetch_buffer(devp);

#ifdef YKDBG
	printk("=====Charsd read: addr = 0x%08X size = %lu\n",(unsigned int)*ppos,(unsigned long)count);
#endif
	if(!devp || !devp->host || devp->host->eject) {
		printk("CharSD-%d:Card already removed!\n",devp->pdev_id);
		return -ESERVERFAULT;
	}

	if (full_pb != NULL) {
		wait_event_interruptible_timeout(devp->palu_wait_queue, (full_pb->state == PBUF_STATE_EMPTY) || devp->host->eject, 5 * HZ);
		if(!devp || !devp->host || devp->host->eject) {
			printk("CharSD-%d:Card already removed!\n",devp->pdev_id);
			return -ESERVERFAULT;
		}
		
		if(devp->host->eject) {
			printk("CharSD-%d:data trans eject while waiting empty buffer!!\n", devp->pdev_id);
			up(&devp->sem);
			
			return -ESERVERFAULT;
		}
		
		if(full_pb->state != PBUF_STATE_EMPTY) {
			printk("CharSD-%d:Time out while waiting empty buffer!!\n", devp->pdev_id);
			up(&devp->sem);
			
			return -ESERVERFAULT;
		}
	}
	down(&devp->sem);

	if(!devp) {
		printk("Error: Don't read a unopened file!!\n");
		up(&devp->sem);
		return -EFAULT;
	}

	devp->data_dir = 0;
	devp->data_addr = *ppos;                          //for SDHC this value should >>9;

	devp->data_addr += devp->pa_start_addr;
	
	devp->blks_2wr = count >> 9;
	devp->buf_2wr = buf;

	if(devp->use_cpu){
		if(jz_charsd_start_cpu(devp) < 0) {
			up(&devp->sem);

			return -ESERVERFAULT;			
		}
	} else {
		BUG_ON(count % 64);
		if(jz_charsd_start_dma(devp) < 0) {		
			printk("CharSD-%d:Start DMA error\n", devp->pdev_id);	
			up(&devp->sem);

			return -ESERVERFAULT;
		}
		wait_event_interruptible_timeout(devp->host->dma_wait_queue, devp->host->dma_ack || devp->host->eject, 5 * HZ);

		if(devp->host->dma_ack == 1) {
			jz_charsd_dma_done(devp);
			devp->host->dma_ack = 0;
			
		} else if(devp->host->eject) {			
			printk("CharSD-%d:data trans eject while reading\n", devp->pdev_id);
			jz_mmc_stop_dma(devp->host);
			up(&devp->sem);

			return -ESERVERFAULT;			
		} else {
			up(&devp->sem);

			printk("CharSD-%d:DMA time out!!\n", devp->pdev_id);
			return -ESERVERFAULT;
		}
	}

	up(&devp->sem);
	return (ssize_t)count;
}

static ssize_t jz_charsd_write(struct file *filp,const char __user *buf,size_t size,loff_t *ppos)
{
	struct charsd_dev *devp = filp->private_data;
	struct charsd_palu_buffer *pb;

#ifdef YKDBG
	printk("=====Charsd write: addr = 0x%08X size = %lu\n",(unsigned int)*ppos,(unsigned long)size);
#endif
	if(!devp || !devp->host || devp->host->eject) {
		printk("CharSD-%d:Card already removed!\n",devp->pdev_id);
		return -ESERVERFAULT;
	}

	if(!devp) {
		printk("Error: Don't write a unopened file!!\n");
		up(&devp->sem);

		return -EFAULT;
	}

	if(devp->error) {
		printk("CharSD-%d:error in writing kthread\n", devp->pdev_id);
		up(&devp->sem);

		return -ESERVERFAULT;
	}


	pb = devp->next_palu_buffer;

	wait_event_interruptible_timeout(devp->palu_wait_queue, (pb->state == PBUF_STATE_EMPTY) || devp->host->eject, 5 * HZ);

	if(!devp || !devp->host || devp->host->eject) {
		printk("CharSD-%d:Card already removed!\n",devp->pdev_id);
		return -ESERVERFAULT;
	}

	if(devp->host->eject) {
		printk("CharSD-%d:data trans eject while waiting empty buffer!!\n", devp->pdev_id);
		up(&devp->sem);

		return -ESERVERFAULT;
	}

	if(pb->state != PBUF_STATE_EMPTY) {
		printk("CharSD-%d:Time out while waiting empty buffer!!\n", devp->pdev_id);
		up(&devp->sem);

		return -ESERVERFAULT;
	}

	memcpy(pb->buf,buf,size);
	pb->state = PBUF_STATE_FULL;
	
	pb->data_dir = 1;
	pb->data_addr = *ppos;                          //for SDHC this value should >>9;
	
	pb->data_addr += devp->pa_start_addr;
	
	pb->blks_2wr = size >> 9;
	
	wake_up_process(devp->thread);
	
	devp->next_palu_buffer = pb->next;
	
	return (ssize_t)size;
}

static void jz_charsd_do_write(struct charsd_dev *devp)
{
	struct charsd_palu_buffer *pb =	devp->operatable_palu_buffer;

	if(!devp || !devp->host || devp->host->eject) {
		printk("CharSD-%d:Card already removed!\n",devp->pdev_id);
		return;
	}

	down(&devp->sem);

	devp->data_dir = pb->data_dir;
	devp->data_addr = pb->data_addr;                          //for SDHC this value should >>9;		
	devp->blks_2wr = pb->blks_2wr;
	devp->buf_2wr = pb->buf;

	if(devp->use_cpu){
		if(jz_charsd_start_cpu(devp) < 0) {
			devp->error = 1;
//			up(&devp->sem);
			return;
		}
	} else {
		BUG_ON((devp->blks_2wr << 9) % DMA_TS);
		if(jz_charsd_start_dma(devp) < 0) {
			printk("CHARSD-%d: DMA write error!!\n", devp->pdev_id);
			devp->error = 1;
//			up(&devp->sem);
			return;
		}
		wait_event_interruptible_timeout(devp->host->dma_wait_queue, devp->host->dma_ack || devp->host->eject, 5 * HZ);
		if(devp->host->dma_ack == 1) {
			jz_charsd_dma_done(devp);
			devp->host->dma_ack = 0;
			pb = devp->operatable_palu_buffer;
			pb->state = PBUF_STATE_EMPTY;
			wake_up_interruptible(&devp->palu_wait_queue);			

		} else if(devp->host->eject) {			
			printk("CHARSD-%d:data trans eject while writing\n", devp->pdev_id);
				jz_mmc_stop_dma(devp->host);
				devp->error = 1;

		} else {
			printk("CharSD-%d:DMA time out!!\n", devp->pdev_id);
			printk("REG_DMAC_DTCR = %d\n",REG_DMAC_DTCR(2));
			printk("REG_DMAC_DCCSR = %X\n",REG_DMAC_DCCSR(2));
			devp->error = 1;
		}
	}
	up(&devp->sem);
}

static int jz_charsd_write_thread(void *d)
{
	struct charsd_dev *devp = d;
	
	current->flags |= PF_MEMALLOC;
	
	down(&devp->thread_sem);
	do {
		spin_lock_irq(&devp->super_lock);
		set_current_state(TASK_INTERRUPTIBLE);
		devp->operatable_palu_buffer = jz_charsd_fetch_buffer(devp);
		spin_unlock_irq(&devp->super_lock);
		
//		devp->operatable_palu_buffer = jz_charsd_fetch_buffer(devp);

		if (!devp->operatable_palu_buffer || devp->error == 1) {
			if (kthread_should_stop()) {
				set_current_state(TASK_RUNNING);
				break;
			}
			up(&devp->thread_sem);

			schedule();
			down(&devp->thread_sem);
			continue;
		}
		set_current_state(TASK_RUNNING);

		jz_charsd_do_write(devp);
	} while (1);
	up(&devp->thread_sem);

	return 0;
}

static long jz_charsd_ioctl(struct file *flip,unsigned int cmd,unsigned long arg)
{
	switch(cmd) {
	case 1:
		break;
	case 2:
		break;
	default:
		return - ENOTTY;
	}
	return 0;
}

#if 0
static irqreturn_t jz_charsd_rx_callback(int irq, void *devid)
{
	struct charsd_dev *devp = devid;
	struct jz_mmc_host *host = devp->host;
	int chan = host->dma.rxchannel;

	disable_dma(chan);

	if (__dmac_channel_address_error_detected(chan)) {
		printk("ERROR:%s: DMAC address error.\n", __FUNCTION__);
		__dmac_channel_clear_address_error(chan);
	}
	if (__dmac_channel_transmit_end_detected(chan)) {
		__dmac_channel_clear_transmit_end(chan);
		wake_up_interruptible(&devp->dma_wait_queue);
		devp->dma_ack = 1;
	}


 	return IRQ_HANDLED;
}

static irqreturn_t jz_charsd_tx_callback(int irq, void *devid)
{
	struct charsd_dev *devp = devid;
	struct jz_mmc_host *host = devp->host;
	unsigned int chan = host->dma.txchannel;

	disable_dma(chan);
	if (__dmac_channel_address_error_detected(chan)) {
		printk("ERROR:%s: DMAC address error.\n", __FUNCTION__);
		__dmac_channel_clear_address_error(chan);
	}
	if (__dmac_channel_transmit_end_detected(chan)) {
		__dmac_channel_clear_transmit_end(chan);
		wake_up_interruptible(&devp->dma_wait_queue);
		devp->dma_ack = 1;
	}

 	return IRQ_HANDLED;
}

static int jz_charsd_change_irqh(struct charsd_dev *devp, unsigned choose)
{
	struct jz_mmc_host *host;

	host = devp->host;

	jz_free_dma((unsigned int)host->dma.rxchannel);
	jz_free_dma((unsigned int)host->dma.txchannel);
	
	host->dma.rxchannel = -1;
	host->dma.txchannel = -1;
	
	if (!host->dmares)
		return -ENODEV;
	
	if(choose == CHARSD_IRQ)
		host->dma.rxchannel = jz_request_dma(host->dmares->start, "dma-rx", jz_charsd_rx_callback, 0, devp);
	else if(choose == DEFSD_IRQ)
		host->dma.rxchannel = jz_request_dma(host->dmares->start, "dma-rx", jz_mmc_dma_rx_callback, 0, host);
	else
		goto err1;

	__dmac_channel_enable_clk(host->dma.rxchannel);
	
	printk("host->dma.rxchannel = %d\n", host->dma.rxchannel);
	if (host->dma.rxchannel < 0) {
		printk(KERN_ERR "jz_request_dma failed for MMC Rx\n");
		goto err1;
	}

	if(choose == CHARSD_IRQ)
		host->dma.txchannel = jz_request_dma(host->dmares->start, "dma-tx", jz_charsd_tx_callback, 0, devp);
	else if(choose == DEFSD_IRQ)
		host->dma.txchannel = jz_request_dma(host->dmares->start, "dma-tx", jz_mmc_dma_tx_callback, 0, host);
	else
		goto err1;

	__dmac_channel_enable_clk(host->dma.txchannel);
	
	printk("host->dma.txchannel = %d\n", host->dma.txchannel);
	if (host->dma.txchannel < 0) {
		printk(KERN_ERR "jz_request_dma failed for MMC Tx\n");
		goto err2;
	}
	
	return 0;
err2:
	jz_free_dma(host->dma.rxchannel);
err1:
	return -ENODEV;
}
#endif

int jz_charsd_open(struct inode *inode,struct file *filp)
{
	struct charsd_dev *devp;
	struct jz_mmc_host *host;

	devp = container_of(inode->i_cdev,struct charsd_dev,cdev);
	host = devp->host;

	filp->private_data = devp;
#ifdef YKDBG
	printk("CharSD-%d is opend!! card->rca = 0x%X\n", devp->pdev_id, devp->rca);
#endif
	if (devp->thread == NULL) {
		printk("CharSD-%d error: charsd thread was stopped!!!!!!!\n", devp->pdev_id);
		return -1;
	}
/*	if(host->pdev_id == 2) {
		if(jz_charsd_change_irqh(devp,CHARSD_IRQ) < 0)
			return -1;
	}
*/
	return 0;
}

int jz_charsd_release(struct inode *inode,struct file *filp)
{
	struct charsd_dev *devp;
	struct jz_mmc_host *host;

	devp = container_of(inode->i_cdev,struct charsd_dev,cdev);
	host = devp->host;
	devp->error = 0;
#ifdef YKDBG
	printk("CharSD-%d is closed!!\n", devp->pdev_id);
#endif
	return 0;
}

static const struct file_operations jz_charsd_fops = {
	.owner = THIS_MODULE,
	.llseek = jz_charsd_llseek,
	.read = jz_charsd_read,
	.write = jz_charsd_write,
	.compat_ioctl = jz_charsd_ioctl,
	.open = jz_charsd_open,
	.release = jz_charsd_release,
};

static int jz_charsd_setup_cdev(struct charsd_dev *devp,int index)
{
	int i,err,devno = MKDEV(jz_charsd_major,index);
	char thread_name[20];

	cdev_init(&devp->cdev,&jz_charsd_fops);
	devp->cdev.owner = THIS_MODULE;
	devp->use_cpu = 0;                                                        //swith use of cpu/dma transfer

	err = cdev_add(&devp->cdev,devno,1);
	if (err)
		printk(KERN_NOTICE "Error %d adding jz_charsd %d",err,index);

	switch (index) {
	case 0:
		device_create(jz_charsd_class, NULL, devno, NULL,"jz_charsd_0");
		break;
	case 1:
		device_create(jz_charsd_class, NULL, devno, NULL,"jz_charsd_1");
		break;
	default:
		printk("Charsd: Can't support so many Charsd drivers!!\n");
		break;
	}

	for (i = 0; i < NUM_OF_BUFFERS; i++) {
		struct charsd_palu_buffer *pb = &devp->palu_buffers[i];

		pb->buf = (unsigned char *)__get_free_pages(GFP_KERNEL,4);		//malloc 64k buffer
		pb->num = i;
		pb->state = PBUF_STATE_EMPTY;

		if (!pb->buf) {
			printk("===Jz_charsd: Palu_buffer got error!!\n");
			return -1;
		}
		pb->next = pb + 1;
	}
	devp->palu_buffers[NUM_OF_BUFFERS - 1].next = &devp->palu_buffers[0];
	devp->next_palu_buffer = &devp->palu_buffers[0];
	sprintf(thread_name ,"charsd_thread_%d", index);
	devp->thread = kthread_run(jz_charsd_write_thread, devp, thread_name);
	return 0;
}

static int __init init_jz_charsd_cdev(void)
{

	int result = 0;
	dev_t devno = MKDEV(jz_charsd_major,0);                               //get major & minor
	unsigned char num = NUMBER_OF_CHARSD;
	unsigned char i = 0;

	jz_charsd_class = class_create(THIS_MODULE, DEVNAME);
	if (IS_ERR(jz_charsd_class))
	{
		printk( KERN_DEBUG "Charsd: class_create error\n" );
		return -1;
	}
	
	if (jz_charsd_major) {
		register_chrdev_region(devno,num,DEVNAME);
	} else {
		result = alloc_chrdev_region(&devno,0,num,DEVNAME);
		jz_charsd_major = MAJOR(devno);
	}
#ifdef YKDBG
	printk("%s,major = %d\n",__func__,jz_charsd_major);
#endif

	if (result < 0)
		return result;
	
	jz_charsd_devp = kmalloc(num*sizeof(struct charsd_dev),GFP_KERNEL);
	if (!jz_charsd_devp) {
		result  = -ENOMEM;
		goto fail_malloc;
	}
	memset(jz_charsd_devp,0,num*sizeof(struct charsd_dev));
	
	for(i = 0;i < num;i++) {
		if (jz_charsd_setup_cdev(&jz_charsd_devp[i],i) < 0) {
			result  = -ENOMEM;
			goto fail_malloc;
		}
//		init_waitqueue_head(&(&jz_charsd_devp[i])->dma_wait_queue);
		init_waitqueue_head(&(&jz_charsd_devp[i])->palu_wait_queue);
		sema_init(&(&jz_charsd_devp[i])->sem, 1);
		sema_init(&(&jz_charsd_devp[i])->thread_sem, 1);
	}

	return 0;
	
fail_malloc:
	unregister_chrdev_region(devno,1);
	return result;

}

static void __exit exit_jz_charsd_cdev(void)
{
	unsigned char num = NUMBER_OF_CHARSD;
	unsigned char i = 0;
	struct charsd_dev *devp;

#ifdef YKDBG
	printk("%s!!\n",__func__);
#endif
	for(i = 0;i < num;i++) {
		devp = &(jz_charsd_devp[i]);
		cdev_del(&(devp->cdev));
		for (i = 0; i < NUM_OF_BUFFERS; i++)
			free_pages((unsigned int)(devp->palu_buffers[i].buf), 4);
		kthread_stop(devp->thread);
	}

	kfree(jz_charsd_devp);
	unregister_chrdev_region(MKDEV(jz_charsd_major,0),num);
}

module_init(init_jz_charsd_cdev);
module_exit(exit_jz_charsd_cdev);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ingenic");
MODULE_DESCRIPTION("Direct read/write for Android USB-SDcard transfer");
