/*
 *  linux/drivers/mmc/host/jz_mmc/dma/jz_mmc_dma.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */

#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>

#include <asm/jzsoc.h>
#include <asm/jzmmc/jz_mmc_dma.h>

#define WAITMASK  \
(MSC_STAT_CRC_RES_ERR | \
MSC_STAT_CRC_READ_ERROR | MSC_STAT_CRC_WRITE_ERROR_MASK | \
 MSC_STAT_TIME_OUT_RES | MSC_STAT_TIME_OUT_READ)

static void sg_to_desc(struct scatterlist *sgentry, struct jz_mmc_host *host, int mode) {
	dma_addr_t dma_addr, best_dma_addr;
	unsigned int dma_len;
	unsigned int head_unalign_size = 0;
	unsigned int aligned_times;
#ifdef CONFIG_PARALLEL_DMA_MODE1
	if(host->trans_state == MSC_CARD_MAL) {
		dma_len = SBUF * 2;
		dma_addr = CPHYSADDR(host->dma_stream_buf);
	} else {
		dma_addr = sg_phys(sgentry);
		dma_len = sg_dma_len(sgentry);
	}
#else
	dma_addr = sg_phys(sgentry);
	dma_len = sg_dma_len(sgentry);
#endif     
	//BUG_ON(dma_len % 4); /* we do NOT support transfer size < 4byte */

	if (!(dma_addr & 0xf0000000)) {
		dma_addr += 0x20000000;
	}

	if(dma_len <= DMA_TS) {

		// CPU
		host->cpu_start_addr = (unsigned int)sg_virt(sgentry);
		host->cpu_trans_bytes = dma_len;

		host->dma_start_addr = 0;

		host->flag_cp = 0;

		//printk("host->cpu_start_addr = %x, host->cpu_trans_bytes = %d\n", host->cpu_start_addr, host->cpu_trans_bytes);

	} else { 

		aligned_times = dma_len / DMA_TS;
		host->aligned_bytes = aligned_times * DMA_TS;
		host->unaligned_bytes = dma_len - host->aligned_bytes;
		host->cpu_trans_bytes = host->unaligned_bytes;

		best_dma_addr = (dma_addr + DMA_TS - 1) & ~(DMA_TS - 1);
		
		head_unalign_size =  best_dma_addr - dma_addr;
		
		BUG_ON(head_unalign_size % 4);
		
		//printk("aligned_times = %d host->aligned_bytes = %d head_unalign_size = %d\n", aligned_times, host->aligned_bytes, head_unalign_size);

		//printk("host->unaligned_bytes = %d, host->cpu_trans_bytes = %d\n", host->unaligned_bytes, host->cpu_trans_bytes);

		if(head_unalign_size) {

			if (DMA_MODE_WRITE == mode) {
				memcpy(host->dma_buf, (const void *)sg_virt(sgentry), host->aligned_bytes);
				dma_cache_wback_inv((unsigned long)host->dma_buf, host->aligned_bytes);
				
				host->dma_start_addr = CPHYSADDR(host->dma_buf);
				host->cpu_start_addr = (unsigned int)sg_virt(sgentry) + host->aligned_bytes;

				host->flag_cp = 0;

			} else if (DMA_MODE_READ == mode){
				dma_cache_inv((unsigned long)host->dma_buf, host->aligned_bytes);

				host->dma_start_addr = CPHYSADDR(host->dma_buf);
				host->cpu_start_addr = (unsigned int)sg_virt(sgentry) + host->aligned_bytes;
				
				host->flag_cp = 1;
			} else {

				printk("ERROR:%s: DMA mode error\n", __FUNCTION__);
			}		
			
		} else {
			if (DMA_MODE_WRITE == mode) {
				dma_cache_wback_inv((unsigned long)sg_virt(sgentry), host->aligned_bytes);

				host->dma_start_addr = dma_addr;

				if(host->cpu_trans_bytes)
					host->cpu_start_addr = (unsigned int)sg_virt(sgentry) + host->aligned_bytes;

				host->flag_cp = 0;
			} else if (DMA_MODE_READ == mode){
				dma_cache_inv((unsigned long)sg_virt(sgentry), host->aligned_bytes);

				host->dma_start_addr = dma_addr;

				if(host->cpu_trans_bytes)
					host->cpu_start_addr = (unsigned int)sg_virt(sgentry) + host->aligned_bytes;

				host->flag_cp = 0;
			} else {
				printk("ERROR:%s: DMA mode error\n", __FUNCTION__);
			}
		}
		
	}
}

void jz_mmc_start_scatter_dma(int chan, struct jz_mmc_host *host,
			      struct scatterlist *sg, unsigned int sg_len, int mode) {
	int i = 0;
	struct mmc_data *data = host->curr.data;
	struct scatterlist *sgentry;
	unsigned long flags;
	unsigned int dcmd = 0;

	flags = claim_dma_lock();
	for_each_sg(data->sg, sgentry, host->dma.len, i) {
		sg_to_desc(sgentry, host, mode);
	}

	if(host->dma_start_addr != 0) {

		//printk("DMA start: host->aligned_bytes = %d\n", host->aligned_bytes);

		/* Clear DMA channel control/status register */
		REG_DMAC_DCCSR(chan) = 0;
		
		REG_DMAC_DTCR(chan) = host->aligned_bytes / DMA_TS;
		
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
		
		/* Setup request source */
		if (DMA_MODE_WRITE == mode) {
			
			REG_DMAC_DCMD(chan) = (dcmd | DMAC_DCMD_SAI);
			
			REG_DMAC_DSAR(chan) = host->dma_start_addr;
			
			REG_DMAC_DTAR(chan) = CPHYSADDR(MSC_TXFIFO(host->pdev_id));
			
			if(host->pdev_id == 0)
				REG_DMAC_DRSR(chan) = DMAC_DRSR_RS_MSC0OUT;
			else if(host->pdev_id == 1)
				REG_DMAC_DRSR(chan) = DMAC_DRSR_RS_MSC1OUT;
			else
				REG_DMAC_DRSR(chan) = DMAC_DRSR_RS_MSC2OUT;
		} else if (DMA_MODE_READ == mode) {
			
			REG_DMAC_DCMD(chan) = (dcmd | DMAC_DCMD_DAI);
			
			REG_DMAC_DSAR(chan) = CPHYSADDR(MSC_RXFIFO(host->pdev_id));
			
			REG_DMAC_DTAR(chan) = host->dma_start_addr;
			
			if(host->pdev_id == 0)
				REG_DMAC_DRSR(chan) = DMAC_DRSR_RS_MSC0IN;
			else if(host->pdev_id == 1)
				REG_DMAC_DRSR(chan) = DMAC_DRSR_RS_MSC1IN;
			else
				REG_DMAC_DRSR(chan) = DMAC_DRSR_RS_MSC2IN;
		} else {
			printk("ERROR:%s: DMA mode error\n", __FUNCTION__);
		}
		
		/* Enable DMA */
		REG_DMAC_DMACR(chan / HALF_DMA_NUM) |= DMAC_DMACR_DMAE;
		
		/* Setup DMA channel control/status register */
		REG_DMAC_DCCSR(chan) = DMAC_DCCSR_NDES | DMAC_DCCSR_EN;
	}

	release_dma_lock(flags);
}

void jz_mmc_stop_dma(struct jz_mmc_host *host)
{
	printk("mmc:%s\n", __FUNCTION__);

	// Stop all
	REG_DMAC_DCCSR(host->dma.channel) = 0;
	
	// Clear all
	REG_DMAC_DCMD(host->dma.channel) = 0;
	REG_DMAC_DSAR(host->dma.channel) = 0;
	REG_DMAC_DTAR(host->dma.channel) = 0;
	REG_DMAC_DTCR(host->dma.channel) = 0;
	REG_DMAC_DRSR(host->dma.channel) = 0;
	REG_DMAC_DDA(host->dma.channel) = 0;
}

static irqreturn_t jz_mmc_dma_callback(int irq, void *devid)
{
	struct jz_mmc_host *host = devid;
	unsigned int chan = host->dma.channel;

//	printk("**************************tx\n");

	disable_dma(chan);
	if (__dmac_channel_address_error_detected(chan)) {
		printk("ERROR:%s: DMAC address error.\n", __FUNCTION__);
		__dmac_channel_clear_address_error(chan);
	}
	if (__dmac_channel_transmit_end_detected(chan)) {
		__dmac_channel_clear_transmit_end(chan);
#ifdef CONFIG_PARALLEL_DMA_MODE1
		if(host->trans_state == MSC_CARD_ALAL) {
			REG_MSC_IREG(host->pdev_id) = MSC_IREG_DATA_TRAN_DONE;
			host->dma_flag = 1;
			wake_up_interruptible(&host->parallel_queue);
			return IRQ_HANDLED;
		} else {
			host->dma_flag = 1;
			wake_up_interruptible(&host->dma_wait_queue);
		}
#else
		wake_up_interruptible(&host->dma_wait_queue);
#endif
		host->dma_ack = 1;
	}

 	return IRQ_HANDLED;
}

static int jz_mmc_init_dma(struct jz_mmc_host *host)
{
	host->dma.channel = -1;

	if (!host->dmares)
		return -ENODEV;

	host->dma.channel = jz_request_dma(host->dmares->start, "dma-msc", jz_mmc_dma_callback,
			       0, host);
	/* Start clock */
	__dmac_channel_enable_clk(host->dma.channel);

	printk("host->dma.channel = %d\n", host->dma.channel);
	if (host->dma.channel < 0) {
		printk(KERN_ERR "jz_request_dma failed for MMC \n");
		goto err1;
	}

	host->dma_buf = (unsigned int *)__get_free_pages(GFP_KERNEL,5);
#ifdef CONFIG_PARALLEL_DMA_MODE1
		host->dma_stream_buf = (unsigned int *)__get_free_pages(GFP_KERNEL,5);
#endif
        /* setup descriptor */
	host->dma_desc = (jz_dma_desc_8word *)__get_free_page(GFP_KERNEL);
	host->dma_desc_phys_addr = CPHYSADDR((unsigned long)host->dma_desc);

	memset(host->dma_desc, 0, 4096);
	dma_cache_wback_inv((unsigned long)host->dma_desc, 4096);

	return 0;

	jz_free_dma(host->dma.channel);
err1:
	return -ENODEV;
}

static void jz_mmc_deinit_dma(struct jz_mmc_host *host)
{
	free_page((unsigned int)host->dma_desc);
	free_pages((unsigned int)host->dma_buf, 5);
#ifdef CONFIG_PARALLEL_DMA_MODE1
	free_pages((unsigned int)host->dma_stream_buf, 5);
#endif
	jz_free_dma((unsigned int)host->dma.channel);
}

int jz_mmc_dma_register(struct jz_mmc_dma *dma)
{
	if(dma == NULL)
		return -ENOMEM;

	dma->init = jz_mmc_init_dma;
	dma->deinit = jz_mmc_deinit_dma;

	return 0;
}

#ifdef CONFIG_PARALLEL_DMA_MODE1
#ifndef CONFIG_RESP_PROG_IRQ_MODE
static int jz_mmc_qread_polling(struct jz_mmc_host *host, unsigned int target)
{
	unsigned int stat = 0;

	while( !((REG_MSC_STAT(host->pdev_id) & target) ||
		 (host->eject) || (REG_MSC_STAT(host->pdev_id) & WAITMASK)) )
		;

	if(host->eject) {
		
		printk("PARALLEL_DMA:data trans eject while polling\n");
		jz_mmc_stop_dma(host);

		return -1;		
	} else if(REG_MSC_STAT(host->pdev_id) & WAITMASK) {
		
		printk("JZMMC: CMD error report--");
 		
		stat = REG_MSC_STAT(host->pdev_id);		

		if (stat & MSC_STAT_TIME_OUT_READ) {
			REG_MSC_IREG(host->pdev_id) = MSC_IREG_TIMEOUT_READ;
			printk("PARALLEL_DMA:Timeout while reading\n");
		} else if (stat & MSC_STAT_TIME_OUT_RES) {
			REG_MSC_IREG(host->pdev_id) = MSC_IREG_TIMEOUT_RES;
			printk("PARALLEL_DMA:Timeout while responsing\n");
		} else if (stat & MSC_STAT_CRC_WRITE_ERROR) {
			REG_MSC_IREG(host->pdev_id) = MSC_IREG_CRC_WRITE_ERR;
			printk("PARALLEL_DMA:CRC error while writing\n");
		} else if (stat & MSC_STAT_CRC_READ_ERROR) {
			REG_MSC_IREG(host->pdev_id) = MSC_IREG_CRC_READ_ERR;
			printk("PARALLEL_DMA:CRC error while reading\n");
		} else if (stat & MSC_STAT_CRC_RES_ERR) {
			REG_MSC_IREG(host->pdev_id) = MSC_IREG_CRC_RES_ERR;
			printk("PARALLEL_DMA:CRC error while responsing\n");
		} else {
			printk("stat = %x\n", stat);
		}

		jz_mmc_stop_dma(host);

		return -1;
	} else {
		//REG_MSC_IREG(host->pdev_id) = target;
		return 0;
	}
}
#endif
int jz_mmc_quick_read(struct jz_mmc_host *host)
{
	unsigned long flags;
	unsigned int mask = MSC_IMASK_END_CMD_RES;

	jz_mmc_quick_dma(host);

	REG_MSC_CMD(host->pdev_id) = host->cmd_bak;

	REG_MSC_ARG(host->pdev_id) = host->arg_last + host->buf_gap;

	REG_MSC_CMDAT(host->pdev_id) = host->cmdat_bak | MSC_CMDAT_SEND_AS_STOP;
	REG_MSC_RESTO(host->pdev_id) = 0xff;

#ifdef CONFIG_RESP_PROG_IRQ_MODE
	spin_lock_irqsave(&host->lock, flags);
	host->imask &= ~mask;
	REG_MSC_IMASK(host->pdev_id) = host->imask;
	spin_unlock_irqrestore(&host->lock, flags);

	if (host->irq_is_on == 0) {
		enable_irq(host->irqres->start);
		host->irq_is_on = 1;
	}
#endif	
	REG_MSC_STRPCL(host->pdev_id) |= MSC_STRPCL_START_OP;

#ifdef CONFIG_RESP_PROG_IRQ_MODE
	if(jz_mmc_wait_resprog(host, MSC_IREG_END_CMD_RES)) {
		printk("PARALLEL_DMA:data trans error while waiting respons\n");
		return -1;
	}
#else
	if(jz_mmc_qread_polling(host, MSC_STAT_END_CMD_RES))
		return -1;
#endif
	REG_MSC_IREG(host->pdev_id) = MSC_IREG_END_CMD_RES;	/* clear irq flag */
	host->dma_ack = 0;
	return 0;
}

void jz_mmc_quick_dma(struct jz_mmc_host *host)
{
	unsigned long flags;

	REG_MSC_RDTO(host->pdev_id) = 0xffffff;
	REG_MSC_NOB(host->pdev_id) = SBUF / 512;
	REG_MSC_BLKLEN(host->pdev_id) = 512;

	flags = claim_dma_lock();

	REG_DMAC_DCCSR(host->dma.channel) = 0;
	REG_DMAC_DTCR(host->dma.channel) = SBUF / 64;
	REG_DMAC_DCMD(host->dma.channel) = DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_32 | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_TIE | DMAC_DCMD_DS_64BYTE | DMAC_DCMD_DAI;
	
	REG_DMAC_DSAR(host->dma.channel) = CPHYSADDR(MSC_RXFIFO(host->pdev_id));
	
	REG_DMAC_DTAR(host->dma.channel) = CPHYSADDR(host->dma_stream_buf + SBUF / 4);     //buffer

	dma_cache_inv((unsigned long)sg_virt(host->curr.data->sg), host->aligned_bytes);

	/* Enable DMA */
	REG_DMAC_DMACR(host->dma.channel / HALF_DMA_NUM) |= DMAC_DMACR_DMAE;	
	/* Setup DMA channel control/status register */
	REG_DMAC_DCCSR(host->dma.channel) = DMAC_DCCSR_NDES | DMAC_DCCSR_EN;
	release_dma_lock(flags);
}

#endif
