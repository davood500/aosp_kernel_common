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

static inline unsigned int get_tsz(unsigned int dma_len)
{
	unsigned int tsz = 0;

	switch (dma_len) {
	case 0 :
		printk("JZMMC: Descriptor-len = 0!!\n");
		break;
	case 1 ... 3:
		tsz = 1;
		break;
	case 4 ... 15:
		tsz = 4;
		break;
	case 16 ... 31:
		tsz = 16;
		break;
	case 32 ... 63:
		tsz = 32;
		break;
	case 64:
		tsz = 32;
		break;
	default:
		tsz = 64;
		break;
	}
	return tsz;
}

static inline unsigned int get_dcmd_ds(unsigned int tsz)
{
	unsigned int dcmd = 0;

	switch (tsz) { 
	case 64:
		dcmd = DMAC_DCMD_DS_64BYTE;
		break;
	case 32:
		dcmd = DMAC_DCMD_DS_32BYTE;
		break;
	case 16:
		dcmd = DMAC_DCMD_DS_16BYTE;
		break;
	case 4:
		dcmd = DMAC_DCMD_DS_32BIT;
		break;
	case 1:
		printk("JZMMC-Descriptor: Can not set Data-size as 8 bit!!\n");
		dcmd = DMAC_DCMD_DS_8BIT;
		break;
	default:
		printk("JZMMC: get_dcmd error!\n");
		break;
	}
	
	return dcmd;
}

static void jz_sg_cache(struct scatterlist *sgl, unsigned int nents, int is_write, unsigned int dma_len)
{
	struct sg_mapping_iter miter;
	unsigned long flags;
	unsigned int sg_flags = SG_MITER_ATOMIC;
	unsigned int offset = 0;

	dma_addr_t page_phys = 0;
	dma_addr_t page_phys_pre = 0;

	if (is_write)
		sg_flags |= SG_MITER_FROM_SG;
	else
		sg_flags |= SG_MITER_TO_SG;

	sg_miter_start(&miter, sgl, nents, sg_flags);

	local_irq_save(flags);
	while (sg_miter_next(&miter) && offset < dma_len) {
		unsigned int len;
		
		len = min(miter.length, dma_len - offset);
		page_phys = page_to_phys(miter.page);
		page_phys_pre = page_phys;
		
		if (is_write)
			dma_cache_wback_inv((unsigned long)miter.addr, len);
		else
			dma_cache_inv((unsigned long)miter.addr, len);

		offset += len;
		if (offset >= dma_len)
			break;
	}

	sg_miter_stop(&miter);
	local_irq_restore(flags);
}

static void sg_to_desc(struct scatterlist *sgentry, struct jz_mmc_host *host, int mode, struct desc_hd *dhd, unsigned int sgs_last) {
	dma_addr_t dma_addr, best_dma_addr;
	unsigned int dma_len;
	unsigned int head_unalign_size = 0;
	unsigned int tsz = 0;
	unsigned int dma_addr_next = 0;
	unsigned int dma_addr_now = 0;
	unsigned int dma_len_next = 0;
	unsigned int dcmd = 0;
	unsigned int cnt = 0;
	unsigned char loop_cnt = 0;

	dma_addr = sg_phys(sgentry);
	dma_len = sg_dma_len(sgentry);

	if (!(dma_addr & 0xf0000000)) {
		dma_addr += 0x20000000;
	}

	tsz = get_tsz(dma_len);
	if (tsz == 1) {
		host->cpu_start_addr = (unsigned int)sg_virt(sgentry);
		host->cpu_trans_bytes = dma_len;
		
		host->dma_start_addr = 0;
		
		host->flag_cp = 0;
		return;
	}

	host->cpu_start_addr = 0;
	host->cpu_trans_bytes = 0;

	best_dma_addr = (dma_addr + tsz - 1) & ~(tsz - 1);

	head_unalign_size =  best_dma_addr - dma_addr;

	if (head_unalign_size) {

		if (DMA_MODE_WRITE == mode) {
			memcpy(host->dma_buf, (const void *)sg_virt(sgentry), dma_len - (dma_len % 4));
			dma_cache_wback_inv((unsigned int)host->dma_buf, dma_len - (dma_len % 4));
			host->dma_start_addr = CPHYSADDR(host->dma_buf);

			host->flag_cp = 0;			
		} else if (DMA_MODE_READ == mode){
			dma_cache_inv((unsigned int)host->dma_buf, dma_len - (dma_len % 4));
			host->dma_start_addr = CPHYSADDR(host->dma_buf);

			host->flag_cp = 1;
		} else {
			
			printk("ERROR:%s: DMA mode error\n", __FUNCTION__);
		}		
		
	} else {
		if (DMA_MODE_WRITE == mode) {

			jz_sg_cache(sgentry, host->dma.len, 1, dma_len);
			host->dma_start_addr = dma_addr;

			host->flag_cp = 0;	
		} else if (DMA_MODE_READ == mode){

			jz_sg_cache(sgentry, host->dma.len, 0, dma_len);
			host->dma_start_addr = dma_addr;

			host->flag_cp = 0;
		} else {
			printk("ERROR:%s: DMA mode error\n", __FUNCTION__);
		}
	}

	dma_addr_next = (unsigned int)host->dma_start_addr;

	dhd->dma_desc->ddadr = 0;

	do {
		tsz = get_tsz(dma_len);
		dcmd = get_dcmd_ds(tsz);
		dcmd |= DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_32 | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_LINK;
		cnt = dma_len / tsz;
		dhd->dma_desc->ddadr = cnt;
		dhd->dma_desc->dcmd = dcmd;
		dma_len_next = dma_len % tsz;
		dma_addr_now = dma_addr_next;
		dma_addr_next += (cnt * tsz);

		if (DMA_MODE_WRITE == mode) {
			dhd->dma_desc->dcmd |= DMAC_DCMD_SAI;
			dhd->dma_desc->dsadr = dma_addr_now;
			dhd->dma_desc->dtadr = CPHYSADDR(MSC_TXFIFO(host->pdev_id));

			switch (host->pdev_id) {
			case 0 :
				dhd->dma_desc->dreqt = DMAC_DRSR_RS_MSC0OUT;
				break;
			case 1 :
				dhd->dma_desc->dreqt = DMAC_DRSR_RS_MSC1OUT;
				break;
			case 2 :
				dhd->dma_desc->dreqt = DMAC_DRSR_RS_MSC2OUT;
				break;
			default:
				;
			}

		} else if (DMA_MODE_READ == mode) {

			dhd->dma_desc->dcmd |= DMAC_DCMD_DAI;
			dhd->dma_desc->dsadr = CPHYSADDR(MSC_RXFIFO(host->pdev_id));
			dhd->dma_desc->dtadr = dma_addr_now;

			switch (host->pdev_id) {
			case 0 :
				dhd->dma_desc->dreqt = DMAC_DRSR_RS_MSC0IN;
				break;
			case 1 :
				dhd->dma_desc->dreqt = DMAC_DRSR_RS_MSC1IN;
				break;
			case 2 :
				dhd->dma_desc->dreqt = DMAC_DRSR_RS_MSC2IN;
				break;
			default:
				;
			}
		} else
			printk("%s: DMA mode error\n", __FUNCTION__);

		if ((dma_len_next > 3) || sgs_last) {
			dhd->dma_desc->ddadr |= (dhd->next->dma_desc_phys_addr >> 4) << 24;
			dhd = dhd->next;
			dma_len = dma_len_next;
		}

		loop_cnt++;

		if ((dma_len_next < 4) && dma_len_next) {
			host->cpu_start_addr = (unsigned long)sg_virt(sgentry) + host->dma_start_addr - dma_addr_now;
			host->cpu_trans_bytes = dma_len_next;
			break;
		}

	} while (dma_len_next);

	if (!(host->mmc->card && (host->mmc->card->type < 2))) {
		dhd->dma_desc->dcmd |= DMAC_DCMD_TIE;
		dhd->dma_desc->dcmd &= ~DMAC_DCMD_LINK;
		dhd->dma_desc->ddadr &= ~0xff000000;
		dma_cache_wback_inv((unsigned long)host->decshds[0].dma_desc, loop_cnt * (sizeof(jz_dma_desc_8word)));
	}
}

void jz_mmc_start_scatter_dma(int chan, struct jz_mmc_host *host,
			      struct scatterlist *sg, unsigned int sg_len, int mode) {
	int i = 0;
	struct mmc_data *data = host->curr.data;
	struct scatterlist *sgentry;
	unsigned long flags;
	struct desc_hd *dhd = &(host->decshds[0]);

	flags = claim_dma_lock();

	for_each_sg(data->sg, sgentry, host->dma.len, i) {
		sg_to_desc(sgentry, host, mode, dhd, host->dma.len - i - 1);
		if ((host->dma.len - i) > 1) {
			if (dhd->next == NULL)
				printk("JZMMC-%d:dhd->next == NULL\n",host->pdev_id);
			else
				dhd = dhd->next;
		}
	}

	if (host->mmc->card && (host->mmc->card->type < 2)) {
		dhd->dma_desc->dcmd |= DMAC_DCMD_TIE;
		dhd->dma_desc->dcmd &= ~DMAC_DCMD_LINK;
		dhd->dma_desc->ddadr &= ~0xff000000;
		dma_cache_wback_inv((unsigned long)host->decshds[0].dma_desc, host->dma.len * (sizeof(jz_dma_desc_8word)));
	}

	if(host->dma_start_addr != 0) {

		//printk("DMA start: host->aligned_bytes = %d\n", host->aligned_bytes);

		/* Clear DMA channel control/status register */
		REG_DMAC_DCCSR(chan) = 0;

		REG_DMAC_DDA(chan) = host->decshds[0].dma_desc_phys_addr;
		/* Setup request source */
		if (DMA_MODE_WRITE == mode) {

			if(host->pdev_id == 0)
				REG_DMAC_DRSR(chan) = DMAC_DRSR_RS_MSC0OUT;
			else if(host->pdev_id == 1)
				REG_DMAC_DRSR(chan) = DMAC_DRSR_RS_MSC1OUT;
			else
				REG_DMAC_DRSR(chan) = DMAC_DRSR_RS_MSC2OUT;
		} else if (DMA_MODE_READ == mode) {

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

		REG_DMAC_DMADBSR(chan / HALF_DMA_NUM) = 1 << (chan - (chan / HALF_DMA_NUM) * HALF_DMA_NUM);
		REG_DMAC_DCCSR(chan) = DMAC_DCCSR_EN | DMAC_DCCSR_DES8;
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

#ifdef DESC_DBG
	printk("JZMMC: DMA-IRQ is comming!!!\n");
#endif

	disable_dma(chan);
	if (__dmac_channel_address_error_detected(chan)) {
		printk("ERROR:%s: DMAC address error.\n", __FUNCTION__);
		__dmac_channel_clear_address_error(chan);
	}
	if (__dmac_channel_transmit_end_detected(chan)) {
		__dmac_channel_clear_transmit_end(chan);

		wake_up_interruptible(&host->dma_wait_queue);
		host->dma_ack = 1;
	}

 	return IRQ_HANDLED;
}

static int jz_mmc_init_dma(struct jz_mmc_host *host)
{
	jz_dma_desc_8word *next_desc; 
	unsigned char i = 0;

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

        /* setup descriptor */
	host->decshds[0].dma_desc = (jz_dma_desc_8word *)__get_free_page(GFP_KERNEL);
	next_desc = host->decshds[0].dma_desc;
	memset(host->decshds[0].dma_desc, 0, 4096);

	for (i = 0; i < NUM_DESC; ++i) {
		struct desc_hd *dhd = &host->decshds[i];
		dhd->dma_desc = next_desc;
		dhd->dma_desc_phys_addr = CPHYSADDR((unsigned long)dhd->dma_desc);
		next_desc += 1;
		dhd->next = dhd + 1;
	}
	host->decshds[NUM_DESC - 1].next = NULL;	

	dma_cache_wback_inv((unsigned long)host->decshds[0].dma_desc, 4096);
	return 0;
err1:
	jz_free_dma(host->dma.channel);

	return -ENODEV;
}

static void jz_mmc_deinit_dma(struct jz_mmc_host *host)
{
	free_page((unsigned int)host->dma_desc);
	free_pages((unsigned int)host->dma_buf, 5);
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
