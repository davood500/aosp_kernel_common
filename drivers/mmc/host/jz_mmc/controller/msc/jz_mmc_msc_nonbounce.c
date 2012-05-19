/*
 *  linux/drivers/mmc/host/jz_mmc/msc/jz_mmc_msc.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *
 * James for Z800 V.ALPHA
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */

#include <linux/dma-mapping.h>
#include <linux/semaphore.h>
#include <linux/kthread.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio.h>
#include <linux/scatterlist.h>

#include <asm/jzsoc.h>
#include <asm/jzmmc/jz_mmc_msc.h>
#include <asm/jzmmc/jz_mmc_charsd.h>
//#define SG_DBG

#define WAITMASK  \
(MSC_STAT_CRC_RES_ERR | \
MSC_STAT_CRC_READ_ERROR | MSC_STAT_CRC_WRITE_ERROR_MASK | \
 MSC_STAT_TIME_OUT_RES | MSC_STAT_TIME_OUT_READ)

static int jz_mmc_data_done(struct jz_mmc_host *host);

static inline void jz_mmc_clean_curr(struct jz_mmc_host *host)
{
	host->curr.mrq = NULL;
	host->curr.cmd = NULL;
	host->curr.data = NULL;	
}

static void jz_mmc_handle_exception(struct jz_mmc_host *host)
{
	jz_mmc_stop_dma(host);
	host->curr.mrq->cmd->error = -ENOMEDIUM;
	mmc_request_done(host->mmc, host->curr.mrq);
	jz_mmc_clean_curr(host);
}

static void msc_irq_mask_all(int msc_id)
{
	REG_MSC_IMASK(msc_id) = 0xffff;
	REG_MSC_IREG(msc_id) = 0xffff;
}

static void jz_mmc_reset(struct jz_mmc_host *host)
{
	REG_MSC_STRPCL(host->pdev_id) = MSC_STRPCL_RESET;
 	while (REG_MSC_STAT(host->pdev_id) & MSC_STAT_IS_RESETTING);
}

#if defined(CONFIG_SOC_JZ4760B) && defined(CONFIG_NEED_HIGHSPEED)
static inline int msc_4760B_calc_clkrt(struct jz_mmc_host *host, int is_low, u32 rate)
{
	u32 clkrt;
	u32 clk_src = is_low ? 25000000 : 50000000;

	clkrt = 0;
	while (rate < clk_src) {
		clkrt++;
		clk_src >>= 1;
	}

	clkrt &= ~MSC_CLKRT_CLK_DIV_MASK;
	clkrt |= is_low ? MSC_CLKRT_CLK_SRC_DIV_2 : MSC_CLKRT_CLK_SRC_DIV_1;
	
//	REG_MSC_LPM(host->pdev_id) &= ~MSC_SET_HISPD;
//	REG_MSC_LPM(host->pdev_id) |= is_low ? 0 : MSC_SET_HISPD;
	
	return clkrt;
}

void jz_mmc_4760B_set_clock(struct jz_mmc_host *host, int rate)
{
	int clkrt;

	if(host->plat->support_sdio == 0)
		REG_MSC_LPM(host->pdev_id) = 0x1;	// Low power mode
	else
		REG_MSC_STRPCL(host->pdev_id) |= MSC_STRPCL_CLOCK_CONTROL_START;

	cpm_set_clock(CGU_MSCCLK, 50 * 1000 * 1000);
	if (rate > SD_CLOCK_FAST) {
		clkrt = msc_4760B_calc_clkrt(host, 0, rate);
		if(host->plat->strong_clk)
			host->plat->strong_clk();
	} else {
		clkrt = msc_4760B_calc_clkrt(host, 1, rate);
		if(host->plat->common_clk)
			host->plat->common_clk();
	}
//	printk("%d: rate = %d, clkrt = %d\n", host->pdev_id, rate, clkrt);

	REG_MSC_CLKRT(host->pdev_id) = clkrt;
}

#elif defined(CONFIG_SOC_JZ4770)
static inline int msc_4770_calc_clkrt(struct jz_mmc_host *host, int is_low, u32 rate)
{
	u32 clkrt;
	u32 clk_src = is_low ? 25000000 : 50000000;

	clkrt = 0;
	while (rate < clk_src) {
		clkrt++;
		clk_src >>= 1;
	}

	REG_MSC_LPM(host->pdev_id) &= ~MSC_SET_HISPD;
	REG_MSC_LPM(host->pdev_id) |= is_low ? 0 : MSC_SET_HISPD;

	return clkrt;
}

void jz_mmc_4770_set_clock(struct jz_mmc_host *host, int rate)
{
	int clkrt;
	int rate_real = rate;

	if(host->plat->support_sdio == 0)
		REG_MSC_LPM(host->pdev_id) = 0x1;	// Low power mode
	else
		REG_MSC_STRPCL(host->pdev_id) |= MSC_STRPCL_CLOCK_CONTROL_START;

// Definef in kernel/arch/mips/include/asm/jzmmc/jz_mmc_host.h
// SD_CLOCK_FAST   25000000
// SDIO_CLOCK_HIGH 25000000
// SD_CLOCK_HIGH   50000000
// Make sure SD_CLOCK_HIGH >= SD_CLOCK_FAST, SD_CLOCK_HIGH >= SDIO_CLOCK_HIGH
 
	if(host->plat->support_sdio) {
		if(rate > SDIO_CLOCK_HIGH)
			rate = SDIO_CLOCK_HIGH;
		if(rate >= SD_CLOCK_HIGH) {
			rate = SD_CLOCK_HIGH;

			switch(host->pdev_id) {
			case 0:
				cpm_set_clock(CGU_MSC0CLK, 50 * 1000 * 1000);
				break;
			case 1:
				cpm_set_clock(CGU_MSC1CLK, 50 * 1000 * 1000);
				break;
			case 2:
				cpm_set_clock(CGU_MSC2CLK, 50 * 1000 * 1000);
				break;
			}
			clkrt = msc_4770_calc_clkrt(host, 0, rate);
		} else {
			switch(host->pdev_id) {
			case 0:
				cpm_set_clock(CGU_MSC0CLK, 25 * 1000 * 1000);
				break;
			case 1:
				cpm_set_clock(CGU_MSC1CLK, 25 * 1000 * 1000);
				break;
			case 2:
				cpm_set_clock(CGU_MSC2CLK, 25 * 1000 * 1000);
				break;
			}
			clkrt = msc_4770_calc_clkrt(host, 1, rate);
		}
	} else {
		if (rate > SD_CLOCK_FAST) {
			if(rate > SD_CLOCK_HIGH)
				rate = SD_CLOCK_HIGH;

			switch(host->pdev_id) {
			case 0:
				cpm_set_clock(CGU_MSC0CLK, 50 * 1000 * 1000);
				break;
			case 1:
				cpm_set_clock(CGU_MSC1CLK, 50 * 1000 * 1000);
				break;
			case 2:
				cpm_set_clock(CGU_MSC2CLK, 50 * 1000 * 1000);
				break;
			}
			clkrt = msc_4770_calc_clkrt(host, 0, rate);
		} else {
			switch(host->pdev_id) {
			case 0:
				cpm_set_clock(CGU_MSC0CLK, 25 * 1000 * 1000);
				break;
			case 1:
				cpm_set_clock(CGU_MSC1CLK, 25 * 1000 * 1000);
				break;
			case 2:
				cpm_set_clock(CGU_MSC2CLK, 25 * 1000 * 1000);
				break;
			}
			clkrt = msc_4770_calc_clkrt(host, 1, rate);
		}
	}

	printk("%d: real-rate = %d, rate = %d, clkrt = %d\n", host->pdev_id, rate_real, rate, clkrt);

	REG_MSC_CLKRT(host->pdev_id) = clkrt;
}

#else

static inline int msc_default_calc_clkrt(int is_low, u32 rate)
{
	u32 clkrt;
	u32 clk_src = is_low ? 24000000 : 48000000;

	clkrt = 0;
	while (rate < clk_src) {
		clkrt++;
		clk_src >>= 1;
	}
	return clkrt;
}

void jz_mmc_default_set_clock(struct jz_mmc_host *host, int rate)
{
	int clkrt;

	/* __cpm_select_msc_clk_high will select 48M clock for MMC/SD card
	 * perhaps this will made some card with bad quality init fail,or
	 * bad stabilization.
	*/
	
	// Cause there is only ONE devider in CPM, the clock must only <= 24MHz
	int rate_real = rate;
#if 0
	if (rate > SD_CLOCK_FAST) {
		cpm_set_clock(CGU_MSCCLK, 48 * 1000 * 1000);
		clkrt = msc_default_calc_clkrt(0, rate);
	} else {
		cpm_set_clock(CGU_MSCCLK, 24 * 1000 * 1000);
		clkrt = msc_default_calc_clkrt(1, rate);
	}
#else
	if (rate > SD_CLOCK_FAST) {
		rate = SD_CLOCK_FAST;
		cpm_set_clock(CGU_MSCCLK, 25 * 1000 * 1000);
		clkrt = msc_default_calc_clkrt(1, rate);
	} else {
		cpm_set_clock(CGU_MSCCLK, 25 * 1000 * 1000);
		clkrt = msc_default_calc_clkrt(1, rate);
	}
#endif
//	printk("%d: real-rate = %d, rate = %d, clkrt = %d\n", host->pdev_id, rate_real, rate, clkrt);

	REG_MSC_CLKRT(host->pdev_id) = clkrt;
}
 
#endif

static void jz_mmc_enable_irq(struct jz_mmc_host *host, unsigned int mask)
{
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	host->imask &= ~mask;
	REG_MSC_IMASK(host->pdev_id) = host->imask;
	spin_unlock_irqrestore(&host->lock, flags);
}

static void jz_mmc_disable_irq(struct jz_mmc_host *host, unsigned int mask)
{
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	host->imask |= mask;
	REG_MSC_IMASK(host->pdev_id) = host->imask;
	spin_unlock_irqrestore(&host->lock, flags);
}

static int jz_mmc_status_polling(struct jz_mmc_host *host, unsigned int target)
{
	unsigned int stat = 0;
	struct mmc_command *cmd = host->curr.cmd;
	struct mmc_command *stop_cmd = host->curr.mrq->stop;
	struct mmc_data *data = host->curr.data;

	while( !((REG_MSC_STAT(host->pdev_id) & target) ||
		 (host->eject) || (REG_MSC_STAT(host->pdev_id) & WAITMASK)) )
		;

	if(host->eject) {

		printk("WARNNING:data trans eject\n");
		jz_mmc_handle_exception(host);
		return -1;
		
	} else if(REG_MSC_STAT(host->pdev_id) & WAITMASK) {
		
		printk("JZMMC(%d): CMD%d error report--", host->pdev_id, cmd->opcode);
 		
		stat = REG_MSC_STAT(host->pdev_id);		

		if (stat & MSC_STAT_TIME_OUT_READ) {
			REG_MSC_IREG(host->pdev_id) = MSC_IREG_TIMEOUT_READ;
			if(data)
				data->error = -ETIMEDOUT;
			printk("Timeout while reading\n");
		} else if (stat & MSC_STAT_TIME_OUT_RES) {
			REG_MSC_IREG(host->pdev_id) = MSC_IREG_TIMEOUT_RES;
			cmd->error = -ETIMEDOUT;
			printk("Timeout while responsing\n");
		} else if (stat & MSC_STAT_CRC_WRITE_ERROR_NOSTS) {
			REG_MSC_IREG(host->pdev_id) = MSC_IREG_CRC_WRITE_ERR;
			if(data)
				data->error = -EIO;
			printk("No CRC status sent back while writing\n");
		} else if (stat & MSC_STAT_CRC_WRITE_ERROR) {
			REG_MSC_IREG(host->pdev_id) = MSC_IREG_CRC_WRITE_ERR;
			if(data)
				data->error = -EIO;
			printk("CRC error while writing\n");
		} else if (stat & MSC_STAT_CRC_READ_ERROR) {
			REG_MSC_IREG(host->pdev_id) = MSC_IREG_CRC_READ_ERR;
			if(data)
				data->error = -EIO;
			printk("CRC error while reading\n");
		} else if (stat & MSC_STAT_CRC_RES_ERR) {
			REG_MSC_IREG(host->pdev_id) = MSC_IREG_CRC_RES_ERR;
			cmd->error = -EIO;
			printk("CRC error while responsing\n");
		} else {
			cmd->error = -EIO;
			printk("stat = %x\n", stat);
		}

		jz_mmc_stop_dma(host);

		if (stop_cmd) {
//			printk("--------------------\n");
			// Send CMD12 to stop data trans
			REG_MSC_CMD(host->pdev_id) = stop_cmd->opcode;
			REG_MSC_ARG(host->pdev_id) = stop_cmd->arg;

			REG_MSC_CMDAT(host->pdev_id) = MSC_CMDAT_BUSY | MSC_CMDAT_RESPONSE_R1;
			
			REG_MSC_RESTO(host->pdev_id) = 0xff;
	
			REG_MSC_STRPCL(host->pdev_id) |= MSC_STRPCL_START_OP;

			host->curr.mrq->stop = NULL;

			if(jz_mmc_status_polling(host, MSC_STAT_PRG_DONE)) {
//				printk("--------------------1\n");
				return -1;
			}
			REG_MSC_IREG(host->pdev_id) = MSC_IREG_PRG_DONE;
		}

	        if(cmd->opcode != 53) {
			mmc_request_done(host->mmc, host->curr.mrq);
			jz_mmc_clean_curr(host);
		} else 
			cmd->error = 0;

//		printk("JZMMC(%d):in %s, opcode = %d, arg = %X\n",host->pdev_id,__func__,(int)cmd->opcode,cmd->arg);

		return -1;

	} else {
		//REG_MSC_IREG(host->pdev_id) = target;
		return 0;
	}
}

static int jz_mmc_cmd_done(struct jz_mmc_host *host)
{
	struct mmc_command *cmd = host->curr.cmd;
	struct mmc_data *data = host->curr.data;
	int i;
	unsigned short buf[5];
	unsigned int res, v, w1, w2;

	if (!cmd)
		return -1;

	switch (host->curr.r_type) {
	case 1:
	{
		
		res = REG_MSC_RES(host->pdev_id);
		buf[0] = (res >> 8) & 0xff;
		buf[1] = res & 0xff;
		res = REG_MSC_RES(host->pdev_id);
		buf[2] = (res >> 8) & 0xff;
		buf[3] = res & 0xff;

		res = REG_MSC_RES(host->pdev_id);
		buf[4] = res & 0xff;

		cmd->resp[0] =
			buf[1] << 24 | buf[2] << 16 | buf[3] << 8 |
			buf[4];

		//printk("cmd->resp[0] = 0x%08x\n",cmd->resp[0]);

		if(cmd->opcode == 5 || cmd->opcode == 41 || cmd->opcode == 1) {

			if(buf[0] == buf[2] && buf[1] == buf[3]) {
				//printk("cmd->opcode = %d cmd->resp[0] = 0x%08x\n", cmd->opcode, cmd->resp[0]);
				
				if(host->plat->status_irq) {

					if(host->detect_retry++ < 10) {
						mmc_detect_change(host->mmc, 50);
						mdelay(900);
					} else {
						printk("The card is there, BUT can't work, please check the connection...\n");
						host->detect_retry = 0;
					}
				}


				cmd->error = -ENOMEDIUM;
			}
		}

		//printk("cmd->resp = 0x%08x\n", cmd->resp);
		break;
	}
	case 2:
	{
		res = REG_MSC_RES(host->pdev_id);
		v = res & 0xffff;
		for (i = 0; i < 4; i++) {
			res = REG_MSC_RES(host->pdev_id);
			w1 = res & 0xffff;
			res = REG_MSC_RES(host->pdev_id);
			w2 = res & 0xffff;
			cmd->resp[i] = v << 24 | w1 << 8 | w2 >> 8;
			v = w2;
		}
		break;
	}
	case 0:
		break;
	}

	if (data && cmd->error == 0) {
		if((data->flags == MMC_DATA_WRITE)) {
			jz_mmc_enable_irq(host, MSC_IMASK_DATA_TRAN_DONE);
			if(host->irq_is_on == 0) {
				enable_irq(host->irqres->start);
				host->irq_is_on = 1;
			}
		}
			
	} else {
		mmc_request_done(host->mmc, host->curr.mrq);
		jz_mmc_clean_curr(host);
	}

	return -1;
}

void jz_mmc_data_start(struct jz_mmc_host *host)
{
	struct mmc_data *data = host->curr.data;
	unsigned int nob = data->blocks;
	unsigned int block_size = data->blksz;
	int channel;
	int mode;
	
	REG_MSC_RDTO(host->pdev_id) = 0xffffff;

	if (data->flags & MMC_DATA_WRITE) {
		mode = DMA_MODE_WRITE;
		host->dma.dir = DMA_TO_DEVICE;
	} else {
		mode = DMA_MODE_READ;
		host->dma.dir = DMA_FROM_DEVICE;
	}

	channel = host->dma.channel;

	if (data->flags & MMC_DATA_STREAM)
		nob = 0xffff;

	REG_MSC_NOB(host->pdev_id) = nob;
	REG_MSC_BLKLEN(host->pdev_id) = block_size;

	//printk("nob = %d block_size = %d\n", nob, block_size);

	host->dma.len = data->sg_len;

	jz_mmc_start_scatter_dma(channel, host, data->sg, host->dma.len, mode);

	if((data->flags & MMC_DATA_READ) && (host->dma_start_addr != 0)) {
		jz_mmc_enable_irq(host, MSC_IMASK_DATA_TRAN_DONE);
		if(host->irq_is_on == 0) {
			enable_irq(host->irqres->start);
			host->irq_is_on = 1;
		}
	}
}

#ifdef CONFIG_RESP_PROG_IRQ_MODE
int jz_mmc_wait_resprog(struct jz_mmc_host *host, unsigned int target)
{
	unsigned int stat = 0;
	struct mmc_command *cmd = host->curr.cmd;
	struct mmc_command *stop_cmd = host->curr.mrq->stop;
	struct mmc_data *data = host->curr.data;

	switch (target){
	case MSC_IREG_END_CMD_RES:
		if((data || (cmd && (cmd->opcode == 13))) && host->mmc->card) {
			wait_event_interruptible_timeout(host->msc_wait_queue, 
							 (((host->resp_ack) || (host->eject) 
							   || (REG_MSC_STAT(host->pdev_id) & WAITMASK))),
							 5 * HZ);
			jz_mmc_disable_irq(host, MSC_IMASK_END_CMD_RES);
			
			if(!((host->resp_ack) || (host->eject) || (REG_MSC_STAT(host->pdev_id) & WAITMASK)))
				printk("JZMMC(%d): CMD%d error report--Time out while waiting irq-responsing ,%d\n"
				       ,host->pdev_id,cmd->opcode,host->mmc->card->type);
			host->resp_ack = 0;
			break;
		} else 
			return jz_mmc_status_polling(host, MSC_STAT_END_CMD_RES);
		
	case MSC_IREG_PRG_DONE:
		if(host->mmc->card && (host->mmc->card->type < 2)) {
			jz_mmc_enable_irq(host, MSC_IREG_PRG_DONE);
			if (host->irq_is_on == 0) {
				enable_irq(host->irqres->start);
				host->irq_is_on = 1;
			}		
			wait_event_interruptible_timeout(host->msc_wait_queue, 
							 (((host->resp_ack) || (host->eject) 
							   || (REG_MSC_STAT(host->pdev_id) & WAITMASK))),
							 5 * HZ);
			jz_mmc_disable_irq(host, MSC_IMASK_PRG_DONE);
			if(!((host->resp_ack) || (host->eject) || (REG_MSC_STAT(host->pdev_id) & WAITMASK)))
				printk("JZMMC(%d): CMD%d error report--Time out while waiting irq-programing\n"
				       ,host->pdev_id,cmd->opcode);
			host->resp_ack = 0;
			break;
		} else
			return jz_mmc_status_polling(host, MSC_STAT_PRG_DONE);
	default:
		printk("JZMMC(%d): unrecognized target in %s\n",host->pdev_id, __func__);
		break;
	}

	if(host->eject) {
		
		printk("WARNNING(%d):data trans eject\n",host->pdev_id);
		jz_mmc_handle_exception(host);		
		return -1;
		
	} else if (REG_MSC_STAT(host->pdev_id) & WAITMASK) {
		
		printk("JZMMC(%d): CMD%d error report target = %X--", host->pdev_id, (int)cmd->opcode, target);
		
		stat = REG_MSC_STAT(host->pdev_id);		
		
		if (stat & MSC_STAT_TIME_OUT_READ) {
			REG_MSC_IREG(host->pdev_id) = MSC_IREG_TIMEOUT_READ;
			if(data)
				data->error = -ETIMEDOUT;
			printk("Timeout while reading\n");
		} else if (stat & MSC_STAT_TIME_OUT_RES) {
			REG_MSC_IREG(host->pdev_id) = MSC_IREG_TIMEOUT_RES;
			cmd->error = -ETIMEDOUT;
			printk("Timeout while responsing\n");
		} else if (stat & MSC_STAT_CRC_WRITE_ERROR) {
			REG_MSC_IREG(host->pdev_id) = MSC_IREG_CRC_WRITE_ERR;
			if(data)
				data->error = -EIO;
			printk("CRC error while writing\n");
		} else if (stat & MSC_STAT_CRC_READ_ERROR) {
			REG_MSC_IREG(host->pdev_id) = MSC_IREG_CRC_READ_ERR;
			if(data)
				data->error = -EIO;
			printk("CRC error while reading\n");
		} else if (stat & MSC_STAT_CRC_RES_ERR) {
			REG_MSC_IREG(host->pdev_id) = MSC_IREG_CRC_RES_ERR;
			cmd->error = -EIO;
			printk("CRC error while responsing\n");
		} else {
			cmd->error = -EIO;
			printk("stat = %x\n", stat);
		}
		
		jz_mmc_stop_dma(host);
		
		if (stop_cmd) {
//			printk("--------------------\n");
			// Send CMD12 to stop data trans
			REG_MSC_CMD(host->pdev_id) = stop_cmd->opcode;
			REG_MSC_ARG(host->pdev_id) = stop_cmd->arg;
			
			REG_MSC_CMDAT(host->pdev_id) = MSC_CMDAT_BUSY | MSC_CMDAT_RESPONSE_R1;
			
			REG_MSC_RESTO(host->pdev_id) = 0xff;
			
			REG_MSC_STRPCL(host->pdev_id) |= MSC_STRPCL_START_OP;
			
			host->curr.mrq->stop = NULL;
			
			if(jz_mmc_status_polling(host, MSC_STAT_PRG_DONE)) {
//				printk("--------------------1\n");
				return -1;
			}
			REG_MSC_IREG(host->pdev_id) = MSC_IREG_PRG_DONE;
		}
		
		if(cmd->opcode != 53) {
			mmc_request_done(host->mmc, host->curr.mrq);
			jz_mmc_clean_curr(host);
		} else 
			cmd->error = 0;
		
		return -1;
		
	}
	return 0;
}
#endif

static void jz_mmc_execute_cmd(struct jz_mmc_host *host, struct mmc_command *cmd, unsigned int cmdat)
{
	struct mmc_data *data = host->curr.data;
	unsigned int stat = 0;
	int retry_count = 0;

	WARN_ON(host->curr.cmd != NULL);
	host->curr.cmd = cmd;

CMD_RETRY:

//	printk("jz_mmc_execute_cmd: pdev_id = %d,cmd->opcode = %d\n",host->pdev_id,cmd->opcode);

#ifdef SG_DBG
	printk("===id: %d | opcode = %d | arg = 0x%08X",host->pdev_id,cmd->opcode,cmd->arg);
	if(host->curr.data) {
		printk(" | data->sg_num = %d | data->blocks = 0x%08X | len = %dk\n"
		       ,host->curr.data->sg_len,data->blocks,data->blocks / 2);
	} else {
		printk("\n");
	}
#endif
	/* mask interrupts */
	REG_MSC_IMASK(host->pdev_id) = 0xffff;

	/* clear status */
	REG_MSC_IREG(host->pdev_id) = 0xffff;

	if(cmd->opcode == MMC_WRITE_BLOCK || cmd->opcode == MMC_WRITE_MULTIPLE_BLOCK) {

		//printk("cmd->arg = %d data->blksz = %d\n", cmd->arg, data->blksz);
		if(host->plat->partitions) {

			if(host->plat->permission == MMC_BOOT_AREA_PROTECTED && (cmd->arg < (host->plat->protect_boundary / data->blksz))) {
				
			printk("WARNNING: This erea is write_protected cmd->arg = %d data->blksz = %d\n", cmd->arg, data->blksz);
			data->error = -EACCES;
			jz_mmc_stop_dma(host);
			
			mmc_request_done(host->mmc, host->curr.mrq);
			jz_mmc_clean_curr(host);
			
			return;
			}
		}
	}

	if (cmd->flags & MMC_RSP_BUSY)
		cmdat |= MSC_CMDAT_BUSY;

	switch (RSP_TYPE(mmc_resp_type(cmd))) {
	case RSP_TYPE(MMC_RSP_R1):	// r1, r1b, r5, r6, r7
		cmdat |= MSC_CMDAT_RESPONSE_R1;
		host->curr.r_type = 1;
		break;
	case RSP_TYPE(MMC_RSP_R3):	// r3, r4
		cmdat |= MSC_CMDAT_RESPONSE_R3;
		host->curr.r_type = 1;
		break;
	case RSP_TYPE(MMC_RSP_R2):	// r2
		cmdat |= MSC_CMDAT_RESPONSE_R2;
		host->curr.r_type = 2;
		break;
	default:
		break;
	}

	REG_MSC_CMD(host->pdev_id) = cmd->opcode;

	/* Set argument */
	if(host->plat->bus_width == 1) {
		if (cmd->opcode == 6) {
			/* set  1 bit sd card bus*/
			if (cmd->arg == 2)
				REG_MSC_ARG(host->pdev_id) = 0;

			/* set  1 bit mmc card bus*/
			if (cmd->arg == 0x3b70101) {
				REG_MSC_ARG(host->pdev_id) = 0x3b70001;
			}
		} else
			REG_MSC_ARG(host->pdev_id) = cmd->arg;
	} else if(host->plat->bus_width == 8) {
		if (cmd->opcode == 6) {
			/* set  8 bit mmc card bus*/
			if (cmd->arg == 0x3b70101)
				REG_MSC_ARG(host->pdev_id) = 0x3b70201;
			else
				REG_MSC_ARG(host->pdev_id) = cmd->arg;
		} else
			REG_MSC_ARG(host->pdev_id) = cmd->arg;
	} else {
		REG_MSC_ARG(host->pdev_id) = cmd->arg;
	}

	/* Set command */
	REG_MSC_CMDAT(host->pdev_id) = cmdat | MSC_CMDAT_RTRG_EQUALT_16 | MSC_CMDAT_TTRG_LESS_16;
	
	// Multi-read || Multi-write
	if(cmd->opcode == MMC_READ_MULTIPLE_BLOCK || cmd->opcode == MMC_WRITE_MULTIPLE_BLOCK)
		REG_MSC_CMDAT(host->pdev_id) |= MSC_CMDAT_SEND_AS_STOP;
	
	if(data && (data->flags & MMC_DATA_READ))
		jz_mmc_data_start(host);

	REG_MSC_RESTO(host->pdev_id) = 0xff;

#ifdef CONFIG_RESP_PROG_IRQ_MODE
	if((data || (cmd && (cmd->opcode == 13))) && host->mmc->card) {
		jz_mmc_enable_irq(host, MSC_IMASK_END_CMD_RES);
		if (host->irq_is_on == 0) {
			enable_irq(host->irqres->start);
			host->irq_is_on = 1;
		}
	}
#endif 
        /* Send command */
	REG_MSC_STRPCL(host->pdev_id) |= MSC_STRPCL_START_OP;

#ifdef CONFIG_RESP_PROG_IRQ_MODE
	if(jz_mmc_wait_resprog(host, MSC_IREG_END_CMD_RES) < 0) {
		if((cmd != NULL) && (cmd->opcode == 53)) {
			if(retry_count++ < 3) {
				printk("=========>CMD53 retry\n");
				if(host->irq_is_on) {
					disable_irq_nosync(host->irqres->start);
					host->irq_is_on = 0;
				}				
				goto CMD_RETRY;
			} else {
				if(host->irq_is_on) {
					disable_irq_nosync(host->irqres->start);
					host->irq_is_on = 0;
				}
				cmd->error = -ETIMEDOUT;
				mmc_request_done(host->mmc, host->curr.mrq);
				jz_mmc_clean_curr(host);
			}
		} else if(host->irq_is_on) { 
			disable_irq_nosync(host->irqres->start);
			host->irq_is_on = 0;
		}
		
		return;
	}
#else
	if(jz_mmc_status_polling(host, MSC_STAT_END_CMD_RES)) {
		if((cmd != NULL) && (cmd->opcode == 53)) {
			if(retry_count++ < 3) {
				printk("=========>CMD53 retry\n");
				if(host->irq_is_on) {
					disable_irq_nosync(host->irqres->start);
					host->irq_is_on = 0;
				}				
				goto CMD_RETRY;
			} else {
				if(host->irq_is_on) {
					disable_irq_nosync(host->irqres->start);
					host->irq_is_on = 0;
				}
				cmd->error = -ETIMEDOUT;
				mmc_request_done(host->mmc, host->curr.mrq);
				jz_mmc_clean_curr(host);
			}
		} else if(host->irq_is_on) { 
			disable_irq_nosync(host->irqres->start);
			host->irq_is_on = 0;
		}

		return;
	}
#endif
	REG_MSC_IREG(host->pdev_id) = MSC_IREG_END_CMD_RES;	// clear irq flag 

	jz_mmc_cmd_done(host);

	if (host->curr.data) {
		
		if(host->curr.data->flags & MMC_DATA_WRITE) {

			jz_mmc_data_start(host);

			if(host->cpu_trans_bytes == 0)
				wait_event_interruptible_timeout(host->msc_wait_queue, 
								 ((host->msc_ack) || (host->eject) 
								  || (REG_MSC_STAT(host->pdev_id) & WAITMASK)),
								 5 * HZ);

			else
				wait_event_interruptible_timeout(host->dma_wait_queue, 
							       ((host->dma_ack) || (host->eject) 
								|| (REG_MSC_STAT(host->pdev_id) & WAITMASK) || 
								(host->dma_start_addr == 0)), 5 * HZ);

			//printk("host->msc_ack = %d host->eject = %d\n", host->msc_ack, host->eject);

			if((host->cpu_trans_bytes == 0 && host->msc_ack) || (host->cpu_trans_bytes != 0 && host->dma_ack)
			   || (host->cpu_trans_bytes != 0 && host->dma_start_addr == 0)) {

				jz_mmc_data_done(host);

			} else if(host->eject) {
				
				printk("WARNNING:data trans eject\n");
				jz_mmc_handle_exception(host);
				
			} else {
				stat = REG_MSC_STAT(host->pdev_id);

				if (stat & MSC_STAT_CRC_WRITE_ERROR_MASK) {
					REG_MSC_IREG(host->pdev_id) = MSC_IREG_CRC_READ_ERR;
					data->error = -EIO;
					printk("CRC error while writing\n");
				} else {
					data->error = -EIO;
					printk("stat = %x\n", stat);
				}
				
				printk("W-REG_DMAC_DTCR(%d) = %x\n", host->dma.channel, REG_DMAC_DTCR(host->dma.channel));

				jz_mmc_handle_exception(host);
			}

			host->msc_ack = 0;
			host->dma_ack = 0;

		} else if(host->curr.data->flags & MMC_DATA_READ) {
			wait_event_interruptible_timeout(host->dma_wait_queue, 
							 ((host->dma_ack) || (host->eject) 
							  || (REG_MSC_STAT(host->pdev_id) & WAITMASK) || 
							  (host->dma_start_addr == 0)),
							 5 * HZ);

//			printk("host->msc_ack = %d host->eject = %d\n", host->msc_ack, host->eject);

			if(host->dma_ack || host->dma_start_addr == 0) {

				jz_mmc_data_done(host);

			} else if(host->eject) {
				
				printk("WARNNING:data trans eject\n");
				jz_mmc_handle_exception(host);
				
			} else {
				stat = REG_MSC_STAT(host->pdev_id);

				if (stat & MSC_STAT_TIME_OUT_READ) {
					REG_MSC_IREG(host->pdev_id) = MSC_IREG_TIMEOUT_READ;
					data->error = -ETIMEDOUT;
					printk("Timeout while reading\n");
				} else if (stat & MSC_STAT_CRC_READ_ERROR) {
					REG_MSC_IREG(host->pdev_id) = MSC_IREG_CRC_READ_ERR;
					data->error = -EIO;
					printk("CRC error while reading\n");
				} else {
					data->error = -EIO;
					printk("stat = %x\n", stat);
				}

				printk("R-REG_DMAC_DTCR(%d) = %x\n", host->dma.channel, REG_DMAC_DTCR(host->dma.channel));

				jz_mmc_handle_exception(host);
			}

			host->dma_ack = 0;

		} else {
			;
	
		}
	}
}

static int jz_mmc_data_done(struct jz_mmc_host *host)
{
	struct mmc_data *data = host->curr.data;
	unsigned int tmp_buf = 0, buf = 0;
	int error = 0;
	unsigned int *buf_32;
	unsigned char *buf_8;
	int i;
	unsigned int dma_len = sg_dma_len(host->curr.data->sg);

	if (!data){
		return -1;
	}

	if(host->cpu_trans_bytes) {

		if(host->flag_cp)
			memcpy((void *)sg_virt(data->sg), host->dma_buf, dma_len - (dma_len % 4));		

		REG_MSC_CMDAT(host->pdev_id) &= (~MSC_CMDAT_DMA_EN);
		
		buf_32 = (unsigned int *)host->cpu_start_addr;

		if(host->curr.data->flags & MMC_DATA_WRITE) {

			for(i = 0; i < host->cpu_trans_bytes / 4; i++) {
				error = 0;
				while ((REG_MSC_STAT(host->pdev_id) & MSC_STAT_DATA_FIFO_FULL) && !host->eject) {
					if(REG_MSC_STAT(host->pdev_id) & WAITMASK) {
						error = 1;
						break;
					}
				}

				if(error) {
					printk("ERROR: error while writing stat = %x\n", REG_MSC_STAT(host->pdev_id));
					data->error = -EIO;
					data->bytes_xfered = 0;
					mmc_request_done(host->mmc, host->curr.mrq);
					jz_mmc_clean_curr(host);
					return -1;
				} else if(host->eject) {
					data->error = -ENOMEDIUM;
					data->bytes_xfered = 0;
					mmc_request_done(host->mmc, host->curr.mrq);
					jz_mmc_clean_curr(host);
					return -1;
				}
				
				REG_MSC_TXFIFO(host->pdev_id) = *buf_32++;
			}

			if(host->cpu_trans_bytes % 4) {

				buf_8 = (unsigned char *)buf_32;
				
				switch (host->cpu_trans_bytes % 4) {
				case 1:	
					tmp_buf = *buf_8;
					break;
				case 2:
					buf = *buf_8++;
					tmp_buf = buf;
					buf = *buf_8++;
					tmp_buf |= buf << 8;
					break;
				case 3:
					buf = *buf_8++;
					tmp_buf = buf;
					buf = *buf_8++;
					tmp_buf |= buf << 8;
					buf = *buf_8++;
					tmp_buf |= buf << 16;
					break;
				default:
					break;
				}

				error = 0;
				while ((REG_MSC_STAT(host->pdev_id) & MSC_STAT_DATA_FIFO_FULL) && !host->eject) {
					if(REG_MSC_STAT(host->pdev_id) & WAITMASK) {
						error = 1;
						break;
					}
				}

				if(error) {
					printk("ERROR: error while writing stat = %x\n", REG_MSC_STAT(host->pdev_id));
					data->error = -EIO;
					data->bytes_xfered = 0;
					mmc_request_done(host->mmc, host->curr.mrq);
					jz_mmc_clean_curr(host);
					return -1;
				} else if(host->eject) {
					data->error = -ENOMEDIUM;
					data->bytes_xfered = 0;
					mmc_request_done(host->mmc, host->curr.mrq);
					jz_mmc_clean_curr(host);
					return -1;
				}
				
				REG_MSC_TXFIFO(host->pdev_id) = tmp_buf;
			}

		} else if(host->curr.data->flags & MMC_DATA_READ) {

			for(i = 0; i < host->cpu_trans_bytes / 4; i++) {
				error = 0;
				while ((REG_MSC_STAT(host->pdev_id) & MSC_STAT_DATA_FIFO_EMPTY) && !host->eject) {
					if(REG_MSC_STAT(host->pdev_id) & WAITMASK) {
						error = 1;
						break;
					}
				}

				if(error) {
					printk("ERROR: error while reading stat = %x\n", REG_MSC_STAT(host->pdev_id));
					data->error = -EIO;
					data->bytes_xfered = 0;
					mmc_request_done(host->mmc, host->curr.mrq);
					jz_mmc_clean_curr(host);
					return -1;
				} else if(host->eject) {
					data->error = -ENOMEDIUM;
					data->bytes_xfered = 0;
					mmc_request_done(host->mmc, host->curr.mrq);
					jz_mmc_clean_curr(host);
					return -1;
				}

				*buf_32++ = REG_MSC_RXFIFO(host->pdev_id);
			}

			if(host->cpu_trans_bytes % 4)
			{
				error = 0;
				while ((REG_MSC_STAT(host->pdev_id) & MSC_STAT_DATA_FIFO_EMPTY) && !host->eject) {
					if(REG_MSC_STAT(host->pdev_id) & WAITMASK) {
						error = 1;
						break;
					}
				}

				if(error) {
					printk("ERROR: error while reading stat = %x\n", REG_MSC_STAT(host->pdev_id));
					data->error = -EIO;
					data->bytes_xfered = 0;
					mmc_request_done(host->mmc, host->curr.mrq);
					jz_mmc_clean_curr(host);
					return -1;
				} else if(host->eject) {
					data->error = -ENOMEDIUM;
					data->bytes_xfered = 0;
					mmc_request_done(host->mmc, host->curr.mrq);
					jz_mmc_clean_curr(host);
					return -1;
				}

				tmp_buf = REG_MSC_RXFIFO(host->pdev_id);
				buf_8 = (unsigned char *)buf_32;
				switch (host->cpu_trans_bytes % 4) {
				case 1:	
					*buf_8 = tmp_buf & 0xff;
					break;
				case 2:
					*buf_8++ = ((tmp_buf >> (0)) & 0xff);
					*buf_8++ = ((tmp_buf >> (8)) & 0xff);
					break;
				case 3:
					
					*buf_8++ = ((tmp_buf >> (0)) & 0xff);
					*buf_8++ = ((tmp_buf >> (8)) & 0xff);
					*buf_8++ = ((tmp_buf >> (16)) & 0xff);
					break;
				default:
					break;
				}
			}
		}

		host->flag_cp = 0;
	
	} else {
		if(host->flag_cp)
			memcpy((void *)sg_virt(data->sg), host->dma_buf, dma_len);

		host->flag_cp = 0;
	}

	/*
	 * There appears to be a hardware design bug here.  There seems to
	 * be no way to find out how much data was transferred to the card.
	 * This means that if there was an error on any block, we mark all
	 * data blocks as being in error.
	 */
	if (data->error == 0)
		data->bytes_xfered = data->blocks * data->blksz;
	else
		data->bytes_xfered = 0;
	if (host->curr.mrq->stop) {
                // Multi-read || Multi-write)
		//if(cmd->opcode == MMC_READ_MULTIPLE_BLOCK || cmd->opcode == MMC_WRITE_MULTIPLE_BLOCK)
		
		if(jz_mmc_status_polling(host, MSC_STAT_AUTO_CMD_DONE)) {
			return -1;
		}

		REG_MSC_CMDAT(host->pdev_id) &= ~(MSC_CMDAT_SEND_AS_STOP);
	}
	
	if(host->curr.data->flags & MMC_DATA_WRITE) {

#ifdef CONFIG_RESP_PROG_IRQ_MODE1
		if(!(REG_MSC_STAT(host->pdev_id) & MSC_STAT_PRG_DONE)) {
			jz_mmc_wait_resprog(host, MSC_IREG_PRG_DONE);
		}
#else
		if(jz_mmc_status_polling(host, MSC_STAT_PRG_DONE)) {
			if((host->curr.cmd != NULL) && (host->curr.cmd->opcode == 53)) {
				if(host->irq_is_on) {
					disable_irq_nosync(host->irqres->start);
					host->irq_is_on = 0;
				}
				host->curr.cmd->error = -ETIMEDOUT;
				mmc_request_done(host->mmc, host->curr.mrq);
				jz_mmc_clean_curr(host);
			} else if(host->irq_is_on) { 
				disable_irq_nosync(host->irqres->start);
				host->irq_is_on = 0;
			}
			return -1;
		}
#endif
		REG_MSC_IREG(host->pdev_id) = MSC_IREG_PRG_DONE;
		REG_MSC_IREG(host->pdev_id) = MSC_IREG_DATA_TRAN_DONE;
	} else if(host->curr.data->flags & MMC_DATA_READ) {
		if(jz_mmc_status_polling(host, MSC_STAT_DATA_TRAN_DONE)) {
			if((host->curr.cmd != NULL) && (host->curr.cmd->opcode == 53)) {
				if(host->irq_is_on) {
					disable_irq_nosync(host->irqres->start);
					host->irq_is_on = 0;
				}
				host->curr.cmd->error = -ETIMEDOUT;
				mmc_request_done(host->mmc, host->curr.mrq);
				jz_mmc_clean_curr(host);
			} else if(host->irq_is_on) { 
				disable_irq_nosync(host->irqres->start);
				host->irq_is_on = 0;
			}
			return -1;
		}
		REG_MSC_IREG(host->pdev_id) = MSC_IREG_DATA_TRAN_DONE;
	}

	mmc_request_done(host->mmc, host->curr.mrq);
	jz_mmc_clean_curr(host);

	return 0;
}

static irqreturn_t jz_mmc_irq(int irq, void *devid)
{
	struct jz_mmc_host *host = devid;
	struct mmc_data *data = host->curr.data;
	unsigned int ireg = 0;
	unsigned int ireg2 = 0;
	int handled = 0;
	
	disable_irq_nosync(host->irqres->start);
	host->irq_is_on = 0;

	ireg = REG_MSC_IREG(host->pdev_id);
#ifdef CONFIG_RESP_PROG_IRQ_MODE
	switch (ireg & (MSC_IREG_END_CMD_RES | MSC_IREG_DATA_TRAN_DONE)) {// | MSC_IREG_PRG_DONE)) {

	case MSC_IREG_END_CMD_RES:
		if(host->eject) {
			handled = 1;
			break;
		}
		host->resp_ack = 1;
		handled = 1;
		wake_up_interruptible(&host->msc_wait_queue);
		ireg2 = REG_MSC_IREG(host->pdev_id) & 0xffff;
		if(!(ireg2 & MSC_IREG_DATA_TRAN_DONE))
			break;

	case (MSC_IREG_END_CMD_RES | MSC_IREG_DATA_TRAN_DONE):
		jz_mmc_disable_irq(host, MSC_IMASK_DATA_TRAN_DONE);
		host->resp_ack = 1;
		handled = 1;
		wake_up_interruptible(&host->msc_wait_queue);
		if(data == 0) {
			handled = 1;
			break;
		}		
		if(host->eject) {
			handled = 1;
			break;
		}
				
		if(data->flags & MMC_DATA_WRITE) {
			if(ireg & MSC_IREG_CRC_WRITE_ERR)
				;
			else
				host->msc_ack = 1;
			wake_up_interruptible(&host->msc_wait_queue);
		} else if(data->flags & MMC_DATA_READ) {
			if(ireg & (MSC_IREG_TIMEOUT_READ | MSC_IREG_CRC_READ_ERR))
				wake_up_interruptible(&host->dma_wait_queue);
		}
		handled = 1;
		break;

	case MSC_IREG_DATA_TRAN_DONE:
		jz_mmc_disable_irq(host, MSC_IMASK_DATA_TRAN_DONE);
		
		if(data == 0) {
			return IRQ_RETVAL(1);
		} 
		
		if(host->eject) {
			return IRQ_RETVAL(1);
		}
		if(data->flags & MMC_DATA_WRITE) {
			host->msc_ack = 1;
			wake_up_interruptible(&host->msc_wait_queue);
		}
		handled = 1;
		break;

	case (MSC_IREG_DATA_TRAN_DONE | MSC_IREG_PRG_DONE):
		jz_mmc_disable_irq(host, MSC_IMASK_DATA_TRAN_DONE);
		jz_mmc_disable_irq(host, MSC_IMASK_PRG_DONE);

		if(data == 0) {
			return IRQ_RETVAL(1);
		} 
		
		if(host->eject) {
			return IRQ_RETVAL(1);
		}
		if(host->mmc->card && (host->mmc->card->type < 2))
			if (host->msc_ack)
				host->resp_ack = 1;
			else
				host->msc_ack = 1;
		else
			host->msc_ack = 1;
		handled = 1;
		wake_up_interruptible(&host->msc_wait_queue);
		break;

	default:
		break;
	}

	if(data && (data->flags & MMC_DATA_WRITE)) {
		if(ireg & MSC_IREG_CRC_WRITE_ERR)
			wake_up_interruptible(&host->msc_wait_queue);
	} else if(data && (data->flags & MMC_DATA_READ)) {
		if(ireg & (MSC_IREG_TIMEOUT_READ | MSC_IREG_CRC_READ_ERR))
			wake_up_interruptible(&host->dma_wait_queue);
	}
	
	return IRQ_RETVAL(handled);
#else
	if (ireg) {

		if (ireg & MSC_IREG_DATA_TRAN_DONE) {

			jz_mmc_disable_irq(host, MSC_IMASK_DATA_TRAN_DONE);

			if(data == 0) {
				return IRQ_RETVAL(1);
			}

			if(host->eject) {
				return IRQ_RETVAL(1);
			}

	
			if(data->flags & MMC_DATA_WRITE) {
				if(ireg & MSC_IREG_CRC_WRITE_ERR)
					;
				else
					host->msc_ack = 1;
				wake_up_interruptible(&host->msc_wait_queue);
			} else if(data->flags & MMC_DATA_READ) {
				if(ireg & (MSC_IREG_TIMEOUT_READ | MSC_IREG_CRC_READ_ERR))
					wake_up_interruptible(&host->dma_wait_queue);
			}
			
			handled = 1;
		}
	}

	return IRQ_RETVAL(handled);
#endif
}

static int jz_mmc_msc_init(struct jz_mmc_host *host)
{
	int ret = 0;

	jz_mmc_reset(host);

	REG_MSC_RDTO(host->pdev_id) = 0xffffff;
	REG_MSC_RESTO(host->pdev_id) = 0xff;
	
	if(host->plat->support_sdio == 0)
		REG_MSC_LPM(host->pdev_id) = 0x1;	// Low power mode
	else
		REG_MSC_STRPCL(host->pdev_id) |= MSC_STRPCL_CLOCK_CONTROL_START;

	host->msc_ack = 0;
	host->dma_ack = 0;
	host->detect_retry = 0;

	init_waitqueue_head(&host->msc_wait_queue);
	init_waitqueue_head(&host->dma_wait_queue);

	msc_irq_mask_all(host->pdev_id);

	ret = request_irq(host->irqres->start, jz_mmc_irq, 0, "jz-msc (msc)", host);
	if (ret) {
		printk(KERN_ERR "MMC/SD: can't request MMC/SD IRQ\n");
		return ret;
	}
	disable_irq_nosync(host->irqres->start);
	host->irq_is_on = 0;

	return 0;
}

static void jz_mmc_msc_deinit(struct jz_mmc_host *host)
{
	free_irq(host->irqres->start, &host);
}

int jz_mmc_msc_register(struct jz_mmc_msc *msc)
{
	if(msc == NULL)
		return -ENOMEM;

	msc->init = jz_mmc_msc_init;
	msc->deinit = jz_mmc_msc_deinit;
	msc->execute_cmd = jz_mmc_execute_cmd;
#if defined(CONFIG_SOC_JZ4760B) && defined(CONFIG_NEED_HIGHSPEED)
	msc->set_clock = jz_mmc_4760B_set_clock;
#elif defined(CONFIG_SOC_JZ4770)
	msc->set_clock = jz_mmc_4770_set_clock;
#else
	msc->set_clock = jz_mmc_default_set_clock;
#endif
	return 0;
}
