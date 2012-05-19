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

#define WAITMASK  \
(MSC_STAT_CRC_RES_ERR | \
MSC_STAT_CRC_READ_ERROR | MSC_STAT_CRC_WRITE_ERROR_MASK | \
 MSC_STAT_TIME_OUT_RES | MSC_STAT_TIME_OUT_READ)

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

	        if(!(stat & MSC_STAT_TIME_OUT_RES) || cmd->opcode != 53) {
			mmc_request_done(host->mmc, host->curr.mrq);
			jz_mmc_clean_curr(host);
		} else 
			cmd->error = 0;

		return -1;

	} else {
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

		if(cmd->opcode == 5 || cmd->opcode == 41 || cmd->opcode == 1) {

			if(buf[0] == buf[2] && buf[1] == buf[3]) {
				
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
	unsigned int *cpu_addr_ptr = NULL;
	unsigned int cpu_len = 0;
	struct scatterlist *sgentry;
	int i = 0;

	REG_MSC_RDTO(host->pdev_id) = 0xffffff;
	REG_MSC_NOB(host->pdev_id) = nob;
	REG_MSC_BLKLEN(host->pdev_id) = block_size;

	for_each_sg(data->sg, sgentry,data->sg_len, i) {		//scccccccc list
		cpu_addr_ptr = sg_virt(sgentry);
		cpu_len = sg_dma_len(sgentry);
	}

	if (data->flags & MMC_DATA_WRITE) {
		for(i = 0;i < (cpu_len / 4);i++) {
			while ((REG_MSC_STAT(host->pdev_id) & MSC_STAT_DATA_FIFO_FULL) && !host->eject);
			if(host->eject) {
				data->error = -ENOMEDIUM;
				break;
			}
			REG_MSC_TXFIFO(host->pdev_id) = *cpu_addr_ptr++;
		}
	} else if(data->flags & MMC_DATA_READ) {
		for(i = 0; i < cpu_len / 4; i++) {
			while ((REG_MSC_STAT(host->pdev_id) & MSC_STAT_DATA_FIFO_EMPTY) && !host->eject)
			if(host->eject) {
				data->error = -ENOMEDIUM;
				break;
			}
			*cpu_addr_ptr++ = REG_MSC_RXFIFO(host->pdev_id);
		}
	}
}

static int jz_mmc_data_done(struct jz_mmc_host *host)
{
	struct mmc_data *data = host->curr.data;

	if (data->error == 0)
		data->bytes_xfered = data->blocks * data->blksz;
		else
			data->bytes_xfered = 0;

	if (host->curr.mrq->stop) {
		if(jz_mmc_status_polling(host, MSC_STAT_AUTO_CMD_DONE)) {
			return 0;
		}

		REG_MSC_CMDAT(host->pdev_id) &= ~(MSC_CMDAT_SEND_AS_STOP);
	}

	if(host->curr.data->flags & MMC_DATA_WRITE) {
		if(jz_mmc_status_polling(host, MSC_STAT_PRG_DONE)) {
			return 0;
		}
		REG_MSC_IREG(host->pdev_id) = MSC_IREG_PRG_DONE;
	} else if(host->curr.data->flags & MMC_DATA_READ) {
		REG_MSC_IREG(host->pdev_id) = MSC_IREG_DATA_TRAN_DONE;
	}

	mmc_request_done(host->mmc, host->curr.mrq);
	host->curr.mrq = NULL;
	host->curr.cmd = NULL;
	host->curr.data = NULL;

	return 1;
}

static void jz_mmc_execute_cmd(struct jz_mmc_host *host, struct mmc_command *cmd, unsigned int cmdat)
{
	struct mmc_data *data = host->curr.data;
	int retry_count = 0;

	WARN_ON(host->curr.cmd != NULL);
	host->curr.cmd = cmd;

CMD_RETRY:

//	printk("jz_mmc_execute_cmd: pdev_id = %d,cmd->opcode = %d\n",host->pdev_id,cmd->opcode);

	/* mask interrupts */
	REG_MSC_IMASK(host->pdev_id) = 0xffff;

	/* clear status */
	REG_MSC_IREG(host->pdev_id) = 0xffff;

	if(cmd->opcode == MMC_WRITE_BLOCK || cmd->opcode == MMC_WRITE_MULTIPLE_BLOCK) {

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

		cmdat &= (~MSC_CMDAT_DMA_EN);

	/* Set command */
	REG_MSC_CMDAT(host->pdev_id) = cmdat | MSC_CMDAT_RTRG_EQUALT_16 | MSC_CMDAT_TTRG_LESS_16;
	
	// Multi-read || Multi-write
	if(cmd->opcode == MMC_READ_MULTIPLE_BLOCK || cmd->opcode == MMC_WRITE_MULTIPLE_BLOCK)
		REG_MSC_CMDAT(host->pdev_id) |= MSC_CMDAT_SEND_AS_STOP;
	
	REG_MSC_RESTO(host->pdev_id) = 0xff;

        /* Send command */
	REG_MSC_STRPCL(host->pdev_id) |= MSC_STRPCL_START_OP;


	if(jz_mmc_status_polling(host, MSC_STAT_END_CMD_RES)) {
		if(cmd->opcode == 53) {
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

	REG_MSC_IREG(host->pdev_id) = MSC_IREG_END_CMD_RES;

	jz_mmc_cmd_done(host);

	if (host->curr.data) {
		jz_mmc_data_start(host);
		jz_mmc_data_done(host);
	}
}


static int jz_mmc_msc_init(struct jz_mmc_host *host)
{
	jz_mmc_reset(host);
	REG_MSC_RDTO(host->pdev_id) = 0xffffff;
	REG_MSC_RESTO(host->pdev_id) = 0xff;
	
	if(host->plat->support_sdio == 0)
		REG_MSC_LPM(host->pdev_id) = 0x1;
	else
		REG_MSC_STRPCL(host->pdev_id) |= MSC_STRPCL_CLOCK_CONTROL_START;

	host->msc_ack = 0;
	host->dma_ack = 0;
	host->detect_retry = 0;

	msc_irq_mask_all(host->pdev_id);

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
