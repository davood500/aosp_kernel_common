
/*
 *  linux/drivers/mmc/host/jz_mmc/jz_mmc_main.c - JZ SD/MMC driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/mm.h>
#include <linux/signal.h>
#include <linux/pm.h>
#include <linux/scatterlist.h>
#include <linux/mfd/wm831x/core.h>
#include <asm/io.h>
#include <asm/scatterlist.h>
#include <asm/sizes.h>
#include <asm/jzsoc.h>
#include <asm/jzmmc/jz_mmc_host.h>
#include <asm/jzmmc/jz_mmc_controller.h>

#define NUMBER_OF_CTRL 3

struct jz_mmc_controller controller[NUMBER_OF_CTRL];

static void jz_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct jz_mmc_host *host = mmc_priv(mmc);
	unsigned int cmdat;
	struct jz_mmc_functions *functions = host->plat->driver_data;

	cmdat = host->cmdat;
	host->cmdat &= ~MSC_CMDAT_INIT;

	if (host->eject) {

		host->for_sync++;

		if (mrq->data && !(mrq->data->flags & MMC_DATA_READ)) {
			mrq->cmd->error = -EIO;
			mrq->data->bytes_xfered = 0;
			//mrq->data->bytes_xfered = mrq->data->blksz *
			//			  mrq->data->blocks;
		} else
			mrq->cmd->error = -ENOMEDIUM;

		mmc_request_done(mmc, mrq);
		return;
	}

	host->curr.mrq = mrq;
	host->curr.data = mrq->data;
	host->curr.cmd = NULL;

	if(mrq->data) {
		cmdat &= ~MSC_CMDAT_BUSY;

		if ((mrq->cmd->opcode == 51) | (mrq->cmd->opcode == 8) | (mrq->cmd->opcode == 6)) {
			cmdat &= ~MSC_CMDAT_BUS_WIDTH_MASK;
			cmdat |= MSC_CMDAT_BUS_WIDTH_1BIT | MSC_CMDAT_DATA_EN | MSC_CMDAT_DMA_EN;
		} else
			cmdat |= MSC_CMDAT_DATA_EN | MSC_CMDAT_DMA_EN;

		if (mrq->data->flags & MMC_DATA_WRITE)
			cmdat |= MSC_CMDAT_WRITE;

		if (mrq->data->flags & MMC_DATA_STREAM)
			cmdat |= MSC_CMDAT_STREAM_BLOCK;
	}

	functions->execute_cmd(host, mrq->cmd, cmdat);
}

static int jz_mmc_get_ro(struct mmc_host *mmc)
{
	struct jz_mmc_host *host = mmc_priv(mmc);

	if(host->plat->write_protect != NULL)
		return host->plat->write_protect(mmc_dev(host->mmc));
	else
		return 0;
}

static int jz_mmc_get_cd(struct mmc_host *mmc)
{
	struct jz_mmc_host *host = mmc_priv(mmc);

	if(host->plat->status != NULL) {
		return host->plat->status(mmc_dev(host->mmc));
	}
	else
		return 1;
}

/* set clock and power */
static void jz_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct jz_mmc_host *host = mmc_priv(mmc);
	struct jz_mmc_functions *functions = host->plat->driver_data;
	struct device *dev = &mmc->class_dev;

	if(!functions) {
		printk("%s: functions is NULL!\n", __FUNCTION__);
		while(1);
	}

	if (ios->clock) {
		functions->set_clock(host, ios->clock);
	}

	switch(ios->power_mode) {
	case MMC_POWER_ON:
#ifndef CONFIG_JZ4760_Z800
		host->plat->power_on((struct device *)dev);
#endif
		host->cmdat |= CMDAT_INIT;
		break;
	case MMC_POWER_OFF:
#ifndef CONFIG_JZ4760_Z800
		host->plat->power_off((struct device *)dev);
#endif
		break;
	default:
		break;
	}

	if (ios->bus_width == MMC_BUS_WIDTH_4) {

		host->cmdat &= ~MSC_CMDAT_BUS_WIDTH_MASK;

		if(host->plat->bus_width == 4)
			host->cmdat |= MSC_CMDAT_BUS_WIDTH_4BIT;
		else
			host->cmdat |= host->plat->bus_width;
	} else if (ios->bus_width == MMC_BUS_WIDTH_8) {

		host->cmdat &= ~MSC_CMDAT_BUS_WIDTH_MASK;

		if(host->plat->bus_width == 8)
			host->cmdat |= MSC_CMDAT_BUS_WIDTH_8BIT;
//		else
//			host->cmdat |= host->plat->bus_width;
	} else {
		/* 1 bit bus*/
		host->cmdat &= ~MSC_CMDAT_BUS_WIDTH_8BIT;
	}
}

static ssize_t jz_mmc_partitions_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	int i;
	struct jz_mmc_platform_data *pdata = dev->platform_data;
	ssize_t count = 0;

	if(pdata->num_partitions == 0) {
		count = sprintf(buf, "null\n");
		return count;
	}

	for(i=0;i<pdata->num_partitions;i++)
		count += sprintf(buf+count, "%s %x %x %d\n",
				pdata->partitions[i].name,
				pdata->partitions[i].saddr,
				pdata->partitions[i].len,
				pdata->partitions[i].type);

	return count;
}

static ssize_t jz_mmc_permission_set(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct jz_mmc_platform_data *pdata = dev->platform_data;

	if (buf == NULL)
		return count;

	if (strcmp(buf, "RECOVERY_MODE") == 0) {
		printk("host->permission: MMC_BOOT_AREA_PROTECTED->MMC_BOOT_AREA_OPENED\n");
		pdata->permission = MMC_BOOT_AREA_OPENED;
	} else {
		printk("host->permission: MMC_BOOT_AREA_OPENED->MMC_BOOT_AREA_PROTECTED\n");
		pdata->permission = MMC_BOOT_AREA_PROTECTED;
	}

	return count;
}

static DEVICE_ATTR(partitions, S_IRUSR | S_IRGRP | S_IROTH, jz_mmc_partitions_show, NULL);
static DEVICE_ATTR(permission, S_IWUSR, NULL, jz_mmc_permission_set);

static struct attribute *jz_mmc_attributes[] = {
	&dev_attr_partitions.attr,
	&dev_attr_permission.attr,
	NULL
};

static const struct attribute_group jz_mmc_attr_group = {
	.attrs = jz_mmc_attributes,
};

static const struct mmc_host_ops jz_mmc_ops = {
	.request = jz_mmc_request,
	.get_ro = jz_mmc_get_ro,
	.set_ios = jz_mmc_set_ios,
	.get_cd = jz_mmc_get_cd,
};

static int jz_mmc_probe(struct platform_device *pdev)
{
	struct jz_mmc_platform_data *plat = pdev->dev.platform_data;
	struct mmc_host *mmc;
	struct jz_mmc_host *host = NULL;
//	struct jz_mmc_controller controller;
	struct jz_mmc_functions *functions;

	struct resource *irqres = NULL;
	struct resource *memres = NULL;
	struct resource *dmares = NULL;
	int i,ret;

	plat->cpm_start(&pdev->dev);

	if (!plat) {
		printk(KERN_ERR "%s: Platform data not available\n", __func__);
		return -EINVAL;
	}

	if (pdev->id < 0 || pdev->id > 3)
		return -EINVAL;

	// IORESOURCE_DMA is NOT required
	if (pdev->resource == NULL || pdev->num_resources < 2) {
		printk(KERN_ERR "%s: Invalid resource\n", __func__);
		return -ENXIO;
	}

	for (i = 0; i < pdev->num_resources; i++) {
		if (pdev->resource[i].flags & IORESOURCE_MEM)
			memres = &pdev->resource[i];
		if (pdev->resource[i].flags & IORESOURCE_IRQ)
			irqres = &pdev->resource[i];
		if (pdev->resource[i].flags & IORESOURCE_DMA)
			dmares = &pdev->resource[i];
	}
	if (!irqres || !memres) {
		printk(KERN_ERR "%s: Invalid resource\n", __func__);
		return -ENXIO;
	}

	/*
	 * Setup our host structure
	 */
	mmc = mmc_alloc_host(sizeof(struct jz_mmc_host), &pdev->dev);
	if (!mmc) {
		return -ENOMEM;
	}

	host = mmc_priv(mmc);
	host->pdev_id = pdev->id;
	host->plat = plat;
	host->mmc = mmc;
	host->for_sync = 0;

	// base address of MSC controller
	host->base = ioremap(memres->start, PAGE_SIZE);
	if (!host->base) {
		return -ENOMEM;
	}

	// back up these info. for future using
	host->irqres = irqres;
	host->memres = memres;
	host->dmares = dmares;
	host->imask = 0xffff;

	spin_lock_init(&host->lock);

	/*
	 * Setup MMC host structure
	 */
	if(host->plat->partitions)
		host->plat->permission = MMC_BOOT_AREA_PROTECTED;
	else
		host->plat->permission = MMC_BOOT_AREA_OPENED;

	mmc->ops = &jz_mmc_ops;
	mmc->f_min = MMC_CLOCK_SLOW;
	mmc->f_max = SD_CLOCK_HIGH;
	mmc->ocr_avail = plat->ocr_mask;
	mmc->caps |= host->plat->max_bus_width;
#ifdef CONFIG_MMC_BLOCK_BOUNCE
	mmc->max_segs = NR_SG;
	mmc->max_blk_count = 65535;
	mmc->max_req_size = PAGE_SIZE * 16;
#else
	mmc->max_segs = 128;
	mmc->max_blk_count = 4095;
	mmc->max_req_size = 4096 * 512 - 1;
#endif
	mmc->max_blk_size = 4095;
	mmc->max_seg_size = mmc->max_req_size;

	plat->init(&pdev->dev);
	plat->power_on(&pdev->dev);

	/*
	 * Initialize controller and register some functions
	 * From here, we can do everything!
	 */
	controller_register(&controller[host->pdev_id], host);

	functions = host->plat->driver_data;

//	printk("%s: functions->set_clock = %x  jz_mmc_set_clock = %x\n", __FUNCTION__, functions->set_clock, jz_mmc_set_clock);

	mmc_set_drvdata(pdev, mmc);
	mmc_add_host(mmc);

	ret = sysfs_create_group(&pdev->dev.kobj, &jz_mmc_attr_group);

	if(controller[host->pdev_id].init(&controller[host->pdev_id], host, pdev))
		goto out;

	printk("JZ %s driver registered\n", pdev->name);

	return 0;

out:
	return -1;
}

static int jz_mmc_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct jz_mmc_platform_data *plat = pdev->dev.platform_data;

	platform_set_drvdata(pdev, NULL);

	if (mmc) {
		struct jz_mmc_host *host = mmc_priv(mmc);
		struct jz_mmc_functions *functions = host->plat->driver_data;

		plat->power_off(&pdev->dev);

		functions->deinit(host, pdev);

		mmc_remove_host(mmc);
		mmc_free_host(mmc);
	}
	sysfs_remove_group(&pdev->dev.kobj, &jz_mmc_attr_group);
	return 0;
}

static int jz_mmc_suspend(struct platform_device *dev, pm_message_t state)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
	struct jz_mmc_host *host = mmc_priv(mmc);
	int ret = 0;

	if (mmc) {
		
		if (host->plat->detect_pin)
			enable_irq_wake(host->plat->detect_pin);

		if (mmc->card && mmc->card->type != MMC_TYPE_SDIO) {
			ret = mmc_suspend_host(mmc);
		}
	}
	return ret;
}

static int jz_mmc_resume(struct platform_device *dev)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
//	struct jz_mmc_host *host = mmc_priv(mmc);
	int ret = 0;

	if (mmc) {
		if (mmc->card && mmc->card->type != MMC_TYPE_SDIO) {
			ret = mmc_resume_host(mmc);
			//disable_irq_wake(host->plat->detect_pin);
		}
	}

	return ret;
}

static struct platform_driver jz_mmc_driver = {
	.probe = jz_mmc_probe,
	.remove = jz_mmc_remove,
	.suspend = jz_mmc_suspend,
	.resume = jz_mmc_resume,
	.driver = {
		   .name = "jz-msc",
		   },
};

static int __init jz_mmc_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&jz_mmc_driver);

	return ret;
}

static void __exit jz_mmc_exit(void)
{
	platform_driver_unregister(&jz_mmc_driver);
}

module_init(jz_mmc_init);
module_exit(jz_mmc_exit);

MODULE_DESCRIPTION("JZ47XX SD/Multimedia Card Interface Driver");
MODULE_LICENSE("GPL");
