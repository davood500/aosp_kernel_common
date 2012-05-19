/*
 *  linux/drivers/mmc/host/jz_mmc/gpio/jz_mmc_gpio.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */

#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <asm/jzmmc/jz_mmc_gpio.h>

#define	TRY_TIME	100
#define	RETRY_TIME	10
#define CARD_REINSERT_DURATTION	(7 * HZ)

int present = 0;
DEFINE_SEMAPHORE(detect_start_mutex);
DEFINE_SEMAPHORE(detect_done_mutex);

EXPORT_SYMBOL(detect_start_mutex);
EXPORT_SYMBOL(detect_done_mutex);
EXPORT_SYMBOL(present);

//static unsigned long msmsdcc_irqtime = 0;

static int manual_detect(void *data)
{
	struct jz_mmc_host *host = (struct jz_mmc_host *)data;
	
	while(1) {
		
		if(down_interruptible(&detect_start_mutex))
			printk("%s: down error!\n", __FUNCTION__);
		
		if(present)
			host->eject = 0;
		else
			host->eject = 1;
	      	mmc_detect_change(host->mmc, 0);
		up(&detect_done_mutex);
	}
}

static void jiq_de_quiver(struct work_struct *ptr)
{
	struct jz_mmc_host *host = container_of(ptr, struct jz_mmc_host, gpio_jiq_work);
	unsigned int  i, tmp, counter = 0;
	unsigned long sync_tmp;
	
	unsigned int pretmp,stableflag,eject;

	if (!host->plat->status) {
		mmc_detect_change(host->mmc, 0);
		return;
	}

//detect gpio is stable
	tmp = host->plat->status(mmc_dev(host->mmc));
	if(tmp != 0)
	{
		pretmp = tmp;

		stableflag = 0;
		for(i = 0; i < RETRY_TIME * TRY_TIME; i++)
		{
			msleep(20);
			tmp = host->plat->status(mmc_dev(host->mmc));  //1-insert

			if(tmp == pretmp) {
				counter++;
				if(counter > TRY_TIME) {
					stableflag = 1; 
					eject = tmp ? 0 : 1;
					printk("%s %s card\n",__FUNCTION__,tmp ? "Insert":"Eject");
					break;
				}

			} else {
				pretmp = tmp;
				counter = 0;
			}
		}
	}else{
		eject = 1;
		stableflag = 1; //remove card 
	}
		 
//detect finish
	if(stableflag) {
		if (host->eject ^ eject) {
			/* Delay the process for Card re-insert */
#if 0
			duration = jiffies - msmsdcc_irqtime;
			
			if ((!eject) && (duration < CARD_REINSERT_DURATTION)) {
				duration = CARD_REINSERT_DURATTION - duration;
				//printk("+++++++++++++++++++++++duration = %d\n", duration);
			} else {
				duration = 0;
				//printk("-----------------------duration = %d\n", duration);
			}
#endif		

			host->eject = eject;

//			mmc_detect_change(host->mmc, duration);
			mmc_detect_change(host->mmc, 10);
//			msmsdcc_irqtime = jiffies;

			if(eject) {

				host->msc_ack = 0;
				host->dma_ack = 0;

				wake_up_interruptible(&host->msc_wait_queue);
				wake_up_interruptible(&host->dma_wait_queue);
#ifdef CONFIG_MSC2_PARALLEL_DMA
				wake_up_interruptible(&host->parallel_queue);
#endif
				jz_mmc_stop_dma(host);

				sync_tmp = host->for_sync;
				msleep(1000); //for queue handle time
				while(host->for_sync != sync_tmp) {
					sync_tmp = host->for_sync;
					msleep(1000); //for queue handle time
				}
				
				printk("=============================> QUEUE IS OVER !\n");

				host->for_sync = 0;
				
				REG_MSC_STRPCL(host->pdev_id) = MSC_STRPCL_RESET;
				while (REG_MSC_STAT(host->pdev_id) & MSC_STAT_IS_RESETTING);
				
				if(host->plat->support_sdio == 0)
					REG_MSC_LPM(host->pdev_id) = 0x1;	// Low power mode
				else
				REG_MSC_STRPCL(host->pdev_id) |= MSC_STRPCL_CLOCK_CONTROL_START;
				
			}
			
		}

		//handle next irq 
		if(host->eject) {
			host->plat->plug_change(CARD_REMOVED);
		} else {
			host->plat->plug_change(CARD_INSERTED);
		}

	}

	enable_irq(host->plat->status_irq);
}

static irqreturn_t jz_mmc_detect_irq(int irq, void *devid)
{
	struct jz_mmc_host *host = (struct jz_mmc_host *) devid;
	

	disable_irq_nosync(host->plat->status_irq);
	queue_work(host->gpio_jiq_queue,&(((struct jz_mmc_host *) devid)->gpio_jiq_work));
	//flush_workqueue(host->mmc->detect_workqueue);

	//queue_work(host->mmc->detect_workqueue,&(((struct jz_mmc_host *) devid)->gpio_jiq_work));

	return IRQ_HANDLED;
}

static int jz_mmc_gpio_init(struct jz_mmc_host *host, struct platform_device *pdev)
{
	int ret = 0;
	struct task_struct *task;

	/*
	 * Setup card detect change
	 */
	if (host->plat->status_irq) {

		device_init_wakeup(&pdev->dev, 1);

		INIT_WORK(&(host->gpio_jiq_work), jiq_de_quiver);
		host->gpio_jiq_queue = create_workqueue("sd_detect");
		if(!host->gpio_jiq_queue) {
			printk("%s create_workqueue error!",__FILE__);
			ret = -ENODEV;
			goto createworkqueue_error;
		}
		
		// Check if there were any card present
		if (host->plat->status) {

			host->eject = !(host->plat->status(mmc_dev(host->mmc)));
			if(host->eject) {
				host->plat->plug_change(CARD_REMOVED);
			} else {
				host->plat->plug_change(CARD_INSERTED);
			}
		}
		ret = request_irq(host->plat->status_irq,
						  jz_mmc_detect_irq,
						  0,
						  "jz-msc (gpio)",
						  host);
		if (ret) {
			printk(KERN_ERR "Unable to get slot IRQ %d (%d)\n",
			       host->plat->status_irq, ret);
			goto request_irq_error;
		}
		//enable_irq(host->plat->status_irq);
	} else
		printk(KERN_ERR "%s: No card detect facilities available\n",
		       mmc_hostname(host->mmc));

	if(host->plat->need_mdetect == 1) {
		
		task = kthread_create(manual_detect, (void *)host, "manual_detect");
		
		wake_up_process(task);
	}
	return 0;
request_irq_error:
	destroy_workqueue(host->gpio_jiq_queue);
createworkqueue_error:
	return ret;

}

static void jz_mmc_gpio_deinit(struct jz_mmc_host *host, struct platform_device *pdev)
{
	if(host->plat->status_irq) {
		if(host->gpio_jiq_queue)
			destroy_workqueue(host->gpio_jiq_queue);
		free_irq(host->plat->status_irq, &host);
		device_init_wakeup(&pdev->dev, 0);
	}
}

int jz_mmc_gpio_register(struct jz_mmc_gpio *gpio)
{
	if(gpio == NULL)
		return -ENOMEM;

	gpio->init = jz_mmc_gpio_init;
	gpio->deinit = jz_mmc_gpio_deinit;

	return 0;
}
