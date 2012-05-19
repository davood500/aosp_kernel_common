
/*
 * I2C adapter for the INGENIC I2C bus access.
 *
 * Copyright (C) 2006 - 2009 Ingenic Semiconductor Inc.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/module.h>
#include <asm/addrspace.h>

#include <asm/jzsoc.h>
#include "i2c-jz4770.h"

/* I2C protocol */
#define I2C_READ	1
#define I2C_WRITE	0
#define I2C_CLIENT_NUM  20
#define TIMEOUT         0xffff

#define I2C_FIFO_LEN 16

struct i2c_speed {
	unsigned int speed;
	unsigned char slave_addr;
};

struct jz_i2c {
	int                     id;
	unsigned int            irq;
	struct i2c_adapter	adap;
	wait_queue_head_t	wait;
	int rxfifo_not_empty;
	int dma_channel;
	int dma_terminate;
	char dma_name[32];
};

static struct i2c_speed jz4760_i2c_speed[I2C_CLIENT_NUM];
static int client_cnt = 0;

void i2c_jz_setclk(struct i2c_client *client,unsigned long i2cclk)
{
	jz4760_i2c_speed[client_cnt].slave_addr = client->addr;
	if (i2cclk > 0 && i2cclk <= 400000) {
		jz4760_i2c_speed[client_cnt].speed      = i2cclk/1000;
	} else if (i2cclk <= 0) {
		jz4760_i2c_speed[client_cnt].speed      = 100;
	} else {
		jz4760_i2c_speed[client_cnt].speed      = 400;
	}

	printk("Device 0x%2x with i2c speed:%dK\n",jz4760_i2c_speed[client_cnt].slave_addr,
			jz4760_i2c_speed[client_cnt].speed );

	client_cnt++;
}
EXPORT_SYMBOL_GPL(i2c_jz_setclk);

/* 
 *jz_i2c_irq
 */
static irqreturn_t jz_i2c_irq(int irqno, void *dev_id)
{
	struct jz_i2c *i2c = dev_id;
	if(irqno == i2c->irq) {
		if(REG_I2C_INTST(i2c->id) & I2C_INTST_RXFL) {
			REG_I2C_INTM(i2c->id) &= ~(I2C_INTM_MRXFL);
			i2c->rxfifo_not_empty = 1;
			wake_up(&i2c->wait);
		}
	} else if(i2c->dma_channel > 0) {
		disable_dma(i2c->dma_channel);
		if (__dmac_channel_transmit_halt_detected(i2c->dma_channel))
			__dmac_channel_clear_transmit_halt(i2c->dma_channel);

		if (__dmac_channel_address_error_detected(i2c->dma_channel)) {
			REG_DMAC_DSAR(i2c->dma_channel) = 0; /* clear source address register */
			REG_DMAC_DTAR(i2c->dma_channel) = 0; /* clear target address register */
			__dmac_channel_clear_address_error(i2c->dma_channel);
		}

		if (__dmac_channel_transmit_end_detected(i2c->dma_channel)) {
			__dmac_channel_clear_transmit_end(i2c->dma_channel);
			i2c->dma_terminate = 1;
			wake_up(&i2c->wait);
		}
	}

	return IRQ_HANDLED;
}

static int i2c_disable(int id)
{
	int timeout = TIMEOUT;

	__i2c_disable(id);
	while(__i2c_is_enable(id) && (timeout > 0)) {
		udelay(10);
		timeout--;
	}
	if(timeout)
		return 0;
	else
		return 1;
}

static int i2c_set_clk(int i2c_clk, int id)
{
	int dev_clk_khz = cpm_get_clock(CGU_PCLK) / 1000;
	int cnt_high = 0;	/* HIGH period count of the SCL clock */
	int cnt_low = 0;	/* LOW period count of the SCL clock */
	int cnt_period = 0;	/* period count of the SCL clock */
	int setup_time = 0;
	int hold_time = 0;

	if (i2c_clk <= 0 || i2c_clk > 400)
		goto Set_clk_err;

	cnt_period = dev_clk_khz / i2c_clk;
	if (i2c_clk <= 100) {
		/* i2c standard mode, the min LOW and HIGH period are 4700 ns and 4000 ns */
		cnt_high = (cnt_period * 4000) / (4700 + 4000);
	} else {
		/* i2c fast mode, the min LOW and HIGH period are 1300 ns and 600 ns */
		cnt_high = (cnt_period * 600) / (1300 + 600);
	}

	cnt_low = cnt_period - cnt_high;

	if (i2c_clk <= 100) {
		REG_I2C_CTRL(id) = 0x43 | (1<<5);      /* standard speed mode*/
		REG_I2C_SHCNT(id) = I2CSHCNT_ADJUST(cnt_high);
		REG_I2C_SLCNT(id) = I2CSLCNT_ADJUST(cnt_low);
		setup_time = 400;
		hold_time = 400;
	} else {
		REG_I2C_CTRL(id) = 0x45 | (1<<5);       /* high speed mode*/
		REG_I2C_FHCNT(id) = I2CFHCNT_ADJUST(cnt_high);
		REG_I2C_FLCNT(id) = I2CFLCNT_ADJUST(cnt_low);
		setup_time = 400;
		hold_time = 0;
	}

	hold_time = (setup_time / (1000000 / dev_clk_khz) - 1);
	setup_time = (setup_time / (1000000 / dev_clk_khz) + 1);

	if (setup_time > 255)
		setup_time = 255;
	if (setup_time <= 0)
		setup_time = 1;
	if (hold_time > 255)
		hold_time = 255;

	__i2c_set_setup_time(id, setup_time);
	
	if (hold_time <= 0) {
		__i2c_hold_time_disable(id);
	} else {
		__i2c_hold_time_enable(id);
		__i2c_set_hold_time(id, hold_time);
	}

	return 0;

Set_clk_err:

	printk("i2c set sclk faild,i2c_clk=%d KHz,dev_clk=%dKHz.\n", i2c_clk, dev_clk_khz);
	return -1;
}

static void i2c_init_clk(int id,unsigned char device)
{
	int i;
	if(i2c_disable(id))
		printk("i2c not disable\n");

	for (i = 0; i < I2C_CLIENT_NUM; i++) {
		if(device == jz4760_i2c_speed[i].slave_addr) {
			i2c_set_clk(jz4760_i2c_speed[i].speed,id);
			break;
		}
	}
	if (i == I2C_CLIENT_NUM) {
		i2c_set_clk(100,id);
	}
	REG_I2C_INTM(id) = 0x0; /*mask all interrupt*/
	REG_I2C_TXTL(id) = 0x0;
	REG_I2C_ENB(id) = 1;   /*enable i2c*/
}

static unsigned int i2c_rdq[3] = {
	0x29,                 /* 6'b101001 */
	0x2b,                 /* 6'b101011 */
	0x3b                  /* 6'b111011 */
};

static inline int xfer_read(unsigned char addr,unsigned char *buf,int length, struct jz_i2c *i2c,int msgcnt,int msgidx)
{
	int i,timeout,ret;

	if(msgidx < msgcnt - 1)
		__i2c_nsend_stop(i2c->id);

	if(length <= I2C_FIFO_LEN) {
		REG_I2C_RXTL(i2c->id) = length - 1;
		i2c->rxfifo_not_empty = 0;
		REG_I2C_INTM(i2c->id) |= I2C_INTM_MRXFL;

		for(i=0;i<length;i++) {
			timeout = 0xfff;
			while(!__i2c_txfifo_not_full(i2c->id) && --timeout);
			if(timeout < 1) {
				ret = -20;
				goto xfer_read_timeout;
			}
			REG_I2C_DC(i2c->id) = (I2C_READ << 8);
		}

		if(msgidx == msgcnt - 1)
			__i2c_send_stop(i2c->id);

		timeout = wait_event_timeout(i2c->wait, i2c->rxfifo_not_empty == 1, HZ);//100ms wait timeout
		i2c->rxfifo_not_empty = 0;

		__i2c_clear_interrupts(i2c->id);
		while (length && ((REG_I2C_STA(i2c->id) & I2C_STA_RFNE)))
		{
			length--;
			*buf++ = REG_I2C_DC(i2c->id) & 0xff;
		}
	} else if((i2c->dma_channel > 0) && (length > I2C_FIFO_LEN)) {
		dma_cache_inv((unsigned long)buf, length * sizeof(unsigned char));
		REG_DMAC_DCCSR(i2c->dma_channel) = 0;
		REG_DMAC_DRSR(i2c->dma_channel) = i2c_rdq[i2c->id];
		REG_DMAC_DSAR(i2c->dma_channel) = CPHYSADDR(I2C_DC(i2c->id));
#if 0
		REG_DMAC_DTAR(i2c->dma_channel) = CPHYSADDR(buf);
#else
		if((CPHYSADDR(buf) & 0xf0000000) == 0)
			REG_DMAC_DTAR(i2c->dma_channel) = CPHYSADDR(buf) | 0x20000000;
		else
			REG_DMAC_DTAR(i2c->dma_channel) = CPHYSADDR(buf);
#endif
		REG_DMAC_DTCR(i2c->dma_channel) = length;
		REG_DMAC_DCMD(i2c->dma_channel) = 
			DMAC_DCMD_DAI | DMAC_DCMD_SWDH_8 | DMAC_DCMD_DWDH_8 | DMAC_DCMD_DS_8BIT | DMAC_DCMD_TIE; 
		i2c->dma_terminate = 0;
		REG_DMAC_DCCSR(i2c->dma_channel) = DMAC_DCCSR_NDES | DMAC_DCCSR_EN;
		REG_DMAC_DMACR(i2c->dma_channel/HALF_DMA_NUM) = DMAC_DMACR_DMAE; /* global DMA enable bit */

		__i2c_set_dma_rd_level(i2c->id, 0);
		__i2c_dma_rd_enable(i2c->id);

		for(i=0;i<length;i++) {
			while(!__i2c_txfifo_not_full(i2c->id));
			REG_I2C_DC(i2c->id) = (I2C_READ << 8);
		}

		if(msgidx == msgcnt - 1)
			__i2c_send_stop(i2c->id);

		timeout = wait_event_timeout(i2c->wait, i2c->dma_terminate == 1, HZ);
		i2c->dma_terminate = 0;
	} else {
		printk("addr[0x%02x]xfer_read error : not support read length more than %d without dma!\n",addr,I2C_FIFO_LEN);
	}

	if (__i2c_abrt(i2c->id)) {
		printk("addr[0x%02x]xfer_read error : ABORT or NACK -3\n",addr);
		printk("msgidx %d | msgcnt %d\n",msgidx,msgcnt);
		return -3;
	}

	if (!timeout){
		printk("addr[0x%02x]xfer_read error : WAIT FIFO TIMEOUT -4\n",addr);
		printk("msgidx %d | msgcnt %d\n",msgidx,msgcnt);
		ret = -4;
		goto xfer_read_timeout;
	}

	timeout = 0xfff;
	while (__i2c_master_active(i2c->id) && --timeout);
	if(timeout < 1) {
		ret = -21;
		goto xfer_read_timeout;
	}

	return 0;
xfer_read_timeout:
	REG_I2C_ENB(i2c->id) = 0;   /*disable i2c*/
	__gpio_as_i2c_outlf(i2c->id,10000);
	REG_I2C_ENB(i2c->id) = 1;   /*enable i2c*/
	printk("xfer_read timeout ret=%d",ret);
	return ret;
}

static inline int xfer_write(unsigned char addr,unsigned char *buf,int length, struct jz_i2c *i2c,int msgcnt,int msgidx)
{
	int i,timeout;

	__i2c_nsend_stop(i2c->id);

	for(i=0;i<length;i++) {
		timeout = 0xfff;
		while(!__i2c_txfifo_not_full(i2c->id) && --timeout);
		REG_I2C_DC(i2c->id) = (I2C_WRITE << 8) | *buf++;
		if(timeout < 1) goto xfer_write_timeout;
	}

	if(msgidx == msgcnt - 1)
		__i2c_send_stop(i2c->id);

	if (__i2c_abrt(i2c->id)) {
		printk("addr[0x%02x]xfer_write error : ABORT or NACK -1\n",addr);
		printk("msgidx %d | msgcnt %d\n",msgidx,msgcnt);
		return -1;
	}

	return 0;
xfer_write_timeout:
	REG_I2C_ENB(i2c->id) = 0;   /*disable i2c*/
	__gpio_as_i2c_outlf(i2c->id,10000);
	REG_I2C_ENB(i2c->id) = 1;   /*enable i2c*/
	printk("addr[0x%02x]xfer_read error : __i2c_txfifo_not_full -2\n",addr);
	printk("msgidx %d | msgcnt %d\n",msgidx,msgcnt);
	return -2;
}

static int i2c_jz_xfer(struct i2c_adapter *adap, struct i2c_msg *pmsg, int num)
{
	int ret=0, i=0,timeout=0xffff,retry;
	struct jz_i2c *i2c = adap->algo_data;

	for(retry=0;retry<3;retry++) {
		while((!__i2c_txfifo_is_empty(i2c->id) || __i2c_master_active(i2c->id)) && --timeout)
			;
		if(timeout > 1) 
			break;
		else
			__gpio_as_i2c(i2c->id);//output low 10us
	}

	if(timeout<1) {
		printk("i2c_jz_xfer wait state time out.\n");
		printk("i2c_jz_xfer wait state time out.\n");
		printk("i2c_jz_xfer wait state time out.\n");
		return -19;
	}

	if (pmsg->addr != REG_I2C_TAR(i2c->id)) {
		REG_I2C_TAR(i2c->id) = pmsg->addr;
		i2c_init_clk(i2c->id,pmsg->addr);
	}
	
	for (i = 0; i < num; i++,pmsg++) {
		if (pmsg->flags & I2C_M_RD){
			ret = xfer_read(pmsg->addr,pmsg->buf, pmsg->len,i2c,num,i);
		} else {
			ret = xfer_write(pmsg->addr,pmsg->buf, pmsg->len,i2c,num,i);
		}

		if (ret) {
			return ret;
		}
	}

	return i;
}

static u32 i2c_jz_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm i2c_jz_algorithm = {
	.master_xfer	= i2c_jz_xfer,
	.functionality	= i2c_jz_functionality,
};

static int i2c_jz_probe(struct platform_device *pdev)
{
	struct jz_i2c *i2c;
	struct i2c_jz_platform_data *plat = pdev->dev.platform_data;
	int ret;

	i2c = kzalloc(sizeof(struct jz_i2c), GFP_KERNEL);
	if (!i2c) {
		printk("There is no enough memory\n");
		ret = -ENOMEM;
		goto emalloc;
	}

	i2c->id            = pdev->id;
	i2c->adap.owner   = THIS_MODULE;
	i2c->adap.algo    = &i2c_jz_algorithm;
	i2c->adap.retries = 6;
	sprintf(i2c->adap.name, "jz_i2c-i2c.%u", pdev->id);
	i2c->adap.algo_data = i2c;
	i2c->adap.dev.parent = &pdev->dev;

	if (plat) {
		i2c->adap.class = plat->class;
	}

	if (i2c->id == 0) {
		cpm_start_clock(CGM_I2C0);
	} else if (i2c->id == 1) {
		cpm_start_clock(CGM_I2C1);
	} else {
		cpm_start_clock(CGM_I2C2);
	}
	__gpio_as_i2c(i2c->id);
	i2c_init_clk(i2c->id,0xff);

	i2c->irq = platform_get_irq(pdev, 0);
	ret = request_irq(i2c->irq, jz_i2c_irq, IRQF_DISABLED,
			dev_name(&pdev->dev), i2c);

	memset(i2c->dma_name,0,32);
	sprintf(i2c->dma_name,"dma-i2c.%d",i2c->id);
	i2c->dma_channel = jz_request_dma(DMA_ID_AUTO,i2c->dma_name,jz_i2c_irq,IRQF_DISABLED,i2c);
	if(i2c->dma_channel > 0)
		__dmac_channel_enable_clk(i2c->dma_channel);

	if (ret != 0) {
		dev_err(&pdev->dev, "cannot claim IRQ %d\n", i2c->irq);
		goto emalloc;
	}
	/*
	 * If "dev->id" is negative we consider it as zero.
	 * The reason to do so is to avoid sysfs names that only make
	 * sense when there are multiple adapters.
	 */

	i2c->adap.nr = pdev->id != -1 ? pdev->id : 0;
	ret = i2c_add_numbered_adapter(&i2c->adap);
	if (ret < 0) {
		printk(KERN_INFO "I2C: Failed to add bus\n");
		goto eadapt;
	}

	platform_set_drvdata(pdev, i2c);
	dev_info(&pdev->dev, "JZ4770 i2c bus driver.\n");

	init_waitqueue_head(&i2c->wait);
	return 0;

eadapt:
emalloc:

	kfree(i2c);
	return ret;
}

static int i2c_jz_remove(struct platform_device *pdev)
{
	struct i2c_adapter *adapter = platform_get_drvdata(pdev);
	int rc;

	rc = i2c_del_adapter(adapter);
	platform_set_drvdata(pdev, NULL);
	return rc;
}


static int i2c_jz_resume(struct platform_device *pdev)
{
	__gpio_as_i2c_outlf(pdev->id,20000);
	return 0;
}

#ifdef  CONFIG_I2C0_JZ4770
static struct platform_driver i2c_0_jz_driver = {
	.probe		= i2c_jz_probe,
	.remove		= i2c_jz_remove,
	.resume		= i2c_jz_resume,
	.driver		= {
		.name	= "jz_i2c0",
	},
};
#endif

#ifdef  CONFIG_I2C1_JZ4770
static struct platform_driver i2c_1_jz_driver = {
	.probe		= i2c_jz_probe,
	.remove		= i2c_jz_remove,
	.resume		= i2c_jz_resume,
	.driver		= {
		.name	= "jz_i2c1",
	},
};
#endif

#ifdef  CONFIG_I2C2_JZ4770
static struct platform_driver i2c_2_jz_driver = {
	.probe		= i2c_jz_probe,
	.remove		= i2c_jz_remove,
	.resume		= i2c_jz_resume,
	.driver		= {
		.name	= "jz_i2c2",
	},
};
#endif

static int __init i2c_adap_jz_init(void)
{
	int ret = 0;

#ifdef  CONFIG_I2C0_JZ4770
	ret = platform_driver_register(&i2c_0_jz_driver);
#endif

#ifdef  CONFIG_I2C1_JZ4770
	ret = platform_driver_register(&i2c_1_jz_driver);
#endif

#ifdef  CONFIG_I2C2_JZ4770
	ret = platform_driver_register(&i2c_2_jz_driver);
#endif
	return ret;
}

static void __exit i2c_adap_jz_exit(void)
{
#ifdef  CONFIG_I2C0_JZ4770
	platform_driver_unregister(&i2c_0_jz_driver);
#endif

#ifdef  CONFIG_I2C1_JZ4770
	platform_driver_unregister(&i2c_1_jz_driver);
#endif

#ifdef  CONFIG_I2C2_JZ4770
	platform_driver_unregister(&i2c_2_jz_driver);
#endif
}

MODULE_LICENSE("GPL");
subsys_initcall(i2c_adap_jz_init);
module_exit(i2c_adap_jz_exit);
