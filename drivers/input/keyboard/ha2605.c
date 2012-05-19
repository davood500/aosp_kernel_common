
/*
 *  drivers/input/keyboard/ha2605_key.c
 *
 *  Cypress HA260520180 capsense touch key driver
 *
 *  Copyright (C) 2010 Ingenic Semiconductor, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/earlysuspend.h>
#include <linux/slab.h>
#include <asm/jzsoc.h>
#include "ha2605.h"
//#define DEBUG
#ifdef DEBUG
#define dprintk(x...)	printk(x)
#else
#define dprintk(x...)
#endif

static int diable_ha2605 = 0;
static struct i2c_client *this_client=NULL;
static struct ha2605_platform_data *pdata=NULL;
struct ha2605_data {
	struct	input_dev	*input_dev;
	struct	work_struct	key_event_work;
	struct workqueue_struct *key_queue;
	u8	current_key;
	struct early_suspend early_suspend;
	int is_suspend;
};

static int ha2605_i2c_rxdata(char *rxdata, int length)
{
	int ret;
	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};
	ret = i2c_transfer(this_client->adapter, msgs, 1);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);
	return ret;
}

static int ha2605_i2c_txdata(char *txdata, int length)
{
	int ret  = -1;
	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};
	dprintk("%s ................ %d %d \n",__FUNCTION__, this_client->addr);
	ret = i2c_transfer(this_client->adapter, msg, 1);    
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}

static int ha2605_write_config(void){

	u8 txdata[12] = "";
	u32 verify=0;
	int ret;

	memset(txdata,0,sizeof(txdata));

	txdata[0]=0x88;
	txdata[1]=0x05;

	txdata[2]=0x40;
	txdata[3]=txdata[4]=txdata[5]=txdata[6]=0x42;
	txdata[7]=0xf8;

	txdata[8]=0x00;
	txdata[9]=0x05;
	txdata[10]=0x00;

	verify = txdata[1]+txdata[2]+txdata[3]+txdata[4]+txdata[5]+txdata[6]+txdata[7]+txdata[8]+txdata[9]+txdata[10];

	txdata[11]=(u8)verify;
	dprintk("verify::%x\n",txdata[11]);

	ret = ha2605_i2c_txdata(txdata,12);
	return ret;
}
static int ha2605_read_data(struct ha2605_data *ha2605)
{
	char buf;
	int ret;
	/* read port status */
	struct touch_keys_button *buttons = pdata->buttons;
	int nbuttons = pdata->nbuttons;

	buf = 0;
	msleep(40);
	ret = ha2605_i2c_rxdata(&buf, 1);
	if (ret < 0){
		printk("read error:%s\n",__FUNCTION__);
		
		return ret;
	}

	dprintk(" %x %s \n",buf&0xff,__FUNCTION__);
	if (buf>0&&buf<=nbuttons&&buttons[buf-1].code) {
		ha2605 -> current_key = buf;
		dprintk("key down :: %d code :: %d \n",ha2605 -> current_key,buttons[ha2605 -> current_key-1].code);
		if(!diable_ha2605){
			input_report_key(ha2605->input_dev, buttons[ha2605 -> current_key-1].code, 1);
			input_sync(ha2605->input_dev);
		}
	}else{
		return 0;
	}

	do{
		msleep(40);
	}while(ha2605_i2c_rxdata(&buf, 1) >= 0 && ha2605 -> current_key == buf && !ha2605->is_suspend);

	dprintk("key up :: %d code :: %d \n ",ha2605 -> current_key, buttons[ha2605 -> current_key-1].code);
	if(!diable_ha2605){
		input_report_key(ha2605->input_dev, buttons[ha2605 -> current_key-1].code, 0);
		input_sync(ha2605->input_dev); 
	}   
	return 0;
}

static void ha2605_key_irq_work(struct work_struct *work)
{
	//process key touch
	struct ha2605_data *ha2605;
	ha2605 = container_of(work, struct ha2605_data, key_event_work);
	ha2605_read_data(ha2605);
	enable_irq(this_client->irq);
	return;
}


static irqreturn_t ha2605_interrupt(int irq, void *dev_id)
{
	struct ha2605_data *ha2605 = dev_id;
	int ret;
	dprintk("%s\n",__FUNCTION__);
	disable_irq_nosync(this_client->irq);
	if(ha2605->is_suspend == 1){
		return IRQ_HANDLED;
	}
	if (!work_pending(&ha2605->key_event_work)) {
		ret = queue_work(ha2605->key_queue,&ha2605->key_event_work);  
	}
	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ha2605_suspend(struct early_suspend *h);
static void ha2605_resume(struct early_suspend *h);
#endif

#if 1

static ssize_t ha2605_disable_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	printk( "%s(%d):\n", __func__,diable_ha2605);
	return sprintf(buf, "%d\n",diable_ha2605);
	
}
static ssize_t ha2605_disable_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int i = (int)(*buf) -48;
	printk("%s(%d):\n", __func__,i);
	diable_ha2605 = i;

	return count;
}

static DEVICE_ATTR(ha2605_disable, 0660, ha2605_disable_show, ha2605_disable_store);

#endif

static int ha2605_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ha2605_data *ha2605; 
	struct input_dev *input_dev;
	int err = 0;
	int i;
	struct touch_keys_button *buttons =NULL; //pdata->buttons;
	int nbuttons = 0;//pdata->nbuttons;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ha2605 = kzalloc(sizeof(*ha2605), GFP_KERNEL);
	if (!ha2605)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	this_client = client;
	i2c_set_clientdata(client, ha2605);
//	i2c_jz_setclk(client,100 * 1000);
	ha2605->is_suspend = 0;
	INIT_WORK(&ha2605->key_event_work, ha2605_key_irq_work);
	ha2605->key_queue = create_singlethread_workqueue(HA_KEY_NAME);
	if (!ha2605->key_queue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}


	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		dev_err(&client->dev, "%s: platform data is null\n", __func__);
		goto exit_platform_data_null;
	}
	buttons = pdata->buttons;
	nbuttons = pdata->nbuttons;

	ha2605_write_config();
	err = request_irq(client->irq, ha2605_interrupt, IRQF_DISABLED, "ha2605", ha2605);
	if (err < 0) {
		dev_err(&client->dev, "ha2605_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}
	//__gpio_as_irq_rise_edge(pdata->intr);
	__gpio_as_irq_high_level(pdata->intr);

	//input_dev
	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	ha2605->input_dev = input_dev;


	set_bit(EV_KEY, input_dev->evbit);
	for(i =0;i<nbuttons;i++){
		if(buttons[i].code){
			set_bit(buttons[i].code, input_dev->keybit);
		}            
	}
	input_dev->name		= "ha2605_key";	//dev_name(&client->dev);
	input_dev->id.bustype	= BUS_I2C;
	input_dev->id.vendor	= 0x1587;
	input_dev->id.product	= 0x1010;
	input_dev->id.version	= 0x1011;

	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
				"ha2605_probe: failed to register input device: %s\n",
				dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

 #ifdef CONFIG_HAS_EARLYSUSPEND
         ha2605->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
         ha2605->early_suspend.suspend = ha2605_suspend;
         ha2605->early_suspend.resume = ha2605_resume;
         register_early_suspend(&ha2605->early_suspend);
 #endif
 
	err = device_create_file(&input_dev->dev, &dev_attr_ha2605_disable);
	if (err) {
		dev_err(&client->dev, "Error to create ha2605_disable\n");
		goto err_creat_file;
	}
	//write_config
	/*      while(1){ 
		u8 key = 0xff; 
		ha2605_i2c_rxdata(&key,1); 
		dprintk("key::%d\n",key); 
		} */
	return 0;

err_creat_file:
exit_input_register_device_failed:
	input_unregister_device(ha2605->input_dev);
exit_input_dev_alloc_failed:
	input_free_device(input_dev);
exit_irq_request_failed:
	free_irq(client->irq, ha2605);
exit_platform_data_null:
	cancel_work_sync(&ha2605->key_event_work);
	i2c_set_clientdata(client, NULL);
	kfree(ha2605);
exit_create_singlethread:
	destroy_workqueue(ha2605->key_queue);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

static int __devexit ha2605_remove(struct i2c_client *client)
{
	struct ha2605_data *ha2605 = i2c_get_clientdata(client);
	cancel_work_sync(&(ha2605->key_event_work));
	unregister_early_suspend(&ha2605->early_suspend);
	input_unregister_device(ha2605->input_dev);
	if(NULL != ha2605->input_dev)
		input_free_device(ha2605->input_dev);
	free_irq(client->irq, ha2605);
	i2c_set_clientdata(client, NULL);
	if(NULL != ha2605)
		kfree(ha2605);
	destroy_workqueue(ha2605->key_queue);
	return 0;
}

static void ha2605_suspend(struct early_suspend *h)
{
	struct ha2605_data *ha2605;
	ha2605= container_of(h, struct ha2605_data, early_suspend);
	ha2605->is_suspend = 1;
	disable_irq_nosync(this_client->irq);
	flush_workqueue(ha2605->key_queue);
	if(cancel_work_sync(&ha2605->key_event_work)){
		enable_irq(this_client->irq);
	}
	__gpio_as_input(pdata->intr);
	__gpio_disable_pull(pdata->intr);
}

static void ha2605_resume(struct early_suspend *h)
{
	struct ha2605_data *ha2605;
       	ha2605	= container_of(h, struct ha2605_data, early_suspend);
	//__gpio_as_irq_rise_edge(pdata->intr);
	__gpio_as_irq_high_level(pdata->intr);
	ha2605->is_suspend = 0;
	enable_irq(this_client->irq);
}

static const struct i2c_device_id ha2605_id[] = {
	{ HA_KEY_NAME, 0 },
	{ }
};

static struct i2c_driver ha2605_driver = {
	.driver	= {
		.name	= HA_KEY_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= ha2605_probe,
	.remove		= __devexit_p(ha2605_remove),
	.id_table	= ha2605_id,
};

static int __init ha2605_init(void)
{
	return i2c_add_driver(&ha2605_driver);
}

static void __exit ha2605_exit(void)
{
	i2c_del_driver(&ha2605_driver);
}

module_init(ha2605_init);
module_exit(ha2605_exit);

MODULE_AUTHOR("Ross Bai <fdbai@ingenic.cn>");
MODULE_DESCRIPTION("Cypress HA2605 touch sensor module driver");
MODULE_LICENSE("GPL");
