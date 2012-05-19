/*---------------------------------------------------------------------------------------------------------
 * driver/input/touchscreen/gt801_ts.c
 *---------------------------------------------------------------------------------------------------------*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <asm/jzsoc.h>
#include <asm/gpio.h>
#include <linux/slab.h>

#include "jz_tsc_gt801.h"
/* three configs about init , but use best now*/

static unsigned char config_info[54] = {
	0x30,
	0x0F,0x05,0x05,0x28,0x02,0x14,0x14,0x10,0x28,0xF2,
	0x14,0x00,0x1E,0x00,0x01,0x23,0x45,0x67,0x89,0xAB,
	0xCD,0xE0,0x00,0x00,0x37,0x2E,0x4D,0xC1,0x20,0x00,
	0x01,0xA0,0x3C,0x3C,0x1E,0xB4,0x10,0x35,0x2C,0x01,
	0xEC,0x28,0x32,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x01};

/*
static unsigned char config_info[54] = {
	0x30,
	0x19,0x05,0x03,0x28,0x02,0x14,0x14,0x10,0x32,0xB2,
	0x14,0x00,0x1E,0x00,0x01,0x23,0x45,0x67,0x89,0xAB,
	0xCD,0xE0,0x00,0x00,0x37,0x2E,0x4D,0xCF,0x20,0x1F,
	0x01,0x63,0x3C,0x3C,0x1E,0xB4,0x10,0x35,0x2C,0x01,
	0xEC,0x28,0x46,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x01};
static unsigned char config_info[54] = {
	0x30,
	0x0F,0x03,0x07,0x28,0x02,0x14,0x14,0x10,0x2D,0xF2,
	0x14,0x00,0x1E,0x00,0x01,0x23,0x45,0x67,0x89,0xAB,
	0xCD,0xE0,0x00,0x00,0x37,0x2E,0x4D,0xC1,0x20,0x00,
	0x00,0x83,0x3C,0x3C,0x1E,0xB4,0x10,0x35,0x2C,0x01,
	0xEC,0x28,0x3C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x01};
*/

//----------------------
static struct workqueue_struct *gt801_wq;
static struct point_queue  finger_list;	//record the fingers list 
	
#ifdef CONFIG_HAS_EARLYSUSPEND
static void gt801_ts_early_suspend(struct early_suspend *h);
static void gt801_ts_late_resume(struct early_suspend *h);
#endif

int resume_filter_touch;

static int i2c_read_bytes(struct i2c_client *client, uint8_t *buf, int len)
{
	int err = 0;
	struct i2c_msg msgs[]={
		{
			.addr=client->addr,
			.flags=0,
			.len=1,
			.buf=buf,
		},
		{
			.addr=client->addr,
			.flags= I2C_M_RD ,
			.len=len-1,
			.buf=buf+1,
		},
	};
	err =  i2c_transfer(client->adapter,msgs,2);
	if(err < 0)
		printk("gt801 touch screen i2c read is error\n");
	return err;
}
static int i2c_write_bytes(struct i2c_client *client,uint8_t *data,int len)
{
	int err;
	struct i2c_msg msg[] = {
		{
			.addr=client->addr,
			.flags= 0,
			.len=len,
			.buf=data,		
		}
	};
	
	err =  i2c_transfer(client->adapter,msg,1);
	if(err < 0)
		printk("gt801 touch screen i2c write is error\n");
	return err;
}


//----------------
static int gt801_init(struct gt801_ts_data *ts)
{
	int ret=-1;
	__gpio_as_output(GPIO_GT801_SHUTDOWN);
	__gpio_set_pin(GPIO_GT801_SHUTDOWN);
	msleep(100);
	__gpio_clear_pin(GPIO_GT801_SHUTDOWN);
	msleep(100);
	ret=i2c_write_bytes(ts->client,config_info,54);
	if(ret<0) {
		printk("gt801 init i2c write error\n");
		return ret;
	}
	return 0;
}

static void gt801_init_panel(struct work_struct *work_init)
{
	int ret=-1;
	int i=0;
	struct gt801_ts_data *ts;

	ts = container_of(work_init, struct gt801_ts_data, work_init);
	for(i=0;i<3;i++){
		ret = gt801_init(ts);
		if (ret < 0){
			printk("gt801 init is error in resume \n");
			__gpio_as_output(GPIO_TP_DRV_EN);                  
			__gpio_clear_pin(GPIO_TP_DRV_EN);
			msleep(100);
			__gpio_set_pin(GPIO_TP_DRV_EN);
			msleep(100);
		}else{
			break;
		}
	}
	msleep(133);
	ts->is_suspend = 0;
	enable_irq(ts->client->irq);
}
//------------------
static void gt801_ts_work_func(struct work_struct *work)
{
	static struct point_node pointer[MAX_FINGER_NUM];
	struct gt801_ts_data *ts;
	static uint8_t finger_last = 0;				//last time fingers' state
	struct point_node * p = NULL;
	uint8_t read_position = 0;
	uint8_t point_data[READ_BYTES_NUM]={ 0 };
	uint8_t finger, finger_current;				//record which finger is changed
	unsigned int x=0, y=0;
	int count = 0;
	int ret = -1;
        int finger_count = 0;	
	ts = container_of(work, struct gt801_ts_data, work);
	if ( gpio_get_value(GPIO_GT801_SHUTDOWN)){
		goto NO_ACTION;					//The data is invalid.
	}

	ret=i2c_read_bytes(ts->client, point_data, sizeof(point_data));
	if(ret < 0)	
	{
		printk("I2C transfer error. ERROR Number:%d\n ", ret);
		ts->retry++;
		if(ts->retry >= 100){				//It's not normal for too much i2c-error.
			printk("Reset the chip for i2c error.\n ");
			ts->retry = 0;
			if(ts->power){
				ts->power(ts, 0);
				ts->power(ts, 1);
			}else{
				gt801_init(ts);
				msleep(100);
			}
		}
		goto XFER_ERROR;
	}	
	//The bits indicate which fingers pressed down
	finger_current = point_data[1] & 0x1f;
	finger = finger_current^finger_last; 	
	
	if(resume_filter_touch){
		resume_filter_touch--;
		goto NO_ACTION;
	}

	if(finger == 0 && finger_current == 0){
		goto NO_ACTION;					//no action
	}else if(finger == 0){						
		goto BIT_NO_CHANGE;				//the same as last time
	}
		
	//check which point(s) DOWN or UP
	for(count = 0; count < MAX_FINGER_NUM;  count++)
	{
		p = &pointer[count];
		p->id = count;
		if((finger_current & FLAG_MASK) != 0){	
			p->state = FLAG_DOWN;
		}else{
			if((finger & FLAG_MASK) != 0){		//send press release.
				p->state = FLAG_UP;
			}else{
				p->state = FLAG_INVALID;
			}
		}
		
		finger>>=1;
		finger_current>>=1;	
	}
	finger_last = point_data[1] & 0x1f;			//restore last presse state.

BIT_NO_CHANGE:
	for(count = 0; count < MAX_FINGER_NUM; count++)
	{	
		p = &pointer[count];
		if(p->state == FLAG_INVALID)
			continue;
		
		if(p->state == FLAG_UP){
			x = y = 0;
			p->pressure = 0;
			continue;
		}else{
			finger_count++;
		}
		
		if(p->id < 3){
			read_position = p->id *5 + 3;
		}else{
			read_position = 29;
		}
		if(p->id != 3){
			x = (unsigned int) (point_data[read_position]<<8) + (unsigned int)( point_data[read_position+1]);
			y = (unsigned int)(point_data[read_position+2]<<8) + (unsigned int) (point_data[read_position+3]);
			p->pressure = point_data[read_position+4];
		}
	#if MAX_FINGER_NUM > 3
		else{
			x = (unsigned int) (point_data[18]<<8) + (unsigned int)( point_data[25]);
			y = (unsigned int)(point_data[26]<<8) + (unsigned int) (point_data[27]);
			p->pressure = point_data[28];
		}
#endif
        x =  x*SCREEN_MAX_WIDTH/TOUCH_MAX_WIDTH;		
        y =  y*SCREEN_MAX_HEIGHT/TOUCH_MAX_HEIGHT ;		
        swap_xy(x, y); 

#ifdef CONFIG_TOUCHSCREEN_X_FLIP                                                                                                                         
        p->x = SCREEN_MAX_HEIGHT - x;
#else
        p->x = x;
#endif
#ifdef CONFIG_TOUCHSCREEN_Y_FLIP
        p->y = SCREEN_MAX_WIDTH - y;
#else
        p->y = y;
#endif
    }

#ifdef CONFIG_TOUCHSCREEN_JZ_GT801_MULTI_TOUCH
	/* ABS_MT_TOUCH_MAJOR is used as ABS_MT_PRESSURE in android. */
	if(finger_count){
		for(count = 0; count < MAX_FINGER_NUM; count++){
			p = &pointer[count];

			if(p->state == FLAG_INVALID){
				continue;
			}

			if(p->state == FLAG_DOWN)
			{
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, p->x);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, p->y);
				dev_dbg(&(ts->client->dev), "Id:%d, x:%d, y:%d\n", p->id, p->x, p->y);
				input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, p->id);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, p->pressure);
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, p->pressure);

				input_mt_sync(ts->input_dev);	
			} 
		}
	}else{
		input_mt_sync(ts->input_dev);	
	}
#else
	if(pointer[0].state == FLAG_DOWN){
		input_report_abs(ts->input_dev, ABS_X, pointer[0].x);
		input_report_abs(ts->input_dev, ABS_Y, pointer[0].y);	
	} 
	input_report_abs(ts->input_dev, ABS_PRESSURE, pointer[0].pressure);
	input_report_key(ts->input_dev, BTN_TOUCH, pointer[0].state == FLAG_INVALID?FLAG_UP:pointer[0].state);  
#endif
	input_sync(ts->input_dev);

XFER_ERROR:	
NO_ACTION:
	enable_irq(ts->client->irq);

}

static irqreturn_t gt801_ts_irq_handler(int irq, void *dev_id)
{
	struct gt801_ts_data *ts = dev_id;
	disable_irq_nosync(ts->client->irq);
	if(ts->is_suspend == 1)
		return IRQ_HANDLED;
	if(!work_pending(&ts->work))
		queue_work(gt801_wq, &ts->work);
	return IRQ_HANDLED;
}

static int gt801_ts_power(struct gt801_ts_data * ts, int on)
{ 
	int ret = 0;
	if(ts == NULL)
		return -1;
	switch(on){
		case 0:
			__gpio_set_pin(GPIO_GT801_SHUTDOWN);
			break;
		case 1:
			queue_work(gt801_wq, &ts->work_init);
			break;	
	}
	return ret;
}

static int gt801_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct gt801_ts_data *ts;
	int ret = 0;
	struct gt801_ts_platform_data *pdata;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)){
		dev_err(&client->dev, "System need I2C function.\n");
		return  -ENODEV;
	}
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		return  -ENOMEM;
	}

	__gpio_as_output(GPIO_TP_DRV_EN);                  
	__gpio_set_pin(GPIO_TP_DRV_EN);
	msleep(100);

	pdata = client->dev.platform_data;
	ts->client = client;
	i2c_set_clientdata(client,ts);
//	i2c_jz_setclk(ts->client,100*1000);
	
	gt801_init(ts);

	INIT_WORK(&ts->work, gt801_ts_work_func);
	INIT_WORK(&ts->work_init, gt801_init_panel);
	gt801_wq = create_singlethread_workqueue("gt801_ts");

	if (!gt801_wq) {
		printk(" gt801 touch screen Creat workqueue faiked\n");
		return -ENOMEM;
	}

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		printk("gt801 touch screen Input allocate is err\n");
		return  -ENOMEM;
	}

	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(EV_SYN, ts->input_dev->evbit);

	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
#ifdef CONFIG_TOUCHSCREEN_JZ_GT801_MULTI_TOUCH
	set_bit(ABS_MT_TRACKING_ID, ts->input_dev->keybit);
	set_bit(ABS_MT_TOUCH_MAJOR, ts->input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, ts->input_dev->absbit);

	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, SCREEN_MAX_HEIGHT, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, SCREEN_MAX_WIDTH, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, MAX_FINGER_NUM, 0, 0);	
#else
	set_bit(ABS_X, ts->input_dev->absbit);
	set_bit(ABS_Y, ts->input_dev->absbit);
	set_bit(ABS_PRESSURE, ts->input_dev->absbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);

	input_set_abs_params(ts->input_dev, ABS_X, 0, SCREEN_MAX_HEIGHT, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, SCREEN_MAX_WIDTH, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
#endif	
	ts->input_dev->name = GT801_NAME;
	ret = input_register_device(ts->input_dev);
	if (ret) {
		dev_err(&client->dev,"Probe: Unable to register %s input device\n", ts->input_dev->name);
		return ret;
	}
	finger_list.length = 0;
	__gpio_as_irq_fall_edge(pdata->intr);
	ret  = request_irq(client->irq, gt801_ts_irq_handler ,IRQF_DISABLED,
			client->name, ts);
	if (ret != 0) {
		dev_err(&client->dev,"Can't allocate touchscreen's interrupt!ERRNO:%d\n", ret);
		printk("gt801 request irq is error\n");
		return ret;
	}
	ts->power = gt801_ts_power;

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = gt801_ts_early_suspend;
	ts->early_suspend.resume = gt801_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
	dev_info(&client->dev,"Start  %s in %s mode\n", 
		ts->input_dev->name, "Interrupt");
	printk("gt801 ts probe suscess!\n");
	return 0;
}

static int gt801_ts_remove(struct i2c_client *client)
{
	struct gt801_ts_data *ts = i2c_get_clientdata(client);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	free_irq(client->irq, ts);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	i2c_set_clientdata(client, NULL);
	__gpio_as_input(GPIO_GT801_SHUTDOWN);
	gpio_free(GPIO_GT801_SHUTDOWN);
	dev_notice(&client->dev,"The driver is removing...\n");
	if(ts->input_dev)
		kfree(ts->input_dev);
	return 0;
}

static int gt801_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	__gpio_clear_pin(GPIO_TP_DRV_EN);
	return 0;
}
static int gt801_ts_resume(struct i2c_client *client)
{
	__gpio_set_pin(GPIO_TP_DRV_EN);
	msleep(10);
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void gt801_ts_early_suspend(struct early_suspend *h)
{
	int ret;
	struct gt801_ts_data *ts;
	ts = container_of(h, struct gt801_ts_data, early_suspend);
	ts->is_suspend = 1;
	disable_irq(ts->client->irq);
	flush_workqueue(gt801_wq);
	ret = cancel_work_sync(&ts->work);	
	if(ret){
		enable_irq(ts->client->irq);
	}
	if (ts->power) {
		ret = ts->power(ts,0);
		if (ret < 0)
			printk(KERN_ERR "%s power off failed\n", GT801_NAME);
	}
}

static void gt801_ts_late_resume(struct early_suspend *h)
{
	int ret;
	struct gt801_ts_data *ts;

	ts = container_of(h, struct gt801_ts_data, early_suspend);
	if (ts->power) {
		ret = ts->power(ts, 1);
		if (ret < 0)
			printk(KERN_ERR "%s power on failed\n", GT801_NAME);
	}
	resume_filter_touch = 1;
}
#endif

static const struct i2c_device_id gt801_ts_id[] = {
	{ GT801_NAME, 0 },
	{ }
};

static struct i2c_driver gt801_ts_driver = {
	.probe		= gt801_ts_probe,
	.remove		= gt801_ts_remove,
	.suspend	= gt801_ts_suspend,
	.resume		= gt801_ts_resume,
	.id_table	= gt801_ts_id,
	.driver = {
		.name	= GT801_NAME,
		.owner = THIS_MODULE,
	},
};
static int __devinit gt801_ts_init(void)
{
	printk("Gt801 touch screen init! \n");
	return i2c_add_driver(&gt801_ts_driver);
}

static void __exit gt801_ts_exit(void)
{
	i2c_del_driver(&gt801_ts_driver);
	if (gt801_wq)
		destroy_workqueue(gt801_wq);
}

late_initcall(gt801_ts_init);
module_exit(gt801_ts_exit);

MODULE_AUTHOR("bcjia@ingenic.cn");
MODULE_DESCRIPTION("Ingenic Gt801 Touchscreen Driver");
MODULE_LICENSE("GPL");
