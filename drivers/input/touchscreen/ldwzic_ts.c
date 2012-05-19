/* 
 * drivers/input/touchscreen/ldwzic_ts.c
 *
 * FocalTech ldwzic TouchScreen driver. 
 *
 * Copyright (c) 2010  Ingenic Semiconductor Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 *	note: only support mulititouch	liaoqizhen 2010-09-01
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>
#include <asm/jzsoc.h>
#include <linux/slab.h>
#include "ldwzic_ts.h"

static struct i2c_client *this_client;
static struct ldwzic_ts_platform_data *pdata;
//#undef CONFIG_LINDAWZ_DEBUG
//#define CONFIG_LINDAWZ_DEBUG

//#define PENUP_TIMEOUT_TIMER 1
//#define P2_PACKET_LEN 14
//#define P1_PACKET_LEN 9
#define POINT_READ_LEN 9 //16 //14 //9
#define REG_SUM  14 //18 //14

#define CONFIG_TOUCHSCREEN_X_FLIP 1
#define CONFIG_TOUCHSCREEN_Y_FLIP 1

struct ts_event {
	u16	x1;
	u16	y1;
	u16	x2;
	u16	y2;
	u16 pen_up;
	u16	pressure;
	u8  touch_point;
};

struct ldwzic_ts_data {
	struct input_dev	*input_dev;
	struct ts_event		event;
	struct work_struct 	pen_event_work;
	struct workqueue_struct *ts_workqueue;
	struct early_suspend	early_suspend;
#ifdef PENUP_TIMEOUT_TIMER
	struct timer_list penup_timeout_timer;
#endif
#ifdef CONFIG_LINDAWZ_DEBUG
	long int count;
#endif
};

static int ldwzic_i2c_rxdata(u8 *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0, //|I2C_M_NOSTART,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD, //|I2C_M_NOSTART,
			.len	= length,
			.buf	= rxdata,
		},
	};
	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);	
	return ret;
}

static int ldwzic_i2c_txdata(u8 *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0, //|I2C_M_NOSTART,
			.len	= length,
			.buf	= txdata,
		},
	};

	msleep(1);
	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}

static int ldwzic_set_reg(u8 addr, u8 para)
{
	u8 buf[3];
	int ret = -1;
	buf[0] = addr;
	buf[1] = para;
	ret = ldwzic_i2c_txdata(buf, 2);
	if (ret < 0) {
		pr_err("write reg failed! %#x ret: %d", buf[0], ret);
		return -1;
	}

	return 0;
}

static void ldwzic_ts_release(struct ldwzic_ts_data *ldwzic_ts)
{
#ifdef CONFIG_LDWZIC_MULTI_TOUCH	
	input_mt_sync(ldwzic_ts->input_dev);
#else
	input_report_abs(ldwzic_ts->input_dev, ABS_PRESSURE, 0);
	input_report_key(ldwzic_ts->input_dev, BTN_TOUCH, 0);
#endif
	input_sync(ldwzic_ts->input_dev);
#ifdef PENUP_TIMEOUT_TIMER
	del_timer(&(ldwzic_ts->penup_timeout_timer));
#endif
}

static void ldwzic_chip_reset(void)
{
	__gpio_as_output(GPIO_LDWZIC_RST_PIN);
	__gpio_set_pin(GPIO_LDWZIC_RST_PIN);
	msleep(20); //40);
	__gpio_clear_pin(GPIO_LDWZIC_RST_PIN);
	msleep(20); //40);
	__gpio_set_pin(GPIO_LDWZIC_RST_PIN);
	msleep(20); //40);
}

#if 1
static unsigned long transform_to_screen_x(unsigned long x )
{
	if (x < TP_MIN_X) 
		x = TP_MIN_X;
	if (x > TP_MAX_X) 
		x = TP_MAX_X;

	return (x - TP_MIN_X) * SCREEN_MAX_X / (TP_MAX_X - TP_MIN_X);
}

static unsigned long transform_to_screen_y(unsigned long y)
{
	if (y < TP_MIN_Y) 
		y = TP_MIN_Y;
	if (y > TP_MAX_Y) 
		y = TP_MAX_Y;

	//return (TP_MAX_Y - y) * SCREEN_MAX_Y / (TP_MAX_Y - TP_MIN_Y);
	return (y - TP_MIN_Y) * SCREEN_MAX_Y / (TP_MAX_Y - TP_MIN_Y);
}

static int ldwzic_read_data(void)
{
	struct ldwzic_ts_data *ldwzic_ts = i2c_get_clientdata(this_client);
	struct ts_event *event = &ldwzic_ts->event;
	u8 buf[REG_SUM];
	int ret = -1;

	buf[0] = 0;// read from reg 0
	ret = ldwzic_i2c_rxdata(buf, POINT_READ_LEN);
	if (ret < 0) {
		ldwzic_chip_reset();
		printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}
	memset(event, 0, sizeof(struct ts_event));
	event->touch_point = buf[0] & REG_STATE_MASK;
	//CLIVIA DON'T KNOW WHAT DO FOR	event->pen_up = ((buf[1] >> 6)==0x01);

	if ((event->touch_point == 0)) {
		ldwzic_ts_release(ldwzic_ts);
		return 1; 
	}
#ifdef CONFIG_LDWZIC_MULTI_TOUCH
	switch (event->touch_point) {
		case 2:
			event->x1 = ((((u16)buf[5])<<8)&0x0f00) |buf[6];
			event->y1 = ((((u16)buf[7])<<8)&0x0f00) |buf[8];
			event->touch_point = 1;// ONE POINT

			break;
		case 1:
			event->x1 = ((((u16)buf[1])<<8)&0x0f00) |buf[2];
			event->y1 = ((((u16)buf[3])<<8)&0x0f00) |buf[4];

			break;
		case 3:
			event->x1 = ((((u16)buf[1])<<8)&0x0f00) |buf[2];
			event->y1 = ((((u16)buf[3])<<8)&0x0f00) |buf[4];
			event->x2 = ((((u16)buf[5])<<8)&0x0f00) |buf[6];
			event->y2 = ((((u16)buf[7])<<8)&0x0f00) |buf[8];
			event->touch_point = 2;// TWO POINT
			break;
		default:
			return -1;
	}
#ifdef CONFIG_TOUCHSCREEN_X_FLIP
	event->x1 = SCREEN_MAX_X - transform_to_screen_x(event->x1);
#else
	event->x1 = transform_to_screen_x(event->x1);
#endif

#ifdef CONFIG_TOUCHSCREEN_Y_FLIP
	event->y1 = SCREEN_MAX_Y - transform_to_screen_y(event->y1);
#else
	event->y1 = transform_to_screen_y(event->y1);
#endif

	if(event->touch_point == 2)
	{

#ifdef CONFIG_TOUCHSCREEN_X_FLIP
		event->x2 = SCREEN_MAX_X - transform_to_screen_x(event->x2);
#else
		event->x2 = transform_to_screen_x(event->x2);
#endif

#ifdef CONFIG_TOUCHSCREEN_Y_FLIP
		event->y2 = SCREEN_MAX_Y - transform_to_screen_y(event->y2);
#else
		event->y2 = transform_to_screen_y(event->y2);
#endif

	}
#else
	switch (event->touch_point) {
		case 2:
			event->x1 = ((((u16)buf[5])<<8)&0x0f00) |buf[6];
			event->y1 = ((((u16)buf[7])<<8)&0x0f00) |buf[8];
		case 1:
			event->x1 = ((((u16)buf[1])<<8)&0x0f00) |buf[2];
			event->y1 = ((((u16)buf[3])<<8)&0x0f00) |buf[4];
			break;
		default:
			return -1;
	}
#ifdef CONFIG_TOUCHSCREEN_X_FLIP
	event->x1 = SCREEN_MAX_X - transform_to_screen_x(event->x1);
#else
	event->x1 = transform_to_screen_x(event->x1);
#endif

#ifdef CONFIG_TOUCHSCREEN_Y_FLIP
	event->y1 = SCREEN_MAX_Y - transform_to_screen_y(event->y1);
#else
	event->y1 = transform_to_screen_y(event->y1);
#endif
#endif
	event->pressure = 200;
#ifdef PENUP_TIMEOUT_TIMER
	mod_timer(&(ldwzic_ts->penup_timeout_timer), jiffies+40);
#endif
	return 0;
}

#else
static int ldwzic_read_data(void)
{
	struct ldwzic_ts_data *ldwzic_ts = i2c_get_clientdata(this_client);
	struct ts_event *event = &ldwzic_ts->event;
	u8 buf[P2_PACKET_LEN+1] = {0};
	//int i = 0;
	int ret = -1;

	buf[0] = 2;
#ifdef CONFIG_LDWZIC_MULTI_TOUCH
	ret = ldwzic_i2c_rxdata(buf, P2_PACKET_LEN);
#else
	ret = ldwzic_i2c_rxdata(buf, P1_PACKET_LEN);
#endif
	if (ret < 0) {
		ldwzic_chip_reset();
		printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}


	if((buf[1]==0xff) && (buf[2]==0xff) && (buf[3]==0xff) && (buf[4]==0xff)) {
		//ldwzic_ts_release(ldwzic_ts);
		return 1;
	}

	memset(event, 0, sizeof(struct ts_event));
	event->touch_point = buf[0] & 0x0f;
	event->pen_up = ((buf[1] >> 6)==0x01);

	if ((event->touch_point == 0)) {
		ldwzic_ts_release(ldwzic_ts);
		return 1; 
	}

#ifdef CONFIG_LDWZIC_MULTI_TOUCH
	switch (event->touch_point) {
		case 2:
			event->x2 = ((((u16)buf[9])<<8)&0x0f00) |buf[10];
			event->y2 = ((((u16)buf[7])<<8)&0x0f00) |buf[8];
		case 1:
			event->x1 = ((((u16)buf[3])<<8)&0x0f00) |buf[4];
			event->y1 = ((((u16)buf[1])<<8)&0x0f00) |buf[2];
			break;
		default:
			return -1;
	}
#else
	if (event->touch_point == 1) {
		event->x1 = ((((s16)buf[3])<<8)&0x0f00) |buf[4];
		event->y1 =((((s16)buf[1])<<8)&0x0f00) |buf[2]; 
	}
#endif
	event->pressure = 200;
#ifdef PENUP_TIMEOUT_TIMER
	mod_timer(&(ldwzic_ts->penup_timeout_timer), jiffies+40);
#endif


	return 0;
}

#endif
static void ldwzic_report_value(void)
{
	struct ldwzic_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;

#ifdef CONFIG_LDWZIC_MULTI_TOUCH
	switch(event->touch_point) {
		case 2:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x2);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y2);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input_dev);
		case 1:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x1);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y1);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input_dev);
		default:
			break;
	}
#else	/* CONFIG_LDWZIC_MULTI_TOUCH*/
	if (event->touch_point == 1) {
		input_report_abs(data->input_dev, ABS_X, event->x1);
		input_report_abs(data->input_dev, ABS_Y, event->y1);
		input_report_abs(data->input_dev, ABS_PRESSURE, event->pressure);
	}
	input_report_key(data->input_dev, BTN_TOUCH, 1);
#endif	/* CONFIG_LDWZIC_MULTI_TOUCH*/
	input_sync(data->input_dev);

	dev_dbg(&this_client->dev, "%s: 1:%d %d 2:%d %d \n", __func__,
			event->x1, event->y1, event->x2, event->y2);
}	/*end ldwzic_report_value*/

static void ldwzic_ts_pen_irq_work(struct work_struct *work)
{
	int ret = -1;
#ifdef CONFIG_LINDAWZ_DEBUG
	long int count = 0;
	struct ldwzic_ts_data *ts;
	ts =  container_of(work, struct ldwzic_ts_data, pen_event_work);
	count = ts->count;
	printk("==ldwzic read data start(%ld)=\n", count);
#endif

	ret = ldwzic_read_data();	
	if (ret == 0) {	
		ldwzic_report_value();
	}
#ifdef CONFIG_LINDAWZ_DEBUG
	printk("==ldwzic report data end(%ld)=\n", count++);
#endif
	//msleep(1);
	enable_irq(this_client->irq);
}

static irqreturn_t ldwzic_ts_interrupt(int irq, void *dev_id)
{
	struct ldwzic_ts_data *ldwzic_ts = dev_id;
#ifdef CONFIG_LINDAWZ_DEBUG
	static long int count = 0;
#endif
	disable_irq_nosync(this_client->irq);

#ifdef CONFIG_LINDAWZ_DEBUG
	ldwzic_ts->count = count;	
	printk("==ctp int(%ld)=\n", count++);	
#endif

	if (!work_pending(&ldwzic_ts->pen_event_work)) {
		queue_work(ldwzic_ts->ts_workqueue, &ldwzic_ts->pen_event_work);
	}

	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ldwzic_ts_suspend(struct early_suspend *handler)
{
	struct ldwzic_ts_data *ts;
	ts =  container_of(handler, struct ldwzic_ts_data, early_suspend);

#ifdef CONFIG_LINDAWZ_DEBUG
	printk("==ldwzic_ts_suspend=\n");
#endif
	disable_irq(this_client->irq);
	if(cancel_work_sync(&ts->pen_event_work))
		enable_irq(this_client->irq);
	flush_workqueue(ts->ts_workqueue);
	//ldwzic_set_reg(LDWZIC_REG_CTRL_OPMODE, REG_OPMODE_POFF | REG_OPMODE_128MS_DSLEEP_5_ADC);
	//msleep(20);
}

static void ldwzic_ts_resume(struct early_suspend *handler)
{
#ifdef CONFIG_LINDAWZ_DEBUG
	printk("==ldwzic_ts_resume=\n");
#endif

	ldwzic_chip_reset();	//add by KM: 20111010

	ldwzic_set_reg(LDWZIC_REG_CTRL_OPMODE, REG_TOUCH_USED_OPMODE);
	msleep(20);
	enable_irq(this_client->irq);
}
#endif  //CONFIG_HAS_EARLYSUSPEND

	static int 
ldwzic_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ldwzic_ts_data *ldwzic_ts;
	struct input_dev *input_dev;
	int err = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ldwzic_ts = kzalloc(sizeof(*ldwzic_ts), GFP_KERNEL);
	if (!ldwzic_ts)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	this_client = client;
	i2c_set_clientdata(client, ldwzic_ts);
	//	i2c_jz_setclk(client, 200*1000);

	INIT_WORK(&ldwzic_ts->pen_event_work, ldwzic_ts_pen_irq_work);
	ldwzic_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!ldwzic_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}

	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		dev_err(&client->dev, "%s: platform data is null\n", __func__);
		goto exit_platform_data_null;
	}

	err = request_irq(client->irq, ldwzic_ts_interrupt, IRQF_DISABLED, "ldwzic_ts", ldwzic_ts);
	if (err < 0) {
		dev_err(&client->dev, "ldwzic_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

	__gpio_as_input(pdata->intr);
	__gpio_as_irq_fall_edge(pdata->intr);
	disable_irq(this_client->irq);

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	ldwzic_ts->input_dev = input_dev;

#ifdef CONFIG_LDWZIC_MULTI_TOUCH
	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);

	input_set_abs_params(input_dev,
			ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev,
			ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev,
			ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
#else
	set_bit(ABS_X, input_dev->absbit);
	set_bit(ABS_Y, input_dev->absbit);
	set_bit(ABS_PRESSURE, input_dev->absbit);
	set_bit(BTN_TOUCH, input_dev->keybit);

	input_set_abs_params(input_dev, ABS_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			ABS_PRESSURE, 0, PRESS_MAX, 0 , 0);
#endif

	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);

	input_dev->name		= LDWZIC_NAME;		//dev_name(&client->dev)
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
				"ldwzic_ts_probe: failed to register input device: %s\n",
				dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

#ifdef PENUP_TIMEOUT_TIMER
	init_timer(&(ldwzic_ts->penup_timeout_timer));
	ldwzic_ts->penup_timeout_timer.data = (unsigned long)ldwzic_ts;
	ldwzic_ts->penup_timeout_timer.function  =	(void (*)(unsigned long))ldwzic_ts_release;
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	ldwzic_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ldwzic_ts->early_suspend.suspend = ldwzic_ts_suspend;
	ldwzic_ts->early_suspend.resume	= ldwzic_ts_resume;
	register_early_suspend(&ldwzic_ts->early_suspend);
#endif


	ldwzic_chip_reset();	//add by KM: 20111010

	//__gpio_as_input(SENSOR_INT1);

	ldwzic_set_reg(LDWZIC_REG_CTRL_OPMODE, REG_TOUCH_USED_OPMODE); //5, 6,7,8
	msleep(40);

	enable_irq(this_client->irq);

	return 0;

exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
	free_irq(client->irq, ldwzic_ts);
exit_irq_request_failed:
exit_platform_data_null:
	cancel_work_sync(&ldwzic_ts->pen_event_work);
	destroy_workqueue(ldwzic_ts->ts_workqueue);
exit_create_singlethread:
	i2c_set_clientdata(client, NULL);
	kfree(ldwzic_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

static int __devexit ldwzic_ts_remove(struct i2c_client *client)
{
	struct ldwzic_ts_data *ldwzic_ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ldwzic_ts->early_suspend);
	free_irq(client->irq, ldwzic_ts);
	input_unregister_device(ldwzic_ts->input_dev);
	kfree(ldwzic_ts);
	cancel_work_sync(&ldwzic_ts->pen_event_work);
	destroy_workqueue(ldwzic_ts->ts_workqueue);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id ldwzic_ts_id[] = {
	{ LDWZIC_NAME, 0 },
};
MODULE_DEVICE_TABLE(i2c, ldwzic_ts_id);

static struct i2c_driver ldwzic_ts_driver = {
	.probe		= ldwzic_ts_probe,
	.remove		= __devexit_p(ldwzic_ts_remove),
	.id_table	= ldwzic_ts_id,
	.driver	= {
		.name	= LDWZIC_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init ldwzic_ts_init(void)
{
	return i2c_add_driver(&ldwzic_ts_driver);
}

static void __exit ldwzic_ts_exit(void)
{
	i2c_del_driver(&ldwzic_ts_driver);
}

module_init(ldwzic_ts_init);
module_exit(ldwzic_ts_exit);

MODULE_AUTHOR("<clivia_cui@163.com>");
MODULE_DESCRIPTION("linda unknown test ic TouchScreen driver");
MODULE_LICENSE("GPL");
