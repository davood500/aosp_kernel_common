/*
 * drivers/i2c/chips/lis35de.c
 * - lis35de 3-axis digital low-g accelerometer driver
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
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/input.h>
#include <asm/gpio.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>

#include <asm/jzsoc.h>
#include <linux/linux_sensors.h>
#include "lis33de_35de.h"


#define DEBUG 0
#define MMA_DELAY_MAX	40
#define MMA_DELAY_MIN	5

static struct i2c_client *this_client;
static struct workqueue_struct *mma_wqueue;
static atomic_t a_active;
static int mma_delay = HZ/10;

struct lis35de_data {
	struct	input_dev *input_dev;
	struct  sensors_platform_data *pdata;
	struct	work_struct work;
	struct	delayed_work mma_work;
	struct	early_suspend early_suspend;
	
	struct	mutex lock;
	unsigned disabled:1;	/* P: mutex */

	struct 	mutex gsensor_data_lock;
};
static struct linux_sensor_t hardware_data = {
		"lis35de 3-axis Accelerometer",
                 "Mems motion sensor",
               // SENSOR_TYPE_ACCELEROMETER,0,1,64, 4, { } // 类型是重力感应硬件,重力感应的数值是1个G，上传的数据向量最大值为64，估计电流为4mA.
							 SENSOR_TYPE_ACCELEROMETER,0,64,1, 1, { }
};
#ifdef CONFIG_SENSORS_LIS33DE_35DE_ORI
extern void orientation_report_values(int x,int y,int z);
#endif
static int LIS35DE_I2C_RxData(char *rxData, int length)
{
	int ret;

	ret = i2c_master_send(this_client,rxData ,1);
	if (ret < 0) {
		printk("write error\n");
		return ret;
	} else if (ret != 1){
		return -EIO;
	}
	ret = i2c_master_recv(this_client, rxData ,length);
	if (ret < 0) {
		printk("Read error ,ret=%d\n",ret);
		return ret;
	}
	
	return 0;
}

static int LIS35DE_I2C_TxData(char *txData, int length)
{
	int retry;
	struct i2c_msg msg[] = {
		{
			.addr = this_client->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	for (retry = 0; retry <= 10; retry++) {

		if(i2c_transfer(this_client->adapter, msg, 1) > 0)
			break;
		else
			mdelay(10);
	}
	if (retry > 10) {
		printk(KERN_ERR "%s: retry over 10\n", __func__);
		return -EIO;
	}

	return 0;
}
static int lis35de_read_xyz(short *rbuf)
{
	char buffer[1];
	int ret;

	buffer[0] = LIS35DE_OUT_X;
	ret = LIS35DE_I2C_RxData(buffer, 1);
	if (ret < 0)
		return ret;
	rbuf[0] = (buffer[0]&0xff);
	if (rbuf[0] & 0x80)
		rbuf[0] |= 0xff00;
	
	buffer[0] = LIS35DE_OUT_Y;
	ret = LIS35DE_I2C_RxData(buffer, 1);
	if (ret < 0)
		return ret;
	rbuf[1] = (buffer[0]&0xff);
	if (rbuf[1] & 0x80)
		rbuf[1] |= 0xff00;

	
	buffer[0] = LIS35DE_OUT_Z;
	ret = LIS35DE_I2C_RxData(buffer, 1);
	if (ret < 0)
		return ret;
	rbuf[2] = (buffer[0]&0xff);
	if (rbuf[2] & 0x80)
		rbuf[2] |= 0xff00;
#if DEBUG
	printk("raw data: %d %d %d\n", buffer[0], buffer[1], buffer[2]);
#endif

	return 0;
}

static int lis35de_startup(void)
{
	char buffer[8];
	int ret;

	memset(buffer, 0, 8);
	buffer[0] = LIS35DE_CTRL_REG1;
	buffer[1] = 0x47;
	ret = LIS35DE_I2C_TxData(buffer, 2);
	if (ret < 0)
		return ret;
	buffer[0] = LIS35DE_CTRL_REG2;
	buffer[1] = 0x0;
	ret = LIS35DE_I2C_TxData(buffer, 2);
	if (ret < 0)
		return ret;

	buffer[0] = LIS35DE_CTRL_REG3;
	buffer[1] = 0x0;
	ret = LIS35DE_I2C_TxData(buffer, 2);
	if (ret < 0)
		return ret;
#if DEBUG
	buffer[0] = LIS35DE_CLICK_CFG;
	buffer[1]=0x7;
	ret = LIS35DE_I2C_RxData(buffer, 2);
	if (ret < 0)
		return ret;
	printk("lis35de userinfo: %2x, whoami: %2x\n", buffer[0], buffer[1]);
	buffer[0] = LIS35DE_CTRL_REG2;
	buffer[1]=0x6;
	ret = LIS35DE_I2C_RxData(buffer, 2);
	if (ret < 0)
		return ret;
	
	printk("lis35de userinfo: %2x, whoami: %2x\n", buffer[0], buffer[1]);
	buffer[0] = LIS35DE_CTRL_REG3;
	buffer[1]=0x7;
	ret = LIS35DE_I2C_RxData(buffer, 2);
	if (ret < 0)
		return ret;
	
	printk("lis35de userinfo: %2x, whoami: %2x\n", buffer[0], buffer[1]);
	buffer[4] = 0x50;
	printk("lis35de userinfo: %2x, whoami: %2x\n", buffer[4], buffer[4]);
#endif
	return 0;
}


/*
 * set operation mode
 * 0 = standby
 * 1 = active
 */
static int lis35de_set_mode(char mode, char range)
{
	char buffer[2];
	int ret;

	buffer[0] = LIS35DE_CTRL_REG1;
	ret = LIS35DE_I2C_RxData(buffer, 1);
	if (ret < 0)
		return -1;

	buffer[1] = buffer[0] | (mode << 6);
	buffer[0] = LIS35DE_CTRL_REG1;
	return LIS35DE_I2C_TxData(buffer, 2); 
}


static int lis35de_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int lis35de_release(struct inode *inode, struct file *file)
{
	return 0;
}

#ifdef CONFIG_LIS33DE_35DE_AS_INPUT
static void lis35de_report_value(short *rbuf)
{
	short report[3];
	short i;
	struct lis35de_data *data = i2c_get_clientdata(this_client);
	for(i = 0; i < 3;i++)
	{
		report[i] = 	rbuf[0]*(data->pdata->plate_data->matrix[i])+
				rbuf[1]*(data->pdata->plate_data->matrix[i+3])+
				rbuf[2]*(data->pdata->plate_data->matrix[i+6]);
		//report[i] *= 64;
		//report[i] /= 57;
	}
	for(i = 0;i < 3;i++)
	{
		report[i] = report[i] + (data->pdata->plate_data->matrix[i+9]);
	}	
#if DEBUG
	printk( "x: %d, y: %d, z: %d LSB\n", rbuf[0], rbuf[1], rbuf[2]);
	printk( "x: %d, y: %d, z: %d change\n", report[0], report[1], report[2]);
#endif

	/* Report acceleration sensor information */
	if (atomic_read(&a_active)) {
		input_report_abs(data->input_dev, ABS_X, report[0]);
		input_report_abs(data->input_dev, ABS_Y, report[1]);
		input_report_abs(data->input_dev, ABS_Z, report[2]);
	/* input_report_abs(data->input_dev, ABS_WHEEL, rbuf[5]); */
		input_sync(data->input_dev);
	#ifdef CONFIG_SENSORS_LIS33DE_35DE_ORI
	orientation_report_values(report[0],report[1],report[2]);
#endif
	}
}
#endif

static long lis35de_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{

	struct lis35de_data *lis35de = i2c_get_clientdata(this_client);
	void __user *argp = (void __user *)arg;
	unsigned long  flag;
	
	switch (cmd) {
	case SENSOR_IOCTL_SET_DELAY:
		if (copy_from_user(&flag, argp, sizeof(flag)))
			return -EFAULT;
		if(flag > MMA_DELAY_MAX) flag = MMA_DELAY_MAX;
		if(flag < MMA_DELAY_MIN) flag = MMA_DELAY_MIN;
		mma_delay = flag;
		printk("------------>mma_delay = %d\n",mma_delay);
		break;
	case SENSOR_IOCTL_GET_DELAY:
		flag = mma_delay;
		if (copy_to_user(argp, &flag, sizeof(flag)))
			return -EFAULT;
		break;
	case SENSOR_IOCTL_SET_ACTIVE:
		if (copy_from_user(&flag, argp, sizeof(flag)))
			return -EFAULT;
		if (flag < 0 || flag > 1)
			return -EFAULT;
		atomic_set(&a_active, flag);
		if(flag != 0)
			queue_delayed_work(mma_wqueue, &lis35de->mma_work, mma_delay);
		break;
	case SENSOR_IOCTL_GET_ACTIVE:
		flag = atomic_read(&a_active);
		if (copy_to_user(argp, &flag, sizeof(flag)))
			return -EFAULT;
		break;
	case SENSOR_IOCTL_GET_DATA:
		if (copy_to_user(argp, &hardware_data, sizeof(hardware_data)))
			return -EFAULT;
		break;
	default:
		break;
	}
	
	return 0;
}

static void lis35de_disable(struct lis35de_data *lis35de)
{
	mutex_lock(&lis35de->lock);
	if (!lis35de->disabled)
	{
		lis35de->disabled = 1;	
		cancel_delayed_work_sync(&lis35de->mma_work);
		flush_workqueue(mma_wqueue);
	}
	mutex_unlock(&lis35de->lock);
}

static void lis35de_enable(struct lis35de_data *lis35de)
{
	mutex_lock(&lis35de->lock);
	if (lis35de->disabled) {
		lis35de->disabled = 0;
		queue_delayed_work(mma_wqueue, &lis35de->mma_work, mma_delay);
	}
	mutex_unlock(&lis35de->lock);
}


#ifdef CONFIG_LIS33DE_35DE_AS_INPUT
static void lis35de_work(struct work_struct *work)
{
	struct lis35de_data *lis35de = i2c_get_clientdata(this_client);
	int ret = 0;
	short buf[3];
	ret = lis35de_read_xyz(buf);
	if (ret < 0) {
		printk(KERN_ERR "get acceleration data failed\n");
	} else {
		lis35de_report_value(buf);
	}
	if (atomic_read(&a_active)) {
		queue_delayed_work(mma_wqueue, &lis35de->mma_work, mma_delay);
	}
}
#endif


#ifdef CONFIG_HAS_EARLYSUSPEND
static void lis35de_early_suspend(struct early_suspend *handler)
{
	struct lis35de_data *lis35de = i2c_get_clientdata(this_client);
	lis35de_disable(lis35de);
	lis35de_set_mode(LIS35DE_MODE_STANDBY, LIS_RANGE_2G);
}

static void lis35de_early_resume(struct early_suspend *handler)
{
	struct lis35de_data *lis35de = i2c_get_clientdata(this_client);
	lis35de_set_mode(LIS35DE_MODE_ACTIVE, LIS_RANGE_2G);
	lis35de_enable(lis35de);
}
#endif

static struct file_operations lis35de_fops = {
	.owner = THIS_MODULE,
	.open = lis35de_open,
	.release = lis35de_release,
	.unlocked_ioctl = lis35de_ioctl,
};

static struct miscdevice lis35de_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = G_SENSOR_NAME,
	.fops = &lis35de_fops,
};

static ssize_t lis35de_show_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", mma_delay);
}

static ssize_t lis35de_store_delay(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long val;
	int ret = strict_strtoul(buf, 10, &val);
	if (ret)
		return -EINVAL;	
	mma_delay = val;

	return ret ? ret : count;
}

static ssize_t lis35de_show_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&a_active));
}

static ssize_t lis35de_store_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long val;
	struct lis35de_data *lis35de = i2c_get_clientdata(this_client);
	int ret = strict_strtoul(buf, 10, &val);
	if (ret)
		return -EINVAL;
	if (val < 0 || val > 1)
		return -EINVAL;

	atomic_set(&a_active, val);
	if(val != 0)
		queue_delayed_work(mma_wqueue, &lis35de->mma_work, mma_delay);
	
	return ret ? ret : count;
}

/*
 * ATTRIBUTE
 * /sys/class/i2c-adapter/i2c-%d/%d-004c/delay [rw]
 * /sys/class/i2c-adapter/i2c-%d/%d-004c/enable [rw]
 */

static DEVICE_ATTR(delay, S_IWUSR | S_IRUGO,
		   lis35de_show_delay,
		   lis35de_store_delay);
static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO,
		   lis35de_show_enable,
		   lis35de_store_enable);
static struct attribute *lis35de_attributes[] = {
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	NULL
};

static const struct attribute_group lis35de_attr_group = {
	.attrs	= lis35de_attributes,
};


static int lis35de_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct lis35de_data *lis35de;
	int err = 0;
#if DEBUG 
	int i,j;
#endif
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	//i2c_jz_setclk(client, 100*1000);
	lis35de = kzalloc(sizeof(struct lis35de_data), GFP_KERNEL);
	if (!lis35de) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	i2c_set_clientdata(client, lis35de);

	lis35de->pdata = client->dev.platform_data;
	if (lis35de->pdata == NULL) {
		printk(KERN_ERR "lis35de_init_client: platform data is NULL\n");
		goto exit_platform_data_null;
	}
#if DEBUG
	printk("lis35de plate data status %d\n",lis35de->pdata->plate_data->status);
	for(i = 0; i < 9;i++)
	{
		printk("lis35de plate data matrix %d\n",lis35de->pdata->plate_data->matrix[i]);
	}
#endif
	this_client = client;
	
	mutex_init(&lis35de->lock);
	mutex_init(&lis35de->gsensor_data_lock);
	err = lis35de_startup();
	if (err < 0) {
		printk(KERN_ERR "lis35de_probe: mma_init failed\n");
		goto exit_init_failed;
	}

	atomic_set(&a_active, 1);
#ifdef CONFIG_LIS33DE_35DE_AS_INPUT
	INIT_DELAYED_WORK(&lis35de->mma_work, lis35de_work);

	mma_wqueue = create_singlethread_workqueue("lis35de");
	if (!mma_wqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}
	queue_delayed_work(mma_wqueue, &lis35de->mma_work, HZ*10);
#endif


	lis35de->input_dev = input_allocate_device();
	if (!lis35de->input_dev) {
		err = -ENOMEM;
		printk(KERN_ERR
		       "LIS35DE lis35de_probe: Failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	set_bit(EV_ABS, lis35de->input_dev->evbit);
	input_set_abs_params(lis35de->input_dev, ABS_X, -1872, 1872, 0, 0);
	input_set_abs_params(lis35de->input_dev, ABS_Y, -1872, 1872, 0, 0);
	input_set_abs_params(lis35de->input_dev, ABS_Z, -1872, 1872, 0, 0);

	lis35de->input_dev->name = G_SENSOR_NAME;

	err = input_register_device(lis35de->input_dev);
	if (err) {
		printk(KERN_ERR
		       "LIS35DE lis35de_probe: Unable to register input device: %s\n",
		       lis35de->input_dev->name);
		goto exit_input_register_device_failed;
	}

	err = misc_register(&lis35de_device);
	if (err) {
		printk(KERN_ERR "lis35de_probe: device register failed\n");
		goto exit_misc_device_register_failed;
	}

	err = sysfs_create_group(&client->dev.kobj, &lis35de_attr_group);
	if (err){
		printk(KERN_ERR "lis35de_probe: sysfs_create_group failed\n");
		goto err_remove_attr;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	lis35de->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 2;
	lis35de->early_suspend.suspend = lis35de_early_suspend;
	lis35de->early_suspend.resume = lis35de_early_resume;
	register_early_suspend(&lis35de->early_suspend);
#endif

	return 0;
err_remove_attr:
	sysfs_remove_group(&client->dev.kobj, &lis35de_attr_group);
exit_misc_device_register_failed:
	input_unregister_device(lis35de->input_dev);
exit_input_register_device_failed:
	input_free_device(lis35de->input_dev);
exit_input_dev_alloc_failed:
	free_irq(client->irq, lis35de);
	destroy_workqueue(mma_wqueue);
exit_create_singlethread:
exit_init_failed:
exit_platform_data_null:
	i2c_set_clientdata(client, NULL);
	kfree(lis35de);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

static int __devexit lis35de_remove(struct i2c_client *client)
{
	struct lis35de_data *lis35de = i2c_get_clientdata(client);
	sysfs_remove_group(&client->dev.kobj, &lis35de_attr_group);
	input_unregister_device(lis35de->input_dev);
	i2c_set_clientdata(client, NULL);
	kfree(lis35de);
	return 0;
}

static const struct i2c_device_id lis35de_id[] = {
	{ "lis35de", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lis35de_id);

static  void lis35de_shutdown(struct i2c_client *lis35de)
{
	struct lis35de_data *data = i2c_get_clientdata(this_client);
	lis35de_disable(data);
	destroy_workqueue(mma_wqueue);
};

static struct i2c_driver lis35de_driver = {
	.driver	= {
		.name	= "lis35de",
		.owner	= THIS_MODULE,
	},
	.shutdown = lis35de_shutdown,
	.probe	= lis35de_probe,
	.remove	= __devexit_p(lis35de_remove),
	.id_table = lis35de_id,
};

static int __init lis35de_init(void)
{
	printk(KERN_INFO "LIS35DE g-sensor module init\n");
	return i2c_add_driver(&lis35de_driver);
}

static void __exit lis35de_exit(void)
{
	i2c_del_driver(&lis35de_driver);
}

module_init(lis35de_init);
module_exit(lis35de_exit);

MODULE_AUTHOR("lzhang@ingenic.cn");
MODULE_DESCRIPTION("MEMS LIS35DE 3-Axis digital low-g accelerometer");
MODULE_LICENSE("GPL");
