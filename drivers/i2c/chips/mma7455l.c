/*
 * drivers/i2c/chips/mma7455l.c
 * - mma7455l 3-axis digital low-g accelerometer driver
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
#include "mma7455l.h"


#define DEBUG 0
#define CALIBRATION_COUNT	3

static struct i2c_client *this_client;
static struct workqueue_struct *mma_wqueue;
static atomic_t a_active;
static int mma_delay = 0;

struct mma7455l_data {
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
		"mma7455l 3-axis Accelerometer",
                 "freescale motion sensor",
                SENSOR_TYPE_ACCELEROMETER,0,1,64, 1, { } // 类型是重力感应硬件,重力感应的数值是1个G，上传的数据向量最大值为64，估计电流为4mA.
};
short tmp_xyz[3];
static int mma7455l_set_mode(char mode, char range);
static int MMA7455L_I2C_RxData(char *rxData, int length)
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

static int MMA7455L_I2C_TxData(char *txData, int length)
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
static char data_ready(void)
{
	char buffer[2];
	int ret;

	buffer[0] = MMA7455L_REG_STATUS;
	ret = MMA7455L_I2C_RxData(buffer, 1);
	if (ret < 0)
		return ret;
	if (buffer[0]&0x1) {
		return 1;
	}

	return 0;
}

static int mma7455l_read_xyz(short *rbuf)
{
	char buffer[1];
	int ret;

	memset(buffer, 0, 3);
	while (!data_ready());
	buffer[0] = MMA7455L_REG_XOUT8;
	ret = MMA7455L_I2C_RxData(buffer, 3);
	if (ret < 0)
		return ret;
#if DEBUG
	printk("raw data: %d %d %d\n", buffer[0], buffer[1], buffer[2]);
#endif
	rbuf[0] = (buffer[0]&0xff);
	if (rbuf[0]&0x80)
		rbuf[0] |= 0xff00;

	rbuf[1] = (buffer[1]&0xff);
	if (rbuf[1]&0x80)
		rbuf[1] |= 0xff00;

	rbuf[2] = (buffer[2]&0xff);
	if (rbuf[2]&0x80)
		rbuf[2] |= 0xff00;
	return 0;
}

static int mma7455l_startup(void)
{
	char buffer[8];
	int ret;
	buffer[0] = MMA7455L_REG_MCTRL;
	buffer[1] = 0x45;
	ret = MMA7455L_I2C_TxData(buffer, 2);
	if (ret < 0)
		return ret;

	memset(buffer, 0, 8);
	buffer[0] = MMA7455L_REG_CTRL1;
	ret = MMA7455L_I2C_TxData(buffer, 8);
	if (ret < 0)
		return ret;

	buffer[0] = MMA7455L_REG_USRINFO;
	ret = MMA7455L_I2C_RxData(buffer, 2);
	if (ret < 0)
		return ret;
#if DEBUG
	pr_info("mma7455l userinfo: %#x, whoami: %#x\n", buffer[0], buffer[1]);
#endif
	mma7455l_set_mode(MMA7455L_MODE_ACTIVE, MMA_RANGE_2G);
	return 0;
}


/*
 * set operation mode
 * 0 = standby
 * 1 = measurement
 * 2 = level detection
 * 3 = pulse detection
 */
static int mma7455l_set_mode(char mode, char range)
{
	char buffer[2];
	int ret;
	buffer[0] = MMA7455L_REG_MCTRL;
	ret = MMA7455L_I2C_RxData(buffer, 1);
	if (ret < 0)
		return -1;

	buffer[1] = (buffer[0]&0xf0) | mode | (range << 2);
	buffer[0] = MMA7455L_REG_MCTRL;
	return MMA7455L_I2C_TxData(buffer, 2); 
}


static int mma7455l_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int mma7455l_release(struct inode *inode, struct file *file)
{
	return 0;
}
short abs_num(short num)
{
	if(num < 0)
		return -num;
	return num;
}
#ifdef CONFIG_MMA7455L_AS_INPUT
static void mma7455l_report_value(short *rbuf)
{
	short report[3];
	short i,j;
	unsigned int num = 0;
	struct mma7455l_data *data = i2c_get_clientdata(this_client);
	for(i = 0; i < 3;i++){
#ifdef CONFIG_SENSORS_NEW
		report[i] = 	rbuf[0]*(data->pdata->plate_data->matrix[i])+
				rbuf[1]*(data->pdata->plate_data->matrix[i+3])+
				rbuf[2]*(data->pdata->plate_data->matrix[i+6]);
		
#else
		report[i] = rbuf[i];
#endif
		num += abs(tmp_xyz[i]-report[i]);
		tmp_xyz[i] = report[i]; 
	}
#if DEBUG
	printk( "x: %d, y: %d, z: %d LSB\n", rbuf[0], rbuf[1], rbuf[2]);
	printk( "x: %d, y: %d, z: %d change\n", report[0], report[1], report[2]);
#endif

	
	/* Report acceleration sensor information */
	if (atomic_read(&a_active)) {
		if(num < 40){
			input_report_abs(data->input_dev, ABS_X, report[0]);
			input_report_abs(data->input_dev, ABS_Y, report[1]);
			input_report_abs(data->input_dev, ABS_Z, report[2]);
			/* input_report_abs(data->input_dev, ABS_WHEEL, rbuf[5]); */
			input_sync(data->input_dev);
		}
	}

}
#endif
/*---------------------------------------------------------------*/
static int mma7455l_sample(short *rbuf, int count)
{
	int i;
	short tmp[3];

	for (i = 0; i < count; i++) {
		mma7455l_read_xyz(tmp);
		rbuf[0] += tmp[0];
		rbuf[1] += tmp[1];
		rbuf[2] += tmp[2];
	}
	rbuf[0] /= count;
	rbuf[1] /= count;
	rbuf[2] /= count;
#if DEBUG
	printk("sample data: %d %d %d\n", rbuf[0], rbuf[1], rbuf[2]);
#endif
	return 0;
}

static int mma7455l_docalibration(char *buffer)
{
	int i, ret;
	short sample[3];

	for (i = 0; i < CALIBRATION_COUNT; i++) {
		memset(sample, 0, sizeof(sample));
		memset(buffer, 0, 7);
		mma7455l_sample(sample, 8);

		buffer[0] = MMA7455L_REG_XOFFL;
		ret = MMA7455L_I2C_RxData(buffer, 6);
		if (ret < 0)
			return ret;

		sample[0] *= -2;
		sample[1] *= -2;
		if (sample[2] >= 0)
			sample[2] -= 63;
		else
			sample[2] += 63;
		sample[2] *= -2;

		//printk("tmp: %d %x %x\n", sample[0], buffer[0], buffer[1]);

		sample[0] += ((buffer[1]<<8) | (buffer[0]&0xff));
		sample[1] += ((buffer[3]<<8) | (buffer[2]&0xff));
		sample[2] += ((buffer[5]<<8) | (buffer[4]&0xff));

#if DEBUG
		printk("%d %d %d\n", sample[0], sample[1], sample[2]);
#endif

		buffer[1] = sample[0]&0xff;
		buffer[2] = (sample[0]>>8)&0xff;
		buffer[3] = sample[1]&0xff;
		buffer[4] = (sample[1]>>8)&0xff;
		buffer[5] = sample[2]&0xff;
		buffer[6] = (sample[2]>>8)&0xff;

		buffer[0] = MMA7455L_REG_XOFFL;
		ret = MMA7455L_I2C_TxData(buffer, 7);
		if (ret < 0)
			return ret;
		udelay(100);
	}

	return 0;
}

static int mma7455l_write_offset(char *buf)
{
	struct mma7455l_data *mma7455l;
	char buffer[7];
	int ret;

	memset(buffer, 0, 7);
	memcpy(&buffer[1], buf, 6);
	buffer[0] = MMA7455L_REG_XOFFL;
	ret = MMA7455L_I2C_TxData(buffer, 7);
	if (ret < 0)
		return ret;
	udelay(100);	/* ensure that values are written */

	mma7455l = i2c_get_clientdata(this_client);

#ifdef CONFIG_MMA7455L_AS_INPUT
	queue_delayed_work(mma_wqueue, &mma7455l->mma_work, HZ / 10);
#endif

	return 0;
}


/*---------------------------------------------------------------*/

static long mma7455l_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{

	int ret = -1;
	unsigned long  flag;
	void __user *argp = (void __user *)arg;
	short buf[3];
	char rwbuf[7];
	struct mma7455l_data *mma7455l = i2c_get_clientdata(this_client);
	
	switch (cmd) {
	case SENSOR_IOCTL_SET_DELAY:
		if (copy_from_user(&flag, argp, sizeof(flag)))
			return -EFAULT;
		mma_delay = flag;
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
			queue_delayed_work(mma_wqueue, &mma7455l->mma_work, mma_delay);
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
	case MMA7455_IOCTL_DO_CALIBRATION:
		if (copy_from_user(&buf, argp, sizeof(buf)))
			return -EFAULT;
		ret = mma7455l_docalibration(rwbuf);
		if (ret < 0)
			return ret;
		if (copy_to_user(argp, &rwbuf[1], sizeof(rwbuf)))
			return -EFAULT;
		break;
	case MMA7455_IOCTL_WRITE_OFFSET:
		if (copy_from_user(&rwbuf, argp, sizeof(rwbuf)))
			return -EFAULT;
		ret = mma7455l_write_offset(rwbuf);
		if (ret < 0)
			return ret;
		break;
	default:
		break;
	}
	
	return 0;
}

static void mma7455l_disable(struct mma7455l_data *mma7455l)
{
	mutex_lock(&mma7455l->lock);
	if (!mma7455l->disabled)
	{
		mma7455l->disabled = 1;	
		cancel_delayed_work_sync(&mma7455l->mma_work);
		flush_workqueue(mma_wqueue);
	}
	mutex_unlock(&mma7455l->lock);
}

static void mma7455l_enable(struct mma7455l_data *mma7455l)
{
	mutex_lock(&mma7455l->lock);
	if (mma7455l->disabled) {
		mma7455l->disabled = 0;
		queue_delayed_work(mma_wqueue, &mma7455l->mma_work, mma_delay);
	}
	mutex_unlock(&mma7455l->lock);
}


#ifdef CONFIG_MMA7455L_AS_INPUT
static void mma7455l_work(struct work_struct *work)
{
	struct mma7455l_data *mma7455l = i2c_get_clientdata(this_client);
	int ret = 0;
	short buf[3];
	ret = mma7455l_read_xyz(buf);
	if (ret < 0) {
		printk(KERN_ERR "get acceleration data failed\n");
	} else {
		mma7455l_report_value(buf);
	}
	if (atomic_read(&a_active)) {
		queue_delayed_work(mma_wqueue, &mma7455l->mma_work, mma_delay);
	}
}
#endif


#ifdef CONFIG_HAS_EARLYSUSPEND
static void mma7455l_early_suspend(struct early_suspend *handler)
{
	struct mma7455l_data *mma7455l = i2c_get_clientdata(this_client);
	mma7455l_disable(mma7455l);
	mma7455l_set_mode(MMA7455L_MODE_STANDBY, MMA_RANGE_2G);
}

static void mma7455l_early_resume(struct early_suspend *handler)
{
	struct mma7455l_data *mma7455l = i2c_get_clientdata(this_client);
	mma7455l_set_mode(MMA7455L_MODE_ACTIVE, MMA_RANGE_2G);
	mma7455l_enable(mma7455l);
}
#endif

static struct file_operations mma7455l_fops = {
	.owner = THIS_MODULE,
	.open = mma7455l_open,
	.release = mma7455l_release,
	.unlocked_ioctl = mma7455l_ioctl,
};

static struct miscdevice mma7455l_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = G_SENSOR_NAME,
	.fops = &mma7455l_fops,
};

static ssize_t mma7455l_show_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", mma_delay);
}

static ssize_t mma7455l_store_delay(struct device *dev,
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

static ssize_t mma7455l_show_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&a_active));
}

static ssize_t mma7455l_store_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long val;
	struct mma7455l_data *mma7455l = i2c_get_clientdata(this_client);
	int ret = strict_strtoul(buf, 10, &val);
	if (ret)
		return -EINVAL;
	if (val < 0 || val > 1)
		return -EINVAL;

	atomic_set(&a_active, val);
	if(val != 0)
		queue_delayed_work(mma_wqueue, &mma7455l->mma_work, mma_delay);
	
	return ret ? ret : count;
}

/*
 * ATTRIBUTE
 * /sys/class/i2c-adapter/i2c-%d/%d-004c/delay [rw]
 * /sys/class/i2c-adapter/i2c-%d/%d-004c/enable [rw]
 */

static DEVICE_ATTR(delay, S_IWUSR | S_IRUGO,
		   mma7455l_show_delay,
		   mma7455l_store_delay);
static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO,
		   mma7455l_show_enable,
		   mma7455l_store_enable);
static struct attribute *mma7455l_attributes[] = {
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	NULL
};

static const struct attribute_group mma7455l_attr_group = {
	.attrs	= mma7455l_attributes,
};


static int mma7455l_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct mma7455l_data *mma7455l;
	int err = 0;
#if DEBUG 
	int i,j;
#endif
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	//i2c_jz_setclk(client, 100*1000);
	mma7455l = kzalloc(sizeof(struct mma7455l_data), GFP_KERNEL);
	if (!mma7455l) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	i2c_set_clientdata(client, mma7455l);

	mma7455l->pdata = client->dev.platform_data;
	if (mma7455l->pdata == NULL) {
		printk(KERN_ERR "mma7455l_init_client: platform data is NULL\n");
		goto exit_platform_data_null;
	}
#ifdef CONFIG_SENSORS_NEW
#if DEBUG
	printk("mma7455l plate data status %d\n",mma7455l->pdata->plate_data->status);
	for(i = 0; i < 9;i++)
	{
		printk("mma7455l plate data matrix %d\n",mma7455l->pdata->plate_data->matrix[i]);
	}
#endif
#endif
	this_client = client;
	
	mutex_init(&mma7455l->lock);
	mutex_init(&mma7455l->gsensor_data_lock);
	err = mma7455l_startup();
	if (err < 0) {
		printk(KERN_ERR "mma7455l_probe: mma_init failed\n");
		goto exit_init_failed;
	}

	atomic_set(&a_active, 1);
#ifdef CONFIG_MMA7455L_AS_INPUT
	INIT_DELAYED_WORK(&mma7455l->mma_work, mma7455l_work);

	mma_wqueue = create_singlethread_workqueue("mma7455l");
	if (!mma_wqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}
//	queue_delayed_work(mma_wqueue, &mma7455l->mma_work, HZ*10);
#endif


	mma7455l->input_dev = input_allocate_device();
	if (!mma7455l->input_dev) {
		err = -ENOMEM;
		printk(KERN_ERR
		       "MMA7455L mma7455l_probe: Failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	set_bit(EV_ABS, mma7455l->input_dev->evbit);
	input_set_abs_params(mma7455l->input_dev, ABS_X, -1872, 1872, 0, 0);
	input_set_abs_params(mma7455l->input_dev, ABS_Y, -1872, 1872, 0, 0);
	input_set_abs_params(mma7455l->input_dev, ABS_Z, -1872, 1872, 0, 0);

#ifdef CONFIG_SENSORS_NEW
	mma7455l->input_dev->name = G_SENSOR_NAME;
#else 
 	mma7455l->input_dev->name = "g-sensor";
#endif
	err = input_register_device(mma7455l->input_dev);
	if (err) {
		printk(KERN_ERR
		       "MMA7455L mma7455l_probe: Unable to register input device: %s\n",
		       mma7455l->input_dev->name);
		goto exit_input_register_device_failed;
	}

	err = misc_register(&mma7455l_device);
	if (err) {
		printk(KERN_ERR "mma7455l_probe: device register failed\n");
		goto exit_misc_device_register_failed;
	}

	err = sysfs_create_group(&client->dev.kobj, &mma7455l_attr_group);
	if (err){
		printk(KERN_ERR "mma7455l_probe: sysfs_create_group failed\n");
		goto err_remove_attr;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	mma7455l->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 2;
	mma7455l->early_suspend.suspend = mma7455l_early_suspend;
	mma7455l->early_suspend.resume = mma7455l_early_resume;
	register_early_suspend(&mma7455l->early_suspend);
#endif

	return 0;
err_remove_attr:
	sysfs_remove_group(&client->dev.kobj, &mma7455l_attr_group);
exit_misc_device_register_failed:
	input_unregister_device(mma7455l->input_dev);
exit_input_register_device_failed:
	input_free_device(mma7455l->input_dev);
exit_input_dev_alloc_failed:
	free_irq(client->irq, mma7455l);
exit_irq_request_failed:
	destroy_workqueue(mma_wqueue);
exit_create_singlethread:
exit_init_failed:
exit_platform_data_null:
	i2c_set_clientdata(client, NULL);
	kfree(mma7455l);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

static int __devexit mma7455l_remove(struct i2c_client *client)
{
	struct mma7455l_data *mma7455l = i2c_get_clientdata(client);
	sysfs_remove_group(&client->dev.kobj, &mma7455l_attr_group);
	input_unregister_device(mma7455l->input_dev);
	i2c_set_clientdata(client, NULL);
	kfree(mma7455l);
	return 0;
}

static const struct i2c_device_id mma7455l_id[] = {
	{ "mma7455l", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mma7455l_id);

static  void mma7455l_shutdown(struct i2c_client *mma7455)
{
	struct mma7455l_data *mma7455l = i2c_get_clientdata(this_client);
	mma7455l_disable(mma7455l);
	destroy_workqueue(mma_wqueue);
};
static struct i2c_driver mma7455l_driver = {
	.driver	= {
		.name	= "mma7455l",
		.owner	= THIS_MODULE,
	},
	.shutdown = mma7455l_shutdown,
	.probe	= mma7455l_probe,
	.remove	= __devexit_p(mma7455l_remove),
	.id_table = mma7455l_id,
};

static int __init mma7455l_init(void)
{
	printk(KERN_INFO "MMA7455L g-sensor module init\n");
	return i2c_add_driver(&mma7455l_driver);
}

static void __exit mma7455l_exit(void)
{
	i2c_del_driver(&mma7455l_driver);
}

module_init(mma7455l_init);
module_exit(mma7455l_exit);

MODULE_AUTHOR("lzhang@ingenic.cn");
MODULE_DESCRIPTION("Freescale MMA7455L 3-Axis digital low-g accelerometer");
MODULE_LICENSE("GPL");
