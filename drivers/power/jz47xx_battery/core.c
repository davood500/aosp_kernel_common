/*
 * linux/drivers/power/jz47xx_battery
 *
 * Battery measurement code for Ingenic JZ SOC.
 *
 * Copyright (C) 2011 ztyan <ztyan@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/syscalls.h>
#include <asm/jzsoc.h>
#include <linux/jz47xx_battery.h>

#define WAIT_STFULL_TIMES 4
#define VOLTAGE_BOUNDARY 95

int g_jz_battery_min_voltage = 0x0;

extern unsigned int jz_read_battery(void);

static enum power_supply_property power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_PRESENT,
};

static int calculate_capacity(struct jz47xx_battery_device *dev,int adv,int ac,int usb)
{
	int i = 0;

	if(ac) {
		if(adv <= dev->info.dc_chg_min_vol) return 0;
		if(adv >= dev->info.dc_chg_max_vol) return 100;
		return (adv - dev->info.dc_chg_min_vol) * VOLTAGE_BOUNDARY / (dev->info.dc_chg_max_vol - dev->info.dc_chg_min_vol);
	}

	if(usb) {
		if(adv <= dev->info.usb_chg_min_vol) return 0;
		if(adv >= dev->info.usb_chg_max_vol) return 100;
		return (adv - dev->info.usb_chg_min_vol) * VOLTAGE_BOUNDARY / (dev->info.usb_chg_max_vol - dev->info.usb_chg_min_vol);
	}

	if(dev->curve) {
		if(adv >= dev->curve[1][0])
			return 100;
		while((dev->curve[i][0]) && (dev->curve[i][1])) {
			if(adv < dev->curve[i][0] && adv >= dev->curve[i+1][0]) {
				return	(dev->curve[i][1] - dev->curve[i+1][1]) * 
					(adv - dev->curve[i+1][0]) /
					(dev->curve[i][0] - dev->curve[i+1][0]) +
					(dev->curve[i+1][1]);
			}
			++i;
		}
		return 0;
	}

	if(adv >= dev->info.max_vol) return 100;
	if(adv <= dev->info.min_vol) return 0;

	return (adv - dev->info.min_vol) * 100 / (dev->info.max_vol - dev->info.min_vol);
}

#define MAX_CT 180
#define MIN_CT 20
static void battery_calculate_falling(struct jz47xx_battery_device *dev,int adv)
{
	int tmp = calculate_capacity(dev,adv,0,0);

	if(adv < dev->info.min_vol+50000) {
		dev->next_scan_time = 15; // 15 sec
	} else {
		dev->next_scan_time = MIN_CT + dev->info.capacity * (MAX_CT - MIN_CT) / 100;
	}

	if(tmp+20 < dev->info.capacity) {
		dev->next_scan_time /= 5;
	} else if(tmp+15 < dev->info.capacity) {
		dev->next_scan_time /= 4;
	} else if(tmp+10 < dev->info.capacity) {
		dev->next_scan_time /= 3;
	} else if(tmp < dev->info.capacity) {
		dev->next_scan_time /= 2;
	}

	if(dev->next_scan_time < 15)
		dev->next_scan_time = 15;

	if(tmp < dev->info.capacity) {
		dev->info.capacity--;
		dev->wait_stfull_timeout = WAIT_STFULL_TIMES;
	}

	if(dev->info.capacity < 0) dev->info.capacity = 0;
}

static void battery_calculate_rising(struct jz47xx_battery_device *dev,int adv,int ac,int usb)
{
	int tmp = 0;
	static int count = 0;
	if(ac) {
		if(dev->info.capacity >= VOLTAGE_BOUNDARY) {
			dev->next_scan_time = 5 * 60;//5min * (100 - VOLTAGE_BOUNDARY) = 30min
			dev->info.capacity++;
		} else {
			tmp = calculate_capacity(dev,adv,ac,usb);
			if(tmp > dev->info.capacity) {
				dev->info.capacity++;
				count = 0;
				dev->next_scan_time = dev->dc_charg_time >> 1;//3 * 30;//1.5min
			} else {
				if(++count > 5) {//15min
					dev->info.capacity++;
					count = 0;
				}
				dev->next_scan_time = dev->dc_charg_time;//3 * 60;//3min
			}
		}//15 * 5min + 85 * 2min = 245 min = 4 hours
		printk("%s DC\n",__FUNCTION__);
	} else {
		tmp = calculate_capacity(dev,adv,ac,usb);
		dev->next_scan_time = dev->usb_charg_time;//3 * 60;//3min
		if(tmp > dev->info.capacity)
			dev->info.capacity++;
		printk("%s USB\n",__FUNCTION__);
	}

	if (dev->info.capacity >= 99) {
		if (dev->state == POWER_SUPPLY_STATUS_FULL || 
				!dev->wait_stfull_timeout) {
			dev->info.capacity = 100;
			dev->next_scan_time = 60;
		} else {
			dev->info.capacity = 99;

#ifdef CONFIG_ACT8600_HAS_CHARGE_LED
			dev->next_scan_time = 5;
#else
			dev->wait_stfull_timeout--;
#endif
		}
	}
}

void battery_calculate(struct jz47xx_battery_device *dev,int adv,int state,int ac,int usb)
{
	switch(state) {
		case POWER_SUPPLY_STATUS_CHARGING:
		case POWER_SUPPLY_STATUS_FULL:
			battery_calculate_rising(dev,adv,ac,usb);
			return;
		case POWER_SUPPLY_STATUS_DISCHARGING:
		case POWER_SUPPLY_STATUS_NOT_CHARGING:
			battery_calculate_falling(dev,adv);
			return;
		case POWER_SUPPLY_STATUS_UNKNOWN:
			dev->next_scan_time = 60;
			return;
	}
}


static int get_power_prop(struct power_supply *psy,enum power_supply_property psp,union power_supply_propval *val)
{
	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
				struct jz47xx_battery_device *dev = container_of(psy,struct jz47xx_battery_device,ac);
				val->intval = dev->get_ad_online_state(dev->interface);
			} else if(psy->type == POWER_SUPPLY_TYPE_USB) {
				struct jz47xx_battery_device *dev = container_of(psy,struct jz47xx_battery_device,usb);
				val->intval = dev->get_usb_online_state(dev->interface);
			}
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static int battery_get_property(struct power_supply *psy,enum power_supply_property psp,union power_supply_propval *val)
{
	struct jz47xx_battery_device *dev = container_of(psy,struct jz47xx_battery_device,battery);

	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			val->intval = dev->state;
			break;
		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
			break;
		case POWER_SUPPLY_PROP_HEALTH:
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			val->intval = dev->info.capacity;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val->intval = dev->read_battery();
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
			val->intval = dev->info.max_vol;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:	
			val->intval = dev->info.min_vol;
			break;
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = 1;
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static int charg_changed_callback(void *data)
{
	struct jz47xx_battery_device *dev = data;
	if(!dev->enabled) return 0;

	dev->state = dev->get_charging_state(dev->interface);

	if(dev->state == POWER_SUPPLY_STATUS_FULL && dev->info.capacity != 100)
		dev->state = POWER_SUPPLY_STATUS_CHARGING;

	power_supply_changed(&dev->battery);

	if(dev->info.capacity <= 5) {
		cancel_delayed_work(&dev->update_work);
		schedule_delayed_work(&dev->update_work,15 * HZ);
		printk("next scan time : %d\n",dev->next_scan_time);
	}

	printk("battery charg change!\n");
	wake_lock_timeout(&dev->wake_lock,5*HZ);
	return 0;
}

static void update_work_func(struct work_struct *work)
{
	struct delayed_work *tmp = container_of(work,struct delayed_work,work);
	struct jz47xx_battery_device *dev = container_of(tmp,struct jz47xx_battery_device,update_work);

	dev->state = dev->get_charging_state(dev->interface);

	battery_calculate(dev,dev->read_battery(),dev->state,
			dev->get_ad_online_state(dev->interface),
			dev->get_usb_online_state(dev->interface));

	schedule_delayed_work(&dev->update_work,dev->next_scan_time * HZ);
	printk("next scan time : %d\n",dev->next_scan_time);
	power_supply_changed(&dev->battery);
}

static int proc_read_info(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int i=0,len=0;
	struct jz47xx_battery_device *dev = data;
	struct battery_info *info = &dev->info;

	len += sprintf (page+len, "capacity: %d\n",info->capacity);
	len += sprintf (page+len, "max_vol : %d\n",info->max_vol);
	len += sprintf (page+len, "min_vol : %d\n",info->min_vol);
	len += sprintf (page+len, "dc_chg_max_vol : %d\n",info->dc_chg_max_vol);
	len += sprintf (page+len, "dc_chg_min_vol : %d\n",info->dc_chg_min_vol);
	len += sprintf (page+len, "usb_chg_max_vol : %d\n",info->usb_chg_max_vol);
	len += sprintf (page+len, "usb_chg_min_vol : %d\n",info->usb_chg_min_vol);
	len += sprintf (page+len, "battery_mah: %d\n",info->battery_mah);
	len += sprintf (page+len, "dc_charg_ma: %d\n",info->dc_charg_ma);
	len += sprintf (page+len, "usb_charg_ma: %d\n",info->usb_charg_ma);
	len += sprintf (page+len, "update_time: %ld\n",info->update_time);
	len += sprintf (page+len, "curve:\n");

	if(!dev->curve) {
		len += sprintf (page+len, "-- NULL --\n");
		return len;
	}

	while((dev->curve[i][0]) && (dev->curve[i][0])) {
		len += sprintf (page+len, "%d\t%d\n",dev->curve[i][0],dev->curve[i][1]);
		++i;
	}
	return len;
}

static int proc_write_curve(struct file *file, const char *buffer, unsigned long count, void *data)
{
	int *tmp = (int *)buffer;
	struct jz47xx_battery_device *dev = data;

	if(count < 4 * sizeof(int))
		return 0;
	if(tmp[1] != 0x7fffffff || tmp[0] != 0x7fffffff)
		return 0;

	if(dev->curve) {
		kfree(dev->curve);
	}
	dev->curve = kzalloc(count, GFP_KERNEL);
	memcpy(dev->curve,buffer,count);

	return count;
}

static int proc_write_enable(struct file *file, const char *buffer, unsigned long count, void *data)
{
	struct jz47xx_battery_device *dev = data;
	if(!dev->enabled) {
		dev->enable(dev);
		dev->enabled = 1;
	}
	return count;
}

static int proc_read_param(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len = sizeof(struct battery_info);
	struct jz47xx_battery_device *dev = data;
	dev->info.update_time = rtc_read_reg(RTC_RTCSR);
	if(count < len)
		return -1;
	memcpy(page,&dev->info,len);
	return len;
}

static int proc_write_param(struct file *file, const char __user *buffer,
		unsigned long count, void *data)
{
	struct jz47xx_battery_device *dev = data;
	if(count != sizeof(struct battery_info))
		return -1;
	memcpy(&dev->info,buffer,sizeof(struct battery_info));
	return count;
}

static int jz47xx_read_battery(void)
{
	int i,retval = 0;

	for(i=0;i<5;i++) {
		jz_read_battery();
		msleep(10);
	}

	for(i=0;i<10;i++) {
		retval += jz_read_battery();
		msleep(10);
	}

	retval /= 10;
#if defined(CONFIG_SOC_JZ4770)
	retval = retval * 1200 / 4096 * 4;
#endif
#if defined(CONFIG_SOC_JZ4760) || defined(CONFIG_SOC_JZ4760B)
	retval = retval * 2500 / 4096 * 4;
#endif
	return retval * 1000;
}

static void jz47xx_battery_enable(void *data)
{
	struct jz47xx_battery_device *dev = data;

	dev->state = dev->get_charging_state(dev->interface);

	if(rtc_read_reg(RTC_RTCSR) - dev->info.update_time > 60*60) {
		dev->info.capacity = calculate_capacity(dev,dev->read_battery(),
				dev->get_ad_online_state(dev->interface),
				dev->get_usb_online_state(dev->interface));
		printk("jz47xx_battery power reset [%d]\n",dev->info.capacity);
	} else {
		printk("jz47xx_battery use storage [%d]\n",dev->info.capacity);
	}

	power_supply_changed(&dev->battery);
	schedule_delayed_work(&dev->update_work, 1 * HZ);
	g_jz_battery_min_voltage = dev->info.min_vol;
}

static void resume_work_func(struct work_struct *work)
{
	int adv;
	int timecount;
	struct delayed_work *tmp = container_of(work,struct delayed_work,work);
	struct jz47xx_battery_device *dev = container_of(tmp,struct jz47xx_battery_device,resume_work);

	dev->resume_time = rtc_read_reg(RTC_RTCSR);
	timecount = dev->resume_time - dev->suspend_time;
	adv = dev->read_battery();

	printk("timecount = %d\n",timecount);
	if(dev->suspend_state == POWER_SUPPLY_STATUS_CHARGING 
			|| dev->suspend_state == POWER_SUPPLY_STATUS_FULL) {
		printk("suspend_ac = %d\n",dev->suspend_ac);
		printk("suspend_usb = %d\n",dev->suspend_usb);
		if(timecount > 1800) 
			timecount = timecount * 6 / 5;//external timecount
		timecount -= dev->next_scan_time;
		while(timecount > 0) {
			battery_calculate_rising(dev,adv,dev->suspend_ac,dev->suspend_usb);
			timecount -= dev->next_scan_time;
		}
	} else if(timecount > 15*60) {
		dev->info.capacity = calculate_capacity(dev,dev->read_battery(),
				dev->get_ad_online_state(dev->interface),
				dev->get_usb_online_state(dev->interface));
		printk("reset battery capacity = %d\n",dev->info.capacity);
	}

	power_supply_changed(&dev->battery);
}

#ifdef CONFIG_PM
static int jz47xx_battery_suspend(struct platform_device *device, pm_message_t state)
{
	struct jz47xx_battery_device *dev = platform_get_drvdata(device);
	dev->suspend_ac = dev->get_ad_online_state(dev->interface);
	dev->suspend_usb = dev->get_usb_online_state(dev->interface);
	dev->suspend_state = dev->get_charging_state(dev->interface);
	dev->suspend_time = rtc_read_reg(RTC_RTCSR);
	return 0;
}

static int jz47xx_battery_resume(struct platform_device *device)
{
	struct jz47xx_battery_device *dev = platform_get_drvdata(device);
	wake_lock_timeout(&dev->wake_lock,5*HZ);

	cancel_delayed_work(&dev->resume_work);
	schedule_delayed_work(&dev->resume_work,HZ);
	return 0;
}
#else
#define jz47xx_battery_suspend NULL
#define jz47xx_battery_resume NULL
#endif

static int __devinit jz47xx_battery_probe(struct platform_device *device)
{
	int retval = 0;
	struct proc_dir_entry *root;
	struct proc_dir_entry *res;

	struct jz47xx_battery_platform_data *pdata;
	struct jz47xx_battery_device *dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		retval = -ENOMEM;
		goto dev_alloc_failed;
	}

	pdata = device->dev.platform_data;
	if(pdata == NULL) {
		goto null_platform_data;
	}

	dev->read_battery = jz47xx_read_battery;
	dev->enable = jz47xx_battery_enable;

	memcpy(&dev->info,pdata->info,sizeof(struct battery_info));

	dev->info.capacity = -1;
	dev->wait_stfull_timeout = WAIT_STFULL_TIMES;

	dev->dc_charg_time = dev->info.battery_mah / dev->info.dc_charg_ma * 36;
	dev->usb_charg_time = dev->info.battery_mah / dev->info.usb_charg_ma * 36;

	if(dev->dc_charg_time < 30) dev->dc_charg_time = 30;
	if(dev->usb_charg_time < 30) dev->usb_charg_time = 30;

	dev->interface = jz47xx_battery_register_interface(pdata->interface_pdata,
			charg_changed_callback,dev);

	wake_lock_init(&dev->wake_lock,WAKE_LOCK_SUSPEND, "jz47xx_battery");
	INIT_DELAYED_WORK(&dev->update_work, update_work_func);
	INIT_DELAYED_WORK(&dev->resume_work, resume_work_func);

	platform_set_drvdata(device, dev);

	dev->ac.name = "Mains";
	dev->ac.type = POWER_SUPPLY_TYPE_MAINS;
	dev->ac.properties = power_props;
	dev->ac.num_properties = ARRAY_SIZE(power_props);
	dev->ac.get_property = get_power_prop;

	dev->usb.name = "USB";
	dev->usb.type = POWER_SUPPLY_TYPE_USB;
	dev->usb.properties = power_props;
	dev->usb.num_properties = ARRAY_SIZE(power_props);
	dev->usb.get_property = get_power_prop;

	dev->battery.name		= "Battery";
	dev->battery.type		= POWER_SUPPLY_TYPE_BATTERY;
	dev->battery.properties		= battery_props;
	dev->battery.num_properties	= ARRAY_SIZE(battery_props);
	dev->battery.get_property	= battery_get_property;
	dev->battery.use_for_apm	= 1;

#if CONFIG_ACTBAT_DC
	retval += power_supply_register(&device->dev, &dev->ac);
#endif
#if CONFIG_ACTBAT_USB
	retval += power_supply_register(&device->dev, &dev->usb);
#endif
	retval += power_supply_register(&device->dev, &dev->battery);

	root = proc_mkdir("jz47xx_battery", 0);

	res = create_proc_entry("info", 0644, root);
	if (res) {
		res->read_proc = proc_read_info;
		res->data = dev;
	}

	res = create_proc_entry("curve", 0644, root);
	if (res) {
		res->write_proc = proc_write_curve;
		res->data = dev;
	}

	res = create_proc_entry("enable", 0644, root);
	if (res) {
		res->write_proc = proc_write_enable;
		res->data = dev;
	}

	res = create_proc_entry("param", 0644, root);
	if (res) {
		res->read_proc = proc_read_param;
		res->write_proc = proc_write_param;
		res->data = dev;
	}

null_platform_data:
dev_alloc_failed:
	return retval;
}

static int __devexit jz47xx_battery_remove(struct platform_device *dev)
{
	return 0;
}

static struct platform_driver jz47xx_battery_driver = {
	.driver.name	= "jz47xx-battery",
	.driver.owner	= THIS_MODULE,
	.probe		= jz47xx_battery_probe,
	.remove		= jz47xx_battery_remove,
	.suspend	= jz47xx_battery_suspend,
	.resume		= jz47xx_battery_resume,
};


static int __init jz47xx_battery_init(void)
{
	return platform_driver_register(&jz47xx_battery_driver);
}

static void __exit jz47xx_battery_exit(void)
{
	platform_driver_unregister(&jz47xx_battery_driver);
}

late_initcall(jz47xx_battery_init);
module_exit(jz47xx_battery_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("ztyan<ztyan@ingenic.cn>");
MODULE_DESCRIPTION("SOC Jz47xx battery driver V2");
