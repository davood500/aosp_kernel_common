/*
 *  drivers/switch/switch_gpio.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
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
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
//#include <base/base.h>

#include <asm/jzsoc.h>
#define gpio_to_irq(n)		(IRQ_GPIO_0 + n);
int gstate_dock;
extern int gstate_hp;
extern struct workqueue_struct *hp_and_dock_detect_work_queue;

struct gpio_switch_data {
	struct switch_dev sdev;
	unsigned gpio;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	int irq;
	struct work_struct work;
	int valid_level;
	/* add board realted handler */
//	void *(state_chage_handler)(gpio_switch_data *, int );
};

static void switch_set_state_class(struct gpio_switch_data *dev, int state)
{
	switch_set_state(&dev->sdev, state);
	gstate_dock = state;
#ifdef CONFIG_JZ4760_TABLET_8II
	if (state == 1) {
			printk("tablet-8II dock open!\n");
			__gpio_as_output(GPIO_USB_VCC_EN);
			__gpio_set_pin(GPIO_USB_VCC_EN);			
		} else {
			printk("tablet-8II dock close!\n");
			__gpio_as_output(GPIO_USB_VCC_EN);
			__gpio_clear_pin(GPIO_USB_VCC_EN);
		}
#endif
#if defined(CONFIG_JZ4760_PT701) || defined(CONFIG_JZ4760_PT701I)
	/* if headphone not attached */
	if (!gstate_hp) {
		if (state == 1) {
			printk("pt701 audio select external speaker!");
			/* disable internal speaker output */
			__gpio_as_output(GPIO_SPK_EN);
			__gpio_clear_pin(GPIO_SPK_EN);
			/* enable external speaker output */
			__gpio_as_output(GPIO_DOCK_SPK_EN);
			__gpio_set_pin(GPIO_DOCK_SPK_EN);
		} else {
			printk("pt701 audio select internal speaker!");
			/* enable internal speaker output */
			__gpio_as_output(GPIO_SPK_EN);
			__gpio_set_pin(GPIO_SPK_EN);
			/* disable external speaker output */
			__gpio_as_output(GPIO_DOCK_SPK_EN);
			__gpio_clear_pin(GPIO_DOCK_SPK_EN);
		}
	}
#endif
}

static void gpio_switch_work(struct work_struct *work)
{
	int state, tmp_state, i;
	struct gpio_switch_data	*data =
		container_of(work, struct gpio_switch_data, work);

	msleep(100);

	/* Anti-shock ... */
	__gpio_enable_pull(data->gpio);
	state = __gpio_get_pin(data->gpio) == 1 ? 0 : 1;

	for (i = 0; i < 5; i++) {
		msleep(50);
		tmp_state = __gpio_get_pin(data->gpio) == 1 ? 0 : 1;
		if (tmp_state != state) {
			i = -1;
			state = __gpio_get_pin(data->gpio) == 1 ? 0 : 1;
			continue;
		}
	}

	/* BUG here! */
	/* I will fix it later */
	if (state == 1) {
		__gpio_as_irq_high_level(data->gpio);
	} else {
		__gpio_as_irq_low_level(data->gpio);
	}

	switch_set_state_class(data, state);
}

static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	struct gpio_switch_data *switch_data =
	    (struct gpio_switch_data *)dev_id;
	__gpio_as_input(switch_data->gpio);

	//schedule_work(&switch_data->work);
	queue_work(hp_and_dock_detect_work_queue, &switch_data->work);
	return IRQ_HANDLED;
}

static ssize_t switch_gpio_print_state(struct switch_dev *sdev, char *buf)
{
	struct gpio_switch_data	*switch_data =
		container_of(sdev, struct gpio_switch_data, sdev);
	const char *state;
	if (switch_get_state(sdev))
		state = switch_data->state_on;
	else
		state = switch_data->state_off;

	if (state)
		return sprintf(buf, "%s\n", state);
	return -1;
}

static int gpio_switch_probe(struct platform_device *pdev)
{
	struct gpio_switch_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_switch_data *switch_data;
	int ret = 0;

	if (!pdata)
		return -EBUSY;

	switch_data = kzalloc(sizeof(struct gpio_switch_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;

	switch_data->sdev.name = pdata->name;
	switch_data->gpio = pdata->gpio;
	switch_data->name_on = pdata->name_on;
	switch_data->name_off = pdata->name_off;
	switch_data->state_on = pdata->state_on;
	switch_data->state_off = pdata->state_off;
	switch_data->sdev.print_state = switch_gpio_print_state;

	dev_set_drvdata(&pdev->dev, (void *)switch_data);

	ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	INIT_WORK(&switch_data->work, gpio_switch_work);

	switch_data->irq = gpio_to_irq(switch_data->gpio);

	if (switch_data->irq < 0) {
		ret = switch_data->irq;
		goto err_detect_irq_num_failed;
	}

	ret = request_irq(switch_data->irq, gpio_irq_handler,
			  IRQF_DISABLED, pdev->name, switch_data);
	if (ret < 0)
		goto err_request_irq;

	__gpio_enable_pull(switch_data->gpio);
	ret = __gpio_get_pin(switch_data->gpio) == 1 ? 0 : 1;

	/* BUG here */
	/* I will fix it later */
	if (ret == 1) {
		__gpio_as_irq_high_level(switch_data->gpio);
	} else {
		__gpio_as_irq_low_level(switch_data->gpio);
	}

	/* Perform initial detection */
	switch_set_state_class(switch_data, ret);

	return 0;

err_request_irq:
err_detect_irq_num_failed:
	gpio_free(switch_data->gpio);

err_switch_dev_register:
	switch_dev_unregister(&switch_data->sdev);
	kfree(switch_data);

	return ret;
}

static int switch_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int switch_resume(struct platform_device *pdev)
{
	struct gpio_switch_data *switch_data = platform_get_drvdata(pdev);
	int ret;
	if (switch_data) {
		/* restore GPIO setting after suspend */
		__gpio_enable_pull(switch_data->gpio);
		ret = __gpio_get_pin(switch_data->gpio) == 1 ? 0 : 1;

		/* BUG here */
		/* I will fix it later */
		if (ret == 1) {
			__gpio_as_irq_high_level(switch_data->gpio);
		} else {
			__gpio_as_irq_low_level(switch_data->gpio);
		}

		/* Perform initial detection */
		switch_set_state(&switch_data->sdev, ret);
		gstate_dock = ret;
		
		//switch_set_state_class(switch_data, ret);
	}

	return 0;
}

static int __devexit gpio_switch_remove(struct platform_device *pdev)
{
	struct gpio_switch_data *switch_data = platform_get_drvdata(pdev);

	if (switch_data) {
		cancel_work_sync(&switch_data->work);
		gpio_free(switch_data->gpio);
		switch_dev_unregister(&switch_data->sdev);
		kfree(switch_data);
	}

	return 0;
}

static struct platform_driver gpio_switch_driver = {
	.probe		= gpio_switch_probe,
	.remove		= __devexit_p(gpio_switch_remove),
	.driver		= {
		.name	= "switch-dock",
		.owner	= THIS_MODULE,
	},
	.suspend        = switch_suspend,
	.resume         = switch_resume,
};

static int __init gpio_switch_init(void)
{
	return platform_driver_register(&gpio_switch_driver);
}

static void __exit gpio_switch_exit(void)
{
	platform_driver_unregister(&gpio_switch_driver);
}

module_init(gpio_switch_init);
module_exit(gpio_switch_exit);

MODULE_AUTHOR("Mike Lockwood <lockwood@android.com>");
MODULE_DESCRIPTION("GPIO Switch driver");
MODULE_LICENSE("GPL");
