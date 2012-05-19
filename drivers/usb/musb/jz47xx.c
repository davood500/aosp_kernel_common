/*
 * Copyright (C) 2005-2006 by Texas Instruments
 *
 * This file is part of the Inventra Controller Driver for Linux.
 *
 * The Inventra Controller Driver for Linux is free software; you
 * can redistribute it and/or modify it under the terms of the GNU
 * General Public License version 2 as published by the Free Software
 * Foundation.
 *
 * The Inventra Controller Driver for Linux is distributed in
 * the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with The Inventra Controller Driver for Linux ; if not,
 * write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include <asm/jzsoc.h>
#include "musb_core.h"
#include "jz47xx.h"

/* button debouncer, how many 10ms to debounce, 100ms is usually plenty
 * can't be used in interrupt context
 */
#define GPIO_DEBOUNCE 10

static unsigned int read_gpio_pin(unsigned int pin)
{
	unsigned int t, v;
	unsigned int i;

	i = GPIO_DEBOUNCE;

	v = t = 0;

	while (i--) {
		t = __gpio_get_pin(pin);
		if (v != t)
			i = GPIO_DEBOUNCE;

		v = t;
		msleep(10);
	}

	return v;
}

static inline void jz_musb_phy_reset(void)
{
	REG_CPM_USBPCR |= USBPCR_POR;
	udelay(30);
	REG_CPM_USBPCR &= ~USBPCR_POR;

	udelay(300);
}

static inline void jz_musb_set_device_only_mode(void)
{
	printk(KERN_INFO "jz47xx: Device only mode.\n");

	/* Device Mode. */
	REG_CPM_USBPCR &= ~(1 << 31);

	REG_CPM_USBPCR |= USBPCR_VBUSVLDEXT;
}

static inline void jz_musb_set_normal_mode(void)
{
	printk(KERN_INFO "jz47xx: Normal mode.\n");

	__gpio_as_otg_drvvbus();

	/* OTG Mode. */
	REG_CPM_USBPCR |= (1 << 31); //work as OTG

	REG_CPM_USBPCR &= ~((1 << 24) | (1 << 23) | (1 << 20));

	REG_CPM_USBPCR |= ((1 << 28) | (1 << 29)); //idpullup always active
}

static inline void jz_musb_init(struct musb *musb)
{

	cpm_set_clock(CGU_OTGCLK, 12 * 1000 * 1000);
	udelay(100);

	/* fil */
	REG_CPM_USBVBFIL = 0x80;

	/* rdt */
	REG_CPM_USBRDT = (600 * (cpm_get_clock(CGU_CCLK) / 1000000)) / 1000;
	//at last 600 us 
	//The hub will continue to send chirp pairs up until 100 - 500 us before the end of reset
	//and the K-J chirp needs 100 us

	/* rdt - filload_en */
	REG_CPM_USBRDT |= (1 << 25);

	/* TXRISETUNE & TXVREFTUNE. */
	REG_CPM_USBPCR &= ~0x3f;
	REG_CPM_USBPCR |= 0x35;

	/* enable tx pre-emphasis */
	REG_CPM_USBPCR |= 0x40;

	if (is_host_enabled(musb))
		jz_musb_set_normal_mode();
	else
		jz_musb_set_device_only_mode();

	jz_musb_phy_reset();
}

static irqreturn_t jz47xx_vdetect_interrupt(int irq, void *dev_id);

static void jz47xx_musb_enable(struct musb *musb)
{
	struct jz47xx_musb_glue *glue = musb_to_glue(musb);
	static int virgen = 1;
	
	printk(KERN_INFO "jz47xx: Enable USB PHY.\n");

	/* enable OTG phy */
	SETREG32(CPM_OPCR, OPCR_OTGPHY_ENABLE);

	/* Wait PHY Clock Stable. */
	udelay(300);

	/* invoke when system up */
	if (virgen) {
		virgen = 0;
		jz47xx_vdetect_interrupt(glue->gpio_vdetect, glue);
	}
}

static void jz47xx_musb_disable(struct musb *musb)
{
	printk(KERN_INFO "jz47xx: Disable USB PHY.\n");

	/* if remove the delay, few board can't role as B Device
	 * after role as A Host, refer redmine 2603.
	 */
	mdelay(400);
	/* disable OTG phy */
	CLRREG32(CPM_OPCR, OPCR_OTGPHY_ENABLE);
}

static void jz47xx_musb_set_vbus(struct musb *musb, int is_on)
{
	struct jz47xx_musb_glue *glue = musb_to_glue(musb);
	if(glue->set_vbus)
		glue->set_vbus(is_on);
}


static int jz47xx_musb_set_mode(struct musb *musb, u8 mode)
{
	u8 devctl;
	unsigned long flags;
	spin_lock_irqsave(&musb->lock, flags);

	devctl = musb_readb(musb->mregs, MUSB_DEVCTL);

	if (mode == MUSB_HOST) {
		musb->xceiv->default_a = 1;
		musb->xceiv->state = OTG_STATE_A_WAIT_VRISE;

		devctl |= MUSB_DEVCTL_SESSION;
		musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);
		/* turn on vbus as soon as possible after set session */
		jz47xx_musb_set_vbus(musb, 1);

		musb->is_active = 1;
		MUSB_HST_MODE(musb);
		printk("USB AS HOST MODE\n");
	} else if(mode == MUSB_PERIPHERAL){
		musb->is_active = 0;

		musb->xceiv->default_a = 0;
		musb->xceiv->state = OTG_STATE_B_IDLE;
		devctl &= ~MUSB_DEVCTL_SESSION;
		musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);
		jz47xx_musb_set_vbus(musb, 0);

		MUSB_DEV_MODE(musb);
		printk("USB AS PERIPHERAL MODE\n");
	} else {
		musb->is_active = 0;
		return -EIO;
	}

	spin_unlock_irqrestore(&musb->lock, flags);
	return 0;
}

static void jz47xx_musb_vsense_func(struct work_struct *this_work)
{
	struct jz47xx_musb_glue *glue =
				vsense_to_glue(to_delayed_work(this_work));
	u8 devctl = musb_readb(glue->musb->mregs, MUSB_DEVCTL);

	static int old_state = 0;
	static int wait_times = 0;

	if (glue->musb->xceiv->state != old_state) {
		/* printk("\e[34m\e[1m%s-->%s\e[0m\n", otg_state_string(old_state), */
		/* 	otg_state_string(glue->musb->xceiv->state)); */
		old_state = glue->musb->xceiv->state;
		wait_times = 0;
	}

	wait_times++;
	if ((glue->musb->xceiv->state == OTG_STATE_A_WAIT_VRISE) ||
	    (glue->musb->xceiv->state == OTG_STATE_A_HOST))
	{
		
		if (((devctl & 0x18) < 0x18) && (wait_times > 70)) {
			printk("redmine2517, devctl=0x%x, state=%s\n",
			       musb_readb(glue->musb->mregs, MUSB_DEVCTL),
			       otg_state_string(glue->musb->xceiv->state));
			jz47xx_musb_set_mode(glue->musb, MUSB_PERIPHERAL);
			msleep(400);
			jz47xx_musb_set_mode(glue->musb, MUSB_HOST);
			msleep(20);
			wait_times = 0;
		}
		
	}
	queue_delayed_work(glue->vsense_queue, &glue->vsense_work,
			   msecs_to_jiffies(100));
}

static void jz47xx_musb_vstate_func(struct work_struct *this_work)
{
	struct jz47xx_musb_glue *glue =
				vstate_to_glue(to_delayed_work(this_work));

	if (glue->vstate_irq_flag) {
		glue->vstate_irq_flag = 0;
		if (read_gpio_pin(glue->gpio_vdetect)) {
			glue->vstate |= (VSTATE_DEV_ENABLE | VSTATE_POWER);
			glue->vstate &= ~VSTATE_DEV_DISABLE;
			__gpio_as_irq_fall_edge(glue->gpio_vdetect);
		} else {
			glue->vstate |= VSTATE_DEV_DISABLE;
			glue->vstate &= ~(VSTATE_DEV_ENABLE | VSTATE_POWER);
			__gpio_as_irq_rise_edge(glue->gpio_vdetect);
		}
	}

	if(glue->vstate & VSTATE_DEV_ENABLE) {
		glue->vstate &= ~(VSTATE_DEV_ENABLE);
		jz47xx_musb_enable(glue->musb);
		cancel_delayed_work(&glue->vstate_work);
		queue_delayed_work(glue->vstate_queue, &glue->vstate_work,
				   msecs_to_jiffies(1000));
		return;
	} 

	if(glue->vstate & VSTATE_DEV_DISABLE) {
		glue->vstate = 0;
		jz47xx_musb_disable(glue->musb);
		if(is_peripheral_active(glue->musb))
			musb_g_disconnect(glue->musb);
		if(glue->usb_disconnect_callback)
			glue->usb_disconnect_callback();
	}

	if(is_host_active(glue->musb))
		return;

	if(glue->vstate & VSTATE_CONNECT) {
		if(glue->usb_connect_callback)
			glue->usb_connect_callback();
	} else if(glue->vstate & VSTATE_POWER) {
		if(glue->usb_power_callback)
			glue->usb_power_callback();
	}
}

static irqreturn_t jz47xx_vdetect_interrupt(int irq, void *dev_id)
{
	struct jz47xx_musb_glue *glue = dev_id;

	glue->vstate_irq_flag = 1;
	cancel_delayed_work(&glue->vstate_work);
	queue_delayed_work(glue->vstate_queue, &glue->vstate_work,
			   msecs_to_jiffies(10));
	return IRQ_HANDLED;
}

#if defined(CONFIG_USB_OTG)
static void jz47xx_musb_id_func(struct work_struct *this_work)
{
	struct jz47xx_musb_glue *glue =
				otgid_to_glue(to_delayed_work(this_work));

	if(read_gpio_pin(glue->gpio_otg_id)) {
		jz47xx_musb_set_mode(glue->musb, MUSB_PERIPHERAL);
		__gpio_as_irq_fall_edge(glue->gpio_otg_id);
		__gpio_enable_pull(glue->gpio_otg_id);
	} else {
		jz47xx_musb_set_mode(glue->musb, MUSB_HOST);
		__gpio_as_irq_rise_edge(glue->gpio_otg_id);
		__gpio_enable_pull(glue->gpio_otg_id);
	}
}

static irqreturn_t jz47xx_otgid_interrupt(int irq, void *dev_id)
{
	struct jz47xx_musb_glue *glue = dev_id;
	cancel_delayed_work(&glue->otgid_work);
	queue_delayed_work(glue->otgid_queue, &glue->otgid_work,
			   msecs_to_jiffies(200));
	return IRQ_HANDLED;
}
#endif

static irqreturn_t jz47xx_musb_interrupt(int irq, void *__hci)
{
	u8 devctl;
	unsigned long flags;
	irqreturn_t retval = IRQ_NONE;
	struct musb *musb = __hci;
	struct jz47xx_musb_glue *glue = musb_to_glue(musb);

	if(glue->irq_dma_handler)
		glue->irq_dma_handler(irq,musb->dma_controller);

	spin_lock_irqsave(&musb->lock, flags);

	musb->int_usb = musb_readb(musb->mregs, MUSB_INTRUSB);
	musb->int_tx = musb_readw(musb->mregs, MUSB_INTRTX);
	musb->int_rx = musb_readw(musb->mregs, MUSB_INTRRX);
	devctl = musb_readb(musb->mregs, MUSB_DEVCTL);

	if (musb->int_usb || musb->int_tx || musb->int_rx)
		retval = musb_interrupt(musb);

	if(!(devctl & MUSB_DEVCTL_HM)) {
		if((musb->int_usb & MUSB_INTR_SOF) && !(glue->vstate & VSTATE_CONNECT)) {
			glue->vstate |= VSTATE_CONNECT;
			glue->vstate &= ~VSTATE_DISCONNECT;
			cancel_delayed_work(&glue->vstate_work);
			queue_delayed_work(glue->vstate_queue, &glue->vstate_work,
					   msecs_to_jiffies(10));
		} else if(musb->int_usb & MUSB_INTR_DISCONNECT) {
			glue->vstate |= VSTATE_DISCONNECT;
			glue->vstate &= ~VSTATE_CONNECT;
			cancel_delayed_work(&glue->vstate_work);
			queue_delayed_work(glue->vstate_queue, &glue->vstate_work,
					   msecs_to_jiffies(10));
		}
	}

	spin_unlock_irqrestore(&musb->lock, flags);

	return retval;
}

static int jz47xx_musb_init(struct musb *musb)
{
	struct jz47xx_musb_glue *glue = musb_to_glue(musb);
	glue->musb = musb;
	usb_nop_xceiv_register();
	musb->xceiv = otg_get_transceiver();
	if (!musb->xceiv) {
		printk("HS USB OTG: no transceiver configured\n");
		return -ENODEV;
	}

	jz_musb_init(musb);

#if defined(CONFIG_USB_OTG)
	if(read_gpio_pin(glue->gpio_otg_id)) {
		jz47xx_musb_set_mode(musb, MUSB_PERIPHERAL);
		__gpio_as_irq_fall_edge(glue->gpio_otg_id);
		__gpio_enable_pull(glue->gpio_otg_id);
	} else {
		jz47xx_musb_set_mode(musb, MUSB_HOST);
		__gpio_as_irq_rise_edge(glue->gpio_otg_id);
		__gpio_enable_pull(glue->gpio_otg_id);
	}
#else
	jz47xx_musb_set_mode(musb, MUSB_PERIPHERAL);
#endif

	if(read_gpio_pin(glue->gpio_vdetect))
		__gpio_as_irq_fall_edge(glue->gpio_vdetect);
	else
		__gpio_as_irq_rise_edge(glue->gpio_vdetect);

	musb->isr = jz47xx_musb_interrupt;

	return 0;
}

static int jz47xx_musb_exit(struct musb *musb)
{
	jz47xx_musb_disable(musb);
	return 0;
}

static const struct musb_platform_ops jz47xx_musb_ops = {
	.init		= jz47xx_musb_init,
	.exit		= jz47xx_musb_exit,

	.enable		= jz47xx_musb_enable,
	.disable	= jz47xx_musb_disable,

	.set_mode	= jz47xx_musb_set_mode,

	.set_vbus	= jz47xx_musb_set_vbus,
};

static u64 jz47xx_musb_dmamask = DMA_BIT_MASK(32);

static int __init jz47xx_musb_probe(struct platform_device *pdev)
{
	struct platform_device			*musb_device;
	struct jz47xx_musb_glue			*glue;
	struct jz47xx_musb_platform_data	*pdata = pdev->dev.platform_data;

	int ret = -ENOMEM;
	int id_irq,vdetect_irq;

	glue = kzalloc(sizeof(*glue), GFP_KERNEL);
	if (!glue) {
		printk("failed to allocate glue context\n");
		goto err0;
	}

	musb_device = platform_device_alloc("musb-hdrc", -1);
	if (!musb_device) {
		printk("failed to allocate musb_device\n");
		goto err1;
	}

	musb_device->dev.parent			= &pdev->dev;
	musb_device->dev.dma_mask		= &jz47xx_musb_dmamask;
	musb_device->dev.coherent_dma_mask	= jz47xx_musb_dmamask;

#if defined(CONFIG_USB_OTG)
	id_irq = platform_get_irq_byname(pdev, "id");
	ret = request_irq(id_irq, jz47xx_otgid_interrupt, IRQF_DISABLED, "jz47xx_musb_id", (void *)glue);
	if (ret) {
		printk("failed to request jz47xx_musb_id interrupt\n");
		goto err2;
	}

	enable_irq_wake(id_irq);//FIX LATER:no need id wakeup for otg sleep
	glue->gpio_otg_id 		= id_irq - IRQ_GPIO_0;
#endif
	vdetect_irq = platform_get_irq_byname(pdev, "vbus_detect");
	ret = request_irq(vdetect_irq, jz47xx_vdetect_interrupt, IRQF_DISABLED, "jz47xx_musb_vdetect", (void *)glue);
	if (ret) {
		printk("failed to request jz47xx_musb_vdetect interrupt\n");
		goto err3;
	}

	glue->dev			= &pdev->dev;
	glue->musb_device		= musb_device;
	glue->gpio_vdetect 		= vdetect_irq - IRQ_GPIO_0;
	glue->usb_power_callback	= pdata->usb_power_callback;
	glue->usb_connect_callback	= pdata->usb_connect_callback;
	glue->usb_disconnect_callback	= pdata->usb_disconnect_callback;
	glue->set_vbus			= pdata->set_vbus;

#if defined(CONFIG_USB_OTG)
	INIT_DELAYED_WORK(&glue->otgid_work, jz47xx_musb_id_func);
	glue->otgid_queue = create_singlethread_workqueue("OTGID");
#endif
	INIT_DELAYED_WORK(&glue->vstate_work, jz47xx_musb_vstate_func);
	glue->vstate_queue = create_singlethread_workqueue("VSTATE");

	INIT_DELAYED_WORK(&glue->vsense_work, jz47xx_musb_vsense_func);
	glue->vsense_queue = create_singlethread_workqueue("VSENSE");
	queue_delayed_work(glue->vsense_queue, &glue->vsense_work,
			   msecs_to_jiffies(1000));

	pdata->musb_pdata->platform_ops	= &jz47xx_musb_ops;

	platform_set_drvdata(pdev, glue);

	ret = platform_device_add_resources(musb_device, pdev->resource,
			pdev->num_resources);
	if (ret) {
		printk("failed to add resources\n");
		goto err4;
	}

	ret = platform_device_add_data(musb_device, pdata->musb_pdata, sizeof(*pdata->musb_pdata));
	if (ret) {
		printk("failed to add platform_data\n");
		goto err4;
	}

	ret = platform_device_add(musb_device);
	if (ret) {
		printk("failed to register musb device\n");
		goto err4;
	}

	return 0;

err4:
	free_irq(vdetect_irq, "jz47xx_musb_vdetect");
err3:
#if defined(CONFIG_USB_OTG)
	free_irq(id_irq, "jz47xx_musb_id");
#endif
err2:
	kfree(glue);
err1:
	platform_device_put(musb_device);
err0:
	return ret;
}

static int __exit jz47xx_musb_remove(struct platform_device *pdev)
{
	struct jz47xx_musb_glue		*glue = platform_get_drvdata(pdev);

	platform_device_del(glue->musb_device);
	platform_device_put(glue->musb_device);
	kfree(glue);

	return 0;
}

static struct platform_driver jz47xx_musb_driver = {
	.remove		= __exit_p(jz47xx_musb_remove),
	.driver		= {
		.name	= "musb-jz47xx",
	},
};

MODULE_DESCRIPTION("Ingenic MUSB Glue Layer");
MODULE_AUTHOR("ztyan<ztyan@ingenic.com>");
MODULE_LICENSE("GPL v2");

static int __init jz47xx_init(void)
{
	return platform_driver_probe(&jz47xx_musb_driver, jz47xx_musb_probe);
}
subsys_initcall(jz47xx_init);

static void __exit jz47xx_exit(void)
{
	platform_driver_unregister(&jz47xx_musb_driver);
}
module_exit(jz47xx_exit);
