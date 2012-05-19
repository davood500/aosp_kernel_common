/*
 * Copyright (c) 2006-2010  Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <asm/jzsoc.h>

#include <../sound/oss/xb47XX/jz47XX_codec.h>

/*=========================================================*/

#define DUMP_FUNC() //printk("DUMP:%s:%s\tline:%d\n", __FILE__, __func__, __LINE__)

/*=========================================================*/
#define SYS_CLK			SYS_CLK_12M /* 12MHZ sys_clk */
#define DMIC_CLK		DMIC_CLK_OFF /* dmic clock off */
#define REPLAY_VOLUME_BASE	+0  /* 0dB, the volume must be in -25 ~ +6 by step 1dB*/
#define RECORD_VOLUME_BASE	+20  /* 0dB, the volume must be +0,+4,+8,+12,+16,+20 */
#define RECORD_DIGITAL_VOLUME_BASE      10 /*val:  0 ~ +43 (dB)*/
#define REPLAY_DIGITAL_VOLUME_BASE      0  /*val:  -31 ~ 0 (dB) */
/*=========================================================*/

void audio_enable_speaker(void)
{
	__gpio_as_output(GPIO_SPEAKER_EN);
	__gpio_set_pin(GPIO_SPEAKER_EN);
}

void audio_disable_speaker(void)
{
	__gpio_as_output(GPIO_SPEAKER_EN);
	__gpio_clear_pin(GPIO_SPEAKER_EN);
}

void audio_enable_hp_mute(void)
{
	__gpio_as_output(GPIO_HP_MUTE);
	__gpio_set_pin(GPIO_HP_MUTE);
}

void audio_disable_hp_mute(void)
{
	__gpio_as_output(GPIO_HP_MUTE);
	__gpio_clear_pin(GPIO_HP_MUTE);		
}

static int o1_dlv_set_device(struct snd_device_config *snd_dev_cfg)
{
	int ret;
	
	DUMP_FUNC();

	switch (snd_dev_cfg->device) {
	case SND_DEVICE_HEADSET:
		ret = dlv_set_route(REPLAY_HP_STEREO_WITH_CAP);
		if(ret != REPLAY_HP_STEREO_WITH_CAP)
		{
			printk("JZ CODEC: set device SND_DEVICE_HEADSET error!\n");
			return -1;
		}
		break;

	case SND_DEVICE_SPEAKER:
		ret = dlv_set_route(REPLAY_LINEOUT);
		if(ret != REPLAY_LINEOUT)
		{
			printk("JZ CODEC: set device SND_DEVICE_SPEAKER error!\n");
			return -1;
		}
		break;

	case SND_DEVICE_HEADSET_AND_SPEAKER:
		ret = dlv_set_route(REPLAY_HP_STEREO_WITH_CAP_AND_LINEOUT);
		if(ret != REPLAY_HP_STEREO_WITH_CAP_AND_LINEOUT)
		{
			printk("JZ CODEC: set device SND_DEVICE_HEADSET_AND_SPEAKER error!\n");
			return -1;
		}
		break;

	default:
		printk("%s: device not under control in SND_SET_DEVICE\n", __func__);
	};

	return 0;
}

static int o1_dlv_set_gpio_before_set_route(int route)
{
	DUMP_FUNC();

	switch(route){

	case ROUTE_ALL_CLEAR:
	case REPLAY_HP_STEREO_WITH_CAP:
	case REPLAY_LINEOUT:
	case REPLAY_HP_STEREO_WITH_CAP_AND_LINEOUT:
		/*hp mute*/
		audio_enable_hp_mute();
		/* disable speaker output gpio */
		audio_disable_speaker();
		break;

	default:
		printk("dlv set route gpio error!, undecleard route\n");
	}

	return 0;
}

static int o1_dlv_set_gpio_after_set_route(int route)
{
	DUMP_FUNC();

	switch(route){

	case ROUTE_ALL_CLEAR:
		break;

	case REPLAY_HP_STEREO_WITH_CAP:
		dlv_sleep(10);
		/*disable hp mute*/
		audio_disable_hp_mute();
		break;
		
	case REPLAY_LINEOUT:
		dlv_sleep(10);
		/* enable speaker output */
		audio_enable_speaker();
		break;

	case REPLAY_HP_STEREO_WITH_CAP_AND_LINEOUT:
		dlv_sleep(10);
		/*disable hp mute*/
		audio_disable_hp_mute();
		/* enable speaker output */
		audio_enable_speaker();
		break;

	default:
		printk("dlv set route gpio error!, undecleard route\n");
	}

	return 0;
}

static int o1_dlv_turn_off_part(int mode)
{
	DUMP_FUNC();

	return -1;
}

static int o1_dlv_shutdown_part(void)
{
	DUMP_FUNC();
	
	/*hp mute*/
	audio_enable_hp_mute();
	/* disable speaker output */
	audio_disable_speaker();
	
	return 0;
}

static int o1_dlv_suspend_part(void)
{
	DUMP_FUNC();

	return -1;
}

static int o1_dlv_resume_part(void)
{
	DUMP_FUNC();

	return -1;
} 

static int o1_dlv_anti_pop_part(void)
{
	DUMP_FUNC();

	/*hp mute*/
	audio_enable_hp_mute();
	/* disable internal speaker output */
	audio_disable_speaker();

	return -1;
}

/*======================================================================*/

static jz_dlv_platform_data_t jz_dlv_platform_data = {
	.dlv_sys_clk = SYS_CLK,
	.dlv_dmic_clk = DMIC_CLK,
	.dlv_replay_volume_base = REPLAY_VOLUME_BASE,
	.dlv_record_volume_base = RECORD_VOLUME_BASE,
	.dlv_record_digital_volume_base = RECORD_DIGITAL_VOLUME_BASE,
	.dlv_replay_digital_volume_base = REPLAY_DIGITAL_VOLUME_BASE,

	.default_replay_route = REPLAY_HP_STEREO_WITH_CAP,
	.default_record_route = RECORD_MIC1_MONO_DIFF_WITH_BIAS,

	.dlv_set_device = o1_dlv_set_device,
        .dlv_set_gpio_before_set_route = o1_dlv_set_gpio_before_set_route,
	.dlv_set_gpio_after_set_route = o1_dlv_set_gpio_after_set_route,
	.dlv_turn_off_part = o1_dlv_turn_off_part,
	.dlv_shutdown_part = o1_dlv_shutdown_part,
	.dlv_suspend_part = o1_dlv_suspend_part,
	.dlv_resume_part = o1_dlv_resume_part,
	.dlv_anti_pop_part = o1_dlv_anti_pop_part,
};

static struct platform_device jz_dlv_device = {
	.name		= "jz_dlv",
	.id		= -1,
	.dev		= {
		.platform_data = &jz_dlv_platform_data,
	},
};

/*---------------------*/

static int __init o1_dlv_board_init(void)
{
	int ret = 0;

	ret = platform_device_register(&jz_dlv_device);

	return ret;
}

/*---------------------*/

arch_initcall(o1_dlv_board_init);
