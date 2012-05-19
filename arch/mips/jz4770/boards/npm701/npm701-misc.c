/*
 * linux/arch/mips/jz4760/board-npm701.c
 *
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
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/wakelock.h>

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>

#include <linux/timed_gpio.h>
#include <linux/android_pmem.h>
#include <asm/jzsoc.h>
#include <asm/jzmmc/jz_mmc_platform_data.h>

#include <linux/gpio_keys.h>
#include <../drivers/input/keyboard/ha2605.h>
#include <linux/jz_cim_core.h>

#ifdef CONFIG_TOUCHSCREEN_JZ_GT801
#include <../drivers/input/touchscreen/jz_tsc_gt801.h>
#endif
#include <linux/act8600_power.h>
#include <linux/jz47xx_battery.h>
#include <../drivers/i2c/chips/jz_sensor_mma8452.h>
#include <../drivers/input/keyboard/jz_remote.h>

#include <../drivers/staging/android/timed_gpio.h>

#include <linux/jz_hdmi.h>
#define RESET  0
#define NORMAL 1

void __init board_msc_init(void);
extern int jz_add_msc_devices(unsigned int controller, struct jz_mmc_platform_data *plat);

extern struct semaphore detect_start_mutex;
extern struct semaphore detect_done_mutex;
extern int present;

static struct wake_lock wlan_power;

/* MSC SETUP */
static void npm701_inand_gpio_init(struct device *dev)
{
	__gpio_as_msc0_pa_4bit();
}

static void npm701_inand_power_on(struct device *dev)
{
}

static void npm701_inand_power_off(struct device *dev)
{
}

static void npm701_inand_cpm_start(struct device *dev)
{
	cpm_set_clock(CGU_MSC0CLK, 25 * 1000 * 1000);
	cpm_start_clock(CGM_MSC0);
}

static unsigned int npm701_inand_status(struct device *dev)
{
	unsigned int status;

	status = 1;
	return (status);
}

#define KBYTE  (1024)
#define MBYTE  ((KBYTE)*(KBYTE))
#define UINT32_MAX             (0xffffffffU)

struct mmc_partition_info npm701_partitions[] = {
	[0] = {"mbr",           0,       512, 0}, 	//0 - 512KB
	[1] = {"xboot",		0,     2*MBYTE, 0}, 	//0 - 2MB
	[2] = {"boot",      3*MBYTE,   8*MBYTE, 0}, 	//3MB - 8MB
	[3] = {"recovery", 12*MBYTE,   8*MBYTE, 0}, 	//12MB - 8MB
	[4] = {"misc",     21*MBYTE,   4*MBYTE, 0}, 	//21MB - 4MB
	[5] = {"battery",  26*MBYTE,   1*MBYTE, 0}, 	//26MB - 1MB
	[6] = {"cache",    28*MBYTE,  30*MBYTE, 1}, 	//28MB - 30MB
	[7] = {"device_id",59*MBYTE,   2*MBYTE, 0},	//59MB - 2MB
	[8] = {"system",   64*MBYTE, 256*MBYTE, 1}, 	//64MB - 256MB
	[9] = {"data",    321*MBYTE, 512*MBYTE, 1}, 	//321MB - 512MB
	[10]= {"test_0",         0, UINT32_MAX, 0},
};

static struct jz_mmc_platform_data npm701_inand_data = {
#ifndef CONFIG_MSC0_SDIO_SUPPORT
	.support_sdio   = 0,
#else
	.support_sdio   = 1,
#endif
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
	.init           = npm701_inand_gpio_init,
	.power_on       = npm701_inand_power_on,
	.power_off      = npm701_inand_power_off,
	.cpm_start      = npm701_inand_cpm_start,
	.status		= npm701_inand_status,
	.max_bus_width  = MMC_CAP_SD_HIGHSPEED | MMC_CAP_MMC_HIGHSPEED | MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_MSC0_BUS_1
	.bus_width      = 1,
#elif defined  CONFIG_MSC0_BUS_4
	.bus_width      = 4,
#elif defined  CONFIG_MSC0_BUS_8
	.bus_width      = 8,
#else
	.bus_width      = 4,
#endif
	.protect_boundary = 21*MBYTE,
	.partitions = npm701_partitions,
	.num_partitions = ARRAY_SIZE(npm701_partitions),
};

static void npm701_tf_gpio_init(struct device *dev)
{
	__gpio_as_msc2_pb_4bit();
	__gpio_as_output(GPIO_SD2_VCC_EN_N);
	__gpio_set_pin(GPIO_SD2_VCC_EN_N);            //always power up
}

static void npm701_tf_power_on(struct device *dev)
{
	__gpio_set_pin(GPIO_SD2_VCC_EN_N);
	//__msc2_enable_power();
}

static void npm701_tf_power_off(struct device *dev)
{
//	__gpio_clear_pin(GPIO_SD2_VCC_EN_N);
	//__msc2_disable_power();
}

static void npm701_tf_cpm_start(struct device *dev)
{
	cpm_set_clock(CGU_MSC2CLK, 25 * 1000 * 1000);
	cpm_start_clock(CGM_MSC2);
}

static unsigned int npm701_tf_status(struct device *dev)
{
	unsigned int status;

	status = (unsigned int) !__gpio_get_pin(GPIO_SD2_CD_N);
	return (status);
}

static void npm701_tf_plug_change(int state)
{
	if(state == CARD_INSERTED)
		__gpio_as_irq_high_level(MSC2_HOTPLUG_PIN);
	else
		__gpio_as_irq_low_level(MSC2_HOTPLUG_PIN);
}

static struct jz_mmc_platform_data npm701_tf_data = {
#ifndef CONFIG_MSC2_SDIO_SUPPORT
	.support_sdio   = 0,
#else
	.support_sdio   = 1,
#endif
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
	.init           = npm701_tf_gpio_init,
	.status_irq	= MSC2_HOTPLUG_IRQ,
	.detect_pin     = GPIO_SD2_CD_N,
	.power_on       = npm701_tf_power_on,
	.power_off      = npm701_tf_power_off,
	.cpm_start      = npm701_tf_cpm_start,
	.status		= npm701_tf_status,
	.plug_change	= npm701_tf_plug_change,
	.max_bus_width  = MMC_CAP_SD_HIGHSPEED | MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_MSC2_BUS_1
	.bus_width      = 1,
#elif defined  CONFIG_MSC2_BUS_4
	.bus_width      = 4,
#else
	.bus_width      = 4,
#endif
};

static void npm701_sdio_gpio_init(struct device *dev)
{
	__gpio_as_msc1_pd_4bit();
}

static void npm701_sdio_power_on(struct device *dev)
{
}

static void npm701_sdio_power_off(struct device *dev)
{
}

static void npm701_sdio_cpm_start(struct device *dev)
{
	cpm_set_clock(CGU_MSC1CLK, 25 * 1000 * 1000);
	cpm_start_clock(CGM_MSC1);
}

static unsigned int npm701_sdio_status(struct device *dev)
{
	unsigned int status;

	status = 1;
	return (status);
}

static struct jz_mmc_platform_data npm701_sdio_data = {
#ifndef CONFIG_MSC1_SDIO_SUPPORT
	.support_sdio   = 0,
#else
	.support_sdio   = 1,
#endif
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
	.init           = npm701_sdio_gpio_init,
	.power_on       = npm701_sdio_power_on,
	.power_off      = npm701_sdio_power_off,
	.cpm_start      = npm701_sdio_cpm_start,
	.status		= npm701_sdio_status,
	.max_bus_width  = MMC_CAP_4_BIT_DATA, 
#ifdef CONFIG_MSC1_BUS_1
	.bus_width      = 1,
#elif defined  CONFIG_MSC1_BUS_4
	.bus_width      = 4,
#else
	.bus_width      = 4,
#endif
	.need_mdetect   = 1,
};

void __init board_msc_init(void)
{
#ifdef CONFIG_MSC0_JZ4770
	jz_add_msc_devices(0, &npm701_inand_data);
#endif

#ifdef CONFIG_MSC1_JZ4770
	jz_add_msc_devices(1, &npm701_sdio_data);
#endif

#ifdef CONFIG_MSC2_JZ4770
	jz_add_msc_devices(2, &npm701_tf_data);
#endif
}

void IW8101_wlan_power_off(int flag)
{
	printk(">>>>>>>>IW8101_wlan_power_off\n");
	switch(flag) {
		case RESET:
			__gpio_as_output(GPIO_WL_RST_N);

			__gpio_clear_pin(GPIO_WL_RST_N);

			// Turn off power supply
			act8600_output_enable(ACT8600_OUT6, ACT8600_OUT_OFF);
			break;

		case NORMAL:
			__gpio_as_output(GPIO_WL_RST_N);

			__gpio_clear_pin(GPIO_WL_RST_N);

			// Turn off power supply
			act8600_output_enable(ACT8600_OUT6, ACT8600_OUT_OFF);

			present = 0;

			up(&detect_start_mutex);

			if(down_interruptible(&detect_done_mutex))
				printk("\ndown error!\n\n");

			break;
	}

	wake_unlock(&wlan_power);
	/*disable wifi 32k clk*/
	rtc_disable_clk32k();
}

void IW8101_wlan_power_on(int flag)
{
	/*enable wifi 32k clk*/
	printk(">>>>>>>>IW8101_wlan_power_on\n");
	rtc_enable_clk32k();
	mdelay(200);
	switch(flag) {
		case RESET:
			// Turn on power supply
			act8600_output_enable(ACT8600_OUT6, ACT8600_OUT_ON);
			__gpio_as_output(GPIO_WL_RST_N);
			__gpio_clear_pin(GPIO_WL_RST_N);

			mdelay(200);

			__gpio_set_pin(GPIO_WL_RST_N);

			mdelay(200);

			break;

		case NORMAL:
			// Turn on power supply
			act8600_output_enable(ACT8600_OUT6, ACT8600_OUT_ON);
			__gpio_as_output(GPIO_WL_RST_N);
			__gpio_clear_pin(GPIO_WL_RST_N);

			mdelay(200);

			__gpio_set_pin(GPIO_WL_RST_N);

			mdelay(200);

			present = 1;

			up(&detect_start_mutex);

			if(down_interruptible(&detect_done_mutex))
				printk("\ndown error!\n\n");

			break;
	}

	wake_lock(&wlan_power);
}
static void __init board_cpm_setup(void)
{
	/* Stop unused module clocks here.
	 * We have started all module clocks at arch/mips/jz4760/setup.c.
	 */
}

static void __init board_gpio_setup(void)
{
	__gpio_as_input(GPIO_USB_DETE);
	__gpio_as_input(GPIO_OTG_ID_PIN);
}

static void __init board_wlan_setup(void)
{
	wake_lock_init(&wlan_power, WAKE_LOCK_SUSPEND, "wlan_power");
}

void __init jz_board_setup(void)
{
	printk("JZ4770 NP_M701 board setup\n");
	//	jz_restart(NULL);
	board_cpm_setup();
	board_gpio_setup();
	board_wlan_setup();
}

/**
 * Called by arch/mips/kernel/proc.c when 'cat /proc/cpuinfo'.
 * Android requires the 'Hardware:' field in cpuinfo to setup the init.%hardware%.rc.
 */
const char *get_board_type(void)
{
	return "npm701";
}

static struct android_pmem_platform_data pmem_pdata = {
	.name = "pmem",
	.no_allocator = 1,
	.cached = 1,
	.start = JZ_PMEM_BASE,
	.size = JZ_PMEM_SIZE,
};

static struct android_pmem_platform_data pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.no_allocator = 0,
	.cached = 1,
	.start = JZ_PMEM_ADSP_BASE,
	.size = JZ_PMEM_ADSP_SIZE,
};

static struct platform_device pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &pmem_pdata },
};

static struct platform_device pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &pmem_adsp_pdata },
};

#ifdef CONFIG_TOUCHSCREEN_JZ_GT801
static struct gt801_ts_platform_data gt801_ts_pdata = {   
	.intr = GPIO_GT801_INT,
};
#endif

#ifdef CONFIG_KEYBOARD_HA2605

static struct touch_keys_button  touch_buttons[7] = {
	{
		.code   	= KEY_HOME,//key4
	},
	{
		.code   	= KEY_BACK,//key5
	},
	{
		.code   	= KEY_MENU,//key3
	},
	{
		.code   	= KEY_VOLUMEDOWN,//key2
	},
	{
		.code   	= KEY_VOLUMEUP,//key1
	},
};

static struct ha2605_platform_data ha2605_pdata = {
	.intr = GPIO_HA2605_INT,
	.buttons	= touch_buttons,
	.nbuttons	= ARRAY_SIZE(touch_buttons),
};
#endif
#ifdef CONFIG_GC0308
static struct jz_cim_sensor_platform_data gc0308_platform_pdata = {
	.facing = CAMERA_FACING_FRONT,
	.orientation = 270,
  	.gpio_rst = GPIO_CAMERA_RST,
  	.gpio_en = GPIO_CAMERA_PDN,

};
#endif
#ifdef CONFIG_GT2005
static struct jz_cim_sensor_platform_data gt2005_platform_pdata = {
	.facing = CAMERA_FACING_BACK,
	.orientation = 90,
  	.gpio_rst = GPIO_CAMERA_RST,
  	.gpio_en = GPIO_CAMERA_PDN,

};
#endif

#if defined(CONFIG_JZ_CIM)
#ifdef CONFIG_NPM701_V_1_1 /* old code */


static void cim_power_off(void)
{
	cpm_stop_clock(CGM_CIM);

	__gpio_as_output(GPIO_CAM2_PWR_ON);
	__gpio_clear_pin(GPIO_CAM2_PWR_ON);
	__gpio_as_output(GPIO_CAM1_PWR_ON);
	__gpio_clear_pin(GPIO_CAM1_PWR_ON);
}

static void cim_power_on(void)
{
	cpm_stop_clock(CGM_CIM);
	cpm_set_clock(CGU_CIMCLK,24000000);
	cpm_start_clock(CGM_CIM);

	__gpio_as_output(GPIO_CAM2_PWR_ON);
	__gpio_set_pin(GPIO_CAM2_PWR_ON);
	__gpio_as_output(GPIO_CAM1_PWR_ON);
	__gpio_set_pin(GPIO_CAM1_PWR_ON);
}
#else /* Code after gpio fix */
static void cim_power_off(void)
{
	cpm_stop_clock(CGM_CIM);
	__gpio_as_output(GPIO_CAM_PWR_ON);
	__gpio_clear_pin(GPIO_CAM_PWR_ON);
}

static void cim_power_on(void)
{
	cpm_stop_clock(CGM_CIM);
	cpm_set_clock(CGU_CIMCLK,24000000);
	cpm_start_clock(CGM_CIM);

	__gpio_as_output(GPIO_CAM_PWR_ON);
	__gpio_set_pin(GPIO_CAM_PWR_ON);
	mdelay(10);
}
#endif


static struct jz_cim_platform_data jz_cim_pdata = {
	.power_on = cim_power_on,
	.power_off = cim_power_off,
};

static struct platform_device jz_cim_device = {
	.name = "jz_cim",
	.dev = { .platform_data = &jz_cim_pdata },
};
#endif /* defined(CONFIG_JZ_CIM) */

static struct i2c_board_info npm701_i2c1_devs[] __initdata = {
#ifndef CONFIG_NPM701_V_1_1
#ifdef CONFIG_TOUCHSCREEN_JZ_GT801
	{
		I2C_BOARD_INFO(GT801_NAME, 0x55),
		.irq = GPIO_GT801_IRQ,
		.platform_data = &gt801_ts_pdata,
	},
#endif
#else /* NPM701_V_1_1 */
#ifdef CONFIG_KEYBOARD_HA2605
	{
		I2C_BOARD_INFO(HA_KEY_NAME, 0x62),
		.irq = GPIO_HA2605_IRQ,
		.platform_data = &ha2605_pdata,
	},
#endif

#endif
	{
	},
};
static struct jz_remote jz_remote_pdata = {
	.RMC_INT = GPIO_RMC_INT,
	.RMC_IRQ = GPIO_RMC_IRQ,	
};
static struct platform_device jz_remote_device = {
	.name = "jz-remote",
	.id = 2,
	.dev = { .platform_data = &jz_remote_pdata },
};

static struct gpio_keys_button board_buttons[] = {
#ifdef GPIO_CALL
	{
		.gpio		= GPIO_CALL,
		.code   	= KEY_SEND,
		.desc		= "call key",
		.active_low	= ACTIVE_LOW_CALL,
	},
#endif
#ifdef GPIO_HOME
	{
		.gpio		= GPIO_HOME,
		.code   	= KEY_HOME,
		.desc		= "home key",
		.active_low	= ACTIVE_LOW_HOME,
	},
#endif
#ifdef GPIO_BACK
	{
		.gpio		= GPIO_BACK,
		.code   	= KEY_BACK,
		.desc		= "back key",
		.active_low	= ACTIVE_LOW_BACK,
	},
#endif
#ifdef GPIO_MENU
	{
		.gpio		= GPIO_MENU,
		.code   	= KEY_MENU,
		.desc		= "menu key",
		.active_low	= ACTIVE_LOW_MENU,
	},
#endif
#ifdef GPIO_ENDCALL
	{
		.gpio		= GPIO_ENDCALL,
		.code   	= KEY_END,
		.desc		= "end call key",
		.active_low	= ACTIVE_LOW_ENDCALL,
	},
#endif
#ifdef GPIO_VOLUMEDOWN
	{
		.gpio		= GPIO_VOLUMEDOWN,
		.code   	= KEY_VOLUMEDOWN,
		.desc		= "volum down key",
		.active_low	= ACTIVE_LOW_VOLUMEDOWN,
	},
	{
		.gpio		= GPIO_VOLUMEUP,
		.code   	= KEY_VOLUMEUP,
		.desc		= "volum up key",
		.active_low	= ACTIVE_LOW_VOLUMEUP,
	},
#endif
};

static struct gpio_keys_platform_data board_button_data = {
	.buttons	= board_buttons,
	.nbuttons	= ARRAY_SIZE(board_buttons),
};

static struct platform_device board_button_device = {
	.name		= "jz-gpio-key",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &board_button_data,
	}
};

static struct act8600_outputs_t act8600_outputs[] = {
	{1,0x1a,1},//out1 for core   1.3v
	{4,0x57,1},//out4 for OTG   5.3 - 0.15 ~~~ 5.15 		
	{5,0x31,1},//out5 		2.5  - 0b110001
	{6,0x39,0},//out6 wifi_io 	3.3V  - 0b111001
	{7,0x39,0},//out7 hdmi 		3.3V CON - 0b011000
	{8,0x24,0},//out8 hdmi		1.8V CON - 0b100100
};

static struct act8600_platform_pdata_t act8600_platform_pdata = {
	.outputs = act8600_outputs,
	.nr_outputs = ARRAY_SIZE(act8600_outputs),
};

#ifdef  CONFIG_SENSORS_JZ_MMA8452
static struct mma8452_platform_data mma8452_platform_pdata = {
	.intr = GPIO_MMA8452_INT1,
	.poll_interval = 100,
	.min_interval = 40,
	.max_interval = 200,
	.g_range = MMA8452_2G,
	.axis_map_x = 1,
	.axis_map_y = 0,
	.axis_map_z = 2,
	.negate_x = 1,
	.negate_y = 0,
	.negate_z = 1,
};
#endif


static struct i2c_board_info npm701_i2c0_devs[] __initdata = {
#ifdef CONFIG_SENSORS_JZ_MMA8452
	{
		I2C_BOARD_INFO("mma8452",0x1c),
		.irq = GPIO_MMA8452_IRQ1,
		.platform_data = &mma8452_platform_pdata,
	},
#endif
#ifndef CONFIG_NPM701_V_1_1
#ifdef CONFIG_GT2005
	{
		I2C_BOARD_INFO("gt2005", 0x3C),
		.platform_data = &gt2005_platform_pdata,
	},
#endif /* NPM701_V_1_1 */
#ifdef CONFIG_GC0308	
	//camera
	{
		I2C_BOARD_INFO("gc0308", 0x21),
		.platform_data = &gc0308_platform_pdata,
	},
#endif
#endif
	{
	},
};
static struct i2c_board_info npm701_i2c2_devs[] __initdata = {
#ifndef CONFIG_NPM701_V_1_1

#else /* NPM701_V_1_1 */
#ifdef CONFIG_TOUCHSCREEN_JZ_GT801
	{
		I2C_BOARD_INFO(GT801_NAME, 0x55),
		.irq = GPIO_GT801_IRQ,
		.platform_data = &gt801_ts_pdata,
	},
#endif
#endif
	{
	},
};



#if defined(CONFIG_I2C_GPIO)
static struct i2c_board_info npm701_gpio_i2c3_devs[] __initdata = {
#ifdef CONFIG_PMU_ACT8600_SUPPORT
	{
		I2C_BOARD_INFO(ACT8600_NAME, 0x5a),
		.platform_data = &act8600_platform_pdata,
	},
#endif
};
static struct i2c_gpio_platform_data i2c3_gpio_data = {
	.sda_pin	= GPIO_I2C3_SDA,
	.scl_pin	= GPIO_I2C3_SCK,
};

static struct platform_device i2c3_gpio_device = {
	.name	= "i2c-gpio",
	.id	= 3,
	.dev	= {
		.platform_data = &i2c3_gpio_data,
	},
};


static struct i2c_board_info npm701_gpio_i2c4_devs[] __initdata = {
#ifndef CONFIG_NPM701_V_1_1
#ifdef CONFIG_KEYBOARD_HA2605
	{
		I2C_BOARD_INFO(HA_KEY_NAME, 0x62),
		.irq = GPIO_HA2605_IRQ,
		.platform_data = &ha2605_pdata,
	},
#endif
#else /* NPM701_V_1_1 */
#ifdef CONFIG_GT2005
	{
		I2C_BOARD_INFO("gt2005", 0x3C),
		.platform_data = &gt2005_platform_pdata,
	},
#endif
#ifdef CONFIG_GC0308	
	//camera
	{
		I2C_BOARD_INFO("gc0308", 0x21),
		.platform_data = &gc0308_platform_pdata,
	},
#endif
#endif
};



static struct i2c_gpio_platform_data i2c4_gpio_data = {
	.sda_pin	= GPIO_I2C4_SDA,
	.scl_pin	= GPIO_I2C4_SCK,
};

static struct platform_device i2c4_gpio_device = {
	.name	= "i2c-gpio",
	.id	= 4,
	.dev	= {
		.platform_data = &i2c4_gpio_data,
	},
};

static struct i2c_board_info npm701_gpio_i2c5_devs[] __initdata = {
};



static struct i2c_gpio_platform_data i2c5_gpio_data = {
	.sda_pin	= GPIO_I2C5_SDA,
	.scl_pin	= GPIO_I2C5_SCK,
};

static struct platform_device i2c5_gpio_device = {
	.name	= "i2c-gpio",
	.id	= 5,
	.dev	= {
		.platform_data = &i2c5_gpio_data,
	},
};

#endif

static struct battery_info jz47xx_battery_info = {
	.max_vol = 4100000,
	.min_vol = 3600000,
	.dc_chg_max_vol = 4120000,
	.dc_chg_min_vol = 3720000,
	.usb_chg_max_vol = 4120000,
	.usb_chg_min_vol = 3675000,
	.battery_mah = 4000,
	.dc_charg_ma = 800,
	.usb_charg_ma = 600,
};

static struct act8600_interface_platform_data act8600_pdata = {
	.irq = GPIO_PMU_IRQ,
};

static struct jz47xx_battery_platform_data jz47xx_battery_pdata = {
	.interface_pdata = &act8600_pdata,
	.info = &jz47xx_battery_info,
};

static struct platform_device jz47xx_battery_device = {
	.name = "jz47xx-battery",
	.dev = { .platform_data = &jz47xx_battery_pdata },
};

#ifdef CONFIG_HDMI_IT6610
void npm701_hdmi_board_init(void)
{
}
void npm701_hdmi_power_on(void)
{
	act8600_output_enable(ACT8600_OUT7, ACT8600_OUT_ON);
	act8600_output_enable(ACT8600_OUT8, ACT8600_OUT_ON);
}
void npm701_hdmi_power_off(void)
{
	act8600_output_enable(ACT8600_OUT7, ACT8600_OUT_OFF);
	act8600_output_enable(ACT8600_OUT8, ACT8600_OUT_OFF);
}

struct hdmi_it6610_pin_info npm701_it6610_pin_info = {
	.PIN_HPD =  GPIO_HDMI_HPD,
	//	.PIN_SYSRSTN = GPIO_HDMI_RST_N,
	.PIN_INT =  GPIO_HDMI_INT_N,
	.PIN_PCSCL = GPIO_HDMI_PS2_KCLK,
	.PIN_PCSDA = GPIO_HDMI_PS2_KDATA,
};
struct hdmi_it6610_board_info npm701_ite6610_board_info = {
	.pin_info = &npm701_it6610_pin_info,
#ifdef CONFIG_HDMI_HOTPLUG_HPD_CONNECT_LOW_ACTIVE
	.hpd_connect_active = 0,
#else
	.hpd_connect_active = 1,
#endif
	.hdmi_board_init = npm701_hdmi_board_init,
	.hdmi_power_on = npm701_hdmi_power_on,
	.hdmi_power_off = npm701_hdmi_power_off,
};

static struct platform_device board_ite6610_device = {
	.name = "ite6610-drv",
	.dev = { .platform_data = &npm701_ite6610_board_info },
};
#endif

void board_usb_power_callback(void)
{
	//TODO: place your codes to handler usb_vbus actions just like the name of function
	//Context: can sleep here

	printk("--------> %s\n", __FUNCTION__);
	act8600_set_double_q3(1);
}

void board_usb_disconnect_callback(void)
{
	printk("--------> %s\n", __FUNCTION__);
	act8600_set_double_q3(0);
}

void board_usb_connect_callback(void)
{
	printk("--------> %s\n", __FUNCTION__);
	act8600_set_double_q3(0);
}

/* If no PMU, keep this function as empty */
void board_set_vbus(int on)
{
	if (on)
		act8600_set_q1(1);
	else
		act8600_set_q3(1);
}


/* ########################################################################### */
struct timed_gpio vibrator_timed_gpio = {
	.name         = "vibrator",
	.gpio         = GPIO_MOTOR_PIN,
	.active_low   = 0,
	.max_timeout  = 15000,
};
static struct timed_gpio_platform_data vibrator_platform_data ={
	.num_gpios    =1,
	.gpios        =&vibrator_timed_gpio,
};

static struct platform_device jz_timed_gpio_device = {
	.name = TIMED_GPIO_NAME,
	.id   = 0,
	.dev  = {
		.platform_data         = &vibrator_platform_data,
	},
};
/* ######################################################################### */
static int __init npm701_board_init(void)
{

#if defined(CONFIG_I2C_GPIO)
	i2c_register_board_info(3, npm701_gpio_i2c3_devs, ARRAY_SIZE(npm701_gpio_i2c3_devs));
	i2c_register_board_info(4, npm701_gpio_i2c4_devs, ARRAY_SIZE(npm701_gpio_i2c4_devs));
	i2c_register_board_info(5, npm701_gpio_i2c5_devs, ARRAY_SIZE(npm701_gpio_i2c5_devs));	
	platform_device_register(&i2c3_gpio_device);
	platform_device_register(&i2c4_gpio_device);
	platform_device_register(&i2c5_gpio_device);
#endif

	i2c_register_board_info(0, npm701_i2c0_devs, ARRAY_SIZE(npm701_i2c0_devs));
	i2c_register_board_info(1, npm701_i2c1_devs, ARRAY_SIZE(npm701_i2c1_devs));
	i2c_register_board_info(2, npm701_i2c2_devs, ARRAY_SIZE(npm701_i2c2_devs));

#if defined(CONFIG_JZ_CIM)
	platform_device_register(&jz_cim_device);
#endif
	platform_device_register(&jz_timed_gpio_device);	
	platform_device_register(&jz_remote_device);
	platform_device_register(&pmem_device);
	platform_device_register(&pmem_adsp_device);
	platform_device_register(&board_button_device);
	platform_device_register(&jz47xx_battery_device);
#ifdef CONFIG_HDMI_IT6610
	platform_device_register(&board_ite6610_device);
#endif
	return 0;
}

arch_initcall(npm701_board_init);
EXPORT_SYMBOL(IW8101_wlan_power_off);
EXPORT_SYMBOL(IW8101_wlan_power_on);
