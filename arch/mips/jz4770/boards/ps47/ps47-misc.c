/*
 * linux/arch/mips/jz4760/board-ps47.c
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
#include <../drivers/input/touchscreen/ft5x0x_ts.h>
#include <../drivers/input/touchscreen/ctp_it7260.h>
#include <linux/act8600_power.h>
#include <linux/jz47xx_battery.h>
#include <../drivers/i2c/chips/jz_sensor_mma8452.h>
#include <linux/jz_cim_core.h>

#include <../drivers/staging/android/timed_gpio.h>

#define RESET  0
#define NORMAL 1

void __init board_msc_init(void);
extern int jz_add_msc_devices(unsigned int controller, struct jz_mmc_platform_data *plat);
extern void (*jz_timer_callback)(void);

extern struct semaphore detect_start_mutex;
extern struct semaphore detect_done_mutex;
extern int present;

static struct wake_lock wlan_power;

/* MSC SETUP */
static void ps47_inand_gpio_init(struct device *dev)
{
	__gpio_as_msc0_pa_4bit();
}

static void ps47_inand_power_on(struct device *dev)
{
}

static void ps47_inand_power_off(struct device *dev)
{
}

static void ps47_inand_cpm_start(struct device *dev)
{
	cpm_set_clock(CGU_MSC0CLK, 25 * 1000 * 1000);
	cpm_start_clock(CGM_MSC0);
}

static unsigned int ps47_inand_status(struct device *dev)
{
	unsigned int status;

	status = 1;
	return (status);
}

#define KBYTE  (1024)
#define MBYTE  ((KBYTE)*(KBYTE))
#define UINT32_MAX             (0xffffffffU)

struct mmc_partition_info ps47_partitions[] = {
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

static struct jz_mmc_platform_data ps47_inand_data = {
#ifndef CONFIG_MSC0_SDIO_SUPPORT
	.support_sdio   = 0,
#else
	.support_sdio   = 1,
#endif
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
	.init           = ps47_inand_gpio_init,
	.power_on       = ps47_inand_power_on,
	.power_off      = ps47_inand_power_off,
	.cpm_start      = ps47_inand_cpm_start,
	.status		= ps47_inand_status,
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
	.partitions = ps47_partitions,
	.num_partitions = ARRAY_SIZE(ps47_partitions),
};

static void ps47_tf_gpio_init(struct device *dev)
{
	__gpio_as_msc1_pd_4bit();
	__gpio_as_output(GPIO_SD1_VCC_EN_N);
	__gpio_set_pin(GPIO_SD1_VCC_EN_N);            //always power up
}

static void ps47_tf_power_on(struct device *dev)
{
	__gpio_set_pin(GPIO_SD1_VCC_EN_N);
}

static void ps47_tf_power_off(struct device *dev)
{
	__gpio_clear_pin(GPIO_SD1_VCC_EN_N);
}

static void ps47_tf_cpm_start(struct device *dev)
{
	cpm_set_clock(CGU_MSC1CLK, 25 * 1000 * 1000);
	cpm_start_clock(CGM_MSC1);
}

static unsigned int ps47_tf_status(struct device *dev)
{
	unsigned int status;

	status = (unsigned int) !__gpio_get_pin(GPIO_SD1_CD_N);
	return (status);
}

static void ps47_tf_plug_change(int state)
{
	if(state == CARD_INSERTED)
		__gpio_as_irq_high_level(MSC1_HOTPLUG_PIN);
	else
		__gpio_as_irq_low_level(MSC1_HOTPLUG_PIN);
}

static struct jz_mmc_platform_data ps47_tf_data = {
#ifndef CONFIG_MSC1_SDIO_SUPPORT
	.support_sdio   = 0,
#else
	.support_sdio   = 1,
#endif
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
	.init           = ps47_tf_gpio_init,
	.status_irq	= MSC1_HOTPLUG_IRQ,
	.detect_pin     = GPIO_SD1_CD_N,
	.power_on       = ps47_tf_power_on,
	.power_off      = ps47_tf_power_off,
	.cpm_start      = ps47_tf_cpm_start,
	.status		= ps47_tf_status,
	.plug_change	= ps47_tf_plug_change,
	.max_bus_width  = MMC_CAP_SD_HIGHSPEED | MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_MSC1_BUS_1
	.bus_width      = 1,
#elif defined  CONFIG_MSC1_BUS_4
	.bus_width      = 4,
#else
	.bus_width      = 4,
#endif
};

static void ps47_sdio_gpio_init(struct device *dev)
{
	__gpio_as_msc2_pe_4bit();
}

static void ps47_sdio_power_on(struct device *dev)
{
}

static void ps47_sdio_power_off(struct device *dev)
{
}

static void ps47_sdio_cpm_start(struct device *dev)
{
	cpm_set_clock(CGU_MSC2CLK, 25 * 1000 * 1000);
	cpm_start_clock(CGM_MSC2);
}

static unsigned int ps47_sdio_status(struct device *dev)
{
	unsigned int status;

	status = 1;
	return (status);
}

static struct jz_mmc_platform_data ps47_sdio_data = {
#ifndef CONFIG_MSC2_SDIO_SUPPORT
	.support_sdio   = 0,
#else
	.support_sdio   = 1,
#endif
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
	.init           = ps47_sdio_gpio_init,
	.power_on       = ps47_sdio_power_on,
	.power_off      = ps47_sdio_power_off,
	.cpm_start      = ps47_sdio_cpm_start,
	.status		= ps47_sdio_status,
	.max_bus_width  = MMC_CAP_4_BIT_DATA, 
#ifdef CONFIG_MSC2_BUS_1
	.bus_width      = 1,
#elif defined  CONFIG_MSC2_BUS_4
	.bus_width      = 4,
#else
	.bus_width      = 4,
#endif
	.need_mdetect   = 1,
};

void __init board_msc_init(void)
{
#ifdef CONFIG_MSC0_JZ4770
	jz_add_msc_devices(0, &ps47_inand_data);
#endif

#ifdef CONFIG_MSC2_JZ4770
	jz_add_msc_devices(1, &ps47_tf_data);
		
#endif
#ifdef CONFIG_MSC1_JZ4770
	jz_add_msc_devices(2, &ps47_sdio_data);	
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
	printk("JZ4770 PS47 board setup\n");
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
	return "ps47";
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

#ifdef CONFIG_TOUCHSCREEN_FT5X0X
static struct ft5x0x_ts_platform_data ft5x0x_ts_pdata = {   
	.x_res = 800,
	.y_res = 480,
	.pressure_min = 0,
	.pressure_max = 255,
  .intr = GPIO_CTP_INT,
	.wake =	GPIO_CTP_WAKE,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_CTP_IT7260
static struct ctp_it7260_platform_data ctp_it7260_pdata = {   
	.intr = GPIO_CTP_INT,
	.power = GPIO_CTP_VCC_EN,
	.width = 800,
	.height = 600,
	.is_i2c_gpio = 0,
};
#endif

#ifdef CONFIG_SIV120B
static struct jz_cim_sensor_platform_data siv120b_platform_pdata = {
	.facing = CAMERA_FACING_FRONT,
  	.orientation = 0,
	.gpio_rst = GPIO_CAMERA_RST,
  	.gpio_en = GPIO_CAMERA_PDN,
};
#endif

#if defined(CONFIG_JZ_CIM)
static void cim_power_off(void)
{
	act8600_output_enable(ACT8600_OUT7, ACT8600_OUT_OFF);
	act8600_output_enable(ACT8600_OUT8, ACT8600_OUT_OFF);
	msleep(1);
	cpm_stop_clock(CGM_CIM);
}

static void cim_power_on(void)
{
	act8600_output_enable(ACT8600_OUT7, ACT8600_OUT_ON);
	act8600_output_enable(ACT8600_OUT8, ACT8600_OUT_ON);
	msleep(100);
	cpm_stop_clock(CGM_CIM);
	cpm_set_clock(CGU_CIMCLK,24000000);
	cpm_start_clock(CGM_CIM);
}

static struct jz_cim_platform_data jz_cim_pdata = {
	.power_on = cim_power_on,
	.power_off = cim_power_off,
};

static struct platform_device jz_cim_device = {
	.name = "jz_cim",
	.dev = { .platform_data = &jz_cim_pdata },
};
#endif 

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
	{4,0x57,1},//out4 for OTG   5.3 - 0.15 ~~~ 5.15 		
	{5,0x31,1},//out5 		2.5  - 0b110001
	{6,0x39,0},//out6 wifi_io 	3.3V  - 0b111001
	{7,0x34,0},//out7 cim_2.8V 	2.8V CON - 0b110100
	{8,0x1e,0},//out8 cim_1.5V	1.5V CON - 0b011110
};

static struct act8600_platform_pdata_t act8600_platform_pdata = {
	.outputs = act8600_outputs,
	.nr_outputs = ARRAY_SIZE(act8600_outputs),
};

#ifdef  CONFIG_SENSORS_MMA8452
static struct mma8452_platform_data mma8452_platform_pdata = {
	.intr = GPIO_MMA8452_INT1,
	.poll_interval = 100,
	.min_interval = 40,
	.max_interval = 200,
	.g_range = MMA8452_2G,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 1,
	.negate_y = 0, //0,
	.negate_z = 0,
};
#endif

static struct i2c_board_info ps47_i2c1_devs[] __initdata = {
#ifdef CONFIG_SENSORS_MMA8452
	{
		I2C_BOARD_INFO("mma8452",0x1c),
		.irq = GPIO_MMA8452_IRQ1,
		.platform_data = &mma8452_platform_pdata,
	},
#endif
};

static struct i2c_board_info ps47_i2c0_devs[] __initdata = {
#ifdef CONFIG_TOUCHSCREEN_CTP_IT7260
	{
		I2C_BOARD_INFO(IT7260_NAME, 0x46),
		.irq = GPIO_CTP_IRQ,
		.platform_data = &ctp_it7260_pdata,
	},
#endif

	{
	},
};



#if defined(CONFIG_I2C_GPIO)
static struct i2c_board_info ps47_gpio_i2c3_devs[] __initdata = {
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

static struct i2c_board_info ps47_gpio_i2c4_devs[] __initdata = {
#ifdef CONFIG_SIV120B	
	{
		I2C_BOARD_INFO("siv120b", 0x33),
		.platform_data = &siv120b_platform_pdata,
	},
#endif
	{
	},
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
#endif

static struct battery_info jz47xx_battery_info = {
	.max_vol = 4080000,
	.min_vol = 3600000,
	.dc_chg_max_vol = 4150000,
	.dc_chg_min_vol = 3740000,
	.usb_chg_max_vol = 4150000,
	.usb_chg_min_vol = 3675000,
	.battery_mah = 2550,
	.dc_charg_ma = 750,
	.usb_charg_ma = 100,
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

/* If no PMU, keep this function as empty */
int board_set_vbus(struct device *dev, int is_on)
{
	if (is_on)
		act8600_set_q1(is_on);
	else
		act8600_set_q3(!is_on);
	return 0;
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
static int __init ps47_board_init(void)
{

#if defined(CONFIG_I2C_GPIO)
	i2c_register_board_info(3, ps47_gpio_i2c3_devs, ARRAY_SIZE(ps47_gpio_i2c3_devs));
	i2c_register_board_info(4, ps47_gpio_i2c4_devs, ARRAY_SIZE(ps47_gpio_i2c4_devs));
	//i2c_register_board_info(5, ps47_gpio_i2c5_devs, ARRAY_SIZE(ps47_gpio_i2c5_devs));	
	platform_device_register(&i2c3_gpio_device);
	platform_device_register(&i2c4_gpio_device);
	//platform_device_register(&i2c5_gpio_device);
#endif

	i2c_register_board_info(1, ps47_i2c1_devs, ARRAY_SIZE(ps47_i2c1_devs));
	//i2c_register_board_info(1, ps47_i2c1_devs, ARRAY_SIZE(ps47_i2c1_devs));
	i2c_register_board_info(0, ps47_i2c0_devs, ARRAY_SIZE(ps47_i2c0_devs));

#if defined(CONFIG_JZ_CIM)
	platform_device_register(&jz_cim_device);
#endif
	platform_device_register(&jz_timed_gpio_device);	
	//platform_device_register(&jz_remote_device);
	platform_device_register(&pmem_device);
	platform_device_register(&pmem_adsp_device);
	platform_device_register(&board_button_device);
	platform_device_register(&jz47xx_battery_device);


	return 0;
}

arch_initcall(ps47_board_init);
EXPORT_SYMBOL(IW8101_wlan_power_off);
EXPORT_SYMBOL(IW8101_wlan_power_on);
