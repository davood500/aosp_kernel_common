/*
 * Platform device support for Jz4760 SoC.
 *
 * Copyright 2007, <yliu@ingenic.cn>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/resource.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>

#ifdef CONFIG_ANDROID_PMEM
#include <linux/android_pmem.h>
#endif

#include <asm/jzsoc.h>

#include <linux/usb/musb.h>
#include <mach/jz_audio.h>
#include <linux/switch.h>

#include <asm/jzmmc/jz_mmc_platform_data.h>

extern void __init board_msc_init(void);
extern void board_set_vbus(int on);
extern void board_usb_power_callback(void);
extern void board_usb_disconnect_callback(void);
extern void board_usb_connect_callback(void);
int __init jz_add_msc_devices(unsigned int controller, struct jz_mmc_platform_data *plat);

/* OHCI (USB full speed host controller) */
static struct resource jz_usb_ohci_resources[] = {
	[0] = {
		.start		= CPHYSADDR(UHC_BASE), // phys addr for ioremap
		.end		= CPHYSADDR(UHC_BASE) + 0x10000 - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_UHC,
		.end		= IRQ_UHC,
		.flags		= IORESOURCE_IRQ,
	},
};

/* The dmamask must be set for OHCI to work */
static u64 ohci_dmamask = ~(u32)0;

static struct platform_device jz_usb_ohci_device = {
	.name		= "jz-ohci",
	.id		= 0,
	.dev = {
		.dma_mask		= &ohci_dmamask,
		.coherent_dma_mask	= 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(jz_usb_ohci_resources),
	.resource	= jz_usb_ohci_resources,
};

/*** LCD controller ***/
static struct resource jz_lcd_resources[] = {
	[0] = {
		.start          = CPHYSADDR(LCD_BASE),
		.end            = CPHYSADDR(LCD_BASE) + 0x10000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	[1] = {
		.start          = IRQ_LCD,
		.end            = IRQ_LCD,
		.flags          = IORESOURCE_IRQ,
	}
};

static u64 jz_lcd_dmamask = ~(u32)0;

struct platform_device jz_lcd_device = {
	.name           = "jz-lcd",
	.id             = 0,
	.dev = {
		.dma_mask               = &jz_lcd_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(jz_lcd_resources),
	.resource       = jz_lcd_resources,
};

/* USB OTG Controller */
static struct musb_hdrc_config jz_musb_hdrc_config = {
	.multipoint	= 1,
	.dyn_fifo	= 0,
	.soft_con	= 1,
	.dma		= 1,
	/* Max EPs scanned. Driver will decide which EP can be used automatically. */
	.num_eps	= 5,
};

static struct musb_hdrc_platform_data jz_musb_hdrc_pdata = {
#if defined(CONFIG_USB_MUSB_OTG)
	.mode           = MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode           = MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode           = MUSB_PERIPHERAL,
#endif
	.config		= &jz_musb_hdrc_config,
};

static struct jz47xx_musb_platform_data jz_musb_platform_data = {
	.musb_pdata	= &jz_musb_hdrc_pdata,
	.set_vbus	= board_set_vbus,
	.usb_power_callback = board_usb_power_callback,
	.usb_disconnect_callback = board_usb_disconnect_callback,
	.usb_connect_callback = board_usb_connect_callback,
};

static struct resource jz_musb_resources[] = {
	[0] = {
		.start		= CPHYSADDR(UDC_BASE),
		.end		= CPHYSADDR(UDC_BASE) + 0x10000 - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.name		= "mc",
		.start		= IRQ_OTG,
		.end		= IRQ_OTG,
		.flags		= IORESOURCE_IRQ,
	},
	[2] = {
		.name		= "id",
		.start		= (IRQ_GPIO_0 + GPIO_OTG_ID_PIN),
		.end		= (IRQ_GPIO_0 + GPIO_OTG_ID_PIN),
		.flags		= IORESOURCE_IRQ,
	},
	[3] = {
		.name		= "vbus_detect",
		.start		= (IRQ_GPIO_0 + GPIO_USB_DETE),
		.end		= (IRQ_GPIO_0 + GPIO_USB_DETE),
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device jz_musb_device = {
	.name	= "musb-jz47xx",
	.id	= 0,
	.dev = {
		.platform_data		= &jz_musb_platform_data,
	},
	.resource	= jz_musb_resources,
	.num_resources	= ARRAY_SIZE(jz_musb_resources),
};

/** MMC/SD controller MSC0**/
static struct resource jz_msc0_resources[] = {
	{
		.start          = CPHYSADDR(MSC0_BASE),
		.end            = CPHYSADDR(MSC0_BASE) + 0x1000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	{
		.start          = IRQ_MSC0,
		.end            = IRQ_MSC0,
		.flags          = IORESOURCE_IRQ,
	},
	{
		.start          = DMA_ID_MSC0_RX,
		.end            = DMA_ID_MSC0_TX,
		.flags          = IORESOURCE_DMA,
	},
};

static u64 jz_msc0_dmamask =  ~(u32)0;

static struct platform_device jz_msc0_device = {
	.name = "jz-msc",
	.id = 0,
	.dev = {
		.dma_mask               = &jz_msc0_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(jz_msc0_resources),
	.resource       = jz_msc0_resources,
};

/** MMC/SD controller MSC1**/
static struct resource jz_msc1_resources[] = {
	{
		.start          = CPHYSADDR(MSC1_BASE),
		.end            = CPHYSADDR(MSC1_BASE) + 0x1000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	{
		.start          = IRQ_MSC1,
		.end            = IRQ_MSC1,
		.flags          = IORESOURCE_IRQ,
	},
	{
		.start          = DMA_ID_MSC1_RX,
		.end            = DMA_ID_MSC1_TX,
		.flags          = IORESOURCE_DMA,
	},

};

static u64 jz_msc1_dmamask =  ~(u32)0;

static struct platform_device jz_msc1_device = {
	.name = "jz-msc",
	.id = 1,
	.dev = {
		.dma_mask               = &jz_msc1_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(jz_msc1_resources),
	.resource       = jz_msc1_resources,
};

/** MMC/SD controller MSC2**/
static struct resource jz_msc2_resources[] = {
	{
		.start          = CPHYSADDR(MSC2_BASE),
		.end            = CPHYSADDR(MSC2_BASE) + 0x1000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	{
		.start          = IRQ_MSC2,
		.end            = IRQ_MSC2,
		.flags          = IORESOURCE_IRQ,
	},
	{
		.start          = DMA_ID_MSC2_RX,
		.end            = DMA_ID_MSC2_TX,
		.flags          = IORESOURCE_DMA,
	},

};

static u64 jz_msc2_dmamask =  ~(u32)0;

static struct platform_device jz_msc2_device = {
	.name = "jz-msc",
	.id = 2,
	.dev = {
		.dma_mask               = &jz_msc2_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(jz_msc2_resources),
	.resource       = jz_msc2_resources,
};

static struct platform_device *jz_msc_devices[] __initdata = {
	&jz_msc0_device,
	&jz_msc1_device,
	&jz_msc2_device,
};

int __init jz_add_msc_devices(unsigned int controller, struct jz_mmc_platform_data *plat)
{
	struct platform_device	*pdev;

	if (controller < 0 || controller > 2)
		return -EINVAL;

	pdev = jz_msc_devices[controller];

	pdev->dev.platform_data = plat;

	return platform_device_register(pdev);
}

static struct resource jz_i2c0_resources[] = {
	[0] = {
		.start          = CPHYSADDR(I2C0_BASE),
		.end            = CPHYSADDR(I2C0_BASE) + 0x1000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	[1] = {
		.start          = IRQ_I2C0,
		.end            = IRQ_I2C0,
		.flags          = IORESOURCE_IRQ,
	},
};

static struct resource jz_i2c1_resources[] = {
	[0] = {
		.start          = CPHYSADDR(I2C1_BASE),
		.end            = CPHYSADDR(I2C1_BASE) + 0x1000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	[1] = {
		.start          = IRQ_I2C1,
		.end            = IRQ_I2C1,
		.flags          = IORESOURCE_IRQ,
	},
};

static struct resource jz_i2c2_resources[] = {
	[0] = {
		.start          = CPHYSADDR(I2C2_BASE),
		.end            = CPHYSADDR(I2C2_BASE) + 0x1000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	[1] = {
		.start          = IRQ_I2C2,
		.end            = IRQ_I2C2,
		.flags          = IORESOURCE_IRQ,
	},
};

static u64 jz_i2c_dmamask =  ~(u32)0;

static struct platform_device jz_i2c0_device = {
	.name = "jz_i2c0",
	.id = 0,
	.dev = {
		.dma_mask               = &jz_i2c_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(jz_i2c0_resources),
	.resource       = jz_i2c0_resources,
};

static struct platform_device jz_i2c1_device = {
	.name = "jz_i2c1",
	.id = 1,
	.dev = {
		.dma_mask               = &jz_i2c_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(jz_i2c1_resources),
	.resource       = jz_i2c1_resources,
};

static struct platform_device jz_i2c2_device = {
	.name = "jz_i2c2",
	.id = 2,
	.dev = {
		.dma_mask               = &jz_i2c_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(jz_i2c2_resources),
	.resource       = jz_i2c2_resources,
};

struct platform_device jz4770_rtc_device = {
	.name		= "jz4770-rtc",
	.id		= -1,
};

#ifdef CONFIG_GPIO_HP_DETECT
__attribute__ ((weakref))struct gpio_switch_platform_data jz_hp_switch_data = {
	.name		= "h2w",
	.gpio		= GPIO_HEAD_DET,
	.state_on	= "1",
	.state_off	= "0",
	.valid_level    = ACTIVE_LEVEL_HEAD_DET,
};

__attribute__ ((weakref))struct platform_device jz_hp_switch_device = {
	.name		= "switch-gpio",
	.id		= -1,
	.dev		= {
		.platform_data = &jz_hp_switch_data,
	},
};

#ifdef CONFIG_HP_SENSE_DETECT
#error "you can not define both (CONFIG_HP_SENSE_DETECT and CONFIG_GPIO_HP_DETECT), only one can be select!"
#endif

#else  /* if not define CONFIG_GPIO_HP_DETECT */ 

#ifdef CONFIG_HP_SENSE_DETECT
static jz_hp_switch_platform_data_t jz_hp_sense_data = {
	.name		= "h2w",
	.state_on	= "1",
	.state_off	= "0",
};

static struct platform_device jz_hp_sense_device = {
	.name		= "hp-switch",
	.id		= -1,
	.dev		= {
		.platform_data = &jz_hp_sense_data,
	},
};
#else /* if not define HP_SENSE_DETECT */
#endif

#endif

/* + Sound device */
#define SND(num, desc) { .name = desc, .id = num }
static struct snd_endpoint snd_endpoints_list[] = {
	SND(0, "HANDSET"),
	SND(1, "SPEAKER"),
	SND(2, "HEADSET"),
};
#undef SND

static struct msm_snd_endpoints jz_snd_endpoints = {
	.endpoints = snd_endpoints_list,
	.num = ARRAY_SIZE(snd_endpoints_list),
};

static struct platform_device jz_snd_device = {
	.name = "mixer",
	.id = -1,
	.dev = {
		.platform_data = &jz_snd_endpoints,
	},
};
/* - Sound device */

#ifdef CONFIG_GPIO_DOCK_DETECT
static struct gpio_switch_platform_data jz_dock_switch_data = {
	.name		= "dock",
	.gpio		= GPIO_DOCK_DET,
	.state_on	= "1",
	.state_off	= "0",
	.valid_level    = 0,
};

static struct platform_device jz_dock_switch_device = {
	.name		= "switch-dock",
	.id		= -1,
	.dev		= {
		.platform_data = &jz_dock_switch_data,
	},
};
#endif

#if defined(CONFIG_SPI_GPIO)
static struct spi_gpio_platform_data jz4760_spi_gpio_data = {
	.sck	= GPIO_SSI1_CLK,
	.mosi	= GPIO_SSI1_DT,
	.miso	= GPIO_SSI1_DR,
	.num_chipselect	= 2,
};

static struct platform_device jz4760_spi_gpio_device = {
	.name	= "spi_gpio",
	.dev	= {
		.platform_data = &jz4760_spi_gpio_data,
	},
};
#endif

#if defined(CONFIG_WIFI_BRCM)

#include <linux/wlan_plat.h>

static char dhd_prealloc_buf_prot[16 * 1024];
static char dhd_prealloc_buf_rx[16 * 1024];
static char dhd_prealloc_buf_data[32 * 1024];
static char dhd_prealloc_buf_osl[150 * 1024];
static char dhd_prealloc_buf_defalut[1 * 1024];

enum wifi_prealloc_index {
	DHD_PREALLOC_PROT = 0,
	DHD_PREALLOC_RXBUF,
	DHD_PREALLOC_DATABUF,
	DHD_PREALLOC_OSL_BUF
};

static void *wifi_memory_prealloc(int section, unsigned long size)
{
	switch(section) {
	case DHD_PREALLOC_PROT:
		return dhd_prealloc_buf_prot;
		break;
	case DHD_PREALLOC_RXBUF:
		return dhd_prealloc_buf_rx;
		break;
	case DHD_PREALLOC_DATABUF:
		return dhd_prealloc_buf_data;
		break;
	case DHD_PREALLOC_OSL_BUF:
		return dhd_prealloc_buf_osl;
		break;
	default:
		return dhd_prealloc_buf_defalut;
		break;
	}
}

static struct wifi_platform_data wifi_brcm_data = {
	.mem_prealloc = wifi_memory_prealloc,
};

static struct platform_device wifi_brcm_device = {
	.name = "bcm4329_wlan",
	.id = -1,
	.dev = {
		.platform_data = &wifi_brcm_data,
	},
};
#endif

/* All */
static struct platform_device *jz_platform_devices[] __initdata = {
	&jz4770_rtc_device,
	&jz_i2c0_device,
	&jz_i2c1_device,
	&jz_i2c2_device,
	&jz_usb_ohci_device,
	&jz_musb_device,
	&jz_snd_device,
	&jz_lcd_device,
#if defined(CONFIG_SPI_GPIO)
	&jz4760_spi_gpio_device,
#endif
#ifdef CONFIG_GPIO_HP_DETECT
	&jz_hp_switch_device,
#endif
#ifdef CONFIG_HP_SENSE_DETECT
	&jz_hp_sense_device
#endif
#ifdef CONFIG_GPIO_DOCK_DETECT
	&jz_dock_switch_device,
#endif
#ifdef CONFIG_WIFI_BRCM
	&wifi_brcm_device,
#endif
};

static int __init jz_platform_init(void)
{
	int ret = platform_add_devices(jz_platform_devices, ARRAY_SIZE(jz_platform_devices));

	printk("jz_platform_init\n");
	board_msc_init();
	return ret;
}

arch_initcall(jz_platform_init);
