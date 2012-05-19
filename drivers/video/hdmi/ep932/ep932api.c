
#include <linux/module.h>
#include <linux/sched.h>
#include <asm/jzsoc.h>

#include <linux/kthread.h>
#include <linux/timer.h>
#include "ep932api.h"
#include "ep932controller.h"  // HDMI Transmiter
#include "edid.h"  // HDMI Transmiter

#if (defined CONFIG_JZ4760B_LYNX || defined CONFIG_JZ4760_LYNX)
#include <asm/jzpm/jz_act8930.h>
#define     EP932M_5VEN_PIN (32*5+6)
#define     EP932M_RESET_PIN (32*4+11)
int ep932m_5ven	= 1;
#else
#define     EP932M_RESET_PIN (32*4+11)
#endif

#define     IIC_EP932E 0x70
#define		USE_KTHREAD	1
#define 	HDMI_HGCHECK_DELAY  3*100  //3s
int hdmi_power_on= 0;
int hdmi_hg_out = 0;
struct task_struct *hdmi_kthread;

void ep_ep932m_reset(void)
{
	__gpio_as_output(EP932M_RESET_PIN);
	__gpio_clear_pin(EP932M_RESET_PIN);	// set to output low
	mdelay(10);
	__gpio_set_pin(EP932M_RESET_PIN);
	mdelay(10);
}

#if (defined CONFIG_JZ4760B_LYNX || defined CONFIG_JZ4760_LYNX)
void ep932m_power_on()
{
#ifdef CONFIG_JZ4760B_LYNX	
  __gpio_set_driving_strength(GPIO_LCD_PCLK_DS,GPIO_LCD_DS_HDMI	);
  __gpio_set_driving_strength(GPIO_LCD_HSYN_DS,GPIO_LCD_DS_HDMI	);
  __gpio_set_driving_strength(GPIO_LCD_VSYN_DS,GPIO_LCD_DS_HDMI	);
#endif
	ep932m_5ven = __gpio_get_pin(EP932M_5VEN_PIN);	
	if(!ep932m_5ven){
		printk("open ep932m_5ven \n");
		__gpio_as_output(EP932M_5VEN_PIN);
		__gpio_set_pin(EP932M_5VEN_PIN);
	}
//	__hdmi_power_on();
}
void ep932m_power_off()
{
#ifdef CONFIG_JZ4760B_LYNX	
	__gpio_set_driving_strength(GPIO_LCD_PCLK_DS,GPIO_LCD_DS_DEFAULT);
	__gpio_set_driving_strength(GPIO_LCD_HSYN_DS,GPIO_LCD_DS_DEFAULT);
	__gpio_set_driving_strength(GPIO_LCD_VSYN_DS,GPIO_LCD_DS_DEFAULT);
#endif
//	__hdmi_power_off();
	if(!ep932m_5ven){
		printk("close ep932m_5ven \n");
		__gpio_as_output(EP932M_5VEN_PIN);
		__gpio_clear_pin(EP932M_5VEN_PIN);
	}
}
#endif


unsigned char ep_hdmi_deinit(void)
{   
#if (defined CONFIG_JZ4760B_LYNX || defined CONFIG_JZ4760_LYNX)
	ep932m_power_off();
#endif
	return 0;
}

unsigned char ep_hdmi_init(void)
{
#if (defined CONFIG_JZ4760B_LYNX || defined CONFIG_JZ4760_LYNX)
	ep932m_power_on();
	DBG_PRINTK(("ep932controller_init\r\n"));
#endif
	return 0; //HDMI_SUCCESS;
}


static int hdmi_thread(void *data)
{
	while(!kthread_should_stop()){
	//	set_current_state(TASK_INTERRUPTIBLE);
	//	schedule_timeout(HDMI_HGCHECK_DELAY);
		msleep(HDMI_HGCHECK_DELAY * 10);
		ep932controller_task();
	}
	return 0;
}
void hdmi_init()
{
		ep932controller_initial();
	//	ep932controller_task();
	//	hdmi_kthread = kthread_run(hdmi_thread, NULL, "hdmi_run");       	
}

int hdmi_match_edid(int video_timing)
{
	ep_hdmi_init();				// Variables initial	
	return ep932controller_matchedid(video_timing);
}

void hdmi_start(int video_timing)
{	
	DBG_PRINTK(("hdmi_start\r\n"));
	if(!hdmi_power_on){
		ep_hdmi_init();				// Variables initial	
		ep932controller_start(video_timing);
#if USE_KTHREAD
		hdmi_kthread = kthread_run(hdmi_thread, NULL, "hdmi_run");
		if (IS_ERR(hdmi_kthread)) {
			printk(": Failed to create HDMI monitor thread.\n");
			PTR_ERR(hdmi_kthread);
		}
#endif
		printk(": create HDMI monitor thread.\n");
		hdmi_power_on = 1;
	}
}
void hdmi_stop()
{
	if(	hdmi_power_on){
#if	USE_KTHREAD
	   if(hdmi_kthread){
			kthread_stop(hdmi_kthread);
		}
#endif
		ep932controller_stop();
		printk(": stoped HDMI monitor thread.\n");
		ep_hdmi_deinit();
		hdmi_power_on = 0;
	}
}

EXPORT_SYMBOL(hdmi_init);
EXPORT_SYMBOL(hdmi_match_edid);
EXPORT_SYMBOL(hdmi_start);
EXPORT_SYMBOL(hdmi_stop);

