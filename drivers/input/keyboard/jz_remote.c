#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/slab.h>

#include <linux/gpio.h>
#include <mach/chip-gpio.h>
#include <asm/jzsoc.h>
#include "jz_remote.h"

#include <asm/gpio.h>
#include <asm/jzsoc.h>
/*power event button*/
#define  POWER				116
#define  ENDCALL			62
#define  ONESEC_TIMES			100
#define  SEC_NUM			1
#define  SLEEP_TIME			2	/*per 40ms*/

#define IR_back		18
#define IR_vol_down	1	
#define IR_vol_up	3
#define IR_up		5
#define IR_down		27
#define IR_left		7
#define IR_right	9
#define IR_menu		8
#define IR_play		30

//linux key tab :
#define BACK		158
#define VOL_DOWN	114
#define VOL_UP		115
#define UP		103
#define DOWN		108
#define LEFT		105
#define RIGHT		106
#define MENU		229	//59//
#define	PLAY		28	//enter		
/**********************************/
struct jz_remote *jz_pp_remote;

#define KEY_PHYS_NAME	"jz_remote_button/input0"

//key code tab
static unsigned char initkey_code[ ] = 
{
	BACK,VOL_DOWN,VOL_UP,UP,DOWN,LEFT,RIGHT,MENU,PLAY,
	ENDCALL	,KEY_WAKEUP
};


struct jz_remote_button {
	struct input_dev *input_dev;
	unsigned char keycodes[11];
};


struct jz_remote_button *ppjz__button;



static int jz_remote_ctrl_open(struct input_dev *dev)
{
	return 0;
}

static void jz_remote_ctrl_close(struct input_dev *dev)
{
}

void jz_remote_send_key( unsigned char key ) 
{
	uint8_t  send_key = 0;
	if(key==IR_back){
		send_key=BACK;
	}else if(key==IR_vol_down){
		send_key=VOL_DOWN;
	}else if(key==IR_vol_up){
		send_key=VOL_UP;
	}else if(key==IR_up){
		send_key=UP;
	}else if(key==IR_down){
		send_key=DOWN;
	}else if(key==IR_left){
		send_key=LEFT;
	}else if(key==IR_right){
		send_key=RIGHT;
	}else if(key==IR_menu){
		send_key=MENU;
	}else if(key==IR_play){
		send_key=PLAY;
	}
        input_report_key(ppjz__button->input_dev,send_key,1);
        input_sync(ppjz__button->input_dev);
        input_report_key(ppjz__button->input_dev,send_key,0);
        input_sync(ppjz__button->input_dev);
}

static char lianji_state=0;
static unsigned char curr_dat ,lianji_cnt=0;


void lianji_CallBack(unsigned long data)
{
	if(lianji_state==0){
		if(__gpio_get_pin(jz_pp_remote->RMC_INT)==0){
			while(!__gpio_get_pin(jz_pp_remote->RMC_INT));
			lianji_cnt=0;
			jz_pp_remote->timer.expires  = jiffies + usecs_to_jiffies(2500);
			add_timer(&jz_pp_remote->timer); 
			lianji_state=1;
		}else{
			lianji_cnt++;
			if(lianji_cnt>30){	//RE-PRESS TIME OUT
				enable_irq(jz_pp_remote->RMC_IRQ);
				lianji_state=0;
				lianji_cnt=0;
			}else{
				jz_pp_remote->timer.expires  = jiffies + usecs_to_jiffies(2000);
				add_timer(&jz_pp_remote->timer); 
			}
		}
	}else if(lianji_state==1){
		if(__gpio_get_pin(jz_pp_remote->RMC_INT)==0){
			//printk("\nlianji_start ++\n");
			lianji_cnt=0;
			jz_pp_remote->timer.expires  = jiffies + msecs_to_jiffies(100);
			add_timer(&jz_pp_remote->timer); 
			lianji_state=0;
#if CONFIG_JZ_REMOTE_SUPPORT_LONGPRESS
			jz_remote_send_key(curr_dat);
#endif

		}else{
			lianji_cnt++;
			if(lianji_cnt>30){
				enable_irq(jz_pp_remote->RMC_IRQ);
				lianji_state=0;
				lianji_cnt=0;
			}else{
				jz_pp_remote->timer.expires  = jiffies + usecs_to_jiffies(2000);
				add_timer(&jz_pp_remote->timer); 
			}
		}
	}
}
void wx_ir_handler(void)
{
	int i, m , n;
	uint8_t da[4]={0,0,0,0};
	for(i=0;i<10;i++){
		udelay(880);
		if(__gpio_get_pin(jz_pp_remote->RMC_INT)==1){	
			goto exit;
		}
	}
	while(!__gpio_get_pin(jz_pp_remote->RMC_INT));
	udelay(4700);
	for(m=0;m<4;m++){
		for(n=0;n<8;n++){	
			while(!__gpio_get_pin(jz_pp_remote->RMC_INT));
			udelay(840);
			if(__gpio_get_pin(jz_pp_remote->RMC_INT)==1){
				udelay(1000);
				da[m]=da[m]>>1;
				da[m]=da[m]|0x80;
			}else{
				da[m]=da[m]>>1;
				da[m]=da[m]|0x00;
			}
		}
	}
	if(da[2]!=0xff){
		curr_dat=da[2];
		jz_remote_send_key(da[2]);
		/********************************************************/
		//detect long-press and release
		{
			lianji_state=0;
			lianji_cnt=0;
			setup_timer(&jz_pp_remote->timer, lianji_CallBack, (unsigned long)jz_pp_remote);
			jz_pp_remote->timer.expires  = jiffies + msecs_to_jiffies(102);//41
			add_timer(&jz_pp_remote->timer);
			return;
		}
		/********************************************************/
	}
exit:
	enable_irq(jz_pp_remote->RMC_IRQ);
	lianji_state=0;
}

static irqreturn_t jz_remote_irq_handler(s32 irq, void *dev_id)
{
	disable_irq_nosync(jz_pp_remote->RMC_IRQ);
	wx_ir_handler();
    	return IRQ_HANDLED;
}

static int __devinit jz_remote_probe(struct platform_device *pdev)
{
	struct jz_remote_button *remote_button;
	struct input_dev *input_dev;
	int  error,i,ret;
	jz_pp_remote=  pdev->dev.platform_data;
	remote_button = kzalloc(sizeof(struct jz_remote_button), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!input_dev || !remote_button) {
		dev_err(&pdev->dev, "failed to allocate input device for remote ctrl \n");
		error = -ENOMEM;
		goto failed1;
	}
	__gpio_as_irq_fall_edge(jz_pp_remote->RMC_INT);	
	__gpio_enable_pull(jz_pp_remote->RMC_INT);	
	ret = request_irq(jz_pp_remote->RMC_IRQ, jz_remote_irq_handler, IRQF_DISABLED, "jz-remote", NULL);
	if (ret < 0) {

		printk(KERN_CRIT "Can't register IRQ for jz_remote__irq\n");
		return ret;
	}
	input_dev->name = pdev->name;
	input_dev->open = jz_remote_ctrl_open;
	input_dev->close = jz_remote_ctrl_close;
	input_dev->dev.parent = &pdev->dev;
	input_dev->phys = KEY_PHYS_NAME;
	input_dev->id.vendor = 0x0001;

	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;
	input_dev->keycodesize = sizeof(unsigned char);

	for (i = 0; i < ARRAY_SIZE(initkey_code); i++)
		set_bit(initkey_code[i], input_dev->keybit);
	clear_bit(0, input_dev->keybit);
	remote_button->input_dev = input_dev;
	input_set_drvdata(input_dev, remote_button);


	input_dev->evbit[0] = BIT_MASK(EV_KEY) ;

	platform_set_drvdata(pdev, remote_button);

	ppjz__button=remote_button;
	error = input_register_device(input_dev);
	if (error) {
		dev_err(&pdev->dev, "failed to register input jz_remote  device\n");
		goto failed2;
	}
	return 0;
failed2:
	input_unregister_device(remote_button->input_dev);
	platform_set_drvdata(pdev, NULL);	

failed1:
	input_free_device(input_dev);
	kfree(remote_button);
	return error;

}

static int __devexit jz_remote__remove(struct platform_device *pdev)
{
	struct jz_remote_button *remote_button = platform_get_drvdata(pdev);

	input_unregister_device(remote_button->input_dev);
	input_free_device(remote_button->input_dev);
	kfree(remote_button);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver jz_remote_driver = {
	.probe		= jz_remote_probe,
	.remove 	= __devexit_p(jz_remote__remove),
	.driver 	= {
		.name	= "jz-remote",
		.owner	= THIS_MODULE,
	},
};

 int __init jz_remote_init(void)
{
	return platform_driver_register(&jz_remote_driver);
}

static void __exit jz_remote_exit(void)
{
	platform_driver_unregister(&jz_remote_driver);
}

module_init(jz_remote_init);
module_exit(jz_remote_exit);

MODULE_DESCRIPTION("jz_remote__irq Driver");
MODULE_LICENSE("GPL");


