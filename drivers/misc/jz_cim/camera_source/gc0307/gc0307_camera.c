#include <asm/jzsoc.h>
#include <linux/i2c.h>
#include "gc0307_camera.h"
#include "gc0307_set.h"
#include "gc0307_set_mode.h"

#define gc0307_DEBUG
#ifdef gc0307_DEBUG
#define dprintk(x...)   do{printk("gc0307---\t");printk(x);printk("\n");}while(0)
#else
#define dprintk(x...)
#endif


struct gc0307_sensor gc0307;
static struct jz_cim_sensor_platform_data *pdata = NULL;

void gc0307_power_down(void)
{
	printk("===gc0307_power_down==\n");
	__gpio_as_output(pdata->gpio_en);
	__gpio_set_pin(pdata->gpio_en);
	mdelay(50);
}    

void gc0307_power_up(void)
{ 
	printk("===gc0307_power_up==\n");
	__gpio_as_output(pdata->gpio_en);
	__gpio_clear_pin(pdata->gpio_en);
	mdelay(50);
}

void gc0307_reset(void)
{
	__gpio_as_output(pdata->gpio_rst);
	mdelay(250);
	__gpio_clear_pin(pdata->gpio_rst);
	mdelay(250);
	__gpio_set_pin(pdata->gpio_rst);
	mdelay(250);
}

int gc0307_set_balance(balance_flag_t balance_flag,int arg)
{
	dprintk("gc0307_set_balance");
	switch(balance_flag)
	{
		case WHITE_BALANCE_AUTO:
			gc0307_set_wb_auto_mode(gc0307.client);
			dprintk("wb_auto ");
			break;
		case WHITE_BALANCE_INCANDESCENT :
			gc0307_set_wb_office_mode(gc0307.client);
			dprintk("wb_INCANDESCENT ");
			break;

		case WHITE_BALANCE_FLUORESCENT :
			gc0307_set_wb_home_mode(gc0307.client);
			dprintk("wb_FLUORESCENT ");
			break;
		case WHITE_BALANCE_WARM_FLUORESCENT :
			
			dprintk("wb_FLUORESCENT ");
			break;
		case WHITE_BALANCE_DAYLIGHT ://ri guang
			gc0307_set_wb_sunny_mode(gc0307.client);
			dprintk("wb_daylight ");
			break;
		case WHITE_BALANCE_CLOUDY_DAYLIGHT ://ying tian
			gc0307_set_wb_cloudy_mode(gc0307.client);
			dprintk("wb_cloudy daylight ");
			break;
		case WHITE_BALANCE_TWILIGHT :
			dprintk("WHITE_BALANCE_TWILIGHT ");
			break;
		case WHITE_BALANCE_SHADE :
			dprintk("WHITE_BALANCE_SHADE ");
			break;
	}
	return 0;
}

int gc0307_set_effect(effect_flag_t effect_flag,int arg)
{
	dprintk("gc0307_set_effect");
	switch(effect_flag)
	{
		case EFFECT_NONE:
			gc0307_set_effect_normal(gc0307.client);
			dprintk("effect_none");
			break;
		case EFFECT_MONO :
			gc0307_set_effect_blackwhite(gc0307.client);  
			dprintk("effect_mono ");
			break;
		case EFFECT_NEGATIVE :
			gc0307_set_effect_negative(gc0307.client);
			dprintk("effect_negative ");
			break;
		case EFFECT_SOLARIZE ://bao guang
			dprintk("effect_solarize ");
			break;
		case EFFECT_SEPIA :
			gc0307_set_effect_sepia(gc0307.client);
			dprintk("effect_sepia ");
			break;
		case EFFECT_POSTERIZE ://se diao fen li
			dprintk("effect_posterize ");
			break;
		case EFFECT_WHITEBOARD :
			dprintk("effect_whiteboard ");
			break;
		case EFFECT_BLACKBOARD :
			dprintk("effect_blackboard ");
			break;
		case EFFECT_AQUA ://qian lv se
			gc0307_set_effect_greenish(gc0307.client);
			dprintk("effect_aqua  ");
			break;
		case EFFECT_PASTEL:
			dprintk("effect_pastel");
			break;
		case EFFECT_MOSAIC:
			dprintk("effect_mosaic");
			break;
		case EFFECT_RESIZE:
			dprintk("effect_resize");
			break;
	}
	return 0;
}

int gc0307_set_antibanding(int antibanding_flag,int arg)
{
	dprintk("gc0307_set_antibanding");
	switch(antibanding_flag)
	{
		case ANTIBANDING_AUTO :
			gc0307_ab_auto(gc0307.client);
			dprintk("ANTIBANDING_AUTO ");
			break;
		case ANTIBANDING_50HZ :
			gc0307_ab_50hz(gc0307.client);
			dprintk("ANTIBANDING_50HZ ");
			break;
		case ANTIBANDING_60HZ :
			gc0307_ab_60hz(gc0307.client);
			dprintk("ANTIBANDING_60HZ ");
			break;
		case ANTIBANDING_OFF :
			gc0307_ab_off(gc0307.client);
			dprintk("ANTIBANDING_OFF ");
			break;
	}
	return 0;
}


int gc0307_set_flash_mode(flash_mode_flag_t flash_mode_flag,int arg)
{
	return 0;
}

int gc0307_set_scene_mode(scene_mode_flag_t scene_mode_flag,int arg)
{
	switch(scene_mode_flag)
	{
		case SCENE_MODE_AUTO :
			dprintk("SCENE_MODE_AUTO ");
			break;
		case SCENE_MODE_ACTION :
			dprintk("SCENE_MODE_ACTION ");
			break;
		case SCENE_MODE_PORTRAIT   :
			dprintk("SCENE_MODE_PORTRAIT   ");
			break;
		case SCENE_MODE_LANDSCAPE  :
			dprintk("SCENE_MODE_LANDSCAPE  ");
			break;
		case SCENE_MODE_NIGHT     :
			dprintk("SCENE_MODE_NIGHT     ");
			break;
		case SCENE_MODE_NIGHT_PORTRAIT   :
			dprintk("SCENE_MODE_NIGHT_PORTRAIT   ");
			break;
		case SCENE_MODE_THEATRE  :
			dprintk("SCENE_MODE_THEATRE  ");
			break;
		case SCENE_MODE_BEACH   :
			dprintk("SCENE_MODE_BEACH   ");
			break;
		case SCENE_MODE_SNOW    :
			dprintk("SCENE_MODE_SNOW    ");
			break;
		case SCENE_MODE_SUNSET    :
			dprintk("SCENE_MODE_SUNSET    ");
			break;
		case SCENE_MODE_STEADYPHOTO   :
			dprintk("SCENE_MODE_STEADYPHOTO   ");
			break;
		case SCENE_MODE_FIREWORKS    :
			dprintk("SCENE_MODE_FIREWORKS    ");
			break;
		case SCENE_MODE_SPORTS    :
			dprintk("SCENE_MODE_SPORTS    ");
			break;
		case SCENE_MODE_PARTY   :
			dprintk("SCENE_MODE_PARTY   ");
			break;
		case SCENE_MODE_CANDLELIGHT    :
			dprintk("SCENE_MODE_CANDLELIGHT    ");
			break;
	}
	return 0;
}

int gc0307_set_focus_mode(focus_mode_flag_t flash_mode_flag,int arg)
{
	return 0;
}


int gc0307_set_fps(int fps)
{
	dprintk("set fps : %d",fps);
	return 0;
}

int gc0307_set_luma_adaptation(int arg)
{
	dprintk("luma_adaptation : %d",arg);
	return 0;
}

int gc0307_set_parameter(int cmd, int mode, int arg)
{
	switch(cmd)
	{
		case CPCMD_SET_BALANCE :
			gc0307_set_balance(mode,arg);
			break;
		case CPCMD_SET_EFFECT :
			gc0307_set_effect(mode,arg);
			break;
		case CPCMD_SET_ANTIBANDING :
			gc0307_set_antibanding(mode,arg);
			break;
		case CPCMD_SET_FLASH_MODE :
			gc0307_set_flash_mode(mode,arg);
			break;
		case CPCMD_SET_SCENE_MODE :
			gc0307_set_scene_mode(mode,arg);
			break;
		case CPCMD_SET_PIXEL_FORMAT :
			break;
		case CPCMD_SET_FOCUS_MODE :
			gc0307_set_focus_mode(mode,arg);
			break;
		case CPCMD_SET_PREVIEW_FPS:
			gc0307_set_fps(arg);
			break;
		case CPCMD_SET_NIGHTSHOT_MODE:
			gc0307_set_nightmode(gc0307.client,mode);
			break;
		case CPCMD_SET_LUMA_ADAPTATION:
			gc0307_set_luma_adaptation(arg);
			break;
	}
	return 0;
}

int gc0307_set_power(int state)
{
	switch (state)
	{
		case 0:           
			/* hardware power up first */
			gc0307_power_up();
			/* software power up later if it implemented */
			break;
		case 1:
			gc0307_power_down();
			break;
		case 2:
			break;
		default:
			break;
	}
	return 0;
}

int gc0307_sensor_init(void)
{
	//gc0307_reset();
	gc0307_init_setting(gc0307.client);
	return 0;
}

int gc0307_sensor_probe(void)
{
	int sensor_id = 0;

	gc0307_power_up();
	gc0307_reset();
	sensor_id = sensor_read_reg(gc0307.client,0x00);
	printk("-----------------------------gc0307 read is %d\n",sensor_id);
	gc0307_power_down();
	printk("===GC0307 ID:%2x===\n", sensor_id);
	if(sensor_id == 0x99)
		return 0; 
	return -1;
}

/* sensor_set_function use for init preview or capture.there may be some difference between preview and capture.
 * so we divided it into two sequences.param: function indicated which function
 * 0: preview
 * 1: capture
 * 2: recording
 */
int gc0307_set_function(int function,void *cookie)
{
	switch (function)
	{
		case 0:
			preview_set(gc0307.client);
			break;
		case 1:
			capture_set(gc0307.client);
			break;
		case 2:		
			break;
	}
	return 0;
}

int gc0307_set_resolution(int width,int height,int bpp,pixel_format_flag_t fmt,camera_mode_t mode)
{
	size_switch(gc0307.client,width,height,mode);
	return 0;         
}

static ssize_t gc0307_write_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	uint32_t reg,val;
	sscanf(buf,"%x,%x",&reg,&val);
	printk("write reg : %x | value : %x\n",reg&0xff,val&0xff);
	sensor_write_reg(gc0307.client,reg&0xff,val&0xff);

	return count;
}

static ssize_t gc0307_read_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	uint32_t reg;
	sscanf(buf,"%x",&reg);
	printk("read reg 0x%x = 0x%x\n",reg&0xff,sensor_read_reg(gc0307.client,reg&0xff));

	return count;
}


static DEVICE_ATTR(write, 0664, NULL, gc0307_write_store);
static DEVICE_ATTR(read, 0664, NULL, gc0307_read_store);

static struct attribute *gc0307_attributes[] = {
	&dev_attr_write.attr,
	&dev_attr_read.attr,
	NULL
};

static const struct attribute_group gc0307_attr_group = {
	.attrs = gc0307_attributes,
};

static int gc0307_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = -1;
	gc0307.client = client;
	pdata = client->dev.platform_data;
	if(pdata==NULL){
		printk("err!!! no camera i2c pdata!!! \n\n");
		return -1;
	}
	gc0307.desc.facing = pdata->facing;
	gc0307.desc.orientation = pdata->orientation;

	ret = sysfs_create_group(&client->dev.kobj, &gc0307_attr_group);
	sensor_set_i2c_speed(client,300000);
	return camera_sensor_register(&gc0307.desc);
}

struct camera_sensor_ops gc0307_sensor_ops = {
	.sensor_init = gc0307_sensor_init,
	.camera_sensor_probe = gc0307_sensor_probe,
	.sensor_set_function = gc0307_set_function,
	.sensor_set_resolution = gc0307_set_resolution, 
	.sensor_set_parameter = gc0307_set_parameter, 
	.sensor_set_power = gc0307_set_power,
};

struct resolution_info gc0307_resolution_table[] = {	
	{640,480,16,PIXEL_FORMAT_YUV422I},
	{352,288,16,PIXEL_FORMAT_YUV422I},  //must be included
	{320,240,16,PIXEL_FORMAT_YUV422I},  //must be included
	{176,144,16,PIXEL_FORMAT_YUV422I},  //must be included in order to recording 
};

struct gc0307_sensor gc0307 = {
	.desc = {
		.name = "gc0307",
		.wait_frames = 2,

		.ops = &gc0307_sensor_ops,

		.resolution_table = gc0307_resolution_table,
		.resolution_table_nr=ARRAY_SIZE(gc0307_resolution_table),

		.capture_parm = {640,480, 16,PIXEL_FORMAT_YUV422I},
		.max_capture_parm = {640,480, 16,PIXEL_FORMAT_YUV422I},

		.preview_parm = {640,480, 16,PIXEL_FORMAT_YUV422I}, 
		.max_preview_parm = {640,480, 16,PIXEL_FORMAT_YUV422I},

		.cfg_info = {
			.configure_register= 0x0
			//CIM_CFG_PACK_2			/* pack mode : 4 3 2 1 */
				|CIM_CFG_BYPASS			/* Bypass Mode */	
			//			|CIM_CFG_VSP             	/* VSYNC Polarity:1-falling edge active */
			//     |CIM_CFG_HSP 
				|CIM_CFG_PCP             	/* PCLK working edge:1-falling */
				|CIM_CFG_DSM_GCM,		/* Gated Clock Mode */
			.format = SENSOR_OUTPUT_FORMAT_Y0UY1V,
		},

		.flags = {
			.effect_flag = 0
				|EFFECT_NONE
				|EFFECT_MONO
				|EFFECT_SEPIA
				|EFFECT_NEGATIVE 
				|EFFECT_AQUA,
			.balance_flag = 0
				| WHITE_BALANCE_AUTO 
				| WHITE_BALANCE_DAYLIGHT 
				| WHITE_BALANCE_CLOUDY_DAYLIGHT 
				| WHITE_BALANCE_INCANDESCENT
				| WHITE_BALANCE_FLUORESCENT,
			.antibanding_flag = ~0x0,
			.flash_mode_flag = 0,
			.scene_mode_flag = 0,
			.pixel_format_flag = 0,
			.focus_mode_flag = 0,
		},

	},
};

static const struct i2c_device_id gc0307_id[] = {
	{ "gc0307", 0 },
	{ }	/* Terminating entry */
};
MODULE_DEVICE_TABLE(i2c, gc0307_id);

static struct i2c_driver gc0307_driver = {
	.probe		= gc0307_i2c_probe,
	.id_table	= gc0307_id,
	.driver	= {
		.name = "gc0307",
	},
};

static int __init gc0307_i2c_register(void)
{
	return i2c_add_driver(&gc0307_driver);
}

module_init(gc0307_i2c_register);
