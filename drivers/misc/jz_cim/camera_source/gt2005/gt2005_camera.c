#include <asm/jzsoc.h>
#include <linux/i2c.h>
#include "gt2005_camera.h"
#include "gt2005_set.h"

//#define gt2005_DEBUG
#ifdef gt2005_DEBUG
#define dprintk(x...)   do{printk("gt2005---\t");printk(x);printk("\n");}while(0)
#else
#define dprintk(x...)
#endif

struct gt2005_sensor gt2005; 
static struct jz_cim_sensor_platform_data *pdata = NULL;
void gt2005_power_down(void)
{
	dprintk("===gt2005_power_down==\n");
	__gpio_as_output(pdata->gpio_en);
	__gpio_clear_pin(pdata->gpio_en);
	mdelay(20);
}    

void gt2005_power_up(void)
{ 
	dprintk("===gt2005_power_up==\n");
	__gpio_as_output(pdata->gpio_en);
	__gpio_set_pin(pdata->gpio_en);
	mdelay(20);
}

void gt2005_reset(void)
{
	__gpio_as_output(pdata->gpio_rst);
	__gpio_clear_pin(pdata->gpio_rst);
	mdelay(20);
	__gpio_set_pin(pdata->gpio_rst);
	mdelay(20);
}

int gt2005_set_balance(balance_flag_t balance_flag,int arg)
{
	dprintk("--------------------------------gt2005_set_balance");
	switch(balance_flag)
	{
		case WHITE_BALANCE_AUTO:
			gt2005_set_wb_auto(gt2005.client);
			break;
		case WHITE_BALANCE_INCANDESCENT :
			gt2005_set_wb_incandescence(gt2005.client);
			dprintk("WHITE_BALANCE_INCANDESCENT ");
			break;
		case WHITE_BALANCE_FLUORESCENT ://ying guang
			gt2005_set_wb_fluorescent(gt2005.client);
			dprintk("WHITE_BALANCE_FLUORESCENT ");
			break;
		case WHITE_BALANCE_WARM_FLUORESCENT :
			dprintk("WHITE_BALANCE_WARM_FLUORESCENT ");
			break;
		case WHITE_BALANCE_DAYLIGHT ://ri guang
			gt2005_set_wb_daylight(gt2005.client);
			dprintk("WHITE_BALANCE_DAYLIGHT ");
			break;
		case WHITE_BALANCE_CLOUDY_DAYLIGHT ://ying tian
			gt2005_set_wb_cloud(gt2005.client);
			dprintk("WHITE_BALANCE_CLOUDY_DAYLIGHT ");
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

int gt2005_set_effect(effect_flag_t effect_flag,int arg)
{
	dprintk("gt2005_set_effect");
	switch(effect_flag)
	{
		case EFFECT_NONE:
			gt2005_set_effect_normal(gt2005.client);
			break;
		case EFFECT_MONO :
			gt2005_set_effect_grayscale(gt2005.client);
			dprintk("EFFECT_MONO ");
			break;
		case EFFECT_NEGATIVE :
			gt2005_set_effect_colorinv(gt2005.client);
			dprintk("EFFECT_NEGATIVE ");
			break;
		case EFFECT_SOLARIZE ://bao guang
			dprintk("EFFECT_SOLARIZE ");
			break;
		case EFFECT_SEPIA :
			gt2005_set_effect_sepia(gt2005.client);
			dprintk("EFFECT_SEPIA ");
			break;
		case EFFECT_POSTERIZE ://se diao fen li
			dprintk("EFFECT_POSTERIZE ");
			break;
		case EFFECT_WHITEBOARD :
			dprintk("EFFECT_WHITEBOARD ");
			break;
		case EFFECT_BLACKBOARD :
			dprintk("EFFECT_BLACKBOARD ");
			break;
		case EFFECT_PASTEL:
			dprintk("EFFECT_PASTEL");
			break;
		case EFFECT_MOSAIC:
			dprintk("EFFECT_MOSAIC");
			break;
		case EFFECT_RESIZE:
			dprintk("EFFECT_RESIZE");
			break;
	}
	return 0;
}

int gt2005_set_antibanding(antibanding_flag_t antibanding_flag,int arg)
{
	dprintk("gt2005_set_antibanding");
	switch(antibanding_flag)
	{
		case ANTIBANDING_AUTO :
			break;
		case ANTIBANDING_50HZ :
			gt2005_set_ab_50hz(gt2005.client);
			dprintk("ANTIBANDING_50HZ ");
			break;
		case ANTIBANDING_60HZ :
			gt2005_set_ab_60hz(gt2005.client);
			dprintk("ANTIBANDING_60HZ ");
			break;
		case ANTIBANDING_OFF :
			dprintk("ANTIBANDING_OFF ");
			break;
	}
	return 0;
}

int gt2005_set_flash_mode(flash_mode_flag_t flash_mode_flag,int arg)
{
	return 0;
}

int gt2005_set_scene_mode(scene_mode_flag_t scene_mode_flag,int arg)
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

int gt2005_set_focus_mode(focus_mode_flag_t flash_mode_flag,int arg)
{
	return 0;
}

int gt2005_set_night_mode(int enable)
{
	return 0;
}

int gt2005_set_fps(int fps)
{
	gt2005_set_f_p_s(gt2005.client,fps);
	return 0;
}
int gt2005_set_parameter(int cmd, int mode, int arg)
{
	switch(cmd)
	{
		case CPCMD_SET_BALANCE :
			gt2005_set_balance(mode,arg);
			break;
		case CPCMD_SET_EFFECT :
			gt2005_set_effect(mode,arg);
			break;
		case CPCMD_SET_ANTIBANDING :
			gt2005_set_antibanding(mode,arg);
			break;
		case CPCMD_SET_FLASH_MODE :
			gt2005_set_flash_mode(mode,arg);
			break;
		case CPCMD_SET_SCENE_MODE :
			gt2005_set_scene_mode(mode,arg);
			break;
		case CPCMD_SET_PIXEL_FORMAT :
			break;
		case CPCMD_SET_FOCUS_MODE :
			break;
		case CPCMD_SET_PREVIEW_FPS:
			gt2005_set_fps(arg);
			break;
		case CPCMD_SET_NIGHTSHOT_MODE:
			gt2005_set_nightmode(gt2005.client,arg);
			break;
		case CPCMD_SET_LUMA_ADAPTATION:
			break;
	}
	return 0;
}

int gt2005_set_power(int state)
{
	switch (state)
	{
		case 0:           
			/* hardware power up first */
			gt2005_power_up();
			/* software power up later if it implemented */
			break;
		case 1:
			gt2005_power_down();
			break;
		case 2:
			break;
		default:
			printk("%s : EINVAL! \n",__FUNCTION__);
	}
	return 0;
}

int gt2005_sensor_init(void)
{
    gt2005_reset();
	gt2005_init_setting(gt2005.client);
	return 0;
}

int gt2005_sensor_probe(void)
{
	gt2005_reset();
	gt2005_power_up();


	int sensor_id = 0;
	sensor_id = (sensor_read_reg16(gt2005.client,0x0000) << 8) | sensor_read_reg16(gt2005.client,0x0001);
	printk("-----------------------------gt2005 read is 0x%04x\n",sensor_id);

	gt2005_power_down();
	if(sensor_id == 0x5138)
		return 0; 
	return -1;
}

/* sensor_set_function use for init preview or capture.there may be some difference between preview and capture.
 * so we divided it into two sequences.param: function indicated which function
 * 0: preview
 * 1: capture
 * 2: recording
 */
int gt2005_set_function(int function)
{

	switch (function)
	{
		case 0:
			gt2005_preview_set(gt2005.client);
			break;
		case 1:
			gt2005_capture_set(gt2005.client);
			break;
		case 2:		
			break;
	}
	return 0;
}

int gt2005_set_resolution(int width,int height,int bpp,pixel_format_flag_t fmt,camera_mode_t mode)
{
  	gt2005_size_switch(gt2005.client,width,height,mode);
	return 0;         
}

static ssize_t gt2005_write_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	uint32_t reg,val;
	sscanf(buf,"%x,%x",&reg,&val);
	printk("write reg : %x | value : %x\n",reg&0xffff,val&0xff);
	sensor_write_reg16(gt2005.client,reg&0xffff,val&0xff);

	return count;
}

static ssize_t gt2005_read_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	uint32_t reg;
	sscanf(buf,"%x",&reg);
	printk("read reg 0x%x = 0x%x\n",reg&0xffff,sensor_read_reg16(gt2005.client,reg&0xffff));

	return count;
}


static DEVICE_ATTR(write, 0664, NULL, gt2005_write_store);
static DEVICE_ATTR(read, 0664, NULL, gt2005_read_store);

static struct attribute *gt2005_attributes[] = {
	&dev_attr_write.attr,
	&dev_attr_read.attr,
	NULL
};

static const struct attribute_group gt2005_attr_group = {
	.attrs = gt2005_attributes,
};

static int gt2005_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	gt2005.client = client;
	pdata = client->dev.platform_data;
	if(pdata==NULL){
		printk("err!!! no camera i2c pdata!!! \n\n");
		return -1;
	}
	gt2005.desc.facing = pdata->facing;
	gt2005.desc.orientation = pdata->orientation;
	sysfs_create_group(&client->dev.kobj, &gt2005_attr_group);
	sensor_set_i2c_speed(client,300000);//set i2c speed : 300khz
	return camera_sensor_register(&gt2005.desc);
}

struct camera_sensor_ops gt2005_sensor_ops = {
	.sensor_init = gt2005_sensor_init,
	.camera_sensor_probe = gt2005_sensor_probe,
	.sensor_set_function = gt2005_set_function,
	.sensor_set_resolution = gt2005_set_resolution, 
	.sensor_set_parameter = gt2005_set_parameter, 
	.sensor_set_power = gt2005_set_power,
};

struct resolution_info gt2005_resolution_table[] = {	


    {1600, 1200, 16,PIXEL_FORMAT_YUV422I},
    {1280, 720, 16,PIXEL_FORMAT_YUV422I},
    {1024, 768, 16,PIXEL_FORMAT_YUV422I},
    {800, 600, 16,PIXEL_FORMAT_YUV422I},
    {720, 576, 16,PIXEL_FORMAT_YUV422I},
    {720, 480, 16,PIXEL_FORMAT_YUV422I},
    {640, 480, 16,PIXEL_FORMAT_YUV422I},
    {352, 288,16,PIXEL_FORMAT_YUV422I}, //must be included
    {320, 240, 16,PIXEL_FORMAT_YUV422I},
    {176,144,16,PIXEL_FORMAT_YUV422I}, /**/ //must be included in order to recording 
};

struct gt2005_sensor gt2005 = { 
    .desc = {
        .name = "gt2005",
        .wait_frames = 2,
        .ops = &gt2005_sensor_ops,
        .resolution_table = gt2005_resolution_table,
        .resolution_table_nr=ARRAY_SIZE(gt2005_resolution_table),
#if 1
        .capture_parm = {1600, 1200, 16,PIXEL_FORMAT_YUV422I},
        .max_capture_parm = {1600, 1200, 16,PIXEL_FORMAT_YUV422I},
#else
        .capture_parm = {1600, 1200, 16,PIXEL_FORMAT_YUV422I},
        .max_capture_parm = {1600, 1200, 16,PIXEL_FORMAT_YUV422I},
#endif

        .preview_parm = {640,480, 16,PIXEL_FORMAT_YUV422I}, 
        .max_preview_parm = {1280,720,  16,PIXEL_FORMAT_YUV422I},
        .cfg_info = {
            .configure_register= 0x0
             // |CIM_CFG_PACK_4			/* pack mode : 4 3 2 1 */
                |CIM_CFG_BYPASS			/* Bypass Mode */	
                //	|CIM_CFG_VSP             	/* VSYNC Polarity:1-falling edge active */
                //	|CIM_CFG_HSP 
                //	|CIM_CFG_PCP             	/* PCLK working edge:1-falling */
                |CIM_CFG_DSM_GCM,		/* Gated Clock Mode */
            .format =  SENSOR_OUTPUT_FORMAT_UY0VY1,
        },
        .flags = {
            .effect_flag = 0
                |EFFECT_NONE
                |EFFECT_MONO
                |EFFECT_SEPIA
                |EFFECT_NEGATIVE,
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

static const struct i2c_device_id gt2005_id[] = {
    { "gt2005", 0 },
    { }	/* Terminating entry */
};
MODULE_DEVICE_TABLE(i2c, gt2005_id);

static struct i2c_driver gt2005_driver = {
    .probe		= gt2005_i2c_probe,
    .id_table	= gt2005_id,
    .driver	= {
        .name = "gt2005",
    },
};

static int __init gt2005_i2c_register(void)
{
    return i2c_add_driver(&gt2005_driver);
}

module_init(gt2005_i2c_register);
