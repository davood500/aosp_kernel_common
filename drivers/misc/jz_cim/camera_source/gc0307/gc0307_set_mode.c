#include <asm/jzsoc.h>
#include "../../jz_sensor.h"
#include <linux/i2c.h>

#define gc0307_DEBUG
#ifdef gc0307_DEBUG
#define dprintk(x...)   do{printk("cm3511---\t");printk(x);printk("\n");}while(0)
#else
#define dprintk(x...)
#endif


static void night_enable(struct i2c_client *client)
{
	sensor_write_reg(client,0xdd  ,0x32);
}

static void night_disable(struct i2c_client *client)
{
	sensor_write_reg(client,0xdd  ,0x22);
}

void gc0307_set_nightmode(struct i2c_client *client,int enable)
{
	if(enable){
		night_enable(client);
	}else{
		night_disable(client);
	}
	return 0;
}

/*----------------------------------set white balance------------------------------------------------*/

void gc0307_set_wb_auto_mode(struct i2c_client *client)
{
	unsigned ret = (sensor_read_reg(client,0x41) |0x04);
	sensor_write_reg(client,0x41,ret);
	sensor_write_reg(client,0xc7,0x4c);
	sensor_write_reg(client,0xc8,0x40);
	sensor_write_reg(client,0xc9,0x4a);
}

void gc0307_set_wb_sunny_mode(struct i2c_client *client)
{
	unsigned ret = (sensor_read_reg(client,0x41) & ~0x04);
	sensor_write_reg(client,0x41,ret);
	sensor_write_reg(client,0xc7,0x50);
	sensor_write_reg(client,0xc8,0x45);
	sensor_write_reg(client,0xc9,0x40);
	
}

void gc0307_set_wb_cloudy_mode(struct i2c_client *client)
{ 
	unsigned ret = (sensor_read_reg(client,0x41) & ~0x04);
	sensor_write_reg(client,0x41,ret);
	sensor_write_reg(client,0xc7,0x5a);
	sensor_write_reg(client,0xc8,0x42);
	sensor_write_reg(client,0xc9,0x40);
}

void gc0307_set_wb_office_mode(struct i2c_client *client)
{ 
	unsigned ret = (sensor_read_reg(client,0x41) & ~0x04);
	sensor_write_reg(client,0x41,ret);
	sensor_write_reg(client,0xc7,0x40);
	sensor_write_reg(client,0xc8,0x42);
	sensor_write_reg(client,0xc9,0x50);
}

void gc0307_set_wb_home_mode(struct i2c_client *client)
{
	unsigned ret = (sensor_read_reg(client,0x41) & ~0x04);
	sensor_write_reg(client,0x41,ret);
	sensor_write_reg(client,0xc7,0x40);
	sensor_write_reg(client,0xc8,0x45);
	sensor_write_reg(client,0xc9,0x60);
}

/*--------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------*/





/*---------------------------------------set banding------------------------------------------------*/

void gc0307_ab_auto(struct i2c_client *client)
{
	
}

void gc0307_ab_50hz(struct i2c_client *client)
{
	
}


void gc0307_ab_60hz(struct i2c_client *client)
{
	
}


void gc0307_ab_off(struct i2c_client *client)
{
	
}
/*--------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------*/





/*---------------------------------------set effect------------------------------------------------*/

void gc0307_set_effect_normal(struct i2c_client *client)
{
	unsigned ret = sensor_read_reg(client,0x47);
 	sensor_write_reg(client,0x41,0x2f);
	sensor_write_reg(client,0x40,0x7e);
	sensor_write_reg(client,0x42,0x10);
	sensor_write_reg(client,0x47,ret& 0xef);
	sensor_write_reg(client,0x48,0xc3);
	sensor_write_reg(client,0x8a,0x50);
	sensor_write_reg(client,0x8b,0x50);
	sensor_write_reg(client,0x8c,0x07);
	sensor_write_reg(client,0x50,0x0c);
	sensor_write_reg(client,0x77,0x80);
	sensor_write_reg(client,0xa1,0x40);
	sensor_write_reg(client,0x7a,0x00);
	sensor_write_reg(client,0x78,0x00);
	sensor_write_reg(client,0x79,0x00);
	sensor_write_reg(client,0x7b,0x40);
	sensor_write_reg(client,0x7c,0x00);
	//printk("===gc0307_set_effect_normal=0x47:%2x %2x, %2x\n", ret, ret&& 0xef, sensor_read_reg(client,0x47));

}

void gc0307_set_effect_sepia(struct i2c_client *client)
{
	unsigned ret = (sensor_read_reg(client,0x47) |~ 0xef);
	sensor_write_reg(client,0x41,0x2f);		
	sensor_write_reg(client,0x40,0x7e);
	sensor_write_reg(client,0x42,0x10);
	sensor_write_reg(client,0x47,ret);
	sensor_write_reg(client,0x48,0xc3);
	sensor_write_reg(client,0x8a,0x60);
	sensor_write_reg(client,0x8b,0x60);
	sensor_write_reg(client,0x8c,0x07);
	sensor_write_reg(client,0x50,0x0c);
	sensor_write_reg(client,0x77,0x80);
	sensor_write_reg(client,0xa1,0x40);
	sensor_write_reg(client,0x7a,0x00);
	sensor_write_reg(client,0x78,0xc0);
	sensor_write_reg(client,0x79,0x20);
	sensor_write_reg(client,0x7b,0x40);
	sensor_write_reg(client,0x7c,0x00);			
}

void gc0307_set_effect_bluish(struct i2c_client *client)
{
	unsigned ret = (sensor_read_reg(client,0x47) |~ 0xef);
	sensor_write_reg(client,0x41,0x2f);		
	sensor_write_reg(client,0x40,0x7e);
	sensor_write_reg(client,0x42,0x10);
	sensor_write_reg(client,0x47,ret);
	sensor_write_reg(client,0x48,0xc3);
	sensor_write_reg(client,0x8a,0x60);
	sensor_write_reg(client,0x8b,0x60);
	sensor_write_reg(client,0x8c,0x07);
	sensor_write_reg(client,0x50,0x0c);
	sensor_write_reg(client,0x77,0x80);
	sensor_write_reg(client,0xa1,0x40);
	sensor_write_reg(client,0x7a,0x00);
	sensor_write_reg(client,0x78,0x70);
	sensor_write_reg(client,0x79,0x00);
	sensor_write_reg(client,0x7b,0x3f);
	sensor_write_reg(client,0x7c,0xf5);		
}

void gc0307_set_effect_greenish(struct i2c_client *client)
{
	unsigned ret = (sensor_read_reg(client,0x47) |~ 0xef);
	sensor_write_reg(client,0x41,0x2f);		
	sensor_write_reg(client,0x40,0x7e);
	sensor_write_reg(client,0x42,0x10);
	sensor_write_reg(client,0x47,ret);
	sensor_write_reg(client,0x48,0xc3);
	sensor_write_reg(client,0x8a,0x60);
	sensor_write_reg(client,0x8b,0x60);
	sensor_write_reg(client,0x8c,0x07);
	sensor_write_reg(client,0x50,0x0c);
	sensor_write_reg(client,0x77,0x80);
	sensor_write_reg(client,0xa1,0x40);
	sensor_write_reg(client,0x7a,0x00);
	sensor_write_reg(client,0x78,0xc0);
	sensor_write_reg(client,0x79,0xc0);
	sensor_write_reg(client,0x7b,0x40);
	sensor_write_reg(client,0x7c,0x00);		
}

void gc0307_set_effect_reddish(struct i2c_client *client)
{
	unsigned ret = (sensor_read_reg(client,0x47) |~ 0xef);
	sensor_write_reg(client,0x41,0x2f);		
	sensor_write_reg(client,0x40,0x7e);
	sensor_write_reg(client,0x42,0x10);
	sensor_write_reg(client,0x47,ret);
	sensor_write_reg(client,0x48,0xc3);
	sensor_write_reg(client,0x8a,0x60);
	sensor_write_reg(client,0x8b,0x60);
	sensor_write_reg(client,0x8c,0x07);
	sensor_write_reg(client,0x50,0x0c);
	sensor_write_reg(client,0x77,0x80);
	sensor_write_reg(client,0xa1,0x40);
	sensor_write_reg(client,0x7a,0x00);
	sensor_write_reg(client,0x78,0x00);
	sensor_write_reg(client,0x79,0x30);
	sensor_write_reg(client,0x7b,0x40);
	sensor_write_reg(client,0x7c,0x00);				
}

void gc0307_set_effect_yellowish(struct i2c_client *client)
{
	unsigned ret = (sensor_read_reg(client,0x47) |~ 0xef);
	sensor_write_reg(client,0x41,0x2f);		
	sensor_write_reg(client,0x40,0x7e);
	sensor_write_reg(client,0x42,0x10);
	sensor_write_reg(client,0x47,ret);
	sensor_write_reg(client,0x48,0xc3);
	sensor_write_reg(client,0x8a,0x60);
	sensor_write_reg(client,0x8b,0x60);
	sensor_write_reg(client,0x8c,0x07);
	sensor_write_reg(client,0x50,0x0c);
	sensor_write_reg(client,0x77,0x80);
	sensor_write_reg(client,0xa1,0x40);
	sensor_write_reg(client,0x7a,0x00);
	sensor_write_reg(client,0x78,0x30);
	sensor_write_reg(client,0x79,0x30);
	sensor_write_reg(client,0x7b,0x40);
	sensor_write_reg(client,0x7c,0x00);	
}	

void gc0307_set_effect_blackwhite(struct i2c_client *client)
{
	unsigned ret = (sensor_read_reg(client,0x47) |~ 0xef);
	sensor_write_reg(client,0x41,0x2f);		
	sensor_write_reg(client,0x40,0x7e);
	sensor_write_reg(client,0x42,0x10);
	sensor_write_reg(client,0x47,ret);
	sensor_write_reg(client,0x48,0xc3);
	sensor_write_reg(client,0x8a,0x60);
	sensor_write_reg(client,0x8b,0x60);
	sensor_write_reg(client,0x8c,0x07);
	sensor_write_reg(client,0x50,0x0c);
	sensor_write_reg(client,0x77,0x80);
	sensor_write_reg(client,0xa1,0x40);
	sensor_write_reg(client,0x7a,0x00);
	sensor_write_reg(client,0x78,0x00);
	sensor_write_reg(client,0x79,0x00);
	sensor_write_reg(client,0x7b,0x40);
	sensor_write_reg(client,0x7c,0x00);			
}

void gc0307_set_effect_negative(struct i2c_client *client)
{
	unsigned ret = (sensor_read_reg(client,0x47) & 0xef);
	sensor_write_reg(client,0x41,0x2f);
	sensor_write_reg(client,0x40,0x7e);
	sensor_write_reg(client,0x42,0x10);
	sensor_write_reg(client,0x47,ret);
	sensor_write_reg(client,0x48,0xc3);
	sensor_write_reg(client,0x8a,0x60);
	sensor_write_reg(client,0x8b,0x60);
	sensor_write_reg(client,0x8c,0x07);
	sensor_write_reg(client,0x50,0x0c);
	sensor_write_reg(client,0x77,0x80);
	sensor_write_reg(client,0xa1,0x40);
	sensor_write_reg(client,0x7a,0x00);
	sensor_write_reg(client,0x78,0x00);
	sensor_write_reg(client,0x79,0x00);
	sensor_write_reg(client,0x7b,0x40);
	sensor_write_reg(client,0x7c,0x00);		
	
}

/*--------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------*/





