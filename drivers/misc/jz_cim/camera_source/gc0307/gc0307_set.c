#include <linux/jz_cim_core.h>
#include "../../jz_sensor.h"
#include <linux/i2c.h>
#include <asm/jzsoc.h>

#define gc0307_DEBUG
#ifdef gc0307_DEBUG
#define dprintk(x...)   do{printk("gc0307-\t");printk(x);printk("\n");}while(0)
#else
#define dprintk(x...)
#endif
void gc0307_reset(void);

static unsigned char miffor_flag = 0;

void preview_set(struct i2c_client *client)                   
{
	
}    //===preview setting end===


void capture_reg_set(struct i2c_client *client)
{
	
}

static void set_size_640x480(struct i2c_client *client,int mode)
{
		dprintk(" UXGA->VGA 640x480 1");
	
		sensor_write_reg(client,0xf0, 0x00);

		sensor_write_reg(client,0x05, 0x00); //row_start
		sensor_write_reg(client,0x06, 0x00);
		sensor_write_reg(client,0x07, 0x00); //col start
		sensor_write_reg(client,0x08, 0x00); 

		sensor_write_reg(client,0x09,0x01);
		sensor_write_reg(client,0x0a,0xe8);
		sensor_write_reg(client,0x0b,0x02);
		sensor_write_reg(client,0x0c,0x80);
		
		sensor_write_reg(client,0x0e, 0x02); //row even skip
		sensor_write_reg(client,0x43, 0x40); //more boundary mode opclk output enable
		
		if (miffor_flag ==0x30)
			{
				sensor_write_reg(client,0X45, 0X27);// 
			}
		if (miffor_flag ==0x20)
			{
				sensor_write_reg(client,0X45, 0X26);// 
			}
		if (miffor_flag ==0x10)
			{
				sensor_write_reg(client,0X45, 0X25);// 
			}
		if (miffor_flag ==0x00)
			{
				sensor_write_reg(client,0X45, 0X24);// 
			}

		sensor_write_reg(client,0x4e, 0x23);

		sensor_write_reg(client,0x01, 0x6a); //6a
		sensor_write_reg(client,0x02, 0x70); //70
		sensor_write_reg(client,0x10, 0x00); //00
		sensor_write_reg(client,0xd6, 0x96); //0x96

		sensor_write_reg(client,0x28, 0x03); //AEC_exp_level_1bit11to8   // 25fps
		sensor_write_reg(client,0x29, 0x84); //AEC_exp_level_1bit7to0
		sensor_write_reg(client,0x2a, 0x03); //AEC_exp_level_2bit11to8   // 16.6fps
		sensor_write_reg(client,0x2b, 0x84); //AEC_exp_level_2bit7to0
		sensor_write_reg(client,0x2c, 0x05); //AEC_exp_level_3bit11to8    // 12.5fps
		sensor_write_reg(client,0x2d, 0xdc); //AEC_exp_level_3bit7to0
		sensor_write_reg(client,0x2e, 0x09); //AEC_exp_level_4bit11to8   // 6.25fps
		sensor_write_reg(client,0x2f, 0x60);//AEC_exp_level_4bit7to0

		sensor_write_reg(client,0xe0,0x03);
		sensor_write_reg(client,0xe1,0x02);
		sensor_write_reg(client,0xe2,0x27);
		sensor_write_reg(client,0xe3,0x1e);
	
		sensor_write_reg(client,0xe8,0x3b);
		sensor_write_reg(client,0xe9,0x6e);
		sensor_write_reg(client,0xea,0x2c);
		sensor_write_reg(client,0xeb,0x50);
		sensor_write_reg(client,0xec,0x73);
		
		sensor_write_reg(client,0xae, 0x18);//black pixel target number
		sensor_write_reg(client,0xc3, 0x40);//number limit
		sensor_write_reg(client,0x74, 0x3c); //lsc_row_center , 0x3c
		sensor_write_reg(client,0x75, 0x52);//lsc_col_center , 0x52
		
		dprintk(" UXGA->VGA 640x480");
}

static void set_size_352x288(struct i2c_client *client,int mode)
{
	if(mode == CAMERA_MODE_PREVIEW)
	{
		
	}

	sensor_write_reg(client,0xf0, 0x00);

	sensor_write_reg(client,0x05, 0x00); //row_start
	sensor_write_reg(client,0x06, 0x60);
	sensor_write_reg(client,0x07, 0x00); //col start
	sensor_write_reg(client,0x08, 0x8c); 
	
	sensor_write_reg(client,0x09,0x01);
	sensor_write_reg(client,0x0a,0x28);
	sensor_write_reg(client,0x0b,0x01);
	sensor_write_reg(client,0x0c,0x60);

	sensor_write_reg(client,0x0e, 0x02); //row even skip
	sensor_write_reg(client,0x43, 0x40); //more boundary mode opclk output enable

	if (miffor_flag ==0x30)
			{
				sensor_write_reg(client,0X45, 0X27);// 
			}
		if (miffor_flag ==0x20)
			{
				sensor_write_reg(client,0X45, 0X26);// 
			}
		if (miffor_flag ==0x10)
			{
				sensor_write_reg(client,0X45, 0X25);// 
			}
		if (miffor_flag ==0x00)
			{
				sensor_write_reg(client,0X45, 0X24);// 
			}
	sensor_write_reg(client,0x4e, 0x23);

	sensor_write_reg(client,0x01, 0x8a); //6a
	sensor_write_reg(client,0x02, 0x30); //70
	sensor_write_reg(client,0x10, 0x11); //00
	sensor_write_reg(client,0xd6, 0x96); //0x96

	sensor_write_reg(client,0x28, 0x02); //AEC_exp_level_1bit11to8   // 25fps
	sensor_write_reg(client,0x29, 0x58); //AEC_exp_level_1bit7to0
	sensor_write_reg(client,0x2a, 0x03); //AEC_exp_level_2bit11to8   // 16.6fps
	sensor_write_reg(client,0x2b, 0x84); //AEC_exp_level_2bit7to0
	sensor_write_reg(client,0x2c, 0x04); //AEC_exp_level_3bit11to8    // 12.5fps
	sensor_write_reg(client,0x2d, 0xb0); //AEC_exp_level_3bit7to0
	sensor_write_reg(client,0x2e, 0x09); //AEC_exp_level_4bit11to8   // 6.25fps
	sensor_write_reg(client,0x2f, 0x60);//AEC_exp_level_4bit7to0

	sensor_write_reg(client,0xe0,0x03);
	sensor_write_reg(client,0xe1,0x02);
	sensor_write_reg(client,0xe2,0x15);//0x27
	sensor_write_reg(client,0xe3,0x12);//0x1e
	
	sensor_write_reg(client,0xe8,0x28);//160
	sensor_write_reg(client,0xe9,0x44);//160+112
	sensor_write_reg(client,0xea,0x18);
	sensor_write_reg(client,0xeb,0x30);
	sensor_write_reg(client,0xec,0x48);
		
	sensor_write_reg(client,0xae, 0x08);//black pixel target number
	sensor_write_reg(client,0xc3, 0x14);//number limit
	sensor_write_reg(client,0x74, 0x24); //lsc_row_center , 0x3c
	sensor_write_reg(client,0x75, 0x2c);//lsc_col_center , 0x52

	dprintk(" UXGA->CIF 352x288");
}  

static void set_size_320x240(struct i2c_client *client,int mode)
{
	if(mode == CAMERA_MODE_PREVIEW)
	{
		sensor_write_reg(client,0xf0, 0x00);  
		
		sensor_write_reg(client,0x05, 0x00); //row_start
		sensor_write_reg(client,0x06, 0x00);
		sensor_write_reg(client,0x07, 0x00); //col start
		sensor_write_reg(client,0x08, 0x00); 

		sensor_write_reg(client,0x09,0x01);
		sensor_write_reg(client,0x0a,0xe8);
		sensor_write_reg(client,0x0b,0x02);
		sensor_write_reg(client,0x0c,0x80);
		
		sensor_write_reg(client,0x0e, 0x0a); //row even skip        
		sensor_write_reg(client,0x43, 0xc0); //more boundary mode op

		if (miffor_flag ==0x30)
			{
				sensor_write_reg(client,0X45, 0X2b);// 
			}
		if (miffor_flag ==0x20)
			{
				sensor_write_reg(client,0X45, 0X2a);// 
			}
		if (miffor_flag ==0x10)
			{
				sensor_write_reg(client,0X45, 0X29);// 
			}
		if (miffor_flag ==0x00)
			{
				sensor_write_reg(client,0X45, 0X28);// 
			}
	      
		sensor_write_reg(client,0x4e, 0x33); //  32 opclk gate in su
		
		sensor_write_reg(client,0x01, 0xd1);                        
		sensor_write_reg(client,0x02, 0x82);                        
		sensor_write_reg(client,0x10, 0x00);                        
		sensor_write_reg(client,0xd6, 0xce);                        
		sensor_write_reg(client,0x28, 0x02); //AEC_exp_level_1bit11t
		sensor_write_reg(client,0x29, 0x6a); //AEC_exp_level_1bit7to
		sensor_write_reg(client,0x2a, 0x04); //AEC_exp_level_2bit11t
		sensor_write_reg(client,0x2b, 0x06); //AEC_exp_level_2bit7to
		sensor_write_reg(client,0x2c, 0x06); //AEC_exp_level_3bit11t
		sensor_write_reg(client,0x2d, 0x70); //AEC_exp_level_3bit7to
		sensor_write_reg(client,0x2e, 0x0c); //AEC_exp_level_4bit11t
		sensor_write_reg(client,0x2f, 0xe0); //AEC_exp_level_4bit7to
		
		sensor_write_reg(client,0xe0,0x03);
		sensor_write_reg(client,0xe1,0x02);
		sensor_write_reg(client,0xe2,0x27);
		sensor_write_reg(client,0xe3,0x1e);
		
		sensor_write_reg(client,0xe8,0x3b);
		sensor_write_reg(client,0xe9,0x6e);
		sensor_write_reg(client,0xea,0x2c);
		sensor_write_reg(client,0xeb,0x50);
		sensor_write_reg(client,0xec,0x73);
		
		sensor_write_reg(client,0xae, 0x0c); //black pixel target nu
		sensor_write_reg(client,0xc3, 0x20); //number limit         
		sensor_write_reg(client,0x74, 0x1e); //lsc_row_center , 0x3c
		sensor_write_reg(client,0x75, 0x52); //lsc_col_center , 0x52

	}

	dprintk(" CIF 320x240");
}
 
static void set_size_176x144(struct i2c_client *client,int mode)
{
	if(mode == CAMERA_MODE_PREVIEW)
	{
	sensor_write_reg(client,0xf0, 0x00);

	sensor_write_reg(client,0x05, 0x00); //row_start
	sensor_write_reg(client,0x06, 0x60);
	sensor_write_reg(client,0x07, 0x00); //col start
	sensor_write_reg(client,0x08, 0x8c); 
	
	sensor_write_reg(client,0x09,0x01);
	sensor_write_reg(client,0x0a,0x28);
	sensor_write_reg(client,0x0b,0x01);
	sensor_write_reg(client,0x0c,0x60);

	sensor_write_reg(client,0x0e, 0x02); //row even skip
	sensor_write_reg(client,0x43, 0x40); //more boundary mode opclk output enable

		if (miffor_flag ==0x30)
			{
				sensor_write_reg(client,0X45, 0X4b);// 
			}
		if (miffor_flag ==0x20)
			{
				sensor_write_reg(client,0X45, 0X4a);// 
			}
		if (miffor_flag ==0x10)
			{
				sensor_write_reg(client,0X45, 0X49);// 
			}
		if (miffor_flag ==0x00)
			{
				sensor_write_reg(client,0X45, 0X48);// 
			}
	sensor_write_reg(client,0x4e, 0x33);

	sensor_write_reg(client,0x01, 0x8a); //6a
	sensor_write_reg(client,0x02, 0x30); //70
	sensor_write_reg(client,0x10, 0x11); //00
	sensor_write_reg(client,0xd6, 0x96); //0x96

	sensor_write_reg(client,0x28, 0x02); //AEC_exp_level_1bit11to8   // 25fps
	sensor_write_reg(client,0x29, 0x58); //AEC_exp_level_1bit7to0
	sensor_write_reg(client,0x2a, 0x03); //AEC_exp_level_2bit11to8   // 16.6fps
	sensor_write_reg(client,0x2b, 0x84); //AEC_exp_level_2bit7to0
	sensor_write_reg(client,0x2c, 0x04); //AEC_exp_level_3bit11to8    // 12.5fps
	sensor_write_reg(client,0x2d, 0xb0); //AEC_exp_level_3bit7to0
	sensor_write_reg(client,0x2e, 0x09); //AEC_exp_level_4bit11to8   // 6.25fps
	sensor_write_reg(client,0x2f, 0x60);//AEC_exp_level_4bit7to0

	sensor_write_reg(client,0xe0,0x03);
	sensor_write_reg(client,0xe1,0x02);
	sensor_write_reg(client,0xe2,0x15);//0x27
	sensor_write_reg(client,0xe3,0x12);//0x1e
	
	sensor_write_reg(client,0xe8,0x28);//160
	sensor_write_reg(client,0xe9,0x44);//160+112
	sensor_write_reg(client,0xea,0x18);
	sensor_write_reg(client,0xeb,0x30);
	sensor_write_reg(client,0xec,0x48);
	
	sensor_write_reg(client,0xae, 0x08);//black pixel target number
	sensor_write_reg(client,0xc3, 0x14);//number limit
	sensor_write_reg(client,0x74, 0x24); //lsc_row_center , 0x3c
	sensor_write_reg(client,0x75, 0x2c);//lsc_col_center , 0x52

	}

	dprintk(" UXGA->QCIF 176x144");
} 

void size_switch(struct i2c_client *client,int width,int height,int setmode)
{
	dprintk("%dx%d - mode(%d)",width,height,setmode);

	if(width == 640 && height == 480)
	{
		set_size_640x480(client,setmode);
	} 
	else if(width == 352 && height == 288)
	{
		set_size_352x288(client,setmode);
	} 
	else if(width == 320 && height == 240)
	{
		set_size_320x240(client,setmode);
	}   
	else if(width == 176 && height == 144)
	{
		set_size_176x144(client,setmode);
	}  
	else
		return;
	//	mdelay(500);
}

void gc0307_init_setting(struct i2c_client *client)
{
	gc0307_reset();
	//sensor_write_reg(client,0x00,0x99);
	sensor_write_reg(client,0x01,0x6a);
	sensor_write_reg(client,0x02,0x70);
	
	sensor_write_reg(client,0x03,0x07);
	sensor_write_reg(client,0x04,0x08);
	sensor_write_reg(client,0x05,0x00);
	sensor_write_reg(client,0x06,0x00);
	sensor_write_reg(client,0x07,0x00);
	sensor_write_reg(client,0x08,0x00);
	sensor_write_reg(client,0x09,0x01);
	sensor_write_reg(client,0x0a,0xe8);
	sensor_write_reg(client,0x0b,0x02);
	sensor_write_reg(client,0x0c,0x80);
	sensor_write_reg(client,0x0d,0x22);
	sensor_write_reg(client,0x0e,0x02);
	sensor_write_reg(client,0x0f,0xa2);//0xb2 //0xa2 ,0x82,0x92
	sensor_write_reg(client,0x10,0x00);
	sensor_write_reg(client,0x11,0x05);
	sensor_write_reg(client,0x12,0x70);
	sensor_write_reg(client,0x13,0x80);
	sensor_write_reg(client,0x14,0x00);
	sensor_write_reg(client,0x15,0xba);
	sensor_write_reg(client,0x16,0x13);
	sensor_write_reg(client,0x17,0x52);
	sensor_write_reg(client,0x18,0x00);//0x50
	sensor_write_reg(client,0x19,0x06);
	sensor_write_reg(client,0x1a,0x06);
	sensor_write_reg(client,0x1b,0x00);
	sensor_write_reg(client,0x1c,0x00);
	sensor_write_reg(client,0x1d,0x00);
	sensor_write_reg(client,0x1e,0x0d);
	sensor_write_reg(client,0x1f,0x32);
	sensor_write_reg(client,0x20,0x06);
	sensor_write_reg(client,0x21,0xc0);
	sensor_write_reg(client,0x22,0x60);
	sensor_write_reg(client,0x23,0x88);
	sensor_write_reg(client,0x24,0x96);
	sensor_write_reg(client,0x25,0x30);
	sensor_write_reg(client,0x26,0xd0);
	sensor_write_reg(client,0x27,0x00);
	
	sensor_write_reg(client,0x28,0x03);
	sensor_write_reg(client,0x29,0x84);
	sensor_write_reg(client,0x2a,0x03);
	sensor_write_reg(client,0x2b,0x84);
	sensor_write_reg(client,0x2c,0x07);
	sensor_write_reg(client,0x2d,0x08);
	sensor_write_reg(client,0x2e,0x0d);
	sensor_write_reg(client,0x2f,0x7a);
	
	sensor_write_reg(client,0x30,0x20);
	sensor_write_reg(client,0x31,0x00);
	sensor_write_reg(client,0x32,0x04);
	sensor_write_reg(client,0x33,0x90);
	sensor_write_reg(client,0x34,0x10);
	sensor_write_reg(client,0x35,0xd8);
	sensor_write_reg(client,0x36,0x40);
	sensor_write_reg(client,0x37,0x02);
	sensor_write_reg(client,0x38,0x02);
	sensor_write_reg(client,0x39,0x02);
	sensor_write_reg(client,0x3a,0x02);
	sensor_write_reg(client,0x3b,0x00);
	sensor_write_reg(client,0x3c,0xfe);
	sensor_write_reg(client,0x3d,0x00);
	sensor_write_reg(client,0x3e,0x01);
	sensor_write_reg(client,0x3f,0xfe);
	sensor_write_reg(client,0x40,0x7e);
	sensor_write_reg(client,0x41,0x3f);
	sensor_write_reg(client,0x42,0x10);
	sensor_write_reg(client,0x43,0x40);
	sensor_write_reg(client,0x44,0xe2);
	sensor_write_reg(client,0x45,0x27);
	sensor_write_reg(client,0x46,0x42);
	sensor_write_reg(client,0x47,0x2c);
	sensor_write_reg(client,0x48,0xc3);
	sensor_write_reg(client,0x49,0x00);
	sensor_write_reg(client,0x4a,0x00);
	sensor_write_reg(client,0x4b,0x00);
	sensor_write_reg(client,0x4c,0x00);
	sensor_write_reg(client,0x4d,0x00);
	sensor_write_reg(client,0x4e,0x22);
	sensor_write_reg(client,0x4f,0x01);
	sensor_write_reg(client,0x50,0x0c);
	sensor_write_reg(client,0x51,0x20);
	sensor_write_reg(client,0x52,0x08);
	sensor_write_reg(client,0x53,0x00);
	sensor_write_reg(client,0x54,0x00);
	sensor_write_reg(client,0x55,0x00);
	sensor_write_reg(client,0x56,0x88);
	sensor_write_reg(client,0x57,0x08);
	sensor_write_reg(client,0x58,0x88);
	sensor_write_reg(client,0x59,0x0f);
	sensor_write_reg(client,0x5a,0x41);
	sensor_write_reg(client,0x5b,0x40);
	sensor_write_reg(client,0x5c,0x48);//54
	sensor_write_reg(client,0x5d,0x58);//64
	sensor_write_reg(client,0x5e,0x00);
	sensor_write_reg(client,0x5f,0x00);
	sensor_write_reg(client,0x60,0x00);
	sensor_write_reg(client,0x61,0x80);
	sensor_write_reg(client,0x62,0x00);
	sensor_write_reg(client,0x63,0x80);
	sensor_write_reg(client,0x64,0x00);
	sensor_write_reg(client,0x65,0xa0);
	sensor_write_reg(client,0x66,0x00);
	sensor_write_reg(client,0x67,0x80);
	sensor_write_reg(client,0x68,0x18);
	sensor_write_reg(client,0x69,0x4c);
	sensor_write_reg(client,0x6a,0xf6);
	sensor_write_reg(client,0x6b,0xfe);
	sensor_write_reg(client,0x6c,0xfa);
	sensor_write_reg(client,0x6d,0x58);
	sensor_write_reg(client,0x6e,0xed);
	sensor_write_reg(client,0x6f,0x00);
	sensor_write_reg(client,0x70,0x14);
	sensor_write_reg(client,0x71,0x1c);
	sensor_write_reg(client,0x72,0x20);
	sensor_write_reg(client,0x73,0x10);
	sensor_write_reg(client,0x74,0x3c);
	sensor_write_reg(client,0x75,0x52);
	sensor_write_reg(client,0x76,0x00);
	sensor_write_reg(client,0x77,0x60);
	sensor_write_reg(client,0x78,0x00);
	sensor_write_reg(client,0x79,0x00);
	sensor_write_reg(client,0x7a,0x00);
	sensor_write_reg(client,0x7b,0x40);
	sensor_write_reg(client,0x7c,0x00);
	sensor_write_reg(client,0x7d,0x1f);
	sensor_write_reg(client,0x7e,0x45);//33
	sensor_write_reg(client,0x7f,0x86);//83
	sensor_write_reg(client,0x80,0x0c);
	sensor_write_reg(client,0x81,0x0c);
	sensor_write_reg(client,0x82,0x88);
	sensor_write_reg(client,0x83,0x18);
	sensor_write_reg(client,0x84,0x18);
	sensor_write_reg(client,0x85,0x04);
	sensor_write_reg(client,0x86,0x05);
	sensor_write_reg(client,0x87,0x32);
	sensor_write_reg(client,0x88,0x06);
	sensor_write_reg(client,0x89,0x02);
	sensor_write_reg(client,0x8a,0x50);
	sensor_write_reg(client,0x8b,0x50);
	sensor_write_reg(client,0x8c,0x07);
	sensor_write_reg(client,0x8d,0x56);
	sensor_write_reg(client,0x8e,0x05);
	sensor_write_reg(client,0x8f,0x1d);
	sensor_write_reg(client,0x90,0x30);
	sensor_write_reg(client,0x91,0x45);
	sensor_write_reg(client,0x92,0x56);
	sensor_write_reg(client,0x93,0x66);
	sensor_write_reg(client,0x94,0x74);
	sensor_write_reg(client,0x95,0x81);
	sensor_write_reg(client,0x96,0x96);
	sensor_write_reg(client,0x97,0xa8);
	sensor_write_reg(client,0x98,0xb7);
	sensor_write_reg(client,0x99,0xc3);
	sensor_write_reg(client,0x9a,0xce);
	sensor_write_reg(client,0x9b,0xd7);
	sensor_write_reg(client,0x9c,0xdf);
	sensor_write_reg(client,0x9d,0xeb);
	sensor_write_reg(client,0x9e,0xf5);
	sensor_write_reg(client,0x9f,0xfc);
	sensor_write_reg(client,0xa0,0x40);
	sensor_write_reg(client,0xa1,0x44);
	sensor_write_reg(client,0xa2,0x44);
	sensor_write_reg(client,0xa3,0x44);
	sensor_write_reg(client,0xa4,0xc8);
	sensor_write_reg(client,0xa5,0x02);
	sensor_write_reg(client,0xa6,0x28);
	sensor_write_reg(client,0xa7,0x02);
	sensor_write_reg(client,0xa8,0xee);
	sensor_write_reg(client,0xa9,0x12);
	sensor_write_reg(client,0xaa,0x01);
	sensor_write_reg(client,0xab,0x20);
	sensor_write_reg(client,0xac,0xf0);
	sensor_write_reg(client,0xad,0x10);
	sensor_write_reg(client,0xae,0x20);//12
	sensor_write_reg(client,0xaf,0x74);//74
	sensor_write_reg(client,0xb0,0xc5);
	sensor_write_reg(client,0xb1,0x20);
	sensor_write_reg(client,0xb2,0x6c);
	sensor_write_reg(client,0xb3,0x40);
	sensor_write_reg(client,0xb4,0x04);
	sensor_write_reg(client,0xb5,0x70);
	sensor_write_reg(client,0xb6,0x40);
	sensor_write_reg(client,0xb7,0x00);
	sensor_write_reg(client,0xb8,0x38);
	sensor_write_reg(client,0xb9,0xc3);
	sensor_write_reg(client,0xba,0x0f);
	sensor_write_reg(client,0xbb,0x42);
	sensor_write_reg(client,0xbc,0x60);
	sensor_write_reg(client,0xbd,0x50);
	sensor_write_reg(client,0xbe,0x50);
	sensor_write_reg(client,0xbf,0x0c);
	sensor_write_reg(client,0xc0,0x06);
	sensor_write_reg(client,0xc1,0x60);
	sensor_write_reg(client,0xc2,0xf1);
	sensor_write_reg(client,0xc3,0x40);
	sensor_write_reg(client,0xc4,0x1c);
	sensor_write_reg(client,0xc5,0x56);
	sensor_write_reg(client,0xc6,0x1d);
	sensor_write_reg(client,0xc7,0x55);
	sensor_write_reg(client,0xc8,0x40);
	sensor_write_reg(client,0xc9,0x40);
	sensor_write_reg(client,0xca,0x56);
	sensor_write_reg(client,0xcb,0x52);
	sensor_write_reg(client,0xcc,0x66);
	sensor_write_reg(client,0xcd,0x82);
	sensor_write_reg(client,0xce,0x82);
	sensor_write_reg(client,0xcf,0x83);
	sensor_write_reg(client,0xd0,0x34);
	sensor_write_reg(client,0xd1,0x50);
	sensor_write_reg(client,0xd2,0x61);
	sensor_write_reg(client,0xd3,0x56);
	sensor_write_reg(client,0xd4,0x96);
	sensor_write_reg(client,0xd5,0x01);
	
	sensor_write_reg(client,0xd6,0x96);
	
	sensor_write_reg(client,0xd7,0x96);
	sensor_write_reg(client,0xd8,0x02);
	sensor_write_reg(client,0xd9,0x00);
	sensor_write_reg(client,0xda,0x00);
	sensor_write_reg(client,0xdb,0x40);
	sensor_write_reg(client,0xdc,0x40);
	sensor_write_reg(client,0xdd,0x22);
	sensor_write_reg(client,0xde,0x00);
	sensor_write_reg(client,0xdf,0x00);
	sensor_write_reg(client,0xe0,0x03);
	sensor_write_reg(client,0xe1,0x02);
	sensor_write_reg(client,0xe2,0x27);
	sensor_write_reg(client,0xe3,0x1e);
	sensor_write_reg(client,0xe4,0x00);
	sensor_write_reg(client,0xe5,0x00);
	sensor_write_reg(client,0xe6,0x00);
	sensor_write_reg(client,0xe7,0x00);
	sensor_write_reg(client,0xe8,0x3b);
	sensor_write_reg(client,0xe9,0x6e);
	sensor_write_reg(client,0xea,0x2c);
	sensor_write_reg(client,0xeb,0x50);
	sensor_write_reg(client,0xec,0x73);
	sensor_write_reg(client,0xed,0x00);
	sensor_write_reg(client,0xee,0x00);
	sensor_write_reg(client,0xef,0x00);
	sensor_write_reg(client,0xf0,0x00);
	sensor_write_reg(client,0xf1,0x03);
	sensor_write_reg(client,0xf2,0x33);
	sensor_write_reg(client,0xf3,0x00);
	sensor_write_reg(client,0xf4,0x00);
	sensor_write_reg(client,0xf5,0x00);
	sensor_write_reg(client,0xf6,0x00);
	sensor_write_reg(client,0xf7,0x00);
	sensor_write_reg(client,0xf8,0x00);
	sensor_write_reg(client,0xf9,0x00);
	sensor_write_reg(client,0xfa,0x00);
	sensor_write_reg(client,0xfb,0x00);
	sensor_write_reg(client,0xfc,0x00);
	sensor_write_reg(client,0xfd,0x00);
	sensor_write_reg(client,0xfe,0x00);
	sensor_write_reg(client,0xff,0x00);

	miffor_flag = (sensor_read_reg(client,0x0f) & 0x30);

	if (miffor_flag ==0x30)
			{
				sensor_write_reg(client,0X47, 0X2c);// 
			}
		if (miffor_flag ==0x20)
			{
				sensor_write_reg(client,0X47, 0X28);// 
			}
		if (miffor_flag ==0x10)
			{
				sensor_write_reg(client,0X47, 0X24);// 
			}
		if (miffor_flag ==0x00)
			{
				sensor_write_reg(client,0X47, 0X20);// 
			}
}

void capture_set(struct i2c_client *client)
{	
	dprintk("capture_set");
}

