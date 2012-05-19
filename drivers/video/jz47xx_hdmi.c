/*
 * linux/drivers/video/jz47xx_hdmi.c -- Ingenic Jz4760 hdmi operation
 * interface.
 * Copyright (C) 2005-2010, Ingenic Semiconductor Inc.
 * Author: hao liu, <hliu@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <asm/jzsoc.h>
#include "jz47xx_hdmi.h"

struct jz47xx_hdmi_info_t jz47xx_hdmi_info[] =
{
	{
		.hdmi_type = HDMI_Unkown,
		.hdmi_info = {
			.panel = {
#if defined(CONFIG_HDMI_IT6610)
				.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
				       LCD_CFG_NEWDES | LCD_CFG_RECOVER |
#if defined(CONFIG_JZ4770_NPM701) || defined(CONFIG_JZ4770_AURORA)
			 		LCD_CFG_HSP | LCD_CFG_VSP,
#elif defined(CONFIG_JZ4770_TVB) || defined(CONFIG_JZ4770_N55)
				    LCD_CFG_PCP,
#else
			 		LCD_CFG_HSP | LCD_CFG_VSP ,
#endif
				//       LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,
#else
				.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
				       LCD_CFG_NEWDES | LCD_CFG_RECOVER |
				       LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,
#endif
				.slcd_cfg = 0,
				.ctrl = LCD_CTRL_BST_32,
				//  width,height,freq,hsync,vsync,elw,blw,efw,bfw
#if defined(CONFIG_JZ4770_TVB) || defined(CONFIG_JZ4770_N55)
				1280, 720, 60, 30, 16, 48, 16, 23, 7,
#else
				//640,480, 60, 96, 2,48,16,  33,10,
			//	800,480, 56, 30,22,48,16,  23, 1,  
				800,480, 60, 30,16,48,16,  23, 7,
#endif
			},                                      
			.osd = {                                
				.osd_cfg =  LCD_OSDC_OSDEN      |               // Use OSD mode
				 LCD_OSDC_ALPHAEN                  |            // enable alpha
				 LCD_OSDC_F0EN,                               // enable Foreground0   
			//	 LCD_OSDC_F1EN,                               // enable Foreground1   				 
		
				 .osd_ctrl = 0,                                 // disable ipu,          
				 .rgb_ctrl = 0,                                                                  
				 .bgcolor = 0x000000,                           // set background color Black    
				 .colorkey0 = 0,                                // disable colorkey
				 .colorkey1 = 0,                                // disable colorkey
				 .alpha = 0xa0,                                 // alpha value
				 .ipu_restart = 0x8000085d,                     // ipu restart
				 .fg_change = FG_CHANGE_ALL,            // change all initially
#if defined(CONFIG_JZ4770_TVB) || defined(CONFIG_JZ4770_N55)
				 .fg0 = {32, 0, 0, 1280, 720},   // bpp, x, y, w, h
				 .fg1 = {32, 0, 0, 1280, 720},   // bpp, x, y, w, h
#else
				 .fg0 = {32, 0, 0, 800, 480},   // bpp, x, y, w, h
				 .fg1 = {32, 0, 0, 800, 480},   // bpp, x, y, w, h
#endif
			},
		},
	},
	{
		.hdmi_type = HDMI_640x480p60 ,
		.hdmi_info = {
			.panel = {                                                     
#if defined(CONFIG_HDMI_IT6610)
				.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
				       LCD_CFG_NEWDES | LCD_CFG_RECOVER | LCD_CFG_PCP,
				//	   LCD_CFG_HSP | LCD_CFG_VSP,
				//       LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,           
#else
				.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
				       LCD_CFG_NEWDES | LCD_CFG_RECOVER |                 
				       LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,           
#endif	
				.slcd_cfg = 0,                                            
				.ctrl = LCD_CTRL_BST_32,                                  
				//  width,height,freq,hsync,vsync,elw,blw,efw,bfw         
				640,480, 60, 96, 2,48,16,  33,10,
			},                                      
			.osd = {                                
				.osd_cfg =  LCD_OSDC_OSDEN      |               // Use OSD mode
				 LCD_OSDC_ALPHAEN                  |            // enable alpha
				 //LCD_OSDC_F0EN,                               // enable Foreground0   
				 LCD_OSDC_F1EN,                               // enable Foreground1   				 
		
				 .osd_ctrl = 0,                                 // disable ipu,          
				 .rgb_ctrl = 0,                                                                  
				 .bgcolor = 0x000000,                           // set background color Black    
				 .colorkey0 = 0,                                // disable colorkey
				 .colorkey1 = 0,                                // disable colorkey
				 .alpha = 0xa0,                                 // alpha value
				 .ipu_restart = 0x8000085d,                     // ipu restart
				 .fg_change = FG_CHANGE_ALL,            // change all initially
				 .fg0 = {32, 0, 0, 640, 480},   // bpp, x, y, w, h
				 .fg1 = {32, 0, 0, 640, 480},   // bpp, x, y, w, h
			},
		},
	},
	{
		.hdmi_type = HDMI_480p60_16x9,
		.hdmi_info = {
			.panel = {                                                     
#if defined(CONFIG_HDMI_IT6610)
				.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
				       LCD_CFG_NEWDES | LCD_CFG_RECOVER| LCD_CFG_PCP,
#else
				.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
				       LCD_CFG_NEWDES | LCD_CFG_RECOVER |                 
				       LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,           
#endif	
				.slcd_cfg = 0,                                            
				.ctrl = LCD_CTRL_BST_32,                                  
				//  width,height,freq,hsync,vsync,elw,blw,efw,bfw     
				720,480, 60, 62, 6,60,16,  30,9,
			},                                      
			.osd = {                                
				.osd_cfg =  LCD_OSDC_OSDEN      |               // Use OSD mode
				 LCD_OSDC_ALPHAEN                  |            // enable alpha
				 //LCD_OSDC_F0EN                          ,       // enable Foreground0    
				 LCD_OSDC_F1EN,                               // enable Foreground1    
		
				 .osd_ctrl = 0,                                 // disable ipu,          
				 .rgb_ctrl = 0,                                                                  
				 .bgcolor = 0x000000,                           // set background color Black    
				 .colorkey0 = 0,                                // disable colorkey
				 .colorkey1 = 0,                                // disable colorkey
				 .alpha = 0xa0,                                 // alpha value
				 .ipu_restart = 0x8000085d,                     // ipu restart
				 .fg_change = FG_CHANGE_ALL,            // change all initially
				 .fg0 = {32, 0, 0, 720, 480,},   // bpp, x, y, w, h
				 .fg1 = {32, 0, 0, 720, 480,},   // bpp, x, y, w, h
			},
		},							
	},
	{
		.hdmi_type = HDMI_720p60,
		.hdmi_info = {
			.panel = {                                                     
#if defined(CONFIG_HDMI_IT6610)
				.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
				       LCD_CFG_NEWDES | LCD_CFG_RECOVER |LCD_CFG_PCP,
#else
				.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
				       LCD_CFG_NEWDES | LCD_CFG_RECOVER |
				       LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,           
#endif	
				.slcd_cfg = 0,                                            
				.ctrl = LCD_CTRL_BST_32,                                  
				//  width,height,freq,hsync,vsync,elw,blw,efw,bfw         
//		1280,720,60,40,5,440,220,20,5,
		1280,720,60,40,5,110,220,5,20,
			},                                      
			.osd = {                                
				.osd_cfg =  LCD_OSDC_OSDEN      |               // Use OSD mode
				 LCD_OSDC_ALPHAEN                  |            // enable alpha
				 //LCD_OSDC_F0EN                          ,       // enable Foreground0    
				LCD_OSDC_F1EN,                               // enable Foreground1    
		
				 .osd_ctrl = 0,                                 // disable ipu,          
				 .rgb_ctrl = 0,                                                                  
				 .bgcolor = 0x000000,                           // set background color Black    
				 .colorkey0 = 0,                                // disable colorkey
				 .colorkey1 = 0,                                // disable colorkey
				 .alpha = 0xa0,                                 // alpha value
				 .ipu_restart = 0x8000085d,                     // ipu restart
				 .fg_change = FG_CHANGE_ALL,            // change all initially
			//	 .fg0 = {32, 20, 20, 1240, 680,},   // bpp, x, y, w, h
			//	 .fg1 = {32, 20, 20, 1240, 680,},   // bpp, x, y, w, h
				 .fg0 = {32, 0, 0, 1280, 720,},   // bpp, x, y, w, h
				 .fg1 = {32, 0, 0, 1280, 720,},   // bpp, x, y, w, h
			},
		},
	},
	{
		.hdmi_type = HDMI_1080i60,
		.hdmi_info = {
			.panel = {                                                     
#if defined(CONFIG_HDMI_IT6610)
				.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
				       LCD_CFG_NEWDES | LCD_CFG_RECOVER,
#else
				.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
				       LCD_CFG_NEWDES | LCD_CFG_RECOVER |                 
				       LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,           
#endif	
				.slcd_cfg = 0,                                            
				.ctrl = LCD_CTRL_BST_32,                                  
				//  width,height,freq,hsync,vsync,elw,blw,efw,bfw         
				//1920,1080,60,44,5,148,88,15,2,
				1920,1080,60,44,5,88,148,3,15,
			},                                      
			.osd = {                                
				.osd_cfg =  LCD_OSDC_OSDEN      |               // Use OSD mode
				 LCD_OSDC_ALPHAEN                  |            // enable alpha
				 //LCD_OSDC_F0EN                          ,       // enable Foreground0    
				LCD_OSDC_F1EN,                               // enable Foreground1    
		
				 .osd_ctrl = 0,                                 // disable ipu,          
				 .rgb_ctrl = 0,                                                                  
				 .bgcolor = 0x000000,                           // set background color Black    
				 .colorkey0 = 0,                                // disable colorkey
				 .colorkey1 = 0,                                // disable colorkey
				 .alpha = 0xa0,                                 // alpha value
				 .ipu_restart = 0x8000085d,                     // ipu restart
				 .fg_change = FG_CHANGE_ALL,            // change all initially
				 .fg0 = {32, 0, 0, 1920, 1080,},   // bpp, x, y, w, h
				 .fg1 = {32, 0, 0, 1920, 1080,},   // bpp, x, y, w, h
			},
		},
	},
	{
		.hdmi_type = HDMI_1080i50,
		.hdmi_info = {
			.panel = {                                                     
#if defined(CONFIG_HDMI_IT6610)
				.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
				       LCD_CFG_NEWDES | LCD_CFG_RECOVER,
#else
				.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
				       LCD_CFG_NEWDES | LCD_CFG_RECOVER |                 
				       LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,           
#endif	
				.slcd_cfg = 0,                                            
				.ctrl = LCD_CTRL_BST_32,                                  
				//  width,height,freq,hsync,vsync,elw,blw,efw,bfw         
				1920,1080,50,44,5,528,148,3,15,
			},                                      
			.osd = {                                
				.osd_cfg =  LCD_OSDC_OSDEN      |               // Use OSD mode
				 LCD_OSDC_ALPHAEN                  |            // enable alpha
				// LCD_OSDC_F0EN                          ,       // enable Foreground0    
				 LCD_OSDC_F1EN,                               // enable Foreground1    
		
				 .osd_ctrl = 0,                                 // disable ipu,          
				 .rgb_ctrl = 0,                                                                  
				 .bgcolor = 0x000000,                           // set background color Black    
				 .colorkey0 = 0,                                // disable colorkey
				 .colorkey1 = 0,                                // disable colorkey
				 .alpha = 0xa0,                                 // alpha value
				 .ipu_restart = 0x8000085d,                     // ipu restart
				 .fg_change = FG_CHANGE_ALL,            // change all initially
				 .fg0 = {32, 0, 0, 1920, 1080,},   // bpp, x, y, w, h
				 .fg1 = {32, 0, 0, 1920, 1080,},   // bpp, x, y, w, h
			},
		},
	},
	{
		.hdmi_type = HDMI_576p50_16x9,
		.hdmi_info = {
			.panel = {                                                     
#if defined(CONFIG_HDMI_IT6610)
				.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
				       LCD_CFG_NEWDES | LCD_CFG_RECOVER,
#else
				.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
				       LCD_CFG_NEWDES | LCD_CFG_RECOVER |                 
				       LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,           
#endif	
				.slcd_cfg = 0,                                            
				.ctrl = LCD_CTRL_BST_32,                                  
				//  width,height,freq,hsync,vsync,elw,blw,efw,bfw    
				720,576,50,64,5,68,12,40,4,     
			},                                      
			.osd = {                                
				.osd_cfg =  LCD_OSDC_OSDEN      |               // Use OSD mode
				 LCD_OSDC_ALPHAEN                       |               // enable alpha
				// LCD_OSDC_F0EN                          ,               // enable Foreground0    
				LCD_OSDC_F1EN,                                         // enable Foreground1    
				 .osd_ctrl = 0,                                         // disable ipu,          
				 .rgb_ctrl = 0,                                                                  
				 .bgcolor = 0x000000,                           // set background color Black    
				 .colorkey0 = 0,                                        // disable colorkey
				 .colorkey1 = 0,                                        // disable colorkey
				 .alpha = 0xa0,                                         // alpha value
				 .ipu_restart = 0x8000085d,                     // ipu restart
				 .fg_change = FG_CHANGE_ALL,            // change all initially
				 .fg0 = {32, 10, 10, 700, 556},   // bpp, x, y, w, h
				 .fg1 = {32, 10, 10, 700, 556},       // bpp, x, y, w, h		
			},
		},
	},
	{
		.hdmi_type = HDMI_720p50,
		.hdmi_info = {
			.panel = {                                                     
#if defined(CONFIG_HDMI_IT6610)
				.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
				       LCD_CFG_NEWDES | LCD_CFG_RECOVER,
#else
				.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
				       LCD_CFG_NEWDES | LCD_CFG_RECOVER |                 
				       LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,           
#endif	
				.slcd_cfg = 0,                                            
				.ctrl = LCD_CTRL_BST_32,                                  
				//  width,height,freq,hsync,vsync,elw,blw,efw,bfw  
				//1280,720,50,40,5,440,220,20,5,  
				1280,720,50,40,5,440,220,5,20, 
			},                                      
			.osd = {                                
				.osd_cfg =  LCD_OSDC_OSDEN      |               // Use OSD mode
				 LCD_OSDC_ALPHAEN                  |            // enable alpha
				// LCD_OSDC_F0EN                          ,       // enable Foreground0    
				LCD_OSDC_F1EN,                               // enable Foreground1    
		
				 .osd_ctrl = 0,                                 // disable ipu,          
				 .rgb_ctrl = 0,                                                                  
				 .bgcolor = 0x000000,                           // set background color Black    
				 .colorkey0 = 0,                                // disable colorkey
				 .colorkey1 = 0,                                // disable colorkey
				 .alpha = 0xa0,                                 // alpha value
				 .ipu_restart = 0x8000085d,                     // ipu restart
				 .fg_change = FG_CHANGE_ALL,            // change all initially
				 //.fg0 = {32, 20, 20, 1240, 680},   // bpp, x, y, w, h
                 //.fg1 = {32, 20, 20, 1240, 680},       // bpp, x, y, w, h
				 .fg0 = {32, 0, 0, 1280, 720,},   // bpp, x, y, w, h
				 .fg1 = {32, 0, 0, 1280, 720,},   // bpp, x, y, w, h
			},
		},
	},
	{
		.hdmi_type = HDMI_1080p60,
		.hdmi_info = {
			.panel = {                                                     
#if defined(CONFIG_HDMI_IT6610)
				.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
				       LCD_CFG_NEWDES | LCD_CFG_RECOVER | LCD_CFG_PCP,
				//	   LCD_CFG_HSP | LCD_CFG_VSP,
				//       LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,           
#else
				.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
				       LCD_CFG_NEWDES | LCD_CFG_RECOVER |                 
				       LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,            
#endif	
				.slcd_cfg = 0,                                            
				.ctrl = LCD_CTRL_BST_32,                                  
				//  width,height,freq,hsync,vsync,elw,blw,efw,bfw         
				1920,1080, 60, 44,5,88,148,  5, 35,  
			},                                      
			.osd = {                                
				.osd_cfg =  LCD_OSDC_OSDEN      |               // Use OSD mode
				 LCD_OSDC_ALPHAEN                  |            // enable alpha
				 //LCD_OSDC_F0EN,                               // enable Foreground0   
				 LCD_OSDC_F1EN,                               // enable Foreground1   				 
		
				 .osd_ctrl = 0,                                 // disable ipu,          
				 .rgb_ctrl = 0,                                                                  
				 .bgcolor = 0x000000,                           // set background color Black    
				 .colorkey0 = 0,                                // disable colorkey
				 .colorkey1 = 0,                                // disable colorkey
				 .alpha = 0xa0,                                 // alpha value
				 .ipu_restart = 0x8000085d,                     // ipu restart
				 .fg_change = FG_CHANGE_ALL,            // change all initially
				 .fg0 = {32, 0, 0, 1920, 1080},   // bpp, x, y, w, h
				 .fg1 = {32, 0, 0, 1920, 1080},   // bpp, x, y, w, h
			},
		},
	},
	{
		.hdmi_type = HDMI_1080p50,
		.hdmi_info = {
			.panel = {                                                     
#if defined(CONFIG_HDMI_IT6610)
				.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
				       LCD_CFG_NEWDES | LCD_CFG_RECOVER |LCD_CFG_PCP,
				//	   LCD_CFG_HSP | LCD_CFG_VSP,
				//       LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,           
#else
				.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
				       LCD_CFG_NEWDES | LCD_CFG_RECOVER |                 
				       LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,            
#endif	
				.slcd_cfg = 0,                                            
				.ctrl = LCD_CTRL_BST_32,                                  
				//  width,height,freq,hsync,vsync,elw,blw,efw,bfw         
				1920,1080, 50, 44,5,528,148,  5, 35,  
			},                                      
			.osd = {                                
				.osd_cfg =  LCD_OSDC_OSDEN      |               // Use OSD mode
				 LCD_OSDC_ALPHAEN                  |            // enable alpha
				 //LCD_OSDC_F0EN,                               // enable Foreground0   
				 LCD_OSDC_F1EN,                               // enable Foreground1   				 
		
				 .osd_ctrl = 0,                                 // disable ipu,          
				 .rgb_ctrl = 0,                                                                  
				 .bgcolor = 0x000000,                           // set background color Black    
				 .colorkey0 = 0,                                // disable colorkey
				 .colorkey1 = 0,                                // disable colorkey
				 .alpha = 0xa0,                                 // alpha value
				 .ipu_restart = 0x8000085d,                     // ipu restart
				 .fg_change = FG_CHANGE_ALL,            // change all initially
				 .fg0 = {32, 0, 0, 1920, 1080},   // bpp, x, y, w, h
				 .fg1 = {32, 0, 0, 1920, 1080},   // bpp, x, y, w, h
			},
		},
	},
	{
		.hdmi_type = HDMI_1080p24,
		.hdmi_info = {
			.panel = {                                                     
#if defined(CONFIG_HDMI_IT6610)
				.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
				       LCD_CFG_NEWDES | LCD_CFG_RECOVER ,
				//	   LCD_CFG_HSP | LCD_CFG_VSP,
				//       LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,           
#else
				.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
				       LCD_CFG_NEWDES | LCD_CFG_RECOVER |                 
				       LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,            
#endif	
				.slcd_cfg = 0,                                            
				.ctrl = LCD_CTRL_BST_32,                                  
				//  width,height,freq,hsync,vsync,elw,blw,efw,bfw         
				1920,1080, 24, 44,5,638,148,  5, 35,  
			},                                      
			.osd = {                                
				.osd_cfg =  LCD_OSDC_OSDEN      |               // Use OSD mode
				 LCD_OSDC_ALPHAEN                  |            // enable alpha
				 //LCD_OSDC_F0EN,                               // enable Foreground0   
				 LCD_OSDC_F1EN,                               // enable Foreground1   				 
		
				 .osd_ctrl = 0,                                 // disable ipu,          
				 .rgb_ctrl = 0,                                                                  
				 .bgcolor = 0x000000,                           // set background color Black    
				 .colorkey0 = 0,                                // disable colorkey
				 .colorkey1 = 0,                                // disable colorkey
				 .alpha = 0xa0,                                 // alpha value
				 .ipu_restart = 0x8000085d,                     // ipu restart
				 .fg_change = FG_CHANGE_ALL,            // change all initially
				 .fg0 = {32, 0, 0, 1920, 1080},   // bpp, x, y, w, h
				 .fg1 = {32, 0, 0, 1920, 1080},   // bpp, x, y, w, h
			},
		},
	},
	{
		.hdmi_type = HDMI_1080p25,
		.hdmi_info = {
			.panel = {                                                     
#if defined(CONFIG_HDMI_IT6610)
				.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
				       LCD_CFG_NEWDES | LCD_CFG_RECOVER ,
				//	   LCD_CFG_HSP | LCD_CFG_VSP,
				//       LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,           
#else
				.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
				       LCD_CFG_NEWDES | LCD_CFG_RECOVER |                 
				       LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,            
#endif	
				.slcd_cfg = 0,                                            
				.ctrl = LCD_CTRL_BST_32,                                  
				//  width,height,freq,hsync,vsync,elw,blw,efw,bfw         
				1920,1080, 25, 44,5,528,148,  5, 35,  
			},                                      
			.osd = {                                
				.osd_cfg =  LCD_OSDC_OSDEN      |               // Use OSD mode
				 LCD_OSDC_ALPHAEN                  |            // enable alpha
				 //LCD_OSDC_F0EN,                               // enable Foreground0   
				 LCD_OSDC_F1EN,                               // enable Foreground1   				 
		
				 .osd_ctrl = 0,                                 // disable ipu,          
				 .rgb_ctrl = 0,                                                                  
				 .bgcolor = 0x000000,                           // set background color Black    
				 .colorkey0 = 0,                                // disable colorkey
				 .colorkey1 = 0,                                // disable colorkey
				 .alpha = 0xa0,                                 // alpha value
				 .ipu_restart = 0x8000085d,                     // ipu restart
				 .fg_change = FG_CHANGE_ALL,            // change all initially
				 .fg0 = {32, 0, 0, 1920, 1080},   // bpp, x, y, w, h
				 .fg1 = {32, 0, 0, 1920, 1080},   // bpp, x, y, w, h
			},
		},
	},
	{
		.hdmi_type = HDMI_1080p30,
		.hdmi_info = {
			.panel = {                                                     
#if defined(CONFIG_HDMI_IT6610)
				.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
				       LCD_CFG_NEWDES | LCD_CFG_RECOVER ,
				//	   LCD_CFG_HSP | LCD_CFG_VSP,
				//       LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,           
#else
				.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
				       LCD_CFG_NEWDES | LCD_CFG_RECOVER |                 
				       LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,            
#endif	
				.slcd_cfg = 0,                                            
				.ctrl = LCD_CTRL_BST_32,                                  
				//  width,height,freq,hsync,vsync,elw,blw,efw,bfw         
				1920,1080, 30, 44,5,528,148,  5, 35,  
			},                                      
			.osd = {                                
				.osd_cfg =  LCD_OSDC_OSDEN      |               // Use OSD mode
				 LCD_OSDC_ALPHAEN                  |            // enable alpha
				 //LCD_OSDC_F0EN,                               // enable Foreground0   
				 LCD_OSDC_F1EN,                               // enable Foreground1   				 
		
				 .osd_ctrl = 0,                                 // disable ipu,          
				 .rgb_ctrl = 0,                                                                  
				 .bgcolor = 0x000000,                           // set background color Black    
				 .colorkey0 = 0,                                // disable colorkey
				 .colorkey1 = 0,                                // disable colorkey
				 .alpha = 0xa0,                                 // alpha value
				 .ipu_restart = 0x8000085d,                     // ipu restart
				 .fg_change = FG_CHANGE_ALL,            // change all initially
				 .fg0 = {32, 0, 0, 1920, 1080},   // bpp, x, y, w, h
				 .fg1 = {32, 0, 0, 1920, 1080},   // bpp, x, y, w, h
			},
		},
	},
	
};

struct jz47xxlcd_info * jz47xx_set_hdmi_info(int index)
{
	int i;
	for(i = 0; i < sizeof(jz47xx_hdmi_info)/sizeof(jz47xx_hdmi_info[0]); i ++)
	{
		printk("i:%d, hdmi_type:%d  index:%d\r\n",i,jz47xx_hdmi_info[i].hdmi_type,index);
		if(jz47xx_hdmi_info[i].hdmi_type == index)
		{
			printk("match info i:%d, hdmi_type:%d  index:%d\r\n",i,jz47xx_hdmi_info[i].hdmi_type,index);
			return &jz47xx_hdmi_info[i].hdmi_info;
		}
	}
    printk("***** didn't match, use default hdmi_info !!!\n");
	return &jz47xx_hdmi_info[HDMI_Unkown].hdmi_info;
}

