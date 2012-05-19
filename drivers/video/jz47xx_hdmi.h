#ifndef __JZ47XX_HDMI_H__
#define __JZ47XX_HDMI_H__
#include "jz47xx_android_lcd.h"
typedef enum tagHDMI_Video_Type {
HDMI_Unkown = 0 ,
HDMI_640x480p60 = 1 ,
HDMI_480p60,
HDMI_480p60_16x9,
HDMI_720p60,
HDMI_1080i60,
HDMI_480i60,
HDMI_480i60_16x9,
HDMI_1080p60 = 16,
HDMI_576p50,
HDMI_576p50_16x9,
HDMI_720p50,
HDMI_1080i50,
HDMI_576i50,
HDMI_576i50_16x9,
HDMI_1080p50 = 31,
HDMI_1080p24,
HDMI_1080p25,
HDMI_1080p30,
} HDMI_Video_Type ;

#define HDMI_MAX_NUM	7

struct jz47xx_hdmi_info_t {
	int hdmi_type;
	struct jz47xxlcd_info hdmi_info;
};

struct jz47xxlcd_info * jz47xx_set_hdmi_info(int);


#endif	/* __JZ47XX_HDMI_H__ */
