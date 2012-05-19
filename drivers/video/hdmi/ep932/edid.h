/******************************************************************************\

          (c) Copyright Explore Semiconductor, Inc. Limited 2005
                           ALL RIGHTS RESERVED 

--------------------------------------------------------------------------------

  File        :  Edid.h 

  Description :  Head file of Edid IO Interface 

\******************************************************************************/

#ifndef EDID_H
#define EDID_H

#define EDID_BLOCK_SIZE  128
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
#define HDMI_MAX_SUPPORT_NUM	8


// Structure Definitions

extern unsigned char edid_gethdmicap(unsigned char *ptarget);
extern int edid_HdmiMatchVDB(unsigned char *ptarget,int hdmi_type);
extern unsigned char edid_getpcmfreqcap(unsigned char *ptarget);
extern unsigned char edid_getpcmchannelcap(unsigned char *ptarget);
extern unsigned char edid_getdatablockaddr(unsigned char *ptarget, unsigned char tag);

#endif // EDID_H


