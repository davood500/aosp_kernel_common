#include "edid.h"
#include "ep932api.h"

static int i, j;

int match_edid[HDMI_MAX_SUPPORT_NUM]=
{
	HDMI_720p60,
	HDMI_720p50,
	HDMI_576p50_16x9,
	HDMI_480p60_16x9,
	HDMI_640x480p60,
	HDMI_Unkown,
	HDMI_Unkown,
   	HDMI_Unkown 
};
//--------------------------------------------------------------------------------------------------

unsigned char edid_gethdmicap(unsigned char *ptarget)
{
	for(i = 0; i < 128; ++i) {
                if(i%16 == 0) DBG_PRINTK(("\r\n"));
                if(i%8 == 0) DBG_PRINTK((" "));
                DBG_PRINTK(("0x%02X, ", (int)ptarget[i] ));
        }

	if(ptarget[126] == 0x01) {
		for(i=4; i<ptarget[EDID_BLOCK_SIZE+2]; ++i) {
			if((ptarget[EDID_BLOCK_SIZE+i] & 0xE0) == 0x60) { // find tag code - Vendor Specific Block
				if( (ptarget[EDID_BLOCK_SIZE+1+i] == 0x03) && (ptarget[EDID_BLOCK_SIZE+2+i] == 0x0C) && (ptarget[EDID_BLOCK_SIZE+3+i] == 0x00) ) {

					return 1;
				}
			}
			else {
				i += (ptarget[EDID_BLOCK_SIZE+i] & 0x1F);
			}
		}
		if(i >= ptarget[EDID_BLOCK_SIZE+2]) { // Error, can not find the Vendor Specific Block

			return 0;
		}
	}
	return 0;
}

int edid_HdmiMatchVDB(unsigned char *ptarget,int hdmi_type)
{
   	unsigned char offset;
   	unsigned char count;
   	unsigned char tag;
   	int i,j;
	hdmi_type = 100;
	if(ptarget[126] == 0x01) {
		for(offset = EDID_BLOCK_SIZE + 4; offset < EDID_BLOCK_SIZE + ptarget[EDID_BLOCK_SIZE+2]; ++offset) {
			 tag = ptarget[offset] >> 5 ;
			 count = ptarget[offset] & 0x1f ;
			 if(tag == 0x02){// Video Data Block ;
				offset++;
				int vdb[count];
				for( i = 0; i < count ; i++, offset++ )
				{
					vdb[i]= ptarget[offset] & (~0x80) ;
                	if(i%16 == 0) DBG_PRINTK(("\r\n"));
                	if(i%8 == 0) DBG_PRINTK((" "));
                	DBG_PRINTK(("0x%02X, ", (int)vdb[i] ));
					if( ptarget[offset] & 0x80 )
					{
					}
					if(vdb[i] == hdmi_type)
					{
					   return hdmi_type;
					}
					
				}
				for(i = 0; i < HDMI_MAX_SUPPORT_NUM; i ++ )
				{
					for(j = 0; j < count; j ++)
					{
					   if(vdb[j] == match_edid[i]){
                			DBG_PRINTK(("match_edid i:%d  j:%d   vbd[j]: 0x%02X match_edid: %d, ",i,j, (int)vdb[j],(int)match_edid[i] ));
						  return match_edid[i];
					   }
					}
				}
			 }

			}
		}
	return hdmi_type;
}
unsigned char edid_getpcmfreqcap(unsigned char *ptarget)
{
	if(ptarget[126] >= 0x01) {
		for(i=4; i<ptarget[EDID_BLOCK_SIZE+2]; ++i) {
			if((ptarget[EDID_BLOCK_SIZE+i] & 0xE0) == 0x20) { // find tag code - Audio Data Block
				for(j=1; j<(ptarget[EDID_BLOCK_SIZE+i] & 0x1F); j+=3) {
					if((ptarget[EDID_BLOCK_SIZE+i+j] >> 3) == 1) {
						return ptarget[EDID_BLOCK_SIZE+i+j+1];
					}
				}
			}
			else {
				i += (ptarget[EDID_BLOCK_SIZE+i] & 0x1F);
			}
		}
		if(i>=ptarget[EDID_BLOCK_SIZE+2]) { // Error, can not find the Audio Data Block
			return 0x07;
		}
	}

	return 0x00;
}

unsigned char edid_getpcmchannelcap(unsigned char *ptarget)
{
	unsigned char max_pcm_channel = 1;
	if(ptarget[126] >= 0x01) {
		for(i=4; i<ptarget[EDID_BLOCK_SIZE+2]; ++i) {
			if((ptarget[EDID_BLOCK_SIZE+i] & 0xE0) == 0x20) { // find tag code - Audio Data Block
				for(j=1; j<(ptarget[EDID_BLOCK_SIZE+i] & 0x1F); j+=3) {
					if((ptarget[EDID_BLOCK_SIZE+i+j] >> 3) == 1) {
						//return ptarget[EDID_BLOCK_SIZE+i+j] & 0x07;
						max_pcm_channel = max(max_pcm_channel, ptarget[EDID_BLOCK_SIZE+i+j] & 0x07);
					}
				}
			}
			else {
				i += (ptarget[EDID_BLOCK_SIZE+i] & 0x1F);
			}
		}
		return max_pcm_channel;
	}

	return 0;
}

unsigned char edid_getdatablockaddr(unsigned char *ptarget, unsigned char tag)
{
	if(ptarget[126] >= 0x01) {
		for(i=4; i<ptarget[EDID_BLOCK_SIZE+2]; ++i) {
			if((ptarget[EDID_BLOCK_SIZE+i] & 0xE0) == tag) { // find tag code 
				return i+128;
			}
			else {
				i += (ptarget[EDID_BLOCK_SIZE+i] & 0x1F);
			}
		}
		if(i>=ptarget[EDID_BLOCK_SIZE+2]) { // Error, can not find
			return 0;
		}
	}
	return 0;
}

