#include <linux/kernel.h>
#include <linux/delay.h>

#include "ep932api.h"
#include "edid.h"
#include "ddc_if.h"
#include "ep932controller.h"
#include "ep932settingsdata.h"

#include "./i2c_drivers/hdmi.h"

#define AV_STABLE_TIME            1000

typedef enum {
	TXS_SEARCH_EDID,
	TXS_WAIT_UPSTREAM,
	TXS_STREAM,
	TXS_HDCP
} tx_state_t;

// HDCP Key  
unsigned char hdcp_key[64][8];

extern ep932c_register_map ep932c_registers;


unsigned char is_cap_hdmi;
unsigned char is_cap_ycc444;
unsigned char is_cap_ycc422;
unsigned char is_connected;

unsigned char is_receiver_sense;

unsigned char is_timing_changing;
unsigned char is_video_changing;
unsigned char is_audio_changing;
unsigned char is_hdcp_info_bksv_rdy;
unsigned char is_hot_plug;


// temp data
unsigned char chksum, vc_temp, connection_state,backup_analog_test_control;

unsigned char htplg_now = 0, htplg_last = 0;
unsigned char is_system_inited;

// system data
tx_state_t tx_state;





vdo_params video_params;
ado_params audio_params;
unsigned char gamut_packet_header_backup[3];

// Register
pep932c_register_map pep932c_registers;

// Private Functions
void ep932controller_reset(void);

void txs_rollback_wait_upstream(void);
void txs_rollback_stream(void);
void txs_rollback_hdcp(void);

// Hardware
void read_interruput_flags(void);
//void On_HDMI_Int();
void  ep_hdmi_dump_message(void);

ep932c_register_map ep932c_registers;
ep932c_callback ep932c_generateint;


void ep932controller_initial()
{
	// Save the Logical Hardware Assignment
	pep932c_registers = &ep932c_registers;//pep932c_regmap;

	ep_ep932m_reset();
	
	ep932_if_initial();
	is_system_inited = 0;
	connection_state = 0;
	is_cap_hdmi = 0;
	is_cap_ycc444 = is_cap_ycc422 = 0;
	is_connected = 0;
	is_video_changing = 0;
	is_audio_changing = 0;
	htplg_last = 0;
	tx_state = TXS_SEARCH_EDID;  
	memset(gamut_packet_header_backup, 0, 3);

	// Reset all EP932C registers
	memset(pep932c_registers, 0, sizeof(ep932c_register_map));
	pep932c_registers->video_interface[0] = 0x80;
	pep932c_registers->power_control = EP932E_POWER_CONTROL__PD_HDMI;
	pep932c_registers->audio_interface = 0x10; // 2 Channel audio

	// Update Version Registers
	pep932c_registers->vendorid = 0x177A;
	pep932c_registers->deviceid = 0x0932;
	pep932c_registers->version_major = VERSION_MAJOR;
	pep932c_registers->version_minor = VERSION_MINOR;
	DBG_PRINTK(("Version %d.%d\r\n", (int)VERSION_MAJOR, (int)VERSION_MINOR ));
	// Initial HDCP Info
	memset(pep932c_registers->hdcp_aksv, 0x00, sizeof(pep932c_registers->hdcp_aksv));
	memset(pep932c_registers->hdcp_bksv, 0x00, sizeof(pep932c_registers->hdcp_bksv));
	
	ep932_reg_read(EP932_CONFIGURATION, ddc_data, 1);

	// Set Revocation List address
	hdcp_extract_bksv_bcaps3(pep932c_registers->hdcp_bksv);
	hdcp_extract_fifo((unsigned char*)pep932c_registers->hdcp_ksv_fifo, sizeof(pep932c_registers->hdcp_ksv_fifo));
	hdcp_stop();

	// Reset EP932 Control Program
	ep932controller_reset();
				
//	txs_rollback_stream();
	//txs_rollback_wait_upstream();
}

void ep932controller_reset(void)
{
	smbus_status status = SMBUS_STATUS_SUCCESS;

	// Reset Hardware
	DBG_PRINTK(("Reset EP932\r\n"));

	ep_ep932m_reset();
	
	// Initial Variables
	ep932_reg_set_bit(EP932_PIXEL_REPETITION_CONTROL, EP932_PIXEL_REPETITION_CONTROL__OSCSEL);
	ep932_reg_read(EP932_PIXEL_REPETITION_CONTROL, ddc_data, 1);
	DBG_PRINTK(("EP932_PIXEL_REPETITION_CONTROL 0X%02x,0x%02x\r\n",ddc_data[0],EP932_PIXEL_REPETITION_CONTROL__OSCSEL));

	pep932c_registers->system_status &= ~EP932E_SYSTEM_STATUS__KEY_FAIL;
	pep932c_registers->system_configuration |= EP932E_SYSTEM_CONFIGURATION__HDCP_DIS;

#if 0
	// Check HDCP key and up load the key
	if(status) {
		// Do not upload the default Key!
		pep932c_registers->system_configuration |= EP932E_SYSTEM_CONFIGURATION__HDCP_DIS;
		pep932c_registers->system_status |= EP932E_SYSTEM_STATUS__KEY_FAIL;
		DBG_PRINTK(("No HDCP Key\r\n"));
	}
	else {
		// Check HDCP key and up load the key
		chksum = 0;
		for(i=0; i<328; ++i) {
			chksum += *((unsigned char *)hdcp_key+i);
		}	
		DBG_PRINTK(("HDCP Key Check Sum 0x%02X\r\n", (int)chksum ));
		if(hdcp_key[3][7] != 0x50 || hdcp_key[12][7] != 0x01 || chksum != 0x00) {// || hdcp_key[40][0] != 0xA5) {
			HDCP_Fake(1);
			pep932c_registers->system_status |= EP932E_SYSTEM_STATUS__KEY_FAIL;
			DBG_PRINTK(("Check Key failed!\r\n"));
			pep932c_registers->system_configuration |= EP932E_SYSTEM_CONFIGURATION__HDCP_DIS;
			//DBG_PRINTK(("Disable HDCP \r\n"));
		}
		else {
			// Upload the key 0-39
			for(i=0; i<40; ++i) {
				ddc_data[0] = (unsigned char)i;
				status |= ep932_reg_write(EP932_Key_Add, ddc_data, 1);
				memcpy(ddc_data,&hdcp_key[i][0],7);
				status |= ep932_reg_write(EP932_Key_Data, ddc_data, 7);
			}
			// Read and check	
			for(i=0; i<40; ++i) {
				ddc_data[0] = (unsigned char)i;
				status |= ep932_reg_write(EP932_Key_Add, ddc_data, 1);
				status |= ep932_reg_read(EP932_Key_Data, ddc_data, 7);
				if((memcmp(ddc_data,&hdcp_key[i][0],7) != 0) || status) {
					// Test failed
					HDCP_Fake(1);
					pep932c_registers->system_status |= EP932E_SYSTEM_STATUS__KEY_FAIL;
					DBG_PRINTK(("Check Key failed!\r\n"));
					pep932c_registers->system_configuration |= EP932E_SYSTEM_CONFIGURATION__HDCP_DIS;
					//DBG_PRINTK(("Disable HDCP \r\n"));
					break;
				}
			}
			// Upload final KSV 40
			ddc_data[0] = 40;
			status |= ep932_reg_write(EP932_Key_Add, ddc_data, 1);
			memcpy(ddc_data,&hdcp_key[40][0],7);
			status |= ep932_reg_write(EP932_Key_Data, ddc_data, 7);
			// Read back and check
	    	if(!hdmi_tx_read_aksv(pep932c_registers->hdcp_aksv)) {
				// Test failed
				HDCP_Fake(1);
				pep932c_registers->system_status |= EP932E_SYSTEM_STATUS__KEY_FAIL;
				DBG_PRINTK(("Check KSV failed!\r\n"));
				pep932c_registers->system_configuration |= EP932E_SYSTEM_CONFIGURATION__HDCP_DIS;
				//DBG_PRINTK(("Disable HDCP \r\n"));
			}
		}	
	}
#endif
	ep932_if_reset();

	is_receiver_sense = 0;

	// data

	if(tx_state > TXS_SEARCH_EDID) {
		DBG_PRINTK(("\r\nState Transist: Reset -> [TXS_WAIT_UPSTREAM]\r\n"));
		tx_state = TXS_WAIT_UPSTREAM;
	}

	DBG_PRINTK(("ep932controller_reset finish\r\n"));
}




void ep_hdmi_setaudfmt(hdmi_audfmt_t aud_fmt, hdmi_audfreq  aud_freq)
{
	if(aud_fmt == AUD_I2S)
	{
		pep932c_registers->audio_interface = 0x18;		// 2 channel IIS
		//DBG_PRINTK(("Audio interface is IIS - 2.0 CH, "));
	}
	else
	{
		pep932c_registers->audio_interface = 0x10;		// SPDIF
		//DBG_PRINTK(("Audio interface is SPDIF, "));
	}

	if(aud_freq == 0)
	{
		pep932c_registers->system_configuration = 0x02;
		//DBG_PRINTK(("(AUDIO MUTE)"));
	}
	else
	{
		pep932c_registers->system_configuration = 0x00;
	}
	pep932c_registers->audio_input_format = aud_freq;	// set Audio frequency
	DBG_PRINTK(("freq = "));
	switch(aud_freq)
	{
		case AUD_SF_32000HZ:
			DBG_PRINTK(("32K HZ\r\n"));
			break;
			
		case AUD_SF_44100HZ:
			DBG_PRINTK(("44.1K HZ\r\n"));
			break;
			
		case AUD_SF_48000HZ:
			DBG_PRINTK(("48K HZ\r\n"));
			break;
			
		case AUD_SF_88200HZ:
			DBG_PRINTK(("88.2K HZ\r\n"));
			break;
			
		case AUD_SF_96000HZ:
			DBG_PRINTK(("96K HZ\r\n"));
			break;
			
		case AUD_SF_176400HZ:
			DBG_PRINTK(("176.4K HZ\r\n"));
			break;
			
		case AUD_SF_192000HZ:
			DBG_PRINTK(("192K HZ\r\n"));
			break;
			
	}
}

void ep_hdmi_set_video_timing(int timing)
{
	unsigned char temp_setting = 0;
	DBG_PRINTK(("\r\n\r\n"));
	switch (timing)
	{
        case HDMI_640x480p60: 
			DBG_PRINTK(("timing is HDMI_640x480p60!!!!!!!!!\r\n"));
			pep932c_registers->video_input_format[0] = HDMI_640x480p60;//0x00;	// video format timing
			temp_setting =EDGE_RISING | BSEL_24BIT /*| EDGE_RISING */ /*| FMT_12*/;
			pep932c_registers->video_interface[0] = temp_setting;
			pep932c_registers->video_interface[1] = 0x00; 	// DE,HS,VS, RGB444
			pep932c_registers->power_control = 0x00;
			break;
        case HDMI_480p60_16x9: 
			DBG_PRINTK(("timing is HDMI_480p60_16x9!!!!!!!!!\r\n"));
			pep932c_registers->video_input_format[0] = HDMI_480p60_16x9;//0x00;	// video format timing
			temp_setting =EDGE_RISING | BSEL_24BIT /*| EDGE_RISING */ /*| FMT_12*/;
			pep932c_registers->video_interface[0] = temp_setting;
			pep932c_registers->video_interface[1] = 0x00; 	// DE,HS,VS, RGB444
			pep932c_registers->power_control = 0x00;
			break;
		case HDMI_720p60: 
			DBG_PRINTK(("timing is HDMI_720p60!!!!!!!!!\r\n"));
			pep932c_registers->video_input_format[0] = HDMI_720p60;//0x04;
			pep932c_registers->video_interface[0] = 0x86;
			pep932c_registers->video_interface[1] = 0x00;//0x0a;

			pep932c_registers->power_control = 0x00;
			break;
        case HDMI_1080i60: 
			DBG_PRINTK(("timing is HDMI_1080i60!!!!!!!!!\r\n"));
			pep932c_registers->video_input_format[0] = HDMI_1080i60;//0x05;
			pep932c_registers->video_interface[0] = 0x84;
			pep932c_registers->video_interface[1] = 0x00;//0x0a;
			pep932c_registers->power_control = 0x00;
			break;
	   case HDMI_576p50_16x9: 
			DBG_PRINTK(("timing is HDMI_576p50_16x9!!!!!!!!!\r\n"));
			ep932c_registers.video_input_format[0] = HDMI_576p50_16x9;		
			ep932c_registers.video_interface[0] = 0x86;		
			ep932c_registers.video_interface[1] = 0x00;//0x0a;
			ep932c_registers.power_control = 0x00;
		    break;
		case HDMI_720p50: 
			DBG_PRINTK(("timing is HDMI_720p50!!!!!!!!!\r\n"));
			ep932c_registers.video_input_format[0] = HDMI_720p50;
			ep932c_registers.video_interface[0] = 0x86;
			ep932c_registers.video_interface[1] = 0x00;//0x0a;
			ep932c_registers.power_control = 0x00;
			break;
   	}
	ep_hdmi_setaudfmt(AUD_I2S, AUD_SF_44100HZ);
	DBG_PRINTK(("##############################################\r\n"));

}

void ep932controller_start(int timing)
{	
	DBG_PRINTK(("hdmi_power_up\r\n"));
	pep932c_registers->power_control &= ~EP932E_POWER_CONTROL__PD_HDMI;
	// 	Power Up
	//	hdmi_tx_power_up();
	//	tx_state = TXS_STREAM;
	ep_hdmi_set_video_timing(timing);
	connection_state =1;
	ep932controller_task();
}
void ep932controller_stop(void)
{
	// Power Down
	pep932c_registers->power_control |= EP932E_POWER_CONTROL__PD_HDMI;
	txs_rollback_stream();
	tx_state = TXS_WAIT_UPSTREAM;
 //add for cma
        is_timing_changing = 1;
        is_video_changing=1;
        is_audio_changing=1;
 //add end
	hdmi_tx_vmute_enable();
	hdmi_tx_amute_enable();
//	connection_state =1;
}
int ep932controller_matchedid(int hdmi_type)
{
   	int i;
	ep932controller_htplg();
	ep932controller_readedid();
	if(pep932c_registers->system_status & EP932E_SYSTEM_STATUS__HTPLG) {
		return edid_HdmiMatchVDB(pep932c_registers->readed_edid,hdmi_type);			
	}
	return 0;
}
unsigned char ep932controller_htplg()
{
// Read Interrupt Flag and updat the internal information
		read_interruput_flags();		
		htplg_now = hdmi_tx_htplg();
		printk("HotPlug out htplg_now:%d###\r\n",htplg_now);
		if(htplg_now) {
			pep932c_registers->system_status |= EP932E_SYSTEM_STATUS__HTPLG;
		}
		else {


			pep932c_registers->system_status &= ~EP932E_SYSTEM_STATUS__HTPLG;
		}
		if(htplg_last != htplg_now)
		{
			htplg_last = htplg_now;			
			if(htplg_now == 0)
			{
				printk(("HotPlug out ###\r\n"));	
				ep_hdmi_dump_message();				
			}
			else
			{
				printk(("Detect HotPlug  ###\r\n"));
			}
			return 1;
		}
		else{
			return 0;
		}
}

unsigned char ep932controller_readedid(void)
{
			printk(("TXS_SEARCH_EDID  ###\r\n"));
			if(pep932c_registers->system_status & EP932E_SYSTEM_STATUS__HTPLG) {
					unsigned char edid_ddc_status;
					// Read EDID
					DBG_PRINTK(("\r\nState Transist: Read EDID -> [TXS_WAIT_UPSTREAM]\r\n"));				
					memset(pep932c_registers->readed_edid, 0xff, 256);
					edid_ddc_status = downstream_rx_read_edid(pep932c_registers->readed_edid);
					
					if(edid_ddc_status) {
						//if(edid_ddc_status == EDID_STATUS_NOACT) {
						if(edid_ddc_status != EDID_STATUS_CHECKSUMERROR) {
							DBG_PRINTK(("WARNING: EDID read failed 0x%02X\r\n", (int)edid_ddc_status));
						}
					}	 
					// Set Output
					if(pep932c_registers->system_configuration & EP932E_SYSTEM_CONFIGURATION__FORCE_HDMI_CAP) {
						is_cap_hdmi = 1;
					}
					else {
						is_cap_hdmi = edid_gethdmicap(pep932c_registers->readed_edid);
					}
					if(is_cap_hdmi) {			
						DBG_PRINTK(("Support HDMI"));

						// Default Capability
						is_cap_ycc444 =	is_cap_ycc422 = 0;
						pep932c_registers->edid_asfreq = 0x07;
						pep932c_registers->edid_achannel = 1;

						pep932c_registers->edid_videodataaddr = 0x00;
						pep932c_registers->edid_audiodataaddr = 0x00;
						pep932c_registers->edid_speakerdataaddr = 0x00;
						pep932c_registers->edid_vendordataaddr = 0x00;

						if(!edid_ddc_status) {

							if(pep932c_registers->readed_edid[131] & 0x20) {	// Support YCC444
								is_cap_ycc444 = 1;
								DBG_PRINTK((" YCC444"));
							}
							if(pep932c_registers->readed_edid[131] & 0x10) {	// Support YCC422
								is_cap_ycc422 = 1;
								DBG_PRINTK((" YCC422"));
							}
							DBG_PRINTK(("\r\n"));
							pep932c_registers->edid_asfreq = edid_getpcmfreqcap(pep932c_registers->readed_edid);
							DBG_PRINTK(("EDID ASFreq = 0x%02X\r\n",(int)pep932c_registers->edid_asfreq));

							pep932c_registers->edid_achannel = edid_getpcmchannelcap(pep932c_registers->readed_edid);
							DBG_PRINTK(("EDID AChannel = 0x%02X\r\n",(int)pep932c_registers->edid_achannel));

							pep932c_registers->edid_videodataaddr = edid_getdatablockaddr(pep932c_registers->readed_edid, 0x40);
							pep932c_registers->edid_audiodataaddr = edid_getdatablockaddr(pep932c_registers->readed_edid, 0x20);
							pep932c_registers->edid_speakerdataaddr = edid_getdatablockaddr(pep932c_registers->readed_edid, 0x80);
							pep932c_registers->edid_vendordataaddr = edid_getdatablockaddr(pep932c_registers->readed_edid, 0x60);
						}
					}
					else {
						DBG_PRINTK(("Support DVI RGB only\r\n"));
						is_cap_ycc444 =	is_cap_ycc422 = 0;
						pep932c_registers->edid_asfreq = pep932c_registers->edid_achannel = 0;
					}

					if(is_cap_hdmi)
						pep932c_registers->edid_status = edid_ddc_status | EP932E_EDID_STATUS__HDMI;
					else
						pep932c_registers->edid_status = edid_ddc_status;
					DBG_PRINTK(("Support Max Audio Channel %d\r\n", (int)pep932c_registers->edid_achannel+1));
					DBG_PRINTK(("Support Audio Freq 0x%02X\r\n", (int)pep932c_registers->edid_asfreq));

					// Report EDID Change
					pep932c_registers->interrupt_flags |= EP932E_INTERRUPT_FLAGS__EDID_CHG;
					if(ep932c_generateint && (pep932c_registers->interrupt_enable & EP932E_INTERRUPT_ENABLE__EDID_CHG) ) ep932c_generateint();
	
					tx_state = TXS_WAIT_UPSTREAM;
			}
			else {	
				pep932c_registers->edid_status = EDID_STATUS_NOACT;
			}

		
}

unsigned char ep932controller_wait_upstream(void)
{
			printk(("TXS_WAIT_UPSTREAM  ###\r\n"));
			if(!(pep932c_registers->system_status & EP932E_SYSTEM_STATUS__HTPLG)) {

				txs_rollback_wait_upstream();
				tx_state = TXS_SEARCH_EDID;
			}
			else if(!(pep932c_registers->power_control & (EP932E_POWER_CONTROL__PD_HDMI/* | EP932E_POWER_CONTROL__PD_TOT*/)) ) {
				DBG_PRINTK(("\r\nState Transist: Power Up -> [TXS_STREAM]\r\n"));							
				// Power Up
				hdmi_tx_power_up();
				tx_state = TXS_STREAM;
			}
#if 0
			else {
				// Check Force HDMI bit
				if(!is_cap_hdmi) {
					if(pep932c_registers->system_configuration & EP932E_SYSTEM_CONFIGURATION__FORCE_HDMI_CAP) {
						txs_rollback_wait_upstream();
						tx_state = TXS_SEARCH_EDID;
					}
				}
			}		
#endif
}
unsigned char ep932controller_stream(void)
{
			if(!(pep932c_registers->system_status & EP932E_SYSTEM_STATUS__HTPLG)) {

				txs_rollback_stream();
				txs_rollback_wait_upstream();
				tx_state = TXS_SEARCH_EDID;
			}
			else if(pep932c_registers->power_control & (EP932E_POWER_CONTROL__PD_HDMI/* | EP932E_POWER_CONTROL__PD_TOT*/) ) {
				pep932c_registers->power_control |= EP932E_POWER_CONTROL__PD_HDMI;
				printk(("TXS_STREAM TO WAIT UP  ###\r\n"));
				txs_rollback_stream();
				tx_state = TXS_WAIT_UPSTREAM;
				hdmi_tx_vmute_enable();
				hdmi_tx_amute_enable();
			}
			else{
				// Video Change Debouncing
				//ep932controller_updatevideoparams();
				if(is_video_changing) 
				{	
					    DBG_PRINTK(("### VideoChanging \r\n"));
						hdmi_tx_video_config(&video_params);   
						//if(is_timing_changing)
					   	  // hdmi_tx_audio_config(&audio_params);    
						is_video_changing = 0;
						// Report Video Change
						pep932c_registers->interrupt_flags |= EP932E_INTERRUPT_FLAGS__VIDEO_CHG;
						if(ep932c_generateint && (pep932c_registers->interrupt_enable & EP932E_INTERRUPT_ENABLE__VIDEO_CHG) ) ep932c_generateint();

				DBG_PRINTK(("### VideoChanging end^^^^^^^^^\r\n"));
				}
			
				ep932controller_updateaudioparams();
				// Audio Change Debouncing
				if(is_audio_changing) 
					{
					    DBG_PRINTK(("### AudioChanging \r\n"));
						hdmi_tx_audio_config(&audio_params);
						is_audio_changing = 0;					
						// Report Audio Change
						pep932c_registers->interrupt_flags |= EP932E_INTERRUPT_FLAGS__AUDIO_CHG;
						if(ep932c_generateint && (pep932c_registers->interrupt_enable & EP932E_INTERRUPT_ENABLE__AUDIO_CHG) ) ep932c_generateint();

					    DBG_PRINTK(("### AudioChanging end^^^^^^^^^^ \r\n"));

				}
				hdmi_tx_vmute_disable();
				hdmi_tx_amute_disable();
#if 1	
				// HDMI Mode
				if(!is_cap_hdmi || (pep932c_registers->system_configuration & EP932E_SYSTEM_CONFIGURATION__HDMI_DIS) ) {
					hdmi_tx_dvi();  // Set to DVI mode (The Info Frame and Audio Packets would not be send)
			 	}
				else {
					hdmi_tx_hdmi(); // Set to HDMI mode
				}
#endif
			}
}

unsigned char ep932controller_updatevideoparams(void)
{
			//
			// Update Video Params
			//
			printk(("Update Video Params  ###\r\n"));		
			// Video interface
			video_params.interface = pep932c_registers->video_interface[0];
		
			// Video Timing
			if(pep932c_registers->video_input_format[0]) { 
				// Manul set the Video Timing
				if(pep932c_registers->video_input_format[0] < 128) {
					video_params.videosettingindex = pep932c_registers->video_input_format[0];
				}
				else {
					video_params.videosettingindex = pep932c_registers->video_input_format[0] - (128 - EP932_VDO_SETTINGS_IT_START);
				}
			} 
		
			// Select Sync Mode
			video_params.syncmode = (pep932c_registers->video_interface[1] & EP932E_VIDEO_INTERFACE_SETTING_1__SYNC) >> 2;
		
			// Select Color Space
			switch(pep932c_registers->video_interface[1] & EP932E_VIDEO_INTERFACE_SETTING_1__COLOR) {
				default:
				case EP932E_VIDEO_INTERFACE_SETTING_1__COLOR__AUTO:
					switch(video_params.videosettingindex) {
						case  4: case  5: case 16: case 19: case 20: case 31: case 32: 
						case 33: case 34: case 39: case 40: case 41: case 46: case 47:		// HD Timing
							video_params.colorspace = COLORSPACE_709;
							break;
		
						default:
							if(video_params.videosettingindex && video_params.videosettingindex < EP932_VDO_SETTINGS_IT_START) { // SD Timing
								video_params.colorspace = COLORSPACE_601;
							}
							else {															// IT Timing
								video_params.colorspace = COLORSPACE_709;
							}
					}
					break;
				case EP932E_VIDEO_INTERFACE_SETTING_1__COLOR__601:
					video_params.colorspace = COLORSPACE_601;
					break;
				case EP932E_VIDEO_INTERFACE_SETTING_1__COLOR__709:
					video_params.colorspace = COLORSPACE_709;
					break;
			}
		
			// Set Input Format
			switch(pep932c_registers->video_interface[1] & EP932E_VIDEO_INTERFACE_SETTING_1__VIN_FMT) {
				default:
				case EP932E_VIDEO_INTERFACE_SETTING_1__VIN_FMT__RGB:
					video_params.formatin = COLORFORMAT_RGB;
					video_params.formatout = COLORFORMAT_RGB;
					break;
				case EP932E_VIDEO_INTERFACE_SETTING_1__VIN_FMT__YCC444:
					video_params.formatin = COLORFORMAT_YCC444;
					if(is_cap_ycc444) {
						video_params.formatout = COLORFORMAT_YCC444;
					}
					else if(is_cap_ycc422) {
						video_params.formatout = COLORFORMAT_YCC422;
					}
					else {
						video_params.formatout = COLORFORMAT_RGB;
					}
					break;
				case EP932E_VIDEO_INTERFACE_SETTING_1__VIN_FMT__YCC422:
					video_params.formatin = COLORFORMAT_YCC422;
					if(is_cap_ycc422) {
						video_params.formatout = COLORFORMAT_YCC422;
					}
					else if(is_cap_ycc444) {
						video_params.formatout = COLORFORMAT_YCC444;
					}
					else {
						video_params.formatout = COLORFORMAT_RGB;
					}
					break;
			}

			// DVI mode settings overwrite
			if(!is_cap_hdmi || (pep932c_registers->system_configuration & EP932E_SYSTEM_CONFIGURATION__HDMI_DIS) ) {
				video_params.formatout = COLORFORMAT_RGB;
			}
		
			// AFAR
			video_params.afarate = ((pep932c_registers->video_input_format[1] & EP932E_VIDEO_INPUT_FORMAT_1__AFAR) >> 4) | 0x08;
		
			// Video Change
			if(memcmp(&video_params, &pep932c_registers->video_params_backup, sizeof(vdo_params)) != 0) {
				if(memcmp(&video_params, &pep932c_registers->video_params_backup, 6) != 0) {
					is_timing_changing = 1;
				}
				
				pep932c_registers->video_params_backup = video_params;			
				is_video_changing = 1;
			}
			return is_video_changing;
		
}
unsigned char ep932controller_updateaudioparams(void)
{
		printk(("Update Audio Params  ###\r\n"));		
			//
			// Update Audio Params
			//
			audio_params.interface = pep932c_registers->audio_interface & 0x0F; // IIS, WS_M, WS_POL, SCK_POL
			audio_params.videosettingindex = video_params.videosettingindex;

			// Update Audio Channel Number
			if(ep932_vdo_settings[video_params.videosettingindex].pix_freq_type <= PIX_FREQ_27027KHZ) {
				audio_params.channelnumber = 1;
			}
			else {
				audio_params.channelnumber = min(((pep932c_registers->audio_interface & 0x70) >> 4), pep932c_registers->edid_achannel);
			}

			// Update VFS
			if(audio_params.videosettingindex < EP932_VDO_SETTINGS_IT_START) {
				// Pixel Clock Type shift (59.94/60)
				audio_params.vfs = (pep932c_registers->video_input_format[1] & EP932E_VIDEO_INPUT_FORMAT_1__VIF)? 1:0;
			}
			else {
				audio_params.vfs = 0;
			}
			audio_params.nocopyright = (pep932c_registers->audio_input_format & EP932E_AUDIO_INPUT_FORMAT__NOCOPYRIGHT)?1:0;
		
			// Write Frequency info (Use ADO_FREQ or Auto)
			switch( pep932c_registers->audio_input_format & EP932E_AUDIO_INPUT_FORMAT__ADO_FREQ ) {
		
				case EP932E_AUDIO_INPUT_FORMAT__ADO_FREQ__32000HZ:
					audio_params.inputfrequency = ADSFREQ_32000HZ;
					// Disable Down Sample
					audio_params.adsrate = 0;
					break;
		
				default:
				case EP932E_AUDIO_INPUT_FORMAT__ADO_FREQ__44100HZ:
					audio_params.inputfrequency = ADSFREQ_44100HZ;
					// Disable Down Sample
					audio_params.adsrate = 0;
					break;
		
				case EP932E_AUDIO_INPUT_FORMAT__ADO_FREQ__48000HZ:
					audio_params.inputfrequency = ADSFREQ_48000HZ;
					// Disable Down Sample
					audio_params.adsrate = 0;
					break;
		
				case EP932E_AUDIO_INPUT_FORMAT__ADO_FREQ__88200HZ:
					audio_params.inputfrequency = ADSFREQ_88200HZ;
					if(pep932c_registers->edid_asfreq & 0x08) { // 88.2kHZ
						// Disable Down Sample
						audio_params.adsrate = 0;
					}
					else {
						// Enable Down Sample 1/2
						audio_params.adsrate = 1;
					}
					break;
		
				case EP932E_AUDIO_INPUT_FORMAT__ADO_FREQ__96000HZ:
					audio_params.inputfrequency = ADSFREQ_96000HZ;
					if(pep932c_registers->edid_asfreq & 0x10) { // 96kHZ
						// Disable Down Sample
						audio_params.adsrate = 0;
					}
					else {
						if(pep932c_registers->edid_asfreq & 0x04) { // 48kHZ
							// Enable Down Sample 1/2
							audio_params.adsrate = 1;
						}
						else {
							// Enable Down Sample 1/3
							audio_params.adsrate = 2;
						}
					}
					break;
		
				case EP932E_AUDIO_INPUT_FORMAT__ADO_FREQ__176400HZ:
					audio_params.inputfrequency = ADSFREQ_176400HZ;
					if(pep932c_registers->edid_asfreq & 0x20) { // 176kHZ
						// Disable Down Sample
						audio_params.adsrate = 0;
					}
					else {
						if(pep932c_registers->edid_asfreq & 0x08) { // 88.2kHZ
							// Enable Down Sample 1/2
							audio_params.adsrate = 1;
						}
						else {
							// Enable Down Sample 1/4
							audio_params.adsrate = 3;
						}
					}
					break;
		
				case EP932E_AUDIO_INPUT_FORMAT__ADO_FREQ__192000HZ:
					audio_params.inputfrequency = ADSFREQ_192000HZ;
					if(pep932c_registers->edid_asfreq & 0x40) { // 192kHZ
						// Disable Down Sample
						audio_params.adsrate = 0;
					}
					else {
						if(pep932c_registers->edid_asfreq & 0x10) { // 96kHZ
							// Enable Down Sample 1/2
							audio_params.adsrate = 1;
						}
						else {
							// Enable Down Sample 1/4
							audio_params.adsrate = 3;
						}
					}
					break;
			}
		
			// Audio Change
			if(memcmp(&audio_params, &pep932c_registers->audio_params_backup, sizeof(ado_params)) != 0) {
				pep932c_registers->audio_params_backup = audio_params;	
				is_audio_changing = 1;
			}
			return is_audio_changing;
}

unsigned char ep932controller_proc(void)
{
	if(tx_state == TXS_SEARCH_EDID) {
		ep932controller_readedid();
	}
	ep932controller_updatevideoparams();
	if(is_timing_changing){
		ep932controller_reset();
		is_timing_changing = 0;
	}
	if(tx_state == TXS_WAIT_UPSTREAM) {
		ep932controller_wait_upstream();
	}			
	if(tx_state == TXS_STREAM){			
		ep932controller_stream();
	}

}

unsigned char ep932controller_task(void)
{
//		if(is_system_inited  || connection_state){
			if(ep932controller_htplg() || connection_state){
				ep932controller_proc();
				if(connection_state == 1){
					connection_state = 0;
					is_system_inited = 1;
				// Update TREG
					if(pep932c_registers->analog_test_control != backup_analog_test_control) {
						backup_analog_test_control = pep932c_registers->analog_test_control;
				
						if(pep932c_registers->analog_test_control & 0x01) {
							ep932_reg_set_bit(EP932_COLOR_SPACE_CONTROL, 0x01);
						}
						else {
							ep932_reg_clear_bit(EP932_COLOR_SPACE_CONTROL, 0x01);
						}
						if(pep932c_registers->analog_test_control & 0x02) {
							ep932_reg_set_bit(EP932_COLOR_SPACE_CONTROL, 0x02);
						}
						else {
							ep932_reg_clear_bit(EP932_COLOR_SPACE_CONTROL, 0x02);
						}
					}
				}
//			}		
			// Return the status
			if(pep932c_registers->power_control & (EP932E_POWER_CONTROL__PD_HDMI | EP932E_POWER_CONTROL__PD_TOT)) {
		
				return EP932C_TASK_IDLE;
				
			}
			else {
				return EP932C_TASK_PENDING;
			}
	}
}

void txs_rollback_wait_upstream(void)
{
	DBG_PRINTK(("\r\nState Rollback: Reset EDID -> [TXS_SEARCH_EDID]\r\n"));

	// Reset EDID
	memset(pep932c_registers->readed_edid, 0xFF, 256);

	// Report EDID Change
	pep932c_registers->interrupt_flags |= EP932E_INTERRUPT_FLAGS__EDID_CHG;
	if(ep932c_generateint && (pep932c_registers->interrupt_enable & EP932E_INTERRUPT_ENABLE__EDID_CHG) ) ep932c_generateint();

}

void txs_rollback_stream(void)
{
	DBG_PRINTK(("\r\nState Rollback: Power Down -> [TXS_WAIT_UPSTREAM]\r\n"));

	// Power Down
	hdmi_tx_power_down();

	// Reset HDCP Info
	memset(pep932c_registers->hdcp_bksv, 0x00, sizeof(pep932c_registers->hdcp_bksv));
	is_hdcp_info_bksv_rdy = 0;
}

void txs_rollback_hdcp(void)
{
	DBG_PRINTK(("\r\nState Rollback: Stop HDCP -> [TXS_STREAM]\r\n"));

	hdcp_stop();
	pep932c_registers->hdcp_status = 0;
	pep932c_registers->hdcp_state = 0;
}

//----------------------------------------------------------------------------------------------------------------------

void read_interruput_flags(void) 
{
	//DBG_PRINTK(("EP932 read_interruput_flags \r\n"));
	ep932_reg_read(EP932_GENERAL_CONTROL_2, ddc_data, 1);

	if(ddc_data[0] & EP932_GENERAL_CONTROL_2__RIF) {
		hdcp_ext_ri_trigger();
		// Clear the interrupt flag
		ddc_data[0] = EP932_GENERAL_CONTROL_2__RIF;
		ep932_reg_write(EP932_GENERAL_CONTROL_2, ddc_data, 1);
	}
/*
	// Clear the interrupt flag
	ddc_data[0] = EP932_GENERAL_CONTROL_2__RIF;
	ep932_reg_write(EP932_GENERAL_CONTROL_2, ddc_data, 1);
*/
}

//----------------------------------------------------------------------------------------------------------------------

void  ep_hdmi_dump_message(void)
{
	unsigned short temp_ushort;
	unsigned char temp_r[2];
	unsigned char reg_addr;

	// System Status
	DBG_PRINTK(("\r\n\r\n======= Dump EP932E information =======\r\n"));

	DBG_PRINTK(("\r\n[EDID Data]"));
	for(temp_ushort = 0; temp_ushort < 256; ++temp_ushort) {
		if(temp_ushort%16 == 0) DBG_PRINTK(("\r\n"));
		if(temp_ushort%8 == 0) DBG_PRINTK((" "));
		DBG_PRINTK(("0x%02X,", (int)ep932c_registers.readed_edid[temp_ushort] ));
	}
	DBG_PRINTK(("\r\n"));

	DBG_PRINTK(("\r\n[Revision & Configuration]\r\n"));
	DBG_PRINTK(("vendorid=0x%04X, ", ep932c_registers.vendorid ));
	DBG_PRINTK(("deviceid=0x%04X, ", ep932c_registers.deviceid ));
	DBG_PRINTK(("Version=%d.%d, CFG=0x%02X\r\n", (int)ep932c_registers.version_major, (int)ep932c_registers.version_minor, (int)ep932c_registers.configuration ));

	DBG_PRINTK(("\r\n[Interrupt Flags]\r\n"));
	DBG_PRINTK(("EDID_CHG=%d, ", (int)((ep932c_registers.interrupt_flags & EP932E_INTERRUPT_FLAGS__EDID_CHG)?1:0) ));
	DBG_PRINTK(("VIDEO_CHG=%d, ", (int)((ep932c_registers.interrupt_flags & EP932E_INTERRUPT_FLAGS__VIDEO_CHG)?1:0) ));
	DBG_PRINTK(("AUDIO_CHG=%d\r\n", (int)((ep932c_registers.interrupt_flags & EP932E_INTERRUPT_FLAGS__AUDIO_CHG)?1:0) ));

	DBG_PRINTK(("\r\n[System Status]\r\n"));
	DBG_PRINTK(("RSEN=%d, ", (int)((ep932c_registers.system_status & EP932E_SYSTEM_STATUS__RSEN)?1:0) ));
	DBG_PRINTK(("HTPLG=%d, ", (int)((ep932c_registers.system_status & EP932E_SYSTEM_STATUS__HTPLG)?1:0) ));
	DBG_PRINTK(("KEY_FAIL=%d, ", (int)((ep932c_registers.system_status & EP932E_SYSTEM_STATUS__KEY_FAIL)?1:0) ));
	DBG_PRINTK(("DEF_KEY=%d\r\n", (int)((ep932c_registers.system_status & EP932E_SYSTEM_STATUS__DEF_KEY)?1:0) ));

	DBG_PRINTK(("\r\n[EDID Status]\r\n"));
	DBG_PRINTK(("EDID_HDMI=%d, ", (int)((ep932c_registers.edid_status & EP932E_EDID_STATUS__HDMI)?1:0) ));
	DBG_PRINTK(("DDC_STATUS=%d\r\n", (int)(ep932c_registers.edid_status & 0x0F) ));
	DBG_PRINTK(("VIDEO_DATA_ADDR=0x%02X, ", (int)ep932c_registers.edid_videodataaddr ));
	DBG_PRINTK(("AUDIO_DATA_ADDR=0x%02X, ", (int)ep932c_registers.edid_audiodataaddr ));
	DBG_PRINTK(("SPEAKER_DATA_ADDR=0x%02X, ", (int)ep932c_registers.edid_speakerdataaddr ));
	DBG_PRINTK(("VENDOR_DATA_ADDR=0x%02X\r\n", (int)ep932c_registers.edid_vendordataaddr ));
	DBG_PRINTK(("ASFREQ=0x%02X, ", (int)ep932c_registers.edid_asfreq ));
	DBG_PRINTK(("ACHANNEL=%d\r\n", (int)ep932c_registers.edid_achannel ));

	DBG_PRINTK(("\r\n[Video Status]\r\n"));
	DBG_PRINTK(("interface=0x%02X, ", (int)ep932c_registers.video_params_backup.interface ));
	DBG_PRINTK(("videosettingindex=%d, ", (int)ep932c_registers.video_params_backup.videosettingindex ));
	DBG_PRINTK(("hvpol=%d, ", (int)ep932c_registers.video_params_backup.hvpol ));
	DBG_PRINTK(("syncmode=%d, ", (int)ep932c_registers.video_params_backup.syncmode ));
	DBG_PRINTK(("formatin=%d, ", (int)ep932c_registers.video_params_backup.formatin ));
	DBG_PRINTK(("formatout=%d, ", (int)ep932c_registers.video_params_backup.formatout ));
	DBG_PRINTK(("colorspace=%d, ", (int)ep932c_registers.video_params_backup.colorspace ));
	DBG_PRINTK(("afarate=%d\r\n", (int)ep932c_registers.video_params_backup.afarate ));

	DBG_PRINTK(("\r\n[Audio Status]\r\n"));
	DBG_PRINTK(("interface=0x%02X, ", (int)ep932c_registers.audio_params_backup.interface ));
	DBG_PRINTK(("videosettingindex=%d, ", (int)ep932c_registers.audio_params_backup.videosettingindex ));
	DBG_PRINTK(("channelnumber=%d, ", (int)ep932c_registers.audio_params_backup.channelnumber ));
	DBG_PRINTK(("adsrate=%d, ", (int)ep932c_registers.audio_params_backup.adsrate ));
	DBG_PRINTK(("inputfrequency=%d, ", (int)ep932c_registers.audio_params_backup.inputfrequency ));
	DBG_PRINTK(("VFS=%d, ", (int)ep932c_registers.audio_params_backup.vfs ));
	DBG_PRINTK(("nocopyright=%d\r\n", (int)ep932c_registers.audio_params_backup.nocopyright ));

	DBG_PRINTK(("\r\n[Power Control]\r\n"));
	DBG_PRINTK(("PD_HDMI=%d, ", (int)((ep932c_registers.power_control & EP932E_POWER_CONTROL__PD_HDMI)?1:0) ));
	DBG_PRINTK(("PD_TOT=%d\r\n", (int)((ep932c_registers.power_control & EP932E_POWER_CONTROL__PD_TOT)?1:0) ));

	DBG_PRINTK(("\r\n[System Configuration]\r\n"));
	DBG_PRINTK(("HDCP_DIS=%d, ", (int)((ep932c_registers.system_configuration & EP932E_SYSTEM_CONFIGURATION__HDCP_DIS)?1:0) ));
	DBG_PRINTK(("HDMI_DIS=%d, ", (int)((ep932c_registers.system_configuration & EP932E_SYSTEM_CONFIGURATION__HDMI_DIS)?1:0) ));
	DBG_PRINTK(("AUDIO_DIS=%d, ", (int)((ep932c_registers.system_configuration & EP932E_SYSTEM_CONFIGURATION__AUDIO_DIS)?1:0) ));
	DBG_PRINTK(("VIDEO_DIS=%d\r\n", (int)((ep932c_registers.system_configuration & EP932E_SYSTEM_CONFIGURATION__VIDEO_DIS)?1:0) ));

	DBG_PRINTK(("\r\n[Interrupt Enable]\r\n"));
	DBG_PRINTK(("EDID_CHG=%d, ", (int)((ep932c_registers.interrupt_enable & EP932E_INTERRUPT_ENABLE__EDID_CHG)?1:0) ));
	DBG_PRINTK(("VS_PERIOD_CHG=%d, ", (int)((ep932c_registers.interrupt_enable & EP932E_INTERRUPT_ENABLE__VIDEO_CHG)?1:0) ));
	DBG_PRINTK(("AS_FREQ_CHG=%d\r\n", (int)((ep932c_registers.interrupt_enable & EP932E_INTERRUPT_ENABLE__AUDIO_CHG)?1:0) ));

	DBG_PRINTK(("\r\n[Video interface 0]\r\n"));
	DBG_PRINTK(("DK=%d, ", (int)((ep932c_registers.video_interface[0] & EP932E_VIDEO_INTERFACE_SETTING_0__DK)?1:0) ));
	DBG_PRINTK(("DKEN=%d, ", (int)((ep932c_registers.video_interface[0] & EP932E_VIDEO_INTERFACE_SETTING_0__DKEN)?1:0) ));
	DBG_PRINTK(("DSEL=%d, ", (int)((ep932c_registers.video_interface[0] & EP932E_VIDEO_INTERFACE_SETTING_0__DSEL)?1:0) ));
	DBG_PRINTK(("BSEL=%d, ", (int)((ep932c_registers.video_interface[0] & EP932E_VIDEO_INTERFACE_SETTING_0__BSEL)?1:0) ));
	DBG_PRINTK(("EDGE=%d, ", (int)((ep932c_registers.video_interface[0] & EP932E_VIDEO_INTERFACE_SETTING_0__EDGE)?1:0) ));
	DBG_PRINTK(("FMT12=%d\r\n", (int)((ep932c_registers.video_interface[0] & EP932E_VIDEO_INTERFACE_SETTING_0__FMT12)?1:0) ));

	DBG_PRINTK(("\r\n[Video interface 1]\r\n"));
	DBG_PRINTK(("COLOR=%d, ", (int)((ep932c_registers.video_interface[1] & EP932E_VIDEO_INTERFACE_SETTING_1__COLOR)>>4) ));
	DBG_PRINTK(("SYNC=%d, ", (int)((ep932c_registers.video_interface[1] & EP932E_VIDEO_INTERFACE_SETTING_1__SYNC)>>2) ));
	DBG_PRINTK(("VIN_FMT=%d\r\n", (int)((ep932c_registers.video_interface[1] & EP932E_VIDEO_INTERFACE_SETTING_1__VIN_FMT)>>0) ));

	DBG_PRINTK(("\r\n[Audio interface]\r\n"));
	DBG_PRINTK(("CHANNEL=%d, ", (int) (ep932c_registers.audio_interface & EP932E_AUDIO_INTERFACE_SETTING__CHANNEL)>>4 ));
	DBG_PRINTK(("IIS=%d, ", (int)((ep932c_registers.audio_interface & EP932E_AUDIO_INTERFACE_SETTING__IIS)?1:0) ));
	DBG_PRINTK(("WS_M=%d, ", (int)((ep932c_registers.audio_interface & EP932E_AUDIO_INTERFACE_SETTING__WS_M)?1:0) ));
	DBG_PRINTK(("WS_POL=%d, ", (int)((ep932c_registers.audio_interface & EP932E_AUDIO_INTERFACE_SETTING__WS_POL)?1:0) ));
	DBG_PRINTK(("SCK_POL=%d\r\n", (int)((ep932c_registers.audio_interface & EP932E_AUDIO_INTERFACE_SETTING__SCK_POL)?1:0) ));	


	DBG_PRINTK(("\r\n[Video Input Format 0]\r\n"));
	DBG_PRINTK(("VIC=%d\r\n", (int)ep932c_registers.video_input_format[0] ));	

	DBG_PRINTK(("\r\n[Video Input Format 1]\r\n"));
	DBG_PRINTK(("AFAR_VIF=0x%02X\r\n", (int)ep932c_registers.video_input_format[1] ));	


	DBG_PRINTK(("\r\n[EP932 Register value]"));
	for(reg_addr = 0; reg_addr<=0x88; reg_addr++)
	{
		ep932_reg_read(reg_addr, temp_r, 1);
		if(reg_addr%8 == 0)DBG_PRINTK(("\r\n"));
		DBG_PRINTK(("[%02X]%02X, ",(int)reg_addr,(int)temp_r[0]));
	}
	DBG_PRINTK(("\r\n"));
}


