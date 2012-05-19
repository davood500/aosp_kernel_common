#ifndef EP932_API_H
#define EP932_API_H

#include <linux/kernel.h>


#define DBG_PRINTK(x) printk x
//#define DBG_PRINTK(x) 

#ifndef min
#define min((a), (b))	((a) > (b)) ? (a) : (b)
#endif

typedef enum {
	// Master
	SMBUS_STATUS_SUCCESS = 0x00,
	SMBUS_STATUS_PENDING,//	SMBUS_STATUS_Abort,
	SMBUS_STATUS_NOACT = 0x02,
	SMBUS_STATUS_TIMEOUT,
	SMBUS_STATUS_ARBITRATIONLOSS = 0x04
}smbus_status;

void ep_ep932m_reset(void);
void hdmi_main (void);
unsigned char ep_hdmi_init(void); 

#endif
