#ifndef __BTL_TS_H__
#define __BTL_TS_H__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/device.h>

#include <linux/interrupt.h>

#include <asm/system.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/bitops.h>
#include <asm/processor.h>
#include <asm/jzsoc.h>

#define BTL_RREG8(addr)	*((volatile unsigned char *)(addr))
#define BTL_RREG16(addr)	*((volatile unsigned short *)(addr))
#define BTL_RREG32(addr)	*((volatile unsigned int *)(addr))

#define BTL_WREG8(addr , value)	(*((volatile unsigned char *)(addr)) = (volatile unsigned char)(value))
#define BTL_WREG16(addr , value)	(*((volatile unsigned short *)(addr)) = (volatile unsigned short)(value))
#define BTL_WREG32(addr , value)	(*((volatile unsigned int *)(addr)) = (volatile unsigned int)(value))

#define MAX_PID_NUM                     15
#define TSSI_PACK_SIZE           19200		//192 * 100
//#define TSSI_PACK_SIZE           4800		//9600		//192 * 100
//#define TSSI_PACK_SIZE           19200		//192 * 100
//#define TSSI_PACK_SIZE           1880
//#define TSSI_PACK_SIZE           15040 // 188 x 80

#define BBDC_TPPKTMAX 			1636

//#define CPM_CLKGR_TSSI    	(1 << 26)		//CIM Interface	-- Demod lock is fail
#define CPM_CLKGR_TSSI    	(1 << 9)	//TSSI Interface
#define __cpm_start_tssi()       (REG_CPM_CLKGR &= ~CPM_CLKGR_TSSI)
#define __cpm_stop_tssi()       (REG_CPM_CLKGR |= CPM_CLKGR_TSSI)

#define __tssi_set_trigger_num(n)			\
	do {						\
		REG_TSSI_CFG &= ~TSSI_CFG_TRIG_MASK;	\
		REG_TSSI_CFG |= TSSI_CFG_TRIG_##n;	\
	} while (0)

#define CPM_CLKGR_DMAC         (1 << 21)
#define __cpm_start_dmac()	(REG_CPM_CLKGR &= ~CPM_CLKGR_DMAC)
//#define __cpm_stop_dmac()       (REG_CPM_CLKGR |= CPM_CLKGR_DMAC)

/* DTV_VCC_EN as PWM2 */
#define DTV_PWM2 (32*0 + 25) //as it set GPA25 in linden111(ingenic)	

#define __dtv_special_on() \
do { \
	__gpio_clear_pin(DTV_PWM2); \
} while (0)

#define __dtv_special_off() \
do { \
	__gpio_as_output(DTV_PWM2); \
	__gpio_set_pin(DTV_PWM2);		\
} while (0)

/* DTV_RST as DTV Reset */
#define DTV_RST (32*0+26) // as in linden111(ingenic), it set as GPA26 	

#define __dtv_reset_init() \
do { \
		__gpio_as_output(DTV_RST);		\
} while (0)

#define __dtv_reset_low() \
do { \
	__gpio_clear_pin(DTV_RST);	\
} while (0)

#if 1
#define __dtv_reset_high() \
do { \
	__gpio_set_pin(DTV_RST);	\
} while (0)
#else
#define __dtv_reset_high() \
do { \
	__gpio_disable_pull(DTV_RST);	\
	__gpio_set_pin(DTV_RST);	\
	__gpio_as_output(DTV_RST);	\
} while (0)
#endif

#define BBDRV_MAJOR 222U
#define BBDRV_NAME "ISDBTDemod"

#define BBIOCTL_TSOPEN        _IOW( BBDRV_MAJOR, 0x00U, unsigned int )
#define BBIOCTL_TSCLOSE       _IOW( BBDRV_MAJOR, 0x01U, unsigned int )
#define	BBIOCTL_TSDUMP		  _IOW( BBDRV_MAJOR, 0x02U, unsigned int )
#define	BBIOCTL_CHIPRESET	  _IOW( BBDRV_MAJOR, 0x03U, unsigned int )
#define	BBIOCTL_TSRESET	  	  _IOW( BBDRV_MAJOR, 0x04U, unsigned int )

struct tssi_buf
{
	unsigned int      *buf;
	unsigned int       pos;
	unsigned int       index;
	struct tssi_buf   *next;
};

struct jz_tssi_buf_ring_t
{
	struct tssi_buf	   *front;
	struct tssi_buf	   *rear;
	unsigned int        fu_num;
};

struct jz_tssi_t
{
	int 		      		pid_num;
	int 	       			dma_chan;
	struct task_struct	       *tssi_thread;
	struct jz_tssi_buf_ring_t      *cur_buf;
  	struct mutex			buf_lock;
};

#endif /* __BTL_TS_H__ */

