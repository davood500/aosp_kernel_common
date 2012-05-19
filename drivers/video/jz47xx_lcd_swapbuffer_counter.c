/*
 *
 * for cts test
 * cts/tests/tests/view/src/android/view/cts/DisplayRefreshRateTest.java
 *
*/


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/gfp.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <linux/earlysuspend.h>
#include <linux/pm.h>
#include <linux/leds.h>
#include <linux/wait.h>
#include <linux/timer.h>
#include <linux/list.h>


#include "jz47xx_lcd_swapbuffer_counter.h"

//#define FBIO_GET_FPS       0x5000 /* get fps */

#ifdef JZ47XX_LCD_SWAPBUFFER_GET_LATEST_FPS

#define DBG(aaa, bbb...) //printk(aaa, ##bbb)


#define MAX_RECORD_SECOND 9		/* 9 sec;  remove the lastest second, valid 8 second */
#define LATEST_SECOND MAX_RECORD_SECOND /* 8sec, must match cts/tests/tests/view/src/android/view/cts/DisplayRefreshRateTest.java TEST_SECONDS   = 8.0f */


struct swap_count_struct {
	unsigned int sec;		/* second id */
	unsigned int cnt;		/* swap count in this second */
};

static struct swap_count_struct g_swap_bank[MAX_RECORD_SECOND] = {{0,}};

static unsigned int get_latest_sec(void)
{
	int iii;
	struct swap_count_struct *l;
	unsigned int sec;

	l = &g_swap_bank[0];
	sec = 0;

	for (iii=0; iii < MAX_RECORD_SECOND; iii++) {
		DBG("get_latest_sec() l->sec=%x, sec=%x\n", l->sec, sec);
		if (l->sec > sec) {
			sec = l->sec;
		}
		l++;
	}

	DBG("get_latest_sec() sec=%x\n", sec);

	return sec;
}

static struct swap_count_struct * get_my_swap_count(unsigned int sec)
{
	int iii;
	struct swap_count_struct * sc, *l, *old;

	sc = NULL;
	old = l = &g_swap_bank[0];

	for (iii=0; iii < MAX_RECORD_SECOND; iii++) {
		if (l->sec == sec) {
			sc = l;
			break;
		}
		if (l->sec <= (sec - LATEST_SECOND)) {
			old = l;
		}
		l++;
	}

	/* if not, create new record */
	if (sc == NULL) {
		sc = old;
		sc->sec = sec;
		sc->cnt = 0;
	}

	return sc;
}


int jz47xx_lcd_swapbuffer_count_increase(int n)
{
	struct timeval tv1;
	struct swap_count_struct * sc;
	unsigned int sec;

	do_gettimeofday(&tv1);
	sec = (unsigned int)tv1.tv_sec;

	/* get swap count struct */
	sc = get_my_swap_count(sec);
	if (sc == NULL) {
		return 0;
	}

	sc->cnt++;

	return 1;
}


int jz47xx_lcd_swapbuffer_get_latest_fps(int t)
{
	struct swap_count_struct *l;
	int iii;
	unsigned int latest;
	unsigned int count;
	int fps;

	latest = get_latest_sec();

	l = &g_swap_bank[0];
	count = 0;
	for (iii=0; iii < MAX_RECORD_SECOND; iii++) {
		DBG("iii: %d  l->sec=%x, cnt=%d\n", iii, l->sec, l->cnt);
		if (l->sec > (latest - LATEST_SECOND) && l->sec != latest ) { /* latest sec maybe 0.5 second valid, remove the latest */
			count += l->cnt;
			DBG("count=%d \n", count);
		}
		l++;
	}

	fps = count/(LATEST_SECOND-1); // remove the lastest second
	DBG("jz47xx_lcd_swapbuffer_get_latest_fps(int t) fps=%d\n", fps);

	fps = (count+((LATEST_SECOND-1)>>1))/(LATEST_SECOND-1); // remove the lastest second
	DBG("jz47xx_lcd_swapbuffer_get_latest_fps(int t) fps=%d\n", fps);

	return fps;
}


#endif // JZ47XX_LCD_SWAPBUFFER_GET_LATEST_FPS

