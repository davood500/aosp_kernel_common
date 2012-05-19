/*
 * linux/arch/mips/jz4770/pm.c
 * 
 * JZ4770 Power Management Routines
 * 
 * Copyright (C) 2006 - 2010 Ingenic Semiconductor Inc.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 * 
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 * 
 */

#include <linux/init.h>
#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/proc_fs.h>
#include <linux/syscalls.h>
#include <linux/fs.h> 
#include <linux/sysctl.h>
#include <asm/cacheops.h>
#include <asm/jzsoc.h>
#include <linux/writeback.h>

extern void jz4770_irq_suspend(void);
extern void jz4770_irq_resume(void);
	
#undef DEBUG
//#define DEBUG 
#ifdef DEBUG
#define dprintk(x...)	printk(x)
void print_gpio(void)
{
	for(i=0;i<GPIO_PORT_NUM;i++) {
		dprintk("pxint:%x pxmsk:%x pxpat1:%x pxpat0:%x pxpen:%x\n",        \
				REG_GPIO_PXINT(i),REG_GPIO_PXMSK(i),REG_GPIO_PXPAT1(i),REG_GPIO_PXPAT0(i),REG_GPIO_PXPEN(i));
	}
}
#else
#define dprintk(x...)
#define print_gpio()
#endif

void jz_flush_cache_all(void);

extern void wakeup_key_setup(void);
#if defined(CONFIG_I2S_DLV_4770)
extern void audio_disable_speaker(void);
extern void audio_enable_hp_mute(void);
#endif
#if defined(CONFIG_PM_POWERDOWN_P0)
extern void jz_cpu_sleep(void);
extern void jz_cpu_resume(void);
#endif

#if defined(CONFIG_INPUT_WM831X_ON)
extern void wm8310_power_off(void);
#endif

#if defined(CONFIG_BATTERY_JZ47XX) && defined(CONFIG_RTC_DRV_JZ4770)
extern unsigned int jz_read_battery_p0(void);
unsigned long pm_read_bat(void)
{
	int i,retval = 0;
	for(i=0;i<10;i++) {
		retval += jz_read_battery_p0();
	}
	retval /= 10;

	retval = retval * 1200 / 4096 * 4;
	return retval * 1000;
}

extern int g_jz_battery_min_voltage;
static unsigned int usr_alarm_data = 0;
static int alarm_state = 0;
#endif

static unsigned int gpio_output_low[6];
static unsigned int gpio_output_high[6];
static unsigned int gpio_intput_pull[6];
static unsigned int gpio_intput_nopull[6];
extern int gpio_sleep_state_table[][2];
extern void board_power_off(void);
/* NOTES:
 * 1: Pins that are floated (NC) should be set as input and pull-enable.
 * 2: Pins that are pull-up or pull-down by outside should be set as input 
 *    and pull-disable.
 * 3: Pins that are connected to a chip except sdram and nand flash 
 *    should be set as input and pull-disable, too.
 */
void jzsoc_do_sleep(unsigned long *ptr)
{
	unsigned char i;

	/* Print messages of GPIO registers for debug */
	print_gpio();
	/* Save GPIO registers */

	for(i = 0; i < GPIO_PORT_NUM; i++) {
		*ptr++ = REG_GPIO_PXINT(i);
		*ptr++ = REG_GPIO_PXMSK(i);
		*ptr++ = REG_GPIO_PXPAT1(i);
		*ptr++ = REG_GPIO_PXPAT0(i);
		*ptr++ = REG_GPIO_PXPEN(i);
	}

	for(i = 0; i < GPIO_PORT_NUM; i++) {
		__gpio_group_as_output_low(i, gpio_output_low[i]);
		__gpio_group_as_output_high(i, gpio_output_high[i]);
		__gpio_group_as_input_pull(i, gpio_intput_pull[i]);
		__gpio_group_as_input_nopull(i, gpio_intput_nopull[i]);
	}

#ifdef DEBUG
	/* Keep uart function for printing debug message */
	__gpio_as_uart0();
	__gpio_as_uart1();
	__gpio_as_uart2();
	__gpio_as_uart3();

	/* Print messages of GPIO registers for debug */
	print_gpio();
#endif
	wakeup_key_setup();
}

void jzsoc_do_resume(unsigned long *ptr)
{
	unsigned char i;

	/* Restore GPIO registers */
	for(i = 0; i < GPIO_PORT_NUM; i++) {
		REG_GPIO_PXINTS(i) = *ptr;
		REG_GPIO_PXINTC(i) = ~(*ptr++);
		REG_GPIO_PXMSKS(i) = *ptr;
		REG_GPIO_PXMSKC(i) = ~(*ptr++);
		REG_GPIO_PXPAT1S(i) = *ptr;
		REG_GPIO_PXPAT1C(i) = ~(*ptr++);
		REG_GPIO_PXPAT0S(i) = *ptr;
		REG_GPIO_PXPAT0C(i) = ~(*ptr++);
		REG_GPIO_PXPENS(i) = *ptr;
		REG_GPIO_PXPENC(i) = ~(*ptr++);
	}

	/* Print messages of GPIO registers for debug */
	print_gpio();
}

extern unsigned int dirty_writeback_interval;
extern unsigned int dirty_expire_interval;

/* Put CPU to HIBERNATE mode 
 *----------------------------------------------------------------------------
 * Power Management sleep sysctl interface
 *
 * Write "mem" to /sys/power/state invokes this function 
 * which initiates a poweroff.
 */
void pm_sync_filesystem(void)
{
	int dirty_writeback_interval_save = dirty_writeback_interval * 10;
	int dirty_expire_interval_save = dirty_expire_interval * 10;
	
	dirty_writeback_interval = 0;
	dirty_expire_interval = 0;
	
	msleep(dirty_writeback_interval_save * 2);
	msleep(dirty_expire_interval_save * 2);
	msleep(1000);
}

void jz_pm_hibernate(void)
{
	printk("Sync Filesystem ...");
	pm_sync_filesystem();
	printk("done.\n");
#if defined(CONFIG_I2S_DLV_4770)
	audio_enable_hp_mute();
	audio_disable_speaker();
#endif
	__lcd_close_backlight();
	board_power_off();
#if 1
#if defined(CONFIG_INPUT_WM831X_ON)
	printk("The power will be off.\n");
	wm8310_power_off();
	while(1);
#endif
#if defined(CONFIG_PMU_ACT8930_SUPPORT)
	__pmu_power_off();
#endif

	printk("Put CPU into hibernate mode.\n");

	/* Mask all interrupts */
	OUTREG32(INTC_ICMSR(0), 0xffffffff);
	OUTREG32(INTC_ICMSR(1), 0x7ff);

	/* 
	 * RTC Wakeup or 1Hz interrupt can be enabled or disabled 
	 * through  RTC driver's ioctl (linux/driver/char/rtc_jz.c).
	 */

	/* Set minimum wakeup_n pin low-level assertion time for wakeup: 100ms */
	rtc_write_reg(RTC_HWFCR, HWFCR_WAIT_TIME(2000));

	/* Set reset pin low-level assertion time after wakeup: must  > 60ms */
	rtc_write_reg(RTC_HRCR, HRCR_WAIT_TIME(60));

	/* Scratch pad register to be reserved */
	rtc_write_reg(RTC_HSPR, HSPR_RTCV);

	/* clear wakeup status register */
	rtc_write_reg(RTC_HWRSR, 0x0);

	/* set wake up valid level as low  and disable rtc alarm wake up.*/
	rtc_write_reg(RTC_HWCR,0x8);

	/* Put CPU to hibernate mode */
	rtc_write_reg(RTC_HCR, HCR_PD);

	while (1) {
		printk("We should NOT come here, please check the jz4770rtc.h!!!\n");
	};
	/* We can't get here */
#endif
}


#if (defined(CONFIG_BATTERY_JZ) || defined(CONFIG_BATTERY_JZ47XX)) && defined(CONFIG_RTC_DRV_JZ4770)
#define ALARM_TIME (60 * 60)
static void jz_set_alarm(void)
{
	uint32_t rtc_rtcsr,rtc_rtcsar;

	rtc_clr_reg(RTC_RTCCR,0x7<<2);
	rtc_rtcsr = rtc_read_reg(RTC_RTCSR);
	rtc_rtcsar = rtc_read_reg(RTC_RTCSAR);

	if((rtc_rtcsar < rtc_rtcsr + ALARM_TIME * 2) && (rtc_rtcsar > rtc_rtcsr)) {
		alarm_state = 0;
		usr_alarm_data = 0;
		rtc_set_reg(RTC_RTCCR,0x3<<2);
	} else {
		alarm_state = 1;
		usr_alarm_data = rtc_rtcsar;
		rtc_write_reg(RTC_RTCSAR,rtc_rtcsr + ALARM_TIME);
		rtc_set_reg(RTC_RTCCR,0x3<<2);
	}
}
#undef ALARM_TIME
#endif

unsigned int jz_set_div(unsigned int div)
{
	unsigned int cpccr;
	unsigned int retval;

	cpccr = INREG32(CPM_CPCCR);
	retval = cpccr;
	cpccr = (cpccr & CPCCR_MEM) | CPCCR_CE | div;
	cache_prefetch(jz_set_div_L1);
jz_set_div_L1:
	SETREG32(CPM_CPPSR,0x1);
	OUTREG32(CPM_CPCCR,cpccr);
	while(!(INREG32(CPM_CPPCR0) & CPPCR0_PLLS));

	return retval;
}

static int jz_pm_do_sleep(void)
{ 
	unsigned int h1div,c1div,cdiv,div,cpccr;
	unsigned long nfcsr = REG_NEMC_NFCSR;
	unsigned long opcr = INREG32(CPM_OPCR);
	unsigned long sadc = INREG8(SADC_ADENA);
	//unsigned long pmembs0 = REG_EMC_PMEMBS0;
	unsigned long sleep_gpio_save[5*(GPIO_PORT_NUM)];
#if defined(CONFIG_JZ4770_BOOT_FROM_MSC0)
	unsigned long msc0cdr = INREG32(CPM_MSC0CDR);
#endif

#if (defined(CONFIG_BATTERY_JZ) || defined(CONFIG_BATTERY_JZ47XX)) && defined(CONFIG_RTC_DRV_JZ4770)
__jz_pm_do_sleep_start:
	jz_set_alarm();
#endif
	jz4770_irq_suspend();

	/* set SLEEP mode */
	CMSREG32(CPM_LCR, LCR_LPM_SLEEP, LCR_LPM_MASK);

	/* Disable nand flash */
	REG_NEMC_NFCSR = ~0xff;

	/* stop sadc */
	SETREG8(SADC_ADENA,ADENA_POWER);
	while ((INREG8(SADC_ADENA) & ADENA_POWER) != ADENA_POWER) {
		dprintk("INREG8(SADC_ADENA) = 0x%x\n",INREG8(SADC_ADENA));
		udelay(100);
	}

	/* stop uhc */
	SETREG32(CPM_OPCR, OPCR_UHCPHY_DISABLE);

	/* set Oscillator Stabilize Time*/
	SETREG32(CPM_OPCR, 0x7f<<8);

	/* if on suspend otg and gps */
	CLRREG32(CPM_OPCR, OPCR_OTGPHY_ENABLE);

#if defined(CONFIG_RTC_DRV_JZ4770)
	/* unmask rtc interrupts */
	OUTREG32(INTC_ICMCR(1), 0x1);
#endif
	/* Sleep on-board modules */
	jzsoc_do_sleep(sleep_gpio_save);

	/* disable externel clock Oscillator in sleep mode */
	CLRREG32(CPM_OPCR, OPCR_O1SE);

	/* select 32K crystal as RTC clock in sleep mode */
	SETREG32(CPM_OPCR, OPCR_ERCS);

#if defined(CONFIG_JZ4770_BOOT_FROM_MSC0)
	/* select EXTAL clock for sd boot wakeup */
	CLRREG32(CPM_MSC0CDR, MSCCDR_MCSG);
	OUTREG32(CPM_MSC0CDR, MSCCDR_MPCS | (cpm_get_pllout1() / JZ_EXTAL - 1));
	cpm_start_clock(CGM_MSC0);
#endif

#if 1 /*for cpu 533M 1:2:4*/
	OUTREG32(CPM_PSWC0ST, 0);
	OUTREG32(CPM_PSWC1ST, 8);
	OUTREG32(CPM_PSWC2ST, 11);
	OUTREG32(CPM_PSWC3ST, 0);
#endif
#if 0 /*for cpu 600M 1:2:4*/
	OUTREG32(CPM_PSWC0ST, 0);
	OUTREG32(CPM_PSWC1ST, 10);
	OUTREG32(CPM_PSWC2ST, 15);
	OUTREG32(CPM_PSWC3ST, 0);
#endif
	cpccr = INREG32(CPM_CPCCR) & (CPCCR_CDIV_MASK | 
		CPCCR_H0DIV_MASK | CPCCR_PDIV_MASK | CPCCR_C1DIV_MASK | 
		CPCCR_H2DIV_MASK | CPCCR_H1DIV_MASK);
	c1div = (cpccr & CPCCR_C1DIV_MASK) >> CPCCR_C1DIV_LSB;
	h1div = (c1div << 1) << CPCCR_H1DIV_LSB;
	cdiv = c1div << CPCCR_CDIV_LSB;
	div = (cpccr & (~(CPCCR_CDIV_MASK | CPCCR_H1DIV_MASK))) | cdiv | h1div;
	div = jz_set_div(div);
#if defined(CONFIG_PM_POWERDOWN_P0)
	/* power down the p0 */
	SETREG32(CPM_OPCR, OPCR_PD);

	/* Clear previous reset status */
	CLRREG32(CPM_RSR, RSR_PR | RSR_WR | RSR_P0R);

	/* Set resume return address */
	OUTREG32(CPM_CPSPPR, 0x00005a5a);
	udelay(1);
	OUTREG32(CPM_CPSPR, virt_to_phys(jz_cpu_resume));
	OUTREG32(CPM_CPSPPR, 0x0000a5a5);

	/* *** go zzz *** */
	jz_cpu_sleep();

	/* Clear previous reset status */
	CLRREG32(CPM_RSR, RSR_PR | RSR_WR | RSR_P0R);
#else
	cache_prefetch(jz_pm_do_sleep_L1);
jz_pm_do_sleep_L1:
#define DDRC_PMEMCTRL3	(0xB302005C)
	__asm__ volatile (".set\tmips32\n\t"
			"sync\n\t"
			"sync\n\t"
			".set\tmips32");
	*((volatile unsigned int *)DDRC_PMEMCTRL3) |= 0xff000000;
	__asm__ volatile (".set\tmips32\n\t"
			"wait\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			".set\tmips32");
	/* End of the prefetching codes */
	*((volatile unsigned int *)DDRC_PMEMCTRL3) &= ~0xff000000;
#endif
	/* restore pll div */
	jz_set_div(div);

	/*if power down p0 ,return from sleep.S*/

	/* Restore to IDLE mode */
	CMSREG32(CPM_LCR, LCR_LPM_IDLE, LCR_LPM_MASK);

	/* Restore nand flash control register, it must be restored,
	   because it will be clear to 0 in bootrom. */
	REG_NEMC_NFCSR = nfcsr;

	/* Restore sadc */
	OUTREG8(SADC_ADENA, sadc);

#if defined(CONFIG_JZ4770_BOOT_FROM_MSC0)
	/* Restore msc0 clock */
	OUTREG32(CPM_MSC0CDR, msc0cdr);
#endif

	/* Resume on-board modules */
	jzsoc_do_resume(sleep_gpio_save);

	/* Restore Oscillator and Power Control Register */
	OUTREG32(CPM_OPCR, opcr);

#if (defined(CONFIG_BATTERY_JZ) || defined(CONFIG_BATTERY_JZ47XX)) && defined(CONFIG_RTC_DRV_JZ4770)
	if((INREG32(RTC_RTCCR) & RTCCR_AF) && alarm_state) {
		rtc_clr_reg(RTC_RTCCR,RTCCR_AF);
		rtc_write_reg(RTC_RTCSAR,usr_alarm_data);

		if(g_jz_battery_min_voltage > pm_read_bat())
			pm_power_off();
		else
			goto __jz_pm_do_sleep_start;
	}
#endif
	jz4770_irq_resume();

	return 0;
}

#define K0BASE  KSEG0
void jz_flush_cache_all(void)
{
	unsigned long addr;

	/* Clear CP0 TagLo */
	asm volatile ("mtc0 $0, $28\n\t"::);

	for (addr = K0BASE; addr < (K0BASE + 0x4000); addr += 32) {
		asm volatile (
				".set mips32\n\t"
				" cache %0, 0(%1)\n\t"
				".set mips32\n\t"
				:
				: "I" (Index_Writeback_Inv_D), "r"(addr));

		asm volatile (
				".set mips32\n\t"
				" cache %0, 0(%1)\n\t"
				".set mips32\n\t"
				:
				: "I" (Index_Store_Tag_I), "r"(addr));
	}

	asm volatile ("sync\n\t"::);

	/* invalidate BTB */
	asm volatile (
			".set mips32\n\t"
			" mfc0 %0, $16, 7\n\t"
			" nop\n\t"
			" ori $0, 2\n\t"
			" mtc0 %0, $16, 7\n\t"
			" nop\n\t"
			".set mips32\n\t"
			:
			: "r"(addr));
}

/*
 * valid states, only support mem(sleep) and disk(hibernate)
 */
static int jz4770_pm_valid(suspend_state_t state)
{
	return state == PM_SUSPEND_MEM;
}

/*
 * Jz CPU enter save power mode
 */
static int jz4770_pm_enter(suspend_state_t state)
{
	jz_pm_do_sleep();
	return 0;
}

static struct platform_suspend_ops jz4770_pm_ops = {
	.valid		= jz4770_pm_valid,
	.enter		= jz4770_pm_enter,
};

/*
 * Initialize power interface
 */
int __init jz_pm_init(void)
{
	printk("Power Management for JZ\n");

	suspend_set_ops(&jz4770_pm_ops);
	return 0;
}

int __init gpio_sleep_state_check(void)
{
	unsigned int i,state,group,index;
	unsigned int panic_flags[6] = {0,};

	for(i = 0; i < GPIO_PORT_NUM; i++)
		gpio_intput_pull[i] = 0xffffffff;

	for(i = 0; gpio_sleep_state_table[i][1] != GSS_TABLET_END;i++) {
		group = gpio_sleep_state_table[i][0] / 32;
		index = gpio_sleep_state_table[i][0] % 32;
		state = gpio_sleep_state_table[i][1];

		gpio_intput_pull[group] = gpio_intput_pull[group] & ~(1 << index);
		if(panic_flags[group] & (1 << index)) {
			printk("\nwarning : (%d line) same gpio already set before this line!\n",i);
			printk("\nwarning : (%d line) same gpio already set before this line!\n",i);
			printk("\nwarning : (%d line) same gpio already set before this line!\n",i);
			printk("\nwarning : (%d line) same gpio already set before this line!\n",i);
			printk("\nwarning : (%d line) same gpio already set before this line!\n",i);
			printk("\nwarning : (%d line) same gpio already set before this line!\n",i);
			printk("\nwarning : (%d line) same gpio already set before this line!\n",i);
			panic("gpio_sleep_state_table has iterant gpio set , system halt\n");
			while(1);
		} else {
			panic_flags[group] |= 1 << index;
		}

		switch(state) {
			case GSS_OUTPUT_HIGH:gpio_output_high[group] |= 1 << index;break;
			case GSS_OUTPUT_LOW:gpio_output_low[group] |= 1 << index;break;
			case GSS_INPUT_PULL:gpio_intput_pull[group] |= 1 << index;break;
			case GSS_INPUT_NOPULL:gpio_intput_nopull[group] |= 1 << index;break;
		}
	}

	for(i = 0; i < GPIO_PORT_NUM; i++) {
		printk("%8x\t\t", gpio_output_low[i]);
		printk("%8x\t\t", gpio_output_high[i]);
		printk("%8x\t\t", gpio_intput_pull[i]);
		printk("%8x\n", gpio_intput_nopull[i]);
	}

	return 0;
}

