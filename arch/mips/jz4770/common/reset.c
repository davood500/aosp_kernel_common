/*
 * linux/arch/mips/jz4760/reset.c
 *
 * JZ4760 reset routines.
 *
 * Copyright (c) 2006-2007  Ingenic Semiconductor Inc.
 * Author: <yliu@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/sched.h>
#include <linux/mm.h>
#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/processor.h>
#include <asm/reboot.h>
#include <asm/system.h>
#include <asm/jzsoc.h>

#define RECOVERY_SIGNATURE	(0x001a1a)
#define UNMSAK_SIGNATURE	(0x7c0000)//do not use these bits

void jz_restart(char *command);
void jz_hibernate_restart(char *command);
void pm_sync_filesystem(void);
void jz_hibernate_restart(char *command)
{
	uint32_t rtc_rtcsr;
	pm_sync_filesystem();
	if ((command != NULL) && !strcmp(command, "recovery")) {
		jz_restart(command);
	} else {
		cpm_set_scrpad(0);
	}

	rtc_rtcsr = rtc_read_reg(RTC_RTCSR);
	rtc_write_reg(RTC_RTCSAR,rtc_rtcsr + 2);
	rtc_set_reg(RTC_RTCCR,0x3<<2);

	/* Mask all interrupts */
	OUTREG32(INTC_ICMSR(0), 0xffffffff);
	OUTREG32(INTC_ICMSR(1), 0x7ff);

      	/* Clear reset status */
	CLRREG32(CPM_RSR, RSR_PR | RSR_WR | RSR_P0R);

	/* 
	 * RTC Wakeup or 1Hz interrupt can be enabled or disabled 
	 * through  RTC driver's ioctl (linux/driver/char/rtc_jz.c).
	 */

	/* Set minimum wakeup_n pin low-level assertion time for wakeup: 100ms */
	rtc_write_reg(RTC_HWFCR, HWFCR_WAIT_TIME(100));

	/* Set reset pin low-level assertion time after wakeup: must  > 60ms */
	rtc_write_reg(RTC_HRCR, HRCR_WAIT_TIME(60));

	/* Scratch pad register to be reserved */
	rtc_write_reg(RTC_HSPR, HSPR_RTCV);

	/* clear wakeup status register */
	rtc_write_reg(RTC_HWRSR, 0x0);

	/* set wake up valid level as low  and disable rtc alarm wake up.*/
	rtc_write_reg(RTC_HWCR,0x9);

	/* Put CPU to hibernate mode */
	rtc_write_reg(RTC_HCR, HCR_PD);

	while (1) {
		printk("We should NOT come here, please check the jz4760rtc.h!!!\n");
	};
}

void jz_restart(char *command)
{
	pm_sync_filesystem();
	printk("Restarting after 4 ms\n");
	if ((command != NULL) && !strcmp(command, "recovery")) {
		while(cpm_get_scrpad() != RECOVERY_SIGNATURE) {
			printk("set RECOVERY_SIGNATURE\n");
			cpm_set_scrpad(RECOVERY_SIGNATURE);
			msleep(20);
		}
	} else {
		cpm_set_scrpad(0);
	}

	REG_WDT_WCSR = WCSR_PRESCALE4 | WCSR_CLKIN_EXT;
	REG_WDT_WCNT = 0;
	REG_WDT_WDR = JZ_EXTAL/1000;   /* reset after 4ms */
	REG_TCU_TSCR = TSCR_WDT; /* enable wdt clock */
	REG_WDT_WCER = WCER_TCEN;  /* wdt start */
	while (1);
}

void jz_halt(void)
{
	printk(KERN_NOTICE "\n** You can safely turn off the power\n");

	while (1)
		__asm__(".set\tmips3\n\t"
	                "wait\n\t"
			".set\tmips0");
}

void jz_power_off(void)
{
	jz_halt();
}
