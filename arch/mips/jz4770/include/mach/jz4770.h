/*
 *  arch/mips/mach-jz4760/include/mach/jz4770.h
 *
 *  JZ4770 common definition.
 *
 *  Copyright (C) 2008 Ingenic Semiconductor Inc.
 *
 *  Author: <cwjia@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4770_H__
#define __ASM_JZ4770_H__

#include <mach/regs.h>

#include <mach/chip-misc.h>
#include <mach/chip-gpio.h>
#include <mach/chip-dmac.h>
#include <mach/chip-intc.h>
#include <mach/chip-aic.h>
#include <mach/chip-bch.h>
#include <mach/chip-bdma.h>
#include <mach/chip-cim.h>
#include <mach/chip-cpm.h>
#include <mach/chip-ddrc.h>
#include <mach/chip-emc.h>
#include <mach/chip-i2c.h>
#include <mach/chip-ipu.h>
#include <mach/chip-lcdc.h>
#include <mach/chip-mc.h>
#include <mach/chip-me.h>
#include <mach/chip-msc.h>
#include <mach/chip-nemc.h>
#include <mach/chip-otg.h>
#include <mach/chip-owi.h>
#include <mach/chip-pcm.h>
#include <mach/chip-rtc.h>
#include <mach/chip-sadc.h>
#include <mach/chip-scc.h>
#include <mach/chip-ssi.h>
#include <mach/chip-tcu.h>
#include <mach/chip-tssi.h>
#include <mach/chip-tve.h>
#include <mach/chip-uart.h>
#include <mach/chip-wdt.h>
#include <mach/chip-ost.h>
#include <mach/chip-aosd.h>
#include <mach/chip-lvds.h>
#include <mach/chip-aux.h>
#include <mach/chip-fuse.h>

#include <mach/dma.h>
#include <mach/misc.h>
#include <mach/chip-pin.h>
/*------------------------------------------------------------------
 * Follows are related to platform definitions
 */

#include <mach/serial.h>

#ifdef CONFIG_JZ4770_LINDEN
#include <linden/linden.h>
#endif

#ifdef CONFIG_JZ4770_LINDEN111
#include <linden111/linden111.h>
#endif

#ifdef CONFIG_JZ4770_NPM701
#include <npm701/npm701.h>
#endif

#ifdef CONFIG_JZ4770_NPM702
#include <npm702/npm702.h>
#endif

#ifdef CONFIG_JZ4770_NPM703
#include <npm703/npm703.h>
#endif

#ifdef CONFIG_JZ4770_O1
#include <o1/o1.h>
#endif

#ifdef CONFIG_JZ4770_COBY7022
#include <coby7022/coby7022.h>
#endif

#ifdef CONFIG_JZ4770_AURORA
#include <aurora/aurora.h>
#endif

#ifdef CONFIG_JZ4770_PS47
#include <ps47/ps47.h>
#endif

#ifdef CONFIG_JZ4770_PI3800
#include <pi3800/pi3800.h>
#endif

#ifdef CONFIG_JZ4770_TABLET_7_HD_C
#include <Tablet_7_HD_C/Tablet_7_HD_C.h>
#endif

#ifdef CONFIG_JZ4770_TVB
#include <tvb/tvb.h>
#endif

#ifdef CONFIG_JZ4770_N55
#include <n55/n55.h>
#endif

#ifdef CONFIG_JZ4770_JB01
#include <jb01/jb01.h>
#endif

#define JZ4770_V1 "4770v1"
extern char jz4770_cpu_version[64];
#endif /* __ASM_JZ4770_H__ */

#ifdef CONFIG_JZ4770_PISCES
#include <pisces/pisces.h>
#endif


