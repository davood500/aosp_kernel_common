/*
 * linux/arch/mips/jz4770/cpufreq.c
 *
 * cpufreq driver for JZ4770 
 *
 * Copyright (c) 2006-2008  Ingenic Semiconductor Inc.
 * Author: <lhhuang@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <asm/cacheops.h>
#include <linux/cpufreq.h>
#include <asm/jzsoc.h>
#include <asm/processor.h>

#define dprintk(msg...) cpufreq_debug_printk(CPUFREQ_DEBUG_DRIVER, \
		"cpufreq-jz4770", msg)

#define FREQSCALING 0

static struct cpufreq_frequency_table freqtable[] = {
	{0,0},{1,0},{2,CPUFREQ_TABLE_END},
};

struct jz4770_freq_percpu_info {
	struct cpufreq_frequency_table *table;
	int index;
};

static struct jz4770_freq_percpu_info jz4770_freq = {freqtable,0};

static void jz4770_set_cpu_divider_index(unsigned int cpu, unsigned int index)
{
	unsigned long cpuflags;
	unsigned int cpccr;
	struct cpufreq_freqs freqs;
	int old_index = jz4770_freq.index;

	if(old_index != index)
	{
		freqs.cpu = cpu;
		freqs.new = jz4770_freq.table[index].frequency;
		freqs.old = jz4770_freq.table[old_index].frequency;
#if FREQSCALING

		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

		local_irq_save(cpuflags);
		cpccr = INREG32(CPM_CPCCR);
		cpccr = (cpccr & ~(CPCCR_CDIV_MASK)) | (index & CPCCR_CDIV_MASK) | CPCCR_CE;
		cache_prefetch(jz4770_set_cpu_divider_index_L1);
jz4770_set_cpu_divider_index_L1:
		SETREG32(CPM_CPPSR,0x1);
		OUTREG32(CPM_CPCCR,cpccr);
		while(!(INREG32(CPM_CPPCR0) & CPPCR0_PLLS));
		local_irq_restore(cpuflags);
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
#endif	
		jz4770_freq.index = index;
	}
}

static int jz4770_freq_target(struct cpufreq_policy *policy,
		unsigned int target_freq,
		unsigned int relation)
{

	unsigned int index;

	if (cpufreq_frequency_table_target(policy,jz4770_freq.table,target_freq, relation, &index))
		return -EINVAL;
	jz4770_set_cpu_divider_index(policy->cpu, index);

	return 0;
}

static int jz4770_freq_verify(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy,jz4770_freq.table);
}

static unsigned int jz4770_freq_get(unsigned int cpu)
{
	int old_index = jz4770_freq.index;
#if FREQSCALING
	return cpm_get_clock(CGU_CCLK) / 1000;
#else
	return jz4770_freq.table[old_index].frequency;
#endif
}

static int __init jz4770_freq_init(struct cpufreq_policy *policy)
{
	if (policy->cpu != 0)
		return -EINVAL;

	jz4770_freq.table[0].index = 0;
	jz4770_freq.table[0].frequency = cpm_get_clock(CGU_CCLK) / 1000;
	jz4770_freq.table[1].index = 1;
	jz4770_freq.table[1].frequency = cpm_get_clock(CGU_CCLK) / 1000 / 2;
	jz4770_freq.index = 0;

	policy->cur = jz4770_freq.table[0].frequency;
	policy->governor = CPUFREQ_DEFAULT_GOVERNOR;
	policy->cpuinfo.transition_latency = 1000; /* in 10^(-9) s = nanoseconds */
	policy->cpuinfo.max_freq = jz4770_freq.table[0].frequency;
	policy->cpuinfo.min_freq = jz4770_freq.table[1].frequency;

#ifdef CONFIG_CPU_FREQ_STAT_DETAILS
	cpufreq_frequency_table_get_attr(jz4770_freq.table, policy->cpu); /* for showing /sys/devices/system/cpu/cpuX/cpufreq/stats/ */
#endif
	return  cpufreq_frequency_table_cpuinfo(policy, jz4770_freq.table);
}

static struct cpufreq_driver cpufreq_jz4770_driver = {
	.init		= jz4770_freq_init,
	.verify		= jz4770_freq_verify,
	.target		= jz4770_freq_target,
	.get		= jz4770_freq_get,
	.name		= "jz4770",
};
static int __init jz4770_cpufreq_init(void)
{
	return cpufreq_register_driver(&cpufreq_jz4770_driver);
}

static void __exit jz4770_cpufreq_exit(void)
{
	cpufreq_unregister_driver(&cpufreq_jz4770_driver);
}

module_init(jz4770_cpufreq_init);
module_exit(jz4770_cpufreq_exit);

MODULE_AUTHOR("Regen <lhhuang@ingenic.cn>");
MODULE_DESCRIPTION("cpufreq driver for Jz4770");
MODULE_LICENSE("GPL");
