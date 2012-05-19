/*
 * linux/drivers/misc/tcsm.c
 *
 * Virtual device driver with tricky appoach to manage TCSM 
 *
 * Copyright (C) 2006  Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/spinlock.h>
#include <linux/wakelock.h>

#include <asm/pgtable.h>
#include <asm/mipsregs.h>
#include <asm/mipsmtregs.h>

#include <asm/irq.h>
#include <asm/thread_info.h>
#include <asm/uaccess.h>
#include <asm/jzsoc.h>
#include <linux/syscalls.h>


#include "jz_tcsm.h"

MODULE_AUTHOR("Jianli Wei<jlwei@ingenic.cn>");
MODULE_DESCRIPTION("Virtual Driver of TCSM");
MODULE_LICENSE("GPL");

/*
 * fops routines
 */

static int tcsm_open(struct inode *inode, struct file *filp);
static int tcsm_release(struct inode *inode, struct file *filp);
static ssize_t tcsm_read(struct file *filp, char *buf, size_t size, loff_t *l);
static ssize_t tcsm_write(struct file *filp, const char *buf, size_t size, loff_t *l);
static long tcsm_ioctl (struct file *filp, unsigned int cmd, unsigned long arg);
static int tcsm_mmap(struct file *file, struct vm_area_struct *vma);
static struct file_operations tcsm_fops = 
{
open:		tcsm_open,
		release:	tcsm_release,
		read:		tcsm_read,
		write:		tcsm_write,
		unlocked_ioctl:		tcsm_ioctl,
		mmap:           tcsm_mmap,
};

static spinlock_t lock;
#if defined(CONFIG_SOC_JZ4760B)
#else
static void *save_cpu_wait = NULL;
#endif
static struct wake_lock tcsm_wake_lock;

static struct tcsm_mmap param;
static struct completion tcsm_comp;

static void init_vpu_irq(void)
{
	__asm__ __volatile__ (
			"mfc0  $2, $12,  0   \n\t"
			"ori   $2, $2, 0x800 \n\t"
			"mtc0  $2, $12,  0  \n\t"
			"nop                  \n\t");
}

static struct tcsm_sem tcsm_sem;

static void tcsm_sem_init(struct tcsm_sem *tcsm_sem)
{
	sema_init(&(tcsm_sem->sem),1);
	tcsm_sem->tcsm_file_mode_pre = R_W;
}

static inline void update_file_state(struct tcsm_sem *tcsm_sem, struct file *filp)
{
	tcsm_sem->tcsm_file_mode_pre = tcsm_sem->tcsm_file_mode_now;

	switch(filp->f_mode & 0b11) {
	case FMODE_READ:
		tcsm_sem->tcsm_file_mode_now = R_ONLY;
		break;
	case (FMODE_READ | FMODE_WRITE):
		tcsm_sem->tcsm_file_mode_now = R_W;
		break;
	case FMODE_WRITE:
		tcsm_sem->tcsm_file_mode_now = W_ONLY;
		break;
	default:
		tcsm_sem->tcsm_file_mode_now = UNOPENED;
		break;
	}
}

static inline enum tcsm_file_cmd tcsm_sem_get_cmd(struct tcsm_sem tcsm_sem)
{
	enum tcsm_file_cmd file_cmd = 0;

	switch((enum tcsm_file_mode)tcsm_sem.tcsm_file_mode_now) {
	case R_ONLY:
	case R_W:
		file_cmd = BLOCK;
		break;

	case W_ONLY:
		switch((enum tcsm_file_mode)tcsm_sem.tcsm_file_mode_pre) {
		case R_W:
		case W_ONLY:
			file_cmd = RETURN_NOW;
			break;
		case R_ONLY:
			file_cmd = BLOCK;
			break;			
		default:
			file_cmd = RETURN_NOW;
			break;
		}
		break;
	default:
		return RETURN_NOW;
	}
	return file_cmd;
}

static int tcsm_open(struct inode *inode, struct file *filp)
{
	struct pt_regs *info = task_pt_regs(current);
	unsigned int dat;

	if (tcsm_sem.owner_pid == current->pid) {
		printk("In %s:%s-->pid[%d] can't open tcsm twice!\n", __FILE__, __func__, current->pid);
		return -EBUSY;
	}

	update_file_state(&tcsm_sem, filp);

	if (down_trylock(&tcsm_sem.sem)) {
		switch (tcsm_sem_get_cmd(tcsm_sem)) {
		case BLOCK:
			printk("In %s:%s-->block\n", __FILE__, __func__);
			if (down_interruptible(&tcsm_sem.sem) != 0) {
				return -EBUSY;
			}
			break;
		case RETURN_NOW:
			printk("In %s:%s-->return now\n", __FILE__, __func__);
			return -EBUSY;
		default:
			printk("In %s:%s-->return now\n", __FILE__, __func__);
			printk("Error tcsm file state!\n");
			return -EBUSY;
		}
	}

	tcsm_sem.owner_pid = current->pid;
	spin_lock_irq(&lock);

#if defined(CONFIG_SOC_JZ4760B) || defined(CONFIG_SOC_JZ4770)
	if(INREG32(CPM_OPCR) & BIT31) {
		spin_unlock_irq(&lock);
		up(&tcsm_sem.sem);
		return -EBUSY;
	}
	SETREG32(CPM_OPCR, BIT31);
#else
	if(save_cpu_wait) {
		spin_unlock_irq(&lock);
		up(&tcsm_sem.sem);
		return -EBUSY;
	}
	save_cpu_wait = (void *)cpu_wait;
	cpu_wait = NULL;
#endif

	dat = INREG32(CPM_CLKGR1);

#if defined(CONFIG_SOC_JZ4770)
	dat &= ~(CLKGR1_AUX | CLKGR1_VPU | CLKGR1_CABAC | CLKGR1_SRAM | CLKGR1_DCT | CLKGR1_DBLK | CLKGR1_MC | CLKGR1_ME);
#else
	dat &= ~(CLKGR1_AHB1 | CLKGR1_CABAC | CLKGR1_SRAM | CLKGR1_DCT | 
			CLKGR1_DBLK | CLKGR1_MC | CLKGR1_ME | CLKGR1_AUX);    //ME to reset
#endif

	OUTREG32(CPM_CLKGR1,dat);
	cpm_power_ctrl(CPM_POWER_AHB1,CPM_POWER_ON);
	SETREG32(CPM_CLKGR1,CLKGR1_ME); //no use me

#if !defined(CONFIG_SOC_JZ4770)
	info->cp0_status &= ~0x10;
	info->cp0_status |= 0x08000000; // a tricky
#else
	__asm__ __volatile__ (
			"mfc0  $2, $16,  7   \n\t"
			"ori   $2, $2, 0x340 \n\t"
			"andi  $2, $2, 0x3ff \n\t"
			"mtc0  $2, $16,  7  \n\t"
			"nop                  \n\t");
	enable_irq(IRQ_VPU);

#endif	
	spin_unlock_irq(&lock);
	//memcpy speed
	printk("cp0 status=0x%08x\n", (unsigned int)info->cp0_status);
	wake_lock(&tcsm_wake_lock);
	return 0;
}

static int tcsm_release(struct inode *inode, struct file *filp)
{
	struct pt_regs *info = task_pt_regs(current);
	unsigned int dat;
	printk("close tcsm\n");
	
#if 1
	/*power down ahb1*/
	cpm_power_ctrl(CPM_POWER_AHB1,CPM_POWER_OFF);
#endif
	dat = INREG32(CPM_CLKGR1);

#if defined(CONFIG_SOC_JZ4770)
	disable_irq_nosync(IRQ_VPU);
	dat |= (CLKGR1_AUX | CLKGR1_VPU | CLKGR1_CABAC | CLKGR1_SRAM | CLKGR1_DCT | CLKGR1_DBLK | CLKGR1_MC | CLKGR1_ME);
#else
	dat |= (CLKGR1_AHB1 | CLKGR1_CABAC | CLKGR1_SRAM | CLKGR1_DCT | 
			CLKGR1_DBLK | CLKGR1_MC);
#endif
	printk("dat = 0x%08x\n",dat);

	OUTREG32(CPM_CLKGR1,dat);

	spin_lock_irq(&lock);
#if !defined(CONFIG_SOC_JZ4770)
	info->cp0_status |= 0x10;
	info->cp0_status &= ~0x08000000; // a tricky
#else
	__asm__ __volatile__ (
			"mfc0  $2, $16,  7   \n\t"
			"andi  $2, $2, 0xbf \n\t"
			"mtc0  $2, $16,  7  \n\t"
			"nop                  \n\t");
#endif

#if defined(CONFIG_SOC_JZ4760B) || defined(CONFIG_SOC_JZ4770)     
	CLRREG32(CPM_OPCR, BIT31);
#else
	cpu_wait = save_cpu_wait;
	save_cpu_wait = NULL;
#endif
	spin_unlock_irq(&lock);
	wake_unlock(&tcsm_wake_lock);
	up(&tcsm_sem.sem);
	tcsm_sem.owner_pid = 0;
	return 0;	
}

static ssize_t tcsm_read(struct file *filp, char *buf, size_t size, loff_t *l)
{
	printk("tcsm: read is not implemented\n");
	return -1;
}

static ssize_t tcsm_write(struct file *filp, const char *buf, size_t size, loff_t *l)
{
	printk("tcsm: write is not implemented\n");
	return -1;
}

static long tcsm_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
		case TCSM_TOCTL_WAIT_COMPLETE:
			return wait_for_completion_interruptible_timeout(&tcsm_comp,msecs_to_jiffies(arg));

		case TCSM_TOCTL_SET_MMAP:
			if (copy_from_user(&param, (void *)arg, sizeof(param)))
				return -EFAULT;
			return 0;
	}
	printk(KERN_ERR "%s:cmd(0x%x) error !!!",__func__,cmd);
	return -1;
}

static int tcsm_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long off, start;
	u32 len;

	off = vma->vm_pgoff << PAGE_SHIFT;
	start = param.start;
	len = PAGE_ALIGN(start & ~PAGE_MASK) + param.len;
	start &= PAGE_MASK;
	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;
	off += start;
	vma->vm_pgoff = off >> PAGE_SHIFT;

	/* This is an IO map - tell maydump to skip this VMA */
	vma->vm_flags |= VM_IO;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);	// Uncacheable
#if  defined(CONFIG_MIPS32)
	pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
	pgprot_val(vma->vm_page_prot) |= _CACHE_UNCACHED;		/* Uncacheable */
#endif
	if (io_remap_pfn_range(vma,vma->vm_start, off >> PAGE_SHIFT,vma->vm_end - vma->vm_start,vma->vm_page_prot))
		return -EAGAIN;
	return 0; 
}

static struct miscdevice tcsm_dev = {
	TCSM_MINOR,
	"tcsm",
	&tcsm_fops
};


/*
 * Module init and exit
 */

#if defined(CONFIG_SOC_JZ4770)
static irqreturn_t vpu_interrupt(int irq, void *dev)
{
	CLRREG32(AUX_MIRQP, 0x1);
	complete(&tcsm_comp);
	return IRQ_HANDLED; 
}
#endif

static int __init tcsm_init(void)
{
	int ret;

	ret = misc_register(&tcsm_dev);
	if (ret < 0) {
		return ret;
	}
	wake_lock_init(&tcsm_wake_lock, WAKE_LOCK_SUSPEND, "tcsm");

	tcsm_sem_init(&tcsm_sem);

#if defined(CONFIG_SOC_JZ4770)
	init_completion(&tcsm_comp);
    request_irq(IRQ_VPU,vpu_interrupt,IRQF_DISABLED,"vpu",NULL);
    disable_irq_nosync(IRQ_VPU);
#endif
	printk("Virtual Driver of JZ TCSM registered\n");
	return 0;
}

static void __exit tcsm_exit(void)
{
	misc_deregister(&tcsm_dev);
	wake_lock_destroy(&tcsm_wake_lock);
#if defined(CONFIG_SOC_JZ4770)
    	free_irq(IRQ_VPU,NULL);
#endif
}

module_init(tcsm_init);
module_exit(tcsm_exit);
