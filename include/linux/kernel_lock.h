#ifndef KERNEL_LOCK_H
#define KERNEL_LOCK_H


#ifdef CONFIG_PREEMPT
void lock_kernel(void);
void unlock_kernel(void);

extern int try_release_gpu_driver_ioctl_lock(struct task_struct *tsk);
extern int try_reaquire_gpu_driver_ioctl_lock(struct task_struct *tsk);
#else

#endif	/* CONFIG_PREEMPT */
#endif	/* KERNEL_LOCK_H */
