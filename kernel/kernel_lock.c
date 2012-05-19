/*
 * lib/kernel_lock.c
 *
 * This is the traditional BKL - big kernel lock. Largely
 * relegated to obsolescence, but used by various less
 * important (or lazy) subsystems.
 */
//#include <linux/smp_lock.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/kallsyms.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/kernel_lock.h>

/*
 * The 'big kernel lock'
 *
 * This spinlock is taken and released recursively by lock_kernel()
 * and unlock_kernel().  It is transparently dropped and reacquired
 * over schedule().  It is used to protect legacy code that hasn't
 * been migrated to a proper locking design yet.
 *
 * Don't use in new code.
 */
static  __cacheline_aligned_in_smp DEFINE_SPINLOCK(kernel_flag);
static  __cacheline_aligned_in_smp DEFINE_SPINLOCK(tmp_lock);


/*
 * Acquire/release the underlying lock from the scheduler.
 *
 * This is called with preemption disabled, and should
 * return an error value if it cannot get the lock and
 * TIF_NEED_RESCHED gets set.
 *
 * If it successfully gets the lock, it should increment
 * the preemption count like any spinlock does.
 *
 * (This works on UP too - _raw_spin_trylock will never
 * return false in that case)
 */
int __lockfunc __reacquire_kernel_lock(void)
{
	while (!_raw_spin_trylock(&kernel_flag)) {
		if (need_resched())
			return -EAGAIN;
		cpu_relax();
	}
	preempt_disable();
	return 0;
}

void __lockfunc __release_kernel_lock(void)
{
	_raw_spin_unlock(&kernel_flag);
	preempt_enable_no_resched();
}

/*
 * These are the BKL spinlocks - we try to be polite about preemption.
 * If SMP is not on (ie UP preemption), this all goes away because the
 * _raw_spin_trylock() will always succeed.
 */
#ifdef CONFIG_PREEMPT
static inline void __lock_kernel(void)
{
	preempt_disable();
	if (unlikely(!_raw_spin_trylock(&kernel_flag))) {
		/*
		 * If preemption was disabled even before this
		 * was called, there's nothing we can be polite
		 * about - just spin.
		 */
		if (preempt_count() > 1) {
			_raw_spin_lock(&kernel_flag);
			return;
		}

		/*
		 * Otherwise, let's wait for the kernel lock
		 * with preemption enabled..
		 */
		do {
			preempt_enable();
			while (spin_is_locked(&kernel_flag))
				cpu_relax();
			preempt_disable();
		} while (!_raw_spin_trylock(&kernel_flag));
	}
}

#else

/*
 * Non-preemption case - just get the spinlock
 */
static inline void __lock_kernel(void)
{
	_raw_spin_lock(&kernel_flag);
}
#endif

static inline void __unlock_kernel(void)
{
	/*
	 * the BKL is not covered by lockdep, so we open-code the
	 * unlocking sequence (and thus avoid the dep-chain ops):
	 */
	_raw_spin_unlock(&kernel_flag);
	preempt_enable();
}

#if 0
/*
 * Getting the big kernel lock.
 *
 * This cannot happen asynchronously, so we only need to
 * worry about other CPU's.
 */
void __lockfunc lock_kernel(void)
{
	int depth = current->lock_depth+1;
	if (likely(!depth))
		__lock_kernel();
	current->lock_depth = depth;
}

void __lockfunc unlock_kernel(void)
{
	BUG_ON(current->lock_depth < 0);
	if (likely(--current->lock_depth < 0))
		__unlock_kernel();
}

EXPORT_SYMBOL(lock_kernel);
EXPORT_SYMBOL(unlock_kernel);

#include <linux/sched.h>

#define kernel_locked()		(current->lock_depth >= 0)

extern int __lockfunc __reacquire_kernel_lock(void);
extern void __lockfunc __release_kernel_lock(void);

/*
 * Release/re-acquire global kernel lock for the scheduler
 */
#define release_kernel_lock(tsk) do { 		\
	if (unlikely((tsk)->lock_depth >= 0))	\
		__release_kernel_lock();	\
} while (0)

static inline int reacquire_kernel_lock(struct task_struct *task)
{
	if (unlikely(task->lock_depth >= 0))
		return __reacquire_kernel_lock();
	return 0;
}

extern void __lockfunc lock_kernel(void)	__acquires(kernel_lock);
extern void __lockfunc unlock_kernel(void)	__releases(kernel_lock);

#endif				/* if 0 */



/* ********************************************************************* */

/* ********************************************************************* */

struct pid_list {
	struct list_head list;
	int pid;
	int ppid;
	int tgid;
	int lock_depth;
};

static struct pid_list g_pid_head = {.list = LIST_HEAD_INIT(g_pid_head.list)};

//static int g_insert_counter=0;

static struct pid_list * insert_to_list(struct pid_list * head, int pid, int ppid, int tgid)
{
	struct list_head * ll;
	struct list_head * hh;
	struct pid_list * l ;
//	MY_DBG();
	if (head == NULL) {
		return NULL;
	}

	//++g_insert_counter; //printk("g_insert_counter=%d\n", g_insert_counter);

	l = (struct pid_list * )kmalloc(sizeof(struct pid_list), GFP_KERNEL);
	memset((void*)l, 0, sizeof(struct pid_list));
	l->pid = pid;
	l->ppid = ppid;
	l->tgid = tgid;
	l->lock_depth = -1;

	hh = (struct list_head *)head;
	ll = (struct list_head *)l;

	list_add_tail(ll, hh);
//	MY_DBG();
	return l;
}

/* index form head to tail */
static int delete_from_list(struct pid_list * head, int pid, int ppid, int tgid)
{
	struct pid_list * match;
	struct list_head * ll;
	struct list_head * hh;
//	MY_DBG();

	hh = (struct list_head *)head;

	if ( head == NULL ) {
		return 0;
	}
	match = NULL;
	list_for_each(ll, hh) {
		struct pid_list * l;
		l = (struct pid_list *)ll;
		if (l->pid == pid && l->ppid == ppid && l->tgid == tgid) {
			match = (struct pid_list *)l; /* got the last match. */
			//break;
		}
	}

	if ( match == NULL) {
		printk("failed delete node: pid=%d ppid=%d, tgid=%d", match->pid, match->ppid, match->tgid);
		return 0;
	}

	ll = (struct list_head *)match;
	/* delete node */
	//MY_DBG("delete node: pid=%d ppid=%d, tgid=%d", match->pid, match->ppid, match->tgid);
	list_del(ll);
	kfree(match);
//	MY_DBG();
	return 1;
}

#if 0
/* index form head to tail */
static int get_list_count(struct pid_list * head )
{
	struct list_head * ll;
	struct list_head * hh;
	int count;

	hh = (struct list_head *)head;
	if ( head == NULL ) {
		return 0;
	}
	count = 0;

	list_for_each(ll, hh) {
		++count;
	}
//	MY_DBG();
	return count;
}
#endif


/* */
static int dump_list(struct pid_list * head )
{
	struct list_head * ll;
	struct list_head * hh;
	int count;

	hh = (struct list_head *)head;
//	MY_DBG();
	if ( head == NULL) {
		return 0;
	}
	printk("dump list:\n");
	count=0;
	list_for_each(ll, hh) {
		struct pid_list * l;
		l = (struct pid_list *)ll;
		++count;
		//printk("%02d node: %d, %d, %d\n", count, l->pid, l->ppid, l->tgid);
		printk("%02d node: pid=%d, tgid=%d\n", count, l->pid, l->tgid);
	}

//	MY_DBG();
	return count;
}

#if 0
static int get_process_threads_count(struct pid_list * head, int pid, int ppid, int tgid)
{
	struct list_head * ll;
	struct list_head * hh;
	int count;
//	MY_DBG();
	if ( head == NULL ) {
		return 0;
	}

	hh = (struct list_head *)head;
	count = 0;
	list_for_each(ll, hh) {
		struct pid_list * l;
		l = (struct pid_list *)ll;
		/* count thread in the same process(tgid) */
		if ( l->tgid == tgid) {
			++count;
		}
	}

	return count;
}
#endif

static struct pid_list * current_pid_in_pid_list(struct pid_list * head, int pid, int ppid, int tgid)
{
	struct pid_list * pl;
	struct list_head * ll;
	struct list_head * hh;
	int count;
//	MY_DBG();
	if ( head == NULL ) {
		return NULL;
	}

	hh = (struct list_head *)head;
	count = 0;
	pl = NULL;
	list_for_each(ll, hh) {
		struct pid_list * l;
		l = (struct pid_list *)ll;
		/* count thread in the same process(tgid) */
		if ( l->pid == pid) {
			pl = l;
			++count;
		}
	}
	if ( count > 1 ) {
		printk("current_pid_in_pid_list() count=%d\n", count);
		dump_list(head);
	}

//	MY_DBG("count=%d", count);
	return pl;
}




void lock_kernel(void)
{
	struct pid_list * pi;
	int pid, tgid;

	spin_lock(&tmp_lock);
	pid = current->pid;
	tgid = current->tgid;
	pi = current_pid_in_pid_list(&g_pid_head, pid, -1, tgid);
	if ( !pi ) {
		/* create pid_list */
		pi = insert_to_list(&g_pid_head, pid, -1, tgid);
	}
	if ( pi ) {
		int depth = pi->lock_depth+1;
		if (likely(!depth))
			__lock_kernel();
		pi->lock_depth = depth;
	}

	/* if ( ++g_insert_counter%1000 == 0 ) { */
	/* 	dump_list(&g_pid_head); */
	/* } */

	spin_unlock(&tmp_lock);

//	MY_DBG("2 pid=%d, tgid=%d", pid, tgid);
	return;
}

void unlock_kernel(void)
{
	struct pid_list * pi;
	int pid, tgid;

	spin_lock(&tmp_lock);
	pid = current->pid;
	tgid = current->tgid;
	pi = current_pid_in_pid_list(&g_pid_head, pid, -1, tgid);

	if ( pi != NULL)  {
		BUG_ON(pi->lock_depth < 0);
		if (likely(--pi->lock_depth < 0)) {
			__unlock_kernel();
			delete_from_list(&g_pid_head, pid, -1, tgid);
		}
	}
	else {
		printk("unlock_kernel  !pi.\n");
	}
//	MY_DBG("pid=%d, tgid=%d", pid, tgid);
	spin_unlock(&tmp_lock);
	return ;
}

EXPORT_SYMBOL(lock_kernel);
EXPORT_SYMBOL(unlock_kernel);

int try_release_gpu_driver_ioctl_lock(struct task_struct *tsk)
{
	struct pid_list * pi;
	int pid, tgid;

	spin_lock(&tmp_lock);
	pid = tsk->pid;
	tgid = tsk->tgid;

	pi = current_pid_in_pid_list(&g_pid_head, pid, -1, tgid);
	spin_unlock(&tmp_lock);

	if ( pi != NULL)  {
		if (unlikely(pi->lock_depth >= 0))
			__release_kernel_lock();
	}
	else {
//		printk("try_release_gpu_driver_tmp_lock !pi.\n");
	}

	return 0;
}

int try_reaquire_gpu_driver_ioctl_lock(struct task_struct *tsk)
{
	struct pid_list * pi;
	int pid, tgid;
	int ret;
	ret = 0;
	spin_lock(&tmp_lock);
	pid = tsk->pid;
	tgid = tsk->tgid;
//	MY_DBG("try_release_gpu_driver_tmp_lock pid=%d", pid);
	pi = current_pid_in_pid_list(&g_pid_head, pid, -1, tgid);	
	spin_unlock(&tmp_lock);
	if ( pi != NULL)  {
		if (unlikely(pi->lock_depth >= 0))
			ret = __reacquire_kernel_lock();
	}
	else {
//		printk("try_release_gpu_driver_tmp_lock !pi.\n");
	}

	return ret;
}

EXPORT_SYMBOL(try_release_gpu_driver_ioctl_lock);
EXPORT_SYMBOL(try_reaquire_gpu_driver_ioctl_lock);

/* ********************************************************************* */
