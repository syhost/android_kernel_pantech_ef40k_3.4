#ifndef PANTECH_DEBUG_H
#define PANTECH_DEBUG_H

#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/mm_types.h>

extern int pantech_debug_enable;
extern int pantech_debug_init(void);

#define PANTECH_MAX_STACKFRAME_DEPTH 8
#define PANTECH_DBG_LOG_BUF_SIZE 0x100000 

#ifdef CONFIG_PANTECH_DEBUG_SCHED_LOG 
#define SCHED_LOG_MAX 1024*2
struct sched_log {
	unsigned long long time;
	char comm[TASK_COMM_LEN];
	pid_t pid;
};
#endif
#ifdef CONFIG_PANTECH_DEBUG_IRQ_LOG 
struct irq_log {
	unsigned long long time;
	unsigned long long end_time;
	unsigned long long elapsed_time;
	int irq;
	void *fn;
	int en;
	int preempt_count;
	void *context;
};
#endif
#ifdef CONFIG_PANTECH_DEBUG_DCVS_LOG  
#define DCVS_LOG_MAX 512
struct dcvs_debug {
	unsigned long long time;
	int cpu_no;
	unsigned int prev_freq;
	unsigned int new_freq;
};
#endif
#ifdef CONFIG_PANTECH_DEBUG_RPM_LOG  
#define RPM_LOG_MAX 512
struct rpm_debug {
	unsigned long long time;
	unsigned int set;
	unsigned int id;
	unsigned int value;
};
#endif

#define PAGEALLOC_LOG_MAX 1024*2
struct pagealloc_debug {
	unsigned long long time;
	unsigned int was_free;
	unsigned int caller[PANTECH_MAX_STACKFRAME_DEPTH];
	unsigned int alloc_order;
	unsigned int free_order;
	unsigned int zone;
	unsigned int high_free;
	unsigned int low_free;
	struct page *page;
};


#ifdef CONFIG_PANTECH_DEBUG_SCHED_LOG  
extern void pantech_debug_task_sched_log_short_msg(char *msg);
extern void pantech_debug_task_sched_log(int cpu, struct task_struct *task);
extern void pantech_debug_sched_log_init(void);
#define pantechdbg_sched_msg(fmt, ...) \
	do { \
		char ___buf[16]; \
		snprintf(___buf, sizeof(___buf), fmt, ##__VA_ARGS__); \
		pantech_debug_task_sched_log_short_msg(___buf); \
	} while (0)
#else
static inline void pantech_debug_task_sched_log(int cpu, struct task_struct *task)
{
}
static inline void pantech_debug_sched_log_init(void)
{
}
#define pantechdbg_sched_msg(fmt, ...)
#endif

#ifdef CONFIG_PANTECH_DEBUG_SCHED_LOG  
extern void pantech_debug_irq_sched_log(unsigned int irq, void *fn, int en, unsigned long long start_time);
#else
static inline void pantech_debug_irq_sched_log(unsigned int irq, void *fn, int en)
{
}
#endif

#ifdef CONFIG_PANTECH_DEBUG_DCVS_LOG  
extern void pantech_debug_dcvs_log(int cpu_no, unsigned int prev_freq,
			unsigned int new_freq);
#else
static inline void pantech_debug_dcvs_log(int cpu_no, unsigned int prev_freq,
					unsigned int new_freq)
{
}
#endif

#ifdef CONFIG_PANTECH_DEBUG_RPM_LOG 
extern void pantech_debug_rpm_log(unsigned int set, unsigned int id, unsigned int value);
#else
static inline void pantech_debug_rpm_log(unsigned int set, unsigned int id, unsigned int value)
{
}
#endif

extern void pantech_debug_pagealloc_log(struct page *p, unsigned int order, unsigned int zone, 
	unsigned int high_free, unsigned int low_free, unsigned int was_free);

#endif

