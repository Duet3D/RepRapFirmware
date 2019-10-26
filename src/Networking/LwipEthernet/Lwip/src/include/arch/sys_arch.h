#ifndef __ARCH_SYS_ARCH_H__
#define __ARCH_SYS_ARCH_H__

#define SYS_MBOX_NULL   NULL
#define SYS_SEM_NULL    NULL

typedef void * sys_prot_t;

typedef void * sys_sem_t;

typedef struct
{
	struct QueueBase *m;
} sys_mbox_t;

typedef void * sys_thread_t;

typedef struct
{
	struct Mutex *m;
} sys_mutex_t;

#endif /* __ARCH_SYS_ARCH_H__ */
