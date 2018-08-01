Notes on using FreeRTOS
=======================

A. Reentrancy issues in newlib (the GNU C library)

Newlib-nano has at least the following reentrancy issues:

1. malloc and free are by default not thread safe, however they call _malloc_lock and _malloc_unlock to allow the user to make them thread safe. They are sometimes called recursively. Therefore we have them take and release a recursive mutex. The mutex must not be taken/released if malloc is called when the scheduler is suspended, because then the call to take the mutex will cause an assertion failure.

2. strtod uses bigint structs allocated from the heap with pointers in the _reent struct addressed by _impure_ptr. It's possible to give each task its own _reent struct by compiling FreeRTOS with option configUSE_NEWLIB_REENTRANT. However, the _reent struct is huge because it contains lots of stuff we don't need, so it makes every task a lot bigger and wastes many kb of RAM. So instead we provide our own implementation of strtod and friends that doesn't use _reent or heap memory. This also saves around 3Kb of RAM.

3. printf and friends (in particular we use vsnprintf) use a buffer in the _reent struct if we try to print floating point formats. It looks like printing integer formats doesn't do that. So we use our own SafeVsnprintf and SafeSnprintf functions, which are based on the FreeRTOS version but with limited support for floating point formats added. This also avoids some heap memory allocations that vsnprintf was doing.

4. A few other functions use the _reent struct, for example sscanf. So we no longer use sscanf and we must avoid these functions.

B. Critical sections

There are at least the following ways of implementing critical sections to protect shared data:

1. Mutex. Heavy, but doesn't interrupt task scheduling. Preferably use class MutexLocker.

2. Suspend scheduler. Allows interrupts to continue, but prevents task scheduling. Preferably use class TaskCriticalSectionLocker. You can't make a blocking call (e.g. acquire a mutex) while the scheduler is suspended.

3. taskENTER_CRITICAL and taskEXIT_CRITICAL. These disable the lower priority interrupts (the ones that can unblock tasks) but let the higher priority ones happen. Preferably use class InterruptCriticalSectionLocker.

4. cpu_irq_save and cpu_irq_restore. Disables all interrupts.

Use #4 if the data is accessed/mutated by high priority ISRs
Use #3 if the data is accessed/mutated by low priority ISRs
Use #2 if the data is not accessed by ISRs but the critical section lasts only a short time
Use #1 if the data is not accessed by ISRs and the critical section lasts a long time

DC 2018-07-30
