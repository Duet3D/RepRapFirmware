/*
 * RTOS.h
 *
 *  Created on: 30 Mar 2018
 *      Author: David
 */

#ifndef SRC_RTOSIFACE_H_
#define SRC_RTOSIFACE_H_

#include <cstdint>

// Type declarations to hide the type-unsafe definitions in the FreeRTOS headers

class Task_undefined;					// this class is never defined
typedef Task_undefined *TaskHandle;

#ifdef RTOS
# include "FreeRTOS.h"
# include "task.h"
# include "semphr.h"
#else
# include "asf.h"
#endif

class Mutex
{
public:
	Mutex() { handle = nullptr; }

	void Create();
	bool Take(uint32_t timeout = TimeoutUnlimited) const;
	bool Release() const;
	TaskHandle GetHolder() const;

	static constexpr uint32_t TimeoutUnlimited = 0xFFFFFFFF;

	Mutex(const Mutex&) = delete;
	Mutex& operator=(const Mutex&) = delete;

private:

#ifdef RTOS
	SemaphoreHandle_t handle;
	StaticSemaphore_t storage;
#else
	void *handle;
#endif

};

#ifdef RTOS

class TaskBase
{
public:
	TaskBase() { handle = nullptr; }

	TaskHandle GetHandle() const { return static_cast<TaskHandle>(handle); }
	void Suspend() const { vTaskSuspend(handle); }
	const TaskBase *GetNext() const { return next; }

	TaskBase(const TaskBase&) = delete;				// it's not save to copy these
	TaskBase& operator=(const TaskBase&) = delete;	// it's not safe to assign these
	// Ideally we would declare the destructor as deleted too, because it's unsafe to delete these because they are linked together via the 'next' field.
	// But that prevents us from declaring static instances of tasks.
	// Possible solutions:
	// 1. Just be careful that after we allocate a task using 'new', we never delete it.
	// 2. Write a destructor that removes the task from the linked list.
	// 3. Don't allocate tasks statically, allocate them all using 'new'.
	//~TaskBase() = delete;							// it's not safe to delete these because they are linked together via the 'next' field

	static const TaskBase *GetTaskList() { return taskList; }

	static constexpr int SpinPriority = 1;			// priority for tasks that rarely block
	static constexpr int HeatPriority = 2;

protected:
	TaskHandle_t handle;
	TaskBase *next;
	StaticTask_t storage;

	static TaskBase *taskList;
};

template<unsigned int StackWords> class Task : public TaskBase
{
public:
	// The Create function assumes that only the main task creates other tasks, so we don't need a mutex to protect the task list
	void Create(TaskFunction_t pxTaskCode, const char * pcName, void *pvParameters, unsigned int uxPriority)
	{
		next = taskList;
		taskList = this;
		handle = xTaskCreateStatic(pxTaskCode, pcName, StackWords, pvParameters, uxPriority, stack, &storage);
	}

private:
	uint32_t stack[StackWords];
};

#endif

// Class to lock a mutex and automatically release it when it goes out of scope
// If we pass a null mutex handle to the Locker constructor, it means there is nothing to lock and we pretend the lock has been acquired.
class MutexLocker
{
public:
	MutexLocker(const Mutex *pm, uint32_t timeout = Mutex::TimeoutUnlimited);	// acquire lock
	MutexLocker(const Mutex& pm, uint32_t timeout = Mutex::TimeoutUnlimited);	// acquire lock
	void Release();															// release the lock early (else gets released by destructor)
	~MutexLocker();
	operator bool() const { return acquired; }

	MutexLocker(const MutexLocker&) = delete;
	MutexLocker& operator=(const MutexLocker&) = delete;

private:
	const Mutex *handle;
	bool acquired;
};

// Interface to RTOS or RTOS substitute
namespace RTOSIface
{
	TaskHandle GetCurrentTask();

#ifndef RTOS
	static volatile unsigned int interruptCriticalSectionNesting = 0;
#endif

	// Enter a critical section, where modificatio0n to variables by interrupts (and perhaps also other tasks) must be avoided
	inline void EnterInterruptCriticalSection()
	{
#ifdef RTOS
		taskENTER_CRITICAL();
#else
		cpu_irq_disable();
		++interruptCriticalSectionNesting;
#endif
	}

	// Leave an interrupt-critical section
	inline void LeaveInterruptCriticalSection()
	{
#ifdef RTOS
		taskEXIT_CRITICAL();
#else
		--interruptCriticalSectionNesting;
		if (interruptCriticalSectionNesting == 0)
		{
			cpu_irq_enable();
		}
#endif
	}

	// Enter a task-critical region. Used to protect concurrent access to variable from different tasks, where the variable are not used/modified by interrupts.
	inline void EnterTaskCriticalSection()
	{
#ifdef RTOS
		vTaskSuspendAll();
#else
		// nothing to do here because there is no task preemption
#endif
	}

	// Exit a task-critical region, returning true if a task switch occurred
	inline bool LeaveTaskCriticalSection()
	{
#ifdef RTOS
		return xTaskResumeAll() == pdTRUE;
#else
		// nothing to do here because there is no task preemption
		return false;
#endif
	}

	inline void Yield()
	{
#ifdef RTOS
		taskYIELD();
#endif
	}
}

class InterruptCriticalSectionLocker
{
public:
	InterruptCriticalSectionLocker() { RTOSIface::EnterInterruptCriticalSection(); }
	~InterruptCriticalSectionLocker() { (void)RTOSIface::LeaveInterruptCriticalSection(); }
};

class TaskCriticalSectionLocker
{
public:
	TaskCriticalSectionLocker() { RTOSIface::EnterTaskCriticalSection(); }
	~TaskCriticalSectionLocker() { RTOSIface::LeaveTaskCriticalSection(); }
};

#endif /* SRC_RTOSIFACE_H_ */
