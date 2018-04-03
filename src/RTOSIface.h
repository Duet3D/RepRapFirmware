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

	TaskBase(const TaskBase&) = delete;
	TaskBase& operator=(const TaskBase&) = delete;

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
	static volatile unsigned int criticalSectionNesting = 0;
#endif

	inline void EnterCriticalSection()
	{
#ifdef RTOS
		taskENTER_CRITICAL();
#else
		cpu_irq_disable();
		++criticalSectionNesting;
#endif
	}

	inline void LeaveCriticalSection()
	{
#ifdef RTOS
		taskEXIT_CRITICAL();
#else
		--criticalSectionNesting;
		if (criticalSectionNesting == 0)
		{
			cpu_irq_enable();
		}
#endif
	}

	inline void Yield()
	{
#ifdef RTOS
		taskYIELD();
#endif
	}
}

class CriticalSectionLocker
{
public:
	CriticalSectionLocker() { RTOSIface::EnterCriticalSection(); }
	~CriticalSectionLocker() { RTOSIface::LeaveCriticalSection(); }
};

#endif /* SRC_RTOSIFACE_H_ */
