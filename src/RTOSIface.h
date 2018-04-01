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

class Task
{
public:
	Task() { handle = nullptr; }

	TaskHandle GetHandle() const { return static_cast<TaskHandle>(handle); }
	void Create(TaskFunction_t pxTaskCode, const char * pcName, uint32_t ulStackDepth, void *pvParameters, unsigned int uxPriority, uint32_t * const puxStackBuffer);
	void Suspend() { vTaskSuspend(handle); }

	Task(const Task&) = delete;
	Task& operator=(const Task&) = delete;

private:
	TaskHandle_t handle;
	StaticTask_t storage;
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
}

class CriticalSectionLocker
{
public:
	CriticalSectionLocker() { RTOSIface::EnterCriticalSection(); }
	~CriticalSectionLocker() { RTOSIface::LeaveCriticalSection(); }
};

#endif /* SRC_RTOSIFACE_H_ */
