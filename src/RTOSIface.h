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

class Mutex_undefined;					// this class is never defined
typedef Mutex_undefined *MutexHandle;

#ifdef RTOS
# include "FreeRTOS.h"
# include "task.h"
# include "semphr.h"

typedef StaticSemaphore_t MutexStorage;
typedef StaticTask_t TaskStorage;

#else

typedef int MutexStorage;
typedef int TaskStorage;

#endif

// Class to lock a mutex and automatically release it when it goes out of scope
// If we pass a null mutex handle to the Locker constructor, it means there is nothing to lock and we pretend the lock has been acquired.
class Locker
{
public:
	Locker(MutexHandle hnd);						// acquire lock, no timeout
	Locker(MutexHandle hnd, uint32_t timeout);	// acquire lock with timeout
	~Locker();
	operator bool() const { return acquired; }

private:
	MutexHandle handle;
	bool acquired;
};

namespace RTOSIface
{
	constexpr uint32_t TimeoutUnlimited = 0xFFFFFFFF;

#if RTOS
	TaskHandle CreateTask(TaskFunction_t pxTaskCode, const char * pcName, uint32_t ulStackDepth, void *pvParameters, unsigned int uxPriority,
							uint32_t * const puxStackBuffer, TaskStorage& taskBuffer);
	void SuspendTask(TaskHandle hnd);
#endif

	MutexHandle CreateMutex(MutexStorage& st);
	bool TakeMutex(MutexHandle hnd, uint32_t timeout = TimeoutUnlimited);
	bool ReleaseMutex(MutexHandle hnd);
	TaskHandle GetMutexHolder(MutexHandle hnd);
	TaskHandle GetCurrentTask();
}

#endif /* SRC_RTOSIFACE_H_ */
