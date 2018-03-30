/*
 * RTOS.cpp
 *
 *  Created on: 30 Mar 2018
 *      Author: David
 */

#include "RTOSIface.h"

#ifdef RTOS

# include "FreeRTOS.h"
# include "task.h"
# include "semphr.h"

static_assert(RTOSIface::TimeoutUnlimited == portMAX_DELAY, "Bad value for TimeoutUnlimited");

#endif

Locker::Locker(MutexHandle hnd)
{
	handle = hnd;
	acquired =
#ifdef RTOS
				hnd == nullptr || xSemaphoreTakeRecursive(hnd, portMAX_DELAY);
#else
				true;
#endif
}

Locker::Locker(MutexHandle hnd, uint32_t timeout)
{
	handle = hnd;
	acquired =
#ifdef RTOS
				hnd == nullptr || xSemaphoreTakeRecursive(hnd, timeout);
#else
				true;
#endif
}

Locker::~Locker()
{
#ifdef RTOS
	if (acquired && handle != nullptr)
	{
		xSemaphoreGiveRecursive(handle);
	}
#endif
}

namespace RTOSIface
{

#ifdef RTOS

	TaskHandle CreateTask(TaskFunction_t pxTaskCode, const char * pcName, uint32_t ulStackDepth, void *pvParameters, unsigned int uxPriority,
							uint32_t * const puxStackBuffer, TaskStorage& taskBuffer)
	{
		return static_cast<TaskHandle>(xTaskCreateStatic(pxTaskCode, pcName, ulStackDepth, pvParameters, uxPriority, puxStackBuffer, &taskBuffer));
	}

	void SuspendTask(TaskHandle hnd)
	{
		vTaskSuspend(hnd);
	}

	MutexHandle CreateMutex(MutexStorage& st)
	{
		return static_cast<MutexHandle>(xSemaphoreCreateRecursiveMutexStatic(&st));
	}

	bool TakeMutex(MutexHandle hnd, uint32_t timeout)
	{
		return xSemaphoreTakeRecursive(hnd, timeout) == pdTRUE;
	}

	bool ReleaseMutex(MutexHandle hnd)
	{
		return xSemaphoreGiveRecursive(hnd) == pdTRUE;
	}

	TaskHandle GetMutexHolder(MutexHandle hnd)
	{
		return static_cast<TaskHandle>(xSemaphoreGetMutexHolder(hnd));
	}

	TaskHandle GetCurrentTask()
	{
		return static_cast<TaskHandle>(xTaskGetCurrentTaskHandle());
	}

#else

	MutexHandle CreateMutex(MutexStorage& st)
	{
		return static_cast<MutexHandle>((void *)&st);
	}

	bool TakeMutex(MutexHandle hnd, uint32_t timeout)
	{
		return true;
	}

	bool ReleaseMutex(MutexHandle hnd)
	{
		return true;
	}

	TaskHandle GetMutexHolder(MutexHandle hnd)
	{
		return nullptr;
	}

	TaskHandle GetCurrentTask()
	{
		return nullptr;
	}

#endif

}

// End
