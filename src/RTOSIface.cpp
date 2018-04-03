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

static_assert(Mutex::TimeoutUnlimited == portMAX_DELAY, "Bad value for TimeoutUnlimited");

void Mutex::Create()
{
	if (handle == nullptr)
	{
		handle = xSemaphoreCreateRecursiveMutexStatic(&storage);
	}
}

bool Mutex::Take(uint32_t timeout) const
{
	return xSemaphoreTakeRecursive(handle, timeout) == pdTRUE;
}

bool Mutex::Release() const
{
	return xSemaphoreGiveRecursive(handle) == pdTRUE;
}

TaskHandle Mutex::GetHolder() const
{
	return static_cast<TaskHandle>(xSemaphoreGetMutexHolder(handle));
}

TaskBase *TaskBase::taskList = nullptr;

#else

void Mutex::Create()
{
}

bool Mutex::Take(uint32_t timeout) const
{
	return true;
}

bool Mutex::Release() const
{
	return true;
}

TaskHandle Mutex::GetHolder() const
{
	return nullptr;
}

#endif

MutexLocker::MutexLocker(const Mutex *m, uint32_t timeout)
{
	handle = m;
	acquired =
#ifdef RTOS
				m == nullptr || m->Take(timeout);
#else
				true;
#endif
}

MutexLocker::MutexLocker(const Mutex& m, uint32_t timeout)
{
	handle = &m;
	acquired =
#ifdef RTOS
				m.Take(timeout);
#else
				true;
#endif
}

void MutexLocker::Release()
{
#ifdef RTOS
	if (acquired && handle != nullptr)
	{
		handle->Release();
		acquired = false;
	}
#endif
}

MutexLocker::~MutexLocker()
{
	Release();
#ifdef RTOS
	if (acquired && handle != nullptr)
	{
		handle->Release();
	}
#endif
}

namespace RTOSIface
{

#ifdef RTOS

	TaskHandle GetCurrentTask()
	{
		return static_cast<TaskHandle>(xTaskGetCurrentTaskHandle());
	}

#else

	TaskHandle GetCurrentTask()
	{
		return nullptr;
	}

#endif

}

// End
