/*
 * Startup.h
 *
 *  Created on: 26 Mar 2018
 *      Author: David
 */

#ifndef SRC_TASKS_H_
#define SRC_TASKS_H_

#include <RepRapFirmware.h>
#include <RTOSIface/RTOSIface.h>

namespace Tasks
{
	void Diagnostics(MessageType mtype) noexcept;
	TaskHandle GetMainTask() noexcept;
	void TerminateMainTask() noexcept;
	ptrdiff_t GetNeverUsedRam() noexcept;
	void *AllocPermanent(size_t sz, std::align_val_t align = (std::align_val_t)__STDCPP_DEFAULT_NEW_ALIGNMENT__) noexcept;
	const char* GetHeapTop() noexcept;
	Mutex *GetI2CMutex() noexcept;
	Mutex *GetFilamentsMutex() noexcept;
	void *GetNVMBuffer(const uint32_t *stk) noexcept;
}

#if SUPPORT_CAN_EXPANSION

// Functions called by CanMessageBuffer in CANlib
void *MessageBufferAlloc(size_t sz, std::align_val_t align) noexcept;
void MessageBufferDelete(void *ptr, std::align_val_t align) noexcept;

#endif

#endif /* SRC_TASKS_H_ */
