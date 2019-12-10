/*
 * Startup.h
 *
 *  Created on: 26 Mar 2018
 *      Author: David
 */

#ifndef SRC_TASKS_H_
#define SRC_TASKS_H_

#include "RepRapFirmware.h"
#include "MessageType.h"
#include "RTOSIface/RTOSIface.h"

namespace Tasks
{
	void Diagnostics(MessageType mtype) noexcept;
	uint32_t GetNeverUsedRam() noexcept;
	const Mutex *GetSpiMutex() noexcept;
	const Mutex *GetI2CMutex() noexcept;
	const Mutex *GetSysDirMutex() noexcept;
}

#endif /* SRC_TASKS_H_ */
