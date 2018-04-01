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
#include "RTOSIface.h"

namespace Tasks
{
#ifdef RTOS
	void GetHandlerStackUsage(uint32_t* maxStack, uint32_t* neverUsed);
	void TaskDiagnostics(MessageType mtype, const Task& ct);
	void CurrentTaskDiagnostics(MessageType mtype);
#else
	void GetStackUsage(uint32_t* currentStack, uint32_t* maxStack, uint32_t* neverUsed);
#endif
	const Mutex *GetSpiMutex();
}

#endif /* SRC_TASKS_H_ */
