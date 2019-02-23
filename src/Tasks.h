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

#if USE_CACHE

#include "sam/drivers/cmcc/cmcc.h"

inline void EnableCache()
{
	cmcc_invalidate_all(CMCC);
	cmcc_enable(CMCC);
}

inline void DisableCache()
{
	cmcc_disable(CMCC);
}

#else

inline void EnableCache() {}
inline void DisableCache() {}

#endif


namespace Tasks
{
	void Diagnostics(MessageType mtype);
	uint32_t GetNeverUsedRam();
	const Mutex *GetSpiMutex();
	const Mutex *GetI2CMutex();
	const Mutex *GetSysDirMutex();
}

#endif /* SRC_TASKS_H_ */
