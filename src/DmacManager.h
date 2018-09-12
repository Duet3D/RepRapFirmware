/*
 * DmacManager.h
 *
 *  Created on: 12 Sep 2018
 *      Author: David
 */

#ifndef SRC_DMACMANAGER_H_
#define SRC_DMACMANAGER_H_

#include "RepRapFirmware.h"

#if SAME70

//#include "WInterrupts.h"
namespace DmacManager
{
	void Init();
	void SetInterruptCallback(const uint8_t channel, StandardCallbackFunction fn, CallbackParameter param);
}

#endif

#endif /* SRC_DMACMANAGER_H_ */
