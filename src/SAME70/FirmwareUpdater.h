/*
 * FirmwareUpdater.h
 *
 *  Created on: 21 May 2016
 *      Author: David
 */

#ifndef SRC_SAME70_FIRMWAREUPDATER_H_
#define SRC_SAME70_FIRMWAREUPDATER_H_

#include <cstdint>
#include "Libraries/General/StringRef.h"

namespace FirmwareUpdater
{
	bool CheckFirmwareUpdatePrerequisites(uint8_t moduleMap, StringRef& reply);
	bool IsReady();
	void UpdateModule(unsigned int module);
}

#endif /* SRC_SAME70_FIRMWAREUPDATER_H_ */
