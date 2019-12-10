/*
 * FirmwareUpdater.h
 *
 *  Created on: 21 May 2016
 *      Author: David
 */

#ifndef SRC_NETWORKING_FIRMWAREUPDATER_H_
#define SRC_NETWORKING_FIRMWAREUPDATER_H_

#include "RepRapFirmware.h"

namespace FirmwareUpdater
{
	bool CheckFirmwareUpdatePrerequisites(uint8_t moduleMap, const StringRef& reply) noexcept;
	bool IsReady() noexcept;
	void UpdateModule(unsigned int module) noexcept;
}

#endif /* SRC_NETWORKING_FIRMWAREUPDATER_H_ */
