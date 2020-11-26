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
#if HAS_AUX_DEVICES
	const unsigned int PanelDueFirmwareModule = 4;
#endif

	bool CheckFirmwareUpdatePrerequisites(uint8_t moduleMap, const StringRef& reply) noexcept;
	bool IsReady() noexcept;
	void UpdateModule(unsigned int module) noexcept;
}

#endif /* SRC_NETWORKING_FIRMWAREUPDATER_H_ */
