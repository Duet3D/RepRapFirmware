/*
 * FirmwareUpdater.h
 *
 *  Created on: 21 May 2016
 *      Author: David
 */

#ifndef SRC_NETWORKING_FIRMWAREUPDATER_H_
#define SRC_NETWORKING_FIRMWAREUPDATER_H_

#include <RepRapFirmware.h>

namespace FirmwareUpdater
{
#if HAS_AUX_DEVICES
	const unsigned int PanelDueFirmwareModule = 4;
#endif

	GCodeResult CheckFirmwareUpdatePrerequisites(
			Bitmap<uint8_t> moduleMap,
			GCodeBuffer& gb,
			const StringRef& reply,
			size_t serialChannel,
			const StringRef& filenameRef) noexcept;
	bool IsReady() noexcept;
	void UpdateModule(unsigned int module, const size_t serialChannel, const StringRef& filenameRef) noexcept;
}

#endif /* SRC_NETWORKING_FIRMWAREUPDATER_H_ */
