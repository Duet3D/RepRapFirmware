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
	enum {
		Mainboard = 0,
		Sbc = 2,				// Module 2 used to be the DWC binary file, now it is used to update SBC firmware
#if HAS_WIFI_NETWORKING
		WifiFirmwareModule = 1,
		_unused = 3,			// Module 2 was WifiExternalFirmwareModule, no longer supported
#endif
#if NUM_ASYNC_CHANNELS != 0
		PanelDueFirmwareModule = 4,
#endif
		NumUpdateModules
	};

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
