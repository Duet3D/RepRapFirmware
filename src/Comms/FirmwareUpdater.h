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
		unused = 2, // Module 2 used to be the DWC binary file but is no longer used
#if HAS_WIFI_NETWORKING
		WifiFirmwareModule = 1,
		WifiExternalFirmwareModule = 3,
#endif
#if HAS_AUX_DEVICES
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
