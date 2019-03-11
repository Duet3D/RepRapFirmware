/*
 * FirmwareUpdater.cpp
 *
 *  Created on: 21 May 2016
 *      Author: David
 */

#include "FirmwareUpdater.h"

#include "RepRapFirmware.h"
#include "Network.h"
#include "Platform.h"
#include "RepRap.h"

#if HAS_WIFI_NETWORKING
#include "ESP8266WiFi/WifiFirmwareUploader.h"
#endif

namespace FirmwareUpdater
{
	const unsigned int WifiFirmwareModule = 1;
	// Module 2 used to be the DWC binary file but is no longer used
	const unsigned int WifiExternalFirmwareModule = 3;

	// Check that the prerequisites are satisfied.
	// Return true if yes, else print a message and return false.
	bool CheckFirmwareUpdatePrerequisites(uint8_t moduleMap, const StringRef& reply)
	{
#if HAS_WIFI_NETWORKING
		if ((moduleMap & (1 << WifiExternalFirmwareModule)) != 0 && (moduleMap & (1 << WifiFirmwareModule)) != 0)
		{
			reply.copy("Invalid combination of firmware update modules");
			return false;
		}
		if ((moduleMap & (1 << WifiFirmwareModule)) != 0 && !reprap.GetPlatform().FileExists(DEFAULT_SYS_DIR, WIFI_FIRMWARE_FILE))
		{
			reply.printf("File %s not found", WIFI_FIRMWARE_FILE);
			return false;
		}
#endif
		return true;
	}

	bool IsReady()
	{
#if HAS_WIFI_NETWORKING
		WifiFirmwareUploader * const uploader = reprap.GetNetwork().GetWifiUploader();
		return uploader == nullptr || uploader->IsReady();
#else
		return true;
#endif
	}

	void UpdateModule(unsigned int module)
	{
#if HAS_WIFI_NETWORKING
# ifdef DUET_NG
		if (reprap.GetPlatform().IsDuetWiFi())
		{
# endif
			switch(module)
			{
			case WifiExternalFirmwareModule:
				reprap.GetNetwork().ResetWiFiForUpload(true);
				break;

			case WifiFirmwareModule:
				{
					WifiFirmwareUploader * const uploader = reprap.GetNetwork().GetWifiUploader();
					if (uploader != nullptr)
					{
						uploader->SendUpdateFile(WIFI_FIRMWARE_FILE, DEFAULT_SYS_DIR, WifiFirmwareUploader::FirmwareAddress);
					}
				}
				break;
			}
# ifdef DUET_NG
		}
# endif
#endif
	}
}

// End
