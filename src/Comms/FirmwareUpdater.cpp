/*
 * FirmwareUpdater.cpp
 *
 *  Created on: 21 May 2016
 *      Author: David
 */

#include "FirmwareUpdater.h"

#include "RepRapFirmware.h"
#include "Platform.h"
#include "RepRap.h"
#include "GCodes/GCodes.h"

#if HAS_WIFI_NETWORKING
# include "Network.h"
# include "ESP8266WiFi/WifiFirmwareUploader.h"
#endif

#if HAS_AUX_DEVICES
# include "Comms/PanelDueUpdater.h"
#endif

namespace FirmwareUpdater
{
	const unsigned int WifiFirmwareModule = 1;
	// Module 2 used to be the DWC binary file but is no longer used
	const unsigned int WifiExternalFirmwareModule = 3;

	// Check that the prerequisites are satisfied.
	// Return true if yes, else print a message and return false.
	GCodeResult CheckFirmwareUpdatePrerequisites(uint8_t moduleMap, GCodeBuffer& gb, const StringRef& reply, const size_t serialChannel) noexcept
	{
#if HAS_WIFI_NETWORKING
		GCodeResult result;
		if (!reprap.GetGCodes().CheckNetworkCommandAllowed(gb, reply, result))
		{
			return result;
		}
		if ((moduleMap & (1 << WifiExternalFirmwareModule)) != 0 && (moduleMap & (1 << WifiFirmwareModule)) != 0)
		{
			reply.copy("Invalid combination of firmware update modules");
			return GCodeResult::error;
		}
		if ((moduleMap & (1 << WifiFirmwareModule)) != 0
				&& !reprap.GetPlatform().FileExists(DEFAULT_SYS_DIR, WIFI_FIRMWARE_FILE))
		{
			reply.printf("File %s not found", WIFI_FIRMWARE_FILE);
			return GCodeResult::error;
		}
#endif
#if HAS_AUX_DEVICES
		if ((moduleMap & (1 << PanelDueFirmwareModule)) != 0)
		{
			if (!reprap.GetPlatform().IsAuxEnabled(serialChannel-1) || reprap.GetPlatform().IsAuxRaw(serialChannel-1))
			{
				reply.printf("Aux port %d is not enabled or not in PanelDue mode", serialChannel-1);
				return GCodeResult::error;
			}
			if (!reprap.GetPlatform().FileExists(DEFAULT_SYS_DIR, PANEL_DUE_FIRMWARE_FILE))
			{
				reply.printf("File %s not found", PanelDueUpdater::firmwareFilename);
				return GCodeResult::error;
			}
		}
#endif
		return GCodeResult::ok;
	}

	bool IsReady() noexcept
	{
#if HAS_WIFI_NETWORKING
		WifiFirmwareUploader * const uploader = reprap.GetNetwork().GetWifiUploader();
		if(!(uploader == nullptr || uploader->IsReady())) {
			return false;
		}
#endif
#if HAS_AUX_DEVICES
		PanelDueUpdater *panelDueUpdater = reprap.GetPlatform().GetPanelDueUpdater();
		if (panelDueUpdater != nullptr && !panelDueUpdater->Idle()) {
			return false;
		}
#endif
		return true;
	}

	void UpdateModule(unsigned int module, const size_t serialChannel) noexcept
	{
#if HAS_WIFI_NETWORKING || HAS_AUX_DEVICES
		switch(module)
		{
# if HAS_WIFI_NETWORKING
		case WifiExternalFirmwareModule:
# ifdef DUET_NG
			if (reprap.GetPlatform().IsDuetWiFi())
# endif
			{
				reprap.GetNetwork().ResetWiFiForUpload(true);
			}
			break;

		case WifiFirmwareModule:
# ifdef DUET_NG
			if (reprap.GetPlatform().IsDuetWiFi())
# endif
			{
				WifiFirmwareUploader * const uploader = reprap.GetNetwork().GetWifiUploader();
				if (uploader != nullptr)
				{
					uploader->SendUpdateFile(WIFI_FIRMWARE_FILE, DEFAULT_SYS_DIR, WifiFirmwareUploader::FirmwareAddress);
				}
			}
			break;
# endif
# if HAS_AUX_DEVICES
		case PanelDueFirmwareModule:
			{
				Platform& platform = reprap.GetPlatform();
				if (platform.GetPanelDueUpdater() == nullptr)
				{
					platform.InitPanelDueUpdater();
				}
				platform.GetPanelDueUpdater()->Start(serialChannel);
			}
# endif
		}
#endif
	}
}

// End
