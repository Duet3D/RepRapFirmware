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
	GCodeResult CheckFirmwareUpdatePrerequisites(
			Bitmap<uint8_t> moduleMap,
			GCodeBuffer& gb,
			const StringRef& reply,
			const size_t serialChannel,
			const StringRef& filenameRef) noexcept
	{
#if HAS_WIFI_NETWORKING
		if (moduleMap.IsBitSet(WifiExternalFirmwareModule) || moduleMap.IsBitSet(WifiFirmwareModule))
		{
			GCodeResult result;
			if (!reprap.GetGCodes().CheckNetworkCommandAllowed(gb, reply, result))
			{
				return result;
			}
			if (moduleMap.IsBitSet(WifiExternalFirmwareModule) && moduleMap.IsBitSet(WifiFirmwareModule))
			{
				reply.copy("Invalid combination of firmware update modules");
				return GCodeResult::error;
			}
			if (moduleMap.IsBitSet(WifiFirmwareModule))
			{
				String<MaxFilenameLength> location;
				if (!MassStorage::CombineName(location.GetRef(), FIRMWARE_DIRECTORY, filenameRef.IsEmpty() ? WIFI_FIRMWARE_FILE : filenameRef.c_str())
						|| !MassStorage::FileExists(location.c_str()))
				{
					reply.printf("File %s not found", location.c_str());
					return GCodeResult::error;
				}
			}
		}
#endif
#if HAS_AUX_DEVICES
		if (moduleMap.IsBitSet(PanelDueFirmwareModule))
		{
			if (!reprap.GetPlatform().IsAuxEnabled(serialChannel-1) || reprap.GetPlatform().IsAuxRaw(serialChannel-1))
			{
				reply.printf("Aux port %d is not enabled or not in PanelDue mode", serialChannel-1);
				return GCodeResult::error;
			}
			String<MaxFilenameLength> location;
			if (!MassStorage::CombineName(location.GetRef(), FIRMWARE_DIRECTORY, filenameRef.IsEmpty() ? PANEL_DUE_FIRMWARE_FILE : filenameRef.c_str())
					|| !MassStorage::FileExists(location.c_str()))
			{
				reply.printf("File %s not found", location.c_str());
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

	void UpdateModule(unsigned int module, const size_t serialChannel, const StringRef& filenameRef) noexcept
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
					const char* binaryFilename = filenameRef.IsEmpty() ? WIFI_FIRMWARE_FILE : filenameRef.c_str();
					uploader->SendUpdateFile(binaryFilename, FIRMWARE_DIRECTORY, WifiFirmwareUploader::FirmwareAddress);
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
				platform.GetPanelDueUpdater()->Start(filenameRef, serialChannel);
			}
# endif
		}
#endif
	}
}

// End
