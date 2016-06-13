/*
 * FirmwareUpdater.cpp
 *
 *  Created on: 21 May 2016
 *      Author: David
 */

#include "FirmwareUpdater.h"
#include "RepRapFirmware.h"
#include "WifiFirmwareUploader.h"

namespace FirmwareUpdater
{
	const unsigned int WifiFirmwareModule = 1;
	const unsigned int WifiFilesModule = 2;
	const unsigned int WifiExternalFirmwareModule = 3;

	// Check that the prerequisites are satisfied.
	// Return true if yes, else print a message and return false.
	bool CheckFirmwareUpdatePrerequisites(uint8_t moduleMap)
	{
		if ((moduleMap & (1 << WifiExternalFirmwareModule)) != 0 && (moduleMap & ((1 << WifiFirmwareModule) | (1 << WifiFilesModule))) != 0)
		{
			reprap.GetPlatform()->Message(GENERIC_MESSAGE, "Invalid combination of firmware update modules\n");
			return false;
		}
		if ((moduleMap & (1 << WifiFirmwareModule)) != 0 && !reprap.GetPlatform()->GetMassStorage()->FileExists(SYS_DIR, WIFI_FIRMWARE_FILE))
		{
			reprap.GetPlatform()->MessageF(GENERIC_MESSAGE, "File %s not found\n", WIFI_FIRMWARE_FILE);
			return false;
		}
		if ((moduleMap & (1 << WifiFilesModule)) != 0 && !reprap.GetPlatform()->GetMassStorage()->FileExists(SYS_DIR, WIFI_WEB_FILE))
		{
			reprap.GetPlatform()->MessageF(GENERIC_MESSAGE, "File %s not found\n", WIFI_WEB_FILE);
			return false;
		}
		return true;
	}

	bool IsReady()
	{
		return reprap.GetNetwork()->GetWifiUploader()->IsReady();
	}

	void UpdateModule(unsigned int module)
	{
		switch(module)
		{
		case WifiExternalFirmwareModule:
			Network::ResetWiFiForExternalUpload();
			break;

		case WifiFirmwareModule:
			reprap.GetNetwork()->GetWifiUploader()->SendUpdateFile(WIFI_FIRMWARE_FILE, SYS_DIR, WifiFirmwareUploader::FirmwareAddress);
			break;

		case WifiFilesModule:
			reprap.GetNetwork()->GetWifiUploader()->SendUpdateFile(WIFI_WEB_FILE, SYS_DIR, WifiFirmwareUploader::WebFilesAddress);
			break;
		}
	}
}

// End
