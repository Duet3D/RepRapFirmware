/*
 * DataCollection.h
 *
 *  Created on: 26 Aug 2024
 *      Author: Andy Everitt
 */

#ifndef SRC_COMMS_DATACOLLECTION_H_
#define SRC_COMMS_DATACOLLECTION_H_

#include <RepRapFirmware.h>

# if SUPPORT_DATA_COLLECTION

namespace DataCollection
{
	struct AnalogSensorInfo
	{
		size_t number;
		size_t decimals;
	};

	constexpr size_t MaxBufferLen = 46;		// max number of bytes supported in a single message
	constexpr uint32_t SendInterval = 2;	// interval in ms between sending data
	constexpr size_t DefaultAuxChannel = 0;
	constexpr AnalogSensorInfo AnalogSensors[] = {
			{.number=20, .decimals=0},
			{.number=21, .decimals=1}
	};		// analog sensor numbers to log

	bool SendDataToUart();
	void ClearBuffer();
	bool AddDataToBuffer(uint8_t val);
	void CollectAndSendData();

	uint32_t GetLastTransmissionTime();
	uint32_t GetTransmissionTime();
}

# endif

#endif /* SRC_COMMS_DATACOLLECTION_H_ */
