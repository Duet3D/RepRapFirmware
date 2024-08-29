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
	constexpr size_t MaxBufferLen = 46;		// max number of bytes supported in a single message
	constexpr uint32_t SendInterval = 2;	// interval in ms between sending data
	constexpr size_t DefaultAuxChannel = 0;
	constexpr size_t AnalogSensorNum = 2;		// analog sensor number to log

	bool SendDataToUart();
	void ClearBuffer();
	bool AddDataToBuffer(uint8_t val);
	void CollectAndSendData();

	uint32_t GetLastTransmissionTime();
	uint32_t GetTransmissionTime();
}

# endif

#endif /* SRC_COMMS_DATACOLLECTION_H_ */
