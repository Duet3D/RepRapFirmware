/*
 * AnalogIn.h
 *
 *  Created on: 6 Sep 2018
 *      Author: David
 */

#ifndef SRC_HARDWARE_ANALOGIN_H_
#define SRC_HARDWARE_ANALOGIN_H_

#include "RepRapFirmware.h"

typedef void (*AnalogInCallbackFunction)(CallbackParameter p, uint16_t reading);

namespace AnalogIn
{
	// The number of bits that the ADCs return
	constexpr unsigned int AdcBits = 16;

	// Initialise the analog input subsystem. Call this just once.
	void Init();

	// Enable analog input on a pin.
	// Readings will be taken and about every 'ticksPerCall' milliseconds the callback function will be called with the specified parameter and ADC reading.
	// Set ticksPerCall to 0 to get a callback on every reading.
	// Warning! there is nothing to stop you enabling a channel twice, in which case in the SAME5x configuration, it will be read twice in the sequence.
	bool EnableChannel(AdcInput adcin, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t ticksPerCall);

	// Readings will be taken and about every 'ticksPerCall' milliseconds the callback function will be called with the specified parameter and ADC reading.
	// Set ticksPerCall to 0 to get a callback on every reading.
	bool SetCallback(AdcInput adcin, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t ticksPerCall);

	// Return whether or not the channel is enabled
	bool IsChannelEnabled(AdcInput adcin);

	// Disable a previously-enabled channel
	void DisableChannel(AdcInput adcin);

	// Get the latest result from a channel. the channel must have been enabled first.
	uint16_t ReadChannel(AdcInput adcin);

	// Get the number of conversions that were started
	void GetDebugInfo(uint32_t &convsStarted, uint32_t &convsCompleted, uint32_t &convTimeouts);

	// Enable an on-chip MCU temperature sensor
	bool EnableTemperatureSensor(unsigned int sensorNumber, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t ticksPerCall, unsigned int adcnum);
}

// This is for backwards compatibility
inline uint16_t AnalogInReadChannel(AdcInput adcin)
{
	return AnalogIn::ReadChannel(adcin);
}

inline void AnalogInEnableChannel(AdcInput adcin, bool enable)
{
	if (enable)
	{
		if (!AnalogIn::IsChannelEnabled(adcin))
		{
			AnalogIn::EnableChannel(adcin, nullptr, CallbackParameter(), 1000);
		}
	}
	else
	{
		AnalogIn::DisableChannel(adcin);
	}
}

#endif /* SRC_HARDWARE_ANALOGIN_H_ */
