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
#ifdef SAMC21
	constexpr unsigned int AdcBits = 16;
#else
	constexpr unsigned int AdcBits = 16;
#endif

	// Initialise the analog input subsystem. Call this just once.
	void Init();

	// Enable analog input on a pin.
	// Readings will be taken and about every 'ticksPerCall' milliseconds the callback function will be called with the specified parameter and ADC reading.
	// Set ticksPerCall to 0 to get a callback on every reading.
	// Warning! there is nothing to stop you enabling a channel twice, in which case in the SAME51 configuration, it will be read twice in the sequence.
	bool EnableChannel(Pin pin, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t ticksPerCall, bool useAlternateAdc);

	// Readings will be taken and about every 'ticksPerCall' milliseconds the callback function will be called with the specified parameter and ADC reading.
	// Set ticksPerCall to 0 to get a callback on every reading.
	bool SetCallback(Pin pin, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t ticksPerCall, bool useAlternateAdc);

	// Return whether or not the channel is enabled
	bool IsChannelEnabled(Pin pin, bool useAlternateAdc = false);

	// Disable a previously-enabled channel
	//bool DisableChannel(Pin pin);

	// Get the latest result from a channel. the channel must have been enabled first.
	uint16_t ReadChannel(AdcInput adcin);

	// Get the number of conversions that were started
	void GetDebugInfo(uint32_t &convsStarted, uint32_t &convsCompleted, uint32_t &convTimeouts);

#ifdef SAME51
	// Enable an on-chip MCU temperature sensor. We don't use this on the SAMC21 because that chip has a separate TSENS peripheral.
	bool EnableTemperatureSensor(unsigned int sensorNumber, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t ticksPerCall, unsigned int adcnum);
#endif

#ifdef SAMC21
	void EnableTemperatureSensor(AnalogInCallbackFunction fn, CallbackParameter param, uint32_t ticksPerCall);
#endif
}

#endif /* SRC_HARDWARE_ANALOGIN_H_ */
