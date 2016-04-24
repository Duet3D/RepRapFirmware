/*
 * TemperatureError.h
 *
 *  Created on: 21 Apr 2016
 *      Author: David
 */

#ifndef TEMPERATUREERROR_H_
#define TEMPERATUREERROR_H_

#include <cstdint>

// Result codes returned by temperature sensor drivers
enum class TemperatureError : uint8_t
{
	success,
	shortCircuit,
	shortToVcc,
	shortToGround,
	openCircuit,
	tooHigh,
	timeout,
	ioError,
	hardwareError,
	busBusy,
	badResponse
};

const char* TemperatureErrorString(TemperatureError err);

// Indicate if a temp sensor error is a "permanent" error: whether it is an error condition which will not rectify over time
// and so the heater should just be shut off immediately.
bool IsPermanentError(TemperatureError err);

#endif /* TEMPERATUREERROR_H_ */
