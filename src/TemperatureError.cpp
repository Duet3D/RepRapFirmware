/*
 * TemperatureError.cpp
 *
 *  Created on: 21 Apr 2016
 *      Author: David
 */

#include "TemperatureError.h"

const char* TemperatureErrorString(TemperatureError err)
{
	switch(err)
	{
	case TemperatureError::success:			return "success";
	case TemperatureError::shortCircuit:	return "short-circuit in sensor";
	case TemperatureError::shortToVcc:		return "short to Vcc";
	case TemperatureError::shortToGround:	return "short to ground";
	case TemperatureError::openCircuit:		return "open circuit";
	case TemperatureError::tooHigh:			return "temperature above limit";
	case TemperatureError::timeout:			return "timeout";
	case TemperatureError::ioError:			return "I/O error";
	case TemperatureError::hardwareError:	return "hardware error";
	case TemperatureError::busBusy:			return "bus busy";
	case TemperatureError::badResponse:		return "bad response";
	default:								return "unknown temperature sense error";
	}
}

// Indicate if a temp sensor error is a "permanent" error: whether it is an error condition which will not rectify over time
// and so the heater should just be shut off immediately.
bool IsPermanentError(TemperatureError err)
{
	switch (err)
	{
	case TemperatureError::success:
	case TemperatureError::busBusy:
	case TemperatureError::ioError:
		return false;
	default:
		return true;
	}
}

// End
